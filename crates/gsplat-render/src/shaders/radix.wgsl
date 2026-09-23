// Subgroup-free LSD radix sort (4 bits per pass) for a (key: u32, value: u32)
// pair, fully on the device (no per-pass readback).
//
// Three kernels per pass:
//   radix_histogram : per-workgroup digit histogram -> hist[wg * 16 + d]
//   radix_scan      : exclusive scan of hist in digit-major order
//                     -> base[d * num_blocks + wg] (start offset)
//   radix_scatter   : stable scatter of each element to its new slot
//
// Stability is what makes LSD radix correct: elements with the same digit must
// keep their relative order. radix_scatter ranks the block with stable 1-bit
// splits in shared memory (no atomics, no subgroups).
//
// Block layout: 256 threads, ITEMS consecutive elements per thread (thread t
// owns elements t*ITEMS .. t*ITEMS+ITEMS).

const BINS: u32 = 16u;
const WG: u32 = 256u;
const ITEMS: u32 = 16u;
const BLOCK: u32 = WG * ITEMS; // 4096 elements per workgroup (keep = sort.rs BLOCK)

struct RadixParams {
    shift: u32,
    num_elements: u32,
    num_blocks: u32,
    pad0: u32,
};

@group(0) @binding(0) var<uniform> params: RadixParams;
@group(0) @binding(1) var<storage, read> keys_in: array<u32>;
@group(0) @binding(2) var<storage, read> values_in: array<u32>;
@group(0) @binding(3) var<storage, read_write> keys_out: array<u32>;
@group(0) @binding(4) var<storage, read_write> values_out: array<u32>;
// hist[wg * BINS + d]; fully rewritten by `radix_histogram` every pass.
@group(0) @binding(5) var<storage, read_write> hist: array<atomic<u32>>;
// base[d * num_blocks + wg] exclusive start offset for this (digit, block).
@group(0) @binding(6) var<storage, read_write> base: array<u32>;

// Packed (digit << 12 | index) entries ranked by `radix_scatter`.
var<workgroup> ranks: array<u32, BLOCK>;
const_assert BLOCK <= 4096u;
// Per-thread partial sums: chunked scan in `radix_scan`, split scans in
// `radix_scatter`.
var<workgroup> partial: array<u32, WG>;
// Per-block digit totals and their exclusive prefix, used by `radix_scatter`.
var<workgroup> block_digit_total: array<u32, BINS>;
var<workgroup> block_digit_start: array<u32, BINS>;
// The block's digit counts, accumulated by `radix_histogram`.
var<workgroup> block_hist: array<atomic<u32>, BINS>;
// Entries of the digit-major histogram each thread handles per scan chunk.
const SCAN_ITEMS: u32 = 16u;

// Pass 1: count each digit in the workgroup's block and store the block's
// histogram row (every entry is written, so no separate clear pass).
//
// Per-thread counts are packed 8 bits per digit into a vec4 (a thread sees
// at most ITEMS = 16 keys), which avoids a runtime-indexed array that would
// spill to local memory; the workgroup then combines them with shared
// atomics instead of global ones.
@compute @workgroup_size(256)
fn radix_histogram(
    @builtin(workgroup_id) wid: vec3<u32>,
    @builtin(local_invocation_id) lid: vec3<u32>,
) {
    if (lid.x < BINS) {
        atomicStore(&block_hist[lid.x], 0u);
    }
    workgroupBarrier();
    var packed = vec4<u32>(0u, 0u, 0u, 0u);
    let block_base = wid.x * BLOCK;
    let n = params.num_elements;
    for (var e = lid.x; e < BLOCK; e = e + WG) {
        let i = block_base + e;
        if (i < n) {
            let d = (keys_in[i] >> params.shift) & 0xFu;
            packed[d >> 2u] = packed[d >> 2u] + (1u << ((d & 3u) * 8u));
        }
    }
    for (var d = 0u; d < BINS; d = d + 1u) {
        let c = (packed[d >> 2u] >> ((d & 3u) * 8u)) & 0xFFu;
        if (c > 0u) {
            atomicAdd(&block_hist[d], c);
        }
    }
    workgroupBarrier();
    if (lid.x < BINS) {
        atomicStore(&hist[wid.x * BINS + lid.x], atomicLoad(&block_hist[lid.x]));
    }
}

// Pass 2: exclusive scan of hist in digit-major order.
//
// base[d * num_blocks + wg] = sum of counts of all digits < d over all blocks
//                          + sum of counts of digit d in blocks < wg.
//
// That is exactly an exclusive scan of the flat digit-major sequence
// j = d * num_blocks + wg (value hist[wg * BINS + d]). One workgroup walks it
// in chunks of WG * SCAN_ITEMS: each thread sums SCAN_ITEMS consecutive
// entries, the workgroup scans the 256 partial sums, then each thread writes
// its entries' exclusive offsets. (A per-digit serial walk over the blocks was
// latency-bound: ~2 * num_blocks dependent global loads per thread.)
@compute @workgroup_size(256)
fn radix_scan(@builtin(local_invocation_id) lid: vec3<u32>) {
    let nblocks = params.num_blocks;
    let total_len = BINS * nblocks;
    let t = lid.x;
    var carry = 0u;
    var chunk = 0u;
    loop {
        if (chunk >= total_len) { break; }
        let first = chunk + t * SCAN_ITEMS;
        var sum = 0u;
        for (var k = 0u; k < SCAN_ITEMS; k = k + 1u) {
            let j = first + k;
            if (j < total_len) {
                sum = sum + atomicLoad(&hist[(j % nblocks) * BINS + j / nblocks]);
            }
        }
        partial[t] = sum;
        workgroupBarrier();
        // Inclusive Hillis-Steele over the 256 partial sums.
        var offset = 1u;
        loop {
            if (offset >= WG) { break; }
            var v = partial[t];
            if (t >= offset) {
                v = v + partial[t - offset];
            }
            workgroupBarrier();
            partial[t] = v;
            workgroupBarrier();
            offset = offset * 2u;
        }
        var running = carry + partial[t] - sum;
        for (var k = 0u; k < SCAN_ITEMS; k = k + 1u) {
            let j = first + k;
            if (j < total_len) {
                base[j] = running;
                running = running + atomicLoad(&hist[(j % nblocks) * BINS + j / nblocks]);
            }
        }
        carry = carry + partial[WG - 1u];
        // Everyone has read partial[] before the next chunk overwrites it.
        workgroupBarrier();
        chunk = chunk + WG * SCAN_ITEMS;
    }
}

// Pass 3: stable scatter.
//
// Rank the block's elements digit-major in shared memory, then write them out.
// Each shared entry packs (digit << 12 | index-in-block); four stable 1-bit
// split rounds (one per digit bit, LSB first) sort the entries by digit while
// keeping index order within a digit. A split only needs one 256-wide scan of
// per-thread zero counts, instead of a 16-digit scan over the thread axis with
// runtime-indexed per-thread arrays. Entry j then goes to
// base[d][block] + (j - start of digit d in the block): consecutive entries of
// a digit land in consecutive slots.
@compute @workgroup_size(256)
fn radix_scatter(
    @builtin(workgroup_id) wid: vec3<u32>,
    @builtin(local_invocation_id) lid: vec3<u32>,
) {
    let n = params.num_elements;
    let nblocks = params.num_blocks;
    let block_base = wid.x * BLOCK;
    let count = min(BLOCK, n - min(n, block_base));
    let t = lid.x;

    // Where each digit starts inside this block: exclusive prefix of the
    // block's histogram row (written by radix_histogram this pass).
    if (t < BINS) {
        block_digit_total[t] = atomicLoad(&hist[wid.x * BINS + t]);
    }
    workgroupBarrier();
    if (t == 0u) {
        var acc = 0u;
        for (var d = 0u; d < BINS; d = d + 1u) {
            block_digit_start[d] = acc;
            acc = acc + block_digit_total[d];
        }
    }

    // Packed entries in input order (coalesced key loads). Padding past `n`
    // gets digit 15 and the largest indices, so it sorts to positions >= count.
    for (var m = 0u; m < ITEMS; m = m + 1u) {
        let j = m * WG + t;
        var d = BINS - 1u;
        if (j < count) {
            d = (keys_in[block_base + j] >> params.shift) & 0xFu;
        }
        ranks[j] = (d << 12u) | j;
    }
    workgroupBarrier();

    for (var bit = 0u; bit < 4u; bit = bit + 1u) {
        // Each thread owns ITEMS consecutive entries, so thread order is
        // entry order and the split stays stable.
        var mine: array<u32, ITEMS>;
        var zeros = 0u;
        for (var k = 0u; k < ITEMS; k = k + 1u) {
            let v = ranks[t * ITEMS + k];
            mine[k] = v;
            zeros = zeros + (1u - ((v >> (12u + bit)) & 1u));
        }
        partial[t] = zeros;
        workgroupBarrier();
        var offset = 1u;
        loop {
            if (offset >= WG) { break; }
            var acc = partial[t];
            if (t >= offset) {
                acc = acc + partial[t - offset];
            }
            workgroupBarrier();
            partial[t] = acc;
            workgroupBarrier();
            offset = offset * 2u;
        }
        let total_zeros = partial[WG - 1u];
        var z = partial[t] - zeros;
        var o = total_zeros + t * ITEMS - z;
        for (var k = 0u; k < ITEMS; k = k + 1u) {
            let v = mine[k];
            if (((v >> (12u + bit)) & 1u) == 0u) {
                ranks[z] = v;
                z = z + 1u;
            } else {
                ranks[o] = v;
                o = o + 1u;
            }
        }
        // Writes done before the next round reads; partial[WG - 1] read
        // before the next round overwrites it.
        workgroupBarrier();
    }

    for (var m = 0u; m < ITEMS; m = m + 1u) {
        let j = m * WG + t;
        if (j < count) {
            let v = ranks[j];
            let d = v >> 12u;
            let src = block_base + (v & 0xFFFu);
            let dst = base[d * nblocks + wid.x] + (j - block_digit_start[d]);
            keys_out[dst] = keys_in[src];
            values_out[dst] = values_in[src];
        }
    }
}
