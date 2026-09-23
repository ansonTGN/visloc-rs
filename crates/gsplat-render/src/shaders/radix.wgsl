// Subgroup-free LSD radix sort (4 bits per pass) for a (key: u32, value: u32)
// pair, fully on the device (no per-pass readback).
//
// Four kernels per pass:
//   radix_clear     : zero hist[wg * 16 + d] on device (no host write_buffer)
//   radix_histogram : per-workgroup digit histogram -> hist[wg * 16 + d]
//   radix_scan      : exclusive scan of hist in digit-major order
//                     -> base[d * num_blocks + wg] (start offset)
//   radix_scatter   : stable scatter of each element to its new slot
//
// Stability is what makes LSD radix correct: elements with the same digit must
// keep their relative order. Each workgroup computes a *stable* per-element rank
// within its block with an in-block per-digit exclusive scan (double-buffered,
// no atomics, no subgroups).
//
// Block layout: 256 threads, ITEMS consecutive elements per thread (thread t
// owns elements t*ITEMS .. t*ITEMS+ITEMS).

const BINS: u32 = 16u;
const WG: u32 = 256u;
const ITEMS: u32 = 4u;
const BLOCK: u32 = WG * ITEMS; // 1024 elements per workgroup

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
// hist[wg * BINS + d]; zeroed on device by `radix_clear` before each pass.
@group(0) @binding(5) var<storage, read_write> hist: array<atomic<u32>>;
// base[d * num_blocks + wg] exclusive start offset for this (digit, block).
@group(0) @binding(6) var<storage, read_write> base: array<u32>;

// Shared in-block per-digit exclusive-scan buffer (16 digits x 256 threads).
var<workgroup> scan_cur: array<u32, BINS * WG>;
// Per-digit totals and exclusive digit prefix, used by `radix_scan`.
var<workgroup> digit_totals: array<u32, BINS>;
var<workgroup> digit_prefix: array<u32, BINS>;

// Pass 0: zero every histogram entry for this pass. One invocation per entry;
// the dispatch covers num_blocks * BINS entries.
@compute @workgroup_size(256)
fn radix_clear(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if (i < params.num_blocks * BINS) {
        atomicStore(&hist[i], 0u);
    }
}

// Pass 1: count each digit in the workgroup's block.
@compute @workgroup_size(256)
fn radix_histogram(
    @builtin(workgroup_id) wid: vec3<u32>,
    @builtin(local_invocation_id) lid: vec3<u32>,
) {
    var local: array<u32, 16>;
    for (var b = 0u; b < 16u; b = b + 1u) {
        local[b] = 0u;
    }
    let block_base = wid.x * BLOCK;
    let n = params.num_elements;
    for (var e = lid.x; e < BLOCK; e = e + WG) {
        let i = block_base + e;
        if (i < n) {
            let d = (keys_in[i] >> params.shift) & 0xFu;
            local[d] = local[d] + 1u;
        }
    }
    for (var b = 0u; b < 16u; b = b + 1u) {
        if (local[b] > 0u) {
            atomicAdd(&hist[wid.x * BINS + b], local[b]);
        }
    }
}

// Pass 2: exclusive scan of hist in digit-major order.
//
// base[d * num_blocks + wg] = sum of counts of all digits < d over all blocks
//                          + sum of counts of digit d in blocks < wg.
//
// One workgroup; the grid is BINS * num_blocks entries. Two serial reductions
// keep this obviously correct.
@compute @workgroup_size(256)
fn radix_scan(@builtin(local_invocation_id) lid: vec3<u32>) {
    let nblocks = params.num_blocks;
    // Each digit's grand total (one thread per digit), then the exclusive
    // prefix over digits, then one thread per digit walks the blocks. O(BINS *
    // nblocks) total.
    if (lid.x < BINS) {
        var total = 0u;
        for (var w = 0u; w < nblocks; w = w + 1u) {
            total = total + atomicLoad(&hist[w * BINS + lid.x]);
        }
        digit_totals[lid.x] = total;
    }
    workgroupBarrier();
    if (lid.x == 0u) {
        var acc = 0u;
        for (var d = 0u; d < BINS; d = d + 1u) {
            digit_prefix[d] = acc;
            acc = acc + digit_totals[d];
        }
    }
    workgroupBarrier();
    if (lid.x < BINS) {
        let d = lid.x;
        var running = digit_prefix[d];
        for (var wg = 0u; wg < nblocks; wg = wg + 1u) {
            base[d * nblocks + wg] = running;
            running = running + atomicLoad(&hist[wg * BINS + d]);
        }
    }
}

// Pass 3: stable scatter.
@compute @workgroup_size(256)
fn radix_scatter(
    @builtin(workgroup_id) wid: vec3<u32>,
    @builtin(local_invocation_id) lid: vec3<u32>,
) {
    let n = params.num_elements;
    let nblocks = params.num_blocks;
    let block_base = wid.x * BLOCK;

    // Load this thread's ITEMS elements; derive their digits.
    var my_keys: array<u32, ITEMS>;
    var my_vals: array<u32, ITEMS>;
    var my_digits: array<u32, ITEMS>;
    var local: array<u32, 16>;
    for (var b = 0u; b < 16u; b = b + 1u) {
        local[b] = 0u;
    }
    for (var k = 0u; k < ITEMS; k = k + 1u) {
        let i = block_base + lid.x * ITEMS + k;
        var key = 0xffffffffu;
        var val = 0u;
        if (i < n) {
            key = keys_in[i];
            val = values_in[i];
        }
        my_keys[k] = key;
        my_vals[k] = val;
        let d = (key >> params.shift) & 0xFu;
        my_digits[k] = d;
        local[d] = local[d] + 1u;
    }

    // Exclusive scan, per digit, over the thread axis: rank[d][t] = number of
    // elements with digit d owned by threads < t. In-place Hillis-Steele over
    // 256 entries x 16 digits (16 KB of workgroup storage), safe because the
    // read and the write are separated by a barrier.
    for (var b = 0u; b < 16u; b = b + 1u) {
        scan_cur[b * WG + lid.x] = local[b];
    }
    workgroupBarrier();
    var offset = 1u;
    loop {
        if (offset >= WG) { break; }
        var staged: array<u32, 16>;
        for (var b = 0u; b < 16u; b = b + 1u) {
            let idx = b * WG + lid.x;
            var v = scan_cur[idx];
            if (lid.x >= offset) {
                v = v + scan_cur[b * WG + lid.x - offset];
            }
            staged[b] = v;
        }
        workgroupBarrier();
        for (var b = 0u; b < 16u; b = b + 1u) {
            scan_cur[b * WG + lid.x] = staged[b];
        }
        workgroupBarrier();
        offset = offset * 2u;
    }
    // scan_cur now holds the inclusive scan; exclusive base = inclusive - local.
    // Build the per-element rank and scatter.
    for (var k = 0u; k < ITEMS; k = k + 1u) {
        let i = block_base + lid.x * ITEMS + k;
        if (i >= n) {
            continue;
        }
        let d = my_digits[k];
        let inclusive = scan_cur[d * WG + lid.x];
        let excl = inclusive - local[d];
        // Within the thread, earlier elements of the same digit come first.
        var within = 0u;
        for (var kk = 0u; kk < k; kk = kk + 1u) {
            if (my_digits[kk] == d) {
                within = within + 1u;
            }
        }
        let out_i = base[d * nblocks + wid.x] + excl + within;
        keys_out[out_i] = my_keys[k];
        values_out[out_i] = my_vals[k];
    }
}
