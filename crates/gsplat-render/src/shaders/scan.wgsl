// Subgroup-free inclusive prefix sum over `u32` values, device-side.
//
// Three kernels, dispatched in order:
//   scan_blocks     : each workgroup inclusively scans one BLOCK and writes its
//                     block total to block_sums
//   scan_block_sums : ONE workgroup turns block_sums into its exclusive prefix
//                     (in place; must not run concurrently with any reader)
//   scan_apply      : adds block_sums[wg] (the exclusive prefix) to each block
//
// After all three, `output[i] = sum(input[0..=i])`.

const WG: u32 = 256u;
const ELEMENTS_PER_THREAD: u32 = 8u;
const BLOCK: u32 = WG * ELEMENTS_PER_THREAD;

struct ScanParams {
    num_elements: u32,
    num_blocks: u32,
    pad0: u32,
    pad1: u32,
};

@group(0) @binding(0) var<uniform> params: ScanParams;
@group(0) @binding(1) var<storage, read> input: array<u32>;
@group(0) @binding(2) var<storage, read_write> output: array<u32>;
@group(0) @binding(3) var<storage, read_write> block_sums: array<u32>;

var<workgroup> tile: array<u32, BLOCK>;

// In-place inclusive Hillis-Steele over the whole workgroup block.
fn scan_tile(tid: u32, n: u32) {
    var offset = 1u;
    loop {
        if (offset >= BLOCK) { break; }
        var staged: array<u32, ELEMENTS_PER_THREAD>;
        for (var e = 0u; e < ELEMENTS_PER_THREAD; e = e + 1u) {
            let i = tid + e * WG;
            var v = tile[i];
            if (i >= offset && i < n) {
                v = v + tile[i - offset];
            }
            staged[e] = v;
        }
        workgroupBarrier();
        for (var e = 0u; e < ELEMENTS_PER_THREAD; e = e + 1u) {
            let i = tid + e * WG;
            if (i < n) {
                tile[i] = staged[e];
            }
        }
        workgroupBarrier();
        offset = offset * 2u;
    }
}

@compute @workgroup_size(256)
fn scan_blocks(
    @builtin(workgroup_id) wid: vec3<u32>,
    @builtin(local_invocation_id) lid: vec3<u32>,
) {
    let base = wid.x * BLOCK;
    let n = min(params.num_elements - base, BLOCK);
    for (var e = 0u; e < ELEMENTS_PER_THREAD; e = e + 1u) {
        let i = lid.x + e * WG;
        if (i < n) {
            tile[i] = input[base + i];
        } else {
            tile[i] = 0u;
        }
    }
    workgroupBarrier();
    scan_tile(lid.x, n);
    for (var e = 0u; e < ELEMENTS_PER_THREAD; e = e + 1u) {
        let i = lid.x + e * WG;
        if (i < n) {
            output[base + i] = tile[i];
        }
    }
    if (lid.x == 0u && n > 0u) {
        block_sums[wid.x] = tile[n - 1u];
    }
}

// Exclusive prefix of block_sums, in place. Dispatch exactly one workgroup:
// with more, workgroups would race on the in-place rewrite (workgroupBarrier
// does not synchronise across workgroups). Thread 0 walks it serially
// (num_blocks is small: n / 2048).
@compute @workgroup_size(256)
fn scan_block_sums(@builtin(local_invocation_id) lid: vec3<u32>) {
    if (lid.x == 0u) {
        var acc = 0u;
        let nb = params.num_blocks;
        for (var b = 0u; b < nb; b = b + 1u) {
            let s = block_sums[b];
            block_sums[b] = acc;
            acc = acc + s;
        }
    }
}

@compute @workgroup_size(256)
fn scan_apply(
    @builtin(workgroup_id) wid: vec3<u32>,
    @builtin(local_invocation_id) lid: vec3<u32>,
) {
    let carry = block_sums[wid.x];
    let base = wid.x * BLOCK;
    for (var e = 0u; e < ELEMENTS_PER_THREAD; e = e + 1u) {
        let i = base + lid.x + e * WG;
        if (i < params.num_elements) {
            output[i] = output[i] + carry;
        }
    }
}
