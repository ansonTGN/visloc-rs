// Batched brute-force top-2 nearest neighbours over a device-resident
// descriptor bank (the GPU half of visloc-vision's `BruteForceMatcher`).
//
// Each entry names a query block and a train block of the bank; for every
// query row it finds the two smallest scores ||t||^2 - 2 q.t (= ||q - t||^2
// - ||q||^2, the CPU matcher's ranking) and the best train index. Ties go
// to the lower index, like the CPU's strict `<` scan. The host turns
// scores into distances, applies the ratio test and cross-checks with the
// swapped entry.
//
// One workgroup = 64 query rows x all train rows of one entry, in 64-row
// train tiles; a classic shared-memory GEMM with 32-wide k chunks, each
// thread holding a 4 x 4 register tile (rows ty + 16 i, cols tx + 16 j).

struct MatchParams {
    // vec4s per descriptor row (dim / 4); dim is a multiple of 32.
    dim4: u32,
    num_entries: u32,
    pad0: u32,
    pad1: u32,
};

struct Entry {
    q_off: u32,
    nq: u32,
    t_off: u32,
    nt: u32,
    out_off: u32,
    pad0: u32,
    pad1: u32,
    pad2: u32,
};

@group(0) @binding(0) var<uniform> p: MatchParams;
@group(0) @binding(1) var<storage, read> desc: array<vec4<f32>>;
@group(0) @binding(2) var<storage, read> norms: array<f32>;
@group(0) @binding(3) var<storage, read> entries: array<Entry>;
// Per query row: best index, best score, second score, second index.
@group(0) @binding(4) var<storage, read_write> out: array<vec4<u32>>;

const BIG: f32 = 3.0e38;
const NONE: u32 = 0xFFFFFFFFu;

// k-major tiles: qs[k * 64 + row].
var<workgroup> qs: array<f32, 2048>;
var<workgroup> ts: array<f32, 2048>;
var<workgroup> red_i1: array<u32, 1024>;
var<workgroup> red_i2: array<u32, 1024>;

struct Top2 {
    s1: f32,
    i1: u32,
    s2: f32,
    i2: u32,
};

fn before(sa: f32, ia: u32, sb: f32, ib: u32) -> bool {
    return sa < sb || (sa == sb && ia < ib);
}

fn insert(t: Top2, s: f32, i: u32) -> Top2 {
    var r = t;
    if (before(s, i, r.s1, r.i1)) {
        r.s2 = r.s1;
        r.i2 = r.i1;
        r.s1 = s;
        r.i1 = i;
    } else if (before(s, i, r.s2, r.i2)) {
        r.s2 = s;
        r.i2 = i;
    }
    return r;
}

fn load_tile(dst_is_q: bool, row_off: u32, nrows: u32, row0: u32, kc: u32, t: u32) {
    // 64 rows x 8 vec4 = 512 vec4, two per thread.
    for (var s = 0u; s < 2u; s = s + 1u) {
        let idx = t + 256u * s;
        let row = idx / 8u;
        let v = idx % 8u;
        var val = vec4<f32>(0.0);
        if (row0 + row < nrows) {
            val = desc[(row_off + row0 + row) * p.dim4 + kc * 8u + v];
        }
        let k = v * 4u;
        if (dst_is_q) {
            qs[(k + 0u) * 64u + row] = val.x;
            qs[(k + 1u) * 64u + row] = val.y;
            qs[(k + 2u) * 64u + row] = val.z;
            qs[(k + 3u) * 64u + row] = val.w;
        } else {
            ts[(k + 0u) * 64u + row] = val.x;
            ts[(k + 1u) * 64u + row] = val.y;
            ts[(k + 2u) * 64u + row] = val.z;
            ts[(k + 3u) * 64u + row] = val.w;
        }
    }
}

@compute @workgroup_size(256)
fn top2(@builtin(workgroup_id) wid: vec3<u32>, @builtin(local_invocation_id) lid: vec3<u32>) {
    let e = entries[wid.y];
    let row0 = wid.x * 64u;
    if (row0 >= e.nq) {
        return;
    }
    let t = lid.x;
    let tx = t % 16u;
    let ty = t / 16u;
    let nchunks = p.dim4 / 8u;
    var r0 = Top2(BIG, NONE, BIG, NONE);
    var r1 = Top2(BIG, NONE, BIG, NONE);
    var r2 = Top2(BIG, NONE, BIG, NONE);
    var r3 = Top2(BIG, NONE, BIG, NONE);

    for (var col0 = 0u; col0 < e.nt; col0 = col0 + 64u) {
        var a0 = vec4<f32>(0.0);
        var a1 = vec4<f32>(0.0);
        var a2 = vec4<f32>(0.0);
        var a3 = vec4<f32>(0.0);
        for (var kc = 0u; kc < nchunks; kc = kc + 1u) {
            load_tile(true, e.q_off, e.nq, row0, kc, t);
            load_tile(false, e.t_off, e.nt, col0, kc, t);
            workgroupBarrier();
            for (var k = 0u; k < 32u; k = k + 1u) {
                let b = vec4<f32>(ts[k * 64u + tx], ts[k * 64u + tx + 16u],
                    ts[k * 64u + tx + 32u], ts[k * 64u + tx + 48u]);
                a0 = a0 + qs[k * 64u + ty] * b;
                a1 = a1 + qs[k * 64u + ty + 16u] * b;
                a2 = a2 + qs[k * 64u + ty + 32u] * b;
                a3 = a3 + qs[k * 64u + ty + 48u] * b;
            }
            workgroupBarrier();
        }
        for (var j = 0u; j < 4u; j = j + 1u) {
            let col = col0 + tx + 16u * j;
            if (col < e.nt) {
                let tn = norms[e.t_off + col];
                r0 = insert(r0, tn - 2.0 * a0[j], col);
                r1 = insert(r1, tn - 2.0 * a1[j], col);
                r2 = insert(r2, tn - 2.0 * a2[j], col);
                r3 = insert(r3, tn - 2.0 * a3[j], col);
            }
        }
    }

    // Reduce the 16 column-threads of each row: (row, tx) partials.
    let rows = array<u32, 4>(ty, ty + 16u, ty + 32u, ty + 48u);
    let parts = array<Top2, 4>(r0, r1, r2, r3);
    for (var i = 0u; i < 4u; i = i + 1u) {
        let slot = rows[i] * 16u + tx;
        qs[slot] = parts[i].s1;
        ts[slot] = parts[i].s2;
        red_i1[slot] = parts[i].i1;
        red_i2[slot] = parts[i].i2;
    }
    workgroupBarrier();
    if (t < 64u && row0 + t < e.nq) {
        var r = Top2(BIG, NONE, BIG, NONE);
        for (var x = 0u; x < 16u; x = x + 1u) {
            let slot = t * 16u + x;
            r = insert(r, qs[slot], red_i1[slot]);
            r = insert(r, ts[slot], red_i2[slot]);
        }
        out[e.out_off + row0 + t] = vec4<u32>(r.i1, bitcast<u32>(r.s1), bitcast<u32>(r.s2), r.i2);
    }
}
