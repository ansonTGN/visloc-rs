// Backward of `rasterize`: per-(tile, splat) gradients of the screen-space
// splat parameters for L = sum_px dot(d_image[px], C[px]).
//
// One workgroup per tile, one thread per pixel, walking the tile's list back
// to front from the tile's last blended entry. Each pixel recovers the
// transmittance in front of a splat as T_before = T_after / (1 - alpha) from
// the forward residuals (final transmittance, last blended index) and keeps
// the colour composited behind it, so
//   dC/dcolour_i = alpha_i T_i,  dC/dalpha_i = T_i (colour_i - behind_i).
// alpha = min(0.99, opacity exp(-sigma)), sigma = 0.5 (A dx^2 + C dy^2) + B dx dy
// with d = pixel centre - mean; a clamped alpha passes no gradient to opacity,
// conic or mean.
//
// The 256 per-pixel contributions of a splat are summed with subgroupAdd and
// a shared-memory float add (CAS on u32 bits) across subgroups; the tile's
// total is then added to the gaussian's record in screen_grads with one
// global CAS float add per component (no per-intersection storage).
//
// Record layout (9 floats, per compact id): du, dv, dA, dB, dC, dopacity,
// dr, dg, db.

@group(0) @binding(0) var<uniform> u: RasterUniforms;
@group(0) @binding(1) var<storage, read> projected_splats: array<f32>;
@group(0) @binding(2) var<storage, read> compact_sorted: array<u32>;
@group(0) @binding(3) var<storage, read> tile_offsets: array<u32>;
@group(0) @binding(5) var<storage, read> final_t: array<f32>;
@group(0) @binding(6) var<storage, read> last_idx: array<u32>;
@group(0) @binding(7) var<storage, read> d_image: array<f32>;
// f32 bits, accumulated with CAS (no float atomics in wgpu here).
@group(0) @binding(8) var<storage, read_write> screen_grads: array<atomic<u32>>;

const TILE_W: u32 = 16u;
const TILE_H: u32 = 16u;
const BB: u32 = 128u;
const NG: u32 = 9u;

var<workgroup> bsplat: array<f32, BB * 9u>;
var<workgroup> bcompact: array<u32, BB>;
var<workgroup> gacc: array<atomic<u32>, BB * NG>;
var<workgroup> tile_last: atomic<u32>;
var<workgroup> tile_last_u: u32;

fn gacc_add(i: u32, v: f32) {
    var old = atomicLoad(&gacc[i]);
    loop {
        let r = atomicCompareExchangeWeak(&gacc[i], old, bitcast<u32>(bitcast<f32>(old) + v));
        if (r.exchanged) {
            break;
        }
        old = r.old_value;
    }
}

fn global_add(i: u32, v: f32) {
    var old = atomicLoad(&screen_grads[i]);
    loop {
        let r = atomicCompareExchangeWeak(&screen_grads[i], old, bitcast<u32>(bitcast<f32>(old) + v));
        if (r.exchanged) {
            break;
        }
        old = r.old_value;
    }
}

@compute @workgroup_size(256)
fn rasterize_backward(
    @builtin(workgroup_id) wid: vec3<u32>,
    @builtin(local_invocation_id) lid: vec3<u32>,
    @builtin(subgroup_invocation_id) lane: u32,
) {
    let tile = wid.x;
    let tid = lid.x;
    let tile_x = tile % u.tile_bw;
    let tile_y = tile / u.tile_bw;
    let px = tile_x * TILE_W + (tid % TILE_W);
    let py = tile_y * TILE_H + (tid / TILE_W);
    let sample_x = f32(px) + 0.5;
    let sample_y = f32(py) + 0.5;
    let in_image = px < u.img_w && py < u.img_h;

    let start = tile_offsets[tile * 2u];
    let end = tile_offsets[tile * 2u + 1u];

    var t_cur = 1.0;
    var last = 0u;
    var g = vec3<f32>(0.0, 0.0, 0.0);
    var behind = vec3<f32>(u.bg_r, u.bg_g, u.bg_b);
    if (in_image) {
        let pix = py * u.img_w + px;
        t_cur = final_t[pix];
        last = last_idx[pix];
        g = vec3<f32>(d_image[pix * 3u], d_image[pix * 3u + 1u], d_image[pix * 3u + 2u]);
    }
    if (tid == 0u) {
        atomicStore(&tile_last, 0u);
    }
    workgroupBarrier();
    atomicMax(&tile_last, last);
    workgroupBarrier();
    if (tid == 0u) {
        tile_last_u = atomicLoad(&tile_last);
    }
    // Empty tiles have start = u32::MAX, end = 0; their pixels have last = 0.
    let stop = min(workgroupUniformLoad(&tile_last_u), end);

    var bend = stop;
    loop {
        if (bend <= start || bend == 0u) { break; }
        var bstart = start;
        if (bend - start > BB) {
            bstart = bend - BB;
        }
        let cnt = bend - bstart;
        if (tid < cnt) {
            let j = bstart + tid;
            let cg = compact_sorted[j];
            let src = cg * 9u;
            for (var k = 0u; k < 9u; k = k + 1u) {
                bsplat[tid * 9u + k] = projected_splats[src + k];
            }
            bcompact[tid] = cg;
        }
        for (var i = tid; i < BB * NG; i = i + 256u) {
            atomicStore(&gacc[i], 0u);
        }
        workgroupBarrier();

        var s = cnt;
        loop {
            if (s == 0u) { break; }
            s = s - 1u;
            let j = bstart + s;
            var gr: array<f32, 9>;
            for (var k = 0u; k < 9u; k = k + 1u) {
                gr[k] = 0.0;
            }
            if (in_image && j < last) {
                let opac = bsplat[s * 9u + 5u];
                let dx = sample_x - bsplat[s * 9u + 0u];
                let dy = sample_y - bsplat[s * 9u + 1u];
                let c00 = bsplat[s * 9u + 2u];
                let c01 = bsplat[s * 9u + 3u];
                let c11 = bsplat[s * 9u + 4u];
                // Same expressions as the forward, so the same splats pass.
                let sigma = 0.5 * (c00 * dx * dx + c11 * dy * dy) + c01 * dx * dy;
                if (sigma >= 0.0) {
                    let e = exp(-sigma);
                    let raw = opac * e;
                    let a = min(0.99, raw);
                    if (a >= 1.0 / 255.0) {
                        let t_before = t_cur / (1.0 - a);
                        let col = max(
                            vec3<f32>(bsplat[s * 9u + 6u], bsplat[s * 9u + 7u], bsplat[s * 9u + 8u]),
                            vec3<f32>(0.0, 0.0, 0.0),
                        );
                        let dcol = a * t_before * g;
                        let d_alpha = t_before * dot(col - behind, g);
                        behind = a * col + (1.0 - a) * behind;
                        t_cur = t_before;
                        if (raw <= 0.99) {
                            let d_sigma = -d_alpha * a;
                            gr[0] = d_sigma * -(c00 * dx + c01 * dy);
                            gr[1] = d_sigma * -(c01 * dx + c11 * dy);
                            gr[2] = d_sigma * 0.5 * dx * dx;
                            gr[3] = d_sigma * dx * dy;
                            gr[4] = d_sigma * 0.5 * dy * dy;
                            gr[5] = d_alpha * e;
                        }
                        gr[6] = dcol.x;
                        gr[7] = dcol.y;
                        gr[8] = dcol.z;
                    }
                }
            }
            // Uniform control flow: every lane of every subgroup reaches this.
            for (var k = 0u; k < 9u; k = k + 1u) {
                let tot = subgroupAdd(gr[k]);
                if (lane == 0u && tot != 0.0) {
                    gacc_add(s * NG + k, tot);
                }
            }
        }
        workgroupBarrier();
        for (var i = tid; i < cnt * NG; i = i + 256u) {
            let v = bitcast<f32>(atomicLoad(&gacc[i]));
            if (v != 0.0) {
                global_add(bcompact[i / NG] * NG + i % NG, v);
            }
        }
        // bsplat / bcompact / gacc are reused by the next batch.
        workgroupBarrier();
        bend = bstart;
    }
}
