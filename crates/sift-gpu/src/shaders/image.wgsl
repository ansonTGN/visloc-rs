// Scale-space image kernels (f32 ports of visloc-vision's legacy SIFT
// `double_up`, separable `blur`, `halve` and DoG). All layers of an octave
// live in one buffer at `level * w * h`; coordinates are clamped at the
// borders exactly like `Layer::get`.

struct ImageParams {
    w: u32,
    h: u32,
    src_off: u32,
    dst_off: u32,
    radius: u32,
    wt_off: u32,
    src_w: u32,
    src_h: u32,
};

@group(0) @binding(0) var<uniform> p: ImageParams;
@group(0) @binding(1) var<storage, read> src: array<f32>;
@group(0) @binding(2) var<storage, read_write> dst: array<f32>;
@group(0) @binding(3) var<storage, read> weights: array<f32>;

fn src_at(x: i32, y: i32, w: u32, h: u32) -> f32 {
    let xi = u32(clamp(x, 0, i32(w) - 1));
    let yi = u32(clamp(y, 0, i32(h) - 1));
    return src[p.src_off + yi * w + xi];
}

// Nearest-neighbour 2x upsampling at half-sample offsets (`double_up`).
//
// NOTE: this indexes the source with `sx = (x+2)/2` (integer division),
// which puts source pixel `i` at doubled indices `{2i-2, 2i-1}` instead of
// the `{2i, 2i+1}` the keypoint-coordinate formula (`x_orig = x_doubled *
// upsample`) assumes. That is a ~0.7-original-pixel systematic bias,
// confirmed against COLMAP's own keypoints
// (`docs/euroc_gpu_sfm_vs_colmap.md`, 2026-09-28). Kept as-is for the
// legacy byte-identical contract; `upsample2x_aligned` below is the fix.
@compute @workgroup_size(16, 16)
fn upsample2x(@builtin(global_invocation_id) g: vec3<u32>) {
    if (g.x >= p.w || g.y >= p.h) {
        return;
    }
    let sx = min((g.x + 2u) / 2u, p.src_w - 1u);
    let sy = min((g.y + 2u) / 2u, p.src_h - 1u);
    dst[p.dst_off + g.y * p.w + g.x] = src[p.src_off + sy * p.src_w + sx];
}

// Align-corners bilinear 2x upsampling: `dst(2i) = src(i)`,
// `dst(2i+1) = lerp(src(i), src(i+1), 0.5)`, matching VLFeat/COLMAP's
// `copy_and_upsample_rows` and the CPU `double_up_vlfeat` this mirrors.
// Unbiased: doubled index `x` truly corresponds to original coordinate
// `x/2`, consistent with the keypoint-coordinate formula.
@compute @workgroup_size(16, 16)
fn upsample2x_aligned(@builtin(global_invocation_id) g: vec3<u32>) {
    if (g.x >= p.w || g.y >= p.h) {
        return;
    }
    let sx0 = min(g.x / 2u, p.src_w - 1u);
    let sx1 = min(sx0 + 1u, p.src_w - 1u);
    let sy0 = min(g.y / 2u, p.src_h - 1u);
    let sy1 = min(sy0 + 1u, p.src_h - 1u);
    let fx = select(0.0, 0.5, (g.x % 2u) == 1u);
    let fy = select(0.0, 0.5, (g.y % 2u) == 1u);
    let p00 = src[p.src_off + sy0 * p.src_w + sx0];
    let p10 = src[p.src_off + sy0 * p.src_w + sx1];
    let p01 = src[p.src_off + sy1 * p.src_w + sx0];
    let p11 = src[p.src_off + sy1 * p.src_w + sx1];
    let top = p00 * (1.0 - fx) + p10 * fx;
    let bot = p01 * (1.0 - fx) + p11 * fx;
    dst[p.dst_off + g.y * p.w + g.x] = top * (1.0 - fy) + bot * fy;
}

@compute @workgroup_size(16, 16)
fn blur_h(@builtin(global_invocation_id) g: vec3<u32>) {
    if (g.x >= p.w || g.y >= p.h) {
        return;
    }
    let r = i32(p.radius);
    var acc = 0.0;
    for (var k = -r; k <= r; k = k + 1) {
        acc = acc + weights[p.wt_off + u32(k + r)] * src_at(i32(g.x) + k, i32(g.y), p.w, p.h);
    }
    dst[p.dst_off + g.y * p.w + g.x] = acc;
}

@compute @workgroup_size(16, 16)
fn blur_v(@builtin(global_invocation_id) g: vec3<u32>) {
    if (g.x >= p.w || g.y >= p.h) {
        return;
    }
    let r = i32(p.radius);
    var acc = 0.0;
    for (var k = -r; k <= r; k = k + 1) {
        acc = acc + weights[p.wt_off + u32(k + r)] * src_at(i32(g.x), i32(g.y) + k, p.w, p.h);
    }
    dst[p.dst_off + g.y * p.w + g.x] = acc;
}

// Decimate by two (`halve`): dst(x, y) = src(2x, 2y).
@compute @workgroup_size(16, 16)
fn halve(@builtin(global_invocation_id) g: vec3<u32>) {
    if (g.x >= p.w || g.y >= p.h) {
        return;
    }
    dst[p.dst_off + g.y * p.w + g.x] = src_at(i32(2u * g.x), i32(2u * g.y), p.src_w, p.src_h);
}

// DoG level z = gaussian(z + 1) - gaussian(z).
@compute @workgroup_size(16, 16)
fn dog(@builtin(global_invocation_id) g: vec3<u32>) {
    if (g.x >= p.w || g.y >= p.h) {
        return;
    }
    let n = p.w * p.h;
    let i = g.y * p.w + g.x;
    dst[g.z * n + i] = src[(g.z + 1u) * n + i] - src[g.z * n + i];
}
