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
@compute @workgroup_size(16, 16)
fn upsample2x(@builtin(global_invocation_id) g: vec3<u32>) {
    if (g.x >= p.w || g.y >= p.h) {
        return;
    }
    let sx = min((g.x + 2u) / 2u, p.src_w - 1u);
    let sy = min((g.y + 2u) / 2u, p.src_h - 1u);
    dst[p.dst_off + g.y * p.w + g.x] = src[p.src_off + sy * p.src_w + sx];
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
