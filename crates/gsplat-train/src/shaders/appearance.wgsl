// Per-image appearance: an affine colour transform C' = A C + b (3x4, one
// per training view, identity at start) applied to the render before the
// photometric loss. It absorbs per-photo exposure / white balance, so the
// splat does not grow floaters or bake view-dependent colour to explain it.
// Held-out views are rendered without it (identity).
//
//   app_apply     C' = A C + b into `corr`             (per pixel)
//   app_backward  d_image <- A^T dL/dC', and dL/dA, dL/db summed over the
//                 image into `grad` (workgroup tree sum + one CAS add per
//                 workgroup and component)            (per pixel)
//   app_adam      Adam on this view's 12 parameters (per-view step count),
//                 with a weak pull towards identity; clears `grad`.

struct AppUniforms {
    npix: u32,
    view: u32,
    lr: f32,
    // weight of 0.5 * reg * |p - identity|^2
    reg: f32,
};

@group(0) @binding(0) var<uniform> au: AppUniforms;
// 12 floats per view, row-major [A | b]: a00 a01 a02 b0 a10 ... b2.
@group(0) @binding(1) var<storage, read_write> params: array<f32>;
@group(0) @binding(2) var<storage, read> render: array<f32>;
@group(0) @binding(3) var<storage, read_write> corr: array<f32>;
@group(0) @binding(4) var<storage, read_write> d_image: array<f32>;
@group(0) @binding(5) var<storage, read_write> grad: array<atomic<u32>>;
@group(0) @binding(6) var<storage, read_write> m1: array<f32>;
@group(0) @binding(7) var<storage, read_write> m2: array<f32>;
@group(0) @binding(8) var<storage, read_write> steps: array<u32>;

fn pixel_id(gid3: vec3<u32>, nwg: vec3<u32>) -> u32 {
    return gid3.x + gid3.y * nwg.x * 256u;
}

fn row(r: u32) -> vec4<f32> {
    let o = au.view * 12u + r * 4u;
    return vec4<f32>(params[o], params[o + 1u], params[o + 2u], params[o + 3u]);
}

@compute @workgroup_size(256)
fn app_apply(
    @builtin(global_invocation_id) gid3: vec3<u32>,
    @builtin(num_workgroups) nwg: vec3<u32>,
) {
    let i = pixel_id(gid3, nwg);
    if (i >= au.npix) {
        return;
    }
    let c = vec4<f32>(render[i * 3u], render[i * 3u + 1u], render[i * 3u + 2u], 1.0);
    corr[i * 3u] = dot(row(0u), c);
    corr[i * 3u + 1u] = dot(row(1u), c);
    corr[i * 3u + 2u] = dot(row(2u), c);
}

var<workgroup> part: array<f32, 3072>; // 256 x 12

fn grad_add(k: u32, v: f32) {
    var old = atomicLoad(&grad[k]);
    loop {
        let r = atomicCompareExchangeWeak(&grad[k], old, bitcast<u32>(bitcast<f32>(old) + v));
        if (r.exchanged) {
            break;
        }
        old = r.old_value;
    }
}

@compute @workgroup_size(256)
fn app_backward(
    @builtin(global_invocation_id) gid3: vec3<u32>,
    @builtin(num_workgroups) nwg: vec3<u32>,
    @builtin(local_invocation_id) lid: vec3<u32>,
) {
    let i = pixel_id(gid3, nwg);
    let t = lid.x;
    var g = vec3<f32>(0.0);
    var c = vec4<f32>(0.0);
    if (i < au.npix) {
        g = vec3<f32>(d_image[i * 3u], d_image[i * 3u + 1u], d_image[i * 3u + 2u]);
        c = vec4<f32>(render[i * 3u], render[i * 3u + 1u], render[i * 3u + 2u], 1.0);
        let r0 = row(0u);
        let r1 = row(1u);
        let r2 = row(2u);
        // dL/dC = A^T dL/dC'
        d_image[i * 3u] = r0.x * g.x + r1.x * g.y + r2.x * g.z;
        d_image[i * 3u + 1u] = r0.y * g.x + r1.y * g.y + r2.y * g.z;
        d_image[i * 3u + 2u] = r0.z * g.x + r1.z * g.y + r2.z * g.z;
    }
    // dL/d[A | b] row r = g_r * (C, 1)
    for (var r = 0u; r < 3u; r = r + 1u) {
        let v = g[r] * c;
        part[t * 12u + r * 4u] = v.x;
        part[t * 12u + r * 4u + 1u] = v.y;
        part[t * 12u + r * 4u + 2u] = v.z;
        part[t * 12u + r * 4u + 3u] = v.w;
    }
    workgroupBarrier();
    for (var s = 128u; s > 0u; s = s >> 1u) {
        if (t < s) {
            for (var k = 0u; k < 12u; k = k + 1u) {
                part[t * 12u + k] = part[t * 12u + k] + part[(t + s) * 12u + k];
            }
        }
        workgroupBarrier();
    }
    if (t < 12u) {
        grad_add(t, part[t]);
    }
}

@compute @workgroup_size(16)
fn app_adam(@builtin(local_invocation_id) lid: vec3<u32>) {
    let k = lid.x;
    let step = steps[au.view] + 1u;
    workgroupBarrier();
    if (k < 12u) {
        let o = au.view * 12u + k;
        let identity = select(0.0, 1.0, k == 0u || k == 5u || k == 10u);
        let g = bitcast<f32>(atomicLoad(&grad[k])) + au.reg * (params[o] - identity);
        atomicStore(&grad[k], 0u);
        let m = 0.9 * m1[o] + 0.1 * g;
        let v = 0.999 * m2[o] + 0.001 * g * g;
        m1[o] = m;
        m2[o] = v;
        let mh = m / (1.0 - pow(0.9, f32(step)));
        let vh = v / (1.0 - pow(0.999, f32(step)));
        params[o] = params[o] - au.lr * mh / (sqrt(vh) + 1e-15);
    }
    workgroupBarrier();
    if (k == 0u) {
        steps[au.view] = step;
    }
}
