// Adam step over one parameter buffer, in place.
//
// The buffer is a sequence of fixed-size records (`stride` floats, e.g. 10
// for mean|quat|log-scale); element `k = i % stride` uses learning rate
// lr_a for k < split_a, lr_b for k < split_b, else lr_c, so one dispatch
// covers parameter groups that share a buffer. Bias corrections are passed in
// as bc1 = 1 - beta1^t, bc2 = 1 - beta2^t. Dispatched in 2D.

struct AdamUniforms {
    n: u32,
    stride: u32,
    split_a: u32,
    split_b: u32,
    lr_a: f32,
    lr_b: f32,
    lr_c: f32,
    beta1: f32,
    beta2: f32,
    eps: f32,
    bc1: f32,
    bc2: f32,
};

@group(0) @binding(0) var<uniform> au: AdamUniforms;
// Four consecutive elements per thread: every buffer is allocated as a
// 16-byte multiple, and elements past `n` in the last vec4 are zero-gradient
// padding (their update is exactly zero).
@group(0) @binding(1) var<storage, read_write> params: array<vec4<f32>>;
// Read and zeroed: the renderer skips its own gradient clear
// (`Renderer::set_grads_zeroed_by_caller`).
@group(0) @binding(2) var<storage, read_write> grads: array<vec4<f32>>;
@group(0) @binding(3) var<storage, read_write> m1: array<vec4<f32>>;
@group(0) @binding(4) var<storage, read_write> m2: array<vec4<f32>>;

fn lr_of(e: u32) -> f32 {
    let k = e % au.stride;
    if (k < au.split_a) {
        return au.lr_a;
    }
    if (k < au.split_b) {
        return au.lr_b;
    }
    return au.lr_c;
}

@compute @workgroup_size(256)
fn adam(
    @builtin(global_invocation_id) gid3: vec3<u32>,
    @builtin(num_workgroups) nwg: vec3<u32>,
) {
    let i = gid3.x + gid3.y * nwg.x * 256u;
    if (i >= (au.n + 3u) / 4u) {
        return;
    }
    let e = 4u * i;
    let lr = vec4<f32>(lr_of(e), lr_of(e + 1u), lr_of(e + 2u), lr_of(e + 3u));
    let g = grads[i];
    grads[i] = vec4<f32>(0.0);
    let m = au.beta1 * m1[i] + (1.0 - au.beta1) * g;
    let v = au.beta2 * m2[i] + (1.0 - au.beta2) * g * g;
    m1[i] = m;
    m2[i] = v;
    params[i] = params[i] - lr * (m / au.bc1) / (sqrt(v / au.bc2) + au.eps);
}
