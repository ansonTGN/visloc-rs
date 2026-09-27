// brush's auxiliary losses (brush-train 0.3 `train.rs`), added to the
// parameter gradients after the backward pass:
//   opacity: w_o * sum_i raw_opacity_i * v_i
//   scale:   w_s / median_size * sum_i sum_k exp(log_scale_ik) * v_i
// with v_i = 1 + 1e-3 for gaussians visible this frame and 1e-3 otherwise
// ("invisible splats still have a loss, otherwise they would never die
// off"). Tiny weights, but Adam normalises: a gaussian no view sees gets only
// these gradients, so it fades and shrinks until refine prunes it.
//
// `aux_all` adds the 1e-3 part to every gaussian, `aux_visible` the rest to
// the visible ones (compact order).

struct AuxUniforms {
    n: u32,
    num_visible: u32,
    // w_o and w_s / median_size, already times the time ramp
    opac_coef: f32,
    scale_coef: f32,
};

@group(0) @binding(0) var<uniform> au: AuxUniforms;
@group(0) @binding(1) var<storage, read> a_transforms: array<f32>;
@group(0) @binding(2) var<storage, read_write> a_grad_transforms: array<f32>;
@group(0) @binding(3) var<storage, read_write> a_grad_opacity: array<f32>;
@group(0) @binding(4) var<storage, read> a_global_from_compact: array<u32>;

fn add_aux(gid: u32, v: f32) {
    a_grad_opacity[gid] = a_grad_opacity[gid] + au.opac_coef * v;
    let b = gid * 10u + 7u;
    for (var k = 0u; k < 3u; k = k + 1u) {
        a_grad_transforms[b + k] = a_grad_transforms[b + k] + au.scale_coef * v * exp(a_transforms[b + k]);
    }
}

@compute @workgroup_size(256)
fn aux_all(
    @builtin(global_invocation_id) gid3: vec3<u32>,
    @builtin(num_workgroups) nwg: vec3<u32>,
) {
    let i = gid3.x + gid3.y * nwg.x * 256u;
    if (i >= au.n) {
        return;
    }
    add_aux(i, 1e-3);
}

@compute @workgroup_size(256)
fn aux_visible(
    @builtin(global_invocation_id) gid3: vec3<u32>,
    @builtin(num_workgroups) nwg: vec3<u32>,
) {
    let c = gid3.x + gid3.y * nwg.x * 256u;
    if (c >= au.num_visible) {
        return;
    }
    add_aux(a_global_from_compact[c], 1.0);
}
