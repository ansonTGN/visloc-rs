// On-device adaptive density control (the Inria clone / split / prune rule of
// crate::densify, without the host round-trip).
//
//   densify_classify: per gaussian, an action and its output row count
//       0 prune (0 rows), 1 keep (1), 2 clone (keep + a copy, 2), 3 split
//       (two children drawn from the parent, 2); the counts are then prefix
//       summed on the device to give each gaussian its output offset.
//   densify_scatter: one dispatch per parameter group (transforms | opacity |
//       SH), copying rows and Adam moments to the new buffers. Kept rows keep
//       their moments, new rows start at zero; split children get a mean
//       sampled from the parent (hashed normals) and scales / 1.6. An opacity
//       reset clamps every opacity logit and zeroes that group's moments.

struct ClassifyUniforms {
    n: u32,
    grow: u32,
    prune_large: u32,
    pad0: u32,
    grad_threshold: f32,
    big: f32,
    min_opacity: f32,
    max_scale: f32,
};

@group(0) @binding(0) var<uniform> cu: ClassifyUniforms;
@group(0) @binding(1) var<storage, read> c_transforms: array<f32>;
@group(0) @binding(2) var<storage, read> c_opacity: array<f32>;
@group(0) @binding(3) var<storage, read> grad_accum: array<f32>;
@group(0) @binding(4) var<storage, read> grad_count: array<f32>;
@group(0) @binding(5) var<storage, read_write> actions: array<u32>;
@group(0) @binding(6) var<storage, read_write> counts: array<u32>;
// cloned, split, pruned
@group(0) @binding(7) var<storage, read_write> tallies: array<atomic<u32>, 3>;

@compute @workgroup_size(256)
fn densify_classify(
    @builtin(global_invocation_id) gid3: vec3<u32>,
    @builtin(num_workgroups) nwg: vec3<u32>,
) {
    let i = gid3.x + gid3.y * nwg.x * 256u;
    if (i >= cu.n) {
        return;
    }
    let b = i * 10u;
    let max_scale = exp(max(c_transforms[b + 7u], max(c_transforms[b + 8u], c_transforms[b + 9u])));
    let opacity = 1.0 / (1.0 + exp(-c_opacity[i]));
    var grad = 0.0;
    if (grad_count[i] > 0.0) {
        grad = grad_accum[i] / grad_count[i];
    }
    let dense = cu.grow == 1u && grad > cu.grad_threshold;
    var action = 1u;
    var count = 1u;
    if (dense && max_scale > cu.big) {
        action = 3u;
        count = 2u;
        atomicAdd(&tallies[1], 1u);
    } else if (cu.grow == 1u
        && (opacity < cu.min_opacity || (cu.prune_large == 1u && max_scale > cu.max_scale))) {
        action = 0u;
        count = 0u;
        atomicAdd(&tallies[2], 1u);
    } else if (dense) {
        action = 2u;
        count = 2u;
        atomicAdd(&tallies[0], 1u);
    }
    actions[i] = action;
    counts[i] = count;
}

