// Sum each visible gaussian's per-isect screen gradients (written by
// rasterize_backward) into one 9-float record, indexed by compact id. A
// gaussian's isects are the contiguous unsorted range ending at
// cum_tiles_hit[c] (inclusive prefix sum of its tile counts), so this is a
// plain, deterministic loop -- no atomics.

@group(0) @binding(0) var<uniform> u: ProjectUniforms;
@group(0) @binding(1) var<storage, read> cum_tiles_hit: array<u32>;
@group(0) @binding(2) var<storage, read> isect_grads: array<f32>;
@group(0) @binding(3) var<storage, read_write> screen_grads: array<f32>;

@compute @workgroup_size(256)
fn grad_reduce(@builtin(global_invocation_id) gid3: vec3<u32>) {
    let c = gid3.x;
    if (c >= u.num_visible) {
        return;
    }
    let hi = min(cum_tiles_hit[c], u.num_intersections);
    var lo = 0u;
    if (c > 0u) {
        lo = min(cum_tiles_hit[c - 1u], u.num_intersections);
    }
    var acc: array<f32, 9>;
    for (var k = 0u; k < 9u; k = k + 1u) {
        acc[k] = 0.0;
    }
    for (var i = lo; i < hi; i = i + 1u) {
        for (var k = 0u; k < 9u; k = k + 1u) {
            acc[k] = acc[k] + isect_grads[i * 9u + k];
        }
    }
    for (var k = 0u; k < 9u; k = k + 1u) {
        screen_grads[c * 9u + k] = acc[k];
    }
}
