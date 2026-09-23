// Pass 1: project every gaussian and build a compacted list of visible ones.
//
// One invocation per global gaussian. Visible gaussians are appended to the
// compact list via a global atomic counter; each stores its camera-space depth
// (for sorting) and the number of screen tiles its 3-sigma ellipse overlaps
// (for the tile-range scan).

@group(0) @binding(0) var<uniform> u: ProjectUniforms;
@group(0) @binding(1) var<storage, read> transforms: array<f32>;
@group(0) @binding(2) var<storage, read> opacity_in: array<f32>;
@group(0) @binding(3) var<storage, read_write> global_from_compact: array<u32>;
@group(0) @binding(4) var<storage, read_write> depths: array<f32>;
@group(0) @binding(5) var<storage, read_write> intersect_counts: array<u32>;
@group(0) @binding(6) var<storage, read_write> num_visible: atomic<u32>;
@group(0) @binding(7) var<storage, read_write> num_intersections: atomic<u32>;

@compute @workgroup_size(256)
fn project_forward(@builtin(global_invocation_id) gid3: vec3<u32>) {
    let gid = gid3.x;
    if (gid >= u.total_splats) {
        return;
    }
    intersect_counts[gid] = 0u;

    let base = gid * 10u;
    let mean = vec3<f32>(transforms[base], transforms[base + 1u], transforms[base + 2u]);
    let ls = vec3<f32>(transforms[base + 7u], transforms[base + 8u], transforms[base + 9u]);
    let p = compute_projected(
        u, mean,
        transforms[base + 3u], transforms[base + 4u],
        transforms[base + 5u], transforms[base + 6u],
        ls, opacity_in[gid],
    );
    if (!p.ok) {
        return;
    }

    let tile_count = tile_hits(p, u.img_w, u.img_h);
    intersect_counts[gid] = tile_count;
    atomicAdd(&num_intersections, tile_count);
    let slot = atomicAdd(&num_visible, 1u);
    global_from_compact[slot] = gid;
    depths[slot] = p.depth;
}
