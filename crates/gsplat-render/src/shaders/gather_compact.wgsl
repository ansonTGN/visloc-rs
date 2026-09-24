// After the tile sort: compact_sorted[j] = compact_gid_from_isect[isect_id[j]],
// the compact gaussian id of each tile-sorted list entry (what `rasterize`
// walks). Dispatched in 2D (see `dispatch_threads`).

@group(0) @binding(0) var<uniform> u: ProjectUniforms;
@group(0) @binding(1) var<storage, read> compact_gid_from_isect: array<u32>;
@group(0) @binding(2) var<storage, read> isect_id: array<u32>;
@group(0) @binding(3) var<storage, read_write> compact_sorted: array<u32>;

@compute @workgroup_size(256)
fn gather_compact(
    @builtin(global_invocation_id) gid3: vec3<u32>,
    @builtin(num_workgroups) nwg: vec3<u32>,
) {
    let j = gid3.x + gid3.y * nwg.x * 256u;
    if (j >= u.num_intersections) {
        return;
    }
    compact_sorted[j] = compact_gid_from_isect[isect_id[j]];
}
