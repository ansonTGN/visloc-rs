// Pass 4: from the tile-sorted `tile_id_from_isect`, produce per-tile
// `[start, end)` ranges into the sorted isect list.
//
// The list is sorted by tile, so only run boundaries matter: the element that
// starts a run writes the tile's start and the one that ends it writes the end.
// Every write has a unique owner, so no atomics are needed (the previous
// atomicMin/atomicMax version serialised thousands of threads per tile).
// Empty tiles keep the host-initialised (u32::MAX, 0) range.
//
// Dispatched in 2D (see `dispatch_threads`) so the isect count may exceed
// 65535 * 256.

@group(0) @binding(0) var<uniform> u: ProjectUniforms;
@group(0) @binding(1) var<storage, read> tile_id_from_isect: array<u32>;
// tile_offsets is [num_tiles * 2]: (start, end) pairs.
@group(0) @binding(2) var<storage, read_write> tile_offsets: array<u32>;

@compute @workgroup_size(256)
fn get_tile_offsets(
    @builtin(global_invocation_id) gid3: vec3<u32>,
    @builtin(num_workgroups) nwg: vec3<u32>,
) {
    let i = gid3.x + gid3.y * nwg.x * 256u;
    let n = u.num_intersections;
    if (i >= n) {
        return;
    }
    let tile = tile_id_from_isect[i];
    let num_tiles = u.tile_bw * u.tile_bh;
    if (tile >= num_tiles) {
        return;
    }
    if (i == 0u || tile_id_from_isect[i - 1u] != tile) {
        tile_offsets[tile * 2u] = i;
    }
    if (i + 1u == n || tile_id_from_isect[i + 1u] != tile) {
        tile_offsets[tile * 2u + 1u] = i + 1u;
    }
}
