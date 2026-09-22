// Pass 4: from the tile-sorted `tile_id_from_isect`, produce per-tile
// `[start, end)` ranges into the sorted isect list.
//
// Many threads may touch the same tile, so the start/end updates are atomic.

@group(0) @binding(0) var<uniform> u: ProjectUniforms;
@group(0) @binding(1) var<storage, read> tile_id_from_isect: array<u32>;
// tile_offsets is [num_tiles * 2]: (start, end) pairs.
@group(0) @binding(2) var<storage, read_write> tile_offsets: array<atomic<u32>>;

@compute @workgroup_size(256)
fn get_tile_offsets(@builtin(global_invocation_id) gid3: vec3<u32>) {
    let i = gid3.x;
    if (i >= u.num_intersections) {
        return;
    }
    let tile = tile_id_from_isect[i];
    let num_tiles = u.tile_bw * u.tile_bh;
    if (tile >= num_tiles) {
        return;
    }
    atomicMin(&tile_offsets[tile * 2u], i);
    atomicMax(&tile_offsets[tile * 2u + 1u], i + 1u);
}
