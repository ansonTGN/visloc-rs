// Pass 3: expand each visible gaussian into one `(tile_id, compact_gid)` entry
// per screen tile its 3-sigma ellipse overlaps.
//
// `cum_tiles_hit` is the *inclusive* prefix sum of `intersect_counts` over the
// depth-sorted compact order, so the base offset for compact index `c` is
// `cum_tiles_hit[c] - intersect_counts[c]`.

@group(0) @binding(0) var<uniform> u: ProjectUniforms;
@group(0) @binding(1) var<storage, read> transforms: array<f32>;
@group(0) @binding(2) var<storage, read> opacity_in: array<f32>;
@group(0) @binding(3) var<storage, read> cum_tiles_hit: array<u32>;
@group(0) @binding(4) var<storage, read> global_from_compact: array<u32>;
@group(0) @binding(5) var<storage, read_write> tile_id_from_isect: array<u32>;
@group(0) @binding(6) var<storage, read_write> compact_gid_from_isect: array<u32>;

@compute @workgroup_size(256)
fn map_gaussians(@builtin(global_invocation_id) gid3: vec3<u32>) {
    let compact = gid3.x;
    if (compact >= u.num_visible) {
        return;
    }
    let gid = global_from_compact[compact];
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

    var count = cum_tiles_hit[compact];
    if (compact > 0u) {
        count = count - cum_tiles_hit[compact - 1u];
    }
    var base_isect = cum_tiles_hit[compact] - count;
    for (var ty = p.ty0; ty <= p.ty1; ty = ty + 1u) {
        for (var tx = p.tx0; tx <= p.tx1; tx = tx + 1u) {
            if (count == 0u) {
                break;
            }
            let tile_id = ty * u.tile_bw + tx;
            tile_id_from_isect[base_isect] = tile_id;
            compact_gid_from_isect[base_isect] = compact;
            base_isect = base_isect + 1u;
            count = count - 1u;
        }
    }
}
