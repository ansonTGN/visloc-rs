// Pass 2: after the depth sort, re-project each visible gaussian and compute its
// view-dependent SH colour, writing the compact `projected_splats` array.
//
// One invocation per *visible* gaussian (`compact` index). `global_from_compact`
// maps the sorted compact index to the original gaussian id.

@group(0) @binding(0) var<uniform> u: ProjectUniforms;
@group(0) @binding(1) var<storage, read> transforms: array<f32>;
@group(0) @binding(2) var<storage, read> opacity_in: array<f32>;
@group(0) @binding(3) var<storage, read> sh_in: array<f32>;
@group(0) @binding(4) var<storage, read> global_from_compact: array<u32>;
@group(0) @binding(5) var<storage, read_write> projected_splats: array<f32>;
@group(0) @binding(6) var<storage, read_write> compact_from_global: array<u32>;
// Per-gaussian tile counts (indexed by global id) and the compact-order gather
// that the prefix scan consumes.
@group(0) @binding(7) var<storage, read> intersect_counts: array<u32>;
@group(0) @binding(8) var<storage, read_write> counts_sorted: array<u32>;

@compute @workgroup_size(256)
fn project_visible(@builtin(global_invocation_id) gid3: vec3<u32>) {
    let compact = gid3.x;
    if (compact >= u.num_visible) {
        return;
    }
    let gid = global_from_compact[compact];
    compact_from_global[gid] = compact;
    // Gather the tile count into compact order for the prefix scan.
    counts_sorted[compact] = intersect_counts[gid];

    let base = gid * 10u;
    let mean = vec3<f32>(transforms[base], transforms[base + 1u], transforms[base + 2u]);
    let ls = vec3<f32>(transforms[base + 7u], transforms[base + 8u], transforms[base + 9u]);
    let p = compute_projected(
        u, mean,
        transforms[base + 3u], transforms[base + 4u],
        transforms[base + 5u], transforms[base + 6u],
        ls, opacity_in[gid],
    );

    // View-dependent colour: direction is world-space mean - camera centre.
    let dir_raw = mean - u.camera_center.xyz;
    var dir = vec3<f32>(0.0, 0.0, 1.0);
    let dn = length(dir_raw);
    if (dn > 1e-6) {
        dir = dir_raw / dn;
    }
    let cpc = u.sh_degree + 1u;
    let cpc2 = cpc * cpc;
    let sh_base = gid * 3u * cpc2;
    let dc = vec3<f32>(sh_in[sh_base], sh_in[sh_base + 1u], sh_in[sh_base + 2u]);
    var rest: array<f32, 45>;
    let rest_per_channel = 3u * (cpc2 - 1u);
    for (var i = 0u; i < 45u; i = i + 1u) {
        if (i < rest_per_channel) {
            rest[i] = sh_in[sh_base + 3u + i];
        } else {
            rest[i] = 0.0;
        }
    }
    let color = eval_sh_color(cpc2, dir, dc, &rest);

    let out = compact * PROJECTED_STRIDE;
    projected_splats[out + 0u] = select(0.0, p.proj_u, p.ok);
    projected_splats[out + 1u] = select(0.0, p.proj_v, p.ok);
    projected_splats[out + 2u] = p.conic.x;
    projected_splats[out + 3u] = p.conic.y;
    projected_splats[out + 4u] = p.conic.z;
    // A non-ok gaussian gets alpha 0 so it never contributes.
    projected_splats[out + 5u] = select(0.0, p.opacity, p.ok);
    projected_splats[out + 6u] = color.r;
    projected_splats[out + 7u] = color.g;
    projected_splats[out + 8u] = color.b;
}
