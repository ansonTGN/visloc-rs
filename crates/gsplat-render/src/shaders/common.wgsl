// Common types and helpers for the 3DGS forward rasterizer.
//
// Conventions match `visloc_gsplat_core::cpu_render` exactly so the GPU output
// can be compared against the CPU reference pixel-for-pixel:
//   * world -> camera:  p_cam = R * mean + t
//   * screen covariance: Sigma' = J W (R S S^T R^T) W^T J^T  + 0.3 * I
//   * extent:           3 * sqrt(largest eigenvalue of Sigma')
//   * alpha:            min(0.99, opacity * exp(-0.5 * d^T Sigma'^-1 d))

struct ProjectUniforms {
    view_rot_0: vec4<f32>,
    view_rot_1: vec4<f32>,
    view_rot_2: vec4<f32>,
    view_t: vec4<f32>,
    camera_center: vec4<f32>,
    fx: f32,
    fy: f32,
    cx: f32,
    cy: f32,
    img_w: u32,
    img_h: u32,
    tile_bw: u32,
    tile_bh: u32,
    sh_degree: u32,
    total_splats: u32,
    num_visible: u32,
    num_intersections: u32,
};

struct RasterUniforms {
    tile_bw: u32,
    img_w: u32,
    img_h: u32,
    tile_bh: u32,
    bg_r: f32,
    bg_g: f32,
    bg_b: f32,
    pad0: f32,
};

// Number of floats per projected splat: xy(2), conic(3), alpha(1), color(3).
const PROJECTED_STRIDE: u32 = 9u;

fn rot_from_quat(qw: f32, qx: f32, qy: f32, qz: f32) -> mat3x3<f32> {
    let n = qw * qw + qx * qx + qy * qy + qz * qz;
    var w = qw;
    var x = qx;
    var y = qy;
    var z = qz;
    if (n > 1e-12) {
        let inv = inverseSqrt(n);
        w = w * inv;
        x = x * inv;
        y = y * inv;
        z = z * inv;
    } else {
        w = 1.0;
        x = 0.0;
        y = 0.0;
        z = 0.0;
    }
    // Column-major rotation matrix (same as nalgebra's from_quaternion).
    let xx = x * x;
    let yy = y * y;
    let zz = z * z;
    let xy = x * y;
    let xz = x * z;
    let yz = y * z;
    let wx = w * x;
    let wy = w * y;
    let wz = w * z;
    return mat3x3<f32>(
        vec3<f32>(1.0 - 2.0 * (yy + zz), 2.0 * (xy + wz), 2.0 * (xz - wy)),
        vec3<f32>(2.0 * (xy - wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz + wx)),
        vec3<f32>(2.0 * (xz + wy), 2.0 * (yz - wx), 1.0 - 2.0 * (xx + yy)),
    );
}

fn view_rot(u: ProjectUniforms) -> mat3x3<f32> {
    // Stored row-major; columns of the mat3 are the rows of the rotation.
    return mat3x3<f32>(
        vec3<f32>(u.view_rot_0.x, u.view_rot_1.x, u.view_rot_2.x),
        vec3<f32>(u.view_rot_0.y, u.view_rot_1.y, u.view_rot_2.y),
        vec3<f32>(u.view_rot_0.z, u.view_rot_1.z, u.view_rot_2.z),
    );
}

fn world_to_camera(u: ProjectUniforms, p: vec3<f32>) -> vec3<f32> {
    let r = view_rot(u);
    return r * p + u.view_t.xyz;
}

// Evaluate the real SH basis (degree 0..3) for unit direction `d`.
// Returns coefficients in the flattened `(l, m)` order used by the core crate.
fn eval_sh_basis(d: vec3<f32>) -> array<f32, 16> {
    let x = d.x;
    let y = d.y;
    let z = d.z;
    let c = array<f32, 15>(
        0.4886025, 0.4886025, 0.4886025,
        1.0925485, 1.0925485, 0.3153916, 1.0925485, 1.0925485,
        0.3731762, 2.8906113, 1.843772, 0.5900436, 1.843772, 2.8906113, 0.3731762,
    );
    var b: array<f32, 16>;
    b[0] = 0.2820948;
    b[1] = -c[0] * y;
    b[2] = c[1] * z;
    b[3] = -c[2] * x;
    let xx = x * x;
    let yy = y * y;
    let zz = z * z;
    let xy = x * y;
    let yz = y * z;
    let xz = x * z;
    b[4] = c[3] * xy;
    b[5] = -c[4] * yz;
    b[6] = c[5] * (2.0 * zz - xx - yy);
    b[7] = -c[6] * xz;
    b[8] = c[7] * (xx - yy);
    b[9] = -c[8] * y * (3.0 * xx - yy);
    b[10] = c[9] * xy * z;
    b[11] = -c[10] * y * (4.0 * zz - xx - yy);
    b[12] = c[11] * z * (2.0 * zz - 3.0 * xx - 3.0 * yy);
    b[13] = -c[12] * x * (4.0 * zz - xx - yy);
    b[14] = c[13] * z * (xx - yy);
    b[15] = -c[14] * x * (xx - 3.0 * yy);
    return b;
}

// Per-gaussian SH block layout: 3 DC values then channel-major rest.
// `coeffs_per_channel = 1 + rest_per_channel`; block size = 3 * that.
fn eval_sh_color(
    coeffs_per_channel: u32,
    d: vec3<f32>,
    dc: vec3<f32>,
    rest: ptr<function, array<f32, 45>>,
) -> vec3<f32> {
    let basis = eval_sh_basis(d);
    let rest_per_channel = coeffs_per_channel - 1u;
    var color: vec3<f32>;
    for (var ch = 0u; ch < 3u; ch = ch + 1u) {
        var acc = 0.2820948 * dc[ch];
        let base = ch * rest_per_channel;
        for (var k = 0u; k < rest_per_channel; k = k + 1u) {
            acc = acc + basis[k + 1u] * (*rest)[base + k];
        }
        color[ch] = acc + 0.5;
    }
    return color;
}

// Screen-space conic (inverse of the 2x2 covariance) plus its determinant,
// from the three unique entries of the covariance.
fn cov2d_conic(a: f32, b: f32, c: f32) -> vec3<f32> {
    let det = a * c - b * b;
    if (det <= 0.0) {
        return vec3<f32>(0.0, 0.0, 0.0);
    }
    let inv = 1.0 / det;
    // conic = [c, -b, a] / det  (so conic.x corresponds to cov[0,0])
    return vec3<f32>(c * inv, -b * inv, a * inv);
}

// Geometric result of projecting one gaussian: everything needed to decide
// visibility and to rasterize it (except the view-dependent colour).
struct Projected {
    // false when any geometric gate rejects the gaussian.
    ok: bool,
    proj_u: f32,
    proj_v: f32,
    // Inverse 2D covariance [c00, c01, c11] (sort key is depth separately).
    conic: vec3<f32>,
    opacity: f32,
    // 3-sigma screen extent in pixels.
    radius: f32,
    // Camera-space depth (positive in front).
    depth: f32,
    // Screen tile range [tx0, tx1] x [ty0, ty1], inclusive, clamped to image.
    tx0: u32,
    ty0: u32,
    tx1: u32,
    ty1: u32,
};

fn compute_projected(
    u: ProjectUniforms,
    mean: vec3<f32>,
    qw: f32, qx: f32, qy: f32, qz: f32,
    ls: vec3<f32>,
    raw_opac: f32,
) -> Projected {
    var p: Projected;
    p.ok = false;
    p.proj_u = 0.0;
    p.proj_v = 0.0;
    p.conic = vec3<f32>(0.0, 0.0, 0.0);
    p.opacity = 0.0;
    p.radius = 0.0;
    p.depth = 0.0;
    p.tx0 = 0u;
    p.ty0 = 0u;
    p.tx1 = 0u;
    p.ty1 = 0u;

    let opacity = 1.0 / (1.0 + exp(-raw_opac));
    if (opacity < 1e-4) {
        return p;
    }
    let p_cam = world_to_camera(u, mean);
    if (p_cam.z <= 0.1) {
        return p;
    }

    let scale = exp(ls);
    let s2 = scale * scale;
    let r = rot_from_quat(qw, qx, qy, qz);
    let rw = view_rot(u) * r;
    // cov_cam = (W R) diag(s2) (W R)^T, WGSL column-major.
    let r00 = rw[0].x; let r01 = rw[1].x; let r02 = rw[2].x;
    let r10 = rw[0].y; let r11 = rw[1].y; let r12 = rw[2].y;
    let r20 = rw[0].z; let r21 = rw[1].z; let r22 = rw[2].z;
    let k00 = r00 * s2.x; let k01 = r01 * s2.y; let k02 = r02 * s2.z;
    let k10 = r10 * s2.x; let k11 = r11 * s2.y; let k12 = r12 * s2.z;
    let k20 = r20 * s2.x; let k21 = r21 * s2.y; let k22 = r22 * s2.z;
    let cov00 = k00 * r00 + k01 * r01 + k02 * r02;
    let cov01 = k00 * r10 + k01 * r11 + k02 * r12;
    let cov02 = k00 * r20 + k01 * r21 + k02 * r22;
    let cov11 = k10 * r10 + k11 * r11 + k12 * r12;
    let cov12 = k10 * r20 + k11 * r21 + k12 * r22;
    let cov22 = k20 * r20 + k21 * r21 + k22 * r22;
    let cov = mat3x3<f32>(
        vec3<f32>(cov00, cov01, cov02),
        vec3<f32>(cov01, cov11, cov12),
        vec3<f32>(cov02, cov12, cov22),
    );

    let inv_z = 1.0 / p_cam.z;
    let inv_z2 = inv_z * inv_z;
    // Linearise at a point clamped to 1.3x the frustum (Inria rasterizer; same
    // as the CPU reference) so off-screen near primitives do not explode.
    let lim_x = 1.3 * 0.5 * f32(u.img_w) / u.fx;
    let lim_y = 1.3 * 0.5 * f32(u.img_h) / u.fy;
    let tx = clamp(p_cam.x * inv_z, -lim_x, lim_x) * p_cam.z;
    let ty = clamp(p_cam.y * inv_z, -lim_y, lim_y) * p_cam.z;
    let j00 = u.fx * inv_z;
    let j02 = -u.fx * tx * inv_z2;
    let j11 = u.fy * inv_z;
    let j12 = -u.fy * ty * inv_z2;
    let j0 = vec3<f32>(j00, 0.0, j02);
    let j1 = vec3<f32>(0.0, j11, j12);
    let cj0 = vec3<f32>(
        j0.x * cov[0].x + j0.y * cov[0].y + j0.z * cov[0].z,
        j0.x * cov[1].x + j0.y * cov[1].y + j0.z * cov[1].z,
        j0.x * cov[2].x + j0.y * cov[2].y + j0.z * cov[2].z,
    );
    let cj1 = vec3<f32>(
        j1.x * cov[0].x + j1.y * cov[0].y + j1.z * cov[0].z,
        j1.x * cov[1].x + j1.y * cov[1].y + j1.z * cov[1].z,
        j1.x * cov[2].x + j1.y * cov[2].y + j1.z * cov[2].z,
    );
    let a = dot(cj0, j0) + 0.3;
    let b = dot(cj0, j1);
    let c = dot(cj1, j1) + 0.3;
    let det = a * c - b * b;
    if (!(det > 0.0)) {
        return p;
    }
    let mid = 0.5 * (a + c);
    let rad = sqrt(max(mid * mid - det, 0.0));
    let lambda1 = mid + rad;
    if (lambda1 <= 0.0) {
        return p;
    }
    var radius = 3.0 * sqrt(lambda1);
    if (!(radius == radius)) {
        return p;
    }
    // No footprint cap: capping cuts large primitives off at a tile-aligned
    // rectangle (hard edges). The frustum clamp above bounds near-plane blow-ups
    // and the tile rect below is clipped to the image.

    let proj_u = p_cam.x * j00 + u.cx;
    let proj_v = p_cam.y * j11 + u.cy;
    let w = f32(u.img_w);
    let h = f32(u.img_h);
    if (proj_u + radius < 0.0 || proj_v + radius < 0.0 ||
        proj_u - radius > w - 1.0 || proj_v - radius > h - 1.0) {
        return p;
    }
    let x0f = max(proj_u - radius, 0.0);
    let y0f = max(proj_v - radius, 0.0);
    let x1f = min(proj_u + radius, w - 1.0);
    let y1f = min(proj_v + radius, h - 1.0);
    if (x1f < x0f || y1f < y0f) {
        return p;
    }

    p.ok = true;
    p.proj_u = proj_u;
    p.proj_v = proj_v;
    p.conic = cov2d_conic(a, b, c);
    p.opacity = opacity;
    p.radius = radius;
    p.depth = p_cam.z;
    p.tx0 = u32(floor(x0f)) / 16u;
    p.ty0 = u32(floor(y0f)) / 16u;
    p.tx1 = u32(floor(x1f)) / 16u;
    p.ty1 = u32(floor(y1f)) / 16u;
    return p;
}

fn tile_span(p: Projected) -> u32 {
    return (p.tx1 - p.tx0 + 1u) * (p.ty1 - p.ty0 + 1u);
}
