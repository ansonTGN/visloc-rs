// Backward of the projection: chain each visible gaussian's screen-space
// gradient (grad_reduce: du, dv, dA, dB, dC, dopacity, dr, dg, db) to its
// parameters, written in the same layouts as the forward inputs:
//   grad_transforms[gid * 10 + ..] = d(mean xyz, quat wxyz (un-normalised),
//                                      log-scale xyz)
//   grad_opacity[gid]             = d(opacity logit)
//   grad_sh[gid * 3 * cpc2 + ..]  = d(SH block: 3 DC, channel-major rest)
// Gaussians not visible this frame are not touched (the host clears them).
//
// Forward being differentiated (compute_projected / project_visible):
//   p = W m + t;  Rg = R(q / |q|);  M3 = Rg diag(exp(ls));  Sigma = M3 M3^T
//   Sc = W Sigma W^T;  J rows j0 = (fx/z, 0, -fx tx/z^2), j1 = (0, fy/z, -fy ty/z^2)
//   with tx = clamp(x/z, +-1.3 tan(fovx/2)) z (likewise ty);
//   [a b; b c] = J Sc J^T + 0.3 I;  conic Q = [a b; b c]^-1 = [A B; B C]
//   (u, v) = (fx x/z + cx, fy y/z + cy);  opacity = sigmoid(logit)
//   colour = max(0, SH(dir) + 0.5),  dir = (m - camera centre) / |.|
// All matrices below are row-major arrays (r[i][j] = row i, column j).

@group(0) @binding(0) var<uniform> u: ProjectUniforms;
@group(0) @binding(1) var<storage, read> transforms: array<f32>;
@group(0) @binding(2) var<storage, read> opacity_in: array<f32>;
@group(0) @binding(3) var<storage, read> sh_in: array<f32>;
@group(0) @binding(4) var<storage, read> global_from_compact: array<u32>;
@group(0) @binding(5) var<storage, read> screen_grads: array<f32>;
@group(0) @binding(6) var<storage, read_write> grad_transforms: array<f32>;
@group(0) @binding(7) var<storage, read_write> grad_opacity: array<f32>;
@group(0) @binding(8) var<storage, read_write> grad_sh: array<f32>;

// d(basis[k]) / d(dir) for the real SH basis of `eval_sh_basis`.
fn sh_basis_grad(d: vec3<f32>) -> array<vec3<f32>, 16> {
    let x = d.x;
    let y = d.y;
    let z = d.z;
    let c = array<f32, 15>(
        0.4886025, 0.4886025, 0.4886025,
        1.0925485, 1.0925485, 0.3153916, 1.0925485, 1.0925485,
        0.3731762, 2.8906113, 1.843772, 0.5900436, 1.843772, 2.8906113, 0.3731762,
    );
    let xx = x * x;
    let yy = y * y;
    let zz = z * z;
    var g: array<vec3<f32>, 16>;
    g[0] = vec3<f32>(0.0, 0.0, 0.0);
    g[1] = vec3<f32>(0.0, -c[0], 0.0);
    g[2] = vec3<f32>(0.0, 0.0, c[1]);
    g[3] = vec3<f32>(-c[2], 0.0, 0.0);
    g[4] = vec3<f32>(c[3] * y, c[3] * x, 0.0);
    g[5] = vec3<f32>(0.0, -c[4] * z, -c[4] * y);
    g[6] = vec3<f32>(-2.0 * c[5] * x, -2.0 * c[5] * y, 4.0 * c[5] * z);
    g[7] = vec3<f32>(-c[6] * z, 0.0, -c[6] * x);
    g[8] = vec3<f32>(2.0 * c[7] * x, -2.0 * c[7] * y, 0.0);
    g[9] = vec3<f32>(-6.0 * c[8] * x * y, -c[8] * (3.0 * xx - 3.0 * yy), 0.0);
    g[10] = vec3<f32>(c[9] * y * z, c[9] * x * z, c[9] * x * y);
    g[11] = vec3<f32>(2.0 * c[10] * x * y, -c[10] * (4.0 * zz - xx - 3.0 * yy), -8.0 * c[10] * y * z);
    g[12] = vec3<f32>(-6.0 * c[11] * x * z, -6.0 * c[11] * y * z, c[11] * (6.0 * zz - 3.0 * xx - 3.0 * yy));
    g[13] = vec3<f32>(-c[12] * (4.0 * zz - 3.0 * xx - yy), 2.0 * c[12] * x * y, -8.0 * c[12] * x * z);
    g[14] = vec3<f32>(2.0 * c[13] * x * z, -2.0 * c[13] * y * z, c[13] * (xx - yy));
    g[15] = vec3<f32>(-c[14] * (3.0 * xx - 3.0 * yy), 6.0 * c[14] * x * y, 0.0);
    return g;
}

@compute @workgroup_size(256)
fn project_backward(@builtin(global_invocation_id) gid3: vec3<u32>) {
    let compact = gid3.x;
    if (compact >= u.num_visible) {
        return;
    }
    let gid = global_from_compact[compact];
    let sg = compact * 9u;
    let g_u = screen_grads[sg + 0u];
    let g_v = screen_grads[sg + 1u];
    let g_ca = screen_grads[sg + 2u];
    let g_cb = screen_grads[sg + 3u];
    let g_cc = screen_grads[sg + 4u];
    let g_op = screen_grads[sg + 5u];
    let g_col = vec3<f32>(screen_grads[sg + 6u], screen_grads[sg + 7u], screen_grads[sg + 8u]);

    let base = gid * 10u;
    let m = vec3<f32>(transforms[base], transforms[base + 1u], transforms[base + 2u]);
    let q = vec4<f32>(transforms[base + 3u], transforms[base + 4u], transforms[base + 5u], transforms[base + 6u]);
    let ls = vec3<f32>(transforms[base + 7u], transforms[base + 8u], transforms[base + 9u]);

    // View rotation W (row-major) and camera-space mean.
    var wr: array<array<f32, 3>, 3>;
    wr[0] = array<f32, 3>(u.view_rot_0.x, u.view_rot_0.y, u.view_rot_0.z);
    wr[1] = array<f32, 3>(u.view_rot_1.x, u.view_rot_1.y, u.view_rot_1.z);
    wr[2] = array<f32, 3>(u.view_rot_2.x, u.view_rot_2.y, u.view_rot_2.z);
    var p: array<f32, 3>;
    for (var i = 0u; i < 3u; i = i + 1u) {
        p[i] = wr[i][0] * m.x + wr[i][1] * m.y + wr[i][2] * m.z + u.view_t[i];
    }

    // Normalised quaternion and Rg (same as rot_from_quat).
    let n2 = dot(q, q);
    var qn = vec4<f32>(1.0, 0.0, 0.0, 0.0);
    var inv_n = 0.0;
    if (n2 > 1e-12) {
        inv_n = inverseSqrt(n2);
        qn = q * inv_n;
    }
    let w = qn.x;
    let x = qn.y;
    let y = qn.z;
    let z = qn.w;
    var rg: array<array<f32, 3>, 3>;
    rg[0] = array<f32, 3>(1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y));
    rg[1] = array<f32, 3>(2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x));
    rg[2] = array<f32, 3>(2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y));
    let s = exp(ls);
    var m3: array<array<f32, 3>, 3>;
    for (var i = 0u; i < 3u; i = i + 1u) {
        m3[i] = array<f32, 3>(rg[i][0] * s.x, rg[i][1] * s.y, rg[i][2] * s.z);
    }
    // Sigma = M3 M3^T, Sc = W Sigma W^T.
    var sig: array<array<f32, 3>, 3>;
    for (var i = 0u; i < 3u; i = i + 1u) {
        for (var j = 0u; j < 3u; j = j + 1u) {
            sig[i][j] = m3[i][0] * m3[j][0] + m3[i][1] * m3[j][1] + m3[i][2] * m3[j][2];
        }
    }
    var ws: array<array<f32, 3>, 3>;
    for (var i = 0u; i < 3u; i = i + 1u) {
        for (var j = 0u; j < 3u; j = j + 1u) {
            ws[i][j] = wr[i][0] * sig[0][j] + wr[i][1] * sig[1][j] + wr[i][2] * sig[2][j];
        }
    }
    var sc: array<array<f32, 3>, 3>;
    for (var i = 0u; i < 3u; i = i + 1u) {
        for (var j = 0u; j < 3u; j = j + 1u) {
            sc[i][j] = ws[i][0] * wr[j][0] + ws[i][1] * wr[j][1] + ws[i][2] * wr[j][2];
        }
    }

    // EWA Jacobian with the 1.3x frustum clamp.
    let inv_z = 1.0 / p[2];
    let inv_z2 = inv_z * inv_z;
    let lim_x = 1.3 * 0.5 * f32(u.img_w) / u.fx;
    let lim_y = 1.3 * 0.5 * f32(u.img_h) / u.fy;
    let rx = p[0] * inv_z;
    let ry = p[1] * inv_z;
    let clamped_x = rx < -lim_x || rx > lim_x;
    let clamped_y = ry < -lim_y || ry > lim_y;
    let tx = clamp(rx, -lim_x, lim_x) * p[2];
    let ty = clamp(ry, -lim_y, lim_y) * p[2];
    let j0 = vec3<f32>(u.fx * inv_z, 0.0, -u.fx * tx * inv_z2);
    let j1 = vec3<f32>(0.0, u.fy * inv_z, -u.fy * ty * inv_z2);
    // Sc j0, Sc j1.
    var sj0 = vec3<f32>(0.0, 0.0, 0.0);
    var sj1 = vec3<f32>(0.0, 0.0, 0.0);
    for (var i = 0u; i < 3u; i = i + 1u) {
        sj0[i] = sc[i][0] * j0.x + sc[i][1] * j0.y + sc[i][2] * j0.z;
        sj1[i] = sc[i][0] * j1.x + sc[i][1] * j1.y + sc[i][2] * j1.z;
    }
    let ca = dot(j0, sj0) + 0.3;
    let cb = dot(j0, sj1);
    let cc = dot(j1, sj1) + 0.3;
    let det = ca * cc - cb * cb;
    let qa = cc / det;
    let qb = -cb / det;
    let qc = ca / det;

    // Conic -> 2D covariance: dL/dM = -Q GQ Q with GQ = [gA gB/2; gB/2 gC].
    let gq01 = 0.5 * g_cb;
    // T = GQ Q
    let t00 = g_ca * qa + gq01 * qb;
    let t01 = g_ca * qb + gq01 * qc;
    let t10 = gq01 * qa + g_cc * qb;
    let t11 = gq01 * qb + g_cc * qc;
    // GM = -Q T
    let gm00 = -(qa * t00 + qb * t10);
    let gm01 = -(qa * t01 + qb * t11);
    let gm11 = -(qb * t01 + qc * t11);
    let g_a = gm00;
    let g_b = 2.0 * gm01;
    let g_c = gm11;

    // 2D covariance -> Sc (general 3x3 gradient) and J.
    var gsc: array<array<f32, 3>, 3>;
    for (var i = 0u; i < 3u; i = i + 1u) {
        for (var j = 0u; j < 3u; j = j + 1u) {
            gsc[i][j] = g_a * j0[i] * j0[j] + g_b * j0[i] * j1[j] + g_c * j1[i] * j1[j];
        }
    }
    let gj0 = 2.0 * g_a * sj0 + g_b * sj1;
    let gj1 = 2.0 * g_c * sj1 + g_b * sj0;

    // J and the screen mean -> camera-space point.
    var gp = vec3<f32>(0.0, 0.0, 0.0);
    gp.z = gp.z + gj0.x * (-u.fx * inv_z2) + gj1.y * (-u.fy * inv_z2);
    if (clamped_x) {
        gp.z = gp.z + gj0.z * u.fx * clamp(rx, -lim_x, lim_x) * inv_z2;
    } else {
        gp.x = gp.x + gj0.z * (-u.fx * inv_z2);
        gp.z = gp.z + gj0.z * 2.0 * u.fx * p[0] * inv_z2 * inv_z;
    }
    if (clamped_y) {
        gp.z = gp.z + gj1.z * u.fy * clamp(ry, -lim_y, lim_y) * inv_z2;
    } else {
        gp.y = gp.y + gj1.z * (-u.fy * inv_z2);
        gp.z = gp.z + gj1.z * 2.0 * u.fy * p[1] * inv_z2 * inv_z;
    }
    gp.x = gp.x + g_u * u.fx * inv_z;
    gp.y = gp.y + g_v * u.fy * inv_z;
    gp.z = gp.z - (g_u * u.fx * p[0] + g_v * u.fy * p[1]) * inv_z2;

    // p = W m + t  ->  dL/dm = W^T gp.
    var gmean = vec3<f32>(0.0, 0.0, 0.0);
    for (var j = 0u; j < 3u; j = j + 1u) {
        gmean[j] = wr[0][j] * gp.x + wr[1][j] * gp.y + wr[2][j] * gp.z;
    }

    // Sc = W Sigma W^T  ->  GSigma = W^T GSc W.
    var tmp: array<array<f32, 3>, 3>;
    for (var i = 0u; i < 3u; i = i + 1u) {
        for (var j = 0u; j < 3u; j = j + 1u) {
            tmp[i][j] = wr[0][i] * gsc[0][j] + wr[1][i] * gsc[1][j] + wr[2][i] * gsc[2][j];
        }
    }
    var gsig: array<array<f32, 3>, 3>;
    for (var i = 0u; i < 3u; i = i + 1u) {
        for (var j = 0u; j < 3u; j = j + 1u) {
            gsig[i][j] = tmp[i][0] * wr[0][j] + tmp[i][1] * wr[1][j] + tmp[i][2] * wr[2][j];
        }
    }
    // Sigma = M3 M3^T  ->  GM3 = (GSigma + GSigma^T) M3.
    var gm3: array<array<f32, 3>, 3>;
    for (var i = 0u; i < 3u; i = i + 1u) {
        for (var k = 0u; k < 3u; k = k + 1u) {
            var acc = 0.0;
            for (var j = 0u; j < 3u; j = j + 1u) {
                acc = acc + (gsig[i][j] + gsig[j][i]) * m3[j][k];
            }
            gm3[i][k] = acc;
        }
    }
    // M3 = Rg diag(s): scale and rotation gradients.
    var gls = vec3<f32>(0.0, 0.0, 0.0);
    var gr: array<array<f32, 3>, 3>;
    for (var k = 0u; k < 3u; k = k + 1u) {
        var gs = 0.0;
        for (var i = 0u; i < 3u; i = i + 1u) {
            gs = gs + gm3[i][k] * rg[i][k];
            gr[i][k] = gm3[i][k] * s[k];
        }
        gls[k] = gs * s[k];
    }
    // Rg(qn) -> normalised quaternion.
    let gw = 2.0 * (-z * gr[0][1] + y * gr[0][2] + z * gr[1][0] - x * gr[1][2] - y * gr[2][0] + x * gr[2][1]);
    let gx = 2.0 * (y * gr[0][1] + z * gr[0][2] + y * gr[1][0] - 2.0 * x * gr[1][1] - w * gr[1][2]
        + z * gr[2][0] + w * gr[2][1] - 2.0 * x * gr[2][2]);
    let gy = 2.0 * (-2.0 * y * gr[0][0] + x * gr[0][1] + w * gr[0][2] + x * gr[1][0] + z * gr[1][2]
        - w * gr[2][0] + z * gr[2][1] - 2.0 * y * gr[2][2]);
    let gz = 2.0 * (-2.0 * z * gr[0][0] - w * gr[0][1] + x * gr[0][2] + w * gr[1][0] - 2.0 * z * gr[1][1]
        + y * gr[1][2] + x * gr[2][0] + y * gr[2][1]);
    let gqn = vec4<f32>(gw, gx, gy, gz);
    // qn = q / |q|  ->  gq = (gqn - qn (qn . gqn)) / |q|.
    let gq = (gqn - qn * dot(qn, gqn)) * inv_n;

    // Opacity.
    let op = 1.0 / (1.0 + exp(-opacity_in[gid]));
    grad_opacity[gid] = g_op * op * (1.0 - op);

    // Colour -> SH coefficients, and -> mean through the view direction.
    let cpc = u.sh_degree + 1u;
    let cpc2 = cpc * cpc;
    let rest_pc = cpc2 - 1u;
    let sh_base = gid * 3u * cpc2;
    let dir_raw = m - u.camera_center.xyz;
    let dn = length(dir_raw);
    var dir = vec3<f32>(0.0, 0.0, 1.0);
    if (dn > 1e-6) {
        dir = dir_raw / dn;
    }
    let basis = eval_sh_basis(dir);
    let dbasis = sh_basis_grad(dir);
    var gdir = vec3<f32>(0.0, 0.0, 0.0);
    for (var ch = 0u; ch < 3u; ch = ch + 1u) {
        var raw = 0.2820948 * sh_in[sh_base + ch];
        for (var k = 0u; k < rest_pc; k = k + 1u) {
            raw = raw + basis[k + 1u] * sh_in[sh_base + 3u + ch * rest_pc + k];
        }
        raw = raw + 0.5;
        var gc = g_col[ch];
        if (raw < 0.0) {
            gc = 0.0;
        }
        grad_sh[sh_base + ch] = gc * 0.2820948;
        for (var k = 0u; k < rest_pc; k = k + 1u) {
            let coef = sh_in[sh_base + 3u + ch * rest_pc + k];
            grad_sh[sh_base + 3u + ch * rest_pc + k] = gc * basis[k + 1u];
            gdir = gdir + gc * coef * dbasis[k + 1u];
        }
    }
    if (dn > 1e-6) {
        gmean = gmean + (gdir - dir * dot(dir, gdir)) / dn;
    }

    grad_transforms[base + 0u] = gmean.x;
    grad_transforms[base + 1u] = gmean.y;
    grad_transforms[base + 2u] = gmean.z;
    grad_transforms[base + 3u] = gq.x;
    grad_transforms[base + 4u] = gq.y;
    grad_transforms[base + 5u] = gq.z;
    grad_transforms[base + 6u] = gq.w;
    grad_transforms[base + 7u] = gls.x;
    grad_transforms[base + 8u] = gls.y;
    grad_transforms[base + 9u] = gls.z;
}
