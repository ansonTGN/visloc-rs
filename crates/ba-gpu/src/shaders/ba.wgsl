// GPU linearization + implicit-Schur PCG for monocular pinhole bundle
// adjustment (mirrors visloc-slam's CPU LM step: residual r = pi(R X + t)
// - xy, right-perturbation pose Jacobian J_pi [R | -R [X]x], landmark
// Jacobian J_pi R, Huber IRLS weight, additive lambda I damping on both
// blocks, landmarks eliminated by Schur complement, block-Jacobi PCG over
// the 6x6 pose blocks, landmark back-substitution).
//
// Layouts (f32): pose transforms 12 (R row-major, t); points 4 (xyz, pad);
// per-observation W = w J_p^T J_l (6x3 row-major), H_pp upper 21 + g_p 6,
// H_ll upper 6 + g_l 3. Pose blocks store full 6x6 (36) + g (6).

struct Params {
    n_obs: u32,
    n_poses: u32,      // variable poses
    n_lms: u32,        // variable landmarks
    max_iters: u32,
    lambda: f32,
    fx: f32,
    fy: f32,
    cx: f32,
    cy: f32,
    huber: f32,        // 0 = plain least squares
    rel_tol: f32,
    pad0: u32,
};

@group(0) @binding(0) var<uniform> p: Params;
@group(0) @binding(1) var<storage, read> poses: array<f32>;
@group(0) @binding(2) var<storage, read> points: array<vec4<f32>>;
// (pose index, landmark index, variable pose slot, variable landmark slot)
@group(0) @binding(3) var<storage, read> obs: array<vec4<u32>>;
@group(0) @binding(4) var<storage, read> obs_xy: array<vec2<f32>>;
@group(0) @binding(5) var<storage, read_write> obs_w: array<f32>;
@group(0) @binding(6) var<storage, read_write> obs_hpp: array<f32>;
@group(0) @binding(7) var<storage, read_write> obs_hll: array<f32>;
@group(0) @binding(8) var<storage, read> pose_off: array<u32>;
@group(0) @binding(9) var<storage, read> pose_obs: array<u32>;
@group(0) @binding(10) var<storage, read> lm_off: array<u32>;
@group(0) @binding(11) var<storage, read> lm_obs: array<u32>;
@group(0) @binding(12) var<storage, read_write> pose_h: array<f32>;
@group(0) @binding(13) var<storage, read_write> lm_h: array<f32>;
@group(0) @binding(14) var<storage, read_write> lm_inv: array<f32>;
@group(0) @binding(15) var<storage, read_write> minv: array<f32>;
@group(0) @binding(16) var<storage, read_write> vx: array<f32>;
@group(0) @binding(17) var<storage, read_write> vr: array<f32>;
@group(0) @binding(18) var<storage, read_write> vd: array<f32>;
@group(0) @binding(19) var<storage, read_write> vq: array<f32>;
@group(0) @binding(20) var<storage, read_write> vz: array<f32>;
@group(0) @binding(21) var<storage, read_write> lm_u: array<f32>;
// 0 rho, 1 |b|, 2 done flag, 3 iterations, 4 |r| at exit, 5 failure flag
@group(0) @binding(22) var<storage, read_write> scal: array<f32>;
@group(0) @binding(23) var<storage, read_write> dl: array<f32>;

const NONE: u32 = 0xFFFFFFFFu;

// Upper-triangle index of (i, j), i <= j, in a 6x6 symmetric matrix.
fn ut6(i: u32, j: u32) -> u32 {
    return i * 6u - (i * (i + 1u)) / 2u + j;
}

// ---------------------------------------------------------------- linearize

@compute @workgroup_size(256)
fn linearize(@builtin(global_invocation_id) g: vec3<u32>, @builtin(num_workgroups) nw: vec3<u32>) {
    let o = g.x + g.y * nw.x * 256u;
    if (o >= p.n_obs) {
        return;
    }
    let ob = obs[o];
    let pb = ob.x * 12u;
    let R = mat3x3<f32>(
        vec3<f32>(poses[pb + 0u], poses[pb + 3u], poses[pb + 6u]),
        vec3<f32>(poses[pb + 1u], poses[pb + 4u], poses[pb + 7u]),
        vec3<f32>(poses[pb + 2u], poses[pb + 5u], poses[pb + 8u]),
    );
    let t = vec3<f32>(poses[pb + 9u], poses[pb + 10u], poses[pb + 11u]);
    let X = points[ob.y].xyz;
    let xc = R * X + t;
    let valid = xc.z > 0.0;
    let zi = select(0.0, 1.0 / xc.z, valid);
    let res = vec2<f32>(p.fx * xc.x * zi + p.cx - obs_xy[o].x, p.fy * xc.y * zi + p.cy - obs_xy[o].y);
    let s = dot(res, res);
    var w = 1.0;
    if (p.huber > 0.0 && s > p.huber * p.huber) {
        w = p.huber / sqrt(s);
    }
    if (!valid) {
        w = 0.0;
    }
    // J_pi rows.
    let a0 = vec3<f32>(p.fx * zi, 0.0, -p.fx * xc.x * zi * zi);
    let a1 = vec3<f32>(0.0, p.fy * zi, -p.fy * xc.y * zi * zi);
    // J_l = J_pi R  (row vectors: a R).
    let l0 = a0 * R;
    let l1 = a1 * R;
    // J_p = [l | -l [X]x] with l = J_pi R; since [X]x v = X x v,
    // l . (X x v) = v . (l x X), i.e. the row vector l [X]x = l x X.
    let skx0 = vec3<f32>(l0.y * X.z - l0.z * X.y, l0.z * X.x - l0.x * X.z, l0.x * X.y - l0.y * X.x);
    let skx1 = vec3<f32>(l1.y * X.z - l1.z * X.y, l1.z * X.x - l1.x * X.z, l1.x * X.y - l1.y * X.x);
    let p0a = l0;
    let p0b = -skx0;
    let p1a = l1;
    let p1b = -skx1;
    let jp0 = array<f32, 6>(p0a.x, p0a.y, p0a.z, p0b.x, p0b.y, p0b.z);
    let jp1 = array<f32, 6>(p1a.x, p1a.y, p1a.z, p1b.x, p1b.y, p1b.z);
    let jl0 = array<f32, 3>(l0.x, l0.y, l0.z);
    let jl1 = array<f32, 3>(l1.x, l1.y, l1.z);
    // W = w J_p^T J_l (6x3)
    for (var i = 0u; i < 6u; i = i + 1u) {
        for (var j = 0u; j < 3u; j = j + 1u) {
            obs_w[o * 18u + i * 3u + j] = w * (jp0[i] * jl0[j] + jp1[i] * jl1[j]);
        }
    }
    // H_pp upper + g_p
    var k = 0u;
    for (var i = 0u; i < 6u; i = i + 1u) {
        for (var j = i; j < 6u; j = j + 1u) {
            obs_hpp[o * 27u + k] = w * (jp0[i] * jp0[j] + jp1[i] * jp1[j]);
            k = k + 1u;
        }
    }
    for (var i = 0u; i < 6u; i = i + 1u) {
        obs_hpp[o * 27u + 21u + i] = w * (jp0[i] * res.x + jp1[i] * res.y);
    }
    // H_ll upper + g_l
    obs_hll[o * 9u + 0u] = w * (l0.x * l0.x + l1.x * l1.x);
    obs_hll[o * 9u + 1u] = w * (l0.x * l0.y + l1.x * l1.y);
    obs_hll[o * 9u + 2u] = w * (l0.x * l0.z + l1.x * l1.z);
    obs_hll[o * 9u + 3u] = w * (l0.y * l0.y + l1.y * l1.y);
    obs_hll[o * 9u + 4u] = w * (l0.y * l0.z + l1.y * l1.z);
    obs_hll[o * 9u + 5u] = w * (l0.z * l0.z + l1.z * l1.z);
    obs_hll[o * 9u + 6u] = w * (l0.x * res.x + l1.x * res.y);
    obs_hll[o * 9u + 7u] = w * (l0.y * res.x + l1.y * res.y);
    obs_hll[o * 9u + 8u] = w * (l0.z * res.x + l1.z * res.y);
}

// ------------------------------------------------------- per-pose reduction

var<workgroup> red: array<f32, 1792>; // 64 threads x 28

@compute @workgroup_size(64)
fn pose_reduce(@builtin(workgroup_id) wid: vec3<u32>, @builtin(local_invocation_id) lid: vec3<u32>) {
    let pi = wid.x;
    let t = lid.x;
    var acc: array<f32, 27>;
    for (var k = 0u; k < 27u; k = k + 1u) {
        acc[k] = 0.0;
    }
    for (var e = pose_off[pi] + t; e < pose_off[pi + 1u]; e = e + 64u) {
        let o = pose_obs[e];
        for (var k = 0u; k < 27u; k = k + 1u) {
            acc[k] = acc[k] + obs_hpp[o * 27u + k];
        }
    }
    for (var k = 0u; k < 27u; k = k + 1u) {
        red[t * 28u + k] = acc[k];
    }
    workgroupBarrier();
    if (t < 27u) {
        var s = 0.0;
        for (var i = 0u; i < 64u; i = i + 1u) {
            s = s + red[i * 28u + t];
        }
        if (t < 21u) {
            // unpack upper index t -> (i, j)
            var i = 0u;
            var base = 0u;
            loop {
                let row_len = 6u - i;
                if (t < base + row_len) {
                    break;
                }
                base = base + row_len;
                i = i + 1u;
            }
            let j = i + (t - base);
            pose_h[pi * 42u + i * 6u + j] = s;
            pose_h[pi * 42u + j * 6u + i] = s;
        } else {
            pose_h[pi * 42u + 36u + (t - 21u)] = s;
        }
    }
}

@compute @workgroup_size(256)
fn lm_reduce(@builtin(global_invocation_id) g: vec3<u32>, @builtin(num_workgroups) nw: vec3<u32>) {
    let l = g.x + g.y * nw.x * 256u;
    if (l >= p.n_lms) {
        return;
    }
    var acc: array<f32, 9>;
    for (var k = 0u; k < 9u; k = k + 1u) {
        acc[k] = 0.0;
    }
    for (var e = lm_off[l]; e < lm_off[l + 1u]; e = e + 1u) {
        let o = lm_obs[e];
        for (var k = 0u; k < 9u; k = k + 1u) {
            acc[k] = acc[k] + obs_hll[o * 9u + k];
        }
    }
    for (var k = 0u; k < 9u; k = k + 1u) {
        lm_h[l * 9u + k] = acc[k];
    }
}

// ---------------------------------------------------- per-lambda preparation

// Damped landmark inverse (symmetric 3x3 via cofactors) and v = H^-1 g.
@compute @workgroup_size(256)
fn lm_prep(@builtin(global_invocation_id) g: vec3<u32>, @builtin(num_workgroups) nw: vec3<u32>) {
    let l = g.x + g.y * nw.x * 256u;
    if (l >= p.n_lms) {
        return;
    }
    let b = l * 9u;
    let a = lm_h[b + 0u] + p.lambda;
    let bb = lm_h[b + 1u];
    let c = lm_h[b + 2u];
    let d = lm_h[b + 3u] + p.lambda;
    let e = lm_h[b + 4u];
    let f = lm_h[b + 5u] + p.lambda;
    let c00 = d * f - e * e;
    let c01 = c * e - bb * f;
    let c02 = bb * e - c * d;
    let c11 = a * f - c * c;
    let c12 = bb * c - a * e;
    let c22 = a * d - bb * bb;
    let det = a * c00 + bb * c01 + c * c02;
    var inv = array<f32, 6>(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    let scale = max(max(abs(a), abs(d)), abs(f));
    if (det > 1e-12 * scale * scale * scale && det == det) {
        let id = 1.0 / det;
        inv = array<f32, 6>(c00 * id, c01 * id, c02 * id, c11 * id, c12 * id, c22 * id);
    }
    let gl = vec3<f32>(lm_h[b + 6u], lm_h[b + 7u], lm_h[b + 8u]);
    let o = l * 12u;
    for (var k = 0u; k < 6u; k = k + 1u) {
        lm_inv[o + k] = inv[k];
    }
    lm_inv[o + 6u] = inv[0] * gl.x + inv[1] * gl.y + inv[2] * gl.z;
    lm_inv[o + 7u] = inv[1] * gl.x + inv[3] * gl.y + inv[4] * gl.z;
    lm_inv[o + 8u] = inv[2] * gl.x + inv[4] * gl.y + inv[5] * gl.z;
}

fn hinv_apply(l: u32, v: vec3<f32>) -> vec3<f32> {
    let o = l * 12u;
    let i0 = lm_inv[o + 0u];
    let i1 = lm_inv[o + 1u];
    let i2 = lm_inv[o + 2u];
    let i3 = lm_inv[o + 3u];
    let i4 = lm_inv[o + 4u];
    let i5 = lm_inv[o + 5u];
    return vec3<f32>(i0 * v.x + i1 * v.y + i2 * v.z, i1 * v.x + i3 * v.y + i4 * v.z,
        i2 * v.x + i4 * v.y + i5 * v.z);
}

fn w_row(o: u32, i: u32) -> vec3<f32> {
    return vec3<f32>(obs_w[o * 18u + i * 3u], obs_w[o * 18u + i * 3u + 1u], obs_w[o * 18u + i * 3u + 2u]);
}

// Preconditioner block M_p = D_p - sum W Hinv W^T (inverted by Cholesky),
// reduced right-hand side rhs_p = -g_p + sum W v_l, and PCG start vectors.
@compute @workgroup_size(64)
fn pose_prep(@builtin(workgroup_id) wid: vec3<u32>, @builtin(local_invocation_id) lid: vec3<u32>) {
    let pi = wid.x;
    let t = lid.x;
    var acc: array<f32, 27>;
    for (var k = 0u; k < 27u; k = k + 1u) {
        acc[k] = 0.0;
    }
    for (var e = pose_off[pi] + t; e < pose_off[pi + 1u]; e = e + 64u) {
        let o = pose_obs[e];
        let l = obs[o].w;
        if (l == NONE) {
            continue;
        }
        var wh: array<vec3<f32>, 6>;
        for (var i = 0u; i < 6u; i = i + 1u) {
            wh[i] = hinv_apply(l, w_row(o, i));
        }
        let v = vec3<f32>(lm_inv[l * 12u + 6u], lm_inv[l * 12u + 7u], lm_inv[l * 12u + 8u]);
        var k = 0u;
        for (var i = 0u; i < 6u; i = i + 1u) {
            let wi = w_row(o, i);
            for (var j = i; j < 6u; j = j + 1u) {
                acc[k] = acc[k] + dot(wh[j], wi);
                k = k + 1u;
            }
            acc[21u + i] = acc[21u + i] + dot(wi, v);
        }
    }
    for (var k = 0u; k < 27u; k = k + 1u) {
        red[t * 28u + k] = acc[k];
    }
    workgroupBarrier();
    if (t != 0u) {
        return;
    }
    var m: array<f32, 36>;
    var k = 0u;
    for (var i = 0u; i < 6u; i = i + 1u) {
        for (var j = i; j < 6u; j = j + 1u) {
            var s = 0.0;
            for (var q = 0u; q < 64u; q = q + 1u) {
                s = s + red[q * 28u + k];
            }
            var v = pose_h[pi * 42u + i * 6u + j] - s;
            if (i == j) {
                v = v + p.lambda;
            }
            m[i * 6u + j] = v;
            m[j * 6u + i] = v;
            k = k + 1u;
        }
    }
    var rhs: array<f32, 6>;
    for (var i = 0u; i < 6u; i = i + 1u) {
        var s = 0.0;
        for (var q = 0u; q < 64u; q = q + 1u) {
            s = s + red[q * 28u + 21u + i];
        }
        rhs[i] = -pose_h[pi * 42u + 36u + i] + s;
    }
    // Cholesky M = L L^T (in place, lower).
    var ok = true;
    for (var j = 0u; j < 6u; j = j + 1u) {
        var s = m[j * 6u + j];
        for (var q = 0u; q < j; q = q + 1u) {
            s = s - m[j * 6u + q] * m[j * 6u + q];
        }
        if (!(s > 0.0)) {
            ok = false;
            break;
        }
        let ljj = sqrt(s);
        m[j * 6u + j] = ljj;
        for (var i = j + 1u; i < 6u; i = i + 1u) {
            var s2 = m[i * 6u + j];
            for (var q = 0u; q < j; q = q + 1u) {
                s2 = s2 - m[i * 6u + q] * m[j * 6u + q];
            }
            m[i * 6u + j] = s2 / ljj;
        }
    }
    // Inverse = L^-T L^-1, column by column.
    var inv: array<f32, 36>;
    for (var c = 0u; c < 6u; c = c + 1u) {
        var y: array<f32, 6>;
        for (var i = 0u; i < 6u; i = i + 1u) {
            var s = select(0.0, 1.0, i == c);
            for (var q = 0u; q < i; q = q + 1u) {
                s = s - m[i * 6u + q] * y[q];
            }
            y[i] = s / m[i * 6u + i];
        }
        for (var ii = 0u; ii < 6u; ii = ii + 1u) {
            let i = 5u - ii;
            var s = y[i];
            for (var q = i + 1u; q < 6u; q = q + 1u) {
                s = s - m[q * 6u + i] * inv[q * 6u + c];
            }
            inv[i * 6u + c] = s / m[i * 6u + i];
        }
    }
    if (!ok) {
        // Fall back to the inverse diagonal of the damped camera block.
        for (var i = 0u; i < 36u; i = i + 1u) {
            inv[i] = 0.0;
        }
        for (var i = 0u; i < 6u; i = i + 1u) {
            inv[i * 6u + i] = 1.0 / max(pose_h[pi * 42u + i * 6u + i] + p.lambda, 1e-20);
        }
    }
    for (var i = 0u; i < 36u; i = i + 1u) {
        minv[pi * 36u + i] = inv[i];
    }
    for (var i = 0u; i < 6u; i = i + 1u) {
        var z = 0.0;
        for (var j = 0u; j < 6u; j = j + 1u) {
            z = z + inv[i * 6u + j] * rhs[j];
        }
        vr[pi * 6u + i] = rhs[i];
        vx[pi * 6u + i] = 0.0;
        vz[pi * 6u + i] = z;
        vd[pi * 6u + i] = z;
    }
}

// ---------------------------------------------------------------- PCG

var<workgroup> sred: array<f32, 768>;
var<workgroup> wg_done: f32;

// PCG termination flag, made workgroup-uniform so barriers may follow.
fn done_uniform(t: u32) -> bool {
    if (t == 0u) {
        wg_done = scal[2];
    }
    return workgroupUniformLoad(&wg_done) != 0.0;
}

fn wg_sum3(t: u32, a: f32, b: f32, c: f32) -> vec3<f32> {
    sred[t] = a;
    sred[256u + t] = b;
    sred[512u + t] = c;
    workgroupBarrier();
    for (var s = 128u; s > 0u; s = s >> 1u) {
        if (t < s) {
            sred[t] = sred[t] + sred[t + s];
            sred[256u + t] = sred[256u + t] + sred[256u + t + s];
            sred[512u + t] = sred[512u + t] + sred[512u + t + s];
        }
        workgroupBarrier();
    }
    let r = vec3<f32>(sred[0], sred[256], sred[512]);
    workgroupBarrier();
    return r;
}

@compute @workgroup_size(256)
fn pcg_init(@builtin(local_invocation_id) lid: vec3<u32>) {
    let t = lid.x;
    let n = p.n_poses * 6u;
    var rz = 0.0;
    var rr = 0.0;
    for (var i = t; i < n; i = i + 256u) {
        rz = rz + vr[i] * vz[i];
        rr = rr + vr[i] * vr[i];
    }
    let s = wg_sum3(t, rz, rr, 0.0);
    if (t == 0u) {
        scal[0] = s.x;
        scal[1] = sqrt(s.y);
        scal[2] = select(0.0, 1.0, s.y <= 0.0 || !(s.x > 0.0));
        scal[3] = 0.0;
        scal[4] = sqrt(s.y);
        scal[5] = 0.0;
    }
}

// u_l = Hinv_l sum_{o in l} W_o^T d_{p(o)}
@compute @workgroup_size(256)
fn apply_lm(@builtin(global_invocation_id) g: vec3<u32>, @builtin(num_workgroups) nw: vec3<u32>) {
    if (scal[2] != 0.0) {
        return;
    }
    let l = g.x + g.y * nw.x * 256u;
    if (l >= p.n_lms) {
        return;
    }
    var tv = vec3<f32>(0.0);
    for (var e = lm_off[l]; e < lm_off[l + 1u]; e = e + 1u) {
        let o = lm_obs[e];
        let pi = obs[o].z;
        if (pi == NONE) {
            continue;
        }
        for (var i = 0u; i < 6u; i = i + 1u) {
            tv = tv + w_row(o, i) * vd[pi * 6u + i];
        }
    }
    let u = hinv_apply(l, tv);
    lm_u[l * 3u + 0u] = u.x;
    lm_u[l * 3u + 1u] = u.y;
    lm_u[l * 3u + 2u] = u.z;
}

var<workgroup> pred: array<f32, 384>; // 64 x 6

// q_p = D_p d_p - sum_{o in p} W_o u_{l(o)}
@compute @workgroup_size(64)
fn apply_pose(@builtin(workgroup_id) wid: vec3<u32>, @builtin(local_invocation_id) lid: vec3<u32>) {
    let pi = wid.x;
    let t = lid.x;
    if (done_uniform(t)) {
        return;
    }
    var acc = array<f32, 6>(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    for (var e = pose_off[pi] + t; e < pose_off[pi + 1u]; e = e + 64u) {
        let o = pose_obs[e];
        let l = obs[o].w;
        if (l == NONE) {
            continue;
        }
        let u = vec3<f32>(lm_u[l * 3u], lm_u[l * 3u + 1u], lm_u[l * 3u + 2u]);
        for (var i = 0u; i < 6u; i = i + 1u) {
            acc[i] = acc[i] + dot(w_row(o, i), u);
        }
    }
    for (var i = 0u; i < 6u; i = i + 1u) {
        pred[t * 6u + i] = acc[i];
    }
    workgroupBarrier();
    if (t < 6u) {
        var s = 0.0;
        for (var q = 0u; q < 64u; q = q + 1u) {
            s = s + pred[q * 6u + t];
        }
        var dv = 0.0;
        for (var j = 0u; j < 6u; j = j + 1u) {
            dv = dv + pose_h[pi * 42u + t * 6u + j] * vd[pi * 6u + j];
        }
        dv = dv + p.lambda * vd[pi * 6u + t];
        vq[pi * 6u + t] = dv - s;
    }
}

// One PCG iteration's vector work (single workgroup).
@compute @workgroup_size(256)
fn pcg_step(@builtin(local_invocation_id) lid: vec3<u32>) {
    let t = lid.x;
    if (done_uniform(t)) {
        return;
    }
    let n = p.n_poses * 6u;
    var dq = 0.0;
    for (var i = t; i < n; i = i + 256u) {
        dq = dq + vd[i] * vq[i];
    }
    let c = wg_sum3(t, dq, 0.0, 0.0).x;
    let rho = scal[0];
    if (!(c > 0.0)) {
        if (t == 0u) {
            scal[2] = 1.0;
            scal[5] = 1.0;
        }
        return;
    }
    let alpha = rho / c;
    var rz = 0.0;
    var rr = 0.0;
    for (var i = t; i < n; i = i + 256u) {
        vx[i] = vx[i] + alpha * vd[i];
        vr[i] = vr[i] - alpha * vq[i];
    }
    storageBarrier();
    workgroupBarrier();
    for (var i = t; i < n; i = i + 256u) {
        let pi = i / 6u;
        let row = i % 6u;
        var z = 0.0;
        for (var j = 0u; j < 6u; j = j + 1u) {
            z = z + minv[pi * 36u + row * 6u + j] * vr[pi * 6u + j];
        }
        vz[i] = z;
        rz = rz + vr[i] * z;
        rr = rr + vr[i] * vr[i];
    }
    let s = wg_sum3(t, rz, rr, 0.0);
    let beta = s.x / rho;
    for (var i = t; i < n; i = i + 256u) {
        vd[i] = vz[i] + beta * vd[i];
    }
    if (t == 0u) {
        scal[0] = s.x;
        scal[3] = scal[3] + 1.0;
        scal[4] = sqrt(s.y);
        if (sqrt(s.y) <= p.rel_tol * scal[1] || scal[3] >= f32(p.max_iters) || !(s.x > 0.0)) {
            scal[2] = 1.0;
        }
    }
}

// dl_l = -Hinv_l (g_l + sum_{o in l} W_o^T x_{p(o)})
@compute @workgroup_size(256)
fn backsub(@builtin(global_invocation_id) g: vec3<u32>, @builtin(num_workgroups) nw: vec3<u32>) {
    let l = g.x + g.y * nw.x * 256u;
    if (l >= p.n_lms) {
        return;
    }
    var tv = vec3<f32>(lm_h[l * 9u + 6u], lm_h[l * 9u + 7u], lm_h[l * 9u + 8u]);
    for (var e = lm_off[l]; e < lm_off[l + 1u]; e = e + 1u) {
        let o = lm_obs[e];
        let pi = obs[o].z;
        if (pi == NONE) {
            continue;
        }
        for (var i = 0u; i < 6u; i = i + 1u) {
            tv = tv + w_row(o, i) * vx[pi * 6u + i];
        }
    }
    let u = hinv_apply(l, tv);
    dl[l * 3u + 0u] = -u.x;
    dl[l * 3u + 1u] = -u.y;
    dl[l * 3u + 2u] = -u.z;
}
