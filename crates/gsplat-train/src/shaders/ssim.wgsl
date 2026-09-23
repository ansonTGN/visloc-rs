// D-SSIM loss term: adds lambda * d(1 - mean SSIM)/d render to d_image
// (the L1 kernel has already written (1 - lambda) * its own gradient).
//
// SSIM per channel with an 11x11 gaussian window (sigma 1.5), zero padding
// (as F.conv2d(padding=5) in the Inria code), C1 = 0.01^2, C2 = 0.03^2:
//   S = (2 mx my + C1)(2 sxy + C2) / ((mx^2 + my^2 + C1)(sx^2 + sy^2 + C2))
// with mx = G*x, Sxx = G*(x^2), Sxy = G*(xy), sx^2 = Sxx - mx^2,
// sxy = Sxy - mx my. With A = dS/dmx, B = dS/dSxx, C = dS/dSxy at each
// window centre q, the gradient of sum_q S_q w.r.t. pixel p is
//   (G*A)_p + 2 x_p (G*B)_p + y_p (G*C)_p
// (G is symmetric, so correlation = convolution). Four separable passes.

struct SsimUniforms {
    width: u32,
    height: u32,
    // lambda / (3 * npix): d(lambda * (1 - mean SSIM)) / dS_q per channel.
    scale: f32,
    pad0: u32,
};

@group(0) @binding(0) var<uniform> su: SsimUniforms;
@group(0) @binding(1) var<storage, read> render: array<f32>;
@group(0) @binding(2) var<storage, read> gt: array<u32>;
// 15 floats per pixel: per channel (x, y, x^2, y^2, xy) blurred horizontally.
@group(0) @binding(3) var<storage, read_write> tmp5: array<f32>;
// 9 floats per pixel: per channel (A, B, C).
@group(0) @binding(4) var<storage, read_write> abc: array<f32>;
// 9 floats per pixel: per channel (A, B, C) blurred horizontally.
@group(0) @binding(5) var<storage, read_write> tmp3: array<f32>;
@group(0) @binding(6) var<storage, read_write> d_image: array<f32>;
// Sum of SSIM over pixels and channels, fixed point (1e-3 units).
@group(0) @binding(7) var<storage, read_write> ssim_acc: atomic<u32>;

const RADIUS: i32 = 5;
const C1: f32 = 0.0001;
const C2: f32 = 0.0009;

fn gauss_w(k: i32) -> f32 {
    // Normalised 1D gaussian, sigma 1.5, taps -5..5.
    let w = array<f32, 11>(
        0.0010284, 0.0075988, 0.0360008, 0.1093607, 0.2130055, 0.2660117,
        0.2130055, 0.1093607, 0.0360008, 0.0075988, 0.0010284,
    );
    return w[k + RADIUS];
}

fn gt_rgb(i: u32) -> vec3<f32> {
    let p = gt[i];
    return vec3<f32>(f32(p & 0xFFu), f32((p >> 8u) & 0xFFu), f32((p >> 16u) & 0xFFu)) / 255.0;
}

fn render_rgb(i: u32) -> vec3<f32> {
    return vec3<f32>(render[i * 3u], render[i * 3u + 1u], render[i * 3u + 2u]);
}

fn pixel_index(gid: vec3<u32>, nwg: vec3<u32>) -> u32 {
    return gid.x + gid.y * nwg.x * 256u;
}

// Pass 1: horizontal blur of x, y, x^2, y^2, xy.
@compute @workgroup_size(256)
fn ssim_blur_h(@builtin(global_invocation_id) gid: vec3<u32>, @builtin(num_workgroups) nwg: vec3<u32>) {
    let i = pixel_index(gid, nwg);
    if (i >= su.width * su.height) {
        return;
    }
    let px = i32(i % su.width);
    let row = i - u32(px);
    var acc: array<f32, 15>;
    for (var k = 0u; k < 15u; k = k + 1u) {
        acc[k] = 0.0;
    }
    for (var k = -RADIUS; k <= RADIUS; k = k + 1) {
        let x = px + k;
        if (x < 0 || x >= i32(su.width)) {
            continue;
        }
        let w = gauss_w(k);
        let j = row + u32(x);
        let r = render_rgb(j);
        let g = gt_rgb(j);
        for (var c = 0u; c < 3u; c = c + 1u) {
            acc[c * 5u + 0u] = acc[c * 5u + 0u] + w * r[c];
            acc[c * 5u + 1u] = acc[c * 5u + 1u] + w * g[c];
            acc[c * 5u + 2u] = acc[c * 5u + 2u] + w * r[c] * r[c];
            acc[c * 5u + 3u] = acc[c * 5u + 3u] + w * g[c] * g[c];
            acc[c * 5u + 4u] = acc[c * 5u + 4u] + w * r[c] * g[c];
        }
    }
    for (var k = 0u; k < 15u; k = k + 1u) {
        tmp5[i * 15u + k] = acc[k];
    }
}

// Pass 2: vertical blur -> moments -> SSIM and its partials A, B, C.
@compute @workgroup_size(256)
fn ssim_stats(@builtin(global_invocation_id) gid: vec3<u32>, @builtin(num_workgroups) nwg: vec3<u32>) {
    let i = pixel_index(gid, nwg);
    if (i >= su.width * su.height) {
        return;
    }
    let py = i32(i / su.width);
    let px = i % su.width;
    var m: array<f32, 15>;
    for (var k = 0u; k < 15u; k = k + 1u) {
        m[k] = 0.0;
    }
    for (var k = -RADIUS; k <= RADIUS; k = k + 1) {
        let y = py + k;
        if (y < 0 || y >= i32(su.height)) {
            continue;
        }
        let w = gauss_w(k);
        let j = u32(y) * su.width + px;
        for (var c = 0u; c < 15u; c = c + 1u) {
            m[c] = m[c] + w * tmp5[j * 15u + c];
        }
    }
    var s_sum = 0.0;
    for (var c = 0u; c < 3u; c = c + 1u) {
        let mx = m[c * 5u + 0u];
        let my = m[c * 5u + 1u];
        let sxx = m[c * 5u + 2u];
        let syy = m[c * 5u + 3u];
        let sxy = m[c * 5u + 4u];
        let vx = sxx - mx * mx;
        let vy = syy - my * my;
        let cxy = sxy - mx * my;
        let a = 2.0 * mx * my + C1;
        let b = 2.0 * cxy + C2;
        let cc = mx * mx + my * my + C1;
        let d = vx + vy + C2;
        let s = a * b / (cc * d);
        s_sum = s_sum + s;
        // dS/dmx (through a, b, c, d), dS/dSxx (through d), dS/dSxy (through b).
        let ds_dmx = (2.0 * my * b - 2.0 * my * a) / (cc * d) - s * (2.0 * mx / cc - 2.0 * mx / d);
        let ds_dsxx = -s / d;
        let ds_dsxy = 2.0 * a / (cc * d);
        abc[i * 9u + c * 3u + 0u] = ds_dmx;
        abc[i * 9u + c * 3u + 1u] = ds_dsxx;
        abc[i * 9u + c * 3u + 2u] = ds_dsxy;
    }
    atomicAdd(&ssim_acc, u32(max(s_sum, 0.0) * 1e3 + 0.5));
}

// Pass 3: horizontal blur of A, B, C.
@compute @workgroup_size(256)
fn ssim_grad_h(@builtin(global_invocation_id) gid: vec3<u32>, @builtin(num_workgroups) nwg: vec3<u32>) {
    let i = pixel_index(gid, nwg);
    if (i >= su.width * su.height) {
        return;
    }
    let px = i32(i % su.width);
    let row = i - u32(px);
    var acc: array<f32, 9>;
    for (var k = 0u; k < 9u; k = k + 1u) {
        acc[k] = 0.0;
    }
    for (var k = -RADIUS; k <= RADIUS; k = k + 1) {
        let x = px + k;
        if (x < 0 || x >= i32(su.width)) {
            continue;
        }
        let w = gauss_w(k);
        let j = row + u32(x);
        for (var c = 0u; c < 9u; c = c + 1u) {
            acc[c] = acc[c] + w * abc[j * 9u + c];
        }
    }
    for (var k = 0u; k < 9u; k = k + 1u) {
        tmp3[i * 9u + k] = acc[k];
    }
}

// Pass 4: vertical blur, combine, and add -scale * dS/dx to d_image.
@compute @workgroup_size(256)
fn ssim_grad_v(@builtin(global_invocation_id) gid: vec3<u32>, @builtin(num_workgroups) nwg: vec3<u32>) {
    let i = pixel_index(gid, nwg);
    if (i >= su.width * su.height) {
        return;
    }
    let py = i32(i / su.width);
    let px = i % su.width;
    var g: array<f32, 9>;
    for (var k = 0u; k < 9u; k = k + 1u) {
        g[k] = 0.0;
    }
    for (var k = -RADIUS; k <= RADIUS; k = k + 1) {
        let y = py + k;
        if (y < 0 || y >= i32(su.height)) {
            continue;
        }
        let w = gauss_w(k);
        let j = u32(y) * su.width + px;
        for (var c = 0u; c < 9u; c = c + 1u) {
            g[c] = g[c] + w * tmp3[j * 9u + c];
        }
    }
    let x = render_rgb(i);
    let yv = gt_rgb(i);
    for (var c = 0u; c < 3u; c = c + 1u) {
        let ds_dx = g[c * 3u + 0u] + 2.0 * x[c] * g[c * 3u + 1u] + yv[c] * g[c * 3u + 2u];
        d_image[i * 3u + c] = d_image[i * 3u + c] - su.scale * ds_dx;
    }
}
