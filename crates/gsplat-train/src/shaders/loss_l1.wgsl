// L1 photometric loss gradient: d_image = sign(render - gt) / (3 * npix),
// the gradient of mean |render - gt| over all pixels and channels. Also
// accumulates the loss (fixed point, 1e-6 units) for logging.

struct LossUniforms {
    npix: u32,
    pad0: u32,
    pad1: u32,
    pad2: u32,
};

@group(0) @binding(0) var<uniform> lu: LossUniforms;
// Rendered image: row-major RGB f32.
@group(0) @binding(1) var<storage, read> render: array<f32>;
// Ground truth: one packed RGBA8 u32 per pixel (R in the low byte).
@group(0) @binding(2) var<storage, read> gt: array<u32>;
@group(0) @binding(3) var<storage, read_write> d_image: array<f32>;
@group(0) @binding(4) var<storage, read_write> loss_acc: atomic<u32>;

@compute @workgroup_size(256)
fn loss_l1(
    @builtin(global_invocation_id) gid3: vec3<u32>,
    @builtin(num_workgroups) nwg: vec3<u32>,
) {
    let i = gid3.x + gid3.y * nwg.x * 256u;
    if (i >= lu.npix) {
        return;
    }
    let packed = gt[i];
    let g = vec3<f32>(
        f32(packed & 0xFFu),
        f32((packed >> 8u) & 0xFFu),
        f32((packed >> 16u) & 0xFFu),
    ) / 255.0;
    let r = vec3<f32>(render[i * 3u], render[i * 3u + 1u], render[i * 3u + 2u]);
    let diff = r - g;
    let scale = 1.0 / (3.0 * f32(lu.npix));
    let d = sign(diff) * scale;
    d_image[i * 3u] = d.x;
    d_image[i * 3u + 1u] = d.y;
    d_image[i * 3u + 2u] = d.z;
    let l = (abs(diff.x) + abs(diff.y) + abs(diff.z)) * scale;
    atomicAdd(&loss_acc, u32(l * 1e6 + 0.5));
}
