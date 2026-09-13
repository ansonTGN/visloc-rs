// Diagnostic-only fixture for the UpstreamF32 quaternion state boundary.
// The active window stores only q.xyz, then decode_rotation_with_mode()
// reconstructs q.w from 1 - ||q.xyz||^2.  This probe keeps the authoritative
// predicted-state q bits and reports the candidate reduction/rounding paths.

#[inline]
fn f(bits: u32) -> f32 {
    f32::from_bits(bits)
}

#[inline]
fn bits(value: f32) -> String {
    format!("{:08x}", value.to_bits())
}

#[inline]
fn decode_xyz(q: [f32; 4], norm_sq: f32) -> f32 {
    let _ = q;
    (1.0_f32 - norm_sq).max(0.0_f32).sqrt()
}

fn main() {
    // Authoritative clean/native state endpoint quaternions, xyzw order.
    let states = [
        (1_u32, [0xbd5cdea6, 0xbf4d341f, 0xbb2aa78b, 0x3f186f67]),
        (2_u32, [0xbd5cf5f2, 0xbf4d97c0, 0xbbac4997, 0x3f17e7a6]),
        (3_u32, [0xbd5e760f, 0xbf4e5e85, 0xbbc2d95f, 0x3f16d68a]),
        (4_u32, [0xbd5f79e6, 0xbf4f45a2, 0xbbb53f68, 0x3f159719]),
    ];
    for (frame, raw_bits) in states {
        let q = raw_bits.map(f);
        let x = q[0];
        let y = q[1];
        let z = q[2];
        let native_w = q[3];
        // This is exactly the active Rust decode_rotation_with_mode order.
        let active = (x * x + y * y + z * z).max(0.0_f32).sqrt();
        let active_w = decode_xyz(q, x * x + y * y + z * z);
        let alt_right = (x * x + (y * y + z * z)).max(0.0_f32).sqrt();
        let alt_pair = ((x * x + z * z) + y * y).max(0.0_f32).sqrt();
        let fma_right = (x.mul_add(x, y.mul_add(y, z * z))).max(0.0_f32).sqrt();
        let fma_left = ((x.mul_add(x, y * y)).mul_add(1.0_f32, z * z))
            .max(0.0_f32)
            .sqrt();
        let complement_left = ((1.0_f32 - x * x) - y * y - z * z)
            .max(0.0_f32)
            .sqrt();
        let complement_right = (1.0_f32 - (x * x + (y * y + z * z)))
            .max(0.0_f32)
            .sqrt();
        println!(
            "frame={frame} native_w={} active_norm={} active_w={} alt_right={} alt_pair={} fma_right={} fma_left={} complement_left={} complement_right={}",
            bits(native_w),
            bits(active),
            bits(active_w),
            bits(alt_right),
            bits(alt_pair),
            bits(fma_right),
            bits(fma_left),
            bits(complement_left),
            bits(complement_right),
        );
    }
}
