fn bits(x: f32) -> String {
    format!("{:08x}", x.to_bits())
}

fn run(label: &str, omega_bits: [u32; 3]) {
    let x = f32::from_bits(omega_bits[0]);
    let y = f32::from_bits(omega_bits[1]);
    let z = f32::from_bits(omega_bits[2]);
    let theta_sq = x.mul_add(x, y.mul_add(y, z * z));
    let theta_sq_plain = (x * x + y * y) + z * z;
    let theta = theta_sq.sqrt();
    let half = 0.5_f32 * theta;
    let sin = half.sin();
    let cos = half.cos();
    let (sin_cos_s, sin_cos_c) = half.sin_cos();
    let imag = sin / theta;
    let imag_sc = sin_cos_s / theta;
    println!(
        "{label} omega={},{},{} theta_sq={} theta_sq_plain={} theta={} half={} sin={} cos={} sin_cos={} {} imag={} imag_sc={}",
        bits(x), bits(y), bits(z), bits(theta_sq), bits(theta_sq_plain), bits(theta), bits(half),
        bits(sin), bits(cos), bits(sin_cos_s), bits(sin_cos_c), bits(imag), bits(imag_sc)
    );
}

fn main() {
    run("frame1", [0xb92b55aa, 0xb9418699, 0x38359f2f]);
    run("frame2", [0xb84253bc, 0xb99a78ae, 0x387d38e4]);
    run("frame3", [0x38781aee, 0xb9dc2926, 0x38c40d56]);
    run("frame4", [0x3910e3ec, 0xba0d5f2c, 0x3903baf9]);
}
