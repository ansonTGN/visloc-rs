//! M7bw Rust-side scalar trace for the first cam0/frame1 track-2 update.

use nalgebra::Vector3;
use visloc_basalt::Se2;

fn bits(value: f32) -> String {
    format!("0x{:08x}", value.to_bits())
}

fn main() {
    let tangent = Vector3::new(
        f32::from_bits(0xbeb28e40),
        f32::from_bits(0xbe1adabe),
        f32::from_bits(0xbb9ce858),
    );
    let theta = tangent.z;
    let sin_theta = theta.sin();
    let cos_theta = theta.cos();
    let sin_over_theta = sin_theta / theta;
    let one_minus_cos_over_theta = (1.0 - cos_theta) / theta;
    let first = sin_over_theta * tangent.x;
    let second = one_minus_cos_over_theta * tangent.y;
    let direct_x = first - second;
    let fma_x = (-one_minus_cos_over_theta).mul_add(tangent.y, first);
    let native_x = sin_over_theta.mul_add(tangent.x, -second);
    let direct_y = one_minus_cos_over_theta * tangent.x + sin_over_theta * tangent.y;
    let fma_y = one_minus_cos_over_theta.mul_add(tangent.x, sin_over_theta * tangent.y);
    let update = Se2::exp(tangent);

    println!("theta={}", bits(theta));
    println!("sin_cos={},{}", bits(sin_theta), bits(cos_theta));
    println!(
        "coefficients={},{}",
        bits(sin_over_theta),
        bits(one_minus_cos_over_theta)
    );
    println!("products={},{}", bits(first), bits(second));
    println!("direct={},{}", bits(direct_x), bits(direct_y));
    println!("fma={},{}", bits(fma_x), bits(fma_y));
    println!("native_order={},{}", bits(native_x), bits(direct_y));
    println!(
        "update={},{}",
        bits(update.translation.x),
        bits(update.translation.y)
    );
}
