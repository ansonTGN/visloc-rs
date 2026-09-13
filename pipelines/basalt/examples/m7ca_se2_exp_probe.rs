//! M7ca Rust-side scalar replay for the pinned 20 temporal increments and
//! small-angle branch fixtures.

use nalgebra::Vector3;
use visloc_basalt::Se2;

fn bits(value: f32) -> String {
    format!("0x{:08x}", value.to_bits())
}

fn emit(label: &str, tangent: Vector3<f32>) {
    let update = Se2::exp(tangent);
    let theta = tangent.z;
    let (sin_theta, cos_theta) = (theta.sin(), theta.cos());
    let (sin_over_theta, one_minus_cos_over_theta) = if theta.abs() < 1e-5 {
        let theta_sq = theta * theta;
        (
            1.0 - (1.0 / 6.0) * theta_sq,
            0.5 * theta - (1.0 / 24.0) * theta * theta_sq,
        )
    } else {
        (sin_theta / theta, (1.0 - cos_theta) / theta)
    };
    let candidate_x = sin_over_theta.mul_add(tangent.x, -(one_minus_cos_over_theta * tangent.y));
    let candidate_y = one_minus_cos_over_theta.mul_add(tangent.x, sin_over_theta * tangent.y);
    println!(
        "{label} theta={} rotation={},{},{},{} translation={},{} candidate={},{}",
        bits(tangent.z),
        bits(update.rotation[(0, 0)]),
        bits(update.rotation[(0, 1)]),
        bits(update.rotation[(1, 0)]),
        bits(update.rotation[(1, 1)]),
        bits(update.translation.x),
        bits(update.translation.y),
        bits(candidate_x),
        bits(candidate_y),
    );
}

fn from_bits(value: u32) -> f32 {
    f32::from_bits(value)
}

fn main() {
    let recorded = [
        (0xbeb28e40, 0xbe1adabe, 0xbb9ce858),
        (0x3ce12820, 0x3c65ae81, 0x3a9ce3d8),
        (0x3bd231f5, 0x3a8e8548, 0x3ad33e98),
        (0x3b0d8a18, 0x39772500, 0x3a04bb70),
        (0x3a2c8748, 0x38a7cb00, 0x3924e040),
        (0xbc6a7346, 0x3c3caeea, 0x3b9704a2),
        (0x3b085770, 0xbb2a77d4, 0xb8b79bc0),
        (0xba159230, 0x3a155be0, 0xb81a9e80),
        (0x393ca440, 0xb917ac40, 0x37592200),
        (0xb86ddc00, 0x38311a00, 0xb682a000),
        (0x3e5bb15e, 0x3d8bad19, 0xbc6e3b6a),
        (0xbdd326a0, 0x3ad60820, 0xb9fc3940),
        (0x3d4e2d10, 0xbbf5bb48, 0x3b3a9694),
        (0xbccec6c0, 0x3ba90ec0, 0xbb17b3fc),
        (0x3c5276da, 0xbb4111e0, 0x3abf9c20),
        (0xbe758c7b, 0xbd1b2fc4, 0x3b12b2a8),
        (0x3d91e281, 0x3beecf2a, 0xbc826c56),
        (0xbce5ac64, 0xb92ec0c0, 0x3bea2a84),
        (0x3c2f6de9, 0xb9e95350, 0xbb3feb72),
        (0xbb8701ec, 0x39921b40, 0x3a966db8),
    ];
    for (index, &(tx, ty, theta)) in recorded.iter().enumerate() {
        emit(
            &format!("recorded[{index}]"),
            Vector3::new(from_bits(tx), from_bits(ty), from_bits(theta)),
        );
    }

    let small_thetas = [
        0x00000000, 0x33000000, 0x33ffffff, 0x34000000, 0xb3000000, 0xb3ffffff, 0xb4000000,
        0x35000000,
    ];
    for (index, theta) in small_thetas.into_iter().enumerate() {
        emit(
            &format!("small[{index}]"),
            Vector3::new(
                from_bits(0x3f4ccccd),
                from_bits(0xbf19999a),
                from_bits(theta),
            ),
        );
    }
}
