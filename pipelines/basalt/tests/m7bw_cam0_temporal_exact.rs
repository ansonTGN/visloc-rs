use nalgebra::{Matrix2, Vector2, Vector3};
use serde::Deserialize;
use visloc_basalt::{AffineCompact2f, Se2};

#[derive(Debug, Deserialize)]
struct Fixture {
    source_bits: [String; 2],
    first_level: u32,
    first_increment_bits: [String; 3],
    native_exp_rotation_bits: [String; 4],
    native_exp_translation_bits: [String; 2],
    rust_exp_translation_bits: [String; 2],
    native_order_translation_x_bits: String,
    after_first_update_local_translation_bits: [String; 2],
    frame1_cam0_track2_endpoint_bits: [String; 2],
}

fn parse_bits(value: &str) -> u32 {
    u32::from_str_radix(value.strip_prefix("0x").unwrap_or(value), 16).unwrap()
}

fn from_bits(value: &str) -> f32 {
    f32::from_bits(parse_bits(value))
}

#[test]
fn cam0_temporal_fixture_matches_first_general_se2_native_order() {
    let fixture: Fixture =
        serde_json::from_str(include_str!("fixtures/m7bw_cam0_temporal_track2.json")).unwrap();

    let tangent = Vector3::new(
        from_bits(&fixture.first_increment_bits[0]),
        from_bits(&fixture.first_increment_bits[1]),
        from_bits(&fixture.first_increment_bits[2]),
    );
    let update = Se2::exp(tangent);

    let native_rotation = [
        update.rotation[(0, 0)].to_bits(),
        update.rotation[(0, 1)].to_bits(),
        update.rotation[(1, 0)].to_bits(),
        update.rotation[(1, 1)].to_bits(),
    ];
    let expected_rotation = fixture
        .native_exp_rotation_bits
        .iter()
        .map(|value| parse_bits(value))
        .collect::<Vec<_>>();
    assert_eq!(native_rotation, expected_rotation.as_slice());

    assert_eq!(
        update.translation.y.to_bits(),
        parse_bits(&fixture.native_exp_translation_bits[1])
    );
    assert_eq!(
        update.translation.x.to_bits(),
        parse_bits(&fixture.native_exp_translation_bits[0])
    );
    // Retain the pre-M7ca direct-product bit as a provenance baseline; the
    // production result above is now the pinned native-order value.
    assert_eq!(
        parse_bits(&fixture.rust_exp_translation_bits[0]),
        0xbeb2bd72
    );

    let native_order_x = {
        let theta = tangent.z;
        let sin_over_theta = theta.sin() / theta;
        let one_minus_cos_over_theta = (1.0 - theta.cos()) / theta;
        let second = one_minus_cos_over_theta * tangent.y;
        sin_over_theta.mul_add(tangent.x, -second)
    };
    assert_eq!(
        native_order_x.to_bits(),
        parse_bits(&fixture.native_order_translation_x_bits)
    );

    let source = Vector2::new(
        from_bits(&fixture.source_bits[0]),
        from_bits(&fixture.source_bits[1]),
    );
    let scale = (1_u32 << fixture.first_level) as f32;
    let mut transform = AffineCompact2f::new(Matrix2::identity(), source / scale);
    transform.right_compose_se2(tangent);
    assert_eq!(
        transform.translation().x.to_bits(),
        parse_bits(&fixture.after_first_update_local_translation_bits[0])
    );
    assert_eq!(
        transform.translation().y.to_bits(),
        parse_bits(&fixture.after_first_update_local_translation_bits[1])
    );

    assert_eq!(
        [
            parse_bits(&fixture.frame1_cam0_track2_endpoint_bits[0]),
            parse_bits(&fixture.frame1_cam0_track2_endpoint_bits[1]),
        ],
        [0x422e3d26, 0x42ea1cb7]
    );
}
