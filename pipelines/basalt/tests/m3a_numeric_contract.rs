use nalgebra::{Matrix2, Vector2, Vector3};
use serde::Deserialize;
use visloc_basalt::{
    AffineCompact2f, MeanNormalizedPatch51, Pattern51, RawU16Image, RawU16Pyramid,
};

#[derive(Debug, Deserialize)]
struct PyramidFixture {
    width: usize,
    height: usize,
    pixels: Vec<u16>,
    levels: Vec<LevelFixture>,
}

#[derive(Debug, Deserialize)]
struct LevelFixture {
    width: usize,
    height: usize,
    pixels: Vec<u16>,
}

#[derive(Debug, Deserialize)]
struct PatchFixture {
    width: usize,
    height: usize,
    position: [f32; 2],
    value_formula: String,
    mean: f32,
    data_prefix: Vec<f32>,
    jacobian_prefix: Vec<[f32; 3]>,
}

#[derive(Debug, Deserialize)]
struct Se2Fixture {
    linear: [[f32; 2]; 2],
    translation: [f32; 2],
    increment: [f32; 3],
    expected_linear: [[f32; 2]; 2],
    expected_translation: [f32; 2],
}

#[test]
fn raw_u16_pyramid_matches_upstream_binomial_golden_fixture() {
    let fixture: PyramidFixture =
        serde_json::from_str(include_str!("fixtures/raw_u16_pyramid_golden.json")).unwrap();
    let image = RawU16Image::new(fixture.width, fixture.height, fixture.pixels).unwrap();
    let pyramid = RawU16Pyramid::from_image(image, fixture.levels.len() - 1).unwrap();

    assert_eq!(pyramid.levels().len(), fixture.levels.len());
    for (actual, expected) in pyramid.levels().iter().zip(fixture.levels) {
        assert_eq!(actual.width(), expected.width);
        assert_eq!(actual.height(), expected.height);
        assert_eq!(actual.pixels(), expected.pixels.as_slice());
    }
}

#[test]
fn pattern51_and_mean_normalized_jacobian_match_upstream_golden_fixture() {
    let fixture: PatchFixture =
        serde_json::from_str(include_str!("fixtures/patch51_golden.json")).unwrap();
    assert_eq!(fixture.value_formula, "1000 + 2*x + 3*y + x*y");
    assert_eq!(Pattern51::SAMPLE_COUNT, 52);

    let image = RawU16Image::from_fn(fixture.width, fixture.height, |x, y| {
        (1000 + 2 * x + 3 * y + x * y) as u16
    })
    .unwrap();
    let patch = MeanNormalizedPatch51::from_image(
        &image,
        Vector2::new(fixture.position[0], fixture.position[1]),
    );

    assert!(patch.valid);
    assert_eq!(patch.valid_samples, 52);
    assert!((patch.mean - fixture.mean).abs() < 1e-5);
    for (actual, expected) in patch.data.iter().zip(fixture.data_prefix) {
        assert!((actual - expected).abs() < 2e-6, "{actual} != {expected}");
    }
    for (row, expected) in patch.jacobian_se2.row_iter().zip(fixture.jacobian_prefix) {
        for (actual, expected) in row.iter().zip(expected) {
            assert!((actual - expected).abs() < 2e-6, "{actual} != {expected}");
        }
    }
}

#[test]
fn affine_compact2f_se2_ic_update_matches_upstream_golden_fixture() {
    let fixture: Se2Fixture =
        serde_json::from_str(include_str!("fixtures/se2_update_golden.json")).unwrap();
    let mut transform = AffineCompact2f::new(
        Matrix2::new(
            fixture.linear[0][0],
            fixture.linear[0][1],
            fixture.linear[1][0],
            fixture.linear[1][1],
        ),
        Vector2::new(fixture.translation[0], fixture.translation[1]),
    );
    transform
        .try_right_compose_se2(Vector3::new(
            fixture.increment[0],
            fixture.increment[1],
            fixture.increment[2],
        ))
        .unwrap();

    for row in 0..2 {
        for col in 0..2 {
            assert!(
                (transform.linear()[(row, col)] - fixture.expected_linear[row][col]).abs() < 2e-6
            );
        }
    }
    for index in 0..2 {
        assert!(
            (transform.translation()[index] - fixture.expected_translation[index]).abs() < 2e-6
        );
    }
}
