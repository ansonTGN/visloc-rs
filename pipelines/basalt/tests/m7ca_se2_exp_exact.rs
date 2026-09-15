use nalgebra::Vector3;
use serde::Deserialize;
use visloc_basalt::Se2;

#[derive(Debug, Deserialize)]
struct Fixture {
    recorded: Vec<Case>,
    small_angle: Vec<Case>,
}

#[derive(Debug, Deserialize)]
struct Case {
    label: String,
    tangent: [String; 3],
    rotation: [String; 4],
    translation: [String; 2],
}

fn bits(value: &str) -> u32 {
    u32::from_str_radix(value.strip_prefix("0x").unwrap_or(value), 16).unwrap()
}

fn value(value: &str) -> f32 {
    f32::from_bits(bits(value))
}

fn assert_case(case: &Case) {
    let tangent = Vector3::new(
        value(&case.tangent[0]),
        value(&case.tangent[1]),
        value(&case.tangent[2]),
    );
    let update = Se2::exp(tangent);
    let actual_rotation = [
        update.rotation[(0, 0)].to_bits(),
        update.rotation[(0, 1)].to_bits(),
        update.rotation[(1, 0)].to_bits(),
        update.rotation[(1, 1)].to_bits(),
    ];
    let expected_rotation = case
        .rotation
        .iter()
        .map(|entry| bits(entry))
        .collect::<Vec<_>>();
    assert_eq!(
        actual_rotation.as_slice(),
        expected_rotation.as_slice(),
        "{} rotation",
        case.label
    );

    let actual_translation = [
        update.translation.x.to_bits(),
        update.translation.y.to_bits(),
    ];
    let expected_translation = case
        .translation
        .iter()
        .map(|entry| bits(entry))
        .collect::<Vec<_>>();
    assert_eq!(
        actual_translation.as_slice(),
        expected_translation.as_slice(),
        "{} translation",
        case.label
    );
}

#[test]
fn recorded_temporal_exp_outputs_match_pinned_native() {
    let fixture: Fixture =
        serde_json::from_str(include_str!("fixtures/m7ca_se2_exp_native.json")).unwrap();
    assert_eq!(fixture.recorded.len(), 20);
    for case in &fixture.recorded {
        assert_case(case);
    }
}

#[test]
fn small_angle_exp_outputs_match_pinned_native() {
    let fixture: Fixture =
        serde_json::from_str(include_str!("fixtures/m7ca_se2_exp_native.json")).unwrap();
    assert_eq!(fixture.small_angle.len(), 8);
    for case in &fixture.small_angle {
        assert_case(case);
    }
}
