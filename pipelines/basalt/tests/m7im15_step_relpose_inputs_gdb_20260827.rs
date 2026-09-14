//! Test-only oracle for the untouched step-only Basalt `computeRelPose` run.
//!
//! The fixture is deliberately bit-oriented: it protects the exact poseLin
//! inputs, return boundary, drel packets, and the pass-dependent temporary
//! translation schedule without changing production code.

use serde_json::Value;

const FIXTURE: &str = include_str!("fixtures/m7im15_step_relpose_inputs_gdb_20260827.json");

fn words(value: &Value, field: &str) -> Vec<u32> {
    value[field]
        .as_array()
        .unwrap_or_else(|| panic!("{field} must be an array"))
        .iter()
        .map(|word| {
            u32::from_str_radix(
                word.as_str()
                    .unwrap_or_else(|| panic!("{field} words must be strings")),
                16,
            )
            .unwrap_or_else(|error| panic!("{field} contains invalid bits: {error}"))
        })
        .collect()
}

fn ordered_bits(bits: u32) -> u32 {
    if bits & 0x8000_0000 != 0 {
        !bits
    } else {
        bits | 0x8000_0000
    }
}

fn ulp_distance(actual: u32, expected: u32) -> u32 {
    ordered_bits(actual).abs_diff(ordered_bits(expected))
}

#[test]
fn authoritative_step_relpose_inputs_and_tmp_schedules_are_stable() {
    let fixture: Value = serde_json::from_str(FIXTURE).expect("GDB fixture must be valid JSON");
    assert_eq!(
        fixture["schema"].as_str(),
        Some("basalt.m7im15.step_relpose_inputs_gdb_test_fixture.v1")
    );
    assert_eq!(fixture["records"].as_array().map(Vec::len), Some(8));
    assert_eq!(
        fixture["filter"]["host_frame_id"].as_u64(),
        Some(1_403_636_579_763_555_584)
    );
    assert_eq!(
        fixture["filter"]["target_frame_id"].as_u64(),
        Some(1_403_636_579_913_555_456)
    );

    let host_lin = [
        0xbd58_2e43,
        0xbf4d_686f,
        0x0000_0000,
        0x3f18_2ffd,
        0x0000_0000,
        0x0000_0000,
        0x0000_0000,
    ];
    let target_lin_pass1 = [
        0xbd5e_3af5,
        0xbf4e_66c7,
        0xbbc3_1a02,
        0x3f16_cb91,
        0x3b2b_6284,
        0xbb51_bffc,
        0xbd4c_7b94,
    ];
    let candidate_t = [0xbd23_5394, 0xbd83_bd75, 0xbc80_1294];
    let older_fixture_t = [0xbd23_5394, 0xbd83_bd78, 0xbc80_12a8];
    let records = fixture["records"].as_array().unwrap();

    let mut candidate_exact_passes = Vec::new();
    let mut older_exact_passes = Vec::new();
    for (expected_pass, record) in records.iter().enumerate() {
        assert_eq!(record["pass"].as_u64(), Some(expected_pass as u64));
        assert_eq!(words(record, "host_pose_lin_qt_u32"), host_lin);
        assert_eq!(words(record, "rpl_d_rel_d_h_column_major_u32").len(), 36);
        assert_eq!(words(record, "rpl_d_rel_d_t_column_major_u32").len(), 36);
        assert_eq!(record["host_is_linearized"].as_bool(), Some(true));
        assert_eq!(record["target_is_linearized"].as_bool(), Some(false));

        let observed_t = words(record, "tmp_t_xyz_u32");
        assert_eq!(observed_t.len(), 3);
        let candidate_ulp = observed_t
            .iter()
            .zip(candidate_t)
            .map(|(actual, expected)| ulp_distance(*actual, expected))
            .collect::<Vec<_>>();
        let older_ulp = observed_t
            .iter()
            .zip(older_fixture_t)
            .map(|(actual, expected)| ulp_distance(*actual, expected))
            .collect::<Vec<_>>();
        if candidate_ulp == [0, 0, 0] {
            candidate_exact_passes.push(expected_pass);
        }
        if older_ulp == [0, 0, 0] {
            older_exact_passes.push(expected_pass);
        }
        if expected_pass == 1 {
            assert_eq!(words(record, "target_pose_lin_qt_u32"), target_lin_pass1);
            assert_eq!(
                words(record, "tmp_q_xyzw_u32"),
                [0x3c34_2b3c, 0xbc4f_602d, 0xbf33_55a4, 0x3f36_a35f,]
            );
            assert_eq!(observed_t, candidate_t);
            assert_ne!(observed_t, older_fixture_t);
            assert_eq!(
                words(record, "T_t_h_sophus_qt_u32"),
                [
                    0xba74_46f7,
                    0xbbcd_d358,
                    0x3adb_39c1,
                    0x3f7f_fe96,
                    0xbddf_b9b6,
                    0xbd47_e71b,
                    0xbc52_7981,
                ]
            );
            assert_eq!(
                words(record, "rpl_d_rel_d_h_column_major_u32"),
                [
                    0x3de3_fe8e,
                    0x3e93_06e1,
                    0xbf73_8e5b,
                    0x0000_0000,
                    0x0000_0000,
                    0x0000_0000,
                    0x3f7e_3f01,
                    0xbd87_ecbc,
                    0x3dc4_f985,
                    0x0000_0000,
                    0x0000_0000,
                    0x0000_0000,
                    0xbd11_8207,
                    0xbf74_a0e5,
                    0xbe95_cd73,
                    0x0000_0000,
                    0x0000_0000,
                    0x0000_0000,
                    0x3d86_87db,
                    0xbd22_8425,
                    0xbb8c_8d8c,
                    0x3de3_fe8e,
                    0x3e93_06e1,
                    0xbf73_8e5b,
                    0xbbec_bb07,
                    0xbc3f_8e51,
                    0x3d88_41e7,
                    0x3f7e_3f01,
                    0xbd87_ecbc,
                    0x3dc4_f985,
                    0x3b7e_5e58,
                    0xbc36_0bff,
                    0x3d12_b627,
                    0xbd11_8207,
                    0xbf74_a0e5,
                    0xbe95_cd73,
                ]
            );
        }
    }

    // This is the requested schedule comparison: the candidate translation
    // is native-exact at iter1/pass1, while the older generic fixture schedule
    // is not exact in any of the eight captured calls.
    assert_eq!(candidate_exact_passes, vec![1]);
    assert!(older_exact_passes.is_empty());
    assert_eq!(
        fixture["schedule_evaluation"]["candidate_exact_passes"],
        serde_json::json!([1])
    );
    assert_eq!(
        fixture["schedule_evaluation"]["fixture_exact_passes"],
        serde_json::json!([])
    );
}
