//! Focused regression checks for the actual mapper-packet oracle.

use serde_json::Value;

const ORACLE: &str = include_str!("../../../benchmarks/basalt/m6_actual_packet_oracle_v1.json");

#[test]
fn checked_in_m6_oracle_covers_only_actual_kf_removal_packets() {
    let root: Value = serde_json::from_str(ORACLE).expect("M6 oracle JSON");
    assert_eq!(root["schema"], "basalt.m6_actual_packet_oracle.v1");
    assert_eq!(root["packet_count"], 5);

    let records = root["records"].as_array().expect("oracle records");
    assert_eq!(
        records
            .iter()
            .map(|record| record["rust_file"].as_str().unwrap().contains("frame_000"))
            .filter(|present| *present)
            .count(),
        5
    );
    for record in records {
        let comparisons = &record["comparisons"];
        assert!(comparisons["aom_order"].is_boolean());
        assert!(comparisons["kfs_all"].is_boolean());
        assert!(comparisons["kfs_to_marg"].is_boolean());
        assert!(comparisons["frame_state_ids"].as_bool().unwrap());
        assert!(comparisons["fej_flags"].as_bool().unwrap());
        assert!(comparisons["targets_pose_selection"].as_bool().unwrap());
        assert!(comparisons["images_keyframe_ids"].as_bool().unwrap());
        assert!(comparisons["image_count_is_two_per_kf"].as_bool().unwrap());
        assert!(comparisons["used_imu"].as_bool().unwrap());
        assert!(comparisons["prior_wire_contract"].as_bool().unwrap());
        assert_eq!(record["rust"]["sqrt_jacobian_shape"][1], 72);
        assert_eq!(record["rust"]["row_counts"].as_array().unwrap().len(), 4);
        assert_eq!(
            record["rust"]["row_count_sum"],
            record["rust"]["sqrt_jacobian_shape"][0]
        );
        assert_eq!(
            record["rust"]["marginalization"]["poses_to_marg"]
                .as_array()
                .unwrap()
                .len(),
            1
        );
        assert!(record["rust"]["marginalization"]["states_to_marg_all"]
            .as_array()
            .unwrap()
            .is_empty());
        assert_eq!(
            record["rust"]["marginalization"]["states_to_marg_vel_bias"]
                .as_array()
                .unwrap()
                .len(),
            1
        );
        assert_eq!(record["rust"]["aom_order"].as_array().unwrap().len(), 9);
        assert_eq!(record["rust"]["frame_poses"].as_array().unwrap().len(), 7);
        assert_eq!(record["rust"]["frame_states"].as_array().unwrap().len(), 3);
        assert_eq!(record["frame_tables"]["poses"]["rust_count"], 7);
        assert_eq!(record["frame_tables"]["states"]["rust_count"], 3);
        assert!(
            record["frame_tables"]["poses"]["matched_count"]
                .as_u64()
                .unwrap()
                <= 7
        );
        assert_eq!(record["frame_tables"]["states"]["matched_count"], 3);
        assert_eq!(
            record["upstream"]["abs_h_shape"],
            serde_json::json!([72, 72])
        );
    }
}
