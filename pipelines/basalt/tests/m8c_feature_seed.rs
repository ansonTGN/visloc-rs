//! Bounded M8c seed contract.
//!
//! The upstream diagnostic intentionally contains only the first eight
//! `TimeCamId`s (the current bounded task is not the final first-20 gate).  We
//! nevertheless validate every retained descriptor/hash/BoW/match entry so a
//! later image-selection expansion cannot silently weaken this seed.

use std::collections::{BTreeMap, BTreeSet};

use serde_json::Value;
use visloc_basalt::mapper::{compute_hash_bow, hash_descriptor};

const FIXTURE: &str = include_str!("../../../benchmarks/basalt/m8c_feature_oracle.json");

fn usize_field(value: &Value, field: &str) -> usize {
    value[field].as_u64().unwrap() as usize
}

fn f64_field(value: &Value, field: &str) -> f64 {
    value[field].as_f64().unwrap()
}

fn descriptor(value: &Value) -> [u8; 32] {
    let row = value.as_array().unwrap();
    assert_eq!(row.len(), 32);
    let mut out = [0_u8; 32];
    for (index, byte) in row.iter().enumerate() {
        out[index] = byte.as_u64().unwrap() as u8;
    }
    out
}

#[test]
fn bounded_m8c_seed_fixture_is_self_consistent() {
    let root: Value = serde_json::from_str(FIXTURE).unwrap();
    assert_eq!(root["schema"], "basalt.m8c_feature_oracle.v1");
    assert_eq!(
        root["upstream"]["commit"],
        "0f3b2b52c807f70ff4e2973ce253c73329eea7bc"
    );
    assert_eq!(root["selection"]["max_time_cam_ids"], 8);
    assert_eq!(root["selection"]["full_gate_target_time_cam_ids"], 20);
    assert_eq!(
        root["selection"]["full_gate_status"],
        "not_claimed; this artifact intentionally caps at 8"
    );

    let features = root["features"].as_array().unwrap();
    assert_eq!(features.len(), 8);
    let expected_counts = [728, 737, 741, 750, 717, 681, 730, 747];
    assert_eq!(
        features
            .iter()
            .map(|feature| usize_field(feature, "corner_count"))
            .collect::<Vec<_>>(),
        expected_counts
    );

    for (image_index, feature) in features.iter().enumerate() {
        let count = usize_field(feature, "corner_count");
        assert_eq!(usize_field(feature, "descriptor_count"), count);
        assert_eq!(usize_field(feature, "ray_count"), count);
        assert_eq!(usize_field(feature, "hash_count"), count);
        let hashes = feature["hashes_16bit"].as_array().unwrap();
        assert_eq!(hashes.len(), count);
        assert!(hashes.iter().all(|hash| hash.as_u64().unwrap() < (1 << 16)));

        let bow = feature["bow_vector"].as_array().unwrap();
        let mut bow_map = BTreeMap::new();
        for entry in bow {
            let hash = entry["hash"].as_u64().unwrap() as u32;
            let weight = entry["weight"].as_f64().unwrap();
            assert!(bow_map.insert(hash, weight).is_none());
        }
        assert!((bow_map.values().sum::<f64>() - 1.0).abs() < 1e-12);

        // Every descriptor byte is retained for the first image; later images
        // deliberately carry a cryptographic summary instead of the full rows.
        if image_index == 0 {
            let corners = feature["corners_xy"].as_array().unwrap();
            let angles = feature["corner_angles"].as_array().unwrap();
            let rays = feature["corners_3d"].as_array().unwrap();
            let descriptors = feature["descriptor_bytes"].as_array().unwrap();
            assert_eq!(corners.len(), count);
            assert_eq!(angles.len(), count);
            assert_eq!(rays.len(), count);
            assert_eq!(descriptors.len(), count);
            assert!(angles
                .iter()
                .all(|angle| angle.as_f64().is_some_and(f64::is_finite)));
            assert!(corners.iter().all(|corner| {
                let xy = corner.as_array().unwrap();
                xy.len() == 2 && xy.iter().all(|value| value.as_f64().is_some())
            }));
            assert_eq!(feature["corner_ids"][0], 0);
            assert_eq!(feature["corner_ids"][count - 1], (count - 1) as u64);
            assert_eq!(
                descriptors[0],
                serde_json::json!([
                    69, 209, 155, 99, 180, 6, 75, 176, 36, 224, 15, 229, 209, 135, 217, 100, 22,
                    148, 124, 254, 201, 144, 125, 20, 236, 141, 175, 196, 44, 163, 19, 112
                ])
            );
            assert_eq!(feature["corners_xy"][0], serde_json::json!([379, 48]));
            assert_eq!(
                feature["corners_3d"][0],
                serde_json::json!([
                    0.027530503179258863,
                    -0.42417840480524077,
                    0.9051600699829715,
                    0
                ])
            );

            let descriptor_bytes = descriptors.iter().map(descriptor).collect::<Vec<_>>();
            let (expected_hashes, expected_bow) = compute_hash_bow(&descriptor_bytes, 16);
            let actual_hashes = hashes
                .iter()
                .map(|hash| hash.as_u64().unwrap() as u32)
                .collect::<Vec<_>>();
            assert_eq!(actual_hashes, expected_hashes);
            assert_eq!(
                expected_bow
                    .iter()
                    .map(|entry| (entry.hash, entry.weight))
                    .collect::<Vec<_>>(),
                bow_map.into_iter().collect::<Vec<_>>()
            );
            for ray in rays {
                let values = ray.as_array().unwrap();
                assert_eq!(values.len(), 4);
                let norm = values[..3]
                    .iter()
                    .map(|value| value.as_f64().unwrap().powi(2))
                    .sum::<f64>()
                    .sqrt();
                assert!((norm - 1.0).abs() < 1e-12);
                assert_eq!(values[3], 0.0);
            }
        } else {
            assert!(feature.get("descriptor_bytes").is_none());
            assert!(feature["descriptor_bytes_sha256"].as_str().unwrap().len() == 64);
            assert!(feature["corners_xy_sha256"].as_str().unwrap().len() == 64);
            assert!(feature["corners_3d_sha256"].as_str().unwrap().len() == 64);
        }

        // HashBow's per-descriptor values are the exact 16-bit permutation;
        // check the public single-descriptor helper against a fixture row too.
        if image_index == 0 {
            let first = descriptor(&feature["descriptor_bytes"][0]);
            assert_eq!(
                hash_descriptor(&first, 16),
                feature["hashes_16bit"][0].as_u64().unwrap() as u32
            );
        }
    }

    let stereo = root["stereo"]["pairs"].as_array().unwrap();
    assert_eq!(stereo.len(), 4);
    let expected_raw = [245, 222, 187, 322];
    let expected_inliers = [157, 154, 118, 210];
    for (index, pair) in stereo.iter().enumerate() {
        assert_eq!(usize_field(pair, "raw_match_count"), expected_raw[index]);
        assert_eq!(
            usize_field(pair, "essential_inlier_count"),
            expected_inliers[index]
        );
        assert!(pair["mapper_feature_matches_stored"].as_bool().unwrap());
        let raw = pair["raw_mutual_hamming"].as_array().unwrap();
        let raw_ids = raw
            .iter()
            .map(|entry| {
                (
                    usize_field(entry, "left_feature_id"),
                    usize_field(entry, "right_feature_id"),
                )
            })
            .collect::<BTreeSet<_>>();
        assert_eq!(raw_ids.len(), raw.len());
        assert!(raw.iter().all(|entry| f64_field(entry, "hamming") <= 70.0));
        let inliers = pair["essential_inlier_ids"].as_array().unwrap();
        assert!(inliers.iter().all(|entry| {
            let ids = entry.as_array().unwrap();
            raw_ids.contains(&(
                ids[0].as_u64().unwrap() as usize,
                ids[1].as_u64().unwrap() as usize,
            ))
        }));
    }
}
