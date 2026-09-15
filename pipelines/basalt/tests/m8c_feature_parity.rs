//! Ignored real-image M8c frontend parity gate.
//!
//! This test deliberately stays out of normal CI: it reads the external
//! MH_01_easy EuRoC images and the pinned Basalt calibration/config.  The
//! `M8C_FEATURE_PARITY_MEASURE=1` mode is a diagnostic escape hatch used while
//! porting the detector; the default path is a strict golden assertion.  An
//! explicit run must set `VISLOC_BASALT_MH01_ROOT`,
//! `VISLOC_BASALT_CALIBRATION`, and `VISLOC_BASALT_CONFIG`; this probe has no
//! machine-specific or target-directory fallbacks.

use std::{
    collections::{BTreeMap, BTreeSet},
    env,
    path::PathBuf,
};

use nalgebra::Point2;
use serde_json::Value;
use visloc_basalt::mapper::{
    extract_mapper_features, match_stereo_features, match_temporal_ransac_seeded,
    match_temporal_stage, query_bow_candidates, DescriptorMatch, MapperImageFeatures,
    MapperImageId, OfflineMapperConfig,
};
use visloc_basalt::{BasaltCalibration, EurocSensorDataset};

const FIXTURE: &str = include_str!("../../../benchmarks/basalt/m8c_feature_oracle20.json");
const RANSAC_FIXTURE: &str =
    include_str!("../../../benchmarks/basalt/m8c_feature_oracle20_ransac.json");
const CANONICAL_FLOAT_SCALE: f64 = 1e12;

const SHA256_K: [u32; 64] = [
    0x428a_2f98,
    0x7137_4491,
    0xb5c0_fbcf,
    0xe9b5_dba5,
    0x3956_c25b,
    0x59f1_11f1,
    0x923f_82a4,
    0xab1c_5ed5,
    0xd807_aa98,
    0x1283_5b01,
    0x2431_85be,
    0x550c_7dc3,
    0x72be_5d74,
    0x80de_b1fe,
    0x9bdc_06a7,
    0xc19b_f174,
    0xe49b_69c1,
    0xefbe_4786,
    0x0fc1_9dc6,
    0x240c_a1cc,
    0x2de9_2c6f,
    0x4a74_84aa,
    0x5cb0_a9dc,
    0x76f9_88da,
    0x983e_5152,
    0xa831_c66d,
    0xb003_27c8,
    0xbf59_7fc7,
    0xc6e0_0bf3,
    0xd5a7_9147,
    0x06ca_6351,
    0x1429_2967,
    0x27b7_0a85,
    0x2e1b_2138,
    0x4d2c_6dfc,
    0x5338_0d13,
    0x650a_7354,
    0x766a_0abb,
    0x81c2_c92e,
    0x9272_2c85,
    0xa2bf_e8a1,
    0xa81a_664b,
    0xc24b_8b70,
    0xc76c_51a3,
    0xd192_e819,
    0xd699_0624,
    0xf40e_3585,
    0x106a_a070,
    0x19a4_c116,
    0x1e37_6c08,
    0x2748_774c,
    0x34b0_bcb5,
    0x391c_0cb3,
    0x4ed8_aa4a,
    0x5b9c_ca4f,
    0x682e_6ff3,
    0x748f_82ee,
    0x78a5_636f,
    0x84c8_7814,
    0x8cc7_0208,
    0x90be_fffa,
    0xa450_6ceb,
    0xbef9_a3f7,
    0xc671_78f2,
];

fn sha256(input: &[u8]) -> [u8; 32] {
    let mut message = input.to_vec();
    let bit_length = (message.len() as u64) * 8;
    message.push(0x80);
    while message.len() % 64 != 56 {
        message.push(0);
    }
    message.extend_from_slice(&bit_length.to_be_bytes());

    let mut state = [
        0x6a09_e667_u32,
        0xbb67_ae85,
        0x3c6e_f372,
        0xa54f_f53a,
        0x510e_527f,
        0x9b05_688c,
        0x1f83_d9ab,
        0x5be0_cd19,
    ];
    for chunk in message.chunks_exact(64) {
        let mut words = [0_u32; 64];
        for (index, word) in words[..16].iter_mut().enumerate() {
            let offset = index * 4;
            *word = u32::from_be_bytes([
                chunk[offset],
                chunk[offset + 1],
                chunk[offset + 2],
                chunk[offset + 3],
            ]);
        }
        for index in 16..64 {
            let s0 = words[index - 15].rotate_right(7)
                ^ words[index - 15].rotate_right(18)
                ^ (words[index - 15] >> 3);
            let s1 = words[index - 2].rotate_right(17)
                ^ words[index - 2].rotate_right(19)
                ^ (words[index - 2] >> 10);
            words[index] = words[index - 16]
                .wrapping_add(s0)
                .wrapping_add(words[index - 7])
                .wrapping_add(s1);
        }

        let [mut a, mut b, mut c, mut d, mut e, mut f, mut g, mut h] = state;
        for index in 0..64 {
            let sigma1 = e.rotate_right(6) ^ e.rotate_right(11) ^ e.rotate_right(25);
            let choose = (e & f) ^ ((!e) & g);
            let temp1 = h
                .wrapping_add(sigma1)
                .wrapping_add(choose)
                .wrapping_add(SHA256_K[index])
                .wrapping_add(words[index]);
            let sigma0 = a.rotate_right(2) ^ a.rotate_right(13) ^ a.rotate_right(22);
            let majority = (a & b) ^ (a & c) ^ (b & c);
            let temp2 = sigma0.wrapping_add(majority);
            h = g;
            g = f;
            f = e;
            e = d.wrapping_add(temp1);
            d = c;
            c = b;
            b = a;
            a = temp1.wrapping_add(temp2);
        }
        state[0] = state[0].wrapping_add(a);
        state[1] = state[1].wrapping_add(b);
        state[2] = state[2].wrapping_add(c);
        state[3] = state[3].wrapping_add(d);
        state[4] = state[4].wrapping_add(e);
        state[5] = state[5].wrapping_add(f);
        state[6] = state[6].wrapping_add(g);
        state[7] = state[7].wrapping_add(h);
    }

    let mut output = [0_u8; 32];
    for (index, word) in state.iter().enumerate() {
        output[index * 4..index * 4 + 4].copy_from_slice(&word.to_be_bytes());
    }
    output
}

fn hex_digest(digest: [u8; 32]) -> String {
    digest.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn push_canonical_float(bytes: &mut Vec<u8>, value: f64) {
    let fixed = (value * CANONICAL_FLOAT_SCALE) as i64;
    bytes.extend_from_slice(&fixed.to_le_bytes());
}

fn required_path(name: &str, description: &str) -> PathBuf {
    env::var_os(name)
        .map(PathBuf::from)
        .unwrap_or_else(|| panic!("set {name} to {description} for this ignored gate"))
}

fn diagnostics_enabled() -> bool {
    env::var_os("M8C_FEATURE_PARITY_DIAGNOSTIC").is_some()
}

fn mapper_config(dataset: &EurocSensorDataset) -> OfflineMapperConfig {
    let value = |key: &str| {
        dataset
            .config()
            .value::<serde_json::Value>(key)
            .unwrap_or_else(|error| panic!("pinned config key {key}: {error}"))
    };
    OfflineMapperConfig {
        max_points: value("config.mapper_detection_num_points")
            .as_u64()
            .unwrap() as usize,
        max_hamming: value("config.mapper_max_hamming_distance")
            .as_u64()
            .unwrap() as u32,
        second_best_ratio: value("config.mapper_second_best_test_ratio")
            .as_f64()
            .unwrap(),
        bow_bits: value("config.mapper_bow_num_bits").as_u64().unwrap() as u8,
        match_window: value("config.mapper_num_frames_to_match").as_u64().unwrap(),
        frames_to_match_threshold: value("config.mapper_frames_to_match_threshold")
            .as_f64()
            .unwrap(),
        min_matches: value("config.mapper_min_matches").as_u64().unwrap() as usize,
        ransac_threshold: value("config.mapper_ransac_threshold").as_f64().unwrap(),
        min_track_length: value("config.mapper_min_track_length").as_u64().unwrap() as usize,
        min_triangulation_distance: value("config.mapper_min_triangulation_dist")
            .as_f64()
            .unwrap(),
    }
}

fn point_array(value: &Value) -> Point2<f64> {
    let values = value.as_array().unwrap();
    Point2::new(values[0].as_f64().unwrap(), values[1].as_f64().unwrap())
}

fn descriptor_array(value: &Value) -> [u8; 32] {
    let values = value.as_array().unwrap();
    assert_eq!(values.len(), 32);
    let mut descriptor = [0_u8; 32];
    for (index, value) in values.iter().enumerate() {
        descriptor[index] = value.as_u64().unwrap() as u8;
    }
    descriptor
}

fn ray_array(value: &Value) -> [f64; 4] {
    let values = value.as_array().unwrap();
    [
        values[0].as_f64().unwrap(),
        values[1].as_f64().unwrap(),
        values[2].as_f64().unwrap(),
        values[3].as_f64().unwrap_or(0.0),
    ]
}

fn max_point_error(actual: &[Point2<f64>], expected: &[Point2<f64>]) -> (f64, usize) {
    actual
        .iter()
        .zip(expected)
        .enumerate()
        .map(|(index, (a, e))| ((a.coords - e.coords).norm(), index))
        .max_by(|left, right| left.0.total_cmp(&right.0))
        .unwrap_or((0.0, 0))
}

fn max_scalar_error(actual: &[f64], expected: &[f64]) -> (f64, usize) {
    actual
        .iter()
        .zip(expected)
        .enumerate()
        .map(|(index, (a, e))| ((a - e).abs(), index))
        .max_by(|left, right| left.0.total_cmp(&right.0))
        .unwrap_or((0.0, 0))
}

fn max_ray_error(actual: &[[f64; 4]], expected: &[[f64; 4]]) -> (f64, usize) {
    actual
        .iter()
        .zip(expected)
        .enumerate()
        .map(|(index, (a, e))| {
            (
                a.iter()
                    .zip(e)
                    .map(|(a, e)| (a - e).abs())
                    .fold(0.0, f64::max),
                index,
            )
        })
        .max_by(|left, right| left.0.total_cmp(&right.0))
        .unwrap_or((0.0, 0))
}

fn expected_features(
    value: &Value,
) -> (
    Vec<Point2<f64>>,
    Vec<f64>,
    Vec<[u8; 32]>,
    Vec<[f64; 4]>,
    Vec<u32>,
) {
    let corners = value["corners_xy"]
        .as_array()
        .unwrap()
        .iter()
        .map(point_array)
        .collect::<Vec<_>>();
    let angles = value["corner_angles"]
        .as_array()
        .unwrap()
        .iter()
        .map(|value| value.as_f64().unwrap())
        .collect::<Vec<_>>();
    let descriptors = value["descriptor_bytes"]
        .as_array()
        .unwrap()
        .iter()
        .map(descriptor_array)
        .collect::<Vec<_>>();
    let rays = value["corners_3d"]
        .as_array()
        .unwrap()
        .iter()
        .map(ray_array)
        .collect::<Vec<_>>();
    let hashes = value["hashes_16bit"]
        .as_array()
        .unwrap()
        .iter()
        .map(|value| value.as_u64().unwrap() as u32)
        .collect::<Vec<_>>();
    (corners, angles, descriptors, rays, hashes)
}

fn expected_hashes_and_bow(value: &Value) -> (Vec<u32>, Vec<(u32, f64)>) {
    let hashes = value["hashes_16bit"]
        .as_array()
        .unwrap()
        .iter()
        .map(|value| value.as_u64().unwrap() as u32)
        .collect::<Vec<_>>();
    let bow = value["bow_vector"]
        .as_array()
        .unwrap()
        .iter()
        .map(|entry| {
            (
                entry["hash"].as_u64().unwrap() as u32,
                entry["weight"].as_f64().unwrap(),
            )
        })
        .collect::<Vec<_>>();
    (hashes, bow)
}

fn actual_canonical_hashes(features: &MapperImageFeatures) -> BTreeMap<&'static str, String> {
    let mut corners = Vec::with_capacity(features.corners.len() * 16);
    for corner in &features.corners {
        push_canonical_float(&mut corners, corner.x);
        push_canonical_float(&mut corners, corner.y);
    }

    let mut angles = Vec::with_capacity(features.corner_angles.len() * 8);
    for &angle in &features.corner_angles {
        push_canonical_float(&mut angles, angle);
    }

    let descriptors = features
        .descriptors
        .iter()
        .flat_map(|descriptor| descriptor.iter().copied())
        .collect::<Vec<_>>();

    let mut rays = Vec::with_capacity(features.rays.len() * 32);
    for ray in &features.rays {
        for &value in ray {
            push_canonical_float(&mut rays, value);
        }
    }

    let hashes = features
        .hashes
        .iter()
        .flat_map(|hash| hash.to_le_bytes())
        .collect::<Vec<_>>();

    let mut bow = Vec::with_capacity(features.bow_vector.len() * 16);
    for entry in &features.bow_vector {
        bow.extend_from_slice(&entry.hash.to_le_bytes());
        push_canonical_float(&mut bow, entry.weight);
    }

    [
        ("corners_xy", corners),
        ("corner_angles", angles),
        ("descriptor_bytes", descriptors),
        ("corners_3d", rays),
        ("hashes_16bit", hashes),
        ("bow_vector", bow),
    ]
    .into_iter()
    .map(|(name, bytes)| (name, hex_digest(sha256(&bytes))))
    .collect()
}

fn assert_canonical_hashes(actual: &MapperImageFeatures, oracle: &Value, context: &str) {
    let expected = oracle["canonical_sha256"].as_object().unwrap();
    for (name, digest) in actual_canonical_hashes(actual) {
        assert_eq!(
            digest,
            expected[name].as_str().unwrap(),
            "{context}: canonical SHA-256 {name}"
        );
    }
}

fn assert_bow_close(actual: &[(u32, f64)], expected: &[(u32, f64)], context: &str) {
    assert_eq!(actual.len(), expected.len(), "{context}: BoW length");
    for (index, ((actual_hash, actual_weight), (expected_hash, expected_weight))) in
        actual.iter().zip(expected).enumerate()
    {
        assert_eq!(actual_hash, expected_hash, "{context}: BoW hash {index}");
        assert!(
            (actual_weight - expected_weight).abs() <= 1e-15,
            "{context}: BoW weight {index}: actual={actual_weight:.17e} expected={expected_weight:.17e}"
        );
    }
}

fn assert_feature_summary(actual: &MapperImageFeatures, oracle: &Value, context: &str) {
    let expected_count = oracle["corner_count"].as_u64().unwrap() as usize;
    assert_eq!(
        actual.corners.len(),
        expected_count,
        "{context}: corner count"
    );
    assert_eq!(
        actual.corner_angles.len(),
        expected_count,
        "{context}: angle count"
    );
    assert_eq!(
        actual.descriptors.len(),
        expected_count,
        "{context}: descriptor count"
    );
    assert_eq!(actual.rays.len(), expected_count, "{context}: ray count");
    let (expected_hashes, expected_bow) = expected_hashes_and_bow(oracle);
    assert_eq!(actual.hashes, expected_hashes, "{context}: 16-bit hashes");
    let actual_bow = actual
        .bow_vector
        .iter()
        .map(|entry| (entry.hash, entry.weight))
        .collect::<Vec<_>>();
    assert_bow_close(&actual_bow, &expected_bow, context);
    assert_canonical_hashes(actual, oracle, context);
}

fn expected_stereo_pair(value: &Value) -> (Vec<DescriptorMatch>, Vec<(usize, usize)>, bool) {
    let raw = value["raw_mutual_hamming"]
        .as_array()
        .unwrap()
        .iter()
        .map(|entry| DescriptorMatch {
            left: entry["left_feature_id"].as_u64().unwrap(),
            right: entry["right_feature_id"].as_u64().unwrap(),
            distance: entry["hamming"].as_u64().unwrap() as u32,
        })
        .collect::<Vec<_>>();
    let inliers = value["essential_inlier_ids"]
        .as_array()
        .unwrap()
        .iter()
        .map(|entry| {
            (
                entry[0].as_u64().unwrap() as usize,
                entry[1].as_u64().unwrap() as usize,
            )
        })
        .collect::<Vec<_>>();
    let stored = value["mapper_feature_matches_stored"].as_bool().unwrap();
    (raw, inliers, stored)
}

fn temporal_raw_digest(matches: &[DescriptorMatch]) -> String {
    let mut bytes = Vec::with_capacity(matches.len() * 12);
    for match_ in matches {
        bytes.extend_from_slice(&(match_.left as u32).to_le_bytes());
        bytes.extend_from_slice(&(match_.right as u32).to_le_bytes());
        bytes.extend_from_slice(&match_.distance.to_le_bytes());
    }
    hex_digest(sha256(&bytes))
}

fn expected_temporal_candidates(value: &Value) -> Vec<(MapperImageId, f64)> {
    value["candidates"]
        .as_array()
        .unwrap()
        .iter()
        .map(|candidate| {
            (
                MapperImageId {
                    frame_id: candidate["time_cam_id"]["frame_id"].as_u64().unwrap(),
                    cam_id: candidate["time_cam_id"]["cam_id"].as_u64().unwrap() as u8,
                },
                candidate["score"].as_f64().unwrap(),
            )
        })
        .collect()
}

fn temporal_image_id(value: &Value, field: &str) -> MapperImageId {
    MapperImageId {
        frame_id: value[field]["frame_id"].as_u64().unwrap(),
        cam_id: value[field]["cam_id"].as_u64().unwrap() as u8,
    }
}

fn expected_inlier_set(value: &Value, field: &str) -> Vec<(u64, u64)> {
    let mut result = value[field]
        .as_array()
        .unwrap()
        .iter()
        .map(|pair| (pair[0].as_u64().unwrap(), pair[1].as_u64().unwrap()))
        .collect::<Vec<_>>();
    result.sort_unstable();
    result
}

fn expected_model_matrix(value: &Value, field: &str) -> [[f64; 3]; 3] {
    let rows = value[field].as_array().unwrap();
    let mut result = [[0.0; 3]; 3];
    for row in 0..3 {
        for column in 0..3 {
            result[row][column] = rows[row][column].as_f64().unwrap();
        }
    }
    result
}

fn expected_model_vector(value: &Value, field: &str) -> [f64; 3] {
    let values = value[field].as_array().unwrap();
    [
        values[0].as_f64().unwrap(),
        values[1].as_f64().unwrap(),
        values[2].as_f64().unwrap(),
    ]
}

#[test]
#[ignore = "requires explicit MH_01_easy, calibration, and config paths"]
fn mh01_m8c_feature_frontend_matches_upstream_oracle() {
    let diagnostics = diagnostics_enabled();
    let root = required_path(
        "VISLOC_BASALT_MH01_ROOT",
        "the external MH_01_easy sensor root",
    );
    let calibration_path = required_path(
        "VISLOC_BASALT_CALIBRATION",
        "the pinned euroc_ds_calib.json",
    );
    let config_path = required_path("VISLOC_BASALT_CONFIG", "the pinned euroc_config.json");
    let dataset = EurocSensorDataset::open(root, &calibration_path, &config_path).unwrap();
    let calibration = BasaltCalibration::from_path(&calibration_path).unwrap();
    let config = mapper_config(&dataset);
    let frame = dataset.frame(0).unwrap();
    let left =
        extract_mapper_features(&frame.cam0, calibration.camera(0).unwrap(), config).unwrap();
    let right_image = frame.cam1.as_ref().unwrap();
    let right =
        extract_mapper_features(right_image, calibration.camera(1).unwrap(), config).unwrap();

    let oracle: Value = serde_json::from_str(FIXTURE).unwrap();
    let ransac_oracle: Value = serde_json::from_str(RANSAC_FIXTURE).unwrap();
    let oracle_feature = &oracle["features"][0];
    let (expected_corners, expected_angles, expected_descriptors, expected_rays, expected_hashes) =
        expected_features(oracle_feature);

    let (corner_error, corner_index) = max_point_error(&left.corners, &expected_corners);
    let (angle_error, angle_index) = max_scalar_error(&left.corner_angles, &expected_angles);
    let (ray_error, ray_index) = max_ray_error(&left.rays, &expected_rays);
    let descriptor_row_mismatches = left
        .descriptors
        .iter()
        .zip(&expected_descriptors)
        .filter(|(actual, expected)| actual != expected)
        .count();
    let descriptor_byte_mismatches = left
        .descriptors
        .iter()
        .zip(&expected_descriptors)
        .flat_map(|(actual, expected)| actual.iter().zip(expected))
        .filter(|(actual, expected)| actual != expected)
        .count();
    let hash_mismatches = left
        .hashes
        .iter()
        .zip(&expected_hashes)
        .filter(|(actual, expected)| actual != expected)
        .count();
    let actual_corner_set = left
        .corners
        .iter()
        .map(|point| (point.x as i64, point.y as i64))
        .collect::<BTreeSet<_>>();
    let expected_corner_set = expected_corners
        .iter()
        .map(|point| (point.x as i64, point.y as i64))
        .collect::<BTreeSet<_>>();
    let missing_corners = expected_corner_set
        .difference(&actual_corner_set)
        .copied()
        .collect::<Vec<_>>();
    let extra_corners = actual_corner_set
        .difference(&expected_corner_set)
        .copied()
        .collect::<Vec<_>>();

    if diagnostics {
        eprintln!(
            "M8c before/after diagnostic: corners rust={} oracle={} max_xy={corner_error:.17e}@{corner_index}; angles max_abs={angle_error:.17e}@{angle_index}; descriptors rows={} bytes={}; rays max_abs={ray_error:.17e}@{ray_index}; hashes mismatches={hash_mismatches}",
            left.corners.len(),
            expected_corners.len(),
            descriptor_row_mismatches,
            descriptor_byte_mismatches,
        );
        eprintln!("M8c corner set delta: missing={missing_corners:?}; extra={extra_corners:?}");
    }

    let transform = calibration
        .imu_to_camera(0)
        .unwrap()
        .compose(calibration.camera_to_imu(1).unwrap());
    let stereo = match_stereo_features(&left, &right, &transform, config).unwrap();
    let oracle_pair = &oracle["stereo"]["pairs"][0];
    let expected_raw = oracle_pair["raw_mutual_hamming"].as_array().unwrap().len();
    let expected_inliers = oracle_pair["essential_inlier_ids"]
        .as_array()
        .unwrap()
        .len();
    if diagnostics {
        eprintln!(
            "M8c stereo before/after diagnostic: raw rust={} oracle={expected_raw}; inliers rust={} oracle={expected_inliers}; stored={}",
            stereo.raw_matches.len(),
            stereo.essential_inliers.len(),
            stereo.mapper_feature_matches_stored,
        );
    }

    if env::var_os("M8C_FEATURE_PARITY_MEASURE").is_some() {
        return;
    }

    assert_eq!(left.corners, expected_corners);
    assert_eq!(left.corner_angles.len(), expected_angles.len());
    for (index, (actual, expected)) in left.corner_angles.iter().zip(&expected_angles).enumerate() {
        assert!(
            (actual - expected).abs() <= 1e-14,
            "corner angle {index}: actual={actual:.17e} expected={expected:.17e}"
        );
    }
    assert_eq!(left.descriptors, expected_descriptors);
    assert_eq!(left.rays.len(), expected_rays.len());
    for (index, (actual, expected)) in left.rays.iter().zip(&expected_rays).enumerate() {
        let error = actual
            .iter()
            .zip(expected)
            .map(|(actual, expected)| (actual - expected).abs())
            .fold(0.0, f64::max);
        assert!(
            error <= 1e-14,
            "ray {index}: max_abs={error:.17e} actual={actual:?} expected={expected:?}"
        );
    }
    assert_eq!(left.hashes, expected_hashes);
    let (expected_raw_matches, expected_inlier_ids, expected_stored) =
        expected_stereo_pair(oracle_pair);
    assert_eq!(stereo.raw_matches, expected_raw_matches);
    assert_eq!(stereo.essential_inliers, expected_inlier_ids);
    assert_eq!(stereo.mapper_feature_matches_stored, expected_stored);

    // Re-run all 20 selected TimeCamIds from the external EuRoC images.  The
    // bounded fixture keeps canonical full-array hashes and full hashes/BoW
    // for every image, so these checks are independent of the first-image-only
    // raw descriptor payload.
    let mut generated = BTreeMap::<(u64, u64), MapperImageFeatures>::new();
    for feature in oracle["features"].as_array().unwrap() {
        let frame_id = feature["time_cam_id"]["frame_id"].as_u64().unwrap();
        let cam_id = feature["time_cam_id"]["cam_id"].as_u64().unwrap();
        let frame_index = dataset
            .cam0_images()
            .iter()
            .position(|entry| entry.timestamp_ns as u64 == frame_id)
            .unwrap();
        let selected_frame = dataset.frame(frame_index).unwrap();
        let image = if cam_id == 0 {
            &selected_frame.cam0
        } else {
            selected_frame.cam1.as_ref().unwrap()
        };
        let features =
            extract_mapper_features(image, calibration.camera(cam_id as u16).unwrap(), config)
                .unwrap();
        assert_feature_summary(&features, feature, &format!("TCID {frame_id}/{cam_id}"));
        generated.insert((frame_id, cam_id), features);
    }

    let database = generated
        .iter()
        .map(|(&(frame_id, cam_id), features)| {
            (
                MapperImageId {
                    frame_id,
                    cam_id: cam_id as u8,
                },
                features,
            )
        })
        .collect::<Vec<_>>();
    for query in oracle["bow_query"]["queries"].as_array().unwrap() {
        let query_id = temporal_image_id(query, "time_cam_id");
        let query_features = generated
            .get(&(query_id.frame_id, u64::from(query_id.cam_id)))
            .unwrap();
        let actual = query_bow_candidates(
            query_id,
            query_features,
            &database,
            config.match_window as usize,
        );
        let expected = expected_temporal_candidates(query);
        assert_eq!(
            actual.len(),
            expected.len(),
            "BoW query {:?} length",
            query_id
        );
        for (index, (actual, (expected_id, expected_score))) in
            actual.iter().zip(expected).enumerate()
        {
            assert_eq!(
                actual.image, expected_id,
                "BoW query {:?} ID {index}",
                query_id
            );
            assert!(
                (actual.score - expected_score).abs() <= 1e-12,
                "BoW query {:?} score {index}: actual={:.17e} expected={:.17e}",
                query_id,
                actual.score,
                expected_score
            );
        }
    }

    // The raw temporal matcher and strict pre-RANSAC gate are fully
    // reproducible in Rust.  OpenGV's randomized model/inlier set is retained
    // as a fixture SHA and reported as the explicit remaining port boundary.
    for temporal in oracle["bow_query"]["temporal_matches"].as_array().unwrap() {
        let left_id = temporal_image_id(temporal, "left");
        let right_id = temporal_image_id(temporal, "right");
        let left_features = generated
            .get(&(left_id.frame_id, u64::from(left_id.cam_id)))
            .unwrap();
        let right_features = generated
            .get(&(right_id.frame_id, u64::from(right_id.cam_id)))
            .unwrap();
        let actual = match_temporal_stage(left_features, right_features, config);
        assert_eq!(
            actual.raw_matches.len(),
            temporal["raw_match_count"].as_u64().unwrap() as usize,
            "temporal {:?}->{:?} raw count",
            left_id,
            right_id
        );
        assert_eq!(
            temporal_raw_digest(&actual.raw_matches),
            temporal["raw_mutual_hamming_sha256"].as_str().unwrap(),
            "temporal {:?}->{:?} raw canonical SHA",
            left_id,
            right_id
        );
        assert_eq!(
            actual.ransac_attempted,
            temporal["ransac_attempted"].as_bool().unwrap(),
            "temporal {:?}->{:?} RANSAC gate",
            left_id,
            right_id
        );
        assert_eq!(
            temporal["reject_stage"].as_str().unwrap(),
            if actual.raw_match_gate_passed {
                "accepted"
            } else {
                "raw_match_gate"
            },
            "temporal {:?}->{:?} reject stage",
            left_id,
            right_id
        );
    }

    // Fixed-seed OpenGV hook: this exercises the literal STEWENIUS adapter,
    // sample-size-8 RANSAC, reprojection threshold/selectWithinDistance, and
    // Cayley refinement.  The default production API remains time-seeded;
    // only this explicit seed path is used for deterministic golden checks.
    let only_seed = env::var("M8C_FEATURE_PARITY_ONLY_SEED")
        .ok()
        .and_then(|value| value.parse::<u32>().ok());
    let only_cam_pair = env::var("M8C_FEATURE_PARITY_ONLY_CAM_PAIR").ok();
    for seeded in ransac_oracle["bow_query"]["seeded_temporal_oracle"]
        .as_array()
        .unwrap()
    {
        let left_id = temporal_image_id(seeded, "left");
        let right_id = temporal_image_id(seeded, "right");
        if only_seed.is_some_and(|seed| seeded["seed"].as_u64() != Some(u64::from(seed))) {
            continue;
        }
        if only_cam_pair
            .as_deref()
            .is_some_and(|pair| pair != format!("{}->{}", left_id.cam_id, right_id.cam_id))
        {
            continue;
        }
        let left_features = generated
            .get(&(left_id.frame_id, u64::from(left_id.cam_id)))
            .unwrap();
        let right_features = generated
            .get(&(right_id.frame_id, u64::from(right_id.cam_id)))
            .unwrap();
        let actual = match_temporal_ransac_seeded(
            left_features,
            right_features,
            config,
            seeded["seed"].as_u64().unwrap() as u32,
        );
        if diagnostics {
            let mut actual_ransac = actual.ransac_inlier_ids.clone();
            actual_ransac.sort_unstable();
            let mut actual_refined = actual.refined_inlier_ids.clone();
            actual_refined.sort_unstable();
            let expected_ransac = expected_inlier_set(seeded, "ransac_inlier_ids");
            let expected_refined = expected_inlier_set(seeded, "refined_inlier_ids");
            let missing_ransac = expected_ransac
                .iter()
                .filter(|id| !actual_ransac.contains(id))
                .copied()
                .collect::<Vec<_>>();
            let extra_ransac = actual_ransac
                .iter()
                .filter(|id| !expected_ransac.contains(id))
                .copied()
                .collect::<Vec<_>>();
            let missing_refined = expected_refined
                .iter()
                .filter(|id| !actual_refined.contains(id))
                .copied()
                .collect::<Vec<_>>();
            let extra_refined = actual_refined
                .iter()
                .filter(|id| !expected_refined.contains(id))
                .copied()
                .collect::<Vec<_>>();
            let expected_ransac_rotation = expected_model_matrix(seeded, "ransac_model_rotation");
            let expected_ransac_translation =
                expected_model_vector(seeded, "ransac_model_translation");
            let expected_refined_rotation = expected_model_matrix(seeded, "refined_model_rotation");
            let expected_refined_translation =
                expected_model_vector(seeded, "refined_model_translation");
            let ransac_model_error = (0..3)
                .flat_map(|row| (0..3).map(move |column| (row, column)))
                .map(|(row, column)| {
                    (actual.ransac_model_rotation[row][column]
                        - expected_ransac_rotation[row][column])
                        .abs()
                })
                .chain((0..3).map(|row| {
                    (actual.ransac_model_translation[row] - expected_ransac_translation[row]).abs()
                }))
                .fold(0.0, f64::max);
            let refined_model_error = (0..3)
                .flat_map(|row| (0..3).map(move |column| (row, column)))
                .map(|(row, column)| {
                    (actual.refined_model_rotation[row][column]
                        - expected_refined_rotation[row][column])
                        .abs()
                })
                .chain((0..3).map(|row| {
                    (actual.refined_model_translation[row] - expected_refined_translation[row])
                        .abs()
                }))
                .fold(0.0, f64::max);
            eprintln!(
                "AUDIT {:?}->{:?} seed={} iter={} found={} accepted={} ransac={}/{} refined={}/{} model_err_ransac={:.17e} model_err_refined={:.17e}",
                left_id,
                right_id,
                seeded["seed"],
                actual.ransac_iterations,
                actual.model_found,
                actual.accepted,
                actual.ransac_inlier_ids.len(),
                expected_ransac.len(),
                actual.refined_inlier_ids.len(),
                expected_refined.len(),
                ransac_model_error,
                refined_model_error,
            );
            eprintln!(
                "  actual model ransac_t={:?} refined_t={:?} ransac_r00={:.17e} refined_r00={:.17e}",
                actual.ransac_model_translation,
                actual.refined_model_translation,
                actual.ransac_model_rotation[0][0],
                actual.refined_model_rotation[0][0],
            );
            eprintln!("  ransac missing={missing_ransac:?} extra={extra_ransac:?}");
            eprintln!("  refined missing={missing_refined:?} extra={extra_refined:?}");
            if env::var_os("M8C_FEATURE_PARITY_ONE").is_some() {
                break;
            }
            continue;
        }
        if diagnostics {
            eprintln!(
                "seeded {:?}->{:?} seed={} actual iter={} ransac={} refined={} found={} accepted={}",
                left_id,
                right_id,
                seeded["seed"],
                actual.ransac_iterations,
                actual.ransac_inlier_ids.len(),
                actual.refined_inlier_ids.len(),
                actual.model_found,
                actual.accepted
            );
        }
        assert_eq!(
            actual.ransac_iterations,
            seeded["ransac_iterations"].as_u64().unwrap() as usize,
            "seeded {:?}->{:?} iterations",
            left_id,
            right_id
        );
        assert_eq!(actual.model_found, seeded["model_found"].as_bool().unwrap());
        assert_eq!(actual.accepted, seeded["accepted"].as_bool().unwrap());
        assert_eq!(
            actual.ransac_inlier_ids.len(),
            seeded["ransac_inlier_count"].as_u64().unwrap() as usize,
            "seeded {:?}->{:?} RANSAC inlier count",
            left_id,
            right_id
        );
        assert_eq!(
            actual.refined_inlier_ids.len(),
            seeded["refined_inlier_count"].as_u64().unwrap() as usize,
            "seeded {:?}->{:?} refined inlier count",
            left_id,
            right_id
        );
        let mut actual_ransac = actual.ransac_inlier_ids.clone();
        actual_ransac.sort_unstable();
        assert_eq!(
            actual_ransac,
            expected_inlier_set(seeded, "ransac_inlier_ids"),
            "seeded {:?}->{:?} RANSAC inlier set",
            left_id,
            right_id
        );
        let mut actual_refined = actual.refined_inlier_ids.clone();
        actual_refined.sort_unstable();
        assert_eq!(
            actual_refined,
            expected_inlier_set(seeded, "refined_inlier_ids"),
            "seeded {:?}->{:?} refined inlier set",
            left_id,
            right_id
        );
        let expected_ransac_rotation = expected_model_matrix(seeded, "ransac_model_rotation");
        let expected_ransac_translation = expected_model_vector(seeded, "ransac_model_translation");
        let expected_refined_rotation = expected_model_matrix(seeded, "refined_model_rotation");
        let expected_refined_translation =
            expected_model_vector(seeded, "refined_model_translation");
        let ransac_model_error = (0..3)
            .flat_map(|row| (0..3).map(move |column| (row, column)))
            .map(|(row, column)| {
                (actual.ransac_model_rotation[row][column] - expected_ransac_rotation[row][column])
                    .abs()
            })
            .chain((0..3).map(|row| {
                (actual.ransac_model_translation[row] - expected_ransac_translation[row]).abs()
            }))
            .fold(0.0, f64::max);
        let refined_model_error = (0..3)
            .flat_map(|row| (0..3).map(move |column| (row, column)))
            .map(|(row, column)| {
                (actual.refined_model_rotation[row][column]
                    - expected_refined_rotation[row][column])
                    .abs()
            })
            .chain((0..3).map(|row| {
                (actual.refined_model_translation[row] - expected_refined_translation[row]).abs()
            }))
            .fold(0.0, f64::max);
        if diagnostics {
            eprintln!("model max errors ransac={ransac_model_error:.6e} refined={refined_model_error:.6e}");
            eprintln!(
                "actual refined R={:?} t={:?}",
                actual.refined_model_rotation, actual.refined_model_translation
            );
            eprintln!(
                "expected refined R={:?} t={:?}",
                expected_refined_rotation, expected_refined_translation
            );
            eprintln!(
                "actual ransac t={:?} expected ransac t={:?}",
                actual.ransac_model_translation, expected_ransac_translation
            );
        }
        let actual_ransac_rotation = actual.ransac_model_rotation;
        let actual_ransac_translation = actual.ransac_model_translation;
        let actual_refined_rotation = actual.refined_model_rotation;
        let actual_refined_translation = actual.refined_model_translation;
        for row in 0..3 {
            for column in 0..3 {
                assert!(
                    (actual_ransac_rotation[row][column] - expected_ransac_rotation[row][column])
                        .abs()
                        <= 1e-3,
                    "seeded {:?}->{:?} RANSAC model [{row},{column}]",
                    left_id,
                    right_id
                );
            }
            assert!(
                (actual_ransac_translation[row] - expected_ransac_translation[row]).abs() <= 1e-5,
                "seeded {:?}->{:?} RANSAC translation [{row}]",
                left_id,
                right_id
            );
        }
        // The result model is the post-refinement model; keep a separate
        // tolerance block so a future exact LM port can tighten this gate.
        for row in 0..3 {
            for column in 0..3 {
                assert!(
                    (actual_refined_rotation[row][column] - expected_refined_rotation[row][column])
                        .abs()
                        <= 1e-3,
                    "seeded {:?}->{:?} refined model [{row},{column}]",
                    left_id,
                    right_id
                );
            }
            assert!(
                (actual_refined_translation[row] - expected_refined_translation[row]).abs() <= 1e-3,
                "seeded {:?}->{:?} refined translation [{row}]",
                left_id,
                right_id
            );
        }
    }

    let transform = calibration
        .imu_to_camera(0)
        .unwrap()
        .compose(calibration.camera_to_imu(1).unwrap());
    for pair in oracle["stereo"]["pairs"].as_array().unwrap() {
        let left_id = (
            pair["left"]["frame_id"].as_u64().unwrap(),
            pair["left"]["cam_id"].as_u64().unwrap(),
        );
        let right_id = (
            pair["right"]["frame_id"].as_u64().unwrap(),
            pair["right"]["cam_id"].as_u64().unwrap(),
        );
        let stereo = match_stereo_features(
            generated.get(&left_id).unwrap(),
            generated.get(&right_id).unwrap(),
            &transform,
            config,
        )
        .unwrap();
        let (expected_raw_matches, expected_inlier_ids, expected_stored) =
            expected_stereo_pair(pair);
        assert_eq!(
            stereo.raw_matches, expected_raw_matches,
            "stereo pair {left_id:?}"
        );
        assert_eq!(
            stereo.essential_inliers, expected_inlier_ids,
            "stereo pair {left_id:?}"
        );
        assert_eq!(stereo.mapper_feature_matches_stored, expected_stored);
    }
}
