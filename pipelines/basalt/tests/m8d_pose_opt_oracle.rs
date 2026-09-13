//! Optional end-to-end M8d setup parity against the pinned C++ oracle.
//!
//! The normal M8d track gate is portable and runs in `m8d_track_oracle.rs`.
//! This test is ignored because the setup oracle needs an upstream
//! Basalt/Sophus/Eigen executable and a pose-bearing `M8DOPT1` stream.  Set
//! `VISLOC_BASALT_M8D_SETUP_ORACLE`, `VISLOC_BASALT_CALIBRATION`,
//! `VISLOC_BASALT_CONFIG`, and `VISLOC_BASALT_M8D_SETUP_INPUT` to run it.

use std::{
    collections::{BTreeMap, HashMap},
    env, fs,
    path::PathBuf,
    process::Command,
};

use nalgebra::{Point2, Quaternion, UnitQuaternion, Vector3};
use serde_json::Value;
use visloc_basalt::{
    config::BasaltConfig,
    mapper::{
        setup_opt, FeatureTracks, MapperImageFeatures, SetupOptInput, SetupOptRejectReason,
        TimeCamId,
    },
    BasaltCalibration,
};
use visloc_core::geometry::SE3;

const MAGIC: &[u8] = b"M8DOPT1\n";

struct Cursor {
    bytes: Vec<u8>,
    offset: usize,
}

impl Cursor {
    fn new(path: &PathBuf) -> Self {
        Self {
            bytes: fs::read(path).expect("M8DOPT1 input"),
            offset: 0,
        }
    }

    fn take<const N: usize>(&mut self) -> [u8; N] {
        let end = self.offset + N;
        let bytes = self.bytes[self.offset..end]
            .try_into()
            .expect("M8DOPT1 truncated record");
        self.offset = end;
        bytes
    }

    fn u64(&mut self) -> u64 {
        u64::from_le_bytes(self.take())
    }

    fn i64(&mut self) -> i64 {
        i64::from_le_bytes(self.take())
    }

    fn f64(&mut self) -> f64 {
        f64::from_le_bytes(self.take())
    }
}

fn parse_input(
    path: &PathBuf,
) -> (
    BTreeMap<TimeCamId, MapperImageFeatures>,
    BTreeMap<u64, SE3>,
    FeatureTracks,
) {
    let mut cursor = Cursor::new(path);
    assert_eq!(&cursor.bytes[..MAGIC.len()], MAGIC);
    cursor.offset = MAGIC.len();
    let mut features = BTreeMap::new();
    for _ in 0..cursor.u64() {
        let image = TimeCamId::new(cursor.i64() as u64, cursor.u64() as u16);
        let mut corners = Vec::new();
        for _ in 0..cursor.u64() {
            corners.push(Point2::new(cursor.f64(), cursor.f64()));
        }
        features.insert(
            image,
            MapperImageFeatures {
                corners,
                corner_angles: Vec::new(),
                descriptors: Vec::new(),
                rays: Vec::new(),
                hashes: Vec::new(),
                bow_vector: Vec::new(),
            },
        );
    }
    let mut poses = BTreeMap::new();
    for _ in 0..cursor.u64() {
        let frame_id = cursor.i64() as u64;
        let translation = Vector3::new(cursor.f64(), cursor.f64(), cursor.f64());
        let xyzw = [cursor.f64(), cursor.f64(), cursor.f64(), cursor.f64()];
        poses.insert(
            frame_id,
            SE3::new(
                UnitQuaternion::new_normalize(Quaternion::new(xyzw[3], xyzw[0], xyzw[1], xyzw[2])),
                translation,
            ),
        );
    }
    let mut tracks = FeatureTracks::new();
    for _ in 0..cursor.u64() {
        let track_id = cursor.u64();
        let mut track = BTreeMap::new();
        for _ in 0..cursor.u64() {
            track.insert(
                TimeCamId::new(cursor.i64() as u64, cursor.u64() as u16),
                cursor.u64(),
            );
        }
        tracks.insert(track_id, track);
    }
    (features, poses, tracks)
}

fn required_path(name: &str, description: &str) -> PathBuf {
    env::var_os(name)
        .map(PathBuf::from)
        .unwrap_or_else(|| panic!("set {name} to {description} for this ignored gate"))
}

fn expected_rejection_name(reason: SetupOptRejectReason) -> &'static str {
    match reason {
        SetupOptRejectReason::InvalidMinimumDistance => "invalid_minimum_distance",
        SetupOptRejectReason::TrackTooShort => "track_too_short",
        SetupOptRejectReason::MissingFeatureImage => "missing_feature_image",
        SetupOptRejectReason::FeatureIndexOutOfRange => "feature_index_out_of_range",
        SetupOptRejectReason::MissingFramePose => "missing_frame_pose",
        SetupOptRejectReason::MissingCameraCalibration => "missing_camera_calibration",
        SetupOptRejectReason::BaselineTooSmall => "baseline_too_small",
        SetupOptRejectReason::NonFiniteBaseline => "nonfinite_baseline",
        SetupOptRejectReason::TriangulationNonFinite => "triangulation_nonfinite",
        SetupOptRejectReason::InverseDistanceNonFinite => "inverse_distance_nonfinite",
        SetupOptRejectReason::InverseDistanceNonPositive => "inverse_distance_nonpositive",
        SetupOptRejectReason::InverseDistanceTooLarge => "inverse_distance_too_large",
        SetupOptRejectReason::UnprojectionFailed => "unprojection_failed",
        SetupOptRejectReason::StereographicProjectionFailed => "stereographic_projection_failed",
    }
}

const ALL_REJECTION_REASONS: &[SetupOptRejectReason] = &[
    SetupOptRejectReason::InvalidMinimumDistance,
    SetupOptRejectReason::TrackTooShort,
    SetupOptRejectReason::MissingFeatureImage,
    SetupOptRejectReason::FeatureIndexOutOfRange,
    SetupOptRejectReason::MissingFramePose,
    SetupOptRejectReason::MissingCameraCalibration,
    SetupOptRejectReason::UnprojectionFailed,
    SetupOptRejectReason::NonFiniteBaseline,
    SetupOptRejectReason::BaselineTooSmall,
    SetupOptRejectReason::TriangulationNonFinite,
    SetupOptRejectReason::InverseDistanceNonFinite,
    SetupOptRejectReason::InverseDistanceNonPositive,
    SetupOptRejectReason::InverseDistanceTooLarge,
    SetupOptRejectReason::StereographicProjectionFailed,
];

#[test]
#[ignore = "requires explicit oracle, calibration, config, and input paths"]
fn m8d_setup_opt_matches_pinned_cpp_oracle() {
    let oracle = required_path(
        "VISLOC_BASALT_M8D_SETUP_ORACLE",
        "the pinned upstream C++ setup_opt executable",
    );
    let calibration_path = required_path(
        "VISLOC_BASALT_CALIBRATION",
        "the pinned euroc_ds_calib.json",
    );
    let config_path = required_path("VISLOC_BASALT_CONFIG", "the pinned euroc_config.json");
    let input_path = required_path(
        "VISLOC_BASALT_M8D_SETUP_INPUT",
        "the pose-bearing M8DOPT1 input stream",
    );
    let output_path = env::temp_dir().join(format!(
        "visloc-basalt-m8d-setup-{}.json",
        std::process::id()
    ));
    let status = Command::new(&oracle)
        .args([
            calibration_path.as_os_str(),
            config_path.as_os_str(),
            input_path.as_os_str(),
            output_path.as_os_str(),
        ])
        .status()
        .expect("run pinned C++ setup_opt oracle");
    assert!(status.success(), "C++ setup_opt oracle failed: {status}");

    let expected: Value =
        serde_json::from_str(&fs::read_to_string(&output_path).expect("C++ oracle output"))
            .expect("C++ oracle JSON");
    let (features, poses, tracks) = parse_input(&input_path);
    let calibration = BasaltCalibration::from_path(&calibration_path).expect("calibration");
    let config = BasaltConfig::from_json(&fs::read_to_string(&config_path).expect("config"))
        .expect("config schema");
    let min_distance = config
        .value::<f64>("config.mapper_min_triangulation_dist")
        .expect("mapper baseline");
    let actual = setup_opt(SetupOptInput::new(
        &tracks,
        &features,
        &poses,
        &calibration,
        min_distance,
    ));

    for (actual_value, field) in [
        (actual.report.input_track_count, "input_track_count"),
        (actual.report.attempted_track_count, "attempted_track_count"),
        (actual.report.accepted_track_count, "accepted_track_count"),
        (
            actual.report.skipped_short_track_count,
            "skipped_short_track_count",
        ),
        (
            actual.report.candidate_attempt_count,
            "candidate_attempt_count",
        ),
        (actual.report.observation_count, "observation_count"),
    ] {
        assert_eq!(
            actual_value,
            expected[field].as_u64().expect(field) as usize,
            "setup_opt {field}"
        );
    }
    let expected_counts = expected["rejection_counts"]
        .as_object()
        .expect("rejection counts");
    for reason in ALL_REJECTION_REASONS {
        assert_eq!(
            expected_counts
                .get(expected_rejection_name(*reason))
                .and_then(Value::as_u64)
                .unwrap_or_default() as usize,
            actual
                .report
                .rejection_counts
                .get(reason)
                .copied()
                .unwrap_or_default(),
            "rejection count for {reason:?}"
        );
    }

    let mut expected_tracks = HashMap::new();
    for track in expected["tracks"].as_array().expect("track details") {
        expected_tracks.insert(track["track_id"].as_u64().expect("track id"), track);
    }
    for (&track_id, landmark) in &actual.landmarks {
        let expected_track = expected_tracks.get(&track_id).expect("accepted track");
        let host = &expected_track["host"];
        assert_eq!(
            host["frame_id"].as_i64().unwrap() as u64,
            landmark.host.frame_id
        );
        assert_eq!(
            host["cam_id"].as_u64().unwrap() as u16,
            landmark.host.cam_id
        );
        let second = &expected_track["second"];
        assert_eq!(
            second["frame_id"].as_i64().unwrap() as u64,
            landmark.second.frame_id
        );
        assert_eq!(
            second["cam_id"].as_u64().unwrap() as u16,
            landmark.second.cam_id
        );
        let direction = expected_track["direction"].as_array().unwrap();
        assert!((direction[0].as_f64().unwrap() - landmark.direction.xy.x).abs() < 1e-10);
        assert!((direction[1].as_f64().unwrap() - landmark.direction.xy.y).abs() < 1e-10);
        assert!(
            (expected_track["inverse_distance"].as_f64().unwrap() - landmark.inverse_distance)
                .abs()
                < 1e-10
        );
    }
    assert_eq!(
        actual.report.canonical_hash,
        expected["canonical_hash"].as_u64().expect("canonical hash")
    );
    let _ = fs::remove_file(output_path);
}
