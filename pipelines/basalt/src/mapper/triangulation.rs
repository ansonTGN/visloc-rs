//! Basalt NFR mapper landmark setup.
//!
//! This module is the literal Rust boundary for the pinned
//! `NfrMapper::setup_opt()` implementation.  It intentionally consumes the
//! products of the M8d track builder and does not perform matching, feature
//! filtering, or bundle adjustment.  In particular, a track's first ordered
//! [`TimeCamId`] is always its host and the first later observation that
//! passes Basalt's baseline/triangulation gates wins.

use super::{FeatureId, FeatureTracks, TimeCamId};
use crate::{calibration::BasaltCalibration, mapper::features::MapperImageFeatures};
use nalgebra::{Matrix4, Point2, Vector3, Vector4};
use std::collections::BTreeMap;
use visloc_core::geometry::SE3;

use crate::vio::landmarks::StereographicDirection;

/// Inputs owned by one `NfrMapper::setup_opt()` call.
///
/// `frame_poses` are `T_w_i` poses keyed by the same timestamp/frame ID used
/// by [`TimeCamId`].  The calibration transform is `T_i_c`, i.e. camera to
/// IMU, so the camera pose used below is exactly
/// `T_w_c = T_w_i * T_i_c`.
#[derive(Debug, Clone, Copy)]
pub struct SetupOptInput<'a> {
    pub tracks: &'a FeatureTracks,
    pub feature_corners: &'a BTreeMap<TimeCamId, MapperImageFeatures>,
    pub frame_poses: &'a BTreeMap<u64, SE3>,
    pub calibration: &'a BasaltCalibration,
    pub min_triangulation_distance: f64,
}

impl<'a> SetupOptInput<'a> {
    pub const fn new(
        tracks: &'a FeatureTracks,
        feature_corners: &'a BTreeMap<TimeCamId, MapperImageFeatures>,
        frame_poses: &'a BTreeMap<u64, SE3>,
        calibration: &'a BasaltCalibration,
        min_triangulation_distance: f64,
    ) -> Self {
        Self {
            tracks,
            feature_corners,
            frame_poses,
            calibration,
            min_triangulation_distance,
        }
    }
}

/// One observation retained in a mapper landmark.
///
/// Keeping the feature index as well as its pixel is important: upstream's
/// `LandmarkDatabase` receives the complete `FeatureTrack`, and silently
/// reducing this to a frame-only bearing would lose camera-stream identity.
#[derive(Debug, Clone, PartialEq)]
pub struct MapperObservation {
    pub image: TimeCamId,
    pub feature_id: FeatureId,
    pub pixel: Point2<f64>,
}

/// A landmark initialized from one exported upstream track.
#[derive(Debug, Clone, PartialEq)]
pub struct MapperLandmark {
    pub track_id: u64,
    pub host: TimeCamId,
    pub second: TimeCamId,
    pub direction: StereographicDirection,
    pub inverse_distance: f64,
    pub observations: Vec<MapperObservation>,
}

impl MapperLandmark {
    pub fn position_in_host(&self) -> Option<Vector3<f64>> {
        if !self.inverse_distance.is_finite() || self.inverse_distance <= 0.0 {
            return None;
        }
        Some(self.direction.bearing() / self.inverse_distance)
    }
}

/// Why one candidate pair or track was not inserted.
///
/// The first eight variants are malformed-input/configuration guards.  The
/// remaining variants correspond directly to the gates in upstream
/// `setup_opt()`, and are kept separately in [`SetupOptReport`] so an oracle
/// mismatch can identify the first rejected candidate rather than only a
/// final landmark count.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum SetupOptRejectReason {
    InvalidMinimumDistance,
    TrackTooShort,
    MissingFeatureImage,
    FeatureIndexOutOfRange,
    MissingFramePose,
    MissingCameraCalibration,
    UnprojectionFailed,
    NonFiniteBaseline,
    BaselineTooSmall,
    TriangulationNonFinite,
    InverseDistanceNonFinite,
    InverseDistanceNonPositive,
    InverseDistanceTooLarge,
    StereographicProjectionFailed,
}

/// One candidate-pair decision in deterministic track order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SetupOptCandidate {
    pub observation: TimeCamId,
    pub accepted: bool,
    pub rejection: Option<SetupOptRejectReason>,
    /// IEEE-754 bits for the camera baseline gate, retained only so the
    /// pinned Eigen-vs-nalgebra setup audit can compare values without a
    /// textual rounding step.  `None` means the candidate was not reached.
    pub baseline_squared_bits: Option<u64>,
    /// IEEE-754 bits for the raw homogeneous DLT point returned by
    /// `triangulate_ba`, before the inverse-distance gates and normalization.
    pub point_bits: Option<[u64; 4]>,
}

/// Per-track diagnostics emitted by [`setup_opt`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SetupOptTrackReport {
    pub track_id: u64,
    pub observation_count: usize,
    pub host: Option<TimeCamId>,
    pub chosen_second: Option<TimeCamId>,
    pub attempted: bool,
    pub accepted: bool,
    pub candidates: Vec<SetupOptCandidate>,
    /// The last track-level rejection, if the track did not become a
    /// landmark. Candidate rejection details remain in `candidates`.
    pub rejection: Option<SetupOptRejectReason>,
}

/// Aggregate setup diagnostics suitable for an oracle comparison.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SetupOptReport {
    pub input_track_count: usize,
    pub attempted_track_count: usize,
    pub accepted_track_count: usize,
    pub skipped_short_track_count: usize,
    pub observation_count: usize,
    pub candidate_attempt_count: usize,
    pub rejection_counts: BTreeMap<SetupOptRejectReason, usize>,
    pub tracks: Vec<SetupOptTrackReport>,
    pub canonical_hash: u64,
}

impl SetupOptReport {
    fn new(input_track_count: usize) -> Self {
        Self {
            input_track_count,
            attempted_track_count: 0,
            accepted_track_count: 0,
            skipped_short_track_count: 0,
            observation_count: 0,
            candidate_attempt_count: 0,
            rejection_counts: BTreeMap::new(),
            tracks: Vec::with_capacity(input_track_count),
            canonical_hash: 1469598103934665603,
        }
    }

    fn reject(&mut self, reason: SetupOptRejectReason) {
        *self.rejection_counts.entry(reason).or_default() += 1;
    }
}

/// Result of one mapper setup pass.
#[derive(Debug, Clone, PartialEq)]
pub struct SetupOptResult {
    /// Landmarks remain keyed by the unrenumbered union-find root ID.
    pub landmarks: BTreeMap<u64, MapperLandmark>,
    pub report: SetupOptReport,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum LookupError {
    MissingFeatureImage,
    FeatureIndexOutOfRange,
    MissingFramePose,
    MissingCameraCalibration,
    UnprojectionFailed,
}

fn lookup_pixel<'a>(
    feature_corners: &'a BTreeMap<TimeCamId, MapperImageFeatures>,
    image: TimeCamId,
    feature_id: FeatureId,
) -> Result<&'a Point2<f64>, LookupError> {
    let features = feature_corners
        .get(&image)
        .ok_or(LookupError::MissingFeatureImage)?;
    features
        .corners
        .get(usize::try_from(feature_id).map_err(|_| LookupError::FeatureIndexOutOfRange)?)
        .ok_or(LookupError::FeatureIndexOutOfRange)
}

fn lookup_camera_pose(
    frame_poses: &BTreeMap<u64, SE3>,
    calibration: &BasaltCalibration,
    image: TimeCamId,
) -> Result<SE3, LookupError> {
    let pose = frame_poses
        .get(&image.frame_id)
        .ok_or(LookupError::MissingFramePose)?;
    let extrinsic = calibration
        .camera_to_imu(image.cam_id)
        .ok_or(LookupError::MissingCameraCalibration)?;
    Ok(pose.compose(extrinsic))
}

fn lookup_bearing(
    feature_corners: &BTreeMap<TimeCamId, MapperImageFeatures>,
    calibration: &BasaltCalibration,
    image: TimeCamId,
    feature_id: FeatureId,
) -> Result<(Point2<f64>, Vector3<f64>), LookupError> {
    let pixel = *lookup_pixel(feature_corners, image, feature_id)?;
    let camera = calibration
        .camera(image.cam_id)
        .ok_or(LookupError::MissingCameraCalibration)?;
    // `unproject_raw` is the direct Double Sphere implementation.  Its
    // output is already unit length for valid DS pixels, just as upstream's
    // `DoubleSphereCamera::unproject` output is; unlike a generic normalized
    // ray helper it preserves the exact calibrated formula.
    let bearing = camera
        .unproject_raw(&pixel)
        .ok_or(LookupError::UnprojectionFailed)?;
    Ok((pixel, bearing))
}

const fn lookup_reason(error: LookupError) -> SetupOptRejectReason {
    match error {
        LookupError::MissingFeatureImage => SetupOptRejectReason::MissingFeatureImage,
        LookupError::FeatureIndexOutOfRange => SetupOptRejectReason::FeatureIndexOutOfRange,
        LookupError::MissingFramePose => SetupOptRejectReason::MissingFramePose,
        LookupError::MissingCameraCalibration => SetupOptRejectReason::MissingCameraCalibration,
        LookupError::UnprojectionFailed => SetupOptRejectReason::UnprojectionFailed,
    }
}

/// Faithful pinned Basalt BA triangulation.
///
/// `relative_pose` is `T_0_1`, mapping camera-1 coordinates to camera-0
/// coordinates.  The result is `[unit_direction_0, inverse_distance]` and
/// follows `BundleAdjustmentBase::triangulate` exactly: DLT/SVD, normalize by
/// the first three coordinates, and flip the complete homogeneous vector when
/// the first bearing points in the opposite direction.
pub fn triangulate_ba(
    bearing_0: Vector3<f64>,
    bearing_1: Vector3<f64>,
    relative_pose: &SE3,
) -> Option<Vector4<f64>> {
    if !bearing_0.iter().all(|value| value.is_finite())
        || !bearing_1.iter().all(|value| value.is_finite())
        || !relative_pose
            .translation
            .iter()
            .all(|value| value.is_finite())
        || relative_pose
            .rotation
            .quaternion()
            .coords
            .iter()
            .any(|value| !value.is_finite())
    {
        return None;
    }

    // P1 = [I | 0]. P2 = T_0_1^-1.matrix3x4(), matching the pinned Eigen
    // implementation.  Building the 4x4 matrix explicitly avoids accidentally
    // applying the inverse transform in the opposite direction.
    let p2 = relative_pose.inverse().matrix();
    let identity = Matrix4::<f64>::identity();
    let mut a = Matrix4::<f64>::zeros();
    for column in 0..4 {
        a[(0, column)] = bearing_0.x * identity[(2, column)] - bearing_0.z * identity[(0, column)];
        a[(1, column)] = bearing_0.y * identity[(2, column)] - bearing_0.z * identity[(1, column)];
        a[(2, column)] = bearing_1.x * p2[(2, column)] - bearing_1.z * p2[(0, column)];
        a[(3, column)] = bearing_1.y * p2[(2, column)] - bearing_1.z * p2[(1, column)];
    }

    let v_t = a.svd(false, true).v_t?;
    let mut point = Vector4::new(v_t[(3, 0)], v_t[(3, 1)], v_t[(3, 2)], v_t[(3, 3)]);
    let direction_norm = point.fixed_rows::<3>(0).norm();
    if !direction_norm.is_finite() || direction_norm <= f64::EPSILON {
        return None;
    }
    point /= direction_norm;

    // Basalt flips all four entries, not just rho, so the host direction and
    // inverse-distance sign remain a homogeneous pair.
    if bearing_0.dot(&point.fixed_rows::<3>(0)) < 0.0 {
        point = -point;
    }
    point.iter().all(|value| value.is_finite()).then_some(point)
}

const fn map_lookup_reason(error: LookupError) -> SetupOptRejectReason {
    lookup_reason(error)
}

/// Triangulate one (host, second) observation pair using exactly the gates
/// `setup_opt` applies to a candidate. Returns the landmark direction,
/// inverse distance, and the two `MapperObservation`s on success, or the same
/// `SetupOptRejectReason` `setup_opt` would have recorded.
///
/// `host` must sort before `second` in `TimeCamId` order, matching `setup_opt`'s
/// host choice. This is the building block for incremental local mapping; it
/// deliberately duplicates `setup_opt`'s gate order so an incrementally
/// triangulated landmark is indistinguishable from a batch one.
pub fn triangulate_pair(
    host: TimeCamId,
    host_feature_id: FeatureId,
    second: TimeCamId,
    second_feature_id: FeatureId,
    feature_corners: &BTreeMap<TimeCamId, MapperImageFeatures>,
    frame_poses: &BTreeMap<u64, SE3>,
    calibration: &BasaltCalibration,
    min_triangulation_distance: f64,
) -> Result<
    (
        StereographicDirection,
        f64,
        MapperObservation,
        MapperObservation,
    ),
    SetupOptRejectReason,
> {
    if !min_triangulation_distance.is_finite() || min_triangulation_distance < 0.0 {
        return Err(SetupOptRejectReason::InvalidMinimumDistance);
    }
    let min_distance2 = min_triangulation_distance * min_triangulation_distance;

    let (host_pixel, host_bearing) =
        lookup_bearing(feature_corners, calibration, host, host_feature_id)
            .map_err(map_lookup_reason)?;
    let host_pose =
        lookup_camera_pose(frame_poses, calibration, host).map_err(map_lookup_reason)?;
    let (second_pixel, second_bearing) =
        lookup_bearing(feature_corners, calibration, second, second_feature_id)
            .map_err(map_lookup_reason)?;
    let second_pose =
        lookup_camera_pose(frame_poses, calibration, second).map_err(map_lookup_reason)?;

    let relative_pose = host_pose.inverse().compose(&second_pose);
    let baseline2 = relative_pose.translation.norm_squared();
    if !baseline2.is_finite() {
        return Err(SetupOptRejectReason::NonFiniteBaseline);
    }
    if baseline2 < min_distance2 {
        return Err(SetupOptRejectReason::BaselineTooSmall);
    }

    let point = triangulate_ba(host_bearing, second_bearing, &relative_pose)
        .ok_or(SetupOptRejectReason::TriangulationNonFinite)?;
    if point.iter().any(|value| !value.is_finite()) {
        return Err(SetupOptRejectReason::TriangulationNonFinite);
    }
    let inverse_distance = point[3];
    if !inverse_distance.is_finite() {
        return Err(SetupOptRejectReason::InverseDistanceNonFinite);
    }
    if inverse_distance <= 0.0 {
        return Err(SetupOptRejectReason::InverseDistanceNonPositive);
    }
    if inverse_distance > 2.0 {
        return Err(SetupOptRejectReason::InverseDistanceTooLarge);
    }
    let direction =
        StereographicDirection::from_bearing(Vector3::new(point[0], point[1], point[2]))
            .ok_or(SetupOptRejectReason::StereographicProjectionFailed)?;

    Ok((
        direction,
        inverse_distance,
        MapperObservation {
            image: host,
            feature_id: host_feature_id,
            pixel: host_pixel,
        },
        MapperObservation {
            image: second,
            feature_id: second_feature_id,
            pixel: second_pixel,
        },
    ))
}

/// Run the pinned `NfrMapper::setup_opt()` initialization pass.
pub fn setup_opt(input: SetupOptInput<'_>) -> SetupOptResult {
    let mut report = SetupOptReport::new(input.tracks.len());
    let mut landmarks = BTreeMap::new();

    if !input.min_triangulation_distance.is_finite() || input.min_triangulation_distance < 0.0 {
        report.reject(SetupOptRejectReason::InvalidMinimumDistance);
        for (&track_id, track) in input.tracks {
            let mut track_report = SetupOptTrackReport {
                track_id,
                observation_count: track.len(),
                host: track.keys().next().copied(),
                chosen_second: None,
                attempted: false,
                accepted: false,
                candidates: Vec::new(),
                rejection: Some(SetupOptRejectReason::InvalidMinimumDistance),
            };
            track_report.rejection = Some(SetupOptRejectReason::InvalidMinimumDistance);
            report.tracks.push(track_report);
        }
        report.canonical_hash = canonical_setup_opt_hash(&landmarks);
        return SetupOptResult { landmarks, report };
    }

    let min_distance2 = input.min_triangulation_distance * input.min_triangulation_distance;
    for (&track_id, track) in input.tracks {
        let mut track_report = SetupOptTrackReport {
            track_id,
            observation_count: track.len(),
            host: track.keys().next().copied(),
            chosen_second: None,
            attempted: false,
            accepted: false,
            candidates: Vec::new(),
            rejection: None,
        };

        if track.len() < 2 {
            report.skipped_short_track_count += 1;
            report.reject(SetupOptRejectReason::TrackTooShort);
            track_report.rejection = Some(SetupOptRejectReason::TrackTooShort);
            report.tracks.push(track_report);
            continue;
        }
        report.attempted_track_count += 1;
        track_report.attempted = true;

        let (&host_image, &host_feature_id) = track.first_key_value().expect("track len >= 2");
        let host_lookup = lookup_bearing(
            input.feature_corners,
            input.calibration,
            host_image,
            host_feature_id,
        );
        let (_host_pixel, host_bearing) = match host_lookup {
            Ok(value) => value,
            Err(error) => {
                let reason = map_lookup_reason(error);
                report.reject(reason);
                track_report.rejection = Some(reason);
                report.tracks.push(track_report);
                continue;
            }
        };
        let host_pose = match lookup_camera_pose(input.frame_poses, input.calibration, host_image) {
            Ok(pose) => pose,
            Err(error) => {
                let reason = map_lookup_reason(error);
                report.reject(reason);
                track_report.rejection = Some(reason);
                report.tracks.push(track_report);
                continue;
            }
        };

        // Materialize all observations before insertion.  Upstream's
        // `feature_corners.at(...).corners[...]` requires every feature to be
        // valid; rejecting the whole track here prevents a partial landmark
        // when a malformed synthetic input omits a later observation.
        let mut observations = Vec::with_capacity(track.len());
        let mut observation_error = None;
        for (&image, &feature_id) in track {
            match lookup_pixel(input.feature_corners, image, feature_id) {
                Ok(&pixel) => observations.push(MapperObservation {
                    image,
                    feature_id,
                    pixel,
                }),
                Err(error) => {
                    observation_error = Some(map_lookup_reason(error));
                    break;
                }
            }
        }
        if let Some(reason) = observation_error {
            report.reject(reason);
            track_report.rejection = Some(reason);
            report.tracks.push(track_report);
            continue;
        }

        let mut accepted = None;
        for (&observation_image, &observation_feature_id) in track.iter().skip(1) {
            report.candidate_attempt_count += 1;
            let mut decision = SetupOptCandidate {
                observation: observation_image,
                accepted: false,
                rejection: None,
                baseline_squared_bits: None,
                point_bits: None,
            };

            let (_observation_pixel, observation_bearing) = match lookup_bearing(
                input.feature_corners,
                input.calibration,
                observation_image,
                observation_feature_id,
            ) {
                Ok(value) => value,
                Err(error) => {
                    let reason = map_lookup_reason(error);
                    report.reject(reason);
                    decision.rejection = Some(reason);
                    track_report.candidates.push(decision);
                    continue;
                }
            };
            let observation_pose =
                match lookup_camera_pose(input.frame_poses, input.calibration, observation_image) {
                    Ok(pose) => pose,
                    Err(error) => {
                        let reason = map_lookup_reason(error);
                        report.reject(reason);
                        decision.rejection = Some(reason);
                        track_report.candidates.push(decision);
                        continue;
                    }
                };

            let relative_pose = host_pose.inverse().compose(&observation_pose);
            let baseline2 = relative_pose.translation.norm_squared();
            decision.baseline_squared_bits = Some(baseline2.to_bits());
            if !baseline2.is_finite() {
                let reason = SetupOptRejectReason::NonFiniteBaseline;
                report.reject(reason);
                decision.rejection = Some(reason);
                track_report.candidates.push(decision);
                continue;
            }
            if baseline2 < min_distance2 {
                let reason = SetupOptRejectReason::BaselineTooSmall;
                report.reject(reason);
                decision.rejection = Some(reason);
                track_report.candidates.push(decision);
                continue;
            }

            let point = match triangulate_ba(host_bearing, observation_bearing, &relative_pose) {
                Some(point) => point,
                None => {
                    let reason = SetupOptRejectReason::TriangulationNonFinite;
                    report.reject(reason);
                    decision.rejection = Some(reason);
                    track_report.candidates.push(decision);
                    continue;
                }
            };
            decision.point_bits = Some([
                point[0].to_bits(),
                point[1].to_bits(),
                point[2].to_bits(),
                point[3].to_bits(),
            ]);
            if point.iter().any(|value| !value.is_finite()) {
                let reason = SetupOptRejectReason::TriangulationNonFinite;
                report.reject(reason);
                decision.rejection = Some(reason);
                track_report.candidates.push(decision);
                continue;
            }
            let inverse_distance = point[3];
            if !inverse_distance.is_finite() {
                let reason = SetupOptRejectReason::InverseDistanceNonFinite;
                report.reject(reason);
                decision.rejection = Some(reason);
                track_report.candidates.push(decision);
                continue;
            }
            if inverse_distance <= 0.0 {
                let reason = SetupOptRejectReason::InverseDistanceNonPositive;
                report.reject(reason);
                decision.rejection = Some(reason);
                track_report.candidates.push(decision);
                continue;
            }
            if inverse_distance > 2.0 {
                let reason = SetupOptRejectReason::InverseDistanceTooLarge;
                report.reject(reason);
                decision.rejection = Some(reason);
                track_report.candidates.push(decision);
                continue;
            }
            let direction = match StereographicDirection::from_bearing(Vector3::new(
                point[0], point[1], point[2],
            )) {
                Some(direction) => direction,
                None => {
                    let reason = SetupOptRejectReason::StereographicProjectionFailed;
                    report.reject(reason);
                    decision.rejection = Some(reason);
                    track_report.candidates.push(decision);
                    continue;
                }
            };

            decision.accepted = true;
            track_report.candidates.push(decision);
            accepted = Some((observation_image, direction, inverse_distance));
            break;
        }

        if let Some((second, direction, inverse_distance)) = accepted {
            track_report.chosen_second = Some(second);
            track_report.accepted = true;
            report.accepted_track_count += 1;
            report.observation_count += observations.len();
            landmarks.insert(
                track_id,
                MapperLandmark {
                    track_id,
                    host: host_image,
                    second,
                    direction,
                    inverse_distance,
                    observations,
                },
            );
        } else if track_report.rejection.is_none() {
            // Keep candidate reasons in their exact order; this marker only
            // gives callers a useful track-level value when every candidate
            // was rejected by a pair gate.
            track_report.rejection = track_report
                .candidates
                .iter()
                .rev()
                .find_map(|candidate| candidate.rejection);
        }
        report.tracks.push(track_report);
    }

    report.canonical_hash = canonical_setup_opt_hash(&landmarks);
    SetupOptResult { landmarks, report }
}

/// Convenience wrapper for callers that only retain corners rather than the
/// complete M8c feature products.
pub fn setup_opt_from_corners(
    tracks: &FeatureTracks,
    feature_corners: &BTreeMap<TimeCamId, Vec<Point2<f64>>>,
    frame_poses: &BTreeMap<u64, SE3>,
    calibration: &BasaltCalibration,
    min_triangulation_distance: f64,
) -> SetupOptResult {
    let features = feature_corners
        .iter()
        .map(|(&image, corners)| {
            (
                image,
                MapperImageFeatures {
                    corners: corners.clone(),
                    corner_angles: Vec::new(),
                    descriptors: Vec::new(),
                    rays: Vec::new(),
                    hashes: Vec::new(),
                    bow_vector: Vec::new(),
                },
            )
        })
        .collect::<BTreeMap<_, _>>();
    setup_opt(SetupOptInput::new(
        tracks,
        &features,
        frame_poses,
        calibration,
        min_triangulation_distance,
    ))
}

// Eigen and nalgebra choose the same DLT singular vector but can differ by a
// few ulps in the final inverse distance. Ten decimal places preserve the
// mapper decision/landmark contract while making the cross-language hash
// stable at those harmless SVD last bits.
const CANONICAL_FLOAT_SCALE: f64 = 1e10;

fn canonical_float(value: f64) -> i64 {
    (value * CANONICAL_FLOAT_SCALE).round() as i64
}

fn push_i64(bytes: &mut Vec<u8>, value: i64) {
    bytes.extend_from_slice(&value.to_le_bytes());
}

/// Canonical bytes for initialized mapper landmarks.
///
/// Integer fields use little-endian fixed-width values.  Floating point
/// values are rounded to 1e-10 before encoding so a C++ Eigen oracle and the
/// Rust nalgebra port compare the same semantic output despite harmless SVD
/// last-bit differences.
pub fn canonical_setup_opt_bytes(landmarks: &BTreeMap<u64, MapperLandmark>) -> Vec<u8> {
    let mut bytes = Vec::new();
    for (&track_id, landmark) in landmarks {
        bytes.extend_from_slice(&track_id.to_le_bytes());
        bytes.extend_from_slice(&landmark.host.frame_id.to_le_bytes());
        bytes.extend_from_slice(&u64::from(landmark.host.cam_id).to_le_bytes());
        bytes.extend_from_slice(&landmark.second.frame_id.to_le_bytes());
        bytes.extend_from_slice(&u64::from(landmark.second.cam_id).to_le_bytes());
        push_i64(&mut bytes, canonical_float(landmark.direction.xy.x));
        push_i64(&mut bytes, canonical_float(landmark.direction.xy.y));
        push_i64(&mut bytes, canonical_float(landmark.inverse_distance));
        bytes.extend_from_slice(&(landmark.observations.len() as u64).to_le_bytes());
        for observation in &landmark.observations {
            bytes.extend_from_slice(&observation.image.frame_id.to_le_bytes());
            bytes.extend_from_slice(&u64::from(observation.image.cam_id).to_le_bytes());
            bytes.extend_from_slice(&observation.feature_id.to_le_bytes());
            push_i64(&mut bytes, canonical_float(observation.pixel.x));
            push_i64(&mut bytes, canonical_float(observation.pixel.y));
        }
    }
    bytes
}

/// FNV-1a hash of [`canonical_setup_opt_bytes`].
pub fn canonical_setup_opt_hash(landmarks: &BTreeMap<u64, MapperLandmark>) -> u64 {
    canonical_setup_opt_bytes(landmarks)
        .into_iter()
        .fold(1469598103934665603u64, |hash, byte| {
            (hash ^ u64::from(byte)).wrapping_mul(1099511628211)
        })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::camera::DoubleSphereCamera;
    use nalgebra::{Quaternion, UnitQuaternion};

    fn calibration() -> BasaltCalibration {
        let camera = DoubleSphereCamera::new(300.0, 300.0, 320.0, 240.0, 0.5, 0.7, 640, 480)
            .expect("camera");
        BasaltCalibration {
            t_imu_cam: vec![SE3::identity(), SE3::identity()],
            cameras: vec![camera, camera],
            resolutions: vec![(640, 480), (640, 480)],
            calib_accel_bias: vec![0.0; 3],
            calib_gyro_bias: vec![0.0; 3],
            imu_update_rate_hz: 200.0,
            accel_noise_std: Vector3::repeat(1.0),
            gyro_noise_std: Vector3::repeat(1.0),
            accel_bias_std: Vector3::repeat(1.0),
            gyro_bias_std: Vector3::repeat(1.0),
            t_mocap_world: SE3::identity(),
            t_imu_marker: SE3::identity(),
            mocap_time_offset_ns: 0,
            mocap_to_imu_offset_ns: 0,
            cam_time_offset_ns: 0,
        }
    }

    fn corners(
        values: impl IntoIterator<Item = (TimeCamId, Point2<f64>)>,
    ) -> BTreeMap<TimeCamId, Vec<Point2<f64>>> {
        values
            .into_iter()
            .map(|(image, pixel)| (image, vec![pixel]))
            .collect()
    }

    fn translated_pose(x: f64) -> SE3 {
        SE3::new(
            UnitQuaternion::from_quaternion(Quaternion::identity()),
            Vector3::new(x, 0.0, 0.0),
        )
    }

    #[test]
    fn host_and_candidate_follow_time_cam_order_and_baseline_gate() {
        let c0 = TimeCamId::new(10, 0);
        let c1 = TimeCamId::new(11, 0);
        let c2 = TimeCamId::new(12, 0);
        let mut tracks = FeatureTracks::new();
        tracks.insert(42, [(c0, 0), (c1, 0), (c2, 0)].into_iter().collect());
        let image_corners = corners([
            (c0, Point2::new(320.0, 240.0)),
            (c1, Point2::new(320.0, 240.0)),
            (c2, Point2::new(320.0, 240.0)),
        ]);
        let mut poses = BTreeMap::new();
        poses.insert(10, translated_pose(0.0));
        poses.insert(11, translated_pose(0.01));
        poses.insert(12, translated_pose(0.2));
        let result = setup_opt_from_corners(&tracks, &image_corners, &poses, &calibration(), 0.07);
        let landmark = result.landmarks.get(&42).expect("accepted landmark");
        assert_eq!(landmark.host, c0);
        assert_eq!(landmark.second, c2, "first candidate is below baseline");
        assert_eq!(landmark.observations.len(), 3);
        assert_eq!(result.report.candidate_attempt_count, 2);
        assert_eq!(
            result
                .report
                .rejection_counts
                .get(&SetupOptRejectReason::BaselineTooSmall),
            Some(&1)
        );
    }

    #[test]
    fn triangulation_enforces_host_cheirality_and_inverse_distance_gate() {
        let p0 = Vector3::new(0.0, 0.0, 1.0);
        let p1 = Vector3::new(-0.1, 0.0, 1.0);
        let pose = SE3::new(UnitQuaternion::identity(), Vector3::new(0.1, 0.0, 0.0));
        let point = triangulate_ba(p0, p1, &pose).expect("finite DLT");
        assert!(point[0].is_finite() && point[1].is_finite() && point[2].is_finite());
        assert!(p0.dot(&point.fixed_rows::<3>(0)) > 0.0, "host cheirality");
        assert!(point[3] > 0.0, "the positive-depth pair has positive rho");

        // A close point has inverse distance > 2.  The setup pass must reject
        // that pair even though DLT itself returns a finite, cheiral point.
        let calibration = calibration();
        let camera = calibration.camera(0).unwrap();
        let host_pixel = camera
            .project(&nalgebra::Point3::new(0.0, 0.0, 0.25))
            .unwrap();
        let candidate_pixel = camera
            .project(&nalgebra::Point3::new(-0.1, 0.0, 0.25))
            .unwrap();
        let c0 = TimeCamId::new(0, 0);
        let c1 = TimeCamId::new(1, 0);
        let mut tracks = FeatureTracks::new();
        tracks.insert(9, [(c0, 0), (c1, 0)].into_iter().collect());
        let image_corners = corners([(c0, host_pixel), (c1, candidate_pixel)]);
        let mut poses = BTreeMap::new();
        poses.insert(0, SE3::identity());
        poses.insert(1, translated_pose(0.1));
        let result = setup_opt_from_corners(&tracks, &image_corners, &poses, &calibration, 0.05);
        assert!(result.landmarks.is_empty());
        assert_eq!(
            result
                .report
                .rejection_counts
                .get(&SetupOptRejectReason::InverseDistanceTooLarge),
            Some(&1)
        );
    }

    #[test]
    fn rank_three_dlt_with_positive_rho_passes_setup_without_rank_gate() {
        // This is the pinned native degeneracy oracle: the DLT has rank three,
        // but its finite cheiral point has 0 < rho <= 2.  setup_opt does not
        // inspect the DLT rank, so this candidate must still be retained.
        let calibration = calibration();
        let camera = calibration.camera(0).expect("camera");
        let host_pixel = camera
            .project(&nalgebra::Point3::new(0.0, 0.0, 5.0))
            .expect("host projection");
        let candidate_pixel = camera
            .project(&nalgebra::Point3::new(-1.0, 0.0, 5.0))
            .expect("candidate projection");
        let host_bearing = Vector3::new(0.0, 0.0, 1.0);
        let candidate_bearing = Vector3::new(-0.2, 0.0, 1.0).normalize();
        let relative_pose = translated_pose(1.0);
        let point = triangulate_ba(host_bearing, candidate_bearing, &relative_pose)
            .expect("rank-three DLT remains finite");
        assert!(point[3] > 0.0 && point[3] <= 2.0);

        let host = TimeCamId::new(0, 0);
        let candidate = TimeCamId::new(1, 0);
        let mut tracks = FeatureTracks::new();
        tracks.insert(17, [(host, 0), (candidate, 0)].into_iter().collect());
        let image_corners = corners([(host, host_pixel), (candidate, candidate_pixel)]);
        let mut poses = BTreeMap::new();
        poses.insert(0, SE3::identity());
        poses.insert(1, relative_pose);

        let result = setup_opt_from_corners(&tracks, &image_corners, &poses, &calibration, 0.1);
        assert_eq!(result.report.accepted_track_count, 1);
        assert!(result.landmarks.contains_key(&17));
    }

    #[test]
    fn every_observation_is_retained_after_first_valid_pair() {
        let images = [
            TimeCamId::new(1, 0),
            TimeCamId::new(1, 1),
            TimeCamId::new(2, 0),
        ];
        let mut tracks = FeatureTracks::new();
        tracks.insert(7, images.into_iter().map(|image| (image, 0)).collect());
        let image_corners = corners(
            images
                .into_iter()
                .map(|image| (image, Point2::new(320.0, 240.0))),
        );
        let mut poses = BTreeMap::new();
        poses.insert(1, SE3::identity());
        poses.insert(2, translated_pose(0.2));
        let result = setup_opt_from_corners(&tracks, &image_corners, &poses, &calibration(), 0.07);
        let landmark = result.landmarks.get(&7).expect("landmark");
        assert_eq!(landmark.observations.len(), 3);
        assert_eq!(
            landmark
                .observations
                .iter()
                .map(|observation| observation.image)
                .collect::<Vec<_>>(),
            images
        );
    }

    #[test]
    fn canonical_hash_is_order_independent_at_map_boundary() {
        let mut first = BTreeMap::new();
        let mut second = BTreeMap::new();
        let landmark = MapperLandmark {
            track_id: 2,
            host: TimeCamId::new(1, 0),
            second: TimeCamId::new(2, 0),
            direction: StereographicDirection::from_bearing(Vector3::new(0.0, 0.0, 1.0)).unwrap(),
            inverse_distance: 0.5,
            observations: vec![MapperObservation {
                image: TimeCamId::new(1, 0),
                feature_id: 3,
                pixel: Point2::new(320.0, 240.0),
            }],
        };
        first.insert(2, landmark.clone());
        second.insert(2, landmark);
        assert_eq!(
            canonical_setup_opt_hash(&first),
            canonical_setup_opt_hash(&second)
        );
        assert!(!canonical_setup_opt_bytes(&first).is_empty());
    }
}
