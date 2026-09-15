//! Direct frame-to-frame Basalt-style KLT tracking.
//!
//! This module intentionally does not call the repository's descriptor,
//! generic optical-flow, or PnP paths. The public processing order is kept
//! explicit in [`TrackStage`]:
//!
//! 1. cam0 forward SE(2) IC,
//! 2. cam0 backward SE(2) IC,
//! 3. cam0 FB² rejection at `0.04`,
//! 4. cam0 grid FAST replenishment,
//! 5. new cam0 to cam1 stereo KLT,
//! 6. stereo backward KLT and FB² rejection,
//! 7. Double Sphere bearing essential residual at `0.005`,
//! 8. emit observations.
//!
//! Existing stereo observations are also tracked between frames before
//! replenishment, matching Basalt's frame-to-frame map behavior. Each camera's
//! observation map is tracked independently; cam1 is not dropped merely
//! because the corresponding cam0 ID failed. They have separate trace entries
//! so the new-point sequence above remains visible.

use std::collections::BTreeMap;

use nalgebra::{Matrix2, Matrix3, Point2, Vector2};
use rayon::prelude::*;
use thiserror::Error;

use crate::{
    calibration::BasaltCalibration,
    fast::{GridFastConfig, GridFastDetector},
    patch::{MeanNormalizedPatch51, PatchResidualError},
    pyramid::{ImageError, RawU16Pyramid},
    timing::{TimingBreakdown, TimingBucket},
    types::{BasaltFrame, FrameId, TrackId, TrackObservation},
    update::{AffineCompact2f, Se2UpdateError},
};

/// Configuration for the direct KLT stream.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DirectKltConfig {
    pub pyramid_levels: usize,
    pub max_iterations: usize,
    pub fb_squared_threshold: f32,
    pub essential_residual_threshold: f64,
    pub fast: GridFastConfig,
}

impl Default for DirectKltConfig {
    fn default() -> Self {
        Self {
            pyramid_levels: 3,
            max_iterations: 5,
            fb_squared_threshold: 0.04,
            essential_residual_threshold: 0.005,
            fast: GridFastConfig::default(),
        }
    }
}

/// A stereo frame supplied directly to the Basalt KLT stream.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StereoFrame {
    pub frame_id: FrameId,
    pub timestamp_ns: i64,
    pub cam0: crate::RawU16Image,
    pub cam1: Option<crate::RawU16Image>,
}

impl StereoFrame {
    pub fn new(
        frame_id: FrameId,
        timestamp_ns: i64,
        cam0: crate::RawU16Image,
        cam1: Option<crate::RawU16Image>,
    ) -> Self {
        Self {
            frame_id,
            timestamp_ns,
            cam0,
            cam1,
        }
    }
}

/// Public stages emitted by one `process_frame` call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrackStage {
    FrameForwardSe2Ic,
    FrameBackwardSe2Ic,
    FrameFbSquared,
    ExistingStereoForwardSe2Ic,
    ExistingStereoBackwardSe2Ic,
    ExistingStereoFbSquared,
    Cam0GridFastReplenish,
    StereoForwardSe2Ic,
    StereoBackwardSe2Ic,
    StereoFbSquared,
    DsBearingEssentialResidual,
    Emit,
}

/// Fine-grained failure from one direct SE(2) KLT direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum KltFailure {
    SourcePatchInvalid,
    TargetNoValidSamples,
    TargetInsufficientOverlap,
    IncrementNonFinite,
    IncrementTooLarge,
    TargetOutOfBounds,
}

/// Reject reason counters are stage-qualified to make lifecycle failures
/// diagnosable without inspecting an image stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum RejectReason {
    FrameForward(KltFailure),
    FrameBackward(KltFailure),
    FrameFbSquared,
    ExistingStereoForward(KltFailure),
    ExistingStereoBackward(KltFailure),
    ExistingStereoFbSquared,
    FastNoCandidate,
    StereoForward(KltFailure),
    StereoBackward(KltFailure),
    StereoFbSquared,
    StereoBearingInvalid,
    StereoEssentialResidual,
}

/// Per-frame and cumulative reject counters.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct RejectReasonCounters {
    counts: BTreeMap<RejectReason, usize>,
}

impl RejectReasonCounters {
    pub fn record(&mut self, reason: RejectReason) {
        *self.counts.entry(reason).or_default() += 1;
    }

    pub fn count(&self, reason: RejectReason) -> usize {
        self.counts.get(&reason).copied().unwrap_or(0)
    }

    pub fn total(&self) -> usize {
        self.counts.values().sum()
    }

    pub fn iter(&self) -> impl Iterator<Item = (&RejectReason, &usize)> {
        self.counts.iter()
    }

    fn merge_from(&mut self, other: &Self) {
        for (reason, count) in &other.counts {
            *self.counts.entry(*reason).or_default() += count;
        }
    }
}

/// Emitted observations and lifecycle information for one frame.
#[derive(Debug, Clone, PartialEq)]
pub struct TrackFrameOutput {
    pub frame: BasaltFrame,
    pub observations: Vec<TrackObservation>,
    pub created_track_ids: Vec<TrackId>,
    pub retained_track_ids: Vec<TrackId>,
    pub rejected_track_ids: Vec<TrackId>,
    pub reject_counters: RejectReasonCounters,
    pub stage_trace: Vec<TrackStage>,
}

/// Errors that prevent a frame from entering the direct stream.
#[derive(Debug, Error)]
pub enum StreamError {
    #[error("direct KLT configuration is invalid: {0}")]
    InvalidConfig(&'static str),
    #[error("camera 0 calibration is missing")]
    MissingCamera0Calibration,
    #[error("stereo frame supplied cam1 but camera 1 calibration is missing")]
    MissingCamera1Calibration,
    #[error("frame id/timestamp is not newer than the previous frame")]
    NonMonotonicFrame,
    #[error("image/pyramid error: {0}")]
    Image(#[from] ImageError),
}

#[derive(Debug, Clone, Copy)]
struct ActiveTrack {
    /// Upstream keeps one observation map per camera.  They are intentionally
    /// independent: cam1 may survive when the corresponding cam0 point is
    /// rejected (and the reverse is also possible).
    cam0: Option<AffineCompact2f>,
    cam1: Option<AffineCompact2f>,
    created_frame_id: FrameId,
}

#[derive(Debug, Clone)]
struct FramePyramids {
    cam0: RawU16Pyramid,
    cam1: Option<RawU16Pyramid>,
}

/// Direct frame-to-frame KLT tracker with monotonic track IDs.
#[derive(Debug, Clone)]
pub struct DirectKltStream {
    calibration: BasaltCalibration,
    config: DirectKltConfig,
    detector: GridFastDetector,
    previous: Option<FramePyramids>,
    previous_frame: Option<BasaltFrame>,
    tracks: BTreeMap<TrackId, ActiveTrack>,
    next_track_id: TrackId,
    cumulative_rejects: RejectReasonCounters,
    pyramid_scratch: Vec<i32>,
}

impl DirectKltStream {
    pub fn new(
        calibration: BasaltCalibration,
        config: DirectKltConfig,
    ) -> Result<Self, StreamError> {
        if calibration.camera(0).is_none() {
            return Err(StreamError::MissingCamera0Calibration);
        }
        if config.max_iterations == 0 {
            return Err(StreamError::InvalidConfig(
                "max_iterations must be positive",
            ));
        }
        if !config.fb_squared_threshold.is_finite() || config.fb_squared_threshold <= 0.0 {
            return Err(StreamError::InvalidConfig(
                "fb_squared_threshold must be finite and positive",
            ));
        }
        if !config.essential_residual_threshold.is_finite()
            || config.essential_residual_threshold <= 0.0
        {
            return Err(StreamError::InvalidConfig(
                "essential_residual_threshold must be finite and positive",
            ));
        }
        Ok(Self {
            detector: GridFastDetector::new(config.fast),
            calibration,
            config,
            previous: None,
            previous_frame: None,
            tracks: BTreeMap::new(),
            next_track_id: 0,
            cumulative_rejects: RejectReasonCounters::default(),
            pyramid_scratch: Vec::new(),
        })
    }

    pub fn active_track_ids(&self) -> Vec<TrackId> {
        self.tracks.keys().copied().collect()
    }

    pub const fn next_track_id(&self) -> TrackId {
        self.next_track_id
    }

    pub fn cumulative_reject_counters(&self) -> &RejectReasonCounters {
        &self.cumulative_rejects
    }

    /// Processes one stereo frame and emits only direct KLT observations.
    pub fn process_frame(&mut self, frame: StereoFrame) -> Result<TrackFrameOutput, StreamError> {
        let mut timing = TimingBreakdown::from_env();
        self.process_frame_with_timing(frame, &mut timing)
    }

    /// Processes one stereo frame while recording optional disjoint frontend
    /// sub-buckets in the adapter-owned timing collector.
    pub fn process_frame_with_timing(
        &mut self,
        frame: StereoFrame,
        timing: &mut TimingBreakdown,
    ) -> Result<TrackFrameOutput, StreamError> {
        if let Some(previous_frame) = self.previous_frame {
            if frame.frame_id <= previous_frame.frame_id
                || frame.timestamp_ns <= previous_frame.timestamp_ns
            {
                return Err(StreamError::NonMonotonicFrame);
            }
        }
        if frame.cam1.is_some() && self.calibration.camera(1).is_none() {
            return Err(StreamError::MissingCamera1Calibration);
        }

        let pyramid_started = timing.start();
        let cam0 = RawU16Pyramid::from_image_with_scratch(
            frame.cam0,
            self.config.pyramid_levels,
            &mut self.pyramid_scratch,
        )?;
        let cam1 = match frame.cam1 {
            Some(image) => Some(RawU16Pyramid::from_image_with_scratch(
                image,
                self.config.pyramid_levels,
                &mut self.pyramid_scratch,
            )?),
            None => None,
        };
        let current = FramePyramids { cam0, cam1 };
        timing.finish(TimingBucket::FrontendPyramid, pyramid_started);
        let mut counters = RejectReasonCounters::default();
        let stage_trace = vec![
            TrackStage::FrameForwardSe2Ic,
            TrackStage::FrameBackwardSe2Ic,
            TrackStage::FrameFbSquared,
            TrackStage::ExistingStereoForwardSe2Ic,
            TrackStage::ExistingStereoBackwardSe2Ic,
            TrackStage::ExistingStereoFbSquared,
            TrackStage::Cam0GridFastReplenish,
            TrackStage::StereoForwardSe2Ic,
            TrackStage::StereoBackwardSe2Ic,
            TrackStage::StereoFbSquared,
            TrackStage::DsBearingEssentialResidual,
            TrackStage::Emit,
        ];

        let old_tracks = std::mem::take(&mut self.tracks);
        let mut current_tracks = BTreeMap::new();
        let mut retained_track_ids = Vec::new();
        let mut rejected_track_ids = Vec::new();

        let temporal_started = timing.start();
        if let Some(previous) = &self.previous {
            // Each track's forward/backward KLT and FB^2 gate reads only the
            // (read-only, shared) previous/current pyramids plus that one
            // track's own prior observation -- there is no shared mutable
            // state inside `temporal_track_update`. Computing the per-track
            // results with rayon and then folding them into `counters` /
            // `retained_track_ids` / `rejected_track_ids` / `current_tracks`
            // serially, in the same key order `BTreeMap` iteration already
            // used, reproduces the exact push/record sequence (and therefore
            // the exact `current_tracks` contents and the exact, purely
            // integer-count `counters`) the sequential loop produced: no
            // floating-point reduction crosses a track boundary here, so
            // this is bit-identical for any thread count.
            let entries: Vec<(TrackId, ActiveTrack)> =
                old_tracks.iter().map(|(&id, &track)| (id, track)).collect();
            let results: Vec<TemporalTrackResult> = entries
                .par_iter()
                .map(|&(track_id, old_track)| {
                    temporal_track_update(track_id, old_track, previous, &current, &self.config)
                })
                .collect();
            for result in results {
                match result.cam0_classification {
                    Some(Cam0Classification::Retained) => retained_track_ids.push(result.track_id),
                    Some(Cam0Classification::Rejected(reason)) => {
                        counters.record(reason);
                        rejected_track_ids.push(result.track_id);
                    }
                    None => {}
                }
                if let Some(reason) = result.cam1_reject {
                    counters.record(reason);
                }
                // Preserve a shared ID whenever either independent upstream
                // observation map retained it.
                if result.cam0.is_some() || result.cam1.is_some() {
                    current_tracks.insert(
                        result.track_id,
                        ActiveTrack {
                            cam0: result.cam0,
                            cam1: result.cam1,
                            created_frame_id: result.created_frame_id,
                        },
                    );
                }
            }
        }
        timing.finish(TimingBucket::FrontendTemporalKlt, temporal_started);

        let fast_started = timing.start();
        let existing_positions: Vec<_> = current_tracks
            .values()
            .filter_map(|track| track.cam0.map(|cam0| *cam0.translation()))
            .collect();
        let new_positions = self.detector.detect(
            current.cam0.level(0).expect("level zero exists"),
            &existing_positions,
        );
        if new_positions.is_empty() {
            counters.record(RejectReason::FastNoCandidate);
        }
        let mut created_track_ids = Vec::new();
        for position in new_positions {
            let track_id = self.next_track_id;
            self.next_track_id += 1;
            current_tracks.insert(
                track_id,
                ActiveTrack {
                    cam0: Some(AffineCompact2f::new(Matrix2::identity(), position)),
                    cam1: None,
                    created_frame_id: frame.frame_id,
                },
            );
            created_track_ids.push(track_id);
        }
        timing.finish(TimingBucket::FrontendFastReplenish, fast_started);

        // New cam0 points are stereo-tracked only after replenishment, exactly
        // as Basalt's addPoints path does.
        let stereo_started = timing.start();
        if let (Some(current_cam1), Some(_camera1)) = (&current.cam1, self.calibration.camera(1)) {
            for track_id in &created_track_ids {
                let Some(track) = current_tracks.get_mut(track_id) else {
                    continue;
                };
                let Some(cam0_transform) = track.cam0 else {
                    continue;
                };
                let stereo_transform = match track_direction(
                    &current.cam0,
                    current_cam1,
                    cam0_transform,
                    &self.config,
                ) {
                    Ok(transform) => transform,
                    Err(failure) => {
                        counters.record(RejectReason::StereoForward(failure));
                        continue;
                    }
                };
                let recovered = match track_direction(
                    current_cam1,
                    &current.cam0,
                    stereo_transform,
                    &self.config,
                ) {
                    Ok(transform) => transform,
                    Err(failure) => {
                        counters.record(RejectReason::StereoBackward(failure));
                        continue;
                    }
                };
                if (cam0_transform.translation() - recovered.translation()).norm_squared()
                    >= self.config.fb_squared_threshold
                {
                    counters.record(RejectReason::StereoFbSquared);
                } else {
                    track.cam1 = Some(stereo_transform);
                }
            }
        }
        timing.finish(TimingBucket::FrontendNewStereoKlt, stereo_started);

        let essential_started = timing.start();
        if self.calibration.camera(1).is_some() {
            for track in current_tracks.values_mut() {
                let Some(cam1_transform) = track.cam1 else {
                    continue;
                };
                // `filterPoints` only evaluates cam1 observations whose ID is
                // also present in cam0.  An independently tracked cam1 point
                // without a cam0 counterpart remains in the upstream map.
                let Some(cam0_transform) = track.cam0 else {
                    continue;
                };
                let Some(residual) = essential_residual(
                    &self.calibration,
                    *cam0_transform.translation(),
                    *cam1_transform.translation(),
                ) else {
                    counters.record(RejectReason::StereoBearingInvalid);
                    track.cam1 = None;
                    continue;
                };
                if residual > self.config.essential_residual_threshold {
                    counters.record(RejectReason::StereoEssentialResidual);
                    track.cam1 = None;
                }
            }
        }
        timing.finish(TimingBucket::FrontendEssentialFilter, essential_started);

        let output_started = timing.start();
        let mut observations = Vec::new();
        for (track_id, track) in &current_tracks {
            if let Some(cam0) = track.cam0 {
                observations.push(TrackObservation {
                    track_id: *track_id,
                    frame_id: frame.frame_id,
                    timestamp_ns: frame.timestamp_ns,
                    camera_id: 0,
                    pixel: Point2::new(cam0.translation().x as f64, cam0.translation().y as f64),
                });
            }
            if let Some(cam1) = track.cam1 {
                observations.push(TrackObservation {
                    track_id: *track_id,
                    frame_id: frame.frame_id,
                    timestamp_ns: frame.timestamp_ns,
                    camera_id: 1,
                    pixel: Point2::new(cam1.translation().x as f64, cam1.translation().y as f64),
                });
            }
        }
        observations.sort_by_key(|observation| (observation.track_id, observation.camera_id));
        created_track_ids.sort_unstable();
        retained_track_ids.sort_unstable();
        rejected_track_ids.sort_unstable();

        self.cumulative_rejects.merge_from(&counters);
        self.tracks = current_tracks;
        self.previous = Some(current);
        let output = TrackFrameOutput {
            frame: BasaltFrame::new(frame.frame_id, frame.timestamp_ns, 0),
            observations,
            created_track_ids,
            retained_track_ids,
            rejected_track_ids,
            reject_counters: counters,
            stage_trace,
        };
        self.previous_frame = Some(output.frame);
        timing.finish(TimingBucket::FrontendOutput, output_started);
        Ok(output)
    }
}

/// One track's cam0 classification from [`temporal_track_update`], carried
/// out of the parallel worker instead of mutating a shared
/// `RejectReasonCounters` / `Vec<TrackId>` pair so the per-track computation
/// has no shared mutable state.
#[derive(Debug, Clone, Copy)]
enum Cam0Classification {
    Retained,
    Rejected(RejectReason),
}

/// Output of one track's independent temporal KLT update, folded back into
/// the caller's `counters` / `retained_track_ids` / `rejected_track_ids` /
/// `current_tracks` serially in the same order [`DirectKltStream::tracks`]
/// (a `BTreeMap`) already iterates in.
struct TemporalTrackResult {
    track_id: TrackId,
    created_frame_id: FrameId,
    cam0: Option<AffineCompact2f>,
    cam0_classification: Option<Cam0Classification>,
    cam1: Option<AffineCompact2f>,
    cam1_reject: Option<RejectReason>,
}

/// Pure, side-effect-free per-track temporal KLT update: forward/backward
/// cam0 tracking plus its FB^2 gate, and independently, forward/backward
/// cam1 tracking plus its FB^2 gate. Reads only `previous`/`current`
/// (shared, read-only) and this one track's own prior observation, so many
/// tracks can run this concurrently with no coordination and no change to
/// any individual track's arithmetic.
fn temporal_track_update(
    track_id: TrackId,
    old_track: ActiveTrack,
    previous: &FramePyramids,
    current: &FramePyramids,
    config: &DirectKltConfig,
) -> TemporalTrackResult {
    // `FrameToFrameOpticalFlow::trackPoints` runs once for each camera map.
    // Do not gate the cam1 search on cam0 success.
    let mut cam0_classification = None;
    let cam0 = old_track.cam0.and_then(|old_cam0| {
        let frame_transform = match track_direction(&previous.cam0, &current.cam0, old_cam0, config)
        {
            Ok(transform) => transform,
            Err(failure) => {
                cam0_classification = Some(Cam0Classification::Rejected(
                    RejectReason::FrameForward(failure),
                ));
                return None;
            }
        };

        let recovered =
            match track_direction(&current.cam0, &previous.cam0, frame_transform, config) {
                Ok(transform) => transform,
                Err(failure) => {
                    cam0_classification = Some(Cam0Classification::Rejected(
                        RejectReason::FrameBackward(failure),
                    ));
                    return None;
                }
            };
        let fb_squared = (old_cam0.translation() - recovered.translation()).norm_squared();
        if fb_squared >= config.fb_squared_threshold {
            cam0_classification = Some(Cam0Classification::Rejected(RejectReason::FrameFbSquared));
            None
        } else {
            cam0_classification = Some(Cam0Classification::Retained);
            Some(frame_transform)
        }
    });

    let mut cam1_reject = None;
    let cam1 = match (
        old_track.cam1,
        previous.cam1.as_ref(),
        current.cam1.as_ref(),
    ) {
        (Some(old_cam1), Some(previous_cam1), Some(current_cam1)) => {
            match track_direction(previous_cam1, current_cam1, old_cam1, config) {
                Ok(stereo_transform) => {
                    let recovered = match track_direction(
                        current_cam1,
                        previous_cam1,
                        stereo_transform,
                        config,
                    ) {
                        Ok(transform) => Some(transform),
                        Err(failure) => {
                            cam1_reject = Some(RejectReason::ExistingStereoBackward(failure));
                            None
                        }
                    };
                    recovered.and_then(|recovered| {
                        if (old_cam1.translation() - recovered.translation()).norm_squared()
                            >= config.fb_squared_threshold
                        {
                            cam1_reject = Some(RejectReason::ExistingStereoFbSquared);
                            None
                        } else {
                            Some(stereo_transform)
                        }
                    })
                }
                Err(failure) => {
                    cam1_reject = Some(RejectReason::ExistingStereoForward(failure));
                    None
                }
            }
        }
        _ => None,
    };

    TemporalTrackResult {
        track_id,
        created_frame_id: old_track.created_frame_id,
        cam0,
        cam0_classification,
        cam1,
        cam1_reject,
    }
}

fn track_direction(
    old_pyramid: &RawU16Pyramid,
    current_pyramid: &RawU16Pyramid,
    old_transform: AffineCompact2f,
    config: &DirectKltConfig,
) -> Result<AffineCompact2f, KltFailure> {
    track_direction_from_seed(
        old_pyramid,
        current_pyramid,
        *old_transform.translation(),
        old_transform,
        config,
    )
}

fn track_direction_from_seed(
    old_pyramid: &RawU16Pyramid,
    current_pyramid: &RawU16Pyramid,
    source_position: Vector2<f32>,
    initial_transform: AffineCompact2f,
    config: &DirectKltConfig,
) -> Result<AffineCompact2f, KltFailure> {
    track_direction_from_seed_impl(
        old_pyramid,
        current_pyramid,
        source_position,
        initial_transform,
        config,
        #[cfg(test)]
        None,
    )
}

#[cfg(test)]
trait KltIterationObserver {
    fn record_iteration(
        &mut self,
        level: usize,
        iteration: usize,
        patch: &MeanNormalizedPatch51,
        residual: &crate::patch::PatchData51,
        increment: nalgebra::Vector3<f32>,
        before: AffineCompact2f,
        update: crate::update::Se2,
        after: AffineCompact2f,
    );
}

#[cfg(test)]
fn track_direction_from_seed_with_observer(
    old_pyramid: &RawU16Pyramid,
    current_pyramid: &RawU16Pyramid,
    source_position: Vector2<f32>,
    initial_transform: AffineCompact2f,
    config: &DirectKltConfig,
    observer: &mut dyn KltIterationObserver,
) -> Result<AffineCompact2f, KltFailure> {
    track_direction_from_seed_impl(
        old_pyramid,
        current_pyramid,
        source_position,
        initial_transform,
        config,
        Some(observer),
    )
}

fn track_direction_from_seed_impl(
    old_pyramid: &RawU16Pyramid,
    current_pyramid: &RawU16Pyramid,
    source_position: Vector2<f32>,
    initial_transform: AffineCompact2f,
    config: &DirectKltConfig,
    #[cfg(test)] mut observer: Option<&mut dyn KltIterationObserver>,
) -> Result<AffineCompact2f, KltFailure> {
    // Basalt resets the linear part for the current IC search. The previous
    // affine linear part is composed back onto the result after all levels;
    // the old point centre seeds the translation search.
    let base_linear = *initial_transform.linear();
    let mut transform = AffineCompact2f::new(Matrix2::identity(), *initial_transform.translation());
    for level in (0..=config.pyramid_levels).rev() {
        let scale = (1_u32 << level) as f32;
        let old_position = source_position / scale;
        let level_image = old_pyramid
            .level(level)
            .ok_or(KltFailure::SourcePatchInvalid)?;
        let current_image = current_pyramid
            .level(level)
            .ok_or(KltFailure::TargetOutOfBounds)?;
        let patch = MeanNormalizedPatch51::from_image(level_image, old_position);
        if !patch.valid {
            return Err(KltFailure::SourcePatchInvalid);
        }
        let mut level_transform =
            AffineCompact2f::new(*transform.linear(), *transform.translation() / scale);
        // `iteration` is read only by the `#[cfg(test)]` observer hook below;
        // keep the name (rather than `_iteration`) so that branch still compiles.
        #[allow(unused_variables)]
        for iteration in 0..config.max_iterations {
            let residual = patch
                .residual(current_image, &level_transform)
                .map_err(map_patch_error)?;
            let increment = patch.ic_increment(&residual);
            #[cfg(test)]
            let before = level_transform;
            level_transform
                .try_right_compose_se2(increment)
                .map_err(map_update_error)?;
            #[cfg(test)]
            if let Some(observer) = observer.as_deref_mut() {
                observer.record_iteration(
                    level,
                    iteration,
                    &patch,
                    &residual,
                    increment,
                    before,
                    crate::update::Se2::exp(increment),
                    level_transform,
                );
            }
            if !current_image.in_bounds(*level_transform.translation(), 2.0) {
                return Err(KltFailure::TargetOutOfBounds);
            }
        }
        transform = AffineCompact2f::new(
            *level_transform.linear(),
            *level_transform.translation() * scale,
        );
    }
    // The upstream map carries the accumulated affine linear part between
    // frames, while resetting it for the current IC search above.
    Ok(AffineCompact2f::new(
        base_linear * *transform.linear(),
        *transform.translation(),
    ))
}

fn map_patch_error(error: PatchResidualError) -> KltFailure {
    match error {
        PatchResidualError::NoValidTargetSamples => KltFailure::TargetNoValidSamples,
        PatchResidualError::InsufficientOverlap { .. } => KltFailure::TargetInsufficientOverlap,
    }
}

fn map_update_error(error: Se2UpdateError) -> KltFailure {
    match error {
        Se2UpdateError::NonFiniteIncrement => KltFailure::IncrementNonFinite,
        Se2UpdateError::IncrementTooLarge { .. } => KltFailure::IncrementTooLarge,
    }
}

fn essential_residual(
    calibration: &BasaltCalibration,
    cam0_pixel: Vector2<f32>,
    cam1_pixel: Vector2<f32>,
) -> Option<f64> {
    let camera0 = calibration.camera(0)?;
    let camera1 = calibration.camera(1)?;
    let bearing0 = camera0.unproject(&Point2::new(cam0_pixel.x as f64, cam0_pixel.y as f64))?;
    let bearing1 = camera1.unproject(&Point2::new(cam1_pixel.x as f64, cam1_pixel.y as f64))?;
    let t_imu_cam0 = calibration.camera_to_imu(0)?;
    let t_imu_cam1 = calibration.camera_to_imu(1)?;
    let t_cam0_cam1 = t_imu_cam0.inverse().compose(t_imu_cam1);
    let translation = t_cam0_cam1.translation;
    let norm = translation.norm();
    if !norm.is_finite() || norm <= f64::EPSILON {
        return None;
    }
    let t = translation / norm;
    let skew = Matrix3::new(0.0, -t.z, t.y, t.z, 0.0, -t.x, -t.y, t.x, 0.0);
    let essential = skew * t_cam0_cam1.rotation.to_rotation_matrix().into_inner();
    Some((bearing0.dot(&(essential * bearing1))).abs())
}

#[cfg(test)]
mod tests {
    use nalgebra::{Matrix2, Vector2};

    use super::*;
    use crate::{camera::DoubleSphereCamera, types::BasaltNavState};

    #[test]
    fn reject_reason_counters_are_stage_specific_and_deterministic() {
        let mut counters = RejectReasonCounters::default();
        counters.record(RejectReason::FrameFbSquared);
        counters.record(RejectReason::FrameFbSquared);
        counters.record(RejectReason::FrameForward(KltFailure::SourcePatchInvalid));
        assert_eq!(counters.count(RejectReason::FrameFbSquared), 2);
        assert_eq!(counters.total(), 3);
        assert_eq!(counters.iter().count(), 2);
    }

    #[test]
    fn essential_residual_is_zero_for_a_horizontal_stereo_pair() {
        let camera = DoubleSphereCamera::new(40.0, 40.0, 48.0, 48.0, 0.0, 0.5, 96, 96).unwrap();
        let mut calibration = synthetic_calibration(camera);
        calibration.t_imu_cam[1].translation.x = 0.1;
        let residual = essential_residual(
            &calibration,
            Vector2::new(48.0, 48.0),
            Vector2::new(52.0, 48.0),
        )
        .unwrap();
        assert!(residual < 1e-12);
    }

    fn synthetic_calibration(camera: DoubleSphereCamera) -> BasaltCalibration {
        BasaltCalibration {
            t_imu_cam: vec![
                visloc_core::geometry::SE3::identity(),
                visloc_core::geometry::SE3::identity(),
            ],
            cameras: vec![camera, camera],
            resolutions: vec![(96, 96), (96, 96)],
            calib_accel_bias: vec![0.0; 9],
            calib_gyro_bias: vec![0.0; 12],
            imu_update_rate_hz: 200.0,
            accel_noise_std: nalgebra::Vector3::repeat(0.01),
            gyro_noise_std: nalgebra::Vector3::repeat(0.01),
            accel_bias_std: nalgebra::Vector3::repeat(0.01),
            gyro_bias_std: nalgebra::Vector3::repeat(0.01),
            t_mocap_world: visloc_core::geometry::SE3::identity(),
            t_imu_marker: visloc_core::geometry::SE3::identity(),
            mocap_time_offset_ns: 0,
            mocap_to_imu_offset_ns: 0,
            cam_time_offset_ns: 0,
        }
    }

    #[allow(dead_code)]
    fn _keep_types_linked(_: Matrix2<f32>, _: BasaltNavState) {}

    /// Run the real two-image cam1 KLT path on the first frame-12 divergence
    /// witness.  The normal test suite leaves this opt-in when the input-root
    /// variables are absent; a harness supplies the sensor root and trace
    /// path to make the operation reproducible on both MSVC and Linux.
    #[test]
    #[ignore = "diagnostic-only real-image KLT repro; requires explicit input/config/calibration/trace env"]
    fn m11_frame12_cam1_real_path_klt_trace() {
        let input_root = std::path::PathBuf::from(
            std::env::var_os("VISLOC_BASALT_KLT_REPRO_ROOT").expect("VISLOC_BASALT_KLT_REPRO_ROOT"),
        );
        let trace_path = std::path::PathBuf::from(
            std::env::var_os("VISLOC_BASALT_KLT_REPRO_TRACE")
                .expect("VISLOC_BASALT_KLT_REPRO_TRACE"),
        );
        let calibration_path = std::path::PathBuf::from(
            std::env::var_os("VISLOC_BASALT_KLT_REPRO_CALIBRATION")
                .expect("VISLOC_BASALT_KLT_REPRO_CALIBRATION"),
        );
        let config_path = std::path::PathBuf::from(
            std::env::var_os("VISLOC_BASALT_KLT_REPRO_CONFIG")
                .expect("VISLOC_BASALT_KLT_REPRO_CONFIG"),
        );
        let source_timestamp_ns = 1_403_636_580_313_555_456_i64;
        let target_timestamp_ns = 1_403_636_580_363_555_584_i64;
        let dataset =
            crate::euroc::EurocSensorDataset::open(&input_root, &calibration_path, &config_path)
                .expect("open KLT repro EuRoC dataset");
        let source_frame = dataset.frame(11).expect("load KLT repro source frame");
        let target_frame = dataset.frame(12).expect("load KLT repro target frame");
        assert_eq!(source_frame.timestamp_ns, source_timestamp_ns);
        assert_eq!(target_frame.timestamp_ns, target_timestamp_ns);
        let source_path = source_frame.cam1_path.clone().expect("source cam1 path");
        let target_path = target_frame.cam1_path.clone().expect("target cam1 path");
        let config = DirectKltConfig::default();
        let source = RawU16Pyramid::from_image(
            source_frame.cam1.expect("source cam1 image"),
            config.pyramid_levels,
        )
        .expect("source KLT repro pyramid");
        let target = RawU16Pyramid::from_image(
            target_frame.cam1.expect("target cam1 image"),
            config.pyramid_levels,
        )
        .expect("target KLT repro pyramid");
        let source_position = Vector2::new(469.9930419921875_f32, 50.48072052001953_f32);
        let initial = AffineCompact2f::new(Matrix2::identity(), source_position);
        let result = track_direction_from_seed(&source, &target, source_position, initial, &config)
            .expect("real frame-11 to frame-12 cam1 KLT path");
        let record = serde_json::json!({
            "schema": "visloc.basalt.m11.klt_real_path_trace.v1",
            "source": "rust",
            "camera_id": 1,
            "source_frame_id": 11,
            "target_frame_id": 12,
            "source_timestamp_ns": source_timestamp_ns,
            "target_timestamp_ns": target_timestamp_ns,
            "source_path": source_path,
            "target_path": target_path,
            "decoder": "EurocSensorDataset::frame -> euroc::read_raw_u16_png -> dynamic_to_raw_u16",
            "calibration_path": calibration_path,
            "config_path": config_path,
            "image_dimensions": {
                "source": [source.level(0).expect("source level zero").width(), source.level(0).expect("source level zero").height()],
                "target": [target.level(0).expect("target level zero").width(), target.level(0).expect("target level zero").height()]
            },
            "config": {
                "pyramid_levels": config.pyramid_levels,
                "max_iterations": config.max_iterations,
                "source_position_f32": [source_position.x, source_position.y],
                "source_position_f32_bits": [format!("{:08x}", source_position.x.to_bits()), format!("{:08x}", source_position.y.to_bits())],
                "initial_linear_f32_bits_column_major": matrix2_bits(initial.linear()),
                "initial_translation_f32_bits": vector2_bits(initial.translation()),
            },
            "result": {
                "linear_f32": matrix2_values(result.linear()),
                "linear_f32_bits_column_major": matrix2_bits(result.linear()),
                "translation_f32": [result.translation().x, result.translation().y],
                "translation_f32_bits": vector2_bits(result.translation()),
            },
        });
        if let Some(parent) = trace_path.parent() {
            std::fs::create_dir_all(parent).expect("KLT repro trace parent");
        }
        std::fs::write(
            &trace_path,
            serde_json::to_vec_pretty(&record).expect("KLT repro trace JSON"),
        )
        .expect("KLT repro trace output");
        println!(
            "{}",
            serde_json::to_string(&record).expect("KLT repro trace line")
        );
    }

    /// Qualify the finite-angle Arm-derived cosine path on the real decoder and
    /// real two-image KLT path.  This remains ignored and requires explicit
    /// input/config/calibration variables; the assertion is the pinned
    /// native/Linux endpoint witness recorded in
    /// work/m11_frame12_native_endpoint_binding_20260907.
    #[test]
    #[ignore = "diagnostic-only candidate endpoint; requires explicit input/config/calibration env"]
    fn m11_frame12_cam1_real_path_klt_portable_cos_candidate() {
        let input_root = std::path::PathBuf::from(
            std::env::var_os("VISLOC_BASALT_KLT_REPRO_ROOT").expect("VISLOC_BASALT_KLT_REPRO_ROOT"),
        );
        let calibration_path = std::path::PathBuf::from(
            std::env::var_os("VISLOC_BASALT_KLT_REPRO_CALIBRATION")
                .expect("VISLOC_BASALT_KLT_REPRO_CALIBRATION"),
        );
        let config_path = std::path::PathBuf::from(
            std::env::var_os("VISLOC_BASALT_KLT_REPRO_CONFIG")
                .expect("VISLOC_BASALT_KLT_REPRO_CONFIG"),
        );
        let dataset =
            crate::euroc::EurocSensorDataset::open(&input_root, &calibration_path, &config_path)
                .expect("open candidate KLT repro EuRoC dataset");
        let source_frame = dataset.frame(11).expect("load candidate KLT source frame");
        let target_frame = dataset.frame(12).expect("load candidate KLT target frame");
        assert_eq!(source_frame.timestamp_ns, 1_403_636_580_313_555_456_i64);
        assert_eq!(target_frame.timestamp_ns, 1_403_636_580_363_555_584_i64);
        let config = DirectKltConfig::default();
        let source = RawU16Pyramid::from_image(
            source_frame.cam1.expect("candidate source cam1 image"),
            config.pyramid_levels,
        )
        .expect("candidate source KLT pyramid");
        let target = RawU16Pyramid::from_image(
            target_frame.cam1.expect("candidate target cam1 image"),
            config.pyramid_levels,
        )
        .expect("candidate target KLT pyramid");
        let source_position = Vector2::new(469.9930419921875_f32, 50.48072052001953_f32);
        let initial = AffineCompact2f::new(Matrix2::identity(), source_position);
        let result = track_direction_from_seed(&source, &target, source_position, initial, &config)
            .expect("candidate frame-11 to frame-12 cam1 KLT path");
        assert_eq!(
            result.translation().x.to_bits(),
            0x43ebb126,
            "candidate endpoint x must match pinned native/Linux endpoint"
        );
        assert_eq!(
            result.translation().y.to_bits(),
            0x4232f6b6,
            "candidate endpoint y must match pinned native/Linux endpoint"
        );
        println!(
            "{{\"schema\":\"visloc.basalt.m11.klt_portable_cos_candidate_endpoint.v1\",\"candidate_gate\":true,\"translation_f32_bits\":[\"{:08x}\",\"{:08x}\"]}}",
            result.translation().x.to_bits(),
            result.translation().y.to_bits()
        );
    }

    #[derive(Default)]
    struct KltIterationTraceRecorder {
        records: Vec<serde_json::Value>,
    }

    impl KltIterationTraceRecorder {
        fn f32_bits(values: &[f32]) -> Vec<String> {
            values
                .iter()
                .map(|value| format!("{:08x}", value.to_bits()))
                .collect()
        }

        fn patch_record(patch: &crate::patch::MeanNormalizedPatch51) -> serde_json::Value {
            let data: Vec<f32> = patch.data.iter().copied().collect();
            let jacobian: Vec<f32> = (0..52)
                .flat_map(|row| (0..3).map(move |column| patch.jacobian_se2[(row, column)]))
                .collect();
            let inverse_jacobian: Vec<f32> = (0..3)
                .flat_map(|row| (0..52).map(move |column| patch.h_se2_inv_j_se2_t[(row, column)]))
                .collect();
            serde_json::json!({
                "position_f32": [patch.position.x, patch.position.y],
                "position_f32_bits": Self::f32_bits(&[patch.position.x, patch.position.y]),
                "mean_f32": patch.mean,
                "mean_f32_bits": format!("{:08x}", patch.mean.to_bits()),
                "valid_samples": patch.valid_samples,
                "valid": patch.valid,
                "data_f32": data,
                "data_f32_bits": Self::f32_bits(&data),
                "jacobian_se2_f32_row_major": jacobian,
                "jacobian_se2_f32_bits_row_major": Self::f32_bits(&jacobian),
                "h_se2_inv_j_se2_t_f32_row_major": inverse_jacobian,
                "h_se2_inv_j_se2_t_f32_bits_row_major": Self::f32_bits(&inverse_jacobian),
            })
        }

        fn residual_record(residual: &crate::patch::PatchData51) -> serde_json::Value {
            let values: Vec<f32> = residual.iter().copied().collect();
            serde_json::json!({
                "f32": values,
                "f32_bits": Self::f32_bits(&values),
            })
        }

        fn vector3_record(vector: nalgebra::Vector3<f32>) -> serde_json::Value {
            let values = [vector.x, vector.y, vector.z];
            serde_json::json!({
                "f32": values,
                "f32_bits": Self::f32_bits(&values),
            })
        }

        fn affine_record(transform: AffineCompact2f) -> serde_json::Value {
            serde_json::json!({
                "linear_f32": matrix2_values(transform.linear()),
                "linear_f32_bits_column_major": matrix2_bits(transform.linear()),
                "translation_f32": [transform.translation().x, transform.translation().y],
                "translation_f32_bits": vector2_bits(transform.translation()),
            })
        }

        fn se2_exp_record(
            increment: nalgebra::Vector3<f32>,
            update: crate::update::Se2,
        ) -> serde_json::Value {
            let theta = increment.z;
            let sin_theta = update.rotation[(1, 0)];
            let cos_theta = update.rotation[(0, 0)];
            let (sin_over_theta, one_minus_cos_over_theta) = if theta.abs() < 1e-5 {
                let theta_sq = theta * theta;
                (
                    1.0 - (1.0 / 6.0) * theta_sq,
                    0.5 * theta - (1.0 / 24.0) * theta * theta_sq,
                )
            } else {
                (sin_theta / theta, (1.0 - cos_theta) / theta)
            };
            serde_json::json!({
                "theta_f32": theta,
                "theta_f32_bits": format!("{:08x}", theta.to_bits()),
                "sin_theta_f32": sin_theta,
                "sin_theta_f32_bits": format!("{:08x}", sin_theta.to_bits()),
                "cos_theta_f32": cos_theta,
                "cos_theta_f32_bits": format!("{:08x}", cos_theta.to_bits()),
                "sin_over_theta_f32": sin_over_theta,
                "sin_over_theta_f32_bits": format!("{:08x}", sin_over_theta.to_bits()),
                "one_minus_cos_over_theta_f32": one_minus_cos_over_theta,
                "one_minus_cos_over_theta_f32_bits":
                    format!("{:08x}", one_minus_cos_over_theta.to_bits()),
                "rotation_f32": [
                    [update.rotation[(0, 0)], update.rotation[(0, 1)]],
                    [update.rotation[(1, 0)], update.rotation[(1, 1)]],
                ],
                "rotation_f32_bits_column_major": [
                    format!("{:08x}", update.rotation[(0, 0)].to_bits()),
                    format!("{:08x}", update.rotation[(1, 0)].to_bits()),
                    format!("{:08x}", update.rotation[(0, 1)].to_bits()),
                    format!("{:08x}", update.rotation[(1, 1)].to_bits()),
                ],
                "translation_f32": [update.translation.x, update.translation.y],
                "translation_f32_bits": vector2_bits(&update.translation),
            })
        }
    }

    impl KltIterationObserver for KltIterationTraceRecorder {
        fn record_iteration(
            &mut self,
            level: usize,
            iteration: usize,
            patch: &crate::patch::MeanNormalizedPatch51,
            residual: &crate::patch::PatchData51,
            increment: nalgebra::Vector3<f32>,
            before: AffineCompact2f,
            update: crate::update::Se2,
            after: AffineCompact2f,
        ) {
            self.records.push(serde_json::json!({
                "level": level,
                "iteration": iteration,
                "scale_f32": (1_u32 << level) as f32,
                "patch": Self::patch_record(patch),
                "residual": Self::residual_record(residual),
                "increment": Self::vector3_record(increment),
                "recomputed_se2_exp": Self::se2_exp_record(increment, update),
                "affine_before": Self::affine_record(before),
                "affine_after": Self::affine_record(after),
            }));
        }
    }

    #[test]
    #[ignore = "diagnostic-only real-image KLT iteration trace; requires explicit input/config/calibration/trace env"]
    fn m11_frame12_cam1_real_path_klt_iteration_trace() {
        let input_root = std::path::PathBuf::from(
            std::env::var_os("VISLOC_BASALT_KLT_REPRO_ROOT").expect("VISLOC_BASALT_KLT_REPRO_ROOT"),
        );
        let trace_path = std::path::PathBuf::from(
            std::env::var_os("VISLOC_BASALT_KLT_REPRO_ITER_TRACE")
                .expect("VISLOC_BASALT_KLT_REPRO_ITER_TRACE"),
        );
        let calibration_path = std::path::PathBuf::from(
            std::env::var_os("VISLOC_BASALT_KLT_REPRO_CALIBRATION")
                .expect("VISLOC_BASALT_KLT_REPRO_CALIBRATION"),
        );
        let config_path = std::path::PathBuf::from(
            std::env::var_os("VISLOC_BASALT_KLT_REPRO_CONFIG")
                .expect("VISLOC_BASALT_KLT_REPRO_CONFIG"),
        );
        let source_timestamp_ns = 1_403_636_580_313_555_456_i64;
        let target_timestamp_ns = 1_403_636_580_363_555_584_i64;
        let dataset =
            crate::euroc::EurocSensorDataset::open(&input_root, &calibration_path, &config_path)
                .expect("open KLT iteration repro EuRoC dataset");
        let source_frame = dataset.frame(11).expect("load KLT iteration source frame");
        let target_frame = dataset.frame(12).expect("load KLT iteration target frame");
        assert_eq!(source_frame.timestamp_ns, source_timestamp_ns);
        assert_eq!(target_frame.timestamp_ns, target_timestamp_ns);
        let source_path = source_frame.cam1_path.clone().expect("source cam1 path");
        let target_path = target_frame.cam1_path.clone().expect("target cam1 path");
        let config = DirectKltConfig::default();
        let source = RawU16Pyramid::from_image(
            source_frame.cam1.expect("source cam1 image"),
            config.pyramid_levels,
        )
        .expect("source KLT iteration pyramid");
        let target = RawU16Pyramid::from_image(
            target_frame.cam1.expect("target cam1 image"),
            config.pyramid_levels,
        )
        .expect("target KLT iteration pyramid");
        let source_position = Vector2::new(469.9930419921875_f32, 50.48072052001953_f32);
        let initial = AffineCompact2f::new(Matrix2::identity(), source_position);
        let mut observer = KltIterationTraceRecorder::default();
        let result = track_direction_from_seed_with_observer(
            &source,
            &target,
            source_position,
            initial,
            &config,
            &mut observer,
        )
        .expect("real frame-11 to frame-12 KLT iteration path");
        let unobserved_result =
            track_direction_from_seed(&source, &target, source_position, initial, &config)
                .expect("unobserved real frame-11 to frame-12 KLT iteration path");
        assert_eq!(
            result, unobserved_result,
            "iteration observer must not alter the KLT result"
        );
        let record = serde_json::json!({
            "schema": "visloc.basalt.m11.klt_real_path_iteration_trace.v1",
            "source": "rust",
            "capture_status": "TRACE_CAPTURED",
            "observer_result_match_unobserved": true,
            "observer_notes": [
                "residual, increment, affine_before, and affine_after are captured from the real loop",
                "recomputed_se2_exp is evaluated from the same increment after the production update and is not an authoritative intermediate capture"
            ],
            "profile": std::env::var("VISLOC_BASALT_KLT_REPRO_PROFILE")
                .unwrap_or_else(|_| "unspecified".to_owned()),
            "camera_id": 1,
            "source_frame_id": 11,
            "target_frame_id": 12,
            "source_timestamp_ns": source_timestamp_ns,
            "target_timestamp_ns": target_timestamp_ns,
            "source_path": source_path,
            "target_path": target_path,
            "decoder": "EurocSensorDataset::frame -> euroc::read_raw_u16_png -> dynamic_to_raw_u16",
            "calibration_path": calibration_path,
            "config_path": config_path,
            "config": {
                "pyramid_levels": config.pyramid_levels,
                "max_iterations": config.max_iterations,
                "source_position_f32": [source_position.x, source_position.y],
                "source_position_f32_bits": [
                    format!("{:08x}", source_position.x.to_bits()),
                    format!("{:08x}", source_position.y.to_bits())
                ],
                "initial_linear_f32_bits_column_major": matrix2_bits(initial.linear()),
                "initial_translation_f32_bits": vector2_bits(initial.translation()),
            },
            "result": {
                "linear_f32": matrix2_values(result.linear()),
                "linear_f32_bits_column_major": matrix2_bits(result.linear()),
                "translation_f32": [result.translation().x, result.translation().y],
                "translation_f32_bits": vector2_bits(result.translation()),
            },
            "iteration_count": observer.records.len(),
            "iterations": observer.records,
        });
        if let Some(parent) = trace_path.parent() {
            std::fs::create_dir_all(parent).expect("KLT iteration trace parent");
        }
        std::fs::write(
            &trace_path,
            serde_json::to_vec(&record).expect("KLT iteration trace JSON"),
        )
        .expect("KLT iteration trace output");
        println!(
            "{}",
            serde_json::to_string(&record).expect("KLT iteration trace line")
        );
    }

    /// Capture the first current cross-platform endpoint mismatch through the
    /// private real KLT path.  The observer result is compared with an
    /// unobserved invocation so this diagnostic cannot change the endpoint.
    #[test]
    #[ignore = "diagnostic-only frame-14 cam0 KLT endpoint; requires explicit EuRoC paths"]
    fn m11_frame14_cam0_track203_real_path_klt_endpoint_diagnostic() {
        let input_root = std::path::PathBuf::from(
            std::env::var_os("VISLOC_BASALT_KLT_REPRO_ROOT").expect("VISLOC_BASALT_KLT_REPRO_ROOT"),
        );
        let trace_path = std::path::PathBuf::from(
            std::env::var_os("VISLOC_BASALT_KLT_REPRO_ITER_TRACE")
                .expect("VISLOC_BASALT_KLT_REPRO_ITER_TRACE"),
        );
        let calibration_path = std::path::PathBuf::from(
            std::env::var_os("VISLOC_BASALT_KLT_REPRO_CALIBRATION")
                .expect("VISLOC_BASALT_KLT_REPRO_CALIBRATION"),
        );
        let config_path = std::path::PathBuf::from(
            std::env::var_os("VISLOC_BASALT_KLT_REPRO_CONFIG")
                .expect("VISLOC_BASALT_KLT_REPRO_CONFIG"),
        );
        const SOURCE_FRAME: usize = 13;
        const TARGET_FRAME: usize = 14;
        const TRACK_ID: u64 = 203;
        const SOURCE_TIMESTAMP_NS: i64 = 1_403_636_580_413_555_456;
        const TARGET_TIMESTAMP_NS: i64 = 1_403_636_580_463_555_584;
        const SOURCE_X: f32 = 170.6181640625;
        const SOURCE_Y: f32 = 90.51874542236328;
        const NATIVE_TARGET_X: f32 = 170.787841796875;
        const NATIVE_TARGET_Y: f32 = 81.55065155029297;

        let dataset =
            crate::euroc::EurocSensorDataset::open(&input_root, &calibration_path, &config_path)
                .expect("open frame-14 KLT diagnostic EuRoC dataset");
        let source_frame = dataset.frame(SOURCE_FRAME).expect("load frame 13");
        let target_frame = dataset.frame(TARGET_FRAME).expect("load frame 14");
        assert_eq!(source_frame.timestamp_ns, SOURCE_TIMESTAMP_NS);
        assert_eq!(target_frame.timestamp_ns, TARGET_TIMESTAMP_NS);
        let source_path = source_frame.cam0_path.clone();
        let target_path = target_frame.cam0_path.clone();
        let config = DirectKltConfig::default();
        let source = RawU16Pyramid::from_image(source_frame.cam0, config.pyramid_levels)
            .expect("frame-13 cam0 KLT pyramid");
        let target = RawU16Pyramid::from_image(target_frame.cam0, config.pyramid_levels)
            .expect("frame-14 cam0 KLT pyramid");
        let source_position = Vector2::new(SOURCE_X, SOURCE_Y);
        let initial = AffineCompact2f::new(Matrix2::identity(), source_position);
        let mut observer = KltIterationTraceRecorder::default();
        let observed = track_direction_from_seed_with_observer(
            &source,
            &target,
            source_position,
            initial,
            &config,
            &mut observer,
        )
        .expect("observed frame-13 to frame-14 cam0 KLT path");
        let unobserved =
            track_direction_from_seed(&source, &target, source_position, initial, &config)
                .expect("unobserved frame-13 to frame-14 cam0 KLT path");
        assert_eq!(
            observed, unobserved,
            "observer must be observationally neutral"
        );

        let camera = dataset
            .calibration()
            .camera(0)
            .expect("cam0 calibration for endpoint diagnostic");
        let source_bearing = camera
            .unproject(&Point2::new(
                source_position.x as f64,
                source_position.y as f64,
            ))
            .expect("unproject source seed");
        let target_bearing = camera
            .unproject(&Point2::new(
                observed.translation().x as f64,
                observed.translation().y as f64,
            ))
            .expect("unproject observed endpoint");
        let target_x = observed.translation().x;
        let target_y = observed.translation().y;
        assert_eq!(
            target_x.to_bits(),
            NATIVE_TARGET_X.to_bits(),
            "frame-14 cam0 track203 endpoint x must match the pinned native packet"
        );
        assert_eq!(
            target_y.to_bits(),
            NATIVE_TARGET_Y.to_bits(),
            "frame-14 cam0 track203 endpoint y must match the pinned native packet"
        );
        let record = serde_json::json!({
            "schema": "visloc.basalt.m11.frame14_raw_frontend_endpoint_iteration.v1",
            "source": "rust_direct_klt_stream_private_path",
            "camera_id": 0,
            "track_id": TRACK_ID,
            "source_frame_id": SOURCE_FRAME,
            "target_frame_id": TARGET_FRAME,
            "source_timestamp_ns": SOURCE_TIMESTAMP_NS,
            "target_timestamp_ns": TARGET_TIMESTAMP_NS,
            "source_path": source_path,
            "target_path": target_path,
            "decoder": "EurocSensorDataset::frame -> euroc::read_raw_u16_png -> dynamic_to_raw_u16",
            "config": {
                "pyramid_levels": config.pyramid_levels,
                "max_iterations": config.max_iterations,
                "source_position_f32": [source_position.x, source_position.y],
                "source_position_f32_bits": [format!("{:08x}", source_position.x.to_bits()), format!("{:08x}", source_position.y.to_bits())],
                "initial_linear_f32_bits_column_major": matrix2_bits(initial.linear()),
                "initial_translation_f32_bits": vector2_bits(initial.translation())
            },
            "observational_neutrality": {
                "observer_result_equals_unobserved": true,
                "observed_iteration_count": observer.records.len()
            },
            "endpoint": {
                "observed_f32": [target_x, target_y],
                "observed_f32_bits": [format!("{:08x}", target_x.to_bits()), format!("{:08x}", target_y.to_bits())],
                "pinned_native_f32": [NATIVE_TARGET_X, NATIVE_TARGET_Y],
                "pinned_native_f32_bits": [format!("{:08x}", NATIVE_TARGET_X.to_bits()), format!("{:08x}", NATIVE_TARGET_Y.to_bits())],
                "delta_f32": [target_x - NATIVE_TARGET_X, target_y - NATIVE_TARGET_Y]
            },
            "camera_unproject": {
                "source_seed_f64": [source_bearing.x, source_bearing.y, source_bearing.z],
                "target_endpoint_f64": [target_bearing.x, target_bearing.y, target_bearing.z]
            },
            "iterations": observer.records
        });
        let bytes = serde_json::to_vec(&record).expect("frame-14 KLT diagnostic JSON");
        assert!(
            bytes.len() <= 8 * 1024 * 1024,
            "diagnostic trace must remain bounded"
        );
        if let Some(parent) = trace_path.parent() {
            std::fs::create_dir_all(parent).expect("frame-14 KLT trace parent");
        }
        std::fs::write(&trace_path, &bytes).expect("frame-14 KLT trace output");
        println!(
            "{}",
            serde_json::to_string(&record).expect("frame-14 KLT trace line")
        );
    }

    #[test]
    #[ignore = "frame19 track30 cam1 real KLT diagnostic; explicit sensor-only paths required"]
    fn m11_frame19_cam1_track30_real_path_klt_diagnostic() {
        let env_path = |name| std::path::PathBuf::from(std::env::var_os(name).expect(name));
        let dataset = crate::euroc::EurocSensorDataset::open(
            &env_path("VISLOC_BASALT_KLT_REPRO_ROOT"),
            &env_path("VISLOC_BASALT_KLT_REPRO_CALIBRATION"),
            &env_path("VISLOC_BASALT_KLT_REPRO_CONFIG"),
        )
        .expect("sensor-only dataset");
        let source = dataset.frame(18).expect("frame18");
        let target = dataset.frame(19).expect("frame19");
        assert_eq!(target.timestamp_ns, 1_403_636_580_713_555_456);
        let config = DirectKltConfig::default();
        let source =
            RawU16Pyramid::from_image(source.cam1.expect("source cam1"), config.pyramid_levels)
                .unwrap();
        let target =
            RawU16Pyramid::from_image(target.cam1.expect("target cam1"), config.pyramid_levels)
                .unwrap();
        // Verified native and Rust frame18 observation. Search resets the
        // linear part; base_linear only changes the final affine linear part.
        let position = Vector2::new(f32::from_bits(0x433c7382), f32::from_bits(0x42cbefbf));
        let seed = AffineCompact2f::new(Matrix2::identity(), position);
        let mut observer = KltIterationTraceRecorder::default();
        let observed = track_direction_from_seed_with_observer(
            &source,
            &target,
            position,
            seed,
            &config,
            &mut observer,
        )
        .unwrap();
        let plain = track_direction_from_seed(&source, &target, position, seed, &config).unwrap();
        assert_eq!(observed, plain, "diagnostic neutrality");
        let endpoint = [
            observed.translation().x.to_bits(),
            observed.translation().y.to_bits(),
        ];
        let record = serde_json::json!({
            "schema": "visloc.basalt.frame19.track30.klt.v1",
            "source_frame": 18, "target_frame": 19, "camera": 1, "track": 30,
            "source_bits": ["433c7382", "42cbefbf"],
            "endpoint_bits": endpoint.map(|v| format!("{v:08x}")),
            "native_endpoint_bits": ["433c2bc2", "42d65f34"],
            "observer_neutral": true, "iterations": observer.records
        });
        let path = env_path("VISLOC_BASALT_KLT_REPRO_ITER_TRACE");
        let bytes = serde_json::to_vec(&record).unwrap();
        assert!(bytes.len() < 8 * 1024 * 1024);
        use std::io::Write;
        std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(path)
            .unwrap()
            .write_all(&bytes)
            .unwrap();
        // This bounded endpoint oracle does not replace full VIO parity gates.
        assert_eq!(endpoint, [0x433c2bc2, 0x42d65f34]);
    }

    fn matrix2_values(matrix: &Matrix2<f32>) -> [[f32; 2]; 2] {
        [
            [matrix[(0, 0)], matrix[(0, 1)]],
            [matrix[(1, 0)], matrix[(1, 1)]],
        ]
    }

    fn matrix2_bits(matrix: &Matrix2<f32>) -> [String; 4] {
        [
            format!("{:08x}", matrix[(0, 0)].to_bits()),
            format!("{:08x}", matrix[(1, 0)].to_bits()),
            format!("{:08x}", matrix[(0, 1)].to_bits()),
            format!("{:08x}", matrix[(1, 1)].to_bits()),
        ]
    }

    fn vector2_bits(vector: &Vector2<f32>) -> [String; 2] {
        [
            format!("{:08x}", vector.x.to_bits()),
            format!("{:08x}", vector.y.to_bits()),
        ]
    }
}
