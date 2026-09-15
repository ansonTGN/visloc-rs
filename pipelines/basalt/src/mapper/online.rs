//! Online (incremental) wrapper around the offline `NfrMapper`.
//!
//! This module is additive only: it does not modify
//! `pipelines/basalt/src/mapper/{mod.rs,session.rs,features.rs,triangulation.rs}`,
//! which remain the pinned offline/parity path (see
//! `docs/basalt_online_mapper_design.md`). `OnlineNfrMapper` wraps a plain
//! [`NfrMapper`] and calls its existing public methods (`add_marg_data`,
//! `build_tracks`, `setup_opt`, `optimize`, `filter_outliers`,
//! `trajectory_tum`, ...) plus the same lower-level pure frontend functions
//! `session.rs` itself calls (`extract_mapper_features`,
//! `match_stereo_features`, `query_bow_candidates`, `match_temporal_stage`,
//! `match_temporal_ransac[_seeded]`) -- it adds new incremental entry points,
//! it does not change any existing one's behavior.
//!
//! Design basis: `docs/basalt_online_mapper_design.md` Sec3. Three rules
//! drive every method here:
//!
//! 1. **Never retain raw pixel buffers past the packet that produced them.**
//!    Phase A measurement showed `img_data` (retained `OfImageData`) is the
//!    dominant term in the offline mapper's 3-6 GB RSS (Sec1.2 of the design
//!    doc); `ingest_packet` extracts features for every eligible image in a
//!    packet and removes that image's raw buffer from `NfrMapper::img_data`
//!    in the same call.
//! 2. **Query each new keyframe against the BoW database exactly once**,
//!    instead of re-querying every already-processed key on every call the
//!    way batch `NfrMapper::match_all` does. `match_new_keyframe` calls the
//!    identical `query_bow_candidates`/`match_temporal_stage`/
//!    `match_temporal_ransac` sequence `match_all`'s inner loop body uses,
//!    scoped to one query. Because `query_bow_candidates` itself already
//!    restricts candidates to strictly older frame IDs, and this module only
//!    ever calls it for a key once every strictly-older key is already
//!    inserted, the accepted-pair set is identical to a batch call over the
//!    same packets (`tests::incremental_matches_equal_batch` below).
//! 3. **The final trajectory comes from the same optimization code path as
//!    the offline mapper.** `optimize_pass` is literally the tail of
//!    `NfrMapper::run_headless` (`build_tracks` -> `setup_opt` -> `optimize`
//!    -> `filter_outliers` -> `optimize`), called via the same public
//!    methods `run_headless` calls; `finalize` runs it one last time so the
//!    reported trajectory's last optimization is not distinguishable from
//!    the offline path's.

use std::collections::BTreeSet;
use std::sync::mpsc::{Receiver, RecvTimeoutError, SyncSender};
use std::time::{Duration, Instant};

use nalgebra::{Matrix3, UnitQuaternion, Vector3};
use thiserror::Error;
use visloc_core::geometry::SE3;

use crate::{
    calibration::BasaltCalibration,
    pyramid::{ImageError, RawU16Image},
    vio::margdata::{MargData, OfImageData},
};

use crate::mapper::{
    features::{
        extract_mapper_features, match_stereo_features, match_temporal_ransac,
        match_temporal_ransac_seeded, match_temporal_stage, mutual_descriptor_matches,
        query_bow_candidates, FeaturePipelineError, MapperImageFeatures, MapperImageId,
    },
    session::{
        NfrMapper, NfrMapperError, NfrMapperFilterReport, NfrMapperHeadlessConfig,
        NfrMapperMatchData, NfrMapperOptimizeReport, NfrMapperResult,
    },
    GlobalBaConfig, MapperConfig, MatchData, OfflineMapperConfig, TimeCamId,
};

/// Errors raised by the online mapper's incremental entry points.
#[derive(Debug, Error)]
pub enum OnlineMapperError {
    #[error("online mapper requires calibration")]
    MissingCalibration,
    #[error("packet ingest failed: {0:?}")]
    Ingest(NfrMapperError),
    #[error("mapper calibration has no camera {camera_id} for image {timestamp_ns}")]
    MissingCamera { timestamp_ns: i64, camera_id: u16 },
    #[error("raw mapper image {timestamp_ns}/{camera_id} is invalid: {source}")]
    Image {
        timestamp_ns: i64,
        camera_id: u16,
        #[source]
        source: ImageError,
    },
    #[error("mapper feature extraction failed for image {timestamp_ns}/{camera_id}: {source}")]
    Extraction {
        timestamp_ns: i64,
        camera_id: u16,
        #[source]
        source: FeaturePipelineError,
    },
    #[error("stereo essential computation failed for timestamp {timestamp_ns}: {source}")]
    Stereo {
        timestamp_ns: i64,
        #[source]
        source: FeaturePipelineError,
    },
}

/// Policy knobs for the online mapper's background optimization trigger.
#[derive(Debug, Clone, Copy)]
pub struct OnlineMapperConfig {
    /// Run a full `build_tracks -> setup_opt -> optimize -> filter ->
    /// optimize` pass after this many newly accepted keyframes.
    pub optimize_every_k: usize,
    /// An accepted temporal match pair whose two frame IDs differ by more
    /// than this many keyframes is treated as a loop closure for the
    /// trigger policy (immediate optimize on the same packet), separate
    /// from the periodic `optimize_every_k` cadence.
    pub loop_gap_keyframes: u64,
    /// Same knobs `NfrMapper::run_headless` uses for `optimize_pass`
    /// (`num_opt_iter`, `outlier_threshold`, `min_num_obs`,
    /// `temporal_seed`). Production runs use `temporal_seed: None`, exactly
    /// like the offline mapper's default `run_headless` call; tests use a
    /// fixed seed for determinism.
    pub headless: NfrMapperHeadlessConfig,
}

impl Default for OnlineMapperConfig {
    fn default() -> Self {
        Self {
            optimize_every_k: 20,
            loop_gap_keyframes: 30,
            headless: NfrMapperHeadlessConfig::default(),
        }
    }
}

/// Per-packet counters returned by [`OnlineNfrMapper::ingest_packet`], used
/// by the online demo's stats/RTF/lag reporting and by the exactness tests.
#[derive(Debug, Clone, Copy, Default)]
pub struct OnlineIngestReport {
    pub processed_timestamp_count: usize,
    pub skipped_ineligible_timestamp_count: usize,
    pub new_key_count: usize,
    pub detect_seconds: f64,
    pub stereo_seconds: f64,
    pub match_seconds: f64,
    pub accepted_temporal_pair_count: usize,
    pub accepted_loop_pair_count: usize,
    pub optimize_triggered: bool,
    pub optimize_seconds: f64,
    /// Bytes retained in `NfrMapper::img_data` immediately after this call
    /// returns. Phase A identified this field as the dominant term in the
    /// offline mapper's RSS; the online demo asserts this stays ~0 (only
    /// non-zero when a packet's images are not yet pose-eligible, which is
    /// not expected on a live VIO stream since a packet's own frame poses
    /// are installed by `add_marg_data` before this method runs).
    pub retained_image_bytes: usize,
}

/// Result of the final `optimize_pass` run when the ingest channel closes.
#[derive(Debug, Clone, PartialEq)]
pub struct OnlineFinalReport {
    pub first_optimize: NfrMapperOptimizeReport,
    pub filter: NfrMapperFilterReport,
    pub second_optimize: NfrMapperOptimizeReport,
    pub result: NfrMapperResult,
    pub trajectory_tum: String,
}

/// Incremental wrapper around [`NfrMapper`]. See the module documentation
/// for the design rationale.
pub struct OnlineNfrMapper {
    mapper: NfrMapper,
    config: OnlineMapperConfig,
    keyframes_since_optimize: usize,
    total_accepted_loops: usize,
    total_optimize_passes: usize,
    /// Assigns each distinct `frame_id` a small monotonically increasing
    /// rank the first time an image for it is detected. `TimeCamId::frame_id`
    /// is the VIO's raw per-frame index (every processed frame, not just
    /// keyframes -- see `vio/margdata.rs`'s `OfImageData::frame_id` doc), so
    /// a fixed *frame_id* gap is not a stable proxy for "how many keyframes
    /// apart": consecutive mapper keyframes can already differ by dozens of
    /// raw frame IDs. `loop_gap_keyframes` in [`OnlineMapperConfig`] is
    /// compared against this rank gap instead.
    keyframe_rank: std::collections::BTreeMap<u64, u64>,
    next_keyframe_rank: u64,
}

impl OnlineNfrMapper {
    pub fn new(
        mapper_config: MapperConfig,
        calibration: BasaltCalibration,
        feature_config: OfflineMapperConfig,
        optimize_config: GlobalBaConfig,
        config: OnlineMapperConfig,
    ) -> Self {
        let mut mapper = NfrMapper::with_calibration(mapper_config, calibration);
        mapper.set_feature_config(feature_config);
        mapper.set_optimize_config(optimize_config);
        Self {
            mapper,
            config,
            keyframes_since_optimize: 0,
            total_accepted_loops: 0,
            total_optimize_passes: 0,
            keyframe_rank: std::collections::BTreeMap::new(),
            next_keyframe_rank: 0,
        }
    }

    pub fn total_accepted_loops(&self) -> usize {
        self.total_accepted_loops
    }

    pub fn total_optimize_passes(&self) -> usize {
        self.total_optimize_passes
    }

    /// Bytes currently retained in the wrapped mapper's raw-image map. Should
    /// be ~0 after every [`Self::ingest_packet`] call on a live stream (see
    /// [`OnlineIngestReport::retained_image_bytes`]).
    pub fn retained_image_bytes(&self) -> usize {
        self.mapper
            .img_data
            .values()
            .flatten()
            .map(|image| image.data.len() * 2)
            .sum()
    }

    /// Direct access to the wrapped mapper, e.g. for `trajectory_tum`/
    /// `result` mid-run diagnostics. Prefer [`Self::finalize`] for the
    /// end-of-sequence output.
    pub fn inner(&self) -> &NfrMapper {
        &self.mapper
    }

    /// Ingest one MargData packet: `add_marg_data`, then incremental
    /// detect/stereo/BoW-match for exactly the images this packet
    /// introduced, then (if the trigger policy fires) a periodic
    /// optimization pass. `seed` is `None` in production (matching
    /// `NfrMapper::match_all`'s wall-clock RANSAC seed) and `Some` only for
    /// deterministic tests.
    pub fn ingest_packet(
        &mut self,
        data: &mut MargData,
        seed: Option<u32>,
    ) -> Result<OnlineIngestReport, OnlineMapperError> {
        self.mapper
            .add_marg_data(data)
            .map_err(OnlineMapperError::Ingest)?;
        let calibration = self
            .mapper
            .calibration
            .clone()
            .ok_or(OnlineMapperError::MissingCalibration)?;

        // Snapshot the timestamps present right now. On a live stream this
        // is exactly the packet just ingested, because every prior call
        // purges every timestamp it successfully processed (rule 1 above);
        // an entry can only still be here if a previous call found it
        // ineligible (no installed pose yet) and left it for retry.
        let timestamps = self.mapper.img_data.keys().copied().collect::<Vec<_>>();

        let mut new_keys = Vec::new();
        let mut processed_timestamp_count = 0;
        let mut skipped_ineligible_timestamp_count = 0;
        let detect_stereo_start = Instant::now();
        let mut stereo_seconds_accum = 0.0;
        for timestamp_ns in timestamps {
            let Some(images) = self.mapper.img_data.get(&timestamp_ns).cloned() else {
                continue;
            };
            let eligible = images
                .iter()
                .any(|image| self.mapper.frame_poses.contains_key(&image.frame_id));
            if !eligible {
                skipped_ineligible_timestamp_count += 1;
                continue;
            }

            for image in &images {
                if !self.mapper.frame_poses.contains_key(&image.frame_id) {
                    continue;
                }
                let key = self.detect_one_image(image, &calibration)?;
                new_keys.push(key);
            }

            let stereo_start = Instant::now();
            self.match_stereo_one_timestamp(timestamp_ns, &images, &calibration)?;
            stereo_seconds_accum += stereo_start.elapsed().as_secs_f64();

            // Rule 1: drop the raw pixels now that detection and stereo
            // matching (which only needs already-extracted features, not
            // pixels) are both done for this timestamp.
            self.mapper.img_data.remove(&timestamp_ns);
            processed_timestamp_count += 1;
        }
        let detect_seconds =
            (detect_stereo_start.elapsed().as_secs_f64() - stereo_seconds_accum).max(0.0);

        // Rule 2: query each newly detected key exactly once, in ascending
        // frame order, against the database of already-inserted keys.
        new_keys.sort();
        let match_start = Instant::now();
        let mut accepted_temporal_pair_count = 0;
        let mut accepted_loop_pair_count = 0;
        for &key in &new_keys {
            let (accepted, loops) = self.match_new_keyframe(key, seed);
            accepted_temporal_pair_count += accepted;
            accepted_loop_pair_count += loops;
        }
        let match_seconds = match_start.elapsed().as_secs_f64();
        self.total_accepted_loops += accepted_loop_pair_count;

        let new_keyframe_count = new_keys
            .iter()
            .map(|key| key.frame_id)
            .collect::<BTreeSet<_>>()
            .len();
        self.keyframes_since_optimize += new_keyframe_count;

        let should_optimize = !new_keys.is_empty()
            && (self.keyframes_since_optimize >= self.config.optimize_every_k
                || accepted_loop_pair_count > 0);
        let mut optimize_seconds = 0.0;
        if should_optimize {
            let start = Instant::now();
            let _ = self.optimize_pass();
            optimize_seconds = start.elapsed().as_secs_f64();
            self.keyframes_since_optimize = 0;
            self.total_optimize_passes += 1;
        }

        Ok(OnlineIngestReport {
            processed_timestamp_count,
            skipped_ineligible_timestamp_count,
            new_key_count: new_keys.len(),
            detect_seconds,
            stereo_seconds: stereo_seconds_accum,
            match_seconds,
            accepted_temporal_pair_count,
            accepted_loop_pair_count,
            optimize_triggered: should_optimize,
            optimize_seconds,
            retained_image_bytes: self.retained_image_bytes(),
        })
    }

    /// Mirrors `NfrMapper::detect_keypoints`'s per-image body
    /// (`session.rs`), scoped to one already pose-eligible image.
    fn detect_one_image(
        &mut self,
        image: &OfImageData,
        calibration: &BasaltCalibration,
    ) -> Result<TimeCamId, OnlineMapperError> {
        let timestamp_ns = image.timestamp_ns;
        let camera_id = image.camera_id;
        let camera = calibration
            .camera(camera_id)
            .ok_or(OnlineMapperError::MissingCamera {
                timestamp_ns,
                camera_id,
            })?;
        let width = usize::try_from(image.width).map_err(|_| OnlineMapperError::Image {
            timestamp_ns,
            camera_id,
            source: ImageError::DimensionOverflow,
        })?;
        let height = usize::try_from(image.height).map_err(|_| OnlineMapperError::Image {
            timestamp_ns,
            camera_id,
            source: ImageError::DimensionOverflow,
        })?;
        let raw = RawU16Image::new(width, height, image.data.clone()).map_err(|source| {
            OnlineMapperError::Image {
                timestamp_ns,
                camera_id,
                source,
            }
        })?;
        let features = extract_mapper_features(&raw, camera, self.mapper.feature_config).map_err(
            |source| OnlineMapperError::Extraction {
                timestamp_ns,
                camera_id,
                source,
            },
        )?;
        let key = TimeCamId::new(image.frame_id, camera_id);
        self.mapper.feature_corners.insert(key, features);
        self.keyframe_rank.entry(image.frame_id).or_insert_with(|| {
            let rank = self.next_keyframe_rank;
            self.next_keyframe_rank += 1;
            rank
        });
        Ok(key)
    }

    /// Mirrors `NfrMapper::match_stereo`'s per-timestamp body, scoped to one
    /// timestamp whose images were just detected.
    fn match_stereo_one_timestamp(
        &mut self,
        timestamp_ns: i64,
        images: &[OfImageData],
        calibration: &BasaltCalibration,
    ) -> Result<(), OnlineMapperError> {
        let camera_0 = calibration
            .camera_to_imu(0)
            .ok_or(OnlineMapperError::MissingCamera {
                timestamp_ns,
                camera_id: 0,
            })?;
        let camera_1 = calibration
            .camera_to_imu(1)
            .ok_or(OnlineMapperError::MissingCamera {
                timestamp_ns,
                camera_id: 1,
            })?;
        let t_0_1 = camera_0.inverse().compose(camera_1);

        let Some(frame_id) = stereo_frame_id(timestamp_ns, images, 0) else {
            return Ok(());
        };
        let left_id = TimeCamId::new(frame_id, 0);
        let right_id = TimeCamId::new(
            stereo_frame_id(timestamp_ns, images, 1).unwrap_or(frame_id),
            1,
        );
        self.mapper
            .feature_corners
            .entry(left_id)
            .or_insert_with(empty_mapper_features);
        self.mapper
            .feature_corners
            .entry(right_id)
            .or_insert_with(empty_mapper_features);

        let stereo = {
            let left = self
                .mapper
                .feature_corners
                .get(&left_id)
                .expect("stereo left feature map materialized");
            let right = self
                .mapper
                .feature_corners
                .get(&right_id)
                .expect("stereo right feature map materialized");
            match_stereo_features(left, right, &t_0_1, self.mapper.feature_config).map_err(
                |source| OnlineMapperError::Stereo {
                    timestamp_ns,
                    source,
                },
            )?
        };
        if stereo.mapper_feature_matches_stored {
            let inliers = stereo
                .essential_inliers
                .into_iter()
                .map(|(left, right)| (left as u64, right as u64))
                .collect::<Vec<_>>();
            self.mapper.feature_match_data.insert(
                (left_id, right_id),
                NfrMapperMatchData {
                    t_i_j: t_0_1,
                    matches: stereo
                        .raw_matches
                        .into_iter()
                        .map(|match_| (match_.left, match_.right))
                        .collect(),
                    inliers: inliers.clone(),
                },
            );
            self.mapper
                .feature_matches
                .insert((left_id, right_id), MatchData::new(inliers));
        }
        Ok(())
    }

    /// Query the BoW database (already-inserted keys only -- see rule 2 in
    /// the module documentation) for one newly detected image and run the
    /// same descriptor-match/RANSAC accept logic `NfrMapper::match_all`'s
    /// inner loop uses. Returns `(accepted_pair_count, loop_pair_count)`.
    fn match_new_keyframe(&mut self, query_id: TimeCamId, seed: Option<u32>) -> (usize, usize) {
        let config = self.mapper.feature_config;
        let mut match_config = config;
        match_config.max_hamming = 70;
        match_config.second_best_ratio = 1.2;

        let Some(query_features) = self.mapper.feature_corners.get(&query_id).cloned() else {
            return (0, 0);
        };
        let candidates = {
            let database = self
                .mapper
                .feature_corners
                .iter()
                .map(|(&id, features)| (MapperImageId::from(id), features))
                .collect::<Vec<_>>();
            query_bow_candidates(
                MapperImageId::from(query_id),
                &query_features,
                &database,
                config.match_window as usize,
            )
        };

        let mut accepted_count = 0;
        let mut loop_count = 0;
        for candidate in candidates {
            if candidate.image.frame_id == query_id.frame_id
                || candidate.score <= config.frames_to_match_threshold
            {
                continue;
            }
            let other_id = TimeCamId::from(candidate.image);
            let (left_id, right_id) = (query_id, other_id);
            let raw_gate_passed = {
                let Some(left) = self.mapper.feature_corners.get(&left_id) else {
                    continue;
                };
                let Some(right) = self.mapper.feature_corners.get(&right_id) else {
                    continue;
                };
                match_temporal_stage(left, right, match_config).raw_match_gate_passed
            };
            if !raw_gate_passed {
                continue;
            }
            let result = {
                let left = self
                    .mapper
                    .feature_corners
                    .get(&left_id)
                    .expect("left key present");
                let right = self
                    .mapper
                    .feature_corners
                    .get(&right_id)
                    .expect("right key present");
                match seed {
                    Some(seed) => match_temporal_ransac_seeded(left, right, match_config, seed),
                    None => match_temporal_ransac(left, right, match_config),
                }
            };
            if result.accepted && !result.refined_inlier_ids.is_empty() {
                let inliers = result.refined_inlier_ids.clone();
                let raw_matches = {
                    let left = self
                        .mapper
                        .feature_corners
                        .get(&left_id)
                        .expect("left key present");
                    let right = self
                        .mapper
                        .feature_corners
                        .get(&right_id)
                        .expect("right key present");
                    mutual_descriptor_matches(&left.descriptors, &right.descriptors, match_config)
                        .into_iter()
                        .map(|match_| (match_.left, match_.right))
                        .collect::<Vec<_>>()
                };
                self.mapper.feature_match_data.insert(
                    (left_id, right_id),
                    NfrMapperMatchData {
                        t_i_j: temporal_result_se3(&result),
                        matches: raw_matches,
                        inliers: inliers.clone(),
                    },
                );
                self.mapper
                    .feature_matches
                    .insert((left_id, right_id), MatchData::new(inliers));
                accepted_count += 1;
                // Compare keyframe *rank* (assignment order), not raw
                // frame_id: frame_id is the VIO's per-frame index and
                // advances on every processed frame, not just keyframes, so
                // a frame_id gap is not a stable "how many keyframes apart"
                // proxy (see the `keyframe_rank` field doc).
                let left_rank = self
                    .keyframe_rank
                    .get(&left_id.frame_id)
                    .copied()
                    .unwrap_or(0);
                let right_rank = self
                    .keyframe_rank
                    .get(&right_id.frame_id)
                    .copied()
                    .unwrap_or(0);
                let gap = left_rank.abs_diff(right_rank);
                if gap > self.config.loop_gap_keyframes {
                    loop_count += 1;
                }
            }
        }
        (accepted_count, loop_count)
    }

    /// Rule 3: the exact tail of `NfrMapper::run_headless`
    /// (`build_tracks -> setup_opt -> optimize -> filter_outliers ->
    /// optimize`), called through the same public methods `run_headless`
    /// itself calls. Used both by the periodic trigger and by
    /// [`Self::finalize`].
    fn optimize_pass(
        &mut self,
    ) -> Result<
        (
            NfrMapperOptimizeReport,
            NfrMapperFilterReport,
            NfrMapperOptimizeReport,
        ),
        OnlineMapperError,
    > {
        let _ = self.mapper.build_tracks();
        let _ = self
            .mapper
            .setup_opt()
            .map_err(|_| OnlineMapperError::MissingCalibration)?;
        let first_optimize = self
            .mapper
            .optimize(self.config.headless.num_opt_iter)
            .map_err(|_| OnlineMapperError::MissingCalibration)?;
        let filter = self
            .mapper
            .filter_outliers(
                self.config.headless.outlier_threshold,
                self.config.headless.min_num_obs,
            )
            .map_err(|_| OnlineMapperError::MissingCalibration)?;
        let second_optimize = self
            .mapper
            .optimize(self.config.headless.num_opt_iter)
            .map_err(|_| OnlineMapperError::MissingCalibration)?;
        Ok((first_optimize, filter, second_optimize))
    }

    /// Run a final `optimize_pass` (unconditionally, regardless of the
    /// periodic trigger's cadence) and return the propagated trajectory.
    /// Call this once the packet channel/source is exhausted.
    pub fn finalize(&mut self) -> Result<OnlineFinalReport, OnlineMapperError> {
        let (first_optimize, filter, second_optimize) = self.optimize_pass()?;
        self.total_optimize_passes += 1;
        Ok(OnlineFinalReport {
            first_optimize,
            filter,
            second_optimize,
            result: self.mapper.result(),
            trajectory_tum: self.mapper.trajectory_tum(),
        })
    }
}

/// Reason a packet source stopped feeding [`run_mapper_thread`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MapperThreadStopReason {
    /// The sender was dropped (the VIO thread finished the sequence).
    ChannelClosed,
}

/// Drive an [`OnlineNfrMapper`] from an `mpsc` channel of in-memory
/// `MargData` packets until the sender side closes, then run
/// [`OnlineNfrMapper::finalize`]. This is the mapper-thread half of the
/// two-thread design in `docs/basalt_online_mapper_design.md` Sec3.1: the
/// VIO thread owns the `Sender`, moves each mapper packet into it (no JSON/
/// base64 round trip -- `EstimatorOutput::marg_data` is already owned), and
/// drops the sender when the sequence ends; this function is meant to run on
/// a dedicated `std::thread` so the VIO thread is never blocked by it.
///
/// `on_packet` is called with each packet's [`OnlineIngestReport`] as it
/// completes (for live queue-lag/RTF instrumentation in the caller); it must
/// not block, since it runs on the mapper thread between packets.
pub fn run_mapper_thread(
    mut mapper: OnlineNfrMapper,
    receiver: Receiver<MargData>,
    seed: Option<u32>,
    mut on_packet: impl FnMut(&OnlineIngestReport),
) -> (
    OnlineNfrMapper,
    MapperThreadStopReason,
    Vec<OnlineMapperError>,
) {
    let mut errors = Vec::new();
    loop {
        match receiver.recv_timeout(Duration::from_secs(3600)) {
            Ok(mut packet) => match mapper.ingest_packet(&mut packet, seed) {
                Ok(report) => on_packet(&report),
                Err(error) => errors.push(error),
            },
            Err(RecvTimeoutError::Disconnected) => {
                return (mapper, MapperThreadStopReason::ChannelClosed, errors)
            }
            Err(RecvTimeoutError::Timeout) => {
                // No packet in an hour: treat like a closed channel rather
                // than spinning forever. A live VIO stream at EuRoC's ~20 Hz
                // camera rate never approaches this.
                return (mapper, MapperThreadStopReason::ChannelClosed, errors);
            }
        }
    }
}

/// Bounded producer handle used by the VIO thread. A small bound (the
/// design doc uses 8) gives natural backpressure: if the mapper falls
/// behind, `send` blocks the VIO thread only as long as it takes the mapper
/// to drain one packet, and the online demo reports that stall as queue lag
/// rather than letting memory grow unboundedly.
pub type MapperPacketSender = SyncSender<MargData>;

fn empty_mapper_features() -> MapperImageFeatures {
    MapperImageFeatures {
        corners: Vec::new(),
        corner_angles: Vec::new(),
        descriptors: Vec::new(),
        rays: Vec::new(),
        hashes: Vec::new(),
        bow_vector: Vec::new(),
    }
}

/// Verbatim copy of `session.rs`'s private helper of the same name (that
/// file is not modified -- see the module documentation).
fn stereo_frame_id(timestamp_ns: i64, images: &[OfImageData], camera_id: u16) -> Option<u64> {
    images
        .iter()
        .find(|image| image.camera_id == camera_id)
        .or_else(|| images.first())
        .map(|image| image.frame_id)
        .or_else(|| u64::try_from(timestamp_ns).ok())
}

/// Verbatim copy of `session.rs`'s private helper of the same name (that
/// file is not modified -- see the module documentation).
fn temporal_result_se3(result: &crate::mapper::features::TemporalRansacResult) -> SE3 {
    let rotation = Matrix3::from_row_slice(&[
        result.refined_model_rotation[0][0],
        result.refined_model_rotation[0][1],
        result.refined_model_rotation[0][2],
        result.refined_model_rotation[1][0],
        result.refined_model_rotation[1][1],
        result.refined_model_rotation[1][2],
        result.refined_model_rotation[2][0],
        result.refined_model_rotation[2][1],
        result.refined_model_rotation[2][2],
    ]);
    SE3::new(
        UnitQuaternion::from_matrix(&rotation),
        Vector3::new(
            result.refined_model_translation[0],
            result.refined_model_translation[1],
            result.refined_model_translation[2],
        ),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::camera::DoubleSphereCamera;
    use crate::mapper::MapperConfig;
    use crate::vio::margdata::{
        AomBlockData, FramePoseData, FrameStateData, MarginalizationTargets, MatrixData,
        MARGDATA_SCHEMA_VERSION_V3,
    };
    use crate::BasaltCalibration;
    use nalgebra::UnitQuaternion;
    use serde_json::Value;

    const TEST_SEED: u32 = 12345;

    // The exact real MH_01 M8a/M8b marginalization-event fixture the pinned
    // `pipelines/basalt/tests/m8f_nfr_session.rs` gate uses, reused here
    // unmodified so this module's tests exercise real AOM/pose data rather
    // than a synthetic identity system. Path is relative to this file
    // (`pipelines/basalt/src/mapper/online.rs`), four levels up to the repo
    // root.
    const FIXTURE: &str =
        include_str!("../../../../benchmarks/basalt/m8a_m8b_mh01_1403636579763555584.json");

    fn array(value: &Value) -> Vec<f64> {
        value
            .as_array()
            .expect("array")
            .iter()
            .map(|item| item.as_f64().expect("finite number"))
            .collect()
    }

    /// Verbatim adaptation of `tests/m8f_nfr_session.rs::fixture_packet`.
    fn fixture_packet() -> MargData {
        let root: Value = serde_json::from_str(FIXTURE).expect("M8a/M8b fixture JSON");
        let input = &root["input"];

        let h_rows = input["abs_H"].as_array().expect("input abs_H");
        let h_cols = h_rows
            .first()
            .and_then(Value::as_array)
            .expect("abs_H row")
            .len();
        let mut h_column_major = Vec::with_capacity(h_cols * h_rows.len());
        for col in 0..h_cols {
            for row in h_rows {
                h_column_major.push(
                    row.as_array()
                        .and_then(|values| values.get(col))
                        .and_then(Value::as_f64)
                        .expect("abs_H entry"),
                );
            }
        }

        let frame_poses = input["frame_poses"]
            .as_array()
            .expect("frame_poses")
            .iter()
            .map(|pose| {
                let q = array(&pose["pose"]["quaternion_xyzw"]);
                let t = array(&pose["pose"]["translation"]);
                FramePoseData {
                    frame_id: pose["id"].as_u64().expect("pose id"),
                    timestamp_ns: pose["t_ns"].as_i64().expect("pose timestamp"),
                    pose: [t[0], t[1], t[2], q[3], q[0], q[1], q[2]],
                    is_keyframe: true,
                }
            })
            .collect::<Vec<_>>();

        let frame_states = input["frame_states"]
            .as_array()
            .expect("frame_states")
            .iter()
            .map(|state| {
                let q = array(&state["pose"]["quaternion_xyzw"]);
                let t = array(&state["pose"]["translation"]);
                let velocity = array(&state["velocity"]);
                let gyro_bias = array(&state["bias_gyro"]);
                let accel_bias = array(&state["bias_accel"]);
                FrameStateData {
                    frame_id: state["id"].as_u64().expect("state id"),
                    timestamp_ns: state["t_ns"].as_i64().expect("state timestamp"),
                    pose: [t[0], t[1], t[2], q[3], q[0], q[1], q[2]],
                    velocity: velocity.try_into().expect("velocity length"),
                    gyro_bias: gyro_bias.try_into().expect("gyro bias length"),
                    accel_bias: accel_bias.try_into().expect("accel bias length"),
                    linearized: state["linearized"].as_bool().expect("linearized"),
                    is_keyframe: input["kfs_all"]
                        .as_array()
                        .expect("kfs_all")
                        .iter()
                        .any(|id| id.as_u64() == Some(state["id"].as_u64().unwrap())),
                    is_latest: false,
                }
            })
            .collect::<Vec<_>>();

        let aom_order = input["aom"]["blocks"]
            .as_array()
            .expect("AOM blocks")
            .iter()
            .map(|block| AomBlockData {
                frame_id: block["id"].as_u64().expect("block id"),
                offset: block["offset"].as_u64().expect("block offset") as usize,
                dof: block["size"].as_u64().expect("block size") as usize,
                kind: if block["size"].as_u64() == Some(6) {
                    "pose".into()
                } else {
                    "state".into()
                },
            })
            .collect::<Vec<_>>();

        MargData {
            schema_version: MARGDATA_SCHEMA_VERSION_V3,
            aom_sqrt_jacobian: MatrixData::new(0, h_cols, Vec::new()).expect("sqrt shape"),
            aom_sqrt_rhs: Vec::new(),
            aom_abs_h: Some(MatrixData::new(h_rows.len(), h_cols, h_column_major).unwrap()),
            aom_abs_b: Some(array(&input["abs_b"])),
            frame_poses,
            frame_states,
            keyframes: input["kfs_all"]
                .as_array()
                .unwrap()
                .iter()
                .map(|id| id.as_u64().unwrap())
                .collect(),
            kf_to_marg: Vec::new(),
            kfs_all: input["kfs_all"]
                .as_array()
                .unwrap()
                .iter()
                .map(|id| id.as_u64().unwrap())
                .collect(),
            kfs_to_marg: input["kfs_to_marg"]
                .as_array()
                .unwrap()
                .iter()
                .map(|id| id.as_u64().unwrap())
                .collect(),
            aom_order,
            marginalization: MarginalizationTargets::default(),
            prior: None,
            row_counts: [0; 4],
            of_observations: Vec::new(),
            of_images: Vec::new(),
            frame_poses_fej: Default::default(),
            frame_states_fej: Default::default(),
            fej_complete: false,
            used_imu: input["use_imu"].as_bool().expect("use_imu"),
            provenance_version: "basalt-0f3b2b52-online-mapper-test-fixture".into(),
        }
    }

    /// Verbatim adaptation of `tests/m8f_nfr_session.rs::feature_calibration`.
    fn feature_calibration() -> BasaltCalibration {
        let camera = DoubleSphereCamera::new(300.0, 300.0, 320.0, 240.0, 0.5, 0.7, 640, 480)
            .expect("camera");
        BasaltCalibration {
            t_imu_cam: vec![
                SE3::identity(),
                SE3::new(UnitQuaternion::identity(), Vector3::new(0.1, 0.0, 0.0)),
            ],
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

    /// Verbatim adaptation of `tests/m8f_nfr_session.rs::feature_image`.
    fn feature_image(frame_id: u64, timestamp_ns: i64, camera_id: u16) -> OfImageData {
        let pixels = (0..128)
            .flat_map(|y| {
                (0..128).map(move |x| {
                    let mut value = (x as u32).wrapping_mul(0x9e37_79b9)
                        ^ (y as u32).wrapping_mul(0x85eb_ca6b)
                        ^ ((x as u32) << 16 | y as u32);
                    value ^= value >> 13;
                    value = value.wrapping_mul(0xc2b2_ae35);
                    (value ^ u32::from(camera_id)).wrapping_shr(16) as u16
                })
            })
            .collect();
        OfImageData::new(frame_id, timestamp_ns, camera_id, 128, 128, pixels).expect("image")
    }

    /// Two eligible stereo frames from the real fixture's own keyframe
    /// list, split across two packets so the equivalence test exercises
    /// more than one `ingest_packet`/`add_marg_data` call -- one keyframe
    /// per packet, matching how the online demo will actually receive
    /// packets from the VIO stream.
    fn two_packets_with_images() -> (MargData, MargData) {
        let base = fixture_packet();
        let first = base.frame_poses.first().expect("fixture pose").clone();
        let second = base
            .frame_poses
            .get(1)
            .expect("second fixture pose")
            .clone();

        let mut packet_one = base.clone();
        packet_one.of_images = vec![
            feature_image(first.frame_id, first.timestamp_ns, 0),
            feature_image(first.frame_id, first.timestamp_ns, 1),
        ];

        let mut packet_two = base;
        packet_two.of_images = vec![
            feature_image(second.frame_id, second.timestamp_ns, 0),
            feature_image(second.frame_id, second.timestamp_ns, 1),
        ];

        (packet_one, packet_two)
    }

    /// Guardrail (1): the incremental matcher must accept exactly the same
    /// pair set as a batch `match_all` call over the same packets, with the
    /// same seed. This is the same real fixture and the same
    /// two-frame/four-key shape `tests/m8f_nfr_session.rs::
    /// pinned_mh01_packet_match_all_preserves_bow_order_and_payload`
    /// already proves produces stereo *and* accepted temporal pairs (not a
    /// vacuous all-empty comparison).
    #[test]
    fn incremental_matches_equal_batch() {
        let (packet_one, packet_two) = two_packets_with_images();
        let calibration = feature_calibration();

        // Batch reference: ingest both packets, then detect/stereo/match_all
        // once over the fully accumulated state, exactly like the offline
        // demo (`run_headless`'s frontend order).
        let mut batch = NfrMapper::with_calibration(MapperConfig::default(), calibration.clone());
        let mut p1 = packet_one.clone();
        let mut p2 = packet_two.clone();
        batch.add_marg_data(&mut p1).expect("batch ingest 1");
        batch.add_marg_data(&mut p2).expect("batch ingest 2");
        batch.detect_keypoints().expect("batch detect");
        batch.match_stereo().expect("batch stereo");
        let batch_report = batch.match_all_seeded(TEST_SEED);
        assert!(
            batch_report.accepted_pair_count > 0,
            "fixture must produce at least one accepted pair for this test to be meaningful"
        );

        // Incremental: one packet at a time through OnlineNfrMapper.
        let mut online = OnlineNfrMapper::new(
            MapperConfig::default(),
            calibration,
            OfflineMapperConfig::default(),
            GlobalBaConfig::default(),
            OnlineMapperConfig {
                // Isolate the match-graph comparison from the periodic BA
                // trigger for this test.
                optimize_every_k: usize::MAX,
                ..OnlineMapperConfig::default()
            },
        );
        let mut p1 = packet_one;
        let mut p2 = packet_two;
        online
            .ingest_packet(&mut p1, Some(TEST_SEED))
            .expect("online ingest 1");
        online
            .ingest_packet(&mut p2, Some(TEST_SEED))
            .expect("online ingest 2");

        assert_eq!(
            batch.feature_matches, online.mapper.feature_matches,
            "incremental match graph must equal the batch match graph"
        );
        assert_eq!(
            batch.feature_corners.len(),
            online.mapper.feature_corners.len(),
            "incremental detection must produce the same key count as batch"
        );
    }

    /// Guardrail (2): raw image bytes must not accumulate across packets.
    #[test]
    fn retained_image_bytes_stay_bounded() {
        let (mut packet_one, mut packet_two) = two_packets_with_images();
        let mut online = OnlineNfrMapper::new(
            MapperConfig::default(),
            feature_calibration(),
            OfflineMapperConfig::default(),
            GlobalBaConfig::default(),
            OnlineMapperConfig::default(),
        );
        for packet in [&mut packet_one, &mut packet_two] {
            let report = online
                .ingest_packet(packet, Some(TEST_SEED))
                .expect("online ingest");
            assert_eq!(
                report.skipped_ineligible_timestamp_count, 0,
                "fixture packets are expected to be immediately pose-eligible"
            );
            assert_eq!(
                report.retained_image_bytes, 0,
                "raw image bytes must be dropped in the same call that detects them"
            );
            assert_eq!(online.retained_image_bytes(), 0);
        }
    }

    /// The channel-close path exists and runs the tail exactly once even
    /// with zero packets (degenerate but must not panic).
    #[test]
    fn mapper_thread_runs_finalize_on_channel_close() {
        let online = OnlineNfrMapper::new(
            MapperConfig::default(),
            feature_calibration(),
            OfflineMapperConfig::default(),
            GlobalBaConfig::default(),
            OnlineMapperConfig::default(),
        );
        let (sender, receiver) = std::sync::mpsc::sync_channel::<MargData>(1);
        drop(sender);
        let (mut online, reason, errors) =
            run_mapper_thread(online, receiver, Some(TEST_SEED), |_report| {});
        assert_eq!(reason, MapperThreadStopReason::ChannelClosed);
        assert!(errors.is_empty());
        // No packets were ingested, so setup_opt has no tracks and
        // optimize_pass runs on an empty problem -- finalize must still not
        // panic.
        let _ = online.finalize();
    }
}
