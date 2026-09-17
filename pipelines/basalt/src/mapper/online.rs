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
use std::sync::OnceLock;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use nalgebra::{Matrix3, UnitQuaternion, Vector3};
use rayon::prelude::*;
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
        query_bow_candidates, BowQueryCandidate, FeaturePipelineError, MapperImageFeatures,
        MapperImageId,
    },
    session::{
        NfrMapper, NfrMapperError, NfrMapperFilterReport, NfrMapperHeadlessConfig,
        NfrMapperMatchData, NfrMapperOptimizeReport, NfrMapperResult,
    },
    GlobalBaConfig, GlobalBaOptimizerState, MapperConfig, MatchData, OfflineMapperConfig,
    TimeCamId,
};

/// Diagnostic-only fine-grained stage trace, gated by the
/// `BASALT_ONLINE_MAPPER_TRACE` environment variable so it costs nothing
/// (beyond one `OnceLock` read) on every production call site. Added while
/// investigating a `--pipeline` mapper-thread stall (see
/// `docs/basalt_online_mapper_pipelined_stall.md` if present, or the fix
/// commit that introduced this): the per-packet counters in
/// [`OnlineIngestReport`] only ever surface *after* a whole `ingest_packet`
/// call returns, which gives no visibility into which sub-stage a hung call
/// is stuck in. Kept permanently since it is zero-cost when unset and cheap
/// to reach for the next time a live run needs the same visibility.
fn mapper_trace_enabled() -> bool {
    static ENABLED: OnceLock<bool> = OnceLock::new();
    *ENABLED.get_or_init(|| std::env::var_os("BASALT_ONLINE_MAPPER_TRACE").is_some())
}

macro_rules! mapper_trace {
    ($($arg:tt)*) => {
        if mapper_trace_enabled() {
            eprintln!("[mapper-trace] {}", format!($($arg)*));
        }
    };
}

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
///
/// A full warm-started global optimize is superlinear in pose/landmark
/// count (dense pose-pose Hessian, `mod.rs::linearize_vision`), so running
/// one at full iteration count on every `optimize_every_k` keyframes is
/// itself superlinear in total sequence length. The periodic trigger below
/// is deliberately cheap: a small `periodic_iterations` budget, a large
/// `optimize_every_k`, and only the final pass at channel close
/// ([`OnlineNfrMapper::finalize`]) runs the full unbounded
/// `headless.num_opt_iter` budget -- literally `run_headless`'s tail, not a
/// capped approximation of it.
#[derive(Debug, Clone, Copy)]
pub struct OnlineMapperConfig {
    /// Run a periodic `build_tracks -> setup_opt -> optimize ->
    /// filter -> optimize` pass (capped at `periodic_iterations` LM
    /// iterations per `optimize` call) after this many newly accepted
    /// keyframes.
    pub optimize_every_k: usize,
    /// LM iteration budget for each *periodic* optimize call (both the
    /// pre-filter and post-filter `optimize` calls in one pass). Kept small
    /// (default 4) because the periodic trigger's job is to keep the map
    /// roughly consistent between loop events, not to fully converge every
    /// time -- full convergence happens once, in `finalize`, which uses
    /// `headless.num_opt_iter` instead.
    pub periodic_iterations: usize,
    /// An accepted temporal match pair whose two frame IDs differ by more
    /// than this many keyframes is classified as a loop closure (reported
    /// via `OnlineIngestReport::accepted_loop_pair_count`) and, like every
    /// accepted pair, is immediately in `feature_matches`. It does *not*
    /// force its own optimize trigger: the rate limit in `ingest_packet`
    /// (see `last_optimize_started_at`'s field doc) picks it up whenever
    /// the next `optimize_every_k`-or-cooldown trigger fires, so a
    /// loop-rich revisited segment cannot make every single packet start
    /// its own background job.
    pub loop_gap_keyframes: u64,
    /// Candidates considered per new keyframe query in
    /// `match_new_keyframe`, i.e. the `num_results` truncation passed to
    /// `query_bow_candidates` -- independent of (and by default smaller
    /// than) `OfflineMapperConfig::match_window` (30), which batch
    /// `NfrMapper::match_all` still uses unchanged. This is a genuine
    /// semantic difference from the offline/batch path, not merely a
    /// performance tweak: RANSAC (bounded by this many attempts per query)
    /// dominated match_new_keyframe's cost on a real MH_01 run (up to
    /// ~2.7s/packet at match_window=30, well above the real-time budget a
    /// live VIO stream needs from the mapper), and a query's true loop/
    /// temporal-continuation candidates are almost always within its
    /// highest-scoring few. `tests::inverted_index_candidates_equal_full_scan`
    /// and `tests::incremental_matches_equal_batch` compare against a batch
    /// call truncated to this same value, not the offline default, so
    /// exactness is still proven at whatever `k` is configured -- just not
    /// against the *offline mapper's own* candidate count.
    pub match_top_k: usize,
    /// Knobs `NfrMapper::run_headless` uses for `finalize`'s pass
    /// (`num_opt_iter`, `outlier_threshold`, `min_num_obs`,
    /// `temporal_seed`); `num_opt_iter` is unused by the periodic trigger,
    /// which uses `periodic_iterations` instead. Production runs use
    /// `temporal_seed: None`, exactly like the offline mapper's default
    /// `run_headless` call; tests use a fixed seed for determinism.
    pub headless: NfrMapperHeadlessConfig,
}

impl Default for OnlineMapperConfig {
    fn default() -> Self {
        Self {
            optimize_every_k: 100,
            periodic_iterations: 4,
            loop_gap_keyframes: 30,
            match_top_k: 5,
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
    /// A new background optimize job was *started* this packet (see
    /// [`BackgroundOptimizeBreakdown`] -- ingestion is not blocked by it;
    /// its result is merged, and reported via `optimize_merge`, on a later
    /// packet once the background thread finishes).
    pub optimize_triggered: bool,
    /// A previously started background optimize job *finished and was
    /// merged* into the live mapper state during this packet's call. `None`
    /// on every packet where no job happened to complete.
    pub optimize_merge: Option<BackgroundOptimizeBreakdown>,
    /// Bytes retained in `NfrMapper::img_data` immediately after this call
    /// returns. Phase A identified this field as the dominant term in the
    /// offline mapper's RSS; the online demo asserts this stays ~0 (only
    /// non-zero when a packet's images are not yet pose-eligible, which is
    /// not expected on a live VIO stream since a packet's own frame poses
    /// are installed by `add_marg_data` before this method runs).
    pub retained_image_bytes: usize,
}

/// Per-stage timing for one completed background optimize job (see rule 3
/// in [`OnlineNfrMapper`]'s trigger policy): where a periodic pass's wall
/// time actually goes. `build_tracks`/`setup_opt` are full rebuilds over the
/// whole accumulated match/track graph on every call (Sec2 of the design
/// doc -- there is no incremental union-find/triangulation port), so their
/// cost grows with total sequence length regardless of the LM iteration
/// cap; `optimize1`/`optimize2` are bounded by `periodic_iterations`.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct BackgroundOptimizeBreakdown {
    pub build_tracks_seconds: f64,
    pub setup_opt_seconds: f64,
    pub optimize1_seconds: f64,
    pub filter_seconds: f64,
    pub optimize2_seconds: f64,
    pub total_seconds: f64,
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
    /// Inverted HashBoW index: hash bucket -> images whose `bow_vector`
    /// contains that hash. `query_bow_candidates` (batch `match_all`'s own
    /// primitive) only ever scores a database image as a candidate when it
    /// shares at least one hash bucket with the query (`shared` in its
    /// source); every other database image is skipped regardless of the
    /// database's size. So querying only `hash_index`'s union of
    /// hash-bucket members for the query's own hashes -- instead of every
    /// detected image -- is an *exact* reduction of the candidate set
    /// `query_bow_candidates` scans, not an approximation: see
    /// `tests::inverted_index_candidates_equal_full_scan` below.
    hash_index: std::collections::BTreeMap<u32, BTreeSet<TimeCamId>>,
    /// At most one background optimize job in flight at a time (rule 3):
    /// `ingest_packet` never blocks on it, but also never starts a second
    /// one while the first is still running, so the mapper thread's own
    /// ingestion work (detect/stereo/match, all cheap) is what determines
    /// whether the mapper keeps up with the VIO, not a growing backlog of
    /// stacked optimize jobs.
    pending_optimizer: Option<JoinHandle<BackgroundOptimizeResult>>,
    /// When the currently-pending (or most recently completed) job started,
    /// and how long the most recently *completed* one took. Used by the
    /// trigger's rate limit: don't start another job until at least
    /// `last_optimize_duration * 2` has elapsed since the last one started,
    /// even if loop pairs keep arriving in the meantime (they still get
    /// recorded as accepted match-graph factors immediately; they just
    /// don't each force their own optimize job the way the first cut of
    /// this trigger policy did -- confirmed by an MH_01 run where a
    /// loop-rich revisited corridor made `optimize_triggered` fire on
    /// nearly every packet, each costing 80-130s, because every single
    /// accepted loop pair triggered its own full rebuild).
    last_optimize_started_at: Option<Instant>,
    last_optimize_duration: Duration,
    total_optimize_triggers: usize,
}

/// Owned inputs/outputs of one background optimize job (rule 3). Computed
/// entirely on a clone of the mapper state taken at trigger time -- the
/// live `NfrMapper` (and therefore `ingest_packet`) is never touched while
/// this runs.
struct BackgroundOptimizeResult {
    poses: std::collections::BTreeMap<u64, SE3>,
    optimizer_state: GlobalBaOptimizerState,
    breakdown: BackgroundOptimizeBreakdown,
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
            hash_index: std::collections::BTreeMap::new(),
            pending_optimizer: None,
            last_optimize_started_at: None,
            last_optimize_duration: Duration::ZERO,
            total_optimize_triggers: 0,
        }
    }

    pub fn total_optimize_triggers(&self) -> usize {
        self.total_optimize_triggers
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
        mapper_trace!("ingest_packet: enter, images={}", data.of_images.len());
        // Non-blocking: pick up a background optimize job's result if one
        // finished since the last packet (rule 3). Ingestion below proceeds
        // immediately either way.
        mapper_trace!("ingest_packet: poll_pending_optimizer start");
        let optimize_merge = self.poll_pending_optimizer();
        mapper_trace!(
            "ingest_packet: poll_pending_optimizer done merged={}",
            optimize_merge.is_some()
        );

        mapper_trace!("ingest_packet: add_marg_data start");
        self.mapper
            .add_marg_data(data)
            .map_err(OnlineMapperError::Ingest)?;
        mapper_trace!("ingest_packet: add_marg_data done");
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
        mapper_trace!(
            "ingest_packet: detect/stereo loop start, timestamps={}",
            timestamps.len()
        );

        // Phase 1 (sequential, cheap): snapshot every eligible timestamp's
        // images and collect the (still-empty) set of genuinely new keys
        // each one needs detected -- the same overlap-skip filter as
        // before (a MargData packet's `of_images` is the marginalization
        // event's whole AOM window, up to 16 images, and consecutive
        // packets' windows overlap heavily; re-detecting an already-known
        // key would reproduce batch `match_all`'s exact re-querying cost,
        // per the fix note this replaced). This pass only reads mapper
        // state, so it stays a plain sequential loop; the expensive part
        // (phase 2) is what moves to rayon.
        struct EligibleTimestamp {
            timestamp_ns: i64,
            images: Vec<OfImageData>,
            new_keys: Vec<TimeCamId>,
        }
        let mut eligible_timestamps = Vec::new();
        let mut jobs: Vec<(TimeCamId, usize, usize)> = Vec::new();
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
            let timestamp_index = eligible_timestamps.len();
            for (image_index, image) in images.iter().enumerate() {
                if !self.mapper.frame_poses.contains_key(&image.frame_id) {
                    continue;
                }
                let key = TimeCamId::new(image.frame_id, image.camera_id);
                if self.mapper.feature_corners.contains_key(&key) {
                    continue;
                }
                jobs.push((key, timestamp_index, image_index));
            }
            eligible_timestamps.push(EligibleTimestamp {
                timestamp_ns,
                images,
                new_keys: Vec::new(),
            });
        }

        // Phase 2 (parallel): pure per-image feature extraction (FAST
        // corners, descriptors, BoW hashing) on rayon's shared pool --
        // dominated detect_seconds at ~0.1s/packet on a full MH_01
        // --pipeline run before this change; every job reads only its own
        // raw pixel buffer, so this is embarrassingly parallel regardless
        // of which timestamp or camera it belongs to.
        mapper_trace!("ingest_packet: parallel detect start, jobs={}", jobs.len());
        let extracted: Vec<Result<(TimeCamId, usize, MapperImageFeatures), OnlineMapperError>> =
            jobs.par_iter()
                .map(|&(key, timestamp_index, image_index)| {
                    let image = &eligible_timestamps[timestamp_index].images[image_index];
                    extract_one_image_features(image, &calibration, self.mapper.feature_config)
                        .map(|features| (key, timestamp_index, features))
                })
                .collect();
        mapper_trace!("ingest_packet: parallel detect done");

        // Phase 3 (sequential): apply results in original job order,
        // propagating the first error exactly like the prior sequential
        // loop's `?` did -- an image after a failing one is never
        // inserted, matching the old early-return's observable state even
        // though every job was already computed in parallel.
        for result in extracted {
            let (key, timestamp_index, features) = result?;
            self.insert_detected_image(key, key.frame_id, features);
            new_keys.push(key);
            eligible_timestamps[timestamp_index].new_keys.push(key);
        }

        // Phase 4 (sequential): stereo matching + cleanup, unchanged from
        // the prior per-timestamp behavior -- stereo needs both of a
        // timestamp's images already detected (from this call or an
        // earlier one), and this stays cheap (~4ms/packet total) so it is
        // not itself a parallelization target.
        for entry in eligible_timestamps {
            // Same overlap reasoning as detection: if both of this
            // timestamp's camera images were already matched by an earlier
            // packet, re-running stereo matching would just recompute an
            // identical (deterministic) result for free-standing cost.
            if !entry.new_keys.is_empty() {
                mapper_trace!(
                    "ingest_packet: match_stereo_one_timestamp start ts={}",
                    entry.timestamp_ns
                );
                let stereo_start = Instant::now();
                self.match_stereo_one_timestamp(entry.timestamp_ns, &entry.images, &calibration)?;
                stereo_seconds_accum += stereo_start.elapsed().as_secs_f64();
                mapper_trace!(
                    "ingest_packet: match_stereo_one_timestamp done ts={}",
                    entry.timestamp_ns
                );
            }

            // Rule 1: drop the raw pixels now that detection and stereo
            // matching (which only needs already-extracted features, not
            // pixels) are both done for this timestamp.
            self.mapper.img_data.remove(&entry.timestamp_ns);
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
        mapper_trace!(
            "ingest_packet: match loop start, new_keys={}",
            new_keys.len()
        );
        for &key in &new_keys {
            mapper_trace!("ingest_packet: match_new_keyframe start key={key:?}");
            let (accepted, loops) = self.match_new_keyframe(key, seed);
            mapper_trace!("ingest_packet: match_new_keyframe done key={key:?} accepted={accepted} loops={loops}");
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

        // Rate limit (rule 1): a keyframe-count threshold alone re-triggers
        // on literally every packet through a loop-rich revisited segment,
        // since each packet's own newly accepted loop pair satisfies
        // `accepted_loop_pair_count > 0` independently -- confirmed by an
        // MH_01 run where that fired ~120 times in a row at 80-130s each.
        // Loops accepted while a job is running or cooling down still land
        // in `feature_matches`/`feature_match_data` immediately (rule 2 is
        // unaffected); they are simply picked up by whichever optimize job
        // runs next instead of each forcing their own.
        let cooldown_elapsed = self.last_optimize_started_at.is_none_or(|started| {
            started.elapsed() >= self.last_optimize_duration.saturating_mul(2)
        });
        let should_trigger = self.pending_optimizer.is_none()
            && !new_keys.is_empty()
            && self.keyframes_since_optimize >= self.config.optimize_every_k
            && cooldown_elapsed;
        if should_trigger {
            mapper_trace!("ingest_packet: spawn_background_optimize start");
            self.spawn_background_optimize();
            mapper_trace!("ingest_packet: spawn_background_optimize done");
            self.keyframes_since_optimize = 0;
            self.total_optimize_triggers += 1;
        }

        mapper_trace!("ingest_packet: exit");
        Ok(OnlineIngestReport {
            processed_timestamp_count,
            skipped_ineligible_timestamp_count,
            new_key_count: new_keys.len(),
            detect_seconds,
            stereo_seconds: stereo_seconds_accum,
            match_seconds,
            accepted_temporal_pair_count,
            accepted_loop_pair_count,
            optimize_triggered: should_trigger,
            optimize_merge,
            retained_image_bytes: self.retained_image_bytes(),
        })
    }

    /// Mirrors `NfrMapper::detect_keypoints`'s per-image body
    /// (`session.rs`), scoped to one already pose-eligible image: runs the
    /// pure [`extract_one_image_features`] step and applies its result to
    /// mapper state. Detection itself (the expensive part) has moved to
    /// `ingest_packet`'s own parallel phase; this method now only does the
    /// cheap, must-stay-sequential bookkeeping insert.
    fn insert_detected_image(
        &mut self,
        key: TimeCamId,
        frame_id: u64,
        features: MapperImageFeatures,
    ) {
        for entry in &features.bow_vector {
            self.hash_index.entry(entry.hash).or_default().insert(key);
        }
        self.mapper.feature_corners.insert(key, features);
        self.keyframe_rank.entry(frame_id).or_insert_with(|| {
            let rank = self.next_keyframe_rank;
            self.next_keyframe_rank += 1;
            rank
        });
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
    /// Build the candidate list for one query using the inverted
    /// `hash_index` instead of a full scan of every detected image.
    ///
    /// Exact reduction, not an approximation: `query_bow_candidates` (the
    /// same primitive batch `NfrMapper::match_all` uses) only ever scores a
    /// database entry when it shares a hash bucket with the query (see
    /// `hash_index`'s field doc), so restricting the scanned set to the
    /// union of this query's own hash buckets' members is guaranteed to
    /// include every image that could possibly score, and excludes only
    /// images that would have scored zero/unshared anyway. Exposed as its
    /// own method (rather than inlined in `match_new_keyframe`) so
    /// `tests::inverted_index_candidates_equal_full_scan` can compare it
    /// directly against a full-scan call with the same inputs.
    fn bow_candidates_via_index(
        &self,
        query_id: TimeCamId,
        query_features: &MapperImageFeatures,
        match_window: usize,
    ) -> Vec<BowQueryCandidate> {
        let candidate_ids = query_features
            .bow_vector
            .iter()
            .filter_map(|entry| self.hash_index.get(&entry.hash))
            .flatten()
            .copied()
            .collect::<BTreeSet<TimeCamId>>();
        let database = candidate_ids
            .iter()
            .filter_map(|&id| {
                self.mapper
                    .feature_corners
                    .get(&id)
                    .map(|features| (MapperImageId::from(id), features))
            })
            .collect::<Vec<_>>();
        query_bow_candidates(
            MapperImageId::from(query_id),
            query_features,
            &database,
            match_window,
        )
    }

    fn match_new_keyframe(&mut self, query_id: TimeCamId, seed: Option<u32>) -> (usize, usize) {
        let config = self.mapper.feature_config;
        let mut match_config = config;
        match_config.max_hamming = 70;
        match_config.second_best_ratio = 1.2;

        let Some(query_features) = self.mapper.feature_corners.get(&query_id).cloned() else {
            return (0, 0);
        };
        // Deliberately `self.config.match_top_k`, not
        // `config.match_window` (which batch `match_all` still uses
        // unchanged) -- see `OnlineMapperConfig::match_top_k`'s field doc.
        let candidates =
            self.bow_candidates_via_index(query_id, &query_features, self.config.match_top_k);
        mapper_trace!(
            "match_new_keyframe: query={query_id:?} candidates={}",
            candidates.len()
        );

        // Per-candidate descriptor matching + 5-point RANSAC dominates
        // per-packet wall time (~0.5s/packet of the ~2.4s total measured on
        // a full MH_01 --pipeline run, vs ~0.1s/packet for detection) and is
        // embarrassingly parallel: each candidate's outcome depends only on
        // `feature_corners` (read-only here -- no candidate mutates it) and
        // its own gate/RANSAC result, never on another candidate's outcome.
        // Run every candidate's match+RANSAC concurrently on the shared
        // rayon pool the demo's `--threads` flag sizes, collecting outcomes
        // into a `Vec` (an `IndexedParallelIterator`, so `collect` preserves
        // the original candidate order regardless of completion order), and
        // apply the accepted ones -- the `feature_match_data`/
        // `feature_matches` inserts and the loop-count bookkeeping -- back
        // on `self` sequentially, in that same original order. This keeps
        // the accepted-pair set and insertion order bit-identical to the
        // fully sequential version; only the wall-clock cost of getting
        // there changes. (An earlier revision tried a small dedicated pool
        // here on the theory that sharing the global pool would
        // oversubscribe VIO's own concurrent frontend/estimator work; a
        // second full-MH_01 measurement showed that variant with *both*
        // worse per-packet mapper cost and worse whole-system wall time,
        // consistent with rising host contention across the measurement
        // session rather than a real effect of pool choice, so the simpler
        // shared-pool version is what's kept.)
        let feature_corners = &self.mapper.feature_corners;
        let outcomes: Vec<Option<CandidateMatchOutcome>> = candidates
            .par_iter()
            .map(|candidate| {
                evaluate_match_candidate(
                    feature_corners,
                    query_id,
                    candidate,
                    config,
                    match_config,
                    seed,
                )
            })
            .collect();

        let mut accepted_count = 0;
        let mut loop_count = 0;
        for outcome in outcomes.into_iter().flatten() {
            let CandidateMatchOutcome {
                left_id,
                right_id,
                t_i_j,
                raw_matches,
                inliers,
            } = outcome;
            self.mapper.feature_match_data.insert(
                (left_id, right_id),
                NfrMapperMatchData {
                    t_i_j,
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
        mapper_trace!(
            "match_new_keyframe: query={query_id:?} accepted={accepted_count} loops={loop_count}"
        );
        (accepted_count, loop_count)
    }

    /// Rule 3: the exact tail of `NfrMapper::run_headless`
    /// (`build_tracks -> setup_opt -> optimize -> filter_outliers ->
    /// optimize`), called through the same public methods `run_headless`
    /// itself calls. `num_opt_iter` is the LM iteration budget passed to
    /// both `optimize` calls: the periodic trigger passes
    /// `config.periodic_iterations` (small, warm-started off the persistent
    /// `GlobalBaOptimizerState`), while [`Self::finalize`] passes
    /// `config.headless.num_opt_iter` (the same unbounded budget
    /// `run_headless` itself uses) so the shipped trajectory's last
    /// optimization is not a capped approximation of the offline path's.
    fn optimize_pass(
        &mut self,
        num_opt_iter: usize,
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
            .optimize(num_opt_iter)
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
            .optimize(num_opt_iter)
            .map_err(|_| OnlineMapperError::MissingCalibration)?;
        Ok((first_optimize, filter, second_optimize))
    }

    /// Run a final, full-budget `optimize_pass` (unconditionally, regardless
    /// of the periodic trigger's cadence) and return the propagated
    /// trajectory. Call this once the packet channel/source is exhausted.
    ///
    /// Joins any in-flight background optimize job first (rule 3) so the
    /// final pass warm-starts from the freshest `GlobalBaOptimizerState`
    /// and does not race a background merge landing mid-solve; this is the
    /// one place `OnlineNfrMapper` blocks on the background thread, and it
    /// only does so once, at the natural end of the run.
    pub fn finalize(&mut self) -> Result<OnlineFinalReport, OnlineMapperError> {
        mapper_trace!(
            "finalize: enter, pending_optimizer={}",
            self.pending_optimizer.is_some()
        );
        if let Some(handle) = self.pending_optimizer.take() {
            mapper_trace!("finalize: joining in-flight background optimize job");
            let result = handle.join().expect("background optimize thread panicked");
            mapper_trace!("finalize: background optimize job joined");
            self.merge_background_result(result);
        }
        mapper_trace!("finalize: optimize_pass start");
        let (first_optimize, filter, second_optimize) =
            self.optimize_pass(self.config.headless.num_opt_iter)?;
        mapper_trace!("finalize: optimize_pass done");
        self.total_optimize_passes += 1;
        Ok(OnlineFinalReport {
            first_optimize,
            filter,
            second_optimize,
            result: self.mapper.result(),
            trajectory_tum: self.mapper.trajectory_tum(),
        })
    }

    /// Non-blocking: if the in-flight background optimize job (if any) has
    /// finished, join and merge it, returning its breakdown. Called at the
    /// top of every [`Self::ingest_packet`] so a completed job is merged
    /// promptly without ever making ingestion wait for it.
    fn poll_pending_optimizer(&mut self) -> Option<BackgroundOptimizeBreakdown> {
        let finished = self
            .pending_optimizer
            .as_ref()
            .is_some_and(JoinHandle::is_finished);
        if !finished {
            return None;
        }
        let handle = self.pending_optimizer.take().expect("checked Some above");
        let result = handle.join().expect("background optimize thread panicked");
        let breakdown = result.breakdown;
        self.merge_background_result(result);
        Some(breakdown)
    }

    /// Apply a finished background job's result to the live mapper state.
    /// Only poses present in the snapshot are overwritten -- a keyframe
    /// detected *after* the snapshot was taken keeps its live incremental
    /// pose estimate rather than being touched by a solve that never saw
    /// it. `optimizer_state` (lambda/lambda_vee) is always taken from the
    /// background result so the next job's warm start continues from it.
    fn merge_background_result(&mut self, result: BackgroundOptimizeResult) {
        for (frame_id, pose) in result.poses {
            self.mapper.frame_poses.insert(frame_id, pose);
        }
        self.mapper.optimizer_state = result.optimizer_state;
        self.last_optimize_duration = Duration::from_secs_f64(result.breakdown.total_seconds);
        self.total_optimize_passes += 1;
    }

    /// Rule 3: clone the current mapper state and run one periodic
    /// `optimize_pass`-equivalent sequence on the clone, on a dedicated
    /// thread, so ingestion (`detect`/`stereo`/`match_new_keyframe`, the
    /// cheap per-packet work) is never blocked by it. The clone is a
    /// snapshot: it does not observe any packet ingested after this call
    /// returns, and the live mapper is not touched until
    /// [`Self::poll_pending_optimizer`] or [`Self::finalize`] merges the
    /// result back.
    fn spawn_background_optimize(&mut self) {
        let mut snapshot = self.mapper.clone();
        let periodic_iterations = self.config.periodic_iterations;
        let outlier_threshold = self.config.headless.outlier_threshold;
        let min_num_obs = self.config.headless.min_num_obs;
        self.last_optimize_started_at = Some(Instant::now());
        self.pending_optimizer = Some(std::thread::spawn(move || {
            let total_start = Instant::now();
            let start = Instant::now();
            let _ = snapshot.build_tracks();
            let build_tracks_seconds = start.elapsed().as_secs_f64();

            let start = Instant::now();
            // Calibration is always present on an online mapper (checked in
            // `OnlineNfrMapper::new`'s callers); a missing-calibration error
            // here would mean the snapshot itself is malformed, which
            // `ingest_packet` already could not have produced.
            let _ = snapshot.setup_opt();
            let setup_opt_seconds = start.elapsed().as_secs_f64();

            let start = Instant::now();
            let _ = snapshot.optimize(periodic_iterations);
            let optimize1_seconds = start.elapsed().as_secs_f64();

            let start = Instant::now();
            let _ = snapshot.filter_outliers(outlier_threshold, min_num_obs);
            let filter_seconds = start.elapsed().as_secs_f64();

            let start = Instant::now();
            let _ = snapshot.optimize(periodic_iterations);
            let optimize2_seconds = start.elapsed().as_secs_f64();

            BackgroundOptimizeResult {
                poses: snapshot.frame_poses,
                optimizer_state: snapshot.optimizer_state,
                breakdown: BackgroundOptimizeBreakdown {
                    build_tracks_seconds,
                    setup_opt_seconds,
                    optimize1_seconds,
                    filter_seconds,
                    optimize2_seconds,
                    total_seconds: total_start.elapsed().as_secs_f64(),
                },
            }
        }));
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

/// Producer handle used by the VIO thread.
///
/// Bounded (`SyncSender`), not a plain `Sender`. An earlier revision used an
/// unbounded channel on the theory that a queued `MargData` is cheap to hold
/// -- it is consumed (features extracted, raw pixels dropped) in the same
/// `ingest_packet` call that receives it, so *processed* queue growth only
/// costs keypoints/descriptors, not images (Sec1.2 of the design doc). That
/// reasoning misses the *unprocessed* case: a packet sitting in the channel
/// before `ingest_packet` ever sees it still carries its full `of_images`
/// raw pixel buffers, and `--pipeline`'s frontend/estimator overlap (PR
/// #153) makes the VIO producer meaningfully faster than the serial path --
/// fast enough to run many hundreds of packets ahead of a mapper that is
/// still working through an earlier, larger match graph. That gap has no
/// ceiling with an unbounded channel: multi-GB backlog growth was observed
/// on a full-MH_01 `--pipeline` run whose mapper thread also happened to hit
/// a slow packet (production `seed: None` -- see `OnlineIngestReport`'s
/// docs -- makes per-packet cost timing-dependent, and `--pipeline`'s higher
/// throughput changes that timing relative to the serial path), and the
/// resulting memory pressure turned a transient slow packet into a
/// multi-hour apparent stall (100% CPU, no forward progress) rather than a
/// bounded delay. A bounded channel keeps the "VIO never blocks on a
/// keeping-pace mapper" property from rule (1) above (the earlier "a small
/// bound stalled the VIO thread for minutes" regression was measured at a
/// bound small enough to contend during ordinary operation, not one sized
/// to the sliding-AOM-window packet rate this channel actually sees) while
/// putting a hard ceiling on backlog size, and therefore on backlog memory,
/// so a slow mapper packet degrades into bounded VIO backpressure instead of
/// unbounded queue growth. The online demo still reports queue depth/lag as
/// an honest diagnostic; the bound only caps how bad that diagnostic number
/// can get.
pub type MapperPacketSender = SyncSender<MargData>;

/// Pure per-image feature-extraction step of `NfrMapper::detect_keypoints`'s
/// per-image body (`session.rs`): reads only its own raw pixel buffer plus
/// calibration/config, and mutates no mapper state, so `ingest_packet` can
/// run it concurrently across every newly eligible image in a packet (up to
/// the AOM window's 16) without any image's outcome depending on another's
/// or on processing order.
fn extract_one_image_features(
    image: &OfImageData,
    calibration: &BasaltCalibration,
    feature_config: OfflineMapperConfig,
) -> Result<MapperImageFeatures, OnlineMapperError> {
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
    extract_mapper_features(&raw, camera, feature_config).map_err(|source| {
        OnlineMapperError::Extraction {
            timestamp_ns,
            camera_id,
            source,
        }
    })
}

/// One accepted candidate pair's match-graph payload, computed by
/// [`evaluate_match_candidate`] on the shared rayon pool and applied back to
/// `self.mapper` sequentially by [`OnlineNfrMapper::match_new_keyframe`].
struct CandidateMatchOutcome {
    left_id: TimeCamId,
    right_id: TimeCamId,
    t_i_j: SE3,
    raw_matches: Vec<(u64, u64)>,
    inliers: Vec<(u64, u64)>,
}

/// Descriptor-match/RANSAC accept logic for one BoW candidate, factored out
/// of [`OnlineNfrMapper::match_new_keyframe`] so it can run on rayon's
/// shared pool: reads only `feature_corners` (never mutates it, and no
/// candidate's outcome depends on another's), so every candidate for a
/// query can be evaluated concurrently without changing which pairs get
/// accepted or their computed match data. Mirrors the sequential body this
/// replaced exactly -- same gate order, same RANSAC entry point selection,
/// same final `mutual_descriptor_matches` recomputation for the accepted
/// pair's stored raw matches.
fn evaluate_match_candidate(
    feature_corners: &std::collections::BTreeMap<TimeCamId, MapperImageFeatures>,
    query_id: TimeCamId,
    candidate: &BowQueryCandidate,
    config: OfflineMapperConfig,
    match_config: OfflineMapperConfig,
    seed: Option<u32>,
) -> Option<CandidateMatchOutcome> {
    if candidate.image.frame_id == query_id.frame_id
        || candidate.score <= config.frames_to_match_threshold
    {
        return None;
    }
    let other_id = TimeCamId::from(candidate.image);
    let (left_id, right_id) = (query_id, other_id);
    let left = feature_corners.get(&left_id)?;
    let right = feature_corners.get(&right_id)?;

    let stage = match_temporal_stage(left, right, match_config);
    if !stage.raw_match_gate_passed {
        return None;
    }

    let result = match seed {
        Some(seed) => match_temporal_ransac_seeded(left, right, match_config, seed),
        None => match_temporal_ransac(left, right, match_config),
    };
    if !result.accepted || result.refined_inlier_ids.is_empty() {
        return None;
    }

    let raw_matches =
        mutual_descriptor_matches(&left.descriptors, &right.descriptors, match_config)
            .into_iter()
            .map(|match_| (match_.left, match_.right))
            .collect::<Vec<_>>();
    Some(CandidateMatchOutcome {
        left_id,
        right_id,
        t_i_j: temporal_result_se3(&result),
        raw_matches,
        inliers: result.refined_inlier_ids.clone(),
    })
}

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

    /// Like [`two_packets_with_images`], but `packet_two`'s window
    /// *overlaps* `packet_one`'s -- it carries images for both `first` and
    /// `second`, exactly like a real MargData stream's sliding AOM window
    /// (Phase A: one packet's `of_images` is the whole marginalization-event
    /// window, and consecutive packets' windows share all but the oldest
    /// keyframe). Used to prove the overlap-skip fix below.
    fn overlapping_packets_with_images() -> (MargData, MargData) {
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
            feature_image(first.frame_id, first.timestamp_ns, 0),
            feature_image(first.frame_id, first.timestamp_ns, 1),
            feature_image(second.frame_id, second.timestamp_ns, 0),
            feature_image(second.frame_id, second.timestamp_ns, 1),
        ];

        (packet_one, packet_two)
    }

    /// A packet whose window overlaps the previous one's must only detect
    /// and match the genuinely new images, not re-process the whole window
    /// every time -- the exact bug an MH_01 smoke run caught (every packet
    /// reporting `new_key_count=16`, the full window, with per-packet match
    /// cost growing monotonically as a result).
    #[test]
    fn overlapping_packet_window_only_processes_new_images() {
        let (mut packet_one, mut packet_two) = overlapping_packets_with_images();
        let mut online = OnlineNfrMapper::new(
            MapperConfig::default(),
            feature_calibration(),
            OfflineMapperConfig::default(),
            GlobalBaConfig::default(),
            OnlineMapperConfig {
                optimize_every_k: usize::MAX,
                ..OnlineMapperConfig::default()
            },
        );
        let report_one = online
            .ingest_packet(&mut packet_one, Some(TEST_SEED))
            .expect("online ingest 1");
        assert_eq!(
            report_one.new_key_count, 2,
            "packet 1 detects its own 2 images"
        );

        let report_two = online
            .ingest_packet(&mut packet_two, Some(TEST_SEED))
            .expect("online ingest 2");
        assert_eq!(
            report_two.new_key_count, 2,
            "packet 2 must only detect the 2 genuinely new images (second's stereo pair), \
             not re-detect first's images already processed by packet 1"
        );
        assert_eq!(
            online.mapper.feature_corners.len(),
            4,
            "feature_corners must contain exactly 4 keys total, not a duplicate 6"
        );
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
            batch.feature_corners, online.mapper.feature_corners,
            "incremental detection must produce exactly the batch features (same keys, same pixels, same descriptors), \
             not merely the same key count"
        );
        assert_eq!(
            batch.feature_match_data, online.mapper.feature_match_data,
            "incremental per-pair match data must equal the batch match data"
        );
    }

    /// The inverted `hash_index` lookup `match_new_keyframe` uses must
    /// return exactly the same candidate list `query_bow_candidates` returns
    /// from a full scan of every detected image, for every query in a
    /// multi-packet prefix -- not merely the same *final accepted pairs*
    /// (`incremental_matches_equal_batch` above already covers that; this
    /// test isolates the BoW candidate-selection step itself, per the
    /// specific guardrail that the index change must not silently narrow or
    /// reorder candidates before RANSAC ever runs).
    #[test]
    fn inverted_index_candidates_equal_full_scan() {
        let (mut packet_one, mut packet_two) = two_packets_with_images();
        let mut online = OnlineNfrMapper::new(
            MapperConfig::default(),
            feature_calibration(),
            OfflineMapperConfig::default(),
            GlobalBaConfig::default(),
            OnlineMapperConfig {
                optimize_every_k: usize::MAX,
                ..OnlineMapperConfig::default()
            },
        );
        online
            .ingest_packet(&mut packet_one, Some(TEST_SEED))
            .expect("online ingest 1");
        online
            .ingest_packet(&mut packet_two, Some(TEST_SEED))
            .expect("online ingest 2");

        let match_window = online.mapper.feature_config.match_window as usize;
        let keys = online
            .mapper
            .feature_corners
            .keys()
            .copied()
            .collect::<Vec<_>>();
        assert!(
            keys.len() >= 4,
            "fixture must produce more than one frame's worth of keys"
        );
        let mut checked_nonempty = false;
        for query_id in keys {
            let query_features = online
                .mapper
                .feature_corners
                .get(&query_id)
                .expect("key present")
                .clone();

            let via_index =
                online.bow_candidates_via_index(query_id, &query_features, match_window);

            let full_database = online
                .mapper
                .feature_corners
                .iter()
                .map(|(&id, features)| (MapperImageId::from(id), features))
                .collect::<Vec<_>>();
            let via_full_scan = query_bow_candidates(
                MapperImageId::from(query_id),
                &query_features,
                &full_database,
                match_window,
            );

            assert_eq!(
                via_index, via_full_scan,
                "inverted-index candidates must equal a full-scan call for query {query_id:?}"
            );
            checked_nonempty |= !via_full_scan.is_empty();
        }
        assert!(
            checked_nonempty,
            "fixture must produce at least one non-empty candidate list for this test to be meaningful"
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
