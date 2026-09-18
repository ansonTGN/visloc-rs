//! Optional low-overhead timing breakdown for the canonical Basalt replay.
//!
//! The instrumentation is deliberately compile-time opt-in.  The canonical
//! build does not contain the clocking path, even when
//! `VISLOC_BASALT_TIMING_BREAKDOWN=1` is present.  Enable the
//! `basalt-timing-breakdown` Cargo feature to include the runtime environment
//! switch, monotonic clocking, and JSON sidecar.  When enabled, the buckets
//! are cumulative and use monotonic nanoseconds.

use std::path::Path;

#[cfg(any(feature = "basalt-timing-breakdown", test))]
use std::ffi::OsStr;

#[cfg(any(feature = "basalt-timing-breakdown", test))]
use std::time::Instant;

#[cfg(feature = "basalt-timing-breakdown")]
use std::fs;

use serde::Serialize;

#[cfg(feature = "basalt-timing-breakdown")]
const TIMING_ENV: &str = "VISLOC_BASALT_TIMING_BREAKDOWN";
#[cfg(feature = "basalt-timing-breakdown")]
const TIMING_SCHEMA: &str = "basalt.timing_breakdown.v1";
#[cfg(feature = "basalt-timing-breakdown")]
const TIMING_DURATION_UNITS: &str = "nanoseconds";

/// Cargo feature that must be enabled before the timing sidecar can exist.
pub const TIMING_FEATURE_NAME: &str = "basalt-timing-breakdown";

/// Whether this crate was compiled with the timing instrumentation.
pub const TIMING_FEATURE_ENABLED: bool = cfg!(feature = "basalt-timing-breakdown");

/// A cumulative timing bucket.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize)]
pub struct TimingStat {
    pub count: u64,
    pub total_ns: u64,
    pub max_ns: u64,
}

/// Named timing regions emitted by the EuRoC demo and adapter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimingBucket {
    /// One complete sensor/configuration dataset open, including its nested
    /// CSV, calibration, and config regions.
    DatasetOpen,
    /// EuRoC camera/IMU CSV manifest parsing during dataset open.
    DatasetCsvParsing,
    /// Calibration file read and JSON conversion during dataset open.
    DatasetCalibration,
    /// Basalt config file read and JSON conversion during dataset open.
    DatasetConfig,
    DatasetFrameAcquisition,
    /// One image::open operation, including file open and PNG decode.
    DatasetPngOpenDecode,
    /// DynamicImage-to-RawU16Image conversion, including the pinned u16
    /// promotion and raw-image validation.
    DatasetRawU16Conversion,
    AdapterTotal,
    AdapterFrontend,
    /// Raw-u16 pyramid construction for both cameras.
    FrontendPyramid,
    /// Forward/backward frame-to-frame KLT for existing cam0/cam1 tracks.
    FrontendTemporalKlt,
    /// Existing-position collection, grid FAST detection, and track insertion.
    FrontendFastReplenish,
    /// Forward/backward stereo KLT for newly detected cam0 tracks.
    FrontendNewStereoKlt,
    /// Double-Sphere stereo essential-residual filtering.
    FrontendEssentialFilter,
    /// Observation and lifecycle-vector materialization after tracking.
    FrontendOutput,
    AdapterEstimator,
    AdapterOutput,
    DemoOutput,
    EstimatorBuildProblem,
    EstimatorLmSolve,
    EstimatorMarginalization,
    LmLinearize,
    LmLinearSystemSolve,
    LmTrialConstructStep,
    LmTrialCost,
    LmAccept,
    LmLandmarkReduction,
    /// Compact landmark back-substitution generated from the same clean QR
    /// payload as the reduced system.  This is a top-level LM region, not a
    /// nested trial-construction sub-bucket.
    LmCompactBackSubstitution,
    LmNormalSystemPrep,
    LmModelDecrease,
    LmCompactApplyStep,
    LmDecisionStateBookkeeping,
    /// Landmark back-substitution performed while constructing an eager trial view.
    LmTrialLandmarkRecovery,
    /// Full pose/state application used by the eager trial view.
    LmTrialApplyStepFull,
    /// Trial-owned landmark coordinate materialization and increment overlay.
    LmTrialLandmarkMaterialization,
    /// Pose/state sidecars and carried-prior FEJ point materialization.
    LmTrialSidecarOther,
    /// Final trajectory/trace file serialization after the frame loop.
    DemoTrajectoryOutput,
    /// Explicit end-of-run summary/adapter teardown region.  The timing
    /// sidecar itself is written after this bucket is finished so it can
    /// include the completed counter.
    DemoTeardown,
}

/// Cumulative timing sidecar state.
///
/// The fields are public so a harness can inspect the in-memory contract, but
/// mutation is kept behind `Self::start` and `Self::finish` to make it
/// difficult to accidentally record a nested region as a second total.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct TimingBreakdown {
    enabled: bool,
    pub dataset_open: TimingStat,
    pub dataset_csv_parsing: TimingStat,
    pub dataset_calibration: TimingStat,
    pub dataset_config: TimingStat,
    pub dataset_frame_acquisition: TimingStat,
    pub dataset_png_open_decode: TimingStat,
    pub dataset_raw_u16_conversion: TimingStat,
    pub adapter_total: TimingStat,
    pub adapter_frontend: TimingStat,
    pub frontend_pyramid: TimingStat,
    pub frontend_temporal_klt: TimingStat,
    pub frontend_fast_replenish: TimingStat,
    pub frontend_new_stereo_klt: TimingStat,
    pub frontend_essential_filter: TimingStat,
    pub frontend_output: TimingStat,
    pub adapter_estimator: TimingStat,
    pub adapter_output: TimingStat,
    pub demo_output: TimingStat,
    pub estimator_build_problem: TimingStat,
    pub estimator_lm_solve: TimingStat,
    pub estimator_marginalization: TimingStat,
    pub lm_linearize: TimingStat,
    pub lm_linear_system_solve: TimingStat,
    pub lm_trial_construct_step: TimingStat,
    pub lm_trial_cost: TimingStat,
    pub lm_accept: TimingStat,
    pub lm_landmark_reduction: TimingStat,
    pub lm_compact_back_substitution: TimingStat,
    pub lm_normal_system_prep: TimingStat,
    pub lm_model_decrease: TimingStat,
    pub lm_compact_apply_step: TimingStat,
    pub lm_decision_state_bookkeeping: TimingStat,
    /// Nested inside `lm_trial_construct_step`; not part of the disjoint LM
    /// remainder accounting.  The four fields below partition the eager
    /// `WindowTrialView::from_step` work when timing is enabled.
    pub lm_trial_landmark_recovery: TimingStat,
    pub lm_trial_apply_step_full: TimingStat,
    pub lm_trial_landmark_materialization: TimingStat,
    /// This bucket may have two sequential samples per trial: pose/state
    /// sidecars precede the landmark materialization in the faithful source
    /// order, while view/prior sidecars follow it.
    pub lm_trial_sidecar_other: TimingStat,
    pub demo_trajectory_output: TimingStat,
    pub demo_teardown: TimingStat,
}

/// Opaque start marker.  No marker is created when timing is disabled.
#[derive(Debug)]
#[cfg(any(feature = "basalt-timing-breakdown", test))]
pub(crate) struct TimingStart(Instant);

#[derive(Debug)]
#[cfg(all(not(feature = "basalt-timing-breakdown"), not(test)))]
pub(crate) struct TimingStart;

impl TimingBreakdown {
    /// Creates an enabled collector only for the exact value `1`.
    #[cfg(feature = "basalt-timing-breakdown")]
    pub fn from_env() -> Self {
        Self {
            enabled: timing_env_value_is_enabled(std::env::var_os(TIMING_ENV).as_deref()),
            ..Self::default()
        }
    }

    /// Creates the compile-time disabled state.  The environment variable is
    /// intentionally not read unless the timing Cargo feature is enabled.
    #[cfg(not(feature = "basalt-timing-breakdown"))]
    #[inline(always)]
    pub fn from_env() -> Self {
        Self::default()
    }

    /// Creates the disabled state without reading process environment.
    #[cfg(test)]
    fn disabled() -> Self {
        Self::default()
    }

    /// Enables timing for integration tests without mutating the process
    /// environment (which would make parallel tests interfere with one
    /// another).  This is deliberately test-only; production callers use
    /// [`Self::from_env`].
    #[cfg(test)]
    pub(crate) fn enabled_for_test() -> Self {
        Self {
            enabled: true,
            ..Self::default()
        }
    }

    /// Whether the collector will take timestamps and retain counters.
    #[cfg(any(feature = "basalt-timing-breakdown", test))]
    pub const fn enabled(&self) -> bool {
        self.enabled
    }

    /// The default build has no timing branch to evaluate at runtime.
    #[cfg(all(not(feature = "basalt-timing-breakdown"), not(test)))]
    #[inline(always)]
    pub const fn enabled(&self) -> bool {
        false
    }

    /// Starts one region.  Returns `None` without calling `Instant::now()` in
    /// the normal (disabled) path.
    #[cfg(any(feature = "basalt-timing-breakdown", test))]
    pub(crate) fn start(&self) -> Option<TimingStart> {
        self.enabled.then(Instant::now).map(TimingStart)
    }

    /// Compile-time no-op for canonical builds.  `TimingBucket` is retained
    /// in the API so call-sites remain source-compatible, but this body has no
    /// clock, branch, or bucket dispatch.
    #[cfg(all(not(feature = "basalt-timing-breakdown"), not(test)))]
    #[inline(always)]
    pub(crate) const fn start(&self) -> Option<TimingStart> {
        None
    }

    /// Finishes one region.  A `None` marker is a disabled no-op.
    #[cfg(any(feature = "basalt-timing-breakdown", test))]
    pub(crate) fn finish(&mut self, bucket: TimingBucket, started: Option<TimingStart>) {
        let Some(TimingStart(started)) = started else {
            return;
        };
        let elapsed_ns = started.elapsed().as_nanos().min(u128::from(u64::MAX)) as u64;
        let stat = self.stat_mut(bucket);
        stat.count = stat.count.saturating_add(1);
        stat.total_ns = stat.total_ns.saturating_add(elapsed_ns);
        stat.max_ns = stat.max_ns.max(elapsed_ns);
    }

    /// Compile-time no-op for canonical builds.
    #[cfg(all(not(feature = "basalt-timing-breakdown"), not(test)))]
    #[inline(always)]
    pub(crate) const fn finish(&mut self, _bucket: TimingBucket, _started: Option<TimingStart>) {}

    /// Measures one external region, preserving the disabled fast path.
    #[cfg(any(feature = "basalt-timing-breakdown", test))]
    pub fn measure<T>(&mut self, bucket: TimingBucket, operation: impl FnOnce() -> T) -> T {
        let started = self.start();
        let result = operation();
        self.finish(bucket, started);
        result
    }

    /// Runs the operation directly in canonical builds.  With this
    /// `inline(always)` body the timing closure and bucket argument disappear
    /// during optimization while the operation's semantics stay unchanged.
    #[cfg(all(not(feature = "basalt-timing-breakdown"), not(test)))]
    #[inline(always)]
    pub fn measure<T>(&mut self, _bucket: TimingBucket, operation: impl FnOnce() -> T) -> T {
        operation()
    }

    /// Measures a region whose operation needs to add nested samples to this
    /// same collector.  This avoids a borrow/merge temporary for callers such
    /// as the timed EuRoC frame reader while retaining the disabled no-op
    /// behavior of [`Self::measure`].
    #[cfg(any(feature = "basalt-timing-breakdown", test))]
    pub fn measure_with<T>(
        &mut self,
        bucket: TimingBucket,
        operation: impl FnOnce(&mut Self) -> T,
    ) -> T {
        let started = self.start();
        let result = operation(self);
        self.finish(bucket, started);
        result
    }

    /// Direct operation path for canonical builds; no timing closure or
    /// runtime enabled branch remains after inlining.
    #[cfg(all(not(feature = "basalt-timing-breakdown"), not(test)))]
    #[inline(always)]
    pub fn measure_with<T>(
        &mut self,
        _bucket: TimingBucket,
        operation: impl FnOnce(&mut Self) -> T,
    ) -> T {
        operation(self)
    }

    /// Records all enabled counters from another collector.
    #[cfg(any(feature = "basalt-timing-breakdown", test))]
    pub fn merge_from(&mut self, other: &Self) {
        self.enabled |= other.enabled;
        merge_stat(&mut self.dataset_open, other.dataset_open);
        merge_stat(&mut self.dataset_csv_parsing, other.dataset_csv_parsing);
        merge_stat(&mut self.dataset_calibration, other.dataset_calibration);
        merge_stat(&mut self.dataset_config, other.dataset_config);
        merge_stat(
            &mut self.dataset_frame_acquisition,
            other.dataset_frame_acquisition,
        );
        merge_stat(
            &mut self.dataset_png_open_decode,
            other.dataset_png_open_decode,
        );
        merge_stat(
            &mut self.dataset_raw_u16_conversion,
            other.dataset_raw_u16_conversion,
        );
        merge_stat(&mut self.adapter_total, other.adapter_total);
        merge_stat(&mut self.adapter_frontend, other.adapter_frontend);
        merge_stat(&mut self.frontend_pyramid, other.frontend_pyramid);
        merge_stat(&mut self.frontend_temporal_klt, other.frontend_temporal_klt);
        merge_stat(
            &mut self.frontend_fast_replenish,
            other.frontend_fast_replenish,
        );
        merge_stat(
            &mut self.frontend_new_stereo_klt,
            other.frontend_new_stereo_klt,
        );
        merge_stat(
            &mut self.frontend_essential_filter,
            other.frontend_essential_filter,
        );
        merge_stat(&mut self.frontend_output, other.frontend_output);
        merge_stat(&mut self.adapter_estimator, other.adapter_estimator);
        merge_stat(&mut self.adapter_output, other.adapter_output);
        merge_stat(&mut self.demo_output, other.demo_output);
        merge_stat(
            &mut self.estimator_build_problem,
            other.estimator_build_problem,
        );
        merge_stat(&mut self.estimator_lm_solve, other.estimator_lm_solve);
        merge_stat(
            &mut self.estimator_marginalization,
            other.estimator_marginalization,
        );
        merge_stat(&mut self.lm_linearize, other.lm_linearize);
        merge_stat(
            &mut self.lm_linear_system_solve,
            other.lm_linear_system_solve,
        );
        merge_stat(
            &mut self.lm_trial_construct_step,
            other.lm_trial_construct_step,
        );
        merge_stat(&mut self.lm_trial_cost, other.lm_trial_cost);
        merge_stat(&mut self.lm_accept, other.lm_accept);
        merge_stat(&mut self.lm_landmark_reduction, other.lm_landmark_reduction);
        merge_stat(
            &mut self.lm_compact_back_substitution,
            other.lm_compact_back_substitution,
        );
        merge_stat(&mut self.lm_normal_system_prep, other.lm_normal_system_prep);
        merge_stat(&mut self.lm_model_decrease, other.lm_model_decrease);
        merge_stat(&mut self.lm_compact_apply_step, other.lm_compact_apply_step);
        merge_stat(
            &mut self.lm_decision_state_bookkeeping,
            other.lm_decision_state_bookkeeping,
        );
        merge_stat(
            &mut self.lm_trial_landmark_recovery,
            other.lm_trial_landmark_recovery,
        );
        merge_stat(
            &mut self.lm_trial_apply_step_full,
            other.lm_trial_apply_step_full,
        );
        merge_stat(
            &mut self.lm_trial_landmark_materialization,
            other.lm_trial_landmark_materialization,
        );
        merge_stat(
            &mut self.lm_trial_sidecar_other,
            other.lm_trial_sidecar_other,
        );
        merge_stat(
            &mut self.demo_trajectory_output,
            other.demo_trajectory_output,
        );
        merge_stat(&mut self.demo_teardown, other.demo_teardown);
    }

    /// Compile-time no-op for canonical builds.
    #[cfg(all(not(feature = "basalt-timing-breakdown"), not(test)))]
    #[inline(always)]
    pub const fn merge_from(&mut self, _other: &Self) {}

    /// Sum of the four eager trial-construction sub-buckets.  These samples
    /// are nested inside `lm_trial_construct_step`, so callers must not add
    /// this value to the disjoint LM component sum used by
    /// [`Self::lm_remainder_ns`].
    #[cfg(any(feature = "basalt-timing-breakdown", test))]
    pub const fn lm_trial_construct_subbuckets_ns(&self) -> u64 {
        self.lm_trial_landmark_recovery
            .total_ns
            .saturating_add(self.lm_trial_apply_step_full.total_ns)
            .saturating_add(self.lm_trial_landmark_materialization.total_ns)
            .saturating_add(self.lm_trial_sidecar_other.total_ns)
    }

    /// No timing buckets exist in the canonical build.
    #[cfg(all(not(feature = "basalt-timing-breakdown"), not(test)))]
    #[inline(always)]
    pub const fn lm_trial_construct_subbuckets_ns(&self) -> u64 {
        0
    }

    /// Returns the cumulative LM time not covered by the disjoint internal
    /// buckets.  `estimator_lm_solve` is the inclusive estimator-side span;
    /// the component buckets are retained by the active WindowProblem and
    /// merged into this collector before the sidecar is written.  The phase-4
    /// decision/state bucket may contain several sequential samples per LM
    /// attempt because its regions intentionally exclude the timed accept
    /// callback.
    #[cfg(any(feature = "basalt-timing-breakdown", test))]
    pub const fn lm_remainder_ns(&self) -> u64 {
        let accounted = self
            .lm_linearize
            .total_ns
            .saturating_add(self.lm_linear_system_solve.total_ns)
            .saturating_add(self.lm_trial_construct_step.total_ns)
            .saturating_add(self.lm_trial_cost.total_ns)
            .saturating_add(self.lm_accept.total_ns)
            .saturating_add(self.lm_landmark_reduction.total_ns)
            .saturating_add(self.lm_compact_back_substitution.total_ns)
            .saturating_add(self.lm_normal_system_prep.total_ns)
            .saturating_add(self.lm_model_decrease.total_ns)
            .saturating_add(self.lm_compact_apply_step.total_ns)
            .saturating_add(self.lm_decision_state_bookkeeping.total_ns);
        self.estimator_lm_solve.total_ns.saturating_sub(accounted)
    }

    /// No timing buckets exist in the canonical build.
    #[cfg(all(not(feature = "basalt-timing-breakdown"), not(test)))]
    #[inline(always)]
    pub const fn lm_remainder_ns(&self) -> u64 {
        0
    }

    /// Writes the stable JSON sidecar.  Callers should invoke this only when
    /// [`Self::enabled`] is true; the demo follows that rule so disabled runs
    /// produce no timing file.
    #[cfg(feature = "basalt-timing-breakdown")]
    pub fn write_json(&self, path: impl AsRef<Path>) -> std::io::Result<()> {
        let payload = serde_json::json!({
            "schema": TIMING_SCHEMA,
            "enabled": self.enabled,
            "units": {
                "duration": TIMING_DURATION_UNITS,
                "clock": "monotonic_elapsed",
            },
            "overlap_policy": {
                "parent_regions": "inclusive",
                "lm_component_regions": "disjoint_within_estimator_lm_solve",
                "lm_trial_construct_subbuckets": "nested_in_lm_trial_construct_step",
                "demo_teardown_excludes_timing_json_write": true,
            },
            "binding": {
                "owner": "external_run_manifest",
                "engine_input_and_source_hashes": "recorded_by_runner_manifest",
                "sidecar_self_hash": false,
            },
            "buckets": {
                "dataset_open": self.dataset_open,
                "dataset_csv_parsing": self.dataset_csv_parsing,
                "dataset_calibration": self.dataset_calibration,
                "dataset_config": self.dataset_config,
                "dataset_frame_acquisition": self.dataset_frame_acquisition,
                "dataset_png_open_decode": self.dataset_png_open_decode,
                "dataset_raw_u16_conversion": self.dataset_raw_u16_conversion,
                "adapter_total": self.adapter_total,
                "adapter_frontend": self.adapter_frontend,
                "frontend_pyramid": self.frontend_pyramid,
                "frontend_temporal_klt": self.frontend_temporal_klt,
                "frontend_fast_replenish": self.frontend_fast_replenish,
                "frontend_new_stereo_klt": self.frontend_new_stereo_klt,
                "frontend_essential_filter": self.frontend_essential_filter,
                "frontend_output": self.frontend_output,
                "adapter_estimator": self.adapter_estimator,
                "adapter_output": self.adapter_output,
                "demo_output": self.demo_output,
                "estimator_build_problem": self.estimator_build_problem,
                "estimator_lm_solve": self.estimator_lm_solve,
                "estimator_marginalization": self.estimator_marginalization,
                "lm_linearize": self.lm_linearize,
                "lm_linear_system_solve": self.lm_linear_system_solve,
                "lm_trial_construct_step": self.lm_trial_construct_step,
                "lm_trial_cost": self.lm_trial_cost,
                "lm_accept": self.lm_accept,
                "lm_landmark_reduction": self.lm_landmark_reduction,
                "lm_compact_back_substitution": self.lm_compact_back_substitution,
                "lm_normal_system_prep": self.lm_normal_system_prep,
                "lm_model_decrease": self.lm_model_decrease,
                "lm_compact_apply_step": self.lm_compact_apply_step,
                "lm_decision_state_bookkeeping": self.lm_decision_state_bookkeeping,
                "lm_trial_landmark_recovery": self.lm_trial_landmark_recovery,
                "lm_trial_apply_step_full": self.lm_trial_apply_step_full,
                "lm_trial_landmark_materialization": self.lm_trial_landmark_materialization,
                "lm_trial_sidecar_other": self.lm_trial_sidecar_other,
                "demo_trajectory_output": self.demo_trajectory_output,
                "demo_teardown": self.demo_teardown,
            },
            "derived": {
                "lm_remainder_ns": self.lm_remainder_ns(),
                "lm_trial_construct_subbuckets_ns": self.lm_trial_construct_subbuckets_ns(),
            },
        });
        let text = serde_json::to_vec_pretty(&payload)
            .map_err(|error| std::io::Error::other(format!("timing JSON: {error}")))?;
        fs::write(path, text)
    }

    /// The sidecar is intentionally unavailable in a default build.  This
    /// explicit error protects callers that bypass the demo's `enabled()`
    /// guard and makes the feature requirement observable.
    #[cfg(not(feature = "basalt-timing-breakdown"))]
    pub fn write_json(&self, _path: impl AsRef<Path>) -> std::io::Result<()> {
        Err(std::io::Error::new(
            std::io::ErrorKind::Unsupported,
            "timing breakdown requires Cargo feature `basalt-timing-breakdown`",
        ))
    }

    #[cfg(any(feature = "basalt-timing-breakdown", test))]
    const fn stat_mut(&mut self, bucket: TimingBucket) -> &mut TimingStat {
        match bucket {
            TimingBucket::DatasetOpen => &mut self.dataset_open,
            TimingBucket::DatasetCsvParsing => &mut self.dataset_csv_parsing,
            TimingBucket::DatasetCalibration => &mut self.dataset_calibration,
            TimingBucket::DatasetConfig => &mut self.dataset_config,
            TimingBucket::DatasetFrameAcquisition => &mut self.dataset_frame_acquisition,
            TimingBucket::DatasetPngOpenDecode => &mut self.dataset_png_open_decode,
            TimingBucket::DatasetRawU16Conversion => &mut self.dataset_raw_u16_conversion,
            TimingBucket::AdapterTotal => &mut self.adapter_total,
            TimingBucket::AdapterFrontend => &mut self.adapter_frontend,
            TimingBucket::FrontendPyramid => &mut self.frontend_pyramid,
            TimingBucket::FrontendTemporalKlt => &mut self.frontend_temporal_klt,
            TimingBucket::FrontendFastReplenish => &mut self.frontend_fast_replenish,
            TimingBucket::FrontendNewStereoKlt => &mut self.frontend_new_stereo_klt,
            TimingBucket::FrontendEssentialFilter => &mut self.frontend_essential_filter,
            TimingBucket::FrontendOutput => &mut self.frontend_output,
            TimingBucket::AdapterEstimator => &mut self.adapter_estimator,
            TimingBucket::AdapterOutput => &mut self.adapter_output,
            TimingBucket::DemoOutput => &mut self.demo_output,
            TimingBucket::EstimatorBuildProblem => &mut self.estimator_build_problem,
            TimingBucket::EstimatorLmSolve => &mut self.estimator_lm_solve,
            TimingBucket::EstimatorMarginalization => &mut self.estimator_marginalization,
            TimingBucket::LmLinearize => &mut self.lm_linearize,
            TimingBucket::LmLinearSystemSolve => &mut self.lm_linear_system_solve,
            TimingBucket::LmTrialConstructStep => &mut self.lm_trial_construct_step,
            TimingBucket::LmTrialCost => &mut self.lm_trial_cost,
            TimingBucket::LmAccept => &mut self.lm_accept,
            TimingBucket::LmLandmarkReduction => &mut self.lm_landmark_reduction,
            TimingBucket::LmCompactBackSubstitution => &mut self.lm_compact_back_substitution,
            TimingBucket::LmNormalSystemPrep => &mut self.lm_normal_system_prep,
            TimingBucket::LmModelDecrease => &mut self.lm_model_decrease,
            TimingBucket::LmCompactApplyStep => &mut self.lm_compact_apply_step,
            TimingBucket::LmDecisionStateBookkeeping => &mut self.lm_decision_state_bookkeeping,
            TimingBucket::LmTrialLandmarkRecovery => &mut self.lm_trial_landmark_recovery,
            TimingBucket::LmTrialApplyStepFull => &mut self.lm_trial_apply_step_full,
            TimingBucket::LmTrialLandmarkMaterialization => {
                &mut self.lm_trial_landmark_materialization
            }
            TimingBucket::LmTrialSidecarOther => &mut self.lm_trial_sidecar_other,
            TimingBucket::DemoTrajectoryOutput => &mut self.demo_trajectory_output,
            TimingBucket::DemoTeardown => &mut self.demo_teardown,
        }
    }
}

#[cfg(any(feature = "basalt-timing-breakdown", test))]
fn timing_env_value_is_enabled(value: Option<&OsStr>) -> bool {
    value == Some(OsStr::new("1"))
}

#[cfg(any(feature = "basalt-timing-breakdown", test))]
fn merge_stat(target: &mut TimingStat, source: TimingStat) {
    target.count = target.count.saturating_add(source.count);
    target.total_ns = target.total_ns.saturating_add(source.total_ns);
    target.max_ns = target.max_ns.max(source.max_ns);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn timing_is_opt_in_exactly_at_one() {
        assert!(timing_env_value_is_enabled(Some(OsStr::new("1"))));
        assert!(!timing_env_value_is_enabled(Some(OsStr::new("true"))));
        assert!(!timing_env_value_is_enabled(Some(OsStr::new("0"))));
        assert!(!timing_env_value_is_enabled(None));
    }

    #[test]
    fn disabled_regions_have_no_samples() {
        let mut timing = TimingBreakdown::disabled();
        let started = timing.start();
        timing.finish(TimingBucket::AdapterTotal, started);
        assert!(!timing.enabled());
        assert_eq!(timing.adapter_total, TimingStat::default());
    }

    #[test]
    fn enabled_region_records_count_total_and_max() {
        let mut timing = TimingBreakdown {
            enabled: true,
            ..TimingBreakdown::default()
        };
        let first = timing.start();
        timing.finish(TimingBucket::AdapterFrontend, first);
        let second = timing.start();
        timing.finish(TimingBucket::AdapterFrontend, second);
        assert_eq!(timing.adapter_frontend.count, 2);
        assert!(timing.adapter_frontend.total_ns >= timing.adapter_frontend.max_ns);
    }

    #[test]
    fn merge_preserves_cumulative_contract() {
        let mut left = TimingBreakdown {
            enabled: true,
            adapter_total: TimingStat {
                count: 1,
                total_ns: 7,
                max_ns: 7,
            },
            ..TimingBreakdown::default()
        };
        let right = TimingBreakdown {
            enabled: true,
            adapter_total: TimingStat {
                count: 2,
                total_ns: 11,
                max_ns: 8,
            },
            ..TimingBreakdown::default()
        };
        left.merge_from(&right);
        assert_eq!(
            left.adapter_total,
            TimingStat {
                count: 3,
                total_ns: 18,
                max_ns: 8,
            }
        );
    }

    #[test]
    fn estimator_phase_buckets_are_independent_and_mergeable() {
        let mut timing = TimingBreakdown {
            enabled: true,
            estimator_build_problem: TimingStat {
                count: 3,
                total_ns: 30,
                max_ns: 12,
            },
            estimator_lm_solve: TimingStat {
                count: 3,
                total_ns: 90,
                max_ns: 40,
            },
            estimator_marginalization: TimingStat {
                count: 1,
                total_ns: 17,
                max_ns: 17,
            },
            ..TimingBreakdown::default()
        };
        let other = TimingBreakdown {
            enabled: true,
            estimator_build_problem: TimingStat {
                count: 1,
                total_ns: 8,
                max_ns: 8,
            },
            estimator_lm_solve: TimingStat {
                count: 1,
                total_ns: 25,
                max_ns: 25,
            },
            estimator_marginalization: TimingStat {
                count: 2,
                total_ns: 33,
                max_ns: 20,
            },
            ..TimingBreakdown::default()
        };
        timing.merge_from(&other);
        assert_eq!(timing.estimator_build_problem.count, 4);
        assert_eq!(timing.estimator_build_problem.total_ns, 38);
        assert_eq!(timing.estimator_lm_solve.total_ns, 115);
        assert_eq!(timing.estimator_marginalization.count, 3);
        assert_eq!(timing.estimator_marginalization.max_ns, 20);
    }

    #[test]
    fn lm_phase_buckets_and_remainder_are_serialization_ready() {
        let mut timing = TimingBreakdown {
            enabled: true,
            estimator_lm_solve: TimingStat {
                count: 2,
                total_ns: 1020,
                max_ns: 600,
            },
            lm_linearize: TimingStat {
                count: 4,
                total_ns: 100,
                max_ns: 40,
            },
            lm_linear_system_solve: TimingStat {
                count: 4,
                total_ns: 300,
                max_ns: 90,
            },
            lm_trial_construct_step: TimingStat {
                count: 3,
                total_ns: 120,
                max_ns: 60,
            },
            lm_trial_cost: TimingStat {
                count: 3,
                total_ns: 400,
                max_ns: 180,
            },
            lm_accept: TimingStat {
                count: 2,
                total_ns: 50,
                max_ns: 30,
            },
            lm_landmark_reduction: TimingStat {
                count: 4,
                total_ns: 10,
                max_ns: 4,
            },
            lm_compact_back_substitution: TimingStat {
                count: 4,
                total_ns: 20,
                max_ns: 8,
            },
            lm_normal_system_prep: TimingStat {
                count: 4,
                total_ns: 5,
                max_ns: 2,
            },
            lm_model_decrease: TimingStat {
                count: 4,
                total_ns: 7,
                max_ns: 3,
            },
            lm_compact_apply_step: TimingStat {
                count: 4,
                total_ns: 4,
                max_ns: 2,
            },
            lm_decision_state_bookkeeping: TimingStat {
                count: 4,
                total_ns: 2,
                max_ns: 1,
            },
            lm_trial_landmark_recovery: TimingStat {
                count: 3,
                total_ns: 30,
                max_ns: 12,
            },
            lm_trial_apply_step_full: TimingStat {
                count: 3,
                total_ns: 20,
                max_ns: 9,
            },
            lm_trial_landmark_materialization: TimingStat {
                count: 3,
                total_ns: 10,
                max_ns: 4,
            },
            lm_trial_sidecar_other: TimingStat {
                count: 6,
                total_ns: 5,
                max_ns: 2,
            },
            ..TimingBreakdown::default()
        };
        assert_eq!(timing.lm_remainder_ns(), 2);
        assert_eq!(timing.lm_trial_construct_subbuckets_ns(), 65);
        let payload = serde_json::json!({
            "schema": "basalt.timing_breakdown.v1",
            "enabled": true,
            "buckets": {
                "lm_linearize": timing.lm_linearize,
                "lm_linear_system_solve": timing.lm_linear_system_solve,
                "lm_trial_construct_step": timing.lm_trial_construct_step,
                "lm_trial_cost": timing.lm_trial_cost,
                "lm_accept": timing.lm_accept,
                "lm_landmark_reduction": timing.lm_landmark_reduction,
                "lm_compact_back_substitution": timing.lm_compact_back_substitution,
                "lm_normal_system_prep": timing.lm_normal_system_prep,
                "lm_model_decrease": timing.lm_model_decrease,
                "lm_compact_apply_step": timing.lm_compact_apply_step,
                "lm_decision_state_bookkeeping": timing.lm_decision_state_bookkeeping,
                "lm_trial_landmark_recovery": timing.lm_trial_landmark_recovery,
                "lm_trial_apply_step_full": timing.lm_trial_apply_step_full,
                "lm_trial_landmark_materialization": timing.lm_trial_landmark_materialization,
                "lm_trial_sidecar_other": timing.lm_trial_sidecar_other,
            },
            "derived": {
                "lm_remainder_ns": timing.lm_remainder_ns(),
                "lm_trial_construct_subbuckets_ns": timing.lm_trial_construct_subbuckets_ns(),
            },
        });
        assert_eq!(payload["buckets"]["lm_accept"]["count"], 2);
        assert_eq!(
            payload["buckets"]["lm_compact_back_substitution"]["count"],
            4
        );
        assert_eq!(
            payload["buckets"]["lm_compact_back_substitution"]["total_ns"],
            20
        );
        assert_eq!(payload["buckets"]["lm_model_decrease"]["total_ns"], 7);
        assert_eq!(payload["buckets"]["lm_trial_sidecar_other"]["count"], 6);
        assert_eq!(payload["derived"]["lm_remainder_ns"], 2);
        assert_eq!(payload["derived"]["lm_trial_construct_subbuckets_ns"], 65);

        let disabled = TimingBreakdown::disabled();
        assert_eq!(disabled.lm_compact_back_substitution, TimingStat::default());

        let started = timing.start();
        timing.finish(TimingBucket::LmLinearize, started);
        assert_eq!(timing.lm_linearize.count, 5);
    }

    #[cfg(feature = "basalt-timing-breakdown")]
    #[test]
    fn sidecar_declares_units_overlap_policy_and_external_binding_owner() {
        let timing = TimingBreakdown::enabled_for_test();
        let path = std::env::temp_dir().join(format!(
            "visloc_basalt_timing_contract_{}.json",
            std::process::id()
        ));
        timing.write_json(&path).unwrap();
        let payload: serde_json::Value = serde_json::from_slice(&fs::read(&path).unwrap()).unwrap();
        assert_eq!(payload["schema"], "basalt.timing_breakdown.v1");
        assert_eq!(payload["units"]["duration"], "nanoseconds");
        assert_eq!(payload["units"]["clock"], "monotonic_elapsed");
        assert_eq!(payload["overlap_policy"]["parent_regions"], "inclusive");
        assert_eq!(
            payload["overlap_policy"]["lm_component_regions"],
            "disjoint_within_estimator_lm_solve"
        );
        assert_eq!(
            payload["overlap_policy"]["lm_trial_construct_subbuckets"],
            "nested_in_lm_trial_construct_step"
        );
        assert_eq!(payload["binding"]["owner"], "external_run_manifest");
        assert_eq!(payload["binding"]["sidecar_self_hash"], false);
        fs::remove_file(path).unwrap();
    }

    #[cfg(not(feature = "basalt-timing-breakdown"))]
    #[test]
    fn default_build_requires_feature_for_timing_sidecar() {
        assert!(!TIMING_FEATURE_ENABLED);
        assert_eq!(TIMING_FEATURE_NAME, "basalt-timing-breakdown");
        let path = std::env::temp_dir().join(format!(
            "visloc_basalt_timing_feature_required_{}.json",
            std::process::id()
        ));
        let _ = std::fs::remove_file(&path);
        let error = TimingBreakdown::from_env().write_json(&path).unwrap_err();
        assert_eq!(error.kind(), std::io::ErrorKind::Unsupported);
        assert!(error.to_string().contains(TIMING_FEATURE_NAME));
        assert!(!path.exists());
    }
}
