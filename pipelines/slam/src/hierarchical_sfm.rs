//! End-to-end assembly of an ordered sequence into a typed submap hierarchy.
//!
//! This module composes the S2 boundaries without weakening any of them:
//! adaptive overlapping partitioning, independent local reconstruction, shared
//! observation + essential-rotation overlap evidence, R2 Sim(3) verification,
//! and the R3 sparse global submap graph.

#![allow(clippy::result_large_err)]
use std::collections::HashMap;
use std::error::Error;
use std::fmt;

use rayon::prelude::*;
use visloc_core::types::Camera;
use visloc_vision::features::FeatureSet;

use crate::{
    collect_submap_overlap_evidence, estimate_submap_sim3_constraint, partition_ordered_submaps,
    refine_submap_sim3_from_camera_centres, remap_pairs_to_submap, shared_camera_center_matches,
    widen_and_build, AdaptiveSubmapPartitionConfig, AdaptiveSubmapPartitionHints,
    CameraCentreScaleRefinementConfig, CameraCentreScaleRefinementRejection,
    HierarchicalLoopClosureResult, HierarchicalSeamBaConfig, HierarchicalSeamBaError,
    HierarchicalSeamBaResult, HierarchicalSeamLandmarkLink, HierarchicalSubmapGraph,
    HierarchicalSubmapGraphError, HierarchicalSubmapOptimizationResult, IncrementalSfmError,
    LocalSubmap, LocalSubmapBuildError, LocalSubmapBuilder, LocalSubmapConfig,
    PairRotationEvidence, PairwiseMatches, Sim3PoseGraphConfig, SubmapOverlapConfig,
    SubmapOverlapError, SubmapPartitionError, SubmapPointMatch, SubmapSim3AlignmentConfig,
    SubmapSim3Rejection, SubmapSim3RejectionReason, SubmapWindow, VerifiedSubmapConstraint,
    WidenMergeReason,
};

#[derive(Debug, Clone, PartialEq)]
pub struct HierarchicalSfmConfig {
    pub partition: AdaptiveSubmapPartitionConfig,
    pub local_submap: LocalSubmapConfig,
    pub overlap: SubmapOverlapConfig,
    pub alignment: SubmapSim3AlignmentConfig,
    pub pose_graph: Sim3PoseGraphConfig,
    /// Maximum submap-index separation for attempted Sim(3) constraints.
    /// Adjacent constraints (`i -> i+1`) remain mandatory seams; constraints
    /// at separations `2..=submap_constraint_band` are optional redundant
    /// pose-graph edges and are skipped when their unchanged overlap/alignment
    /// gates reject them. Default `4`.
    pub submap_constraint_band: usize,
    /// Discover descriptor-derived constraints beyond
    /// `submap_constraint_band` and solve a second submap pose graph after
    /// optional seam BA. Off by default.
    pub submap_loop_closure: bool,
    /// Raw and essential-inlier match threshold for representative frame
    /// pairs used by loop discovery.
    pub submap_loop_min_matches: usize,
    /// Number of cosine-ranked long-range partners retained per submap before
    /// the expensive center-frame descriptor matcher runs. The union of each
    /// endpoint's top-K list is verified. Default `8`.
    pub submap_loop_top_k: usize,
    /// Additionally retain every long-range pair at or above this cosine
    /// similarity, regardless of rank. `None` (the default) disables this
    /// optional escape hatch.
    pub submap_loop_min_similarity: Option<f32>,
    /// Lowe ratio passed to the same cross-checked brute-force matcher used by
    /// the sequential view graph.
    pub submap_loop_match_ratio: f32,
    /// Maximum registered frames selected around each submap's center for
    /// expensive loop verification. Frames are sampled center-first at an
    /// eight-frame radius. Default `11` (center and approximately ±8 through
    /// ±40).
    pub submap_loop_verification_frame_budget: usize,
    /// Stop matching additional verification frame pairs once this many
    /// mutual 3D-3D landmark correspondences have accumulated. This is only a
    /// work cap; all overlap, RANSAC, and acceptance gates remain unchanged.
    /// Default `150`.
    pub submap_loop_verification_correspondence_target: usize,
    pub camera_centre_refinement: Option<CameraCentreScaleRefinementConfig>,
    pub seam_bundle_adjustment: Option<HierarchicalSeamBaConfig>,
    /// After loop PGO, weld accepted loop-edge landmark inliers together with
    /// the adjacent seam links and run a second transactional global BA.
    /// This is effective only when `submap_loop_closure` is enabled.
    pub submap_loop_bundle_adjustment: bool,
    /// Maximum independent local reconstructions evaluated concurrently.
    pub max_parallel_local_builds: usize,
    /// When a seam between two already-*built* submaps is rejected only
    /// because its shared-correspondence point cloud is (near-)collinear /
    /// rank-deficient — [`SubmapSim3RejectionReason::DegenerateSourceGeometry`]
    /// or [`SubmapSim3RejectionReason::DegenerateTargetGeometry`], the hard
    /// pre-RANSAC SVD gate in `estimate_submap_sim3_constraint` — merge the
    /// two submaps' image ranges into one window, rebuild *only* that merged
    /// window, splice it into the sequence in place of the two originals, and
    /// retry alignment against its new neighbours. This is the direct
    /// structural analogue of `partition.widen_unseedable_windows`, extended
    /// from the *build* stage to the *alignment* stage (see
    /// `SEAM_DEGENERACY_DIAGNOSIS.md`): a degenerate seam usually means the
    /// planned window boundary happened to pin the shared overlap to a
    /// low-baseline-diversity span (e.g. a takeoff/hover head), and growing
    /// the window past it — exactly like widening does for an unseedable
    /// window — is the fix, not relaxing the gate. `HighMeanResidual` is also
    /// routed here unconditionally because sparse cross-submap triangulation
    /// noise is repaired by the same joint rebuild. Other rejection reasons
    /// remain fail-fast unless explicitly admitted by the internal-drift gate
    /// below. Defaults to enabled for the same reason
    /// `widen_unseedable_windows` defaults to enabled: a single degenerate
    /// seam is a structural failure mode of the fixed-window partition, not a
    /// tuning choice.
    pub merge_degenerate_seams: bool,
    /// Cap on the total number of submap-pair merges `merge_degenerate_seams`
    /// may perform across one `hierarchical_sfm` run. Kept as a field
    /// separate from `partition.max_widen_merges` rather than sharing it:
    /// the two loops guard structurally different failure classes (no seed
    /// pair anywhere inside a single window, vs. a degenerate shared-point
    /// cloud between two already-seeded windows) that can both fire in the
    /// same run — e.g. the diagnosed MH_03 2700-frame run trips
    /// `max_widen_merges` on a mid-sequence near-static hover *and*
    /// independently trips this cap on the takeoff head's first seam — so
    /// tuning one budget must not silently starve the other. Also, unlike
    /// `widen_and_build`'s single left-to-right pass over independent window
    /// builds, this stage re-evaluates the *whole* seam chain from scratch
    /// after every merge (submap ids are positional, so a merge shifts every
    /// later index), which makes a per-contiguous-span reset like
    /// `widen_and_build`'s meaningless here — only a single flat run budget
    /// is well-defined. Defaults to 16, the same value as
    /// `max_widen_merges`, for the same reasoning documented there: the
    /// diagnosed failure needed only a handful of merges, so 16 gives ample
    /// headroom while still bounding worst-case rebuild cost (and thus
    /// reconstruction time) for pathological all-degenerate input, which
    /// still fails fast — returning the triggering `Alignment` error
    /// unchanged — once the budget is spent.
    pub max_degenerate_seam_merges: usize,
    /// Allow one bounded structural component rebuild for an alignment seam
    /// that remains fatal after the degenerate, residual, internal-drift, and
    /// catastrophic-consensus routes have all been considered. This changes
    /// routing only; the rebuilt component must still pass the normal seam
    /// acceptance gates. Data-integrity rejections (`NonFinitePoint` and
    /// `NonUniqueCorrespondences`) are excluded.
    /// Defaults to enabled.
    pub last_resort_seam_merge: bool,
    /// Extend `merge_degenerate_seams`'s widen/merge remediation to
    /// `LowInlierRatio` / `NoRobustFit` seam rejections -- but *only* when
    /// [`crate::submap_overlap::seam_step_shape_diagnostic`]
    /// (`LOWINLIERRATIO_DIAGNOSIS.md` Probe 2, formalized: each side's own
    /// camera-centre step-size variation across the *frames the two submaps
    /// share*, compared between the two sides) also identifies one side as
    /// internally inconsistent
    /// (`disagreement_ratio > max_seam_internal_drift_disagreement_ratio`).
    /// This is deliberately narrower than extending the merge trigger to
    /// every seam of these reasons (the original
    /// `NOROBUSTFIT_CLUSTER_DIAGNOSIS.md` explicitly rejected that blanket
    /// extension, §6(c): most `LowInlierRatio` seams have no internal defect
    /// on either side and widening them would just be "try a bigger window
    /// and hope"). The cross-submap comparison is what makes this
    /// principled and GT-free: real camera motion is common to both
    /// independent reconstructions of the same shared frames and cancels
    /// out of the comparison (verified: a single-submap, build-time-only
    /// version of this statistic could not distinguish the diagnosed
    /// submap-9/13 defect from four demonstrably-healthy submaps sitting in
    /// a genuinely fast-accelerating stretch of the same subrange -- see
    /// `WINDOWDRIFT_CALIBRATION.md`), so only a genuine one-sided internal
    /// defect survives it. `true` by default.
    pub seam_internal_drift_gate_enabled: bool,
    /// How many contiguous sub-windows each side's shared-frame step
    /// sequence is split into for
    /// [`crate::submap_overlap::seam_step_shape_diagnostic`]. Default `2`
    /// (first half vs. second half of the shared span) -- the granularity
    /// `LOWINLIERRATIO_DIAGNOSIS.md`'s Probe 2 validated against real data.
    pub seam_internal_drift_window_count: usize,
    /// Reject-into-remediation threshold for
    /// [`crate::submap_overlap::SeamStepShapeDiagnostic::disagreement_ratio`].
    /// Default `1.15`, calibrated against the actual `subrange_1100_1500`
    /// run (all 19 seams, `seam_internal_drift_window_count = 2`): every
    /// passing seam measured `<= 1.019`; every seam touching the diagnosed
    /// submap 9 or 13 measured `>= 1.264` (roughly the geometric mean of the
    /// two, ~13% headroom on each side). See `SEAMDRIFT_CALIBRATION.md` for
    /// the full table, including the fast-motion stretch (seams `16->17`,
    /// `17->18`) that defeated the build-time single-submap version of this
    /// idea but measures `<= 1.014` here, because real common-mode motion
    /// cancels out of this cross-submap comparison.
    pub max_seam_internal_drift_disagreement_ratio: f64,
    /// Severe cross-seam consensus failure below which a `LowInlierRatio`
    /// rejection is structurally remediated even when both submaps agree in
    /// the internal step-shape diagnostic. This is routing only: the Sim(3)
    /// acceptance gates in [`SubmapSim3AlignmentConfig`] are unchanged.
    /// Default `0.20`, chosen from the 2026-07-31 MH_05/MH_02 seam sweep: all
    /// seven healthy/agreeing catastrophic seams measured `<= 0.1987`, while
    /// the diagnosis's deliberately broader class boundary was `< 0.23`.
    pub catastrophic_consensus_inlier_ratio_threshold: f64,
}

impl Default for HierarchicalSfmConfig {
    fn default() -> Self {
        Self {
            partition: AdaptiveSubmapPartitionConfig::default(),
            local_submap: LocalSubmapConfig::default(),
            overlap: SubmapOverlapConfig::default(),
            alignment: SubmapSim3AlignmentConfig::default(),
            pose_graph: Sim3PoseGraphConfig::default(),
            submap_constraint_band: 4,
            submap_loop_closure: false,
            submap_loop_min_matches: 30,
            submap_loop_top_k: 8,
            submap_loop_min_similarity: None,
            submap_loop_match_ratio: 0.8,
            submap_loop_verification_frame_budget: 11,
            submap_loop_verification_correspondence_target: 150,
            camera_centre_refinement: None,
            seam_bundle_adjustment: None,
            submap_loop_bundle_adjustment: true,
            max_parallel_local_builds: 2,
            merge_degenerate_seams: true,
            max_degenerate_seam_merges: 16,
            last_resort_seam_merge: true,
            seam_internal_drift_gate_enabled: true,
            seam_internal_drift_window_count: 2,
            max_seam_internal_drift_disagreement_ratio: 1.15,
            catastrophic_consensus_inlier_ratio_threshold: 0.20,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct HierarchicalSfmSeam {
    pub source_submap_id: u64,
    pub target_submap_id: u64,
    pub shared_point_matches: usize,
    pub sim3_inliers: usize,
    pub sim3_inlier_ratio: f64,
    pub mean_residual_ratio: f64,
    pub essential_rotation_candidates: usize,
    pub essential_rotation_consensus: usize,
    pub essential_rotation_support: usize,
    pub essential_rotation_max_disagreement_deg: f64,
    pub shared_camera_centres: usize,
    pub camera_sim3_inliers: Option<usize>,
    pub camera_sim3_inlier_ratio: Option<f64>,
    pub camera_mean_residual_ratio: Option<f64>,
    pub camera_landmark_log_scale_disagreement: Option<f64>,
    pub camera_landmark_rotation_disagreement_deg: Option<f64>,
    pub camera_refinement_applied: bool,
    pub camera_refinement_rejection: Option<CameraCentreScaleRefinementRejection>,
    pub camera_refinement_abs_log_scale_change: Option<f64>,
    pub camera_refinement_mean_residual_ratio: Option<f64>,
}

#[derive(Debug, Clone)]
pub struct HierarchicalSfmAtlas {
    pub hierarchy: HierarchicalSubmapGraph,
    pub seams: Vec<HierarchicalSfmSeam>,
    /// `None` for a single-submap sequence, which has no global gauge variables.
    pub optimization: Option<HierarchicalSubmapOptimizationResult>,
    pub seam_bundle_adjustment: Option<HierarchicalSeamBaResult>,
    /// Present only when post-BA submap loop closure was explicitly enabled.
    pub loop_closure: Option<HierarchicalLoopClosureResult>,
    /// Accepted loop-welded BA result. `None` when disabled, when no loop edge
    /// was accepted, or when the transactional cost gate retained loop PGO.
    pub loop_bundle_adjustment: Option<HierarchicalSeamBaResult>,
    pub(crate) seam_landmark_links: Vec<HierarchicalSeamLandmarkLink>,
}

#[derive(Debug, Clone)]
pub struct HierarchicalSfmResult {
    pub windows: Vec<SubmapWindow>,
    pub atlas: HierarchicalSfmAtlas,
}

#[derive(Debug, Clone, PartialEq)]
pub enum HierarchicalSfmError {
    SourceFrameCountMismatch {
        ids: usize,
        features: usize,
    },
    Partition(SubmapPartitionError),
    LocalBuild {
        submap_id: u64,
        image_start: usize,
        image_end: usize,
        error: LocalSubmapBuildError,
    },
    Overlap {
        source_submap_id: u64,
        target_submap_id: u64,
        error: SubmapOverlapError,
    },
    Alignment {
        source_submap_id: u64,
        target_submap_id: u64,
        rejection: SubmapSim3Rejection,
    },
    Hierarchy(HierarchicalSubmapGraphError),
    ParallelBuild(String),
    SeamBundleAdjustment(HierarchicalSeamBaError),
    NoSubmaps,
}

impl fmt::Display for HierarchicalSfmError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::SourceFrameCountMismatch { ids, features } => {
                write!(f, "source frame id count {ids} != feature count {features}")
            }
            Self::Partition(error) => write!(f, "submap partition failed: {error}"),
            Self::LocalBuild {
                submap_id,
                image_start,
                image_end,
                error,
            } => write!(
                f,
                "submap {submap_id} build failed for images {image_start}..{image_end}: {error}"
            ),
            Self::Overlap {
                source_submap_id,
                target_submap_id,
                error,
            } => write!(
                f,
                "submap seam {source_submap_id}->{target_submap_id} overlap failed: {error}"
            ),
            Self::Alignment {
                source_submap_id,
                target_submap_id,
                rejection,
            } => write!(
                f,
                "submap seam {source_submap_id}->{target_submap_id} rejected: {:?}",
                rejection.reason
            ),
            Self::Hierarchy(error) => write!(f, "hierarchical graph failed: {error}"),
            Self::ParallelBuild(error) => write!(f, "local submap worker pool failed: {error}"),
            Self::SeamBundleAdjustment(error) => {
                write!(f, "seam bundle adjustment failed: {error}")
            }
            Self::NoSubmaps => write!(f, "hierarchical SfM received no submaps"),
        }
    }
}

impl Error for HierarchicalSfmError {}

impl From<SubmapPartitionError> for HierarchicalSfmError {
    fn from(value: SubmapPartitionError) -> Self {
        Self::Partition(value)
    }
}

impl From<HierarchicalSubmapGraphError> for HierarchicalSfmError {
    fn from(value: HierarchicalSubmapGraphError) -> Self {
        Self::Hierarchy(value)
    }
}

impl From<HierarchicalSeamBaError> for HierarchicalSfmError {
    fn from(value: HierarchicalSeamBaError) -> Self {
        Self::SeamBundleAdjustment(value)
    }
}

/// Build and link an ordered image sequence using only verified image evidence.
pub fn hierarchical_sfm(
    camera: &Camera,
    source_frame_ids: &[u64],
    features: &[FeatureSet],
    pairwise: &[PairwiseMatches],
    pair_rotations: &[PairRotationEvidence],
    partition_hints: &AdaptiveSubmapPartitionHints,
    config: &HierarchicalSfmConfig,
) -> Result<HierarchicalSfmResult, HierarchicalSfmError> {
    if source_frame_ids.len() != features.len() {
        return Err(HierarchicalSfmError::SourceFrameCountMismatch {
            ids: source_frame_ids.len(),
            features: features.len(),
        });
    }
    let windows =
        partition_ordered_submaps(features.len(), pairwise, &config.partition, partition_hints)?;
    if windows.is_empty() {
        return Err(HierarchicalSfmError::NoSubmaps);
    }
    let builder = LocalSubmapBuilder::new(config.local_submap.clone());
    let build_one = |submap_id: u64,
                     window: &SubmapWindow,
                     component_rebuild: bool|
     -> Result<LocalSubmap, HierarchicalSfmError> {
        let range = window.image_range.clone();
        let local_pairs = remap_pairs_to_submap(pairwise, range.clone());
        let build_result = if component_rebuild {
            builder.build_merged_component(
                camera,
                &source_frame_ids[range.clone()],
                &features[range.clone()],
                &local_pairs,
            )
        } else {
            builder.build(
                camera,
                &source_frame_ids[range.clone()],
                &features[range.clone()],
                &local_pairs,
            )
        };
        let result = build_result.map_err(|error| HierarchicalSfmError::LocalBuild {
            submap_id,
            image_start: range.start,
            image_end: range.end,
            error,
        });
        // Unconditional (not gated behind VISLOC_SFM_DEBUG) per-submap build
        // summary: cheap (fields already computed by `builder.build`), and the
        // exhaustive-seam-failure diagnosis showed the previous logs had no
        // per-submap point/reprojection/seed evidence at all, only the planned
        // window list -- see NOROBUSTFIT_CLUSTER_DIAGNOSIS.md.
        if let Ok(submap) = &result {
            eprintln!(
                "hierarchical-submap-built: submap {submap_id} images {range:?} \
                 component_rebuild={component_rebuild} \
                 registered={}/{} points={} mean_reproj_px={:.4} \
                 median_max_parallax_deg={:.4} camera_center_diameter={:.4} \
                 camera_center_step_median={:.6} camera_center_step_max={:.6} \
                 seed_pair_final_distance={:.6} camera_center_window_drift_ratio={:.4} \
                 seed=({}, {}) seed_match_count={}",
                submap.quality.registered_images,
                submap.quality.requested_images,
                submap.landmarks.len(),
                submap.quality.mean_reprojection_px,
                submap.quality.median_max_parallax_deg,
                submap.quality.camera_center_diameter,
                submap.quality.camera_center_step_median,
                submap.quality.camera_center_step_max,
                submap.quality.seed_pair_final_distance,
                submap.quality.camera_center_window_drift_ratio,
                submap.seed_source_frame_i,
                submap.seed_source_frame_j,
                submap.seed_match_count,
            );
        }
        // Unconditional pathology report: the build-time scale-sanity gate
        // (NOROBUSTFIT_CLUSTER_DIAGNOSIS.md §6(a)) already retried on
        // alternate seed candidates inside `builder.build` (§6(b)) before
        // surfacing this; this is the point where the window is actually
        // about to be treated as a build failure and handed to the
        // widen/merge machinery below, exactly like `NoSeedPair`.
        if let Err(HierarchicalSfmError::LocalBuild {
            error:
                LocalSubmapBuildError::QualityRejected {
                    reason: crate::LocalSubmapRejectionReason::ImplausibleScale,
                    quality,
                },
            ..
        }) = &result
        {
            let step_threshold = builder
                .config
                .quality
                .max_camera_center_displacement_outlier_ratio;
            let step_ratio = crate::local_submap::camera_center_step_outlier_ratio(quality);
            let seed_drift_threshold = builder.config.quality.max_seed_pair_scale_drift_ratio;
            let seed_drift_ratio = crate::local_submap::seed_pair_scale_drift_ratio(quality);
            let window_drift_threshold =
                builder.config.quality.max_camera_center_window_drift_ratio;
            let window_drift_count = builder.config.quality.camera_center_drift_window_count;
            eprintln!(
                "hierarchical-scale-pathology: submap {submap_id} images {range:?} \
                 component_rebuild={component_rebuild} \
                 diameter={:.4} median_step={:.6} ratio={:.4} threshold={:.4} \
                 seed_pair_final_distance={:.6} seed_drift_ratio={:.4} \
                 seed_drift_threshold={:.4} window_count={} window_drift_ratio={:.4} \
                 window_drift_threshold={:.4}; treating as build failure",
                quality.camera_center_diameter,
                quality.camera_center_step_median,
                step_ratio,
                step_threshold,
                quality.seed_pair_final_distance,
                seed_drift_ratio,
                seed_drift_threshold,
                window_drift_count,
                quality.camera_center_window_drift_ratio,
                window_drift_threshold,
            );
            if quality.camera_center_window_drift_ratio > window_drift_threshold {
                eprintln!(
                    "hierarchical-scale-drift: submap {submap_id} images {range:?} \
                     component_rebuild={component_rebuild} \
                     window_count={window_drift_count} \
                     window_drift_ratio={:.4} window_drift_threshold={:.4}; \
                     treating as build failure",
                    quality.camera_center_window_drift_ratio, window_drift_threshold,
                );
            }
        }
        result
    };
    // First pass: attempt every planned window, in parallel, without failing
    // fast — widening (below) needs to see every window's outcome, not just
    // the first failure, and this keeps the common (all-succeed) case exactly
    // as fast as before.
    let worker_count = config.max_parallel_local_builds.max(1).min(windows.len());
    let initial: Vec<Result<LocalSubmap, HierarchicalSfmError>> = if worker_count == 1 {
        windows
            .iter()
            .enumerate()
            .map(|(id, window)| build_one(id as u64, window, false))
            .collect()
    } else {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(worker_count)
            .build()
            .map_err(|error| HierarchicalSfmError::ParallelBuild(error.to_string()))?;
        pool.install(|| {
            windows
                .par_iter()
                .enumerate()
                .map(|(id, window)| build_one(id as u64, window, false))
                .collect()
        })
    };
    let mut next_submap_id = windows.len() as u64;
    let (final_windows, submaps) = if config.partition.widen_unseedable_windows {
        // A submap that failed only because *this* window has no seed pair
        // (e.g. it falls inside a near-static hover span too short-parallax
        // for the strict seeding gates) is not a broken-input failure — a
        // wider window covering the same frames usually is seedable. Reuse
        // every first-pass result that survives unchanged (keyed by its
        // original image range) and only rebuild the windows that widening
        // actually reshapes.
        let mut cache: HashMap<(usize, usize), Result<LocalSubmap, HierarchicalSfmError>> = windows
            .iter()
            .map(|window| (window.image_range.start, window.image_range.end))
            .zip(initial)
            .collect();
        let max_merges = config.partition.max_widen_merges;
        let max_windows_per_step = config.partition.max_widen_windows_per_step;
        let min_post_widen_overlap_images = config.partition.min_post_widen_overlap_images;
        let outputs = widen_and_build(
            windows,
            max_merges,
            max_windows_per_step,
            min_post_widen_overlap_images,
            |window: &SubmapWindow| {
                let key = (window.image_range.start, window.image_range.end);
                if let Some(cached) = cache.remove(&key) {
                    cached
                } else {
                    let submap_id = next_submap_id;
                    next_submap_id += 1;
                    build_one(submap_id, window, false)
                }
            },
            is_build_error_widenable,
            is_no_seed_pair_build_error,
            |merge_units, absorbed_windows, before, after, reason| match reason {
                // Fires for `NoSeedPair` and every build-time quality
                // rejection. Any same-window multi-seed retry applicable to
                // the rejection has already run inside `builder.build`; this
                // line reports the widen/merge machinery's response.
                WidenMergeReason::UnbuildableWindow => eprintln!(
                    "hierarchical-widen: images {before:?} failed to build (NoSeedPair or \
                     QualityRejected); absorbing neighbouring window(s) -> images {after:?} \
                     (absorbed_windows_this_step={absorbed_windows}, \
                     merge_units={merge_units}/{max_merges}, \
                     no_seed_step_cap={max_windows_per_step})"
                ),
                WidenMergeReason::PostWidenOverlapSafety => eprintln!(
                    "hierarchical-widen: images {before:?} still bordered a live neighbour \
                     entirely inside the span diagnosed unseedable (min_post_widen_overlap_images \
                     = {min_post_widen_overlap_images}); absorbing that neighbour too -> images \
                     {after:?} (absorbed_windows_this_step={absorbed_windows}, \
                     merge_units={merge_units}/{max_merges}, no_seed_step_reset=1)"
                ),
            },
        )?;
        outputs.into_iter().unzip()
    } else {
        let submaps = initial.into_iter().collect::<Result<Vec<_>, _>>()?;
        (windows, submaps)
    };
    let (final_windows, mut atlas) = if config.merge_degenerate_seams {
        merge_degenerate_seams_and_optimize(
            final_windows,
            submaps,
            pair_rotations,
            config,
            |submap_id, window| build_one(submap_id, window, true),
            &mut next_submap_id,
        )?
    } else {
        match optimize_independent_submaps(submaps.clone(), pair_rotations, config) {
            Ok(atlas) => (final_windows, atlas),
            Err(error) => {
                report_all_failing_seams(&final_windows, &submaps, pair_rotations, config);
                return Err(error);
            }
        }
    };
    let loop_output = crate::hierarchical_loop_closure::maybe_close_hierarchical_loops(
        config.submap_loop_closure,
        &mut atlas.hierarchy,
        camera,
        source_frame_ids,
        features,
        config.submap_constraint_band,
        config.submap_loop_top_k,
        config.submap_loop_min_similarity,
        config.submap_loop_match_ratio,
        config.submap_loop_min_matches,
        config.submap_loop_verification_frame_budget,
        config.submap_loop_verification_correspondence_target,
        config.max_parallel_local_builds,
        &config.overlap,
        &config.alignment,
        &config.pose_graph,
    )?;
    if let Some(loop_output) = loop_output {
        if !loop_output.landmark_links.is_empty() {
            let mut welded_links = atlas.seam_landmark_links.clone();
            welded_links.extend(loop_output.landmark_links);
            let loop_ba_config = config.seam_bundle_adjustment.unwrap_or_default();
            let first_seam_mean_cost = atlas
                .seam_bundle_adjustment
                .as_ref()
                .map(|result| result.final_cost / result.observation_count.max(1) as f64);
            atlas.loop_bundle_adjustment =
                crate::hierarchical_seam_ba::refine_hierarchical_loop_welds(
                    config.submap_loop_bundle_adjustment,
                    &mut atlas.hierarchy,
                    &welded_links,
                    &loop_ba_config,
                    first_seam_mean_cost,
                )?;
        } else {
            eprintln!("hierarchical-loop-ba: skipped reason=no-accepted-loop-welds");
        }
        atlas.loop_closure = Some(loop_output.result);
    }
    Ok(HierarchicalSfmResult {
        windows: final_windows,
        atlas,
    })
}

/// Whether a `build_one` failure should be absorbed by [`widen_and_build`]'s
/// merge-and-rebuild retry rather than failing the whole run immediately.
/// Two classes qualify, both meaning "no window of this exact size/position
/// works, but a wider one may provide enough usable evidence":
///
/// - [`IncrementalSfmError::NoSeedPair`]: no verified pair inside the window
///   cleared the seeding gates at all (e.g. a near-static hover span).
/// - Any [`LocalSubmapBuildError::QualityRejected`]. The builder has already
///   performed any applicable bounded same-window multi-seed retry before
///   returning this error; widening then uses the existing merge cap and
///   overlap-safety budget without relaxing a quality threshold.
const fn is_build_error_widenable(error: &HierarchicalSfmError) -> bool {
    matches!(
        error,
        HierarchicalSfmError::LocalBuild {
            error: LocalSubmapBuildError::Reconstruction(IncrementalSfmError::NoSeedPair),
            ..
        }
    ) || matches!(
        error,
        HierarchicalSfmError::LocalBuild {
            error: LocalSubmapBuildError::QualityRejected { .. },
            ..
        }
    )
}

const fn is_no_seed_pair_build_error(error: &HierarchicalSfmError) -> bool {
    matches!(
        error,
        HierarchicalSfmError::LocalBuild {
            error: LocalSubmapBuildError::Reconstruction(IncrementalSfmError::NoSeedPair),
            ..
        }
    )
}

/// Whether an [`SubmapSim3RejectionReason`] is even *eligible* for the
/// seam-time internal-drift remediation below (before the cross-submap
/// diagnostic is consulted at all). These two reasons are exactly the ones
/// a genuine internal reconstruction defect on one side can produce
/// (`LOWINLIERRATIO_DIAGNOSIS.md`); every other reason (too few
/// correspondences, scale out of bounds, rotation inconsistent, ...) is left
/// fail-fast regardless of what the diagnostic says.
const fn is_seam_rejection_reason_drift_eligible(reason: SubmapSim3RejectionReason) -> bool {
    matches!(
        reason,
        SubmapSim3RejectionReason::LowInlierRatio | SubmapSim3RejectionReason::NoRobustFit
    )
}

/// Bounds-checked wrapper around
/// [`crate::submap_overlap::seam_step_shape_diagnostic`] for a seam named by
/// `source_submap_id`/`target_submap_id` (as `optimize_independent_submaps`
/// assigns them: positional, `target == source + 1`). `None` if the ids are
/// not a valid adjacent pair into `submaps` (defensive; `optimize_independent_submaps`
/// never actually produces such ids) or the diagnostic itself has too few
/// shared frames to compute.
fn seam_internal_drift_diagnostic(
    submaps: &[LocalSubmap],
    source_submap_id: u64,
    target_submap_id: u64,
    config: &HierarchicalSfmConfig,
) -> Option<crate::submap_overlap::SeamStepShapeDiagnostic> {
    if target_submap_id != source_submap_id + 1 {
        return None;
    }
    let index = source_submap_id as usize;
    if index + 1 >= submaps.len() {
        return None;
    }
    crate::submap_overlap::seam_step_shape_diagnostic(
        &submaps[index],
        &submaps[index + 1],
        config.seam_internal_drift_window_count,
    )
}

#[derive(Debug, Clone, Copy)]
enum SeamMergeRoute {
    Degenerate,
    Residual(Option<crate::submap_overlap::SeamStepShapeDiagnostic>),
    Drift(crate::submap_overlap::SeamStepShapeDiagnostic),
    Consensus(crate::submap_overlap::SeamStepShapeDiagnostic),
    LastResort(Option<crate::submap_overlap::SeamStepShapeDiagnostic>),
}

impl SeamMergeRoute {
    const fn is_last_resort(self) -> bool {
        matches!(self, Self::LastResort(_))
    }

    const fn log_family(self) -> &'static str {
        match self {
            Self::Degenerate => "hierarchical-seam-merge",
            Self::Residual(_) => "hierarchical-seam-residual-merge",
            Self::Drift(_) => "hierarchical-seam-drift-merge",
            Self::Consensus(_) => "hierarchical-seam-consensus-merge",
            Self::LastResort(_) => "hierarchical-seam-lastresort-merge",
        }
    }
}

#[derive(Debug, Clone)]
struct SeamMergeCandidate {
    seam_index: usize,
    rejection: SubmapSim3Rejection,
    route: SeamMergeRoute,
}

/// Last-resort structural handling is deliberately broad across geometric
/// alignment failures, but it must not hide malformed correspondence data.
/// `TooFewCorrespondences` is eligible even with no correspondence evidence:
/// such a seam cannot be aligned, so a correspondence-free component rebuild
/// is its only possible remediation.
const fn is_last_resort_seam_rejection_eligible(rejection: &SubmapSim3Rejection) -> bool {
    !matches!(
        rejection.reason,
        SubmapSim3RejectionReason::NonFinitePoint
            | SubmapSim3RejectionReason::NonUniqueCorrespondences
    )
}

/// Exhaustively inspect the current adjacent seam chain and retain every
/// failure that has a structural merge route. Unlike
/// [`optimize_independent_submaps`], this deliberately does not stop at the
/// first rejection: a fatal/non-routable seam must not hide actionable drift,
/// residual, degenerate, or catastrophic-consensus seams later in the round.
fn seam_merge_candidates(
    submaps: &[LocalSubmap],
    pair_rotations: &[PairRotationEvidence],
    config: &HierarchicalSfmConfig,
) -> Vec<SeamMergeCandidate> {
    let mut candidates = Vec::new();
    for index in 0..submaps.len().saturating_sub(1) {
        let source_id = index as u64;
        let target_id = source_id + 1;
        let Ok(overlap) = collect_submap_overlap_evidence(
            &submaps[index],
            &submaps[index + 1],
            pair_rotations,
            &config.overlap,
        ) else {
            continue;
        };
        let Err(rejection) = estimate_submap_sim3_constraint(
            source_id,
            target_id,
            &overlap.point_matches,
            &overlap.target_from_source_rotation,
            &config.alignment,
        ) else {
            continue;
        };
        let diagnostic = seam_internal_drift_diagnostic(submaps, source_id, target_id, config);
        let route = if matches!(
            rejection.reason,
            SubmapSim3RejectionReason::DegenerateSourceGeometry
                | SubmapSim3RejectionReason::DegenerateTargetGeometry
        ) {
            Some(SeamMergeRoute::Degenerate)
        } else if rejection.reason == SubmapSim3RejectionReason::HighMeanResidual {
            Some(SeamMergeRoute::Residual(diagnostic))
        } else if config.seam_internal_drift_gate_enabled
            && is_seam_rejection_reason_drift_eligible(rejection.reason)
            && diagnostic.is_some_and(|value| {
                value.disagreement_ratio > config.max_seam_internal_drift_disagreement_ratio
            })
        {
            Some(SeamMergeRoute::Drift(
                diagnostic.expect("drift guard confirmed a diagnostic"),
            ))
        } else if rejection.reason == SubmapSim3RejectionReason::LowInlierRatio
            && rejection.inlier_ratio < config.catastrophic_consensus_inlier_ratio_threshold
            && diagnostic.is_some_and(|value| {
                value.disagreement_ratio <= config.max_seam_internal_drift_disagreement_ratio
            })
        {
            Some(SeamMergeRoute::Consensus(
                diagnostic.expect("consensus guard confirmed a diagnostic"),
            ))
        } else if config.last_resort_seam_merge
            && is_last_resort_seam_rejection_eligible(&rejection)
        {
            Some(SeamMergeRoute::LastResort(diagnostic))
        } else {
            None
        };
        if let Some(route) = route {
            candidates.push(SeamMergeCandidate {
                seam_index: index,
                rejection,
                route,
            });
        }
    }
    candidates
}

/// Retry alignment around every structurally merge-eligible seam. Adjacent
/// failing seams form a connected component and are rebuilt as one union
/// window: failures `i->i+1` and `i+1->i+2` therefore cause one build of
/// submaps `i..=i+2`, not two pair rebuilds. Collapsing a component containing
/// `k` seams consumes `k` units from the existing pair-merge budget. A
/// component build first exhausts the builder's bounded alternate-seed retry
/// for any quality rejection. If it still returns `QualityRejected` or
/// `NoSeedPair`, one live successor (or the predecessor at the tail) is
/// absorbed and the component is rebuilt; each absorption consumes one more
/// unit from that same budget. The last build error is returned unchanged once
/// no neighbour or budget remains.
fn merge_degenerate_seams_and_optimize(
    mut windows: Vec<SubmapWindow>,
    mut submaps: Vec<LocalSubmap>,
    pair_rotations: &[PairRotationEvidence],
    config: &HierarchicalSfmConfig,
    mut build_one: impl FnMut(u64, &SubmapWindow) -> Result<LocalSubmap, HierarchicalSfmError>,
    next_submap_id: &mut u64,
) -> Result<(Vec<SubmapWindow>, HierarchicalSfmAtlas), HierarchicalSfmError> {
    let max_merges = config.max_degenerate_seam_merges;
    let mut merges_used = 0usize;
    loop {
        let candidates = seam_merge_candidates(&submaps, pair_rotations, config);
        let remaining_budget = max_merges.saturating_sub(merges_used);
        // A fallback seam must never jump ahead of a specific diagnosis later
        // in the chain. Only when the exhaustive scan found no specific route
        // do last-resort candidates participate in component selection.
        let specific_candidates = candidates
            .iter()
            .filter(|candidate| !candidate.route.is_last_resort())
            .cloned()
            .collect::<Vec<_>>();
        let last_resort_candidates;
        let round_candidates = if specific_candidates.is_empty() {
            last_resort_candidates = candidates
                .iter()
                .filter(|candidate| candidate.route.is_last_resort())
                .cloned()
                .collect::<Vec<_>>();
            &last_resort_candidates
        } else {
            &specific_candidates
        };
        // Candidates are emitted in seam order. Select the first maximal run
        // of adjacent seam indices that fits the remaining pair-merge budget.
        let mut selected = None;
        let mut cursor = 0;
        while cursor < round_candidates.len() {
            let start = cursor;
            cursor += 1;
            while cursor < round_candidates.len()
                && round_candidates[cursor].seam_index
                    == round_candidates[cursor - 1].seam_index + 1
            {
                cursor += 1;
            }
            let cost =
                round_candidates[cursor - 1].seam_index - round_candidates[start].seam_index + 1;
            if cost <= remaining_budget {
                selected = Some(&round_candidates[start..cursor]);
                break;
            }
        }
        if let Some(component) = selected {
            let mut first_index = component[0].seam_index;
            let last_seam_index = component.last().expect("component is non-empty").seam_index;
            let mut last_window_index = last_seam_index + 1;
            let component_cost = last_seam_index - first_index + 1;
            let merged_range =
                windows[first_index].image_range.start..windows[last_window_index].image_range.end;
            let mut merged_window = SubmapWindow {
                image_range: merged_range.clone(),
                outgoing_seam_support: windows[last_window_index].outgoing_seam_support,
            };
            let next_merges_used = merges_used + component_cost;
            for candidate in component {
                let index = candidate.seam_index;
                let source_id = index as u64;
                let target_id = source_id + 1;
                let source_range = windows[index].image_range.clone();
                let target_range = windows[index + 1].image_range.clone();
                match candidate.route {
                    SeamMergeRoute::Degenerate => eprintln!(
                        "{}: submaps {source_id}..{target_id} \
                         (images {source_range:?}..{target_range:?}) degenerate seam ({:?}); \
                         component merging -> images {merged_range:?} \
                         (merges {next_merges_used}/{max_merges})",
                        candidate.route.log_family(),
                        candidate.rejection.reason,
                    ),
                    SeamMergeRoute::Residual(diagnostic) => eprintln!(
                        "{}: submaps {source_id}..{target_id} \
                         (images {source_range:?}..{target_range:?}) high-residual seam \
                         (mean_residual_ratio={:?} drift_disagreement_ratio={:?} \
                         source_landmarks={} target_landmarks={}); component merging -> images \
                         {merged_range:?} (merges {next_merges_used}/{max_merges})",
                        candidate.route.log_family(),
                        candidate.rejection.mean_residual_ratio,
                        diagnostic.map(|value| value.disagreement_ratio),
                        submaps[index].landmarks.len(),
                        submaps[index + 1].landmarks.len(),
                    ),
                    SeamMergeRoute::Drift(diagnostic) => eprintln!(
                        "{}: submaps {source_id}..{target_id} \
                         (images {source_range:?}..{target_range:?}) internally-inconsistent seam \
                         (reason={:?} defective_side={:?} shared_frames={} \
                         source_change_factor={:.4} target_change_factor={:.4} \
                         disagreement_ratio={:.4} threshold={:.4}); component merging -> images \
                         {merged_range:?} (merges {next_merges_used}/{max_merges})",
                        candidate.route.log_family(),
                        candidate.rejection.reason,
                        diagnostic.defective_side,
                        diagnostic.shared_frames,
                        diagnostic.source_change_factor,
                        diagnostic.target_change_factor,
                        diagnostic.disagreement_ratio,
                        config.max_seam_internal_drift_disagreement_ratio,
                    ),
                    SeamMergeRoute::Consensus(diagnostic) => eprintln!(
                        "{}: submaps {source_id}..{target_id} \
                         (images {source_range:?}..{target_range:?}) catastrophic consensus \
                         (inlier_ratio={:.4} severe_threshold={:.4} disagreement_ratio={:.4} \
                         drift_threshold={:.4} source_landmarks={} target_landmarks={}); \
                         component merging -> images {merged_range:?} \
                         (merges {next_merges_used}/{max_merges})",
                        candidate.route.log_family(),
                        candidate.rejection.inlier_ratio,
                        config.catastrophic_consensus_inlier_ratio_threshold,
                        diagnostic.disagreement_ratio,
                        config.max_seam_internal_drift_disagreement_ratio,
                        submaps[index].landmarks.len(),
                        submaps[index + 1].landmarks.len(),
                    ),
                    SeamMergeRoute::LastResort(diagnostic) => eprintln!(
                        "{}: submaps {source_id}..{target_id} \
                         (images {source_range:?}..{target_range:?}) fatal seam after specific \
                         routes (reason={:?} correspondences={} inliers={} inlier_ratio={:.4} \
                         mean_residual_ratio={:?} rotation_disagreement_deg={:?} \
                         leave_one_out_log_scale_mad={:?} drift_diagnostic={:?} \
                         source_landmarks={} target_landmarks={}); component merging -> images \
                         {merged_range:?} (merges {next_merges_used}/{max_merges})",
                        candidate.route.log_family(),
                        candidate.rejection.reason,
                        candidate.rejection.correspondence_count,
                        candidate.rejection.inlier_count,
                        candidate.rejection.inlier_ratio,
                        candidate.rejection.mean_residual_ratio,
                        candidate.rejection.rotation_disagreement_deg,
                        candidate.rejection.leave_one_out_log_scale_mad,
                        diagnostic,
                        submaps[index].landmarks.len(),
                        submaps[index + 1].landmarks.len(),
                    ),
                }
            }
            merges_used = next_merges_used;
            let merged_submap = loop {
                let submap_id = *next_submap_id;
                *next_submap_id += 1;
                match build_one(submap_id, &merged_window) {
                    Ok(submap) => break submap,
                    Err(error) if is_build_error_widenable(&error) => {
                        let before = merged_window.image_range.clone();
                        if merges_used >= max_merges
                            || (first_index == 0 && last_window_index + 1 == windows.len())
                        {
                            eprintln!(
                                "hierarchical-widen: component_rebuild=true images {before:?} \
                                 remediation exhausted (merge_units={merges_used}/{max_merges}); \
                                 fatal build error={error:?}"
                            );
                            return Err(error);
                        }

                        // Match normal-window widening: absorb the successor
                        // when one exists, otherwise absorb the predecessor.
                        if last_window_index + 1 < windows.len() {
                            last_window_index += 1;
                            merged_window.image_range.end =
                                windows[last_window_index].image_range.end;
                            merged_window.outgoing_seam_support =
                                windows[last_window_index].outgoing_seam_support;
                        } else {
                            first_index -= 1;
                            merged_window.image_range.start =
                                windows[first_index].image_range.start;
                        }
                        merges_used += 1;
                        eprintln!(
                            "hierarchical-widen: component_rebuild=true images {before:?} failed \
                             to build (NoSeedPair or QualityRejected) after alternate-seed \
                             retries; absorbing neighbouring window -> images {:?} \
                             (absorbed_windows_this_step=1, \
                             merge_units={merges_used}/{max_merges}) error={error:?}",
                            merged_window.image_range,
                        );
                    }
                    Err(error) => return Err(error),
                }
            };
            windows.splice(first_index..=last_window_index, [merged_window]);
            submaps.splice(first_index..=last_window_index, [merged_submap]);
            continue;
        }

        match optimize_independent_submaps(submaps.clone(), pair_rotations, config) {
            Ok(atlas) => return Ok((windows, atlas)),
            Err(error) => {
                report_all_failing_seams(&windows, &submaps, pair_rotations, config);
                return Err(error);
            }
        }
    }
}

/// Evaluate every seam between adjacent `submaps` (the same per-seam overlap
/// and Sim3 alignment pipeline [`optimize_independent_submaps`] runs) and
/// `eprintln` each one that would be rejected, instead of stopping at the
/// first rejection. Purely for observability — it does not affect what error
/// any caller returns, and re-running the same (deterministic) pipeline here
/// costs nothing beyond CPU time already about to be spent failing anyway.
/// Returns the number of failing seams found, for a one-line summary.
///
/// Gate for the `LowInlierRatio`-seam-cluster diagnosis's verbose per-match /
/// per-frame-step dump in [`debug_low_inlier_seam`] (env-checked per call
/// site; not a hot loop). Off by default so ordinary runs are unaffected;
/// set `VISLOC_SEAM_DEBUG=1` to see it. See the investigation this supports:
/// the follow-on to `NOROBUSTFIT_CLUSTER_DIAGNOSIS.md` after its recommended
/// scale-sanity gate (§6(a)) eliminated the `NoRobustFit` cluster but left a
/// `LowInlierRatio` cluster (seams with plenty of correspondences but only
/// partial RANSAC consensus) unexplained.
fn seam_debug_enabled() -> bool {
    std::env::var_os("VISLOC_SEAM_DEBUG").is_some()
}

/// Diagnostic-only (never consulted by any accept/reject decision): for a
/// `LowInlierRatio`-rejected seam, dump (a) every landmark correspondence's
/// residual against the winning consensus refit, tagged with each side's
/// median observed `source_frame_id` ("frame locus") -- inliers/outliers
/// clustering by frame locus is a drift signature (part of the overlap
/// disagrees), while a scattered pattern points at matching contamination
/// instead -- and (b) the two submaps' own camera-centre step sequences
/// restricted to the frames they share, each self-normalized by its own
/// median step over that shared span, so the two step-shape profiles can be
/// compared directly (a genuine shape disagreement means no `Sim3` can
/// reconcile the two independent reconstructions, no matter how the seam is
/// rebuilt).
fn debug_low_inlier_seam(
    source_id: u64,
    target_id: u64,
    source_submap: &LocalSubmap,
    target_submap: &LocalSubmap,
    point_matches: &[SubmapPointMatch],
    alignment_config: &SubmapSim3AlignmentConfig,
) {
    let source_locus = landmark_frame_locus(source_submap);
    let target_locus = landmark_frame_locus(target_submap);
    match crate::submap_alignment::diagnose_sim3_residuals(point_matches, alignment_config) {
        Some(diagnostics) => {
            for (match_index, (point_match, &(residual, is_inlier))) in
                point_matches.iter().zip(diagnostics.iter()).enumerate()
            {
                eprintln!(
                    "hierarchical-seam-debug-match: submap {source_id}->{target_id} \
                     match={match_index} source_landmark={} target_landmark={} \
                     source_frame_locus={:?} target_frame_locus={:?} residual={:.6} inlier={}",
                    point_match.source_landmark_id,
                    point_match.target_landmark_id,
                    source_locus.get(&point_match.source_landmark_id),
                    target_locus.get(&point_match.target_landmark_id),
                    residual,
                    is_inlier,
                );
            }
        }
        None => eprintln!(
            "hierarchical-seam-debug: submap {source_id}->{target_id} \
             diagnose_sim3_residuals found no RANSAC hypothesis at all"
        ),
    }

    let camera_matches = shared_camera_center_matches(source_submap, target_submap);
    let mut by_frame: HashMap<u64, (nalgebra::Point3<f64>, nalgebra::Point3<f64>)> = HashMap::new();
    for point_match in &camera_matches {
        // `shared_camera_center_matches` reuses the landmark-id fields to
        // carry the shared `source_frame_id` on both sides (see that
        // function's doc); they are always equal here.
        by_frame.insert(
            point_match.source_landmark_id,
            (point_match.source_point, point_match.target_point),
        );
    }
    let mut shared_frames: Vec<u64> = by_frame.keys().copied().collect();
    shared_frames.sort_unstable();
    let source_steps: Vec<f64> = shared_frames
        .windows(2)
        .map(|pair| {
            let (a, _) = by_frame[&pair[0]];
            let (b, _) = by_frame[&pair[1]];
            (b - a).norm()
        })
        .collect();
    let target_steps: Vec<f64> = shared_frames
        .windows(2)
        .map(|pair| {
            let (_, a) = by_frame[&pair[0]];
            let (_, b) = by_frame[&pair[1]];
            (b - a).norm()
        })
        .collect();
    let source_median_step = median_f64(source_steps.clone());
    let target_median_step = median_f64(target_steps.clone());
    eprintln!(
        "hierarchical-seam-debug-steps: submap {source_id}->{target_id} shared_frames={} \
         source_median_step={:.6} target_median_step={:.6}",
        shared_frames.len(),
        source_median_step,
        target_median_step,
    );
    for (index, pair) in shared_frames.windows(2).enumerate() {
        let source_step = source_steps[index];
        let target_step = target_steps[index];
        let source_step_norm = if source_median_step > 1.0e-12 {
            source_step / source_median_step
        } else {
            f64::NAN
        };
        let target_step_norm = if target_median_step > 1.0e-12 {
            target_step / target_median_step
        } else {
            f64::NAN
        };
        eprintln!(
            "hierarchical-seam-debug-step: submap {source_id}->{target_id} \
             frames {}->{} source_step={:.6} source_step_norm={:.4} \
             target_step={:.6} target_step_norm={:.4}",
            pair[0], pair[1], source_step, source_step_norm, target_step, target_step_norm,
        );
    }
}

/// Median observed `source_frame_id` per landmark, as an `f64` (even-length
/// interpolation is meaningless for frame ids but harmless -- this is a
/// diagnostic locus label, not consumed by any decision).
fn landmark_frame_locus(submap: &LocalSubmap) -> HashMap<u64, f64> {
    submap
        .landmarks
        .iter()
        .map(|landmark| {
            let mut ids: Vec<f64> = landmark
                .observations
                .iter()
                .map(|observation| observation.source_frame_id as f64)
                .collect();
            ids.sort_by(f64::total_cmp);
            let median = if ids.is_empty() {
                f64::NAN
            } else {
                let middle = ids.len() / 2;
                if ids.len() % 2 == 0 {
                    (ids[middle - 1] + ids[middle]) * 0.5
                } else {
                    ids[middle]
                }
            };
            (landmark.local_landmark_id, median)
        })
        .collect()
}

fn median_f64(mut values: Vec<f64>) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    values.sort_by(f64::total_cmp);
    let middle = values.len() / 2;
    if values.len() % 2 == 0 {
        (values[middle - 1] + values[middle]) * 0.5
    } else {
        values[middle]
    }
}

fn report_all_failing_seams(
    windows: &[SubmapWindow],
    submaps: &[LocalSubmap],
    pair_rotations: &[PairRotationEvidence],
    config: &HierarchicalSfmConfig,
) -> usize {
    if submaps.len() < 2 {
        return 0;
    }
    let mut failing = 0usize;
    for index in 0..submaps.len() - 1 {
        let source_id = index as u64;
        let target_id = source_id + 1;
        let source_range = windows.get(index).map(|w| w.image_range.clone());
        let target_range = windows.get(index + 1).map(|w| w.image_range.clone());
        match collect_submap_overlap_evidence(
            &submaps[index],
            &submaps[index + 1],
            pair_rotations,
            &config.overlap,
        ) {
            Err(error) => {
                failing += 1;
                eprintln!(
                    "hierarchical-seam-failure: submap {source_id}->{target_id} \
                     (images {source_range:?}->{target_range:?}) overlap rejected: {error:?}"
                );
            }
            Ok(overlap) => {
                // Unconditional cross-submap internal-consistency diagnostic
                // (both pass and fail): calibration evidence for
                // `max_seam_internal_drift_disagreement_ratio`, and the same
                // computation the live gate below consults once wired. Cheap
                // (O(shared frames), ~72 here) so this runs for every seam,
                // not just failing ones, unlike the alignment-rejection logs
                // below.
                if let Some(diagnostic) = crate::submap_overlap::seam_step_shape_diagnostic(
                    &submaps[index],
                    &submaps[index + 1],
                    config.seam_internal_drift_window_count,
                ) {
                    eprintln!(
                        "hierarchical-seam-internal-consistency: submap {source_id}->{target_id} \
                         shared_frames={} source_change_factor={:.4} target_change_factor={:.4} \
                         disagreement_ratio={:.4} defective_side={:?}",
                        diagnostic.shared_frames,
                        diagnostic.source_change_factor,
                        diagnostic.target_change_factor,
                        diagnostic.disagreement_ratio,
                        diagnostic.defective_side,
                    );
                }
                if let Err(rejection) = estimate_submap_sim3_constraint(
                    source_id,
                    target_id,
                    &overlap.point_matches,
                    &overlap.target_from_source_rotation,
                    &config.alignment,
                ) {
                    failing += 1;
                    // Per-side landmark totals give scale to `correspondence_count`
                    // (e.g. 888 shared out of how many landmarks each submap holds
                    // in total). The naive scale ratio is a cheap diagnostic
                    // independent of RANSAC: `median_pairwise_distance` over *all*
                    // correspondences on each side, not just inliers, so it is
                    // still informative when `inlier_count == 0` (`NoRobustFit`)
                    // and RANSAC itself produced no scale estimate at all. A ratio
                    // far from 1 with a `NoRobustFit`/`LowInlierRatio` reason
                    // points at a genuine scale/geometry disagreement between the
                    // two submaps' independent reconstructions rather than pure
                    // outlier noise.
                    let source_points = overlap
                        .point_matches
                        .iter()
                        .map(|m| m.source_point)
                        .collect::<Vec<_>>();
                    let target_points = overlap
                        .point_matches
                        .iter()
                        .map(|m| m.target_point)
                        .collect::<Vec<_>>();
                    let source_scene_scale =
                        crate::submap_alignment::median_pairwise_distance(&source_points);
                    let target_scene_scale =
                        crate::submap_alignment::median_pairwise_distance(&target_points);
                    let naive_scale_ratio = if source_scene_scale > 1.0e-12 {
                        Some(target_scene_scale / source_scene_scale)
                    } else {
                        None
                    };
                    eprintln!(
                        "hierarchical-seam-failure: submap {source_id}->{target_id} \
                         (images {source_range:?}->{target_range:?}) alignment rejected: \
                         reason={:?} correspondence_count={} inlier_count={} inlier_ratio={:.4} \
                         mean_residual_ratio={:?} rotation_disagreement_deg={:?} \
                         leave_one_out_log_scale_mad={:?} source_landmarks={} target_landmarks={} \
                         source_scene_scale={:.6} target_scene_scale={:.6} naive_scale_ratio={:?}",
                        rejection.reason,
                        rejection.correspondence_count,
                        rejection.inlier_count,
                        rejection.inlier_ratio,
                        rejection.mean_residual_ratio,
                        rejection.rotation_disagreement_deg,
                        rejection.leave_one_out_log_scale_mad,
                        submaps[index].landmarks.len(),
                        submaps[index + 1].landmarks.len(),
                        source_scene_scale,
                        target_scene_scale,
                        naive_scale_ratio,
                    );
                    if seam_debug_enabled()
                        && rejection.reason == SubmapSim3RejectionReason::LowInlierRatio
                    {
                        debug_low_inlier_seam(
                            source_id,
                            target_id,
                            &submaps[index],
                            &submaps[index + 1],
                            &overlap.point_matches,
                            &config.alignment,
                        );
                    }
                }
            }
        }
    }
    eprintln!(
        "hierarchical-seam-failure-summary: {failing} of {} seam(s) failed \
         (all structurally routable components were considered before the \
         remaining fatal failure was returned)",
        submaps.len() - 1
    );
    failing
}

/// Link already reconstructed independent submaps. This is also the testable
/// transaction boundary for parallel/local builders: no node transform is
/// committed until every adjacent R2 seam is verified and the R3 solve passes.
pub fn optimize_independent_submaps(
    submaps: Vec<LocalSubmap>,
    pair_rotations: &[PairRotationEvidence],
    config: &HierarchicalSfmConfig,
) -> Result<HierarchicalSfmAtlas, HierarchicalSfmError> {
    if submaps.is_empty() {
        return Err(HierarchicalSfmError::NoSubmaps);
    }
    if submaps.len() == 1 {
        let root = submaps.into_iter().next().expect("length checked above");
        return Ok(HierarchicalSfmAtlas {
            hierarchy: HierarchicalSubmapGraph::new(0, root),
            seams: Vec::new(),
            optimization: None,
            seam_bundle_adjustment: None,
            loop_closure: None,
            loop_bundle_adjustment: None,
            seam_landmark_links: Vec::new(),
        });
    }

    let mut constraints = Vec::with_capacity(submaps.len() - 1);
    let mut seam_links = Vec::new();
    let mut seams = Vec::with_capacity(submaps.len() - 1);
    for index in 0..submaps.len() - 1 {
        let source_id = index as u64;
        let target_id = source_id + 1;
        let overlap = collect_submap_overlap_evidence(
            &submaps[index],
            &submaps[index + 1],
            pair_rotations,
            &config.overlap,
        )
        .map_err(|error| HierarchicalSfmError::Overlap {
            source_submap_id: source_id,
            target_submap_id: target_id,
            error,
        })?;
        let landmark_constraint = estimate_submap_sim3_constraint(
            source_id,
            target_id,
            &overlap.point_matches,
            &overlap.target_from_source_rotation,
            &config.alignment,
        )
        .map_err(|rejection| HierarchicalSfmError::Alignment {
            source_submap_id: source_id,
            target_submap_id: target_id,
            rejection,
        })?;
        let camera_matches = shared_camera_center_matches(&submaps[index], &submaps[index + 1]);
        let camera_constraint = estimate_submap_sim3_constraint(
            source_id,
            target_id,
            &camera_matches,
            &overlap.target_from_source_rotation,
            &config.alignment,
        )
        .ok();
        let camera_refinement = config.camera_centre_refinement.as_ref().map(|refinement| {
            refine_submap_sim3_from_camera_centres(
                &landmark_constraint,
                &overlap.point_matches,
                &camera_matches,
                &config.alignment,
                refinement,
            )
        });
        let (constraint, refinement_rejection, refinement_scale_change, refinement_residual) =
            match camera_refinement {
                Some(Ok(refined)) => (
                    refined.constraint,
                    None,
                    Some(refined.abs_log_scale_change),
                    Some(refined.mean_camera_residual_ratio),
                ),
                Some(Err(rejection)) => (landmark_constraint.clone(), Some(rejection), None, None),
                None => (landmark_constraint.clone(), None, None, None),
            };
        seams.push(HierarchicalSfmSeam {
            source_submap_id: source_id,
            target_submap_id: target_id,
            shared_point_matches: overlap.point_matches.len(),
            sim3_inliers: constraint.inlier_match_indices.len(),
            sim3_inlier_ratio: constraint.inlier_ratio,
            mean_residual_ratio: constraint.mean_residual_ratio,
            essential_rotation_candidates: overlap.rotation_candidate_count,
            essential_rotation_consensus: overlap.rotation_consensus_count,
            essential_rotation_support: overlap.rotation_consensus_inlier_support,
            essential_rotation_max_disagreement_deg: overlap.max_rotation_disagreement_deg,
            shared_camera_centres: camera_matches.len(),
            camera_sim3_inliers: camera_constraint
                .as_ref()
                .map(|candidate| candidate.inlier_match_indices.len()),
            camera_sim3_inlier_ratio: camera_constraint
                .as_ref()
                .map(|candidate| candidate.inlier_ratio),
            camera_mean_residual_ratio: camera_constraint
                .as_ref()
                .map(|candidate| candidate.mean_residual_ratio),
            camera_landmark_log_scale_disagreement: camera_constraint.as_ref().map(|candidate| {
                (candidate.target_from_source.scale / landmark_constraint.target_from_source.scale)
                    .ln()
                    .abs()
            }),
            camera_landmark_rotation_disagreement_deg: camera_constraint.as_ref().map(
                |candidate| {
                    candidate
                        .target_from_source
                        .rotation
                        .rotation_to(&landmark_constraint.target_from_source.rotation)
                        .angle()
                        .to_degrees()
                },
            ),
            camera_refinement_applied: refinement_scale_change.is_some(),
            camera_refinement_rejection: refinement_rejection,
            camera_refinement_abs_log_scale_change: refinement_scale_change,
            camera_refinement_mean_residual_ratio: refinement_residual,
        });
        for &match_index in &constraint.inlier_match_indices {
            let point_match = &overlap.point_matches[match_index];
            seam_links.push(HierarchicalSeamLandmarkLink {
                source_submap_id: source_id,
                target_submap_id: target_id,
                source_landmark_id: point_match.source_landmark_id,
                target_landmark_id: point_match.target_landmark_id,
            });
        }
        eprintln!(
            "hierarchical-seam-edge: {source_id}..{target_id} accepted \
             inlier_count={} scale={:.9}",
            constraint.inlier_match_indices.len(),
            constraint.target_from_source.scale,
        );
        constraints.push(constraint);
    }

    let adjacent_edge_count = constraints.len();
    let mut banded_edges_accepted = 0usize;
    let mut banded_edges_rejected = 0usize;
    let max_band = config
        .submap_constraint_band
        .min(submaps.len().saturating_sub(1));
    for separation in 2..=max_band {
        for source_index in 0..submaps.len() - separation {
            let target_index = source_index + separation;
            let source_id = source_index as u64;
            let target_id = target_index as u64;
            let overlap = match collect_submap_overlap_evidence(
                &submaps[source_index],
                &submaps[target_index],
                pair_rotations,
                &config.overlap,
            ) {
                Ok(overlap) => overlap,
                Err(error) => {
                    banded_edges_rejected += 1;
                    eprintln!(
                        "hierarchical-banded-edge: {source_id}..{target_id} rejected reason={error}"
                    );
                    continue;
                }
            };
            let landmark_constraint = match estimate_submap_sim3_constraint(
                source_id,
                target_id,
                &overlap.point_matches,
                &overlap.target_from_source_rotation,
                &config.alignment,
            ) {
                Ok(constraint) => constraint,
                Err(rejection) => {
                    banded_edges_rejected += 1;
                    eprintln!(
                        "hierarchical-banded-edge: {source_id}..{target_id} rejected \
                         reason={:?} correspondences={} inliers={} inlier_ratio={:.6}",
                        rejection.reason,
                        rejection.correspondence_count,
                        rejection.inlier_count,
                        rejection.inlier_ratio,
                    );
                    continue;
                }
            };
            let camera_matches =
                shared_camera_center_matches(&submaps[source_index], &submaps[target_index]);
            let constraint = match config.camera_centre_refinement.as_ref() {
                Some(refinement) => refine_submap_sim3_from_camera_centres(
                    &landmark_constraint,
                    &overlap.point_matches,
                    &camera_matches,
                    &config.alignment,
                    refinement,
                )
                .map(|refined| refined.constraint)
                .unwrap_or(landmark_constraint),
                None => landmark_constraint,
            };
            eprintln!(
                "hierarchical-banded-edge: {source_id}..{target_id} accepted \
                 inlier_count={} scale={:.9}",
                constraint.inlier_match_indices.len(),
                constraint.target_from_source.scale,
            );
            constraints.push(constraint);
            banded_edges_accepted += 1;
        }
    }

    let mut submaps = submaps.into_iter();
    let root = submaps.next().expect("non-empty checked above");
    let mut hierarchy = HierarchicalSubmapGraph::new(0, root);
    for (offset, submap) in submaps.enumerate() {
        hierarchy.insert_independent(offset as u64 + 1, submap)?;
    }
    for constraint in constraints {
        hierarchy.add_constraint(VerifiedSubmapConstraint::Sim3(constraint))?;
    }
    let optimization = hierarchy.optimize(&config.pose_graph)?;
    eprintln!(
        "hierarchical-pose-graph-summary: adjacent_edges={adjacent_edge_count} \
         banded_edges_accepted={banded_edges_accepted} \
         banded_edges_rejected={banded_edges_rejected} initial_cost={:.9} final_cost={:.9}",
        optimization.pose_graph.initial_cost, optimization.pose_graph.final_cost,
    );
    let seam_bundle_adjustment = config
        .seam_bundle_adjustment
        .as_ref()
        .map(|ba_config| {
            crate::hierarchical_seam_ba::refine_hierarchical_seams(
                &mut hierarchy,
                &seam_links,
                ba_config,
            )
        })
        .transpose()?;
    Ok(HierarchicalSfmAtlas {
        hierarchy,
        seams,
        optimization: Some(optimization),
        seam_bundle_adjustment,
        loop_closure: None,
        loop_bundle_adjustment: None,
        seam_landmark_links: seam_links,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Point2, Point3, UnitQuaternion, Vector3};
    use visloc_core::geometry::{Pose, Sim3};

    use crate::{
        LocalSubmapFrame, LocalSubmapLandmark, LocalSubmapObservation, LocalSubmapQuality,
        TrackBuildStats,
    };

    fn local_submap(
        frame_id: u64,
        rotation: UnitQuaternion<f64>,
        landmarks: Vec<LocalSubmapLandmark>,
    ) -> LocalSubmap {
        LocalSubmap {
            camera: Camera::pinhole(0, 64, 48, 50.0, 50.0, 32.0, 24.0),
            source_frame_ids: vec![frame_id],
            frames: vec![LocalSubmapFrame {
                local_frame_index: 0,
                source_frame_id: frame_id,
                pose: Pose::from_world_to_camera(rotation, Vector3::zeros()),
            }],
            landmarks,
            quality: LocalSubmapQuality {
                requested_images: 1,
                registered_images: 1,
                registration_fraction: 1.0,
                landmarks: 15,
                observations: 15,
                median_track_length: 1.0,
                median_max_parallax_deg: 5.0,
                camera_center_diameter: 0.0,
                camera_center_step_median: 0.0,
                camera_center_step_max: 0.0,
                seed_pair_final_distance: 1.0,
                camera_center_window_drift_ratio: 0.0,
                mean_reprojection_px: 0.0,
                leave_one_out_attempts: 0,
                leave_one_out_supported: 0,
                leave_one_out_support_fraction: 0.0,
                median_leave_one_out_reprojection_px: 0.0,
            },
            track_build_stats: TrackBuildStats::default(),
            ba_result: None,
            seed_source_frame_i: frame_id,
            seed_source_frame_j: frame_id,
            seed_match_count: 0,
        }
    }

    fn point_landmark(id: u64, point: Point3<f64>, shared_frame: u64) -> LocalSubmapLandmark {
        LocalSubmapLandmark {
            local_landmark_id: id,
            position: point,
            observations: vec![LocalSubmapObservation {
                local_frame_index: 0,
                source_frame_id: shared_frame,
                keypoint_index: id as usize % 100,
                pixel: Point2::new(id as f64, 0.0),
            }],
        }
    }

    #[test]
    fn links_independent_submaps_through_r2_and_r3_transactionally() {
        let truth = Sim3::new(
            UnitQuaternion::from_euler_angles(0.08, -0.12, 0.21),
            Vector3::new(1.2, -0.4, 0.8),
            2.5,
        );
        let source_rotation = UnitQuaternion::from_euler_angles(-0.1, 0.05, 0.2);
        let target_rotation = UnitQuaternion::from_euler_angles(0.2, -0.08, -0.1);
        let source_points = (0..15)
            .map(|index| {
                let x = (index % 5) as f64 * 0.4;
                let y = (index / 5) as f64 * 0.35;
                let z = ((index * 7) % 4) as f64 * 0.2;
                point_landmark(index, Point3::new(x, y, z), 50)
            })
            .collect::<Vec<_>>();
        let target_points = source_points
            .iter()
            .map(|landmark| {
                let mut transformed = point_landmark(
                    landmark.local_landmark_id,
                    truth.transform_point(&landmark.position),
                    50,
                );
                transformed.local_landmark_id += 100;
                transformed.observations[0].keypoint_index = landmark.local_landmark_id as usize;
                transformed
            })
            .collect();
        let camera_j_from_i = target_rotation * truth.rotation * source_rotation.inverse();
        let atlas = optimize_independent_submaps(
            vec![
                local_submap(10, source_rotation, source_points),
                local_submap(90, target_rotation, target_points),
            ],
            &[PairRotationEvidence {
                image_i: 10,
                image_j: 90,
                image_j_from_i: camera_j_from_i,
                inlier_count: 100,
            }],
            &HierarchicalSfmConfig::default(),
        )
        .unwrap();
        assert_eq!(atlas.seams.len(), 1);
        assert_eq!(atlas.seams[0].sim3_inliers, 15);
        let recovered = &atlas.hierarchy.node(1).unwrap().local_from_atlas;
        let disagreement = recovered
            .as_ref()
            .unwrap()
            .compose(&truth.inverse())
            .log()
            .norm();
        assert!(disagreement < 1e-8, "Sim3 disagreement {disagreement}");
    }

    #[test]
    fn refuses_to_link_without_independent_rotation_evidence() {
        let empty = local_submap(0, UnitQuaternion::identity(), Vec::new());
        let error = optimize_independent_submaps(
            vec![empty.clone(), empty],
            &[],
            &HierarchicalSfmConfig::default(),
        )
        .unwrap_err();
        assert!(matches!(
            error,
            HierarchicalSfmError::Overlap {
                error: SubmapOverlapError::NoRotationCandidates,
                ..
            }
        ));
    }

    #[test]
    fn single_submap_needs_no_global_constraint() {
        let only = local_submap(0, UnitQuaternion::identity(), Vec::new());
        let atlas =
            optimize_independent_submaps(vec![only], &[], &HierarchicalSfmConfig::default())
                .unwrap();
        assert_eq!(atlas.hierarchy.nodes().count(), 1);
        assert!(atlas.optimization.is_none());
        assert!(atlas.seams.is_empty());
    }

    fn sample_quality() -> LocalSubmapQuality {
        LocalSubmapQuality {
            requested_images: 88,
            registered_images: 88,
            registration_fraction: 1.0,
            landmarks: 3306,
            observations: 10_000,
            median_track_length: 4.0,
            median_max_parallax_deg: 4.6577,
            camera_center_diameter: 159_616.92,
            camera_center_step_median: 2056.5,
            camera_center_step_max: 3969.4,
            seed_pair_final_distance: 3969.4,
            camera_center_window_drift_ratio: 1.9,
            mean_reprojection_px: 0.8146,
            leave_one_out_attempts: 500,
            leave_one_out_supported: 400,
            leave_one_out_support_fraction: 0.8,
            median_leave_one_out_reprojection_px: 0.9,
        }
    }

    fn local_build_error(error: LocalSubmapBuildError) -> HierarchicalSfmError {
        HierarchicalSfmError::LocalBuild {
            submap_id: 10,
            image_start: 160,
            image_end: 248,
            error,
        }
    }

    /// The widen/merge machinery (`is_build_error_widenable`) accepts
    /// `NoSeedPair` and every quality rejection, but leaves unrelated
    /// reconstruction errors fail-fast.
    #[test]
    fn all_quality_rejections_are_widenable_like_no_seed_pair() {
        assert!(is_build_error_widenable(&local_build_error(
            LocalSubmapBuildError::Reconstruction(IncrementalSfmError::NoSeedPair)
        )));
        assert!(is_build_error_widenable(&local_build_error(
            LocalSubmapBuildError::QualityRejected {
                reason: crate::LocalSubmapRejectionReason::ImplausibleScale,
                quality: sample_quality(),
            }
        )));

        assert!(is_build_error_widenable(&local_build_error(
            LocalSubmapBuildError::QualityRejected {
                reason: crate::LocalSubmapRejectionReason::LowParallax,
                quality: sample_quality(),
            }
        )));
        assert!(is_build_error_widenable(&local_build_error(
            LocalSubmapBuildError::QualityRejected {
                reason: crate::LocalSubmapRejectionReason::HighReprojectionError,
                quality: sample_quality(),
            }
        )));
        assert!(!is_build_error_widenable(&local_build_error(
            LocalSubmapBuildError::Reconstruction(IncrementalSfmError::SeedInitFailed)
        )));
    }

    #[test]
    fn too_few_registered_images_window_is_widened_instead_of_failing() {
        let windows = vec![
            SubmapWindow {
                image_range: 0..88,
                outgoing_seam_support: 0,
            },
            SubmapWindow {
                image_range: 88..176,
                outgoing_seam_support: 0,
            },
        ];
        let mut build_ranges = Vec::new();
        let outputs = widen_and_build(
            windows,
            1,
            1,
            0,
            |window| {
                build_ranges.push(window.image_range.clone());
                if window.image_range == (0..88) {
                    Err(local_build_error(LocalSubmapBuildError::QualityRejected {
                        reason: crate::LocalSubmapRejectionReason::TooFewRegisteredImages,
                        quality: LocalSubmapQuality {
                            registered_images: 2,
                            registration_fraction: 2.0 / 88.0,
                            ..sample_quality()
                        },
                    }))
                } else {
                    Ok(window.image_range.clone())
                }
            },
            is_build_error_widenable,
            is_no_seed_pair_build_error,
            |_, _, _, _, _| {},
        )
        .expect("quality-rejected window should consume its existing widen budget");

        assert_eq!(build_ranges, vec![0..88, 0..176]);
        assert_eq!(outputs.len(), 1);
        assert_eq!(outputs[0].0.image_range, 0..176);
    }

    /// The seam-time internal-drift remediation's reason filter
    /// (`SEAMDRIFT_CALIBRATION.md` / `LOWINLIERRATIO_DIAGNOSIS.md` option 2):
    /// only the two reasons that still require a genuine one-sided internal
    /// defect
    /// produce are even *eligible* for the diagnostic to override; every
    /// other rejection reason stays fail-fast unconditionally, regardless of
    /// what the diagnostic would say.
    #[test]
    fn seam_rejection_reason_drift_eligibility_is_exactly_low_inliers_and_no_fit() {
        assert!(is_seam_rejection_reason_drift_eligible(
            SubmapSim3RejectionReason::LowInlierRatio
        ));
        assert!(is_seam_rejection_reason_drift_eligible(
            SubmapSim3RejectionReason::NoRobustFit
        ));
        assert!(!is_seam_rejection_reason_drift_eligible(
            SubmapSim3RejectionReason::HighMeanResidual
        ));

        assert!(!is_seam_rejection_reason_drift_eligible(
            SubmapSim3RejectionReason::DegenerateSourceGeometry
        ));
        assert!(!is_seam_rejection_reason_drift_eligible(
            SubmapSim3RejectionReason::DegenerateTargetGeometry
        ));
        assert!(!is_seam_rejection_reason_drift_eligible(
            SubmapSim3RejectionReason::TooFewCorrespondences
        ));
        assert!(!is_seam_rejection_reason_drift_eligible(
            SubmapSim3RejectionReason::ScaleOutOfBounds
        ));
        assert!(!is_seam_rejection_reason_drift_eligible(
            SubmapSim3RejectionReason::RotationInconsistent
        ));
        assert!(!is_seam_rejection_reason_drift_eligible(
            SubmapSim3RejectionReason::UnstableLeaveOneOutScale
        ));
    }

    mod degenerate_seam_merge_tests {
        use super::*;
        use std::cell::RefCell;
        use std::ops::Range;

        fn w(range: Range<usize>) -> SubmapWindow {
            SubmapWindow {
                image_range: range,
                outgoing_seam_support: 0,
            }
        }

        fn rotation_link(image_i: u64, image_j: u64) -> PairRotationEvidence {
            PairRotationEvidence {
                image_i,
                image_j,
                image_j_from_i: UnitQuaternion::identity(),
                inlier_count: 50,
            }
        }

        /// A point cloud with (near-)zero spread along a second axis — the
        /// same shape `collinear_geometry_is_rejected_before_ransac` in
        /// `submap_alignment.rs` uses to trip the pre-RANSAC SVD gate.
        fn collinear_x(index: u64) -> Point3<f64> {
            Point3::new(index as f64, 0.0, 0.0)
        }

        /// A second, independent collinear axis, so a merged submap built
        /// from collinear source geometry can still fail a *different* seam
        /// for the same underlying reason (used by the cap test).
        fn collinear_y(index: u64) -> Point3<f64> {
            Point3::new(0.0, index as f64, 0.0)
        }

        /// Well-conditioned (non-collinear, non-coplanar) spread, identical
        /// to the fixture `links_independent_submaps_through_r2_and_r3_transactionally`
        /// uses above.
        fn spread_point(index: u64) -> Point3<f64> {
            let x = (index % 5) as f64 * 0.4;
            let y = (index / 5) as f64 * 0.35;
            let z = ((index * 7) % 4) as f64 * 0.2;
            Point3::new(x, y, z)
        }

        /// 15 mutually-corresponding landmarks (>= `min_correspondences`)
        /// observed at `shared_frame`, positioned by `position` in both
        /// submaps' local gauges (an identity truth transform between them,
        /// so any geometric pass/fail is driven purely by `position`'s
        /// spread, not by a mismatched rotation/scale).
        fn correspondence(
            shared_frame: u64,
            position: impl Fn(u64) -> Point3<f64> + Copy,
        ) -> (Vec<LocalSubmapLandmark>, Vec<LocalSubmapLandmark>) {
            correspondence_count(15, shared_frame, position, position)
        }

        fn correspondence_count(
            count: u64,
            shared_frame: u64,
            source_position: impl Fn(u64) -> Point3<f64>,
            target_position: impl Fn(u64) -> Point3<f64>,
        ) -> (Vec<LocalSubmapLandmark>, Vec<LocalSubmapLandmark>) {
            let source = (0..count)
                .map(|id| point_landmark(id, source_position(id), shared_frame))
                .collect();
            let target = (0..count)
                .map(|id| point_landmark(id + 100, target_position(id), shared_frame))
                .collect();
            (source, target)
        }

        fn with_camera_centers(mut submap: LocalSubmap, centers: &[(u64, f64)]) -> LocalSubmap {
            submap.source_frame_ids = centers.iter().map(|&(id, _)| id).collect();
            submap.frames = centers
                .iter()
                .enumerate()
                .map(|(index, &(source_frame_id, x))| LocalSubmapFrame {
                    local_frame_index: index,
                    source_frame_id,
                    pose: Pose::from_world_to_camera(
                        UnitQuaternion::identity(),
                        Vector3::new(-x, 0.0, 0.0),
                    ),
                })
                .collect();
            submap
        }

        fn catastrophic_correspondence(
            shared_frame: u64,
        ) -> (Vec<LocalSubmapLandmark>, Vec<LocalSubmapLandmark>) {
            correspondence_count(100, shared_frame, spread_point, |id| {
                let point = spread_point(id);
                if id < 12 {
                    point
                } else {
                    point
                        + Vector3::new(
                            0.35 * ((id * 17 % 23) as f64 - 11.0),
                            0.31 * ((id * 29 % 19) as f64 - 9.0),
                            0.27 * ((id * 37 % 17) as f64 - 8.0),
                        )
                }
            })
        }

        fn midband_correspondence(
            shared_frame: u64,
        ) -> (Vec<LocalSubmapLandmark>, Vec<LocalSubmapLandmark>) {
            correspondence_count(100, shared_frame, spread_point, |id| {
                let point = spread_point(id);
                if id < 44 {
                    point
                } else {
                    point
                        + Vector3::new(
                            0.41 * ((id * 17 % 23) as f64 - 11.0),
                            0.37 * ((id * 29 % 19) as f64 - 9.0),
                            0.33 * ((id * 37 % 17) as f64 - 8.0),
                        )
                }
            })
        }

        #[test]
        fn catastrophic_consensus_without_drift_triggers_merge() {
            let (source, target) = catastrophic_correspondence(100);
            let centers = [(100, 0.0), (101, 1.0), (102, 2.0), (103, 3.0)];
            let submaps = vec![
                with_camera_centers(
                    local_submap(100, UnitQuaternion::identity(), source),
                    &centers,
                ),
                with_camera_centers(
                    local_submap(101, UnitQuaternion::identity(), target),
                    &centers,
                ),
            ];
            let mut config = HierarchicalSfmConfig::default();
            config.alignment.ransac_iterations = 10_000;
            let diagnostic = seam_internal_drift_diagnostic(&submaps, 0, 1, &config)
                .expect("four shared camera centers produce a diagnostic");
            assert!(
                diagnostic.disagreement_ratio <= config.max_seam_internal_drift_disagreement_ratio
            );
            let candidates = seam_merge_candidates(&submaps, &[rotation_link(100, 101)], &config);
            assert_eq!(candidates.len(), 1);
            assert!(matches!(candidates[0].route, SeamMergeRoute::Consensus(_)));
            assert!(
                (0.08..=0.18).contains(&candidates[0].rejection.inlier_ratio),
                "fixture should plant a roughly 0.10 catastrophic ratio, got {}",
                candidates[0].rejection.inlier_ratio
            );

            let calls = RefCell::new(Vec::new());
            let mut next_id = 2;
            let (final_windows, atlas) = merge_degenerate_seams_and_optimize(
                vec![w(0..10), w(10..20)],
                submaps,
                &[rotation_link(100, 101)],
                &config,
                |_id, window| {
                    calls.borrow_mut().push(window.image_range.clone());
                    Ok(local_submap(200, UnitQuaternion::identity(), Vec::new()))
                },
                &mut next_id,
            )
            .expect("catastrophic agreeing-side consensus must rebuild");

            assert_eq!(calls.into_inner(), vec![0..20]);
            assert_eq!(final_windows.len(), 1);
            assert!(atlas.seams.is_empty());
        }

        #[test]
        fn adjacent_failing_seams_rebuild_one_three_submap_component() {
            let (source0, target0) = correspondence(100, collinear_x);
            let (source1, target1) = correspondence(200, collinear_y);
            let mut middle_landmarks = target0;
            middle_landmarks.extend(source1);
            let submaps = vec![
                local_submap(10, UnitQuaternion::identity(), source0),
                local_submap(20, UnitQuaternion::identity(), middle_landmarks),
                local_submap(30, UnitQuaternion::identity(), target1),
            ];
            let calls = RefCell::new(Vec::new());
            let mut next_id = 3;
            let (final_windows, atlas) = merge_degenerate_seams_and_optimize(
                vec![w(0..10), w(10..20), w(20..30)],
                submaps,
                &[rotation_link(10, 20), rotation_link(20, 30)],
                &HierarchicalSfmConfig::default(),
                |_id, window| {
                    calls.borrow_mut().push(window.image_range.clone());
                    Ok(local_submap(40, UnitQuaternion::identity(), Vec::new()))
                },
                &mut next_id,
            )
            .expect("the adjacent degenerate seams must be one component");

            assert_eq!(calls.into_inner(), vec![0..30]);
            assert_eq!(final_windows.len(), 1);
            assert!(atlas.seams.is_empty());
        }

        #[test]
        fn component_quality_failure_absorbs_one_neighbour_then_passes() {
            let (source, target) = correspondence(100, collinear_x);
            let submaps = vec![
                local_submap(10, UnitQuaternion::identity(), source),
                local_submap(20, UnitQuaternion::identity(), target),
                local_submap(30, UnitQuaternion::identity(), Vec::new()),
            ];
            let calls = RefCell::new(Vec::new());
            let mut next_id = 3;
            // One unit collapses the rejected seam and the second absorbs the
            // live successor after the component build's seed retries fail.
            let config = HierarchicalSfmConfig {
                max_degenerate_seam_merges: 2,
                ..Default::default()
            };

            let (final_windows, atlas) = merge_degenerate_seams_and_optimize(
                vec![w(0..10), w(10..20), w(20..30)],
                submaps,
                &[rotation_link(10, 20)],
                &config,
                |_id, window| {
                    calls.borrow_mut().push(window.image_range.clone());
                    if window.image_range == (0..20) {
                        Err(local_build_error(LocalSubmapBuildError::QualityRejected {
                            reason: crate::LocalSubmapRejectionReason::HighReprojectionError,
                            quality: sample_quality(),
                        }))
                    } else {
                        Ok(local_submap(40, UnitQuaternion::identity(), Vec::new()))
                    }
                },
                &mut next_id,
            )
            .expect("one shared-budget neighbour absorption should remediate the component");

            assert_eq!(calls.into_inner(), vec![0..20, 0..30]);
            assert_eq!(final_windows.len(), 1);
            assert!(atlas.seams.is_empty());
        }

        #[test]
        fn component_build_remediation_budget_exhaustion_is_fatal() {
            let (source, target) = correspondence(100, collinear_x);
            let submaps = vec![
                local_submap(10, UnitQuaternion::identity(), source),
                local_submap(20, UnitQuaternion::identity(), target),
                local_submap(30, UnitQuaternion::identity(), Vec::new()),
            ];
            let calls = RefCell::new(Vec::new());
            let mut next_id = 3;
            // The seam collapse spends the only unit, leaving none for the
            // neighbour absorption required by the failed rebuild.
            let config = HierarchicalSfmConfig {
                max_degenerate_seam_merges: 1,
                ..Default::default()
            };

            let error = merge_degenerate_seams_and_optimize(
                vec![w(0..10), w(10..20), w(20..30)],
                submaps,
                &[rotation_link(10, 20)],
                &config,
                |_id, window| {
                    calls.borrow_mut().push(window.image_range.clone());
                    Err(local_build_error(LocalSubmapBuildError::QualityRejected {
                        reason: crate::LocalSubmapRejectionReason::HighReprojectionError,
                        quality: sample_quality(),
                    }))
                },
                &mut next_id,
            )
            .expect_err("an exhausted shared merge budget must preserve the fatal build error");

            assert_eq!(calls.into_inner(), vec![0..20]);
            assert!(matches!(
                error,
                HierarchicalSfmError::LocalBuild {
                    error: LocalSubmapBuildError::QualityRejected {
                        reason: crate::LocalSubmapRejectionReason::HighReprojectionError,
                        ..
                    },
                    ..
                }
            ));
        }

        #[test]
        fn specific_drift_route_takes_precedence_over_last_resort() {
            let (source, target) = catastrophic_correspondence(100);
            let submaps = vec![
                with_camera_centers(
                    local_submap(100, UnitQuaternion::identity(), source),
                    &[(100, 0.0), (101, 1.0), (102, 2.0), (103, 3.0)],
                ),
                with_camera_centers(
                    local_submap(101, UnitQuaternion::identity(), target),
                    &[(100, 0.0), (101, 1.0), (102, 2.0), (103, 4.0)],
                ),
            ];
            let mut config = HierarchicalSfmConfig::default();
            config.alignment.ransac_iterations = 10_000;
            let diagnostic = seam_internal_drift_diagnostic(&submaps, 0, 1, &config)
                .expect("four shared camera centers produce a diagnostic");
            assert!(
                diagnostic.disagreement_ratio > config.max_seam_internal_drift_disagreement_ratio
            );
            let candidates = seam_merge_candidates(&submaps, &[rotation_link(100, 101)], &config);
            assert_eq!(candidates.len(), 1);
            assert!(matches!(candidates[0].route, SeamMergeRoute::Drift(_)));
            assert_eq!(
                candidates[0].route.log_family(),
                "hierarchical-seam-drift-merge"
            );

            let calls = RefCell::new(Vec::new());
            let mut next_id = 2;
            merge_degenerate_seams_and_optimize(
                vec![w(0..10), w(10..20)],
                submaps,
                &[rotation_link(100, 101)],
                &config,
                |_id, window| {
                    calls.borrow_mut().push(window.image_range.clone());
                    Ok(local_submap(200, UnitQuaternion::identity(), Vec::new()))
                },
                &mut next_id,
            )
            .expect("the pre-existing drift route must remain active");
            assert_eq!(calls.into_inner(), vec![0..20]);
        }

        #[test]
        fn later_actionable_seam_is_remediated_before_earlier_fatal_seam_returns() {
            let fatal_source = (0..5u64)
                .map(|id| point_landmark(id + 500, Point3::new(id as f64, 0.0, 0.0), 50))
                .collect::<Vec<_>>();
            let fatal_target = (0..5u64)
                .map(|id| point_landmark(id + 600, Point3::new(id as f64, 0.0, 0.0), 50))
                .collect::<Vec<_>>();
            let rebuilt_fatal_target = fatal_target.clone();
            let (degenerate_source, degenerate_target) = correspondence(100, collinear_x);
            let mut middle_landmarks = fatal_target;
            middle_landmarks.extend(degenerate_source);
            let submaps = vec![
                local_submap(10, UnitQuaternion::identity(), fatal_source),
                local_submap(20, UnitQuaternion::identity(), middle_landmarks),
                local_submap(30, UnitQuaternion::identity(), degenerate_target),
            ];
            let calls = RefCell::new(Vec::new());
            let mut next_id = 3;
            let config = HierarchicalSfmConfig {
                last_resort_seam_merge: false,
                ..Default::default()
            };
            let error = merge_degenerate_seams_and_optimize(
                vec![w(0..10), w(10..20), w(20..30)],
                submaps,
                &[rotation_link(10, 20), rotation_link(20, 30)],
                &config,
                |_id, window| {
                    calls.borrow_mut().push(window.image_range.clone());
                    Ok(local_submap(
                        20,
                        UnitQuaternion::identity(),
                        rebuilt_fatal_target.clone(),
                    ))
                },
                &mut next_id,
            )
            .expect_err("the non-routable first seam must still be returned after remediation");

            assert_eq!(calls.into_inner(), vec![10..30]);
            assert!(matches!(
                error,
                HierarchicalSfmError::Alignment {
                    rejection: SubmapSim3Rejection {
                        reason: SubmapSim3RejectionReason::TooFewCorrespondences,
                        ..
                    },
                    ..
                }
            ));
        }

        #[test]
        fn high_mean_residual_without_drift_flag_merges() {
            let (source, target) = correspondence_count(20, 100, spread_point, |id| {
                let point = spread_point(id);
                let sign = if id % 2 == 0 { 1.0 } else { -1.0 };
                point + Vector3::new(sign * 0.014, (id % 3) as f64 * 0.003 - 0.003, 0.0)
            });
            let windows = vec![w(0..10), w(10..20)];
            let submaps = vec![
                local_submap(10, UnitQuaternion::identity(), source),
                local_submap(20, UnitQuaternion::identity(), target),
            ];
            assert!(
                seam_internal_drift_diagnostic(&submaps, 0, 1, &HierarchicalSfmConfig::default())
                    .is_none(),
                "the routing must not depend on a drift flag"
            );

            let calls = RefCell::new(Vec::new());
            let mut next_id = 2;
            let (final_windows, atlas) = merge_degenerate_seams_and_optimize(
                windows,
                submaps,
                &[rotation_link(10, 20)],
                &HierarchicalSfmConfig::default(),
                |_id, window| {
                    calls.borrow_mut().push(window.image_range.clone());
                    Ok(local_submap(10, UnitQuaternion::identity(), Vec::new()))
                },
                &mut next_id,
            )
            .expect("HighMeanResidual must rebuild the joint window without a drift flag");

            assert_eq!(calls.into_inner(), vec![0..20]);
            assert_eq!(final_windows.len(), 1);
            assert!(atlas.seams.is_empty());
        }

        #[test]
        fn zero_correspondence_adjacent_seam_uses_last_resort_merge() {
            let submaps = vec![
                local_submap(10, UnitQuaternion::identity(), Vec::new()),
                local_submap(20, UnitQuaternion::identity(), Vec::new()),
            ];
            let pair_rotations = vec![rotation_link(10, 20)];
            let config = HierarchicalSfmConfig::default();

            let candidates = seam_merge_candidates(&submaps, &pair_rotations, &config);
            assert_eq!(candidates.len(), 1);
            assert_eq!(candidates[0].rejection.correspondence_count, 0);
            assert_eq!(
                candidates[0].rejection.reason,
                SubmapSim3RejectionReason::TooFewCorrespondences
            );
            assert!(matches!(
                candidates[0].route,
                SeamMergeRoute::LastResort(None)
            ));
            assert_eq!(
                candidates[0].route.log_family(),
                "hierarchical-seam-lastresort-merge"
            );

            let calls = RefCell::new(Vec::new());
            let mut next_id = 2;
            let (final_windows, atlas) = merge_degenerate_seams_and_optimize(
                vec![w(0..10), w(10..20)],
                submaps,
                &pair_rotations,
                &config,
                |_id, window| {
                    calls.borrow_mut().push(window.image_range.clone());
                    Ok(local_submap(30, UnitQuaternion::identity(), Vec::new()))
                },
                &mut next_id,
            )
            .expect("a zero-correspondence seam must receive a structural rebuild");

            assert_eq!(calls.into_inner(), vec![0..20]);
            assert_eq!(final_windows.len(), 1);
            assert!(atlas.seams.is_empty());
        }

        #[test]
        fn midband_low_inlier_ratio_without_drift_uses_last_resort_merge() {
            let (source, target) = midband_correspondence(100);
            let mut config = HierarchicalSfmConfig::default();
            config.alignment.ransac_iterations = 10_000;
            let submaps = vec![
                local_submap(10, UnitQuaternion::identity(), source),
                local_submap(20, UnitQuaternion::identity(), target),
            ];
            assert!(
                seam_internal_drift_diagnostic(&submaps, 0, 1, &config).is_none(),
                "the fixture intentionally has no drift evidence"
            );

            let candidates = seam_merge_candidates(&submaps, &[rotation_link(10, 20)], &config);
            assert_eq!(candidates.len(), 1);
            assert!(matches!(
                candidates[0].route,
                SeamMergeRoute::LastResort(None)
            ));
            assert_eq!(
                candidates[0].route.log_family(),
                "hierarchical-seam-lastresort-merge"
            );
            assert!(
                (0.42..=0.46).contains(&candidates[0].rejection.inlier_ratio),
                "fixture should produce the uncovered ~0.44 band, got {}",
                candidates[0].rejection.inlier_ratio
            );

            let calls = RefCell::new(Vec::new());
            let mut next_id = 2;
            let (final_windows, atlas) = merge_degenerate_seams_and_optimize(
                vec![w(0..10), w(10..20)],
                submaps,
                &[rotation_link(10, 20)],
                &config,
                |_id, window| {
                    calls.borrow_mut().push(window.image_range.clone());
                    Ok(local_submap(30, UnitQuaternion::identity(), Vec::new()))
                },
                &mut next_id,
            )
            .expect("the uncovered mid-band seam must receive a structural rebuild");
            assert_eq!(calls.into_inner(), vec![0..20]);
            assert_eq!(final_windows.len(), 1);
            assert!(atlas.seams.is_empty());
        }

        #[test]
        fn disabling_last_resort_preserves_fail_fast_for_too_few_correspondences() {
            // Only 5 shared correspondences (< min_correspondences = 12):
            // `TooFewCorrespondences`, a real geometric problem, must stay
            // fail-fast rather than being treated as a widenable partition
            // artifact.
            let source = (0..5u64)
                .map(|id| point_landmark(id, Point3::new(id as f64, 0.0, 0.0), 100))
                .collect::<Vec<_>>();
            let target = (0..5u64)
                .map(|id| point_landmark(id + 100, Point3::new(id as f64, 0.0, 0.0), 100))
                .collect::<Vec<_>>();
            let windows = vec![w(0..10), w(10..20)];
            let submaps = vec![
                local_submap(10, UnitQuaternion::identity(), source),
                local_submap(20, UnitQuaternion::identity(), target),
            ];
            let pair_rotations = vec![rotation_link(10, 20)];
            let mut next_id = 2u64;
            let config = HierarchicalSfmConfig {
                last_resort_seam_merge: false,
                ..Default::default()
            };
            let error = merge_degenerate_seams_and_optimize(
                windows,
                submaps,
                &pair_rotations,
                &config,
                |id: u64, window: &SubmapWindow| -> Result<LocalSubmap, HierarchicalSfmError> {
                    panic!("build_one must not be called for a non-degenerate rejection (id {id}, images {:?})", window.image_range);
                },
                &mut next_id,
            )
            .unwrap_err();
            assert!(matches!(
                error,
                HierarchicalSfmError::Alignment {
                    rejection: SubmapSim3Rejection {
                        reason: SubmapSim3RejectionReason::TooFewCorrespondences,
                        ..
                    },
                    ..
                }
            ));
        }

        #[test]
        fn single_merge_succeeds_and_reuses_the_untouched_neighbour() {
            let (source0, target0) = correspondence(100, collinear_x);
            let (merged_source, s2_landmarks) = correspondence(200, spread_point);
            let windows = vec![w(0..10), w(10..20), w(20..30)];
            let submaps = vec![
                local_submap(10, UnitQuaternion::identity(), source0),
                local_submap(20, UnitQuaternion::identity(), target0),
                local_submap(30, UnitQuaternion::identity(), s2_landmarks),
            ];
            let pair_rotations = vec![rotation_link(10, 20), rotation_link(20, 30)];
            let calls = RefCell::new(Vec::new());
            let mut next_id = 3u64;
            let (final_windows, atlas) = merge_degenerate_seams_and_optimize(
                windows,
                submaps,
                &pair_rotations,
                &HierarchicalSfmConfig::default(),
                |_id: u64, window: &SubmapWindow| -> Result<LocalSubmap, HierarchicalSfmError> {
                    calls.borrow_mut().push(window.image_range.clone());
                    Ok(local_submap(
                        20,
                        UnitQuaternion::identity(),
                        merged_source.clone(),
                    ))
                },
                &mut next_id,
            )
            .expect("merged window aligns cleanly with the untouched neighbour");
            assert_eq!(
                final_windows
                    .iter()
                    .map(|window| window.image_range.clone())
                    .collect::<Vec<_>>(),
                vec![0..20, 20..30]
            );
            // Exactly one rebuild, for the merged window only — submap 2
            // (images 20..30) is carried over untouched, never rebuilt.
            assert_eq!(calls.borrow().as_slice(), [Range { start: 0, end: 20 }]);
            assert_eq!(atlas.seams.len(), 1);
            assert_eq!(atlas.seams[0].sim3_inliers, 15);
        }

        #[test]
        fn cascading_degenerate_seams_merge_until_the_chain_aligns() {
            let (source0, target0) = correspondence(100, collinear_x);
            let (m1_source, s2_landmarks) = correspondence(200, collinear_y);
            let (m2_source, s3_landmarks) = correspondence(300, spread_point);
            let windows = vec![w(0..10), w(10..20), w(20..30), w(30..40)];
            let submaps = vec![
                local_submap(10, UnitQuaternion::identity(), source0),
                local_submap(20, UnitQuaternion::identity(), target0),
                local_submap(30, UnitQuaternion::identity(), s2_landmarks),
                local_submap(40, UnitQuaternion::identity(), s3_landmarks),
            ];
            let pair_rotations = vec![
                rotation_link(10, 20),
                rotation_link(20, 30),
                rotation_link(30, 40),
            ];
            let calls = RefCell::new(Vec::new());
            let mut next_id = 4u64;
            let (final_windows, atlas) = merge_degenerate_seams_and_optimize(
                windows,
                submaps,
                &pair_rotations,
                &HierarchicalSfmConfig::default(),
                |id: u64, window: &SubmapWindow| -> Result<LocalSubmap, HierarchicalSfmError> {
                    calls.borrow_mut().push(window.image_range.clone());
                    match window.image_range.clone() {
                        r if r.start == 0 && r.end == 20 => Ok(local_submap(
                            20,
                            UnitQuaternion::identity(),
                            m1_source.clone(),
                        )),
                        r if r.start == 0 && r.end == 30 => Ok(local_submap(
                            30,
                            UnitQuaternion::identity(),
                            m2_source.clone(),
                        )),
                        other => panic!("unexpected rebuild for images {other:?} (id {id})"),
                    }
                },
                &mut next_id,
            )
            .expect("two cascaded merges reach a well-conditioned seam");
            assert_eq!(
                final_windows
                    .iter()
                    .map(|window| window.image_range.clone())
                    .collect::<Vec<_>>(),
                vec![0..30, 30..40]
            );
            // First merge collapses 0..20 (still degenerate against submap
            // 2), second merge collapses 0..30 (finally well-conditioned
            // against submap 3). Submap 3 (30..40) is never rebuilt.
            assert_eq!(calls.borrow().as_slice(), [0..20, 0..30]);
            assert_eq!(atlas.seams.len(), 1);
            assert_eq!(atlas.seams[0].sim3_inliers, 15);
        }

        #[test]
        fn cap_is_respected_and_original_error_returned_when_exhausted() {
            let (source0, target0) = correspondence(100, collinear_x);
            let (_, s2_landmarks) = correspondence(200, collinear_y);
            let windows = vec![w(0..10), w(10..20), w(20..30)];
            let submaps = vec![
                local_submap(10, UnitQuaternion::identity(), source0),
                local_submap(20, UnitQuaternion::identity(), target0),
                local_submap(30, UnitQuaternion::identity(), s2_landmarks),
            ];
            let pair_rotations = vec![rotation_link(10, 20), rotation_link(20, 30)];
            let calls = RefCell::new(Vec::new());
            let mut next_id = 3u64;
            let config = HierarchicalSfmConfig {
                max_degenerate_seam_merges: 1, // one merge short of the second needed
                ..HierarchicalSfmConfig::default()
            };
            let error = merge_degenerate_seams_and_optimize(
                windows,
                submaps,
                &pair_rotations,
                &config,
                |_id: u64, window: &SubmapWindow| -> Result<LocalSubmap, HierarchicalSfmError> {
                    calls.borrow_mut().push(window.image_range.clone());
                    // The rebuilt window is itself collinear, so the seam
                    // against submap 2 stays degenerate: exercises the cap,
                    // not a lucky escape.
                    let (m1_source, _) = correspondence(200, collinear_y);
                    Ok(local_submap(20, UnitQuaternion::identity(), m1_source))
                },
                &mut next_id,
            )
            .unwrap_err();
            assert!(matches!(
                error,
                HierarchicalSfmError::Alignment {
                    rejection: SubmapSim3Rejection {
                        reason: SubmapSim3RejectionReason::DegenerateSourceGeometry,
                        ..
                    },
                    ..
                }
            ));
            assert_eq!(calls.borrow().len(), 1, "cap stops the second merge");
            assert_eq!(calls.borrow()[0], 0..20);
        }

        #[test]
        fn last_resort_budget_exhaustion_still_returns_fatal_seam() {
            let (first_source, first_target) = midband_correspondence(100);
            let (_, final_target) = midband_correspondence(200);
            let windows = vec![w(0..10), w(10..20), w(20..30)];
            let submaps = vec![
                local_submap(10, UnitQuaternion::identity(), first_source),
                local_submap(20, UnitQuaternion::identity(), first_target),
                local_submap(30, UnitQuaternion::identity(), final_target),
            ];
            let pair_rotations = vec![rotation_link(10, 20), rotation_link(20, 30)];
            let calls = RefCell::new(Vec::new());
            let mut next_id = 3u64;
            let mut config = HierarchicalSfmConfig::default();
            config.alignment.ransac_iterations = 10_000;
            config.max_degenerate_seam_merges = 1;

            let error = merge_degenerate_seams_and_optimize(
                windows,
                submaps,
                &pair_rotations,
                &config,
                |_id, window| {
                    calls.borrow_mut().push(window.image_range.clone());
                    Ok(local_submap(20, UnitQuaternion::identity(), Vec::new()))
                },
                &mut next_id,
            )
            .expect_err("the two-seam component must not exceed the one-merge budget");

            assert!(calls.into_inner().is_empty());
            assert!(matches!(
                error,
                HierarchicalSfmError::Alignment {
                    rejection: SubmapSim3Rejection {
                        reason: SubmapSim3RejectionReason::LowInlierRatio,
                        ..
                    },
                    ..
                }
            ));
        }

        #[test]
        fn report_all_failing_seams_counts_every_failing_seam_not_just_the_first() {
            // Three submaps chained with two independently too-few-correspondence
            // seams (0->1 and 1->2). With last-resort routing disabled below,
            // `merge_degenerate_seams_and_optimize` only ever *sees* the first
            // (0->1, via `optimize_independent_submaps`'s own fail-fast loop) —
            // this test's point is that
            // `report_all_failing_seams` independently walks every seam and
            // must find *both*, not stop after the one the caller's error
            // names.
            let shared_100 = (0..5u64)
                .map(|id| point_landmark(id, Point3::new(id as f64, 0.0, 0.0), 100))
                .collect::<Vec<_>>();
            let shared_100_target = (0..5u64)
                .map(|id| point_landmark(id + 100, Point3::new(id as f64, 0.0, 0.0), 100))
                .collect::<Vec<_>>();
            let shared_200_source = (0..5u64)
                .map(|id| point_landmark(id + 200, Point3::new(id as f64, 0.0, 0.0), 200))
                .collect::<Vec<_>>();
            let shared_200_target = (0..5u64)
                .map(|id| point_landmark(id + 300, Point3::new(id as f64, 0.0, 0.0), 200))
                .collect::<Vec<_>>();
            let submaps = vec![
                local_submap(10, UnitQuaternion::identity(), shared_100),
                local_submap(20, UnitQuaternion::identity(), {
                    let mut landmarks = shared_100_target;
                    landmarks.extend(shared_200_source);
                    landmarks
                }),
                local_submap(30, UnitQuaternion::identity(), shared_200_target),
            ];
            let pair_rotations = vec![rotation_link(10, 20), rotation_link(20, 30)];
            let windows = vec![w(0..10), w(10..20), w(20..30)];
            let failing = report_all_failing_seams(
                &windows,
                &submaps,
                &pair_rotations,
                &HierarchicalSfmConfig::default(),
            );
            assert_eq!(
                failing, 2,
                "both the 0->1 and 1->2 seams have only 5 (< min_correspondences \
                 12) shared landmarks and must both be reported"
            );

            // With the independently-tested last-resort route disabled, the
            // actual run is unaffected by the exhaustive reporting scan and
            // still fails fast on the first seam (0->1), unchanged.
            let mut next_id = 3u64;
            let config = HierarchicalSfmConfig {
                last_resort_seam_merge: false,
                ..Default::default()
            };
            let error = merge_degenerate_seams_and_optimize(
                windows,
                submaps,
                &pair_rotations,
                &config,
                |id: u64, window: &SubmapWindow| -> Result<LocalSubmap, HierarchicalSfmError> {
                    panic!("build_one must not be called for a non-degenerate rejection (id {id}, images {:?})", window.image_range);
                },
                &mut next_id,
            )
            .unwrap_err();
            assert!(matches!(
                error,
                HierarchicalSfmError::Alignment {
                    source_submap_id: 0,
                    target_submap_id: 1,
                    rejection: SubmapSim3Rejection {
                        reason: SubmapSim3RejectionReason::TooFewCorrespondences,
                        ..
                    },
                }
            ));
        }

        #[test]
        fn merge_and_align_sequence_is_deterministic_across_runs() {
            fn run() -> (
                Vec<Range<usize>>,
                Vec<HierarchicalSfmSeam>,
                Vec<Range<usize>>,
            ) {
                let (source0, target0) = correspondence(100, collinear_x);
                let (m1_source, s2_landmarks) = correspondence(200, collinear_y);
                let (m2_source, s3_landmarks) = correspondence(300, spread_point);
                let windows = vec![w(0..10), w(10..20), w(20..30), w(30..40)];
                let submaps = vec![
                    local_submap(10, UnitQuaternion::identity(), source0),
                    local_submap(20, UnitQuaternion::identity(), target0),
                    local_submap(30, UnitQuaternion::identity(), s2_landmarks),
                    local_submap(40, UnitQuaternion::identity(), s3_landmarks),
                ];
                let pair_rotations = vec![
                    rotation_link(10, 20),
                    rotation_link(20, 30),
                    rotation_link(30, 40),
                ];
                let calls = RefCell::new(Vec::new());
                let mut next_id = 4u64;
                let (final_windows, atlas) = merge_degenerate_seams_and_optimize(
                    windows,
                    submaps,
                    &pair_rotations,
                    &HierarchicalSfmConfig::default(),
                    |id: u64, window: &SubmapWindow| -> Result<LocalSubmap, HierarchicalSfmError> {
                        calls.borrow_mut().push(window.image_range.clone());
                        match window.image_range.clone() {
                            r if r.start == 0 && r.end == 20 => Ok(local_submap(
                                20,
                                UnitQuaternion::identity(),
                                m1_source.clone(),
                            )),
                            r if r.start == 0 && r.end == 30 => Ok(local_submap(
                                30,
                                UnitQuaternion::identity(),
                                m2_source.clone(),
                            )),
                            other => panic!("unexpected rebuild for images {other:?} (id {id})"),
                        }
                    },
                    &mut next_id,
                )
                .unwrap();
                (
                    final_windows
                        .into_iter()
                        .map(|window| window.image_range)
                        .collect(),
                    atlas.seams,
                    calls.into_inner(),
                )
            }
            assert_eq!(run(), run());
        }
    }
}
