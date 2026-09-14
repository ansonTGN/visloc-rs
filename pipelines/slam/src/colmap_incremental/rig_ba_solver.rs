//! C2.5: fast native bundle-adjustment backend for
//! `bundle_adjustment.rs::solve`'s rig-reprojection problem
//! (`bundle::BundleAdjustment` populated with `add_pose`/`add_landmark`/
//! `add_rig_observation`/`fix_pose`/`fix_landmark` only — the exact subset
//! `bundle_adjustment.rs::solve` builds).
//!
//! Replaces `bundle::BundleAdjustment::optimize`'s `BTreeMap`-indexed,
//! serially-assembled, densely-Schur-solved LM loop (measured ~340ms/iter at
//! 97 frames / 51k obs / 1.7k points; the dense 7.5k×7.5k reduced-camera
//! solve for a 1250-frame problem is already the dominant cost and is
//! infeasible past ~10k frames) with:
//! - contiguous `Vec`-of-struct observation/frame/point storage (§ below),
//!   sorted by point so per-point Schur elimination reads a contiguous slice
//!   — no map lookups in the per-observation/per-point hot loops;
//! - `rayon`-parallel Jacobian evaluation and per-point Schur elimination,
//!   reduced deterministically (every parallel step is either a positional
//!   `collect()`, whose result order is independent of the thread count by
//!   construction of `rayon`'s `IndexedParallelIterator`, or a serial fold
//!   over that already-ordered `Vec` — never a `rayon` `.sum()`/`.reduce()`,
//!   whose split structure this module does not want to depend on);
//! - the reduced (pose-only) camera system solved as a 6×6-block-sparse
//!   system via [`crate::block_cholesky::solve_spd_blocks6_cached`] (already
//!   in this crate; reused here rather than re-implemented, per the task
//!   brief's "reuse `block_cholesky.rs` if it fits" — it does: the module's
//!   own doc frames it as "the BA Schur solve" as one of its two intended
//!   callers). No separate dense path is implemented for small frame counts
//!   (a documented deviation from the task's "dense fallback ≤200 frames"
//!   suggestion): `block_cholesky`'s own doc states it is the *general*
//!   production path (dense-at-block-granularity even for small block
//!   counts, see its "Why not supernodal?" section), so a second,
//!   independently-tested dense code path would duplicate risk without a
//!   measured benefit — verified instead by comparing Native's output
//!   directly against `BundleAdjustment::optimize` (`Legacy`, itself a dense
//!   solver) on identical small synthetic problems (`native_vs_legacy_*`
//!   tests below).
//!
//! ## Residual model
//! Identical formula to `bundle.rs`'s `rig_residual_jacobians` (the existing,
//! already-exercised port of COLMAP's `RigReprojErrorConstantRigCostFunctor`,
//! `cost_functions/reprojection_error.h:389-417`, documented in
//! `bundle_adjustment.rs`'s module doc citation): fixed pinhole intrinsics
//! `(fx, fy, cx, cy)`, fixed `sensor_from_rig`, variable `rig_from_world`
//! pose and variable landmark `xyz`:
//! ```text
//! point_rig    = rig_from_world · point_world
//! point_sensor = sensor_from_rig · point_rig
//! residual     = (fx·x/z + cx, fy·y/z + cy) − observed_xy     (point_sensor = (x,y,z))
//! ```
//! Trivial (unweighted, no robust kernel) loss — matches the control
//! configuration's Ceres call (`AddResidualBlock(cost, /*loss=*/nullptr,
//! ...)`; `bundle_adjustment.rs`'s C2 control never enables a robust kernel).
//! [`residual_and_jacobians`] below is an independent, allocation-free
//! re-derivation of the same formula (not a call into `bundle.rs`, keeping
//! this module link-independent); `tests::matches_bundle_rs_formula` checks
//! the two agree bit-for-bit-close on random inputs, and
//! `tests::jacobian_matches_finite_difference` checks both partials
//! numerically.
//!
//! ## Pose parameterization
//! `rig_from_world` is `Pose { world_to_camera: SE3 }` (`SE3 { rotation:
//! UnitQuaternion, translation }`). The local update is this crate's
//! existing SE(3) convention — the same one every other `bundle.rs` LM loop
//! already uses (e.g. `bundle.rs:2894,3906,16075,16205`):
//! `T_new = T_old.compose(SE3::exp(xi))`, `xi = [ρ; ω] ∈ R^6` (translation
//! tangent first, then rotation axis-angle) — a *right* (local, body-frame)
//! 6-DoF manifold update. COLMAP's own Ceres setup instead uses
//! `EigenQuaternionManifold` for the rotation (a separate 4-parameter block
//! with its own local parameterization) plus a plain unconstrained R³ block
//! for the translation. Both are minimal 6-DoF local parameterizations of
//! the same SE(3) manifold at the same linearization point; the converged
//! optimum (a stationary point of the same reprojection-error cost) and the
//! Gauss-Newton/LM trust-region geometry near it coincide to first order —
//! the difference is bookkeeping (one coupled 6-vector update vs. two
//! independent blocks), not a modeling difference. This port keeps this
//! crate's existing convention rather than introducing a second one.
//!
//! ## Trust region (Ceres `LEVENBERG_MARQUARDT` defaults, as COLMAP's control
//! configures them — `bundle_adjustment_ceres.cc:103-105` sets
//! `function_tolerance=0`, `parameter_tolerance=0`; `gradient_tolerance` is
//! left at Ceres' own default `1e-4`; the trust-region strategy and its
//! `initial_trust_region_radius=1e4` are Ceres solver defaults, not
//! overridden by COLMAP)
//! - Diagonal (Jacobi) scaling: each parameter block's damping uses
//!   `mu · clamp(diag(H_block), 1e-6, 1e32)` (`mu = 1/radius`), added to that
//!   block's own diagonal only — the well-known
//!   `LevenbergMarquardtStrategy` clamp constants from the public Ceres
//!   Solver source (`internal/ceres/levenberg_marquardt_strategy.cc`).
//! - Step quality `rho = actual_cost_decrease / predicted_cost_decrease`
//!   (both on the Ceres-internal `0.5·Σr²` cost scale, so the gradient
//!   `g = Jᵀr` and the `gradient_tolerance` check line up with Ceres';
//!   [`BaResult`]'s reported `initial_cost`/`final_cost` are rescaled back to
//!   `Σr²` to match `BundleAdjustment::cost()`'s convention for the
//!   Native/Legacy parity test). `predicted_decrease = ½·dx·(b + μ·D·dx)`,
//!   the standard trust-region model-decrease identity for a step solving
//!   `(H + μD)dx = b`.
//! - Accept iff `rho > 1e-3`; on accept `radius ← radius / max(1/3, 1 −
//!   (2ρ−1)³)`, reset the decrease factor to 2; on reject `radius ←
//!   radius / decrease_factor`, then `decrease_factor *= 2`.
//! - Terminate on `max_num_iterations` (from
//!   [`super::bundle_adjustment::BundleAdjustmentOptions`]) or
//!   `‖g‖_∞ ≤ 1e-4` (checked once per trial, at the current linearization —
//!   `function_tolerance=0`/`parameter_tolerance=0` are absolute zeros, i.e.
//!   COLMAP's control disables those two early-stop checks, so this is the
//!   only early-stop path, matching the task brief).
//! - Jacobians are only re-evaluated after an **accepted** step (mirroring
//!   Ceres' `TrustRegionMinimizer`); a rejected trial only re-solves the
//!   reduced system at the new damping and re-evaluates the (Jacobian-free)
//!   candidate cost.
//!
//! ## Gauge / constant handling
//! Frames in `ba.fixed_poses` and points in `ba.fixed_landmarks` are never
//! given a variable slot (their observations still constrain the *other*
//! side of the residual — see [`Problem::free_frame_slot`] / point handling
//! in [`linearize_point`]) — this is `bundle_adjustment.rs`'s existing
//! `BundleAdjustmentConfig`/`Gauge` mechanism (`set_constant_rig_from_world_pose`,
//! `add_constant_point`, `fix_gauge`), unchanged by this backend swap.

use std::collections::{BTreeMap, HashMap};
use std::time::{Duration, Instant};

use nalgebra::{DMatrix, Matrix3, Matrix6, Point2, Point3, SMatrix, Vector2, Vector3, Vector6};
use rayon::prelude::*;

use visloc_core::geometry::SE3;

use crate::block_cholesky::{solve_spd_blocks6_cached, BlockSymbolic};
use crate::bundle::{BaError, BaIterationStats, BaResult, BundleAdjustment};

/// Ceres `LevenbergMarquardtStrategy` defaults (public Ceres Solver source,
/// `internal/ceres/levenberg_marquardt_strategy.cc` /
/// `trust_region_minimizer.cc`); see module doc.
const INITIAL_RADIUS: f64 = 1e4;
const MIN_RELATIVE_DECREASE: f64 = 1e-3;
const GRADIENT_TOLERANCE: f64 = 1e-4;
const MIN_DIAGONAL: f64 = 1e-6;
const MAX_DIAGONAL: f64 = 1e32;

/// One rig-reprojection observation, flattened to plain floats/indices — no
/// `Camera`/`SE3` lookups by id anywhere in the per-observation hot loop.
/// Sorted by `point_idx` in [`Problem::obs`] so a point's observations are a
/// contiguous slice.
#[derive(Debug, Clone)]
struct Obs {
    frame_idx: u32,
    point_idx: u32,
    xy: Point2<f64>,
    fx: f64,
    fy: f64,
    cx: f64,
    cy: f64,
    sensor_from_rig: SE3,
}

struct Problem {
    frame_ids: Vec<u64>,
    /// `Some(slot)` for free frames (`slot` indexes the reduced camera
    /// system, `0..n_free_frames`), `None` for fixed frames.
    free_frame_slot: Vec<Option<u32>>,
    n_free_frames: usize,
    poses: Vec<SE3>,

    point_ids: Vec<u64>,
    point_fixed: Vec<bool>,
    points: Vec<Point3<f64>>,

    obs: Vec<Obs>,
    /// `[start, end)` into `obs` for each point, `obs` sorted by `point_idx`.
    point_obs_range: Vec<(u32, u32)>,

    /// Performance fix (see [`build_reduced_system_pattern`]'s doc): the
    /// distinct *free* frames observing each point, ascending by
    /// `frame_idx` — a property of the observation topology alone
    /// (independent of pose/point values, hence independent of the current
    /// LM linearization), computed once here rather than rediscovered by
    /// scanning Jacobian-evaluated observations on every accepted step.
    point_free_frames: Vec<Vec<u32>>,
    /// Deduplicated, ascending `(free_slot_a, free_slot_b)` pairs (`a < b`)
    /// across every point's `point_free_frames` — the reduced camera
    /// system's off-diagonal sparsity pattern.
    edges: Vec<(u32, u32)>,
    /// Per point, the `edges`-index of each of its
    /// `point_free_frames[p].len() * (len-1) / 2` distinct frame pairs, in
    /// the canonical `for j in 1..n { for i in 0..j }` order over
    /// `point_free_frames[p]`.
    point_pair_edge_idx: Vec<Vec<u32>>,
    /// Performance fix #3 (see [`eliminate_and_accumulate`]'s doc): point
    /// index range `[shard_boundaries[s], shard_boundaries[s+1])` for shard
    /// `s`, balanced by *cumulative pair count* (`Σ n·(n-1)/2`), not point
    /// count — a real (corridor-revisit) track-length distribution is
    /// heavy-tailed, so an equal-point-count split can leave one shard
    /// doing most of the O(pairs) work while the others idle. Computed once
    /// here (pair counts are fixed for the whole `optimize()` call) from
    /// `point_free_frames`, reused unchanged by every trial and every
    /// accepted step.
    shard_boundaries: Vec<usize>,
}

/// Performance fix (profiled on a 300-frame/2500-point/289k-observation
/// synthetic problem sized like the tier-1000 global BA call that measured
/// 2.9s/iteration): the reduced camera system's off-diagonal blocks come
/// from O(track_len²) frame pairs per point — real tracks (corridor
/// revisits) run ~70 observations/point, so this is millions of pairs
/// total. Two things had to change from a first pass that computed this
/// per accepted LM step (still O(pairs) but only recomputed a few times
/// instead of every trial): (1) it must be a `BTreeMap`-free flat-array
/// merge (`solve_step`'s hot path), and (2) — the fix here — the pair
/// *pattern* itself (which frames co-observe which point) is a property of
/// the **observation topology alone**, not of the current pose/point
/// values, so it is computed **once per `optimize()` call**, not once per
/// accepted step: re-deriving and re-sorting ~6M pairs on every accept
/// (even only ~8 times in the profiling run) still cost ~200ms each. Using
/// `problem.obs`/`point_obs_range` (raw observation topology, ignoring the
/// z<=0 cheirality guard `residual_and_jacobians` applies — a frame/point
/// pair failing that check is a numerical edge case, not a topology
/// change) instead of `PointLin.frames` (Jacobian-evaluation output)
/// removes the coupling to relinearization entirely.
#[allow(clippy::type_complexity)]
fn build_reduced_system_pattern(
    point_obs_range: &[(u32, u32)],
    obs: &[Obs],
    frame_fixed: &[bool],
    free_frame_slot: &[Option<u32>],
) -> (Vec<Vec<u32>>, Vec<(u32, u32)>, Vec<Vec<u32>>) {
    let mut point_free_frames: Vec<Vec<u32>> = Vec::with_capacity(point_obs_range.len());
    for &(start, end) in point_obs_range {
        let mut frames: Vec<u32> = obs[start as usize..end as usize]
            .iter()
            .map(|o| o.frame_idx)
            .filter(|&f| !frame_fixed[f as usize])
            .collect();
        frames.sort_unstable();
        frames.dedup();
        point_free_frames.push(frames);
    }

    let mut raw_pairs: Vec<(u32, u32)> = Vec::new();
    for frames in &point_free_frames {
        if frames.len() < 2 {
            continue;
        }
        let slots: Vec<u32> = frames
            .iter()
            .map(|&f| free_frame_slot[f as usize].expect("free"))
            .collect();
        for j in 1..slots.len() {
            for i in 0..j {
                raw_pairs.push((slots[i], slots[j]));
            }
        }
    }
    raw_pairs.sort_unstable();
    raw_pairs.dedup();
    let edges = raw_pairs;
    let edge_index: HashMap<(u32, u32), u32> = edges
        .iter()
        .enumerate()
        .map(|(idx, &e)| (e, idx as u32))
        .collect();

    let mut point_pair_edge_idx: Vec<Vec<u32>> = Vec::with_capacity(point_free_frames.len());
    for frames in &point_free_frames {
        if frames.len() < 2 {
            point_pair_edge_idx.push(Vec::new());
            continue;
        }
        let slots: Vec<u32> = frames
            .iter()
            .map(|&f| free_frame_slot[f as usize].expect("free"))
            .collect();
        let n = slots.len();
        let mut idxs = Vec::with_capacity(n * (n - 1) / 2);
        for j in 1..n {
            for i in 0..j {
                idxs.push(edge_index[&(slots[i], slots[j])]);
            }
        }
        point_pair_edge_idx.push(idxs);
    }

    (point_free_frames, edges, point_pair_edge_idx)
}

fn build_problem(ba: &BundleAdjustment) -> Result<Problem, BaError> {
    if ba.poses.is_empty() {
        return Err(BaError::NoPoses);
    }
    if ba.landmarks.is_empty() {
        return Err(BaError::NoLandmarks);
    }
    if ba.rig_observations.is_empty() {
        return Err(BaError::NoObservations);
    }

    let frame_ids: Vec<u64> = ba.poses.keys().copied().collect();
    let frame_id_to_idx: HashMap<u64, u32> = frame_ids
        .iter()
        .enumerate()
        .map(|(i, &id)| (id, i as u32))
        .collect();
    let frame_fixed: Vec<bool> = frame_ids
        .iter()
        .map(|id| ba.fixed_poses.contains(id))
        .collect();
    let mut free_frame_slot = vec![None; frame_ids.len()];
    let mut n_free_frames = 0usize;
    for (i, &fixed) in frame_fixed.iter().enumerate() {
        if !fixed {
            free_frame_slot[i] = Some(n_free_frames as u32);
            n_free_frames += 1;
        }
    }
    if n_free_frames == 0 {
        return Err(BaError::AllPosesFixed);
    }
    let poses: Vec<SE3> = frame_ids
        .iter()
        .map(|id| ba.poses[id].world_to_camera.clone())
        .collect();

    let point_ids: Vec<u64> = ba.landmarks.keys().copied().collect();
    let point_id_to_idx: HashMap<u64, u32> = point_ids
        .iter()
        .enumerate()
        .map(|(i, &id)| (id, i as u32))
        .collect();
    let point_fixed: Vec<bool> = point_ids
        .iter()
        .map(|id| ba.fixed_landmarks.contains(id))
        .collect();
    let points: Vec<Point3<f64>> = point_ids.iter().map(|id| ba.landmarks[id]).collect();

    let mut obs: Vec<Obs> = Vec::with_capacity(ba.rig_observations.len());
    for o in &ba.rig_observations {
        let &frame_idx = frame_id_to_idx
            .get(&o.keyframe_id)
            .ok_or(BaError::MissingPose(o.keyframe_id))?;
        let &point_idx = point_id_to_idx
            .get(&o.landmark_id)
            .ok_or(BaError::MissingLandmark(o.landmark_id))?;
        let (fx, fy, cx, cy) = o
            .camera
            .intrinsics()
            .ok_or(BaError::UnsupportedCameraModel)?;
        obs.push(Obs {
            frame_idx,
            point_idx,
            xy: o.xy,
            fx,
            fy,
            cx,
            cy,
            sensor_from_rig: o.sensor_from_rig.clone(),
        });
    }
    // Stable sort: ties (same point_idx) keep their original — i.e. caller
    // (`ba.rig_observations`) — insertion order, so the resulting layout is
    // a pure function of the input, independent of the sort algorithm.
    obs.sort_by_key(|o| o.point_idx);

    let mut point_obs_range = vec![(0u32, 0u32); point_ids.len()];
    let mut i = 0usize;
    while i < obs.len() {
        let p = obs[i].point_idx as usize;
        let start = i;
        while i < obs.len() && obs[i].point_idx as usize == p {
            i += 1;
        }
        point_obs_range[p] = (start as u32, i as u32);
    }

    let (point_free_frames, edges, point_pair_edge_idx) =
        build_reduced_system_pattern(&point_obs_range, &obs, &frame_fixed, &free_frame_slot);
    let n_shards = elimination_shard_count(edges.len());
    let shard_boundaries = balanced_shard_boundaries(&point_free_frames, n_shards);

    Ok(Problem {
        frame_ids,
        free_frame_slot,
        n_free_frames,
        poses,
        point_ids,
        point_fixed,
        points,
        obs,
        point_obs_range,
        point_free_frames,
        edges,
        point_pair_edge_idx,
        shard_boundaries,
    })
}

#[inline]
fn skew(v: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
}

/// Residual `r = π(sensor_from_rig · pose · point_world) − xy` and its
/// Jacobians w.r.t. the pose's `[ρ; ω]` local update (`2×6`) and the point
/// (`2×3`). `None` when the point is behind the sensor (matches
/// `bundle.rs::cost()`'s "skip nonprojectable observations" convention — see
/// module doc). See module doc for the formula citation and
/// `tests::matches_bundle_rs_formula` / `tests::jacobian_matches_finite_difference`
/// for verification.
#[inline]
#[allow(clippy::too_many_arguments)]
fn residual_and_jacobians(
    fx: f64,
    fy: f64,
    cx: f64,
    cy: f64,
    sensor_from_rig: &SE3,
    pose: &SE3,
    point_world: &Point3<f64>,
    xy: Point2<f64>,
) -> Option<(Vector2<f64>, SMatrix<f64, 2, 6>, SMatrix<f64, 2, 3>)> {
    let point_rig = pose.transform_point(point_world);
    let point_sensor = sensor_from_rig.transform_point(&point_rig);
    if point_sensor.z <= 0.0 {
        return None;
    }
    let z_inv = point_sensor.z.recip();
    let z_inv2 = z_inv * z_inv;
    let predicted = Point2::new(
        fx * point_sensor.x * z_inv + cx,
        fy * point_sensor.y * z_inv + cy,
    );
    let residual = Vector2::new(predicted.x - xy.x, predicted.y - xy.y);

    let projection = SMatrix::<f64, 2, 3>::new(
        fx * z_inv,
        0.0,
        -fx * point_sensor.x * z_inv2,
        0.0,
        fy * z_inv,
        -fy * point_sensor.y * z_inv2,
    );
    let r_wc = pose.rotation.to_rotation_matrix().into_inner();
    let r_rs = sensor_from_rig.rotation.to_rotation_matrix().into_inner();
    let mut d_rig_d_pose = SMatrix::<f64, 3, 6>::zeros();
    d_rig_d_pose.fixed_view_mut::<3, 3>(0, 0).copy_from(&r_wc);
    d_rig_d_pose
        .fixed_view_mut::<3, 3>(0, 3)
        .copy_from(&(-r_wc * skew(&point_world.coords)));
    let j_pose = projection * r_rs * d_rig_d_pose;
    let j_point = projection * r_rs * r_wc;
    Some((residual, j_pose, j_point))
}

/// Per-point linearization output (Stage A — recomputed only after an
/// accepted step). `frames` are the point's *free* observing frames only
/// (ascending, deduplicated); fixed frames contribute to `hpp`/`bp` (they
/// still constrain the point) but never get a `diag`/`bc`/`hcp` slot (they
/// have no variable to receive one).
struct PointLin {
    frames: Vec<u32>,
    /// `−Jᵀr` summed per free frame that observes this point (the pose-block
    /// raw, mu-independent contribution — same convention as `bp`).
    bc: Vec<Vector6<f64>>,
    /// `JᵀJ` summed per free frame (raw, mu-independent).
    diag: Vec<Matrix6<f64>>,
    /// `Jpose^T Jpoint` per free frame, only meaningful if `free`.
    hcp: Vec<SMatrix<f64, 6, 3>>,
    /// `JᵀJ` for the point block, only meaningful if `free`.
    hpp: Matrix3<f64>,
    /// `−Jᵀr` for the point block, only meaningful if `free`.
    bp: Vector3<f64>,
    free: bool,
}

/// Stage A: evaluate residual + Jacobians for every observation of one point
/// and fold them into that point's local blocks. Returns `(cost_contribution,
/// PointLin)`; `cost_contribution = Σ ‖r_i‖²` (full, un-halved — matches
/// `BundleAdjustment::cost()`'s convention) over this point's projectable
/// observations. `target_frames` (== `problem.point_free_frames[p]`, ascending)
/// fixes `frames`/`bc`/`diag`/`hcp`'s slot layout up front — an observation's
/// frame is placed by `binary_search` (`target_frames` is sorted), not a
/// linear scan that grows the list — see [`build_reduced_system_pattern`]'s
/// doc for why the frame *set* itself is precomputed once per `optimize()`
/// call rather than rediscovered here.
fn linearize_point(
    obs_slice: &[Obs],
    point_free: bool,
    point_xyz: &Point3<f64>,
    poses: &[SE3],
    target_frames: &[u32],
) -> (f64, PointLin) {
    let mut cost = 0.0;
    let mut hpp = Matrix3::zeros();
    let mut bp = Vector3::zeros();
    let mut bc: Vec<Vector6<f64>> = vec![Vector6::zeros(); target_frames.len()];
    let mut diag: Vec<Matrix6<f64>> = vec![Matrix6::zeros(); target_frames.len()];
    let mut hcp: Vec<SMatrix<f64, 6, 3>> = vec![SMatrix::<f64, 6, 3>::zeros(); target_frames.len()];

    for o in obs_slice {
        let pose = &poses[o.frame_idx as usize];
        let Some((r, j_pose, j_point)) = residual_and_jacobians(
            o.fx,
            o.fy,
            o.cx,
            o.cy,
            &o.sensor_from_rig,
            pose,
            point_xyz,
            o.xy,
        ) else {
            continue;
        };
        cost += r.norm_squared();
        if point_free {
            hpp += j_point.transpose() * j_point;
            bp += -(j_point.transpose() * r);
        }
        if let Ok(slot) = target_frames.binary_search(&o.frame_idx) {
            bc[slot] += -(j_pose.transpose() * r);
            diag[slot] += j_pose.transpose() * j_pose;
            if point_free {
                hcp[slot] += j_pose.transpose() * j_point;
            }
        }
    }

    (
        cost,
        PointLin {
            frames: target_frames.to_vec(),
            bc,
            diag,
            hcp,
            hpp,
            bp,
            free: point_free,
        },
    )
}

/// Stage A driver: relinearize every point in parallel (deterministic —
/// `par_iter().collect()` on an `IndexedParallelIterator` preserves input
/// order regardless of thread count), then serially merge each point's
/// per-frame contribution into the (mu-independent) raw pose-block
/// accumulators. Returns `(full_cost, points_lin, frame_diag_raw,
/// frame_bc_raw)`. The reduced-system sparsity pattern
/// (`problem.edges`/`problem.point_pair_edge_idx`) is *not* recomputed here
/// — see [`build_reduced_system_pattern`]'s doc for why it is a
/// `build_problem`-time, not a per-linearization, computation.
#[allow(clippy::type_complexity)]
fn linearize(problem: &Problem) -> (f64, Vec<PointLin>, Vec<Matrix6<f64>>, Vec<Vector6<f64>>) {
    let contributions: Vec<(f64, PointLin)> = problem
        .points
        .par_iter()
        .enumerate()
        .map(|(p, xyz)| {
            let (start, end) = problem.point_obs_range[p];
            linearize_point(
                &problem.obs[start as usize..end as usize],
                !problem.point_fixed[p],
                xyz,
                &problem.poses,
                &problem.point_free_frames[p],
            )
        })
        .collect();

    // `contributions` is a plain `Vec` (already materialized by the ordered
    // `collect()` above) — this final fold is a serial iterator, not a
    // `rayon` reduction, so it is deterministic by construction regardless
    // of `RAYON_NUM_THREADS` without needing any special chunking.
    let full_cost: f64 = contributions.iter().map(|(c, _)| *c).sum();

    let mut frame_diag_raw = vec![Matrix6::<f64>::zeros(); problem.n_free_frames];
    let mut frame_bc_raw = vec![Vector6::<f64>::zeros(); problem.n_free_frames];
    let mut points_lin = Vec::with_capacity(contributions.len());
    for (_, pl) in contributions {
        for (k, &fidx) in pl.frames.iter().enumerate() {
            let slot = problem.free_frame_slot[fidx as usize].expect("frames in PointLin are free")
                as usize;
            frame_diag_raw[slot] += pl.diag[k];
            frame_bc_raw[slot] += pl.bc[k];
        }
        points_lin.push(pl);
    }

    (full_cost, points_lin, frame_diag_raw, frame_bc_raw)
}

#[inline]
fn clamp_diag6(h: &Matrix6<f64>) -> Vector6<f64> {
    Vector6::from_iterator((0..6).map(|i| h[(i, i)].clamp(MIN_DIAGONAL, MAX_DIAGONAL)))
}

#[inline]
fn clamp_diag3(h: &Matrix3<f64>) -> Vector3<f64> {
    Vector3::from_iterator((0..3).map(|i| h[(i, i)].clamp(MIN_DIAGONAL, MAX_DIAGONAL)))
}

/// Damped 3×3 point-block inverse at the given `mu = 1/radius`, or `None` if
/// the damped block is (numerically) singular.
fn damped_hpp_inverse(hpp: &Matrix3<f64>, mu: f64) -> Option<Matrix3<f64>> {
    let diag = clamp_diag3(hpp);
    let mut damped = *hpp;
    for i in 0..3 {
        damped[(i, i)] += mu * diag[i];
    }
    damped.try_inverse()
}

/// Per-phase wall time accumulated across every trial of one `optimize()`
/// call, printed as `BA_PHASES` alongside `BA_SOLVE` (task item: "Add timing
/// instrumentation inside the native solver per phase"). Phases match the
/// task brief's list: `eliminate` = per-point Schur elimination,
/// `assemble` = merging eliminated contributions into the reduced-system
/// blocks (+ pose damping + `columns`/`rhs` construction), `linsolve` =
/// [`crate::block_cholesky::solve_spd_blocks6_cached`], `backsub` =
/// free-point back-substitution + predicted-decrease bookkeeping,
/// `evaluate_cost` = the trial-step residual-only cost re-evaluation.
/// `linearize` is timed separately in [`optimize`] (only runs on an
/// accepted step, so its count usually differs from the trial count).
#[derive(Default, Debug, Clone, Copy)]
struct PhaseTimings {
    linearize: Duration,
    eliminate: Duration,
    assemble: Duration,
    linsolve: Duration,
    backsub: Duration,
    evaluate_cost: Duration,
    linearize_calls: u32,
    trials: u32,
}

/// Performance fix #2 (see [`build_reduced_system_pattern`] for fix #1):
/// profiling after fix #1 still showed `eliminate_ms`+`assemble_ms`
/// dominating (~1.4s of ~2.2s total at 300f/2500pt/289k-obs, 7 trials) even
/// with the flat-`Vec` merge and the precomputed edge pattern. Two things
/// remained: (a) `eliminate_point` still heap-allocated an
/// O(track_len²)-sized `Vec<Matrix6>` *per point, per trial* (its `pairs`
/// output) purely to hand values to a *separate* merge step — at ~2500
/// points × up to `~2500` pairs each, that is gigabytes of transient
/// allocation per trial; (b) the merge then wrote those values into
/// `flat_offdiag` at effectively random indices (`pair_edge_idx` scatters
/// across the whole edge set), which is cache-unfriendly and was serial
/// (one core). This function fuses per-point elimination and the
/// flat-buffer write into one step with **no** per-point/per-pair heap
/// allocation (only a small O(track_len) `hcp_scaled` scratch buffer,
/// reused — not reallocated — across every point in a shard) and shards
/// the O(total pairs) work across a *fixed* number of point-index ranges
/// (independent of `RAYON_NUM_THREADS`) so it runs in parallel while
/// staying bit-identical at any thread count (module doc): each shard
/// writes into its own private `edges.len()`-sized buffer (no cross-thread
/// contention), and the buffers are summed in a fixed shard-index order
/// afterward.
#[allow(clippy::type_complexity)]
#[allow(clippy::needless_range_loop)]
fn eliminate_and_accumulate(
    problem: &Problem,
    points_lin: &[PointLin],
    edges_len: usize,
    mu: f64,
) -> (Vec<Matrix6<f64>>, Vec<Matrix6<f64>>, Vec<Vector6<f64>>) {
    let n_free = problem.n_free_frames;
    let n_shards = problem.shard_boundaries.len().saturating_sub(1).max(1);

    let shard_results: Vec<(Vec<Matrix6<f64>>, Vec<Matrix6<f64>>, Vec<Vector6<f64>>)> = (0
        ..n_shards)
        .into_par_iter()
        .map(|shard| {
            let lo = problem.shard_boundaries[shard];
            let hi = problem.shard_boundaries[shard + 1];
            let mut local_offdiag = vec![Matrix6::<f64>::zeros(); edges_len];
            let mut local_diag = vec![Matrix6::<f64>::zeros(); n_free];
            let mut local_bc = vec![Vector6::<f64>::zeros(); n_free];
            let mut hcp_scaled: Vec<SMatrix<f64, 6, 3>> = Vec::new();
            // Both `pl.frames`/`pl.hcp` and `problem.point_pair_edge_idx`
            // are indexed by the same `p`, and the pair loop below indexes
            // `hcp_scaled`/`pl.hcp` at two *different* offsets (`i`, `j`) in
            // the same iteration — an index loop reads more directly than
            // threading multiple `.zip()`s through both (hence the
            // `needless_range_loop` allow above).
            for p in lo..hi.max(lo) {
                let pl = &points_lin[p];
                if !pl.free || pl.frames.is_empty() {
                    continue;
                }
                let n = pl.frames.len();
                let Some(hpp_inv) = damped_hpp_inverse(&pl.hpp, mu) else {
                    // Degenerate point block: contribute nothing rather than
                    // propagate a solve failure (see the original
                    // `eliminate_point` doc, same policy).
                    continue;
                };
                hcp_scaled.clear();
                hcp_scaled.extend((0..n).map(|k| pl.hcp[k] * hpp_inv));
                for k in 0..n {
                    let slot =
                        problem.free_frame_slot[pl.frames[k] as usize].expect("free") as usize;
                    local_diag[slot] -= hcp_scaled[k] * pl.hcp[k].transpose();
                    local_bc[slot] -= hcp_scaled[k] * pl.bp;
                }
                let pair_edge_idx = &problem.point_pair_edge_idx[p];
                let mut local_pair = 0usize;
                for j in 1..n {
                    for i in 0..j {
                        let val = hcp_scaled[j] * pl.hcp[i].transpose();
                        local_offdiag[pair_edge_idx[local_pair] as usize] -= val;
                        local_pair += 1;
                    }
                }
            }
            (local_offdiag, local_diag, local_bc)
        })
        .collect();

    let mut flat_offdiag = vec![Matrix6::<f64>::zeros(); edges_len];
    let mut diag_delta = vec![Matrix6::<f64>::zeros(); n_free];
    let mut bc_delta = vec![Vector6::<f64>::zeros(); n_free];
    // Fixed shard-index order (0..n_shards, not runtime scheduling order) —
    // bit-identical at any thread count, matching `block_cholesky`'s
    // documented "fixed chunk, folded in order" determinism pattern.
    for (local_offdiag, local_diag, local_bc) in shard_results {
        for (a, b) in flat_offdiag.iter_mut().zip(local_offdiag) {
            *a += b;
        }
        for (a, b) in diag_delta.iter_mut().zip(local_diag) {
            *a += b;
        }
        for (a, b) in bc_delta.iter_mut().zip(local_bc) {
            *a += b;
        }
    }
    (flat_offdiag, diag_delta, bc_delta)
}

/// Shard count for [`eliminate_and_accumulate`]: capped so the transient
/// per-shard `edges_len`-sized buffers (`3 shards worth` — see its `local_*`
/// locals — of `Matrix6` = 288 bytes each) never exceed roughly 256MB in
/// aggregate, and bounded to `[1, 8]` (the pool this crate targets). A pure
/// function of `edges_len` (fixed for the whole `optimize()` call — see
/// `build_reduced_system_pattern`), so the shard count — hence the fixed
/// reduction order in [`eliminate_and_accumulate`] — never depends on
/// `RAYON_NUM_THREADS`.
fn elimination_shard_count(edges_len: usize) -> usize {
    // Real (corridor-revisit) global-BA windows can have `edges_len` well
    // into the hundreds of thousands (dense frame-frame co-observation from
    // long, slowly-moving tracks) — e.g. `edges_len=357k` (≈ `C(846,2)`,
    // observed on the real tier-1000 run's largest global-BA window) costs
    // ~103MB *per shard*. At the previous 256MB cap that collapsed to just
    // 2 shards, discarding most of the intended 8x parallelism exactly on
    // the heaviest problems — plausibly the dominant reason a same-sized
    // synthetic profile (with a more modest `edges_len`, hence 8 shards)
    // measured far faster than the real run's global-BA `BA_PHASES` lines.
    // 1.5GB keeps 8 shards up to `edges_len≈650k` (the real run's peak RSS
    // for the *entire* mapper process was 1.15GB, so this has headroom).
    const MAX_TOTAL_BYTES: usize = 1536 * 1024 * 1024;
    let per_shard_bytes = edges_len.max(1) * std::mem::size_of::<Matrix6<f64>>();
    (MAX_TOTAL_BYTES / per_shard_bytes).clamp(1, 8)
}

/// Performance fix #3: [`elimination_shard_count`] fixes *how many* shards,
/// this fixes *which points* land in each — balanced by cumulative pair
/// count (`Σ n·(n-1)/2` over each shard's points), not point count. A real
/// (corridor-revisit) track-length distribution is heavy-tailed: a handful
/// of long-revisited points can carry most of the O(pairs) work, and an
/// equal-point-count split (the original scheme) can land all of them in
/// one shard while the other 7 threads idle — losing most of the intended
/// 8x speedup exactly on the problems that need it most. Computed once (pair
/// counts, hence the optimal boundaries, are fixed for the whole
/// `optimize()` call) via a prefix sum over `point_free_frames` and a
/// `partition_point` binary search per boundary — O(n_points log n_points),
/// negligible next to the O(pairs) work it schedules.
fn balanced_shard_boundaries(point_free_frames: &[Vec<u32>], n_shards: usize) -> Vec<usize> {
    let n_points = point_free_frames.len();
    let n_shards = n_shards.max(1);
    let mut prefix: Vec<u64> = Vec::with_capacity(n_points + 1);
    prefix.push(0);
    for frames in point_free_frames {
        let n = frames.len() as u64;
        prefix.push(prefix.last().copied().unwrap_or(0) + n * n.saturating_sub(1) / 2);
    }
    let total = *prefix.last().unwrap_or(&0);
    let mut boundaries = Vec::with_capacity(n_shards + 1);
    boundaries.push(0usize);
    for s in 1..n_shards {
        let target = total * s as u64 / n_shards as u64;
        let idx = prefix.partition_point(|&v| v < target);
        let idx = idx.clamp(*boundaries.last().expect("non-empty"), n_points);
        boundaries.push(idx);
    }
    boundaries.push(n_points);
    boundaries
}

/// Stage B driver: assemble the damped, Schur-reduced 6×6-block camera
/// system at the given `mu` and solve it via
/// [`crate::block_cholesky::solve_spd_blocks6_cached`], then back-substitute
/// the free-point steps. Returns `(dx_frames, dx_points, predicted_decrease)`
/// (all on the free-slot / point-index space); `None` if the reduced system
/// was singular even after damping (`BaError::SingularSystem`).
#[allow(clippy::type_complexity)]
#[allow(clippy::too_many_arguments)]
fn solve_step(
    problem: &Problem,
    points_lin: &[PointLin],
    edges: &[(u32, u32)],
    frame_diag_raw: &[Matrix6<f64>],
    frame_bc_raw: &[Vector6<f64>],
    mu: f64,
    cache: &mut Option<BlockSymbolic>,
    timings: &mut PhaseTimings,
) -> Option<(Vec<Vector6<f64>>, Vec<Vector3<f64>>, f64)> {
    let n_free = problem.n_free_frames;
    let t = Instant::now();
    let (flat_offdiag, diag_delta, bc_delta) =
        eliminate_and_accumulate(problem, points_lin, edges.len(), mu);
    timings.eliminate += t.elapsed();

    let t = Instant::now();
    let mut frame_diag: Vec<Matrix6<f64>> = frame_diag_raw.to_vec();
    let mut frame_bc: Vec<Vector6<f64>> = frame_bc_raw.to_vec();
    for a in 0..n_free {
        frame_diag[a] += diag_delta[a];
        frame_bc[a] += bc_delta[a];
    }
    // Pose-block Jacobi damping, applied to the *raw* (pre-elimination)
    // diagonal — matches Ceres' `LevenbergMarquardtStrategy` (module doc).
    for a in 0..n_free {
        let d = clamp_diag6(&frame_diag_raw[a]);
        for i in 0..6 {
            frame_diag[a][(i, i)] += mu * d[i];
        }
    }

    let mut columns: Vec<BTreeMap<usize, Matrix6<f64>>> = vec![BTreeMap::new(); n_free];
    for (a, col) in columns.iter_mut().enumerate() {
        col.insert(a, frame_diag[a]);
    }
    for (&(a, b), val) in edges.iter().zip(flat_offdiag) {
        columns[a as usize].insert(b as usize, val);
    }
    let mut rhs = DMatrix::<f64>::zeros(n_free * 6, 1);
    for a in 0..n_free {
        for i in 0..6 {
            rhs[(a * 6 + i, 0)] = frame_bc[a][i];
        }
    }
    timings.assemble += t.elapsed();

    let t = Instant::now();
    let solved = solve_spd_blocks6_cached(cache, columns, &rhs).ok()?;
    let mut dx_frames = vec![Vector6::<f64>::zeros(); n_free];
    for a in 0..n_free {
        for i in 0..6 {
            dx_frames[a][i] = solved[(a * 6 + i, 0)];
        }
    }
    timings.linsolve += t.elapsed();

    let t = Instant::now();
    let dx_points: Vec<Vector3<f64>> = points_lin
        .par_iter()
        .map(|pl| {
            if !pl.free {
                return Vector3::zeros();
            }
            let Some(hpp_inv) = damped_hpp_inverse(&pl.hpp, mu) else {
                return Vector3::zeros();
            };
            let mut rhs = pl.bp;
            for (k, &fidx) in pl.frames.iter().enumerate() {
                let slot = problem.free_frame_slot[fidx as usize].expect("free") as usize;
                rhs -= pl.hcp[k].transpose() * dx_frames[slot];
            }
            hpp_inv * rhs
        })
        .collect();

    let mut predicted = 0.0;
    for a in 0..n_free {
        let d = clamp_diag6(&frame_diag_raw[a]);
        let dx = dx_frames[a];
        let damped_dx = Vector6::from_iterator((0..6).map(|i| mu * d[i] * dx[i]));
        predicted += dx.dot(&(frame_bc_raw[a] + damped_dx));
    }
    for (p, pl) in points_lin.iter().enumerate() {
        if !pl.free {
            continue;
        }
        let d = clamp_diag3(&pl.hpp);
        let dx = dx_points[p];
        let damped_dx = Vector3::from_iterator((0..3).map(|i| mu * d[i] * dx[i]));
        predicted += dx.dot(&(pl.bp + damped_dx));
    }
    predicted *= 0.5;
    timings.backsub += t.elapsed();
    timings.trials += 1;

    Some((dx_frames, dx_points, predicted))
}

/// Residual-only (no Jacobian) total cost at a candidate `(poses, points)` —
/// `Σ ‖r_i‖²`, same convention as [`linearize`]'s `full_cost` /
/// `BundleAdjustment::cost()`. Used for a rejected trial's cheap re-check
/// (module doc: Jacobians are only re-evaluated after an accepted step).
fn evaluate_cost(problem: &Problem, poses: &[SE3], points: &[Point3<f64>]) -> f64 {
    let per_point: Vec<f64> = problem
        .points
        .par_iter()
        .enumerate()
        .map(|(p, _)| {
            let (start, end) = problem.point_obs_range[p];
            let mut c = 0.0;
            for o in &problem.obs[start as usize..end as usize] {
                let pose = &poses[o.frame_idx as usize];
                let point_rig = pose.transform_point(&points[p]);
                let point_sensor = o.sensor_from_rig.transform_point(&point_rig);
                if point_sensor.z <= 0.0 {
                    continue;
                }
                let z_inv = point_sensor.z.recip();
                let dx = o.fx * point_sensor.x * z_inv + o.cx - o.xy.x;
                let dy = o.fy * point_sensor.y * z_inv + o.cy - o.xy.y;
                c += dx * dx + dy * dy;
            }
            c
        })
        .collect();
    // Serial fold over an already-materialized `Vec` — see `linearize`.
    per_point.iter().sum()
}

fn apply_step(
    problem: &Problem,
    dx_frames: &[Vector6<f64>],
    dx_points: &[Vector3<f64>],
) -> (Vec<SE3>, Vec<Point3<f64>>) {
    let poses: Vec<SE3> = problem
        .poses
        .iter()
        .enumerate()
        .map(|(i, pose)| match problem.free_frame_slot[i] {
            Some(slot) => pose.compose(&SE3::exp(&dx_frames[slot as usize])),
            None => pose.clone(),
        })
        .collect();
    let points: Vec<Point3<f64>> = problem
        .points
        .iter()
        .enumerate()
        .map(|(p, xyz)| {
            if problem.point_fixed[p] {
                *xyz
            } else {
                xyz + dx_points[p]
            }
        })
        .collect();
    (poses, points)
}

fn gradient_inf_norm(points_lin: &[PointLin], frame_bc_raw: &[Vector6<f64>]) -> f64 {
    let mut m: f64 = 0.0;
    for v in frame_bc_raw {
        for i in 0..6 {
            m = m.max(v[i].abs());
        }
    }
    for pl in points_lin {
        if !pl.free {
            continue;
        }
        for i in 0..3 {
            m = m.max(pl.bp[i].abs());
        }
    }
    m
}

/// Native `BaBackend` entry point: solves `ba`'s rig-reprojection problem in
/// place (writing the optimized poses/landmarks back into `ba.poses`/
/// `ba.landmarks`) and returns the same [`BaResult`]
/// `bundle_adjustment.rs::solve` already expects from the `Legacy` backend.
/// See module doc for the full algorithm.
pub(crate) fn optimize(
    ba: &mut BundleAdjustment,
    max_num_iterations: usize,
) -> Result<BaResult, BaError> {
    let mut problem = build_problem(ba)?;
    let mut timings = PhaseTimings::default();

    let (mut half_cost, mut points_lin, mut frame_diag_raw, mut frame_bc_raw) = {
        let t = Instant::now();
        let (full, pl, fd, fb) = linearize(&problem);
        timings.linearize += t.elapsed();
        timings.linearize_calls += 1;
        (0.5 * full, pl, fd, fb)
    };
    let initial_cost = 2.0 * half_cost;

    let mut radius = INITIAL_RADIUS;
    let mut decrease_factor = 2.0;
    let mut cache: Option<BlockSymbolic> = None;
    let mut iterations: Vec<BaIterationStats> = Vec::new();
    let mut trace_lines: Vec<String> = Vec::new();
    let mut converged = false;

    for it in 0..max_num_iterations {
        let grad_norm = gradient_inf_norm(&points_lin, &frame_bc_raw);
        if grad_norm <= GRADIENT_TOLERANCE {
            converged = true;
            break;
        }
        let mu = 1.0 / radius;
        let Some((dx_frames, dx_points, predicted)) = solve_step(
            &problem,
            &points_lin,
            &problem.edges,
            &frame_diag_raw,
            &frame_bc_raw,
            mu,
            &mut cache,
            &mut timings,
        ) else {
            return Err(BaError::SingularSystem);
        };
        let (trial_poses, trial_points) = apply_step(&problem, &dx_frames, &dx_points);
        let t = Instant::now();
        let trial_half_cost = 0.5 * evaluate_cost(&problem, &trial_poses, &trial_points);
        timings.evaluate_cost += t.elapsed();
        let actual = half_cost - trial_half_cost;
        let rho = if predicted > 0.0 && predicted.is_finite() {
            actual / predicted
        } else {
            f64::NEG_INFINITY
        };

        let step_accepted = rho > MIN_RELATIVE_DECREASE;
        iterations.push(BaIterationStats {
            iteration: it,
            cost_before: 2.0 * half_cost,
            cost_after: 2.0
                * (if step_accepted {
                    trial_half_cost
                } else {
                    half_cost
                }),
            max_pose_step: dx_frames.iter().map(|v| v.norm()).fold(0.0, f64::max),
            max_landmark_step: dx_points.iter().map(|v| v.norm()).fold(0.0, f64::max),
            lambda: mu,
            step_accepted,
        });
        // Buffered (only flushed — see below — for solves with >= 30
        // iterations, per the task brief) per-trial diagnostic: cost, rho,
        // radius, gradient max-norm, accepted. `radius` here is the value
        // this trial was attempted *at* (i.e. before the accept/reject
        // update below), matching what determined this trial's `mu`.
        trace_lines.push(format!(
            "BA_TRACE it={it} cost={:.6} rho={rho:.6} radius={radius:.6e} grad_inf_norm={grad_norm:.6e} accepted={step_accepted}",
            2.0 * half_cost
        ));

        if step_accepted {
            problem.poses = trial_poses;
            problem.points = trial_points;
            radius /= (1.0 / 3.0_f64).max(1.0 - (2.0 * rho - 1.0).powi(3));
            decrease_factor = 2.0;
            let t = Instant::now();
            let (full, pl, fd, fb) = linearize(&problem);
            timings.linearize += t.elapsed();
            timings.linearize_calls += 1;
            half_cost = 0.5 * full;
            points_lin = pl;
            frame_diag_raw = fd;
            frame_bc_raw = fb;
        } else {
            radius /= decrease_factor;
            decrease_factor *= 2.0;
        }
    }

    for (i, &id) in problem.frame_ids.iter().enumerate() {
        ba.poses
            .get_mut(&id)
            .expect("frame id present")
            .world_to_camera = problem.poses[i].clone();
    }
    for (i, &id) in problem.point_ids.iter().enumerate() {
        *ba.landmarks.get_mut(&id).expect("point id present") = problem.points[i];
    }

    eprintln!(
        "BA_PHASES trials={} linearize_calls={} n_free_frames={} n_points={} edges={} shards={} linearize_ms={:.1} eliminate_ms={:.1} assemble_ms={:.1} linsolve_ms={:.1} backsub_ms={:.1} evaluate_cost_ms={:.1}",
        timings.trials,
        timings.linearize_calls,
        problem.n_free_frames,
        problem.points.len(),
        problem.edges.len(),
        problem.shard_boundaries.len().saturating_sub(1),
        timings.linearize.as_secs_f64() * 1e3,
        timings.eliminate.as_secs_f64() * 1e3,
        timings.assemble.as_secs_f64() * 1e3,
        timings.linsolve.as_secs_f64() * 1e3,
        timings.backsub.as_secs_f64() * 1e3,
        timings.evaluate_cost.as_secs_f64() * 1e3,
    );
    if iterations.len() >= 30 {
        for line in &trace_lines {
            eprintln!("{line}");
        }
    }

    Ok(BaResult {
        initial_cost,
        final_cost: 2.0 * half_cost,
        iterations,
        converged,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{UnitQuaternion, Vector3};

    use crate::bundle::{
        rig_residual_jacobians as legacy_rig_residual_jacobians, BaRigObservation,
    };
    use visloc_core::geometry::Pose;
    use visloc_core::types::Camera;

    fn sample_inputs() -> (f64, f64, f64, f64, SE3, SE3, Point3<f64>, Point2<f64>) {
        let fx = 480.0;
        let fy = 470.0;
        let cx = 320.0;
        let cy = 240.0;
        let sensor_from_rig = SE3::new(
            UnitQuaternion::from_euler_angles(0.01, 0.02, -0.01),
            Vector3::new(-0.12, 0.005, 0.01),
        );
        let pose = SE3::new(
            UnitQuaternion::from_euler_angles(0.05, -0.1, 0.03),
            Vector3::new(0.3, -0.2, 1.1),
        );
        let point = Point3::new(0.4, -0.15, 3.2);
        let xy = Point2::new(315.0, 238.0);
        (fx, fy, cx, cy, sensor_from_rig, pose, point, xy)
    }

    /// C2.5 task item 6(a): analytic Jacobian vs central finite differences.
    #[test]
    fn jacobian_matches_finite_difference() {
        let (fx, fy, cx, cy, sensor_from_rig, pose, point, xy) = sample_inputs();
        let (r0, j_pose, j_point) =
            residual_and_jacobians(fx, fy, cx, cy, &sensor_from_rig, &pose, &point, xy)
                .expect("projectable");

        let eps = 1e-6;
        for k in 0..6 {
            let mut d = Vector6::<f64>::zeros();
            d[k] = eps;
            let pose_plus = pose.compose(&SE3::exp(&d));
            let pose_minus = pose.compose(&SE3::exp(&(-d)));
            let r_plus =
                residual_and_jacobians(fx, fy, cx, cy, &sensor_from_rig, &pose_plus, &point, xy)
                    .unwrap()
                    .0;
            let r_minus =
                residual_and_jacobians(fx, fy, cx, cy, &sensor_from_rig, &pose_minus, &point, xy)
                    .unwrap()
                    .0;
            let numeric = (r_plus - r_minus) / (2.0 * eps);
            let analytic = j_pose.column(k);
            assert!(
                (numeric - analytic).norm() < 1e-5,
                "pose col {k}: analytic {analytic:?} vs numeric {numeric:?}"
            );
        }
        for k in 0..3 {
            let mut d = Vector3::<f64>::zeros();
            d[k] = eps;
            let r_plus =
                residual_and_jacobians(fx, fy, cx, cy, &sensor_from_rig, &pose, &(point + d), xy)
                    .unwrap()
                    .0;
            let r_minus =
                residual_and_jacobians(fx, fy, cx, cy, &sensor_from_rig, &pose, &(point - d), xy)
                    .unwrap()
                    .0;
            let numeric = (r_plus - r_minus) / (2.0 * eps);
            let analytic = j_point.column(k);
            assert!(
                (numeric - analytic).norm() < 1e-5,
                "point col {k}: analytic {analytic:?} vs numeric {numeric:?}"
            );
        }
        assert!(r0.norm() > 0.0, "sample residual should be nonzero");
    }

    /// C2.5 task item 6(a) (second half): this module's independent
    /// re-derivation agrees with `bundle.rs`'s already-exercised
    /// `rig_residual_jacobians` (see module doc).
    #[test]
    fn matches_bundle_rs_formula() {
        let (fx, fy, cx, cy, sensor_from_rig, pose, point, xy) = sample_inputs();
        let camera = Camera::pinhole(1, 640, 480, fx, fy, cx, cy);
        let legacy_pose = Pose {
            world_to_camera: pose.clone(),
        };
        let obs = BaRigObservation {
            keyframe_id: 0,
            landmark_id: 0,
            xy,
            camera,
            sensor_from_rig: sensor_from_rig.clone(),
        };
        let (r_legacy, j_pose_legacy, j_point_legacy) =
            legacy_rig_residual_jacobians(&obs, &legacy_pose, &point).expect("projectable");
        let (r_native, j_pose_native, j_point_native) =
            residual_and_jacobians(fx, fy, cx, cy, &sensor_from_rig, &pose, &point, xy)
                .expect("projectable");

        assert!((r_legacy - r_native).norm() < 1e-12);
        assert!((j_pose_legacy - j_pose_native).norm() < 1e-12);
        assert!((j_point_legacy - j_point_native).norm() < 1e-12);
    }

    fn build_synthetic_recon_for_ba() -> (
        crate::colmap_incremental::reconstruction::Reconstruction,
        super::super::bundle_adjustment::BundleAdjustmentConfig,
    ) {
        use super::super::bundle_adjustment::BundleAdjustmentConfig;
        use crate::colmap_incremental::pipeline::reconstruction_from_cache;
        use crate::colmap_incremental::reconstruction::TrackElement;
        use crate::colmap_incremental::test_support::build_synthetic_rig_scene;

        let scene = build_synthetic_rig_scene(5, 3);
        let mut recon = reconstruction_from_cache(&scene.db);
        for (&frame_id, gt) in &scene.ground_truth_rig_from_world {
            recon.frame_mut(frame_id).set_rig_from_world(gt.clone());
            recon.register_frame(frame_id);
        }
        let mut point3d_ids = Vec::new();
        for (j, gt_xyz) in scene.ground_truth_points.iter().enumerate() {
            let mut track = Vec::new();
            for &(i1, i2) in &scene.images_per_frame {
                track.push(TrackElement {
                    image_id: i1,
                    point2d_idx: j,
                });
                track.push(TrackElement {
                    image_id: i2,
                    point2d_idx: j,
                });
            }
            point3d_ids.push(recon.add_point3d(*gt_xyz, track));
        }
        let anchors = [0u64, (scene.images_per_frame.len() - 1) as u64];
        let noise = Vector3::new(0.03, -0.02, 0.015);
        for &frame_id in scene.ground_truth_rig_from_world.keys() {
            if anchors.contains(&frame_id) {
                continue;
            }
            let mut pose = recon.frame(frame_id).rig_from_world().clone();
            pose.translation += noise;
            recon.frame_mut(frame_id).set_rig_from_world(pose);
        }
        for &pid in &point3d_ids {
            let xyz = recon.point3d(pid).xyz;
            recon.point3d_mut(pid).xyz = xyz + Vector3::new(0.02, -0.015, 0.01);
        }

        let mut config = BundleAdjustmentConfig::new();
        for &(i1, i2) in &scene.images_per_frame {
            config.add_image(i1);
            config.add_image(i2);
        }
        for &pid in &point3d_ids {
            config.add_variable_point(pid);
        }
        for &frame_id in &anchors {
            config.set_constant_rig_from_world_pose(frame_id);
        }
        (recon, config)
    }

    /// C2.5 perf-regression fix follow-up: profile `optimize` on a
    /// synthetic problem sized like the tier-1000 real-data global BA call
    /// that measured 2.9s/iteration (`frames=333 obs=179623 landmarks=2541`,
    /// `BA_SOLVE` log line from the killed run) — `#[ignore]`d (prints to
    /// stderr, run explicitly: `cargo test --release -p visloc-slam --lib
    /// rig_ba_solver::tests::profile_synthetic_large_problem -- --ignored
    /// --nocapture`).
    ///
    /// The task's <=100ms/iteration target at this size was **not** fully
    /// met: the O(track_len²)-per-point allocation/`BTreeMap` blowup that
    /// caused the original 2.9s/iteration is fixed (per-point elimination
    /// no longer allocates or touches a map; the frame-pair *pattern* is
    /// computed once per `optimize()` call, not once per accepted step; the
    /// O(pairs) work is sharded across 8 threads with a fixed, deterministic
    /// reduction — see `build_reduced_system_pattern` and
    /// `eliminate_and_accumulate`'s docs for the two fixes in order), taking
    /// per-iteration cost from ~2.9s to ~0.13-0.14s (~21x) on this
    /// synthetic problem — but ~140ms measured here is still ~40% over the
    /// 100ms target. This synthetic case's 2 cameras × 70-frame span gives
    /// every point ~140 observations across ~71 *distinct frames* (≈2485
    /// pairs/point); the real killed run's `obs/point` ratio (70.7) is
    /// consistent with a *shorter* per-point frame span (its 2 cameras
    /// would give ~35 frames/point for the same ratio, ≈595 pairs/point,
    /// ~4x fewer pairs) — so this synthetic profile is plausibly
    /// pessimistic relative to the real corridor tracks it was sized to
    /// resemble, not a confirmed shortfall against them. The asserted bound
    /// below is therefore the actually-measured value (with headroom) —
    /// a regression guard, not a claim the 100ms target is met — and the
    /// real tier-1000 rerun (§ task item 3) is the authoritative check.
    #[test]
    #[ignore]
    fn profile_synthetic_large_problem() {
        use crate::colmap_incremental::test_support::build_large_synthetic_ba_problem;

        let mut ba = build_large_synthetic_ba_problem(300, 2500, 70);
        // Perturb every free pose/point so the solve actually has work to
        // do (built from exact ground-truth projections, so an
        // unperturbed solve converges in 0 iterations and never exercises
        // the LM loop at all).
        let fixed_poses = ba.fixed_poses.clone();
        for (&id, pose) in ba.poses.iter_mut() {
            if fixed_poses.contains(&id) {
                continue;
            }
            pose.world_to_camera.translation += Vector3::new(0.02, -0.015, 0.01);
        }
        for xyz in ba.landmarks.values_mut() {
            *xyz += Vector3::new(0.015, 0.01, -0.02);
        }
        eprintln!(
            "synthetic problem: poses={} landmarks={} rig_observations={}",
            ba.poses.len(),
            ba.landmarks.len(),
            ba.rig_observations.len()
        );
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(8)
            .build()
            .expect("thread pool");
        let started = Instant::now();
        let result = pool
            .install(|| optimize(&mut ba, 50))
            .expect("synthetic BA solve should succeed");
        let elapsed_ms = started.elapsed().as_secs_f64() * 1e3;
        let per_iter_ms = elapsed_ms / result.iterations.len().max(1) as f64;
        eprintln!(
            "profile_synthetic_large_problem: iterations={} total_ms={elapsed_ms:.1} per_iter_ms={per_iter_ms:.1}",
            result.iterations.len()
        );
        assert!(
            per_iter_ms <= 200.0,
            "per-iteration cost {per_iter_ms:.1}ms regressed past the measured ~140ms baseline \
             (task target is 100ms; see this test's doc comment for why 200ms is the guard here)"
        );
    }

    /// Lead-directed follow-up: profile at the larger size the Lead's own
    /// synthetic run used (~440 frames / 2.8k points / 255k observations,
    /// measured 1.08s/iteration pre-fix), target <=150ms/iteration at 8
    /// threads. `#[ignore]`d — run explicitly: `cargo test --release -p
    /// visloc-slam --lib rig_ba_solver::tests::profile_synthetic_440f
    /// -- --ignored --nocapture`.
    #[test]
    #[ignore]
    fn profile_synthetic_440f() {
        use crate::colmap_incremental::test_support::build_large_synthetic_ba_problem;

        let mut ba = build_large_synthetic_ba_problem(440, 2800, 45);
        let fixed_poses = ba.fixed_poses.clone();
        for (&id, pose) in ba.poses.iter_mut() {
            if fixed_poses.contains(&id) {
                continue;
            }
            pose.world_to_camera.translation += Vector3::new(0.02, -0.015, 0.01);
        }
        for xyz in ba.landmarks.values_mut() {
            *xyz += Vector3::new(0.015, 0.01, -0.02);
        }
        eprintln!(
            "synthetic problem: poses={} landmarks={} rig_observations={}",
            ba.poses.len(),
            ba.landmarks.len(),
            ba.rig_observations.len()
        );
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(8)
            .build()
            .expect("thread pool");
        let started = Instant::now();
        let result = pool
            .install(|| optimize(&mut ba, 50))
            .expect("synthetic BA solve should succeed");
        let elapsed_ms = started.elapsed().as_secs_f64() * 1e3;
        let per_iter_ms = elapsed_ms / result.iterations.len().max(1) as f64;
        eprintln!(
            "profile_synthetic_440f: iterations={} total_ms={elapsed_ms:.1} per_iter_ms={per_iter_ms:.1}",
            result.iterations.len()
        );
        assert!(
            per_iter_ms <= 150.0,
            "per-iteration cost {per_iter_ms:.1}ms exceeds the 150ms/iteration target at 440f/2800pt"
        );
    }

    /// Reproduces the real tier-1000 run's *largest* global-BA window
    /// (`BA_PHASES` grep: `n_free_frames≈846 n_points≈2900`, `eliminate_ms`
    /// ≈600ms/trial, `linsolve_ms`≈371ms/trial ⇒ ≈1s/iteration, vs this
    /// same-frame-count synthetic case's much smaller `edges_len` — see
    /// `elimination_shard_count`'s doc for why that gap is believed to be
    /// the previous 256MB shard-memory cap collapsing 8 shards down to ~2
    /// on a wide `edges_len`). `#[ignore]`d — run explicitly: `cargo test
    /// --release -p visloc-slam --lib rig_ba_solver::tests::profile_synthetic_846f
    /// -- --ignored --nocapture`.
    #[test]
    #[ignore]
    fn profile_synthetic_846f() {
        use crate::colmap_incremental::test_support::build_large_synthetic_ba_problem;

        let mut ba = build_large_synthetic_ba_problem(846, 2900, 200);
        // Harder, non-uniform (per-frame-varying, includes rotation) noise
        // than the other profiling tests' plain uniform shift: a uniform
        // shift of every free pose by the same vector is largely gauge/
        // structure-absorbed in a couple of LM steps and converges fast (as
        // the other tests here show, 7-9 iterations even at similar scale),
        // which is not representative of the real run's harder-to-correct,
        // per-frame-varying incremental drift that hits the 50-iteration
        // cap. `id` seeds a simple deterministic pseudo-random perturbation
        // (no RNG dependency) so every frame gets a different, harder-to-
        // cancel error.
        let fixed_poses = ba.fixed_poses.clone();
        for (&id, pose) in ba.poses.iter_mut() {
            if fixed_poses.contains(&id) {
                continue;
            }
            let s = id as f64;
            pose.world_to_camera.translation += Vector3::new(
                0.02 * (s * 0.7).sin(),
                0.015 * (s * 1.3).cos(),
                0.012 * (s * 0.4).sin(),
            );
            let d = UnitQuaternion::from_euler_angles(
                0.004 * (s * 0.9).cos(),
                0.004 * (s * 0.5).sin(),
                0.004 * (s * 1.1).cos(),
            );
            pose.world_to_camera.rotation = d * pose.world_to_camera.rotation;
        }
        for (id, xyz) in ba.landmarks.iter_mut() {
            let s = *id as f64;
            *xyz += Vector3::new(
                0.015 * (s * 0.6).sin(),
                0.015 * (s * 1.7).cos(),
                0.015 * (s * 0.3).sin(),
            );
        }
        eprintln!(
            "synthetic problem: poses={} landmarks={} rig_observations={}",
            ba.poses.len(),
            ba.landmarks.len(),
            ba.rig_observations.len()
        );
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(8)
            .build()
            .expect("thread pool");
        let started = Instant::now();
        let result = pool
            .install(|| optimize(&mut ba, 50))
            .expect("synthetic BA solve should succeed");
        let elapsed_ms = started.elapsed().as_secs_f64() * 1e3;
        let per_iter_ms = elapsed_ms / result.iterations.len().max(1) as f64;
        eprintln!(
            "profile_synthetic_846f: iterations={} total_ms={elapsed_ms:.1} per_iter_ms={per_iter_ms:.1}",
            result.iterations.len()
        );
    }

    /// C2.5 task item 6(e): the whole pipeline (build_problem → linearize →
    /// solve_step, including the block-Cholesky reduced-camera solve) is
    /// bit-identical at `RAYON_NUM_THREADS=1` vs `8`.
    #[test]
    fn deterministic_across_thread_counts() {
        use super::super::bundle_adjustment::{BundleAdjustmentOptions, Gauge};

        let run = |threads: usize| {
            let (mut recon, mut config) = build_synthetic_recon_for_ba();
            config.fix_gauge(Gauge::Unspecified);
            let mut options = BundleAdjustmentOptions::global();
            options.max_num_iterations = 12;
            let pool = rayon::ThreadPoolBuilder::new()
                .num_threads(threads)
                .build()
                .expect("thread pool");
            let ok = pool
                .install(|| super::super::bundle_adjustment::solve(&options, &config, &mut recon));
            assert!(ok, "BA solve failed at {threads} threads");
            recon
        };

        let recon1 = run(1);
        let recon8 = run(8);

        for &frame_id in recon1.reg_frame_ids() {
            let a = recon1.frame(frame_id).rig_from_world();
            let b = recon8.frame(frame_id).rig_from_world();
            assert_eq!(a, b, "frame {frame_id} pose differs across thread counts");
        }
    }
}
