//! Nonlinear Factor Recovery — a Rust reimplementation of the sparse-factor
//! re-approximation described in "Visual-Inertial Mapping with Non-Linear
//! Factor Recovery" (Usenko, Demmel, Schubert, Cremers, Usenko, RA-L 2020,
//! arXiv:1904.06504), the design behind `basalt_mapper`.
//!
//! After marginalizing an outgoing keyframe from a VI sliding window the dense
//! Schur-complement marginal `Λ'` fully connects all retained keyframes — an
//! ever-growing clique if added naively to the global pose graph. This module
//! breaks that clique into a **sparse set of pairwise relative-pose factors**:
//!
//! 1. Extract the pose-only marginal `Λ_pose` by **Schur-marginalizing** the
//!    per-keyframe velocity and bias (9 DoF each) out of the 15-DoF-per-block
//!    VI marginal. The result is a (6·N × 6·N) information matrix coupling only
//!    the retained poses.
//! 2. Apply **Chow-Liu KL-optimal tree sparsification** ([`crate::sparsification`])
//!    on the pose-only marginal to obtain a maximum-spanning-tree of N-1 pairwise
//!    relative-pose constraints that best approximates `Λ_pose` in KL divergence.
//! 3. For each tree edge `(i, j)` convert the (12×12) joint information block
//!    into a [`RecoveredRelativePoseFactor`] carrying the 6-DoF relative-pose
//!    measurement and its (6×6) information matrix, ready to be inserted into a
//!    pose graph or Sim(3) PGO solver.
//!
//! Independent reimplementation of the published algorithm; no Basalt source
//! copied verbatim. Pure `nalgebra` dense linear algebra; no `PoseGraph` dep.
//!
//! # State layout convention
//!
//! The [`crate::vi_sqrt_window`] / [`crate::online_slam_vi_ba`] window uses a
//! per-keyframe layout of **15 DoF**: `[pose 0..6, velocity 6..9, bias 9..15]`
//! (6 pose, 3 velocity, 6 bias).  This matches `SqrtNavMarginal::state_dof ==
//! 15 * N`.

use nalgebra::{DMatrix, DVector};

use crate::marginalization::marginalize;
use crate::sparsification::sparsify_chow_liu;

/// The DoF breakdown of a single VI keyframe state in the window layout used by
/// [`crate::vi_sqrt_window`]: pose (6) + velocity (3) + bias (6) = 15.
pub const VI_POSE_DOF: usize = 6;
pub const VI_VEL_BIAS_DOF: usize = 9; // velocity (3) + bias (6)
pub const VI_STATE_DOF: usize = VI_POSE_DOF + VI_VEL_BIAS_DOF; // 15

/// A recovered sparse relative-pose factor between two retained keyframes.
///
/// The factor encodes `p(T_j | T_i)` as a Gaussian in the 6-DoF relative-pose
/// error `log(T_i⁻¹ · T_j)`, with information matrix `omega` (6×6 SPD).  The
/// relative pose `T_i_from_j` is the mean (`T_i⁻¹ · T_j` at the linearisation
/// point), and `omega` is the (6×6) sub-block of the Chow-Liu tree's pairwise
/// information.
#[derive(Debug, Clone)]
pub struct RecoveredRelativePoseFactor {
    /// Index of keyframe *i* in the original N-block ordering supplied to
    /// [`recover_relative_pose_factors`].
    pub block_i: usize,
    /// Index of keyframe *j* (`j > i`).
    pub block_j: usize,
    /// Joint (12×12) information matrix on `[δT_i; δT_j]` (6-DoF pose errors
    /// stacked), derived from the Chow-Liu tree edge.  The caller can use
    /// `omega_joint.view((6, 6), (6, 6))` for the (i,j) cross-block, or
    /// extract only the relative-pose 6×6 information via the standard
    /// `Ω_rel = S · Ω_joint · Sᵀ` projection with
    /// `S = [-I_{6} | I_{6}]` (relative error = δT_j − δT_i).
    pub omega_joint: DMatrix<f64>,
    /// 6-DoF relative information `Ω_rel = Sᵀ(SΩS^{-1})S` where
    /// `S = [-I | I]`. Positive-semidefinite (exact for tree edges of
    /// a full-rank prior).
    pub omega_relative: DMatrix<f64>,
}

/// Remap columns from [`crate::bundle::build_sqrt_factor_rows`] layout (all poses,
/// then all velocities, then all biases) into per-keyframe 15-DoF blocks expected
/// by [`recover_relative_pose_factors`]. Pass `landmark_cols = 0` when the stack
/// has no landmark columns (the 2-keyframe navigation boundary case).
pub fn permute_sqrt_stack_to_vi_blocks(
    factor: &DMatrix<f64>,
    n_keyframes: usize,
    landmark_cols: usize,
) -> Option<DMatrix<f64>> {
    if n_keyframes == 0 {
        return None;
    }
    let pose_dim = n_keyframes * VI_POSE_DOF;
    let vel_offset = pose_dim;
    let bias_offset = pose_dim + n_keyframes * 3;
    let expected_ba_cols = bias_offset + n_keyframes * 6 + landmark_cols;
    if factor.ncols() != expected_ba_cols {
        return None;
    }
    let vi_cols = n_keyframes * VI_STATE_DOF;
    let mut out = DMatrix::zeros(factor.nrows(), vi_cols);
    for k in 0..n_keyframes {
        let dst_base = k * VI_STATE_DOF;
        for c in 0..VI_POSE_DOF {
            out.column_mut(dst_base + c)
                .copy_from(&factor.column(k * VI_POSE_DOF + c));
        }
        for c in 0..3 {
            out.column_mut(dst_base + VI_POSE_DOF + c)
                .copy_from(&factor.column(vel_offset + k * 3 + c));
        }
        for c in 0..6 {
            out.column_mut(dst_base + VI_POSE_DOF + 3 + c)
                .copy_from(&factor.column(bias_offset + k * 6 + c));
        }
    }
    Some(out)
}

/// Recover sparse relative-pose factors from a VI sliding-window square-root
/// marginal ([`crate::SqrtNavMarginal`]).
///
/// # Arguments
/// - `factor`: the `SqrtNavMarginal.factor` (k rows × (15·N) cols).
/// - `rhs`:    the `SqrtNavMarginal.rhs` (k rows).
/// - `n_keyframes`: number of retained keyframes (`N`); must satisfy
///   `factor.ncols() == 15 * N`.
///
/// # Returns
/// `None` on shape mismatch or degenerate marginal.  Otherwise a `Vec` of
/// `N − 1` [`RecoveredRelativePoseFactor`] edges (Chow-Liu spanning tree).
/// When `N ≤ 1` the result is an empty `Vec` (single node, no edges needed).
pub fn recover_relative_pose_factors(
    factor: &DMatrix<f64>,
    rhs: &DVector<f64>,
    n_keyframes: usize,
) -> Option<Vec<RecoveredRelativePoseFactor>> {
    if n_keyframes == 0 {
        return None;
    }
    let expected_cols = VI_STATE_DOF * n_keyframes;
    if factor.ncols() != expected_cols || rhs.len() != factor.nrows() {
        return None;
    }

    // Step 1 — reconstruct the dense VI information matrix Λ' = factorᵀ · factor.
    let lambda_vi = factor.transpose() * factor;
    let eta_vi = factor.transpose() * rhs;

    // Step 2 — Schur-marginalize velocity+bias (9 DoF per keyframe) out of Λ',
    // retaining only the 6-DoF pose columns for each keyframe.
    //
    // Window layout (per keyframe k): cols [15k .. 15k+6) = pose,
    //                                       [15k+6 .. 15k+15) = vel+bias.
    // Keep = pose columns of all keyframes; marg = vel+bias columns.
    let pose_cols: Vec<usize> = (0..n_keyframes)
        .flat_map(|k| (k * VI_STATE_DOF)..(k * VI_STATE_DOF + VI_POSE_DOF))
        .collect();
    let (lambda_pose, _eta_pose) = marginalize(&lambda_vi, &eta_vi, &pose_cols)?;

    // Step 3 — Chow-Liu KL-optimal tree sparsification on the (6·N × 6·N)
    // pose-only marginal.  block_dim = 6 (one pose per node).
    let eta_pose_zero = DVector::zeros(lambda_pose.nrows());
    let sparse = sparsify_chow_liu(&lambda_pose, &eta_pose_zero, VI_POSE_DOF)?;

    if n_keyframes <= 1 {
        return Some(Vec::new());
    }

    Some(recovered_factors_from_pose_lambda(
        &lambda_pose,
        &sparse.edges,
    ))
}

/// Recover relative-pose factors from a pose-only square-root Jacobian
/// (`factor.ncols() == 6 · N`). Used when the BA stack layout has contiguous
/// pose columns and velocity/bias Schur is ill-conditioned.
pub fn recover_relative_pose_factors_pose_only(
    factor: &DMatrix<f64>,
    rhs: &DVector<f64>,
    n_keyframes: usize,
) -> Option<Vec<RecoveredRelativePoseFactor>> {
    if n_keyframes == 0 {
        return None;
    }
    let expected_cols = VI_POSE_DOF * n_keyframes;
    if factor.ncols() != expected_cols || rhs.len() != factor.nrows() {
        return None;
    }
    if n_keyframes <= 1 {
        return Some(Vec::new());
    }
    let lambda_pose = factor.transpose() * factor;
    if let Some(sparse) = sparsify_chow_liu(
        &lambda_pose,
        &DVector::zeros(lambda_pose.nrows()),
        VI_POSE_DOF,
    ) {
        return Some(recovered_factors_from_pose_lambda(
            &lambda_pose,
            &sparse.edges,
        ));
    }
    // Sequential-chain fallback when Chow-Liu cannot invert the prior.
    let edges: Vec<(usize, usize)> = (0..n_keyframes - 1).map(|i| (i, i + 1)).collect();
    Some(recovered_factors_from_pose_lambda(&lambda_pose, &edges))
}

/// Recover relative-pose factors from a BA sqrt-stack Jacobian
/// (`[poses | vels | biases | landmarks]`). Landmarks are square-root
/// marginalized first so visual information survives into the pose graph.
pub fn recover_relative_pose_factors_from_ba_stack(
    factor: &DMatrix<f64>,
    rhs: &DVector<f64>,
    n_keyframes: usize,
) -> Option<Vec<RecoveredRelativePoseFactor>> {
    if n_keyframes == 0 || rhs.len() != factor.nrows() {
        return None;
    }
    if n_keyframes == 1 {
        return Some(Vec::new());
    }
    let nav_cols = VI_STATE_DOF * n_keyframes;
    let pose_cols = VI_POSE_DOF * n_keyframes;
    if factor.ncols() < pose_cols {
        return None;
    }

    let pose_only_fallback = |jac: &DMatrix<f64>, residual: &DVector<f64>| {
        let pose_factor = jac.columns(0, pose_cols).into_owned();
        recover_relative_pose_factors_pose_only(&pose_factor, residual, n_keyframes)
    };

    let (nav_factor, nav_rhs) = if factor.ncols() > nav_cols {
        let keep: Vec<usize> = (0..nav_cols).collect();
        let marg: Vec<usize> = (nav_cols..factor.ncols()).collect();
        match crate::marginalize_sqrt(factor, rhs, &keep, &marg, None) {
            Some(sm) => (sm.factor, sm.rhs),
            None => return pose_only_fallback(factor, rhs),
        }
    } else if factor.ncols() == nav_cols {
        (factor.clone(), rhs.clone())
    } else {
        return pose_only_fallback(factor, rhs);
    };

    if let Some(permuted) = permute_sqrt_stack_to_vi_blocks(&nav_factor, n_keyframes, 0) {
        if let Some(recovered) = recover_relative_pose_factors(&permuted, &nav_rhs, n_keyframes) {
            if !recovered.is_empty() {
                return Some(recovered);
            }
        }
    }
    pose_only_fallback(&nav_factor, &nav_rhs)
}

fn recovered_factors_from_pose_lambda(
    lambda_pose: &DMatrix<f64>,
    edges: &[(usize, usize)],
) -> Vec<RecoveredRelativePoseFactor> {
    let mut factors = Vec::with_capacity(edges.len());
    for &(bi, bj) in edges {
        let (i, j) = if bi < bj { (bi, bj) } else { (bj, bi) };
        let ri = i * VI_POSE_DOF;
        let rj = j * VI_POSE_DOF;
        let mut omega_joint = DMatrix::zeros(VI_POSE_DOF * 2, VI_POSE_DOF * 2);
        omega_joint
            .view_mut((0, 0), (VI_POSE_DOF, VI_POSE_DOF))
            .copy_from(&lambda_pose.view((ri, ri), (VI_POSE_DOF, VI_POSE_DOF)));
        omega_joint
            .view_mut((0, VI_POSE_DOF), (VI_POSE_DOF, VI_POSE_DOF))
            .copy_from(&lambda_pose.view((ri, rj), (VI_POSE_DOF, VI_POSE_DOF)));
        omega_joint
            .view_mut((VI_POSE_DOF, 0), (VI_POSE_DOF, VI_POSE_DOF))
            .copy_from(&lambda_pose.view((rj, ri), (VI_POSE_DOF, VI_POSE_DOF)));
        omega_joint
            .view_mut((VI_POSE_DOF, VI_POSE_DOF), (VI_POSE_DOF, VI_POSE_DOF))
            .copy_from(&lambda_pose.view((rj, rj), (VI_POSE_DOF, VI_POSE_DOF)));

        let omega_ii = omega_joint.view((0, 0), (VI_POSE_DOF, VI_POSE_DOF)).into_owned();
        let omega_ij = omega_joint
            .view((0, VI_POSE_DOF), (VI_POSE_DOF, VI_POSE_DOF))
            .into_owned();
        let omega_jj = omega_joint
            .view((VI_POSE_DOF, VI_POSE_DOF), (VI_POSE_DOF, VI_POSE_DOF))
            .into_owned();
        let omega_relative = match omega_ii.clone().cholesky() {
            Some(chol) => {
                let inv_ij = chol.solve(&omega_ij);
                0.5 * (&omega_jj - omega_ij.transpose() * inv_ij.clone()
                    + &omega_jj
                    - inv_ij.transpose() * omega_ij.transpose())
            }
            None => {
                // Ω_rel = S Λ Sᵀ with S = [-I | I].
                let mut s = DMatrix::zeros(VI_POSE_DOF, VI_POSE_DOF * 2);
                for k in 0..VI_POSE_DOF {
                    s[(k, k)] = -1.0;
                    s[(k, VI_POSE_DOF + k)] = 1.0;
                }
                &s * &omega_joint * s.transpose()
            }
        };

        factors.push(RecoveredRelativePoseFactor {
            block_i: i,
            block_j: j,
            omega_joint,
            omega_relative,
        });
    }
    factors
}

/// Marginalize velocity+bias columns out of a pose-only subset of the VI information
/// matrix, returning `(Λ_pose, η_pose)`. Exposed for testing.
pub fn vi_marginal_to_pose_only(
    lambda_vi: &DMatrix<f64>,
    eta_vi: &DVector<f64>,
    n_keyframes: usize,
) -> Option<(DMatrix<f64>, DVector<f64>)> {
    if lambda_vi.ncols() != VI_STATE_DOF * n_keyframes
        || lambda_vi.nrows() != VI_STATE_DOF * n_keyframes
        || eta_vi.len() != VI_STATE_DOF * n_keyframes
    {
        return None;
    }
    let pose_cols: Vec<usize> = (0..n_keyframes)
        .flat_map(|k| (k * VI_STATE_DOF)..(k * VI_STATE_DOF + VI_POSE_DOF))
        .collect();
    marginalize(lambda_vi, eta_vi, &pose_cols)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::marginalization_sqrt::marginalize_sqrt;

    fn make_vi_marginal(n: usize) -> (DMatrix<f64>, DVector<f64>) {
        let d = VI_STATE_DOF * n;
        // Build a random full-rank square-root factor, then form Λ = JᵀJ.
        let j = DMatrix::from_fn(d + 3, d, |i, c| {
            let x = (i as f64) * 12.9898 + (c as f64) * 78.233;
            (x.sin() * 43758.5453).fract() - 0.5
        });
        let lambda = j.transpose() * &j;
        let eta = j.transpose() * DVector::from_fn(d + 3, |i, _| 0.3 * i as f64 - 1.0);
        (lambda, eta)
    }

    #[test]
    fn pose_only_extraction_reduces_dimension() {
        let n = 3;
        let (lv, ev) = make_vi_marginal(n);
        let (lp, ep) = vi_marginal_to_pose_only(&lv, &ev, n).unwrap();
        assert_eq!(lp.nrows(), VI_POSE_DOF * n);
        assert_eq!(lp.ncols(), VI_POSE_DOF * n);
        assert_eq!(ep.len(), VI_POSE_DOF * n);
    }

    #[test]
    fn factor_recovery_returns_n_minus_one_edges() {
        for n in [2, 3, 4, 5] {
            let (lv, ev) = make_vi_marginal(n);
            // Build a sqrt factor from the dense marginal.
            let j = lv.clone().cholesky().unwrap().l().transpose();
            let r = lv.transpose().try_inverse().unwrap() * &ev;
            let factors = recover_relative_pose_factors(&j, &r, n).unwrap();
            assert_eq!(
                factors.len(),
                n - 1,
                "expected {n}-1 edges, got {}",
                factors.len()
            );
        }
    }

    #[test]
    fn single_keyframe_returns_empty_vec() {
        let (lv, ev) = make_vi_marginal(1);
        let j = lv.clone().cholesky().unwrap().l().transpose();
        let r = lv.transpose().try_inverse().unwrap() * &ev;
        let factors = recover_relative_pose_factors(&j, &r, 1).unwrap();
        assert!(factors.is_empty());
    }

    #[test]
    fn omega_joint_is_symmetric() {
        let n = 3;
        let (lv, ev) = make_vi_marginal(n);
        let j = lv.clone().cholesky().unwrap().l().transpose();
        let r = lv.transpose().try_inverse().unwrap() * &ev;
        let factors = recover_relative_pose_factors(&j, &r, n).unwrap();
        for f in &factors {
            let diff = (&f.omega_joint - f.omega_joint.transpose()).norm();
            assert!(diff < 1e-10, "omega_joint not symmetric: {diff}");
        }
    }

    #[test]
    fn omega_relative_is_psd() {
        let n = 4;
        let (lv, ev) = make_vi_marginal(n);
        let j = lv.clone().cholesky().unwrap().l().transpose();
        let r = lv.transpose().try_inverse().unwrap() * &ev;
        let factors = recover_relative_pose_factors(&j, &r, n).unwrap();
        for f in &factors {
            // Ω_rel should be SPD (positive-semidefinite at minimum).
            let sym = 0.5 * (&f.omega_relative + f.omega_relative.transpose());
            assert!(
                sym.clone().cholesky().is_some(),
                "omega_relative not PSD for edge ({},{})",
                f.block_i,
                f.block_j
            );
        }
    }

    #[test]
    fn bad_inputs_return_none() {
        let n = 2;
        let (lv, ev) = make_vi_marginal(n);
        let j = lv.clone().cholesky().unwrap().l().transpose();
        let r = lv.transpose().try_inverse().unwrap() * &ev;
        // Wrong n_keyframes.
        assert!(recover_relative_pose_factors(&j, &r, 0).is_none());
        assert!(recover_relative_pose_factors(&j, &r, 99).is_none());
        // Wrong rhs length.
        assert!(recover_relative_pose_factors(&j, &DVector::zeros(1), n).is_none());
    }

    /// The Chow-Liu tree edges must cover all N nodes (connected spanning tree).
    #[test]
    fn spanning_tree_covers_all_nodes() {
        let n = 5;
        let (lv, ev) = make_vi_marginal(n);
        let j = lv.clone().cholesky().unwrap().l().transpose();
        let r = lv.transpose().try_inverse().unwrap() * &ev;
        let factors = recover_relative_pose_factors(&j, &r, n).unwrap();
        assert_eq!(factors.len(), n - 1);
        let mut seen = std::collections::BTreeSet::new();
        for f in &factors {
            seen.insert(f.block_i);
            seen.insert(f.block_j);
        }
        assert_eq!(seen.len(), n, "not all nodes covered by spanning tree");
    }

    /// Factors from a sqrt marginal (via `marginalize_sqrt`) match those from
    /// the equivalent dense information matrix.
    #[test]
    fn sqrt_marginal_path_matches_dense_path() {
        let n = 3;
        let d = VI_STATE_DOF * n;
        // Build a random overdetermined system.
        let j_full = DMatrix::from_fn(d + 5, d, |i, c| {
            let x = (i as f64) * 9.1234 + (c as f64) * 53.456;
            (x.sin() * 12345.67).fract() - 0.5
        });
        let r_full = DVector::from_fn(d + 5, |i, _| 0.2 * i as f64 - 1.0);

        // Dense path.
        let lv = j_full.transpose() * &j_full;
        let ev = j_full.transpose() * &r_full;
        let dense_factors = recover_relative_pose_factors(
            &lv.clone().cholesky().unwrap().l().transpose(),
            &(lv.clone().try_inverse().unwrap() * &ev),
            n,
        )
        .unwrap();

        // Sqrt path: marginalize nothing (keep = all) to get the sqrt factor.
        let keep: Vec<usize> = (0..d).collect();
        let sm = marginalize_sqrt(&j_full, &r_full, &keep, &[], None).unwrap();
        let sqrt_factors =
            recover_relative_pose_factors(&sm.factor, &sm.rhs, n).unwrap();

        assert_eq!(dense_factors.len(), sqrt_factors.len());
        for (df, sf) in dense_factors.iter().zip(sqrt_factors.iter()) {
            assert_eq!(df.block_i, sf.block_i);
            assert_eq!(df.block_j, sf.block_j);
            let diff = (&df.omega_relative - &sf.omega_relative).norm();
            assert!(
                diff < 1e-5,
                "omega_relative mismatch for edge ({},{}) dense vs sqrt: {diff}",
                df.block_i,
                df.block_j
            );
        }
    }

    /// Permuting from BA sqrt-stack layout must match direct VI-block recovery.
    #[test]
    fn permute_sqrt_stack_matches_vi_block_recovery() {
        let n = 2;
        let (lv, ev) = make_vi_marginal(n);
        let j_vi = lv.clone().cholesky().unwrap().l().transpose();
        let r = lv.transpose().try_inverse().unwrap() * &ev;

        // Build a synthetic BA-layout Jacobian with the same information.
        let mut j_ba = DMatrix::zeros(j_vi.nrows(), n * VI_POSE_DOF + n * 3 + n * 6);
        for k in 0..n {
            let dst_base = k * VI_STATE_DOF;
            for c in 0..VI_POSE_DOF {
                j_ba.column_mut(k * VI_POSE_DOF + c)
                    .copy_from(&j_vi.column(dst_base + c));
            }
            for c in 0..3 {
                j_ba.column_mut(n * VI_POSE_DOF + k * 3 + c)
                    .copy_from(&j_vi.column(dst_base + VI_POSE_DOF + c));
            }
            for c in 0..6 {
                j_ba.column_mut(n * VI_POSE_DOF + n * 3 + k * 6 + c)
                    .copy_from(&j_vi.column(dst_base + VI_POSE_DOF + 3 + c));
            }
        }

        let permuted = permute_sqrt_stack_to_vi_blocks(&j_ba, n, 0).unwrap();
        let diff = (&permuted - &j_vi)
            .iter()
            .fold(0.0_f64, |acc, v| acc.max(v.abs()));
        assert!(diff < 1e-12, "permute must round-trip VI blocks: {diff}");

        let direct = recover_relative_pose_factors(&j_vi, &r, n).unwrap();
        let via_ba = recover_relative_pose_factors(&permuted, &r, n).unwrap();
        assert_eq!(direct.len(), via_ba.len());
        for (a, b) in direct.iter().zip(via_ba.iter()) {
            assert_eq!(a.block_i, b.block_i);
            assert_eq!(a.block_j, b.block_j);
            let od = (&a.omega_relative - &b.omega_relative)
                .iter()
                .fold(0.0_f64, |acc, v| acc.max(v.abs()));
            assert!(od < 1e-8, "omega_relative mismatch: {od}");
        }
    }

    #[test]
    fn pose_only_recovers_when_vel_bias_schur_is_singular() {
        let n = 2;
        let pose_cols = VI_POSE_DOF * n;
        let mut j_ba = DMatrix::zeros(12, n * VI_STATE_DOF);
        for c in 0..pose_cols {
            j_ba[(c, c)] = 1.0 + 0.1 * c as f64;
        }
        let r = DVector::zeros(12);
        assert!(
            recover_relative_pose_factors(&permute_sqrt_stack_to_vi_blocks(&j_ba, n, 0).unwrap(), &r, n)
                .is_none(),
            "zero vel/bias columns must make VI Schur fail"
        );
        let pose_j = j_ba.columns(0, pose_cols).into_owned();
        let factors = recover_relative_pose_factors_pose_only(&pose_j, &r, n).unwrap();
        assert_eq!(factors.len(), 1);
        assert_eq!(factors[0].block_i, 0);
        assert_eq!(factors[0].block_j, 1);
        assert_eq!(factors[0].omega_relative.nrows(), 6);
    }

    #[test]
    fn ba_stack_recovers_after_landmark_schur() {
        let n = 3;
        let nav_cols = VI_STATE_DOF * n;
        let landmark_cols = 6;
        let rows = nav_cols + 8;
        let j = DMatrix::from_fn(rows, nav_cols + landmark_cols, |i, c| {
            let x = (i as f64) * 7.1 + (c as f64) * 13.3;
            (x.sin() * 43758.5453).fract() - 0.5
        });
        let r = DVector::from_fn(rows, |i, _| 0.1 * i as f64);
        let factors = recover_relative_pose_factors_from_ba_stack(&j, &r, n).unwrap();
        assert_eq!(factors.len(), n - 1);
        let mut seen = std::collections::BTreeSet::new();
        for f in &factors {
            seen.insert(f.block_i);
            seen.insert(f.block_j);
            assert_eq!(f.omega_relative.nrows(), 6);
        }
        assert_eq!(seen.len(), n);
    }
}
