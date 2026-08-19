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

    // Step 4 — Convert each Chow-Liu tree edge into a RecoveredRelativePoseFactor.
    let mut factors = Vec::with_capacity(sparse.edges.len());
    for (bi, bj) in sparse.edges {
        let (i, j) = if bi < bj { (bi, bj) } else { (bj, bi) };
        // Extract the (12×12) joint information block on [pose_i; pose_j].
        let ri = i * VI_POSE_DOF;
        let rj = j * VI_POSE_DOF;
        let mut omega_joint = DMatrix::zeros(VI_POSE_DOF * 2, VI_POSE_DOF * 2);
        // [0..6, 0..6] = Λ_pose(i,i)
        omega_joint
            .view_mut((0, 0), (VI_POSE_DOF, VI_POSE_DOF))
            .copy_from(&lambda_pose.view((ri, ri), (VI_POSE_DOF, VI_POSE_DOF)));
        // [0..6, 6..12] = Λ_pose(i,j)
        omega_joint
            .view_mut((0, VI_POSE_DOF), (VI_POSE_DOF, VI_POSE_DOF))
            .copy_from(&lambda_pose.view((ri, rj), (VI_POSE_DOF, VI_POSE_DOF)));
        // [6..12, 0..6] = Λ_pose(j,i)
        omega_joint
            .view_mut((VI_POSE_DOF, 0), (VI_POSE_DOF, VI_POSE_DOF))
            .copy_from(&lambda_pose.view((rj, ri), (VI_POSE_DOF, VI_POSE_DOF)));
        // [6..12, 6..12] = Λ_pose(j,j)
        omega_joint
            .view_mut((VI_POSE_DOF, VI_POSE_DOF), (VI_POSE_DOF, VI_POSE_DOF))
            .copy_from(&lambda_pose.view((rj, rj), (VI_POSE_DOF, VI_POSE_DOF)));

        // Relative-pose information: Ω_rel = Sᵀ(SΩS^{-1})S where S = [-I | I].
        // Expanded: Ω_rel = Ω_jj − Ω_ji · Ω_ii⁻¹ · Ω_ij  (Schur complement of i in the joint).
        // This gives the (6×6) information of δT_j given δT_i marginalized out.
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
            None => omega_jj,
        };

        factors.push(RecoveredRelativePoseFactor {
            block_i: i,
            block_j: j,
            omega_joint,
            omega_relative,
        });
    }
    Some(factors)
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
}
