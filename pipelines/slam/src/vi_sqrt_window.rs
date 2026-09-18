//! Square-root (QR) sliding-window VIO marginal carry — Basalt Step A
//! (SqrtToSqrt window loop + FEJ re-referencing).
//!
//! The merged first step ([`crate::marginalize_sqrt_from_information`]) still starts from a
//! dense `JᵀJ` Hessian. This module carries the window's marginal in **square-root (QR)
//! factor form across every window shift**, mirroring Basalt's `marg_data` protocol
//! (`sqrt_keypoint_vio.cpp marginalize()` + `marg_helper.cpp marginalizeHelperSqrtToSqrt`,
//! ICCV'21, arXiv:2109.02182): each step stacks the window's whitened Jacobian rows and
//! residuals together with the carried sqrt marginal, marginalizes the outgoing states with
//! [`crate::marginalize_sqrt`] (never forming `JᵀJ`), and **re-references the retained rhs to
//! the current estimate** so no information is double-counted across shifts (First-Estimate-
//! Jacobian consistency).
//!
//! Inputs are supplied as a square-root stack: the caller linearizes the current window and
//! provides the stacked whitened Jacobian rows (cols = state DoF for every in-window
//! keyframe, in a fixed order) and the stacked whitened residuals, plus the carried marginal
//! from the previous step. This module owns only the carry state and the QR+FEJ protocol, so
//! it is independently testable and later wired into the real `BundleAdjustment` window via an
//! "emit sqrt rows" build.
//!
//! Independent reimplementation of the published algorithm; no Basalt source copied verbatim.
//! Pure dense `nalgebra` algebra, no [`crate::PoseGraph`] dependency.

use nalgebra::{DMatrix, DVector};

use crate::marginalization_sqrt::marginalize_sqrt;

/// The square-root marginal carried across window shifts (our analogue of Basalt `marg_data`).
///
/// It represents the prior `P(x) = 0.5 ‖J·(δ + x) + r‖²` where `δ` is the difference between
/// the current estimate and the linearization point (`lin_point`). Storing `factor` and `rhs`
/// in square-root form means `factorᵀ·factor` is the Schur-complement marginal of the
/// discarded states and `factorᵀ·rhs` its information vector — but neither `JᵀJ` nor a matrix
/// inverse is ever formed while carrying it.
#[derive(Debug, Clone, PartialEq)]
pub struct SqrtNavMarginal {
    /// Square-root factor on the retained nav states (k rows × N cols).
    pub factor: DMatrix<f64>,
    /// Square-root rhs on the retained nav states (k rows).
    pub rhs: DVector<f64>,
    /// Total DoF of the retained nav states (`N` = `factor.ncols()`).
    pub state_dof: usize,
    /// The estimate (vectorized over `state_dof`) at which `factor`/`rhs` were linearized.
    /// Used for the FEJ re-reference on the next step.
    pub lin_point: DVector<f64>,
}

/// A per-state block descriptor in the window's vectorized order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NavBlock {
    /// Pose-only 6-DoF block.
    Pose6,
    /// Pose+velocity+bias 15-DoF block.
    PoseVelBias15,
}

impl NavBlock {
    pub const fn dof(self) -> usize {
        match self {
            NavBlock::Pose6 => 6,
            NavBlock::PoseVelBias15 => 15,
        }
    }
}

/// One square-root window-marginalization step (Basalt Step A).
///
/// # Inputs
/// - `window_jac`: stacked whitened Jacobian rows of the current window, *excluding* the
///   carried marginal. Rows = factors; cols = `total_dof` in `window_blocks` order.
/// - `window_resid`: whitened residuals, one per `window_jac` row.
/// - `window_blocks`: per in-window block kind, in the order of `window_jac`'s columns.
/// - `carried`: the marginal from the previous step, whose `factor` columns are ordered over
///   the blocks listed in `carried_slots` (so they map 1:1 into those blocks' columns).
///   `None` on the initial fill.
/// - `carried_slots`: in-window block indices that `carried`'s columns correspond to, in order
///   (typically the retained states of the previous window that were not yet marginalized).
/// - `marg_blocks`: in-window block indices to **marginalize** this step (the states leaving
///   the window). The square-root prior is kept over the complement.
/// - `current_estimate`: vectorized current estimate over the **kept** blocks' DoF (in window
///   order), used for the FEJ re-reference of the output. `None` ⇒ no shift.
/// - `rank_threshold`: passed through to [`crate::marginalize_sqrt`]; `None` ⇒ default.
///
/// # Returns
/// The next carried [`SqrtNavMarginal`] on the kept blocks, re-referenced to
/// `current_estimate` (FEJ) if supplied. `None` on any layout mismatch.
#[allow(clippy::too_many_arguments)]
pub fn step_window_marginal(
    window_jac: &DMatrix<f64>,
    window_resid: &DVector<f64>,
    window_blocks: &[NavBlock],
    carried: Option<&SqrtNavMarginal>,
    carried_slots: &[usize],
    marg_blocks: &[usize],
    current_estimate: Option<&DVector<f64>>,
    rank_threshold: Option<f64>,
) -> Option<SqrtNavMarginal> {
    // Per-block column offsets.
    let mut block_offsets = Vec::with_capacity(window_blocks.len() + 1);
    let mut total = 0usize;
    for b in window_blocks {
        block_offsets.push(total);
        total += b.dof();
    }
    block_offsets.push(total);

    if window_jac.ncols() != total || window_resid.len() != window_jac.nrows() {
        return None;
    }
    for &s in carried_slots {
        if s >= window_blocks.len() {
            return None;
        }
    }
    if carried_slots.windows(2).any(|w| w[0] == w[1]) {
        return None;
    }
    for &m in marg_blocks {
        if m >= window_blocks.len() {
            return None;
        }
    }

    // Kept blocks = every block not in `marg_blocks`, in window order.
    let keep_idx: std::collections::BTreeSet<usize> = (0..window_blocks.len())
        .filter(|i| !marg_blocks.contains(i))
        .collect();
    let kept_dof: usize = keep_idx
        .iter()
        .map(|&i| block_offsets[i + 1] - block_offsets[i])
        .sum();
    let carried_dof: usize = carried_slots
        .iter()
        .map(|&s| block_offsets[s + 1] - block_offsets[s])
        .sum();

    // Build the full sqrt stack: append the carried marginal rows (if any) over `carried_slots`.
    let carried_rows = carried.map_or(0, |m| m.factor.nrows());
    let n_rows = window_jac.nrows() + carried_rows;
    let mut jac = DMatrix::zeros(n_rows, total);
    let mut resid = DVector::zeros(n_rows);

    let mut row = 0usize;
    if let Some(m) = carried {
        if m.factor.ncols() != carried_dof || m.rhs.len() != m.factor.nrows() {
            return None;
        }
        // Map carried columns (ordered over carried_slots) to their window columns.
        let mut carried_c = 0usize;
        for &slot in carried_slots {
            let dof = block_offsets[slot + 1] - block_offsets[slot];
            for r in 0..m.factor.nrows() {
                for d in 0..dof {
                    jac[(row + r, block_offsets[slot] + d)] = m.factor[(r, carried_c + d)];
                }
            }
            carried_c += dof;
        }
        for r in 0..m.factor.nrows() {
            resid[row + r] = m.rhs[r];
        }
        row += m.factor.nrows();
    }
    for r in 0..window_jac.nrows() {
        for c in 0..total {
            jac[(row + r, c)] = window_jac[(r, c)];
        }
        resid[row + r] = window_resid[r];
    }

    // Marginalize `marg_blocks` out of the sqrt stack, keeping `keep_idx` in window order.
    let mut keep_cols: Vec<usize> = Vec::with_capacity(kept_dof);
    for &i in keep_idx.iter() {
        keep_cols.extend(block_offsets[i]..block_offsets[i + 1]);
    }
    let mut marg_cols: Vec<usize> = Vec::new();
    for &i in marg_blocks {
        marg_cols.extend(block_offsets[i]..block_offsets[i + 1]);
    }
    let sm = marginalize_sqrt(&jac, &resid, &keep_cols, &marg_cols, rank_threshold)?;

    // FEJ re-reference: shift the output rhs so the prior stays referenced to the current
    // estimate (Basalt: `marg_data.b -= marg_data.H * delta`).
    let (rhs, lin_point) = match current_estimate {
        Some(est) if est.len() == kept_dof => {
            let rhs_s = sm.rhs - &sm.factor * est;
            (rhs_s, est.clone())
        }
        Some(_) => return None,
        None => (sm.rhs, DVector::zeros(kept_dof)),
    };

    Some(SqrtNavMarginal {
        factor: sm.factor,
        rhs,
        state_dof: kept_dof,
        lin_point,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::marginalization::marginalize;

    fn max_abs(a: &DMatrix<f64>, b: &DMatrix<f64>) -> f64 {
        a.iter()
            .zip(b.iter())
            .map(|(x, y)| (x - y).abs())
            .fold(0.0_f64, f64::max)
    }

    /// Build a random full-window sqrt stack over `window_blocks`: returns the whitened
    /// Jacobian rows, residuals, and the dense (JᵀJ, Jᵀr). Uses a full-rank frac-sin hash so
    /// every marginalized principal block stays SPD.
    fn make_window(
        blocks: &[NavBlock],
    ) -> (DMatrix<f64>, DVector<f64>, DMatrix<f64>, DVector<f64>) {
        let total: usize = blocks.iter().map(|b| b.dof()).sum();
        let n_rows = total + 3; // over-determined
        let j = DMatrix::from_fn(n_rows, total, |i, c| {
            let x = (i as f64) * 12.9898 + (c as f64) * 78.233;
            (x.sin() * 43758.5453).fract().abs() - 0.5
        });
        let r = DVector::from_fn(n_rows, |i, _| 0.3 * i as f64 - 1.0);
        let h = j.transpose() * &j;
        let g = j.transpose() * &r;
        (j, r, h, g)
    }

    #[test]
    fn first_fill_matches_dense_schur() {
        // Window of 3 pose-only states; marginalize the oldest (block 0), keep blocks 1,2.
        let blocks = [NavBlock::Pose6, NavBlock::Pose6, NavBlock::Pose6];
        let (j, r, h, g) = make_window(&blocks);
        let sm = step_window_marginal(&j, &r, &blocks, None, &[], &[0usize], None, None).unwrap();

        // Reference dense: info = JᵀJ (6+6+6=18), keep = blocks 1,2 (dof 6..=18).
        let keep_dense: Vec<usize> = (6..18).collect();
        let (hp, gp) = marginalize(&h, &g, &keep_dense).unwrap();

        // sqrt factorᵀ·factor == Schur marginal (dense).
        let hh = sm.factor.transpose() * &sm.factor;
        assert!(
            max_abs(&hh, &hp) < 1e-6,
            "first-fill factor H != Schur: {}",
            max_abs(&hh, &hp)
        );
        let ee = sm.factor.transpose() * &sm.rhs;
        let rhs_err = (ee - &gp).norm();
        assert!(rhs_err < 1e-6, "first-fill rhs != Schur: {rhs_err}");
        assert_eq!(sm.state_dof, 12);
    }

    #[test]
    fn fej_shift_is_invariant_under_reference() {
        // 4 pose+vel+bias states; marginalize blocks 0..3 (all but the last), keep block 3.
        let blocks = [
            NavBlock::PoseVelBias15,
            NavBlock::PoseVelBias15,
            NavBlock::PoseVelBias15,
            NavBlock::PoseVelBias15,
        ];
        let (j, r, _, _) = make_window(&blocks);
        let est = DVector::from_fn(15, |i, _| 0.2 * i as f64 - 1.5);
        let sm = step_window_marginal(
            &j,
            &r,
            &blocks,
            None,
            &[],
            &[0usize, 1, 2],
            Some(&est),
            None,
        )
        .unwrap();
        assert_eq!(sm.state_dof, 15);

        // The output rhs is referenced to `est`. Reconstructing factorᵀ·(rhs + factor·est)
        // must equal factorᵀ·rhs of the un-shifted result.
        let recon = sm.factor.transpose() * (&sm.rhs + &sm.factor * &est);
        let sm0 =
            step_window_marginal(&j, &r, &blocks, None, &[], &[0usize, 1, 2], None, None).unwrap();
        let ref0 = sm0.factor.transpose() * &sm0.rhs;
        let recon_err = (recon - &ref0).norm();
        assert!(recon_err < 1e-8, "FEJ shift not invariant: {recon_err}");
    }

    #[test]
    fn carried_marginal_accumulates_without_gaining_information() {
        // Build ONE underlying 4-block system [a,b,c,d] (24 dof) + residual.
        let n_rows = 30;
        let j = DMatrix::from_fn(n_rows, 24, |i, c| {
            let x = (i as f64) * 12.9898 + (c as f64) * 78.233;
            let v = (x.sin() * 43758.5453).fract().abs();
            v - 0.5
        });
        let r = DVector::from_fn(n_rows, |i, _| 0.3 * i as f64 - 1.0);

        // Step 1: window = [a,b,c] = cols 0..18. Marginalize a (0..6), keep [b,c] (6..18).
        let j_step1 = j.columns(0, 18).into_owned();
        let r_step1 = r.clone();
        let first = step_window_marginal(
            &j_step1,
            &r_step1,
            &[NavBlock::Pose6; 3],
            None,
            &[],
            &[0],
            None,
            None,
        )
        .unwrap();
        assert_eq!(first.state_dof, 12);

        // Step 2: window = [b,c,d] = cols 6..24 (24-6=18 dof). Carry the [b,c] marginal onto
        // slots 0,1 (b,c = cols 6..18 of the original = first 12 of the slice). Marginalize b
        // (the first 6 dof of the slice), keep [c,d].
        let j_step2 = j.columns(6, 18).into_owned(); // 18 cols = [b,c,d]
        let r_step2 = r;
        let second = step_window_marginal(
            &j_step2,
            &r_step2,
            &[NavBlock::Pose6; 3],
            Some(&first),
            &[0usize, 1], // carried on b,c (slots 0,1 of the 3-block slice)
            &[0usize],    // marginalize b
            None,
            None,
        )
        .unwrap();
        // Kept = [c,d] (slots 1,2 of the 3-block slice) = 12 dof.
        assert_eq!(second.state_dof, 12);

        // Reference (dense, same incremental data): step-2 window [b,c,d] (18 dof = cols 0..18
        // of the slice) + the carried [b,c] prior as a dense block on cols 0..6 and 6..12,
        // then marginalize b (cols 0..6), keeping [c,d] (cols 6..18). This is the exact dense
        // analogue of the QR chain on the same window.
        let h2 = j_step2.transpose() * &j_step2; // 18x18 over [b,c,d]
        let g2 = j_step2.transpose() * &r_step2;
        let h_carried = first.factor.transpose() * &first.factor; // 12x12 over [b,c]
        let g_carried = first.factor.transpose() * &first.rhs;
        let mut h_ref = h2;
        let mut g_ref = g2;
        // Place the carried prior on cols 0..12 (b,c of the slice = cols 0..6,6..12 of j_step2).
        for a in 0..12 {
            for b in 0..12 {
                h_ref[(a, b)] += h_carried[(a, b)];
            }
            g_ref[a] += g_carried[a];
        }
        let keep_cd_dense: Vec<usize> = (6..18).collect(); // keep c,d
        let (hp, _) = marginalize(&h_ref, &g_ref, &keep_cd_dense).unwrap();

        let hh = second.factor.transpose() * &second.factor;
        let hdiff = max_abs(&hh, &hp);
        assert!(hdiff < 1e-6, "carried chain H != dense: {hdiff}");
    }
}
