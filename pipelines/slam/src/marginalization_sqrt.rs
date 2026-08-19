//! Square-root (QR) sliding-window marginalization — a Rust reimplementation of the
//! algorithm described in "Square Root Marginalization for Sliding-Window Bundle
//! Adjustment" (Demmel, Schubert, Sommer, Cremers, Usenko, ICCV 2021,
//! arXiv:2109.02182), the design behind Basalt's `basalt_vio` (BSD-3-Clause).
//!
//! The conventional route marginalizes a leaving variable with a Schur complement on the
//! **Hessian** `Λ = JᵀΩJ` (see [`crate::marginalization::marginalize`]). Squaring `J` and
//! inverting `Λ_mm` loses numerical conditioning and yields a dense prior. This module keeps
//! the estimator in **square-root / QR factor form**: stacked weighted Jacobian rows `J` and
//! weighted residuals `r` are carried directly (never `JᵀJ`), and a leaving block is
//! eliminated with a **rank-revealing QR** that updates the square-root factor in place.
//! Applied on the factor matrix (`[marg | keep]`, marg left), the QR triangularizes the marg
//! columns, accumulating their effective rank `marg_rank`; the retained block
//! `J[marg_rank.., marg_size..]` / `r[marg_rank..]` is the exact square root of the
//! Schur-complement marginal — `marg_sqrt_Hᵀ·marg_sqrt_H == Λ'` and
//! `marg_sqrt_Hᵀ·marg_sqrt_b == η'`.
//!
//! Rank-deficiency is handled gracefully: a marg column whose pivoted magnitude is ≤ a cutoff
//! (default `√eps`) is dropped (marginalized with a Moore-Penrose-style zero on the unobservable
//! direction) rather than causing the Schur-inverse to blow up.
//!
//! This is an independent reimplementation of the published algorithm, not a copy of Basalt
//! source. Pure dense linear algebra on [`nalgebra::DMatrix`]/[`nalgebra::DVector`], no
//! [`crate::PoseGraph`] dependency, mirroring [`crate::marginalization`].

use std::collections::HashSet;

use nalgebra::{DMatrix, DVector};

/// The result of a square-root marginalization: the retained block's square-root factor and
/// right-hand side, such that `factorᵀ·factor` equals the Schur-complement marginal
/// `Λ_kk − Λ_km Λ_mm⁻¹ Λ_mk` and `factorᵀ·rhs` equals `η_k − Λ_km Λ_mm⁻¹ η_m`.
#[derive(Debug, Clone)]
pub struct SqrtMarginal {
    /// Kept-block square-root factor (k rows × keep-count cols). `k` is
    /// `max(total_rank − marg_rank, 1)`.
    pub factor: DMatrix<f64>,
    /// Kept-block square-root right-hand side (k rows).
    pub rhs: DVector<f64>,
    /// Rank accumulated by the marginalized columns only.
    pub marg_rank: usize,
}

/// Marginalize every state column *not* in `keep` out of the square-root (weighted) factor
/// system `(jac, resid)`, returning the kept block's square-root factor/rhs.
///
/// - `jac`: stacked weighted Jacobian rows — row count = factor count, col count = state dim,
///   each row the square-root-information-weighted Jacobian of one factor.
/// - `resid`: stacked weighted residuals, same row count as `jac`.
/// - `keep`: retained state column indices (into `jac`); `marg` the leaving ones. `keep` and
///   `marg` must partition the full column range (`keep.len() + marg.len() == ncols`), never
///   overlap, and every index must be `< ncols`.
/// - `rank_threshold`: rank cutoff for Householder diagonals; `None` defaults to `√eps`.
///
/// Returns `None` if shapes disagree, `keep`/`marg` overlap or leave a gap, or an index is out
/// of range.
pub fn marginalize_sqrt(
    jac: &DMatrix<f64>,
    resid: &DVector<f64>,
    keep: &[usize],
    marg: &[usize],
    rank_threshold: Option<f64>,
) -> Option<SqrtMarginal> {
    if jac.ncols() == 0 {
        return None;
    }
    if resid.len() != jac.nrows() {
        return None;
    }
    let n = jac.ncols();
    if keep.len() + marg.len() != n {
        return None;
    }
    let mut seen = HashSet::new();
    for &i in keep {
        if i >= n {
            return None;
        }
        if !seen.insert(i) {
            return None;
        }
    }
    for &i in marg {
        if i >= n {
            return None;
        }
        // marg must not overlap keep.
        if seen.contains(&i) {
            return None;
        }
        seen.insert(i);
    }
    if seen.len() != n {
        return None;
    }

    let threshold = rank_threshold.unwrap_or_else(|| (f64::EPSILON).sqrt());

    // Work copies: rows stay put; we permute columns into [marg | keep] (marg left).
    let mut qj = DMatrix::zeros(jac.nrows(), n);
    let mut qr = resid.clone();

    // Copy jac columns into [marg | keep] order (marg first, then keep).
    let marg_size = marg.len();
    let mut perm: Vec<usize> = Vec::with_capacity(n);
    perm.extend_from_slice(marg);
    perm.extend_from_slice(keep);
    for (pos, &orig) in perm.iter().enumerate() {
        for r in 0..jac.nrows() {
            qj[(r, pos)] = jac[(r, orig)];
        }
    }

    // Rank-revealing QR over cols 0..n, left→right, stopping when no rows remain.
    // `total_rank` = effective rank accumulated; `marg_rank` = that once the marg block passed.
    let rows = jac.nrows();
    let cols = n;
    let mut total_rank = 0usize;
    let mut marg_rank = 0usize;

    for k in 0..cols {
        if total_rank >= rows {
            break;
        }
        let row_start = total_rank;
        let nrows = rows - row_start;
        let ncols_tail = cols - k - 1;

        // Exact Householder reflector for column k over rows [row_start..rows).
        // v = x, with v[0] += sign(x[0])*||x|| ; pivot beta = -sign(x[0])*||x|| .
        let mut norm = 0.0_f64;
        for r in row_start..rows {
            norm += qj[(r, k)] * qj[(r, k)];
        }
        norm = norm.sqrt();
        if norm > threshold {
            let x0 = qj[(row_start, k)];
            let sigma = if x0 >= 0.0 { 1.0 } else { -1.0 };
            let beta = -sigma * norm;
            // v[0] (we will only store the sub-pivot part in qj for the reflector, and put the
            // pivot `beta` at (row_start, k)). Build the full vector in a temp to compute tau.
            let v0 = x0 + sigma * norm;
            // Norm of v.
            let v_norm_sq = v0 * v0 + (norm * norm - x0 * x0);
            let tau = 2.0 / v_norm_sq;

            // Store pivot.
            qj[(row_start, k)] = beta;

            // Apply H = I - tau*v*v^T to the trailing block rows [row_start+1..rows) × cols
            // [k+1..cols). We keep v0 implicit at position 0 and store sub-pivot v in qj rows.
            // For each column c in [k+1..cols): dot = v0*qj[row_start,c] + sum_{r>row_start} v[r]*qj[r,c];
            // then new = old - tau*v*dot.
            apply_reflector_to_block(&mut qj, row_start, k, nrows, ncols_tail, v0, tau);
            // Apply H to the RHS rows [row_start..rows).
            apply_reflector_to_rhs(&mut qr, row_start, nrows, v0, tau, &qj, k);

            // Overwrite the sub-pivot householder-vector entries with 0 (clean factor).
            for r in (row_start + 1)..rows {
                qj[(r, k)] = 0.0;
            }
            total_rank += 1;
        } else {
            // Rank-deficient column: zero it.
            for r in row_start..rows {
                qj[(r, k)] = 0.0;
            }
        }

        if marg_size > 0 && k == marg_size - 1 {
            marg_rank = total_rank;
        }
    }
    // If the marg block is empty, its rank is trivially 0.
    if marg_size == 0 {
        marg_rank = 0;
    }

    let keep_count = keep.len();
    // Rows available after the marg block for the kept square-root factor. Clamp to the
    // physical matrix height so a rank-saturated marg block cannot index past the row range;
    // when no rows remain for the kept block, return `None` (no retained prior) instead of
    // panicking on an out-of-range index.
    let available = rows.saturating_sub(marg_rank);
    if available == 0 {
        return None;
    }
    let nominal = total_rank.saturating_sub(marg_rank).max(1);
    let keep_valid_rows = nominal.min(available).max(1);

    // Extract marg_sqrt_H = qj[marg_rank .. marg_rank+keep_valid_rows, marg_size .. marg_size+keep_count]
    let mut factor = DMatrix::zeros(keep_valid_rows, keep_count);
    for r in 0..keep_valid_rows {
        for c in 0..keep_count {
            factor[(r, c)] = qj[(marg_rank + r, marg_size + c)];
        }
    }
    let rhs = DVector::from_fn(keep_valid_rows, |r, _| qr[marg_rank + r]);

    Some(SqrtMarginal {
        factor,
        rhs,
        marg_rank,
    })
}

/// Apply the Householder reflector `H = I − τ·v·vᵀ` (with `v` the current column `k` over rows
/// `[row_start .. row_start+nrows)`, `v[0] = v0` implicit, the sub-pivot entries stored in
/// `qj[(row_start+1.., k)]`) to the trailing block rows `[row_start+1.., )` × cols
/// `[k+1 .. k+1+ncols_tail)`.
fn apply_reflector_to_block(
    qj: &mut DMatrix<f64>,
    row_start: usize,
    k: usize,
    _nrows: usize,
    ncols_tail: usize,
    v0: f64,
    tau: f64,
) {
    let rows_lo = row_start + 1;
    let row_hi = qj.nrows();
    // For each trailing column, compute dot = v·col over [row_start..row_hi).
    for c in (k + 1)..(k + 1 + ncols_tail) {
        let mut dot = v0 * qj[(row_start, c)];
        for r in rows_lo..row_hi {
            dot += qj[(r, k)] * qj[(r, c)];
        }
        let scale = tau * dot;
        qj[(row_start, c)] -= scale * v0;
        for r in rows_lo..row_hi {
            qj[(r, c)] -= scale * qj[(r, k)];
        }
    }
}

/// Apply the same reflector to the RHS rows `[row_start .. row_start+nrows)`. Note the RHS must
/// use the full `v` vector (v0 at `row_start`, sub-pivot entries previously stored in column
/// `k`), so this is called AFTER the sub-pivot `v` entries are still present in `qj[(.., k)]`
/// (i.e. before they are zeroed).
fn apply_reflector_to_rhs(
    qr: &mut DVector<f64>,
    row_start: usize,
    nrows: usize,
    v0: f64,
    tau: f64,
    qj: &DMatrix<f64>,
    k: usize,
) {
    let row_hi = row_start + nrows;
    let mut dot = v0 * qr[row_start];
    for r in (row_start + 1)..row_hi {
        dot += qj[(r, k)] * qr[r];
    }
    let scale = tau * dot;
    qr[row_start] -= scale * v0;
    for r in (row_start + 1)..row_hi {
        qr[r] -= scale * qj[(r, k)];
    }
}

/// Produce the equivalent square-root marginal from a dense information system `(lambda, eta)`
/// via an LDLᵀ square root (the "SqToSqrt" bridge, Basalt `marginalizeHelperSqToSqrt`): first
/// Schur-marginalize to `(Λ', η')`, then factor `marg_sqrt_H = √D·Lᵀ·Pᵀ` (with `P` the LDLT
/// permutation) and `marg_sqrt_b` such that `marg_sqrt_Hᵀ·marg_sqrt_H == Λ'` and
/// `marg_sqrt_Hᵀ·marg_sqrt_b == η'`. This lets callers that already hold dense `JᵀJ` connect to
/// the square-root slot.
pub fn marginalize_sqrt_from_information(
    lambda: &DMatrix<f64>,
    eta: &DVector<f64>,
    keep: &[usize],
    _marg: &[usize],
) -> Option<SqrtMarginal> {
    let (lambda_prime, eta_prime) = crate::marginalization::marginalize(lambda, eta, keep)?;
    sqrt_factor_of_information(&lambda_prime, &eta_prime)
}

/// Factor an information system `(Λ, η)` into square-root form `(factor, rhs)` so that
/// `factorᵀ·factor == Λ` and `factorᵀ·rhs == η`, via a (rank-revealing) Cholesky with
/// non-negative pivots. Since the input is a Schur-complement marginal of an SPD system, it is
/// symmetric positive-semidefinite; near-zero pivots are clamped/dropped to keep the output a
/// valid square root.
fn sqrt_factor_of_information(lambda: &DMatrix<f64>, eta: &DVector<f64>) -> Option<SqrtMarginal> {
    let n = lambda.nrows();
    if lambda.ncols() != n || eta.len() != n {
        return None;
    }
    // Cholesky R with R^T R == Λ. nalgebra's cholesky consumes self; its `.l()` is the
    // lower-triangular L with L L^T == Λ.
    let chol = lambda.clone().cholesky()?;
    let l = chol.l();

    // Square-root factor = R = L^T (upper-triangular), so factor^T factor == Λ.
    let factor = l.transpose();

    // pivots (diagonal of R) for rank trimming.
    let pivots: Vec<f64> = (0..n).map(|i| factor[(i, i)].abs()).collect();
    let rank_threshold = (f64::EPSILON).sqrt();

    // rhs: we want factor^T·rhs == eta, and factor^T = L (lower-triangular), so
    // forward-substitute L·rhs = eta.
    let l_low = l; // L is lower triangular
    let mut rhs = DVector::zeros(n);
    for i in 0..n {
        let mut acc = eta[i];
        for j in 0..i {
            acc -= l_low[(i, j)] * rhs[j];
        }
        let d = l_low[(i, i)];
        rhs[i] = if d.abs() > rank_threshold {
            acc / d
        } else {
            0.0
        };
    }

    // Drop rows whose pivot is negligible (rank trimming) so the retained factor is a clean
    // square root of the PSD marginal.
    let mut keep_rows: Vec<usize> = Vec::new();
    for (i, &pivot) in pivots.iter().enumerate() {
        if pivot > rank_threshold {
            keep_rows.push(i);
        }
    }
    if keep_rows.is_empty() {
        keep_rows.push(0);
    }
    let k = keep_rows.len();
    let mut out_factor = DMatrix::zeros(k, n);
    let mut out_rhs = DVector::zeros(k);
    for (out_r, &in_r) in keep_rows.iter().enumerate() {
        for c in 0..n {
            out_factor[(out_r, c)] = factor[(in_r, c)];
        }
        out_rhs[out_r] = rhs[in_r];
    }

    Some(SqrtMarginal {
        factor: out_factor,
        rhs: out_rhs,
        marg_rank: 0,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::marginalization::marginalize;

    fn spd_from(m: &DMatrix<f64>) -> DMatrix<f64> {
        let n = m.nrows();
        m.transpose() * m + DMatrix::identity(n, n) * (n as f64)
    }

    fn max_abs(a: &DMatrix<f64>, b: &DMatrix<f64>) -> f64 {
        a.iter()
            .zip(b.iter())
            .map(|(x, y)| (x - y).abs())
            .fold(0.0_f64, f64::max)
    }

    fn vec_abs(a: &DVector<f64>, b: &DVector<f64>) -> f64 {
        (a - b).norm()
    }

    /// Given a dense SPD information `lambda`, synthesize a square-root factor `J` with
    /// `JᵀJ == lambda` via Cholesky, and a residual `r` so that `Jᵀr == eta` via a feasible r.
    fn make_sqrt(lambda: &DMatrix<f64>, eta: &DVector<f64>) -> (DMatrix<f64>, DVector<f64>) {
        let chol = lambda.clone().cholesky().unwrap();
        let j = chol.l().transpose(); // R such that R^T R == Lambda  (upper-triangular)
                                      // Choose r so that J^T r == eta: r = (J^T)^+ eta. Simplest: since J is square invertible,
                                      // r = J^{-T} eta.
        let r = j.transpose().try_inverse().unwrap() * eta;
        (j, r)
    }

    #[test]
    fn sqrt_marginal_matches_schur_without_marg() {
        // keep all → factor stays, factor^T factor == lambda.
        let lambda = spd_from(&DMatrix::from_fn(4, 4, |i, j| {
            ((i * 5 + j * 3) % 7) as f64 * 0.1
        }));
        let eta = DVector::from_fn(4, |i, _| i as f64);
        let (j, r) = make_sqrt(&lambda, &eta);
        let keep: Vec<usize> = (0..4).collect();
        let sm = marginalize_sqrt(&j, &r, &keep, &[], None).unwrap();
        // factor^T factor == lambda
        let hh = sm.factor.transpose() * &sm.factor;
        assert!(
            max_abs(&hh, &lambda) < 1e-8,
            "keep-all factor H: {}",
            max_abs(&hh, &lambda)
        );
        // factor^T rhs == eta (on common span)
        let e = sm.factor.transpose() * &sm.rhs;
        // Since r solves J^T r = eta and J invertible, expect eta.
        assert!(
            vec_abs(&e, &eta) < 1e-6,
            "keep-all rhs: {}",
            vec_abs(&e, &eta)
        );
    }

    #[test]
    fn sqrt_marginal_matches_schur_single_marg() {
        let m = DMatrix::from_fn(5, 5, |i, j| ((i * 3 + j * 7) % 5) as f64 * 0.2 - 0.4);
        let lambda = spd_from(&m);
        let eta = DVector::from_fn(5, |i, _| 0.5 * i as f64 - 1.0);
        let keep = [0usize, 2, 4];
        let marg = [1usize, 3];

        // Reference Schur.
        let (lp, ep) = marginalize(&lambda, &eta, &keep).unwrap();

        // Square-root path: factor the full lambda, marginalize via QR.
        let (j, r) = make_sqrt(&lambda, &eta);
        let sm = marginalize_sqrt(&j, &r, &keep, &marg, None).unwrap();

        // factor^T factor should equal the Schur marginal.
        let hh = sm.factor.transpose() * &sm.factor;
        assert!(
            max_abs(&hh, &lp) < 1e-6,
            "sqrt marginal H != Schur H: {}",
            max_abs(&hh, &lp)
        );
        // factor^T rhs should equal the Schur rhs.
        let ee = sm.factor.transpose() * &sm.rhs;
        assert!(
            vec_abs(&ee, &ep) < 1e-6,
            "sqrt marginal rhs != Schur rhs: {}",
            vec_abs(&ee, &ep)
        );
    }

    #[test]
    fn sqrt_marginal_from_information_matches_schur() {
        let m = DMatrix::from_fn(6, 6, |i, j| ((i * 5 + j * 3) % 7) as f64 * 0.1 - 0.3);
        let lambda = spd_from(&m);
        let eta = DVector::from_fn(6, |i, _| (i as f64) - 2.5);
        let keep = [1usize, 3, 5];

        let (lp, ep) = marginalize(&lambda, &eta, &keep).unwrap();
        let sm = marginalize_sqrt_from_information(&lambda, &eta, &keep, &[0, 2, 4]).unwrap();
        let hh = sm.factor.transpose() * &sm.factor;
        assert!(
            max_abs(&hh, &lp) < 1e-8,
            "info-sqrt H != Schur H: {}",
            max_abs(&hh, &lp)
        );
        let ee = sm.factor.transpose() * &sm.rhs;
        assert!(
            vec_abs(&ee, &ep) < 1e-8,
            "info-sqrt rhs != Schur rhs: {}",
            vec_abs(&ee, &ep)
        );
    }

    #[test]
    fn bad_inputs_are_rejected() {
        let j = DMatrix::from_fn(3, 4, |_, _| 1.0);
        let r = DVector::zeros(3);
        // shapes mismatch
        assert!(marginalize_sqrt(&j, &DVector::zeros(2), &[0], &[1, 2, 3], None).is_none());
        // keep+marg != ncols
        assert!(marginalize_sqrt(&j, &r, &[0, 1], &[3], None).is_none());
        // overlap
        assert!(marginalize_sqrt(&j, &r, &[0, 1], &[1, 2, 3], None).is_none());
        // out of range
        assert!(marginalize_sqrt(&j, &r, &[0, 99], &[1, 2, 3], None).is_none());
    }

    /// A marginalized column with zero information in the factor (an under-constrained
    /// direction) is dropped gracefully (Moore-Penrose-style, matching Basalt's rank cutoff)
    /// rather than corrupting the kept block. Reference: the exact Schur marginal with a
    /// pseudo-inverse `Λ' = (JᵀJ)_kk − (JᵀJ)_km (JᵀJ)_mm⁺ (JᵀJ)_mk` and `η' = η_k −
    /// (JᵀJ)_km (JᵀJ)_mm⁺ η_m` (pseudo-inverse because the marg block is rank-deficient).
    #[test]
    fn rank_deficient_marg_column_is_dropped() {
        let mut j = DMatrix::from_fn(5, 4, |i, c| ((i * 7 + c * 3) % 11) as f64 * 0.1 - 0.5);
        let r = DVector::from_fn(5, |i, _| 0.3 * i as f64 - 1.0);
        // Zero state column 2 -> it becomes unobservable and is then marginalized.
        for i in 0..5 {
            j[(i, 2)] = 0.0;
        }
        let keep = [0usize, 1, 3];

        let sm = marginalize_sqrt(&j, &r, &keep, &[2usize], None).unwrap();
        assert_eq!(sm.marg_rank, 0);
        assert_eq!(sm.factor.ncols(), keep.len());

        // Ground-truth Schur marginal via pseudo-inverse of the (rank-deficient) marg block.
        let lambda = j.transpose() * &j; // 4x4
        let eta = j.transpose() * &r; // 4
        let keep_ix = keep;
        let marg_ix = [2usize];
        let mut lm_kk = DMatrix::zeros(3, 3);
        let mut lm_km = DMatrix::zeros(3, 1);
        let mut lm_mm = DMatrix::zeros(1, 1);
        for (a, &ia) in keep_ix.iter().enumerate() {
            for (b, &ib) in keep_ix.iter().enumerate() {
                lm_kk[(a, b)] = lambda[(ia, ib)];
            }
        }
        for (a, &ia) in keep_ix.iter().enumerate() {
            for (b, &ib) in marg_ix.iter().enumerate() {
                lm_km[(a, b)] = lambda[(ia, ib)];
            }
        }
        lm_mm[(0, 0)] = lambda[(marg_ix[0], marg_ix[0])];
        let lm_mm_pinv = if lm_mm[(0, 0)] > 1e-12 {
            1.0 / lm_mm[(0, 0)]
        } else {
            0.0
        };
        let lp = &lm_kk - lm_km.clone() * (lm_mm_pinv) * lm_km.transpose();
        let eta_k = DVector::from_fn(3, |a, _| eta[keep_ix[a]]);
        let eta_m = eta[marg_ix[0]];
        let ep = &eta_k - lm_km.clone() * (lm_mm_pinv * eta_m);

        let hh = sm.factor.transpose() * &sm.factor;
        assert!(
            max_abs(&hh, &lp) < 1e-6,
            "rank-defic marg H: {}",
            max_abs(&hh, &lp)
        );
        let ee = sm.factor.transpose() * &sm.rhs;
        assert!(
            vec_abs(&ee, &ep) < 1e-6,
            "rank-defic marg rhs: {}",
            vec_abs(&ee, &ep)
        );
    }
}
