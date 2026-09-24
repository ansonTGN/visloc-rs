# Mapper factor information: project indefinite matrices to PSD (2026-09-24)

## Finding

One online V2_03 run on 2026-09-24 ended with a **negative** final mapper
cost:

| | value |
|---|---|
| first optimize final cost | −6.1e5 |
| second optimize final cost | −8.6e5 |
| rejected trials | 70 and 100 |
| ATE | 0.375 m (usual: about 0.076 m) |

Vision residuals are Huber-weighted squares, so they are non-negative. A
negative total therefore means a relative-pose or roll-pitch factor with an
**indefinite information matrix**.

Those matrices are `(J cov Jᵀ)⁻¹`, where `cov` is the QR inverse of the
marginalization Hessian. When that Hessian is ill-conditioned, the result can
have negative eigenvalues. Global BA can then lower `rᵀ W r` without bound
along those directions.

## Change

`nearest_psd_if_indefinite` in `mapper/mod.rs` is applied to both factor
kinds. It leaves a PSD information matrix unchanged, bit for bit. Otherwise it
returns the nearest PSD matrix: it symmetrizes, runs an eigen-decomposition,
and clamps negative eigenvalues to 0. Non-finite input is left to the existing
rejection path, because `symmetric_eigen` does not terminate on NaN.

A process-wide counter records how often the projection fires. The online
demo prints it as `mapper_psd_information_projections=N`. Unit tests check the
bit-identical PSD case and a projected indefinite case.

## Result

The screen ran all 11 EuRoC sequences, one alternating base/fix pair each, on
main with interpolated propagation. It used `--pipeline`, kf5, periodic 100×4
and loop rotation 30. Output is in
`/mnt/win/linux_data/visloc_psd_all11_20260924`.

| seq | projections | ATE base → fix |
|---|---:|---|
| V2_03 | 32 | 0.0657 → **0.0543 (−17%)** |
| MH_03 | 29 | 0.0283 → **0.0268 (−5.4%)** |
| MH_05 | 43 | 0.0621 → **0.0599 (−3.5%)** |
| V2_02 | 21 | 0.0117 → 0.0124 (+5.7%, inside V2_02's 0.0116–0.0133 spread) |
| V2_01, V1_01, V1_02, V1_03, MH_01, MH_02, MH_04 | 0 | identical algorithm; −3% to +1% (async noise) |

* **Frequency.** Indefinite factors are routine on four sequences, not only
  in the divergent run. V2_03 had 32 of them in every run.
* **Repeatability.** Across four V2_03 runs, the fix gave an ATE of
  0.0543–0.0556. The base V2_03 values today were 0.0657–0.0763, plus one
  divergence at 0.375.
* **Sequences without projections.** These are unaffected by construction.
  Their spread shows the noise band of about ±1–3%.
