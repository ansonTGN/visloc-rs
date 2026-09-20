# VI-SLAM speed: compact MargData LM path (`lean_marg_data`)

Date: 2026-09-20
Baseline: `f31ce1a` (Merge PR #187) + uncommitted measurement tooling.
Host: 8 logical cores, Linux, rustc 1.94.0.

## Problem

The online VI-SLAM (Basalt Rust port) is not real-time: EuRoC MH_03 ran at
real-time factor (RTF) 0.05-0.13. Per-frame cost was ~0.76-1.0 s. The mapper
thread was never the bottleneck (it keeps pace; queue lag bounded).

`timing_breakdown` on 200 MH_03 frames (retained MargData path) showed the
VIO estimator is 83% of wall time and the LM solver inside it is ~97% of the
estimator:

| bucket | total | share |
| --- | ---: | ---: |
| adapter_total | 152.96 s | 100% |
| adapter_estimator | 130.84 s | 86% |
| estimator_lm_solve | 126.82 s | 83% |
| lm_trial_construct_step | 42.47 s | 28% |
| lm_trial_landmark_recovery (nested) | 42.42 s | 28% |
| lm_accept | 33.78 s | 22% |
| lm_landmark_reduction | 26.13 s | 17% |
| lm_model_decrease | 10.95 s | 7% |
| lm_linearize | 6.45 s | 4% |
| adapter_frontend | 21.99 s | 14% |
| lm_linear_system_solve | 0.98 s | **0.6%** |

The actual linear algebra (`lm_linear_system_solve`, the reduced dense LDLT) is
0.6% of runtime. The cost is redundant work around it.

## Root cause

`BasaltVioEstimator::process_impl` selected the LM entry point solely from
`retain_marg_data`:

* `retain_marg_data == true` -> `WindowProblem::solve_with_timing`, i.e.
  `retain_factors = true, retain_diagnostics = true`. This is the fully
  diagnostic LM path: for every LM trial it rebuilds each landmark's factor
  and re-runs the per-landmark ABS/QR, then deep-clones pose/state sidecars
  (`WindowTrialView`) and repeats `apply_step_full` on accept.
* `retain_marg_data == false && retain_trace == false` ->
  `solve_without_diagnostics`, i.e. the compact f32 preparation path that
  consumes the Q1/R payload from the *same* reduction walk and does not
  re-factor per trial.

MargData only needs the **post-solve factor snapshot** (`solution_factors` ->
`last_rows`), which is produced by `retain_factors`. It does **not** need the
diagnostic per-trial LM payloads (`retain_diagnostics`). The estimator coupled
the two, so the online mapper always paid the diagnostic cost. The diagnostic
row counters (`prior/visual/imu/bias_factor_rows`) that `emit_marg` reads are
plain metadata and are not consumed by the mapper or by MargData serialization.

## Change

1. `WindowProblem::solve_lean_with_factors[_with_timing]` (window.rs):
   `retain_factors = true, retain_diagnostics = false`. It runs the compact
   f32 LM and derives the four `emit_marg` row counters from the final
   linearization via `fill_diagnostic_factor_rows`, so the duplicate pre-solve
   `initial_window_diagnostics` linearization is avoided. Any Basalt probe /
   compatibility environment variable falls back to the fully diagnostic path.
2. `BasaltVioEstimator::lean_marg_data` (default **off**) plus
   `set_lean_marg_data(bool)`. When on and `retain_marg_data` is set, the
   estimator uses the lean-with-factors entry point.
3. `--retained-marg-diagnostics` opt-out flag on
   `basalt_euroc_vio_demo` and `basalt_euroc_online_slam_demo`; the online
   demo enables the lean path by default and records `lean_marg_data` in its
   run summary.

Default library behavior is unchanged; the diagnostic/provenance path is
preserved bit-for-bit and remains selectable.

## Evidence

### Correctness (byte-identical)

`basalt_euroc_vio_demo`, 400 MH_03 frames, same inputs, trace retained:

* `trajectory.tum` SHA-256 `327056de...c5bd` identical between
  `--retained-marg-diagnostics` and lean.
* `marg_data/` directories identical (`diff -rq` clean).
* 200-frame run also identical (`e0f9602b...ba87`).

Regression test
`vio::window::tests::lean_with_factors_matches_retained_diagnostics_exactly`
asserts equal accepted state, cost, iteration count, every factor block, and
the populated diagnostic row counters.

Note: the *online mapper's* propagated trajectory is not reproducible across
identical runs (pre-existing background-optimizer nondeterminism, unrelated to
this change); the estimator-level VIO trajectory and MargData are deterministic
and byte-identical.

### Speed (`basalt_euroc_vio_demo`, 400 MH_03 frames, two reps)

| metric | retained (diag) | lean | speedup |
| --- | ---: | ---: | ---: |
| adapter_total | 127.2-130.5 s | 39.5-40.1 s | **3.2x** |
| per frame | 318-326 ms | 99-100 ms | **3.2x** |
| estimator_lm_solve | 115.9-118.9 s | 27.8-28.3 s | **4.2x** |
| adapter_frontend | 8.4-8.6 s | 8.7-8.8 s | 1.0x |
| estimator_marginalization | 2.42-2.43 s | 2.56-2.58 s | 0.95x |

End-to-end online mapper (`basalt_euroc_online_slam_demo`, 200 MH_03 frames):

| metric | retained | lean |
| --- | ---: | ---: |
| vio_wall_seconds | 77.9 | 28.5 |
| total_wall_seconds | 79.1 | 30.0 |
| real_time_factor | 0.128 | **0.349** |

The remaining LM cost after this change is `lm_landmark_reduction`
(~12 s / 400 frames), `lm_model_decrease` (~5 s), and `lm_linearize`
(~4.7 s); `lm_landmark_reduction` and `lm_model_decrease` still re-factor the
same landmark set twice per iteration.

## Next levers (not in this change)

* **Inner LM damping loop.** Upstream Basalt re-forms only the ~87x87 reduced
  system per rejected lambda trial; this port relinearizes and re-reduces on
  every rejection. Folding the reduction into an outer loop is the next large
  win, expected to matter most on MH_04/MH_05/V2_03 where rejection rates are
  highest.
* **Consume the compact model-decrease payload** already saved by the clean
  reducer instead of re-factoring in `model_cost_decrease_f32`.
* Parallelize `landmark_steps` over landmarks with ordered `par_iter`
  (bit-exact because order-preserving).
