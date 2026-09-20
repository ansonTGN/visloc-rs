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

## Follow-up 2026-09-20: cache the reduction across rejected trials + payload model decrease

Two further changes, both confined to the lean UpstreamF32 LM loop:

1. **Reduction cache across rejected damping trials.** A rejection leaves the
   linearization point untouched, so `problem.linearize(&state)` and the
   landmark reduction recompute bit-identical values. The loop now caches
   `(LmLinearization, ReducedNormalSystemF32)` and invalidates it only on
   acceptance, re-running just the damping, the small reduced-system solve,
   the model evaluation, and the trial cost per lambda attempt. This mirrors
   upstream Basalt's inner backtracking loop without changing the attempt
   budget or any arithmetic.
2. **Payload model decrease.** `model_cost_decrease_f32` re-factored every
   landmark on every attempt. The loop now prefers
   `ReducedNormalSystemF32::model_cost_decrease_from_payload` (the Q1/Q2
   payload retained by the same reduction) and falls back to the full
   evaluator on any structural mismatch. The payload evaluator is bit-identical
   to the full one on the audited factor mixes (`m7_q2_model_reuse_*` tests).
   Opt out with `VISLOC_RS_PAYLOAD_MODEL_DECREASE=0` (not a `VISLOC_BASALT_*`
   key, which would opt back into the diagnostic path).

### Multi-sequence verification (300-400 frames each, lean vs retained)

| Sequence | retained | lean | speedup | artifacts |
| --- | ---: | ---: | ---: | --- |
| MH_01_easy | 142.1 s | 38.5 s | 3.69x | trajectory + MargData byte-identical |
| MH_02_easy | 121.7 s | 31.3 s | 3.88x | byte-identical |
| MH_03_medium | 98.0 s | 27.5 s | 3.57x | byte-identical |
| MH_04_difficult | 195.4 s | 45.9 s | 4.26x | byte-identical |
| MH_05_difficult | 145.6 s | 36.2 s | 4.02x | byte-identical |

The rejection-heavy MH_04 benefits most (4.26x), consistent with the cache
removing relinearization on rejected trials. MH_04 300-400 frames: LM solve
183.6 s -> 33.9 s (5.4x).

Dataset note: MH_01/02/04/05 were recovered from the local
`machine_hall.zip` bundle (12.7 GB) at
`/mnt/win/linux_data/euroc_mh03_official_20260830/machine_hall.zip`; only
V2_03_difficult is not yet local.

## Accuracy diagnosis 2026-09-20 (MH_04/MH_05)

Local full-sequence online-mapper runs with the lean path reproduce the
documented losses: MH_04 SE(3) ATE **0.0829 m** (repo 0.0824, ORB-SLAM3 0.043),
MH_05 **0.0621 m** (repo 0.0594, ORB-SLAM3 0.055). Dataset frames 2032/2273,
association 97%+.

The gap is **not** frontend tracking loss. On a 400-frame MH_04 prefix the
frontend has *fewer* rejected tracks than MH_03 (16.1 vs 22.5 per frame) and a
similar observation count (~302 vs 285); full MH_04 consecutive-pose RPE is
4.9 mm translation / 0.04 deg rotation. Local tracking is healthy; the residual
ATE is slow accumulated drift.

The mapper's global BA is also not the limiter. `--num-opt-iter` A/B on a
1000-frame MH_04 prefix:

| `num_opt_iter` | ATE | first-pass cost | iters | rejected trials |
| ---: | ---: | ---: | ---: | ---: |
| 10 (default) | **0.0630 m** | 5.41M -> 1.21M | 10 | 0 |
| 50 | 0.0638 m | 5.49M -> 1.33M | 19 | 21 |

More global-BA iterations do not improve ATE (slightly worse). The optimizer
converges by ~10-19 iterations; the constraint structure of the map is the
limit, consistent with the plan's "tracks average ~8 keyframes" diagnosis.
The tractable next lever is therefore **more/longer persistent tracks**
(projection-based re-observation) feeding the global problem, not more solver
iterations or a new frontend.

### Track-length evidence for the L1 diagnosis (MH_04, 600 frames)

Offline NFR mapper on the first 600 MH_04 frames (75 packets, 82 poses,
1697 landmarks):

* median map-point observation count **9**; mean 19.5 (this counts stereo
  camera-images);
* **841 of 1697 landmarks (50%) have only 5-9 observations**, i.e. they are
  seen by a handful of keyframes and then dropped;
* long tail to 151 observations;
* `average_track_length` 17.9, 4194 tracks rejected as short
  (`metrics.tracks`).

With ~82 keyframes over 600 frames, a 9-observation point spans only ~66 VIO
frames (~2 s). ORB-SLAM3 keeps points alive across far more keyframes via
projection-based local-map re-observation, which is exactly the missing L1
mechanism: the online mapper matches by appearance (BoW) + a short temporal
window, not by projecting the persistent landmark map into each new keyframe's
search window. This is now measured, not assumed.

## Next levers (not in this change)

* **Parallelize the landmark reduction** (`reduce_landmark_factors_f32_checked_with_compact_back_substitution`)
  over landmarks with ordered `par_iter` (bit-exact because order-preserving).
  After the cache + payload changes, the reduction is the largest remaining LM
  bucket (MH_03 300f: 7.3 s of 18.4 s LM; MH_04 400f: 12.1 s of 33.9 s).
* **Structureless landmark elimination** (arXiv:2505.12337) removes landmark
  states entirely and also improves MH_04/V2_03 accuracy; high effort.
* **ICE-BA linearization-point freezing** for variables that have converged.
* The diagnostic/provenance path is intentionally unchanged; any of these
  further changes should stay inside the lean loop and be A/B-verified for
  byte-identical trajectory and MargData as above.
