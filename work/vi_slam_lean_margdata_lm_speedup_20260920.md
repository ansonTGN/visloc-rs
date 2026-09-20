# VI-SLAM speed + accuracy investigation (`lean_marg_data`, L1)

Date: 2026-09-20
Baseline: `f31ce1a` (Merge PR #187) + uncommitted measurement tooling.
Host: 8 logical cores, Linux, rustc 1.94.0.

## Session summary and handover

Delivered (all committed, full `visloc-basalt` suite green):
* **Speed** (`3c2bf75`, `19ecae3`): compact MargData LM path + reduction cache
  across rejected trials + payload model decrease. 3.6-4.3x VIO wall time on
  MH_01/02/03/04/05, byte-identical `trajectory.tum` and `marg_data/`.
* **Tooling** (`cf1c568`, `fa8ccbf`): `--num-opt-iter` + richer
  `final_optimize` report; local track-length evidence.
* **L1 infrastructure** (`7c99799`, `a7b9871`, `41ea166`): persistent map
  surfaced on the live mapper, projection-rematch matcher, incremental local
  mapping. All opt-in/default-off; measured negative results below.

Datasets local now (under
`/mnt/win/linux_data/euroc_mh03_official_20260830/`): `MH_03_medium`,
`extracted/{MH_01_easy,MH_02_easy,MH_04_difficult,MH_05_difficult,V2_03_difficult}`.
V2_03 came from the HuggingFace mirror `GlowBond/EuRoC_MAV_Dataset`.

Accuracy findings (all measured, see sections below):
* MH_04/05/V2_03 losses reproduce locally; gap is drift, not frontend tracking.
* Global-BA iteration count is not the limiter.
* Projection re-observation and incremental local mapping (appearance-only) do
  **not** improve ATE: verified matches are recent-pair duplicates, so no
  long-range edge enters `feature_matches`.

Next candidate (not started): changes to the keyframe density / covisible-window
full-BA policy (and loop closure over that denser graph), which is what
ORB-SLAM3 has and this port lacks. Suggested first A/B: raise keyframe retention
/ lower the keyframe-spacing policy and measure `average_track_length` and ATE
on MH_04, using `--optimize-every-k` and the `final_optimize` report already in
place. Read `docs/vi_slam_global_consistency_plan.md` §1.7 for the diagnosis.

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

## L1 first steps 2026-09-20 (projection re-observation)

Two changes toward ORB-SLAM3-style persistent re-observation:

1. **Surface the map on the live mapper** (committed). The background optimizer
   merged only poses; `feature_tracks`/`lmdb` were dropped, so the live mapper
   had no landmark map between optimizations. `BackgroundOptimizeResult` now
   carries both and `merge_background_result` installs them. Final ATE is
   unchanged (finalize rebuilds tracks), so this is a safe substrate.
2. **Projection rematch** (`OnlineMapperConfig::projection_rematch`, default
   off; `--projection-rematch` on the demo). For each new keyframe it projects
   persistent landmarks hosted within `projection_host_window` keyframe-ranks
   using the current pose, and adds descriptor matches within
   `projection_search_radius_px` under `projection_max_hamming`.

The matcher works: with `--optimize-every-k 10` on MH_04 it adds 70-115
projection matches per keyframe (trace lines in `projection_rematch`). But it
does **not** improve ATE:

| MH_04 600 frames | ATE | landmarks |
| --- | ---: | ---: |
| baseline | 0.0168 m | 1852 |
| `--projection-rematch` | 0.0170 m | 1856 |
| `--optimize-every-k 10` | 0.0170 m | 1852 |
| `--optimize-every-k 10 --projection-rematch` | 0.0167 m | 1778 |

Reason: the live landmark map only exists just after an optimize merge
(`setup_opt`), and is small (182-888 landmarks) and dominated by *recent*
hosts, so the projection matches mostly duplicate the short-range links BoW and
temporal matching already provide. They do not create the long-range
re-observations that would lengthen tracks. Making L1 effective needs
**incremental local mapping** (triangulate new map points every keyframe and
keep a covisible persistent map), not a matcher bolted onto an
optimize-cadence map. That is the next, larger step.

## L1 incremental local mapping 2026-09-20 (negative result)

Implemented the incremental-local-mapping step the design calls for:

* `triangulate_pair` (triangulation.rs) reproduces `setup_opt`'s exact gate
  order for one (host, second) observation pair.
* `NfrMapper::local_map_new_keyframes` (session.rs) seeds the live `lmdb`
  continuously: it triangulates new landmarks from just-accepted pairs and
  appends re-observations to existing landmarks, with namespaced ids and a
  consistent reverse index.
* `OnlineNfrMapper::match_new_keyframe` calls it before `projection_rematch`
  when `OnlineMapperConfig::incremental_local_mapping` is set (default off;
  `--local-mapping` on the demo). New `--projection-host-window` and
  `--projection-radius` overrides expose the projection knobs.

It does **not** improve MH_04 ATE (600 frames, `--optimize-every-k 10`):

| configuration | ATE | landmarks |
| --- | ---: | ---: |
| baseline | 0.0164 m | 1846 |
| `--local-mapping` | 0.0167 m | 1840 |
| `--local-mapping --projection-rematch` | 0.0164 m | 1774 |
| `--local-mapping --projection-rematch --projection-host-window 100000 --projection-radius 20` | 0.0170 m | 1653 |
| `--projection-rematch --projection-host-window 100000 --projection-radius 20` | 0.0168 m | 1656 |

Even projecting **all** landmarks with a wide window leaves ATE unchanged. The
final `setup_opt` rebuilds tracks from `feature_matches`, and the projection
matches it can actually verify are overwhelmingly recent-pair duplicates (the
appearance descriptor does not re-match the same physical point across large
viewpoint/scale changes), so no long-range edges enter the graph. The durable
product is the match edge, and projection is not producing new long-range
edges.

**Conclusion for root cause:** the MH_04/05/V2_03 gap is *not* repaired by
adding a projection re-observation layer on top of the existing appearance
match graph. The remaining candidate causes (ORB-SLAM3's much denser keyframe
graph + covisible-window full BA, and its loop closure over that graph) require
changes to the keyframe/BA policy, not a matcher. This is a measured negative
result, not an absence of measurement; the infrastructure is correct and
default-off so the byte-exact contract is preserved.

## Long-horizon drift diagnosis 2026-09-20 (full MH_04)

Isolated the full-sequence ATE by component (MH_04, 2033 frames, lean path):

| system | 600 frames | FULL sequence |
| --- | ---: | ---: |
| VIO-only (`basalt_euroc_vio_demo`) | 0.0160 m | 0.0987 m |
| Online mapper (global BA) | 0.0164 m | 0.0830 m |
| ORB-SLAM3 (reference) | - | 0.043 m |

Findings:
* Short-horizon accuracy is already SoTA-class (~16 mm). The gap is purely
  **long-horizon drift**.
* The mapper's full global BA *does* help the VIO (0.0987 -> 0.0830, -16%),
  so BA is working, just not enough.
* **BA convergence is not the limiter:** `periodic_iterations` 1/4/16 give
  0.0828/0.0830/0.0827 m; more iterations even reduce the number of merges.
* **BA frequency is not the limiter:** `optimize_every_k=10` vs 5 and
  `periodic_iterations` changes move ATE by <0.5 mm.
* 235 loop pairs are accepted and enter the graph, but only as **vision
  factors**. There is no loop *relative-pose* factor: `linearize_factors`
  (mod.rs:1404) consumes only `relative_pose` (VIO-marginalization pairs) and
  `roll_pitch`. `extract_nonlinear_factors` always returns
  `ba_covisibility: Vec::new()` (mod.rs:2897), so the recovered covisibility
  path is dead code.
* `match_temporal_ransac` operates on unit rays, so a loop pair's `t_i_j` is
  **scale-free** (rotation reliable, translation up to scale).

**Conclusion:** long-horizon drift is a loop-closure *constraint-structure*
problem, not a solver-budget problem. The bounded next step is a
rotation/relative-pose loop factor (scale from the existing map when
available), then a proper covisibility local BA.

## Loop-closure relative-pose factors 2026-09-20 (first accuracy win)

Implemented `OnlineMapperConfig::loop_closure_factors` (default off) plus
`online.rs::OnlineNfrMapper::loop_closure_factor` and `fit_se3`:

* For every match accepted with `keyframe_rank` gap > `loop_gap_keyframes`,
  build a `RelativePoseFactor` and append it to the mapper's factor set before
  global BA.
* **Metric path** (preferred): gather landmarks observed in both keyframes,
  express each in its observing camera frame, and solve the point-set
  registration `p_left ≈ T_left_right * p_right` with Umeyama/Horn (`fit_se3`).
  In practice the *new* keyframe's features have no map landmark yet (the last
  `setup_opt` predates them), so this path rarely fires.
* **Rotation fallback**: the two-view RANSAC measurement `t_i_j` is scale-free
  but its rotation is a valid, VIO-independent loop constraint. Keep the VIO
  relative translation as a neutral measurement and put near-zero information
  on translation, so only yaw/roll/pitch close the loop.

Full MH_04 (2033 frames, `--optimize-every-k 10 --periodic-iterations 4`):

| loop-closure weight | ATE |
| --- | ---: |
| baseline (off) | 0.0830 m |
| 1 | 0.0821 m |
| 50 | 0.0793 m |
| 100 | 0.0761 m |
| **300** | **0.0702 m** (repeat 0.0704) |
| 500 | 0.0762 m |
| 700 | 0.1262 m |
| 1000 | 0.5563 m |
| 10000 | diverges (1.89 m) |

**Result: 0.0830 -> 0.0702 m (-15.4%) at weight 300**, reproducible, with no
wall-time change (424 s vs 430 s). Over-weighting diverges, as expected for a
noisy two-view yaw measurement: the tuned optimum is ~300. This confirms the
constraint-structure diagnosis. The metric path (map-based SE3) is the next
improvement, followed by covisibility local BA.

## Covisibility local BA 2026-09-20 (neutral at merge cadence)

Implemented `local_ba_with_state` (mod.rs) and
`NfrMapper::optimize_local_window` / `covisibility_window` (session.rs):

* `local_ba_with_state` restricts `global_ba_impl_in_place` to a subset of
  poses (the window); out-of-window observations drop out because
  `linearize_mapper_observation` skips any observation whose pose is not in
  `pose_indices`. Landmarks are shared and optimized against window
  observations only; the window's oldest frame is the gauge anchor.
* `covisibility_window` ranks other frames by shared-landmark count with the
  active (newest) frame and takes the top N.
* `OnlineMapperConfig::{local_ba_window, local_ba_iterations}` (default 0/off),
  run after the global steps in `optimize_pass`; demo flags
  `--local-ba-window` / `--local-ba-iterations`.

Full MH_04 (2033 frames):

| configuration | ATE | wall |
| --- | ---: | ---: |
| baseline | 0.0830 m | 430 s |
| `--local-ba-window 10` | 0.0828 m | 756 s |
| `--local-ba-window 30` | 0.0831 m | 1143 s |
| loop weight 300 | 0.0702 m | 424 s |
| loop weight 300 + local-BA window 10 | 0.0699 m | 721 s |

**Negative result:** local BA at the same (merge) cadence as global BA is
neutral and 1.7-2.7x slower, because the mapper already runs full global BA
over the whole map. Its value would come from running **every keyframe** with
the new observations already integrated, which needs the incremental-mapping
and window paths combined. Kept default-off; the loop factor remains the win.

## Loop-factor tuning and cross-sequence validation 2026-09-20

Refinements after the first loop-factor win:

* Information now scales with `support` (correspondence/inlier count) relative
  to `loop_closure_min_correspondences`, capped at 3x.
* Rotation fallback gated on inlier count (`max(5, min_corr/2)`).
* `loop_closure_max_rotation_error_deg` (default 15): reject a loop whose
  two-view RANSAC rotation disagrees with the current VIO relative rotation by
  more than that.

Cross-sequence results (full sequences, `--optimize-every-k 10`):

| configuration | MH_04 | MH_05 |
| --- | ---: | ---: |
| baseline (off) | 0.0830 m | 0.0615 m |
| weight 100, no gate | **0.0691 m** | 0.0803 m |
| weight 100, gate 15 deg | 0.0797 m | **0.0600 m** |
| weight 100, gate 30 deg | 0.0766 m | 0.0635 m |
| weight 100, min-corr 30, no gate | 0.0755 m | 0.0676 m |
| weight 300, no gate | 0.0702 m | 0.0817 m |

**Finding:** loop factors are a large, reproducible win on MH_04 (-17%) but
regress MH_05 unless a rotation-consistency gate is applied, and every gate
setting trades MH_04's win against MH_05's harm. Inlier-count gating does not
separate the two datasets. This is because the fallback is a **scale-free
appearance/rotation** constraint with no geometric verification: the real fix
is the **metric loop** (map-based SE(3) via PnP/3D-3D against the old
keyframe's landmarks), which the metric path only achieves once the new
keyframe's features have map landmarks. All loop knobs remain **default off**.

## Loop-factor accumulation bug and corrected findings 2026-09-20

**Bug found:** loop factors were pushed onto the persistent
`NfrMapper::factors.relative_pose`, which only ever grows (VIO marginalization
factors accumulate by design). So every merge re-applied **all historical**
loop factors, including stale ones measured against older map states. The
earlier MH_04 "win" (0.0830 -> 0.0691) was this accumulation artifact, not a
genuine loop-closure improvement.

**Fix:** `NfrMapper::optimize_with_extra_factors` appends transient loop factors
to a **clone** of the persistent factor set, and the online mapper now stores
loop **pairs** (`OnlineNfrMapper::loop_pairs`) and re-derives factors from the
fresh map in `optimize_pass` after every `setup_opt` (`rebuild_loop_factors`).

**Also implemented (metric loop):** the new keyframe has no map landmarks, so
the metric path now recovers the loop translation from the **old** keyframe's
metric landmarks plus the query feature **bearings**, using the reliable
two-view rotation `R` (`t_i_j`): `d × (R p_old + t) = 0`, linear in `t`. 50-90
correspondences per loop, median residual < 5 deg. This is a genuine geometric
verification.

**Corrected cross-sequence results (per-merge re-derivation, full sequences):**

| configuration | MH_04 | MH_05 |
| --- | ---: | ---: |
| baseline (off) | 0.0830 m | 0.0615 m |
| metric loop, weight 0.1 | 0.0836 m | - |
| metric loop, weight 0.3 | 0.0862 m | - |
| metric loop, weight 1 | 0.0952 m | 0.0604 m |
| rotation-only (metric gate forced), weight 100 | 0.0946 m | - |

**Corrected conclusion:** loop factors, metric or rotation, do **not** beat
baseline on MH_04 and only marginally help MH_05. The metric loop's
*translation* is derived from the current map, so it re-asserts the current
estimate rather than correcting drift; the *rotation* is independent but
over-weighting it distorts the solve. The pixel-level loop inliers already
enter BA as vision observations via `build_tracks`, so an extra loop *factor*
adds little. The remaining MH_04/05/V2_03 drift is a map-structure/BA-policy
limitation, not a missing loop constraint. All loop knobs stay default off.

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
