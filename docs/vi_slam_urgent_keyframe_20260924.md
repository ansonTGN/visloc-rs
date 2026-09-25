# Adaptive ("urgent") keyframe spacing screen (2026-09-24)

## Motivation

The keyframe sweep (`vi_slam_campaign_results_20260922.md`) showed that keyframe
density is the only large accuracy lever found so far. Compared with spacing 5,
fixed spacing 2 had a median ATE ratio of 0.945, but it also:

* increased wall time by 1.79x and peak RSS by 2.19x;
* made V2_02 worse;
* produced 2 timeouts.

The hypothesis for this screen is that dense keyframes help mainly during
aggressive-motion segments, where tracks are lost quickly. Keyframes are
therefore added early only when the connected-track ratio collapses.

## Change (opt-in, default off)

The change adds an `EstimatorConfig::urgent_kf: Option<UrgentKeyframePolicy>`
field in `pipelines/basalt/src/vio/estimator.rs`. It is loaded from two optional
config keys, which must be set together:

* `config.vio_urgent_kf_keypoints_thresh`
* `config.vio_urgent_min_frames_after_kf`

A keyframe is taken when either condition holds:

* **Upstream rule:** `ratio < new_kf_keypoints_thresh && frames_after_kf > min_frames_after_kf`
* **Urgent rule:** `ratio < urgent_thresh && frames_after_kf > urgent_min_frames_after_kf`

When the keys are absent, the decision is exactly the upstream one.

Tests:

* `config::tests::urgent_keyframe_keys_are_optional_and_paired`
* `vio::estimator::tests::urgent_keyframe_policy_shortens_gap_only_below_its_threshold`

Together with the existing keyframe and config tests, 15 tests pass.

The screen used threshold 0.5 and urgent spacing 2. Both values were fixed before
the screen and were not tuned against its results.

## Screen

* Runner: `scripts/screen_urgent_keyframe.py`.
* Binary: one frozen binary for both arms (`slam_demo`, SHA256
  `620499e8…a634f6`). Only the config differs.
* Configs: the reference config is the keyframe sweep's `kf5` config (SHA256
  `88f482c6…`). The candidate config adds the two keys (SHA256 `a145e100…`).
* Flags: KF5, 4 threads, periodic 100x4, loop rotation 30. These match the
  recent screens.
* Execution: 1500 s per run, 20/50 GiB disk reserves, 1 GiB output cap, nice 10.
* Output: `/mnt/win/linux_data/visloc_urgent_kf_screen_20260924/experiment`.

**Host caveat:** unrelated processes (colmap and a long-running python3) were
competing for CPU. Absolute wall times are therefore about 2x those of the
2026-09-23 screens. Only the within-screen alternating pairs are comparable.

| Sequence | ATE ref → cand | wall | peak RSS | mapper KFs |
|---|---|---|---|---|
| MH_03 | 0.02696 → 0.02558 (**−5.1%**) | 488.8 → 543.3 s (+11%) | 482444 → 699052 KiB (+45%) | 345 → 437 |
| MH_04 | 0.08678 → 0.07056 (**−18.7%**) | 463.1 → 504.4 s (+9%) | 336752 → 440320 KiB (+31%) | 274 → 353 |
| V2_03 | 0.07573 → 0.04517 (**−40.4%**) | 222.6 → 291.2 s (+31%) | 278084 → 443812 KiB (+60%) | 274 → 470 |
| V2_02 | candidate **TIMEOUT** (1500 s) | — | — | — |

The runner stopped at the first failure by design. The V2_02 reference run was
not executed.

## Result

**Accuracy:** ATE improved on all three completed sequences. The gains are close
to those of fixed spacing 2:

| Sequence | kf2 campaign ATE | kf2 median, earlier sweep |
|---|---|---|
| MH_04 | 0.0665 | — |
| V2_03 | 0.0333 | 0.0372 |

**Cost:** the time and memory cost is substantially smaller than fixed spacing 2.
Wall time rose +9% to +31% (versus about +79%), and RSS rose +31% to +60%
(versus about +119%). This is one pair per sequence and is not yet a
repeat-controlled result.

**Blocking issue:** the V2_02 candidate stalled. The logs show:

* stderr stopped growing at 06:07:52;
* VIO was at about frame 1950 of 2348;
* the mapper had logged packet 404;
* the run was killed at 1500 s.

This matches the pre-existing stall seen in the keyframe sweep: 3 timeouts at
spacing 2/3, including V2_01, where VIO finished but the mapper stopped at packet
259. Denser keyframes increase exposure to that stall.

**Decision:** do not adopt. Before the policy can be used, the stall has to be
localised and fixed.

## Next: stall stack capture

`scripts/capture_stall_stacks.py` runs a single replay as a child of gdb.
`ptrace_scope=1` forbids attaching later. When stderr stops growing for 120 s,
the script interrupts the replay with SIGINT and dumps `thread apply all bt`.

The first attempt reruns the same V2_02 candidate, with output at
`/mnt/win/linux_data/visloc_urgent_kf_stall_capture_20260924/attempt1`. The
mapper is asynchronous, so the stall may not reproduce on every run.

### Attempt 1: no stall

Attempt 1 exited normally after 358.6 s under gdb. The final `kill` then
failed with "program is not being run", which gives exit code 1; that is
harmless.

The run's trajectory had an ATE of 0.012253 m over 2348 poses. For comparison,
the V2_02 medians from the keyframe sweep are 0.013111 m for kf5 and 0.017451 m
for kf2. This run is diagnostic only, because it ran under gdb with a different
host load. Even so, it suggests the urgent policy does not reproduce the kf2
regression on V2_02.

The stall is intermittent. `stall_loop.sh` (in the scratchpad) retries
attempts 2 to 9 until a stall is captured.

### Code-reading hypothesis

Every loop in the mapper's temporal RANSAC is bounded:

* the sample loop;
* the LM outer loop and inner trust-region loop (at most 1000 function evaluations);
* `lmpar` (at most 10 iterations).

However, `stewenius_eigenvectors` calls `Schur::new` at
`pipelines/basalt/src/mapper/features.rs:844`, and `stewenius_model_from_sample`
calls `SVD::new`. Both run with `max_niter = 0`, which means no iteration limit,
so a non-finite or non-convergent 10x10 complex matrix can spin forever. This is
consistent with the earlier observation that a stall happened inside a temporal
matching query (`vi_slam_ransac_capture_20260923.md`).

A bounded replacement already exists in
`/mnt/win/linux_data/visloc_ransac_guard_20260923/source` behind the
`basalt-bounded-ransac-schur` feature. It uses `Schur::try_new` with 10k
iterations and a non-finite reject, and is bit-identical on convergent
fixtures. It was never validated against a real stall.

### Attempts 2–9: no stall, V2_02 accuracy stable

All 9 attempts completed under gdb without a stall. Total wall times ranged
from 358.6 to 560.1 s.

V2_02 ATE per attempt:

| Attempt | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
|---|---|---|---|---|---|---|---|---|---|
| ATE (m) | 0.01225 | 0.01244 | 0.01227 | 0.01235 | 0.01231 | 0.01223 | 0.01256 | 0.01213 | 0.01225 |

All 9 values are below the kf5 sweep median of 0.01311. These runs are
diagnostic, not paired measurements. The stall could not be reproduced under
gdb, which changes timing.

### Guard adopted on main (default, not feature-gated)

The following guards were added in `pipelines/basalt/src/mapper/features.rs`:

* `stewenius_schur_bounded` uses `Schur::try_new` with the same epsilon and a
  budget of 10k iterations. It rejects non-finite input and output, and an
  empty eigenvector set drops that hypothesis.
* The essential-matrix loop skips non-finite matrices before `SVD::new`.

On convergent input the decomposition is bit-identical. This is tested on 64
seeded fixtures, and all 391 `visloc-basalt` lib tests pass. Behaviour changes
only where the old code would have looped forever or propagated NaN.

The guard is not proven to be the cause of the stall. Its value will be judged
by timeout counts in later screens.

Screen 2, with the guard binary and an identical protocol, writes to
`/mnt/win/linux_data/visloc_urgent_kf_screen2_20260924/experiment`.

## Screen 2 result (guard binary, 2026-09-24 08:25)

All 8 runs finished with **0 timeouts and 0 failures**. The load average was
about 9 to 14, so absolute times are inflated as before.

| Sequence | ATE | wall | CPU | peak RSS | mapper KFs |
|---|---|---|---|---|---|
| MH_03 | 0.02715 → 0.02561 (**−5.7%**) | −3.7% | +11.5% | +61.3% | 345 → 437 |
| MH_04 | 0.08413 → 0.07042 (**−16.3%**) | +12.7% | +13.4% | +43.7% | 274 → 353 |
| V2_03 | 0.07501 → 0.04542 (**−39.5%**) | +23.9% | +34.4% | +78.4% | 274 → 470 |
| V2_02 | 0.01240 → 0.01333 (+7.5%) | +24.6% | +27.7% | +57.8% | 334 → 489 |

Reproducibility against screen 1:

* MH_03, MH_04 and V2_03 ATE gains repeat closely: −5.1/−18.7/−40.4% in screen 1.
* MH_03 candidate ATE was 0.02558 and 0.02561.
* V2_03 candidate ATE was 0.04517 and 0.04542.

V2_02 is within its noise band:

* The 9 diagnostic candidate runs gave 0.0121 to 0.0126.
* This paired candidate gave 0.01333.
* The kf5 sweep median is 0.01311, and this baseline gave 0.01240.

Treat V2_02 as neutral with some variance, not as a clear win. Unlike fixed
kf2, which gives +33% on V2_02, it shows no systematic regression.

Assessment:

* **Accuracy:** large gains on the two hard sequences (MH_04 and V2_03), a
  small gain on MH_03, and neutral on V2_02.
* **Cost:** RSS rises 44 to 78% and CPU rises 11 to 34%, driven by 27 to 72%
  more mapper keyframes. This is a clear accuracy-for-resources tradeoff and
  not an all-metric improvement. It is still far cheaper than fixed kf2 for
  similar accuracy.
* **Guard:** 8 plus 1 guarded online runs, including the V2_02 candidate that
  stalled in screen 1, ran with no stall. This is supportive but not
  conclusive, because the stall was rare.

Decision: keep the urgent policy opt-in, using config keys only. It is
recommended for difficult or aggressive-motion sequences where accuracy
matters more than memory. Keep the Schur/SVD guard on by default.

Next options:

1. Reduce the per-keyframe memory cost in the mapper, which is what makes
   denser keyframes expensive.
2. Run a 3-pair confirmation on all four sequences before recommending the
   policy more broadly.

## Held-out sequences, partial (2026-09-24, sparse binary)

This is the 11-sequence kf5 vs urgent screen, with output at
`/mnt/win/linux_data/visloc_urgent_kf_all11_20260924`.

Claude Code stopped the screen at about 14:30 because the *host* was critically
low on memory, with other workloads running. This was not a failure of the run.
9 runs had completed by then; the V1_02 urgent run was in progress and is lost.

These 4 sequences were not used while designing the policy:

| Sequence | ATE kf5 → urgent | wall | peak RSS | mapper KFs |
|---|---|---|---|---|
| MH_01 | 0.01627 → 0.01545 (−5.0%) | +11.8% | 718196 → 1328392 KiB (**+85%**) | 458 → 566 |
| MH_02 | 0.02443 → 0.02583 (+5.7%) | +3.8% | 534428 → 799172 KiB (+50%) | 398 → 487 |
| MH_05 | 0.06339 → 0.06321 (−0.3%) | +5.0% | 343696 → 356200 KiB (+3.6%) | 306 → 382 |
| V1_01 | 0.03545 → 0.03545 (0.0%) | −1.9% | 507832 → 653452 KiB (+29%) | 378 → 416 |

Reading so far:

* **No accuracy gain on held-out sequences.** The four results average out
  to neutral, while memory still rises. The large gains remain specific to the
  sequences that lose tracks quickly: MH_04 and V2_03. Do not make the policy
  the default on this evidence.
* **MH_01 peak RSS is large even for kf5**, at 718 MB and 1.33 GB with urgent.
  MH_01 is the longest sequence at 3682 frames, which points to a separate
  memory component that grows with sequence length, beyond the pose matrix.
  It is the same open item as the MH_03 remaining peak.

Not yet run: V1_02 urgent, V1_03, V2_01, and the 4 design sequences.

## Default in the online demo (2026-09-25)

The urgent policy (threshold 0.5, spacing 2) was re-screened on main after two
changes: interpolated propagation (#215) and PSD factor information (#216). It
used one kf5/urgent pair per sequence, plus two extra pairs for V2_01 and
MH_02.

| seq | ATE kf5 → urgent |
|---|---|
| V2_03 | 0.0546 → **0.0451 (−17%)** |
| MH_04 | 0.0832 → **0.0700 (−16%)** |
| MH_01 | 0.0170 → **0.0155 (−8.5%)** |
| V1_03 | 0.0231 → **0.0213 (−7.7%)** |
| V1_02 | 0.0147 → **0.0137 (−7.0%)** |
| MH_03 | 0.0269 → **0.0255 (−5.2%)** |
| V2_02 | 0.0125 → **0.0120 (−4.5%)** |
| V1_01 | 0.0354 → 0.0354 |
| MH_05 | 0.0627 → 0.0634 (+1%) |
| V2_01 | +24%, +1.7%, −5.3% over three pairs |
| MH_02 | +4.9%, +4.7%, +5.1% over three pairs (small systematic cost) |

The one V2_01 outlier had 80 rejected trials in the final mapper LM, while all
four other V2_01 runs had 0. That is a separate LM robustness issue, not an
effect of the policy.

Wall time rises by 10 to 30%. V2_03 with `--pipeline` still runs at 0.71× real
time: 83 s for 116.7 s of data, with ATE 0.0449.

Decision: the **online demo enables urgent keyframes by default**. It
inserts the two config keys when the config omits them, and
`--no-urgent-keyframes` opts out. The library defaults and the
upstream-compatible config fixtures are unchanged. The screen runner now
passes `--no-urgent-keyframes` for its kf5 reference arm.
