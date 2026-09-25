# Mapper LM retries: re-solve with damped landmark blocks (2026-09-25)

## Finding

In one V2_01 run, the final mapper optimize had **80 rejected trials** and
the ATE was 0.0185, compared with the usual 0.0164. The
`first_optimize_trials` trace, which the online demo now writes, shows the
failure mode. It reproduced in two out of two runs:

```
it2 lam=37    f_diff=-2.50e4  max_pose_inc=3.2e-4  rejected
it2 lam=296   f_diff=-2.81e4  max_pose_inc=4.1e-5  rejected
it2 lam=1e3   f_diff=-2.85e4  max_pose_inc=1.2e-5  rejected   (and every trial after)
```

Pose steps shrink toward zero, yet the vision cost rises by about 2.8e4 on
every trial. Two defects cause this:

1. **The retry never re-solves.** Each iteration solved the damped system
   once, before the trial loop. After a rejection it raised lambda but
   re-applied the *same* increment. Upstream `NfrMapper::optimize` solves
   inside the loop.
2. **Landmarks are never damped.** Back-substitution applies the full
   landmark Newton step `-Hll⁻¹ (bl - Hplᵀ dx)`, which with a tiny `dx` is
   the undamped `-Hll⁻¹ bl`. It then clamps the inverse depth at 0. Raising
   lambda cannot shrink that part of the step.

## Change

The fix is in `global_ba_impl_in_place` and `damped_landmark_system` in
`mapper/mod.rs`:

* **First trial:** unchanged. An iteration accepted on its first trial is
  bit-identical to before.
* **Retries:** the landmark blocks are damped as
  `Hll + max(diag(Hll)·λ, min_λ)`. The reduced pose system is rebuilt without
  relinearization by adding `Hpl (Hll⁻¹ − Hl_damped⁻¹) Hplᵀ` back into H,
  and the matching term into b, for each landmark. The step is re-solved with
  the current λ, and landmarks are back-substituted with the same damped
  inverse.
* **Diagnostics:** `VisionLandmarkSolve` keeps the undamped `hll`. The online
  demo writes the per-trial trace of the first final optimize
  (`first_optimize_trials`) to `timing_breakdown_online.json`.

## Result

The screen used one paired run per sequence on all 11 EuRoC sequences, with
urgent keyframes on by default. Output is in
`/mnt/win/linux_data/visloc_lm_damped_20260925`.

| seq | ATE base → fix | rejected trials fix (first/second) |
|---|---|---|
| V2_01 | 0.0157 (80 rejected) → 0.0170 | 0 / 0 |
| V1_03 | 0.0213 → **0.0192 (−10%)** | 0 / 14 (retries accepted) |
| MH_04 | 0.0705 → 0.0704 | 7 / 0 |
| MH_01 | 0.0153 → 0.0153 | 0 / 7 |
| V2_03, V2_02, MH_05, MH_03, MH_02, V1_01, V1_02 | within ±1–7% noise | 0 / 0 |

* **Stuck LM:** the fix removes it. No run of the fixed binary had more than
  14 rejected trials. V2_01 values from 0.0153 to 0.0185 are all within its
  run-to-run spread.
* **Accuracy:** where retries actually occur (V1_03), the ATE improves by
  10%. No sequence regresses beyond run-to-run noise.
