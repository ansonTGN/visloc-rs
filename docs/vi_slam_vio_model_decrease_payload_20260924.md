# VIO speed: wire the Q2 model-decrease payload (2026-09-24)

## Finding

The online VIO timing breakdown for MH_01 (VIO only, 3682 frames) showed:

| bucket | seconds |
|---|---:|
| `lm_model_decrease` | 70.1 |
| `lm_linearize` | 103.1 |
| `lm_landmark_reduction` | 172.7 |

`lm_model_decrease` only needs the LM predicted decrease, so it should be cheap.
The lean LM loop in `solve_lm_without_diagnostics_f32` already prefers
`ReducedNormalSystemF32::model_cost_decrease_from_payload`. However, the
production reducer in `reduce_landmark_factors_f32_checked_with_options`
always returned `model_decrease_payload: None`; a "rollback variant" comment
said so. `move_model_decrease_payload` was called only from a unit test.

As a result, every damping attempt fell back to `model_cost_decrease_f32`,
which re-factors every landmark with Householder QR. The same behaviour is
confirmed by the fact that `VISLOC_RS_PAYLOAD_MODEL_DECREASE=0/1` made no
difference: 4.70 s against 4.57 s on the first 400 frames of MH_01.

## Change

When compact back-substitution is retained, the reducer now transfers the
projected visual Q2 rows with `move_model_decrease_payload`. That transfer
is all-or-nothing: any structural mismatch returns `None`, and the full
evaluator is used instead. The payload evaluator is bit-identical to the full
one on the audited mixes, as checked by the `m7_q2_model_reuse_*` tests.

## Verification

* All 436 `visloc-basalt` tests pass.
* VIO-only runs of the full sequences (kf5 config, 4 threads) give
  **byte-identical trajectories** between main and this change. Output is in
  `/mnt/win/linux_data/visloc_vio_parity_20260924`.

| seq | trajectory md5 (both) | user CPU main → this |
|---|---|---|
| MH_04 | 8938918e5680b837 | 777.0 → 642.0 s (−17%) |
| V2_03 | 39c6a31b237811d3 | 372.2 → 344.9 s (−7%) |
| MH_01 | c97d58b770568dd1 | 1528.3 → 1177.1 s (−23%) |

The machine was shared with other workloads during these runs, so wall times
are indicative only. On the first 400 frames of MH_01, `lm_model_decrease` fell
from 4.57 s to 3.44 s.

## Related

A `perf` profile of the same prefix attributed about 31% of VIO CPU time to
libm `fmaf` calls, because default builds lacked `+fma`. PR #210 enables
`+avx2,+fma` by default, which gives about a 1.9× wall speedup with a
byte-identical trajectory.
