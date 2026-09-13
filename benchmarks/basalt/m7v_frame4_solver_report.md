# M7v frame-4 solver/state-contract audit

Pinned oracle: Basalt `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.
Dataset prefix: EuRoC `MH_01_easy`, sensor-only during engine execution.

## Reproduction artifacts

- Upstream: `target/basalt_upstream_mh01_400f_core_trace_20260821T000012Z/trace.jsonl`
- Rust before the fixes: `target/basalt_m7v_frame4_run5/trace.jsonl`
- Rust after coupled landmark and bias-weight fixes: `target/basalt_m7v_biasweight_run5/trace.jsonl`
- 20-frame continuation: `target/basalt_m7v_run20/trace.jsonl`
- 80-frame continuation/evaluation: `target/basalt_m7v_run80/{trace.jsonl,evaluation.json}`

Frames 0--3 of the initial Rust trace agree with upstream prediction to floating-point precision. The first divergence is therefore the first active solve at frame 4, not initialization or IMU prediction.

## Proven fixed-SHA contract defects

1. `PoseState::incPose` is `p' = p + upsilon`, `R' = Exp(omega) R`. It is not a full left SE(3) exponential. Rust trial-state updates and factor finite differences now use this chart.
2. Upstream back-substitutes landmark increments before `computeError`, retains them only on accepted trials, and restores on rejection. Rust LM now has coupled trial/accept hooks and grouped anchored-stereographic landmark back-substitution.
3. Upstream uses diagonal-relative LM damping, an inclusive `it <= vio_max_iterations` bound, infinity-norm step convergence, and the `lambda_vee` rejection schedule. The Rust LM core now mirrors these rules.
4. Upstream converts calibration bias standard deviations to square-root weights by inversion before applying the `1/sqrt(dt)` discrete random-walk scaling. Rust previously passed the standard deviations themselves; the adapter now passes inverse RMS weights.
5. A zero reduced camera step does not imply a zero landmark step. Convergence is now checked after the coupled landmark trial, matching upstream order.

Regression coverage includes the upstream pose increment, inverse bias weight, coupled landmark update, anchored visual Jacobian, LM rejection schedule, initialization gate, and first-ten-frame window schedule.

## Frame-4 comparison

| Quantity | Upstream | Rust before | Rust after |
|---|---:|---:|---:|
| LM accepted/rejected | 8 iterations / 0 rejected | 0 / 7 | 5 / 3 |
| position x | -0.001889 | 0.007982 | -0.003518 |
| position y | -0.004031 | -0.002140 | -0.004023 |
| position z | -0.087510 | -0.026876 | -0.087853 |
| velocity x | 0.008004 | 0.081865 | 0.016670 |
| velocity y | -0.028689 | -0.017081 | -0.028955 |
| velocity z | -0.540987 | -0.226508 | -0.536547 |
| gyro bias y | -0.006950 | 0 | -0.007243 |
| accel bias z | -0.042013 | 0 | -0.015117 |
| landmarks | 58 | 94 | 63 |

The post-fix frame-4 MargData contains two navigation states, matching the upstream navigation-state count. Upstream additionally retains one pose-only keyframe; the Rust window still has no equivalent pose-only block.

## Remaining structural divergence

At frame 19 Rust remains close but is not equal (Rust position `[0.01349,-0.01760,-0.27248]`; upstream `[0.00912,-0.02018,-0.24733]`). By frame 79 Rust has drifted to `[0.82330,0.10603,-0.33040]`, while upstream is `[-0.00749,0.00961,-0.14966]`. The upstream window has two navigation states plus seven pose-only keyframes at frame 79; Rust keeps only three 15-DoF navigation blocks and discards the visual keyframe pose blocks. This is now the dominant proven structural gap, rather than the original frame-4 LM rejection cliff.

The 80-frame diagnostic evaluation reports SE(3) ATE RMSE 0.23914 m, Sim(3) ATE RMSE 0.11750 m, scale 0.25458, translation RPE 0.01622 m, and rotation RPE 0.02762 degrees (58/80 associated). These are progress measurements, not a parity pass.

## Verification

`cargo test -p visloc-basalt --all-targets` passes: 89 library tests and all integration/example targets. No optical-flow code was changed in this task.
