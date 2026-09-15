# Upstream IMU golden: MH_01 frames 1→2

This is a diagnostic-only record from the pinned Basalt commit
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc` (tree
`b7afb830d82b45b8209cf784ad9744025d838411`). It calls the upstream
`IntegratedImuMeasurement<float>` header directly in the external WSL clone;
the estimator algorithm and Rust crate are not changed.

Artifacts:

- JSON: `target/basalt_upstream_imu_golden_frame1_2.json`
- C++ source: `target/basalt_upstream_imu_golden_frame1_2.cpp`
- optimized frame-state input: `target/basalt_upstream_imu_golden_frame1_2_states.tsv`
- exact build/run command: `target/basalt_upstream_imu_golden_frame1_2_command.txt`
- manifest entry: `benchmarks/basalt/upstream_manifest.json` → `m1_imu_golden_frame1_2`

The camera interval is `1403636579813555456` → `1403636579863555584` ns
(`dt_ns=50000128`). Upstream selection is `t_start < imu_t <= t_end`, giving
10 samples, first `1403636579818555392` ns and last at the end timestamp. Each
sample records raw and calibrated gyro/accelerometer values. The JSON contains
the 9-element delta state, all elements of the 9×9 covariance and square-root
information matrices, both optimized frame states, raw residuals, whitened
residuals, and costs.

Validation: standalone source compiled with WSL g++ 11.4.0 using the upstream
`-O3 -march=native -DEIGEN_DONT_PARALLELIZE` arithmetic flags and ran
successfully; JSON parsing and both 9×9 matrix dimensions were checked. The
per-frame optimized states are taken from the matching upstream IMU trace
(`target/basalt_upstream_mh01_400f_core_imu_trace_20260821T000014Z/imu_trace.jsonl`)
and are explicitly hash-pinned in the manifest. The residual in this compact
record is intentionally recomputed from those two explicit state records; it
does not claim the private frame-1 window state that the estimator may retain
after processing frame 2 when only the per-frame trace is available.
