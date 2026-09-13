# M1 upstream IMU golden trace

The fixed upstream Basalt commit
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc` was run with the same staged
sensor/config/calibration contract as the M1 oracle.  The engine received no
ground truth.  The diagnostic hook records only values already computed by
the estimator and is enabled separately with `BASALT_IMU_TRACE_JSONL`.

Evidence directory:
`target/basalt_upstream_mh01_400f_core_imu_trace_20260821T000014Z/`

`imu_trace.jsonl` contains frame 0 through frame 19 (20 records).  Frame 0 is
the initialization frame and has no IMU interval.  Frames 1–19 contain 19
intervals.  Every interval integrates 10 upstream IMU samples.  The measured
camera intervals alternate between `49999872 ns` and `50000128 ns`, matching
the staged EuRoC timestamps.  `imu_trace.csv` is the flattened form and
`imu_trace_summary.json` retains every initial record and the field semantics.

## Recorded fields

For each interval the trace includes the start/end timestamps and sample
count, bias linearization point, preintegrated delta position/rotation/
velocity, raw position/rotation/velocity residual blocks, whitened residual
blocks and cost, gyro/accelerometer bias-walk residuals and costs, and the
predicted state versus final optimized state.  The capture point is after
optimization and before marginalization.

Representative checkpoints:

- Frame 1: `dt=49999872 ns`, 10 samples; delta position
  `[0.009641510434448719, -0.0007075904286466539,
  -0.0033708729315549135]`; whitened cost `3.224906777177239e-07`.
- Frame 4: `dt=50000128 ns`, 10 samples; delta velocity
  `[0.4338512420654297, -0.02186434715986252, -0.15699176490306854]`;
  whitened cost `0.24238096177577972`; bias-walk cost
  `5.748373443914545e-10`.
- Frame 19: `dt=49999872 ns`, 10 samples; delta velocity
  `[0.555938720703125, -0.019419504329562187, -0.2567642033100128]`;
  whitened cost `0.3684764504432678`; bias-walk cost `0`.

Across frames 1–19, mean whitened cost is `0.23423962126939193` and mean
total bias-walk cost is `7.410020650796803e-11`.

## Provenance

The exact diagnostic unified diffs are `imu_instrumentation_header.diff` and
`imu_instrumentation_source.diff` in the evidence directory.  Their SHA-256
hashes are respectively
`7c2ece9ef723f194c328944f214079e2c5707dca5eecf6fb367decadcff50091` and
`af4c4f85f5fab7ac3808ec72ddf73e689fea516a43f56e2f1e29d943df13c602`.
The instrumented header/source hashes are
`dc3f58712c8edd3a9d5c206cadee917225405f79695790cc6dcb51441b1cde7b` and
`261028e29adba9f4c82a3348d76f751b134b1c2b81db7f754cede17592bc57a7`.
The executable hash is
`c7e5c33fd1b76818f6d869fe1e2e90e72b12ee8454144edefff0e42d7d3b424e`; the
runtime `libbasalt.so` hash is
`6bbd360d9ba2ff4317375a32ded52767288ad8cc461cf8175653845ffe911db4`.

