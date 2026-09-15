# M7ba IMU endpoint golden decision

Date: 2026-08-22 JST  
Scope: `vio::estimator::tests::upstream_f32_interval_keeps_integer_endpoint_and_sophus_exp_order`

## Decision

The stale test expectation was updated to the full pinned upstream result:

```text
delta_position.z = -0.0035189196933060884
float bits        = BB669DAD
```

The previous expectation was:

```text
-0.003518919460475445
float bits = BB669DAC
```

The authoritative record is the full
`IntegratedImuMeasurement<float>` diagnostic
`target/basalt_upstream_imu_golden_frame1_2.json`, generated from the pinned
Basalt commit `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.  Its
`integration.delta_state.position` reports `BB669DAD` (the decimal value
above).  The same value is retained in `benchmarks/basalt/upstream_manifest.json`.

## Full oracle versus reduced probe

The reduced standalone probe `target/m7aq_f32_intermediate_run.txt`, built
from `benchmarks/basalt/m7aq_f32_intermediate.cpp`, reports terminal z bits
`BB669DAC`.  That probe hand-writes the propagation expression and is useful
for isolating expression-order/FMA boundaries, but it is not the complete
`IntegratedImuMeasurement<float>` execution (including its full Eigen state
and covariance path).  It therefore does not override the full pinned oracle
for this endpoint golden.

The failing Rust test produced `BB669DAD`, matching the full oracle.  No
special-case correction, threshold, or preintegration implementation change
was added; only the stale expected golden was corrected.  `preintegration.rs`
was not modified.

## Verification

Pinned full-oracle diagnostic build metadata is retained in
`target/basalt_upstream_imu_golden_frame1_2_command.txt`; it uses g++ with
`-O3 -march=native -DEIGEN_DONT_PARALLELIZE` against the pinned headers.

Focused release test:

```text
C:\Users\rsasa\.cargo\bin\cargo.exe test --release -p visloc-basalt --lib upstream_f32_interval_keeps_integer_endpoint_and_sophus_exp_order -- --nocapture
```

Result: `1 passed, 0 failed`.

Full release library test:

```text
C:\Users\rsasa\.cargo\bin\cargo.exe test --release -p visloc-basalt --lib
```

Result: `155 passed, 0 failed, 2 ignored`.

