# M7IM15 fresh live expression frontier (2026-08-25)

This is a fresh live capture from the detached GCC11 build of pinned Basalt
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.  It is deliberately independent
of the earlier R21 reconstructed-native oracle.  The expression logger was
selected explicitly with `M7IM15_NATIVE_EXPR_TRACE_CALL_ORDINALS=0`, then
`basalt_vio` was run on `MH_01_easy`, one thread, `--max-frames 7`.

The machine-readable result is
[`target/m7im15_live_expr_frontier_20260825.json`](../../target/m7im15_live_expr_frontier_20260825.json).
The freshly parsed native trace is
[`target/m7im15_native_expr_frontier_ordinal0_r2_20260825.json`](../../target/m7im15_native_expr_frontier_ordinal0_r2_20260825.json).
The Rust source-side record is the first JSONL block in
[`target/m7im15_earliest_rust_imu_20260825.jsonl`](../../target/m7im15_earliest_rust_imu_20260825.jsonl).

## Command and provenance

```text
M7IM15_NATIVE_EXPR_TRACE=/mnt/c/Users/rsasa/Workspace/visloc-rs/target/m7im15_native_expr_frontier_ordinal0_r2_20260825.bin
M7IM15_NATIVE_EXPR_TRACE_CALL_ORDINALS=0
M7IM15_NATIVE_INPUT_LOG=/mnt/c/Users/rsasa/Workspace/visloc-rs/target/m7im15_native_expr_frontier_input_ordinal0_20260825.bin
target/m7im15-native-logger-20260825/build/core-relwithdebinfo/basalt_vio \
  --dataset-path /mnt/e/datasets/euroc_mav/machine_hall/MH_01_easy \
  --cam-calib /mnt/c/Users/rsasa/Workspace/visloc-rs/target/euroc_ds_calib.json \
  --dataset-type euroc \
  --config-path /mnt/c/Users/rsasa/Workspace/visloc-rs/target/euroc_config.json \
  --marg-data /tmp/marg_data_m7im15_expr_frontier_ordinal0_r2_20260825 \
  --show-gui 0 --save-trajectory euroc --num-threads 1 --max-frames 7
```

| artifact | SHA-256 |
|---|---|
| fresh expression binary (`...ordinal0_r2...bin`, 375 bytes) | `AB6300E994910CAC53FA3610480B41AAE998EBDAF8B8F5A32F4B1703A74644AE` |
| fresh live input binary (864 bytes) | `41FFE300003316BFE4285029AEDE065EAE4E555D337BB07C781F22D908200A5E` |
| Rust source artifact | `607F423A54A0BF2A145206E1EE48F613100AC11C7E37D3977C8E328E16861E23` |
| detached `basalt_vio` | `438B9AE448F5F977D7305B28106F8C3A0A8870CAB6A92A29E3A31D6E0CFA3E8F` |
| detached diagnostic header | `0C41B88E6980B724E9B49B3D39AB40A651EB577C22A412782D34D1B93C52EDD0` |

The only detached-source change is the env-gated ordinal selector for the
expression trace.  It does not enter the estimator arithmetic.  No production
Rust arithmetic, commit, or push was made.

## Exact comparison

The live native input frontier is exact: state current/FEJ lanes, delta
position/velocity/quaternion, covariance, and both 9x3 bias Jacobians are all
bit exact.  The 21 named expression fields contain 299/300 exact binary32
values.  Every operand before the velocity GEMV is exact:

| expression field | exact |
|---|---:|
| `dt`, translation/velocity/gravity operands | 1/1 or 3/3 |
| bias corrections | 3/3 each |
| `R0_inv` | 9/9 |
| `dR0_translation` | 3/3 |
| `relative_log`, delta position/velocity | 3/3 |
| position and rotation Jacobian blocks | 9/9 each |
| gyro-bias rotation Jacobian block | 9/9 |
| `dR0_velocity` | **2/3** |

The sole mismatch is `dR0_velocity[0]`: native `3ec46fab`, Rust
`3ec46fac` (one ULP).  Since `R0_inv` and `velocity_operand` are both exact,
the first operand/expression frontier is the live Eigen 3x3-by-3 GEMV
evaluation of `R0_inv * velocity_operand`; it is not a state, preintegration,
FEJ, or bias-input mismatch.  The existing raw residual/J mismatch follows
this expression boundary.

## R21 oracle provenance correction

The earlier R21 result used
`target/m7im15_native_expr_trace_luna20260825_operands.json` and a reconstructed
native product oracle.  That reconstruction is **not** evidence for the live
expression schedule and is not used in this report.  The fresh ordinal-0 live
capture above supersedes it: the first proven mismatch is the one-ULP
`dR0_velocity[0]` GEMV result, while all live operands and preintegrated inputs
are exact.  The old reconstructed-oracle claim must therefore be treated as
misidentified provenance rather than a production arithmetic diagnosis.
