# M7 ABS_QR f32 Householder implementation

This phase adds a safe Rust f32 implementation of Basalt's landmark
`performQRHouseholder` path.  It is limited to the upstream `ABS_QR` f32
landmark reduction, model-decrease, and landmark back-substitution paths;
the existing f64 paths are unchanged.

## Implementation

`pipelines/basalt/src/vio/aom.rs` now contains `LandmarkHouseholderF32` and
the packet helpers used by its row-major workspace.  The storage contract is
`[Jp | pad | Jl | r]`; for the pinned 15-state frame-4 track it is 15 rows by
80 columns, with state columns 0..74, padding column 75, landmark columns
76..78, residual column 79, and zero damping rows 12..14.  Each reflector uses
`remainingRows = num_rows - k - 3`, Eigen-compatible sign/tiny/signed-zero
handling, an implicit leading one, and the packet GEMV/update order validated
against the native probe.

The helper is used by:

* `reduce_landmark_factors_f32` / `landmark_nullspace_projection_f32`;
* `model_cost_decrease_f32`; and
* `back_substitute_landmark_f32` (full transformed `[Jp | Jl | r]`, then
  `-(Q1r + Q1Jp * pose_inc)` before the upper-triangular solve).

## Bitwise fixture gate

The audit fixture is `target/m7_householder_track1_f4_i0.txt`, generated from
the pinned frame-4 track-1 iteration by `target/m7_householder_fixture.py`.
The native evidence is `target/m7_householder_audit.log` and
`target/m7_householder_probe.cpp`.  The fixture checks all 225 Q1 state lanes,
Q1 landmark/residual lanes, Q2 state/residual rows, stored pivots, tau, pad
zeros, damping rows, and tiny/signed-zero behavior.

Release gate:

```text
cargo test -p visloc-basalt --lib m7_householder --release -- --nocapture
2 passed, 0 failed
```

The prior nalgebra f32 QR probe mismatched 85/225 Q1 state lanes; the helper
matches the pinned Eigen fixture bitwise (0/225 mismatches).

The pinned frame-4 track-1 Q2 extension is `target/m7_track1_q2_fixture.txt`
(12 x 75 Q2 plus rhs).  `target/m7_q2_gemm_probe.cpp` consumes that exact
row-major block and emits complete `J^T J`/`J^T r` bit patterns in
`target/m7_track1_q2_eigen_hb.txt`, using the source
`LandmarkBlockAbsDynamic::add_dense_H_b` expression order.  The Rust exact
gate `m7_q2_one_factor_f32_reduction_is_bitwise_eigen_compatible` reports
0/5625 H mismatches and 0/75 rhs mismatches after wiring the safe
left-to-right f32 fused multiply-add rhs helper into only the f32 reduction;
the f64 path is unchanged.  Before the helper, the H path already matched
0/5625 while the nalgebra f32 rhs path mismatched 13/75 lanes (first lane
`b[1]`: Rust `0xc307cc5c`, Eigen `0xc307cc5d`).

## Regression and replay evidence

The focused release M7 filter passes 8/8 tests.  Mapper tests pass 24/24
runnable tests (one ignored), and the M6 packet contract passes.  The full
release library has 148 passed and one ignored; its sole failure is the
pre-existing estimator test
`upstream_f32_interval_keeps_integer_endpoint_and_sophus_exp_order` (the
known z-lane one-ULP difference), unrelated to ABS_QR.

The fresh production replay is retained under
`target/m7_householder_prod_5f_20260822/`, with the f32 iteration snapshots in
`target/m7_householder_prod_5f_detail_20260822.jsonl`:

| quantity | current Rust ABS_QR | pinned upstream reference |
| --- | ---: | ---: |
| frames / observations | 5 / 1114 | — |
| frame-4 initial cost | 4215.86376953125 | 4215.9326171875 |
| frame-4 final cost | 248.47447204589844 | not recorded |
| frame-4 `H[0,5]` | 33107.18359375 | 33102.3125 |
| LM decisions | `AAAAAAAA` (8 accepted, 0 rejected) | `AAAAAAAA` |

The replay is sensor-only and uses no ground-truth or compatibility input.

Because the Q2 rhs helper changes production f32 accumulation, a fresh
post-change five-frame release replay is retained under
`target/m7_q2_hb_exact_5f/` (detail trace:
`target/m7_q2_hb_exact_5f_detail.jsonl`).  It processed 5 frames / 1114
observations; frame 4 initial cost was `4215.86376953125`, final cost was
`248.474639892578125`, and all eight LM trials were accepted (`AAAAAAAA`).
The replay is sensor-only; its trace schema does not emit a post-reduction
`H[0,5]` lane, so the preceding H-only value remains the applicable
`33107.18359375`.
