# M7IM exact f32-field comparison (2026-08-24)

The comparator in `m7im_compare_cov_ldlt.py` now fails closed on the injected
Rust audit schema and records the selected source field for every frame-3 →
frame-4 comparison.  For the `links[3]` schema it selects:

```text
raw_residual  <- raw_residual_f32_exact
jacobian      <- jacobian_f32_exact
```

The widened `raw_residual` and `jacobian` fields remain available in the audit
JSON for diagnostics, but are not valid inputs for this binary32 boundary.

## Run

```text
python benchmarks/basalt/m7im_compare_cov_ldlt.py \
  --oracle target/m7im_cov_ldlt_oracle_20260824.json \
  --rust target/m7im_rust_ldlt_run3.json \
  --rust-packets target/m7hd_step_probe_after_fma.stdout \
  --native-packets target/m7hd_native_intermediate.jsonl \
  --report target/m7im_exact_field_compare_rust_run3_with_packets_20260824.json
```

Machine-readable output: [`target/m7im_exact_field_compare_rust_run3_with_packets_20260824.json`](../../target/m7im_exact_field_compare_rust_run3_with_packets_20260824.json).

## Current counts

The native first-ten covariance trace is exact (`0/810` bit mismatches).  The
historical Rust step trace supplied to this run is `626/810` mismatched and is
retained only as a baseline.  The injected frame-4 link is selected with
`scalar_contract: rust-f32-exact`:

| field | selected Rust field | differing f32 elements |
| --- | --- | ---: |
| covariance | `covariance` | 0/81 |
| square-root information | `sqrt_information` | 0/81 |
| raw residual | `raw_residual_f32_exact` | 6/9 |
| Jacobian | `jacobian_f32_exact` | 51/270 |
| whitened residual | `whitened_residual` | 8/9 |
| whitened Jacobian | `whitened_jacobian` | 86/270 |
| row H | `row_h` | 486/900 |
| row b | `row_b` | 24/30 |

The latest matrix-GEMV-patched sidecar already retained under
`target/m7im_factor_matrixgemvpatched_20260824.json` reports the same exact
field selection and `6/9` raw-residual mismatch; its current Jacobian count is
`45/270` (the sidecar's Rust input is a temporary `/tmp` file and is not
re-runnable from this workspace).  Re-run the comparator against the next
injected evaluator output after the implementation build; do not compare the
widened legacy fields.

