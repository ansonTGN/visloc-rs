# M7IM15 frame-4 iteration-0 IMU component H/b comparison (2026-08-25)

Pinned Basalt commit: `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.

The first four native records (`capture_order=0..3`, all `dense_call=1`) are aligned to the first Rust frame-4/iteration-0 outer record by `start_ns` and `dt_ns`. All four timing keys are exact.

## Local products

| link | start ns | J (15x30) | r (15) | H (30x30) | b (30) |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | 1403636579763555584 | 431/450 exact | 15/15 exact | 754/900 exact | 27/30 exact |
| 1 | 1403636579813555456 | 393/450 exact | 7/15 exact | 549/900 exact | 6/30 exact |
| 2 | 1403636579863555584 | 429/450 exact | 15/15 exact | 749/900 exact | 30/30 exact |
| 3 | 1403636579913555456 | 383/450 exact | 10/15 exact | 532/900 exact | 7/30 exact |

The local products are not bit-exact. The first mismatch per link is recorded in the machine-readable artifact; therefore the discrepancy is already present in the IMU local product boundary and the frontier must not be advanced to visual/prior-only investigation.

## Native local scatter versus Rust IMU accumulator

- H: `4650/5625` exact.
- b: `25/75` exact.
- First H difference: `{'index': 8, 'native': 'bf6ba7c6', 'rust': 'bfcbb184', 'native_row': 8, 'native_col': 0, 'rust_row': 8, 'rust_col': 0}`.
- First b difference: `{'index': 8, 'native': '3c07eabe', 'rust': '3c07eac0', 'native_row': 8, 'native_col': 0, 'rust_row': 8, 'rust_col': 0}`.

The scatter uses column-major local matrices and binary32 addition in the four-link source order, with active offsets `[0,15]`, `[15,30]`, `[30,45]`, `[45,60]`.

## Total solver tie-in

Existing total-step H summary: `4435/5625` exact; b: `27/75` exact.
The total solver artifact is retained as a separate visual+IMU+prior boundary; its first mismatch is not used to erase the component-level IMU finding.

Artifacts:

- JSON: `target/m7im15_frame4_iter0_component_hb_compare_20260825.json`
- Script: `work/m7im15_frame4_iter0_component_hb_compare_20260825.py`
