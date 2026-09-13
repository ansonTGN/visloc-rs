# M7IM15 native/Rust frame-4 iteration-0 step comparison

The native capture is the direct `get_dense_H_b`/LDLT/applyInc boundary.
Native H is Eigen column-major; Rust H is compared semantically after mapping the same `(row,col)` lanes from its row-major sidecar.
H_copy is intentionally not compared because reconstructing `H + Hdiag_lambda` would add an unrecorded f32 schedule.

- Native: `target\m7im15_native_step_frame4_iter0_20260825_fresh.json` (SHA-256 `e8bedee5b81ad2632ec7595e2a7453d5bc2ed2ca0ce74cb125b857402b5726cf`)
- Native binary: `target\m7im15_native_step_frame4_iter0_20260825_fresh.bin` (SHA-256 `5733b94601a64ad7db107821acfca9a486a52b5550f549078154911505b2fd17`, 48577 bytes)
- Rust: `target\m7im15_rust_global_stages_hcopy_frame4_iter0_20260825_step.json` (SHA-256 `f04a50c52cf7524354787ed8ebc0e7444cbcf95fddb4875e6d4cb60e3726ef86`)
- Selection: native frame token `1403636579963555584`, iteration `0`, inner `0`; Rust frame `4`, iteration `0`, trial `0`.

| field | exact | total | mismatch | first difference |
| --- | ---: | ---: | ---: | --- |
| `H` | 5625 | 5625 | 0 | none |
| `H_copy` | — | — | — | not compared: Rust H+diag reconstruction would introduce an unrecorded f32-add schedule. |
| `b` | 75 | 75 | 0 | none |
| `Hdiag_lambda` | 75 | 75 | 0 | none |
| `inc_post_neg` | 0 | 75 | 75 | lane=0 366aec00 vs 366b1c89 (state.pose.translation.x) |

Lambda: exact=1/1 (no difference).
AOM metadata entries exact by offset/size: 5/5.

## Frontier

The first recorded solver-boundary mismatch is **inc_post_neg**: `{'lane': 0, 'native_bits': '366aec00', 'rust_bits': '366b1c89', 'frame_field': {'lane': 0, 'aom_ordinal': 0, 'frame_id': 0, 'timestamp_ns': 1403636579763555584, 'field': 'state.pose.translation.x', 'local_lane': 0}}`. direct H_copy/H/b/damping inputs are exact; the first mismatch is the solved increment.
