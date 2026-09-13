# M7IM15 native/Rust frame-4 iteration-0 step comparison

The native capture is the direct `get_dense_H_b`/LDLT/applyInc boundary.
Native H is Eigen column-major; Rust H is compared semantically after mapping the same `(row,col)` lanes from its row-major sidecar.
H_copy is intentionally not compared because reconstructing `H + Hdiag_lambda` would add an unrecorded f32 schedule.

- Native: `target\m7im15_native_step_frame4_iter0_20260825_fresh.json` (SHA-256 `e8bedee5b81ad2632ec7595e2a7453d5bc2ed2ca0ce74cb125b857402b5726cf`)
- Native binary: `target\m7im15_native_step_frame4_iter0_20260825_fresh.bin` (SHA-256 `5733b94601a64ad7db107821acfca9a486a52b5550f549078154911505b2fd17`, 48577 bytes)
- Rust: `target\m7im15_rust_global_stages_ldlt3_frame4_iter0_20260825_step.json` (SHA-256 `5598a737982aa7fb45ed1755dbb1d9b981bf58013cdbbfa8fabbd555813f9110`)
- Selection: native frame token `1403636579963555584`, iteration `0`, inner `0`; Rust frame `4`, iteration `0`, trial `0`.

| field | exact | total | mismatch | first difference |
| --- | ---: | ---: | ---: | --- |
| `H` | 5625 | 5625 | 0 | none |
| `H_copy` | 5625 | 5625 | 0 | none |
| `b` | 75 | 75 | 0 | none |
| `Hdiag_lambda` | 75 | 75 | 0 | none |
| `inc_post_neg` | 15 | 75 | 60 | lane=1 368cce44 vs 368cccb8 (state.pose.translation.y) |

Lambda: exact=1/1 (no difference).
AOM metadata entries exact by offset/size: 5/5.

## Frontier

The first recorded solver-boundary mismatch is **inc_post_neg**: `{'lane': 1, 'native_bits': '368cce44', 'rust_bits': '368cccb8', 'frame_field': {'lane': 1, 'aom_ordinal': 0, 'frame_id': 0, 'timestamp_ns': 1403636579763555584, 'field': 'state.pose.translation.y', 'local_lane': 1}}`. direct H_copy/H/b/damping inputs are exact; the first mismatch is the solved increment.
