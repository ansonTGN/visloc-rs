# M7IM15 native/Rust frame-4 iteration-0 step comparison

The native capture is the direct `get_dense_H_b`/LDLT/applyInc boundary.
Native H is Eigen column-major; Rust H is compared semantically after mapping the same `(row,col)` lanes from its row-major sidecar.
H_copy is intentionally not compared because reconstructing `H + Hdiag_lambda` would add an unrecorded f32 schedule.

- Native: `target\m7im15_native_step_frame4_iter0_20260825.json` (SHA-256 `368e2782ca8fbedec2440e9bbdde9a0dbde2e68239b5354a016dd517082de48a`)
- Native binary: `target\m7im15_native_step_frame4_iter0_20260825.bin` (SHA-256 `2f451a3d77daae5b8e7c44046bd7c3496ada6a76daba28a389fa7fbe5ef46e14`, 48577 bytes)
- Rust: `target\m7im15_rust_frame4_iter0_step_20260825.json` (SHA-256 `7ce3ab8a55a26efcad56c41000a76e401ce67fc686ab1594ea8e487eb26c6b16`)
- Selection: native frame token `1403636579963555584`, iteration `0`, inner `0`; Rust frame `4`, iteration `0`, trial `0`.

| field | exact | total | mismatch | first difference |
| --- | ---: | ---: | ---: | --- |
| `H` | 4435 | 5625 | 1190 | (row=0, col=1; state.pose.translation.x × state.pose.translation.y) 47020e89 vs 47020e8f |
| `b` | 27 | 75 | 48 | lane=0 45de4ef8 vs 45de4e52 (state.pose.translation.x) |
| `Hdiag_lambda` | 43 | 75 | 32 | lane=21 427be4c9 vs 427be4c7 (state.velocity.x) |
| `inc_post_neg` | 0 | 75 | 75 | lane=0 366aec00 vs 366ae3a9 (state.pose.translation.x) |

Lambda: exact=1/1 (no difference).
AOM metadata entries exact by offset/size: 5/5.

## Frontier

The first recorded semantic mismatch is **b** at lane 0; H first differs at (row=0, col=1). Therefore the lane-order frontier is **b**. This cannot establish internal instruction order inside `get_dense_H_b`, which produces H and b together.
