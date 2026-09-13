# M7IM15 native/Rust frame-4 iteration-0 step comparison

The native capture is the direct `get_dense_H_b`/LDLT/applyInc boundary.
Native H is Eigen column-major; Rust H is compared semantically after mapping the same `(row,col)` lanes from its row-major sidecar.
H_copy is intentionally not compared because reconstructing `H + Hdiag_lambda` would add an unrecorded f32 schedule.

- Native: `target/m7im15_native_step_frame4_iter0_20260825.json` (SHA-256 `368e2782ca8fbedec2440e9bbdde9a0dbde2e68239b5354a016dd517082de48a`)
- Native binary: `target/m7im15_native_step_frame4_iter0_20260825.bin` (SHA-256 `2f451a3d77daae5b8e7c44046bd7c3496ada6a76daba28a389fa7fbe5ef46e14`, 48577 bytes)
- Rust: `target/m7im15_rust_blockwise_frame4_iter0_step_20260825.json` (SHA-256 `a14e79712c78ce20c6f6890c0294cff1250b277e4230c09ed63c8ade34e79255`)
- Selection: native frame token `1403636579963555584`, iteration `0`, inner `0`; Rust frame `4`, iteration `0`, trial `0`.

| field | exact | total | mismatch | first difference |
| --- | ---: | ---: | ---: | --- |
| `H` | 4874 | 5625 | 751 | (row=0, col=1; state.pose.translation.x × state.pose.translation.y) 47020e89 vs 47020e8f |
| `b` | 28 | 75 | 47 | lane=1 c5d8736e vs c5d87372 (state.pose.translation.y) |
| `Hdiag_lambda` | 44 | 75 | 31 | lane=21 427be4c9 vs 427be4c7 (state.velocity.x) |
| `inc_post_neg` | 0 | 75 | 75 | lane=0 366aec00 vs 366b1e02 (state.pose.translation.x) |

Lambda: exact=1/1 (no difference).
AOM metadata entries exact by offset/size: 5/5.

## Frontier

The first recorded semantic mismatch is **b** at lane 0; H first differs at (row=0, col=1). Therefore the lane-order frontier is **b**. This cannot establish internal instruction order inside `get_dense_H_b`, which produces H and b together.
