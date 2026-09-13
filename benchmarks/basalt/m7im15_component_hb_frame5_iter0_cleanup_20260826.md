# M7IM15 frame-5 iteration-0 IMU component H/b comparison (2026-08-25)

Pinned Basalt commit: `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.

Component-only mode: the direct native stage binary and selected Rust frame-5/iteration-0 frontier are compared; legacy local-product artifacts are intentionally omitted.

Artifacts:

- JSON: `target/m7im15_component_hb_frame5_iter0_cleanup_20260826.json`
- Script: `work/m7im15_frame4_iter0_component_hb_compare_20260825.py`

## Direct `get_dense_H_b` checkpoints

These comparisons use the native binary stage logger and the Rust solver-frontier record directly; no final H/b reconstruction is used.

| stage | H exact | b exact | first mismatch |
| --- | ---: | ---: | --- |
| visual_accumulator | 2601/2601 | 51/51 | exact |
| visual_plus_imu | 2601/2601 | 51/51 | exact |
| plus_pose_damping | 2601/2601 | 51/51 | exact |
| plus_marginal_prior | 2547/2601 | 35/51 | H lane 67 c4d40aef vs c4d40af0 |

First checkpoint mismatch: `{'stage': 'plus_marginal_prior', 'field': 'H', 'index': 67, 'native': 'c4d40aef', 'rust': 'c4d40af0', 'native_row': 16, 'native_col': 1, 'rust_row': 16, 'rust_col': 1}`.
