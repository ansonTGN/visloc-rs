# M7IM15 native/Rust global normal-system stages (2026-08-25)

This report compares direct binary32 bit lanes from the native four-stage capture with the Rust frontier sidecar. No stage is reconstructed from a later H/b result.

- Native JSON: `target/m7im15_native_global_stages_frame4_iter0_20260825.json` (SHA-256 `e98469ce72144e856897b95ca335b846f54e090f0b96c517aa83284964ffe600`)
- Rust JSONL: `target/m7im15_rust_global_stages_linux_fixed_frame4_iter0_20260825.jsonl` (SHA-256 `b48085e0ff2fcdbd344c4e3a4bba724a0804cf4246d981d3bc86f66f70076796`)
- Selection: frame `4`, iteration `0`, phase `iteration_start`

| comparison | H exact/total | b exact/total | first H mismatch | first b mismatch |
| --- | ---: | ---: | --- | --- |
| stage1_visual | 5625/5625 | 75/75 | exact | exact |
| stage2_visual_plus_imu | 5625/5625 | 75/75 | exact | exact |
| stage3_pose_damping_checkpoint | 5625/5625 | 75/75 | exact | exact |
| stage4_diagnostic_plus_prior | 5625/5625 | 75/75 | exact | exact |
| stage4_production_reduced | 5625/5625 | 75/75 | exact | exact |

| native transition | H exact/total | b exact/total |
| --- | ---: | ---: |
| native_stage2_to_stage3 | 5625/5625 | 75/75 |
| native_stage3_to_stage4 | 5618/5625 | 75/75 |

The production boundary is native stage 4 versus Rust `production_reduced`; the diagnostic `plus_marginal_prior` checkpoint is reported separately because it is the boundary being corrected.
