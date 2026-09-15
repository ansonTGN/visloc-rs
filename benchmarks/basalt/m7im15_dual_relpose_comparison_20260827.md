# M7IM15 dual relative-pose comparison

Pair: frame0/cam0 -> frame3/cam1, optimization passes 0 and 1. Native inputs and expected values come from `target/m7im15_step_relpose_inputs_gdb_20260827.json` and `target/m7im15_step_relpose_tmp_capture_20260827.log`; the Rust probe is `pipelines/basalt/src/vio/aom.rs::m7im15_relpose_dual_pass_native_drel_and_intermediates`.

| boundary | pass 0 | pass 1 |
| --- | ---: | ---: |
| generic tmp pose | 7/7 exact | 6/7 exact |
| out-of-line candidate tmp pose | 5/7 exact | 5/7 exact |
| generic `d_rel_d_h` | 36/36 exact | 30/36 exact |
| out-of-line candidate `d_rel_d_h` | 23/36 exact | 19/36 exact |

The generic pass-1 temporary differs only at translation y: native `bd83bd75`, Rust `bd83bd76`. That produces six cross-block differences at column-major indices 18, 20, 24, 26, 30, and 32. The candidate path is not a faithful dual-pass replacement because it already changes pass 0 quaternion lanes.

The surrounding state evidence is: iter0 start 80/80 exact, iter1 start 80/80 exact, iter1 frame3 linearized pose 16/16 exact, and iter1 solver boundaries `H=5603/5625`, `b=75/75`, `Hdiag=75/75`, `inc_post_neg=38/75`. Thus this artifact rules out FEJ poseLin as the cause and localizes the remaining shared-pair issue to the final relative-pose translation schedule. No production change is justified by this comparison alone.
