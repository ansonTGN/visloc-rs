# M1: fixed 400-frame Basalt oracle

The fixed upstream source `0f3b2b52c807f70ff4e2973ce253c73329eea7bc` was run on the sensor-only MH_01_easy prefix (400 images per camera plus the full IMU CSV). The engine never received the ground-truth file; evaluation happened after process exit through `benchmarks/basalt/harness.py`.

| metric | upstream Basalt core fallback | Rust M7d | Rust M7e |
|---|---:|---:|---:|
| SE(3) ATE RMSE (m) | 0.00624263 | 0.34717154 | 0.68069429 |
| Sim(3) ATE RMSE (m, diagnostic) | 0.00602965 | 0.17712625 | 0.17599838 |
| consecutive RPE translation RMSE (m) | 0.00126695 | 0.02258629 | 0.02356150 |
| consecutive Sim(3) RPE translation (m, diagnostic) | 0.00124818 | 0.02224104 | 0.02224386 |
| consecutive RPE rotation RMSE (deg) | 0.03158316 | 0.96120458 | 0.23295036 |
| estimate / associated poses | 400 / 378 | 400 / 378 | 400 / 378 |
| association ratio | 0.945 | 0.945 | 0.945 |
| tracked fraction | 1.000 | unavailable in legacy artifact | unavailable in legacy artifact |
| wall time (s) | 31.742450 (harness) | unavailable | unavailable |
| peak RSS (bytes) | 61,689,856 (harness); 398,098,432 Basalt ExecutionStats | unavailable | unavailable |

Evidence paths:

- [paired JSON manifest](m1_400f_comparison.json)
- `target/basalt_upstream_mh01_400f_core_20260821T000010Z/run_manifest.json`
- `target/basalt_upstream_mh01_400f_core_20260821T000010Z/evaluation_result.json`
- `target/basalt_m7d_smoke400/evaluation.json`
- `target/basalt_m7e_run400_reref/eval.json`

The clean Basalt run has 100% engine tracking, no GT-named artifact in the engine workspace, and identical 378-pose association count to both Rust artifacts. The lower Basalt error is therefore a useful M1 behavior anchor, but it is not yet a full-manifest performance claim: the official RealSense2 dependency build remains blocked, and the legacy Rust runs did not capture wall/RSS. The Basalt internal resident-memory statistic is retained because the WSL `psutil` process-tree sample under-reported the child binary; both values are recorded in the JSON and raw engine log.
