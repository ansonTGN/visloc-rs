# M7IM15 bias random-walk f32 schedule (2026-08-25)

Pinned Basalt commit: `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.
Fresh Rust release max-5 frame-4/iteration-0 sidecar is compared with direct detached native GCC11 captures. No reconstructed oracle was used.

## Source schedule

`ImuBlock<float>::linearizeImu` computes `dt = dt_ns * Scalar(1e-9)`,
then evaluates `bias_weight_sqrt / std::sqrt(dt)` in `Scalar=float`.
The Rust upstream path now narrows `dt`, state bias lanes, and the stored
weights before the same f32 sqrt/divide and widens only at the public matrix
boundary.  The calibration adapter stores the scalar inverse-RMS weight as
f64; the helper replays Eigen's float `.array().inverse()` reciprocal so the
EuRoC accelerometer weight is `4479ffff` (`999.99994f`) before the dt divide,
while the gyro weight is `461c4000` (`10000f`).

The direct native production `Jp` capture is the oracle for all six diagonal
weight lanes at both alternating intervals: `49999872` gives gyro/accel
`472eb16b`/`458bc122`, and `50000128` gives `472eb14e`/`458bc10a`.

## Expression gates

- Aggregate named expressions: `364/364` exact; first mismatch: `None`.
| block | named expression | raw r | raw J | W | wide W*r | wide W*J | delta matrix | production Jp | production r | H | b |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0 | 91/91 | 9/9 | 270/270 | 81/81 | 9/9 | 270/270 | 8/9 | 450/450 | 15/15 | 900/900 | 30/30 |
| 1 | 91/91 | 9/9 | 270/270 | 81/81 | 9/9 | 270/270 | 6/9 | 450/450 | 15/15 | 900/900 | 30/30 |
| 2 | 91/91 | 9/9 | 270/270 | 81/81 | 9/9 | 270/270 | 6/9 | 450/450 | 15/15 | 900/900 | 30/30 |
| 3 | 91/91 | 9/9 | 270/270 | 81/81 | 9/9 | 270/270 | 6/9 | 450/450 | 15/15 | 900/900 | 30/30 |

The bias fix closes the production local frontier: `Jp = 1800/1800`,
`r = 60/60`, `H = 3600/3600`, and `b = 120/120` exact.  The remaining
delta-rotation-matrix input lanes are an independent preintegration matrix
representation frontier and do not affect the local production gates here.

First post-schedule input/product mismatch: `{'block': 0, 'group': 'input_stages', 'field': 'delta_rotation_matrix', 'difference': {'index': 4, 'native': '3f7ffefc', 'rust': '3f7ffefd'}}`.

## Artifacts

- Comparison: `target/m7im15_bias_fix_all4_compare_20260825.json` (SHA-256 `4deefe3809fc8e60eab98487bf17c5e7b8bb8f0c999a7e5830a2489800ba5c26`)
- Report: `benchmarks/basalt/m7im15_bias_fix_all4_report_20260825.md`
- Rust sidecar: `target/m7im15_rust_bias_fix_all4_20260825.jsonl` (SHA-256 `bbeb9d1aaa7ed1977184c6db5926f87ef47bc2c83bc05bda07b5b89170537714`)
