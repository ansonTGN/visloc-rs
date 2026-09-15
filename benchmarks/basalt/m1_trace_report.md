# M1 upstream Basalt trace oracle

The fixed upstream Basalt commit `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`
was run on the first 400 frames of `MH_01_easy` with the sensor-only
`basalt-compat` harness.  The engine received cam0/cam1/IMU data only; the
ground-truth firewall reports no GT artifact or path.  The opt-in diagnostic
hook is observational and does not feed any trace value back into the
estimator.

Evidence directory:
`target/basalt_upstream_mh01_400f_core_trace_20260821T000012Z/`

The machine-readable artifacts are `trace.jsonl` (400 records), `trace.csv`,
and `trace_summary.json`.  The latter validates contiguous frame indexes and
contains the complete first 20 records plus complete frame-0/frame-80
checkpoints.  The harness run manifest is `run_manifest.json`.

## Checkpoints

Frame 0 (`t=1403636579763555584 ns`): cam0/cam1 tracks `135/61`, new
`135/61`, lost `0/0`; connected/unconnected cam0 `0/135`; KF `true` (KF count
1); window states/poses `1/0`; landmarks `61` (new `61`, lost `0`); LM
iterations/rejected `0/0`, lambda `9.999999747378752e-05`; marginalization
`false`; pose `t=[0,0,0]`, velocity and gyro/accel biases all zero.

Frame 80 (`t=1403636583763555584 ns`): cam0/cam1 tracks `198/124`, new
`74/6`, lost `77/11`; connected/unconnected cam0 `87/111`, connected
observations `176`; KF `false` (KF count 7); window states/poses `2/7`;
landmarks `110` (new `0`, lost `14`); LM iterations/rejected `8/2`, lambda
`7.999999979801942e-06`; marginalization `true` (states `1`, KFs `0`,
landmarks `14`).

Frame-80 state: pose `t=[-0.006358561106026173, 0.008491757325828075,
-0.18026676774024963]`, quaternion xyzw
`[-0.06533508002758026, -0.8259003162384033, -0.029794717207551003,
0.5592247843742371]`, velocity
`[0.03403424099087715, -0.015221898443996906, -0.5828138589859009]`, gyro
bias `[-0.0012464463943615556, 0.0008329352713190019,
0.002852976555004716]`, accel bias `[-0.06529107689857483,
0.0220854002982378, -0.10419154167175293]`.

The post-run evaluator (GT was read only after engine exit) reports 400/400
tracked frames, 378 associated poses, SE(3) ATE `0.0062422224178479094 m`,
Sim(3) ATE `0.006029150317004683 m`, consecutive RPE translation
`0.0012667335326813167 m`, Sim(3) RPE `0.0012479608963855277 m`, rotation RPE
`0.03150745888522524 deg`, and diagnostic Sim(3) scale
`1.0086376535444246`.  Harness wall time is `26.676643668999986 s` and peak
process-tree RSS is `63148032` bytes.  The full evaluator artifact is
`evaluation_result.json` in the evidence directory.

## Initial 20 states

The columns are `frame`, `cam0/cam1`, `new0/new1`, `lost0/lost1`, `KF?`,
`KF count`, `window states/poses`, `landmarks`, `LM it/rejected`, `marg?`,
`marg states/landmarks`.

| frame | cam0/cam1 | new0/new1 | lost0/lost1 | KF? | KF count | states/poses | landmarks | LM it/rej | marg? | marg states/lm |
|---:|---:|---:|---:|:---:|---:|---:|---:|---:|:---:|---:|
| 0 | 135/61 | 135/61 | 0/0 | yes | 1 | 1/0 | 61 | 0/0 | no | 0/0 |
| 1 | 140/63 | 23/2 | 18/0 | no | 1 | 2/0 | 61 | 0/0 | no | 0/0 |
| 2 | 146/75 | 36/13 | 30/1 | no | 1 | 3/0 | 61 | 0/0 | no | 0/0 |
| 3 | 161/81 | 45/12 | 30/6 | no | 1 | 4/0 | 61 | 0/0 | no | 0/0 |
| 4 | 167/85 | 39/5 | 33/1 | no | 1 | 2/1 | 58 | 8/0 | yes | 3/3 |
| 5 | 178/90 | 37/6 | 26/1 | no | 1 | 2/1 | 58 | 8/0 | yes | 1/0 |
| 6 | 179/89 | 43/5 | 42/6 | no | 1 | 2/1 | 57 | 8/1 | yes | 1/1 |
| 7 | 175/91 | 36/6 | 40/4 | yes | 2 | 2/1 | 139 | 8/3 | yes | 1/3 |
| 8 | 178/90 | 43/0 | 40/1 | no | 2 | 2/1 | 128 | 8/4 | yes | 1/11 |
| 9 | 182/84 | 52/3 | 48/9 | no | 2 | 2/2 | 114 | 7/0 | yes | 1/14 |
| 10 | 182/81 | 55/2 | 55/5 | no | 2 | 2/2 | 107 | 8/0 | yes | 1/7 |
| 11 | 180/70 | 54/0 | 56/11 | no | 2 | 2/2 | 95 | 6/0 | yes | 1/12 |
| 12 | 174/64 | 62/5 | 68/11 | no | 2 | 2/2 | 78 | 7/0 | yes | 1/17 |
| 13 | 177/59 | 66/0 | 63/5 | no | 2 | 2/2 | 73 | 8/0 | yes | 1/5 |
| 14 | 174/53 | 64/0 | 67/6 | yes | 3 | 2/2 | 78 | 8/0 | yes | 1/7 |
| 15 | 171/49 | 73/0 | 76/4 | no | 3 | 2/2 | 70 | 8/2 | yes | 1/8 |
| 16 | 167/48 | 26/1 | 30/2 | no | 3 | 2/3 | 65 | 8/1 | yes | 1/5 |
| 17 | 160/49 | 79/1 | 86/0 | no | 3 | 2/3 | 63 | 8/2 | yes | 1/2 |
| 18 | 156/50 | 82/2 | 86/1 | no | 3 | 2/3 | 60 | 8/3 | yes | 1/3 |
| 19 | 153/57 | 90/8 | 93/1 | no | 3 | 2/3 | 55 | 4/0 | yes | 1/5 |

## Instrumentation provenance

The final instrumented binary is the core-only upstream build; its SHA-256 is
`c7e5c33fd1b76818f6d869fe1e2e90e72b12ee8454144edefff0e42d7d3b424e`.  The
instrumented header/source hashes are respectively
`2f2117a8a87b5aa6829da82d18daf88a2583eb4d9f5c160f45c8d9fa42f4680c` and
`7de198198d78f8b760c9766100ed09702d577a6d280a9f4f012924bcd798a32f`.
The exact unified diffs are `instrumentation_header.diff` and
`instrumentation_source.diff` in the evidence directory (SHA-256
`c1ba1fe96ad8765f2f9649168e308724bdb9473b71349d15ec5e66f5e4f40cbc` and
`692a06690156f746abfbc79ebfc7d044d33cb6797121d4adc31288e8812dce09`).
The instrumented core target rebuilt and completed the 400-frame run.  Its
core CMake profile does not build test executables, so `ctest` in that
instrumented build is `Not Run` (18 registered tests, 0 executables); the
unchanged clean core profile's previously recorded CTest result remains
18/18 passed, and the repository Python parity suite passes 11/11.
