# M7 ABS_QR f32 Householder — MH_01_easy 80-frame release regression

Run date: 2026-08-22 JST. This is a fresh sensor-only release replay after
wiring the exact f32 ABS_QR Householder path. The engine received only the
EuRoC image/IMU directory, calibration, and config; ground truth was opened
only by the evaluator after process exit. Prior output directories were not
overwritten.

## Inputs and commands

- Dataset: `E:\datasets\euroc_mav\machine_hall\MH_01_easy`
- Calibration: `target/euroc_ds_calib.json`, SHA-256
  `AD8C5A18C48C55DACF61D18EBBC18CD7D4F3ACCCD5840BD5A5CD8646ADB6271C`
- Config: `target/euroc_config.json` (byte-identical to
  `configs/basalt/euroc_config.json`), SHA-256
  `82937BD6493E592EF89572D31260C10F7437B4FB3FF1FDA179375713966E34FA`
- Release executable: `target/release/examples/basalt_euroc_vio_demo.exe`,
  SHA-256
  `B1F6D466210768BF5C375961561A132BA0EB4120CAFEFCCF1319913C748BFFD5`

Build:

```powershell
& 'C:\Users\rsasa\.cargo\bin\cargo.exe' build --release --example basalt_euroc_vio_demo
```

Engine command (the exact argv is retained in `process.log`):

```powershell
target\release\examples\basalt_euroc_vio_demo.exe --euroc-dir E:\datasets\euroc_mav\machine_hall\MH_01_easy --calibration target\euroc_ds_calib.json --config target\euroc_config.json --out-dir target\m7absqr_mh01_80f_20260822 --max-frames 80
```

Post-exit evaluation (the only command below that names ground truth):

```powershell
python scripts\evaluate_euroc_trajectory.py --ground-truth-csv E:\datasets\euroc_mav\machine_hall\MH_01_easy\mav0\state_groundtruth_estimate0\data.csv --trajectory target\m7absqr_mh01_80f_20260822\trajectory.tum --max-diff-ns 10000000 --tum-time-unit s --out-json target\m7absqr_mh01_80f_20260822\evaluation.json
```

Primary output directory:
`target/m7absqr_mh01_80f_20260822/` containing `summary.txt`,
`process.log`, `trace.jsonl`, `trajectory.csv`, `trajectory.tum`,
`evaluation.json`, and `marg_data/`.

## Metrics

The prior Rust row is the post-M7 FMA 80-frame release replay in
`target/m7_postm7_fma_run80_20260822_114950/`. The pinned upstream row is
the no-`--marg-data` Basalt replay from commit
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc`, retained in
`target/m7perf_final_upstream_80_r1_nomarg/`.

| metric | current exact QR | prior Rust post-M7 FMA | pinned upstream |
|---|---:|---:|---:|
| processed | 80/80 | 80/80 | 80/80 |
| GT-associated coverage | 58/80 = 0.725 | 58/80 = 0.725 | 58/80 = 0.725 |
| SE(3) ATE RMSE (m) | 0.1927160012 | 0.1923934265 | 0.0054666951 |
| Sim(3) ATE RMSE (m, diagnostic) | 0.1126658119 | 0.1126085188 | 0.0053780410 |
| Sim(3) scale (diagnostic) | 0.3344307102 | 0.3351715620 | 1.0071956461 |
| consecutive SE(3) translation RPE (m) | 0.0138569823 | 0.0138348630 | 0.0018914579 |
| consecutive Sim(3) translation RPE (m, diagnostic) | 0.0190348975 | 0.0190154951 | 0.0018720116 |
| consecutive rotation RPE (deg) | 0.0302371152 | 0.0302234275 | 0.0491913838 |

Relative to the prior Rust replay, coverage and processed count are unchanged;
ATE changes by `+0.0003225747 m`, Sim(3) ATE by `+0.0000572931 m`, scale by
`-0.0007408518`, translation RPE by `+0.0000221193 m`, Sim(3) RPE by
`+0.0000194024 m`, and rotation RPE by `+0.0000136877 deg`.

The run loaded 36,820 IMU samples, delivered 792, and emitted 24,256
observations. `sensor_only=true` and `frames_processed=80` are recorded in
`summary.txt`.

## Runtime, memory, and solver control

The current run was monitored with
`scripts.benchmark_process_metrics.run_monitored` at a 0.1 s poll interval:
wall time was `32.1157548 s`, peak process-tree RSS was `76,476,416 B`, and
the process exited 0. The prior post-M7 FMA direct-binary timing was
`32.310038 s` (RSS was not captured for that run). For context, the pinned
upstream no-`--marg-data` 80-frame median was `8.36 s` and `69,066,752 B`
process-tree RSS; the current run is `3.842x` that wall time and `1.107x` that
RSS. The upstream process-tree RSS is distinct from its larger internal
`resident_memory_peak` statistic.

All 76 post-initialization windows (frames 4–79) completed with status
`success`; there were no solver failures, IMU fallbacks, or process warnings.
The current run has 76 LM passes, eight configured iteration slots per pass,
149 accepted and 423 rejected trials. The discrete control signature is
identical to the prior Rust replay: keyframes are
`0,7,14,21,28,35,42,49,56,63,70,77`; observations/track-ID sets, state/pose
counts, marginalization decisions, and every accept/reject decision match.
Only f32 numeric costs and lambda values move. At frame 4, current initial /
final costs are `4215.86376953125 / 248.47447204589844`, versus prior
`4215.86328125 / 248.47415161132812`; both have `AAAAAAAA` control, while the
pinned upstream frame-4 artifact records initial cost `4215.9326171875` and
the same `AAAAAAAA` sequence.

## Artifact hashes

```text
summary.txt    8308D3E93BF63960D166F3B5980C00594DAE147CAB643A07A61D8CBE42B2ABE1
evaluation.json 00A10A63F803ACC7AF92177BD56404D8FCB712F35EFC26A439470A80FB6E9002
trace.jsonl    BC7AAEDD4DBFC8A61053BAEC8A878343F4EF41804359C96A9C2C217838F0932E
trajectory.tum 01DA796F20AA7C4E4CF5964303BC8AD69E824F9AF82A0084B643DC54E90EC33B
trajectory.csv 0CBE4DD357D2CABE165C51F35B914A444023006502C8EB56598A4A708CFFCE3C
process.log    4724EF6FD5822763F33265EC1363F6B73462628DA1C336968A8794C210B8C51A
```

No production source was edited for this regression, and no commit or push was
made.

Focused release gate after the replay: `cargo test -p visloc-basalt --lib
m7_householder --release -- --nocapture` → **2 passed, 0 failed**.
