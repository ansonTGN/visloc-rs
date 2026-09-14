# M7ed post-stereo H/b measurement

Date: 2026-08-23 JST  
Status: **Pass — read-only M7ec replay and comparison; no production source change**

## Fresh replay

The current release executable was replayed on `MH_01_easy` for five frames
with frame-4 detail enabled using the M7dj/M7dk protocol:

```text
target/release/examples/basalt_euroc_vio_demo.exe --euroc-dir E:/datasets/euroc_mav/machine_hall/MH_01_easy --calibration target/euroc_ds_calib.json --config target/euroc_config.json --out-dir target/m7ed_fresh5_run --max-frames 5
```

The replay processed 5/5 frames, delivered 42 IMU samples, emitted 1,114
observations, and produced 25 detail records (header plus 24 frame-4
snapshots). All eight LM decisions were accepted. Frame 4 retains the M7dk
topology: 70 factors / 1,243 rows, 61 visual factors, 584 observations / 1,168
visual rows, 75 state columns, 58 runtime landmarks, and 4 IMU links.
Iteration-start cost is `4215.93115234375` with lambda
`9.999999747378752e-05`.

The fresh detail is [`target/m7ed_fresh5_detail.jsonl`](../../target/m7ed_fresh5_detail.jsonl),
SHA-256 `5197edd6b782cd2a7e56753212ab74897655bb6cf8747079b4cc7069226b5fe4`.
The full machine comparison is [`target/m7ed_hb_comparison.json`](../../target/m7ed_hb_comparison.json).

## H/b and state boundaries

Against clean native [`target/m7ct_clean_frame4_hb.json`](../../target/m7ct_clean_frame4_hb.json),
after f32 casting and H transpose:

| buffer | exact | mismatch | baseline M7dm/M7dk |
|---|---:|---:|---:|
| H (5,625 lanes) | 3,194 | **2,431** | 2,431 |
| b (75 lanes) | 6 | **69** | 69 |

H first mismatch is native `4de48a64` versus Rust `4de48a6f`; the maximum
absolute delta is 704.0. b first mismatch is native `45de4e52` versus Rust
`45de4e72`; maximum absolute delta is 29.546875.

Against [`target/m7db_clean_frame4_states.json`](../../target/m7db_clean_frame4_states.json),
the compact AOM state boundary is 73/75 exact. The only remaining lanes are
frame-4 velocity y/z, each +2 ULP (`bc8bed06 -> bc8bed08` and
`be67f1c6 -> be67f1c8`). Including quaternion w in the sidecar gives 78/80
exact full-state lanes; no quaternion-w mismatch remains.

## Projection/raw taxonomy

The frozen-detail protocol compares per-observation values after binary32
casting. M7dm reported M7dj versus frozen M7cm at **307/584 exact pairs**.
The M7ec fresh detail is **199/584** versus that same frozen detail (662/1,168
exact scalar lanes) for both projection and raw residual, a delta of −108
pairs. It is 356/584 versus M7dj itself. The relation categories are:

| relation | observations | projection exact pairs | projection lanes | raw exact pairs |
|---|---:|---:|---:|---:|
| same TimeCam identity | 61 | 61/61 | 122/122 | 61/61 |
| same timestamp, stereo cam | 61 | 44/61 | 103/122 | 44/61 |
| cross-time | 462 | 94/462 | 437/924 | 94/462 |

Thus the identity branch remains exact and the M7ec standalone stereo fixture
is independently validated, while the fresh runtime's cross-time aggregate
does not reproduce the prior M7dm 307-pair aggregate. The complete target
frame/camera breakdown and weighted-Jacobian counts are in the JSON artifact;
no unsupported all-factor native claim is made because the clean native H/b
oracle does not serialize every observation's native projection.

## Track fixtures

Track 1 (clean M7dh, frame 0/cam 0 to frame 1/cam 1) remains direct-exact in
the fresh detail: projection `41da6d17,42d70d32` and raw residual
`bc22d800,3f915440`, both 2/2. Track 2's M7ec standalone fixture is confirmed
by the focused test:

```text
cargo test --release -p visloc-basalt --lib m7ec_clean_track2_stereo_factor_is_bitwise_exact -- --nocapture
1 passed, 0 failed
```

That fixture asserts q/t 7/7, target point 3/3, projection 2/2, raw 2/2,
and landmark Jacobian 6/6. In the fresh estimator detail, the matching track-2
observation is the retained runtime value (`4251812c,4304624e` and
`3b788000,bd07a000`) versus clean M7dr's standalone native call
(`42518143,43046250` and `3b7e4000,bd078000`); this is reported separately from
the exact standalone fixture and is not mislabeled as a fresh native match.

No production source, commit, or push was changed. 
