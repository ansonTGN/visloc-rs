# M7ee rebuilt fresh-five MH01 measurement

Date: 2026-08-23 JST  
Status: **Pass — release example rebuilt from current sources; no source edits, commit, or push.**

## Rebuild and replay

The requested package/target combination is not a valid Cargo target in this
workspace: `visloc-basalt` exposes the library, while
`basalt_euroc_vio_demo` is an example of the root `visloc-rs` package. Cargo
metadata confirmed this, so the exact valid package-scoped command was run:

```text
cargo build --release -p visloc-rs --example basalt_euroc_vio_demo
```

The executable SHA-256 changed from
`d500fbad70f51037f45d8b1e866e0804716899d2ea8762cf8336b7a2e9c5de64` to
`00c1518312461eb4fe4a737f37f328d201cf0423ae017691d6f6bf1931c16fcd`
(both 1,903,104 bytes). Current build-input hashes, including
`pipelines/basalt/src/vio/aom.rs`, `landmarks.rs`, `window.rs`, and
`estimator.rs`, are recorded in the machine comparison.

The rebuilt binary was replayed with the established detail environment:

```text
VISLOC_BASALT_DETAIL_ITERATIONS=1
VISLOC_BASALT_DETAIL_FRAME=4
VISLOC_BASALT_DETAIL_TRACE=target/m7ee_fresh5_detail.jsonl
target/release/examples/basalt_euroc_vio_demo.exe --euroc-dir E:/datasets/euroc_mav/machine_hall/MH_01_easy --calibration target/euroc_ds_calib.json --config target/euroc_config.json --out-dir target/m7ee_fresh5_run --max-frames 5
```

It processed 5/5 frames, delivered 42 IMU samples, emitted 1,114
observations, and produced 25 detail records. Frame 4 has 70 factors / 1,243
rows, 61 visual factors, 584 observations / 1,168 visual rows, 75 state
columns, 58 runtime landmarks, and 4 IMU links. All eight LM decisions were
accepted (`AAAAAAAA`); iteration-start cost is `4215.93310546875` and final
cost is `248.4748077392578`.

The fresh detail is [target/m7ee_fresh5_detail.jsonl](../../target/m7ee_fresh5_detail.jsonl),
SHA-256 `ebeaf96dcb7a00deefda36520f78a5158ec0c23e641584ad4d55f5e24ae5f5ea`.

## Clean H/b and state boundaries

Against clean native [target/m7ct_clean_frame4_hb.json](../../target/m7ct_clean_frame4_hb.json),
after f32 casting and H transpose:

| buffer | exact | mismatch | stale M7ed / real M7dk exact delta |
|---|---:|---:|---:|
| H (5,625 lanes) | **3,247** | **2,378** | +53 exact (3,194 → 3,247) |
| b (75 lanes) | 6 | 69 | 0 |

Fresh H first mismatch remains native `4de48a64` vs Rust `4de48a6f`; maximum
absolute delta is 704.0. Fresh b first mismatch is native `45de4e52` vs Rust
`45de4ea7`; maximum absolute delta is 29.654296875. The full comparison,
including bit patterns and baseline deltas, is
[target/m7ee_hb_comparison.json](../../target/m7ee_hb_comparison.json).

Against clean native
[target/m7db_clean_frame4_states.json](../../target/m7db_clean_frame4_states.json),
the compact AOM state boundary is 73/75 exact and the full sidecar including
quaternion w is 78/80 exact. Only frame-4 velocity y/z remain (`+2` ULP each:
`bc8bed06 → bc8bed08`, `be67f1c6 → be67f1c8`). These state counts are unchanged
from both real M7dk and stale M7ed.

## M7ec track-2 factor

The focused test passed:

```text
cargo test --release -p visloc-basalt --lib m7ec_clean_track2_stereo_factor_is_bitwise_exact -- --nocapture
1 passed, 0 failed
```

The rebuilt runtime track 2 (same-timestamp frame 0 cam0 → cam1) matches the
serialized M7ec/M7dr values exactly:

- direction `beba3c0f,be195391`, inverse distance `3e42a1b2`, pixel
  `42517d4a,43046ac8`;
- projection `42518143,43046250` and raw residual `3b7e4000,bd078000`;
- after removing runtime `sqrt_weight=2` and transposing, landmark Jp is
  `44486188,c1c7d418,c1ccfe70,44561706,c21c3dda,40c8c0c7` (6/6).

The runtime JSONL does not serialize q/t or the transformed point. The focused
M7ec test verifies those fixture stages exactly (q/t 7/7, point xyz 3/3), along
with projection/raw/Jp. This is the corrected runtime result; stale M7ed and
real M7dk retained the old track-2 projection/raw (`4251812c,4304624e` /
`3b788000,bd07a000`).

For visual aggregate context, fresh vs frozen M7cm is 135/584 exact projection
pairs (566/1,168 scalar lanes), while fresh vs either stale M7ed or real M7dk
is 374/584 pairs (887/1,168 lanes), reflecting the rebuilt stereo bearing
operation order rather than a topology change.

No production source, clean oracle, commit, or push was changed. Existing
unrelated worktree changes were left untouched.
