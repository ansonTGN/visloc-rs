# M7fl remove double quaternion normalization

Date: 2026-08-23 JST  
Status: **Retained — all M7 gates green; fresh-five projection/raw exact.**

## Change

`eigen_homogeneous_point_gemv_f32` now passes the stored
`UnitQuaternion<f32>` directly to `eigen_quaternion_matrix_native_f32`.
The redundant `UnitQuaternion::new_normalize(*rotation.quaternion())` at the
matrix boundary was removed.  `UnitQuaternion` is already the normalized
Sophus state; renormalizing its stored f32 coefficients introduced a second
rounding point and changed six rotation lanes in cross-time packets.

The change has no context/value/track branch, debug-only branch, or unsafe
code.  Packet quaternion products remain explicitly normalized at their
existing pinned product boundaries.

## Verification

Focused M7 filter (including M7dg track 1, ordinal 13/311, and M7ec):

```text
cargo test -p visloc-basalt m7 --lib -- --nocapture
27 passed, 0 failed, 149 filtered out
```

Full Basalt library tests:

```text
cargo test -p visloc-basalt --lib
175 passed, 0 failed, 1 ignored

cargo test --release -p visloc-basalt --lib
175 passed, 0 failed, 1 ignored
```

The release example was rebuilt from the current source under WSL and replayed
for five MH01 frames:

```text
cargo build --release -p visloc-rs --example basalt_euroc_vio_demo
VISLOC_BASALT_DETAIL_ITERATIONS=1 \
VISLOC_BASALT_DETAIL_FRAME=4 \
VISLOC_BASALT_DETAIL_TRACE=target/m7fl_fresh5_detail.jsonl \
target/release/examples/basalt_euroc_vio_demo \
  --euroc-dir /mnt/e/datasets/euroc_mav/machine_hall/MH_01_easy \
  --calibration target/euroc_ds_calib.json \
  --config target/euroc_config.json \
  --out-dir target/m7fl_fresh5_run --max-frames 5
```

The run processed 5/5 frames, delivered 42 IMU samples, emitted 1,114
observations, and retained the frame-4 topology of 70 factors / 1,243 rows,
61 visual factors / 584 visual observations, 1,168 visual rows, and 75 state
columns.

## Fresh-five comparison against clean native

The 584 native records were matched by exact binary32 direction/rho/pixel keys
against the iteration-0 frame-4 detail snapshot.  The native clean oracle is
`target/m7ef_clean_visual_all.json`; aggregate H/b is compared with
`target/m7ct_clean_frame4_hb.json`.

| quantity | exact | total |
|---|---:|---:|
| projection pairs | **584** | 584 |
| projection scalar lanes | **1,168** | 1,168 |
| raw residual pairs | **584** | 584 |
| raw residual scalar lanes | **1,168** | 1,168 |
| weighted landmark Jp lanes | 3,423 | 3,504 |
| weighted landmark Jp six-lane rows | 536 | 584 |
| raw landmark Jp lanes | 3,189 | 3,504 |
| raw landmark Jp six-lane rows | 384 | 584 |
| aggregate H lanes | **3,301** | 5,625 |
| aggregate b lanes | 6 | 75 |

The first remaining weighted-Jp mismatch is native ordinal 105, track 9,
same-TimeCam, lane 0.  The first remaining raw-Jp mismatch is ordinal 14,
track 11, cross-time, lane 1.  Projection and raw residual have no remaining
mismatch across all 584 records.  The H comparison's first mismatch remains
native `4de48a64` versus Rust `4de48a6f`; maximum absolute H delta is 704.0.

Against clean frame-4 state capture `target/m7db_clean_frame4_states.json`,
the current WSL release replay is 74/75 compact AOM lanes and 79/80 lanes with
quaternion `w` included.  The sole mismatch is frame-4 translation-z,
`bcdc2b7f` (native) versus `bcdc2b7e` (Rust), one ULP lower.  No debug-only or
unsafe production path was found.

## Artifacts

- [visual comparison](../../target/m7fl_visual_all_comparison.json)
- [H/b comparison](../../target/m7fl_hb_comparison.json)
- [fresh detail trace](../../target/m7fl_fresh5_detail.jsonl)
- release example SHA-256: `168650402c5691dac494fb057a1fbcaee096654a9d01ac954cb5a9c238cafd35`
- fresh detail SHA-256: `a3a92550b1525daeb6e397f7276b05d88443851cf31863aa6550755bd45cc44a`

No commit or push was performed.
