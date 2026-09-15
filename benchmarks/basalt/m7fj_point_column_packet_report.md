# M7fj homogeneous point column-major Packet4

Date: 2026-08-23 JST  
Status: **Retained — production path exact on all 27 M7 gates.**

## Implementation

The f32 anchored visual factor now calls `eigen_homogeneous_point_gemv_f32`
for its target point and consumes the first three lanes of the returned
homogeneous `4x1` result.  The helper normalizes the accepted
`eigen_quaternion_matrix_native_f32` quaternion matrix, materializes the
source-shaped `4x4` transform, and retains the affine fourth row.

`eigen_homogeneous_point_product_f32` spells Eigen's column-major Packet4
schedule with safe scalar code: four output-row accumulators are initialized
from `col0 * x`, then each accumulator is updated by
`col1.mul_add(y, ...)`, `col2.mul_add(z, ...)`, and
`col3.mul_add(rho, ...)`.  It does not use the horizontal even/odd reduction
used by row-dot kernels.  There are no value-, factor-, ordinal-, debug-, or
unsafe-code branches.

The retained M7fi fixtures still exercise the helper directly; the anchored
factor test and fresh replay exercise the same helper through production.

## Verification

Debug M7 filter (all requested gates, including ordinal 13, 311, 110, M7ec,
and anchored):

```text
cargo test -p visloc-basalt m7 --lib -- --nocapture
27 passed, 0 failed, 0 ignored
```

Full Basalt library:

```text
cargo test -p visloc-basalt --lib
175 passed, 0 failed, 1 ignored
cargo test --release -p visloc-basalt --lib
175 passed, 0 failed, 1 ignored
```

The one ignored test is the pre-existing M8c diagnostic test.

## Fresh-five / 584-row comparison

The release example was rebuilt and replayed with the established frame-4
detail trace:

```text
cargo build --release -p visloc-rs --example basalt_euroc_vio_demo
VISLOC_BASALT_DETAIL_ITERATIONS=1 \
VISLOC_BASALT_DETAIL_FRAME=4 \
VISLOC_BASALT_DETAIL_TRACE=target/m7fj_point_column_packet_fresh5_detail.jsonl \
target/release/examples/basalt_euroc_vio_demo.exe \
  --euroc-dir E:/datasets/euroc_mav/machine_hall/MH_01_easy \
  --calibration target/euroc_ds_calib.json \
  --config target/euroc_config.json \
  --out-dir target/m7fj_point_column_packet_fresh5_run --max-frames 5
```

The replay processed 5/5 frames, delivered 42 IMU samples, emitted 1,114
observations, and retained 61 visual factors / 584 frame-4 observations.
Against clean native `target/m7ef_clean_visual_all.json`, keyed by exact
binary32 direction/rho/pixel inputs:

| quantity | exact | total |
|---|---:|---:|
| projection pairs | 579 | 584 |
| projection scalar lanes | 1,163 | 1,168 |
| raw residual pairs | 579 | 584 |
| weighted landmark Jp lanes | 3,398 | 3,504 |
| weighted landmark Jp six-lane rows | 525 | 584 |
| raw landmark Jp lanes | 3,168 | 3,504 |
| raw landmark Jp six-lane rows | 377 | 584 |
| aggregate H lanes | 3,259 | 5,625 |
| aggregate b lanes | 6 | 75 |

The first projection/raw mismatch remains native ordinal 311; the first
weighted/raw Jp mismatch remains ordinal 13.  The 584-row key match is
complete, and the fresh Packet4 snapshot is semantically identical to the
retained M7fd detail snapshot (same factors, observations, projection/raw/Jp,
and H/b values).

Fresh detail artifact: `target/m7fj_point_column_packet_fresh5_detail.jsonl`
(SHA-256 `f562608aa0815b7e7c5100781ae5be6408005611fef763bd6f8895bd8dfa59a5`).

No commit or push was performed.
