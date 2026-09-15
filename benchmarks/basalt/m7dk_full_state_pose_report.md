# M7dk full-state pose sidecar

Date: 2026-08-23 JST  
Status: **Implemented and replayed — quaternion state boundary fixed**

## Change

`WindowProblem.states` and `WindowProblem.poses` already own the propagated
full Sophus-compatible pose objects. The compact AOM vector still carries the
source 15 columns per navigation state (`pose6 + velocity3 + gyro-bias3 +
accel-bias3`). M7dk keeps those dimensions and uses the existing full pose
objects as a sidecar:

- `block_nav`/`block_pose` retain the sidecar quaternion when the compact xyz
  chart identifies the same block, so iteration-start, visual, and IMU factor
  evaluation do not reconstruct `w` from `sqrt(1 - xyz²)`.
- Trial increments are applied to full pose objects first, then flattened back
  to the unchanged compact chart. A trial clone carries its full trial poses;
  rejected trials cannot overwrite the accepted sidecar. Accepted increments
  commit the full pose objects.
- Solved writeback preserves the full quaternion when the solved xyz lanes
  match the sidecar. No extra solver columns, frame/value branch, or trace-only
  substitution was added.

The update follows the pinned `PoseState` local chart: additive world
translation and a left SO(3) increment, with the full quaternion product kept
at the manifold boundary.

## Fresh five-frame replay

Command:

```text
target/release/examples/basalt_euroc_vio_demo.exe --euroc-dir E:/datasets/euroc_mav/machine_hall/MH_01_easy --calibration target/euroc_ds_calib.json --config target/euroc_config.json --out-dir target/m7dk_fresh5_run --max-frames 5
```

The replay processed 5 frames, delivered 42 IMU samples, emitted 1,114
observations, and reached the same frame-4 topology: 70 factors, 1,243 rows,
61 visual factors, 584 visual observations, and 75 state columns.

Against [`target/m7db_clean_frame4_states.json`](../../target/m7db_clean_frame4_states.json),
the iteration-start state comparison is now **73/75 exact**. The frame-2 and
frame-4 quaternion `w` mismatches are gone; the only remaining lanes are the
known frame-4 velocity `y` and `z` endpoint differences (`+2 ULP` each).

| comparison | exact | mismatched |
| --- | ---: | ---: |
| state lanes vs clean native | 73/75 | 2 |
| H lanes vs clean native | 3,194/5,625 | 2,431 |
| b lanes vs clean native | 6/75 | 69 |
| H lanes, M7dk vs M7dj | +20 exact | 2,431 vs 2,451 mismatches |
| b lanes, M7dk vs M7dj | −1 exact | 69 vs 68 mismatches |

The H improvement is measured at the frame-4 iteration-start snapshot. The
single b regression is retained in the artifact; it is not attributed to the
quaternion fix because the remaining b/state discrepancy is the known IMU
velocity endpoint boundary.

Machine-readable comparison: [`target/m7dk_state_hb_comparison.json`](../../target/m7dk_state_hb_comparison.json).  
Fresh detail trace: `target/m7dk_fresh5_detail.jsonl`.

The fresh detail SHA-256 is
`30e5b069f880e019a12f987943fbf481825ca39ff69a608b68b5376bbd266c1b`.

## Verification

```text
cargo test --release -p visloc-basalt --lib
```

Result: **166 passed, 0 failed, 1 ignored**. No commit or push was performed.
