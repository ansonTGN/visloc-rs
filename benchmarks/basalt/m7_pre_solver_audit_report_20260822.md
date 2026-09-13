# M7 pre-solver audit (2026-08-22)

This is a read-only audit of the pinned `MH_01_easy` five-frame prefix.  The
Rust replay was run from the current point-action source with the normal
sensor-only demo (`--max-frames 5`).  No ground truth or compatibility input
was supplied.

Evidence used:

- Rust replay: `target/m7_pre_solver_audit_rust_run5_20260822/trace.jsonl`
  (SHA-256 `5BA191EFC688C11A9621FF26151FA2AC4CF6E818C98C4A21F51D15FCB0ED83EA`).
- Fresh Rust frontend endpoint: `target/m7_pre_solver_audit_rust_endpoint5_fresh_20260822.jsonl`
  (SHA-256 `360CF7CC88CCFF49EAC3521DCE55278D5607D249C06BC63B01A95C6D6FF59AA6`).
- Pinned upstream direct IMU trace:
  `target/m7_postm7_upstream_direct5_20260822_imu_trace.jsonl` (SHA-256
  `B158F8AF04856058B63BB0014C646D36339F8A4FFEFF50C281B2EDBEBC7A8309`).
- Pinned upstream zero-bias pre-solver state/link audit:
  `target/m7_postm7_upstream_frame4_states_20260822.tsv` (SHA-256
  `2E77FAACFDED19A947C86FDB0A85ABD6AB84B3379A26EA8B262F79F62BD1568C`).
- Pinned upstream frame-4 detail: `target/m7at_current_upstream_f4.jsonl`;
  current Rust detail: `target/m7_postm7_point_action_fix_detail_20260822.jsonl`.

## First control/state divergence

Each navigation state has 16 float32 lanes: position (3), quaternion (4),
velocity (3), gyro bias (3), and accelerometer bias (3).  Comparing the Rust
`state_trace.predicted_state` against the upstream pre-solver state rows gives:

| frame | exact lanes | first differing lane |
| ---: | ---: | --- |
| 0 | 16/16 | none |
| 1 | 16/16 | none |
| 2 | 16/16 | none |
| 3 | 15/16 | `v_z`: Rust `BE4A5610` (`-0.197593927383423`) vs upstream `BE4A560C` (`-0.19759386777877808`), +4 ULP in the signed bit pattern |
| 4 | 14/16 | `v_y`: `BC8BED08` vs `BC8BED06` (+2 ULP); `v_z`: `BE67F1CC` vs `BE67F1C6` (+6 ULP) |

Thus frame 2 is bit-exact; frame 3 is not bit-exact, although only one
velocity lane differs.  Frame 4 position and quaternion lanes remain exact in
this pre-solver comparison.  The f4 `predicted_state` in the direct upstream
IMU trace is not a valid pre-solver oracle: that diagnostic hook is called
after `optimize()` and recomputes prediction from the updated state.  The
zero-bias link audit/TSV above is the pre-solver f4 source.

There is no frontend/control-flow divergence in the audited prefix.  The
normalized Rust/upstream iteration comparison records
`first_control_flow_difference: null`; all frame-4 LM trials are accepted on
both sides (`AAAAAAAA`, 8 accepted, 0 rejected).

The solver-detail block has one earlier representation-only numeric difference
that must not be confused with the estimator state trace: in the first
iteration-start snapshot, state ordinal 2 `q_w` is Rust `3F17E7A4` versus
upstream `3F17E7A6` (2 ULP).  The state trace itself has frame-2 `q_w`
bit-exact.  This block discrepancy is caused by the Rust detail path
reconstructing a quaternion from its f64 pose matrix.

## Frontend IDs, observations, and lifecycle

The fresh endpoint has exactly the same track-ID set for every camera/frame
pair (zero upstream-only and zero Rust-only IDs):

| frame | cam0 IDs | cam1 IDs | total observations | created | connected / unconnected cam0 | rejected | KF |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | :--- |
| 0 | 135 | 61 | 196 | 135 | 0 / 135 | 0 | yes |
| 1 | 140 | 63 | 203 | 23 | 60 / 80 | 18 | no |
| 2 | 146 | 75 | 221 | 36 | 58 / 88 | 30 | no |
| 3 | 161 | 81 | 242 | 45 | 57 / 104 | 30 | no |
| 4 | 167 | 85 | 252 | 39 | 55 / 112 | 33 | no |

The created, connected/unconnected, keyframe, and lost-landmark decisions
match the pinned upstream trace.  The post-frame window schedule also matches:
`(states, poses, landmarks) = (1,0,61), (2,0,61), (3,0,61),
(4,0,61), (2,1,58)`.

At the frame-4 solver start, the 61 landmark IDs are set-exact.  Their host
camera/frame is exact, and all 584 observation keys (target camera plus target
timestamp) are exact; the observation-count histogram is also identical:
`52` landmarks with 10 observations, `2` with 6, `3` with 7, `1` with 8,
`2` with 9, and `1` with 5.  The numeric observation payload is close but not
bit-exact: 507/1168 pixel float32 lanes differ, with maximum absolute error
`1.8310546875e-4` px and RMSE `2.3890016e-5` px.  Landmark inverse-depth
maximum absolute difference is `1.6391277e-6`.

## Frame-4 solver-input equality

The structural solver input is exact:

| input | Rust | upstream |
| --- | ---: | ---: |
| state blocks / pose blocks | 5 / 0 | 5 / 0 |
| state DoF / reduced H | 75 / 75x75 | 75 / 75x75 |
| landmark IDs | 61 | 61 (same set) |
| landmark observations | 584 | 584 (same topology) |
| visual rows | 1168 | 1168 |
| IMU links / rows | 4 / 36 | 4 / 36 |
| bias factors / rows | 4 / 24 | 4 / 24 |
| prior rows | 15 | 15 |
| total factors / rows | 70 / 1243 | 70 / 1243 |

The Rust first iteration-start snapshot contains 5 state blocks, 0 pose
blocks, and 61 landmarks.  Rust orders the landmark QR IDs ascending while
upstream's order is different; the ID sets are identical and the comparator
normalizes this order-only difference.  The first factor payload difference
is visual, at `track_id=1`, observation 0, residual x: Rust
`-0.008056640625` versus upstream `-0.008089065551757812` (absolute
`3.24249267578125e-5`).  The detailed grouped f32 initial cost is Rust
`4215.86279296875` versus upstream `4215.9326171875` (delta
`-0.06982421875`); this is numeric payload drift, not a lifecycle or solver
decision difference.

## Conclusion

Frontend IDs/observation topology and all control decisions are closed through
frame 4.  The estimator pre-solver state is bit-exact through frame 2; the
first state-trace divergence is frame 3 `v_z` (4 ULP), followed by frame-4
`v_y`/`v_z`.  Frame-4 solver structure and landmark/observation membership
are exact, but float payloads (including the f2 solver-block quaternion
reconstruction and visual factor arithmetic) are not bit-exact.  No claim of
full frame-4 numeric solver-input equality is warranted.
