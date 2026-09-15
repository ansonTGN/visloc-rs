# M7ax same-`TimeCamId` identity closure

Date: 2026-08-22 JST  
Scope: Basalt commit `0f3b2b52...`, absolute visual linearization boundary

## Contract

The absolute landmark factor has two independent upstream predicates:

1. Equal frame/timestamp suppresses both absolute host and target pose
   Jacobian blocks.
2. Equal frame **and** camera (`host TimeCamId == target TimeCamId`) selects
   an exact identity `T_t_h`.

The second predicate must not be inferred from timestamp equality. A
same-timestamp stereo observation therefore retains the target-camera-from-host
extrinsic transform while its absolute pose Jacobians remain exactly zero.

The Rust implementation keeps the existing public factor wrappers and adds
crate-private factor entry points carrying `same_timestamp` and
`same_time_cam_id` separately. Both the f64 and upstream-f32 paths use an
explicit identity pose for exact `TimeCamId` equality. `window.rs` derives the
predicates from frame IDs (`block_frame_id`) and camera IDs in both the solver
and detail-audit paths. No track-specific branch or track ID hack is used.

## Exact fixture and focused tests

The general fixture
`vio::aom::tests::same_time_cam_id_has_exact_identity_and_stereo_keeps_extrinsic`
uses nontrivial state and calibration poses to verify that an exact same-camera
factor equals an all-identity chain. It also compares same-timestamp stereo with
a temporal evaluation using equal poses, proving that the extrinsic transform
is retained while pose Jacobians alone become zero.

Release checks:

```text
cargo test --release -p visloc-basalt --lib m7_ -- --nocapture
11 passed, 0 failed

cargo test --release -p visloc-basalt --lib same_ -- --nocapture
2 passed, 0 failed
```

The existing M7 fixed-lane tests and the same-timestamp stereo pose-zero test
remain green. Rustfmt checks for `aom.rs` and `window.rs` also pass.

## Fresh five-frame replay

Replay inputs were the pinned MH_01 sensor-only run:

```text
--euroc-dir E:\datasets\euroc_mav\machine_hall\MH_01_easy
--calibration target\euroc_ds_calib.json
--config configs\basalt\euroc_config.json
--max-frames 5
```

Artifacts:

| artifact | path | SHA-256 |
| --- | --- | --- |
| replay summary | `target/m7_same_timecam_exact_fresh5_20260822/summary.txt` | `FA46B3B9531B3A540C2F3EC6FE8F47512244D6C6026A694B2BB2FFA5633DC6FF` |
| detail trace | `target/m7_same_timecam_exact_fresh5_detail_20260822.txt` | `FAC153EEC287966A44130FC037EC136B4B4B55056E84C9029D48995074EB7620` |
| zero-tolerance diff | `target/m7_same_timecam_exact_diff_zero.json` | `DA6BD563017ABCE4A3B8E329954C1651797F64514CA1391386ABDEE6E9E20578` |
| replay trace | `target/m7_same_timecam_exact_fresh5_20260822/trace.jsonl` | `9FBC48263E9349D678AA049ED655B2A3EB6C27407498E6FBA61D554070F95EC4` |

Replay summary: 5 frames, 1114 observations emitted, frame 4 has 252
observations, 70 factors, 1243 rows, and 58 landmarks. All eight frame-4 LM
trials were accepted (`AAAAAAAA`).

The canonical comparator was run with `--abs-tol 0 --rel-tol 0` against
`target/m7al_upstream_f4_20260821T000200Z/iteration.jsonl`:

```text
snapshot_count: 24
structural_difference_count: 0
first_control_flow_difference: null
order_observations: 8
```

Track 1, observation 0 (host frame 0, camera 0) is now closed: its first
previous mismatch, `Jl[0][1]`, is exactly `-66.763641357421875`
(`c28586fc`) on both sides, and its homogeneous landmark lanes are exact
positive zero. The first remaining visual factor mismatch is now:

```text
landmark_factors[track_id=1].observations[1].landmark_jacobian[1][1]
upstream = 1690.6376953125
Rust     = 1690.6375732421875
```

This is the same-timestamp camera-1 stereo row, as intended by the contract.

## Frame-4 H/b, cost, and LM decisions

At iteration 0 / `iteration_start`:

| value | upstream | Rust |
| --- | ---: | ---: |
| `global.H[0][0]` | `479284352` | `479284704` |
| `global.H[0][5]` | `33102.375` | `33111.40234375` |
| `global.b[0]` | `7113.7900390625` | `7113.798828125` |
| cost before | `4215.9326171875` | `4215.8642578125` |

All eight LM decisions remain accepted with no control-flow divergence. At the
last accepted iteration, the reported actual costs are upstream
`248.54075622558594` and Rust `248.476318359375`; the remaining numeric drift
is downstream of the closed same-camera identity row.

The comparator's first overall payload difference is an earlier state
representation difference at frame 0 pose translation x (upstream
`3.500492312014103e-06`, Rust `3.4997722195839742e-06`). The first *visual*
boundary after this closure is the stereo landmark-J lane reported above.

No further production change was made for this report.
