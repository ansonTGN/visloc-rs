# M7bj combined post-m7bb/m7bd fresh5 evidence

Date: 2026-08-23 JST  
Pinned Basalt revision: `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`  
Comparison: fresh Rust frame-4 iteration detail versus
`target/m7al_upstream_f4_20260821T000200Z/iteration.jsonl`.

## Artifacts and hashes

| artifact | bytes | SHA-256 |
|---|---:|---|
| Rust detail trace, `target/m7_optflow_norm_fresh5_detail_20260823.jsonl` | 71,792,277 | `81e714da7b9482f5f88634b25ce464c06a9b73a03812c4dd275fad379f839e2a` |
| zero-tolerance comparison, `target/m7_optflow_norm_fresh5_diff_zero.json` | 28,881 | `aca048313535caedf59a9bbc7593002ad4ef5a589ff3da227b25ef8c5268b697` |
| pinned upstream detail, `target/m7al_upstream_f4_20260821T000200Z/iteration.jsonl` | 32,541,464 | `18e87cff493f758729f91e8270c9968e55bc8fb33cdcf1c3307fce9119711f7a` |

The Rust detail file contains 25 JSONL records: one header plus eight
`iteration_start`, eight `trial`, and eight `accepted` snapshots.  The frame-4
boundary has 70 factors, 1,243 rows, 61 landmark factors, and a 75 x 75 global
`H` with a 75-element `b`.  Rust frame ID 4 is aligned to upstream timestamp
`1403636579963555584`.

The headers record the intentional scalar boundary: the Rust detail path
reports camera/observation/landmark/pose/state values as `f64` with an
upstream `float32` reference, while the pinned header reports all estimator
numeric types as `float32`.

## Structural and control-flow comparison

The zero-tolerance comparator reports:

```text
schema:                         basalt.vio_iteration_diff.v1
snapshot_count:                 24
structural_difference_count:    0
first_control_flow_difference:  null
```

All eight LM iterations make the same accepted decision on both sides:
`AAAAAAAA`.  Each side therefore follows the same
`iteration_start -> trial -> accepted` control-flow sequence and reaches the
same final accepted iteration.

There are eight `landmark_qr.order` observations where the 61 IDs form the
same set but appear in a different order.  The comparator intentionally
ignores the order-sensitive `global.reduced_rows` and `global.reduced_rhs`
arrays; no ID, factor-count, row-count, or control-flow divergence is present.

The first numeric payload difference is the frame-0 state translation:

```text
path:     blocks[state,1403636579763555584].pose.translation[0]
upstream: 3.500492312014103e-06
Rust:     3.499838840070879e-06
absolute: 6.534719432238489e-10
relative: 1.8668001097475804e-4
```

## Landmark boundary and ownership

The initial frame-4 landmark direction/rho comparison is exact for 39/61
landmarks.  The 22 mismatched IDs are:

```text
3, 11, 12, 23, 27, 29, 30, 38, 40, 47, 49, 58,
63, 75, 92, 101, 102, 108, 111, 119, 120, 131
```

For every one, the earliest differing operand is the frame-0, camera-1
frontend pixel.  Thus all 22 are frontend-owned; camera bearing, relative
pose, DLT `A`, raw JacobiSVD `V`, normalization, and stereographic/rho are
downstream of non-identical pixel operands.  No independent landmark-side
production change is justified by this artifact pair.

The post-m7bd track-1 observation 2 pixel is now exact on both paths.  This is
the target-frame-1, camera-1 observation:

```text
pixel:      [27.31320571899414, 106.39038848876953]
f32 bits:   [0x41da8172, 0x42d4c7e1]
```

Its first remaining factor difference is the raw residual x component:

```text
path:       landmark_factors[track_id=1].observations[2].raw_residual[0]
upstream:   -0.009939193725585938 (0xbc22d800)
Rust:       -0.009918212890625    (0xbc228000)
```

The corresponding y residuals are upstream `1.1353836059570312`
(`0x3f915440`) and Rust `1.1353530883789062` (`0x3f915340`).

## Global reduction and LM evidence

The frame-4 iteration-0 global packet is:

| value | upstream | Rust |
|---|---:|---:|
| `global.H[0][0]` | `479284352` | `479284704` |
| `global.H[0][5]` | `33102.375` | `33112.58984375` |
| `global.b[0]` | `7113.7900390625` | `7113.7978515625` |
| cost before | `4215.9326171875` | `4215.86328125` |

The accepted actual-cost trace is:

| iteration | upstream actual | Rust actual | decision |
|---:|---:|---:|---|
| 0 | `412.95501708984375` | `413.029541015625` | accepted |
| 1 | `277.50970458984375` | `277.4914245605469` | accepted |
| 2 | `259.79400634765625` | `259.7127380371094` | accepted |
| 3 | `253.37733459472656` | `253.04489135742188` | accepted |
| 4 | `251.387939453125` | `250.46665954589844` | accepted |
| 5 | `249.7353515625` | `249.30860900878906` | accepted |
| 6 | `248.92050170898438` | `248.75677490234375` | accepted |
| 7 | `248.54075622558594` | `248.4747772216797` | accepted |

The final accepted actual costs are therefore upstream
`248.54075622558594` and Rust `248.4747772216797`; both finish with the same
accepted decision and the same final lambda lane
`9.999999974752427e-7`.

## Test evidence

Recorded release verification results for this combined post-m7bb/m7bd state:

```text
cargo test -p visloc-basalt --release --lib
158 passed, 0 failed, 1 ignored

cargo test -p visloc-basalt --release --lib m7_ -- --nocapture
16 passed, 0 failed, 0 ignored

m7bd_obs_pixel_exact::mh01_frame1_cam1_track1_pixel_matches_pinned_bits
1 passed, 0 failed
```

The combined evidence closes the track-1 endpoint pixel and preserves exact
control-flow/structure, while the remaining 22 landmark mismatches are
traceably frontend pixel mismatches and the first factor residual remains
numeric rather than a decision or structural divergence.
