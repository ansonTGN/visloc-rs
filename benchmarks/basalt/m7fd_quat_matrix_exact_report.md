# M7fd quaternion-matrix exactness and clean visual-all comparison

Date: 2026-08-23 JST  
Status: **Implemented and verified.**

## Outcome

All 584 native records match one Rust observation by exact binary32 direction/rho and pixel keys. Projection and raw residual are **579/584 exact pairs (1,163/1,168 lanes)**.

The production relative-pose matrix helper now spells every signed Eigen
cross term as a fused multiply-add (`(-tz).mul_add(q.w, txy)`,
`ty.mul_add(q.w, txz)`, `(-tx).mul_add(q.w, tyz)`, and
`tx.mul_add(q.w, tyz)`). The normalized Sophus homogeneous-point path uses
the same matrix schedule. No value-, factor-, or track-specific branch was
added.

Focused M7 tests pass 17/17, including retained M7dg/M7dx/M7ec and ordinal
0/3/38 gates plus the new ordinal-43 and ordinal-110 fixtures. Full release
library tests pass 173 with one pre-existing ignored diagnostic test.

| input | records/observations | result |
|---|---:|---|
| clean native `m7ef` | 584 | authoritative ordered records |
| fresh Rust `m7fd` snapshot | 584 | iteration 0 / trial 0 / iteration_start |
| unique direction/rho key tuples | 61 | 61/61 exact |
| pixel scalar lanes | 1,168 | 1,168/1,168 exact |

The native ordinal is worker-call order. Rust observations are matched by key and retain their serialized factor/observation ordinal; positional flattening is not assumed.

## Projection/raw relation categories

`pair exact` means both binary32 lanes match; `one` means one lane differs; `both` means both lanes differ.

| relation | observations | projection pair exact / one / both | raw pair exact / one / both |
|---|---:|---:|---:|
| `same_timecam` | 61 | 61 / 0 / 0 (122/122 lanes) | 61 / 0 / 0 (122/122 lanes) |
| `same_timestamp_stereo` | 61 | 61 / 0 / 0 (122/122 lanes) | 61 / 0 / 0 (122/122 lanes) |
| `cross_time` | 462 | 457 / 5 / 0 (919/924 lanes) | 457 / 5 / 0 (919/924 lanes) |

| all observations | 584 | **579 / 5 / 0** (1,163/1,168 lanes) | **579 / 5 / 0** (1,163/1,168 lanes) |

The exact relation totals are 61 same-TimeCam, 61 same-timestamp stereo, and 462 cross-time observations.

## Target frame/camera totals

| target frame | target cam | observations | projection exact pairs | projection exact lanes | raw exact pairs |
|---:|---:|---:|---:|---:|---:|
| 0 | 0 | 61 | 61/61 | 122/122 | 61/61 |
| 0 | 1 | 61 | 61/61 | 122/122 | 61/61 |
| 1 | 0 | 60 | 60/60 | 120/120 | 60/60 |
| 1 | 1 | 61 | 56/61 | 117/122 | 56/61 |
| 2 | 0 | 58 | 58/58 | 116/116 | 58/58 |
| 2 | 1 | 60 | 60/60 | 120/120 | 60/60 |
| 3 | 0 | 57 | 57/57 | 114/114 | 57/57 |
| 3 | 1 | 56 | 56/56 | 112/112 | 56/56 |
| 4 | 0 | 55 | 55/55 | 110/110 | 55/55 |
| 4 | 1 | 55 | 55/55 | 110/110 | 55/55 |

## Weighted/raw landmark Jp

Native `d_res_d_p` is raw, six-lane Eigen column-major. For weighted Jp, native raw lanes are multiplied in binary32 by each Rust observation's exact `sqrt_weight`; Rust `jl` is cast and transposed. For raw Jp, Rust weighted lanes are divided in binary32 by the same exact weight.

| quantity | exact lanes / total | exact six-lane rows | one-lane rows | >=2-lane mismatch rows | first mismatch |
|---|---:|---:|---:|---:|---|
| weighted Jp | 3398/3504 | 525/584 | 20 | 39 | ordinal 13, lane 1 |
| raw Jp | 3168/3504 | 377/584 | 101 | 106 | ordinal 13, lane 1 |

| relation | observations | weighted Jp exact lanes / rows | raw Jp exact lanes / rows |
|---|---:|---:|---:|
| `same_timecam` | 61 | 358/366; 56/61 rows | 358/366; 56/61 rows |
| `same_timestamp_stereo` | 61 | 359/366; 58/61 rows | 359/366; 58/61 rows |
| `cross_time` | 462 | 2681/2772; 411/462 rows | 2451/2772; 263/462 rows |

## First mismatches

The first projection/raw mismatch is native ordinal **311**, track **22**, observation order **3**, target frame/camera **1/1** (`cross_time`). Keys remain exact:

| field | native f32 bits | Rust f32 bits | mask |
|---|---|---|---|
| direction | `be830522,bd01766c` | `be830522,bd01766c` | `11` |
| rho | `3e59788c` | `3e59788c` | `1` |
| pixel | `430891dd,4369ce4d` | `430891dd,4369ce4d` | `11` |
| T_t_h | `3f7ffea4,ba71deac,3bd0c3e6,00000000,3a87685a,3f7ff61a,bc8e2141,00000000,bbd0358b,3c8e2e4e,3f7ff4ce,00000000,bde1e255,baf1ec60,ba884368,3f800000` | unavailable | — |
| target point4 | `bf024930,bd344ff6,3f5f77ea,3e59788c` | unavailable | — |
| projection | `4308bb31,436b0c94` | `4308bb31,436b0c95` | `10` |
| raw residual | `3e255000,3f9f2380` | `3e255000,3f9f2400` | `10` |
| raw d_res_d_p | `4455d042,bef06ec0,c0c3a994,445ed086,c22ecf02,bde2d00e` | Rust weighted jl (see below) | — |
| native raw d_res_d_p * weight | `44bef4d8,bf56bada,c12ebee7,44c6fec7,c29c1f09,be4a90e4` | `44bef4d8,bf56bada,c12ebee3,44c6fec7,c29c1f09,be4a90f2` | `110110` |
| native raw d_res_d_p | `4455d042,bef06ec0,c0c3a994,445ed086,c22ecf02,bde2d00e` | Rust jl / weight `4455d042,bef06ec0,c0c3a990,445ed086,c22ecf02,bde2d01e` | `110110` |

The earliest Jp mismatch is native ordinal **13**, track **66**, observation order **3**, a cross-time target frame 1/cam 1 row whose projection/raw are exact. Its first differing weighted-Jp lane is lane 1: native `c048f92e` versus Rust `c048f928`; raw Jp is `bfd2ca8c` versus `bfd2ca86`.

Rust does not serialize `T_t_h`, transformed target point4, or native relative-pose `d_res_d_xi`; the Rust absolute-pose `jp_anchor`/`jp_target` blocks are not direct substitutes. Therefore those stages are explicitly unavailable rather than treated as mismatches.

## Likely first boundary

The matrix repair closes the previous ordinal-43 same-timestamp Jp frontier and ordinal-110 projection frontier. The remaining first serialized Jp mismatch is ordinal 13, after exact direction/rho/pixel keys and exact projection/raw for that row; the first remaining projection mismatch is ordinal 311. Rust detail does not serialize per-observation `T_t_h` or transformed point, so no unsupported all-factor q/t claim is made.

Supporting exactness: direction/rho/pixel keys are exact; focused M7ec q/t remains 7/7, and the new ordinal-43/110 fixtures assert the general matrix, point, projection, and Jp boundaries. The native target point is not serialized by the runtime detail.

## Aggregate H/b support

The m7fd iteration-start aggregate comparison against clean native H/b gives:

| buffer | exact | mismatch | first mismatch | max absolute delta |
|---|---:|---:|---|---:|
| H | 3259/5625 | 2366 | index 0: `4de48a64` vs `4de48a6f` | 704.0 |
| b | 6/75 | 69 | index 0: `45de4e52` vs `45de4e9c` | 29.701171875 |

H is an aggregate post-linearization buffer and cannot assign its first mismatch to one visual ordinal; its lane-0 divergence is consistent with the early Jp mismatch but is not standalone proof of causality.

## Artifacts and provenance

- Machine-readable comparison: [`target/m7fd_visual_all_comparison.json`](../../target/m7fd_visual_all_comparison.json)
- Native oracle: [`target/m7ef_clean_visual_all.json`](../../target/m7ef_clean_visual_all.json)
- Fresh detail: [`target/m7fd_fresh5_detail.jsonl`](../../target/m7fd_fresh5_detail.jsonl)
- Aggregate H/b support: [`target/m7fd_hb_comparison.json`](../../target/m7fd_hb_comparison.json)

The release library and example were rebuilt; the five-frame sensor-only replay processed 5/5 frames, emitted 1,114 observations, and retained 61 visual factors / 584 observations at frame 4. No commit or push was performed.
