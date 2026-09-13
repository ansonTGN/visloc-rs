# M7ei clean visual-all native/Rust comparison

Date: 2026-08-23 JST  
Status: **Complete read-only comparison; no production changes.**

## Outcome

All 584 native records match one Rust observation by exact binary32 direction/rho and pixel keys. Projection and raw residual are **564/584 exact pairs (1143/1,168 lanes)**.

| input | records/observations | result |
|---|---:|---|
| clean native `m7ef` | 584 | authoritative ordered records |
| fresh Rust `m7ee` snapshot | 584 | iteration 0 / trial 0 / iteration_start |
| unique direction/rho key tuples | 61 | 61/61 exact |
| pixel scalar lanes | 1,168 | 1,168/1,168 exact |

The native ordinal is worker-call order. Rust observations are matched by key and retain their serialized factor/observation ordinal; positional flattening is not assumed.

## Projection/raw relation categories

`pair exact` means both binary32 lanes match; `one` means one lane differs; `both` means both lanes differ.

| relation | observations | projection pair exact / one / both | raw pair exact / one / both |
|---|---:|---:|---:|
| `same_timecam` | 61 | 59 / 1 / 1 (119/122 lanes) | 59 / 1 / 1 (119/122 lanes) |
| `same_timestamp_stereo` | 61 | 57 / 4 / 0 (118/122 lanes) | 57 / 4 / 0 (118/122 lanes) |
| `cross_time` | 462 | 448 / 10 / 4 (906/924 lanes) | 448 / 10 / 4 (906/924 lanes) |

| all observations | 584 | **564 / 15 / 5** (1,143/1,168 lanes) | **564 / 15 / 5** (1,143/1,168 lanes) |

The exact relation totals are 61 same-TimeCam, 61 same-timestamp stereo, and 462 cross-time observations.

## Target frame/camera totals

| target frame | target cam | observations | projection exact pairs | projection exact lanes | raw exact pairs |
|---:|---:|---:|---:|---:|---:|
| 0 | 0 | 61 | 59/61 | 119/122 | 59/61 |
| 0 | 1 | 61 | 57/61 | 118/122 | 57/61 |
| 1 | 0 | 60 | 58/60 | 117/120 | 58/60 |
| 1 | 1 | 61 | 60/61 | 121/122 | 60/61 |
| 2 | 0 | 58 | 56/58 | 113/116 | 56/58 |
| 2 | 1 | 60 | 59/60 | 119/120 | 59/60 |
| 3 | 0 | 57 | 56/57 | 113/114 | 56/57 |
| 3 | 1 | 56 | 55/56 | 111/112 | 55/56 |
| 4 | 0 | 55 | 53/55 | 107/110 | 53/55 |
| 4 | 1 | 55 | 51/55 | 105/110 | 51/55 |

## Weighted/raw landmark Jp

Native `d_res_d_p` is raw, six-lane Eigen column-major. For weighted Jp, native raw lanes are multiplied in binary32 by each Rust observation's exact `sqrt_weight`; Rust `jl` is cast and transposed. For raw Jp, Rust weighted lanes are divided in binary32 by the same exact weight.

| quantity | exact lanes / total | exact six-lane rows | one-lane rows | >=2-lane mismatch rows | first mismatch |
|---|---:|---:|---:|---:|---|
| weighted Jp | 2860/3504 | 364/584 | 29 | 191 | ordinal 0, lane 0 |
| raw Jp | 2656/3504 | 256/584 | 87 | 241 | ordinal 0, lane 0 |

| relation | observations | weighted Jp exact lanes / rows | raw Jp exact lanes / rows |
|---|---:|---:|---:|
| `same_timecam` | 61 | 309/366; 40/61 rows | 309/366; 40/61 rows |
| `same_timestamp_stereo` | 61 | 281/366; 33/61 rows | 281/366; 33/61 rows |
| `cross_time` | 462 | 2270/2772; 291/462 rows | 2066/2772; 183/462 rows |

## First mismatches

The first projection/raw mismatch is native ordinal **38**, track **18**, observation order **9**, target frame/camera **4/1** (`cross_time`). Keys remain exact:

| field | native f32 bits | Rust f32 bits | mask |
|---|---|---|---|
| direction | `be8ebf80,be6ab112` | `be8ebf80,be6ab112` | `11` |
| rho | `3e1d5fdc` | `3e1d5fdc` | `1` |
| pixel | `42f1f732,422fb6ee` | `42f1f732,422fb6ee` | `11` |
| T_t_h | `3f7ffa86,3b3e17c4,3c4e6ae4,00000000,bb462acc,3f7ffc92,3c20171f,00000000,bc4df140,bc20b37c,3f7ff7ab,00000000,bde17018,bce785d8,baa35d96,3f800000` | unavailable | — |
| target point4 | `bf04c770,bed67432,3f425028,3e1d5fdc` | unavailable | — |
| projection | `42f25155,424034ff` | `42f25158,42403505` | `00` |
| raw residual | `3e344600,4083f088` | `3e344c00,4083f0b8` | `00` |
| raw d_res_d_p | `44519eff,c2077378,c20a94f4,4454b36d,c22afb61,c08a28bc` | Rust weighted jl (see below) | — |
| native raw d_res_d_p * weight | `444e5f8d,c2055a37,c2086f48,445167c4,c2285532,c08804bd` | `444e5f8d,c2055a37,c2086f44,445167c7,c2285532,c08804c5` | `110010` |
| native raw d_res_d_p | `44519eff,c2077378,c20a94f4,4454b36d,c22afb61,c08a28bc` | Rust jl / weight `44519eff,c2077378,c20a94f0,4454b370,c22afb61,c08a28c4` | `110010` |

The earliest Jp mismatch is native ordinal **0**, track **66**, observation order **0**, a same-TimeCam identity row whose projection/raw are exact. Its weighted-Jp first lane is `4465f922` raw, native-weighted `44e5f922` versus Rust `44e5f920`.

Rust does not serialize `T_t_h`, transformed target point4, or native relative-pose `d_res_d_xi`; the Rust absolute-pose `jp_anchor`/`jp_target` blocks are not direct substitutes. Therefore those stages are explicitly unavailable rather than treated as mismatches.

## Likely first boundary

The earliest localized mismatch is weighted/raw landmark Jp at ordinal 0, same-TimeCam identity (track 66 observation 0), with projection/raw exact. This points first to the d_res_d_p camera-projection-Jacobian or landmark-J chain, after the input key and projection/residual values, rather than to state or landmark identity. The first projection/raw divergence is later at ordinal 38, track 18 target frame 4/cam 1 cross-time; q/t/target-point cannot be separated there because the Rust detail omits both.

Supporting exactness: direction/rho/pixel keys are exact; m7ee clean-state support is 73/75 compact AOM lanes (only frame-4 velocity y/z differ); focused clean M7ec q/t is 7/7, but q/t is not serialized for all rows. The native target point is present but has no Rust counterpart.

## Aggregate H/b support

The same m7ee iteration-start aggregate comparison against clean native H/b gives:

| buffer | exact | mismatch | first mismatch | max absolute delta |
|---|---:|---:|---|---:|
| H | 3247/5625 | 2378 | index 0: `4de48a64` vs `4de48a6f` | 704.0 |
| b | 6/75 | 69 | index 0: `45de4e52` vs `45de4ea7` | 29.654296875 |

H is an aggregate post-linearization buffer and cannot assign its first mismatch to one visual ordinal; its lane-0 divergence is consistent with the early Jp mismatch but is not standalone proof of causality.

## Artifacts and provenance

- Machine-readable comparison: [`target/m7ei_visual_all_comparison.json`](../../target/m7ei_visual_all_comparison.json)
- Native oracle: [`target/m7ef_clean_visual_all.json`](../../target/m7ef_clean_visual_all.json)
- Fresh detail: [`target/m7ee_fresh5_detail.jsonl`](../../target/m7ee_fresh5_detail.jsonl)
- Aggregate H/b support: [`target/m7ee_hb_comparison.json`](../../target/m7ee_hb_comparison.json)

No build, replay, source edit, commit, or push was performed by this comparison tool.
