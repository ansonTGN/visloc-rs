# M7ez clean visual-all native/Rust comparison

Date: 2026-08-23 JST  
Status: **Complete read-only comparison; no production changes.**

## Outcome

All 584 native records match one Rust observation by exact binary32 direction/rho and pixel keys. Projection and raw residual are **577/584 exact pairs (1161/1,168 lanes)**.

| input | records/observations | result |
|---|---:|---|
| clean native `m7ef` | 584 | authoritative ordered records |
| fresh Rust `m7ez` snapshot | 584 | iteration 0 / trial 0 / iteration_start |
| unique direction/rho key tuples | 61 | 61/61 exact |
| pixel scalar lanes | 1,168 | 1,168/1,168 exact |

The native ordinal is worker-call order. Rust observations are matched by key and retain their serialized factor/observation ordinal; positional flattening is not assumed.

## Projection/raw relation categories

`pair exact` means both binary32 lanes match; `one` means one lane differs; `both` means both lanes differ.

| relation | observations | projection pair exact / one / both | raw pair exact / one / both |
|---|---:|---:|---:|
| `same_timecam` | 61 | 61 / 0 / 0 (122/122 lanes) | 61 / 0 / 0 (122/122 lanes) |
| `same_timestamp_stereo` | 61 | 58 / 3 / 0 (119/122 lanes) | 58 / 3 / 0 (119/122 lanes) |
| `cross_time` | 462 | 458 / 4 / 0 (920/924 lanes) | 458 / 4 / 0 (920/924 lanes) |

| all observations | 584 | **577 / 7 / 0** (1,161/1,168 lanes) | **577 / 7 / 0** (1,161/1,168 lanes) |

The exact relation totals are 61 same-TimeCam, 61 same-timestamp stereo, and 462 cross-time observations.

## Target frame/camera totals

| target frame | target cam | observations | projection exact pairs | projection exact lanes | raw exact pairs |
|---:|---:|---:|---:|---:|---:|
| 0 | 0 | 61 | 61/61 | 122/122 | 61/61 |
| 0 | 1 | 61 | 58/61 | 119/122 | 58/61 |
| 1 | 0 | 60 | 60/60 | 120/120 | 60/60 |
| 1 | 1 | 61 | 60/61 | 121/122 | 60/61 |
| 2 | 0 | 58 | 58/58 | 116/116 | 58/58 |
| 2 | 1 | 60 | 60/60 | 120/120 | 60/60 |
| 3 | 0 | 57 | 57/57 | 114/114 | 57/57 |
| 3 | 1 | 56 | 55/56 | 111/112 | 55/56 |
| 4 | 0 | 55 | 55/55 | 110/110 | 55/55 |
| 4 | 1 | 55 | 53/55 | 108/110 | 53/55 |

## Weighted/raw landmark Jp

Native `d_res_d_p` is raw, six-lane Eigen column-major. For weighted Jp, native raw lanes are multiplied in binary32 by each Rust observation's exact `sqrt_weight`; Rust `jl` is cast and transposed. For raw Jp, Rust weighted lanes are divided in binary32 by the same exact weight.

| quantity | exact lanes / total | exact six-lane rows | one-lane rows | >=2-lane mismatch rows | first mismatch |
|---|---:|---:|---:|---:|---|
| weighted Jp | 3370/3504 | 513/584 | 23 | 48 | ordinal 43, lane 1 |
| raw Jp | 3140/3504 | 369/584 | 102 | 113 | ordinal 14, lane 1 |

| relation | observations | weighted Jp exact lanes / rows | raw Jp exact lanes / rows |
|---|---:|---:|---:|
| `same_timecam` | 61 | 358/366; 56/61 rows | 358/366; 56/61 rows |
| `same_timestamp_stereo` | 61 | 341/366; 50/61 rows | 341/366; 50/61 rows |
| `cross_time` | 462 | 2671/2772; 407/462 rows | 2441/2772; 263/462 rows |

## First mismatches

The first projection/raw mismatch is native ordinal **110**, track **21**, observation order **9**, target frame/camera **4/1** (`cross_time`). Keys remain exact:

| field | native f32 bits | Rust f32 bits | mask |
|---|---|---|---|
| direction | `be80b1ef,bd33aa7c` | `be80b1ef,bd33aa7c` | `11` |
| rho | `3e4db2e6` | `3e4db2e6` | `1` |
| pixel | `4308f5bb,434d03fc` | `4308f5bb,434d03fc` | `11` |
| T_t_h | `3f7ffa86,3b3e17c4,3c4e6ae4,00000000,bb462acc,3f7ffc92,3c20171f,00000000,bc4df140,bc20b37c,3f7ff7ab,00000000,bde17018,bce785d8,baa35d96,3f800000` | unavailable | — |
| target point4 | `bf013d67,bdc8bcda,3f5ee20a,3e4db2e6` | unavailable | — |
| projection | `430a717f,43515578` | `430a717f,43515579` | `10` |
| raw residual | `3fbde200,408a2f80` | `3fbde200,408a2fa0` | `10` |
| raw d_res_d_p | `4456726d,c0c5eb70,c10b3b00,445f0855,c22da038,c13ae5c9` | Rust weighted jl (see below) | — |
| native raw d_res_d_p * weight | `4448b737,c0b93f24,c10250bd,4450c063,c2228226,c12eee2a` | `4448b737,c0b93f24,c10250b9,4450c063,c2228226,c12eee2a` | `110111` |
| native raw d_res_d_p | `4456726d,c0c5eb70,c10b3b00,445f0855,c22da038,c13ae5c9` | Rust jl / weight `4456726d,c0c5eb70,c10b3afc,445f0855,c22da038,c13ae5c9` | `110111` |

The earliest Jp mismatch is native ordinal **43**, track **119**, observation order **1**, a same-timestamp stereo row whose projection/raw are exact.

Rust does not serialize `T_t_h`, transformed target point4, or native relative-pose `d_res_d_xi`; the Rust absolute-pose `jp_anchor`/`jp_target` blocks are not direct substitutes. Therefore those stages are explicitly unavailable rather than treated as mismatches.

## Likely first boundary

The earliest localized mismatch is weighted/raw landmark Jp at ordinal 43, track 119 observation 1, a same-timestamp stereo row, with projection/raw exact. The first projection/raw divergence is ordinal 110, track 21 target frame 4/cam 1 cross-time; q/t/target-point cannot be separated there because the Rust detail omits both.

Supporting exactness: direction/rho/pixel keys are exact; focused clean M7ec q/t is 7/7, but q/t is not serialized for all rows. The native target point is present but has no Rust counterpart.

## Aggregate H/b support

The m7ez iteration-start aggregate comparison against clean native H/b gives:

| buffer | exact | mismatch | first mismatch | max absolute delta |
|---|---:|---:|---|---:|
| H | 3253/5625 | 2372 | index 0: `4de48a64` vs `4de48a6f` | 704.0 |
| b | 6/75 | 69 | index 0: `45de4e52` vs `45de4e9e` | 29.654296875 |

H is an aggregate post-linearization buffer and cannot assign its first mismatch to one visual ordinal; its lane-0 divergence is consistent with the early Jp mismatch but is not standalone proof of causality.

## Artifacts and provenance

- Machine-readable comparison: [`target/m7ez_visual_all_comparison.json`](../../target/m7ez_visual_all_comparison.json)
- Native oracle: [`target/m7ef_clean_visual_all.json`](../../target/m7ef_clean_visual_all.json)
- Fresh detail: [`target/m7ez_fresh5_detail.jsonl`](../../target/m7ez_fresh5_detail.jsonl)
- Aggregate H/b support: clean `target/m7ct_clean_frame4_hb.json` versus the fresh snapshot's `global.h`/`global.b`

No build, replay, source edit, commit, or push was performed by this comparison tool.
