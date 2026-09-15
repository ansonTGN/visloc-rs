# M7fs remaining weighted landmark-J lanes

Date: 2026-08-24 JST  
Status: **Complete read-only comparison; no source edits.**

## Result

`target/m7fq_fresh5_detail.jsonl` (the retained M7fr diagonal-bearing-FMA
replay) matches the clean native M7ef visual oracle by exact binary32
direction/rho/pixel keys for all **584/584** observations.  Recomputing the
weighted landmark point Jacobian (`native d_res_d_p6 * f32(sqrt_weight)` versus
Rust `jl`, transposed to six column-major lanes) gives:

| quantity | exact | total | mismatch rows |
|---|---:|---:|---:|
| weighted Jp lanes | **3,476** | 3,504 | 13 / 584 |
| raw Jp lanes (Rust weighted `jl` divided by the same f32 weight) | 3,240 | 3,504 | 176 / 584 |

The first weighted mismatch is native ordinal **109**, serialized factor
**4**, track **9**, observation **1**, relation `same_timestamp_stereo`, target
frame/camera `0/1`, lane **1 (`r1c0`)**: native `c2874d44`, Rust `c2874d48`,
**4 ULP**.  The first factor is therefore factor 4 / track 9; its second
remaining row is ordinal 125 (observation 5, cross-time).

## Relation partition

| relation | observations | weighted exact lanes / rows | raw exact lanes / rows |
|---|---:|---:|---:|
| `same_timecam` | 61 | 364 / 60 | 364 / 60 |
| `same_timestamp_stereo` | 61 | 362 / 59 | 362 / 59 |
| `cross_time` | 462 | 2,750 / 452 | 2,514 / 289 |

The 28 weighted mismatches have ULP distribution **11×1, 10×2, 4×4,
3×8**.  The complete lane list below is grouped by serialized factor and
relation; `rNcM` is the six-lane column-major lane (row N, column M).

| factor | track | relation | obs | target | lane | ULP | native | Rust |
|---:|---:|---|---:|---|---:|---:|---|---|
| 4 | 9 | `same_timestamp_stereo` | 1 | 0/1 | 1 (`r1c0`) | 4 | `c2874d44` | `c2874d48` |
| 4 | 9 | `same_timestamp_stereo` | 1 | 0/1 | 3 (`r1c1`) | 2 | `44d3fe6c` | `44d3fe6e` |
| 4 | 9 | `cross_time` | 5 | 2/1 | 3 (`r1c1`) | 1 | `449d5e0c` | `449d5e0b` |
| 4 | 9 | `cross_time` | 5 | 2/1 | 5 (`r1c2`) | 2 | `40d23f23` | `40d23f25` |
| 11 | 19 | `same_timestamp_stereo` | 1 | 0/1 | 1 (`r1c0`) | 8 | `c236848c` | `c2368494` |
| 11 | 19 | `same_timestamp_stereo` | 1 | 0/1 | 3 (`r1c1`) | 1 | `44d93308` | `44d93309` |
| 12 | 20 | `cross_time` | 8 | 4/0 | 3 (`r1c1`) | 1 | `4444902c` | `4444902b` |
| 12 | 20 | `cross_time` | 8 | 4/0 | 5 (`r1c2`) | 1 | `c13eed52` | `c13eed51` |
| 32 | 64 | `cross_time` | 3 | 1/1 | 1 (`r1c0`) | 2 | `c0d41cff` | `c0d41d01` |
| 32 | 64 | `cross_time` | 3 | 1/1 | 3 (`r1c1`) | 2 | `44d42262` | `44d42260` |
| 32 | 64 | `cross_time` | 3 | 1/1 | 5 (`r1c2`) | 2 | `bff45cac` | `bff45caa` |
| 34 | 72 | `cross_time` | 4 | 2/0 | 3 (`r1c1`) | 2 | `44d3ca10` | `44d3ca0e` |
| 34 | 72 | `cross_time` | 4 | 2/0 | 5 (`r1c2`) | 2 | `c0e1f122` | `c0e1f120` |
| 38 | 82 | `same_timecam` | 0 | 0/0 | 1 (`r1c0`) | 8 | `41b42d48` | `41b42d40` |
| 38 | 82 | `same_timecam` | 0 | 0/0 | 3 (`r1c1`) | 2 | `44df44d3` | `44df44d1` |
| 41 | 87 | `cross_time` | 8 | 4/0 | 3 (`r1c1`) | 1 | `43b59a84` | `43b59a85` |
| 41 | 87 | `cross_time` | 8 | 4/0 | 5 (`r1c2`) | 1 | `c0a852f7` | `c0a852f8` |
| 46 | 99 | `cross_time` | 8 | 4/0 | 3 (`r1c1`) | 2 | `44b69cfc` | `44b69cfe` |
| 46 | 99 | `cross_time` | 8 | 4/0 | 5 (`r1c2`) | 1 | `c1a7677c` | `c1a7677d` |
| 51 | 108 | `cross_time` | 3 | 1/1 | 1 (`r1c0`) | 4 | `4282565c` | `42825658` |
| 51 | 108 | `cross_time` | 3 | 1/1 | 3 (`r1c1`) | 2 | `44d2026c` | `44d2026a` |
| 51 | 108 | `cross_time` | 6 | 3/0 | 1 (`r1c0`) | 4 | `42c58aa0` | `42c58a9c` |
| 51 | 108 | `cross_time` | 6 | 3/0 | 5 (`r1c2`) | 1 | `c17b5375` | `c17b5374` |
| 53 | 110 | `cross_time` | 9 | 4/1 | 1 (`r1c0`) | 8 | `424fe378` | `424fe380` |
| 53 | 110 | `cross_time` | 9 | 4/1 | 3 (`r1c1`) | 1 | `44c659cc` | `44c659cd` |
| 56 | 118 | `cross_time` | 6 | 3/0 | 1 (`r1c0`) | 4 | `42923e08` | `42923e0c` |
| 56 | 118 | `cross_time` | 6 | 3/0 | 3 (`r1c1`) | 1 | `44d890fa` | `44d890fb` |
| 56 | 118 | `cross_time` | 6 | 3/0 | 5 (`r1c2`) | 1 | `c18492f1` | `c18492f2` |

## Raw-intermediate versus weight diagnosis

Every one of the 28 weighted-Jp mismatches is also a mismatch after dividing
the Rust weighted lane by the exact f32 `sqrt_weight` (**28/28**).  There are
zero weighted-only mismatches; 236 additional raw-only lanes are ordinary
deweighting roundoff where the weighted lane is nevertheless exact.  Thus the
remaining weighted frontier is best localized to the raw/intermediate
landmark-Jp chain (camera-J/Jpp/product/reduction), not to the scalar robust
weight.  This is a localization result, not a claim that the omitted native
intermediate is uniquely identified by the JSONL.

## Reproduction and artifacts

```text
python benchmarks/basalt/m7fs_compare_visual.py
```

The comparator is [m7fs_compare_visual.py](m7fs_compare_visual.py).  Its
machine-readable output is
[target/m7fs_remaining_jp.json](../../target/m7fs_remaining_jp.json), with
input SHA-256 hashes and all 28 lane records.  No build, source edit, commit,
or push was performed.
