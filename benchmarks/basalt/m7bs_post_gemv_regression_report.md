# m7bs post-GEMV regression audit

**Date:** 2026-08-23  
**Scope:** read-only comparison of the current GEMV endpoint/fresh5 artifacts with the authoritative native fixtures and the m7bn/m7bi endpoint baselines. No source edits or benchmark runs were performed.

## Executive finding

The retained I0 inverse-Jacobian contraction and affine warp contraction are consistent with the native traces cited by m7bm. The production GEMV contraction is not proven: the native vectorized I1 reduction order was unresolved, while the current `aom.rs` path uses a scalar `mul_add` row reduction. The current endpoint regresses aggregate exactness against both baselines (6,306 exact point pairs / 19,752 exact coordinate fields, versus m7bn 6,325 / 19,910 and m7bi 6,350 / 19,873). Thus the proven I0/warp pieces do not explain away the regression; the production GEMV remains the unproven change.

The fresh5 run has the same snapshot structure and all eight LM iterations accepted (`AAAAAAAA`), with final cost 248.47360229492188 versus authoritative 248.54075622558594. That lower scalar cost is not evidence of native parity: initial landmarks, `H`, and `b` are materially non-bit-exact and the later `H`/`b` divergence grows.

## Evidence and provenance

The machine-readable companion is [`target/m7bs_post_gemv_regression_report_20260823.json`](../../target/m7bs_post_gemv_regression_report_20260823.json). The principal input hashes are:

| Artifact | SHA-256 (prefix) |
|---|---:|
| Current endpoint `target/m7_gemv_endpoint80_20260823.jsonl` | `55B8926B…F915C9F4` |
| Native endpoint `benchmarks/basalt/mh01_stereo_endpoint_v1_first80.jsonl` | `CEE7E291…D2205B2C` |
| m7bn endpoint | `8D126D26…51136CD0` |
| m7bi endpoint | `1470DECF…B4DBA036` |
| Current fresh5 detail | `DE4DA4F0…8B33ACC0` |
| Authoritative fresh5 detail | `18E87CFF…119711F7A` |

## Endpoint structure and exactness

All four endpoint artifacts contain the same 80 records, schemas, frame/timestamp fields, per-camera point counts, and track-ID sets/order. The camera totals are 15,347 points for cam0 and 8,909 for cam1 (24,256 points; 48,512 coordinate fields). There are zero structural mismatches between current GEMV and the native fixture, m7bn, or m7bi.

| Camera | Points | Exact pairs | Exact fields | Max ULP (x/y) | Max absolute delta |
|---|---:|---:|---:|---:|---:|
| cam0 | 15,347 | 5,381 (35.0622%) | 14,952/30,694 (48.7131%) | 73 / 145 | 0.0011062622 px |
| cam1 | 8,909 | 925 (10.3828%) | 4,800/17,818 (26.9391%) | 29 / 122 | 0.0008850098 px |
| **total** | **24,256** | **6,306 (25.9977%)** | **19,752/48,512 (40.7157%)** | **145** | **0.0011062622 px** |

The first cam1 mismatch is frame 0, track 3, x: `0x42395ac3` versus native `0x42395ac4` (1 ULP). The first cam0 mismatch is frame 1, track 2, with both x and y one ULP high (`0x422e3d27` vs `0x422e3d26`; `0x42ea1cb8` vs `0x42ea1cb7`). The worst cam0 case is frame 79, track 1755, y: 145 ULP and 0.0011062622 px absolute. The worst cam1 ULP case is frame 45, track 1805, y: 122 ULP.

### ULP distribution against native

The following bins count only mismatched coordinate fields; exact fields are reported separately above.

| Camera/axis | 1 | 2 | 3–4 | 5–8 | 9–16 | 17–32 | 33–64 | 65+ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| cam0 x | 3,738 | 1,588 | 1,280 | 556 | 162 | 23 | 7 | 1 |
| cam0 y | 3,260 | 1,627 | 1,541 | 1,162 | 534 | 218 | 39 | 6 |
| cam1 x | 2,738 | 1,419 | 1,167 | 496 | 237 | 37 | 0 | 0 |
| cam1 y | 2,315 | 1,255 | 1,356 | 1,124 | 627 | 211 | 34 | 2 |

ULP sums (including exact fields in the denominator for the mean) are cam0 x 17,080 / 1.1129, cam0 y 32,088 / 2.0907, cam1 x 15,957 / 1.7911, and cam1 y 29,790 / 3.3439. Signed deviations are mixed rather than a single global bias.

## Effect relative to m7bn and m7bi

### Aggregate exactness

| Comparison | Pair delta (cam0 / cam1 / total) | Field delta (cam0 / cam1 / total) |
|---|---:|---:|
| Current GEMV − m7bn | −66 / +47 / **−19** | −176 / +18 / **−158** |
| Current GEMV − m7bi | −55 / +11 / **−44** | −46 / −75 / **−121** |

The changes are mixed per observation, but losses exceed gains in the aggregate. Versus m7bn, pair transitions are 562 gains versus 581 losses; field transitions are 2,441 gains versus 2,599 losses. Versus m7bi, pair transitions are 611 gains versus 655 losses; field transitions are 2,674 gains versus 2,795 losses.

### Track-level improvements and regressions

For cumulative ULP distance, current versus m7bn has:

- cam0: 231 tracks improved, 3,249 unchanged, 251 regressed;
- cam1: 143 improved, 164 unchanged, 134 regressed.

The largest cam0 improvements are track 384 (−640 ULP), 178 (−401), 1035 (−320), 1309 (−233), and 1108 (−231). The largest cam0 regressions are track 11 (+571), 622 (+316), 119 (+299), 219 (+288), and 74 (+249). On cam1, the largest improvements are tracks 82 (−587), 351 (−536), 91 (−359), 289 (−355), and 109 (−276); the largest regressions are 100 (+408), 242 (+331), 30 (+240), 93 (+225), and 219 (+210).

Versus m7bi, cumulative ULP categories are cam0 261 improved / 3,216 same / 254 regressed and cam1 137 improved / 149 same / 155 regressed. The largest improvements are cam0 tracks 384 (−771), 178 (−442), 1095 (−247), 1035 (−214), 64 (−209), and cam1 tracks 351 (−626), 20 (−488), 1101 (−463), 1548 (−327), 242 (−225). Largest regressions are cam0 tracks 48 (+520), 11 (+453), 128 (+276), 1114 (+255), 119 (+252), and cam1 tracks 100 (+506), 11 (+422), 30 (+386), 315 (+242), 219 (+220).

## Counters and structure

The supplied 80-frame endpoint JSONL has no reject-stage counters, so no current-vs-baseline 80-frame counter delta is asserted. The m7bn taxonomy contains 7,132 total reject-stage events; its change from m7bi is structurally small (for example, `StereoFbSquared +6`, `StereoBackward(TargetOutOfBounds) −6`, and the other listed changes are within ±2). This is baseline context, not a current GEMV measurement.

The current fresh5 frame-4 trace has 68 reject-stage events over 33 rejected track IDs:

`FrameForward(TargetInsufficientOverlap)=8`, `FrameForward(TargetOutOfBounds)=1`, `FrameFbSquared=24`, `ExistingStereoFbSquared=1`, `StereoForward(TargetInsufficientOverlap)=1`, `StereoForward(TargetOutOfBounds)=8`, `StereoBackward(SourcePatchInvalid)=1`, `StereoBackward(TargetInsufficientOverlap)=3`, `StereoBackward(TargetOutOfBounds)=2`, `StereoFbSquared=18`, and `StereoEssentialResidual=1`.

The frame-4 stage multiset matches the nearest m7_ldlt trace apart from the same combined `StereoBackward(TargetOutOfBounds)`/`StereoFbSquared` split (current 2/18 versus 3/17). This does not establish an 80-frame counter result.

## Fresh5 landmarks, H, b, and LM decisions

Current and authoritative detail have the same 24 snapshot keys (8 iterations × `iteration_start`, `trial`, `accepted`). The current detail is f64-backed while the upstream reference is f32-backed, so numeric equality is assessed at serialized f32 fields where applicable.

At the initial snapshot, all 61 landmark host camera/timestamps and the 584-observation topology match. Pixel equality is 216/584 exact pairs and 660/1,168 exact scalar fields: 216 observations have both fields exact, 228 have one exact field, and 140 have neither. Landmark direction lanes are 94/122 exact, rho is 41/61 exact, and the complete direction+rho triple is exact for 39/61 landmarks. Maximum initial direction and rho ULPs are 73 and 559.

Initial `H` is 75×75: 3,175/5,625 f32 entries exact, 2,450 mismatched, maximum absolute difference 960 at `H[34,34]` (543242688 versus 543241728). Initial `b` has 6/75 exact entries, maximum absolute difference 97.59375 at `b[49]` (96959.75 versus 96862.15625). Initial damping diagonal equality is 14/75 with maximum difference 0.09765625. At later iteration starts, maximum absolute `H` differences are 2.09e6–2.80e6 and maximum `b` differences reach 22,486.5195, so the fresh5 trajectory is not native-bit identical despite the same decision sequence.

| Iteration | Current before | Current actual | Authoritative before | Authoritative actual | Decision |
|---:|---:|---:|---:|---:|:---:|
| 0 | 4215.862793 | 413.031281 | 4215.932617 | 412.955017 | A |
| 1 | 413.031281 | 277.490448 | 412.954834 | 277.509705 | A |
| 2 | 277.490448 | 259.712341 | 277.509735 | 259.794006 | A |
| 3 | 259.712341 | 253.044067 | 259.794128 | 253.377335 | A |
| 4 | 253.044067 | 250.474731 | 253.377365 | 251.387939 | A |
| 5 | 250.474731 | 249.304901 | 251.387985 | 249.735352 | A |
| 6 | 249.304901 | 248.754974 | 249.735260 | 248.920502 | A |
| 7 | 248.754974 | **248.473602** | 248.920578 | **248.540756** | A |

Both runs therefore make eight accepted steps with final lambda 1e−6. Current final cost is lower by 0.0671539306640625, but this is an aggregate scalar outcome, not proof that the GEMV reduction order is native-compatible.

## Contraction assessment

The m7bm evidence supports two retained contractions:

1. `patch.rs` uses inverse-Jacobian coefficient order `[1,2,0]`, fusing the second and first products. All 156 level-3 coefficients matched the native trace.
2. `update.rs` spells the affine warp as the Eigen 2×2 first-column product, second-column FMA, then translation. The resulting L3/I0 residual scalars matched native/Rust exactly.

The unresolved item is the production GEMV in `aom.rs`. The m7bm diagnostic balanced 52-term reduction matched L3/I0 but not native vectorized L3/I1: native vectorized increment bits were `bf47a90b, bd615534, be91b7c1`, balanced reduction was `bf47a90b, bd615532, be91b7c1`, and Eigen `DONT_VECTORIZE` was `bf47a90c, bd61553c, be91b7c1`. The remaining native path was identified as fixed-size 3×52-by-52×1 Eigen GEMV with packet size 8; no native order proof was recorded.

The current endpoint's 1-ULP first mismatch and the aggregate pair/field losses against both m7bn and m7bi are consistent with leaving that GEMV order unproven. The correct audit conclusion is therefore: **I0 and warp are supported by native fixtures; production GEMV is an unproven change and remains the regression candidate.**

