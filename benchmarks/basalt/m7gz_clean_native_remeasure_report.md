# M7gg current Rust-vs-clean Basalt remeasurement

Date: 2026-08-24 JST  
Status: **Complete read-only measurement; no production source or binary changes.**

## Scope and provenance

The current input is `target/m7gb_fresh5_detail.jsonl`, matched by SHA-256 and exact binary32 direction/rho/pixel keys. No prior Rust trace is used as an oracle. Clean references are M7ct H/b, M7db state bits, M7ef per-observation visual Jacobians, and M7fw relative-pose Jacobians.

- Current M7gb SHA-256: `3875ffc0849b597dce208aca3db47a8ea635740efc659db13d6d99db46c196c3`
- Snapshot: frame 4, iteration 0, `iteration_start`, 584 observations
- Exact visual-key matches: 584/584 (all keys exact: `True`)

## Clean H/b and state bits

| buffer | exact | mismatches | first mismatch | max abs delta |
|---|---:|---:|---|---:|
| H 75Å~75 (Eigen column-major clean vs Rust row-major transposed) | 3306/5625 | 2319 | `4de48a64 Å® 4de48a6f` at H(0,0) | 704.0 |
| b 75Å~1 | 6/75 | 69 | `45de4e52 Å® 45de4e98` at b[0] | 29.619140625 |
| compact state (15 DoF/frame; quaternion xyz, no w) | 75/75 | 0 | None | 0.0 |
| full state sidecar (includes quaternion w; 16 scalar/frame) | 80/80 | 0 | None | 0.0 |

The compact gate has one remaining clean-vs-current mismatch: frame 4 `translation_xyz[2]`, clean `bcdc2b7f` versus Rust `bcdc2b7e` (+1 ULP actual-minus-clean).

## Individual pose blocks (clean M7ef Å~ clean M7fw chain)

Each metric is one 2Å~6 weighted `jp_anchor` or `jp_target` block. The direct clean-chain row keeps the clean same-timestamp stereo relative Jacobian; the Rust-policy row zeros same-timestamp pose blocks only to expose the source policy separately.

| relation | observations | direct clean-chain exact lanes | Rust-policy exact lanes |
|---|---:|---:|---:|
| `same_timecam_identity` | 61 | 1464/1464 (0 mismatches) | 1464/1464 (0 mismatches) |
| `same_timestamp_stereo` | 61 | 1464/1464 (0 mismatches) | 0/1464 (1464 mismatches) |
| `cross_time` | 462 | 11088/11088 (0 mismatches) | 11088/11088 (0 mismatches) |
| **overall / 1,168 blocks** | **584** | **14016/14016** (0 mismatches, 1168 exact blocks) | **12552/14016** (1464 mismatches, 1046 exact blocks) |

First direct clean-chain mismatch: `None`. First cross-time mismatch: `None`.

Rust-policy same-timestamp stereo is reported separately; aggregate scatter is checked independently and remains exact for that relation.

## Aggregate pose/state rows

The expected aggregate is formed from clean M7efÅ~M7fw weighted blocks using the source-order f32 scatter (`old + anchor`, then `result + target`) into each frame's six pose columns. Current `factor.state_jacobian` is compared after f32 casting.

| scope | exact lanes | mismatches | first mismatch |
|---|---:|---:|---|
| full 2Å~75 state rows, 584 observations (87,600 lanes) | 87600/87600 | 0 | None |
| active host/target pose support only (12,552 lanes) | 12552/12552 | 0 | None |
| aggregate `same_timecam_identity` | 9150/9150 | 0 | None |
| aggregate `same_timestamp_stereo` | 9150/9150 | 0 | None |
| aggregate `cross_time` | 69300/69300 | 0 | None |

Aggregate identity and same-timestamp stereo rows are exact (9,150/9,150 full-state lanes each); the stereo individual blocks cancel exactly in the same frame. All 0 aggregate mismatches are cross-time and share the first clean-chain relative/absolute pose boundary shown above.

## Existing M7gb comparison artifact

`target/m7gb_clean_rel_jacs_comparison.json` records the same current M7gb SHA (`1e1f1bb3cc490171947ccb297c0fac28df9b04cfd2be2c5caf446fb48a64197e`) and 584/584 keyed matches. Its compact direct clean-chain result is 13982/14016 lanes; its Rust-policy result is 12518/14016 lanes.

## Narrowest next implementation boundary

The same-timestamp stereo implementation boundary is closed at aggregate scatter: both individual blocks are retained and their aggregate state pose columns are exact zero. The remaining pose discrepancy is 0/14016 cross-time individual lanes and the same 0 aggregate lanes. Separately, the only state-input gate visible in the compact state capture is frame-4 translation-z (+1 ULP).

Artifacts: [`m7fy_clean_native_remeasure.py`](m7fy_clean_native_remeasure.py), [`target/m7gg_clean_native_remeasure.json`](../../target/m7gg_clean_native_remeasure.json), and [`target/m7gb_clean_rel_jacs_comparison.json`](../../target/m7gb_clean_rel_jacs_comparison.json).
