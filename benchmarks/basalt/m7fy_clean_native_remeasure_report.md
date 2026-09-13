# M7fy current Rust-vs-clean Basalt remeasurement

Date: 2026-08-24 JST  
Status: **Complete read-only measurement; no production source or binary changes.**

## Scope and provenance

The current input is `target/m7fx_fresh5_detail.jsonl`, matched by SHA-256 and exact binary32 direction/rho/pixel keys. No prior Rust trace is used as an oracle. Clean references are M7ct H/b, M7db state bits, M7ef per-observation visual Jacobians, and M7fw relative-pose Jacobians.

- Current M7fx SHA-256: `b9e14cea1ab763536e605b6575bc9a4c18ce881bb3bbc10c6128d8bdcb193ac6`
- Snapshot: frame 4, iteration 0, `iteration_start`, 584 observations
- Exact visual-key matches: 584/584 (all keys exact: `True`)

## Clean H/b and state bits

| buffer | exact | mismatches | first mismatch | max abs delta |
|---|---:|---:|---|---:|
| H 75×75 (Eigen column-major clean vs Rust row-major transposed) | 3295/5625 | 2330 | `4de48a64 → 4de48a6f` at H(0,0) | 704.0 |
| b 75×1 | 6/75 | 69 | `45de4e52 → 45de4e9a` at b[0] | 29.619140625 |
| compact state (15 DoF/frame; quaternion xyz, no w) | 74/75 | 1 | {'index': 65, 'expected_bits': 'bcdc2b7f', 'actual_bits': 'bcdc2b7e', 'expected': -0.026876209303736687, 'actual': -0.026876207441091537, 'ulp_delta_actual_minus_expected': 1, 'delta_actual_minus_expected': 1.862645149230957e-09, 'frame_id': 4, 'field': 'translation_xyz', 'lane': 2} | 1.862645149230957e-09 |
| full state sidecar (includes quaternion w; 16 scalar/frame) | 79/80 | 1 | {'index': 70, 'expected_bits': 'bcdc2b7f', 'actual_bits': 'bcdc2b7e', 'expected': -0.026876209303736687, 'actual': -0.026876207441091537, 'ulp_delta_actual_minus_expected': 1, 'delta_actual_minus_expected': 1.862645149230957e-09, 'frame_id': 4, 'field': 'translation_xyz', 'lane': 2} | 1.862645149230957e-09 |

The compact gate has one remaining clean-vs-current mismatch: frame 4 `translation_xyz[2]`, clean `bcdc2b7f` versus Rust `bcdc2b7e` (+1 ULP actual-minus-clean).

## Individual pose blocks (clean M7ef × clean M7fw chain)

Each metric is one 2×6 weighted `jp_anchor` or `jp_target` block. The direct clean-chain row keeps the clean same-timestamp stereo relative Jacobian; the Rust-policy row zeros same-timestamp pose blocks only to expose the source policy separately.

| relation | observations | direct clean-chain exact lanes | Rust-policy exact lanes |
|---|---:|---:|---:|
| `same_timecam_identity` | 61 | 1464/1464 (0 mismatches) | 1464/1464 (0 mismatches) |
| `same_timestamp_stereo` | 61 | 898/1464 (566 mismatches) | 0/1464 (1464 mismatches) |
| `cross_time` | 462 | 7161/11088 (3927 mismatches) | 7161/11088 (3927 mismatches) |
| **overall / 1,168 blocks** | **584** | **9523/14016** (4493 mismatches, 131 exact blocks) | **8625/14016** (5391 mismatches, 129 exact blocks) |

First direct clean-chain mismatch: native ordinal 4, factor 10, track 18, stereo `jp_anchor` lane 1: `c18ae87f → c18ae880` (+1 ULP). First cross-time mismatch is native ordinal 8, factor 10, `jp_anchor` lane 0: `c2713514 → c2713513` (−1 ULP).

The Rust-policy mismatch in same-timestamp stereo is intentionally 1,464/1,464 lanes because current M7fx retains the clean nonzero individual stereo blocks; it is not evidence that the aggregate row is wrong.

## Aggregate pose/state rows

The expected aggregate is formed from clean M7ef×M7fw weighted blocks using the source-order f32 scatter (`old + anchor`, then `result + target`) into each frame's six pose columns. Current `factor.state_jacobian` is compared after f32 casting.

| scope | exact lanes | mismatches | first mismatch |
|---|---:|---:|---|
| full 2×75 state rows, 584 observations (87,600 lanes) | 83673/87600 | 3927 | {'index': 1200, 'expected_bits': 'c2713514', 'actual_bits': 'c2713513', 'expected': -60.30183410644531, 'actual': -60.30183029174805, 'ulp_delta_actual_minus_expected': 1, 'delta_actual_minus_expected': 3.814697265625e-06, 'native_ordinal': 8, 'factor_index': 10, 'track_id': 18, 'observation_order': 2, 'relation': 'cross_time', 'row': 0, 'state_column': 0} |
| active host/target pose support only (12,552 lanes) | 8625/12552 | 3927 | {'index': 96, 'expected_bits': 'c2713514', 'actual_bits': 'c2713513', 'expected': -60.30183410644531, 'actual': -60.30183029174805, 'ulp_delta_actual_minus_expected': 1, 'delta_actual_minus_expected': 3.814697265625e-06, 'native_ordinal': 8, 'factor_index': 10, 'track_id': 18, 'observation_order': 2, 'relation': 'cross_time', 'row': 0, 'state_column': 0} |
| aggregate `same_timecam_identity` | 9150/9150 | 0 | None |
| aggregate `same_timestamp_stereo` | 9150/9150 | 0 | None |
| aggregate `cross_time` | 65373/69300 | 3927 | {'index': 0, 'expected_bits': 'c2713514', 'actual_bits': 'c2713513', 'expected': -60.30183410644531, 'actual': -60.30183029174805, 'ulp_delta_actual_minus_expected': 1, 'delta_actual_minus_expected': 3.814697265625e-06, 'native_ordinal': 8, 'factor_index': 10, 'track_id': 18, 'observation_order': 2, 'relation': 'cross_time', 'row': 0, 'state_column': 0} |

Aggregate identity and same-timestamp stereo rows are exact (9,150/9,150 full-state lanes each); the stereo individual blocks cancel exactly in the same frame. All 3,927 aggregate mismatches are cross-time and share the first clean-chain relative/absolute pose boundary shown above.

## Existing M7fx comparison artifact

`target/m7fx_clean_rel_jacs_all_comparison.json` records the same current M7fx SHA (`b9e14cea1ab763536e605b6575bc9a4c18ce881bb3bbc10c6128d8bdcb193ac6`) and 584/584 keyed matches. Its compact direct clean-chain result is 9523/14016 lanes; its Rust-policy result is 8625/14016. This is Rust-vs-clean; the M7fx report's old M7fv comparison is not used here.

## Narrowest next implementation boundary

The same-timestamp stereo implementation boundary is closed at aggregate scatter: both individual blocks are retained and their aggregate state pose columns are exact zero. The remaining pose discrepancy begins at the cross-time absolute-pose chain/weighted block boundary: clean M7ef relative residual J × clean M7fw relative-pose J versus current Rust `jp_anchor`/`jp_target` differs in 3,927/11,088 cross-time individual lanes and the same 3,927 aggregate lanes. Separately, the only state-input gate visible in the compact state capture is frame-4 translation-z (+1 ULP).

Artifacts: [`m7fy_clean_native_remeasure.py`](m7fy_clean_native_remeasure.py), [`target/m7fy_clean_native_remeasure.json`](../../target/m7fy_clean_native_remeasure.json), and [`target/m7fy_clean_rel_jacs_all_comparison.json`](../../target/m7fy_clean_rel_jacs_all_comparison.json).
