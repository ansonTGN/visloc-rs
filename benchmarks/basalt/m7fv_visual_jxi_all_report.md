# M7fv all-visual raw relative-pose Jacobian comparison

Date: 2026-08-24 JST  
Status: **Complete read-only capture and exact keyed comparison.**

## Outcome

The current Rust `residual_wrt_relative_pose` field matches the clean native `d_res_d_xi` for every frame-4 iteration-start observation: **7,008 / 7,008 binary32 lanes and 584 / 584 2x6 rows exact**. The observations were matched by exact direction/rho/pixel words, not by serialized order.

| stage | exact lanes | rows exact | total rows | first mismatch |
|---|---:|---:|---:|---|
| projection | 1168/1168 | 584 | 584 | none |
| raw residual | 1168/1168 | 584 | 584 | none |
| weighted landmark Jp (native raw Jp × Rust sqrt weight) | 3504/3504 | 584 | 584 | none |
| raw landmark Jp (Rust weighted Jp ÷ weight) | 3266/3504 | 417 | 584 | {'native_ordinal': 14, 'rust_flat_observation_ordinal': 57, 'factor_index': 6, 'track_id': 11, 'observation_order': 3, 'relation': 'cross_time', 'host': {'frame': 0, 'cam': 0}, 'target': {'frame': 1, 'cam': 1}, 'lane': 1, 'lane_name': 'r1c0', 'native_bits': 'c1948510', 'rust_bits': 'c1948511', 'native': -18.564971923828125, 'rust': -18.564973831176758, 'ulp_delta': 1, 'delta_rust_minus_native': -1.9073486328125e-06} |
| raw `d_res_d_xi` / `residual_wrt_relative_pose` | **7008/7008** | **584** | **584** | none |

The raw-Jp row is a derived deweighting check; production detail serializes weighted Jp. Its mismatch count therefore does not contradict the exact weighted-Jp boundary.

## Relation totals

| relation | observations | projection | raw residual | weighted Jp | raw Jp | d_res_d_xi |
|---|---:|---:|---:|---:|---:|---:|
| `same_timecam` | 61 | 122/122 | 122/122 | 366/366 | 366/366 | **732/732** |
| `same_timestamp_stereo` | 61 | 122/122 | 122/122 | 366/366 | 366/366 | **732/732** |
| `cross_time` | 462 | 924/924 | 924/924 | 2772/2772 | 2534/2772 | **5544/5544** |

The relation counts are 61 same-TimeCam identity observations, 61 same-timestamp stereo observations, and 462 cross-time observations. `d_res_d_xi` is exact in each category: 732/732, 732/732, and 5,544/5,544 lanes respectively.

## First mismatch / boundary assessment

There is no first mismatch in the requested raw relative-pose Jacobian: all 12 lanes of every row match, including signed-zero lanes. Projection and raw residual are also exact, and the native raw camera/point Jacobian multiplied by the exact Rust whitening weight matches the serialized weighted landmark Jp at 3,504/3,504 lanes.

Therefore M7fv localizes no divergence before `d_res_d_xi`. The next not-directly-captured boundary is the relative-pose Jacobian's absolute-pose chain (`relative_wrt_anchor` / `relative_wrt_target`) and its weighted `jp_anchor` / `jp_target` accumulation. The native M7ef record has no per-observation absolute-pose blocks, so this artifact deliberately makes no unsupported exactness claim there.

## Capture and provenance

- Native oracle: [`target/m7ef_clean_visual_all.json`](../../target/m7ef_clean_visual_all.json)
- Fresh Rust detail: [`target/m7fv_fresh5_detail.jsonl`](../../target/m7fv_fresh5_detail.jsonl)
- Comparator: [`m7fv_compare_visual_jxi.py`](m7fv_compare_visual_jxi.py)
- Native SHA-256: `9d8fe21633fd895acf03c26dbb1ba5797c540e03b43acf85850a338d373d85c0`
- Rust detail SHA-256: `7cf89ba9dcb08685509a47d3559c0dbf833cfaf022ca423162de9b41c482eecc`

The temporary detail field and source instrumentation were removed after the single fresh-five capture. Focused M7 verification passed 29/29 tests. No production arithmetic, commit, or push was changed by this diagnostic.
