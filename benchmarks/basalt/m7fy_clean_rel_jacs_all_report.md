# M7fw clean relative-pose Jacobian chain

Date: 2026-08-24 JST  
Status: **Complete read-only capture and 584-row comparison.**

## Outcome

The bounded clean GDB capture reached the first five-state frame-4 `linearizeProblem` return and recorded 9 post-write `computeRelPose` calls. They deduplicate to 9 non-identity TimeCam relations; the tenth expected relation is the same-TimeCam identity branch, which does not call `computeRelPose` and supplies zero Jacobians.

| chain used for expected weighted blocks | exact lanes | exact 2x6 blocks | total blocks |
|---|---:|---:|---:|
| direct clean relative-J chain (same-timestamp stereo uses captured clean J) | 9523/14016 | 131 | 1168 |
| Rust same-timestamp-zero policy (cross-time uses captured clean J) | 8625/14016 | 129 | 1168 |

Each metric is a 2x6 block: 12 Eigen column-major binary32 lanes. Expected values use clean M7ef binary32 `d_res_d_xi`, clean M7fw binary32 6x6 outputs, and the Rust snapshot's binary32 `sqrt_weight` (serialized as f64 but widened from f32). Matrix products use the pinned first-product-plus-f32-FMA reduction and scalar f32 rounding.

## Relation capture

| host | target | branch/capture |
|---|---|---|
| (0, cam0) | (0, cam1) | post-write Jacobians captured |
| (0, cam0) | (1, cam0) | post-write Jacobians captured |
| (0, cam0) | (1, cam1) | post-write Jacobians captured |
| (0, cam0) | (2, cam0) | post-write Jacobians captured |
| (0, cam0) | (2, cam1) | post-write Jacobians captured |
| (0, cam0) | (3, cam0) | post-write Jacobians captured |
| (0, cam0) | (3, cam1) | post-write Jacobians captured |
| (0, cam0) | (4, cam0) | post-write Jacobians captured |
| (0, cam0) | (4, cam1) | post-write Jacobians captured |
| (0, cam0) | (0, cam0) | identity branch; no computeRelPose call; d_rel_d_h=d_rel_d_t=zero |

Same-timestamp stereo `(frame 0, cam 0) -> (frame 0, cam 1)` did execute the Jacobian-bearing clean function and both buffers were non-null. Rust's absolute visual-factor path separately zeros both pose blocks whenever only the timestamp matches, so the report keeps the direct clean chain and Rust-policy chain distinct.

## By relation

| relation | observations | clean pose blocks | Rust-policy pose blocks |
|---|---:|---:|---:|
| `same_timecam_identity` | 61 | 1464/1464 | 1464/1464 |
| `same_timestamp_stereo` | 61 | 898/1464 | 0/1464 |
| `cross_time` | 462 | 7161/11088 | 7161/11088 |

The direct clean-chain mismatch is expected to expose any difference between native relative Jacobians and Rust's serialized absolute blocks, including the same-timestamp stereo policy. The Rust-policy mode isolates the remaining cross-time relative-J/absolute-chain boundary without treating the identity branch as a missing call.

## Provenance and artifacts

- Clean raw capture: [`target/m7fw_clean_rel_jacs_all.json`](../../target/m7fw_clean_rel_jacs_all.json)
- Clean raw GDB command: [`target/m7fw_clean_rel_jacs_all.cmd`](../../target/m7fw_clean_rel_jacs_all.cmd)
- Clean raw GDB output: [`target/m7fw_clean_rel_jacs_all.gdb.out`](../../target/m7fw_clean_rel_jacs_all.gdb.out)
- Native raw Jxi: [`target/m7ef_clean_visual_all.json`](../../target/m7ef_clean_visual_all.json)
- Rust detail: [`target/m7fx_fresh5_detail.jsonl`](../../target/m7fx_fresh5_detail.jsonl)
- Comparator: [`m7fw_compare_clean_rel_jacs_all.py`](m7fw_compare_clean_rel_jacs_all.py)
- Comparison JSON: [`target/m7fw_clean_rel_jacs_all_comparison.json`](../../target/m7fw_clean_rel_jacs_all_comparison.json)

No production or clean source was edited, no binary was rebuilt, and no commit or push was performed. The GDB run was one bounded inferior and stopped at the first frame-4 linearization return.
