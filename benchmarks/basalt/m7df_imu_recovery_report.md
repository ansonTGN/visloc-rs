# M7df IMU recovery / production hygiene

Date: 2026-08-23 JST  
Status: **recovered; M7cm production shape retained; M7DC candidate rejected**

## Recovery result

The interrupted `M7DC_TMP_IMU` `eprintln!` was removed from
`pipelines/basalt/src/vio/estimator.rs`.  A source scan of the estimator and
patch implementation contains no `M7DC_TMP_IMU`, `eprintln!`, `println!`,
`dbg!`, `unsafe`, or environment-variable branch.  The temporary diagnostic
artifact created during this recovery (`target/estimator_refs.txt`) was also
removed.

The source-faithful IMU integration operation order remains the established
M7cm shape: the f32 acceleration rotation uses the pinned Eigen/Sophus helper,
the position update contracts `p + v*dt` with `mul_add`, and the general
3x3 LDLT upper solve in `pipelines/basalt/src/patch.rs:475` forms the solved
row dot product followed by one subtraction.  No track-, camera-, frame-,
threshold-, lifecycle-, or environment-specific branch was retained.

## Fresh five-frame gate

The current cleaned source was rebuilt and replayed using the pinned
MH_01_easy sensor-only input:

```text
cargo run --release --example basalt_euroc_vio_demo -- --euroc-dir E:\datasets\euroc_mav\machine_hall\MH_01_easy --calibration target\euroc_ds_calib.json --config target\euroc_config.json --out-dir target\m7df_fresh5 --max-frames 5
```

The replay processed 5 frames, delivered 42 IMU samples, emitted 1114
observations, and reached the frame-4 iteration-start snapshot (70 factors,
1243 rows).  The machine-readable comparison is
[`target/m7df_state_comparison.json`](../../target/m7df_state_comparison.json)
and the fresh detail trace is
`target/m7df_fresh5_detail.jsonl`.

The authoritative clean reference is
`target/m7db_clean_frame4_states.json`; the M7cm baseline is
`target/m7cm_fresh5_detail_20260823.jsonl`.

| comparison | exact lanes | mismatches |
| --- | ---: | ---: |
| clean native vs M7cm baseline | 71/75 | 4 |
| clean native vs cleaned fresh5 | 71/75 | 4 |

The same four lanes remain: frame 2 quaternion `w` (-2 ULP), frame 4
quaternion `w` (-2 ULP), and frame 4 velocity `y`/`z` (+2 ULP each).  No
baseline mismatch was resolved, and no new lane mismatch appeared.  The gate
requires 75/75 exact or at least four fewer mismatches with no regression and
upstream evidence; therefore the candidate does not qualify for production
retention.  The M7cm baseline shape is retained.

The earlier `target/m7dc_tmp_aw_run*` artifacts were sensor-only smoke runs
without a detail trace and supplied no 75-lane improvement evidence.  Their
timestamps, contents, and SHA-256 values, together with the untracked source
file hashes used for the audit, are recorded in the JSON comparison artifact.

## Verification

```text
cargo test --release -p visloc-basalt --lib
```

Result: **166 passed, 0 failed, 1 ignored**.

No commit or push was performed.  Existing unrelated dirty-tree changes were
left untouched.
