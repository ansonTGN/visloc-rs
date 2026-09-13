# M7fx same-timestamp stereo pose Jacobians

Date: 2026-08-24 JST  
Status: **Complete.**

## Change

The absolute visual factor now distinguishes the complete `TimeCamId`
identity from a same-timestamp stereo pair:

- `(frame, camera) == (frame, camera)` keeps the exact identity transform and
  zero `d_rel_d_h`/`d_rel_d_t` pose blocks.
- `(frame 0, cam 0) -> (frame 0, cam 1)` follows the clean `computeRelPose`
  chain and produces nonzero host and target pose Jacobians.
- The UpstreamF32 window scatter performs the source-order sequence
  `old + anchor`, then `result + target` when both terms address the same six
  pose columns. It does not overwrite the first contribution.

No value-specific branch, debug output, `unsafe`, commit, or push was added.

## Clean relation fixture

[`m7fx_same_timestamp_clean_relpose_jacobian.json`](../../pipelines/basalt/tests/fixtures/m7fx_same_timestamp_clean_relpose_jacobian.json)
is sourced from M7fw completion 0 for host `(0, cam0)` to target
`(0, cam1)`. The focused AOM test replays both six-by-six products and matches
all **72/72 binary32 lanes**, including signed zero.

## End-to-end state-row test

`same_timestamp_stereo_rows_accumulate_both_pose_blocks` verifies the grouped
window path. The two individual 2x6 blocks are nonzero; for the equal state
block they cancel in the aggregate row, so the expected source-order f32 sum
is exactly zero. This makes an anchor overwrite detectable while retaining the
source result:

| relation | observations | anchor pose lanes | target pose lanes | aggregate state-pose lanes |
|---|---:|---:|---:|---:|
| same-TimeCam identity | 61 | 0/732 | 0/732 | 0/732 |
| same-timestamp stereo | 61 | 732/732 nonzero | 732/732 nonzero | 0/732 (exact cancellation) |
| cross-time | 462 | unchanged | unchanged | unchanged |

## Fresh-five replay

Command:

```text
VISLOC_BASALT_DETAIL_ITERATIONS=1
VISLOC_BASALT_DETAIL_FRAME=4
VISLOC_BASALT_DETAIL_TRACE=target/m7fx_fresh5_detail.jsonl
cargo run --release --example basalt_euroc_vio_demo -- --euroc-dir E:\datasets\euroc_mav\machine_hall\MH_01_easy --calibration target\euroc_ds_calib.json --config target\euroc_config.json --out-dir target\m7fx_fresh5_run --max-frames 5
```

Compared with the M7fv fresh-five snapshot (`target/m7fv_fresh5_detail.jsonl`)
at frame 4 / iteration 0 / `iteration_start`:

| quantity | exact | total | result |
|---|---:|---:|---|
| global H | 5,625 | 5,625 | exact |
| global b | 75 | 75 | exact |
| factor state rows | 87,600 | 87,600 | exact |
| same-TimeCam `jp_anchor` | 732 | 732 | exact zero |
| same-TimeCam `jp_target` | 732 | 732 | exact zero |
| stereo `jp_anchor` vs prior zero baseline | 0 | 732 | 732 nonzero lanes |
| stereo `jp_target` vs prior zero baseline | 0 | 732 | 732 nonzero lanes |
| cross-time `jp_anchor` | 5,544 | 5,544 | exact |
| cross-time `jp_target` | 5,544 | 5,544 | exact |

The 61 stereo observations have aggregate pose-column norm **0.0** after the
sequential add, while the retained individual block norms are
`||J_anchor||₂ = 10670.4609841552` and
`||J_target||₂ = 10670.4609841552`; each individual block has 732 nonzero
lanes. The snapshot retains the same cost-before `4215.93212890625`, 70
factors, 1,243 rows, and visual row spans as M7fv.

Artifacts: [`target/m7fx_fresh5_detail.jsonl`](../../target/m7fx_fresh5_detail.jsonl),
[`target/m7fx_fresh5_run`](../../target/m7fx_fresh5_run), and the keyed
M7fw comparison at [`target/m7fx_clean_rel_jacs_all_comparison.json`](../../target/m7fx_clean_rel_jacs_all_comparison.json).

## Verification

- `cargo test --release -p visloc-basalt m7 -- --nocapture`: **30 passed**.
- `cargo test --release -p visloc-basalt --lib`: **178 passed, 1 ignored**.
- Fresh five-frame replay completed: **5/5 frames**, sensor-only mode.
