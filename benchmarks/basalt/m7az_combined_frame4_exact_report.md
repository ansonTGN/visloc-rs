# M7az combined frame-4 exactness verification

Date: 2026-08-22 JST  
Scope: combined camera-bearing FMA, landmark triangulation, TimeCamId/Jpp, and
window changes for the pinned Basalt commit `0f3b2b52...`

## Verification run

A single fresh release replay used the pinned MH_01_easy sensor-only inputs:

```text
cargo run --release --example basalt_euroc_vio_demo -- --euroc-dir E:\datasets\euroc_mav\machine_hall\MH_01_easy --calibration target\euroc_ds_calib.json --config target\euroc_config.json --out-dir target\m7_combined_exact_fresh5_20260822 --max-frames 5
```

Detail tracing was enabled for frame 4, iteration 0 with
`VISLOC_BASALT_DETAIL_ITERATIONS=1`, `VISLOC_BASALT_DETAIL_FRAME=4`, and
`VISLOC_BASALT_DETAIL_TRACE=target/m7_combined_exact_fresh5_detail_20260822.jsonl`.
The replay completed normally: 5 frames, 1114 observations, frame 4 with 70
factors, 1243 rows, and 58 landmarks.

Artifacts and SHA-256:

| artifact | path | SHA-256 |
| --- | --- | --- |
| replay summary | `target/m7_combined_exact_fresh5_20260822/summary.txt` | `15E87096CA5F2CB83F671A0DE2E1CC56A5BB41331E9B5B1182A97C8FA720C48A` |
| replay trace | `target/m7_combined_exact_fresh5_20260822/trace.jsonl` | `AA4D9F4157755761E73F011AF7FD493E87ADA94B2DA294618D408EC4178DFC10` |
| detail trace | `target/m7_combined_exact_fresh5_detail_20260822.jsonl` | `E15282CF96C218D1B86BAA34B656D06ACAAACF452B4F8BAB211879022C236FD6` |
| zero-tolerance diff | `target/m7_combined_exact_diff_zero.json` | `C424CA511535A7CFFEC43C3B6930D90F3C76F012AB2E4CC67926D5F353A6D022` |

## Canonical comparison

The comparator used absolute and relative tolerance zero against
`target/m7al_upstream_f4_20260821T000200Z/iteration.jsonl`:

```text
snapshot_count: 24
structural_difference_count: 0
first_control_flow_difference: null
order_observations: 8
```

The first overall payload difference remains the frame-0 state translation x:

```text
upstream: 3.500492312014103e-06
Rust:     3.4996728572878055e-06
```

## Initial landmark boundary

All 61 initial landmark IDs are present on both sides. The exact f32 landmark
triple count is `34/61`. The first ascending-ID mismatch is track 4:

```text
field:    direction.y
upstream: -0.02641083300113678 (0xbcd85b88)
Rust:     -0.02641083113849163 (0xbcd85b87)
```

## First visual boundary

The same-timestamp stereo Jpp/Jl boundary is closed. The first remaining factor
difference is the temporal track-1 observation 2 (frame 1, camera 0):

```text
path:     landmark_factors[track_id=1].observations[2].raw_residual[0]
upstream: -0.009939193725585938
Rust:     -0.009916305541992188
```

## Frame-4 iteration-0 reduction

| value | upstream | Rust |
| --- | ---: | ---: |
| `global.H[0][0]` | `479284352` | `479284704` |
| `global.H[0][5]` | `33102.375` | `33108.85546875` |
| `global.b[0]` | `7113.7900390625` | `7113.81689453125` |
| cost before | `4215.9326171875` | `4215.8642578125` |

All eight LM decisions are accepted on both sides: `AAAAAAAA`. The final
accepted actual costs are upstream `248.54075622558594` and Rust
`248.4744415283203`.

## Focused test evidence

The production slice was covered by release focused tests before this replay:

```text
cargo test --release -p visloc-basalt --lib m7_ -- --nocapture
13 passed, 0 failed

cargo test --release -p visloc-basalt --lib same_time -- --nocapture
3 passed, 0 failed
```

The combined replay generated verification artifacts only; no production source,
landmarks, camera, window, `HANDOFF`, or `work` files were edited for this
report.
