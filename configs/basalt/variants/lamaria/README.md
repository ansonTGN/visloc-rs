# LaMAria Basalt variants

Per-sequence calibration/config variants for the Project Aria LaMAria benchmark
(see [`docs/lamaria_plan.md`](../../../docs/lamaria_plan.md) and
[`docs/lamaria_stage0.md`](../../../docs/lamaria_stage0.md)).

## Calibrations

- `<seq>_calib.json`: LaMAria's shipped Kalibr IMU noise, converted from the
  pinhole camera model to Basalt's Double Sphere with `xi = 0`, `alpha = 0`.
- `<seq>_calib_variantA_default_noise.json`: the same cameras, but Basalt's
  own EuRoC-default IMU noise. **Use this one** — on R_01_easy the datasheet
  noise gives SE3 ATE 1.60 m vs 0.43 m for variant A.

## VIO configs

- `euroc_config.json`: EuRoC-derived Basalt VIO config (default sliding window:
  `vio_max_states = 3`, `vio_max_kfs = 7`).
- `euroc_config_mapper_reduced_points.json`: the same config with
  `mapper_detection_num_points` 800 → 200, used for the offline mapper runs.
- `euroc_config_big_window.json`: **the larger sliding window found to matter
  at km scale** — `vio_max_states = 10`, `vio_max_kfs = 30`, everything else
  unchanged.

## Why the big window

Basalt's default window is tuned for short, room-scale sequences. On the
1.5 km LaMAria `sequence_1_19` (18,352 frames) it leaves `sequence_1_19`
VIO-only at official CP Score **12.75**; enlarging the window is a
config-only, parity-safe change that lifts it to **27.09** (CP@1m 7.1 % →
14.3 %, pGT pose recall @5 m 5.4 % → 30.0 %), essentially matching the best
published academic baseline (27.7). Global ATE keeps improving further at
`vio_max_states = 15` / `vio_max_kfs = 45` (SE3 13.60 → 11.91 m) but the
official 14-control-point Score drops to 22.27, so the window should be
selected on the official metric, not ATE. Wall time for the 18,352-frame
sequence: 12 min (3/7), 58 min (10/30), 1 h 51 min (15/45).
