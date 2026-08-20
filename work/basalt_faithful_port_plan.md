# Basalt faithful (ベタ) port plan

Date: 2026-08-20. Decision: stop SuperPoint A/B cliff digging as the
primary path; port Basalt’s EuRoC VIO profile **parameter-for-parameter**
and fill missing frontend / window pieces to match.

Upstream: https://github.com/VladyslavUsenko/basalt (BSD-3-Clause).
We reimplement; we do **not** vendor Basalt C++ sources.

## Canonical config

Checked in: [`configs/basalt/euroc_config.json`](../configs/basalt/euroc_config.json)
(verbatim from Basalt `data/euroc_config.json`).

Key VIO / OF knobs (EuRoC defaults):

| Key | Value | Meaning |
|---|---|---|
| `optical_flow_type` | `frame_to_frame` | KLT between consecutive frames |
| `optical_flow_detection_grid_size` | 50 | Detection cell size (px) |
| `optical_flow_pattern` | 51 | Patch pattern id |
| `optical_flow_max_iterations` | 5 | LK iters per level |
| `optical_flow_levels` | 3 | Pyramid levels |
| `optical_flow_epipolar_error` | 0.005 | Stereo epipolar gate |
| `optical_flow_skip_frames` | 1 | Process every frame |
| `vio_linearization_type` | `ABS_QR` | Absolute QR linearization |
| `vio_sqrt_marg` | true | Square-root marginalization |
| `vio_max_states` | 3 | Non-KF states in window |
| `vio_max_kfs` | 7 | Keyframes in window |
| `vio_min_frames_after_kf` | 5 | Min gap before new KF |
| `vio_new_kf_keypoints_thresh` | 0.7 | KF trigger on track loss ratio |
| `vio_obs_std_dev` | 0.5 | Observation σ |
| `vio_obs_huber_thresh` | 1.0 | Huber |
| `vio_max_iterations` | 7 | VIO LM iters |
| `vio_use_lm` | true | Levenberg–Marquardt |

## Already in visloc (reuse, do not re-tune away)

- Sqrt / SqrtToSqrt window + FEJ carry: `pipelines/slam/src/vi_sqrt_window.rs`,
  `marginalization_sqrt.rs`, `online_slam_vi_ba.rs`
  (`use_sqrt_window_marginalization`, default on in the EuRoC demo).
- NFR seed: `nonlinear_factor_recovery.rs`.
- IMU preintegration + motion-VI init.

## Missing for faithful parity (ordered)

1. **Optical-flow frontend** (`crates/vision/src/optical_flow/`) matching
   Basalt frame-to-frame KLT with the table above. Requires grayscale
   images on the VI path (not descriptor-only SuperPoint frames).
2. **Basalt profile loader** — deserialize `euroc_config.json` into
   `BasaltVioConfig` and drive demo / `OnlineSlamLocalBaConfig` /
   keyframe policy from those numbers (`vio_max_kfs=7`, etc.).
3. **Wire OF tracks → landmarks / VI-BA** instead of appearance-global PnP.
4. **Mapper / NFR** with Basalt mapper_* knobs (after VIO tracks).
5. Freeze the SuperPoint “tight-vi-tracking-cliff” stack as an alternate
   profile; do not mix its jump/inlier gates into the Basalt profile.

## Success gate (Basalt-faithful)

On EuRoC MH_01 **full sequence**, Basalt-profile run must reach literature
ballpark (~0.09 m SE3 ATE, high coverage) under the same evo protocol as
[`benchmarks/protocols/euroc_vi_sota_bakeoff_v1.json`](../benchmarks/protocols/euroc_vi_sota_bakeoff_v1.json).

## License note

Basalt is BSD-3-Clause. Any future direct port of algorithms must keep
attribution; prefer clean-room reimplementation from papers + public
config values (this plan).
