# m7ap IMU normal-equation boundary report

Date: 2026-08-22  
Dataset: `MH_01_easy`, first five frames, frame index 4  
Upstream trace: `target/m7al_upstream_f4_20260821T000200Z/iteration.jsonl`  
Current trace: `target/m7ap_rust_final.jsonl`

## First divergence and fix

The pinned Basalt header (`basalt/imu/preintegration.h`) computes the IMU
residual Jacobian analytically in `[position, rotation, velocity]` row order.
Rust was finite-differencing the residual with
`SE3::exp(delta).compose(state)`.  That operation rotates the world-frame
translation, unlike Basalt `PoseState::incPose`, which adds translation and
left-multiplies only SO(3).  The resulting pose Jacobian had a spurious
position/rotation term and was the first normal-equation divergence.

`imu/factors.rs` now ports the upstream closed-form Jacobian, including the
right-Jacobian state blocks and the left-Jacobian gyro-bias rotation block.
The f32 path also uses an Eigen-structure LDLT left whitener
`D^-1/2 L^-1 P` before the f32 row products.  A regression test compares the
analytic Jacobian with the corrected left-pose finite difference (maximum
absolute error `< 2e-6`).

## Frame-4 initial H/b comparison

All values below are Rust minus upstream unless stated otherwise:

| entry | upstream | Rust | difference |
|---|---:|---:|---:|
| H[0,0] | 479284352 | 479284096 | -256 |
| H[19,19] | 557600000 | 557595520 | -4480 |
| H[24,24] | 4000629248 | 4000628736 | -512 |
| H[45,49] | -1666753.125 | -1665133.125 | +1620 |
| H[49,49] | 533197056 | 533192864 | -4192 |
| b[0] | 7113.7900390625 | 7114.30615234375 | +0.51611328125 |
| b[19] | -32461.0546875 | -32402.4765625 | +58.578125 |
| b[49] | 96862.15625 | 96841.578125 | -20.578125 |

The former Rust H[45,49] was `-14007340`; the analytic Jacobian reduces the
error from roughly 12.3M to 1.62k.  Across the 75x75 system, the current
maximum absolute H difference is `147351.25` at `(54,48)`, RMS is
`10153.41648509783`; b maximum is `58.578125` at index 19, RMS is
`10.298902961920618`.  The initial costs are upstream `4215.9326171875` and
Rust `4216.1083984375`.

The remaining b[19] gap is not changed by switching from f64 Cholesky to the
active-f32 LDLT whitener; it is therefore not an LDLT/solver issue.  The
factor-level trace records four active IMU links and an initial IMU objective
of `1.5854905533345082e-6` (bias objective zero), allowing the next audit to
continue at residual/Jacobian scalar arithmetic if needed.

## Current LM decision/cost trace

The current Rust frame-4 run accepts every trial (no rejected trial):

```
iteration 0: 4216.1083984375 -> 412.91525244140625, lambda 1e-4 -> 3.33333337e-5
iteration 1:  412.91525244140625 -> 277.4624328613281, lambda -> 1.11111112e-5
iteration 2:  277.4624328613281 -> 259.6768798828125, lambda -> 3.70370390e-6
iteration 3:  259.6768798828125 -> 253.01812744140625, lambda -> 1.23456800e-6
iteration 4:  253.01812744140625 -> 250.4379119873047, lambda -> 1e-6
iteration 5:  250.4379119873047 -> 249.2764434814453, lambda stays 1e-6
iteration 6:  249.2764434814453 -> 248.7306213378906, lambda stays 1e-6
iteration 7:  248.7306213378906 -> 248.4530487060547, lambda stays 1e-6
```

## Verification

```
& 'C:\Users\rsasa\.cargo\bin\cargo.exe' check -p visloc-basalt --tests
& 'C:\Users\rsasa\.cargo\bin\cargo.exe' test --release -p visloc-basalt --lib analytic_imu_jacobian_matches_left_pose_finite_difference -- --nocapture
& 'C:\Users\rsasa\.cargo\bin\cargo.exe' test --release -p visloc-basalt --lib same_timestamp_stereo_rows_have_no_absolute_pose_jacobian -- --nocapture
& 'C:\Users\rsasa\.cargo\bin\cargo.exe' test --release -p visloc-basalt --lib                         # 122 passed
& 'C:\Users\rsasa\.cargo\bin\cargo.exe' test --release -p visloc-basalt --test m7f_camera_contract -- --nocapture  # 3 passed
& 'C:\Users\rsasa\.cargo\bin\cargo.exe' test --release -p visloc-basalt --tests --no-run
$env:VISLOC_BASALT_DETAIL_ITERATIONS='1'; $env:VISLOC_BASALT_DETAIL_FRAME='4'; $env:VISLOC_BASALT_DETAIL_TRACE='target/m7ap_rust_final.jsonl'; & 'C:\Users\rsasa\.cargo\bin\cargo.exe' run --release --example basalt_euroc_vio_demo -- --euroc-dir 'E:\datasets\euroc_mav\machine_hall\MH_01_easy' --calibration target\euroc_ds_calib.json --config target\euroc_config.json --out-dir target\m7ap_rust_final_run --max-frames 5
```
