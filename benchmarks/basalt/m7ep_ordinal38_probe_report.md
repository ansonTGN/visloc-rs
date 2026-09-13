# M7ep clean ordinal-38 T/point probe

Date: 2026-08-23 JST  
Status: **Completed — read-only probe; first mismatch is in `T_t_h`, while the transformed point is exact.**

## Selection and inputs

Clean record 38 in [target/m7ef_clean_visual_all.json](../../target/m7ef_clean_visual_all.json)
matches Rust detail line 2 (`iteration_start`, iteration 0) in
[target/m7ee_fresh5_detail.jsonl](../../target/m7ee_fresh5_detail.jsonl):

```text
track_id 18, host frame 0 cam 0 -> target frame 4 cam 1
pixel     42f1f732,422fb6ee
direction be8ebf80,be6ab112
rho       3e1d5fdc
```

The full frame-0/frame-4 state poses, both `T_imu_cam` extrinsics, all ten
track-18 observations, and f32 input packets are retained in the machine
artifact [target/m7ep_ordinal38_probe.json](../../target/m7ep_ordinal38_probe.json).
The calibration source is [target/euroc_ds_calib.json](../../target/euroc_ds_calib.json).

## Isolated current-path replay

`target/m7ep_probe.rs` copies only the current M7ec/M7dx f32 operations:

```text
T_t_h = inverse(T_imu_cam_target) * inverse(T_w_i_target)
        * T_w_i_host * T_imu_cam_host
point = current bearing_f32 + normalized native quaternion matrix/FMA rows
```

It was compiled without changing production or test sources:

```text
rustc target/m7ep_probe.rs -C opt-level=3 -C target-feature=+avx,+fma \
  -o target/m7ep_probe.exe
```

As a packet sanity check, the same isolated chain reproduces the retained
M7ec same-timestamp stereo q/t witness exactly (`7/7` lanes):

```text
q: bbefaaa9 b9eadbb0 ba8bff3a 3f7ffe35
t: bde1c0f0 3997d300 b9e136c8
```

## Bit comparison

The matrix layout is Eigen 4×4 column-major. The only difference is:

| stage | equal | first mismatch | clean | current | meaning |
|---|---:|---|---|---|---|
| `T_t_h` matrix16 | 15/16 | index 9 = row 1, col 2 | `bc20b37c` | `bc20b37b` | one-ULP bit difference; current is slightly less negative |
| target point4 `(x,y,z,rho)` | **4/4** | none | `bf04c770 bed67432 3f425028 3e1d5fdc` | `bf04c770 bed67432 3f425028 3e1d5fdc` | exact |

For the mismatch lane, clean is `-0.009808417409658432` and current is
`-0.009808416478335857` (numeric current-minus-clean `+9.313225746154785e-10`). All other transform lanes, including translation,
are bitwise equal. Therefore the first divergence in the requested T-versus-
point audit is **`T_t_h[9]`**, and the point stage subsequently remains exact.

No production source, test, clean artifact, commit, or push was changed.
