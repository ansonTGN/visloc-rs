# M7dv Rust relative-pose Jacobian comparison

Date: 2026-08-23 JST  
Status: **Pass — 64/72 current Rust raw lanes match; the remaining 8 are signed-zero lanes with a confirmed arithmetic cause.**

## Scope and probe

The isolated probe at
[`target/m7dv_rel_jac_probe/src/main.rs`](../../target/m7dv_rel_jac_probe/src/main.rs)
replays the current f32 AOM relative-pose Jacobian path for the exact clean
m7ds factor: frame 4, iteration 0, track 1, host `(frame 0, cam 0)` to target
`(frame 1, cam 1)`. It uses the captured f32 pose/calibration words from
m7bo/m7ds, the current packet quaternion products, Eigen rotation-matrix
schedule, 3x3 product, and 6x6 fixed-product FMA schedule. No production
source was imported or changed; the probe is intentionally isolated under
`target/`.

At this worktree snapshot there is no public Rust symbol literally named
`compute_relative_camera_transform`; the audited raw equivalent is the
`eigen_adjoint_times_rotation_blocks_f32` path called by
`anchored_visual_reprojection_factor_f32_with_time_cam`.

Run:

```powershell
& 'C:\Users\rsasa\.cargo\bin\cargo.exe' run --release `
  --manifest-path target/m7dv_rel_jac_probe/Cargo.toml
```

The complete machine-readable comparison is
[`target/m7dv_rel_jac_comparison.json`](../../target/m7dv_rel_jac_comparison.json).
Both matrices use Eigen column-major order (`r0c0,r1c0,...,r5c5`).

## Results

| raw buffer | equal | total | result |
|---|---:|---:|---|
| `d_rel_d_h` | 36 | 36 | exact |
| `d_rel_d_t` | 28 | 36 | 8 signed-zero differences |
| both | 64 | 72 | all nonzero values exact |

The first mismatch is `d_rel_d_t[3]` (row 3, column 0): current Rust
`0x80000000`, clean target `0x00000000`. The other mismatches are indices
4, 5, 9, 10, 15, 16, and 17. The clean target intentionally retains one
negative zero at index 11; it is not normalized away.

## First divergent operation

The pinned upstream `ba_utils.h` formula is:

```cpp
*d_rel_d_h = tmp.Adj() * RR;
*d_rel_d_t = -tmp2.Adj() * RR;
```

In C++, unary `-` binds to `tmp2.Adj()` before the Eigen 6x6 product. The
current Rust AOM path forms the product and then negates the returned matrix:

```rust
let target = -eigen_adjoint_times_rotation_blocks_f32(...);
```

That post-product negation flips every zero in the lower block to negative
zero. An isolated sign-left replay (`(-adjoint) * RR`) matches the clean
target at **36/36** target-buffer lanes, including the mixed signed-zero
pattern; the host buffer remains **36/36** exact. This isolates the first
divergence to negation placement, not quaternion composition, rotation-matrix
construction, or the 6x6 multiplication tree.

## Proposed next arithmetic change

In `pipelines/basalt/src/vio/aom.rs`, add/use a target-side helper that negates
the assembled 6x6 adjoint before `eigen_matrix_product_6x6_f32`, and keep the
host-side helper unchanged. Do not normalize signed zeros afterward. The
probe predicts 72/72 raw lanes exact against m7ds after that one operation
placement change.

Integrity: production arithmetic and production debug output were untouched;
no commit or push was performed.
