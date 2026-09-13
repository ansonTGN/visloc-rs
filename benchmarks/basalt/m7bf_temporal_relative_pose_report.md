# M7bf temporal relative-pose boundary

Date: 2026-08-23 JST  
Scope: `pipelines/basalt/src/vio/aom.rs` only (relative-pose arithmetic and
the fixed visual-factor fixture).

## Result

Production remains on `F32Pose`.  The f32 anchored factor converts the public
f64 poses once, composes them with the audited Sophus packet product, and
normalizes every SO3 product at the same boundary as Sophus' `SO3` quaternion
constructor.  The relative chain is therefore:

```text
T_t_c_t = inverse(T_i_c_t)
T_t_i_h_i = (T_w_i_t.so3().inverse() * T_w_i_h.so3(),
             T_w_i_t.so3().inverse() * (t_h - t_t))
T_t_h = T_t_c_t * T_t_i_h_i * T_i_c_h
```

This is the pinned `basalt/utils/ba_utils.h::computeRelPose` order.  The
existing `m7_relative_pose_chain_matches_pinned_lanes` test retains exact
intermediate and final assertions, and
`m7_anchored_factor_matches_pinned_projection_and_raw` asserts the downstream
projection/raw boundary (`41da6d22,42d70d2e` and `bc228000,3f915340`).

## Audit of the rejected raw alternate

The temporary `F32SourcePose`/`source_quat_*` path used an unnormalized raw
quaternion product.  That path is not a faithful general replacement: pinned
Sophus `SO3::operator*` returns an `SO3` constructed from the product
quaternion, and the constructor normalizes it.  The raw path also failed to
reproduce the supplied alternate fixture, so it was removed rather than
leaving dead production-adjacent code or weakening the exact test.

The pinned native C++ probe
`benchmarks/basalt/m7bf_sophus_oracle_tmp.cpp`, compiled from the local
Basalt commit `0f3b2b52c807f70ff4e2973ce253c73329eea7bc` with
`-std=c++17 -O2 -march=native -DEIGEN_DONT_PARALLELIZE`, reports:

```text
target_camera_from_anchor_camera=bc0e2956,bb507eff,ba002940,3f7ffd31
target_camera_from_anchor_camera_t=bde1e254,baf1ecc0,ba884364
compute_rel=bc0e2956,bb507eff,ba002940,3f7ffd31
compute_rel_t=bde1e254,baf1ecc0,ba884364
```

The requested alternate values
`t=bde1e255,baf1ec60,ba884368` and
`q_xyzw=bc0e2957,bb507f02,ba002d40,3f7ffd31` are therefore not proven by the
pinned source/compiler.  A raw Rust trial produced yet another result
(`t=bde1e254,baf1ecc0,ba884364`,
`q_xyzw=bc0e2957,bb507f01,ba002941,3f7ffd33`), confirming that simply removing
normalization is not the missing source operation.  Until a native probe with
the alternate provenance is supplied, changing the production relative pose
would be speculative.

## Verification

Release focused M7 tests:

```text
cargo test --release -p visloc-basalt --lib m7_ -- --nocapture
16 passed, 0 failed, 144 filtered out
```

Release full library:

```text
cargo test --release -p visloc-basalt --lib
158 passed, 0 failed, 2 ignored
```

