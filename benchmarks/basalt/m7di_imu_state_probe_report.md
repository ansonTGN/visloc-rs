# M7di IMU state-boundary probe

Date: 2026-08-23 JST  
Status: **diagnosis complete; M7cm baseline retained; no production edit**

## Result

The four clean-frame-4 mismatches have two independent first-divergence sites.

| lane | first divergent operation | evidence |
|---|---|---|
| frame 2 quaternion `w` | Rust UpstreamF32 solver chart: `q.xyz` is flattened, then `q.w = sqrt(1 - (x*x + y*y + z*z))` is reconstructed | `target/m7di_state_boundary_probe` and `decode_rotation_with_mode` in `pipelines/basalt/src/vio/window.rs` |
| frame 4 quaternion `w` | same q.xyz-only flatten/decode boundary | direct `predict_nav` trace is native-exact; decoded solver block is `3f159717` |
| frame 4 velocity `y,z` | preintegration packet arithmetic, not `predictState`: O2/Rust and clean O3 differ first at packet 6 velocity-x | `m7au_imu_pre_solver_velocity_report_20260822.md`, O2/O3 step logs |

The pinned Sophus `SO3 * SO3` product itself is exact for both endpoint
products: `target/m7di_predict_product_probe.log` reports native
`3f17e7a6` (frame 2) and `3f159719` (frame 4).  Therefore changing quaternion
product normalization, SO3 exp, or `SE3::new` is not justified.

## Quaternion boundary

The live fresh-5 `state_trace.predicted_state` values are direct f32-owned
navigation states.  Their q bits are native-exact for all five frames,
including frame 2 `3f17e7a6` and frame 4 `3f159719`.  Before the solver
snapshot, `flatten_nav_with_mode` retains only xyz (the 3-DoF chart), and
`decode_rotation_with_mode` reconstructs w from the xyz norm.  The standalone
fixture gives:

```text
frame 2 native w=3f17e7a6, decoded w=3f17e7a4
frame 4 native w=3f159719, decoded w=3f159717
```

Alternative f32 reduction associations and FMA norm accumulation do not
recover those direct-product w bits.  The missing fourth component is
information lost by the 3-DoF solver chart; carrying it would change the
state layout/factor contract.  No general q.w-only production patch can be
accepted from these four samples.

## Velocity boundary

The fresh-5 replay was run with the current source into
`target/m7di_fresh5` (5 frames, 42 IMU samples, 1114 observations, frame-4
window 70 factors / 1243 rows).  Direct predicted velocity bits were exact
through frame 3; frame 4 was:

```text
Rust/current: 3da7a8b8 bc8bed08 be67f1c8
clean native: 3da7a8b8 bc8bed06 be67f1c6
```

The pinned frame-3→4 probe shows the current interval delta-v is the O2
pattern `3ede21be,bcb31cdc,be20c273`; the clean O3 pattern is
`3ede21c0,bcb31cdc,be20c273`.  Per-sample O2/O3 output first differs at
packet 6 in velocity-x (`3e85be12` vs `3e85be13`); acceleration rotation
lanes before that point are equal.  A per-sample FMA velocity candidate moved
the final x toward O3 but regressed frame-1/2/3 exact y lanes, so it was
reverted.  The remaining y/z endpoint mismatch is target/compiler packet
evaluation variance, not a source-faithful `rotation*accel` or prediction
association error.

## Acceptance and verification

The authoritative comparison remains 71/75 exact with exactly the four lanes
listed above.  No candidate improves all four without regression; production
source was not changed and no diagnostic print was added to production.

The fresh-5 replay command completed successfully.  The prior M7df clean
release verification for the unchanged baseline was **166 passed, 0 failed,
1 ignored** (`cargo test --release -p visloc-basalt --lib`); its four-lane
gate and state comparison are retained in `target/m7df_state_comparison.json`.
No commit or push was performed.
