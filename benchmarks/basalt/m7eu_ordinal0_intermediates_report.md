# M7eu ordinal-0 Rust factor-call intermediates

Date: 2026-08-23 JST  
Status: **Complete — read-only arithmetic probe; temporary source instrumentation removed.**

## Scope and result

The current Rust `anchored_visual_reprojection_factor_f32_with_time_cam` was
called with the exact M7eo ordinal-0 inputs: cam0, pixel `(388, 193)`,
direction `3cc4c95a,bd7b2508`, inverse distance `3e46b55a`, identity `T_t_h`,
and the same-`TimeCamId` branch. The full capture is
[`target/m7eu_ordinal0_intermediates.json`](../../target/m7eu_ordinal0_intermediates.json).

The first mismatch is **not** in the camera projection Jacobian. Rust camera
`J` is exact in all 8 row-major lanes. The first mismatch is the bearing
Jacobian copied into the active homogeneous landmark block `Jpp`, lane 0:

```text
Rust Jpp lane 0:   3ffe9a0a
Native Jpp lane 0: 3ffe9a0c
distance:          2 ULP
```

Because `T_t_h` is identity, `bearing_jacobian_f32` directly supplies the
active 3x2 Jpp block. All six active bearing/Jpp lanes differ; the four
homogeneous/zero lanes and the final affine `Jpp[3,2] = 1` lane are exact.

## Intermediate lane comparison

| boundary | Rust lanes | native lanes | exact |
|---|---|---|---:|
| target point4 `(x,y,z,rho)` | `3d43efd6,bdfa0f6e,3f7dca10,3e46b55a` | `3d43efd6,bdfa0f6e,3f7dca10,3e46b55a` | 4/4 |
| bearing `(x,y,z)` (identity point4 prefix) | `3d43efd6,bdfa0f6e,3f7dca10` | same native point4 prefix; no separate native bearing capture | 3/3 |
| camera projection `J` row-major | `43e6b4aa,3fe6b2f7,c1b056e5,00000000,3fe604cb,43e4157f,426062c3,00000000` | `43e6b4aa,3fe6b2f7,c1b056e5,00000000,3fe604cb,43e4157f,426062c3,00000000` | 8/8 |
| homogeneous `Jpp` column-major | `3ffe9a0a,3bbf6402,bdc3173f,00000000,3bbf6402,3ffcfc82,3e78fb03,00000000,00000000,00000000,00000000,3f800000` | `3ffe9a0c,3bbf6405,bdc31742,00000000,3bbf6405,3ffcfc84,3e78fb07,00000000,00000000,00000000,00000000,3f800000` | 6/12 |
| raw landmark `Jp` column-major | `4465f920,3f652458,3f65d1e8,4464cfbd,00000000,00000000` | `4465f922,3f652460,3f65d1e0,4464cfbf,00000000,00000000` | 2/6 |
| projection `(u,v)` | `43c2000b,43411269` | `43c2000b,43411269` | 2/2 |
| raw residual `(u,v)` | `39b00000,3d934800` | `39b00000,3d934800` | 2/2 |

The six active Jpp lane deltas are `[2, 3, 3, 3, 2, 4]` ULP (column-major,
excluding exact homogeneous lanes). The final raw Jp has the expected first
mismatch at lane 0 and is only 2/6 exact; its lane distances are
`[2, 8, 8, 2, 0, 0]`.

## Probe hygiene and verification

The capture used a temporary environment-gated trace block and a temporary
unit-test driver in `pipelines/basalt/src/vio/aom.rs`. Both were removed after
the run; no production arithmetic was changed, and no `M7EU` marker remains in
that source. The focused release test used for capture passed (`1 passed`),
and the temporary trace output was not used as a production input. Native
reference lanes come from [`target/m7eo_jp_codegen_probe.json`](../../target/m7eo_jp_codegen_probe.json)
and the exact clean ordinal-0 input comes from `target/m7ef_clean_visual_all.json`.

## Conclusion

The camera projection-J boundary is closed for this call. The first actionable
boundary is `bearing_jacobian_f32` / active homogeneous `Jpp`, before the
fixed `2x4 * 4x3` landmark product. The mismatch is value-independent evidence
from the exact same direction/rho/pixel/camera and does not justify changing
camera projection or final Jp arithmetic in this probe.
