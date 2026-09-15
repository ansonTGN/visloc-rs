# M7ew ordinal-3 Jp frontier

Date: 2026-08-23 JST  
Status: **Complete — read-only frontier capture; temporary probes removed.**

## Scope

This is the clean native `m7ef` ordinal 3: track 11, observation 0, cam0,
same `TimeCamId`. The exact captured inputs are pixel `(72,131)`, direction
`bea9d0c4,be0917ab`, inverse distance `3e44c804`, and identity `T_t_h`.
The complete machine-readable comparison is
[`target/m7ew_ordinal3_jp_frontier.json`](../../target/m7ew_ordinal3_jp_frontier.json).

## Boundary comparison

All values below are f32 bit patterns. Matrices use Eigen/nalgebra
column-major order; the camera Jacobian is 2x4, bearing Jacobian 3x2, Jpp
4x3, and raw Jp 2x3.

| boundary | Rust | clean native | exact |
|---|---|---|---:|
| point4 `(x,y,z,rho)` | `bf168e1b,be731644,3f45ede0,3e44c804` | same | 4/4 |
| projection `(u,v)` | `428ffd82,43030c0a` | same | 2/2 |
| raw residual | `bb9f8000,3d40a000` | same | 2/2 |
| bearing J (3x2) | `3fb6b185,be0ef5fc,3f857aad,be0ef5fc,3fdbc056,3ed78424` | same | 6/6 |
| camera J (2x4) | `43bd0848,c243e049,c244749a,43ef3fd0,43883f70,42db566c,00000000,00000000` | `43bd0848,c243e04b,c244749c,43ef3fd0,43883f6e,42db5669,00000000,00000000` | 4/8 |
| homogeneous Jpp (4x3) | `3fb6b185,be0ef5fc,3f857aad,00000000,be0ef5fc,3fdbc056,3ed78424,00000000,00000000,00000000,00000000,3f800000` | same | 12/12 |
| raw Jp (2x3) | `444fa80e,c1b2aa34,c1b33178,445a9f4f,00000000,00000000` | `444fa80c,c1b2aa48,c1b33190,445a9f4f,00000000,00000000` | 3/6 |

The first mismatch is camera-J lane 1 (`row 1, column 0`): Rust
`c243e049`, native `c243e04b` (2 ULP). Active camera-J ULP distances are
`[0,2,2,0,2,3]`. The final raw-Jp active differences are lanes 0, 1, and 2
with distances `[2,20,24]`; lane 3 and the two homogeneous rho lanes are
exact. Thus the former ordinal-0 bearing/Jpp boundary is closed here.

## Proposed source grouping

The mismatch begins inside the Double-Sphere projection Jacobian, before the
landmark product. Keep the native `DoubleSphereCamera::project` temporaries
and operation grouping explicit. In particular, native forms `d1_2` as the
separately rounded `r2 + z*z` sequence (the clean assembly has `vmulss`
followed by `vaddss`), while current Rust uses `z.mul_add(z, r2)`. Preserve
the same explicit `d1_2/d1`, `norm`, `d_norm_d_r2`, and `tmp2` stages before
the six Jacobian stores. Do not change Jpp or the fixed landmark product at
this frontier.

The native camera-J lanes were read at the exact clean C++
`linearizePoint<float, DoubleSphereCamera<float>>` stack immediately before
its `Jp * Jpp` write; clean isolated C++ recomputation also confirms the
identity-call bearing Jup/Jpp lanes. No production arithmetic was changed.

## Hygiene

The temporary Rust boundary test, C++ source/executables, and GDB command
file were removed after capture. The requested artifact is the only M7ew
capture retained.
