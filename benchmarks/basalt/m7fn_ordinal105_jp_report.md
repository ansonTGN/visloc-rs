# M7fn ordinal-105 same-TimeCam Jp frontier

Date: 2026-08-23 JST  
Status: **Complete — bounded read-only capture; production arithmetic unchanged.**

## Selection

The m7fl fresh-five comparator identifies native ordinal **105** as the first
remaining weighted-landmark-Jp mismatch: track **9**, observation 0, host and
target frame 0/cam0 (same timestamp and same camera, so `T_t_h` is the exact
identity). The exact key is pixel `42bc0000,42580000`, direction
`be9dd414,be637e0a`, inverse distance `3e21489f`, and `sqrt_weight` is exactly
`40000000` (2.0).

The machine-readable capture is
[`target/m7fn_ordinal105.json`](../../target/m7fn_ordinal105.json). Clean
intermediates came from `target/m7ef_clean_visual_all.json`; current detail
came from `target/m7fl_fresh5_detail.jsonl`.

## Boundary comparison

All values are f32 bit patterns; matrices are column-major in the order declared
in the JSON artifact.

| boundary | clean native | current Rust | exact |
|---|---|---|---:|
| `T_t_h` 4x4 | identity | identity | 16/16 |
| point4 `(x,y,z,rho)` | `bf09ea90,bec6ca88,3f3f6784,3e21489f` | same | 4/4 |
| projection `(u,v)` | `42bc0378,4257d6ca` | same | 2/2 |
| raw residual `(u,v)` | `3bde0000,bd24d800` | same | 2/2 |
| bearing J 3x2 | `3fba8d50,be563110,3f710856,be563110,3fcc6805,3f2db603` | same | 6/6 |
| camera J 2x4 | `43c863f9,c295504e,c295c15d,43e0b131,4379e614,433391d6,00000000,00000000` | same | 8/8 |
| homogeneous Jpp 4x3 | `3fba8d50,be563110,3f710856,00000000,be563110,3fcc6805,3f2db603,00000000,00000000,00000000,00000000,3f800000` | same | 12/12 |
| raw Jp 2x3 | `4450c412,c206f0e0,c2075714,4455c63b,00000000,00000000` | `4450c413,c206f0e4,c2075714,4455c63b,00000000,00000000` | 4/6 |

With the exact weight, clean weighted Jp is
`44d0c412,c286f0e0,c2875714,44d5c63b,00000000,00000000`; current is
`44d0c413,c286f0e4,c2875714,44d5c63b,00000000,00000000`. The first mismatch
is lane 0 (+1 ULP), followed by lane 1 (+4 ULP). The m7fl detail's weighted
`jl`, deweighted in binary32 by 2.0, reproduces the current raw lanes.

## Diagnosis

Every boundary through camera J and homogeneous Jpp is exact. The mismatch is
therefore isolated to the fixed-size Eigen-shaped `2x4 * 4x3` landmark product
inside `eigen_landmark_jacobian_f32`, not to transform construction,
stereographic unprojection, Double-Sphere projection/Jacobian, residual
subtraction, camera choice, or weighting.

The current safe scalar spelling reduces `(k0,k1)` and `(k2,k3)` pairs with
FMA and then adds the pair results. The clean `linearizePoint` value is emitted
by Eigen's in-context fixed-size expression; even a standalone direct Eigen
product with the same exact camera-J/Jpp inputs has a different packet schedule.
This is a grouping frontier only. No production arithmetic or test source was
changed, and no candidate is retained.

## Hygiene

The external clean probe was temporary and was removed after capture. The clean
checkout/binary was read-only. The existing M7fl focused baseline remains the
green reference; no focused gate was modified or made red.
