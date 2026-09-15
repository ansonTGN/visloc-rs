# M7fa ordinal-43 same-timestamp stereo Jp frontier

Date: 2026-08-23 JST  
Status: **Complete — bounded read-only capture; production arithmetic unchanged.**

## Selection

The first weighted-landmark-Jp mismatch in M7ez is native ordinal **43**:
track 119, observation 1, host frame 0/cam0 to target frame 0/cam1 at the
same timestamp. The exact key is pixel `442d8e69,43048eb8`, bearing direction
`3eb9ae8e,be15bd5c`, and inverse distance `3d8325d2`. Camera 1 is selected by
the target relation. The machine-readable record is
[`target/m7fa_ordinal43_jp.json`](../../target/m7fa_ordinal43_jp.json).

The clean side came from one external C++ `linearizePoint<float,
DoubleSphereCamera<float>>` run using the existing pinned clean headers and
the ordinal-43 `T_t_h` packet. The current side is the M7ez iteration-start
record plus a disposable standalone replay of the current f32 helper chain.
Temporary probe source, executable, and output were removed after capture.

## Boundary comparison

All entries are f32 bit patterns. Matrices are column-major in the order
declared in the JSON artifact.

| boundary | clean native | current Rust | exact |
|---|---|---|---:|
| `T_t_h` 4x4 | `3f7fffd3,bb0b904f,3a6ef290,00000000,3b0c6c2f,3f7ff8d7,bc6fa4f8,00000000,ba66c186,3c6facff,3f7ff8f6,00000000,bde1c0f0,3997d300,b9e136c8,3f800000` | same except lane 9 `3c6facfe` | 15/16 |
| point4 `(x,y,z,rho)` | `3f1ef180,be7a142b,3f3d2a31,3d8325d2` | `3f1ef180,be7a142c,3f3d2a31,3d8325d2` | 3/4 |
| projection `(u,v)` | `442d8e20,430484d3` | same | 2/2 |
| raw residual `(u,v)` | `bb920000,bd1e5000` | same | 2/2 |
| bearing J 3x2 | `3fab62c3,3e236be3,bf8bb205,3e236be3,3fd5cf9a,3ee14f3c` | same | 6/6 |
| camera J 2x4 | `43b7771b,425bcc35,425c75a2,43f1f4ca,c3910beb,42e38757,00000000,00000000` | `43b7771b,425bcc35,425c75a2,43f1f4c8,c3910beb,42e38759,00000000,00000000` | 6/8 |
| homogeneous Jpp 4x3 | `3fab8d55,3e1022a4,bf8bd2ad,00000000,3e26b079,3fd6916d,3ed4d9b2,00000000,bde1c0f0,3997d300,b9e136c8,3f800000` | same | 12/12 |
| raw Jp 2x3 | `444c1b2f,418be6e8,41fc2b64,4458db64,c2213a01,c0bee881` | `444c1b2f,418be6d8,41fc2b64,4458db63,c2213a01,c0bee881` | 4/6 |

With the exact `sqrt_weight = 2.0`, weighted Jp is clean
`44cc1b2f,420be6e8,427c2b64,44d8db64,c2a13a01,c13ee881` versus current
`44cc1b2f,420be6d8,427c2b64,44d8db63,c2a13a01,c13ee881`. Its first mismatch
is lane 1 (16 ULP); lane 3 differs by 1 ULP. This reproduces the M7ez
comparator's first weighted-Jp result.

## First mismatch and grouping proposal

The first mismatch in the requested transform-to-Jp chain is `T_t_h`
column-major lane 9 (row 1, column 2), clean `3c6facff` versus current
`3c6facfe`. The transformed point then differs at y by one ULP, while
projection and residual remain exact. Bearing J and the homogeneous 4x3 Jpp
are exact, so the mismatch is upstream of the landmark product. The first
weighted-Jp mismatch is consequently lane 1, as above.

The current point path uses `eigen_quaternion_matrix_native_f32`, whose
quaternion cross terms are separately multiplied/subtracted. The pinned
Eigen/Sophus operation grouping uses fused cross-term forms. In particular,
the lane-9 expression should be grouped as:

```text
(-tx).mul_add(q.w, tyz)       // row 1, column 2
```

with corresponding `mul_add` forms for the other off-diagonal quaternion
terms. A disposable isolated replay using this FMA-shaped matrix helper
produced the clean lane-9 transform, clean point4 y, clean camera-J lanes 3
and 5, and clean raw-Jp lanes 1 and 3; projection/raw, bearing J, and Jpp
remained exact. This is a source-grouping proposal only—no production change
was retained.

## Hygiene

The pinned clean binary and checkout were not modified. No Rust production or
test source was changed, and the temporary C++/Rust probe files were removed.
The existing focused M7 baseline was not altered or rerun with a candidate.
