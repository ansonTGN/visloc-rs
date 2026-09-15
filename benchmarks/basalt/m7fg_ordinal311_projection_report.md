# M7fg ordinal-311 projection frontier

Date: 2026-08-23 JST  
Status: **Complete read-only probe; temporary probe removed after capture.**

## Result

Native ordinal **311** is track 22, observation 3, frame 0/cam 0 → frame
1/cam 1 (`cross_time`). The current relative pose is exact at all 16
column-major `T_t_h` lanes. The first mismatch is the homogeneous target
point's **y lane**:

| stage | clean/native | current | result |
|---|---|---|---|
| `T_t_h` | `3f7ffea4 ... ba884368 3f800000` | same | 16/16 exact |
| target point `(x,y,z,rho)` | `bf024930,bd344ff6,3f5f77ea,3e59788c` | `bf024930,bd344ff5,3f5f77ea,3e59788c` | y −1 ULP |
| projection `(u,v)` | `4308bb31,436b0c94` | `4308bb31,436b0c95` | v +1 ULP |
| raw residual | `3e255000,3f9f2380` | `3e255000,3f9f2400` | v differs |

## Double-Sphere packet

The same pinned f32 scalar schedule was evaluated on clean and current
target points. The first camera scalar difference is `yy`, caused by the
point-y difference:

| scalar | clean | current |
|---|---|---|
| `xx` | `3e849cd3` | `3e849cd3` |
| `yy` | `3afe0116` | `3afe0113` |
| `r2` | `3e859ad4` | `3e859ad4` |
| `d1_2`, `d1` | `3f82efc6`, `3f8175c1` | same |
| `k`, `d2` | `3f2850f9`, `3f5525b8` | same |
| `norm`, `mx` | `3f422b9f`, `bf2bc5d5` | same |
| `my` | `bd6dbaa9` | `bd6dbaa8` |
| pixel `(u,v)` | `4308bb31,436b0c94` | `4308bb31,436b0c95` |

Thus Double-Sphere grouping is not the first mismatch; `r2` through `norm`
and `mx` round identically. The actionable boundary is the stereographic
bearing / homogeneous 4×4×1 point path. The current source uses the
sequential FMA row fold. A diagnostic input variant changing only bearing-y
from `bd72c8bd` to `bd72c8be` reproduces the clean point, so the remaining
ambiguity is whether the native y packet differs at stereographic unproject
or Eigen's fixed GEMV reduction groups the same f32 inputs differently. The
native capture does not expose bearing scalar temporaries; this is not
claimed as a definitive bearing-vs-GEMV split.

Machine-readable details are in
[`target/m7fg_ordinal311_projection.json`](../../target/m7fg_ordinal311_projection.json).
The disposable source/binary/output were used only for this replay and were
removed; no production or clean artifact was changed.
