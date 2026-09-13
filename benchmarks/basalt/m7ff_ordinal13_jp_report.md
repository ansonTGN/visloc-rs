# M7ff ordinal-13 weighted-Jp frontier

Date: 2026-08-23 JST  
Status: **Complete — bounded read-only capture; production arithmetic unchanged.**

## Selection

`target/m7fd_visual_all_comparison.json` identifies native ordinal **13** as
the first weighted-Jp mismatch: track **66**, observation **3**, host frame
0/cam0 to target frame 1/cam1 (`cross_time`). The exact key is pixel
`43c261c8,434e83d5`, direction `3cc4c95a,bd7b2508`, and inverse distance
`3e46b55a`; `sqrt_weight` is `3ff41396`.

The machine-readable capture is
[`target/m7ff_ordinal13_jp.json`](../../target/m7ff_ordinal13_jp.json).
The clean side was a temporary external `linearizePoint<float,
DoubleSphereCamera<float>>` replay using the authoritative ordinal-13
`T_t_h`; the current side was a disposable replay of the current M7fd f32
helper chain. Both probes were removed after capture.

## Boundary comparison

All values are f32 bit patterns; matrices are column-major.

| boundary | clean native | current Rust | exact |
|---|---|---|---:|
| `T_t_h` 4x4 | `3f7ffea4,ba71deac,3bd0c3e6,00000000,3a87685a,3f7ff61a,bc8e2141,00000000,bbd0358b,3c8e2e4e,3f7ff4ce,00000000,bde1e255,baf1ec60,ba884368,3f800000` | same | 16/16 |
| point4 `(x,y,z,rho)` | `3ca3e5ae,bdd79bb0,3f7e508c,3e46b55a` | `3ca3e5ae,bdd79baf,3f7e508c,3e46b55a` | 3/4 |
| projection `(u,v)` | `43c2506a,434f9b4e` | same | 2/2 |
| raw residual `(u,v)` | `be0af000,3f8bbc80` | same | 2/2 |
| bearing J 3x2 | `3ffe9a0c,3bbf6405,bdc31742,3bbf6405,3ffcfc84,3e78fb07` | same | 6/6 |
| camera J 2x4 | `43e65c2a,3f241ed8,3f249d5a,43e40aec,c1135e71,4241487a,00000000,00000000` | `43e65c2a,3f241ed7,3f249d59,43e40aec,c1135e71,42414878,00000000,00000000` | 5/8 |
| homogeneous Jpp 4x3 | `3ffeacba,3b1a18d0,bda94fdb,00000000,3bcfab24,3ffd7cd7,3e55dcaa,00000000,bde1e255,baf1ec60,ba884368,3f800000` | same | 12/12 |
| raw Jp 2x3 | `44655bb4,bfd2ca8c,401141cf,44645423,c24b3a14,bf76770c` | `44655bb4,bfd2ca86,401141cf,44645423,c24b3a14,bf76770a` | 4/6 |

With the exact `sqrt_weight`, weighted Jp is clean
`44daacf4,c048f92e,408a7dd6,44d9b1a9,c2c1c2e7,bfeafc53` versus current
`44daacf4,c048f928,408a7dd6,44d9b1a9,c2c1c2e7,bfeafc51`.

## First mismatch and source grouping

The earliest mismatch in the requested chain is target-point **y**, one ULP
lower on the current side, even though all 16 `T_t_h` lanes are exact. The
projection and raw residual stay exact. Camera-J then differs in active lanes
1, 2, and 5, and the first weighted-Jp mismatch is lane 1 (row 1, column 0),
matching the M7fd comparator.

Bearing J, `T_t_h`, and Jpp are exact, so the mismatch is not in stereographic
unprojection, relative-pose materialization, or the homogeneous landmark
direction block. The exact source grouping to audit is the point boundary:
current `sophus_homogeneous_point_normalized_rotation_f32` accumulates three
rotation terms and then fuses translation*rho, while clean Basalt evaluates
the source-shaped Eigen fixed-size `T_t_h * [bearing; rho]` product with its
four-term packet/pair reduction (including the homogeneous fourth lane).
The camera-J differences are downstream of that point boundary and its
Double-Sphere derivative schedule; no Jpp or Jp product change is indicated.

## Hygiene and verification

No production or test source was modified, and no focused test was changed or
made red. Temporary C++/Rust probe source, executable, and output were removed
after capture. The clean checkout/binary were read-only.
