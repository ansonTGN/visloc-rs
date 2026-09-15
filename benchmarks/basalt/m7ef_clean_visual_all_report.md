# M7ef clean native visual `linearizePoint` all-call capture

Date: 2026-08-23 JST  
Status: **Pass — bounded clean GDB capture reached the expected 584th
observation with complete point/projection/residual/Jacobian coverage.**

## Scope and stop condition

This is one read-only GDB run against the pinned clean Basalt artifact. The
capture arms only at the exact first `LinearizationAbsQR<float,6>::linearizeProblem`
entry whose frame-state map count is five (the frame-4 iteration-start gate).
It records every subsequent `DoubleSphere<float>` `linearizePoint` call by
sequential ordinal, then stops at the completed 584th call's Jacobian write.
The normal `linearizeProblem` return was not needed because the expected 584
observation bound was reached first.

The machine-readable record is
[`target/m7ef_clean_visual_all.json`](../../target/m7ef_clean_visual_all.json).
Each row contains the raw f32 bit patterns (and decoded f32 values) for:

- observed pixel `(u,v)`;
- keypoint direction `(x,y)` and inverse distance/rho;
- raw Eigen column-major `T_t_h` 4×4;
- final transformed homogeneous target point `(x,y,z,rho)` from `xmm6`;
- projected `(u,v)` before observation subtraction;
- raw residual after subtraction;
- raw `d_res_d_p` 2×3 (6 lanes);
- raw `d_res_d_xi` 2×6 (12 lanes).

The record also retains the ABI pointers, thread number, and ordinal. No
`ptype`, typed expansion, or pretty-printer was used.

## Provenance and clean verification

| item | value |
|---|---|
| clean checkout | `/root/visloc-basalt-clean-m7cr-20260823` |
| Basalt commit | `0f3b2b52c807f70ff4e2973ce253c73329eea7bc` |
| binary | `build/core-relwithdebinfo/basalt_vio` |
| `basalt_vio` SHA-256 | `89e0324ccd04c3b615bf7e945aa32480a2f86524987aa00eaf152dcd6b1d242c` |
| `libbasalt.so` SHA-256 | `55f842642b370edf4025afd1d66c7b9d9011e5b4ce08efeb9d76be401b45b2cc` |
| dataset/config | `MH_01_easy`, four native threads, `--max-frames 5` |
| marginal data | `/marg_data_m7ct` |
| GDB elapsed time | 17.553 s (capture payload timing) |
| post-run processes | no `basalt_vio` or GDB process |

The binary and library are the same hashes retained by M7ct/M7dh. The run
did not edit source, build outputs, the clean checkout, commit history, or
push anything.

## Gate, addresses, and coverage

The live library base was `0x7ffff6e00000`. The active gate saw exactly one
`linearizeProblem` entry with `frame_states` count `5`:

| boundary | ELF offset | live address |
|---|---:|---:|
| `linearizeProblem` entry | `0x2c47f0` | `0x7ffff70c47f0` |
| normal return (fallback bound) | `0x2c5596` | `0x7ffff70c5596` |
| `linearizePoint` entry | `0x275030` | `0x7ffff7075030` |
| target point (`xmm6`) | `0x275158` | `0x7ffff7075158` |
| projected uv | `0x275250` | `0x7ffff7075250` |
| raw residual | `0x275458` | `0x7ffff7075458` |
| Jacobian writes | `0x275774` | `0x7ffff7075774` |

Coverage is complete: **584/584** rows have all requested fields, including
target point, projection, residual, `d_res_d_p` (6/6), and `d_res_d_xi`
(12/12). There are 61 unique keypoint direction/rho tuples and 10 unique
`T_t_h` transforms. The four worker threads contributed 147, 139, 145, and
153 calls; ordinal is the authoritative sequence for later matching.

## Anchor against M7dh

The known M7dh pixel filter appears at ordinal **473** in this gated run. Its
raw values are unchanged and match the retained clean point/Jacobian oracle:

```text
pixel       41da8172,42d4c7e1
direction   becaaef7,be383810     rho 3e160ef3
target      bf2fc73e,be94aaa8,3f2ec6b2,3e160ef3
projection  41da6d17,42d70d32
residual    bc22d800,3f915440
d_res_d_p   44446eae,c1e493b0,c20f4734,445399f4,c21720bd,40df22eb
```

## Artifacts and hashes

- GDB command: [`target/m7ef_clean_visual_all.cmd`](../../target/m7ef_clean_visual_all.cmd), 13,855 bytes, SHA-256 `f117de145d1cd6f6db811c2f5cd8246492bd1d0dd53a2d4471a365e27c0d9d66`.
- Raw GDB output: [`target/m7ef_clean_visual_all.gdb.out`](../../target/m7ef_clean_visual_all.gdb.out), 64,884 bytes, SHA-256 `2d1c6452abeb3ec0997f347528e6368b52ce40f1630bf231b3b272f2b69a90c8`.
- JSON capture: [`target/m7ef_clean_visual_all.json`](../../target/m7ef_clean_visual_all.json), 2,402,181 bytes, SHA-256 `9d8fe21633fd895acf03c26dbb1ba5797c540e03b43acf85850a338d373d85c0`.
