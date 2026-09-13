# M7ex ordinal-38 projection frontier

Date: 2026-08-23 JST  
Status: **Completed — read-only factor probe; no production arithmetic edit.**

## Selection

This is the first native/Rust projection mismatch from M7ei: native ordinal
38, track 18, host frame 0/cam0 to target frame 4/cam1, iteration-start
factor evaluation. The authoritative native row is in
[`target/m7ef_clean_visual_all.json`](../../target/m7ef_clean_visual_all.json);
the current M7ev observation is in line 1 of
[`target/m7ev_bearing_jac_exact_fresh5_detail.jsonl`](../../target/m7ev_bearing_jac_exact_fresh5_detail.jsonl).

Inputs are exact: pixel `42f1f732,422fb6ee`, bearing
`be8ebf80,be6ab112`, and inverse distance `3e1d5fdc`.

## Runtime frontier

The retained M7ep pose replay supplies the current runtime `T_t_h` and
homogeneous target point. The disposable M7ex scalar replay copied the
current `project_double_sphere_with_jacobian_f32` operation order and used
camera 1's f32 calibration. Its final pixel exactly matches the current M7ev
detail record (`42f25158,42403505`), confirming that the captured scalar
chain is the live path.

| stage | current f32 bits |
|---|---|
| `T_t_h` (Eigen column-major, 16 lanes) | `3f7ffa86 3b3e17c4 3c4e6ae4 00000000 bb462acc 3f7ffc92 3c20171f 00000000 bc4df140 bc20b37b 3f7ff7ab 00000000 bde17018 bce785d8 baa35d96 3f800000` |
| target point4 `(x,y,z,rho)` | `bf04c770 bed67432 3f425028 3e1d5fdc` |
| `r2` | `3ee38fca` |
| `d1` | `3f814fa6` |
| `k` | `3f0b3973` |
| `d2` | `3f5c40bd` |
| `norm` | `3f39f41f` |
| `mx,my` | `bf36cb97,bf139e35` |
| final pixel `(u,v)` | `42f25158,42403505` |

The exact scalar values and camera parameter bits are retained in the
machine-readable artifact
[`target/m7ex_ordinal38_projection_frontier.json`](../../target/m7ex_ordinal38_projection_frontier.json).

## Clean/current comparison

| stage | clean/native | current M7ev | result |
|---|---|---|---|
| `T_t_h[9]` (row 1, col 2) | `bc20b37c` | `bc20b37b` | first overall mismatch, one ULP |
| target point4 | `bf04c770 bed67432 3f425028 3e1d5fdc` | same | 4/4 exact |
| projection `u,v` | `42f25155 424034ff` | `42f25158 42403505` | 0/2 exact; +3/+6 ULP |
| raw residual `u,v` | `3e344600 4083f088` | `3e344c00 4083f0b8` | 0/2 exact |

Thus the first mismatch in the requested T-versus-point audit is `T_t_h`
column-major lane 9 (`-0.009808417409658432` clean versus
`-0.009808416478335857` current). That one-ULP transform difference is
absorbed by the homogeneous product: point4 is bitwise exact. The first
observed projection mismatch is at the final Double-Sphere pixel, with clean
`u,v = (121.15885162353516, 48.051753997802734)` and current
`(121.158875, 48.051777)`. M7ef did not capture native scalar temporaries, so
this probe does not claim an earlier native `r2`/`d1`/`k`/`d2`/`norm` mismatch;
it establishes the current scalar values and the final output boundary.

## Hygiene and verification

The temporary source was compiled with `+avx,+fma`, run once, and matched the
current runtime pixel exactly. It was then removed, together with its probe
binary/PDB. No production source or arithmetic was changed. The artifact
records the temporary probe hash and all source-input hashes.

Focused verification used the existing release M7 suite; no test fixture or
production behavior was edited for this read-only probe.
