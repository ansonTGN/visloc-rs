# M7BY inline frame-0 stereo triangulation report

Date: 2026-08-23 (JST)  
Scope: pinned Basalt `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`, frame 0,
track 1, host camera 0 and same-time candidate camera 1.

## Result

No production source was changed. In particular, `update.rs`, `patch.rs`,
`aom.rs`, `estimator.rs`, `landmarks.rs`, and `camera.rs` were not patched;
there was no commit or push.

The known camera/pixel and standalone triangulation path closes bit-for-bit,
but the authoritative fresh5 artifact does not expose the actual inlined
triangulation operands. Consequently the first *unresolved* boundary is the
actual call-site `T_0_1`/composition context, not a proven error in the
camera or DLT implementation. A general production patch is not justified.

## Upstream operation order

The pinned upstream call in `sqrt_keypoint_vio.cpp` is, in order:

1. Cast the two keypoints to `float` pixels.
2. Call each camera's `calib.intrinsics[].unproject`.
3. Form `T_i0_i1 = pose(i0).inverse() * pose(i1)`.
4. Form `T_0_1 = calib.T_i_c[0].inverse() * T_i0_i1 * calib.T_i_c[candidate_cam]`.
5. Call `BundleAdjustmentBase<float>::triangulate(f0, f1, T_0_1)`.

The pinned `triangulate` implementation uses `P1 = I`,
`P2 = T_0_1.inverse().matrix3x4()`, the four standard DLT rows, Eigen
`JacobiSVD<Matrix4f>(A, ComputeFullV)`, `matrixV().col(3)`, homogeneous
normalization by the first three coordinates' norm, and then the positive-ray
sign test. The Rust standalone fixture follows this same order.

## Track-1 operands that are known

The frame-0 endpoint is exact in the authoritative and current Rust detail
artifacts. The pixel values and raw DS rays below are the exact `float` bit
patterns captured in the existing standalone fixture and in the diagnostic
call replay:

| quantity | value / IEEE-754 `u32` bits |
|---|---|
| host pixel `(u,v)` | `(21, 93)` = `41a80000, 42ba0000` |
| candidate pixel `(u,v)` | `(29.425615310668945, 107.3565444946289)` = `41eb67a9, 42d6b68d` |
| host ray `(x,y,z)` | `(-0.66581767797470093, -0.30268716812133789, 0.68195855617523193)` = `bf2a7307, be9af9d0, 3f2e94d6` |
| candidate ray `(x,y,z)` | `(-0.67596513032913208, -0.28791418671607971, 0.67836326360702515)` = `bf2d0c0d, be93697d, 3f2da937` |

For those rays, the current Rust standalone fixture gives the following
relative camera packet, DLT matrix, raw SVD vector, and output. These values
also appeared in the temporary diagnostic replay of the upstream function:

```text
T_0_1 qxyzw = 3befaa96 39eadb91 3a8bfffa 3f7ffe35
T_0_1 t     = 3de1c150 b87a2400 39ac1888

P2 row-major =
  3f7fffd3 3b0c6cef ba66c162 bde1c0f0
  bb0b910f 3f7ff8d7 3c6facec 3997d306
  3a6ef276 bc6fa4e5 3f7ff8f6 b9e136c3

A row-major =
  bf2e94d6 80000000 bf2a7307 80000000
  80000000 bf2e94d6 be9af9d0 80000000
  bf2dd17a 3c0a2d1b bf2ce029 3d99bcd8
  3a9af4a4 bf2c905f be987a21 b898995a

V.col(3)          = bf28a750 be994a0a 3f2cbdf9 3e147902
world (normalized) = bf2a746e be9aed26 3f2e9645 3e160ef3
stereo direction   = becaaef7 be383810
rho                = 3e160ef3
```

This is a **known-operand standalone closure**, not proof that the same
values were present in the uninstrumented authoritative inlined call. In
particular, the authoritative actual `T_0_1`, `A`, raw `V`, and normalized
world point were not captured.

## Authoritative divergence

The frame-0 endpoint pixels are exact, while the final landmark is not:

| output | current Rust | authoritative upstream |
|---|---|---|
| direction x bits | `becaaef7` | `becaaef6` |
| direction y bits | `be383810` | `be38380d` |
| rho bits | `3e160ef3` | `3e160f08` |
| direction values | `(-0.39586612582206726, -0.1799013614654541)` | `(-0.3958660960197449, -0.17990131676197052)` |
| rho value | `0.14654140174388885` | `0.14654171466827393` |

The current standalone Rust fixture and the diagnostic upstream replay agree
through `A`, `V.col(3)`, normalization, stereographic projection, and `rho`.
The replay therefore rules out a generic DLT-row/SVD/normalization-order fix
for the known operands. It does **not** identify the authoritative actual
call's first differing scalar: the missing observation is the inlined pose
composition packet entering the actual call.

This is consistent with the earlier relative-pose provenance work: the same
source-level relative-pose expression has produced different low-bit packets
between a standalone call and an inlined upstream call when the compiler
register/inlining context changes. The temporary logging experiment here
likewise changed that context and emitted the standalone packet above; it is
diagnostic only and must not be treated as authoritative actual-call data.
The external pinned source was restored after that experiment.

## Exact next probe

The next bounded probe should use the **uninstrumented** pinned binary and a
symbol/offset breakpoint, so logging cannot change the inlining/codegen:

```text
BundleAdjustmentBase<float>::triangulate<...> entry:
    libbasalt.so + 0x4d7400

measure<float> call site:
    libbasalt.so + 0x50b668   (measure<float> starts at +0x50a7b0)
    argument setup immediately before call: +0x50b613

normalization sequence inside triangulate:
    libbasalt.so + 0x4d7f3c  (load V/world vector and begin norm)
    libbasalt.so + 0x4d7f78  (after norm/div, before positive-ray sign)
    libbasalt.so + 0x4d7fbb  (post-sign result/return path)
```

At the entry breakpoint, identify the first call whose pixel pair is
`(41a80000,42ba0000)` / `(41eb67a9,42d6b68d)` and dump the SysV arguments
(`$rsi`, `$rdx`, `$rcx`) before stepping: they are the two ray block objects
and the `T_0_1` reference (the sret result is in `$rdi`). Then dump the
actual `T_0_1` packet and the stack-resident DLT/SVD work at the three internal
breakpoints above. Compare, in order, `T_0_1`, `P2`, `A`, `V.col(3)`, norm,
sign, and final world point against the known fixture. This is the exact next
boundary needed to distinguish pose-composition codegen from any later Eigen
operation.

No further GDB/build/run was performed for this bounded handoff.

## Verification state

The existing M7 focused fixture/test evidence is recorded in
`m7bb_norm_packet_report.md` and the related M7 reports. This report adds no
production test or source fixture because the authoritative actual-call
operand packet is still missing; adding a fixture from the codegen-perturbed
replay would encode the wrong boundary.
