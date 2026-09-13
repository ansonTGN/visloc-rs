# M7eo native Jp codegen probe

Date: 2026-08-23 JST

## Result

The isolated C++ probe calls the exact pinned
`basalt::linearizePoint<float, basalt::DoubleSphereCamera<float>>` with the
clean `target/m7ef_clean_visual_all.json` ordinal-0 inputs (identity `T_t_h`,
pixel `388,193`, and the captured keypoint).  Camera 0 is selected by the
projection/residual anchor.  The native raw landmark Jacobian is **6/6 exact**:

```text
native / clean target:
4465f922,3f652460,3f65d1e0,4464cfbf,00000000,00000000
```

This closes the question of the native target value.  The prior interrupted
M7em Rust raw value was:

```text
4465f920,3f652458,3f65d1e8,4464cfbd,00000000,00000000
```

It therefore had only 2/6 exact lanes (zero lanes), with maximum distance 8
ULP; no M7em arithmetic candidate is accepted.

The machine-readable capture is
[`target/m7eo_jp_codegen_probe.json`](../../target/m7eo_jp_codegen_probe.json).
The diagnostic source and native executable are ignored `target/` artifacts;
neither production nor tests were changed.

## Build and provenance

The probe source is `target/m7eo_jp_codegen_probe.cpp`.  It was built in the
WSL pinned clean environment with:

```text
g++ 11.4.0 -std=c++20 -O3 -march=native -ffp-contract=fast
```

Includes were the clean Basalt `ba_utils.h`, pinned Basalt headers from the
oracle checkout, Eigen 5.0.1 (`5.0.1-d487a628b0.clean`), and pinned Sophus.
The executable SHA-256 is
`3ddef2ad80c0816ed185e1c29b903e0520a311920ca7ab72e1a664163335b3a6` and the
probe-source SHA-256 is
`55953cc673d44f4bbd6e0938d71de61715408bfa7d0563a71d977e6a8044f769`.

The native intermediate values also agree with the clean oracle:

| quantity | f32 bits |
|---|---|
| transformed point `(x,y,z,rho)` | `3d43efd6,bdfa0f6e,3f7dca10,3e46b55a` |
| projection `(u,v)` | `43c2000b,43411269` |
| raw residual | `39b00000,3d934800` |
| camera `Jp` (2x4, row-major) | `43e6b4aa,3fe6b2f7,c1b056e5,00000000,3fe604cb,43e4157f,426062c3,00000000` |
| homogeneous `Jpp` (4x3, column-major) | `3ffe9a0c,3bbf6405,bdc31742,00000000,3bbf6405,3ffcfc84,3e78fb07,00000000,00000000,00000000,00000000,3f800000` |

## Code generation / operation order

Disassembly of the final `camera_jacobian * source_jpp` expression at
`run+0x6d0..run+0x8f2` (`0x23e8..0x24b6`) contains:

```text
vpermilps, vshufps, vmulps, vfmadd132ps, vfmadd231ps, vaddps
vmulss, vfmadd132ss, vaddss
```

The first four output lanes are handled as one XMM `Packet4f` packet in
column-major order `[row0-col0,row1-col0,row0-col1,row1-col1]`; the final two
lanes (column 2) are a scalar tail.  This is the Eigen fixed-size coefficient
product path.  The packet partial-product/FMA/add schedule is observably
different from an algebraically collapsed `2x3 * 3x3` product, even though
the fourth homogeneous row contributes zeros for this identity input.

## Minimal implementation direction

No production change is proposed by this probe.  The smallest faithful Rust
candidate must retain the Eigen-shaped `2x4` camera Jacobian and `4x3`
homogeneous landmark Jacobian and reproduce the fixed-size `2x4 * 4x3`
packet/reduction boundary.  Do not drop the homogeneous row or substitute a
generic `2x3 * 3x3` nalgebra product.  The ordinal-0 probe confirms the target,
but a general 6/6 Rust packet harness across independent inputs is still
needed before changing production arithmetic.

