# M7br exact `Packet4f` relative-pose block

Date: 2026-08-23 JST  
Status: diagnostic replay complete; production unchanged.

## Decision

The inlined Eigen `Packet4f` quaternion product and normalization at
`LinearizationAbsQR<float,6>::linearizeProblem + 0x708` (file addresses
`0x2e6208`–`0x2e62c4`) has been decoded and replayed with four-element scalar
arrays.  The replay uses `std::fma` for each contracted lane and has no
`unsafe`, raw pointer, target-specific intrinsic, or SIMD dependency.

It reproduces every captured lane in the block, including the normalized
intermediate quaternion:

```text
target_imu_from_anchor_imu q_xyzw =
  3b322ebb,bab5f931,3a19f84f,3f7fffaf
```

This is not sufficient evidence for a general production relative-pose
change.  The later camera-extrinsic packet product at
`0x2e65b0`–`0x2e6640` still lacks a complete runtime lane trace.  In
particular, the missing authoritative lane is the `xmm6` raw/normalized
camera-product packet after `0x2e65fb`/`0x2e6640`.  Therefore this task leaves
the production `F32Pose` path unchanged.  The replay is intentionally kept as
an auditable benchmark diagnostic in
[`m7br_relative_packet4_trace.cpp`](m7br_relative_packet4_trace.cpp).

## Runtime operands

The operands are the first frame-0/frame-1 target pair from the m7bo native
five-frame run.  Quaternion lanes are Eigen's `[x,y,z,w]` order.

```text
host pose   t = 00000000,00000000,00000000
host pose   q = bd582e43,bf4d686f,00000000,3f182ffd
target pose t = 39c8bb60,b8cebae8,bb0527fc
target pose q = bd5cdea6,bf4d341f,bb2aa78b,3f186f67

cam0 t = bc896b48,bd8d2fdc,3ba86617
cam0 q = bbed3c0f,3bf71cd4,3f33a827,3f365a1e
cam1 t = bc76fa76,3d290319,3b4f4832
cam1 q = bb19188b,3c55012e,3f33d4ed,3f362af6
```

At `0x2e6208`, the loaded `xmm0` is the inverse target quaternion
(`3d5cdea6,3f4d341f,3b2aa78b,3f186f67`) and the loaded `xmm4` is the host
quaternion (`bd582e43,bf4d686f,00000000,3f182ffd`).

For comparison, m7bo's authoritative completed camera-relative result is:

```text
t     = bde1e255,baf1ec60,ba884368
q     = bc0e2957,bb507f02,ba002d40,3f7ffd31
point = bf2fc73e,be94aaa8,3f2ec6b2
pixel = 41da6d17,42d70d32
raw residual = bc22d800,3f915440
```

The standalone/local direct path remains
`q=bc0e2956,bb507eff,ba002940,3f7ffd31` and
`t=bde1e254,baf1ecc0,ba884364`; those differing final bits are why the
untraced camera block cannot be replaced by a guessed composition helper.

## Exact native instructions

These are the relevant instructions from the pinned GCC 11.4 `-O3
-march=native` `libbasalt.so`:

```text
2e6208 vmovaps   (%rdx),%xmm4
2e620c vpermilps $0x3f,%xmm4,%xmm12
2e6212 vpermilps $0x52,%xmm4,%xmm8
2e6218 vpermilps $0x89,%xmm4,%xmm9
2e621e vpermilps $0x49,%xmm0,%xmm1
2e6224 vpermilps $0xff,%xmm0,%xmm2
2e622a vpermilps $0x92,%xmm0,%xmm11
2e6230 vunpckhps  %xmm0,%xmm0,%xmm6
2e6234 vmovaps    %xmm0,%xmm3
2e6238 vmovaps    %xmm0,-0x270(%rbp)
2e6240 vshufps    $0x55,%xmm0,%xmm0,%xmm5
2e6245 vshufps    $0xff,%xmm0,%xmm0,%xmm10
2e624a vpermilps  $0x24,%xmm0,%xmm0
2e6250 vmulps     %xmm12,%xmm0,%xmm0
2e6255 vmovaps    %xmm2,%xmm12
2e6259 vfmsub132ps %xmm4,%xmm0,%xmm12
2e625e vfmadd132ps %xmm4,%xmm0,%xmm2
2e6263 vmovaps    %xmm1,%xmm0
2e6267 vblendps   $0x8,%xmm12,%xmm2,%xmm2
2e626d vfnmadd132ps %xmm8,%xmm2,%xmm0
2e6272 vfmadd231ps %xmm8,%xmm1,%xmm2
2e6277 vblendps   $0x8,%xmm0,%xmm2,%xmm2
2e627d vfnmadd231ps %xmm9,%xmm11,%xmm2
2e6282 vmulps     %xmm2,%xmm2,%xmm0
2e6286 vmovaps    %xmm2,-0x2e0(%rbp)
2e628e vmovhlps   %xmm0,%xmm0,%xmm1
2e6292 vaddps     %xmm1,%xmm0,%xmm0
2e6296 vmovshdup  %xmm0,%xmm1
2e629a vaddps     %xmm1,%xmm0,%xmm0
2e629e vaddss     %xmm7,%xmm0,%xmm0
2e62a2 vinsertps  $0xe,%xmm0,%xmm0,%xmm0
2e62a8 vsqrtss    %xmm0,%xmm0,%xmm0
2e62ac vcomiss    ...,%xmm0
2e62b4 jb         ...
2e62ba vpermilps  $0x0,%xmm0,%xmm0
2e62c0 vdivps     %xmm0,%xmm2,%xmm2
```

The compiler's FMA forms are replayed as follows (the destination is the
old destination register):

```text
vfmsub132ps  d = d*a - b
vfmadd132ps  d = d*a + b
vfnmadd132ps d = b - d*a
vfmadd231ps  d = d + a*b
vfnmadd231ps d = d - a*b
```

The `$0x8` blend keeps lanes 0–2 from the destination and replaces lane 3;
the `$0x0` perm broadcasts the scalar square-root lane to all four lanes.

## Native trace versus scalar replay

[`m7br_relative_packet4_native_trace.txt`](m7br_relative_packet4_native_trace.txt)
is the checked-in external GDB trace captured at the exact target pair, with
a breakpoint after each instruction.  The raw capture is also retained as
`target/m7br_gdb.out`.  The table below is the
bitwise comparison used by the replay.  The expected arrays in the replay are
copied from this native trace; every lane in every row is equal, and the
replay program exits nonzero on the first mismatch.

| stage | native register | native lanes (hex f32) | scalar replay |
|---|---|---|---|
| input inverse target | `xmm0` | `3d5cdea6,3f4d341f,3b2aa78b,3f186f67` | same |
| `0x6208` host load | `xmm4` | `bd582e43,bf4d686f,00000000,3f182ffd` | same |
| `0x620c` perm `$3f` | `xmm12` | `3f182ffd,3f182ffd,3f182ffd,bd582e43` | same |
| `0x6212` perm `$52` | `xmm8` | `00000000,bd582e43,bf4d686f,bf4d686f` | same |
| `0x6218` perm `$89` | `xmm9` | `bf4d686f,00000000,bd582e43,00000000` | same |
| `0x621e` perm `$49` | `xmm1` | `3f4d341f,3b2aa78b,3d5cdea6,3f4d341f` | same |
| `0x6224` perm `$ff` | `xmm2` | `3f186f67,3f186f67,3f186f67,3f186f67` | same |
| `0x622a` perm `$92` | `xmm11` | `3b2aa78b,3d5cdea6,3f4d341f,3b2aa78b` | same |
| `0x6230` unpack-high | `xmm6` | `3b2aa78b,3b2aa78b,3f186f67,3f186f67` | same |
| `0x6234` copy | `xmm3` | `3d5cdea6,3f4d341f,3b2aa78b,3f186f67` | same |
| `0x6240` shuffle `$55` | `xmm5` | `3f4d341f,3f4d341f,3f4d341f,3f4d341f` | same |
| `0x6245` shuffle `$ff` | `xmm10` | `3f186f67,3f186f67,3f186f67,3f186f67` | same |
| `0x624a` perm `$24` | `xmm0` | `3d5cdea6,3f4d341f,3b2aa78b,3d5cdea6` | same |
| `0x6250` multiply | `xmm0` | `3d034d9a,3ef3fad4,3acae6f0,bb3a83c6` | same |
| `0x6255` copy | `xmm12` | `3f186f67,3f186f67,3f186f67,3f186f67` | same |
| `0x6259` FMSUB | `xmm12` | `bd820392,bf744ccf,bacae6f0,3eb6b278` | same |
| `0x625e` FMADD | `xmm2` | `3a2503cc,baa3f5aa,3acae6f0,3eb3c869` | same |
| `0x6263` copy | `xmm0` | `3f4d341f,3b2aa78b,3d5cdea6,3f4d341f` | same |
| `0x6267` blend lane 3 | `xmm2` | `3a2503cc,baa3f5aa,3acae6f0,3eb6b278` | same |
| `0x626d` FNMADD | `xmm0` | `3a2503cc,ba91f223,3d378f9d,3f7fffaf` | same |
| `0x6272` FMADD231 | `xmm2` | `3a2503cc,bab5f931,bd2ae12e,be929a6e` | same |
| `0x6277` blend lane 3 | `xmm2` | `3a2503cc,bab5f931,bd2ae12e,3f7fffaf` | same |
| `0x627d` raw product | `xmm2` | `3b322ebb,bab5f931,3a19f84f,3f7fffaf` | same |
| `0x6282` square | `xmm0` | `36f80a09,36015a52,34b9357f,3f7fff5e` | same |
| `0x6286` spill/copy | `xmm2` | `3b322ebb,bab5f931,3a19f84f,3f7fffaf` | same |
| `0x628e` high unpack | `xmm1` | `34b9357f,3f7fff5e,34b9357f,3f7fff5e` | same |
| `0x6292` pair sum | `xmm0` | `3701ceb0,3f7fff7e,3539357f,3fffff5e` | same |
| `0x6296` odd-lane duplicate | `xmm1` | `3f7fff7e,3f7fff7e,3fffff5e,3fffff5e` | same |
| `0x629a` norm² reduction | `xmm0` | `3f800000,3fffff7e,3fffff64,407fff5e` | same |
| `0x629e` scalar add | `xmm0` | `3f800000,3fffff7e,3fffff64,407fff5e` | same |
| `0x62a2` insert scalar | `xmm0` | `3f800000,00000000,00000000,00000000` | same |
| `0x62a8` sqrt | `xmm0` | `3f800000,00000000,00000000,00000000` | same |
| `0x62ba` divisor broadcast | `xmm0` | `3f800000,3f800000,3f800000,3f800000` | same |
| `0x62c0` normalized product | `xmm2` | `3b322ebb,bab5f931,3a19f84f,3f7fffaf` | same |

The trace also records the register copies at `0x6255`, `0x6263`, and
`0x6286`; the replay explicitly models and checks those copies.  The
`0x629e` scalar add is zero for this call (`xmm7` lane 0 is zero), which is
why its packet is unchanged.

## Verification

The diagnostic replay was compiled and run with:

```text
wsl.exe -- bash -lc "g++ -std=c++17 -O3 -march=native -ffp-contract=off \
  -o /tmp/m7br_relative_packet4_trace \
  benchmarks/basalt/m7br_relative_packet4_trace.cpp && \
  /tmp/m7br_relative_packet4_trace"
```

All expected-lane assertions passed.  The repository checks also passed:

```text
cargo test -p visloc-basalt --lib \
  m7_relative_pose_chain_matches_pinned_lanes -- --nocapture
1 passed; 0 failed

cargo test -p visloc-basalt --lib
162 passed; 0 failed; 1 ignored

cargo run --release --example basalt_euroc_vio_demo -- \
  --euroc-dir E:\\datasets\\euroc_mav\\machine_hall\\MH_01_easy \
  --calibration target\\euroc_ds_calib.json \
  --config target\\euroc_config.json \
  --out-dir target\\m7br_packet4_fresh5 --max-frames 5
sensor_only=true, frames_processed=5, observations_emitted=1114
```

No production Rust file, fixture, estimator, frontend, landmark, or
provenance path was changed by m7br.
