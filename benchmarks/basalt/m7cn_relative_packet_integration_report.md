# M7cn relative-packet integration decision

Date: 2026-08-23 JST  
Status: **not integrated; production unchanged**

## Decision

The captured packet traces are sufficient to state the native operation
trees, but not sufficient to promote a replacement `F32Pose` composition
routine in `pipelines/basalt/src/vio/aom.rs`.  The production file therefore
remains unchanged.  In particular, no camera-pair or track-specific path,
fixture correction, `unsafe`, or debug instrumentation was added.

The authoritative m7bo factor is:

```text
t     = bde1e255,baf1ec60,ba884368
q_xyzw= bc0e2957,bb507f02,ba002d40,3f7ffd31
point = bf2fc73e,be94aaa8,3f2ec6b2
pixel = 41da6d17,42d70d32
raw   = bc22d800,3f915440
```

The current Rust chain still produces the previously recorded local values
(`t=bde1e254,baf1ecc0,ba884364`,
`q=bc0e2956,bb507eff,ba002940,3f7ffd31`,
`point=bf2fc73d,be94aaaa,3f2ec6b2`,
`pixel=41da6d22,42d70d2e`, and `raw=bc228000,3f915340`).  Since those are
not full m7bo parity, the integration gate is not met.

## General expressions recovered from the captures

All quaternion arrays below use Eigen's `[x,y,z,w]` lane order.  The
`fma(a,b,c)` notation means one fused scalar multiply-add; `mul` is a plain
elementwise multiply.  The native normalization is also part of the
contract: square each lane, reduce as

```text
pair = [s0+s2, s1+s3, s2+s2, s3+s3]
norm2 = pair[0] + pair[1] + xmm7[0]
scale = sqrt(norm2)
q_norm = q_raw / broadcast(scale)
```

For the frame-0/frame-1 IMU product (`0x2e6208..0x2e62c4`), with input
packets `a` and `b`, the decoded packet expression is:

```text
x0  = mul(perm(a,0x24), perm(b,0x3f))
x12 = fma(perm(a,0xff), b, -x0)
x2  = fma(perm(a,0xff), b,  x0)
x2[3] = x12[3]
x0  = fma(-perm(a,0x49), perm(b,0x52), x2)
x2  = fma( perm(b,0x52), perm(a,0x49), x2)
x2[3] = x0[3]
qraw = fma(-perm(b,0x89), perm(a,0x92), x2)
```

This is the block reproduced by m7br and gives the normalized intermediate
quaternion `3b322ebb,bab5f931,3a19f84f,3f7fffaf` for the captured pair.

The first camera-prefix product (`0x2e642e..0x2e64dd`) is **not the same
expression tree**.  If `c` is the inverse target-camera quaternion and `p`
is the already-normalized IMU-relative quaternion, the packet trace decodes
to:

```text
x8  = c.w * p                         // broadcast scalar, plain vmulps
x5  = fma(-perm(p,0x3f), perm(c,0x24), x8)
x8  = fma( perm(p,0x3f), perm(c,0x24), x8)
x8[3] = x5[3]
x2  = fma(-perm(p,0x52), perm(c,0x49), x8)
x8  = fma( perm(p,0x52), perm(c,0x49), x8)
x8[3] = x2[3]
qraw = fma(-perm(p,0x89), perm(c,0x92), x8)
```

The captured normalized first-camera packet is
`3b577913,bc82408f,bf33b736,3f36442d`; the existing Rust chain is one ULP
lower in its first three lanes (`3b577912,bc82408e,bf33b735`).  A single
generic `Quaternion * Quaternion` helper cannot claim this boundary without
encoding the call-site-specific packet tree.

The final host-extrinsic product (`0x2e65b0..0x2e6640`) uses yet another
register schedule.  With `a` equal to the normalized first-camera packet and
`b` equal to the host extrinsic, its packet portion is:

```text
x1  = mul(perm(a,0x24), perm(b,0x3f))
x11 = fma(b, perm(a,0xff), -x1)
x8  = fma(b, perm(a,0xff),  x1)
x8[3] = x11[3]
x1  = fma(-perm(b,0x52), perm(a,0x49), x8)
x6  = fma( perm(a,0x49), perm(b,0x52), x8)
x6[3] = x1[3]
qraw = fma(-perm(b,0x89), perm(a,0x92), x6)
```

Its captured normalized packet is
`bc0e2958,bb507f04,ba002d41,3f7ffd33`; the final stored packet is
`bc0e2957,bb507f02,ba002d40,3f7ffd31` after the surrounding call-site
state/translation work.  This confirms the full m7cj endpoint, but it is a
single runtime tuple, not an actual-call test over arbitrary pose and
extrinsic inputs.

## Precise missing production expression

The remaining unproven expression is the complete input-generic camera
composition, including the scalar translation boundary:

```text
T = (R_c, t_c) * (R_p, t_p) * (R_h, t_h)
q_c = normalize(packet_camera_prefix(R_c, R_p))
q   = normalize(packet_suffix(q_c, R_h))
t_p' = t_c + rotate_with_the_native_scalar_FMA_order(R_c, t_p)
t   = t_p' + rotate_with_the_native_scalar_FMA_order(q_c, t_h)
```

The native final stores are explicitly scalar `addss` operations
(`xmm5+xmm12`, `xmm3+xmm0`, `xmm4+xmm2` at `0x2e6644..0x2e6650`).  The
available capture records only the final lanes
`bde1e255,baf1ec60,ba884368`; it does not provide a general runtime lane
trace for every scalar rotation intermediate.  The existing
`sophus_rotate_f32` spelling and one generic quaternion helper therefore do
not prove this call-site association.  Updating them based only on the
m7cj tuple could fix the fixture while silently changing other factors.

## Source reports and verification scope

- [m7bo provenance](m7bo_relative_oracle_provenance_report.md) establishes
  the authoritative transform and downstream point/projection/residual.
- [m7br packet-4 trace](m7br_relative_packet4_report.md) proves the IMU
  packet and pairwise normalization for its captured operands.
- [m7bu camera frontier](m7bu_relative_camera_packet_report.md) identifies
  the camera-prefix and translation boundaries.
- [m7cf hit identity](m7cf_relative_hit_identity_report.md) binds the
  packet to host cam0 → target cam1 rather than the opposite-camera hit.
- [m7cj correct-hit capture](m7cj_relative_correct_hit_packet_report.md)
  captures the final packet and translation lanes for the exact m7bo factor.

No M7/full production test was rerun because this decision leaves production
source unchanged.  The existing M7 tests continue to exercise the retained
local `F32Pose` contract; they are not changed to encode a fixture-specific
replacement.
