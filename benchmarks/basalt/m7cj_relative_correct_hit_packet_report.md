# M7cj correct-hit relative camera packet capture

Date: 2026-08-23 JST  
Status: capture complete; production unchanged.

## Scope and identity

This is the final bounded capture for the already-proven visual-factor hit.
It reused the uninstrumented native `core-relwithdebinfo/basalt_vio` binary,
the exact `rbx` time/camera filter, and one five-frame run.  The compact
register commands were retained in [`target/m7cj_packet.cmd`](../../target/m7cj_packet.cmd)
and the output is [`target/m7cj_packet.out`](../../target/m7cj_packet.out).
No rebuild, instrumentation, fixture correction, or additional run was used.

The filter selected:

```text
host   = (1403636579763555584, cam 0)
target = (1403636579813555456, cam 1)
track  = 1
pixel  = (27.31320571899414, 106.39038848876953)
pixel bits = 41da8172,42d4c7e1
```

The host/target and track/pixel identity is the m7bo line-57 factor; the
camera-pair filter prevents the earlier target-cam0 pose-only hit.

## Captured packet lanes

All vectors below are four 32-bit lanes in increasing lane order.

| native site | value |
|---|---|
| `0x2e6453`, `xmm5` | `b9d269d4,3c44d134,3f33f018,3f362a51` |
| `0x2e6458`, `xmm8` after the first camera-packet arithmetic | `3b8bf1bd,bc6530a1,bf33b950,3f362b27` |
| `0x2e64dd`, normalized first camera packet, `xmm8` | `3b577913,bc82408f,bf33b736,3f36442d` |
| `0x2e65fb`, second camera packet raw, `xmm6` | `bc64e659,ba651f4c,b9c40058,3f01de0d` |
| `0x2e6640`, second camera packet normalized, `xmm6` | `bc0e2958,bb507f04,ba002d41,3f7ffd33` |
| `0x2e6650`, `xmm5` | `bde1e255,bf33d4ed,3f362af6,3f362af6` |
| `0x2e6650`, `xmm0` | `baf1ec60,00000000,00000000,00000000` |
| `0x2e6650`, `xmm2` | `ba884368,bb4f4832,00000000,00000000` |
| `0x2e6650`, final `xmm6` | `bc0e2957,bb507f02,ba002d40,3f7ffd31` |

The final translation is therefore
`bde1e255,baf1ec60,ba884368`, taking lane zero from `xmm5`, `xmm0`, and
`xmm2`; the final quaternion is `bc0e2957,bb507f02,ba002d40,3f7ffd31`.

## Comparison with m7bo

The captured final pose is bitwise identical to the authoritative m7bo
relative pose.  For this same factor, the m7bo provenance record supplies the
downstream chain:

| quantity | m7bo authoritative bits |
|---|---|
| relative `t` | `bde1e255,baf1ec60,ba884368` |
| relative `q_xyzw` | `bc0e2957,bb507f02,ba002d40,3f7ffd31` |
| target point `(x,y,z)` | `bf2fc73e,be94aaa8,3f2ec6b2` |
| projected `(u,v)` | `41da6d17,42d70d32` |
| raw residual | `bc22d800,3f915440` |

Thus the final captured pose and the authoritative point/projection/residual
record agree exactly.  The prior local `F32Pose` path still produces the
non-authoritative bits documented by m7bo (for example final `t`
`bde1e254,baf1ecc0,ba884364` and final `q`
`bc0e2956,bb507eff,ba002940,3f7ffd31`).

## Production decision

Capture alone does not prove a general, safe packet-order implementation for
`F32Pose`: the native sequence includes inlined Eigen/Sophus lane shuffles,
FMA operand ordering, and normalization boundaries.  A general replacement
would need to reproduce those operations for arbitrary packet order and pass
the actual-call tests, rather than correct this fixture.  That derivation was
not completed here, so `pipelines/basalt/src/vio/aom.rs` and all production
tests remain unchanged.  No unsafe/debug path or fixture-specific correction
was added.

The inferior was killed at the final breakpoint.  The final process check was
empty for `gdb` and `basalt_vio`.  No commit or push was performed.
