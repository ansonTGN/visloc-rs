# M7bu relative camera packet frontier

Date: 2026-08-23 JST  
Status: partial native capture; production unchanged.

## Decision

The already-proven m7br block at `0x2e6208`–`0x2e62c4` produces the exact
normalized IMU-relative quaternion.  The next inlined composition is not
promoted to Rust: the native capture reached the first camera-extrinsic
packet through `0x2e6453`, but the next lane (`xmm8` after `0x2e6453`, at the
`0x2e6458` breakpoint) was not emitted before the long GDB run was
interrupted.  Consequently the later `xmm6` packet at `0x2e65fb`/`0x2e6640`
and the final translation adds at `0x2e6644`–`0x2e6650` remain unvalidated.

There is no production `F32Pose` change and no general quaternion-composition
helper was added.  The exact next missing lane is:

```text
first camera packet: xmm8 after 0x2e6453 (breakpoint 0x2e6458)
```

The downstream missing lanes are:

```text
second camera packet: xmm6 after 0x2e65fb (raw), after 0x2e6640 (normalized)
final translation: xmm5/xmm0/xmm2 after 0x2e6644/0x2e6648/0x2e664c
```

## Authoritative pair and composition topology

The target pair is the m7bo frame-0/frame-1 call, with Eigen quaternion lane
order `[x,y,z,w]`:

```text
host q   = bd582e43,bf4d686f,00000000,3f182ffd
target q = bd5cdea6,bf4d341f,bb2aa78b,3f186f67
cam0 q   = bbed3c0f,3bf71cd4,3f33a827,3f365a1e
cam1 q   = bb19188b,3c55012e,3f33d4ed,3f362af6
```

The native call-site sequence is:

```text
0x2e6208..0x2e62c4  inverse target IMU q × host IMU q; normalize xmm2
0x2e62c4..0x2e6433  relative IMU translation scalar work
0x2e642e..0x2e64dd  first camera-extrinsic Packet4f product; normalize xmm8
0x2e65b0..0x2e6640  second camera-extrinsic Packet4f product; normalize xmm6
0x2e6644..0x2e6650  final translation scalar adds and stores
```

m7bo's completed native result (not reproduced by this partial capture) is:

```text
relative t     = bde1e255,baf1ec60,ba884368
relative q     = bc0e2957,bb507f02,ba002d40,3f7ffd31
target point   = bf2fc73e,be94aaa8,3f2ec6b2
projection     = 41da6d17,42d70d32
raw residual   = bc22d800,3f915440
```

## First camera packet: captured native lanes

`target/m7bu_gdb.out` contains the external GDB capture.  The target filter
was verified at `0x2e6208`: `rdx` pointed at the host quaternion and `r10`
pointed at the target quaternion.  The capture then reached the camera-0
packet.  The values below are the captured f32 lane bits.

| point | register/state | captured lanes |
|---|---|---|
| `0x2e6434` pre-`vmulps` | `xmm2` | `3b322ebb,bab5f931,3a19f84f,3f7fffaf` |
| `0x2e6434` pre-`vmulps` | `xmm13` | `3f7fffaf,3f7fffaf,3f7fffaf,3b322ebb` |
| `0x2e6434` pre-`vmulps` | `xmm11` | `3a19f84f,3b322ebb,bab5f931,bab5f931` |
| `0x2e6434` pre-`vmulps` | `xmm4` (inverse cam0 q) | `3bed3c0f,bbf71cd4,bf33a827,3f365a1e` |
| `0x2e6434` pre-`vmulps` | `xmm12` | `3bed3c0f,bbf71cd4,bf33a827,3bed3c0f` |
| `0x2e6434` pre-`vmulps` | `xmm14` | `bf33a827,3bed3c0f,bbf71cd4,bf33a827` |
| `0x2e6438` post `vmulps` | `xmm8` | `3afdd7e5,ba819f38,39db5977,3f3659e4` |
| `0x2e6440` post copies | `xmm5` | `3f7fffaf,3f7fffaf,3f7fffaf,3b322ebb` |
| `0x2e6440` post copies | `xmm2` | `3a19f84f,3b322ebb,bab5f931,bab5f931` |
| `0x2e6440` post copies | `xmm8` | `3afdd7e5,ba819f38,39db5977,3f3659e4` |
| `0x2e644e` packet input | `xmm4` (broadcast w) | `3f365a1e,3f365a1e,3f365a1e,3f365a1e` |
| `0x2e644e` packet input | `xmm10` | `bbf71cd4,bf33a827,3bed3c0f,bbf71cd4` |
| `0x2e644e` packet input | `xmm14` | `bf33a827,3bed3c0f,bbf71cd4,bf33a827` |
| `0x2e644e` packet input | `xmm12` | `3bed3c0f,bbf71cd4,bf33a827,3bed3c0f` |
| `0x2e644e` packet input | `xmm13` | `3f7fffaf,3f7fffaf,3f7fffaf,3b322ebb` |
| `0x2e644e` packet input | `xmm11` | `3a19f84f,3b322ebb,bab5f931,bab5f931` |
| `0x2e644e` packet input | `xmm8` | `3afdd7e5,ba819f38,39db5977,3f3659e4` |
| `0x2e644e` packet input | `xmm5` | `3f7fffaf,3f7fffaf,3f7fffaf,3b322ebb` |
| `0x2e644e` packet input | `xmm2` | `3a19f84f,3b322ebb,bab5f931,bab5f931` |
| `0x2e6453` post `vfnmadd132ps` | `xmm5` | `bbadc5cb,3bd6b4b8,3f33c359,3f36589a` |

The next printed label was the `0x2e6458` breakpoint, but the process was
interrupted before its `xmm8` dump.  No native bits are inferred for that
lane.

The first packet's relevant instructions are:

```text
2e642e vpermilps  $0xff,%xmm4,%xmm8
2e6434 vmulps     %xmm2,%xmm8,%xmm8
2e6438 vmovaps    %xmm13,%xmm5
2e643c vmovaps    %xmm11,%xmm2
2e6440 vshufps    $0xff,%xmm4,%xmm4,%xmm4
2e644e vfnmadd132ps %xmm12,%xmm8,%xmm5
2e6453 vfmadd231ps  %xmm12,%xmm13,%xmm8
2e646e vblendps   $0x8,%xmm5,%xmm8,%xmm8
2e6474 vfnmadd132ps %xmm10,%xmm8,%xmm2
2e6479 vfmadd231ps  %xmm10,%xmm11,%xmm8
2e648b vblendps   $0x8,%xmm2,%xmm8,%xmm8
2e6491 vfnmadd231ps -0x390(%rbp),%xmm14,%xmm8
2e649a vmulps     %xmm8,%xmm8,%xmm2
...
2e64d9 vdivps     %xmm2,%xmm8,%xmm8
```

## Second packet and translation boundary

The second packet starts with:

```text
2e65b0 vmovups    (%rdi),%xmm8
2e65b8 vpermilps  $0x3f,%xmm8,%xmm11
2e65be vmulps     %xmm1,%xmm11,%xmm1
2e65c7 vpermilps  $0x52,%xmm8,%xmm6
2e65d1 vpermilps  $0x89,%xmm8,%xmm9
2e65d7 vfmsub132ps %xmm10,%xmm1,%xmm11
2e65dc vfmadd132ps %xmm10,%xmm1,%xmm8
2e65e5 vblendps   $0x8,%xmm11,%xmm8,%xmm8
2e65eb vfnmadd132ps %xmm14,%xmm8,%xmm1
2e65f0 vfmadd132ps %xmm14,%xmm8,%xmm6
2e65f5 vblendps   $0x8,%xmm1,%xmm6,%xmm6
2e65fb vfnmadd231ps %xmm9,%xmm15,%xmm6
2e6600 vmulps     %xmm6,%xmm6,%xmm1
...
2e6640 vdivps     %xmm1,%xmm6,%xmm6
```

No lane from this block was accepted as authoritative in m7bu.  The final
translation path is scalar after packet normalization:

```text
2e6644 vaddss %xmm5,%xmm12,%xmm5
2e6648 vaddss %xmm3,%xmm0,%xmm0
2e664c vaddss %xmm4,%xmm2,%xmm2
2e6650 vmovss %xmm5,-0x240(%rbp)
2e6658 vmovss %xmm0,-0x23c(%rbp)
2e6660 vmovss %xmm2,-0x238(%rbp)
```

These final `xmm5/xmm0/xmm2` lanes were not captured, so no target point,
projection, or residual comparison can justify production integration.

## Attempts and retained diagnostics

The following diagnostic artifacts remain under ignored `target/` paths:

```text
target/m7bu_gdb.cmd          multi-breakpoint native capture script
target/m7bu_gdb.out          partial capture through the 0x2e6458 boundary
target/m7bu_probe6208.cmd    pointer/target-pair probe script
target/m7bu_probe6208.out    target-pair hit and runtime operand probe
```

The first capture attempt lacked a GDB `file` command.  The next attempt used
only two frames and therefore never reached the m7bo linearization pair.  A
following attempt planted a breakpoint in the middle of the `0x2e6434`
instruction (`0x2e643a`) and stopped with SIGILL; that address was corrected
to the real `0x2e6438` instruction boundary.  The final max-five run reached
the first camera packet but was interrupted after the `0x2e6458` label to
avoid an unbounded native/GDB run.  No additional build, full replay, or
production test was performed for m7bu.

No frontend, estimator, landmark, provenance, HANDOFF, or work artifact was
modified.  No commit or push was performed.
