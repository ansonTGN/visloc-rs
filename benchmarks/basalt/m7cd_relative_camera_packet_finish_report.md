# M7cd relative camera packet finish

Date: 2026-08-23 JST  
Status: bounded diagnostic capture complete; production unchanged.

## Decision

The retained native packet script was rerun against the existing, pinned
`basalt_vio` binary.  `info registers` was used at the missing boundaries so
the capture completed through the translation stores.  The pose-only filter
does not identify the m7bo host-cam0 → target-cam1 factor: it selected the
opposite camera ID.  Consequently the captured final packet is not the
authoritative m7bo relative transform and no `F32Pose`/`aom.rs` integration is
justified.  Production source, fixtures, and tests remain unchanged.

## Completed packet capture (opposite-camera call)

The capture is retained at `target/m7cd_camera_packet.out`.  At the first
camera packet (`0x2e6434`), `xmm2` matched the pinned IMU-relative quaternion,
but `xmm4` was the cam0 inverse packet:

```text
xmm2 = 3b322ebb,bab5f931,3a19f84f,3f7fffaf
xmm4 = 3bed3c0f,bbf71cd4, bf33a827,3f365a1e
```

The requested missing lanes were nevertheless captured through the end of
this call:

```text
after 0x2e6453 / breakpoint 0x2e6458, xmm8
  3c1658df,bc0bc22a,bf338c83,3f365b2e

after first-packet normalization / breakpoint 0x2e64dd, xmm8
  3c064fb2,bc2b16ee,bf338bd4,3f3672ed

before 0x2e65fb, xmm6
  bbd9de25,3b4585e4,3a268c79,3f01fefb

after 0x2e65fb / breakpoint 0x2e6600, raw xmm6
  bab2380e,bb3381e5,3a12baed,3f7fffaf

after 0x2e6640 / breakpoint 0x2e6644, normalized xmm6
  bab2380e,bb3381e5,3a12baed,3f7fffaf

after final adds / breakpoint 0x2e6650
  xmm5 = b87a5800,bf33a827,3f365a1e,3f365a1e
  xmm0 = bb0b5948,00000000,00000000,00000000
  xmm2 = ba28137c,bba86617,00000000,00000000
  xmm6 = bab2380e,bb3381e5,3a12baed,3f7fffaf
```

These final lanes do not equal the m7bo authoritative result:

```text
t      = bde1e255,baf1ec60,ba884368
q_xyzw = bc0e2957,bb507f02,ba002d40,3f7ffd31
point  = bf2fc73e,be94aaa8,3f2ec6b2
pixel  = 41da6d17,42d70d32
raw    = bc22d800,3f915440
```

## Exact bounded failure boundary

The camera-aware follow-up script is `target/m7cd_camera_pair.cmd`.  It
filters at `0x2e6434` for the desired cam1 inverse packet:

```text
3b19188b,bc55012e,bf33d4ed,3f362af6
```

No matching packet was emitted during the bounded five-frame run, so the
exact unresolved boundary is the camera-ID filter at breakpoint `0x2e6434`.
The prior packet run must not be treated as an actual-call proof for the m7bo
factor.

No production integration, exact actual-call test, benchmark, fixture
correction, unsafe code, or debug path was added.  All GDB/native processes
started for this capture were stopped after the bounded runs.
