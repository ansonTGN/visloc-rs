# M7cf relative-hit identity and camera provenance

Date: 2026-08-23 JST  
Status: call-site identity resolved; packet retry bounded at `0x2e6458`; no production change.

## Decision

The m7bo visual factor is the unique relative-pose map entry whose live key at
`0x2e6208` is:

```text
host   frame=1403636579763555584 (0x137ab808528d0d00), cam=0
target frame=1403636579813555456 (0x137ab8085587fd00), cam=1
```

This was read from the uninstrumented binary in `target/m7cf_identity.out`,
hit 2, at `rbx+0x20..0x38`:

```text
0x137ab808528d0d00  0x0000000000000000
0x137ab8085587fd00  0x0000000000000001
```

Hit 1 had the same host key but target camera `0`, which explains the earlier
m7cd packet: a pose-only filter cannot distinguish the two camera factors.
The live target-map node for the accepted hit has the target key
`0x137ab8085587fd00/1` and a 61-entry keypoint-ID set (`+0x58 == 0x3d`).
The retained authoritative visual-factor record (`m7bo`, iteration JSONL line
57) resolves that set member as track `1`, with observation pixel
`(27.31320571899414, 106.39038848876953)` (`41da8172,42d4c7e1`).  Its
authoritative raw residual is `bc22d800,3f915440`.

The pixel is not live at `0x2e6208`: that site only begins the relative-pose
quaternion packet.  The pixel enters later in
`LandmarkBlockAbsDynamic::linearizeLandmark()` while iterating `lm_ptr->obs`
(`landmark_block_abs_dynamic.hpp:289`, passed to `linearizePoint` at line
318).  Therefore the exact pair-key capture and the m7bo track/pixel record
are the appropriate provenance join; no pixel value is inferred from an
unrelated register at `0x2e6208`.

## Source/assembly mapping

Upstream `linearization_abs_qr.cpp:206-216` constructs the key from
`tcid_h`/`tcid_t` and passes

```text
computeRelPose(state_h, calib.T_i_c[tcid_h.cam_id],
               state_t, calib.T_i_c[tcid_t.cam_id])
```

`ba_utils.h:49-57` makes the packet order explicit:

```text
tmp2 = inverse(T_i_c_t);   // target-camera inverse first
res  = tmp2 * T_t_i_h * T_i_c_h;  // host-camera extrinsic last
```

The optimized sequence has the same order.  `0x2e6434` is the first camera
packet (target inverse), while `0x2e65b0` loads the second packet from `%rdi`
(host extrinsic).  Thus m7cd's `0x2e6434` value

```text
3bed3c0f,bbf71cd4,bf33a827,3f365a1e
```

was the inverse of cam0 and necessarily represented a target-cam0 factor.  It
was not the m7bo factor.  The correct bounded retry selected the pair key
above and reached `0x2e6434` with the cam1 inverse:

```text
xmm2 (IMU packet) = 3b322ebb,bab5f931,3a19f84f,3f7fffaf
xmm4 (inverse cam1) = 3b19188b,bc55012e,bf33d4ed,3f362af6
```

At `0x2e6453`, the partial capture also saw:

```text
xmm5 = b9d269d4,3c44d134,3f33f018,3f362a51
```

The register printer stalled at the next requested breakpoint,
`0x2e6458`, while evaluating `xmm8`; no later packet lanes are claimed.  The
bounded retry is retained in `target/m7cf_packet.out`.  Since the full packet,
translation, projection, and residual were not captured in this retry, no
`F32Pose`/`aom.rs` integration, fixture correction, unsafe/debug code, or
actual-call test was justified.

## Scope and process hygiene

Only the existing uninstrumented native binary and bounded max-five-frame
GDB runs were used.  The final process check found no `gdb` or `basalt_vio`
process.  Production source and tests remain unchanged; no commit or push was
performed.
