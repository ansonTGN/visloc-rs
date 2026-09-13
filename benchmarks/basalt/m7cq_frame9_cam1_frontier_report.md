# M7cq frame-9 cam1 new-stereo frontier

Date: 2026-08-23 JST

## Result

The m7cm endpoint frontier remains open.  The pinned first-80 artifact is
unchanged at 48,219/48,512 exact coordinate fields and 24,037/24,256 exact
point pairs, with maximum error 22 ULP (`0.00048828125` px).  Its first
mismatch is frame 9, cam1, track 481, y:

```text
native 0x428105e8 (64.51153564453125)
Rust   0x428105e7 (64.51152801513672)
```

No production arithmetic patch was retained in m7cq.

## Path classification

Track 481 is not present through frame 8.  At frame 9 it is born as an
integer cam0 FAST point at `(684, 53)` and is then passed through the new
point stereo path:

```text
StereoForwardSe2Ic -> StereoBackwardSe2Ic -> StereoFbSquared
```

It is therefore neither an existing-stereo update nor a temporal update.

## Exact operand trace

The standalone pinned Eigen/Sophus probe used the frame-9 cam0/cam1 images,
`Pattern51`, four levels (3 down to 0), and five IC iterations per level.
For the forward direction, source bits are `0x442b0000, 0x42540000`.
At level 3 / iteration 0, patch data, the 156 Jacobian coefficients, all 52
residual lanes, the three-lane increment, the SE2 exponential, and the
composed transform are bit-identical between native and Rust.

At level 3 / iteration 1, the exact common operands are:

```text
pre transform [a00,a01,a10,a11,tx,ty]
  3f7ddbfd be0425d4 3e0425d4 3f7ddbfd 42adb8f5 40f7ec5e
increment [tx,ty,theta]
  be18cf0d 3ebd2b4c be0fde47
SE2 exp [r00,r01,r10,r11,tx,ty]
  3f7d7a40 3e0f653b be0f653b 3f7d7a40 bdfb8ba7 3ec1e7e5
```

The first divergent scalar is the affine linear result immediately after
`AffineCompact2f *= SE2::exp(increment).matrix()`:

```text
native [a00,a01,a10,a11,tx,ty]
  3f7ffbf9 3c359c70 bc359c70 3f7ffbf9 42ad6193 4101b788
Rust [a00,a01,a10,a11,tx,ty]
  3f7ffbfa 3c359c6a bc359c77 3f7ffbf9 42ad6193 4101b788
```

The retained fixture is
`pipelines/basalt/tests/fixtures/m7cq_frame9_cam1_stereo_update_frontier.json`.
The unit regression
`update::tests::m7cq_frame9_cam1_stereo_update_frontier_is_reproduced` checks
the exact operands, SE2 result, and current Rust post-update bits while
retaining the native/Rust mismatch as provenance.

## Patch decision

A candidate general update change that rounded the first-column products and
added the second-column products separately reproduced the isolated native
packet trace.  It was rejected: the existing m7bt native boundary fixture
then failed (`n00` became `0x3f7fe67a` instead of the pinned
`0x3f7fe679`).  This means the candidate is not a proven general Eigen/Sophus
operation-order replacement for the established update path.  It was
reverted; no track/camera/threshold/unsafe/debug branch was added.

## Verification

After reverting the candidate and adding the provenance fixture:

```text
cargo test -p visloc-basalt update --release --lib
8 passed, 0 failed
```

The m7cm first-80 aggregate is the authoritative unchanged baseline above;
the focused full replay should be rerun by the parent after integration of
this report if required.

No commit or push was made.
