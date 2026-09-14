# M7dg relative-pose exactness

Date: 2026-08-23

## Result

The f32 anchored visual-factor path now reproduces the clean native frame 4 /
track 1 relative pose exactly (q in xyzw order, then t in xyz order):

```text
bc0e2957 bb507f02 ba002d40 3f7ffd31
bde1e255 baf1ec60 ba884368
```

The clean downstream homogeneous point and reprojection lanes also match:

```text
point:      bf2fc73e be94aaa8 3f2ec6b2
projection: 41da6d17 42d70d32
raw:        bc22d800 3f915440
```

## Implementation

`aom.rs` keeps the same-`TimeCamId` identity branch and uses two source-context
packet quaternion products for the general camera-prefix and host-extrinsic
suffix.  The products preserve the native Packet4f FMA lane schedule and
normalization boundary.  The relative-pose matrix boundary separately mirrors
Eigen's materialized scalar `toRotationMatrix` products and normalizes the
Sophus temporary before the homogeneous point action.  The existing generic
helpers and fixed landmark fixtures remain unchanged; no value/frame/track
branch or tolerance adjustment was added.

## Verification

```text
cargo test --release -p visloc-basalt --lib \
  vio::aom::tests::m7_ -- --nocapture
10 passed, 0 failed
```

The focused test set covers the relative-pose chain, anchored projection/raw
residual, homogeneous point, matrix lanes, Jacobian boundary, and the existing
same-timestamp stereo identity behavior.  No production or test debug output,
unsafe code, commit, or push was added.
