# M7fk post-IMU / production point frontier

Date: 2026-08-23 JST  
Status: **Complete — read-only production-factor probe; arithmetic unchanged.**

## Replay

The current combined M7fj Packet4 point path and M7fh IMU-position path were
rebuilt and replayed for five MH01 frames. A temporary, exact-key probe inside
the f32 production factor captured direction parameters, bearing3, T16, point4,
projection, camera J, homogeneous Jpp, raw/weighted Jp, and weights. The probe
was removed afterward.

Machine-readable capture: [`target/m7fk_post_imu_point_frontier.json`](../../target/m7fk_post_imu_point_frontier.json)

Focused verification after probe removal:

```text
cargo test -p visloc-basalt m7 --lib -- --nocapture
27 passed, 0 failed, 149 filtered
```

## Key mapping

The clean m7ef mapping identifies native ordinal 13 as **cross-time** (track
66, observation 3), not same-TimeCam. The same landmark's same-TimeCam row is
native ordinal 0 (observation 0), so both packets were captured explicitly.
Native ordinal 311 is track 22, observation 3, also cross-time.

| packet | relation | exact input/stage result |
|---|---|---|
| track 66 / native 0 | same-TimeCam identity | camera, direction/rho, point4, projection, camera J, Jpp, raw Jp all exact |
| native 13 | cross-time | direction/rho/bearing/camera/pixel exact; T16 10/16; point4 3/4; projection/raw 2/2; camera J 5/8; Jpp 12/12; raw/weighted Jp 4/6; sqrt weight exact |
| native 311 | cross-time | direction/rho/bearing/camera/pixel exact; T16 10/16; point4 3/4; projection/raw 1/2 |

For both cross-time packets, T16 mismatches are exactly rotation lanes
1,2,4,6,8,9; translation lanes and homogeneous row remain exact. Ordinal 13
point-y is `bdd79baf` versus clean `bdd79bb0`; ordinal 311 point-y is
`bd344ff5` versus clean `bd344ff6`.

## Diagnosis

This is a production-wrapper boundary, not an input bearing, state, or weight
problem. The current wrapper at `aom.rs:680` normalizes the F32Pose quaternion
before materializing T16 and calling `eigen_homogeneous_point_product_f32`.
The clean m7ef T16 uses the unnormalized clean q-to-matrix packet; normalization
changes six rotation lanes while leaving translation unchanged. The retained
M7fh state audit is 74/75 exact, with its sole frame-4 translation-z ULP in an
IMU-only lane.

The direct M7fi tests inject an already exact T16 into
`eigen_homogeneous_point_product_f32`, so they bypass this production-wrapper
normalization. Same-TimeCam passes because its explicit identity branch bypasses
the relative transform. Ordinal-13 projection/raw happen to round exact despite
the point-y ULP; ordinal 311 propagates point-y into projection/raw v by +1 ULP.

No production arithmetic was changed. The next fix frontier is the
cross-time relative-pose/matrix normalization boundary.
