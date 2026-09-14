# m7cx affine call-site/codegen report

## Decision

Production is unchanged.  The pinned upstream source does not provide three
different affine-composition call sites or template instantiations for
temporal KLT, existing stereo, and new stereo.  Consequently there is no
upstream boundary at which an explicit per-category update policy/type could
be selected.  A track, id, value, or trace-id branch would be the only way to
make the observed m7cq bits and the m7bt bits disagree, and is not an
acceptable implementation.

The m7cq packet result is therefore treated as a compiler-context limitation,
not as evidence for a production category split.  `update.rs`, the fixtures,
and the tests were not changed by m7cx.

## Pinned source and call-site inventory

The source audit used the clean pinned Basalt checkout at
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc` (the checkout documented by
m7cr), with Eigen 5.0.1, GCC 11.4, and the native O3 configuration
`-O3 -g -DNDEBUG -march=native -std=c++17 -DEIGEN_DONT_PARALLELIZE
-DEIGEN_INITIALIZE_MATRICES_BY_NAN`.

The relevant source is
`include/basalt/optical_flow/frame_to_frame_optical_flow.h`:

| Runtime category | Upstream route | Composition boundary |
|---|---|---|
| Temporal KLT | `processFrame` loops over `calib.intrinsics` and calls `trackPoints` (around lines 161--165).  Camera 0 is the temporal case. | `trackPoints`' one forward/backward lambda calls `trackPoint`; `trackPointAtLevel` contains the only update composition, `transform *= SE2::exp(inc).matrix()` (around line 299). |
| Existing stereo | The same `processFrame` loop and the same `trackPoints` call, with camera 1. | The same `FrameToFrameOpticalFlow<Scalar, Pattern>` method and the same `trackPointAtLevel` line; there is no stereo-policy parameter. |
| New stereo | `addPoints` calls `trackPoints(pyramid->at(0), pyramid->at(1), new_poses0, new_poses1)` (around lines 311--343, call around line 338). | It enters the same lambda, `trackPoint`, and `trackPointAtLevel` template as the two `processFrame` cases. |

`src/optical_flow/optical_flow.cpp` selects one
`FrameToFrameOpticalFlow<float, Pattern51>` factory instantiation for this
configuration (around lines 74--86).  The clean source has no
`StereoForwardSe2Ic`, `ExistingStereoForward`, or temporal-specific affine
type; those names are labels in the diagnostic/lifecycle reports, not distinct
upstream compiled instantiations.

## Why the observed orders differ

Eigen 5.0.1 implements `Transform::operator*=(EigenBase)` through its
right-product machinery.  For this type the affine case reaches a fixed-size
`2x3 * 3x3` expression (`T.affine() * other`) and then writes the homogeneous
row.  The expression does not specify a reproducible scalar accumulation order.
Depending on inlining, live-register pressure, and Eigen's packet/scalar
dispatch, GCC can emit either a packet/add schedule or scalar/FMA instructions.

The m7cq diagnostic binary is not a clean production call context.  Its
instrumented include adds `trace_id`, camera/stereo/direction state, and a
`trace_id == 481` conditional trace/printing path inside the tracking code.
That changes inlining and register allocation around the Eigen expression.  It
can produce the packet/add order seen in the m7cq trace.  The m7bt artifact's
leaner context produces the scalar/FMA-looking order.  This is a codegen
context difference, not a source-level category distinction.

A temporary faithful O3 probe against the same clean pinned Eigen headers
compared direct `AffineCompact2f` Eigen composition with the explicit current
scalar/FMA spelling in `update.rs`.  It used
`-std=c++17 -O3 -march=native -DEIGEN_DONT_PARALLELIZE`; the temporary source
was removed after the observation.  For the m7cq L3/I1 operands and the m7bt
L3/I1 operands, both spellings produced the following exact bits:

```text
m7cq L3/I1 pair: 3f7ffbfa 3c359c6a bc359c77 3f7ffbf9 42ad6193 4101b788
m7bt L3/I1 pair: 3f7fe679 bce4a105 3ce4a103 3f7fe67a 40c07598 41d9e26b
```

The same direct-Eigen/scalar-FMA equality held for three temporal composed
pairs in the probe.  Thus the clean probe does not reproduce a stable
m7cq-only packet path.  The exact limitation is that the packet order is
observable in the instrumented m7cq context, but there is no clean pinned
un-instrumented native binary proving that it belongs only to new stereo (or
to any source-level category).

## Three observations per category

The following observations are retained from the m7bt/m7cq fixtures and the
clean temporal oracle.  They are deliberately reported as observations of the
shared boundary; the source audit above is what maps all three categories to
that boundary.

### New stereo

| Observation | Native/fixture bits | Rust/current bits | Classification |
|---|---|---|---|
| m7cq frame 9, cam 1, L3/I0 post | `3f7ddbfd 3e0425d4 be0425d4 3f7ddbfd 42adb8f5 40f7ec5e` | exact at this point | The first update is not a distinguishing mismatch. |
| m7cq frame 9, cam 1, L3/I1 post | `3f7ffbf9 3c359c70 bc359c70 3f7ffbf9 42ad6193 4101b788` | `3f7ffbfa 3c359c6a bc359c77 3f7ffbf9 42ad6193 4101b788` | The trace shows the packet/add result; the clean expression/probe selects the scalar/FMA result. |
| m7cq frame 9, cam 1, track 481 endpoint, first differing coordinate | `0x428105e8` | `0x428105e7` | The endpoint preserves the one-ULP consequence; it does not identify a separate upstream instantiation. |

These are the three observations recorded in
`m7cq_frame9_cam1_frontier_report.md` and its fixture.  They establish the
frontier, not a category-wide policy: the diagnostic path is explicitly
instrumented and one traced track cannot define all new-stereo calls.

### Existing stereo

The m7bt L3 trace provides three sequential affine observations at the shared
boundary.  The trace itself is track-3-only and does not carry a source-level
category tag; m7bx separately classifies the requested frame-1/cam-1 track-117
residual as existing stereo.  Since that existing-stereo route is the same
`processFrame`/`trackPoints` instantiation, the m7bt observations are the
scalar/FMA representative for this category rather than evidence of a
private existing-stereo call site.  The `pre` linear bits and the following
`upd` bits are shown because they are the inputs to the same
`AffineCompact2f` boundary:

| Observation | `pre` linear bits | `upd` linear bits | Scalar/FMA fixture translation |
|---|---|---|---|
| m7bt L3/I0 | `3f800000 00000000 00000000 3f800000` | `3f73999b be9d6ac3 3e9d6ac3 3f73999b` | `40d8ad14 41db5ebe` |
| m7bt L3/I1 | `3f73999b be9d6ac3 3e9d6ac3 3f73999b` | `3f75b3a9 3e8fc230 be8fc230 3f75b3a9` | `40c07598 41d9e26b` |
| m7bt L3/I2 | `3f7fe679 bce4a105 3ce4a103 3f7fe67a` | `3f7fe5b9 3ce7f745 bce7f745 3f7fe5b9` | `40b87334 41dccbb3` |

The three sequential observations use the scalar/FMA fixture order
(second coefficient, homogeneous term, then first coefficient).  They do not
prove that existing stereo has a private call site: the clean source maps it
to the same `processFrame`/`trackPoints` instantiation as temporal camera 0.

### Temporal KLT

Three clean native temporal observations are available in
`target/m7ca_native_composed_warp.txt`; the m7bw fixture additionally confirms
the first temporal update's local translation and endpoint:

| Observation | Native composed warp bits |
|---|---|
| m7ca temporal L3/I0 | `3f7fff40 3b9ce831 bb9ce831 3f7fff40 40acd429 41699800` |
| m7ca temporal L3/I1 | `3f7fff94 3b6b5ea4 bb6b5ea4 3f7fff94 40adb5cc 4169d0f3` |
| m7ca temporal L3/I2 | `3f7fffdf 3b01bf74 bb01bf74 3f7fffdf 40adea5f 4169d555` |

The m7bw temporal fixture records the first-update local translation as
`40acd429 41699800` and the frame-1 cam0 track-2 endpoint as
`422e3d26 42ea1cb7`.  The clean O3 probe's direct Eigen and scalar/FMA
spellings agreed for three temporal operand pairs, while the native composed
oracle retains the known one-ULP affine linear differences.  This is the same
context-sensitive Eigen behavior, not a temporal-only type.

## Rejected implementation and verification status

The m7cq report's packet-oriented candidate matched the isolated m7cq trace,
but changed the m7bt boundary (`n00` moved to `0x3f7fe67a` instead of the
fixture's `0x3f7fe679`).  It was reverted.  Since the source audit finds no
distinct call-site/type boundary and the clean probe does not reproduce a
category-exclusive packet schedule, no policy selector was added.

The retained fixture/test status is therefore unchanged:

- The m7bt exact boundary remains the scalar/FMA fixture (`0x3f7fe679 ...
  0x41d9e26b`).
- The m7cq frontier test continues to record the expected linear mismatch at
  indices `[0, 1, 2]` and the known one-ULP endpoint frontier.
- Existing m7bt/m7cq update checks remain the verification set from the
  preceding reports; no additional probe or test run was performed for m7cx,
  per the handoff instruction.
- The retained m7cm first-80 aggregate is `24,037 / 24,256` exact point pairs
  and `48,219 / 48,512` exact coordinate fields (max ULP 22).  The m7bt
  boundary report is `16,216 / 24,256` exact pairs and `36,617 / 48,512`
  exact fields (max ULP 47).  With production unchanged, m7cx has no
  beyond-m7cm improvement to claim.

No unsafe code, debug output, threshold, fixture, or test change was added.
