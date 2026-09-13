# M7q upstream state-generation trace (frame 0..4)

This is a source audit of the pinned official Basalt tree
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc` (tree
`b7afb830d82b45b8209cf784ad9744025d838411`).  It is diagnostic-only; no
source in the upstream checkout or this Rust implementation was changed for
the audit.  The default `src/vio.cpp` path is the target: `--use-double` is
not passed, so the estimator is `float` and the input queue's `double` IMU
records are cast when popped.

The preintegration header is the pinned vcpkg dependency
`basalt-headers` REF `aa441ba3e51050c47ba1902537792a2e4db7e43d`, checked out
outside the worktree.  This distinction matters because
`IntegratedImuMeasurement` is supplied by that header-only package, not by a
file in the Basalt application tree.

## Evidence and source provenance

The first-five-frame observation/state trace is
`target/basalt_upstream_mh01_400f_core_trace_20260821T000012Z/trace.jsonl`.
The corresponding first-20-frame IMU trace is
`target/basalt_upstream_mh01_400f_core_imu_trace_20260821T000014Z/imu_trace.jsonl`.
Both runs are sensor-only and use no ground-truth input.

Relevant source files and SHA-256 hashes (LF checkout) are:

| file | SHA-256 |
| --- | --- |
| upstream `src/vi_estimator/sqrt_keypoint_vio.cpp` | `24a4d310f4c9003d3d8cb31fe8947ecc9b51cbd2260091120c9e2ec3b93a8d22` |
| upstream `src/vio.cpp` | `2fc312281b3ba91369445e23a444efaf9c6927b564b08f6809f6d2417c897800` |
| upstream `include/basalt/utils/imu_types.h` | `d8f59eab1ab2195854ab10a856ecd8d9e94843c741f776c02825b4fe730a758e` |
| basalt-headers `include/basalt/imu/preintegration.h` | `7e6dd88f1a04928b622ae260052f268ea9c00a3bbd113b91e7ed317f4bd0d594` |
| basalt-headers `include/basalt/imu/imu_types.h` | `c897ab76edf4e39ef757015465d4dcc9577b705b939b3339d90434f11f3a225f` |

## Timestamp and queue boundary

`src/vio.cpp:167-194` pushes each EuRoC image with exactly the dataset image
timestamp.  `src/vio.cpp:197-212` pushes timestamped `ImuData<double>` records
without re-timing.  `src/vio.cpp:310-318` constructs the estimator with
`basalt::constants::g` and calls `initialize(zero_bg, zero_ba)`; the camera
offset adjustment in `sqrt_keypoint_vio.cpp:182-183` is commented out.
The optical-flow implementations preserve the current input timestamp in
their result (`multiscale_frame_to_frame_optical_flow.h:113-126`, likewise
the patch and non-multiscale implementations), so the VIO interval endpoint is
the current camera timestamp, not an optical-flow completion time.

For MH_01 the first five camera timestamps and upstream interval contracts are:

| frame | camera timestamp (ns) | interval used | `dt` (ns) | IMU packets |
| ---: | ---: | --- | ---: | ---: |
| 0 | 1403636579763555584 | initialization, no interval | — | 0 |
| 1 | 1403636579813555456 | `(frame0, frame1]` | 49999872 | 10 |
| 2 | 1403636579863555584 | `(frame1, frame2]` | 50000128 | 10 |
| 3 | 1403636579913555456 | `(frame2, frame3]` | 49999872 | 10 |
| 4 | 1403636579963555584 | `(frame3, frame4]` | 50000128 | 10 |

The first packet in each interval is 0.5 ms after the previous camera time and
the last packet is at the current camera time (see the IMU trace fields
`first_sample_t_ns` and `last_sample_t_ns`).

## Upstream frame 0 initialization

The no-time `initialize(bg, ba)` overload is the path selected by
`src/vio.cpp:314` (`sqrt_keypoint_vio.cpp:151-273`).  It first pops and
calibrates an IMU packet (`:164-169`).  Until the first camera arrives it
discards packets with `data->t_ns < curr_frame->t_ns` (`:185-192`).  Thus the
packet used for alignment is the first packet at or after the camera timestamp;
there is no interpolation and no preintegration before frame 0.

The initial state is then constructed at `curr_frame->t_ns` (`:194-204`):

* `T_w_i = FromTwoVectors(data.accel, +Z)`;
* position, world velocity, gyro bias, and accelerometer bias are zero;
* an empty `IntegratedImuMeasurement` is keyed by the camera timestamp;
* the frame is inserted into `frame_states` and the marginal order.

`measure(frame0, null)` then creates the first visual keyframe.  The trace
records frame 0 as KF=true, 135/61 cam0/cam1 tracks, pose translation and
velocity/biases all zero.  The quaternion is
`[-0.052778493613004684, -0.8023747801780701, 0, 0.5944822430610657]` in
the trace's `xyzw` order.

## Upstream frame 1 and frame 2 prediction

After frame 0, `sqrt_keypoint_vio.cpp:222-257` creates an
`IntegratedImuMeasurement` whose start is the previous camera timestamp and
whose bias linearization point is the previous state's dynamic bias
(`:225-229`).  It discards packets at or before the previous camera boundary
(`:235-240`) and integrates packets while `data->t_ns <= curr_frame->t_ns`
(`:242-248`).  This is exactly `(t_prev, t_curr]`.

`measure` asserts that the integrated start and end equal the two state/camera
timestamps and predicts the next state before adding visual residuals
(`sqrt_keypoint_vio.cpp:306-331`).  The predicted state is inserted into
`frame_states` at the optical-flow timestamp, and the measurement is stored
under its start timestamp.  The first-five trace gives these pre-optimization
predictions (and, for frames 1--3, identical optimized states because the
upstream optimizer has not started yet):

| frame | predicted `p_w_i` (m) | predicted `v_w_i` (m/s) |
| ---: | --- | --- |
| 0 | `[0, 0, 0]` | `[0, 0, 0]` |
| 1 | `[0.0003828657791, -0.00009857659461, -0.002031802200]` | `[0.02098453045, -0.005973815918, -0.08147591352]` |
| 2 | `[0.001860887976, -0.0004836908192, -0.007697241381]` | `[0.03474110365, -0.008907575160, -0.1420118809]` |
| 3 | `[0.004356339574, -0.001130370889, -0.01629174128]` | `[0.06494957209, -0.01874402538, -0.1975938678]` |
| 4 | `[-0.001888678176, -0.004027507268, -0.08736213297]` | `[0.008318431675, -0.02858878672, -0.5352340341]` |

Frame 4 is the first frame for which an actual LM solve occurs in the
default upstream path: `optimize()` is gated by
`opt_started || frame_states.size() > 4` (`sqrt_keypoint_vio.cpp:1012-1019`).
The trace reports 8 iterations and then marginalization, leaving two states
and one keyframe pose.  Frames 0--3 report zero LM iterations.  The final
frame-4 state therefore differs from its prediction; this is an optimization
effect, not a different IMU sign convention.

## Exact upstream IMU update order (the requested sign/order check)

The authoritative implementation is
`basalt-headers/include/basalt/imu/preintegration.h`.  In
`propagateState` (`:76-100`), let `R` be the current delta rotation,
`v`/`p` the current delta velocity/position, and let `a`/`w` be the incoming
calibrated, bias-linearized accelerometer/gyro sample.  With
`dt=(data.t_ns-curr_state.t_ns)*1e-9`:

```text
R_half = R * Exp(+0.5 * dt * w)
a_i    = R_half * a
p'     = p + v*dt + 0.5*a_i*dt^2
v'     = v + a_i*dt
R'     = R * Exp(+dt*w)
```

The source order is `R_half` at `:89-91`, acceleration rotation at `:93`,
then position/velocity at `:97-100`, and the full rotation composition at
`:96`.  Therefore:

* gyro uses **positive** `SO3::exp(+gyro*dt)`, never `exp(-gyro*dt)`;
* force uses the **half-step** orientation, not the old orientation and not
  the already-full-step orientation;
* position uses the pre-update `v` and the same half-step force used by
  velocity;
* gravity is not in the delta.  It enters only in `predictState`.

`integrate` (`preintegration.h:163-186`) subtracts the fixed bias
linearization point, shifts the packet timestamp by `start_t_ns`, calls
`propagateState`, and then updates covariance/Jacobians.  The state prediction
(`:193-204`) is:

```text
R1 = R0 * DeltaR
v1 = v0 + g*dt + R0*DeltaV
p1 = p0 + v0*dt + 0.5*g*dt^2 + R0*DeltaP
```

The fixed world gravity is `constants::g=(0,0,-9.81)` in the upstream
`include/basalt/utils/imu_types.h:61-64`, passed by `src/vio.cpp:312-314`.
The default float trace prints its cast value `-9.8100004196166992`.  There
is no gravity sign inversion or body/world swap in the upstream prediction.

The residual uses the same convention and `[position, rotation, velocity]`
row order (`preintegration.h:223-254`): position and velocity are rotated by
`R0^-1`, while the rotation block is
`log(Exp(bg_correction) * DeltaR * R1^-1 * R0)`.

## State convention and optimizer writeback

The dependency state type documents `T_w_i` as body/IMU-to-world and
`vel_w_i` as world-frame velocity (`basalt-headers/imu/imu_types.h:64-68,
123-127`).  Its 15-vector increment order is
`[translation, rotation, velocity, bias_gyro, bias_accel]`; pose increments
left-compose `Exp(drot)` and the other blocks add (`:98-101`, `:139-146`,
`:216-224`).  Rust declares the same active `imu_to_world=T_w_i` and world
velocity in `pipelines/basalt/src/types.rs:82-102`, with the same flatten order
in `pipelines/basalt/src/vio/window.rs:803-813`.

Upstream's per-frame order is:

1. preintegrate and insert the predicted state (`sqrt_keypoint_vio.cpp:222-331`);
2. connect observations, decide keyframe, and add visual/IMU factors
   (`:336-484`);
3. `optimize_and_marg` calls `optimize` then `marginalize`
   (`:1430-1435`);
4. LM backs up, applies state increments, and restores rejected trials
   (`:1235-1263`, `:1382-1390`);
5. only after that does it cast/push the latest state to `out_state_queue`
   (`:484-493`).

Rust's analogous order is `process` prediction/state insertion at
`pipelines/basalt/src/vio/estimator.rs:287-336`, one `WindowProblem::solve` at
`:349-352`, explicit state writeback at `:353-357`, optional prior/window
shift at `:373-410`, and public-state refresh/output at `:411-428`.

## Rust comparison: what is equal and what is not

### Equal conventions

The production Rust first-frame path is `initial_nav_from_imu`
(`estimator.rs:945-970`): calibrated first sample `>=` the camera timestamp,
acceleration aligned to world `+Z`, zero position/velocity/biases, and no
pre-camera integration.  This matches upstream `initialize` above.  Rust's
`predict_nav` (`estimator.rs:1019-1041`) has the same `R0*DeltaR`, `+g*dt`,
`+0.5*g*dt²`, `R0*DeltaV`, and `R0*DeltaP` order.  Its factor residual
(`pipelines/basalt/src/imu/factors.rs:125-150`) is algebraically the same
position/rotation/velocity residual as the upstream header.

`pipelines/basalt/src/initialization.rs:43-99` is a separate public stationary
window helper.  It averages gyro/accel and integrates to the camera while
seeding velocity/translation; it is **not** the production VIO path above and
must not be used as the frame-0 oracle.

### Concrete differences affecting frame 1/2 deltas

1. **Packet policy (main source of the fixed-golden delta mismatch).**
   Upstream consumes each packet in `(t_prev,t_curr]` directly.  Its first
   packet spans the partial interval from `t_prev` to that packet timestamp.
   Rust's normal `integrate_between` (`pipelines/basalt/src/imu/sampling.rs:46-79`)
   interpolates synthetic samples at both camera endpoints (`:20-43`), then
   integrates the average of each adjacent pair (`:71-77`).  Thus it uses a
   trapezoidal/endpoint-synthesized sequence, not upstream's zero-order packet
   sequence.  This explains why a fixed frame-1-to-2 delta can differ even
   though Rust's per-sample half-step equation is otherwise the same.

2. **Scalar type.**  The default upstream is float: input `ImuData<double>` is
   cast in `sqrt_keypoint_vio.cpp:287-302`.  Rust's navigation/preintegration
   path is `f64`.  This is a secondary numerical difference after packet
   policy.

3. **Fallback semantics.**  Upstream has no exception-style normal-path
   fallback.  If the integrated end is short of the current camera, it
   temporarily retimestamps the next queued packet to the camera endpoint
   (`sqrt_keypoint_vio.cpp:250-256`); if no packet exists it breaks.  Rust only
   enters `fallback_integrate` after normal interpolation fails
   (`estimator.rs:893-914`).  That fallback selects direct packets with
   `start < t <= end` (`:972-1005`) and repeats the last selected measurement
   for a trailing gap (`:1006-1016`), so it is close to but not byte-identical
   to upstream's use of the *next* queued packet.  The exact MH_01 intervals
   have valid endpoints, so normal Rust interpolation is used and the trace's
   fallback flag is false.

4. **Keyframe/solve schedule.**  Upstream's keyframe decision is track-based:
   connected cam0 ratio below `vio_new_kf_keypoints_thresh` after the minimum
   frame gap (`sqrt_keypoint_vio.cpp:370-385`).  Rust marks every fifth active
   state as a keyframe (`estimator.rs:330-335`).  Upstream does not run LM until
   `frame_states.size()>4` or a previous optimization has started, while Rust
   calls its window solver on every `process` (`estimator.rs:349-356`).  This
   does not change the upstream IMU equation, but it changes frame-4
   post-optimization state and window/marginalization behavior.

## Bottom line for M7q

There is no upstream `-gyro` sign, old/new orientation, or gravity-frame bug
to port: the authoritative order is positive gyro, half-step force rotation,
pre-update velocity for position, then full-step rotation; prediction adds
world `g=[0,0,-9.81]`.  Rust already follows that algebra in
`integrate_sample` (`pipelines/basalt/src/imu/preintegration.rs:100-121`) and
`predict_nav` (`estimator.rs:1019-1041`).  The actionable parity delta for the
reported frame-1/2 p/v/q discrepancy is to reproduce upstream's queue packet
semantics and float scalar before changing signs or state-frame conventions.
The later frame-4 discrepancy additionally includes the different keyframe/
LM schedule and optimizer/marginalization implementation.
