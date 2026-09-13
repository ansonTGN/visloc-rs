# M7au IMU pre-solver velocity boundary (2026-08-22)

This is a narrowly scoped follow-up to the M7 pre-solver audit.  It owns only
the estimator prediction path and the diagnostic probe; no AOM, visual-factor,
or `aom.rs` source was changed.

## Direct pinned boundary

The diagnostic
[`m7aq_predict_velocity_probe.cpp`](m7aq_predict_velocity_probe.cpp) calls
the pinned `IntegratedImuMeasurement<float>::predictState` with the exact
pre-solver frame-2 state and the `(frame-2, frame-3]` interval.  It was built
with `-O2/-O3 -march=native -DEIGEN_DONT_PARALLELIZE`; both logs are
bit-identical (SHA-256
`5F4C087F69A052E2494BF4E2B99D1E7468D8EF5C86FBA0FCE838459D4E5DE4B2`).

The first differing Rust lane was velocity z.  The fixed operands and direct
header result are:

| value | float32 bits (x,y,z) |
| --- | --- |
| frame-2 velocity | `3D0E4CB0, BC11F114, BE116B90` |
| frame-2→3 delta velocity | `3ECF7735, BCFF201E, BE21E85A` |
| `R2 * delta_velocity` | `3CF777C0, BC212910, 3EDEAD68` |
| `g * dt` | `00000000, 00000000, BEFB22A7` |
| direct `predictState` velocity | `3D850448, BC998D12, BE4A560C` |

The ordinary left-associated candidate (`v0 + g*dt + R0*dv`) produced
`BE4A5610`, matching the pre-fix Rust result.  The exact source/O2/O3 result
is obtained when Eigen's packet evaluator contracts the gravity multiply/add
lane first, then adds the rotated delta:

```text
fma(g.x, dt, v0.x) + rotated_dv.x
fma(g.y, dt, v0.y) + rotated_dv.y
fma(g.z, dt, v0.z) + rotated_dv.z
```

The right-associated candidate produced `BE4A560E`; it is not the header
result.  This isolates the first velocity mismatch to the gravity
`multiply+add` boundary, not the SO3 point action.

## Production change and focused regression

`predict_nav` now uses the source-faithful `predict_velocity_f32` helper in
`pipelines/basalt/src/vio/estimator.rs`.  The helper is guarded by
`frame2_to_frame3_velocity_matches_pinned_predict_state_boundary`, which also
checks the pinned frame-2→3 rotated-delta bits and the frame-0→1 velocity bits.
Focused release tests passed:

```text
cargo test --release -p visloc-basalt --lib frame2_to_frame3_velocity_matches_pinned_predict_state_boundary
cargo test --release -p visloc-basalt --lib sophus_point_action_matches_pinned_fixtures
cargo test --release -p visloc-basalt --lib upstream_float_frame1_2_bias_jacobians_golden
```

## Fresh five-frame replay

The clean sensor-only release replay was run after the production edit:
`target/m7au_velocity_fma_run5_20260822/`.

| frame | predicted velocity bits | pinned pre-solver bits | result |
| ---: | --- | --- | --- |
| 1 | `3CABE7C0,BBC3C000,BDA6DCD8` | same | exact; frame-1 fixture not regressed |
| 2 | `3D0E4CB0,BC11F114,BE116B90` | same | exact |
| 3 | `3D850448,BC998D12,BE4A560C` | same | exact; z fixed from `BE4A5610` |
| 4 | `3DA7A8B8,BC8BED08,BE67F1C8` | `3DA7A8B8,BC8BED06,BE67F1C6` | y +2 ULP, z +2 ULP; z improved from +6 ULP |

Frame-4 position and quaternion lanes remain exact.  The frame-4 solver input
has the same 70 factors / 1243 rows and the same `AAAAAAAA` (8 accepted, 0
rejected) schedule.  Initial cost is `4215.86376953125`; final cost is
`248.4736785888672`.

Replay summary: 5 frames, 42 IMU samples, 1114 observations,
`sensor_only=true`.  Artifact hashes:

| artifact | SHA-256 |
| --- | --- |
| `m7aq_predict_velocity_O2_20260822.log` | `5F4C087F69A052E2494BF4E2B99D1E7468D8EF5C86FBA0FCE838459D4E5DE4B2` |
| `m7aq_predict_velocity_O3_20260822.log` | `5F4C087F69A052E2494BF4E2B99D1E7468D8EF5C86FBA0FCE838459D4E5DE4B2` |
| `m7au_velocity_fma_run5_20260822/summary.txt` | `4A4C9921B9F1EB8F7CAE8A78E5D017AF756DBABB1EBB6FD767032DE245F237EB` |
| `m7au_velocity_fma_run5_20260822/trace.jsonl` | `E62DB0A0D1DB8A41E9F147FA05114E69F02911BF70CB400DBEE432F44FCDF0E2` |

## Frame-3 -> frame-4 follow-up

The optional `frame3` mode of
[`m7aq_predict_velocity_probe.cpp`](m7aq_predict_velocity_probe.cpp) pins the
frame-3 state and the `(frame-3, frame-4]` interval.  The exact operands and
direct `predictState<float>` result are:

| value | float32 bits (x,y,z) |
| --- | --- |
| frame-3 velocity | `3D850448,BC998D12,BE4A560C` |
| O2 interval delta-v | `3EDE21BE,BCB31CDC,BE20C273` |
| O3 interval delta-v | `3EDE21C0,BCB31CDC,BE20C273` |
| O2 direct predicted velocity | `3DA7A8B8,BC8BED08,BE67F1C8` |
| O3 direct predicted velocity | `3DA7A8B8,BC8BED06,BE67F1C6` |

The O3 delta-v is the pinned pre-solver operand.  With it, the rotated
delta-v is `3C8A91C0,3ADA00C0,3EEC551F`; the gravity-FMA and old-velocity
association then produce the expected frame-4 y/z bits.  Therefore neither
SO3 point action nor the prediction-side gravity/association expression is a
remaining source-faithful edit.  The Rust interval currently matches the
pinned O2 result.  Per-sample logs show the O2/O3 x-lane first separates at
packet 6 (`3E85BE12` versus `3E85BE13`) and remains two ULP apart at the final
delta; y/z are unchanged in that interval.

The exact-bit regression
`frame3_to_frame4_point_action_matches_pinned_predict_state_boundary` records
the O3 rotation and delta-v action.  The previously accepted
`predict_velocity_f32` gravity-FMA change remains in production; no second
prediction edit was made.

### Preintegration FMA candidate — rejected

As a source-faithful experiment, the preintegrator velocity update was changed
to per-lane FMA for the header expression
`curr_state.vel_w_i + accel_world * dt`.  It moved the final delta-v x to the
O3 bit pattern, but also changed the exact frame-1 velocity y from
`BBC3C000` to `BBC3C008`, frame-2 y, and frame-3 y.  A fresh five-frame
sensor-only replay of that candidate produced:

| frame | predicted velocity bits under rejected candidate |
| ---: | --- |
| 1 | `3CABE7C0,BBC3C008,BDA6DCD8` |
| 2 | `3D0E4CB0,BC11F118,BE116B90` |
| 3 | `3D850448,BC998D14,BE4A560C` |
| 4 | `3DA7A8B8,BC8BED08,BE67F1C6` |

The candidate replay had 70 factors / 1243 rows, initial cost
`4215.86376953125`, and final cost `248.474945068359`.  It was reverted because
it regressed the frame-1 exact fixture and did not close frame-4 y.  Its
artifacts are retained only as negative evidence:

| artifact | SHA-256 |
| --- | --- |
| `m7aq_predict_velocity_frame3_step_O2_20260822.log` | `34627B10F179E321BFE960D3B1E4EFAE36CA5C66A5906F5750F140781263C66C` |
| `m7aq_predict_velocity_frame3_step_O3_20260822.log` | `DA7BE310AC36C97654A36899E88E7B903481E159FBBE3C76E92C8E07AFAD3914` |
| `m7au_velocity_assoc_fma_run5_20260822/summary.txt` | `F6E1EFECAD768B48FD7AD07F4183C7EBBBE26F311C17A3E5362C15830A0A3CE7` |
| `m7au_velocity_assoc_fma_run5_20260822/trace.jsonl` | `79D407B3C697F42ACBF77BA05E04C80531FEB4F65AAE9CF40972B3FC3832A338` |

Final production decision for this boundary: retain the gravity-FMA
prediction fix and the exact tests; do not lane-special-case the O2/O3
preintegration difference.  The safe final state is therefore frame-3 exact,
with frame-4 predicted y `+2 ULP` and z `+2 ULP` as recorded above.

## Post-revert final replay

After reverting the rejected preintegration candidate, a fresh five-frame
sensor-only release replay was completed at
`target/m7au_velocity_final_run5_20260822/`.  Its estimator state bits are
unchanged from the accepted production replay:

```text
frame 1: 3CABE7C0,BBC3C000,BDA6DCD8
frame 2: 3D0E4CB0,BC11F114,BE116B90
frame 3: 3D850448,BC998D12,BE4A560C
frame 4: 3DA7A8B8,BC8BED08,BE67F1C8
```

The run has 70 factors / 1243 rows, initial cost `4215.86376953125`, and
final cost `248.475799560547`.  The small final-cost difference from the
earlier `248.4736785888672` artifact is outside this estimator-only boundary
and reflects concurrent visual/AOM work in the shared workspace; all audited
pre-solver velocity bits are identical.

| artifact | SHA-256 |
| --- | --- |
| `m7au_velocity_final_run5_20260822/summary.txt` | `7A02D2B4C1BFDC4D243E98A013FCDC1A01F97ED36F8320049DDFD3D8E815E07B` |
| `m7au_velocity_final_run5_20260822/trace.jsonl` | `ED03B8064421A5D708471B0ECC335FCF7D14BB66AD6142AF9E571C7B11498155` |
