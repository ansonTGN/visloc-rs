# m7fc IMU delta-velocity O3 schedule probe

Date: 2026-08-23. This is a read-only diagnostic; no production source was edited.

## Result

**Candidate found:** keep the source-order matrix-vector expression and fuse only
the x lane of the `delta_velocity` update:

```text
accel_world[row] = fma(Rr0, ax, fma(Rr2, az, Rr1*ay))
delta_velocity.x = fma(accel_world.x, dt, old_velocity.x)
delta_velocity.y = old_velocity.y + accel_world.y*dt
delta_velocity.z = old_velocity.z + accel_world.z*dt
```

The candidate is implementable as a compiler/build schedule candidate, but is not
applied here. It matches clean native O3 packet endpoints for frame 3 and keeps
the frame-1 golden endpoint unchanged.

## Evidence

The pinned native Basalt header computes `RR_w_i_new_2 * data.accel`, then
`vel += accel_world * dt`. A standalone Rust probe (`target/m7fc_imu_delta_o3_probe.rs`)
replayed the calibrated MH_01 samples with fixed plain/FMA schedules. The plain
schedule reproduces the current Rust/O2 endpoint; x-only FMA reproduces native O3:

| interval | current Rust/O2 | clean native/O3 | Rust x-only FMA | golden |
|---|---|---|---|---|
| frame 3 -> 4 | `3ede21be,bcb31cdc,be20c273` | `3ede21c0,bcb31cdc,be20c273` | `3ede21c0,bcb31cdc,be20c273` | — |
| frame 1 -> 2 | `3ecf7735,bcff201e,be21e85a` | `3ecf7735,bcff201e,be21e85a` | `3ecf7735,bcff201e,be21e85a` | same |

The first frame-3 difference is packet 6, x: `3e85be12` (O2/plain) versus
`3e85be13` (O3/x-FMA). The x-only candidate then yields the clean sequence
`3e85be13`, `3e9be318`, `3eb1fa85`, `3ec809c2`, `3ede21c0` for packets 6--10;
y/z remain equal to the native clean trace. The native C++ probe confirms O2/O3
frame-1 endpoints are identical, so this schedule preserves the existing golden.

## Alternatives checked

All-lane FMA accumulation changes frame-1/frame-3 y by one ULP and is rejected.
The native nested MVM schedule is included because it reproduces every frame-3
packet endpoint; MVM-only FMA does not change the final x mismatch. Therefore the
isolated x-lane accumulation boundary is the smallest general sequence matching
both constraints.

Raw probe output and native O2/O3 logs are listed in `target/m7fc_imu_delta_o3.json`.
