# M7gf IMU position oracle

Diagnostic-only result (no production source change).  The pinned header is
`basalt/imu/preintegration.h`, built with the native oracle flags
`g++ -std=c++17 -O3 -march=native -DEIGEN_DONT_PARALLELIZE` (the probe used
`-g -Wall -Wextra -DEIGEN_INITIALIZE_MATRICES_BY_NAN -fPIC` as well).

The native `IntegratedImuMeasurement::propagateState` translation packet does
two fused multiply-adds per lane, not one FMA followed by a rounded add:

```text
p_v  = fma(old_velocity, dt, old_position)
hdt  = (0.5f * accel_world) * dt
pnew = fma(hdt, dt, p_v)
```

The disassembly in `target/m7ge_integrate_asm.s` shows this at the x/y/z
translation stores (`+32`, `+36`, `+40`): the first FMAs are at lines
70863/70986/71061 and the second FMAs at lines 70875/70998/71102 (the
exact line shifts are compiler/debug-assembly dependent).  The second FMA is
the material boundary: current Rust had materialized `(hdt * dt)` and added it
to `p_v`.

Using the schedule above in a diagnostic Rust replay (nested Eigen MVM and
x-lane velocity FMA retained) gives the pinned full MH01 interval endpoints:

| interval | delta velocity bits | delta position bits |
|---|---|---|
| frame 1 | `3EC46FAA,BCF45E4C,BE0FADF7` | `3C1DF76D,BA397D97,BB5CE9DD` |
| frame 2 | `3ED01376,BCE149D8,BE0DB650` | `3C24D2B5,BA346A76,BB669DAD` |
| frame 3 | `3ECF7735,BCFF201E,BE21E85A` | `3C246285,BA3EE05D,BB808207` |
| frame 4 | `3EDE21C0,BCB31CDC,BE20C273` | `3C320E93,BA2E4F54,BB7F5A2B` |

Thus the live frame-4 z correction is `BB7F5A2C -> BB7F5A2B`; with the native
delta-position operand, the existing two-FMA `predictState` schedule produces
the requested `BCDC2B7F` state z while preserving frame 1--3.

For the final (10th) frame-3→4 packet, the decisive z intermediates are
`p_old=BB4E9296`, `v_old=BE1019F4`, `accel_world=C050382C`,
`dt=3BA3D8A6` (`5,000,192 ns * float(1e-9)`),
`p_v=BB7CAFD3`, `hdt=BC0543FA`, and `fma(hdt,dt,p_v)=BB7F5A2B`.
Materializing `hdt*dt` (`B82A9620`) and adding it to `p_v` rounds to
`BB7F5A2C` instead.

The independent native per-packet trace is
`target/m7ge_native_position_frame3.log`; the diagnostic schedule probe is
`target/m7ge_m7fc_fma2_position_probe.rs`.
