# M7fh IMU position exactness audit

Date: 2026-08-23 JST

## Decision

The source-faithful M7fc IMU kernel is retained.  The fresh five-frame replay
reaches **74/75 exact state lanes**; the only state residual is frame 4
translation z, native `bcdc2b7f` versus Rust `bcdc2b7e` (one ULP).  The
candidate is retained because it is a general source-order kernel, reproduces
the clean native delta-velocity endpoint, and introduces no regression in the
frame-1 golden or library tests.  The attempted position regrouping was
rejected because it breaks the pinned frame-1 golden position.

## Production kernel

`pipelines/basalt/src/vio/estimator.rs` contains the audited M7fc shape:

- Eigen source-order nested matrix-vector evaluation (`x/y/z` each use a
  nested `fma(row0*x, fma(row2*z, row1*y))`).
- The clean native packet schedule's x-lane velocity FMA,
  `accel_world.x.mul_add(dt, old_velocity.x)`; y/z retain the ordinary
  source-order vector update.
- Position keeps the pinned `p + v*dt` contraction before adding the
  acceleration term.  `predictState` position terms are explicitly grouped in
  source order (`(p + v*dt) + 0.5*g*dt*dt + R*delta_p`) for all intervals.

There are no frame/value-specific branches, debug prints, or `unsafe` code in
the production path.

## Fresh replay and residuals

Command used:

```text
target/release/examples/basalt_euroc_vio_demo.exe --euroc-dir E:/datasets/euroc_mav/machine_hall/MH_01_easy --calibration target/euroc_ds_calib.json --config target/euroc_config.json --out-dir target/m7fh_fresh5_run --max-frames 5
```

The replay processed 5/5 frames, 42 IMU samples, and 1114 observations; the
frame-4 iteration-start snapshot contains 70 factors and 1243 rows.  The
machine-readable H/b and state comparison is
`target/m7fh_state_hb.json`:

| comparison | result |
|---|---:|
| state lanes | 74/75 exact |
| H bit mismatches | 2366/5625 |
| b bit mismatches | 69/75 |

The remaining state lane is IMU/state-only (no direct visual row):

```text
frame=4 field=translation_xyz lane=2
native=bcdc2b7f rust=bcdc2b7e delta_ulp=-1
```

The clean native frame-3 to frame-4 preintegration endpoint is reproduced for
delta velocity (`3ede21c0,bcb31cdc,be20c273`) and the frame-1 golden velocity
remains exact.

## Position regrouping audit

The isolated native fixed-operand audit showed that a universal
`p + (v*dt + 0.5*a*dt*dt)` position grouping produces the native frame-4 z
word, but the same grouping changes the frame-1 golden position to
`[0.010060002095997334, -0.0006882318411953747,
-0.003518919460475445]` instead of the pinned
`[0.010060002095997334, -0.0006882318994030356,
-0.0035189196933060884]`.  Both scalar and vector forms fail the existing
`upstream_f32_interval_keeps_integer_endpoint_and_sophus_exp_order` oracle, so
the regrouping is not a valid general fix.  No frame/value special case was
introduced.

## Verification

```text
cargo test --release -p visloc-basalt --lib
```

Result: **173 passed, 0 failed, 1 ignored**.  No commit or push was performed.
