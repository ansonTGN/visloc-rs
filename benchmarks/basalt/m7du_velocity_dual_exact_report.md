# M7DU velocity dual-exact audit

Date: 2026-08-23 JST  
Status: **no source change; m7dk velocity expression retained**

## Decision

No general Eigen-faithful velocity helper reaches both gates.  The retained
production expression is still:

```rust
self.velocity = old_velocity + accel_world * dt;
```

No lane, frame, value, target-feature, debug, or `unsafe` branch was added.

The current Rust release assembly already lowers this expression to the
packet operation required by a fixed `Vector3f`: `vmulps`, then `vaddps`, then
a 16-byte `vmovups` store (the update block is at
`target/release/deps/visloc_basalt-6b1eb6d99168811f.s:157212`).  Re-spelling
the same operation as a hand-written padded-lane helper would not add a
source-level semantic distinction; the fourth lane is not reduced into any
of the three velocity lanes.

## Native codegen evidence

The pinned native audit emits the same source expression through different
compiler schedules:

* O2 uses independent scalar `vmulss`/`vaddss` operations for the three
  velocity lanes (`target/m7aq_predict_velocity_probe_O2_audit.s:12420` and
  `:12462`).
* O3 fuses only the first velocity lane (`vfmadd213ss`) while retaining
  scalar add for the other two (`target/m7aq_predict_velocity_probe_O3_audit.s:11999`-
  `:12013`).

That backend choice is not an Eigen expression distinction that can be
reliably selected in this Rust source without changing the arithmetic
contract.  More importantly, the O2/O3 difference is already present before
the final state assignment: on the frame-3-to-4 probe, native O2 produces
`dv=3ede21be,bcb31cdc,be20c273`, while native O3 produces
`dv=3ede21c0,bcb31cdc,be20c273`.  Their direct endpoint velocities are,
respectively, `3da7a8b8,bc8bed08,be67f1c8` and
`3da7a8b8,bc8bed06,be67f1c6`.  The retained Rust baseline matches the O2
endpoint.  Changing only the final `velocity` spelling cannot generally
repair this upstream delta without making the pinned source contract
compiler-dependent.

## Candidate gates

The existing source-faithful probes were retained as negative evidence:

| spelling | clean state | pinned golden | result |
| --- | ---: | --- | --- |
| packet/vector multiply-plus-add (retained) | 73/75 | exact | retained |
| first-lane `accel_world.x.mul_add(dt, old_velocity.x)` | 74/75 | velocity-x `+1 ULP` (`0.4063985049724579` vs `0.4063984751701355`) | rejected |
| all-lane FMA pre-solver variant | earlier frame-1/frame-2/frame-3 lane regressions | not exact | rejected |
| reversed `old_velocity.x.mul_add(dt, accel_world.x)` | 51/75 | invalid operand order | rejected |

The one-lane candidate's apparent state improvement is a codegen perturbation
and still fails the exact fixture; it is not a valid general helper.  A
scalar multiply-plus-add spelling would be source-equivalent to the retained
operation but cannot guarantee the native O2/O3 scheduling, so it was not
committed or benchmarked as a production change.

## Five-frame state/H/b baseline

The retained fresh-five trace has 70 factors and 1,243 rows:

| comparison | exact / total | mismatches |
| --- | ---: | ---: |
| state lanes | 73 / 75 | frame-4 velocity y/z, each +2 ULP |
| H cells | 3,194 / 5,625 | 2,431 bit mismatches |
| b lanes | 6 / 75 | 69 bit mismatches |

The two state differences are estimator/IMU-only (IMU factor index 3; no
direct visual target rows).  Candidate H/b counts were 2,431 and 69 as well,
so the candidate did not improve the downstream normal-equation boundary.
The complete comparison is in
[`target/m7du_state_hb_comparison.json`](../../target/m7du_state_hb_comparison.json).

## Verification

* `cargo test --release -p visloc-basalt --lib upstream_f32_interval_keeps_integer_endpoint_and_sophus_exp_order -- --nocapture`: **1 passed**.
* `cargo test --release -p visloc-basalt --lib`: **166 passed, 0 failed, 2 ignored**.
* No commit or push was performed.
