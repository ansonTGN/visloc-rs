# M7fe IMU delta schedule exactness gate

Date: 2026-08-23 JST  
Status: **candidate rejected and reverted**

The scoped M7fc candidate was applied temporarily in
`pipelines/basalt/src/vio/estimator.rs`: fixed 3x3 matrix-vector rows used
`R0.mul_add(ax, R2.mul_add(az, R1 * ay))`, while only the x lane of the
`delta_velocity` update used `accel_world.x.mul_add(dt, old_velocity.x)`;
y/z retained `old + accel * dt`.  No frame/value branch, debug output, or
unsafe code was introduced.

The focused frame-1 golden passed:

```text
cargo test --release -p visloc-basalt --lib upstream_f32_interval_keeps_integer_endpoint_and_sophus_exp_order -- --nocapture
1 passed, 0 failed
```

The full library suite also passed while the candidate was present:

```text
cargo test --release -p visloc-basalt --lib
171 passed, 0 failed, 1 ignored
```

The source-stable five-frame release replay processed 5/5 frames, delivered
42 IMU samples, emitted 1,114 observations, and reached 70 factors / 1,243
rows.  Against the clean M7db state capture, the candidate reached **74/75**
exact binary32 state lanes.  The sole residual was frame-4 translation z:

```text
native  bcdc2b7f
candidate bcdc2b7e
delta -1 ULP
```

Because the required gate is 75/75, the candidate was reverted immediately;
the retained estimator expression is the prior source-faithful schedule.
The machine-readable comparison is
[`target/m7fe_state_comparison.json`](../../target/m7fe_state_comparison.json).
The replay detail and summary are retained as
`target/m7fe_fresh5_detail.jsonl` and `target/m7fe_fresh5_run/summary.txt`.

No commit or push was performed.  Existing unrelated worktree changes were
left untouched.
