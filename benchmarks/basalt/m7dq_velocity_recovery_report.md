# M7dq velocity recovery

Date: 2026-08-23 JST  
Status: **m7dk baseline retained; interrupted m7dn velocity edit rejected**

## Recovery

The recent estimator edit was isolated to the first lane of the f32
preintegration velocity update in
[`estimator.rs:2446`](../../pipelines/basalt/src/vio/estimator.rs:2446).
The interrupted spelling was:

```text
old_velocity.x.mul_add(dt, accel_world.x)
```

which computes `old_velocity * dt + accel_world`, reversing the intended
operands.  Its fresh five-frame run produced frame-1 velocity approximately
`[-2.09665, 0.61763, 6.94266]` and only 51/75 exact state lanes.  That edit was
removed.

The source-faithful correction,
`accel_world.x.mul_add(dt, old_velocity.x)`, was also replayed.  It reached
74/75 exact state lanes against the clean native state reference and kept the
same 70 factors / 1,243 rows, but it regressed the pinned library golden:

```text
vio::estimator::tests::upstream_f32_interval_keeps_integer_endpoint_and_sophus_exp_order
left velocity-x  = 0.4063985049724579
expected          = 0.4063984751701355
regression        = +1 ULP
```

Because the gate requires a reduction without any regression, the corrected
candidate was rejected too.  The retained production expression is the m7dk
baseline:

```text
self.velocity = old_velocity + accel_world * dt;
```

No m7dn/m7dq probe labels, debug prints, `unsafe`, or environment-controlled
diagnostic branch remains in the estimator or patch production sources.

## Final five-frame comparison

The final artifact
[`target/m7dq_state_hb_comparison.json`](../../target/m7dq_state_hb_comparison.json)
uses the retained m7dk fresh detail trace against
[`target/m7db_clean_frame4_states.json`](../../target/m7db_clean_frame4_states.json).

| comparison | exact / total | mismatches |
| --- | ---: | ---: |
| retained m7dk state lanes | 73 / 75 | 2 |
| corrected candidate state lanes | 74 / 75 | 1, but golden-test regression |
| retained H lanes | 3,194 / 5,625 | 2,431 bit mismatches |
| retained b lanes | 6 / 75 | 69 bit mismatches |

The retained baseline therefore has the known frame-4 velocity-y/z `+2 ULP`
state differences.  The candidate’s one-lane improvement is recorded as
negative evidence; it is not retained because it breaks the exact frame-1
fixture.

## Verification

```text
cargo test --release -p visloc-basalt --lib
```

Result: **166 passed, 0 failed, 1 ignored**.  No new native probe was built;
the existing m7dn O2/O3 logs were used only to identify the operation-order
candidate.  No commit or push was performed.
