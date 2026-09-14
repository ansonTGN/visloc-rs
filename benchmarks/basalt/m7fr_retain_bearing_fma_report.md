# M7fr retain bearing-J diagonal FMA

Date: 2026-08-24 JST  
Status: **Retained — live native evidence and Jp fidelity take precedence over
the transient aggregate-H exact-count regression.**

## Adoption decision

`StereographicDirection::bearing_jacobian_f32` retains explicit source-level
FMA contractions for both active stereographic diagonal terms:

```rust
(-x2).mul_add(norm_inv2, norm_inv)
(-y2).mul_add(norm_inv2, norm_inv)
```

This is one general binary32 arithmetic schedule, with no value-, ordinal-,
camera-, or debug-only branch.  The live clean-native ordinal-105 capture is
stronger evidence for the bearing/Jpp/Jp boundary than the aggregate H count:
the candidate reproduces the actual in-context Jpp operand and improves the
first remaining weighted Jp frontier.  The two-lane H exact-count decrease in
the transient M7fq aggregate is recorded and accepted as the tradeoff for the
source-faithful local boundary.

No `debug!`, `unsafe`, or `#[ignore]` was added.  The one ignored test reported
by the full library suite is the pre-existing diagnostic M8c test.

## Live ordinal-105 fixture

The permanent fixture/test is:

- [`m7fq_live_ordinal105.json`](../../pipelines/basalt/tests/fixtures/m7fq_live_ordinal105.json)
- `vio::aom::tests::m7fq_live_ordinal105_bearing_jacobian_and_jp_are_bitwise_exact`

It is derived from the bounded live capture
[`target/m7fp_live_ordinal105_inputs.json`](../../target/m7fp_live_ordinal105_inputs.json),
not the isolated M7fn replay.  In particular, it asserts the captured
in-context operands/results:

```text
Jpp: 3fba8d4f,be563110,3f710856,00000000,be563110,3fcc6805,3f2db603,00000000,00000000,00000000,00000000,3f800000
Jp:  4450c412,c206f0e0,c2075714,4455c63b,00000000,00000000
```

The isolated M7fn lane `3fba8d50` is deliberately not used.  The test runs
the live direction, point, projection, camera-J, homogeneous Jpp, raw Jp, and
weighted Jp chain, so the retained contractions are exercised rather than
only asserting a hand-loaded product.

## Verification

Focused M7 suite, including all 27 prior gates plus the permanent live replay:

```text
cargo test -p visloc-basalt --lib m7 -- --nocapture
28 passed, 0 failed
```

Full Basalt library (debug and release):

```text
cargo test -p visloc-basalt --lib
176 passed, 0 failed, 1 ignored

cargo test -p visloc-basalt --lib --release
176 passed, 0 failed, 1 ignored
```

The existing candidate fresh-five artifacts remain sufficient; no second
replay was required.  The M7fq MH_01 frame-4 iteration-start comparison has
the following candidate counts:

| quantity | M7fl baseline | retained diagonal-FMA candidate |
|---|---:|---:|
| projection lanes | 1,168 / 1,168 | **1,168 / 1,168** |
| raw residual lanes | 1,168 / 1,168 | **1,168 / 1,168** |
| weighted landmark Jp lanes | 3,423 / 3,504 | **3,476 / 3,504** |
| weighted landmark Jp six-lane rows | 536 / 584 | **571 / 584** |
| raw landmark Jp lanes | 3,189 / 3,504 | **3,240 / 3,504** |
| raw landmark Jp six-lane rows | 384 / 584 | **408 / 584** |
| aggregate H lanes | 3,301 / 5,625 | 3,299 / 5,625 |
| aggregate b lanes | 6 / 75 | 6 / 75 |

The replay retained 5/5 frames, 584 visual observations, 1,168 visual rows,
70 factors, 1,243 rows, and state DoF 75.  Comparing its frame-4 snapshot
against the clean state capture gives the established **74/75 compact AOM
state lanes** (79/80 including quaternion `w`), with the same lone frame-4
translation-z one-ULP residual as M7fl.

Artifacts:

- [`target/m7fq_fresh5_detail.jsonl`](../../target/m7fq_fresh5_detail.jsonl)
- [`target/m7fq_hb_comparison.json`](../../target/m7fq_hb_comparison.json)
- [`target/m7fq_fresh5_run/summary.txt`](../../target/m7fq_fresh5_run/summary.txt)

No commit or push was performed.
