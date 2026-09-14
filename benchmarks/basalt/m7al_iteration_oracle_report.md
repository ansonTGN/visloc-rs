# M7al iteration oracle — MH01 frame 4

This bounded run compares the pinned upstream checkout at
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc` with the Rust frame-4 detail
trace.  The hooks are opt-in (`BASALT_VIO_ITERATION_TRACE` and
`VISLOC_BASALT_DETAIL_TRACE`) and write the same `basalt.vio_iteration.v1`
JSONL schema.  They include canonical AOM/state order, state/pose/velocity/
bias, landmark host/direction/rho, raw and Huber-weighted observations,
weighted Jp/Jl, per-landmark QR rows/rhs, global H/b, damping, step,
back-substitution, cost, lambda, and decision.

## Artifacts

- Full upstream trace (valid, 1495 records): `target/m7al_upstream_f4_20260821T000200Z/iteration.jsonl`
  (SHA-256 `18E87CFF493F758729F91E8270C9968E55BC8FB33CDCF1C3307FCE9119711F7A`).
- Full Rust trace (1 header + 24 snapshots): `target/m7al_rust_f4_20260821T000300Z.jsonl`
  (SHA-256 `9A9195D36B8FD1FF7E6DE48B74486D79516131AA03D688093DF3FF60D5B3E152`).
- Compact canonical diff: `target/m7al_iteration_diff_f4.json`.
- Reproducible comparator: `benchmarks/basalt/m7al_iteration_diff.py`.

The upstream trace contains 491 landmark-linearization records, 491 QR
records, and 488 back-substitution records.  The frame-4 solve has 5 AOM
blocks, 61 landmarks, a 75-column state system, 1243 global Q2 rows, and
1168 visual rows.  Upstream declares `float32`; Rust declares `f64`.  The
header records the relevant cast/source symbols (`ImuData<double>` to the
upstream float estimator queue, Rust calibrated IMU and visual factors kept
as `f64`).

## First differences

Canonicalization maps Rust local frame IDs to timestamps and sorts landmark
payloads by track ID.  Upstream landmark traversal is an
`aligned_unordered_map`; its observed order is retained as an audit field but
is not treated as a semantic mismatch.  The final upstream iteration has a
60-column active QR representation while Rust retains 75 columns; the extra
Rust columns are exactly zero and are reported as padding by the comparator.

At tolerance `abs=rel=1e-6`, the first payload difference is already in the
iteration-0 state/prediction:

```text
blocks[state,1403636579763555584].pose.quaternion_xyzw[0]
upstream = -0.05285637080669403
rust     = -0.05286063386271856
absolute = 4.263056024529643e-06
```

The first canonical backend visual scalar is track 1, observation 0, raw
residual x:

```text
landmark_factors[track_id=1].observations[0].raw_residual[0]
upstream = -0.008089065551757812
rust     = -0.008063689438984056
absolute = 2.5376112773756176e-05
```

The first control-flow difference is LM iteration 3, trial 0: upstream
accepts and Rust rejects.  The resulting traces are upstream `AAAAAAAA` and
Rust `AAARAAAA`.  This remains true after correcting the ABS_QR row-span
contract; therefore the trace does not prove a unique solver correction.

The valid upstream rerun starts at `4215.9326171875` and Rust at
`4215.972884831595` (delta `0.040267644095`).  At iteration 0 the model cost
is upstream `1866.80517578125` versus Rust `1866.803117544874`; actual trial
cost is upstream `412.955017089844` versus Rust `412.907968449868`.
The earlier upstream golden `4215.93212890625` differs by one accumulation
variation; two upstream runs differ by one ULP in global `H[2][4]` before any
semantic track-level difference, consistent with unordered/TBB accumulation.

A bounded f32 shadow solve was also run.  Casting the Rust H/b/damping to
f32 changes the step toward upstream but still leaves max delta error
`9.1266e-05`; a NumPy solve of the upstream H/b itself differs from the
upstream recorded step by `1.16e-05`.  Scalar width contributes, but this is
not a proof that an f32 cast alone explains the rejection, so no threshold or
production numeric policy was changed.

## Implemented diagnostic/source-contract change

`aom.rs` now appends ABS_QR's three zero landmark-damping rows before QR,
removes only the three Q1 rows, and preserves all observation rows in Q2.
This is required by the pinned upstream storage contract (and changes the
Rust visual span from 985 to 1168 rows); it is covered by
`abs_qr_keeps_observation_rows_after_zero_landmark_damping_rows`.  The
iteration event callback and normal trace path are no-op unless the opt-in
environment variable is set.  No frontend, mapper, marginal-data schedule,
or threshold changes were made.

## 5/20-frame and state check

Both pre-correction and corrected 5-frame runs processed 5/5 frames with
1114 observations.  Frame-4 cross-runtime translation error at the final
iteration-7 state is `1.148359 mm` before and `1.148359 mm` after (the latter
is `1.1483588 mm`); the LM trace is unchanged.  The corrected release binary
also processed 20/20 frames (`4724` observations) in
`target/m7al_rust_run20_after_qr/summary.txt`.

Against the existing 20-frame upstream core trace, Euclidean translation
errors at frames 4/7/9/14/19 are respectively `1.052/3.249/6.671/19.954/37.391
mm`.  These are cross-runtime state errors, not GT comparisons; the pinned
sensor-only run exposes no GT to the engine.

## Verification

- `cargo test --release -p visloc-basalt vio::aom::tests --lib`: **18 passed**.
- `rustfmt --check` on the touched `aom.rs`/`window.rs`: **passed**.
- `python -m py_compile benchmarks/basalt/m7al_iteration_diff.py`: **passed**.
- `cargo check --release -p visloc-basalt --all-targets`: blocked by the
  pre-existing mapper `features.rs:767` `Complex<f64> / f64` error; mapper is
  outside this task and was not touched.

Conclusion: retain the opt-in canonical oracle and the proven ABS_QR row
contract regression, but do not claim a numerical fix for the LM divergence.
