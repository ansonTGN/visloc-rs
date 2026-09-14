# MH01 frame-9 prior-dimension panic diagnosis

Status: diagnosed read-only; no production source, commit, or push changed.

## Finding

The failed executable called the audited `eigen_prior_row_major_gemv_21`
implementation with a 27x27 compact prior. Its contract is exactly 21x21;
the assertion's `left: 27, right: 21` is therefore a stale shape-specialized
dispatch failure, not numerical corruption.

The current worktree already has the required general fix at
`pipelines/basalt/src/vio/aom.rs:4696-4709`: dispatch to the 21x21 helper only
when rows, columns, and vector length are all 21, otherwise use the dynamic
Eigen GEMV. The failing binary was built before that guard (run executable
timestamp 20:38; current `aom.rs` timestamp 21:31).

## Why frame 9 is the first 27-D case

Pinned upstream MH01 schedule (`benchmarks/basalt/m7x_window_schedule_oracle.md:37-43`)
has KF 0 converted to a pose at frame 4, then KF 7 converted at frame 9.
At frame 9, before the shift, the carried prior retains:

```text
pose(frame 0): 6 + pose(frame 7): 6 + nav state(frame 8): 15 = 27 columns
```

Its velocity/bias (9 columns) are marginalized, while the newest state
(frame 9) is excluded from the pre-marginal AOM. Thus a 27x27 prior is the
expected Basalt lifecycle result. The first captured event in
`target/basalt_mh01_qmat_fused_80f_20260828/marg.json` is event 0, 21x21
(keep=21, marginal=39, Q2=96x60), which is the earlier frame-4 boundary; the
run dies before the frame-9 event can be captured.

## Earliest owner and regression

The earliest incorrect owner is the prior RHS reduction call chain:
`window.rs:2531-2617` attaches the compact-to-global column map;
`aom.rs:3856-3873` routes a `Prior` factor;
`aom.rs:4662-4693` forms compact `Hᵀr`; the old code then entered the
21-only helper unconditionally. Keep the 21x21 audited path, but make the
shape guard mandatory and use the dynamic GEMV for every other compact size.

Focused regression: construct a `FactorKind::Prior` with a 27x27 compact
Jacobian (or a 27-column `prior_state_columns` map), run the f32 reducer, and
assert it returns a finite 27-vector / 27x27 contribution without panic;
compare the dynamic helper to the generic row-major GEMV. Add an integration
smoke that reaches MH01 frame 9 and asserts the second prior has 27 columns.
