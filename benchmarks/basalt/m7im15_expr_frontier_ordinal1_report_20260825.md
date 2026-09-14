# M7IM15 ordinal-1 named expression frontier (2026-08-25)

Pinned detached Basalt commit: `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.
The native expression trace and M7IM15INP2 ordinal-1 record are fresh live captures; no reconstructed native oracle was used.

## Boundary selection

- Native ordinal `1`, Rust frame `4` / iteration `0` / block `1 -> 2`.
- start timestamp exact: `True`; dt exact: `True`.
- State inputs: `64/64` values exact.
- Measurement inputs (delta matrix/position/velocity, dstate/dbg/dba, covariance): `147/150` values exact.

## Named frontier

- Expression aggregate: `73/91` values exact.
- First named expression difference: `translation_operand[0]`, native `39e0cf9f`, Rust `39e0cfa0`.
- The first differing named operation is therefore `translation_operand = translation_delta - velocity_term - gravity_term`; its three named operands and all state position/velocity inputs are exact, so the next candidate is the source evaluation/rounding schedule of this subtraction.
- Subsequent named rotation boundary: `R0_inv`.

## Input/raw boundary

- State/measurement first difference: `delta.rotation_matrix[0]`, native `3f7ffee0`, Rust `3f7ffee1`.
- Raw residual: `8/9` exact; first mismatch lane `1` native `ae800000` vs Rust `00000000`.
- Raw Jacobian: `258/270` exact; first mismatch flat column-major index `2` (row `2`, column `0`) native `3f744e16` vs Rust `3f744e15`.
- Sqrt information: `81/81` exact.
- The first raw residual/J differences are upstream of blockwise whitening; blockwise W scheduling cannot explain them.

## Provenance

- Expression capture SHA-256: `1835f432cc629314d02cb624cf705780b946101b2fc25515856c1ef347158c89`.
- Local input capture SHA-256: `7decba8e65d25804fb765e9ecaaa73fd7b21533ca0ebb7714fbf59c7ed8fc23f`.
- Rust sidecar SHA-256: `a46fc830948ee2fd0ec565167969a5ba6c48230c650319d42441f349e05fdef6`.
- Detached native binary SHA-256: `438b9ae448f5f977d7305b28106f8c3a0a8870cab6a92a29e3a31d6e0cfa3e8f`.
- Logger source SHA-256: `497f0d235d331948d050acbfade8c6fd03a0b85668cd415562a92fc00c38727e`.
- Capture filter: `M7IM15_NATIVE_EXPR_TRACE_CALL_ORDINALS=1`; direct source capture only.
