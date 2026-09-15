# M7cm LDLT solve exactness

Date: 2026-08-23 JST

## Decision

Adopt the general upper-triangular solve order in
`pipelines/basalt/src/patch.rs`.  For each matrix-RHS column, the solve now
forms the dot product of all already solved rows with `f32::mul_add`, then
performs one subtraction.  This is the fixed-size Eigen `H.ldlt().solve(I)`
order identified at the m7ci frontier.  It is general over the supplied 3x3
Hessian and does not inspect track, camera, frame, row, or endpoint values.

The change is proven on both available native inverse fixtures:

| fixture | inverse lanes exact |
| --- | ---: |
| original native Eigen 3x3 fixture | 9/9 |
| m7ci frame-4 cam0 temporal Hessian | 9/9 |

The m7ci JSON retains the pre-change Rust inverse as provenance.  That
baseline differs from native at row-major indices 2 and 8; the retained
regression test records those two baseline differences while requiring the
current production result to equal all nine native lanes.

Native/current m7ci inverse bits are:

```text
3f519e8f,3dc644d6,bd3213f2,
3dc644d6,3df0ad3f,bc46d83e,
bd3213f1,bc46d83e,3cd0be6f
```

The interrupted probe work was cleaned up.  `div_probe`, its `eprintln!`,
and the temporary `debug_m7cj_ldlt_boundary` test are absent.  No unsafe,
track-specific, camera-specific, threshold, lifecycle, AOM, or update-path
change was added.

## Endpoint check

A fresh release `stereo_diag` replay of MH_01_easy frames 0--79 used the
current production source and produced 80 structurally aligned records:

```text
points: 24,256
coordinate fields: 48,512
exact coordinate fields: 48,219 / 48,512
exact point pairs: 24,037 / 24,256
maximum ULP difference: 22
maximum absolute delta: 0.00048828125 px
first mismatch: frame 9, cam1, track 481, y
native: 0x428105e8 (64.51153564453125)
Rust:   0x428105e7 (64.51152801513672)
SHA-256: 7d5cbc8a1b5d94107b2508fa7c44228974f20951468c08a3da8a889d50290d81
```

Against the preceding m7ce endpoint, exactness improved by 847 coordinate
fields and 593 point pairs; the worst ULP and absolute delta did not worsen.

## Verification

```text
cargo test -p visloc-basalt patch --release --lib       7 passed
cargo test -p visloc-basalt update --release --lib      7 passed
cargo test -p visloc-basalt --lib --release             165 passed, 1 ignored
m7bd_obs_pixel_exact --ignored                            1 passed
m7bw_cam0_temporal_exact                                 1 passed
```

The m7ci test is included in the seven patch tests and checks both the
pre-change two-lane baseline and the current nine-lane native equality.  No
commit or push was made.
