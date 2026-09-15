# M7bb: pinned Eigen landmark norm packet reduction

Date: 2026-08-22 JST  
Pinned native Basalt revision: `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`

## Finding

The last landmark-only mismatch was the f32 normalization in
`pipelines/basalt/src/vio/landmarks.rs`.  The pinned source does not normalize
the SVD column as a standalone `Vector3f`; `BundleAdjustmentBase::triangulate`
does:

```cpp
worldPoint /= worldPoint.template head<3>().norm();
```

In Eigen 5.0.1, `squaredNorm()` is implemented as a squared unary expression
followed by `sum()`.  The fixed-size `VectorBlock` evaluator used by
`head<3>()` has a different packet/reduction schedule from a standalone
`Vector3f`.  Under the pinned `-O3 -march=native -DEIGEN_DONT_PARALLELIZE`
build, the relevant scalar contraction is:

```text
norm = sqrt(x*x + (z*z + y*y))
```

with both additions contracted.  The safe Rust spelling is therefore:

```rust
x.mul_add(x, z.mul_add(z, y * y)).sqrt()
```

The prior spelling `x.mul_add(x, y.mul_add(y, z * z))` models the standalone
vector reduction and produced the pinned Track 2 norm `3f7b7f58` instead of
the `VectorBlock` value `3f7b7f59`.  This is an expression/evaluator contract,
not a track-ID or value-specific correction.

## Native reduction evidence

The retained probe `target/m7_norm_probe.cpp` exercises the pinned Eigen
headers.  For the Track 2 raw SVD direction
`bf1e7195,be82723d,3f381964`, it reports:

| expression | squared norm | norm |
| --- | ---: | ---: |
| standalone `Vector3f::norm()` | `3f7712f6` | `3f7b7f58` |
| `Vector4f::head<3>().norm()` block | `3f7712f7` | `3f7b7f59` |
| `x*x + (y*y + z*z)` | `3f7712f6` | `3f7b7f58` |
| `x*x + (z*z + y*y)` with contracted adds | `3f7712f7` | `3f7b7f59` |

For the Track 4 raw direction, the same block spelling gives norm bits
`3f7d9191`; Track 1 and Track 4 terminal normalized lanes therefore remain
unchanged while Track 2 returns to its pinned bits.

## Rust changes

`eigen_norm3_f32` now spells the pinned VectorBlock reduction.  It is used at
the f32 bearing conversion, homogeneous DLT normalization, and
stereographic projection boundaries.  The temporary Track 2 `eprintln!`
diagnostic was removed.  A focused regression test,
`m7_eigen_vector_block_norm_reduction_matches_pinned_lanes`, locks the Track 2
and Track 4 norm lanes before the end-to-end landmark fixtures.

## Verification

Command:

```text
C:\Users\rsasa\.cargo\bin\cargo.exe test -p visloc-basalt --release --lib m7_ -- --nocapture
```

Result: `15 passed, 0 failed, 0 ignored`; this includes the three exact
landmark fixtures (Tracks 1, 2, and 4), the new reduction regression, and the
existing M7 camera/AOM fixtures.

Command:

```text
C:\Users\rsasa\.cargo\bin\cargo.exe test -p visloc-basalt --release --lib vio::landmarks::tests -- --nocapture
```

Result: `13 passed, 0 failed, 0 ignored`.
