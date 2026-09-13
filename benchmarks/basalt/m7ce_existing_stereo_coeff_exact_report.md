# M7ce existing-stereo coefficient exactness

Date: 2026-08-23 JST

## Decision

The coefficient frontier was closed with one general arithmetic-order change
in `pipelines/basalt/src/patch.rs`.  The SE(2) rotation column is now formed
as

```rust
gradient_y.mul_add(offset_x, gradient_x * (-offset_y))
```

This is the fixed-size Eigen order for the `1x2 * 2x3` row product used by
Basalt's `valGrad.tail<2>().transpose() * Jw_se2`.  No track-, camera-, or
row-specific branch was added; `update.rs`, `aom`, and the other production
paths were not changed.

## Exact frontier

The probe used native source frame 0, cam1, timestamp
`1403636579763555584`, track 3, at full-resolution position
`0x42395ac4,0x435c70bb` (level 3:
`0x40b95ac4,0x41dc70bb`).  It used the pinned Eigen 5.0.1 headers and the
native O3 build.

Before the fix, the source sample values and all 156 raw gradient lanes were
bit exact.  The first divergent operation was the third (rotation) component
of the raw Jacobian row: the plain two-product sum rounded differently from
Eigen's fused second product.  After the correction, the authoritative
`setDataJacSe2` output is exact at every downstream stage:

| stage | lane count | native/Rust mismatches after fix |
|---|---:|---:|
| source data | 52 | 0 |
| raw value/gradient operands | 156 | 0 |
| normalized `J` (`52x3`) | 156 | 0 |
| Hessian (`JᵀJ`) | 9 | 0 |
| pivoted LDLT inverse | 9 | 0 |
| final `H⁻¹Jᵀ` (`3x52`) | 156 | 0 |

The pinned native Hessian bits are
`3eaaf131,3c225e02,bf891630,3c225e02,3f197537,3f1ffc07,bf891630,3f1ffc07,410f107e`;
the inverse bits are
`40a617b5,bf4aa79a,3f2d5109,bf4aa79b,3ff5c43d,be6a8294,3f2d510a,be6a8293,3e55f315`.

The generated fixed-size assembly confirms the order: the rotation output
starts with `gx * (-offset_y)` and uses a `vfmadd132ss` for
`gy * offset_x + accumulator`.  The prior Rust spelling
`-gx * offset_y + gy * offset_x` differed at four normalized-J lanes (first
sample 15, column 2), even though the Hessian and inverse happened to round
identically for this source patch.  The coefficient difference propagated to
10 final `H⁻¹Jᵀ` lanes, with the first at row 0/sample 15.

The existing native 3x52 coefficient/residual fixture remains in
`pipelines/basalt/tests/fixtures/m7cb_existing_stereo_gemv.json`, and both
the L3/I0 and L3/I1 all-lane packet-8 tests remain enabled.

## Verification

Focused tests passed:

```text
cargo test -p visloc-basalt patch --release --lib
  6 passed
cargo test -p visloc-basalt update --release --lib
  7 passed
cargo test -p visloc-basalt --test m7bd_obs_pixel_exact --release -- --ignored
  1 passed
cargo test -p visloc-basalt --test m7bw_cam0_temporal_exact --release
  1 passed
```

A fresh two-frame release endpoint completed in `0.458 s` and produced two
records.  Frame 0 cam1 track 3 remains exact at
`x=0x42395ac4,y=0x435c70bb`; frame 1 cam1 track 3 is now exact at
`x=0x422f68f8,y=0x435c045a`.

The fresh 80-frame release run completed in `6.147 s` and produced 80
records.  Against the authoritative native endpoint
`target/basalt_upstream_mh01_endpoint_trace.jsonl`:

```text
structure: exact
exact coordinate fields: 47,372 / 48,512
exact point pairs:       23,444 / 24,256
maximum ULP difference:  22
maximum absolute delta:  0.00048828125 px
first mismatch: frame 4, cam0, track 210, x
  native 0x4382f3cb, Rust 0x4382f3cc
```

The M7ca baseline before this fix was 36,654 exact fields, 16,237 exact
pairs, and maximum 47 ULP.  Thus this change gains 10,718 fields and 7,207
pairs while reducing the worst case by 25 ULP.  The M7ce endpoint SHA-256 is
`1bb1c9d44de3c67babd91e112110f041250bc38b3b286769f1ed2ae4ee48efee`;
the authoritative native endpoint SHA-256 is
`1ed8670e8429d12b4956c16508b4a81067098d212572ac543de9b3180b8e6dae`;
the M7ca SHA-256 is
`681c90bc646ef2af92f04f91b4fa8aa4c211f8acc17358c3918c038e134b8403`.

No commit or push was made.
