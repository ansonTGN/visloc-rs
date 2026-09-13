# m7bt post-GEMV track-3 boundary

Date: 2026-08-23 JST

## Boundary and scope

The packet-8 `3x52 * 52x1` GEMV is exact through L3/I1.  The next native/Rust
boundary was the SE2 update composition immediately after that increment:

```text
L3/I1 post-update translation.y: native 0x41d9e26b, Rust 0x41d9e26c
```

This report closes that one boundary.  No track-specific branch, endpoint
correction, landmark change, estimator change, or provenance change was used.

## Native operation order

The oracle was built from Eigen tag `5.0.1`, commit
`bc3b39870ecb690a623a3f49149a358b95c5781d`, with:

```text
g++ -std=c++17 -O3 -march=native -DEIGEN_DONT_PARALLELIZE
```

Basalt stores the warp as Eigen `AffineCompact2f` and applies
`transform *= SE2::exp(inc).matrix()`.  The fixed-size `2x3 * 3x3` packet
kernel seeds each output with its second inner coefficient, folds the
homogeneous third coefficient, then folds the first coefficient.  The Rust
`AffineCompact2f::right_compose_se2` now spells that order explicitly with
`mul_add`, including the homogeneous `1.0`/`0.0` terms.  `Se2::exp` also uses
the upstream direct Sophus translation products instead of a generic
nalgebra matrix-vector product.

## Exact fixture

`pipelines/basalt/src/update.rs` adds
`right_composition_matches_native_l3_i1_boundary`.  It feeds the native
L3/I1 pre-update warp and packet-8 increment and checks the six compact
homogeneous values:

```text
3f7fe679 bce4a105 40c07598
3ce4a103 3f7fe67a 41d9e26b
```

The temporary trace comparison (`target/m7bt_track3_trace_comp.txt` against
the native flow probe) is exact for residual, increment, and post-update
transform values at L3/L2/L1/L0 iterations I0 through I4.  Temporary stream
trace instrumentation was removed after capture.

## Verification

```text
cargo test -p visloc-basalt patch --release --lib       5 passed
cargo test -p visloc-basalt update --release --lib      7 passed
cargo test -p visloc-basalt --test m7bd_obs_pixel_exact --release -- --ignored
                                                         1 passed
```

A fresh optimized one-frame endpoint run produced the required track-3 cam1
bits without a track branch:

```text
x = 0x42395ac4
y = 0x435c70bb
wall = 0.855 s
```

The fresh 80-frame replay produced 80 records with no count or ID differences.
Against `mh01_stereo_endpoint_v1_first80.jsonl`, direct f32-bit comparison
found:

```text
exact fields = 36,617 / 48,512
exact point pairs = 16,216 / 24,256
maximum absolute difference = 0.00048828125
maximum ULP difference = 47
first remaining mismatch = frame 1, cam1, track 117, y:
  native 0x4249983c, Rust 0x42499838
wall = 14.188 s
```

The repository's frozen `mh01_stereo_golden` fixture currently omits the
deserializer-required `frame_index`; therefore the first-80 numbers above
come from timestamp-aligned direct comparison.  No commit or push was made.
