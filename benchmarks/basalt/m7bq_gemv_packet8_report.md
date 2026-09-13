# m7bq fixed packet-8 GEMV

Date: 2026-08-23 JST

## Result

The remaining L3/I1 scalar is in the fixed `3x52 * 52x1` inverse-compositional
GEMV.  `pipelines/basalt/src/patch.rs` now uses a general unrolled helper for
this shape instead of nalgebra's generic product.  The helper preserves the
operation order emitted by the pinned Eigen 5.0.1 AVX packet-8 build:

- 20 short FMA lanes are seeded with a plain multiply and folded with
  `f32::mul_add`;
- the lanes cover the generated groups `(0..5), (6..12), (13..18),
  (19..25), (26..31), (32..38), (39..44), (45..51)`;
- the eight groups are combined with the generated binary horizontal tree.

This is a fixed-shape implementation, but not a fixture or track correction:
all coefficients and residual samples are read from the supplied 3x52 and
52x1 inputs.

## Native decode

The upstream probe was built from Eigen tag `5.0.1`
(`bc3b39870ecb690a623a3f49149a358b95c5781d`) with:

```text
g++ -std=c++17 -O3 -march=native -DEIGEN_DONT_PARALLELIZE
```

The generated row expression starts with the lane order
`(4,5,3), (1,2,0), (11,12), (9,10), (7,8,6)` and continues with the same
three-/two-term pattern through sample 51.  Its final reduction is:

```text
(((A+B)+((C+D)+E))+((F+G)+((H+I)+J)))
 + (((K+L)+((M+N)+O))+((P+Q)+((R+S)+T)))
```

The no-vector Eigen probe produces a different result, confirming that a
portable scalar fold is not the pinned contract.

## Exact fixtures

The patch unit test `packet8_gemv_matches_native_l3_i0_and_i1_all_lanes`
embeds the native L3 Hessian-inverse/Jacobian product and both residuals.  It
checks every increment lane:

| fixture | native increment bits `(x,y,theta)` | result |
| --- | --- | --- |
| L3/I0 | `400fc2da, 3f9be212, 3ea0024b` | exact |
| L3/I1 | `bf47a90b, bd615534, be91b7c1` | exact |

Focused verification passed:

```text
cargo test -p visloc-basalt patch --lib
5 passed
cargo test -p visloc-basalt packet8_gemv_matches_native_l3_i0_and_i1_all_lanes --release --lib
1 passed
```

The existing Hessian, LDLT, identity IC, and all-black patch tests remain
covered by the same patch suite.

## Endpoint and runtime check

A fresh optimized `stereo_diag` run used the pinned MH_01_easy inputs and
completed in `0.314 s` after the release binary was rebuilt.  The track-3
cam1 endpoint remains `x=0x42395ac3, y=0x435c70bb`; the pinned endpoint is
`x=0x42395ac4, y=0x435c70bb`.  Thus the isolated L3/I1 GEMV fixture is exact,
but this one candidate does not yet close the downstream endpoint ULP.

The 80-frame replay completed in `7.122 s` wall time.  It produced 80/80
records, with zero track-count or track-ID differences, and direct bitwise
comparison against the frozen endpoint values found `19,692/48,512` exact
coordinate fields (`6,362/24,256` exact point pairs).  The repository's
`mh01_stereo_golden` test currently cannot consume that frozen file because
its JSON lines omit the test's required `frame_index`; the parity numbers
above use timestamp-aligned records and the same f32-bit comparison.

No track-specific branch or endpoint correction was added.
