# M7fo Jp reduction exactness search

Date: 2026-08-23

## Result

No production change was retained. The existing M7fl/fj `eigen_landmark_jacobian_f32`
pair reduction remains the known-green implementation. A value-specific branch or
non-Eigen reduction would violate the requested common-schedule constraint.

## Search

An external Rust `f32` enumerator evaluated all four-term scalar dot-product
reduction trees made from correctly rounded `f32` multiply/add and `f32::mul_add`,
including:

- all 24 term orders;
- both operand orientations for FMA;
- left-accumulator chains (k0 followed by k1, k2, k3);
- pairwise reductions and both pair/FMA orientations; and
- the equivalent binary expression trees with one multiply leaf fused into an
  adjacent add where legal.

The ordinal 0, ordinal 13, and ordinal 43 fixtures all had matching trees. The
existing pair schedule also remains exact for those fixtures. For the ordinal105
values recorded in `target/m7fn_ordinal105.json`, the expected raw Jp is:

```
4450c412 c206f0e0 c2075714 4455c63b 00000000 00000000
```

The same listed camera-J/Jpp inputs produce, under the current pair schedule:

```
4450c413 c206f0e4 c2075714 4455c63b 00000000 00000000
```

and under the best left-to-right chain tested:

```
4450c413 c206f0e3 c2075714 4455c63b 00000000 00000000
```

Most importantly, the ordinal105 second lane (`c206f0e0`) had **zero** matching
legitimate reduction trees in the exhaustive search; the other five lanes had
matches. Therefore no single legal Eigen-style f32 schedule can satisfy all
listed J/Jpp/expected cases. The discrepancy is upstream input provenance or
capture rounding, not a reduction-order choice.

## Gate decision

Because the required common schedule does not exist for the supplied ordinal105
triplet, no production helper, fixture test, or benchmark claim was changed in
this attempt. The M7fl/fj implementation and its green gate set are preserved;
no debug/unsafe production code or ignored tests were introduced.

