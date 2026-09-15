# M7aw: f32 JacobiSVD FMA portability review

Date: 2026-08-22 JST

## Conclusion

The explicit `f32::mul_add` operations in
`pipelines/basalt/src/vio/landmarks.rs` (Jacobi left/right application and the
2x2 Jacobi helper) reproduce the pinned native AVX/FMA oracle.  They are safe
on targets without FMA: Rust lowers `mul_add` to a fused software/library
operation when the target has no FMA instruction.  This is, however, a fixed
fused-rounding contract, not a promise to reproduce an upstream C++ build
compiled without FMA.

No production source was changed for this review.

## Evidence and commands

Native focused test:

```text
C:\Users\rsasa\.cargo\bin\cargo.exe test -p visloc-basalt m7_first_track_triangulation_matches_pinned_native_bits --lib -- --nocapture
```

Result: `vio::landmarks::tests::m7_first_track_triangulation_matches_pinned_native_bits` passed (`1 passed`).

The retained no-FMA Rust artifact is under `target/review_nonfma/`; its
fingerprint records `rustflags: ["-C", "target-feature=-avx,-fma"]`.

```text
target\review_nonfma\debug\deps\visloc_basalt-ede1541e921e09b1.exe vio::landmarks:: --nocapture
```

Result: all 10 landmark tests passed, including the exact first-track test.
The full retained binary run gave `150 passed, 1 failed, 1 ignored`.

## Exact-bit comparison

Pinned FMA oracle artifacts:

* `target/m7_triang_dlt_probe.log`
* `target/m7_triang_dlt_probe_novec.log`

Both produce the same terminal bits:

```text
raw_bits=bf28a750;be994a0a;3f2cbdf9;3e147902
normalized_bits=bf2a746e;be9aed26;3f2e9645;3e160ef3
projected_bits=becaaef7;be383810
```

The Rust exact test locks the corresponding raw/final values:

```text
raw       [bf28a750, be994a0a, 3f2cbdf9, 3e147902]
actual    [bf2a746e, be9aed26, 3f2e9645, 3e160ef3]
projected [becaaef7, be383810]
```

Non-FMA native artifacts:

* `target/review_native_nofma.log`
* `target/review_native_nofma_vcpkg.log`

The two builds agree with each other, but differ from the pinned FMA result:

```text
raw_bits=bf28a750;be994a09;3f2cbdfa;3e147918
normalized_bits=bf2a746e;be9aed25;3f2e9646;3e160f09
projected_bits=becaaef6;be38380d
```

The DLT input already differs before JacobiSVD (`A` row 2 begins
`bf2dd17a,3c0a2d1b` in the FMA oracle versus `bf2dd179,3c0a2d1a` in both
non-FMA logs), so the terminal delta is not attributable only to the Jacobi
rotation expressions.

## Known unrelated failure

The one failure in the retained no-FMA full-suite run is:

```text
vio::estimator::tests::upstream_f32_interval_keeps_integer_endpoint_and_sophus_exp_order
```

Its observed z position is `-0.0035189196933060884` versus the test's expected
`-0.003518919460475445`.  The current normal test binary
`target/debug/deps/visloc_basalt-36152d1f44541bfa.exe` fails the same test with
the same values, while the focused landmark tests pass; this is therefore
pre-existing/unrelated to the JacobiSVD review.
