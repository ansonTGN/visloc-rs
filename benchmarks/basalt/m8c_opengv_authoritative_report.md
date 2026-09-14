# M8c OpenGV/global-pose authoritative boundary report

This probe isolates the OpenGV `OptimizeNonlinearFunctor1` residual path used
by the Basalt relative-pose optimizer.  It links the pinned OpenGV archive and
calls the public `optimize_nonlinear` API; it does not link a replacement
`methods.cpp` object.

## Authoritative artifacts

- Driver: `target/m8c_cpp_lm_probe_sourcecall`
- Input: `target/m8c_feature_raw20.json`
- Seeded model input: `target/m8c_upstream_rerun.json`
- Strict multi-seed output: `target/m8c_authoritative_archive.log`
- Perturbation sourcecall trace: `target/m8c_cpp_pert_bits_new.log`
- Perturbation Rust trace: `target/m8c_rust_pert_fields_cayley_fma_all.log`
- Strict seed-7 Rust audit: `target/m8c_rust_strict_seed7_cayley_fma_reverted_tail.log`
- Perturbation trace-only driver: `target/m8c_cpp_lm_probe_sourcecall_pert`
- Rejected inverse-tail probe logs: `target/m8c_rust_*_inverse_tail_candidate.log`
- OpenGV archive SHA-256: `5be8beaed6f13438bba3af14132084f783bb46837654d604f49fd0ba3215197a`
- Driver SHA-256: `e52d43e9c00e89bd783191341c6dc94f34105b11d71632d72244535681c1780b`
- Defining archive member: third `methods.cpp.o` occurrence (ordinal 49)
- Pinned `triangulate2` symbol in that member: `0x1ff0`
- Driver-resolved `opengv::triangulation::triangulate2`: `0x6f530`
- Compiler/CPU: GCC 11.4.0; Intel i7-9750H with AVX2/FMA

The strict sourcecall run reaches the recorded seed-7 final translation:

```text
[0.23038300083075863, -0.36313027009593762, -0.078119162298351044]
```

The retained Rust production edit is the pinned OpenGV `cayley2rot` scalar-FMA
schedule.  The schedule was recovered from the authoritative binary's
disassembly and is source-faithful to `opengv/src/math/cayley.cpp`; it makes
all nine rotation-matrix values bit-exact for the unperturbed model and all
six NumericalDiff perturbations.  On the strict fixed seed-7 cam0->cam1
fixture it reduces the refined model error from
`5.72234898790827118e-3` to `3.25210279197454821e-3` while preserving the
`114/114` RANSAC and refined inlier sets.

The sourcecall trace was regenerated/audited as follows (from the repository
root under WSL):

```text
M8C_TRACE_BITS=1 M8C_TRACE_BITS_ALL=1 \
  ./target/m8c_cpp_lm_probe_sourcecall \
  target/m8c_feature_raw20.json target/m8c_upstream_rerun.json \
  > target/m8c_cpp_lm_bits_all.log 2>&1
```

The corresponding Rust fixed-point trace was run with the ignored,
test-only diagnostic (the production path is not enabled by these variables):

```text
M8C_FEATURE_RAW_JSON="$PWD/target/m8c_feature_raw20.json" \
M8C_FEATURE_PARITY_POINT_TRACE=1 M8C_FEATURE_PARITY_POINT_TRACE_ALL=1 \
  cargo test -p visloc-basalt --lib m8c_debug_fixed_oracle_lm_trace \
  -- --ignored --nocapture > target/m8c_rust_authoritative_all.log 2>&1
```

Audited output hashes:

```text
target/m8c_cpp_lm_probe_sourcecall  e52d43e9c00e89bd783191341c6dc94f34105b11d71632d72244535681c1780b
target/m8c_cpp_lm_bits_all.log      3321e30a538f310b3c7060978f894411146bf6f6ea9742f8fe4b82cb62ef4fe0
target/m8c_rust_authoritative_all.log
                                    9e4dd96100bacd6eeef71bef76c24ede3e9a0cefc1fd9f965b01a7559b1f4e8c
target/m8c_authoritative_archive.log
                                    f7b0769411fd2ec811daaa3530f395c88badcc0d520002d207baba7a6ec6b714
target/m8c_cpp_pert_bits_new.log
                                    1867512e5d590d1b86f538964f5b6256470803f2d965f34b73205f524d1a4fcd
target/m8c_cpp_lm_probe_sourcecall_pert
                                    707ce70b7f37993ac303d23acd769602536ebc5c5857b9b207f7abb8121966b4
target/m8c_rust_pert_fields_cayley_fma_all.log
                                    6fb9829292f69476e8996c1929d7a4904cf699d949970250b143b5ac963d1512
target/m8c_rust_strict_seed7_cayley_fma_reverted_tail.log
                                    e9597e664a1bd85603c13ccdbcfd28f58f0b540ae65647195fe92bdd0f20afea2
target/m8c_rust_strict_seed7_inverse_tail_candidate.log
                                    c169459660dc948eca798d81863660286f52aec2447134d90a17d4b2e3e3677d
target/m8c_rust_pert_bits_inverse_tail_candidate.log
                                    3a88d4e072181ede09699a596289995888f1984d5b952b811f07757027c6d16e
```

## Exact seed-7 residual comparison

The first contiguous `TRACE_BITS` group after `seed 7 n 114` in the C++ log
contains all 114 inliers.  C++ prints the row index in hexadecimal, so the
comparison parses that index as base 16 and normalizes only hexadecimal
leading zeroes (`0` and `0000000000000000` are the same bits).

```text
stage                         exact rows       first mismatch
unperturbed residual path      114/114          none
perturbed p (all six columns)  684/684          none
perturbed inverse rotation     684/684          none
perturbed inverse translation  456/684          col=1/5, scalar z (all rows)
perturbed homogeneous second   680/684          col=4 row=97; col=5 rows=20,35,57
perturbed norms/dots            680/684          follows homogeneous-second drift
perturbed residual             682/684          col=4 row=97; col=5 row=57
```

The unperturbed sourcecall/Rust path remains exact for all 114 rows.  With the
retained Cayley schedule, the first seeded NumericalDiff divergence is in the
inverse translation `-R.transpose() * t`, specifically the scalar third-row
component of the Eigen 3x3-by-3x1 product for translation column 1 and Cayley
column 5 (one ulp in every row).  The point and inverse-rotation fields are
exact before that operation.  A separate later one-ulp homogeneous-product
divergence occurs at column 4 row 97 and column 5 rows 20, 35, and 57.  Only
the latter two rows change the final residual bits; all other 682/684
perturbed residuals are exact.  Thus the strict refined-model drift is now
narrowed to Eigen product/reduction ordering in the finite-difference
evaluation, not RANSAC sampling, branch selection, triangulation, scoring, or
the Cayley conversion.

The sourcecall seed-7 initial model in the strict log is:

```text
t = [0.1748501444182278, -0.62903855231643102, -0.27154728261802763]
R = [[ 0.99970128082127652,  0.024177574478791791, 0.0035768724640417725],
     [-0.024005520736904633, 0.99882287103597667, -0.042149819330591316],
     [-0.0045917424199968023, 0.042051363685068933, 0.99910489875376796]]
```

## Assembly/callsite boundary

In the authoritative sourcecall driver, the inverse-translation assignment is
the Eigen `-R.transpose() * t` kernel at `0x2cc00`.  Its scalar tail is:

```text
z = (-m21) * t1                       # vmulsd
z = fma(-m22, t2, z)                  # vfnmadd231sd
z = fma(-m20, t0, z)                  # vfnmadd231sd
```

The fixed-operand probe reproduced this schedule and removed the observed
third-component one-ulp differences, but it did not remove either of the two
final residual mismatches (`2/684` remained) and made the strict Rust seed-7
refined error worse (`5.53315862915335077e-3` versus `3.25210279197454821e-3`
with only the retained Cayley edit).  It is therefore reverted.  The generic
Rust `opengv_eigen_matvec3` schedule remains in production because it also
serves triangulation and model-preparation products; a callsite-specific
substitution would not improve the fixed oracle.

The inverse homogeneous product is the Eigen
`Matrix<double,3,4> * Matrix<double,4,1>` assignment at `0x2cc90`.  Its first
two lanes use `vmulpd` followed by `vfmadd132pd` operations; the scalar third
lane uses an FMA pair and final add.  The residual dot/norm path uses a
two-lane multiply/add reduction and an FMA for the third component.  These
packet-versus-scalar schedules explain the remaining compiler boundary; the
Rust `opengv_eigen_mat34_vec4`, `opengv_eigen_matvec3`, `opengv_eigen_norm3`,
and `opengv_eigen_dot3` helpers preserve the broader exact path.

An earlier report section attributed a plain multiply/add product at `0xa990`
to this authoritative driver.  That was an artifact mix-up.  `0xa990` belongs
to the separate `target/m8c_cpp_lm_archive_probe` binary (SHA-256
`b391e91685a003c70210349b7515fab99f128ff715bdc855280f8d9c34623e32`), which
contains a custom traced triangulation path and no
`OptimizeNonlinearFunctor1`.  Its plain-product rows are retained as
non-authoritative experiments and are not parity fixtures.

Consequently, the previously tried plain-add mat34/dot candidates and the
source-faithful inverse-translation-tail substitution were tested and reverted:
none reduced the remaining `2/684` residual rows while preserving the strict
oracle improvement.  No further production edit is justified.

## Result

The M8c OpenGV/global-pose boundary is closed through RANSAC, model selection,
triangulation, and the unperturbed residual path.  A source-faithful Cayley
FMA edit is retained because it improves the strict fixed oracle.  The exact
remaining boundary is the finite-difference inverse-translation Eigen product
described above; no speculative refinement edit is retained.
