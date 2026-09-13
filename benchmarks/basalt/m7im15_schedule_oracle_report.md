# M7IM15 Eigen product schedule oracle

Date: 2026-08-25 JST  
Pinned upstream commit: `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`

## Result

`target/m7im15_test_new.json` is regenerated with the GCC 11.4 AVX build of
`m7im15_schedule_oracle.cpp`.  All bit comparisons are exact:

| probe | H | b (packet + scalar FMA) | b (packet + scalar mul/add) |
| --- | ---: | ---: | ---: |
| actual 30x15 input | 0/900 | 0/30 | 0/30 |
| synthetic dense input | 0/900 | 0/30 | 0/30 |

The padded full-state probes remain intentionally non-exact.  They exercise a
different dynamic product shape and demonstrate that zero-padding a larger
state is not equivalent to the native 30-column product kernel.

## Decoded schedule

The local `Jp.transpose() * Jp` product uses the AVX `Packet8f`, `mr=24`,
`nr=4`, and an eight-depth peel:

- rows `0..23` use the full Packet8 path;
- rows `24..27` use Packet4 C/D accumulators, reduce the even/odd k lanes,
  then fold scalar k `8..14` with FMA;
- rows `28..29` enter Eigen's remaining-row 1x4 path.  It uses four Packet8
  accumulators for depth pairs `0..7`, reduces them as
  `(C0+C1)+(C2+C3)`, folds paired scalar k `8..13`, applies
  `predux_half_downto4`, and folds k `14` with FMA;
- columns `0..27` use X4 and columns `28..29` use the direct X1 path.

The `Jp.transpose() * r` GEMV starts with Packet8 products for k `0..7` and
the Eigen predux tree, uses ordinary multiply/add for k `8..11`, and uses
scalar FMAs for k `12..14`.  This cleanup is why the dense b probe catches the
tail association that the production structural zeros can hide.

## Reproduction

```text
g++-11 -std=c++17 -O3 -g -Wall -Wextra \
  -DEIGEN_DONT_PARALLELIZE -march=skylake -I/usr/include/eigen3 \
  -o /tmp/m7im15_schedule_oracle_final \
  benchmarks/basalt/m7im15_schedule_oracle.cpp

/tmp/m7im15_schedule_oracle_final \
  target/m7im_cov_ldlt_oracle_20260824.json \
  target/m7aq_upstream_imu_audit.json \
  target/m7im15_test_new.json 3 0.0001 0.001
```

The oracle is diagnostic-only; no production Rust source is part of this
schedule change.
