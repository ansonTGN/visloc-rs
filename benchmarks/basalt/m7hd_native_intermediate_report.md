# M7HD native IMU link-0 intermediate boundary

Date: 2026-08-24  
Scope: clean MH_01_easy frame link 0 (the first 10 IMU packets)  
Pinned header: `basalt/imu/preintegration.h`, SHA-256
`08e2fe5272b3dc81e7b019861d4d8e3613d5cc5f62f81ebee9d3ae0c3399fcbd`

## Method

`m7hd_native_intermediate_probe.sh` copies the pinned header to a generated
`target/m7hd_native_include` directory and adds observer calls around the
existing `propagateState` outputs and covariance assignment. The pinned
checkout and clean `basalt_vio` binary are not edited. The probe is compiled
with the clean binary's native profile (`g++ -std=c++17 -O3 -g -march=skylake
-DEIGEN_DONT_PARALLELIZE`) and runs the exact calibrated CSV interval
`1403636579763555584..1403636579813555456`.

The native probe emitted 10 step records and 10 covariance records. Its final
covariance matches the first clean production capture exactly: **0/81 lane
mismatches**. The covariance term records are observer materializations from
the captured `F/A/G` and pre-step covariance; the original native covariance
assignment remains unchanged.

## Quaternion/matrix split and boundary comparison

Counts below compare native against the current Rust `F32ImuState` step dump
for the same first 10 packets, in Eigen/nalgebra column-major order.

| boundary | mismatches | first mismatch |
| --- | ---: | --- |
| accel input | 0/30 | — |
| gyro input | 0/30 | — |
| `dt` | 0/10 | — |
| `q_pre` | 0/40 | — |
| `q_exp` | 0/40 | — |
| `q_post` | 0/40 | — |
| `r_half` | 0/90 | — |
| `r_new` | 0/90 | — |
| `Jr` | 0/90 | — |
| `Jr2` | 0/90 | — |
| `F` | 0/810 | — |
| `A` | 0/270 | — |
| `G` upper product | 0/90 | — |
| `G` upper `*dt` | 0/90 | — |
| `G` lower `f_rot*r_half` | 0/90 | — |
| `G` lower `*Jr2` | 0/90 | — |
| `G` lower `*0.5` | 0/90 | — |
| `G` lower `*dt` | 0/90 | — |
| `G` position scale | 0/90 | — |
| `G` final | 0/270 | — |
| `term1` | 580/810 | packet 2, lane 0: Rust `2e9e524d`, native `2e9e524f` |
| `term2` | 357/810 | packet 1, lane 0: Rust `2d0cbaef`, native `2d0cbaf1` |
| `term3` | 274/810 | packet 1, lane 3: Rust `a00ed4bc`, native `a00ed4bb` |
| `cov` | 637/810 | packet 1, lane 0: Rust `2d0cbaef`, native `2d0cbaf1` |

The quaternion split is exact before the matrix conversion: all 40 lanes of
`q_pre`, `SO3::exp` (`q_exp`), and the post-product `q_post` match. The pinned
Eigen `QuaternionBase::toRotationMatrix` disassembly uses FMA-contracted
off-diagonals (`vfmadd/vfnmadd`), while separately rounded Rust products first
diverged at `r_new`, packet 1. The production helper now spells those same
source-order `f32::mul_add` operations; all `r_half`, `r_new`, `Jr`, `Jr2`,
`F`, and `A` lanes are exact across 10 packets. The next native trace split
shows Eigen's pinned 3x3 packet product schedule as `p1`, then
`fma(k=2, p2, p1)`, then `fma(k=0, p0, ...)` for each output dot product
(`vmulps` followed by two `vfmadd132ps` instructions). The production
`eigen_matrix_product_f32` helper applies that schedule to every G 3x3
product. All seven captured G stages and the final 27 lanes are now exact
across 10 packets (`0/90` each stage; `0/270` final).

Using the same calibrated discrete noise (`0.016*sqrt(200)` accel and
`0.000282*sqrt(200)` gyro), the four final Rust link covariances versus the
four clean captures are `link0=66/81`, `link1=61/81`, `link2=57/81`, and
`link3=65/81` mismatches (`249/324` total; `75/324` exact). The native probe
itself remains exact against clean link 0 (`0/81`). The remaining term/cov
differences are downstream covariance GEMMs; this slice stops before changing
those operations or starting LDLT.

## Durable artifacts

- `m7hd_native_intermediate_probe.sh` — rebuild/run recipe.
- `m7hd_native_intermediate_probe.cpp` and `m7hd_native_trace_hook.hpp` —
  standalone native probe and generated-header observers.
- `m7hd_compare_intermediates.py` — exact-bit comparator, including all four
  final-link covariance counts.
- `m7hd_quaternion_matrix_probe.cpp` — pinned Eigen/Sophus matrix schedule
  probe and disassembly witness.
- `m7hd_matrix_product_probe.cpp` — pinned Eigen 3x3 packet-product probe;
  its `order120` result matches the native G product schedule.
- `target/m7hd_native_intermediate.jsonl` — 10 native step + 10 covariance
  records, SHA-256
  `7C24E8B02C84479210FC329841A1ECB7FE61320B948893EE0C7BAC9D594B6D00`.
- `target/m7hd_g_fma_probe.stdout` — 40 Rust G-stage and step records with
  temporary capture enabled, SHA-256
  `FCFF19368A87F5DDE7B6644244ED5458F1E67D5FC499F8E17289DD48FF4A5BA5`.

The source-wide quaternion-matrix and 3x3-product schedule fixes are retained;
temporary Rust capture is removed from `estimator.rs`. `cargo check -p
visloc-basalt --lib` passes. No covariance-GEMM/LDLT change, commit, or push
was made.

## Assignment-tail parity run8

The production weighted-Gram port now keeps the assignment GEMM separate from
the observer term. For the one-column AVX tail it computes two independent
zero-seeded products and contracts the third product with an FMA into their
sum: `p2.mul_add(rhs2, p0 + p1)`. This is source-equivalent to the pinned
Eigen `gebp` remainder path; the observer's row-8 reduction remains
diagnostic only. The opt-in `VISLOC_BASALT_M7HD_ASSIGN_DUMP` hook records the
production noise, variance, and term without changing the normal path, and
`m7hd_compare_assignment.py` matches Rust/native records by accel/gyro kind
and ordinal.

Fresh release run8 artifacts:

- `target/m7hd_cov_fresh_v9.stderr` (SHA-256
  `7E0F6422E23FA860FFCE85385E72072477AAAD4F72550A40121EB9A9DBDE8E4C`) —
  source dump with two assignment records and ten covariance records.
- First ten native-vs-Rust counts: observer `term2=0/810`, observer
  `term3=0/810`, and final production `cov=0/810`. The pre-update diagnostic
  `term1` has 6/810 differences and is intentionally not used as the
  production acceptance criterion.
- p1 assignment witnesses: accel lane 72 `1e24dbf0`, gyro lane 72
  `250d7fed`. The self-schema comparison is
  `target/m7hd_assignment_self_compare_v9.json` (SHA-256
  `9214B333D765FAD16C85309BC3A36080A303A14B532FB1E44303657F04C0B1E0`);
  a native full-term dump can be passed to the same comparator without any
  fixture-specific lane rules.
