# m7aq IMU scalar-boundary audit

Date: 2026-08-22  
Dataset: `MH_01_easy`, frame links 0→1, 1→2, 2→3, 3→4  
Oracle: [`m7aq_upstream_imu_audit.cpp`](m7aq_upstream_imu_audit.cpp)  
Rust harness: `pipelines/basalt/examples/m7aq_rust_imu_audit.rs`

## Finding

The first material semantic mismatch was the covariance whitener, before any
row product.  Basalt's `IntegratedImuMeasurement<float>::compute_sqrt_cov_inv`
uses Eigen's *pivoted* lower LDLT and constructs
`D^-1/2 L^-1 P`.  Rust had an unpivoted LDLT.  MH_01's IMU covariance has six
zero leading directions, so the two decompositions put the nonzero whitener
rows in different locations.  This made the f32 whitened Jacobian and normal
equations differ by tens of thousands despite matching raw Jacobians.

The active path now ports the Eigen pivot search, lower-triangle swaps,
transposition sequence, forward solve, and nonpositive-pivot zeroing.  The
f32 residual/Jacobian are also evaluated before widening, matching the
upstream `Scalar=float` boundary.

The actual frame-4 replay then exposed a second assembly issue: the live f32
integrator was transposing its bias-state 9x3 blocks on storage, although the
f32 factor consumes Basalt's direct `[p,R,v]` rows. Removing those transpose
round-trips reduced the initial global-H maximum from `147336.5` to `2112.0`.

## Four-link oracle comparison

`target/m7aq_upstream_imu_audit.json` contains, per link, the integrated delta,
covariance, sqrt information, raw residual, four raw Jacobian blocks, the
whitened row, `JᵀJ`, and `Jᵀr`.  The Rust harness consumes that oracle delta and
covariance so the comparison isolates factor arithmetic.

| quantity | maximum absolute Rust−upstream | location |
|---|---:|---|
| f32 raw residual | `5.96e-8` | link 2, row 8 |
| f32 raw Jacobian | `2.09e-7` | link 2, row 5, col 5 |
| f32 sqrt information | `1.95e-3` | link 1, row 4, col 2 |
| whitened Jacobian | `3.91e-3` | link 2, row 3, col 1 |
| per-link `JᵀJ` | `160` | link 2, row 1, col 1 |
| accumulated `H` | `192` | row 46, col 46 |
| accumulated `b` | `15.19` | index 49 (link 2 bias block) |

The residual/J differences are last-bit f32 matrix/libm arithmetic, amplified
by the approximately `2e4` information weights; they are not a row-order,
pose-chart, or covariance-semantic mismatch.  Before the pivoted port, the
same accumulated system had an `H` gap of `147351.25` and a `b` gap of
`58.578125` in the frame-4 trace.

At the scalar boundary, the first nonzero comparison is link 0's raw
residual row 1 (`upstream = 5.8207661e-11`, `Rust = 0`).  The first Jacobian
difference is link 0 `(row 0, col 0)`, `2.98e-8` absolute.  These precede the
first whitening difference (link 0 sqrt-info `(row 6, col 4)`, `9.77e-4`) and
are why the remaining row-product gap is classified as arithmetic rather than
another LDLT or factor-layout semantic error.  Corrected deltas and
covariances in this harness are the same pinned upstream objects on both
sides; the live f32 integration path has separate Sophus-product coverage.

The post-port actual replay (`target/m7aq_rust_frame4_directj.jsonl`) reports
initial cost `4216.1083984375` versus upstream `4215.9326171875`; initial H
maximum `2112.0` (RMS `77.2710`) and b maximum `83.6796875` at index 19. All
eight LM iterations were accepted. The remaining b gap is isolated to the
residual/integration scalar boundary rather than bias-Jacobian assembly.
Rust source decomposition gives visual `H[20,20]=91354824.4439`,
`b[19]=5043.7070`, with IMU/prior remainder `H[20,20]=477004471.5561`,
`b[19]=-37421.0820`.

The pinned checkout is `/root/visloc-basalt-oracle-0f3b2b52` at commit
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.  Its header path shows that
`propagateState` uses Sophus raw quaternion products, Eigen's
`Quaternion::toRotationMatrix`, and Eigen expression-order matrix/vector
products.  The Rust F32 path now ports those operations without per-step
quaternion normalization.  The interval oracle now matches all velocity and
position bits.  The former z expectation was proven stale: the pinned header
evaluates the z-lane increment as `p + (v*dt + 0.5*a*dt*dt)`, producing bits
`bb669dac`; the prior expectation encoded `bb669dad`.  The expected value was
corrected to that demonstrated header result without changing tolerance.

## Frame-0→1 step-1 quaternion boundary

The remaining rotation mismatch was isolated with exact-bit probes against the
pinned Sophus/Eigen headers.  Rust `so3_exp_f32` and the pinned
`Sophus::SO3<float>::exp` agree for both samples; step 1's exponential is
`b983a523,397a3eaa,b8a42077,3f7fffff` in Eigen `xyzw` order.  With those same
operands, ordinary scalar QuaternionProduct arithmetic produces raw/product
`ba00e63b,3a04e6d7,b9383f50,3f7ffffc`, which was the Rust result.

The authoritative native oracle is built with the documented pinned flags
`-O3 -march=native -DEIGEN_DONT_PARALLELIZE`.  Rebuilding the same pinned
Sophus product probe under those flags changes only the first boundary to
`ba00e63b,3a04e6d7,b9383f51,3f7ffffc`: Eigen's native AVX/FMA QuaternionProduct
contracts the multiply-add lanes before Sophus's packet normalization.  The
existing Rust `sophus_packet` implementation already encodes that recovered
lane order and normalization.  The f32 IMU path now reuses that helper, with
the scalar fallback retained for targets without AVX/FMA.

The resulting Rust rotation sequence matches all ten authoritative upstream
step records, not only the one-ULP step-1 z component.  At this rotation-only
audit boundary, no change was made to `so3_exp_f32`, its `sin`/`cos`
evaluation, or the integration association.
The temporary Rust bit-print test was removed; the auditable step logs remain
`target/m7aq_upstream_imu_audit_step.log` and
`target/m7aq_rust_step_run_after_p.log`.

## Direct full-engine frame-0→1 correction (2026-08-22)

The earlier standalone step log was not sufficient to certify the complete
engine interval.  A fresh direct pinned upstream trace and a matching C++
`IntegratedImuMeasurement<float>` probe show that `so3_exp_f32`, quaternion
product/normalization, matrix-vector acceleration, and velocity integration
already match all ten steps.  The first exact difference was the translation
packet evaluator: Eigen fuses `p + v*dt` before adding `0.5*a*dt*dt`, whereas
the Rust path had materialized `v*dt` and rounded the sum separately.  The
source-faithful Rust `mul_add` edit now makes the complete frame-0→1 delta
exact (`delta_position=3c1df76d,ba397d97,bb5ce9dd`; rotation and velocity
also exact).

The retained direct artifacts are
`target/m7_postm7_upstream_direct5_20260822_imu_trace.jsonl` and
`target/m7aq_predict_state_probe_O2_20260822.log`.  The next remaining
boundary is downstream `predictState` z-lane packet/writeback arithmetic
(`t.z` Rust `bb0527f8` versus upstream `bb0527fc`; `v.z` Rust `bda6dcd4`
versus upstream `bda6dcd8`).  The pinned Sophus point-action probe is retained
at `target/m7aq_predict_boundary_probe_O2_20260822.log`; no source-faithful
Rust `sophus_rotate_f32` edit is justified by the current exact-bit evidence.

## Regression coverage

`f32_ldlt_whitener_keeps_eigen_pivot_permutation` uses a rank-deficient 9×9
fixture and checks the exact nonzero pivot rows.  The existing frame-interval
fixture additionally checks the f32 residual convention against pinned
upstream `[position, rotation, velocity]` values.

The oracle can be regenerated with the pinned Basalt/Eigen headers and the
MH_01 IMU CSV; the Rust harness is invoked as:

```text
cargo run -p visloc-basalt --example m7aq_rust_imu_audit -- \
  target/m7aq_upstream_imu_audit.json target/m7aq_rust_imu_audit.json
```

Verification: release example check passed; pivot-whitener, analytic IMU
Jacobian, and same-timestamp tests passed; `--tests --no-run` passed; and
`m7f_camera_contract` passed 3/3. The legacy f64 covariance injection restored
the synthetic prediction test. Current debug and release lib runs pass 138
tests (1 ignored), and the release `--tests --no-run` build completes all
integration targets; only external-data oracle tests remain ignored when their
datasets are absent.
