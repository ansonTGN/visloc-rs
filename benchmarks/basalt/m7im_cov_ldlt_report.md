# M7 IMU covariance / Eigen LDLT oracle

Date: 2026-08-24 JST  
Scope: MH_01_easy, first link's first 10 IMU packets, the final frame-3 →
frame-4 IMU factor, and the production M7 f32 normal-equation reduction.  The
factor boundary uses the release `factors.rs` implementation; the production
reduction evidence uses the explicit `FactorKind` wiring in `aom.rs` and
`window.rs`.  This report does not claim an `estimator.rs` change or an
end-to-end bit-exact VIO replay.

## Pinned oracle

The diagnostic source [`m7im_cov_ldlt_oracle.cpp`](m7im_cov_ldlt_oracle.cpp)
instantiates the exact pinned `basalt::IntegratedImuMeasurement<float>` and
records, after each of the first 10 integrations:

- covariance before/after integration, in both row-major and Eigen's native
  column-major storage order;
- `Eigen::LDLT<Matrix<float,9,9>>`'s `matrixLDLT`, `matrixL`, `matrixU`,
  `vectorD`, `transpositionsP`, and `solve(I)`;
- the cached square-root inverse and an independently reconstructed whitening
  matrix;
- for frame 3 → 4, raw/Jacobian and whitened J/r/H/b bit arrays.

The pinned header's contract is the source implementation at
`preintegration.h:339-358`:

```text
P = ldlt.transpositionsP()
W = D^{-1/2} * L^{-1} * P
D[i] < numeric_limits<float>::min()  =>  D^{-1/2}[i] = 0
otherwise                             =>  1 / sqrt(D[i])
```

`P` is applied before the lower-triangular solve.  The first ten pivot
vectors are retained in the JSON; the first is
`[8,7,6,5,4,5,6,7,8]`, and the pivots change as the covariance fills in.
The reconstructed W is bit-identical to the header's cached W for all ten
packets.  This is a pivoted lower LDLT contract, not an unpivoted Cholesky or
an upper-triangular whitening convention.

## Machine-readable artifacts

- Oracle: `target/m7im_cov_ldlt_oracle_20260824.json`
  (`B2496D43EB58B3B3719C250B31362F1F2F79E422BF37AF5D61C5C645903BBC49`)
- Frame-4 local exact comparator (injected f32 factor fields, Skylake
  schedule): `target/m7im_cov_ldlt_comparison_rowprod_skylake20260824.json`
  (`88E480A6256F6B57CFFA106AB2F7CBC30435EFE47F4A9CF524789DF34E95CC59`)
- Frame-4 local exact Rust factor artifact:
  `target/m7aq_rust_imu_audit_release_rowprod_skylake20260824.json`
  (`90359855C991D0FDF11DDFA794AD97384657B485B860DF7F624D539AE319627D`)
- Superseded factor comparator (retained for boundary history):
  `target/m7im_cov_ldlt_comparison_whiten_skylake20260824.json`
  (`EB228E63CE6E6FA88D9E8F332354BDCE9CD6C9C4C1D2F3DF1DBBE5F85E144692`)
- Superseded pre-whitening comparator (retained for boundary history):
  `target/m7im_cov_ldlt_comparison_gemv3_verified_20260824.json`
  (`FB5AA3B66FB0D322893F1FC5030B815CA6072E93DF56AAE4656BD993276C72C6`)
- Build/run recipe: `m7im_cov_ldlt_oracle.sh`
- Comparator: `m7im_compare_cov_ldlt.py`

All bit arrays use lowercase eight-digit IEEE-754 binary32 words.  Matrix
fields carry explicit dimensions, `storage_order`, and both flattenings, so a
Rust-side packet dump can be compared without guessing a transpose.

## Current comparison status

The oracle reproduces the existing pinned native first-10 covariance trace:

```text
native_trace: 0/810 covariance-bit mismatches
sqrt reconstruction: exact for 10/10 packets
```

The old Rust diagnostic `m7hd_step_probe_after_fma.stdout` remains a historical
baseline, not a current result.  Its 626/810 mismatch was closed by the
general Eigen GEMM/alias schedule and the production assignment tail.  The
authoritative run8 capture (`target/m7hd_cov_current_run8.stderr`) reports:

```text
observer materialized term1: 6/810 mismatches
observer materialized term2: 0/810 mismatches
observer materialized term3: 0/810 mismatches
production alias-assignment final covariance: 0/810 mismatches
```

`term1` above is intentionally an observer only: it materializes the
diagnostic `F * C * Fᵀ` product and is not the value used by the production
aliased covariance assignment.  The production path uses the proven Eigen
alias schedule, so the observer's six differing lanes must not be interpreted
as a covariance regression.

The run3 LDLT capture and focused checker close the packet whitening gate:

```text
first 10 packets W/sqrt-information: 0/810 mismatches
frame-4 W/sqrt-information:          0/81 mismatches
```

The checker output is retained at
`target/m7im_rust_ldlt_probe_run2.stdout`; the full env-guarded decomposition
capture is `target/m7im_rust_ldlt_run3.json` (with bit dumps in its stderr
companion).  Covariance, pivoted LDLT, D, and W are therefore separated into
two independently auditable gates: run8 covers the covariance recursion and
run3 covers the whitening/factor boundary.  The current release evaluator
also reproduces the frame-4 square-root matrix, residual whitening, and the
local row products exactly; the remaining frontier is the production 15-row
Eigen packet/reduction schedule, not the local frame-4 factor product.

The older `m7aq_rust_imu_audit_after_fma.json` is also accepted by the
comparator, but it was generated against the earlier four-link audit fixture.
It is therefore reported as a mismatch by the legacy-field comparator rather
than used as evidence of a current covariance failure.  In particular, that
comparator compares the legacy widened `raw_residual` field; the production
covariance and sqrt gates above use the current run8/run3 artifacts.  A current
Rust frame-4 factor JSON with the same state/covariance provenance can be
passed with `--rust`; the comparator supports either the legacy `links[3]`
schema or a `frame4_factor` bit-object schema.

## Arithmetic recovered at the M7IM boundary

The exact Rust port records the following source-level schedule decisions:

1. The covariance recursion uses Eigen's aliased
   `general_matrix_matrix_product` chain, including the packet body and the
   scalar ninth-row tail.  The final assignment uses the separate
   packet-compatible `operator+=` schedule; it does not reuse the materialized
   observer term.
2. Pivoted lower LDLT follows Eigen's transposition order.  The A21 rank update
   uses packet FMA lanes and the scalar tail, and the forward solve follows the
   Eigen panel order (pivot row, RHS column, then rows below the pivot).
3. The inverse-square-root scale preserves the pinned float-to-double
   boundary: `(1.0_f64 / (D[i] as f64).sqrt()) as f32`.  The pivot threshold
   remains the binary32 `numeric_limits<float>::min()` comparison.

These choices are required for bit parity; algebraically equivalent scalar
rewrites are not interchangeable at this boundary.

## Production reduction wiring

The production normal-equation path now carries an explicit `FactorKind` on
`WhitenedFactorRowStack`: `Prior`, `Visual`, `Imu`, `Bias`, or `Generic`.
`window.rs` tags the generated visual, prior, IMU, and bias factors, so a
generic nine-row prior cannot enter the IMU schedule by shape alone.

For each temporal link, the reduction consumes the adjacent 9-row IMU factor
and 6-row bias factor as one semantic 15-row stack in the upstream order
`[IMU 9 | gyro-bias 3 | accel-bias 3]`.  It forms `H = JᵀJ` and `b = Jᵀr`
once for that combined stack, then adds the result to the IMU phase.  Visual,
prior, and generic factors retain their own paths, and the phase order remains
visual → IMU → prior.  The public factor list/topology is unchanged.

This is the faithful semantic wiring and closes the accidental shape-based
dispatch hole.  The current implementation uses the generic dynamic
15-row product for the combined stack.  The packetized Eigen 15-row
reduction tree is still **pending**; the semantic implementation must not be
described as bit-exact until that schedule is integrated and replayed.

## Current frame-4 factor gate

The current comparison selects the injected binary32 fields
`raw_residual_f32_exact` and `jacobian_f32_exact` (the widened legacy fields
are not acceptance inputs).  Against the pinned native frame-4 oracle, the
local exact artifact `m7im_cov_ldlt_comparison_rowprod_skylake20260824.json`
reports:

| field | differing f32 elements | status |
| --- | ---: | --- |
| covariance | 0/81 | exact |
| square-root information | 0/81 | exact |
| raw residual | 0/9 | exact |
| Jacobian | 0/270 | exact |
| whitened residual (`W*r`) | 0/9 | exact |
| whitened Jacobian (`W*J`) | 0/270 | exact |
| row H (`JᵀJ`) | 0/900 | exact |
| row b (`Jᵀr`) | 0/30 | exact |

Thus the complete injected frame-3 → frame-4 local factor gate, including
`W*J`, `JᵀJ`, and `Jᵀr`, is closed.  This is a factor-boundary result only;
it does not certify the production 15-row packet schedule or end-to-end VIO
parity.

## Production replay: v11 regression and semantic v12

The retained v11 row-product replay is an intermediate regression record:
against the same one-thread native logger, its 80-frame run had 76 common LM
runs, 9 branch-equal frames, 67 branch-divergent frames, and 8 displayed-
lambda mismatches.  Its first divergence was frame 8 / iteration 4 (native
`R`, Rust `A`), with 52.51 s runtime and 90,464 KB peak RSS.  Its retained
trace and comparator hashes are respectively
`F0548F98B288391AA69AC89AF2E71C0A53D8146F448C57DCA9846A1177BFE219` and
`EA0CEAC45D91D622271789A291C60B45F1445B4486A31585D3BF8EEAA6AA29FB`.

The semantic v12 production replay was rebuilt with all detail/diagnostic
environment variables unset and `--no-marg-data`.  The build snapshot was:

```text
aom.rs    8F7EB952B5806D8E8177A3721662432F777AD9F66ACD4278179A1823AE5F365E
window.rs 4A82B118572EA6AEE728C1C1D42924069B910E1469D14D772A9CE751457EDB8E
ELF       A516F867561746D28E52F2D197F5B5C6FD9AEA639FB3B5FEC82BB6BB86C506B7
```

| run | runtime | peak RSS | common LM runs | branch equal | branch divergent | lambda mismatches | first divergence | trace SHA-256 | comparator SHA-256 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- | --- |
| semantic v12, 5 frames | 0.94 s | 23,152 KB | 1 | 1 | 0 | 0 | none | `277DABCA0881C9BFB853F0380987A547374615FC3298E17636CF346DBEDC22D9` | `8C3725BCA996EEB66B2BF6E1ECD764BE8BFB8BCE7A3F31468478A61462BE9A96` |
| semantic v12, 80 frames | 43.64 s | 90,248 KB | 76 | 11 | 65 | 12 | frame 8 / iter 4: native `R`, Rust `A` | `B20B2801C27441C23CC9759A94689B7F85BC08F9C46614017BCC71060857CA15` | `1D776FE89B6E58EE3FA42DF362344FF5BE1C63BC68806341CEC1CC4338DEDA01` |

The 5-frame smoke run is branch-exact.  The 80-frame run improves the v11
branch-equal count from 9 to 11 and reduces runtime/RSS, but remains divergent
after frame 8; it is not an end-to-end parity gate.  The 15-row packetized
Eigen schedule remains pending.

## Verification and retained hashes

The current release library run passed `193` tests, with `0` failures and `1`
ignored test.  The release demo was rebuilt from the workspace-root
`visloc-rs` package (the demo is not an example of the `visloc-basalt`
package):

```text
target/release/examples/basalt_euroc_vio_demo
size: 2516384 bytes
mtime: 2026-08-24T22:41:44.9794095+09:00
sha256: A516F867561746D28E52F2D197F5B5C6FD9AEA639FB3B5FEC82BB6BB86C506B7
```

| artifact | SHA-256 |
| --- | --- |
| `target/m7hd_cov_current_run8.stdout` | `08D4EF9B52A585A67ABE4B971659BE7D14B5BC98A8433B177DB69E100AC24CA8` |
| `target/m7hd_cov_current_run8.stderr` | `CC9D47787216EF018D8282760401015095C46D4804B67822D0E3865BA85D7286` |
| `target/m7im_rust_ldlt_run3.json` | `E1F8862C993759DE7EE4FCB948C21C6C9E542064BCEBFC535A4EB1F393D8996E` |
| `target/m7im_rust_ldlt_run3.stdout` (empty; diagnostics are stderr) | `E3B0C44298FC1C149AFBF4C8996FB92427AE41E4649B934CA495991B7852B855` |
| `target/m7im_rust_ldlt_run3.stderr` | `57BCB4B7CE902C1D65D8DEDc49411ABD777B1E008E0A635F88D3A04F309B49FB` |
| `target/m7im_rust_ldlt_probe_run2.stdout` | `32325A40BC235750E7D4DB37A06626AB221E57976FDDf345530CC4C44089A2DB` |
| `target/m7im_cov_ldlt_comparison_rowprod_skylake20260824.json` | `88E480A6256F6B57CFFA106AB2F7CBC30435EFE47F4A9CF524789DF34E95CC59` |
| `target/m7aq_rust_imu_audit_release_rowprod_skylake20260824.json` | `90359855C991D0FDF11DDFA794AD97384657B485B860DF7F624D539AE319627D` |
| `target/m7im_cov_ldlt_comparison_whiten_skylake20260824.json` | `EB228E63CE6E6FA88D9E8F332354BDCE9CD6C9C4C1D2F3DF1DBBE5F85E144692` |
| `target/m7aq_rust_imu_audit_release_whiten_skylake20260824.json` | `E7C11E7FE7DBE085059F45EF088C1C45D3EBAFB6176A97C6465A1EE5E9AE6CE9` |
| `target/m7im_cov_ldlt_comparison_gemv3_verified_20260824.json` (superseded) | `FB5AA3B66FB0D322893F1FC5030B815CA6072E93DF56AAE4656BD993276C72C6` |
| `pipelines/basalt/src/imu/factors.rs` | `24ED6C577575D415495DD488A583E521DA02B30937A7A44FDEDAFDFE04595C83` |
| `pipelines/basalt/examples/m7im_rust_ldlt_probe.rs` | `8E5C9447C25D0382A2CE8E549993269740DCF213BDFD24267743EB944F7ABD72` |
| `pipelines/basalt/src/vio/aom.rs` (semantic v12 build snapshot) | `8F7EB952B5806D8E8177A3721662432F777AD9F66ACD4278179A1823AE5F365E` |
| `pipelines/basalt/src/vio/window.rs` (semantic v12 build snapshot) | `4A82B118572EA6AEE728C1C1D42924069B910E1469D14D772A9CE751457EDB8E` |
| `target/m7he_imu15_semantic_v12_5/trace.jsonl` | `277DABCA0881C9BFB853F0380987A547374615FC3298E17636CF346DBEDC22D9` |
| `target/m7he_imu15_semantic_v12_80/trace.jsonl` | `B20B2801C27441C23CC9759A94689B7F85BC08F9C46614017BCC71060857CA15` |
| `target/m7he_lm_branch_compare_imu15_semantic_v12_5.json` | `8C3725BCA996EEB66B2BF6E1ECD764BE8BFB8BCE7A3F31468478A61462BE9A96` |
| `target/m7he_lm_branch_compare_imu15_semantic_v12_80.json` | `1D776FE89B6E58EE3FA42DF362344FF5BE1C63BC68806341CEC1CC4338DEDA01` |

The focused `rustfmt --check pipelines/basalt/src/imu/factors.rs` check passed.
The workspace-wide fmt check remains blocked only by the pre-existing dirty
`pipelines/basalt/examples/basalt_offline_mapper.rs` diff; that unrelated file
is outside this factor audit.  The local frame-4 factor products are exact,
but no end-to-end LM parity is claimed until the production 15-row Eigen
reduction schedule is integrated and replayed.

## Provenance

```text
upstream commit: 0f3b2b52c807f70ff4e2973ce253c73329eea7bc
preintegration.h: 08e2fe5272b3dc81e7b019861d4d8e3613d5cc5f62f81ebee9d3ae0c3399fcbd
Eigen LDLT.h: 9c252a265b05b65e79eb42f6b17ed1fe329e20c2dae78e77b5acb5d3d37deb5a
toolchain: g++ (Ubuntu 11.4.0-1ubuntu1~22.04.3) 11.4.0
flags: -std=c++17 -O3 -g -Wall -Wextra -DEIGEN_DONT_PARALLELIZE -march=skylake
oracle source: DC027FE20E8A8A578E614D4C5CC74706C055098F039EDF1FD2F9A1576E2F18C9
oracle binary: E00472DCFD274C74D56AE1EB9B12D2BE08CB10CA0005413BC38236456DB9651
IMU CSV: 226470999DC8BA758C838982FD7EB2BC9F3F2B489E85A38D0ECF8B9D40785A5B
frame states TSV: 2E77FAACFDED19A947C86FDB0A85ABD6AB84B3379A26EA8B262F79F62BD1568C
native 10-packet trace: 9418B1FC2AE68BD47083276C3FAF4FD52D9BC997A0A11AA919B7B49AB29906A3
```

Reproduction from WSL:

```text
bash benchmarks/basalt/m7im_cov_ldlt_oracle.sh
python3 benchmarks/basalt/m7im_compare_cov_ldlt.py
```
