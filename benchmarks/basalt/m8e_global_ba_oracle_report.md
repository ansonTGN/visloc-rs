# M8e global mapper BA oracle

The diagnostic source is `upstream_m8e_global_ba_oracle.cpp`.  It is pinned to
Basalt commit `0f3b2b52c807f70ff4e2973ce253c73329eea7bc` and calls the upstream
`NfrMapper` symbols directly:

- `linearizeHelper` and `MapperLinearizeAbsReduce<SparseHashAccumulator<double>>`;
- `SparseHashAccumulator::Hdiagonal`/`solve`;
- `ScBundleAdjustmentBase::updatePoints`;
- upstream `computeError`, `computeRelPose`, `computeRollPitch`, `backup`, and
  `restore`.

The Rust solve keeps the upstream gauge behavior: no pose is silently pinned;
`gauge_anchor` is retained only as compatibility metadata.  The LM diagonal
is the sole regularizer, just as in `NfrMapper::optimize()`.

The portable input is the existing `target/m8d_setup_m7v80.bin` (`M8DOPT1`:
20 stereo images, 80 pose records, 554 tracks; `setup_opt()` retains 549
landmarks).  The run used the first matching MargData packet, rather than the
whole directory, because later packets contain factor endpoints beyond the
80-pose setup stream.  That packet contributes three additional factor-endpoint
poses, so the actual optimize trace has 83 six-DoF pose blocks (498 pose
unknowns); this is recorded explicitly in the checked-in summary:

```text
upstream_m8e_global_ba_oracle \
  target/euroc_ds_calib.json target/euroc_config.json \
  marg_data/1403636579763555584.cereal \
  target/m8d_setup_m7v80.bin /tmp/m8e_global_ba_oracle.json
```

The binary was built in Ubuntu-22.04 WSL against the pinned core-only Basalt
build.  The complete JSON trace is a generated run artifact; its compact
summary is checked in as `m8e_global_ba_oracle_summary.json` so the gate does
not depend on a local C++ toolchain.

Observed upstream trace (10 iterations, one accepted trial each):

| value | result |
|---|---:|
| initial total cost | 2711067.758715515 |
| final total cost | 93074.762625592528 |
| final lambda | 1e-32 |
| final state hash (FNV-1a) | 12460248985570421363 |
| per-iteration trace hash (FNV-1a) | 7693564316440188179 |

The Rust trace fields use the same canonical ordering and hash schema.  The
Rust solve uses the same diagonal-preconditioned CG contract and `1e-4`
relative tolerance.  The fixture also makes the 83-pose MargData/setup merge
explicit, so pose-set selection is not hidden behind a hash mismatch.

Rust fixture run: `pipelines/basalt/examples/m8e_rust_fixture.rs` loads
`target/m8e_rust_fixture.json` (83 poses, 549 landmarks, 5687 observations)
and completes all 10 LM iterations with accepted trials.  The authoritative
oracle export now includes full-double pose records immediately after the
MargData/setup merge.  Setup-stream IDs retain their setup values; only the
three MargData-only endpoint IDs are replaced in the Rust fixture.  Initial
total is `2711067.758715517`; final total is `93074.76262564836`; final lambda
is `1e-32`.  The initial seven relative-factor costs now match the upstream
aggregate:
`[7193.165402396683, 6260.74437974839, 445558.40241736, 742644.0766684159,
2126.632837005219, 2095.720187892268, 2232.1673183671983]`, summing to
`1208110.9092111855`; the single roll/pitch factor is
`1.5933187799404398`.  Initial total/vision/relative costs agree to the
printed floating-point boundary.  After the production Jacobian correction,
Hdiag0 is `24980552.89154232` Rust versus `24980552.891542263` upstream, and
the final-cost delta is `5.58e-8`.  The remaining later-iteration drift is the
expected dense-versus-Eigen sparse accumulation/solve arithmetic; no
threshold or acceptance gate is changed to hide it.

## Hdiag0 source isolation and production correction

The temporary category/per-factor diagnostic was run with the exact command
below; its source was restored after the run.  The retained artifacts and
SHA-256 values are recorded in `m8e_global_ba_oracle_summary.json`.

```text
M8E_DIAG=1 target/m8e_hdiag_oracle_diag3 \
  target/euroc_ds_calib.json target/euroc_config.json \
  marg_data/1403636579763555584.cereal target/m8d_setup_m7v80.bin \
  target/m8e_hdiag_oracle_diag4.json \
  > target/m8e_hdiag_oracle_diag4.stdout \
  2> target/m8e_hdiag_oracle_diag4.log
```

Before the correction, the full initial Hdiag0 delta (`6408.202218465`) was
entirely in relative-factor rows.  Vision was
`16521399.458724074` upstream versus `16521399.458724126` Rust; relative was
`8459153.4328181893` versus `8452745.230599724`; and roll/pitch contributed
only Eigen's sparse `DBL_MIN` diagonal placeholder versus Rust zero.  The
seven per-factor relative Hdiag0 values were:

```text
upstream = [3211067.7123373561, 1921693.6373665195, 853975.35784472816,
             844554.81009619776, 560491.93127962435, 559021.66755007161,
             508348.31634369167]
rust     = [3194829.1948195202, 1914745.2364313381, 866508.6073137369,
             856848.5027087119, 558257.4416427151, 555700.0188535247,
             505856.2288301779]
```

The pinned Sophus implementation of
`rightJacobianInvSE3Decoupled` sets its translational 3×3 block to
`SO3::exp(omega).matrix()`.  Rust had incorrectly left that block as identity.
The production mapper now mirrors that exact block in
`se3_right_jacobian_inverse_decoupled`; after the edit, all seven relative
Hdiag0 values match to floating-point noise and the combined Hdiag0 matches
within `5.6e-8`.  This is a source-faithful correction backed by the oracle,
not a threshold or solver-gate adjustment.

Focused verification after removing temporary diagnostics:

```text
cargo test -p visloc-basalt mapper
# 26 passed, 1 ignored
cargo check -p visloc-basalt --all-targets
```

## Installed-pose serialization boundary

`upstream_m8e_global_ba_oracle.cpp` emits schema v2's `installed_poses` array
before `setup_opt()` mutates the map.  The exporter contains 83 records.  The
80 setup IDs are byte-identical to `target/m8d_setup_poses_m7v80.json`; the
remaining records are the MargData-only endpoints
`1403636583963555584`, `1403636584313555456`, and
`1403636584663555584`.  The old builder sourced those three from the M8a JSON,
where they had been float-rounded.  The builder now replaces only those
non-setup IDs from `installed_poses`, preserving the setup stream as
authoritative.

For factor 0 (`1403636579763555584 -> 1403636580113555456`), the corrected
Rust and upstream residuals agree before information accumulation:

```text
r = [0.009680698961120254, 0.000633242734993648,
     -0.008088949724640346, 0.002098169112239301,
      0.009308576788917658, 0.003900235487580269]
factor cost = 7193.165402396683
```

The earlier `58.226...` comparison was against a final-state factor export,
not the initial state, and was discarded.
