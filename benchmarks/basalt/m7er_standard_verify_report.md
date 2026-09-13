# M7er standard release verification

Date: 2026-08-23 JST  
Scope: one standard release-mode focused AOM M7 test run. Verification only;
no production source, fixture, commit, or push was changed.

## Command

PowerShell explicitly removed `RUSTFLAGS` in the test process, then invoked:

```powershell
Remove-Item Env:RUSTFLAGS -ErrorAction SilentlyContinue; & 'C:\Users\rsasa\.cargo\bin\cargo.exe' test --release -p visloc-basalt --lib vio::aom::tests::m7 -- --nocapture
```

## Result

**FAIL** — exit code 1; 8 passed, 4 failed, 0 ignored, 0 measured, and 157
filtered out. The failing pinned f32 lanes were:

```text
f32 lane 3: got 43f04ca8, expected 43f04ca7
f32 lane 1: got c1e49390, expected c1e49388
f32 lane 4: got c2978b07, expected c2978b08
f32 lane 2: got c1ccfe68, expected c1ccfe70
```

## Exact command output

```text
   Compiling visloc-basalt v0.1.0 (C:\Users\rsasa\Workspace\visloc-rs\pipelines\basalt)
    Finished `release` profile [optimized] target(s) in 1m 15s
     Running unittests src\lib.rs (target\release\deps\visloc_basalt-826e871d223e1516.exe)

running 12 tests

thread 'vio::aom::tests::m7_double_sphere_boundary_matches_pinned_lanes' (28824) panicked at pipelines\basalt\src\vio\aom.rs:2715:13:
assertion `left == right` failed: f32 lane 3: got 43f04ca8, expected 43f04ca7
  left: 1139821736
 right: 1139821735
note: run with `RUST_BACKTRACE=1` environment variable to display a backtrace
test vio::aom::tests::m7_anchored_factor_matches_pinned_projection_and_raw ... 
thread 'vio::aom::tests::m7_landmark_jacobian_intermediates_match_pinned_lanes' (37808) panicked at pipelines\basalt\src\vio\aom.rs:2715:13:
assertion `left == right` failed: f32 lane 1: got c1e49390, expected c1e49388
  left: 3252982672
 right: 3252982664
ok
test vio::aom::tests::m7_double_sphere_boundary_matches_pinned_lanes ... FAILED
test vio::aom::tests::m7_landmark_jacobian_intermediates_match_pinned_lanes ... FAILED
test vio::aom::tests::m7_householder_tiny_and_signed_zero_match_eigen_semantics ... ok

thread 'vio::aom::tests::m7_same_timestamp_stereo_homogeneous_jacobian_matches_pinned_lanes' (28512) panicked at pipelines\basalt\src\vio\aom.rs:2715:13:
assertion `left == right` failed: f32 lane 4: got c2978b07, expected c2978b08
  left: 3264711431
 right: 3264711432
test vio::aom::tests::m7_q2_one_factor_f32_reduction_is_bitwise_eigen_compatible ... ok
test vio::aom::tests::m7dx_clean_track1_relative_pose_jacobians_are_bitwise_exact ... ok

thread 'vio::aom::tests::m7ec_clean_track2_stereo_factor_is_bitwise_exact' (23336) panicked at pipelines\basalt\src\vio\aom.rs:2715:13:
assertion `left == right` failed: f32 lane 2: got c1ccfe68, expected c1ccfe70
  left: 3251437160
 right: 3251437168
test vio::aom::tests::m7_eigen_quaternion_matrix_matches_pinned_lanes ... ok
test vio::aom::tests::m7_homogeneous_relative_point_matches_pinned_lanes ... ok
test vio::aom::tests::m7_relative_pose_chain_matches_pinned_lanes ... ok
test vio::aom::tests::m7_same_timestamp_stereo_homogeneous_jacobian_matches_pinned_lanes ... FAILED
test vio::aom::tests::m7ec_clean_track2_stereo_factor_is_bitwise_exact ... FAILED
test vio::aom::tests::m7_householder_track1_fixture_is_bitwise_eigen_compatible ... ok

failures:

failures:
    vio::aom::tests::m7_double_sphere_boundary_matches_pinned_lanes
    vio::aom::tests::m7_landmark_jacobian_intermediates_match_pinned_lanes
    vio::aom::tests::m7_same_timestamp_stereo_homogeneous_jacobian_matches_pinned_lanes
    vio::aom::tests::m7ec_clean_track2_stereo_factor_is_bitwise_exact

test result: FAILED. 8 passed; 4 failed; 0 ignored; 0 measured; 157 filtered out; finished in 0.06s

error: test failed, to rerun pass `-p visloc-basalt --lib`
```

Failing tests and lane deltas are recorded exactly as emitted above; no
additional test run was performed.
