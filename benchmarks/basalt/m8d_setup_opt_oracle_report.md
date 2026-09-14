# M8d `setup_opt()` parity artifact

The Rust implementation in `pipelines/basalt/src/mapper/triangulation.rs`
follows `NfrMapper::setup_opt()` from Basalt commit
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc`:

1. iterate exported `FeatureTracks` in root/`TimeCamId` order;
2. host the first observation;
3. unproject host and later observations with calibrated Double Sphere;
4. form `T_w_c = T_w_i * T_i_c`, then `T_h_o = T_w_h.inverse() * T_w_o`;
5. accept `squaredNorm(T_h_o.translation()) >= mapper_min_triangulation_dist²`;
6. call the upstream `BundleAdjustmentBase::triangulate` DLT/SVD convention;
7. require finite `0 < inverse_distance <= 2`, project stereographically, and
   add every original track observation before stopping at the first valid
   candidate.

## Portable input and oracle

`build_m8d_setup_fixture.py` packs a full-corner M8c export (the real run uses
`target/m8c_feature_raw20_final.json`) and the checked-in
`m8d_track_oracle20.json` exported tracks with a pose JSON emitted by the
pinned MargData reader. The resulting
`M8DOPT1` stream is consumed by
`upstream_m8d_setup_opt_oracle.cpp`, which calls upstream
`BundleAdjustmentBase<double>::triangulate` and emits attempted/accepted
counts, candidate rejection counts, host/second IDs, landmark parameters,
observation count, and the canonical FNV-1a hash.  Floating values in the
canonical stream are rounded to `1e-10` and encoded little-endian; this
absorbs the final few Eigen-vs-nalgebra SVD ulps while retaining landmark
parameters at substantially finer precision than mapper observations.

## Real 20-image parity run

The checked-in track export contains 554 root tracks and 5,713 observations.
For the parity run, `target/m8c_feature_raw20_final.json` supplied the full
corners for the same 20 stereo images and the first-80 MargData packets under
`target/basalt_m7v_run80/marg_data` supplied the timestamped mapper poses. The
pose packer converted each MargData `[t, qw, qx, qy, qz]` state to the fixture's
`quaternion_xyzw` boundary. The pinned C++ executable was compiled from
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc` and run with
`target/euroc_ds_calib.json` and `target/euroc_config.json`.

Rust and C++ matched exactly:

* attempted tracks: 554; accepted tracks: 549; short tracks: 0;
* candidate attempts: 631; retained observations: 5,687;
* rejections: baseline-too-small 43, inverse-distance-nonpositive 26,
  inverse-distance-too-large 13;
* canonical hash: `18384097344545831442`.

The ignored integration test `m8d_setup_opt_matches_pinned_cpp_oracle` runs
this comparison through the Windows-to-WSL wrapper used by the local oracle
build. The generated `target/m8d_setup_m7v80.bin` and oracle JSON are
reproducible working artifacts and intentionally remain outside the source
tree.

The external oracle is intentionally ignored by normal CI because it requires
the pinned C++ Basalt/Sophus/Eigen toolchain and MargData pose packets.  The
Rust synthetic tests remain portable and cover ordering, baseline rejection,
homogeneous sign/cheirality, observation retention, and canonical hashing.

## Rebuilt pinned oracle (2026-08-22 WSL2)

The missing `/tmp/upstream_m8d_setup_opt_oracle` was rebuilt in the existing
Ubuntu-22.04 WSL2 checkout.  The checkout is detached at
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc` (`tree_sha1`
`b7afb830d82b45b8209cf784ad9744025d838411`), with vcpkg submodule
`1e199d32ad53aab1defda61ce41c380302e3f95c`.  It reuses the documented
core-only `RelWithDebInfo` build at
`/root/visloc-basalt-oracle-0f3b2b52/build/core-relwithdebinfo`, configured
with the pinned vcpkg/Eigen/Sophus dependencies and the upstream Ninja flags
(`g++ 11.4.0`, `-std=c++17`, `-O3 -g`, `-march=native`,
`-DEIGEN_DONT_PARALLELIZE`, and `-DEIGEN_INITIALIZE_MATRICES_BY_NAN`).
The optional RealSense package is not involved in this header-only diagnostic
oracle; the core fallback status remains as documented in
`oracle_build_diagnostic.md`.

Exact WSL commands used to compile and link the external diagnostic executable
were:

```bash
g++ -DBASALT_INSTANTIATIONS_DOUBLE -DBASALT_INSTANTIATIONS_FLOAT \
  -DBOOST_ATOMIC_NO_LIB -DBOOST_ATOMIC_STATIC_LINK \
  -DBOOST_CHRONO_NO_LIB -DBOOST_CHRONO_STATIC_LINK \
  -DBOOST_CONTAINER_NO_LIB -DBOOST_CONTAINER_STATIC_LINK \
  -DBOOST_DATE_TIME_NO_LIB -DBOOST_DATE_TIME_STATIC_LINK \
  -DBOOST_FILESYSTEM_NO_LIB -DBOOST_FILESYSTEM_STATIC_LINK=1 \
  -DBOOST_PROGRAM_OPTIONS_NO_LIB -DBOOST_PROGRAM_OPTIONS_STATIC_LINK \
  -DBOOST_THREAD_NO_LIB -DBOOST_THREAD_STATIC_LINK \
  -DBOOST_THREAD_USE_LIB \
  -I/root/visloc-basalt-oracle-0f3b2b52/include \
  -I/root/visloc-basalt-oracle-0f3b2b52/thirdparty/ros/include \
  -I/root/visloc-basalt-oracle-0f3b2b52/thirdparty/apriltag/include \
  -isystem /root/visloc-basalt-oracle-0f3b2b52/thirdparty/vcpkg/buildtrees/eigen3/src/5.0.1-d487a628b0.clean \
  -isystem /root/visloc-basalt-oracle-0f3b2b52/build/relwithdebinfo/vcpkg_installed/x64-linux/include/opencv4 \
  -isystem /root/visloc-basalt-oracle-0f3b2b52/build/relwithdebinfo/vcpkg_installed/x64-linux/include \
  -Wall -Wextra -Werror -Wno-error=unused-parameter \
  -ftemplate-backtrace-limit=0 -Wno-error=maybe-uninitialized \
  -Wno-error=implicit-fallthrough -Wno-error=deprecated-declarations \
  -Wno-error=deprecated-copy -Wno-parentheses -DEIGEN_DONT_PARALLELIZE \
  -march=native -O3 -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN -std=c++17 \
  -c /mnt/c/Users/rsasa/Workspace/visloc-rs/benchmarks/basalt/upstream_m8d_setup_opt_oracle.cpp \
  -o /tmp/upstream_m8d_setup_opt_oracle.o
g++ /tmp/upstream_m8d_setup_opt_oracle.o \
  -o /tmp/upstream_m8d_setup_opt_oracle -O3 -g \
  -Wl,-rpath,/root/visloc-basalt-oracle-0f3b2b52/build/core-relwithdebinfo \
  -L/root/visloc-basalt-oracle-0f3b2b52/build/core-relwithdebinfo \
  -lbasalt -lpthread -ldl -lm -lrt
```

The resulting executable has SHA-256
`a6e69d1f1965f37d9896647fe5252d24829a6cc3bfb986a027d97deb5c4b1e09`.
It was run through the existing Windows-to-WSL wrapper with
`target/m8d_setup_m7v80.bin`, `target/euroc_ds_calib.json`, and
`target/euroc_config.json`.  Input and output hashes are:

| Artifact | SHA-256 |
| --- | --- |
| `benchmarks/basalt/upstream_m8d_setup_opt_oracle.cpp` | `d9d46a144c30faa1e4fd885ddd396a29cb4ff17b3ba6ea594a46859cad31e16a` |
| `target/m8d_setup_m7v80.bin` | `98da4cec1dcb39a0150534313c69beb0c23c28c511e878bc49a2d6db6ccd7aad` |
| `target/euroc_ds_calib.json` | `ad8c5a18c48c55dacf61d18ebbc18cd7d4f3acccd5840bd5a5cd8646adb6271c` |
| `target/euroc_config.json` | `82937bd6493e592ef89572d31260c10f7437b4fb3ff1fda179375713966e34fa` |
| `target/m8d_setup_m7v80.oracle5.json` | `6dd82cdea6972a05bada827d12bb1b5095ef3877ac46969f60273cf29e8aa312` |
| `target/m8d_setup_m7v80.oracle5.json.canonical.bin` | `6e6468260742d59bf7bdbe01e788543cd2ba9a3f2668afd5d33e9f66702c3125` |

The ignored integration gate was then run as:

```powershell
$env:VISLOC_BASALT_M8D_SETUP_ORACLE = (Resolve-Path target/run_m8d_setup_oracle.cmd).Path
$env:VISLOC_BASALT_CALIBRATION = (Resolve-Path target/euroc_ds_calib.json).Path
$env:VISLOC_BASALT_CONFIG = (Resolve-Path target/euroc_config.json).Path
$env:VISLOC_BASALT_M8D_SETUP_INPUT = (Resolve-Path target/m8d_setup_m7v80.bin).Path
& C:\Users\rsasa\.cargo\bin\cargo.exe test -p visloc-basalt `
  --test m8d_setup_opt_oracle -- --ignored --nocapture
```

Result: `1 passed, 0 failed`.  Rust and the rebuilt C++ oracle agree on every
track decision, host/second image, rejection count, observation count, and
canonical bytes.  The aggregate is unchanged: 554 input/attempted tracks,
549 accepted, 631 candidate attempts, 5,687 retained observations,
baseline-too-small 43, inverse-distance-nonpositive 26,
inverse-distance-too-large 13, and canonical hash
`18384097344545831442`.  The retained Rust canonical stream is byte-identical
to the rebuilt oracle stream (both SHA-256
`6e6468260742d59bf7bdbe01e788543cd2ba9a3f2668afd5d33e9f66702c3125`); no
remaining semantic differences were found.  The generated `oracle5` files
remain target artifacts and the executable remains external under `/tmp`.

The mapper's legacy global BA remains a separate compatibility scaffold; this
artifact claims only setup/initialization parity and does not claim full NFR
global BA parity.
