# M7cr clean pinned native Basalt oracle

Date: 2026-08-23 JST  
Scope: a clean, separate Basalt checkout at
0f3b2b52c807f70ff4e2973ce253c73329eea7bc, core RelWithDebInfo build, and
an MH_01_easy five-frame smoke. This artifact exists to replace the
diagnostic checkout used by M7bo/M7co as the native oracle.

## Disposition

**Pass: clean pinned native core oracle.** The source checkout is a detached
worktree at the requested commit with an empty git status --porcelain, its
vcpkg submodule is at the requested pinned commit, and all 75 core build
targets plus 18 CTest tests passed. The resulting basalt_vio and
libbasalt.so are unstripped RelWithDebInfo artifacts with debug symbols.

The build is the documented core-only fallback (VCPKG_MANIFEST_INSTALL=OFF)
because the official full manifest remains blocked by the known RealSense2
2.51.1/GCC 11.4 dependency-source error. No dependency was downloaded or
patched during this run; the existing local vcpkg package/source cache was
reused. This is therefore a clean core algorithm oracle, not a claim that
the optional full RealSense manifest builds.

Machine-readable provenance is in
[target/m7cr_clean_oracle_manifest.json](../../target/m7cr_clean_oracle_manifest.json).

## Clean checkout and source verification

The diagnostic checkout was not cleaned, reverted, or rebuilt. A separate
worktree was made from its existing Git object database:

~~~
source object/cache: /root/visloc-basalt-oracle-0f3b2b52
clean worktree:      /root/visloc-basalt-clean-m7cr-20260823
Basalt commit:       0f3b2b52c807f70ff4e2973ce253c73329eea7bc
Basalt tree SHA-1:    b7afb830d82b45b8209cf784ad9744025d838411
worktree status:      empty
vcpkg submodule:      1e199d32ad53aab1defda61ce41c380302e3f95c
vcpkg baseline:       05442024c3fda64320bd25d2251cc9807b84fb6f
~~~

The worktree's tracked source hashes match upstream_manifest.json,
including:

~~~
CMakeLists.txt                              a33c58fc1753185a6fd963abf67212ea8ae0ed803a299b4327185cb581fe88ea
CMakePresets.json                           37e30c86f6154625fb5f467f9c8e4b45c33d30f7a1523f9dedfc115436eeee75
vcpkg.json                                  3616ac19a2d400b2eaf1a4490d0176dd4bda86c17740e4fb37f7bac2d5f8f0a1
vcpkg-configuration.json                    28aeb591c23215efc5cddd5cb9234bf862489698f9ca0c21591a08bdf9293ae1
src/linearization/linearization_abs_qr.cpp  0998b9d9cad3971af8003374cd343d4b96af7b4014f9ddc23e488bf160434a7e
src/vi_estimator/sqrt_keypoint_vio.cpp      141dc9747dfb60e19e8ab93eab3b5600a6bb95fc249afdd5e15f4d844276c5c9
~~~

git diff --quiet and git status --porcelain=v1 --untracked-files=all both
returned clean. A case-sensitive source search found no
BA_REL_DIAG, RUNTIME_REL, BASALT_TRACE_JSONL, or BASALT_IMU_TRACE_JSONL
marker. Exact-string scans of the built binary and shared library found none
of those markers either. The original diagnostic checkout still reports only
these five pre-existing edits:

~~~
include/basalt/linearization/landmark_block.hpp
include/basalt/linearization/landmark_block_abs_dynamic.hpp
include/basalt/utils/ba_utils.h
src/linearization/linearization_abs_qr.cpp
src/vi_estimator/sqrt_keypoint_vio.cpp
~~~

## Dependencies and toolchain

The clean worktree's vcpkg submodule was initialized from the local existing
cache (no network fetch). CMake reused the already-installed x64-linux
packages from the documented core fallback:

~~~
installed package root: /root/visloc-basalt-oracle-0f3b2b52/build/relwithdebinfo/vcpkg_installed
Eigen:  5.0.1; source cache suffix d487a628b0; archive SHA-512 b337d3bc38440db190a8f1fbc4eabc0098e69fcc95bfba195fe039ffb942cae2a7f0153f3094f35fa26325750d1c62e20cccaf916a41f5c7f248ec5e5d30a942
Sophus: 1.24.6; source cache suffix a6c9c87689; archive SHA-512 cbc01e92c8361937194bed320ac84a7cfd8b71ecc3a842d3d3c9796ff52a08d13aa0b4f30184c4c7ddc223da0141a80176382c8b25a328e53fa00c4627511ec3
~~~

The corresponding vcpkg port git-tree SHAs are Eigen
d76be65aad61fe24ae3d70c10fca0bd2c592308e and Sophus
49be1b6ceb9d34ecbf5a7256a5a670315b9083dc. The full archive SHA-256 values,
port-file hashes, and cache paths are recorded in the manifest.

Tool versions:

~~~
GNU g++ 11.4.0 (Ubuntu 11.4.0-1ubuntu1~22.04.3)
CMake 4.4.2
Ninja 1.10.1
CTest 4.4.2
~~~

The effective target compile flags taken from the generated Ninja rules are:

~~~
-O3 -g -DNDEBUG -march=native -std=c++17
-DEIGEN_DONT_PARALLELIZE -DEIGEN_INITIALIZE_MATRICES_BY_NAN
-Wall -Wextra -Werror
~~~

The build uses Basalt's normal float/double instantiation symbols and
RelWithDebInfo debug information (file reports both outputs as
"with debug_info, not stripped"). The CMake cache retains its standard
CMAKE_CXX_FLAGS_RELWITHDEBINFO=-O2 -g -DNDEBUG, but Basalt's generated target
rules explicitly add -O3; the effective command-line flags above are the
ones used by the compiler.

## Exact build commands

~~~bash
git -C /root/visloc-basalt-oracle-0f3b2b52 \
  worktree add --detach /root/visloc-basalt-clean-m7cr-20260823 \
  0f3b2b52c807f70ff4e2973ce253c73329eea7bc

git -C /root/visloc-basalt-clean-m7cr-20260823 \
  -c protocol.file.allow=always submodule update --init --recursive \
  --reference /root/visloc-basalt-oracle-0f3b2b52/thirdparty/vcpkg

cmake -S /root/visloc-basalt-clean-m7cr-20260823 \
  -B /root/visloc-basalt-clean-m7cr-20260823/build/core-relwithdebinfo \
  -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=/root/visloc-basalt-clean-m7cr-20260823/thirdparty/vcpkg/scripts/buildsystems/vcpkg.cmake \
  -DVCPKG_MANIFEST_INSTALL=OFF \
  -DVCPKG_INSTALLED_DIR=/root/visloc-basalt-oracle-0f3b2b52/build/relwithdebinfo/vcpkg_installed \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCXX_MARCH=native

cmake --build /root/visloc-basalt-clean-m7cr-20260823/build/core-relwithdebinfo --parallel 4
ctest --test-dir /root/visloc-basalt-clean-m7cr-20260823/build/core-relwithdebinfo --output-on-failure
~~~

Configure and build succeeded (75/75); CTest passed 18/18.

## Native artifact hashes

~~~
basalt_vio
  path: /root/visloc-basalt-clean-m7cr-20260823/build/core-relwithdebinfo/basalt_vio
  SHA-256: 89e0324ccd04c3b615bf7e945aa32480a2f86524987aa00eaf152dcd6b1d242c
  size: 18024832 bytes
  Build ID: 5d10252dc9e249bc228a4c25cdc11cc68d8944d8

libbasalt.so
  path: /root/visloc-basalt-clean-m7cr-20260823/build/core-relwithdebinfo/libbasalt.so
  SHA-256: 55f842642b370edf4025afd1d66c7b9d9011e5b4ce08efeb9d76be401b45b2cc
  size: 421516504 bytes
  Build ID: cd04ec0e0ef75097c5a2e8c3bbc4cd9e719be362

data/euroc_config.json
  SHA-256: 82937bd6493e592ef89572d31260c10f7437b4fb3ff1fda179375713966e34fa
  size: 2401 bytes

data/euroc_ds_calib.json
  SHA-256: ad8c5a18c48c55dacf61d18ebbc18cd7d4f3acccd5840bd5a5cd8646adb6271c
  size: 5967 bytes

build/CMakeCache.txt
  SHA-256: 5f010f5a0e7bb1e01bc832f037916f239ffcc988c36ee74f477d03ac7007559d
build/build.ninja
  SHA-256: 9715b9e6a5399365ee473608596ac9a899e84aa352b9a1287d1e56d8cbf5d437
~~~

## MH_01_easy max-5 smoke

The smoke intentionally supplied no ground-truth path or save-groundtruth
option. It ran to completion for five frames and produced five trajectory
data rows (six lines including the header), 61 active frame-0 landmarks, and
an empty stderr. The T_align/error summary prints NaN in this mode because
there is no ground truth to associate; that is expected and not a solver
failure.

~~~bash
mkdir -p /root/basalt-oracle-results-clean-m7cr/MH_01_easy/20260823T113500Z/marg_data
cd /root/basalt-oracle-results-clean-m7cr/MH_01_easy/20260823T113500Z
/root/visloc-basalt-clean-m7cr-20260823/build/core-relwithdebinfo/basalt_vio \
  --dataset-path /mnt/e/datasets/euroc_mav/machine_hall/MH_01_easy \
  --cam-calib /root/visloc-basalt-clean-m7cr-20260823/data/euroc_ds_calib.json \
  --dataset-type euroc --show-gui 0 \
  --config-path /root/visloc-basalt-clean-m7cr-20260823/data/euroc_config.json \
  --marg-data /root/basalt-oracle-results-clean-m7cr/MH_01_easy/20260823T113500Z/marg_data \
  --save-trajectory euroc --num-threads 4 --max-frames 5 \
  > stdout.log 2> stderr.log
~~~

Smoke output paths and hashes:

~~~
output root: /root/basalt-oracle-results-clean-m7cr/MH_01_easy/20260823T113500Z
trajectory.csv SHA-256: fc49e6f5d6397f66e1f24091bec5d41b2c53586c6225c14fcec4ef55dc77dcce
stdout.log    SHA-256: c50cb151b1dfe06ebdba26a415246734826126b2afcd855a8533db9a538a2643
stderr.log    SHA-256: e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855
marg_data/gt.cereal SHA-256: 8d419b19fc35a574247bd7a2970a5f8dffdca68fa1fdc0236072723215685fd2
~~~

All source, build, and smoke paths/hashes are duplicated in
[m7cr_clean_oracle_manifest.json](../../target/m7cr_clean_oracle_manifest.json)
so downstream oracle consumers do not need to infer them from this report.

No Rust production source, Rust workspace build, commit, or push was made.
