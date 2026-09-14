# Basalt oracle build diagnostic

Captured 2026-08-21 on Ubuntu-22.04 WSL2 (Linux x86_64) in support of the
fixed upstream commit `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.

## Official preset result

The source checkout and `thirdparty/vcpkg` submodule are detached at the hashes
in `upstream_manifest.json`. Native Windows was not used: the upstream
CMakeLists supports Linux/macOS only.

The first manifest attempt reached vcpkg installation and stopped at `libusb`
because the host lacked `autoconf-archive`, `libudev-dev`, and the USB build
headers. Those packages were installed once and are now included in
`setup_oracle_wsl.sh`.

The next official `relwithdebinfo` configure/build reached the fixed vcpkg
RealSense 2.51.1 port and stopped at:

```text
.../thirdparty/vcpkg/buildtrees/realsense2/src/v2.51.1-9823fe6540.clean/src/libusb/libusb.h:10:82:
error: extended character ‘ is not valid in an identifier
.../src/libusb/libusb.h:13:82: error: extended character ‘ is not valid in an identifier
.../src/libusb/libusb.h:18:83: error: extended character ‘ is not valid in an identifier
ninja: build stopped: subcommand failed.
```

The file is a `#if 0` diagnostic transcript in the RealSense dependency. GCC
11.4 still tokenizes the non-ASCII typographic quotes in that skipped group.
This is a dependency-source/compiler incompatibility, not an OOM or network
failure. The complete reproducible vcpkg log is outside the repository at:

```text
$HOME/visloc-basalt-oracle-0f3b2b52/thirdparty/vcpkg/buildtrees/realsense2/install-x64-linux-dbg-out.log
$HOME/visloc-basalt-oracle-0f3b2b52/thirdparty/vcpkg/buildtrees/realsense2/error-logs-x64-linux.txt
```

No upstream source, vcpkg port, or repository worktree file was patched to
hide this failure. Repeated retries are not useful until the compiler or the
RealSense port/source is changed under a separately reviewed dependency
policy.

## Core-only smoke fallback

The fixed Basalt source does not require RealSense for `basalt_vio`; its CMake
logic detects the package optionally. To obtain a useful MH_01 smoke without
modifying the source, the already installed non-RealSense packages were reused
with manifest installation disabled and a separate build directory:

```bash
cmake -S "$HOME/visloc-basalt-oracle-0f3b2b52" \
  -B "$HOME/visloc-basalt-oracle-0f3b2b52/build/core-relwithdebinfo" \
  -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE="$HOME/visloc-basalt-oracle-0f3b2b52/thirdparty/vcpkg/scripts/buildsystems/vcpkg.cmake" \
  -DVCPKG_MANIFEST_INSTALL=OFF \
  -DVCPKG_INSTALLED_DIR="$HOME/visloc-basalt-oracle-0f3b2b52/build/relwithdebinfo/vcpkg_installed" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCXX_MARCH=native
cmake --build "$HOME/visloc-basalt-oracle-0f3b2b52/build/core-relwithdebinfo" --parallel 4
ctest --test-dir "$HOME/visloc-basalt-oracle-0f3b2b52/build/core-relwithdebinfo" --output-on-failure
```

The core build completed 75/75 and CTest passed 18/18. The installed
`basalt_vio` was then run headlessly for the first 120 frames of MH_01; no
ground-truth path or `--save-groundtruth` flag was supplied:

```bash
ORACLE_ROOT="$HOME/visloc-basalt-oracle-0f3b2b52"
ORACLE_BUILD_DIR="$ORACLE_ROOT/build/core-relwithdebinfo"
OUT="$HOME/basalt-oracle-results-runner-smoke/MH_01_easy/20260821T000003Z"
mkdir -p "$OUT/marg_data"
cd "$OUT"
"$ORACLE_BUILD_DIR/basalt_vio" \
  --dataset-path /mnt/e/datasets/euroc_mav/machine_hall/MH_01_easy \
  --cam-calib "$ORACLE_ROOT/data/euroc_ds_calib.json" \
  --dataset-type euroc --show-gui 0 \
  --config-path "$ORACLE_ROOT/data/euroc_config.json" \
  --marg-data "$OUT/marg_data" \
  --save-trajectory euroc --num-threads 4 --max-frames 120 \
  > stdout.log 2> stderr.log
```

Recorded outside the repository:

- output: `$HOME/basalt-oracle-results-runner-smoke/MH_01_easy/20260821T000003Z/`
- trajectory rows: 121 (header plus 120 frames)
- `trajectory.csv` SHA-256: `8447b27eca2d95a5f1fcb867eee1a0545cf985036fe5db296e738bfc70c7958f`
- `command.txt` SHA-256: `2adfa88536183e8a72cc76e331b9da96648bc1b87d8693e1a1c6773aea354038`
- reported runtime: 8.997 seconds

This is a build/run smoke only. It is not a full-manifest official oracle and
its short trajectory must not be used for performance claims. The runner
supports this binary through `ORACLE_BUILD_DIR`; the provenance distinction is
also recorded in `upstream_manifest.json`.
