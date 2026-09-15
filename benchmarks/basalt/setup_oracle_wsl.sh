#!/usr/bin/env bash
set -Eeuo pipefail

# Reproducible upstream Basalt build for the M0/M1 oracle.
# This script is intentionally WSL/Linux-only: upstream CMake warns that
# Windows is unsupported. It does not mutate the visloc-rs worktree.

UPSTREAM_URL="https://github.com/VladyslavUsenko/basalt.git"
UPSTREAM_SHA="0f3b2b52c807f70ff4e2973ce253c73329eea7bc"
VCPKG_SHA="1e199d32ad53aab1defda61ce41c380302e3f95c"
DISTRO_EXPECTED="Ubuntu-22.04"
ORACLE_ROOT="${ORACLE_ROOT:-${HOME}/visloc-basalt-oracle-${UPSTREAM_SHA:0:12}}"
BUILD_PRESET="${BUILD_PRESET:-relwithdebinfo}"
JOBS="${JOBS:-$(nproc)}"
INSTALL_SYSTEM_DEPS="${INSTALL_SYSTEM_DEPS:-1}"

die() {
  echo "ERROR: $*" >&2
  exit 1
}

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || die "missing command: $1"
}

if [[ "${INSTALL_SYSTEM_DEPS}" == "1" ]]; then
  command -v apt-get >/dev/null 2>&1 || die "apt-get is required for the Ubuntu WSL setup"
  [[ "$(id -u)" == "0" ]] || die "run as root in WSL or set INSTALL_SYSTEM_DEPS=0"
  export DEBIAN_FRONTEND=noninteractive
  apt-get update -qq
  apt-get install -y --no-install-recommends \
    ca-certificates git curl zip unzip tar pkg-config build-essential \
    python3 python3-pip autoconf autoconf-archive automake libtool \
    libgl1-mesa-dev libglu1-mesa-dev libgles2-mesa-dev \
    libx11-dev libxcursor-dev libxinerama-dev libxrandr-dev libxi-dev \
    libxtst-dev libudev-dev libusb-1.0-0-dev libftdi-dev
fi

require_cmd git
require_cmd ninja
require_cmd g++

if ! command -v cmake >/dev/null 2>&1 || [[ "$(cmake --version | awk 'NR == 1 {print $3}')" < "3.24.0" ]]; then
  python3 -m pip install --no-cache-dir cmake
fi
require_cmd cmake

if [[ "$(uname -s)" != "Linux" || "$(uname -m)" != "x86_64" ]]; then
  die "the upstream oracle requires Linux x86_64 (use Ubuntu-22.04 WSL2)"
fi

if [[ ! -d "${ORACLE_ROOT}/.git" ]]; then
  git clone --filter=blob:none --no-checkout "${UPSTREAM_URL}" "${ORACLE_ROOT}"
fi

git -C "${ORACLE_ROOT}" fetch --depth=1 origin "${UPSTREAM_SHA}"
git -C "${ORACLE_ROOT}" checkout --detach "${UPSTREAM_SHA}"

[[ "$(git -C "${ORACLE_ROOT}" rev-parse HEAD)" == "${UPSTREAM_SHA}" ]] \
  || die "upstream checkout is not the fixed SHA"

git -C "${ORACLE_ROOT}" submodule update --init --depth=1 thirdparty/vcpkg
[[ "$(git -C "${ORACLE_ROOT}/thirdparty/vcpkg" rev-parse HEAD)" == "${VCPKG_SHA}" ]] \
  || die "vcpkg submodule is not the fixed SHA"

# vcpkg manifest mode resolves historical port trees from the configured
# baseline. A shallow checkout cannot provide those trees, so fetch the full
# vcpkg history once. This is required for reproducibility, not an algorithmic
# change to Basalt.
if [[ "$(git -C "${ORACLE_ROOT}/thirdparty/vcpkg" rev-parse --is-shallow-repository)" == "true" ]]; then
  git -C "${ORACLE_ROOT}/thirdparty/vcpkg" fetch --unshallow origin
fi

if [[ ! -x "${ORACLE_ROOT}/thirdparty/vcpkg/vcpkg" ]]; then
  "${ORACLE_ROOT}/thirdparty/vcpkg/bootstrap-vcpkg.sh" -disableMetrics
fi

cmake --version
ninja --version
g++ --version | head -n 1

cd "${ORACLE_ROOT}"
cmake --preset "${BUILD_PRESET}"
cmake --build --preset "${BUILD_PRESET}" --parallel "${JOBS}"

echo "Basalt oracle build complete: ${ORACLE_ROOT}/build/${BUILD_PRESET}"
echo "Binary: ${ORACLE_ROOT}/build/${BUILD_PRESET}/basalt_vio"
