#!/usr/bin/env bash
set -euo pipefail

# Build/run the diagnostic-only Eigen oracle.  The pinned checkout is read
# only; the generated executable and JSON stay under target/.
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
pinned_root="${M7IM_PINNED_ROOT:-/root/visloc-basalt-oracle-0f3b2b52}"
eigen_inc="$pinned_root/thirdparty/vcpkg/packages/eigen3_x64-linux/include/eigen3"
sophus_inc="$pinned_root/thirdparty/vcpkg/packages/sophus_x64-linux/include"
basalt_inc="$pinned_root/thirdparty/vcpkg/packages/basalt-headers_x64-linux/include"
fmt_inc="$pinned_root/thirdparty/vcpkg/packages/fmt_x64-linux/include"
source="$repo_root/benchmarks/basalt/m7im_cov_ldlt_oracle.cpp"
binary="$repo_root/target/m7im_cov_ldlt_oracle"
csv="${M7IM_IMU_CSV:-$repo_root/target/basalt_upstream_input_mh01_400f_20260821T000005Z/dataset/MH_01_easy/mav0/imu0/data.csv}"
states="${M7IM_STATES:-$repo_root/target/m7_postm7_upstream_frame4_states_20260822.tsv}"
output="${M7IM_OUTPUT:-$repo_root/target/m7im_cov_ldlt_oracle_20260824.json}"

test -f "$source"
test -f "$csv"
test -f "$states"
mkdir -p "$(dirname "$binary")" "$(dirname "$output")"

g++ -std=c++17 -O3 -g -Wall -Wextra -DEIGEN_DONT_PARALLELIZE -march=skylake \
  -I"$basalt_inc" -I"$eigen_inc" -I"$sophus_inc" -I"$fmt_inc" \
  "$source" -o "$binary"

rm -f "$output"
"$binary" "$csv" "$states" "$output"
sha256sum "$source" "$binary" "$csv" "$states" "$output"
