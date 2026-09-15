#!/usr/bin/env bash
set -euo pipefail

# Build a diagnostic-only copy of the exact pinned header.  The checkout and
# the clean basalt_vio binary are never modified.  The generated header adds
# observers after the existing propagateState calculations and after the
# existing covariance assignment; it does not replace any estimator formula.

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
pinned_root="${M7HD_PINNED_ROOT:-/root/visloc-basalt-oracle-0f3b2b52}"
header_src="$pinned_root/thirdparty/vcpkg/packages/basalt-headers_x64-linux/include/basalt/imu/preintegration.h"
generated_root="$repo_root/target/m7hd_native_include"
header_dst="$generated_root/basalt/imu/preintegration.h"
probe_src="$repo_root/benchmarks/basalt/m7hd_native_intermediate_probe.cpp"
probe_bin="$repo_root/target/m7hd_native_intermediate_probe"
trace_out="${M7HD_NATIVE_TRACE:-$repo_root/target/m7hd_native_intermediate.jsonl}"

test -f "$header_src"
mkdir -p "$(dirname "$header_dst")"
cp "$header_src" "$header_dst"

python3 - "$header_dst" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text()

needle = '#include <basalt/utils/sophus_utils.hpp>\n'
if text.count(needle) != 1:
    raise SystemExit('unexpected pinned-header include layout')
text = text.replace(needle, needle + '#include <m7hd_native_trace_hook.hpp>\n', 1)

needle = ('      d_next_d_gyro->template block<3, 3>(0, 0) =\n'
          '          Scalar(0.5) * dt * d_next_d_gyro->template block<3, 3>(6, 0);\n'
          '    }\n'
          '  }\n')
replacement = ('      d_next_d_gyro->template block<3, 3>(0, 0) =\n'
               '          Scalar(0.5) * dt * d_next_d_gyro->template block<3, 3>(6, 0);\n'
               '      m7hd_trace_detail::propagate(\n'
               '          data, dt, RR_w_i_new_2,\n'
               '          curr_state.T_w_i.so3().unit_quaternion().coeffs(),\n'
               '          SO3::exp(dt * data.gyro).unit_quaternion().coeffs(),\n'
               '          next_state.T_w_i.so3().unit_quaternion().coeffs(),\n'
               '          next_state, Jr, Jr2, *d_next_d_curr, *d_next_d_accel,\n'
               '          *d_next_d_gyro,\n'
               '          Mat3(next_state.T_w_i.so3().matrix() * Jr),\n'
               '          Mat3(next_state.T_w_i.so3().matrix() * Jr * dt),\n'
               '          Mat3(SO3::hat(-accel_world * dt) * RR_w_i_new_2),\n'
               '          Mat3(SO3::hat(-accel_world * dt) * RR_w_i_new_2 * Jr2),\n'
               '          Mat3(SO3::hat(-accel_world * dt) * RR_w_i_new_2 * Jr2 * Scalar(0.5)),\n'
               '          Mat3(SO3::hat(-accel_world * dt) * RR_w_i_new_2 * Jr2 * Scalar(0.5) * dt),\n'
               '          Mat3(Scalar(0.5) * dt *\n'
               '              Mat3(SO3::hat(-accel_world * dt) * RR_w_i_new_2 * Jr2 * Scalar(0.5) * dt)));\n'
               '    }\n'
               '  }\n')
if text.count(needle) != 1:
    raise SystemExit('unexpected pinned-header propagate layout')
text = text.replace(needle, replacement, 1)

needle = '    cov_ = F * cov_ * F.transpose() +\n'
if text.count(needle) != 1:
    raise SystemExit('unexpected pinned-header covariance layout')
text = text.replace(needle, '    const MatNN cov_before = cov_;\n' + needle, 1)

needle = ('           G * gyro_cov.asDiagonal() * G.transpose();\n'
          '    sqrt_cov_inv_computed_ = false;\n')
replacement = ('           G * gyro_cov.asDiagonal() * G.transpose();\n'
               '    m7hd_trace_detail::covariance(data_corrected, cov_before, F, A, G,\n'
               '                                  accel_cov, gyro_cov, cov_);\n'
               '    sqrt_cov_inv_computed_ = false;\n')
if text.count(needle) != 1:
    raise SystemExit('unexpected pinned-header covariance tail layout')
text = text.replace(needle, replacement, 1)
path.write_text(text)
PY

eigen_inc="$pinned_root/thirdparty/vcpkg/packages/eigen3_x64-linux/include/eigen3"
sophus_inc="$pinned_root/thirdparty/vcpkg/packages/sophus_x64-linux/include"
basalt_inc="$pinned_root/thirdparty/vcpkg/packages/basalt-headers_x64-linux/include"
hook_inc="$repo_root/benchmarks/basalt"

g++ -std=c++17 -O3 -g -march=skylake -DEIGEN_DONT_PARALLELIZE \
  -I"$generated_root" -I"$hook_inc" -I"$basalt_inc" \
  -I"$eigen_inc" -I"$sophus_inc" "$probe_src" -o "$probe_bin"

rm -f "$trace_out"
export M7HD_NATIVE_TRACE="$trace_out"
csv="$repo_root/target/basalt_upstream_input_mh01_400f_20260821T000005Z/dataset/MH_01_easy/mav0/imu0/data.csv"
start_ns=1403636579763555584
end_ns=1403636579813555456
"$probe_bin" "$csv" "$start_ns" "$end_ns"
echo "native_trace=$trace_out"
