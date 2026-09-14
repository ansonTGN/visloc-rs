// Diagnostic-only hook used by m7hd_native_intermediate_probe.sh.
// It is included only in a generated copy of the pinned Basalt header; the
// pinned checkout and production binary are never edited.
#pragma once

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <string>

namespace basalt {
namespace m7hd_trace_detail {

inline std::ofstream& stream() {
  static std::ofstream out([] {
    const char* path = std::getenv("M7HD_NATIVE_TRACE");
    return std::string(path && *path ? path : "/tmp/m7hd_native_intermediate.jsonl");
  }());
  return out;
}

template <typename Value>
inline std::uint32_t bits(Value value) {
  static_assert(sizeof(Value) == sizeof(std::uint32_t));
  std::uint32_t result = 0;
  std::memcpy(&result, &value, sizeof(result));
  return result;
}

template <typename Value>
inline void vector_bits(std::ostream& out, const Value& value) {
  for (Eigen::Index i = 0; i < value.size(); ++i) {
    if (i) out << ',';
    out << std::hex << std::setw(8) << std::setfill('0')
        << bits(value.derived().coeff(i));
  }
  out << std::dec;
}

template <typename Value>
inline void matrix_bits(std::ostream& out, const Value& value) {
  for (Eigen::Index i = 0; i < value.size(); ++i) {
    if (i) out << ',';
    out << std::hex << std::setw(8) << std::setfill('0')
        << bits(value.derived().coeff(i));
  }
  out << std::dec;
}

template <typename Data, typename Mat3, typename PreQuat, typename ExpQuat,
          typename PostQuat, typename State, typename MatNN, typename MatN3>
inline void propagate(const Data& data, float dt, const Mat3& r_half,
                      const PreQuat& pre_q, const ExpQuat& exp_q,
                      const PostQuat& post_q, const State& next_state,
                      const Mat3& jr, const Mat3& jr2, const MatNN& f,
                      const MatN3& a, const MatN3& g,
                      const Mat3& upper_product, const Mat3& upper_dt,
                      const Mat3& lower_frot_rhalf,
                      const Mat3& lower_jr2, const Mat3& lower_half,
                      const Mat3& lower_dt, const Mat3& position_scale) {
  static std::size_t packet = 0;
  ++packet;
  std::ostream& out = stream();
  out << "M7HD_NATIVE_STEP n=" << packet << " data_t_ns=" << data.t_ns
      << " dt_bits=" << std::hex << std::setw(8) << std::setfill('0')
      << bits(dt) << std::dec << " accel=";
  vector_bits(out, data.accel);
  out << " gyro=";
  vector_bits(out, data.gyro);
  out << " q_pre=";
  vector_bits(out, pre_q);
  out << " q_exp=";
  vector_bits(out, exp_q);
  out << " q_post=";
  vector_bits(out, post_q);
  out << " r_half=";
  matrix_bits(out, r_half);
  out << " r_new=";
  matrix_bits(out, next_state.T_w_i.so3().matrix());
  out << " jr=";
  matrix_bits(out, jr);
  out << " jr2=";
  matrix_bits(out, jr2);
  out << " f=";
  matrix_bits(out, f);
  out << " a=";
  matrix_bits(out, a);
  out << " g=";
  matrix_bits(out, g);
  out << " g_upper_product=";
  matrix_bits(out, upper_product);
  out << " g_upper_dt=";
  matrix_bits(out, upper_dt);
  out << " g_lower_frot_rhalf=";
  matrix_bits(out, lower_frot_rhalf);
  out << " g_lower_jr2=";
  matrix_bits(out, lower_jr2);
  out << " g_lower_half=";
  matrix_bits(out, lower_half);
  out << " g_lower_dt=";
  matrix_bits(out, lower_dt);
  out << " g_position_scale=";
  matrix_bits(out, position_scale);
  out << '\n';
  out.flush();
}

template <typename Data, typename MatNN, typename MatN3, typename Vec3>
inline void covariance(const Data& data, const MatNN& cov_before,
                       const MatNN& f, const MatN3& a, const MatN3& g,
                       const Vec3& accel_cov, const Vec3& gyro_cov,
                       const MatNN& cov_after) {
  static std::size_t packet = 0;
  ++packet;
  const MatNN term1 = f * cov_before * f.transpose();
  const MatN3 accel_scaled = a * accel_cov.asDiagonal();
  const MatN3 gyro_scaled = g * gyro_cov.asDiagonal();
  const MatNN term2_scaled = accel_scaled * a.transpose();
  const MatNN term3_scaled = gyro_scaled * g.transpose();
  const MatNN term2 = a * accel_cov.asDiagonal() * a.transpose();
  const MatNN term3 = g * gyro_cov.asDiagonal() * g.transpose();
  std::ostream& out = stream();
  out << "M7HD_NATIVE_COV n=" << packet << " dt_ns=" << data.t_ns
      << " accel_cov=";
  vector_bits(out, accel_cov);
  out << " gyro_cov=";
  vector_bits(out, gyro_cov);
  out << " cov_before=";
  matrix_bits(out, cov_before);
  out << " accel_scaled=";
  matrix_bits(out, accel_scaled);
  out << " gyro_scaled=";
  matrix_bits(out, gyro_scaled);
  out << " term1=";
  matrix_bits(out, term1);
  out << " term2=";
  matrix_bits(out, term2);
  out << " term2_scaled=";
  matrix_bits(out, term2_scaled);
  out << " term3=";
  matrix_bits(out, term3);
  out << " term3_scaled=";
  matrix_bits(out, term3_scaled);
  out << " cov=";
  matrix_bits(out, cov_after);
  out << '\n';
  out.flush();
}

}  // namespace m7hd_trace_detail

using m7hd_trace_detail::covariance;
using m7hd_trace_detail::propagate;

}  // namespace basalt
