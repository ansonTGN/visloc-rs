// Diagnostic-only fixed-operand predictState position probe for M7fh.
#include <basalt/imu/preintegration.h>

#include <bit>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>

namespace {
uint32_t bits(float value) {
  uint32_t result = 0;
  std::memcpy(&result, &value, sizeof(result));
  return result;
}

template <typename Derived>
void print_bits(const char* name, const Eigen::MatrixBase<Derived>& value) {
  std::cout << name << "=";
  for (Eigen::Index i = 0; i < value.size(); ++i) {
    if (i) std::cout << ",";
    std::cout << std::hex << bits(value.derived()(i));
  }
  std::cout << std::dec << "\n";
}
}  // namespace

int main() {
  using S = float;
  using V = Eigen::Matrix<S, 3, 1>;
  using Q = Eigen::Quaternion<S>;
  using SO3 = Sophus::SO3<S>;

  // M7eg clean frame-3 -> frame-4 endpoint operands, all raw f32 bits.
  const Q q0(std::bit_cast<S>(0x3f16d68au), std::bit_cast<S>(0xbd5e760fu),
             std::bit_cast<S>(0xbf4e5e85u), std::bit_cast<S>(0xbbc2d95fu));
  const V p0(std::bit_cast<S>(0x3b8ebfa0u), std::bit_cast<S>(0xba9428f4u),
             std::bit_cast<S>(0xbc857642u));
  const V v0(std::bit_cast<S>(0x3d850448u), std::bit_cast<S>(0xbc998d12u),
             std::bit_cast<S>(0xbe4a560cu));
  const V g(0.0f, 0.0f, std::bit_cast<S>(0xc11cf5c3u));
  const V dp(std::bit_cast<S>(0x3c320e93u), std::bit_cast<S>(0xba2e4f54u),
             std::bit_cast<S>(0xbb7f5a2bu));
  const S dt = std::bit_cast<S>(0x3d4cccefu);

  const SO3 r0(q0);
  const V rotated_dp = r0 * dp;
  const V v_dt = v0 * dt;
  const V half_g = S(0.5) * g;
  const V half_g_dt = half_g * dt;
  const V g_term = half_g_dt * dt;
  const V p_after_v = p0 + v_dt;
  const V p_after_g = p_after_v + g_term;
  const V p_after_rot = p_after_g + rotated_dp;
  const V source_p = p0 + v0 * dt + S(0.5) * g * dt * dt + r0 * dp;

  print_bits("q0", q0.coeffs());
  print_bits("p0", p0);
  print_bits("v0", v0);
  print_bits("g", g);
  print_bits("dp", dp);
  print_bits("rotated_dp", rotated_dp);
  print_bits("v_dt", v_dt);
  print_bits("half_g", half_g);
  print_bits("half_g_dt", half_g_dt);
  print_bits("g_term", g_term);
  print_bits("p_after_v", p_after_v);
  print_bits("p_after_g", p_after_g);
  print_bits("p_after_rot", p_after_rot);
  print_bits("source_p", source_p);
  return 0;
}
