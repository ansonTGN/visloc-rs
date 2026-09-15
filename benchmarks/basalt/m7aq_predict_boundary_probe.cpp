// Diagnostic-only f32 prediction boundary probe for pinned Basalt/Sophus.
// The operands are the authoritative MH_01 frame-0 state and frame-0->1
// preintegrated delta.  It prints the Sophus point action and the exact
// left-associated prediction terms used by IntegratedImuMeasurement::predictState.

#include <basalt/imu/preintegration.h>

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
  using Scalar = float;
  using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
  using Quaternion = Eigen::Quaternion<Scalar>;
  using SO3 = Sophus::SO3<Scalar>;

  const Quaternion q0(Scalar(0.5944822430610657),
                      Scalar(-0.052778493613004684),
                      Scalar(-0.8023747801780701), Scalar(0));
  const Vec3 dp(Scalar(0.009641510434448719),
                Scalar(-0.00070759042864665389),
                Scalar(-0.0033708729315549135));
  const Vec3 dv(Scalar(0.38366442918777466),
                Scalar(-0.029830120503902435),
                Scalar(-0.14031206071376801));
  const Vec3 p0 = Vec3::Zero();
  const Vec3 v0 = Vec3::Zero();
  const Vec3 gravity(Scalar(0), Scalar(0), Scalar(-9.8100004196166992));
  const Scalar dt = Scalar(49999872) * Scalar(1e-9);

  const SO3 r0(q0);
  const Vec3 rotated_dp = r0 * dp;
  const Vec3 rotated_dv = r0 * dv;
  const Vec3 eigen_rotated_dp = q0 * dp;
  const Vec3 eigen_rotated_dv = q0 * dv;

  const Vec3 v_dt = v0 * dt;
  const Vec3 g_half = Scalar(0.5) * gravity;
  const Vec3 g_half_dt = g_half * dt;
  const Vec3 g_term = g_half_dt * dt;
  const Vec3 p_after_v = p0 + v_dt;
  const Vec3 p_after_g = p_after_v + g_term;
  const Vec3 p_after_rot = p_after_g + rotated_dp;
  const Vec3 source_p = p0 + v0 * dt + Scalar(0.5) * gravity * dt * dt +
                        r0 * dp;

  const Vec3 v_after_g = v0 + gravity * dt;
  const Vec3 v_after_rot = v_after_g + rotated_dv;
  const Vec3 source_v = v0 + gravity * dt + r0 * dv;

  std::cout << std::setprecision(17);
  std::cout << "dt=" << dt << "\n";
  print_bits("q0", q0.coeffs());
  print_bits("dp", dp);
  print_bits("dv", dv);
  print_bits("sophus_rotate_dp", rotated_dp);
  print_bits("sophus_rotate_dv", rotated_dv);
  print_bits("eigen_quat_rotate_dp", eigen_rotated_dp);
  print_bits("eigen_quat_rotate_dv", eigen_rotated_dv);
  print_bits("v_dt", v_dt);
  print_bits("g_half", g_half);
  print_bits("g_half_dt", g_half_dt);
  print_bits("g_term", g_term);
  print_bits("p_after_v", p_after_v);
  print_bits("p_after_g", p_after_g);
  print_bits("p_after_rot", p_after_rot);
  print_bits("source_p", source_p);
  print_bits("v_after_g", v_after_g);
  print_bits("v_after_rot", v_after_rot);
  print_bits("source_v", source_v);
  return 0;
}
