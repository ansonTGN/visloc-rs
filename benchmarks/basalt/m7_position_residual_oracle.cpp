// Diagnostic-only Eigen scalar-reduction oracle for the pinned frame-3 -> 4
// IMU position residual.  This is deliberately outside the Rust production
// tree.  All inputs are the exact f32 lanes from the pinned audit fixture
// target/m7aq_upstream_skylake_20260824.json; bit loads avoid decimal parsing.
#include <Eigen/Core>

#include <cstdint>
#include <cstring>
#include <iostream>

using V = Eigen::Matrix<float, 3, 1>;
using M = Eigen::Matrix<float, 3, 3>;

static float f(std::uint32_t u) {
  float x;
  std::memcpy(&x, &u, sizeof(x));
  return x;
}

static std::uint32_t bits(float x) {
  std::uint32_t u;
  std::memcpy(&u, &x, sizeof(u));
  return u;
}

static void dump(const char* label, const V& v) {
  std::cout << label << "=" << std::hex << bits(v.x()) << "," << bits(v.y())
            << "," << bits(v.z()) << std::dec << "\n";
}

static void dump_m(const char* label, const M& m) {
  std::cout << label << "=" << std::hex;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      if (r || c) std::cout << ",";
      std::cout << bits(m(r, c));
    }
  }
  std::cout << std::dec << "\n";
}

static float row2_left(const M& r, const V& x) {
  return (r(2, 0) * x(0) + r(2, 1) * x(1)) + r(2, 2) * x(2);
}

static float row2_right(const M& r, const V& x) {
  return r(2, 0) * x(0) + (r(2, 1) * x(1) + r(2, 2) * x(2));
}

static float row2_fma_left(const M& r, const V& x) {
  return std::fma(r(2, 0), x(0), r(2, 1) * x(1)) + r(2, 2) * x(2);
}

static float row2_fma_right(const M& r, const V& x) {
  return r(2, 0) * x(0) + std::fma(r(2, 1), x(1), r(2, 2) * x(2));
}

static float row2_fma_nested(const M& r, const V& x) {
  return std::fma(r(2, 0), x(0),
                  std::fma(r(2, 1), x(1), r(2, 2) * x(2)));
}

// Eigen's fixed-size packet evaluator on the pinned AVX2 build seeds the
// packet reduction with k=1, contracts k=2, then contracts k=0.
static float row2_packet_k1_k2_k0(const M& r, const V& x) {
  float value = r(2, 1) * x(1);
  value = std::fma(r(2, 2), x(2), value);
  value = std::fma(r(2, 0), x(0), value);
  return value;
}

static float row2_packet_k2_k1_k0(const M& r, const V& x) {
  float value = r(2, 2) * x(2);
  value = std::fma(r(2, 1), x(1), value);
  value = std::fma(r(2, 0), x(0), value);
  return value;
}

static float row2_packet_k0_k2_k1(const M& r, const V& x) {
  float value = r(2, 0) * x(0);
  value = std::fma(r(2, 2), x(2), value);
  value = std::fma(r(2, 1), x(1), value);
  return value;
}

static float row2_plain_k1_k2_k0(const M& r, const V& x) {
  float value = r(2, 1) * x(1);
  value = value + r(2, 2) * x(2);
  value = value + r(2, 0) * x(0);
  return value;
}

int main() {
  // Frame-3 state and frame-4 state from the pinned JSON, converted to the
  // exact native f32 lanes retained by the upstream trace.
  const V p0(f(0x3b8ebfa0), f(0xba9428f4), f(0xbc857642));
  const V p1(f(0x3c02c75a), f(0xbb0c3bda), f(0xbcdc2b7f));
  const V v0(f(0x3d850448), f(0xbc998d12), f(0xbe4a560c));
  const V gravity(f(0), f(0), f(0xc11cf5c3));
  const float dt = f(0x3d4cccef);
  const V dp(f(0x3c320e93), f(0xba2e4f54), f(0xbb7f5a2b));

  // This is q0^{-1}.  It is emitted by the pinned Sophus/Eigen path and is
  // kept as bits here so this probe isolates only matrix-vector reduction.
  const M r0_inv = (M() << f(0xbe997a48), f(0x3da4fb4e), f(0x3f735afd),
                    f(0x3dc1aef8), f(0x3f7e78bc), f(0xbd5ee27f),
                    f(0xbf730653), f(0x3d96b5f6), f(0xbe9c7648))
                       .finished();

  // Keep the same Eigen expression spelling as IntegratedImuMeasurement::
  // residual for the source vector, then enumerate only the row-2 dot.
  const V pos_arg = p1 - p0 - v0 * dt - 0.5f * gravity * dt * dt;
  const V native = r0_inv * pos_arg;
  const V plain_left(
      r0_inv(0, 0) * pos_arg(0) + r0_inv(0, 1) * pos_arg(1) +
          r0_inv(0, 2) * pos_arg(2),
      r0_inv(1, 0) * pos_arg(0) + r0_inv(1, 1) * pos_arg(1) +
          r0_inv(1, 2) * pos_arg(2),
      row2_left(r0_inv, pos_arg));

  dump("p0", p0);
  dump("p1", p1);
  dump("v0", v0);
  dump("gravity", gravity);
  std::cout << "dt=" << std::hex << bits(dt) << std::dec << "\n";
  dump("dp", dp);
  dump("pos_arg", pos_arg);
  dump_m("r0_inv", r0_inv);
  dump("tmp_eigen", native);
  dump("residual_eigen", native - dp);
  dump("tmp_plain_left", plain_left);
  dump("residual_plain_left", plain_left - dp);

  std::cout << "row2_left=" << std::hex << bits(row2_left(r0_inv, pos_arg))
            << "\n";
  std::cout << "row2_right=" << bits(row2_right(r0_inv, pos_arg)) << "\n";
  std::cout << "row2_fma_left=" << bits(row2_fma_left(r0_inv, pos_arg))
            << "\n";
  std::cout << "row2_fma_right=" << bits(row2_fma_right(r0_inv, pos_arg))
            << "\n";
  std::cout << "row2_fma_nested=" << bits(row2_fma_nested(r0_inv, pos_arg))
            << "\n";
  std::cout << "row2_packet_k1_k2_k0="
            << bits(row2_packet_k1_k2_k0(r0_inv, pos_arg)) << "\n";
  std::cout << "row2_packet_k2_k1_k0="
            << bits(row2_packet_k2_k1_k0(r0_inv, pos_arg)) << "\n";
  std::cout << "row2_packet_k0_k2_k1="
            << bits(row2_packet_k0_k2_k1(r0_inv, pos_arg)) << "\n";
  std::cout << "row2_plain_k1_k2_k0="
            << bits(row2_plain_k1_k2_k0(r0_inv, pos_arg)) << "\n";
  return 0;
}
