// Diagnostic-only probe for the pinned Sophus SO3 state-update quaternion.
// Inputs are direct float32 bits from M7IM15 frame-4 step/state captures.
#include <sophus/so3.hpp>

#include <bit>
#include <cstdint>
#include <iomanip>
#include <iostream>

static float f(std::uint32_t bits) { return std::bit_cast<float>(bits); }
static std::uint32_t b(float value) { return std::bit_cast<std::uint32_t>(value); }

static void print(const char* label, const Eigen::Quaternionf& q) {
  std::cout << label << '=' << std::hex << std::setfill('0') << std::setw(8)
            << b(q.x()) << ',' << std::setw(8) << b(q.y()) << ',' << std::setw(8)
            << b(q.z()) << ',' << std::setw(8) << b(q.w()) << std::dec << '\n';
}

static void run(const char* label, const std::uint32_t old_bits[4],
               const std::uint32_t omega_bits[3]) {
  Eigen::Quaternionf old(f(old_bits[3]), f(old_bits[0]), f(old_bits[1]),
                         f(old_bits[2]));
  Eigen::Vector3f omega(f(omega_bits[0]), f(omega_bits[1]), f(omega_bits[2]));
  const Sophus::SO3f delta = Sophus::SO3f::exp(omega);
  const Eigen::Quaternionf raw =
      Sophus::SO3f::QuaternionProduct<Eigen::Quaternionf>(
          delta.unit_quaternion(), old);
  const Sophus::SO3f result = Sophus::SO3f::exp(omega) * Sophus::SO3f(old);
  print(label, result.unit_quaternion());
  print("raw", raw);
  std::cout << "omega=" << std::hex << b(omega.x()) << ',' << b(omega.y())
            << ',' << b(omega.z()) << std::dec << '\n';
}

int main() {
  // Frame-1..4 old state q and the direct 3-lane inc_post_neg rotation.
  const std::uint32_t q1[4] = {0xbd5cdea6, 0xbf4d341f, 0xbb2aa78b,
                               0x3f186f67};
  const std::uint32_t q2[4] = {0xbd5cf5f2, 0xbf4d97c0, 0xbbac4997,
                               0x3f17e7a6};
  const std::uint32_t q3[4] = {0xbd5e760f, 0xbf4e5e85, 0xbbc2d95f,
                               0x3f16d68a};
  const std::uint32_t q4[4] = {0xbd5f79e6, 0xbf4f45a2, 0xbbb53f68,
                               0x3f159719};
  const std::uint32_t w1[3] = {0xb92b55aa, 0xb9418699, 0x38359f2f};
  const std::uint32_t w2[3] = {0xb84253bc, 0xb99a78ae, 0x387d38e4};
  const std::uint32_t w3[3] = {0xbc5fc78c, 0xbe66830b, 0x38e89af4};
  const std::uint32_t w4[3] = {0xbb1a7572, 0xbb34a7b0, 0xbd382799};
  run("frame1", q1, w1);
  run("frame2", q2, w2);
  run("frame3", q3, w3);
  run("frame4", q4, w4);
}
