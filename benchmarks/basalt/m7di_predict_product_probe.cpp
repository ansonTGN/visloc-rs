// Diagnostic-only pinned Sophus product probe for the remaining state lanes.
// Inputs are the authoritative clean frame-1/frame-3 states and frame-link
// deltas; no production code or runtime diagnostics depend on this file.
#include <sophus/so3.hpp>

#include <bit>
#include <cstdint>
#include <iostream>

static float f(std::uint32_t bits) { return std::bit_cast<float>(bits); }
static std::uint32_t b(float value) { return std::bit_cast<std::uint32_t>(value); }

static void print(const char* label, const Sophus::SO3f::QuaternionType& q) {
  std::cout << label << "=" << std::hex << b(q.x()) << "," << b(q.y()) << ","
            << b(q.z()) << "," << b(q.w()) << std::dec << "\n";
}

static void run(const char* label, const std::uint32_t q_bits[4],
               const std::uint32_t dq_bits[4]) {
  Eigen::Quaternionf q(f(q_bits[3]), f(q_bits[0]), f(q_bits[1]), f(q_bits[2]));
  Eigen::Quaternionf dq(f(dq_bits[3]), f(dq_bits[0]), f(dq_bits[1]),
                        f(dq_bits[2]));
  const Sophus::SO3f result = Sophus::SO3f(q) * Sophus::SO3f(dq);
  print(label, result.unit_quaternion());
}

int main() {
  // Clean native frame-1 state and (1,2] delta.
  const std::uint32_t q1[4] = {0xbd5cdea6, 0xbf4d341f, 0xbb2aa78b,
                               0x3f186f67};
  const std::uint32_t dq12[4] = {0xbb1356a1, 0xbb1ec3a1, 0xbad83d21,
                                 0x3f7fff8e};
  // Clean native frame-3 state and (3,4] delta.
  const std::uint32_t q3[4] = {0xbd5e760f, 0xbf4e5e85, 0xbbc2d95f,
                               0x3f16d68a};
  const std::uint32_t dq34[4] = {0xb8675a3d, 0xbbc5a149, 0x396ab03e,
                                 0x3f7ffecf};
  run("frame1_to_frame2", q1, dq12);
  run("frame3_to_frame4", q3, dq34);
}
