#include <sophus/so3.hpp>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>

static float f(std::uint32_t bits) {
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

static std::uint32_t b(float value) {
  std::uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

static void run(const char* label, const std::uint32_t bits[3]) {
  const float x = f(bits[0]);
  const float y = f(bits[1]);
  const float z = f(bits[2]);
  const float theta_sq = x * x + y * y + z * z;
  const float theta_sq_pair = (x * x + y * y) + z * z;
  const Eigen::Vector3f omega(x, y, z);
  const float theta_sq_eigen = omega.squaredNorm();
  const float theta_sq_eigen_dot = omega.dot(omega);
  const float theta_eigen = std::sqrt(theta_sq_eigen);
  const float half_eigen = 0.5f * theta_eigen;
  const float cos_eigen = std::cos(half_eigen);
  const float sophus_w = Sophus::SO3f::exp(omega).unit_quaternion().w();
  const float theta = std::sqrt(theta_sq);
  const float half = 0.5f * theta;
  const float sin_v = std::sin(half);
  const float cos_v = std::cos(half);
  float sin_sc = 0.0f;
  float cos_sc = 0.0f;
  __builtin_sincosf(half, &sin_sc, &cos_sc);
  const float imag = sin_v / theta;
  std::cout << label << " omega=" << std::hex << b(x) << "," << b(y) << "," << b(z)
            << " theta_sq=" << b(theta_sq) << " theta_sq_pair=" << b(theta_sq_pair)
            << " theta_sq_eigen=" << b(theta_sq_eigen)
            << " theta_sq_eigen_dot=" << b(theta_sq_eigen_dot)
            << " theta_eigen=" << b(theta_eigen) << " half_eigen=" << b(half_eigen)
            << " cos_eigen=" << b(cos_eigen) << " sophus_w=" << b(sophus_w)
            << " theta=" << b(theta) << " half=" << b(half) << " sin=" << b(sin_v)
            << " cos=" << b(cos_v) << " sincos=" << b(sin_sc) << "," << b(cos_sc)
            << " imag=" << b(imag) << std::dec << "\n";
}

int main() {
  const std::uint32_t frame1[3] = {0xb92b55aa, 0xb9418699, 0x38359f2f};
  const std::uint32_t frame2[3] = {0xb84253bc, 0xb99a78ae, 0x387d38e4};
  const std::uint32_t frame3[3] = {0x38781aee, 0xb9dc2926, 0x38c40d56};
  const std::uint32_t frame4[3] = {0x3890e3ec, 0xb98d5f2c, 0x3883baf9};
  run("frame1", frame1);
  run("frame2", frame2);
  run("frame3", frame3);
  run("frame4", frame4);
}
