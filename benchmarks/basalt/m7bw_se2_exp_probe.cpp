// M7bw external native Sophus::SE2f::exp probe.
// The tangent is the first increment in the cam0/frame1 track-2 replay.

#include <sophus/se2.hpp>

#include <cstdint>
#include <cstring>
#include <cmath>
#include <iomanip>
#include <iostream>

static float from_bits(std::uint32_t value) {
  float result = 0.0f;
  std::memcpy(&result, &value, sizeof(result));
  return result;
}

static std::uint32_t bits(float value) {
  std::uint32_t result = 0;
  std::memcpy(&result, &value, sizeof(result));
  return result;
}

static void print_bits(float value) {
  std::cout << "0x" << std::hex << std::setw(8) << std::setfill('0')
            << bits(value) << std::dec << std::setfill(' ');
}

int main() {
  const Eigen::Vector3f tangent(
      from_bits(0xbeb28e40), from_bits(0xbe1adabe), from_bits(0xbb9ce858));
  const Sophus::SE2f update = Sophus::SE2f::exp(tangent);
  const float theta = tangent[2];
  const float sin_theta = std::sin(theta);
  const float cos_theta = std::cos(theta);
  const float sin_over_theta = sin_theta / theta;
  const float one_minus_cos_over_theta = (1.0f - cos_theta) / theta;
  const float first = sin_over_theta * tangent[0];
  const float second = one_minus_cos_over_theta * tangent[1];
  const float direct_x = first - second;
  const float fma_x = std::fma(-one_minus_cos_over_theta, tangent[1], first);
  const float direct_y = one_minus_cos_over_theta * tangent[0] +
                         sin_over_theta * tangent[1];
  const float fma_y = std::fma(one_minus_cos_over_theta, tangent[0],
                               sin_over_theta * tangent[1]);
  std::cout << "theta=";
  print_bits(theta);
  std::cout << '\n';
  std::cout << "sin_cos=";
  print_bits(sin_theta);
  std::cout << ',';
  print_bits(cos_theta);
  std::cout << '\n';
  std::cout << "coefficients=";
  print_bits(sin_over_theta);
  std::cout << ',';
  print_bits(one_minus_cos_over_theta);
  std::cout << '\n';
  std::cout << "products=";
  print_bits(first);
  std::cout << ',';
  print_bits(second);
  std::cout << '\n';
  std::cout << "direct=";
  print_bits(direct_x);
  std::cout << ',';
  print_bits(direct_y);
  std::cout << '\n';
  std::cout << "fma=";
  print_bits(fma_x);
  std::cout << ',';
  print_bits(fma_y);
  std::cout << '\n';
  std::cout << "tangent=";
  print_bits(tangent[0]);
  std::cout << ',';
  print_bits(tangent[1]);
  std::cout << ',';
  print_bits(tangent[2]);
  std::cout << '\n';
  std::cout << "rotation=";
  print_bits(update.rotationMatrix()(0, 0));
  std::cout << ',';
  print_bits(update.rotationMatrix()(0, 1));
  std::cout << ',';
  print_bits(update.rotationMatrix()(1, 0));
  std::cout << ',';
  print_bits(update.rotationMatrix()(1, 1));
  std::cout << '\n';
  std::cout << "translation=";
  print_bits(update.translation()[0]);
  std::cout << ',';
  print_bits(update.translation()[1]);
  std::cout << '\n';
  return 0;
}
