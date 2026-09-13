// Exact-bit probe for the pinned Eigen 3x3 product schedule.

#include <Eigen/Core>

#include <cstdint>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>

namespace {

float from_bits(std::uint32_t value) {
  float result = 0;
  std::memcpy(&result, &value, sizeof(result));
  return result;
}

std::uint32_t bits(float value) {
  std::uint32_t result = 0;
  std::memcpy(&result, &value, sizeof(result));
  return result;
}

template <typename Matrix>
void print_matrix(const char* name, const Matrix& value) {
  std::cout << name << '=';
  for (Eigen::Index i = 0; i < value.size(); ++i) {
    if (i) std::cout << ',';
    std::cout << std::hex << std::setw(8) << std::setfill('0')
              << bits(value.derived().coeff(i));
  }
  std::cout << std::dec << '\n';
}

__attribute__((noinline)) Eigen::Matrix3f eigen_product(
    const Eigen::Matrix3f& left, const Eigen::Matrix3f& right) {
  return left * right;
}

__attribute__((noinline)) Eigen::Matrix3f scalar_product(
    const Eigen::Matrix3f& left, const Eigen::Matrix3f& right) {
  Eigen::Matrix3f result;
  for (int col = 0; col < 3; ++col) {
    for (int row = 0; row < 3; ++row) {
      result(row, col) =
          left(row, 0) * right(0, col) +
          left(row, 1) * right(1, col) +
          left(row, 2) * right(2, col);
    }
  }
  return result;
}

__attribute__((noinline)) Eigen::Matrix3f fma_product(
    const Eigen::Matrix3f& left, const Eigen::Matrix3f& right) {
  Eigen::Matrix3f result;
  for (int col = 0; col < 3; ++col) {
    for (int row = 0; row < 3; ++row) {
      const float p0 = left(row, 0) * right(0, col);
      const float p1 = left(row, 1) * right(1, col);
      const float p2 = left(row, 2) * right(2, col);
      result(row, col) = std::fma(p0, 1.0f, std::fma(p1, 1.0f, p2));
    }
  }
  return result;
}

Eigen::Matrix3f fma_order(const Eigen::Matrix3f& left,
                          const Eigen::Matrix3f& right,
                          const int order[3]) {
  Eigen::Matrix3f result;
  for (int col = 0; col < 3; ++col) {
    for (int row = 0; row < 3; ++row) {
      const int k0 = order[0];
      const int k1 = order[1];
      const int k2 = order[2];
      float value = left(row, k0) * right(k0, col);
      value = std::fma(left(row, k1), right(k1, col), value);
      value = std::fma(left(row, k2), right(k2, col), value);
      result(row, col) = value;
    }
  }
  return result;
}

}  // namespace

int main() {
  Eigen::Matrix3f left;
  Eigen::Matrix3f right;
  const std::uint32_t left_bits[9] = {
      0x3f7ffffd, 0xb94c862d, 0xba0caa74, 0x394c40da, 0x3f7ffffe,
      0xb9fc5647, 0x3a0cb0bf, 0x39fc483d, 0x3f7ffffc};
  const std::uint32_t right_bits[9] = {
      0x3f7fffff, 0x38cc4c68, 0x398cafb3, 0xb8cc7aa0, 0x3f7fffff,
      0x397c4a95, 0xb98cab81, 0xb97c53f1, 0x3f7fffff};
  for (int i = 0; i < 9; ++i) {
    left.data()[i] = from_bits(left_bits[i]);
    right.data()[i] = from_bits(right_bits[i]);
  }
  print_matrix("eigen", eigen_product(left, right));
  print_matrix("scalar", scalar_product(left, right));
  print_matrix("fma", fma_product(left, right));
  const int orders[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 0, 2},
                            {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
  for (int i = 0; i < 6; ++i) {
    const std::string name = "order" + std::to_string(orders[i][0]) +
                             std::to_string(orders[i][1]) +
                             std::to_string(orders[i][2]);
    print_matrix(name.c_str(), fma_order(left, right, orders[i]));
  }
  return 0;
}
