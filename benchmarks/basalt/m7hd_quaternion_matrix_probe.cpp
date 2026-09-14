// Exact-bit Eigen quaternion-to-matrix schedule probe for the packet-1 q.

#include <Eigen/Geometry>
#include <sophus/so3.hpp>

#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>

namespace {

__attribute__((noinline)) Eigen::Matrix3f eigen_matrix(
    const Eigen::Quaternion<float>& q) {
  return q.toRotationMatrix();
}

__attribute__((noinline)) Eigen::Matrix3f manual_matrix(
    const Eigen::Quaternion<float>& q) {
  const float tx = 2.0f * q.x();
  const float ty = 2.0f * q.y();
  const float tz = 2.0f * q.z();
  const float twx = tx * q.w();
  const float twy = ty * q.w();
  const float twz = tz * q.w();
  const float txx = tx * q.x();
  const float txy = ty * q.x();
  const float txz = tz * q.x();
  const float tyy = ty * q.y();
  const float tyz = ty * q.z();
  const float tzz = tz * q.z();
  Eigen::Matrix3f result;
  result.coeffRef(0, 0) = 1.0f - (tyy + tzz);
  result.coeffRef(0, 1) = txy - twz;
  result.coeffRef(0, 2) = txz + twy;
  result.coeffRef(1, 0) = txy + twz;
  result.coeffRef(1, 1) = 1.0f - (txx + tzz);
  result.coeffRef(1, 2) = tyz - twx;
  result.coeffRef(2, 0) = txz - twy;
  result.coeffRef(2, 1) = tyz + twx;
  result.coeffRef(2, 2) = 1.0f - (txx + tyy);
  return result;
}

float from_bits(std::uint32_t bits) {
  float value = 0;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

std::uint32_t bits(float value) {
  std::uint32_t result = 0;
  std::memcpy(&result, &value, sizeof(result));
  return result;
}

template <typename Matrix>
void print_matrix(const char* name, const Matrix& matrix) {
  std::cout << name << '=';
  for (Eigen::Index i = 0; i < matrix.size(); ++i) {
    if (i) std::cout << ',';
    std::cout << std::hex << std::setw(8) << std::setfill('0')
              << bits(matrix.derived().coeff(i));
  }
  std::cout << std::dec << '\n';
}

}  // namespace

int main() {
  Eigen::Quaternion<float> q(from_bits(0x3f7fffff), from_bits(0xb97c4f43),
                             from_bits(0x398cad9a), from_bits(0xb8cc6384));
  print_matrix("eigen", eigen_matrix(q));
  print_matrix("manual_fn", manual_matrix(q));

  Sophus::SO3<float> sophus(q);
  print_matrix("sophus", sophus.matrix());
  return 0;
}
