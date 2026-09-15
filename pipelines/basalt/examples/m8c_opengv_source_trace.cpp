// Disposable source-order trace for OpenGV triangulate2.
#include <Eigen/Core>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>

#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <iostream>

static std::uint64_t bits(double value) {
  std::uint64_t out = 0;
  std::memcpy(&out, &value, sizeof(out));
  return out;
}

__attribute__((noinline)) Eigen::Vector3d m8c_traced_triangulate(
    opengv::relative_pose::CentralRelativeAdapter &adapter, std::size_t index) {
  const Eigen::Vector3d translation = adapter.gett12();
  const Eigen::Matrix3d rotation = adapter.getR12();
  const Eigen::Vector3d f1 = adapter.getBearingVector1(index);
  const Eigen::Vector3d f2 = adapter.getBearingVector2(index);
  const Eigen::Vector3d f2_unrotated = rotation * f2;
  Eigen::Vector2d b;
  b[0] = translation.dot(f1);
  b[1] = translation.dot(f2_unrotated);
  Eigen::Matrix2d A;
  A(0, 0) = f1.dot(f1);
  A(1, 0) = f1.dot(f2_unrotated);
  A(0, 1) = -A(1, 0);
  A(1, 1) = -f2_unrotated.dot(f2_unrotated);
  Eigen::Vector2d lambda = A.inverse() * b;
  Eigen::Vector3d xm = lambda[0] * f1;
  Eigen::Vector3d xn = translation + lambda[1] * f2_unrotated;
  Eigen::Vector3d point = (xm + xn) / 2.0;
  if (std::getenv("M8C_TRACE_SOURCE_TRI_CALL") != nullptr && index == 12) {
    const Eigen::Matrix2d inverse = A.inverse();
    std::cout << std::hex << "TRACE_SOURCE_TRI f1=" << bits(f1[0]) << ',' << bits(f1[1])
              << ',' << bits(f1[2]) << " f2=" << bits(f2[0]) << ',' << bits(f2[1]) << ','
              << bits(f2[2]) << " f2u=" << bits(f2_unrotated[0]) << ',' << bits(f2_unrotated[1])
              << ',' << bits(f2_unrotated[2]) << " a=" << bits(A(0, 0)) << ',' << bits(A(1, 0))
              << ',' << bits(A(0, 1)) << ',' << bits(A(1, 1)) << " b=" << bits(b[0]) << ','
              << bits(b[1]) << " inv=" << bits(inverse(0, 0)) << ',' << bits(inverse(0, 1))
              << ',' << bits(inverse(1, 0)) << ',' << bits(inverse(1, 1)) << " lambda="
              << bits(lambda[0]) << ','
              << bits(lambda[1]) << " xm=" << bits(xm[0]) << ',' << bits(xm[1]) << ','
              << bits(xm[2]) << " xn=" << bits(xn[0]) << ',' << bits(xn[1]) << ','
              << bits(xn[2]) << " point=" << bits(point[0]) << ',' << bits(point[1]) << ','
              << bits(point[2]) << std::dec << "\n";
  }
  return point;
}
