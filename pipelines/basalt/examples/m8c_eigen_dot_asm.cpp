#include <Eigen/Core>
#include <Eigen/LU>
#include <cstdio>
#include <cstring>

static double from_bits(unsigned long long bits) {
  double value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

__attribute__((noinline)) double eigen_dot(const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
  return a.dot(b);
}

__attribute__((noinline)) double scalar_dot(const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
  return (a[0] * b[0] + a[1] * b[1]) + a[2] * b[2];
}

__attribute__((noinline)) double eigen_norm(const Eigen::Vector3d &a) {
  return a.norm();
}

__attribute__((noinline)) Eigen::Vector3d eigen_matvec(const Eigen::Matrix3d &m,
                                                        const Eigen::Vector3d &v) {
  return m * v;
}

__attribute__((noinline)) Eigen::Vector3d eigen_mat34vec4(const Eigen::Matrix<double, 3, 4> &m,
                                                          const Eigen::Vector4d &v) {
  return m * v;
}

__attribute__((noinline)) Eigen::Vector2d eigen_mat2vec2(const Eigen::Matrix2d &m,
                                                         const Eigen::Vector2d &v) {
  return m * v;
}

__attribute__((noinline)) Eigen::Vector2d eigen_inverse_lambda(double a00, double a10,
                                                                double a11, double b0,
                                                                double b1) {
  Eigen::Matrix2d a;
  a(0, 0) = a00;
  a(1, 0) = a10;
  a(0, 1) = -a10;
  a(1, 1) = a11;
  return a.inverse() * Eigen::Vector2d(b0, b1);
}

__attribute__((noinline)) Eigen::Vector3d eigen_triangulate(const Eigen::Vector3d &f1,
                                                            const Eigen::Vector3d &f2u,
                                                            const Eigen::Vector3d &t) {
  Eigen::Vector2d b;
  b[0] = t.dot(f1);
  b[1] = t.dot(f2u);
  Eigen::Matrix2d a;
  a(0, 0) = f1.dot(f1);
  a(1, 0) = f1.dot(f2u);
  a(0, 1) = -a(1, 0);
  a(1, 1) = -f2u.dot(f2u);
  Eigen::Vector2d lambda = a.inverse() * b;
  Eigen::Vector3d xm = lambda[0] * f1;
  Eigen::Vector3d xn = t + lambda[1] * f2u;
  return (xm + xn) / 2.0;
}

static void print_bits(const char *label, double value) {
  unsigned long long bits;
  std::memcpy(&bits, &value, sizeof(bits));
  std::printf("%s=%016llx\n", label,
              static_cast<unsigned long long>(bits));
}

int main() {
  Eigen::Vector3d a(0.1, -0.2, 0.3), b(-0.4, 0.5, -0.6);
  const auto lambda = eigen_inverse_lambda(
      from_bits(0x3feffffffffffffdULL),
      from_bits(0x3feff134506aad60ULL),
      from_bits(0xbff0000000000000ULL),
      from_bits(0xbfb9aa083905bf1bULL),
      from_bits(0xbfc230fab61a86ceULL));
  print_bits("lambda0", lambda[0]);
  print_bits("lambda1", lambda[1]);
  Eigen::Vector3d f1;
  f1[0] = from_bits(0xbfe0900570f4d798ULL);
  f1[1] = from_bits(0xbfd4dd3a2ddc3fb4ULL);
  f1[2] = from_bits(0x3fe950a805def931ULL);
  Eigen::Vector3d f2u;
  f2u[0] = from_bits(0xbfe0ded0d151469dULL);
  f2u[1] = from_bits(0xbfd131184c46332cULL);
  f2u[2] = from_bits(0x3fe9cc1c7d4ec893ULL);
  Eigen::Vector3d t;
  t[0] = from_bits(0x3fc6617d51fd13c1ULL);
  t[1] = from_bits(0xbfe421157543e986ULL);
  t[2] = from_bits(0xbfd16107da8a5fd0ULL);
  const auto point = eigen_triangulate(f1, f2u, t);
  print_bits("point0", point[0]);
  print_bits("point1", point[1]);
  print_bits("point2", point[2]);
  const Eigen::Vector3d target_point{from_bits(0xc017df156b19ad5bULL),
                                     from_bits(0xc00e123c8fc4768dULL),
                                     from_bits(0x40223e3d6bc1a0fbULL)};
  const long double rhs0 = 2.0L * target_point[0] - static_cast<long double>(t[0]);
  const long double rhs1 = 2.0L * target_point[1] - static_cast<long double>(t[1]);
  const long double determinant = static_cast<long double>(f1[0]) * f2u[1]
                               - static_cast<long double>(f1[1]) * f2u[0];
  const long double recovered0 = (rhs0 * f2u[1] - rhs1 * f2u[0]) / determinant;
  const long double recovered1 = (f1[0] * rhs1 - f1[1] * rhs0) / determinant;
  print_bits("recovered0", static_cast<double>(recovered0));
  print_bits("recovered1", static_cast<double>(recovered1));
  for (long long delta0 = -32; delta0 <= 32; ++delta0) {
    for (long long delta1 = -32; delta1 <= 32; ++delta1) {
      const double l0 = from_bits(0x40270f820167031cULL + delta0);
      const double l1 = from_bits(0x40274d9c5a75ac87ULL + delta1);
      const Eigen::Vector3d candidate_xm = l0 * f1;
      const Eigen::Vector3d candidate_xn = t + l1 * f2u;
      const Eigen::Vector3d candidate_point = (candidate_xm + candidate_xn) / 2.0;
      if (candidate_point[0] == target_point[0] && candidate_point[1] == target_point[1]
          && candidate_point[2] == target_point[2]) {
        std::printf("target_delta=%lld,%lld\n", delta0, delta1);
      }
    }
  }
  return eigen_dot(a, b) == scalar_dot(a, b) ? 0 : 1;
}
