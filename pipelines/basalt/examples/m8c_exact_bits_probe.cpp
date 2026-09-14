// Disposable M8c Eigen/OpenGV scalar probe.
//
// This is intentionally outside the production mapper.  It consumes the raw
// M8c diagnostic JSON and prints exact f64 bit patterns for one fixed seeded
// pair, one fixed RANSAC model, all six NumericalDiff perturbations, and
// correspondence zero.  The matching Rust probe is
// `tests/m8c_exact_bits_probe.rs`.

#include <Eigen/Core>
#include <nlohmann/json.hpp>
#include <opengv/math/cayley.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/triangulation/methods.hpp>

#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

using json = nlohmann::json;
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

static constexpr std::uint64_t LEFT_FRAME = 1403636580113555456ULL;
static constexpr std::uint64_t RIGHT_FRAME = 1403636579763555584ULL;
static constexpr int LEFT_CAM = 0;
static constexpr int RIGHT_CAM = 1;

static std::uint64_t bits(double value) {
  std::uint64_t out = 0;
  static_assert(sizeof(out) == sizeof(value));
  std::memcpy(&out, &value, sizeof(out));
  return out;
}

static void scalar(const char *name, double value) {
  std::cout << name << "=0x" << std::hex << std::setw(16) << std::setfill('0')
            << bits(value) << std::dec << std::setfill(' ') << "\n";
}

static void vec(const std::string &name, const Vec3 &value) {
  for (int i = 0; i < 3; ++i) scalar((name + "[" + std::to_string(i) + "]").c_str(), value[i]);
}

static void mat(const std::string &name, const Mat3 &value) {
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      scalar((name + "[" + std::to_string(r) + "][" + std::to_string(c) + "]").c_str(), value(r, c));
}

// Eigen's AVX Packet4d reduction for a 3-vector loaded with a zero pad is
// (p0+p2)+p1, while a scalar loop is (p0+p1)+p2. Keep both spellings in
// this disposable probe so the first packet/scalar divergence is explicit.
static double dot3_scalar(const Vec3 &a, const Vec3 &b) {
  return (a[0] * b[0] + a[1] * b[1]) + a[2] * b[2];
}
static double dot3_eigen_unrolled(const Vec3 &a, const Vec3 &b) {
  return a[0] * b[0] + (a[1] * b[1] + a[2] * b[2]);
}
static double dot3_packet4d(const Vec3 &a, const Vec3 &b) {
  return (a[0] * b[0] + a[2] * b[2]) + a[1] * b[1];
}
static double dot3_fma_one(const Vec3 &a, const Vec3 &b, int fused) {
  const double p0 = a[0] * b[0];
  const double p1 = a[1] * b[1];
  const double p2 = a[2] * b[2];
  if (fused == 0) return std::fma(a[0], b[0], p1 + p2);
  if (fused == 1) return std::fma(a[1], b[1], p0 + p2);
  return std::fma(a[2], b[2], p0 + p1);
}
static void emit_dot3_variants(const std::string &prefix, const Vec3 &a, const Vec3 &b) {
  scalar((prefix + ".scalar").c_str(), dot3_scalar(a, b));
  scalar((prefix + ".eigen_unrolled").c_str(), dot3_eigen_unrolled(a, b));
  scalar((prefix + ".packet4d").c_str(), dot3_packet4d(a, b));
  scalar((prefix + ".eigen").c_str(), a.dot(b));
  for (int fused = 0; fused < 3; ++fused)
    scalar((prefix + ".fma_one" + std::to_string(fused)).c_str(), dot3_fma_one(a, b, fused));
}

static const json &feature(const json &raw, std::uint64_t frame, int cam) {
  for (const auto &item : raw.at("features")) {
    if (item.at("time_cam_id").at("frame_id") == frame &&
        item.at("time_cam_id").at("cam_id") == cam)
      return item;
  }
  throw std::runtime_error("feature not found");
}

static Vec3 ray(const json &feature_json, std::size_t index) {
  const auto &value = feature_json.at("corners_3d").at(index);
  return Vec3(value.at(0).get<double>(), value.at(1).get<double>(), value.at(2).get<double>());
}

static Mat3 model_rotation(const json &entry) {
  Mat3 value;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      value(r, c) = entry.at("ransac_model_rotation").at(r).at(c).get<double>();
  return value;
}

static Vec3 model_translation(const json &entry) {
  Vec3 value;
  for (int i = 0; i < 3; ++i)
    value[i] = entry.at("ransac_model_translation").at(i).get<double>();
  return value;
}

static void emit_residual(const std::string &prefix,
                          opengv::relative_pose::CentralRelativeAdapter &adapter,
                          const Vec3 &translation,
                          const Mat3 &rotation,
                          std::size_t index = 0) {
  adapter.sett12(translation);
  adapter.setR12(rotation);
  const Vec3 left = adapter.getBearingVector1(index);
  const Vec3 right = adapter.getBearingVector2(index);
  const Vec3 right_unrotated = rotation * right;
  emit_dot3_variants(prefix + ".b0_dot", translation, left);
  emit_dot3_variants(prefix + ".b1_dot", translation, right_unrotated);
  emit_dot3_variants(prefix + ".a00_dot", left, left);
  emit_dot3_variants(prefix + ".a10_dot", left, right_unrotated);
  emit_dot3_variants(prefix + ".a11_dot", right_unrotated, right_unrotated);
  const double a00 = left.dot(left);
  const double a10 = left.dot(right_unrotated);
  const double a01 = -a10;
  const double a11 = -right_unrotated.dot(right_unrotated);
  const double b0 = translation.dot(left);
  const double b1 = translation.dot(right_unrotated);
  Eigen::Matrix2d A;
  A(0, 0) = a00;
  A(1, 0) = a10;
  A(0, 1) = a01;
  A(1, 1) = a11;
  const double determinant = A.determinant();
  const double inverse_determinant = 1.0 / determinant;
  const Eigen::Matrix2d inverse = A.inverse();
  const Eigen::Vector2d b(b0, b1);
  const Eigen::Vector2d lambda = A.inverse() * b;
  const Vec3 xm = lambda[0] * left;
  const Vec3 xn = translation + lambda[1] * right_unrotated;
  const Vec3 point_direct =
      (lambda[0] * left + translation + lambda[1] * right_unrotated) / 2.0;
  const Vec3 point_nested = (xm + xn) / 2.0;
  const Vec3 point = opengv::triangulation::triangulate2(adapter, index);
  const Vec3 point_again = opengv::triangulation::triangulate2(adapter, index);
  const Mat3 inverse_rotation = rotation.transpose();
  const Vec3 inverse_translation = -inverse_rotation * translation;
  Eigen::Matrix<double, 3, 4> inverse_solution;
  inverse_solution.template block<3, 3>(0, 0) = inverse_rotation;
  inverse_solution.col(3) = inverse_translation;
  Eigen::Matrix<double, 4, 1> point_hom;
  point_hom.template head<3>() = point;
  point_hom[3] = 1.0;
  const Vec3 second_matrix = inverse_solution * point_hom;
  const Vec3 second_split = inverse_rotation * point + inverse_translation;
  // Keep candidate scalar spellings in the disposable trace.  Eigen's
  // inlined fixed 3x4 product can choose a different FMA operand placement
  // than the out-of-line assembly probe, and this makes that choice visible
  // without changing the production mapper.
  const double row2_group_a = std::fma(inverse_solution(2, 2), point_hom[2],
                                      inverse_solution(2, 3) * point_hom[3]);
  const double row2_group_b = std::fma(inverse_solution(2, 0), point_hom[0],
                                      inverse_solution(2, 1) * point_hom[1]);
  const double row2_groups = row2_group_a + row2_group_b;
  const double row2_group_a_rev = std::fma(inverse_solution(2, 3), point_hom[3],
                                           inverse_solution(2, 2) * point_hom[2]);
  const double row2_group_b_rev = std::fma(inverse_solution(2, 1), point_hom[1],
                                           inverse_solution(2, 0) * point_hom[0]);
  const double row2_groups_rev = row2_group_a_rev + row2_group_b_rev;
  const double row2_pair = (inverse_solution(2, 2) * point_hom[2] +
                            inverse_solution(2, 3) * point_hom[3]) +
                           (inverse_solution(2, 0) * point_hom[0] +
                            inverse_solution(2, 1) * point_hom[1]);
  const double row2_chain_0123 = std::fma(
      inverse_solution(2, 3), point_hom[3],
      std::fma(inverse_solution(2, 2), point_hom[2],
               std::fma(inverse_solution(2, 1), point_hom[1],
                        inverse_solution(2, 0) * point_hom[0])));
  const double row2_chain_3210 = std::fma(
      inverse_solution(2, 0), point_hom[0],
      std::fma(inverse_solution(2, 1), point_hom[1],
               std::fma(inverse_solution(2, 2), point_hom[2],
                        inverse_solution(2, 3) * point_hom[3])));
  const double row2_chain_mul =
      inverse_solution(2, 0) * point_hom[0] +
      inverse_solution(2, 1) * point_hom[1] +
      inverse_solution(2, 2) * point_hom[2] +
      inverse_solution(2, 3) * point_hom[3];
  const double first_norm = point.norm();
  const double second_norm_matrix = second_matrix.norm();
  const double second_norm_split = second_split.norm();
  const Vec3 first_unit = point / first_norm;
  const Vec3 second_unit_matrix = second_matrix / second_norm_matrix;
  const Vec3 second_unit_split = second_split / second_norm_split;
  const double left_dot_matrix = left.dot(first_unit);
  const double right_dot_matrix = right.dot(second_unit_matrix);
  const double left_dot_split = left.dot(first_unit);
  const double right_dot_split = right.dot(second_unit_split);
  const double residual_matrix = (1.0 - left_dot_matrix) + (1.0 - right_dot_matrix);
  const double residual_split = (1.0 - left_dot_split) + (1.0 - right_dot_split);
  const double residual_fma_terms = std::fma(-1.0, left_dot_matrix, 1.0) +
                                    std::fma(-1.0, right_dot_matrix, 1.0);
  const double residual_sum_dots = 2.0 - (left_dot_matrix + right_dot_matrix);
  const double residual_fma_sum = std::fma(-1.0, left_dot_matrix + right_dot_matrix, 2.0);
  const double residual_fma_cross = std::fma(-1.0, left_dot_matrix,
                                             1.0 - right_dot_matrix);

  vec(prefix + ".left_bearing", left);
  vec(prefix + ".right_bearing", right);
  vec(prefix + ".right_unrotated", right_unrotated);
  mat(prefix + ".R", rotation);
  vec(prefix + ".t", translation);
  scalar((prefix + ".A00").c_str(), a00);
  scalar((prefix + ".A10").c_str(), a10);
  scalar((prefix + ".A01").c_str(), a01);
  scalar((prefix + ".A11").c_str(), a11);
  scalar((prefix + ".b0").c_str(), b0);
  scalar((prefix + ".b1").c_str(), b1);
  scalar((prefix + ".det_manual").c_str(), a00 * a11 - a10 * a01);
  scalar((prefix + ".det_fma_right").c_str(),
         std::fma(-a10, a01, a00 * a11));
  scalar((prefix + ".det_fma_left").c_str(),
         std::fma(a00, a11, -(a10 * a01)));
  scalar((prefix + ".det_eigen").c_str(), determinant);
  scalar((prefix + ".invdet").c_str(), inverse_determinant);
  scalar((prefix + ".inv00").c_str(), inverse(0, 0));
  scalar((prefix + ".inv01").c_str(), inverse(0, 1));
  scalar((prefix + ".inv10").c_str(), inverse(1, 0));
  scalar((prefix + ".inv11").c_str(), inverse(1, 1));
  scalar((prefix + ".lambda0").c_str(), lambda[0]);
  scalar((prefix + ".lambda1").c_str(), lambda[1]);
  vec(prefix + ".xm", xm);
  vec(prefix + ".xn", xn);
  vec(prefix + ".point_opengv", point);
  vec(prefix + ".point_opengv_again", point_again);
  vec(prefix + ".point_manual", (xm + xn) / 2.0);
  vec(prefix + ".point_direct", point_direct);
  vec(prefix + ".point_nested", point_nested);
  mat(prefix + ".inverse_R", inverse_rotation);
  vec(prefix + ".inverse_t", inverse_translation);
  vec(prefix + ".second_matrix", second_matrix);
  scalar((prefix + ".row2_group_a").c_str(), row2_group_a);
  scalar((prefix + ".row2_group_b").c_str(), row2_group_b);
  scalar((prefix + ".row2_groups").c_str(), row2_groups);
  scalar((prefix + ".row2_group_a_rev").c_str(), row2_group_a_rev);
  scalar((prefix + ".row2_group_b_rev").c_str(), row2_group_b_rev);
  scalar((prefix + ".row2_groups_rev").c_str(), row2_groups_rev);
  scalar((prefix + ".row2_pair").c_str(), row2_pair);
  scalar((prefix + ".row2_chain_0123").c_str(), row2_chain_0123);
  scalar((prefix + ".row2_chain_3210").c_str(), row2_chain_3210);
  scalar((prefix + ".row2_chain_mul").c_str(), row2_chain_mul);
  vec(prefix + ".second_split", second_split);
  scalar((prefix + ".first_norm").c_str(), first_norm);
  scalar((prefix + ".second_norm_matrix").c_str(), second_norm_matrix);
  scalar((prefix + ".second_norm_split").c_str(), second_norm_split);
  vec(prefix + ".first_unit", first_unit);
  vec(prefix + ".second_unit_matrix", second_unit_matrix);
  vec(prefix + ".second_unit_split", second_unit_split);
  scalar((prefix + ".left_dot_matrix").c_str(), left_dot_matrix);
  scalar((prefix + ".right_dot_matrix").c_str(), right_dot_matrix);
  scalar((prefix + ".left_dot_split").c_str(), left_dot_split);
  scalar((prefix + ".right_dot_split").c_str(), right_dot_split);
  scalar((prefix + ".residual_matrix").c_str(), residual_matrix);
  scalar((prefix + ".residual_split").c_str(), residual_split);
  scalar((prefix + ".residual_fma_terms").c_str(), residual_fma_terms);
  scalar((prefix + ".residual_sum_dots").c_str(), residual_sum_dots);
  scalar((prefix + ".residual_fma_sum").c_str(), residual_fma_sum);
  scalar((prefix + ".residual_fma_cross").c_str(), residual_fma_cross);
  scalar((prefix + ".residual_opengv_shape").c_str(),
         (1.0 - (left.transpose() * first_unit).value()) +
             (1.0 - (right.transpose() * second_matrix / second_norm_matrix).value()));
}

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cerr << "usage: m8c_exact_bits_probe RAW_RANSAC_JSON ORACLE_JSON\n";
    return 2;
  }
  std::ifstream raw_file(argv[1]);
  std::ifstream oracle_file(argv[2]);
  json raw, oracle;
  raw_file >> raw;
  oracle_file >> oracle;
  const auto &left_feature = feature(raw, LEFT_FRAME, LEFT_CAM);
  const auto &right_feature = feature(raw, RIGHT_FRAME, RIGHT_CAM);
  for (const auto &entry : oracle.at("bow_query").at("seeded_temporal_oracle")) {
    if (entry.at("seed").get<unsigned>() != 7 ||
        entry.at("left").at("frame_id") != LEFT_FRAME ||
        entry.at("left").at("cam_id") != LEFT_CAM ||
        entry.at("right").at("frame_id") != RIGHT_FRAME ||
        entry.at("right").at("cam_id") != RIGHT_CAM)
      continue;
    opengv::bearingVectors_t left_bearings, right_bearings;
    for (const auto &pair : entry.at("ransac_inlier_ids")) {
      left_bearings.push_back(ray(left_feature, pair.at(0).get<std::size_t>()));
      right_bearings.push_back(ray(right_feature, pair.at(1).get<std::size_t>()));
    }
    opengv::relative_pose::CentralRelativeAdapter adapter(left_bearings, right_bearings);
    const Mat3 rotation = model_rotation(entry);
    const Vec3 translation = model_translation(entry);
    const Vec3 cayley = opengv::math::rot2cayley(rotation);
    const Mat3 rotation_from_cayley = opengv::math::cayley2rot(cayley);
    std::cout << "pair.seed=7\n";
    vec("x", Vec3(translation[0], translation[1], translation[2]));
    for (int i = 0; i < 3; ++i) scalar(("x[" + std::to_string(i + 3) + "]").c_str(), cayley[i]);
    mat("R_initial", rotation);
    mat("R_from_cayley", rotation_from_cayley);
    vec("t_initial", translation);
    for (int col = 0; col < 6; ++col) {
      double x_value = col < 3 ? translation[col] : cayley[col - 3];
      double h = std::sqrt(std::numeric_limits<double>::epsilon()) * std::abs(x_value);
      if (h == 0.0) h = std::sqrt(std::numeric_limits<double>::epsilon());
      scalar(("h[" + std::to_string(col) + "]").c_str(), h);
      scalar(("xh[" + std::to_string(col) + "]").c_str(), x_value + h);
    }
    emit_residual("x", adapter, translation, rotation_from_cayley);
    if (std::getenv("M8C_EXACT_BITS_ALL_ROWS") != nullptr) {
      const std::size_t count = entry.at("ransac_inlier_ids").size();
      for (std::size_t index = 1; index < count; ++index)
        emit_residual("x_row" + std::to_string(index), adapter, translation,
                      rotation_from_cayley, index);
    }
    Vec3 xh_translation = translation;
    Mat3 xh_rotation = rotation_from_cayley;
    // Column zero is the requested NumericalDiff perturbation.  Its x+h
    // state changes translation only, leaving the same rotation/cayley.
    double h0 = std::sqrt(std::numeric_limits<double>::epsilon()) * std::abs(translation[0]);
    if (h0 == 0.0) h0 = std::sqrt(std::numeric_limits<double>::epsilon());
    xh_translation[0] += h0;
    emit_residual("xh_col0", adapter, xh_translation, xh_rotation);
    if (std::getenv("M8C_EXACT_BITS_ALL_ROWS") != nullptr) {
      const std::size_t count = entry.at("ransac_inlier_ids").size();
      for (std::size_t index = 1; index < count; ++index)
        emit_residual("xh_col0_row" + std::to_string(index), adapter, xh_translation,
                      xh_rotation, index);
    }
    return 0;
  }
  throw std::runtime_error("seed-7 cam0->cam1 entry not found");
}
