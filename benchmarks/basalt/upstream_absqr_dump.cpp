// Pinned Basalt M8a/ABS_QR oracle for one serialized MargData packet.
// Build/run from WSL with the pinned checkout's Eigen/cereal include paths.
#include <basalt/serialization/headers_serialization.h>
#include <basalt/utils/imu_types.h>
#include <basalt/vi_estimator/marg_helper.h>
#include <cereal/archives/binary.hpp>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>

namespace {

template <class Derived>
void json_vector(std::ostream& out, const Eigen::MatrixBase<Derived>& vector) {
  out << '[';
  for (Eigen::Index index = 0; index < vector.size(); ++index) {
    if (index != 0) out << ',';
    out << vector(index);
  }
  out << ']';
}

void json_matrix(std::ostream& out, const Eigen::MatrixXd& matrix) {
  out << "{\"rows\":" << matrix.rows() << ",\"cols\":" << matrix.cols()
      << ",\"data\":[";
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    for (Eigen::Index col = 0; col < matrix.cols(); ++col) {
      if (row != 0 || col != 0) out << ',';
      out << matrix(row, col);
    }
  }
  out << "]}";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: upstream_absqr_dump FILE.cereal\n";
    return 2;
  }
  std::ifstream input(argv[1], std::ios::binary);
  if (!input) return 3;

  basalt::MargData data;
  cereal::BinaryInputArchive archive(input);
  archive(data);

  std::set<int> keep;
  std::set<int> marg;
  int keep_blocks = 0;
  int marg_blocks = 0;
  for (const auto& [timestamp, block] : data.aom.abs_order_map) {
    if (block.second == basalt::POSE_SIZE) {
      ++keep_blocks;
      for (int i = 0; i < basalt::POSE_SIZE; ++i) keep.insert(block.first + i);
    } else if (data.kfs_all.count(timestamp) > 0) {
      ++keep_blocks;
      for (int i = 0; i < basalt::POSE_SIZE; ++i) keep.insert(block.first + i);
      for (int i = basalt::POSE_SIZE; i < basalt::POSE_VEL_BIAS_SIZE; ++i)
        marg.insert(block.first + i);
    } else {
      ++marg_blocks;
      for (int i = 0; i < basalt::POSE_VEL_BIAS_SIZE; ++i)
        marg.insert(block.first + i);
    }
  }

  std::cerr << "input=" << data.abs_H.rows() << "x" << data.abs_H.cols()
            << " keep=" << keep.size() << " marg=" << marg.size() << '\n';
  const auto input_rows = data.abs_H.rows();
  const auto input_cols = data.abs_H.cols();
  Eigen::FullPivHouseholderQR<Eigen::MatrixXd> qr_before(data.abs_H);
  const auto input_rank = qr_before.rank();

  Eigen::MatrixXd result_h;
  Eigen::VectorXd result_b;
  basalt::MargHelper<double>::marginalizeHelperSqToSq(
      data.abs_H, data.abs_b, keep, marg, result_h, result_b);
  std::cerr << "marginalized=" << result_h.rows() << "x" << result_h.cols()
            << '\n';

  Eigen::FullPivHouseholderQR<Eigen::MatrixXd> qr_after(result_h);
  std::cout << std::setprecision(17);
  std::cout << "{\"source\":\"pinned-basalt-0f3b2b52\","
            << "\"input_shape\":[" << input_rows << ',' << input_cols
            << "],\"input_rank\":" << input_rank
            << ",\"kept_columns\":[";
  bool first = true;
  for (int index : keep) {
    if (!first) std::cout << ',';
    first = false;
    std::cout << index;
  }
  std::cout << "],\"marginalized_columns\":[";
  first = true;
  for (int index : marg) {
    if (!first) std::cout << ',';
    first = false;
    std::cout << index;
  }
  std::cout << "],\"kept_blocks\":" << keep_blocks
            << ",\"marginalized_blocks\":" << marg_blocks
            << ",\"output_shape\":[" << result_h.rows() << ','
            << result_h.cols() << "],\"output_rank\":" << qr_after.rank()
            << ",\"result_abs_h\":";
  json_matrix(std::cout, result_h);
  std::cout << ",\"result_abs_b\":";
  json_vector(std::cout, result_b);
  std::cout << "}\n";
}
