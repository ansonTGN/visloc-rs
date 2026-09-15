// Isolated Eigen SqrtToSqrt reflector/update probe for M7IM15.
//
// The input is an M7IM15MARG1 capture emitted from an exact Q2/J and RHS
// boundary.  This executable intentionally performs only the pinned Eigen
// permutation, makeHouseholderInPlace, applyHouseholderOnTheLeft, and
// essential-vector clear.  It writes every post-reflector Q2/RHS state so the
// Rust packet emulation can be compared without running the estimator.  The
// dimensions are deliberately generic: the same qtrace format is used for
// the 96x60 first event and the 36x36 second event.

#define main upstream_oracle_main
#include "upstream_sqrt_marginalization_oracle.cpp"
#undef main

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>

namespace {

template <typename T>
void write_binary(std::ofstream& output, const T& value) {
  output.write(reinterpret_cast<const char*>(&value), sizeof(value));
}

void write_f32_bits(std::ofstream& output, float value) {
  const uint32_t value_bits = bits(value);
  write_binary(output, value_bits);
}

void write_matrix_bits(std::ofstream& output, const Matrix& matrix) {
  for (Eigen::Index index = 0; index < matrix.size(); ++index)
    write_f32_bits(output, matrix.data()[index]);
}

void write_vector_bits(std::ofstream& output, const Vector& vector) {
  for (Eigen::Index index = 0; index < vector.size(); ++index)
    write_f32_bits(output, vector[index]);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: m7im15_sqrt_apply_probe <MARG1> <trace.bin>\n";
    return 2;
  }

  Matrix qj;
  Vector qr;
  std::set<int> keep;
  std::set<int> marg;
  if (!read_marg_binary(argv[1], qj, qr, keep, marg)) {
    std::cerr << "invalid M7IM15 MARG capture: " << argv[1] << '\n';
    return 3;
  }
  if (qj.rows() <= 0 || qj.cols() <= 0 || qr.size() != qj.rows() ||
      keep.size() + marg.size() != static_cast<size_t>(qj.cols())) {
    std::cerr << "unexpected dimensions rows=" << qj.rows()
              << " cols=" << qj.cols() << " rhs=" << qr.size()
              << " keep=" << keep.size() << " marg=" << marg.size() << '\n';
    return 4;
  }

  Eigen::Matrix<int, Eigen::Dynamic, 1> indices(qj.cols());
  auto it = marg.begin();
  for (Eigen::Index index = 0; index < static_cast<Eigen::Index>(marg.size());
       ++index, ++it)
    indices[index] = *it;
  it = keep.begin();
  for (Eigen::Index index = 0; index < static_cast<Eigen::Index>(keep.size());
       ++index, ++it)
    indices[marg.size() + index] = *it;
  qj.applyOnTheRight(
      Eigen::PermutationWrapper<Eigen::Matrix<int, Eigen::Dynamic, 1>>(
          indices));

  std::ofstream dump(argv[2], std::ios::binary | std::ios::trunc);
  if (!dump) {
    std::cerr << "cannot open trace output: " << argv[2] << '\n';
    return 5;
  }
  const char magic[] = "M7IM15QTRACE1";
  dump.write(magic, sizeof(magic) - 1);
  const uint32_t version = 1;
  const uint32_t rows = static_cast<uint32_t>(qj.rows());
  const uint32_t cols = static_cast<uint32_t>(qj.cols());
  const uint32_t rhs_size = static_cast<uint32_t>(qr.size());
  const uint32_t marg_size = static_cast<uint32_t>(marg.size());
  const uint32_t keep_size = static_cast<uint32_t>(keep.size());
  write_binary(dump, version);
  write_binary(dump, rows);
  write_binary(dump, cols);
  write_binary(dump, rhs_size);
  write_binary(dump, marg_size);
  write_binary(dump, keep_size);

  std::cout << "trace_input " << qj.rows() << ' ' << qj.cols() << ' '
            << qr.size() << '\n';
  std::cout << "trace_input_hash " << std::hex << matrix_bits_hash(qj) << ' '
            << raw_bits_hash(qr.data(), qr.size()) << std::dec << '\n';

  Eigen::Index total_rank = 0;
  const Eigen::Index rows_eigen = qj.rows();
  const Eigen::Index cols_eigen = qj.cols();
  Vector temp;
  temp.resize(cols_eigen + 1);
  float* temp_data = temp.data();
  const bool focus_k12 = std::getenv("M7IM15_EIGEN_FOCUS_K12") != nullptr;
  const bool dump_workspace_all =
      std::getenv("M7IM15_EIGEN_DUMP_WORKSPACE_ALL") != nullptr;
  const float rank_threshold =
      std::sqrt(std::numeric_limits<float>::epsilon());

  for (Eigen::Index k = 0; k < cols_eigen && total_rank < rows_eigen; ++k) {
    const Eigen::Index row_start = total_rank;
    const Eigen::Index remaining_rows = rows_eigen - total_rank;
    const Eigen::Index remaining_cols = cols_eigen - k - 1;
    const float c0_before = qj.coeff(row_start, k);
    const float full_eigen_norm = qj.col(k).tail(remaining_rows).squaredNorm();
    const float tail_sq_norm =
        qj.col(k).tail(remaining_rows - 1).squaredNorm();
    float h_coeff = 0.0f;
    float beta = 0.0f;
    qj.col(k).tail(remaining_rows).makeHouseholderInPlace(h_coeff, beta);
    const bool accepted = std::abs(beta) > rank_threshold;
    const uint32_t k_u32 = static_cast<uint32_t>(k);
    const uint32_t rank_before = static_cast<uint32_t>(total_rank);
    std::cout << "trace_reflector " << k << ' ' << total_rank << ' '
              << std::hex << bits(c0_before) << ' ' << bits(tail_sq_norm)
              << ' ' << bits(beta) << ' ' << bits(h_coeff) << std::dec << ' '
              << (accepted ? 1 : 0) << '\n';
    const float full_sq_norm = c0_before * c0_before + tail_sq_norm;
    std::cout << "trace_full_norm " << k << ' ' << std::hex
              << bits(full_sq_norm) << std::dec << '\n';
    std::cout << "trace_full_eigen_norm " << k << ' ' << std::hex
              << bits(full_eigen_norm) << std::dec << '\n';

    if (accepted) {
      qj.coeffRef(total_rank, k) = beta;
      auto derived = qj.bottomRightCorner(remaining_rows, remaining_cols);
      const auto bottom = derived.block(1, 0, remaining_rows - 1,
                                        remaining_cols);
      const auto essential = qj.col(k).tail(remaining_rows - 1);
      const auto trans = bottom.transpose();
      (void)trans;
      Eigen::Map<Eigen::RowVectorXf> workspace(temp.data() + k + 1,
                                                remaining_cols);
      if (focus_k12 && k == 15) {
        std::cout << "focus_begin " << k << ' ' << row_start << ' '
                  << remaining_rows << ' ' << remaining_cols << ' '
                  << std::hex << bits(h_coeff) << ' ' << bits(beta)
                  << std::dec << '\n';
        std::cout << "focus_essential";
        for (Eigen::Index index = 0; index < essential.size(); ++index)
          std::cout << ' ' << std::hex << bits(essential[index]);
        std::cout << std::dec << '\n';
        for (Eigen::Index col = 0; col < remaining_cols; ++col) {
          std::cout << "focus_bottom " << col << ' ' << (k + 1 + col);
          for (Eigen::Index row = 0; row < remaining_rows - 1; ++row)
            std::cout << ' ' << std::hex << bits(bottom(row, col));
          std::cout << std::dec << '\n';
        }
      }
      workspace.noalias() = essential.adjoint() * bottom;
      if (dump_workspace_all) {
        std::cout << "trace_workspace_all " << k << ' ' << row_start << ' '
                  << remaining_rows << ' ' << remaining_cols;
        for (Eigen::Index col = 0; col < remaining_cols; ++col)
          std::cout << ' ' << std::hex << bits(workspace[col]);
        std::cout << std::dec << '\n';
      }
      if (focus_k12 && k == 15) {
        std::cout << "focus_workspace_raw";
        for (Eigen::Index col = 0; col < remaining_cols; ++col)
          std::cout << ' ' << std::hex << bits(workspace[col]);
        std::cout << std::dec << '\n';
        std::cout << "focus_row0";
        for (Eigen::Index col = 0; col < remaining_cols; ++col)
          std::cout << ' ' << std::hex << bits(derived(0, col));
        std::cout << std::dec << '\n';
        std::cout << "focus_workspace_plus_row0";
        for (Eigen::Index col = 0; col < remaining_cols; ++col)
          std::cout << ' ' << std::hex << bits(workspace[col] + derived(0, col));
        std::cout << std::dec << '\n';
        std::cout << "focus_scaled_workspace";
        for (Eigen::Index col = 0; col < remaining_cols; ++col)
          std::cout << ' ' << std::hex
                    << bits(h_coeff * (workspace[col] + derived(0, col)));
        std::cout << std::dec << '\n';
      }
      derived.applyHouseholderOnTheLeft(essential, h_coeff,
                                        temp.data() + k + 1);
      if (focus_k12 && k == 15) {
        for (Eigen::Index col = 0; col < remaining_cols; ++col) {
          std::cout << "focus_post " << col << ' ' << (k + 1 + col);
          for (Eigen::Index row = 0; row < remaining_rows; ++row)
            std::cout << ' ' << std::hex << bits(derived(row, col));
          std::cout << std::dec << '\n';
        }
      }
      qr.tail(remaining_rows).applyHouseholderOnTheLeft(
          essential, h_coeff, temp.data() + cols_eigen);
      ++total_rank;
    } else {
      qj.coeffRef(total_rank, k) = 0.0f;
    }
    qj.col(k).tail(remaining_rows - 1).setZero();

    const uint32_t rank_after = static_cast<uint32_t>(total_rank);
    write_binary(dump, k_u32);
    write_binary(dump, rank_before);
    write_binary(dump, rank_after);
    write_f32_bits(dump, c0_before);
    write_f32_bits(dump, tail_sq_norm);
    write_f32_bits(dump, beta);
    write_f32_bits(dump, h_coeff);
    write_matrix_bits(dump, qj);
    write_vector_bits(dump, qr);

    std::cout << "trace_after " << k << ' ' << total_rank << ' '
              << std::hex << matrix_bits_hash(qj) << ' '
              << raw_bits_hash(qr.data(), qr.size()) << std::dec << '\n';
  }
  return dump.good() ? 0 : 6;
}
