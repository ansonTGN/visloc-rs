// Deterministic oracle for Basalt's SqrtToSqrt marginalization boundary.
//
// This is a standalone Eigen oracle for the pinned upstream call sequence. It
// does not link libbasalt (doing so across separately compiled Eigen objects
// caused an ABI/alignment failure); instead it uses the same Eigen primitive
// calls and MargHelper ordering/rank policy. The reference implementation is
// Basalt 0f3b2b52.../src/vi_estimator/marg_helper.cpp, instantiated as float.
// Build/run (inside the pinned WSL checkout):
//
//   c++ -std=c++17 -O2 -DEIGEN_DONT_PARALLELIZE \
//     -isystem /root/visloc-basalt-oracle-0f3b2b52/thirdparty/vcpkg/buildtrees/eigen3/src/5.0.1-d487a628b0.clean \
//     -o /tmp/upstream_sqrt_marginalization_oracle \
//     benchmarks/basalt/upstream_sqrt_marginalization_oracle.cpp
//
// Add -DEIGEN_DONT_VECTORIZE to that command for the scalar-operation control
// used by the Rust f32 golden tests.  The default build retains Eigen's
// vectorized packet reductions and is the pinned-runtime reference; its bits
// can differ from the scalar control even when rank and output shape agree.
//
// The output is deliberately plain text so it can be copied into a small
// Rust golden test without adding a runtime dependency on the upstream tree.

#include <Eigen/Dense>

#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace {

using Matrix = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;
using Vector = Eigen::Matrix<float, Eigen::Dynamic, 1>;

uint32_t bits(float value) {
  uint32_t out;
  std::memcpy(&out, &value, sizeof(out));
  return out;
}

struct Case {
  const char* name;
  Matrix jacobian;
  Vector rhs;
  std::set<int> keep;
  std::set<int> marg;
};

Case normal_case() {
  Matrix j(5, 4);
  j << 0.75f, -1.2f, 0.25f, -0.7f,
      -0.35f, 0.8f, 1.3f, 0.2f,
      1.1f, 0.15f, -0.6f, 0.9f,
      -0.45f, -0.55f, 0.7f, -1.4f,
      0.2f, 1.05f, -0.9f, 0.35f;
  Vector r(5);
  r << 0.4f, -1.2f, 0.7f, 2.1f, -0.3f;
  return {"normal", j, r, {0, 3}, {1, 2}};
}

Case near_rank_case() {
  Matrix j(4, 3);
  // The marginal column's norm is below sqrt(float epsilon), while the kept
  // columns remain well-conditioned.  This exercises Basalt's rank drop.
  j << 1.0e-4f, 0.4f, -0.7f,
      2.0e-4f, -0.2f, 0.9f,
      -1.0e-4f, 0.8f, 0.1f,
      0.0f, -0.5f, 0.3f;
  Vector r(4);
  r << 0.25f, -0.5f, 0.75f, -1.0f;
  return {"near_rank", j, r, {0, 2}, {1}};
}

Case sign_case() {
  Matrix j(4, 2);
  // A negative leading marginal pivot makes Eigen choose beta=+||x||.
  j << -0.8f, 0.3f,
      0.25f, -1.1f,
      -0.45f, 0.6f,
      0.7f, 0.2f;
  Vector r(4);
  r << 0.6f, -0.4f, 1.2f, 0.1f;
  return {"sign", j, r, {1}, {0}};
}

Case zero_case() {
  Matrix j(3, 2);
  j << 0.0f, 0.5f,
      0.0f, -1.0f,
      0.0f, 0.25f;
  Vector r(3);
  r << 0.2f, -0.3f, 0.4f;
  return {"zero", j, r, {1}, {0}};
}

Case denormal_case() {
  Matrix j(3, 2);
  const float d = std::numeric_limits<float>::denorm_min();
  j << d, 0.5f,
      0.0f, -1.0f,
      0.0f, 0.25f;
  Vector r(3);
  r << 0.2f, -0.3f, 0.4f;
  return {"denormal", j, r, {1}, {0}};
}

void print_vector(const char* label, const Vector& v) {
  std::cout << label << " " << v.size() << "\n";
  std::cout << std::setprecision(9);
  for (Eigen::Index i = 0; i < v.size(); ++i)
    std::cout << v[i] << " " << bits(v[i]) << "\n";
}

void print_matrix(const char* label, const Matrix& m) {
  std::cout << label << " " << m.rows() << " " << m.cols() << "\n";
  std::cout << std::setprecision(9);
  for (Eigen::Index row = 0; row < m.rows(); ++row) {
    for (Eigen::Index col = 0; col < m.cols(); ++col)
      std::cout << m(row, col) << " " << bits(m(row, col)) << "\n";
  }
}

void print_set(const char* label, const std::set<int>& values) {
  std::cout << label << " " << values.size();
  for (int value : values) std::cout << " " << value;
  std::cout << "\n";
}

// The pinned source delegates the numerical primitive to Eigen and only
// supplies this ordering/rank policy.  Keep this oracle local to avoid an
// Eigen-ABI dependency on the large upstream shared library; the operations
// below are the same calls made by MargHelper<float> at the pinned SHA.
void marginalize_sqrt_upstream(Matrix& qj, Vector& qr,
                               const std::set<int>& keep,
                               const std::set<int>& marg, Matrix& out_j,
                               Vector& out_r) {
  const Eigen::Index keep_size = static_cast<Eigen::Index>(keep.size());
  const Eigen::Index marg_size = static_cast<Eigen::Index>(marg.size());
  Eigen::Matrix<int, Eigen::Dynamic, 1> indices(keep_size + marg_size);

  auto it = marg.begin();
  for (Eigen::Index i = 0; i < marg_size; ++i, ++it) indices[i] = *it;
  it = keep.begin();
  for (Eigen::Index i = 0; i < keep_size; ++i, ++it)
    indices[marg_size + i] = *it;
  qj.applyOnTheRight(
      Eigen::PermutationWrapper<Eigen::Matrix<int, Eigen::Dynamic, 1>>(
          indices));

  Eigen::Index marg_rank = 0;
  Eigen::Index total_rank = 0;
  const float rank_threshold = std::sqrt(std::numeric_limits<float>::epsilon());
  const Eigen::Index rows = qj.rows();
  const Eigen::Index cols = qj.cols();
  Vector temp;
  temp.resize(cols + 1);
  float* temp_data = temp.data();

  for (Eigen::Index k = 0; k < cols && total_rank < rows; ++k) {
    const Eigen::Index remaining_rows = rows - total_rank;
    const Eigen::Index remaining_cols = cols - k - 1;
    float beta;
    float h_coeff;
    qj.col(k).tail(remaining_rows).makeHouseholderInPlace(h_coeff, beta);
    if (std::abs(beta) > rank_threshold) {
      std::vector<float> stage_col14_before;
      if (k == 9) {
        stage_col14_before.assign(qj.data() + (k + 1 + 14) * rows,
                                  qj.data() + (k + 2 + 14) * rows);
        std::cout << "trace_stage9_col14_pre";
        for (float value : stage_col14_before)
          std::cout << ' ' << std::hex << bits(value);
        std::cout << std::dec << '\n';
      }
      qj.coeffRef(total_rank, k) = beta;
      qj.bottomRightCorner(remaining_rows, remaining_cols)
          .applyHouseholderOnTheLeft(
              qj.col(k).tail(remaining_rows - 1), h_coeff,
              temp_data + k + 1);
      std::cout << "trace_workspace " << k;
      for (Eigen::Index index = 0;
           index < std::min<Eigen::Index>(remaining_cols, 8); ++index) {
        std::cout << ' ' << std::hex << bits(temp_data[k + 1 + index]);
      }
      std::cout << std::dec << '\n';
      if (k == 9) {
        std::cout << "trace_workspace_full " << k;
        for (Eigen::Index index = 0; index < remaining_cols; ++index) {
          std::cout << ' ' << std::hex << bits(temp_data[k + 1 + index]);
        }
        std::cout << std::dec << '\n';
        std::cout << "trace_stage9_operands " << std::hex
                  << bits(temp_data[k + 1 + 14]) << ' '
                  << bits(qj.coeff(total_rank + 1 + 3, k)) << ' '
                  << bits(qj.coeff(total_rank + 1 + 4, k)) << ' '
                  << bits(qj.coeff(total_rank + 1 + 3, k)) << ' '
                  << bits(qj.coeff(total_rank + 1 + 4, k)) << std::dec
                  << '\n';
      }
      if (k == 9) {
        std::cout << "trace_stage9_col14_post";
        for (Eigen::Index index = 0; index < rows; ++index)
          std::cout << ' ' << std::hex << bits(qj.coeff(index, k + 1 + 14));
        std::cout << std::dec << '\n';
      }
      qr.tail(remaining_rows).applyHouseholderOnTheLeft(
          qj.col(k).tail(remaining_rows - 1), h_coeff, temp_data + cols);
      ++total_rank;
    } else {
      qj.coeffRef(total_rank, k) = 0;
    }
    qj.col(k).tail(remaining_rows - 1).setZero();
    if (k == marg_size - 1) marg_rank = total_rank;
  }

  const Eigen::Index keep_valid_rows =
      std::max(total_rank - marg_rank, Eigen::Index(1));
  out_j = qj.block(marg_rank, marg_size, keep_valid_rows, keep_size);
  out_r = qr.segment(marg_rank, keep_valid_rows);
  qj.resize(0, 0);
  qr.resize(0);
}

void run_case(const Case& input) {
  Matrix j = input.jacobian;
  Vector r = input.rhs;
  Matrix out_j;
  Vector out_r;
  marginalize_sqrt_upstream(j, r, input.keep, input.marg, out_j, out_r);

  std::cout << "case " << input.name << "\n";
  print_set("keep", input.keep);
  print_set("marg", input.marg);
  print_matrix("input_j", input.jacobian);
  print_vector("input_r", input.rhs);
  print_matrix("output_j", out_j);
  print_vector("output_r", out_r);
}

bool read_fixture(const char* path, Matrix& jacobian, Vector& rhs,
                  std::set<int>& keep, std::set<int>& marg) {
  std::ifstream input(path);
  std::string magic;
  if (!(input >> magic) || magic != "M6_FIXTURE_V1") return false;
  Eigen::Index rows;
  Eigen::Index cols;
  input >> rows >> cols;
  int marg_count;
  input >> marg_count;
  for (int i = 0; i < marg_count; ++i) {
    int index;
    input >> index;
    marg.insert(index);
  }
  int keep_count;
  input >> keep_count;
  for (int i = 0; i < keep_count; ++i) {
    int index;
    input >> index;
    keep.insert(index);
  }
  int selected_rows;
  input >> selected_rows;
  for (int i = 0; i < selected_rows; ++i) {
    int ignored_row_index;
    input >> ignored_row_index;
  }
  jacobian.resize(rows, cols);
  for (Eigen::Index row = 0; row < rows; ++row) {
    for (Eigen::Index col = 0; col < cols; ++col) input >> jacobian(row, col);
  }
  rhs.resize(rows);
  for (Eigen::Index row = 0; row < rows; ++row) input >> rhs[row];
  return input.good() || input.eof();
}

// Audit-only reader for the detached M7IM15 native boundary capture.  The
// production logger writes Eigen's column-major float matrix verbatim, which
// lets this oracle report the exact reflector schedule without linking the
// full VIO binary.
template <typename T>
bool read_binary(std::ifstream& input, T& value) {
  input.read(reinterpret_cast<char*>(&value), sizeof(value));
  return static_cast<bool>(input);
}

bool read_marg_binary(const char* path, Matrix& jacobian, Vector& rhs,
                      std::set<int>& keep, std::set<int>& marg) {
  std::ifstream input(path, std::ios::binary);
  char magic[11];
  input.read(magic, sizeof(magic));
  if (!input || std::memcmp(magic, "M7IM15MARG1", 11) != 0) return false;
  uint32_t version = 0;
  uint32_t keep_count = 0;
  uint32_t marg_count = 0;
  if (!read_binary(input, version) || !read_binary(input, keep_count) ||
      !read_binary(input, marg_count) || version != 1)
    return false;
  for (uint32_t i = 0; i < keep_count; ++i) {
    uint32_t index = 0;
    if (!read_binary(input, index)) return false;
    keep.insert(static_cast<int>(index));
  }
  for (uint32_t i = 0; i < marg_count; ++i) {
    uint32_t index = 0;
    if (!read_binary(input, index)) return false;
    marg.insert(static_cast<int>(index));
  }
  uint32_t marker = 0;
  if (!read_binary(input, marker) || marker != 1) return false;
  uint32_t rows = 0;
  uint32_t cols = 0;
  if (!read_binary(input, rows) || !read_binary(input, cols)) return false;
  jacobian.resize(rows, cols);
  for (uint32_t col = 0; col < cols; ++col) {
    for (uint32_t row = 0; row < rows; ++row) {
      float value = 0.0f;
      if (!read_binary(input, value)) return false;
      jacobian(row, col) = value;
    }
  }
  uint32_t rhs_size = 0;
  if (!read_binary(input, rhs_size)) return false;
  rhs.resize(rhs_size);
  for (uint32_t row = 0; row < rhs_size; ++row) {
    float value = 0.0f;
    if (!read_binary(input, value)) return false;
    rhs[row] = value;
  }
  return true;
}

uint64_t matrix_bits_hash(const Matrix& matrix) {
  uint64_t hash = 1469598103934665603ULL;
  for (Eigen::Index index = 0; index < matrix.size(); ++index) {
    const uint32_t value = bits(matrix.data()[index]);
    for (int byte = 0; byte < 4; ++byte) {
      hash ^= static_cast<uint8_t>((value >> (8 * byte)) & 0xffU);
      hash *= 1099511628211ULL;
    }
  }
  return hash;
}

uint64_t raw_bits_hash(const float* values, Eigen::Index count) {
  uint64_t hash = 1469598103934665603ULL;
  for (Eigen::Index index = 0; index < count; ++index) {
    const uint32_t value = bits(values[index]);
    for (int byte = 0; byte < 4; ++byte) {
      hash ^= static_cast<uint8_t>((value >> (8 * byte)) & 0xffU);
      hash *= 1099511628211ULL;
    }
  }
  return hash;
}

void trace_marg_binary(const char* path) {
  Matrix qj;
  Vector qr;
  std::set<int> keep;
  std::set<int> marg;
  if (!read_marg_binary(path, qj, qr, keep, marg)) {
    std::cerr << "invalid M7IM15 MARG capture: " << path << '\n';
    std::exit(4);
  }
  Eigen::Matrix<int, Eigen::Dynamic, 1> indices(qj.cols());
  auto it = marg.begin();
  for (Eigen::Index k = 0; k < static_cast<Eigen::Index>(marg.size()); ++k, ++it)
    indices[k] = *it;
  it = keep.begin();
  for (Eigen::Index k = 0; k < static_cast<Eigen::Index>(keep.size()); ++k, ++it)
    indices[marg.size() + k] = *it;
  qj.applyOnTheRight(
      Eigen::PermutationWrapper<Eigen::Matrix<int, Eigen::Dynamic, 1>>(
          indices));

  Eigen::Index total_rank = 0;
  const Eigen::Index rows = qj.rows();
  const Eigen::Index cols = qj.cols();
  Vector temp;
  temp.resize(cols + 1);
  float* temp_data = temp.data();
  const float rank_threshold = std::sqrt(std::numeric_limits<float>::epsilon());
  std::cout << "trace_input " << rows << ' ' << cols << ' ' << qr.size()
            << '\n';
  std::cout << "trace_base "
            << (reinterpret_cast<uintptr_t>(qj.data()) & 0x0fU) << '\n';
  for (Eigen::Index k = 0; k < cols && total_rank < rows; ++k) {
    const Eigen::Index row_start = total_rank;
    const Eigen::Index remaining_rows = rows - total_rank;
    const Eigen::Index remaining_cols = cols - k - 1;
    const float c0_before = qj.coeff(row_start, k);
    const auto tail = qj.col(k).tail(remaining_rows - 1);
    const Eigen::Index tail_aligned =
        Eigen::internal::first_default_aligned(tail);
    const float tail_sq_norm = tail.squaredNorm();
    std::cout << "trace_norm_meta " << k << ' ' << tail.size() << ' '
              << (reinterpret_cast<uintptr_t>(tail.data()) & 0x1fU) << ' '
              << tail_aligned << '\n';
    float beta = 0.0f;
    float h_coeff = 0.0f;
    qj.col(k).tail(remaining_rows).makeHouseholderInPlace(h_coeff, beta);
    std::cout << "trace_reflector " << k << ' ' << total_rank << ' '
              << std::hex << bits(c0_before) << ' ' << bits(tail_sq_norm)
              << ' ' << bits(beta) << ' ' << bits(h_coeff) << std::dec << ' '
              << ((std::abs(beta) > rank_threshold) ? 1 : 0) << '\n';
    if (std::abs(beta) > rank_threshold) {
      if (k == 9) {
        std::cout << "trace_stage9_col14_pre";
        for (Eigen::Index index = 0; index < rows; ++index)
          std::cout << ' ' << std::hex << bits(qj.coeff(index, k + 1 + 14));
        std::cout << std::dec << '\n';
      }
      qj.coeffRef(total_rank, k) = beta;
      qj.bottomRightCorner(remaining_rows, remaining_cols)
          .applyHouseholderOnTheLeft(
              qj.col(k).tail(remaining_rows - 1), h_coeff,
              temp_data + k + 1);
      std::cout << "trace_workspace " << k;
      for (Eigen::Index index = 0;
           index < std::min<Eigen::Index>(remaining_cols, 8); ++index) {
        std::cout << ' ' << std::hex << bits(temp_data[k + 1 + index]);
      }
      std::cout << std::dec << '\n';
      if (k == 9) {
        std::cout << "trace_workspace_full " << k;
        for (Eigen::Index index = 0; index < remaining_cols; ++index) {
          std::cout << ' ' << std::hex << bits(temp_data[k + 1 + index]);
        }
        std::cout << std::dec << '\n';
        std::cout << "trace_stage9_workspace_add "
                  << std::hex
                  << raw_bits_hash(temp_data + k + 1, remaining_cols)
                  << std::dec << '\n';
        std::cout << "trace_stage9_workspace_add_full";
        for (Eigen::Index index = 0; index < remaining_cols; ++index) {
          std::cout << ' ' << std::hex << bits(temp_data[k + 1 + index]);
        }
        std::cout << std::dec << '\n';
        std::cout << "trace_stage9_row0 " << std::hex;
        uint64_t row0_hash = 1469598103934665603ULL;
        for (Eigen::Index index = 0; index < remaining_cols; ++index) {
          const uint32_t value = bits(qj.coeff(total_rank, k + 1 + index));
          for (int byte = 0; byte < 4; ++byte) {
            row0_hash ^= static_cast<uint8_t>((value >> (8 * byte)) & 0xffU);
            row0_hash *= 1099511628211ULL;
          }
        }
        std::cout << row0_hash << std::dec << '\n';
        std::cout << "trace_stage9_colhash";
        for (Eigen::Index index = 0; index < remaining_cols; ++index) {
          std::cout << ' ' << std::hex
                    << raw_bits_hash(qj.data() + (k + 1 + index) * rows, rows);
        }
        std::cout << std::dec << '\n';
        std::cout << "trace_stage9_col14_post";
        for (Eigen::Index index = 0; index < rows; ++index)
          std::cout << ' ' << std::hex << bits(qj.coeff(index, k + 1 + 14));
        std::cout << std::dec << '\n';
      }
      qr.tail(remaining_rows).applyHouseholderOnTheLeft(
          qj.col(k).tail(remaining_rows - 1), h_coeff, temp_data + cols);
      if (std::getenv("M7IM15_TRACE_RHS") != nullptr) {
        std::cout << "trace_rhs " << k << ' ' << std::hex
                  << raw_bits_hash(qr.data(), qr.size()) << std::dec << ' '
                  << bits(qr[total_rank]) << ' '
                  << bits(qr[std::min<Eigen::Index>(total_rank + 1,
                                                     qr.size() - 1)])
                  << '\n';
      }
      ++total_rank;
    } else {
      qj.coeffRef(total_rank, k) = 0.0f;
    }
    qj.col(k).tail(remaining_rows - 1).setZero();
    std::cout << "trace_after " << k << ' ' << total_rank << ' '
              << std::hex << matrix_bits_hash(qj) << std::dec << ' '
              << bits(qj.coeff(total_rank - 1, k)) << ' '
              << bits(qj.coeff(total_rank - 1, std::min<Eigen::Index>(k + 1,
                                                                        cols - 1)))
              << '\n';
  }
  const Eigen::Index keep_valid_rows =
      std::max(total_rank - static_cast<Eigen::Index>(marg.size()),
               Eigen::Index(1));
  const Eigen::Index out_row_start = static_cast<Eigen::Index>(marg.size());
  std::cout << "trace_output " << keep_valid_rows << ' '
            << static_cast<Eigen::Index>(keep.size()) << '\n';
  for (Eigen::Index col = 0; col < static_cast<Eigen::Index>(keep.size());
       ++col) {
    for (Eigen::Index row = 0; row < keep_valid_rows; ++row) {
      std::cout << std::hex
                << bits(qj.coeff(out_row_start + row,
                                 static_cast<Eigen::Index>(marg.size()) + col))
                << '\n';
    }
  }
  std::cout << "trace_output_r " << keep_valid_rows << '\n';
  for (Eigen::Index row = 0; row < keep_valid_rows; ++row)
    std::cout << std::hex << bits(qr[out_row_start + row]) << '\n';
}

void run_fixture(const char* path) {
  Matrix jacobian;
  Vector rhs;
  std::set<int> keep;
  std::set<int> marg;
  if (!read_fixture(path, jacobian, rhs, keep, marg)) {
    std::cerr << "invalid M6 fixture: " << path << '\n';
    std::exit(3);
  }
  Matrix out_j;
  Vector out_r;
  marginalize_sqrt_upstream(jacobian, rhs, keep, marg, out_j, out_r);
  std::cout << std::setprecision(9);
  std::cout << "fixture_result " << out_j.rows() << ' ' << out_j.cols() << ' '
            << out_r.size() << '\n';
  const float j_norm = out_j.norm();
  const float j_max = out_j.cwiseAbs().maxCoeff();
  const float r_norm = out_r.norm();
  const float r_max = out_r.cwiseAbs().maxCoeff();
  std::cout << "fixture_j_norm " << j_norm << " " << bits(j_norm) << " max "
            << j_max << " " << bits(j_max) << '\n';
  std::cout << "fixture_r_norm " << r_norm << " " << bits(r_norm)
            << " max " << r_max << " " << bits(r_max) << '\n';
  const std::vector<std::pair<int, int>> j_indices = {
      {0, 0}, {0, 1}, {0, out_j.cols() - 1}, {1, 0},
      {out_j.rows() / 2, out_j.cols() / 2},
      {out_j.rows() - 1, out_j.cols() - 1}};
  for (const auto [row, col] : j_indices) {
    const float value = out_j(row, col);
    std::cout << "fixture_j " << row << ' ' << col << ' ' << value << ' '
              << bits(value) << '\n';
  }
  const std::vector<Eigen::Index> r_indices = {0, out_r.size() / 2,
                                               out_r.size() - 1};
  for (const Eigen::Index row : r_indices) {
    const float value = out_r[row];
    std::cout << "fixture_r " << row << ' ' << value << ' ' << bits(value)
              << '\n';
  }

  // Keep a complete raw-bit stream in addition to the compact diagnostics
  // above.  The M6 boundary is an exact arithmetic gate: comparing only a
  // handful of selected entries can hide a packet-order mismatch elsewhere
  // in the retained 54x57 Jacobian or 54-element RHS.
  for (Eigen::Index row = 0; row < out_j.rows(); ++row) {
    for (Eigen::Index col = 0; col < out_j.cols(); ++col) {
      std::cout << "fixture_j_bits " << row << ' ' << col << ' '
                << bits(out_j(row, col)) << '\n';
    }
  }
  for (Eigen::Index row = 0; row < out_r.size(); ++row) {
    std::cout << "fixture_r_bits " << row << ' ' << bits(out_r[row])
              << '\n';
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc == 3 && std::string(argv[1]) == "--trace-marg") {
    trace_marg_binary(argv[2]);
    return 0;
  }
  if (argc == 2) {
    run_fixture(argv[1]);
    return 0;
  }
  run_case(normal_case());
  run_case(near_rank_case());
  run_case(sign_case());
  run_case(zero_case());
  run_case(denormal_case());
  return 0;
}
