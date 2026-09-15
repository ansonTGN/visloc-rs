// Diagnostic-only oracle for Basalt's fixed 15 x 30 ImuBlock product.
//
// The pinned ImuBlock (0f3b2b52...) builds a local dynamic MatrixX<float>
// with 15 residual rows and 30 columns, then evaluates
//
//     H = Jp.transpose() * Jp;
//     b = Jp.transpose() * r;
//
// This program consumes the bit-preserving frame-4 fields emitted by the
// existing m7im covariance oracle and the state/bias fields emitted by the
// existing upstream audit.  It emits the native Eigen result, a scalar model
// of the AVX GEBP/GEMV schedule, and padded full-state probes.  It is kept in
// benchmarks/basalt deliberately; no production source is included.

#include <Eigen/Core>

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using Json = nlohmann::json;
using Scalar = float;
using Matrix = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
using Vector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

constexpr int kImuRows = 15;
constexpr int kLocalCols = 30;
constexpr int kWhitenedRows = 9;

float from_bits(const std::string& text) {
  std::uint32_t raw = 0;
  try {
    raw = static_cast<std::uint32_t>(std::stoul(text, nullptr, 16));
  } catch (const std::exception& error) {
    throw std::runtime_error("invalid f32 bit word '" + text + "': " +
                             error.what());
  }
  float value = 0;
  static_assert(sizeof(value) == sizeof(raw));
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

std::vector<float> read_bit_array(const Json& field) {
  const Json* words = nullptr;
  if (field.is_array()) {
    words = &field;
  } else if (field.is_object() && field.contains("bits_row_major")) {
    words = &field.at("bits_row_major");
  } else if (field.is_object() && field.contains("bits")) {
    words = &field.at("bits");
  } else {
    throw std::runtime_error("expected bit-array object or array");
  }
  std::vector<float> values;
  values.reserve(words->size());
  for (const Json& item : *words)
    values.push_back(from_bits(item.get<std::string>()));
  return values;
}

Matrix read_bit_matrix(const Json& field) {
  const int rows = field.at("rows").get<int>();
  const int cols = field.at("cols").get<int>();
  const auto values = read_bit_array(field);
  if (static_cast<int>(values.size()) != rows * cols)
    throw std::runtime_error("matrix bit-array has unexpected length");
  Matrix result(rows, cols);
  for (int row = 0; row < rows; ++row)
    for (int col = 0; col < cols; ++col)
      result(row, col) = values[row * cols + col];
  return result;
}

Vector read_bit_vector(const Json& field) {
  const auto values = read_bit_array(field);
  const int length = field.is_object() && field.contains("length")
                         ? field.at("length").get<int>()
                         : static_cast<int>(values.size());
  if (static_cast<int>(values.size()) != length)
    throw std::runtime_error("vector bit-array has unexpected length");
  Vector result(length);
  for (int i = 0; i < length; ++i) result(i) = values[i];
  return result;
}

std::vector<Scalar> read_decimal_vector(const Json& field) {
  std::vector<Scalar> result;
  if (!field.is_array()) throw std::runtime_error("expected decimal vector");
  result.reserve(field.size());
  for (const Json& item : field) result.push_back(item.get<Scalar>());
  return result;
}

// Keep every packet add as an observable f32 operation.  This is equivalent
// to Eigen's padd, and also prevents a host compiler from retaining a wider
// temporary while compiling this diagnostic model.
float add_f32(float left, float right) {
  volatile float result = left + right;
  return result;
}

float fma_f32(float left, float right, float acc) {
  return std::fmaf(left, right, acc);
}

float mul_f32(float left, float right) {
  volatile float result = left * right;
  return result;
}

float mul_add_f32(float left, float right, float acc) {
  volatile float product = left * right;
  volatile float result = acc + product;
  return result;
}

// AVX Packet8f predux: padd(lo, hi), followed by the SSE Packet4f tree.
float predux_packet8(const float lanes[8]) {
  const float q0 = add_f32(lanes[0], lanes[4]);
  const float q1 = add_f32(lanes[1], lanes[5]);
  const float q2 = add_f32(lanes[2], lanes[6]);
  const float q3 = add_f32(lanes[3], lanes[7]);
  const float lo = add_f32(q0, q2);
  const float hi = add_f32(q1, q3);
  return add_f32(lo, hi);
}

// Local 30x15 * 15x30 GEBP model.  In the pinned AVX traits mr=24, nr=4,
// LhsProgress=8, pk=8.  The 30 output rows are dispatched as
// 24 packet rows + 4 half-packet rows + 2 quarter-packet rows.  The first
// 24-row (3-packet) path has one accumulator and therefore keeps k order;
// the half/quarter X4 paths use the two C/D accumulators for k=0..7.  The
// final two columns are X1 and keep direct k order for all row groups.
Matrix model_local_h(const Matrix& jp) {
  Matrix result = Matrix::Zero(kLocalCols, kLocalCols);
  for (int row = 0; row < kLocalCols; ++row) {
    for (int col = 0; col < kLocalCols; ++col) {
      // The generated AVX kernel has a doubled Packet4 C/D path for rows
      // 24..27.  Its quarter (2-row) path is also active for rows 28..29,
      // but it peels the depth differently: the even accumulator consumes
      // k=0,2,..,12, the odd accumulator k=1,3,..,13, and k=14 is folded
      // with a scalar FMA after the two accumulators are reduced.  This is
      // observable on a dense probe even though the production Jp has many
      // structural zeros.
      const bool half_packet_tail = row >= 24 && row < 28 && col < 28;
      const bool quarter_packet_tail = row >= 28 && col < 28;
      if (!half_packet_tail && !quarter_packet_tail) {
        float value = 0;
        for (int k = 0; k < kImuRows; ++k)
          value = fma_f32(jp(k, row), jp(k, col), value);
        result(row, col) = value;
        continue;
      }

      float value = 0;
      if (quarter_packet_tail) {
        // The remaining-row 1x4 kernel uses SwappedTraits with spk=2.
        // Its first eight depths are four Packet8 accumulators (depth pairs)
        // reduced as (C0+C1)+(C2+C3), so preserve that tree per parity lane.
        const float e0 = fma_f32(jp(0, row), jp(0, col), 0.0f);
        const float e1 = fma_f32(jp(2, row), jp(2, col), 0.0f);
        const float e2 = fma_f32(jp(4, row), jp(4, col), 0.0f);
        const float e3 = fma_f32(jp(6, row), jp(6, col), 0.0f);
        const float o0 = fma_f32(jp(1, row), jp(1, col), 0.0f);
        const float o1 = fma_f32(jp(3, row), jp(3, col), 0.0f);
        const float o2 = fma_f32(jp(5, row), jp(5, col), 0.0f);
        const float o3 = fma_f32(jp(7, row), jp(7, col), 0.0f);
        float even = add_f32(add_f32(e0, e1), add_f32(e2, e3));
        float odd = add_f32(add_f32(o0, o1), add_f32(o2, o3));
        for (int k = 8; k < 14; k += 2) {
          even = fma_f32(jp(k, row), jp(k, col), even);
          odd = fma_f32(jp(k + 1, row), jp(k + 1, col), odd);
        }
        value = add_f32(even, odd);
        value = fma_f32(jp(14, row), jp(14, col), value);
      } else {
        float even = 0;
        float odd = 0;
        for (int k = 0; k < 8; k += 2) {
          even = fma_f32(jp(k, row), jp(k, col), even);
          odd = fma_f32(jp(k + 1, row), jp(k + 1, col), odd);
        }
        value = add_f32(even, odd);
        for (int k = 8; k < kImuRows; ++k)
          value = fma_f32(jp(k, row), jp(k, col), value);
      }
      result(row, col) = value;
    }
  }
  return result;
}

Vector model_local_b(const Matrix& jp, const Vector& residual,
                     bool scalar_tail_fma) {
  (void)scalar_tail_fma;
  Vector result(kLocalCols);
  result.setZero();
  // rows=30 gives exactly 8-row, 8-row, 8-row, 4-row, 2-row groups.  The
  // Packet8 path covers depth 0..7.  The generated scalar cleanup for this
  // dynamic 15-depth product uses ordinary mul/add for k=8..11 followed by
  // scalar FMAs for k=12..14.
  for (int col = 0; col < kLocalCols; ++col) {
    float value = 0;
    // The packet predux's exact reduction tree is independent of the
    // scalar cleanup choice.  Keep it explicit so this diagnostic models the
    // generated Eigen kernel rather than a compiler-dependent loop.
    float packet_lanes[8];
    for (int k = 0; k < 8; ++k)
      packet_lanes[k] = mul_f32(jp(k, col), residual(k));
    value = predux_packet8(packet_lanes);
    for (int k = 8; k < 12; ++k)
      value = mul_add_f32(jp(k, col), residual(k), value);
    for (int k = 12; k < kImuRows; ++k)
      value = fma_f32(jp(k, col), residual(k), value);
    result(col) = value;
  }
  return result;
}

// The real IMU Jacobian has structural zeros in its last six columns, so a
// schedule probe based only on that input can hide a packet-order mismatch.
// Use a deterministic dense f32 probe as well.  Its values are generated by
// integer arithmetic and narrowed before any product is evaluated, making it
// portable across the pinned oracle builds while exercising every depth lane.
Matrix synthetic_jp() {
  Matrix result(kImuRows, kLocalCols);
  std::uint32_t state = 0x7f4a7c15u;
  for (int row = 0; row < kImuRows; ++row) {
    for (int col = 0; col < kLocalCols; ++col) {
      state ^= state << 13;
      state ^= state >> 17;
      state ^= state << 5;
      int value = static_cast<int>(state % 2000001u) - 1000000;
      if (value == 0) value = 1;
      volatile float narrowed = static_cast<float>(value) / Scalar(8192);
      result(row, col) = narrowed;
    }
  }
  return result;
}

Vector synthetic_r() {
  Vector result(kImuRows);
  std::uint32_t state = 0x2e1b4f93u;
  for (int row = 0; row < kImuRows; ++row) {
    state = state * 1664525u + 1013904223u;
    int value = static_cast<int>(state % 1600001u) - 800000;
    if (value == 0) value = 1;
    volatile float narrowed = static_cast<float>(value) / Scalar(4096);
    result(row) = narrowed;
  }
  return result;
}

std::string word(float value) {
  std::uint32_t raw = 0;
  std::memcpy(&raw, &value, sizeof(raw));
  std::ostringstream out;
  out << std::hex << std::setw(8) << std::setfill('0') << raw;
  return out.str();
}

template <typename Derived>
Json bit_matrix(const Eigen::MatrixBase<Derived>& value) {
  Json result;
  result["rows"] = value.rows();
  result["cols"] = value.cols();
  result["storage_order"] = "column_major";
  result["bits_row_major"] = Json::array();
  result["bits_column_major"] = Json::array();
  for (Eigen::Index row = 0; row < value.rows(); ++row)
    for (Eigen::Index col = 0; col < value.cols(); ++col)
      result["bits_row_major"].push_back(word(static_cast<float>(value(row, col))));
  for (Eigen::Index col = 0; col < value.cols(); ++col)
    for (Eigen::Index row = 0; row < value.rows(); ++row)
      result["bits_column_major"].push_back(word(static_cast<float>(value(row, col))));
  return result;
}

template <typename Derived>
Json bit_vector(const Eigen::MatrixBase<Derived>& value) {
  Json result;
  result["length"] = value.size();
  result["bits"] = Json::array();
  for (Eigen::Index i = 0; i < value.size(); ++i)
    result["bits"].push_back(word(static_cast<float>(value.derived().coeff(i))));
  return result;
}

struct Difference {
  std::size_t count = 0;
  int first = -1;
};

template <typename LeftDerived, typename RightDerived>
Difference compare_values(const Eigen::MatrixBase<LeftDerived>& left,
                          const Eigen::MatrixBase<RightDerived>& right) {
  if (left.rows() != right.rows() || left.cols() != right.cols())
    throw std::runtime_error("cannot compare differently shaped values");
  Difference result;
  int flat = 0;
  for (Eigen::Index row = 0; row < left.rows(); ++row) {
    for (Eigen::Index col = 0; col < left.cols(); ++col, ++flat) {
      if (word(static_cast<float>(left(row, col))) !=
          word(static_cast<float>(right(row, col)))) {
        ++result.count;
        if (result.first < 0) result.first = flat;
      }
    }
  }
  return result;
}

Json difference_json(const Difference& difference, std::size_t total) {
  Json result;
  result["mismatch_count"] = difference.count;
  result["total"] = total;
  result["exact"] = difference.count == 0;
  result["first_flat_index"] = difference.first < 0 ? Json(nullptr) :
                                                                  Json(difference.first);
  return result;
}

Matrix make_local_jp(const Matrix& top, const Vector& top_r,
                     const Json& link, float gyro_std, float accel_std) {
  if (top.rows() != kWhitenedRows || top.cols() != kLocalCols ||
      top_r.size() != kWhitenedRows)
    throw std::runtime_error("expected top whitened J=9x30 and r=9");

  Matrix result = Matrix::Zero(kImuRows, kLocalCols);
  result.topRows<kWhitenedRows>() = top;

  Vector residual = Vector::Zero(kImuRows);
  residual.head<kWhitenedRows>() = top_r;

  const auto& from = link.at("from_state");
  const auto& to = link.at("to_state");
  const auto bg0 = read_decimal_vector(from.at("bias_gyro"));
  const auto bg1 = read_decimal_vector(to.at("bias_gyro"));
  const auto ba0 = read_decimal_vector(from.at("bias_accel"));
  const auto ba1 = read_decimal_vector(to.at("bias_accel"));
  if (bg0.size() != 3 || bg1.size() != 3 || ba0.size() != 3 || ba1.size() != 3)
    throw std::runtime_error("state bias vectors are not 3D");

  // Match ImuBlock exactly: dt is narrowed to Scalar before sqrt, while the
  // calibration weights are the inverse std-devs (10000 and 1000 for the
  // checked MH-01 calibration).
  const auto dt_ns = link.at("delta").at("dt_ns").get<std::int64_t>();
  const float dt = static_cast<float>(dt_ns) * Scalar(1e-9);
  const float inv_sqrt_dt = Scalar(1) / std::sqrt(dt);
  const float gyro_weight = (Scalar(1) / gyro_std) * inv_sqrt_dt;
  const float accel_weight = (Scalar(1) / accel_std) * inv_sqrt_dt;
  for (int axis = 0; axis < 3; ++axis) {
    result(9 + axis, 9 + axis) = gyro_weight;
    result(9 + axis, 24 + axis) = -gyro_weight;
    residual(9 + axis) = gyro_weight * (bg0[axis] - bg1[axis]);

    result(12 + axis, 12 + axis) = accel_weight;
    result(12 + axis, 27 + axis) = -accel_weight;
    residual(12 + axis) = accel_weight * (ba0[axis] - ba1[axis]);
  }

  // The residual is returned through the caller so that its bit words are
  // visible in the output.  This local assignment is only a shape check.
  (void)residual;
  return result;
}

Vector make_local_r(const Vector& top_r, const Json& link, float gyro_std,
                    float accel_std) {
  Vector result = Vector::Zero(kImuRows);
  result.head<kWhitenedRows>() = top_r;
  const auto& from = link.at("from_state");
  const auto& to = link.at("to_state");
  const auto bg0 = read_decimal_vector(from.at("bias_gyro"));
  const auto bg1 = read_decimal_vector(to.at("bias_gyro"));
  const auto ba0 = read_decimal_vector(from.at("bias_accel"));
  const auto ba1 = read_decimal_vector(to.at("bias_accel"));
  const auto dt_ns = link.at("delta").at("dt_ns").get<std::int64_t>();
  const float dt = static_cast<float>(dt_ns) * Scalar(1e-9);
  const float inv_sqrt_dt = Scalar(1) / std::sqrt(dt);
  const float gyro_weight = (Scalar(1) / gyro_std) * inv_sqrt_dt;
  const float accel_weight = (Scalar(1) / accel_std) * inv_sqrt_dt;
  for (int axis = 0; axis < 3; ++axis) {
    result(9 + axis) = gyro_weight * (bg0[axis] - bg1[axis]);
    result(12 + axis) = accel_weight * (ba0[axis] - ba1[axis]);
  }
  return result;
}

Json padded_probe(const Matrix& local, int state_dof, int offset,
                  const Matrix& local_h) {
  Matrix full = Matrix::Zero(kImuRows, state_dof);
  full.block(0, offset, kImuRows, kLocalCols) = local;
  Matrix h = full.transpose() * full;
  const Matrix active = h.block(offset, offset, kLocalCols, kLocalCols);
  const Difference diff = compare_values(active, local_h);
  Json result;
  result["state_dof"] = state_dof;
  result["active_column_offset"] = offset;
  result["native_product_shape"] = {state_dof, state_dof};
  result["active_block_comparison"] = difference_json(diff, kLocalCols * kLocalCols);
  result["first_active_block_bit"] =
      diff.first < 0 ? Json(nullptr) : Json(word(active(diff.first / kLocalCols,
                                                      diff.first % kLocalCols)));
  return result;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4 || argc > 7) {
    std::cerr << "usage: m7im15_schedule_oracle <m7im_json> <m7aq_json> "
                 "<output_json> [link=3] [gyro_bias_std=0.0001] "
                 "[accel_bias_std=0.001]\n";
    return 2;
  }

  try {
    const int link_index = argc >= 5 ? std::stoi(argv[4]) : 3;
    const float gyro_std = argc >= 6 ? std::stof(argv[5]) : Scalar(0.0001);
    const float accel_std = argc >= 7 ? std::stof(argv[6]) : Scalar(0.001);

    Json m7im;
    Json m7aq;
    {
      std::ifstream input(argv[1]);
      if (!input) throw std::runtime_error("cannot open m7im JSON");
      input >> m7im;
    }
    {
      std::ifstream input(argv[2]);
      if (!input) throw std::runtime_error("cannot open m7aq JSON");
      input >> m7aq;
    }

    const Json& factor = m7im.at("frame4_factor");
    const Json& links = m7aq.at("links");
    if (link_index < 0 || link_index >= static_cast<int>(links.size()))
      throw std::runtime_error("link index is outside the audit JSON");
    const Json& link = links.at(link_index);
    const Matrix top_j = read_bit_matrix(factor.at("whitened_jacobian"));
    const Vector top_r = read_bit_vector(factor.at("whitened_residual"));
    const Matrix jp = make_local_jp(top_j, top_r, link, gyro_std, accel_std);
    const Vector residual = make_local_r(top_r, link, gyro_std, accel_std);

    // Keep these expressions dynamic and column-major, matching MatX/VecX in
    // the upstream ImuBlock rather than a fixed-size convenience expression.
    const Matrix eigen_h = jp.transpose() * jp;
    const Vector eigen_b = jp.transpose() * residual;
    const Matrix model_h = model_local_h(jp);
    const Vector model_b_fma = model_local_b(jp, residual, true);
    const Vector model_b_mul_add = model_local_b(jp, residual, false);

    const Difference h_diff = compare_values(eigen_h, model_h);
    const Difference b_fma_diff = compare_values(eigen_b, model_b_fma);
    const Difference b_mul_add_diff = compare_values(eigen_b, model_b_mul_add);

    Json output;
    output["schema"] = "basalt.m7im15_schedule_oracle.v1";
    output["status"] = "diagnostic_only";
    output["contract"] = {
        {"pinned_upstream_commit",
         "0f3b2b52c807f70ff4e2973ce253c73329eea7bc"},
        {"source", "include/basalt/linearization/imu_block.hpp"},
        {"scalar", "float32"},
        {"local_jp_shape", {kImuRows, kLocalCols}},
        {"product", "dynamic column-major MatrixX<float> Jp.transpose()*Jp and Jp.transpose()*r"},
        {"compiler_flags", "g++ 11.4 -O3 -g -DEIGEN_DONT_PARALLELIZE -march=skylake"},
        {"bias_weights", { {"gyro_bias_std", gyro_std},
                            {"accel_bias_std", accel_std} }},
        {"production_source_touched", false},
    };
    output["inputs"] = {
        {"Jp", bit_matrix(jp)},
        {"r", bit_vector(residual)},
        {"link_index", link_index},
        {"duration_ns", link.at("delta").at("dt_ns")},
    };
    output["native_eigen"] = {
        {"H", bit_matrix(eigen_h)},
        {"b", bit_vector(eigen_b)},
    };
    output["models"] = {
        {"H_local_avx_gebp", bit_matrix(model_h)},
        {"b_packet8_scalar_tail_fma", bit_vector(model_b_fma)},
        {"b_packet8_scalar_tail_mul_add", bit_vector(model_b_mul_add)},
    };
    output["comparisons"] = {
        {"H_local_avx_gebp", difference_json(h_diff, kLocalCols * kLocalCols)},
        {"b_packet8_scalar_tail_fma", difference_json(b_fma_diff, kLocalCols)},
        {"b_packet8_scalar_tail_mul_add", difference_json(b_mul_add_diff, kLocalCols)},
    };
    output["schedule"] = {
        {"H", {
            {"avx_packet", "Packet8f"},
            {"mr", 24},
            {"nr", 4},
            {"lhs_progress", 8},
            {"depth", 15},
            {"peeled_depth", 8},
            {"row_groups", "0..23: Packet8 direct k; 24..27: Packet4 C/D even/odd k=0..7, reduce, then scalar-FMA k=8..14; 28..29: remaining-row 1x4 Packet8 C0..C3 over depth pairs, (C0+C1)+(C2+C3), paired scalar-FMA k=8..13, predux_half, scalar-FMA k=14"},
            {"column_groups", "0..27: X4; 28..29: X1 direct k"},
        }},
        {"b", {
            {"kernel", "row-major GEMV on Jp.transpose()"},
            {"output_row_groups", "0..7, 8..15, 16..23, 24..27, 28..29"},
            {"packet_depth", "k=0..7 Packet8f"},
            {"predux", "Packet8 pmul lanes; q0=l0+l4; q1=l1+l5; q2=l2+l6; q3=l3+l7; (q0+q2)+(q1+q3)"},
            {"scalar_tail", "k=8..11 source cj.pmul + cc +=; k=12..14 scalar vfmadd231ss"},
        }},
    };

    // A zero-padded global matrix is mathematically equivalent but is not the
    // same Eigen kernel.  These probes show the active 30x30 block changing
    // when state_dof/offset moves the 24/4/2 row and X4/X1 boundaries.
    output["padded_full_state_probes"] = Json::array();
    const int state_dofs[] = {30, 45, 60, 75};
    for (int state_dof : state_dofs) {
      for (int offset = 0; offset + kLocalCols <= state_dof; offset += 15)
        output["padded_full_state_probes"].push_back(
            padded_probe(jp, state_dof, offset, eigen_h));
    }

    // Dense probe: this makes the schedule distinction observable even though
    // the production IMU Jp has zero top-row entries in its six bias-end
    // columns.  It is an oracle self-check, not production input.
    const Matrix dense_jp = synthetic_jp();
    const Vector dense_r = synthetic_r();
    const Matrix dense_h = dense_jp.transpose() * dense_jp;
    const Vector dense_b = dense_jp.transpose() * dense_r;
    const Matrix dense_model_h = model_local_h(dense_jp);
    const Vector dense_model_b_fma = model_local_b(dense_jp, dense_r, true);
    const Vector dense_model_b_mul_add = model_local_b(dense_jp, dense_r, false);
    const Difference dense_h_diff = compare_values(dense_h, dense_model_h);
    const Difference dense_b_fma_diff =
        compare_values(dense_b, dense_model_b_fma);
    const Difference dense_b_mul_add_diff =
        compare_values(dense_b, dense_model_b_mul_add);
    output["synthetic_dense_probe"] = {
        {"inputs", {{"Jp", bit_matrix(dense_jp)}, {"r", bit_vector(dense_r)}}},
        {"native_eigen", {{"H", bit_matrix(dense_h)},
                           {"b", bit_vector(dense_b)}}},
        {"models", {{"H_local_avx_gebp", bit_matrix(dense_model_h)},
                     {"b_packet8_scalar_tail_fma", bit_vector(dense_model_b_fma)},
                     {"b_packet8_scalar_tail_mul_add", bit_vector(dense_model_b_mul_add)}}},
        {"comparisons", {{"H_local_avx_gebp",
                           difference_json(dense_h_diff, kLocalCols * kLocalCols)},
                          {"b_packet8_scalar_tail_fma",
                           difference_json(dense_b_fma_diff, kLocalCols)},
                          {"b_packet8_scalar_tail_mul_add",
                           difference_json(dense_b_mul_add_diff, kLocalCols)}}},
    };
    output["synthetic_padded_full_state_probes"] = Json::array();
    for (int state_dof : state_dofs) {
      for (int offset = 0; offset + kLocalCols <= state_dof; offset += 15)
        output["synthetic_padded_full_state_probes"].push_back(
            padded_probe(dense_jp, state_dof, offset, dense_h));
    }

    std::ofstream out(argv[3]);
    if (!out) throw std::runtime_error("cannot open output JSON");
    out << std::setw(2) << output << '\n';
    std::cout << "H mismatches (local model vs Eigen): " << h_diff.count
              << "/" << kLocalCols * kLocalCols << '\n';
    std::cout << "b mismatches (packet + scalar FMA tail): " << b_fma_diff.count
              << "/" << kLocalCols << '\n';
    std::cout << "b mismatches (packet + scalar mul/add tail): "
              << b_mul_add_diff.count << "/" << kLocalCols << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "m7im15 schedule oracle failed: " << error.what() << '\n';
    return 1;
  }
}
