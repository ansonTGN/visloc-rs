// Exact-bit probe for Eigen's A * diagonal * A.transpose() schedule.
// This reads the first native intermediate record and compares several
// operation associations against the authoritative native term2 payload.

#include <Eigen/Core>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using MatN3 = Eigen::Matrix<float, 9, 3>;
using MatNN = Eigen::Matrix<float, 9, 9>;
using Vec3 = Eigen::Matrix<float, 3, 1>;

float from_bits(std::uint32_t value) {
  float result = 0.0f;
  std::memcpy(&result, &value, sizeof(result));
  return result;
}

std::uint32_t bits(float value) {
  std::uint32_t result = 0;
  std::memcpy(&result, &value, sizeof(result));
  return result;
}

std::vector<std::uint32_t> words(const std::string& text) {
  std::vector<std::uint32_t> result;
  std::stringstream stream(text);
  std::string token;
  while (std::getline(stream, token, ',')) {
    if (!token.empty()) result.push_back(static_cast<std::uint32_t>(
        std::stoul(token, nullptr, 16)));
  }
  return result;
}

std::string field(const std::string& line, const std::string& name) {
  const std::string prefix = name + "=";
  const auto begin = line.find(prefix);
  if (begin == std::string::npos) throw std::runtime_error("missing field " + name);
  const auto value_begin = begin + prefix.size();
  const auto value_end = line.find(' ', value_begin);
  return line.substr(value_begin, value_end == std::string::npos
                                     ? std::string::npos
                                     : value_end - value_begin);
}

template <typename Matrix>
void load_matrix(Matrix& matrix, const std::vector<std::uint32_t>& values) {
  if (values.size() != static_cast<std::size_t>(matrix.size()))
    throw std::runtime_error("unexpected matrix size");
  for (Eigen::Index i = 0; i < matrix.size(); ++i)
    matrix.derived().data()[i] = from_bits(values[static_cast<std::size_t>(i)]);
}

int mismatches(const MatNN& actual, const MatNN& expected, int* first = nullptr) {
  int count = 0;
  int witness = -1;
  for (Eigen::Index i = 0; i < actual.size(); ++i) {
    if (bits(actual.data()[i]) != bits(expected.data()[i])) {
      ++count;
      if (witness < 0) witness = static_cast<int>(i);
    }
  }
  if (first) *first = witness;
  return count;
}

MatNN weighted_eigen(const MatN3& a, const Vec3& cov) {
  return a * cov.asDiagonal() * a.transpose();
}

MatNN weighted_materialized(const MatN3& a, const Vec3& cov) {
  const MatN3 scaled = a * cov.asDiagonal();
  return scaled * a.transpose();
}

MatNN weighted_manual(const MatN3& a, const Vec3& cov, const int order[3],
                      int association) {
  MatNN result;
  for (int col = 0; col < 9; ++col) {
    for (int row = 0; row < 9; ++row) {
      const int k0 = order[0];
      const int k1 = order[1];
      const int k2 = order[2];
      float value = 0.0f;
      auto product = [&](int k) {
        if (association == 0) return (a(row, k) * cov(k)) * a(col, k);
        if (association == 1) return a(row, k) * (cov(k) * a(col, k));
        return (a(row, k) * a(col, k)) * cov(k);
      };
      value = product(k0);
      value = std::fma(product(k1), 1.0f, value);
      value = std::fma(product(k2), 1.0f, value);
      result(row, col) = value;
    }
  }
  return result;
}

MatNN weighted_scaled_manual(const MatN3& a, const Vec3& cov,
                            const int order[3], int association) {
  MatN3 scaled;
  for (int k = 0; k < 3; ++k) {
    for (int row = 0; row < 9; ++row) {
      if (association == 0) scaled(row, k) = a(row, k) * cov(k);
      else if (association == 1) scaled(row, k) = cov(k) * a(row, k);
      else scaled(row, k) = std::fma(a(row, k), cov(k), 0.0f);
    }
  }
  MatNN result;
  for (int col = 0; col < 9; ++col) {
    for (int row = 0; row < 9; ++row) {
      const int k0 = order[0];
      const int k1 = order[1];
      const int k2 = order[2];
      float value = scaled(row, k0) * a(col, k0);
      value = std::fma(scaled(row, k1), a(col, k1), value);
      value = std::fma(scaled(row, k2), a(col, k2), value);
      result(row, col) = value;
    }
  }
  return result;
}

void report(const char* name, const MatNN& value, const MatNN& expected) {
  int first = -1;
  const int count = mismatches(value, expected, &first);
  std::cout << name << " mismatches=" << count << "/81";
  if (first >= 0) {
    std::cout << " first=" << first << " actual=" << std::hex
              << std::setw(8) << std::setfill('0') << bits(value.data()[first])
              << " expected=" << std::setw(8) << bits(expected.data()[first])
              << std::dec;
  }
  std::cout << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  const std::string path = argc > 1 ? argv[1] : "target/m7hd_native_intermediate.jsonl";
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open " + path);
  std::string step;
  std::string cov_line;
  for (std::string line; std::getline(input, line);) {
    if (step.empty() && line.rfind("M7HD_NATIVE_STEP", 0) == 0) step = line;
    if (cov_line.empty() && line.rfind("M7HD_NATIVE_COV", 0) == 0) cov_line = line;
  }
  if (step.empty() || cov_line.empty()) throw std::runtime_error("missing trace records");
  MatN3 a;
  MatNN expected;
  Vec3 cov;
  load_matrix(a, words(field(step, "a")));
  load_matrix(expected, words(field(cov_line, "term2")));
  load_matrix(cov, words(field(cov_line, "accel_cov")));

  report("eigen", weighted_eigen(a, cov), expected);
  report("materialized", weighted_materialized(a, cov), expected);
  const int orders[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 0, 2},
                            {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
  for (const auto& order : orders) {
    for (int association = 0; association < 3; ++association) {
      const std::string name = "manual" + std::to_string(order[0]) +
                               std::to_string(order[1]) + std::to_string(order[2]) +
                               "_assoc" + std::to_string(association);
      report(name.c_str(), weighted_manual(a, cov, order, association), expected);
      const std::string scaled_name = "scaled" + std::to_string(order[0]) +
                                      std::to_string(order[1]) + std::to_string(order[2]) +
                                      "_assoc" + std::to_string(association);
      report(scaled_name.c_str(),
             weighted_scaled_manual(a, cov, order, association), expected);
    }
  }
  return 0;
}


