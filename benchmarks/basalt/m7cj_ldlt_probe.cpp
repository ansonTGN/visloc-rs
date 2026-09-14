#include <Eigen/Dense>

#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>

static std::uint32_t bits(float value) {
  std::uint32_t result;
  std::memcpy(&result, &value, sizeof(result));
  return result;
}

static void print_matrix(const char* name, const Eigen::Matrix3f& matrix) {
  std::cout << name << "=[";
  for (int row = 0; row < 3; ++row) {
    if (row != 0) std::cout << ";";
    for (int col = 0; col < 3; ++col) {
      if (col != 0) std::cout << ",";
      std::cout << std::hex << std::setw(8) << std::setfill('0')
                << bits(matrix(row, col));
    }
  }
  std::cout << std::dec << std::setfill(' ') << "]\n";
}

static void print_diag(const char* name, const Eigen::Vector3f& vector) {
  std::cout << name << "=[";
  for (int index = 0; index < 3; ++index) {
    if (index != 0) std::cout << ",";
    std::cout << std::hex << std::setw(8) << std::setfill('0')
              << bits(vector(index));
  }
  std::cout << std::dec << std::setfill(' ') << "]\n";
}

int main() {
  Eigen::Matrix3f h;
  const std::uint32_t source[] = {
      0x3fb8c0ff, 0xbf7bab82, 0x3fff4a69,
      0xbf7bab82, 0x4119e7f0, 0x4039dda9,
      0x3fff4a69, 0x4039dda9, 0x42301f20,
  };
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col) {
      float value;
      std::uint32_t bits_value = source[row * 3 + col];
      std::memcpy(&value, &bits_value, sizeof(value));
      h(row, col) = value;
    }

  const auto ldlt = h.ldlt();
  print_matrix("h", h);
  print_matrix("factor", ldlt.matrixLDLT());
  print_diag("d", ldlt.vectorD());
  std::cout << "piv=[" << ldlt.transpositionsP().coeff(0) << ","
            << ldlt.transpositionsP().coeff(1) << ","
            << ldlt.transpositionsP().coeff(2) << "]\n";
  print_matrix("l", ldlt.matrixL());
  print_matrix("u", ldlt.matrixU());

  Eigen::Matrix3f result = Eigen::Matrix3f::Identity();
  result = ldlt.transpositionsP() * result;
  print_matrix("after_p", result);
  ldlt.matrixL().solveInPlace(result);
  print_matrix("after_l", result);
  Eigen::Matrix3f manual_l = Eigen::Matrix3f::Identity();
  manual_l = ldlt.transpositionsP() * manual_l;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      const float b = manual_l(i, j);
      for (int i3 = i + 1; i3 < 3; ++i3)
        manual_l(i3, j) -= b * ldlt.matrixL()(i3, i);
    }
  }
  print_matrix("manual_l", manual_l);
  Eigen::Matrix3f scalar_d = result;
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col) scalar_d(row, col) /= ldlt.vectorD()(row);
  print_matrix("scalar_d", scalar_d);
  for (int row = 0; row < 3; ++row) result.row(row) /= ldlt.vectorD()(row);
  print_matrix("after_d", result);
  ldlt.matrixL().transpose().solveInPlace(result);
  print_matrix("after_lt", result);
  result = ldlt.transpositionsP().transpose() * result;
  print_matrix("after_pt", result);

  Eigen::Matrix3f direct = Eigen::Matrix3f::Identity();
  ldlt.solveInPlace(direct);
  print_matrix("direct", direct);
  return 0;
}
