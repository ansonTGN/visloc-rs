// Exact-bit probe for the pinned Eigen fixed-size 9x9 covariance GEMM.
#include <Eigen/Core>

#include <immintrin.h>

#include <algorithm>
#include <cmath>
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
using Mat9 = Eigen::Matrix<float, 9, 9>;
using Mat93 = Eigen::Matrix<float, 9, 3>;
using Mat93Row = Eigen::Matrix<float, 9, 3, Eigen::RowMajor>;
Mat9 weighted_rowmajor_func(const Mat93& noise, const Eigen::Vector3f& var);

float f(std::uint32_t x) { float y; std::memcpy(&y, &x, 4); return y; }
std::uint32_t b(float x) { std::uint32_t y; std::memcpy(&y, &x, 4); return y; }
std::vector<std::uint32_t> words(const std::string& s) {
  std::vector<std::uint32_t> out; std::stringstream ss(s); std::string x;
  while (std::getline(ss, x, ',')) if (!x.empty()) out.push_back(std::stoul(x, nullptr, 16));
  return out;
}
std::string field(const std::string& line, const char* name) {
  const std::string p = std::string(name) + "=";
  auto begin = line.rfind(" " + p);
  if (line.rfind(p, 0) == 0) begin = 0;
  if (begin == std::string::npos) throw std::runtime_error("missing field");
  if (begin != 0) ++begin;
  begin += p.size(); auto end = line.find(' ', begin);
  return line.substr(begin, end == std::string::npos ? end : end - begin);
}
template <typename M> void load(M& m, const std::vector<std::uint32_t>& v) {
  if (v.size() != static_cast<std::size_t>(m.size())) {
    std::cerr << "bad size got=" << v.size() << " want=" << m.size() << '\n';
    throw std::runtime_error("bad size");
  }
  for (Eigen::Index i = 0; i < m.size(); ++i) m.data()[i] = f(v[i]);
}
int mismatch(const Mat9& x, const Mat9& y, int* first = nullptr) {
  int n = 0, w = -1;
  for (int i = 0; i < 81; ++i) if (b(x.data()[i]) != b(y.data()[i])) { ++n; if (w < 0) w = i; }
  if (first) *first = w; return n;
}
void report(const char* name, const Mat9& x, const Mat9& y) {
  int first = -1; int n = mismatch(x, y, &first);
  std::cout << name << "=" << n << "/81";
  if (first >= 0) std::cout << " first=" << first << " got=" << std::hex << b(x.data()[first])
                            << " want=" << b(y.data()[first]) << std::dec;
  std::cout << '\n';
}

__attribute__((noinline)) Mat9 eigen_product(const Mat9& a, const Mat9& c) { return a * c; }
__attribute__((noinline)) Mat9 eigen_chain(const Mat9& f, const Mat9& c) {
  return f * c * f.transpose();
}
__attribute__((noinline)) Mat9 eigen_sum(const Mat9& f, const Mat9& c,
                                         const Mat93& a, const Mat93& g,
                                         const Eigen::Vector3f& av,
                                         const Eigen::Vector3f& gv) {
  return f * c * f.transpose() + a * av.asDiagonal() * a.transpose() +
         g * gv.asDiagonal() * g.transpose();
}

Mat9 scalar_chain(const Mat9& f, const Mat9& c) {
  Mat9 x;
  for (int col = 0; col < 9; ++col) for (int row = 0; row < 9; ++row) {
    float v = f(row, 0) * c(0, col);
    for (int k = 1; k < 9; ++k) v = std::fma(f(row, k), c(k, col), v);
    x(row, col) = v;
  }
  Mat9 y;
  for (int col = 0; col < 9; ++col) for (int row = 0; row < 9; ++row) {
    float v = x(row, 0) * f(col, 0);
    for (int k = 1; k < 9; ++k) v = std::fma(x(row, k), f(col, k), v);
    y(row, col) = v;
  }
  return y;
}

// The AVX2 Eigen gebp kernel uses two independent accumulators for a 1-packet
// by 4-column block (even and odd k), then adds them before the final tail.
// Its scalar row tail uses the swapped 1x4 path: four two-lane packets are
// reduced as ((C0+C1)+(C2+C3)), followed by the half-packet reduction.
Mat9 eigen_schedule_product(const Mat9& f, const Mat9& c) {
  Mat9 x;
  for (int col = 0; col < 9; ++col) {
    for (int row = 0; row < 8; ++row) {
      float v;
      if (col < 8) {
        float even = std::fma(f(row, 0), c(0, col), 0.0f);
        even = std::fma(f(row, 2), c(2, col), even);
        even = std::fma(f(row, 4), c(4, col), even);
        even = std::fma(f(row, 6), c(6, col), even);
        float odd = std::fma(f(row, 1), c(1, col), 0.0f);
        odd = std::fma(f(row, 3), c(3, col), odd);
        odd = std::fma(f(row, 5), c(5, col), odd);
        odd = std::fma(f(row, 7), c(7, col), odd);
        v = even + odd;
        v = std::fma(f(row, 8), c(8, col), v);
      } else {
        v = std::fma(f(row, 0), c(0, col), 0.0f);
        for (int k = 1; k < 9; ++k) v = std::fma(f(row, k), c(k, col), v);
      }
      x(row, col) = v;
    }
    if (col < 8) {
      float c0 = std::fma(f(8, 0), c(0, col), 0.0f);
      float c1 = std::fma(f(8, 2), c(2, col), 0.0f);
      float c2 = std::fma(f(8, 4), c(4, col), 0.0f);
      float c3 = std::fma(f(8, 6), c(6, col), 0.0f);
      float d0 = std::fma(f(8, 1), c(1, col), 0.0f);
      float d1 = std::fma(f(8, 3), c(3, col), 0.0f);
      float d2 = std::fma(f(8, 5), c(5, col), 0.0f);
      float d3 = std::fma(f(8, 7), c(7, col), 0.0f);
      float v = (c0 + c1) + (c2 + c3);
      v = v + ((d0 + d1) + (d2 + d3));
      x(8, col) = std::fma(f(8, 8), c(8, col), v);
    } else {
      float v = std::fma(f(8, 0), c(0, col), 0.0f);
      for (int k = 1; k < 9; ++k) v = std::fma(f(8, k), c(k, col), v);
      x(8, col) = v;
    }
  }
  return x;
}

Mat9 eigen_schedule_chain(const Mat9& f, const Mat9& c) {
  return eigen_schedule_product(eigen_schedule_product(f, c), f.transpose());
}
// The aliasing assignment path dispatches Eigen's general GEMM kernel.  Its
// 8-row packet body has the same even/odd reduction as the fixed kernel, but
// its scalar ninth-row tail walks k in order with scalar FMAs.
Mat9 general_schedule_product(const Mat9& f, const Mat9& c) {
  Mat9 x;
  for (int col = 0; col < 9; ++col) {
    for (int row = 0; row < 8; ++row) {
      if (col < 8) {
        float even = std::fma(f(row, 0), c(0, col), 0.0f);
        even = std::fma(f(row, 2), c(2, col), even);
        even = std::fma(f(row, 4), c(4, col), even);
        even = std::fma(f(row, 6), c(6, col), even);
        float odd = std::fma(f(row, 1), c(1, col), 0.0f);
        odd = std::fma(f(row, 3), c(3, col), odd);
        odd = std::fma(f(row, 5), c(5, col), odd);
        odd = std::fma(f(row, 7), c(7, col), odd);
        x(row, col) = std::fma(f(row, 8), c(8, col), even + odd);
      } else {
        float value = std::fma(f(row, 0), c(0, col), 0.0f);
        for (int k = 1; k < 9; ++k) value = std::fma(f(row, k), c(k, col), value);
        x(row, col) = value;
      }
    }
    for (int row = 8; row < 9; ++row) {
      float value = std::fma(f(row, 0), c(0, col), 0.0f);
      for (int k = 1; k < 9; ++k) value = std::fma(f(row, k), c(k, col), value);
      x(row, col) = value;
    }
  }
  return x;
}
Mat9 general_schedule_chain(const Mat9& f, const Mat9& c) {
  return general_schedule_product(general_schedule_product(f, c), f.transpose());
}

// AVX2 spelling of the swapped row-tail reduction, used only to audit the
// scalarized schedule above.
float eigen_row_tail_avx(const Mat9& a, const Mat9& b, int col) {
  const auto packet = [&](int k0) {
    return _mm256_setr_ps(b(k0, col), b(k0, col), b(k0, col), b(k0, col),
                          b(k0 + 1, col), b(k0 + 1, col), b(k0 + 1, col), b(k0 + 1, col));
  };
  const auto lhs = [&](int k0) {
    return _mm256_setr_ps(a(8, k0), a(8, k0), a(8, k0), a(8, k0),
                          a(8, k0 + 1), a(8, k0 + 1), a(8, k0 + 1), a(8, k0 + 1));
  };
  // This lane-oriented spelling is deliberately equivalent to the packed
  // four-column kernel for one selected column.
  __m256 c0 = _mm256_fmadd_ps(lhs(0), packet(0), _mm256_setzero_ps());
  __m256 c1 = _mm256_fmadd_ps(lhs(2), packet(2), _mm256_setzero_ps());
  __m256 c2 = _mm256_fmadd_ps(lhs(4), packet(4), _mm256_setzero_ps());
  __m256 c3 = _mm256_fmadd_ps(lhs(6), packet(6), _mm256_setzero_ps());
  __m256 c = _mm256_add_ps(_mm256_add_ps(c0, c1), _mm256_add_ps(c2, c3));
  alignas(32) float cv[8]; _mm256_store_ps(cv, c);
  std::cout << "avxterms=" << std::hex;
  for (float v : cv) std::cout << ::b(v) << ',';
  std::cout << std::dec << '\n';
  __m128 low = _mm256_castps256_ps128(c);
  __m128 high = _mm256_extractf128_ps(c, 1);
  __m128 red = _mm_add_ps(low, high);
  red = _mm_fmadd_ps(_mm_set1_ps(a(8, 8)), _mm_set1_ps(b(8, col)), red);
  alignas(32) float out[4]; _mm_store_ps(out, red);
  return out[0];
}
void dump_row_tail_terms(const Mat9& a, const Mat9& rhs, int col) {
  std::cout << "inputs=";
  for (int k = 0; k < 9; ++k) std::cout << std::hex << ::b(a(8, k)) << "," << ::b(rhs(k, col)) << ";";
  std::cout << std::dec << '\n';
  float c0 = std::fma(a(8, 0), rhs(0, col), 0.0f);
  c0 = std::fma(a(8, 2), rhs(2, col), c0);
  float c1 = std::fma(a(8, 4), rhs(4, col), 0.0f);
  c1 = std::fma(a(8, 6), rhs(6, col), c1);
  float d0 = std::fma(a(8, 1), rhs(1, col), 0.0f);
  d0 = std::fma(a(8, 3), rhs(3, col), d0);
  float d1 = std::fma(a(8, 5), rhs(5, col), 0.0f);
  d1 = std::fma(a(8, 7), rhs(7, col), d1);
  float red = (c0 + c1) + (d0 + d1);
  float out = std::fma(a(8, 8), rhs(8, col), red);
  std::cout << "terms=" << std::hex << ::b(c0) << "," << ::b(c1) << "," << ::b(d0) << "," << ::b(d1)
            << " red=" << ::b(red) << " out=" << ::b(out) << std::dec << '\n';
}
float dot_order(const Mat9& a, const Mat9& c, int row, int col, const int* order) {
  float v = a(row, order[0]) * c(order[0], col);
  for (int i = 1; i < 9; ++i)
    v = std::fma(a(row, order[i]), c(order[i], col), v);
  return v;
}
std::string find_order(const Mat9& a, const Mat9& c, int row, int col, float want) {
  int order[9] = {0,1,2,3,4,5,6,7,8};
  do {
    if (b(dot_order(a, c, row, col, order)) == b(want)) {
      std::ostringstream out;
      for (int i = 0; i < 9; ++i) { if (i) out << ','; out << order[i]; }
      return out.str();
    }
  } while (std::next_permutation(order, order + 9));
  return "none";
}
std::string find_weighted_variant(const Mat93& noise, const Eigen::Vector3f& var, int row, int col, float want) {
  float p[3];
  for (int k = 0; k < 3; ++k) p[k] = std::fma(noise(row, k) * var[k], noise(col, k), 0.0f);
  int order[3] = {0, 1, 2};
  do {
    float f = p[order[0]];
    if (::b(f) == ::b(want)) return "p" + std::to_string(order[0]);
    f = std::fma(noise(row, order[1]) * var[order[1]], noise(col, order[1]), p[order[0]]);
    f = std::fma(noise(row, order[2]) * var[order[2]], noise(col, order[2]), f);
    if (::b(f) == ::b(want)) {
      return "fma" + std::to_string(order[0]) + std::to_string(order[1]) + std::to_string(order[2]);
    }
    float add = p[order[0]] + p[order[1]];
    add = add + p[order[2]];
    if (::b(add) == ::b(want)) {
      return "add" + std::to_string(order[0]) + std::to_string(order[1]) + std::to_string(order[2]);
    }
    add = p[order[0]] + (p[order[1]] + p[order[2]]);
    if (::b(add) == ::b(want)) {
      return "addR" + std::to_string(order[0]) + std::to_string(order[1]) + std::to_string(order[2]);
    }
  } while (std::next_permutation(order, order + 3));
  return "none";
}
std::string find_weighted_accum_variant(const Mat93& noise, const Eigen::Vector3f& var,
                                        int row, int col, float destination, float want) {
  float p[3], q[3], r[3], s[3];
  for (int k = 0; k < 3; ++k) {
    p[k] = std::fma(noise(row, k) * var[k], noise(col, k), 0.0f);
    q[k] = std::fma(noise(row, k), var[k] * noise(col, k), 0.0f);
    r[k] = (noise(row, k) * var[k]) * noise(col, k);
    s[k] = noise(row, k) * (var[k] * noise(col, k));
  }
  int order[3] = {0, 1, 2};
  const float* products[] = {p, q, r, s};
  const char* product_names[] = {"P", "Q", "R", "S"};
  for (int product_kind = 0; product_kind < 4; ++product_kind) {
    const float* product = products[product_kind];
    do {
      float term = product[order[0]];
      if (product_kind == 1) {
        term = std::fma(noise(row, order[1]), var[order[1]] * noise(col, order[1]), term);
        term = std::fma(noise(row, order[2]), var[order[2]] * noise(col, order[2]), term);
      } else {
        term = std::fma(noise(row, order[1]) * var[order[1]], noise(col, order[1]), term);
        term = std::fma(noise(row, order[2]) * var[order[2]], noise(col, order[2]), term);
      }
      if (::b(destination + term) == ::b(want)) {
        return std::string(product_names[product_kind]) + "fma" + std::to_string(order[0]) +
               std::to_string(order[1]) + std::to_string(order[2]) + "+";
      }
      if (::b(std::fma(term, 1.0f, destination)) == ::b(want)) {
        return std::string(product_names[product_kind]) + "fma" + std::to_string(order[0]) +
               std::to_string(order[1]) + std::to_string(order[2]) + "*";
      }
      float add = product[order[0]] + product[order[1]];
      add = add + product[order[2]];
      if (::b(destination + add) == ::b(want)) {
        return std::string(product_names[product_kind]) + "add" + std::to_string(order[0]) +
               std::to_string(order[1]) + std::to_string(order[2]) + "+";
      }
      if (::b(std::fma(add, 1.0f, destination)) == ::b(want)) {
        return std::string(product_names[product_kind]) + "add" + std::to_string(order[0]) +
               std::to_string(order[1]) + std::to_string(order[2]) + "*";
      }
      add = product[order[0]] + (product[order[1]] + product[order[2]]);
      if (::b(destination + add) == ::b(want)) {
        return std::string(product_names[product_kind]) + "addR" + std::to_string(order[0]) +
               std::to_string(order[1]) + std::to_string(order[2]) + "+";
      }
      if (::b(std::fma(add, 1.0f, destination)) == ::b(want)) {
        return std::string(product_names[product_kind]) + "addR" + std::to_string(order[0]) +
               std::to_string(order[1]) + std::to_string(order[2]) + "*";
      }
    } while (std::next_permutation(order, order + 3));
    order[0] = 0; order[1] = 1; order[2] = 2;
  }
  return "none";
}
Mat9 weighted_scalar_product(const Mat93& noise, const Eigen::Vector3f& var) {
  Mat9 out;
  for (int col = 0; col < 9; ++col) for (int row = 0; row < 9; ++row) {
    float v = std::fma(noise(row, 0) * var[0], noise(col, 0), 0.0f);
    v = std::fma(noise(row, 1) * var[1], noise(col, 1), v);
    v = std::fma(noise(row, 2) * var[2], noise(col, 2), v);
    out(row, col) = v;
  }
  return out;
}
Mat9 weighted_add_product(const Mat93& noise, const Eigen::Vector3f& var) {
  Mat9 out;
  for (int col = 0; col < 9; ++col) for (int row = 0; row < 9; ++row) {
    float p0 = noise(row, 0) * var[0] * noise(col, 0);
    float p1 = noise(row, 1) * var[1] * noise(col, 1);
    float p2 = noise(row, 2) * var[2] * noise(col, 2);
    out(row, col) = (p0 + p1) + p2;
  }
  return out;
}
Mat9 weighted_add_product_fma_products(const Mat93& noise, const Eigen::Vector3f& var) {
  Mat9 out;
  for (int col = 0; col < 9; ++col) for (int row = 0; row < 9; ++row) {
    float p0 = std::fma(noise(row, 0) * var[0], noise(col, 0), 0.0f);
    float p1 = std::fma(noise(row, 1) * var[1], noise(col, 1), 0.0f);
    float p2 = std::fma(noise(row, 2) * var[2], noise(col, 2), 0.0f);
    out(row, col) = (p0 + p1) + p2;
  }
  return out;
}
__attribute__((noinline)) Mat9 zero_add_func(const Mat93& noise, const Eigen::Vector3f& var) {
  Mat9 out = Mat9::Zero();
  out += noise * var.asDiagonal() * noise.transpose();
  return out;
}
__attribute__((noinline)) Mat9 zero_assign_func(const Mat93& noise, const Eigen::Vector3f& var) {
  Mat9 out;
  out = noise * var.asDiagonal() * noise.transpose();
  return out;
}
Mat9 weighted_scalar_accumulate(Mat9 out, const Mat93& noise, const Eigen::Vector3f& var) {
  for (int col = 0; col < 9; ++col) for (int row = 0; row < 9; ++row) {
    float value = std::fma(noise(row, 0) * var[0], noise(col, 0), 0.0f);
    value = std::fma(noise(row, 1) * var[1], noise(col, 1), value);
    value = std::fma(noise(row, 2) * var[2], noise(col, 2), value);
    if (col < 8) out(row, col) = std::fma(value, 1.0f, out(row, col));
    else out(row, col) += value;
  }
  return out;
}
Mat9 weighted_packet_accumulate(Mat9 out, const Mat93& noise, const Eigen::Vector3f& var) {
  const Mat9 term = zero_assign_func(noise, var);
  for (int col = 0; col < 9; ++col) for (int row = 0; row < 9; ++row) {
    if (col < 8) out(row, col) = std::fma(term(row, col), 1.0f, out(row, col));
    else out(row, col) += term(row, col);
  }
  return out;
}
Mat9 weighted_direct_accumulate(Mat9 out, const Mat93& noise, const Eigen::Vector3f& var) {
  for (int col = 0; col < 9; ++col) for (int row = 0; row < 9; ++row) {
    out(row, col) = std::fma(noise(row, 0) * var[0], noise(col, 0), out(row, col));
    out(row, col) = std::fma(noise(row, 1) * var[1], noise(col, 1), out(row, col));
    out(row, col) = std::fma(noise(row, 2) * var[2], noise(col, 2), out(row, col));
  }
  return out;
}
Mat9 weighted_eigen_term(const Mat93& noise, const Eigen::Vector3f& var) {
  Mat93 scaled;
  for (int k = 0; k < 3; ++k) for (int row = 0; row < 9; ++row)
    scaled(row, k) = noise(row, k) * var[k];
  Mat9 out;
  for (int col = 0; col < 9; ++col) for (int row = 0; row < 9; ++row) {
    float value = std::fma(scaled(row, 0), noise(col, 0), 0.0f);
    if (row == 8 && col < 8) {
      float p0 = std::fma(scaled(row, 0), noise(col, 0), 0.0f);
      float p2 = std::fma(scaled(row, 1), noise(col, 1), 0.0f);
      value = p0 + p2;
      value = std::fma(scaled(row, 2), noise(col, 2), value);
    } else {
      value = std::fma(scaled(row, 1), noise(col, 1), value);
      value = std::fma(scaled(row, 2), noise(col, 2), value);
    }
    out(row, col) = value;
  }
  return out;
}
Mat9 weighted_eigen_accumulate(Mat9 out, const Mat93& noise, const Eigen::Vector3f& var) {
  const Mat9 term = weighted_eigen_term(noise, var);
  for (int col = 0; col < 9; ++col) for (int row = 0; row < 9; ++row)
    out(row, col) = std::fma(term(row, col), 1.0f, out(row, col));
  return out;
}
Mat9 general_weighted_term(const Mat93& noise, const Eigen::Vector3f& var) {
  Mat93 scaled;
  for (int k = 0; k < 3; ++k) for (int row = 0; row < 9; ++row)
    scaled(row, k) = noise(row, k) * var[k];
  Mat9 out;
  for (int col = 0; col < 9; ++col) for (int row = 0; row < 9; ++row) {
    float value = std::fma(scaled(row, 0), noise(col, 0), 0.0f);
    value = std::fma(scaled(row, 1), noise(col, 1), value);
    value = std::fma(scaled(row, 2), noise(col, 2), value);
    out(row, col) = value;
  }
  return out;
}
Mat9 general_weighted_term_addcol(const Mat93& noise, const Eigen::Vector3f& var) {
  Mat93 scaled;
  for (int k = 0; k < 3; ++k) for (int row = 0; row < 9; ++row)
    scaled(row, k) = noise(row, k) * var[k];
  Mat9 out;
  for (int col = 0; col < 9; ++col) for (int row = 0; row < 9; ++row) {
    if (col == 8) {
      const float p0 = std::fma(scaled(row, 0), noise(col, 0), 0.0f);
      const float p1 = std::fma(scaled(row, 1), noise(col, 1), 0.0f);
      const float p2 = std::fma(scaled(row, 2), noise(col, 2), 0.0f);
      out(row, col) = (p0 + p1) + p2;
    } else {
      float value = std::fma(scaled(row, 0), noise(col, 0), 0.0f);
      value = std::fma(scaled(row, 1), noise(col, 1), value);
      value = std::fma(scaled(row, 2), noise(col, 2), value);
      out(row, col) = value;
    }
  }
  return out;
}
Mat9 general_weighted_term_addcol_q(const Mat93& noise, const Eigen::Vector3f& var) {
  Mat9 out;
  for (int col = 0; col < 9; ++col) for (int row = 0; row < 9; ++row) {
    if (col == 8) {
      const float p0 = std::fma(noise(row, 0), var[0] * noise(col, 0), 0.0f);
      const float p1 = std::fma(noise(row, 1), var[1] * noise(col, 1), 0.0f);
      const float p2 = std::fma(noise(row, 2), var[2] * noise(col, 2), 0.0f);
      out(row, col) = (p0 + p1) + p2;
    } else {
      float value = std::fma(noise(row, 0) * var[0], noise(col, 0), 0.0f);
      value = std::fma(noise(row, 1) * var[1], noise(col, 1), value);
      value = std::fma(noise(row, 2) * var[2], noise(col, 2), value);
      out(row, col) = value;
    }
  }
  return out;
}
Mat9 general_weighted_accumulate(Mat9 out, const Mat93& noise, const Eigen::Vector3f& var) {
  const Mat9 term = general_weighted_term(noise, var);
  for (int col = 0; col < 9; ++col) for (int row = 0; row < 9; ++row)
    out(row, col) += term(row, col);
  return out;
}
Mat9 weighted_plain_packet_accumulate(Mat9 out, const Mat93& noise, const Eigen::Vector3f& var) {
  const Mat9 term = zero_assign_func(noise, var);
  for (int col = 0; col < 9; ++col) for (int row = 0; row < 9; ++row)
    out(row, col) += term(row, col);
  return out;
}
Mat9 weighted_plain_eigen_accumulate(Mat9 out, const Mat93& noise, const Eigen::Vector3f& var) {
  const Mat9 term = weighted_eigen_term(noise, var);
  for (int col = 0; col < 9; ++col) for (int row = 0; row < 9; ++row)
    out(row, col) += term(row, col);
  return out;
}
Mat9 weighted_rowmajor_accumulate(Mat9 out, const Mat93& noise, const Eigen::Vector3f& var) {
  const Mat9 term = weighted_rowmajor_func(noise, var);
  for (int col = 0; col < 9; ++col) for (int row = 0; row < 9; ++row)
    out(row, col) += term(row, col);
  return out;
}
Mat9 weighted_zeroadd_accumulate(Mat9 out, const Mat93& noise, const Eigen::Vector3f& var) {
  const Mat9 term = zero_add_func(noise, var);
  for (int col = 0; col < 9; ++col) for (int row = 0; row < 9; ++row)
    out(row, col) += term(row, col);
  return out;
}
Mat9 weighted_accumulate_term_fma_addcol(const Mat93& noise, const Eigen::Vector3f& var) {
  Mat9 out;
  for (int col = 0; col < 9; ++col) for (int row = 0; row < 9; ++row) {
    float p0 = std::fma(noise(row, 0) * var[0], noise(col, 0), 0.0f);
    float p1 = std::fma(noise(row, 1) * var[1], noise(col, 1), 0.0f);
    float p2 = std::fma(noise(row, 2) * var[2], noise(col, 2), 0.0f);
    if (col == 8) out(row, col) = (p0 + p1) + p2;
    else {
      float v = p0;
      v = std::fma(noise(row, 1) * var[1], noise(col, 1), v);
      v = std::fma(noise(row, 2) * var[2], noise(col, 2), v);
      out(row, col) = v;
    }
  }
  return out;
}
Mat9 weighted_accumulate_fma_addcol(Mat9 out, const Mat93& noise, const Eigen::Vector3f& var) {
  const Mat9 term = weighted_accumulate_term_fma_addcol(noise, var);
  for (int col = 0; col < 9; ++col) for (int row = 0; row < 9; ++row)
    out(row, col) += term(row, col);
  return out;
}
Mat9 weighted_accumulate_fma_addcol_fused(Mat9 out, const Mat93& noise, const Eigen::Vector3f& var) {
  const Mat9 term = weighted_accumulate_term_fma_addcol(noise, var);
  for (int col = 0; col < 9; ++col) for (int row = 0; row < 9; ++row)
    out(row, col) = std::fma(term(row, col), 1.0f, out(row, col));
  return out;
}
// The covariance expression's inner diagonal product is lowered by Eigen to
// a materialized column-major 9x3 operand before the outer 9x3*3x9 GEMM.
// Keep the scale loop explicit so this can be compared independently from
// the nested expression used by zero_assign_func.
__attribute__((noinline)) Mat9 manual_outer_loop(const Mat93& noise,
                                                  const Eigen::Vector3f& var) {
  Mat93 scaled;
  for (int k = 0; k < 3; ++k)
    for (int row = 0; row < 9; ++row)
      scaled(row, k) = noise(row, k) * var[k];
  return scaled * noise.transpose();
}
__attribute__((noinline)) Mat9 manual_outer_expr(const Mat93& noise,
                                                  const Eigen::Vector3f& var) {
  const Mat93 scaled = noise * var.asDiagonal();
  return scaled * noise.transpose();
}
__attribute__((noinline)) Mat9 manual_outer_direct(const Mat93& noise,
                                                    const Eigen::Vector3f& var) {
  Mat9 out;
  Mat93 scaled;
  for (int k = 0; k < 3; ++k)
    for (int row = 0; row < 9; ++row)
      scaled(row, k) = noise(row, k) * var[k];
  out.noalias() = scaled * noise.transpose();
  return out;
}
__attribute__((noinline)) Mat9 manual_accumulate(Mat9 out, const Mat93& noise,
                                                   const Eigen::Vector3f& var) {
  Mat93 scaled;
  for (int k = 0; k < 3; ++k)
    for (int row = 0; row < 9; ++row)
      scaled(row, k) = noise(row, k) * var[k];
  out += scaled * noise.transpose();
  return out;
}
__attribute__((noinline)) Mat9 expression_accumulate(Mat9 out, const Mat93& noise,
                                                       const Eigen::Vector3f& var) {
  out += noise * var.asDiagonal() * noise.transpose();
  return out;
}
__attribute__((noinline)) Mat9 expression_zero(const Mat93& noise,
                                                 const Eigen::Vector3f& var) {
  Mat9 out = Mat9::Zero();
  out += noise * var.asDiagonal() * noise.transpose();
  return out;
}
__attribute__((noinline)) Mat9 sequential_prefix(const Mat9& f, const Mat9& c,
                                                   const Mat93& a,
                                                   const Eigen::Vector3f& av) {
  Mat9 out = c;
  out = f * out * f.transpose();
  out += a * av.asDiagonal() * a.transpose();
  return out;
}
__attribute__((noinline)) Mat9 sequential_chain_only(const Mat9& f, const Mat9& c) {
  Mat9 out = c;
  out = f * out * f.transpose();
  return out;
}
__attribute__((noinline)) Mat9 right_associated_chain(const Mat9& f, const Mat9& c) {
  Mat9 out = c;
  out = f * (out * f.transpose());
  return out;
}
__attribute__((noinline)) Mat9 right_associated_chain_noalias(const Mat9& f, const Mat9& c) {
  Mat9 out = c;
  out.noalias() = f * (out * f.transpose());
  return out;
}
__attribute__((noinline)) Mat9 direct_gemm_chain(const Mat9& f, const Mat9& c) {
  Mat9 first;
  first.noalias() = f * c;
  Mat9 out;
  out.noalias() = first * f.transpose();
  return out;
}
__attribute__((noinline)) Mat9 direct_gemm_chain_alias(const Mat9& f, const Mat9& c) {
  Mat9 out = c;
  Mat9 first;
  first.noalias() = f * out;
  out.noalias() = first * f.transpose();
  return out;
}
__attribute__((noinline)) Mat9 inplace_first_product(const Mat9& f, const Mat9& c) {
  Mat9 out = c;
  out = f * out;
  return out;
}
__attribute__((noinline)) Mat9 direct_first_product(const Mat9& f, const Mat9& c) {
  Mat9 out;
  out.noalias() = f * c;
  return out;
}
__attribute__((noinline)) Mat9 inplace_chain_from_first(const Mat9& f, const Mat9& c) {
  Mat9 out = c;
  out = f * out;
  out = out * f.transpose();
  return out;
}
__attribute__((noinline)) Mat9 sequential_g_only(const Mat9& f, const Mat9& c,
                                                   const Mat93& g,
                                                   const Eigen::Vector3f& gv) {
  Mat9 out = f * c * f.transpose();
  out += g * gv.asDiagonal() * g.transpose();
  return out;
}
__attribute__((noinline)) Mat9 sequential_func(const Mat9& f, const Mat9& c,
                                               const Mat93& a, const Mat93& g,
                                               const Eigen::Vector3f& av,
                                               const Eigen::Vector3f& gv) {
  Mat9 out = c;
  out = f * out * f.transpose();
  out += a * av.asDiagonal() * a.transpose();
  out += g * gv.asDiagonal() * g.transpose();
  return out;
}
__attribute__((noinline)) Mat9 weighted_rowmajor_func(const Mat93& noise,
                                                      const Eigen::Vector3f& var) {
  const Mat93Row scaled = noise * var.asDiagonal();
  return scaled * noise.transpose();
}
}

int main(int argc, char** argv) {
  std::ifstream in(argc > 1 ? argv[1] : "target/m7hd_native_intermediate.jsonl");
  if (!in) throw std::runtime_error("cannot open trace");
  std::string step_line, cov_line; std::vector<std::string> steps, covs;
  for (std::string line; std::getline(in, line);) {
    if (line.rfind("M7HD_NATIVE_STEP", 0) == 0) steps.push_back(line);
    if (line.rfind("M7HD_NATIVE_COV", 0) == 0) covs.push_back(line);
  }
  std::cout << "records=" << steps.size() << "/" << covs.size() << '\n';
  for (std::size_t i = 0; i < covs.size(); ++i) {
    Mat9 F, C, want;
    auto Fw = words(field(steps[i], "f"));
    auto Cw = words(field(covs[i], "cov_before"));
    auto Ww = words(field(covs[i], "term1"));
    std::cerr << "p=" << i + 1 << " F=" << Fw.size() << " C=" << Cw.size() << " W=" << Ww.size() << '\n';
    load(F, Fw);
    load(C, Cw);
    load(want, Ww);
    report((std::string("p") + std::to_string(i + 1) + " eigen_chain").c_str(), eigen_chain(F, C), want);
    report((std::string("p") + std::to_string(i + 1) + " scalar_chain").c_str(), scalar_chain(F, C), want);
    report((std::string("p") + std::to_string(i + 1) + " eigen_schedule_chain").c_str(),
           eigen_schedule_chain(F, C), want);
    report((std::string("p") + std::to_string(i + 1) + " general_schedule_chain").c_str(),
           general_schedule_chain(F, C), want);
    report((std::string("p") + std::to_string(i + 1) + " general_vs_sequential_chain").c_str(),
           general_schedule_chain(F, C), sequential_chain_only(F, C));
    report((std::string("p") + std::to_string(i + 1) + " sequential_chain_only").c_str(),
           sequential_chain_only(F, C), want);
    report((std::string("p") + std::to_string(i + 1) + " right_associated_chain").c_str(),
           right_associated_chain(F, C), want);
    report((std::string("p") + std::to_string(i + 1) + " right_associated_chain_noalias").c_str(),
           right_associated_chain_noalias(F, C), want);
    report((std::string("p") + std::to_string(i + 1) + " direct_gemm_chain").c_str(),
           direct_gemm_chain(F, C), want);
    report((std::string("p") + std::to_string(i + 1) + " direct_gemm_chain_alias").c_str(),
           direct_gemm_chain_alias(F, C), want);
    report((std::string("p") + std::to_string(i + 1) + " inplace_first_vs_direct_first").c_str(),
           inplace_first_product(F, C), direct_first_product(F, C));
    report((std::string("p") + std::to_string(i + 1) + " inplace_chain_from_first").c_str(),
           inplace_chain_from_first(F, C), want);
    if (i == 4 || i == 8) {
      const Mat9 d1 = eigen_product(F, C);
      const Mat9 s1 = eigen_schedule_product(F, C);
      const Mat9 d2 = eigen_product(d1, F.transpose());
      const Mat9 s2 = eigen_product(s1, F.transpose());
      const Mat9 ss2 = eigen_schedule_product(s1, F.transpose());
      report((std::string("p") + std::to_string(i + 1) + " schedule_product_vs_eigen").c_str(), s1, d1);
      report((std::string("p") + std::to_string(i + 1) + " schedule_second_vs_eigen").c_str(), s2, d2);
      report((std::string("p") + std::to_string(i + 1) + " schedule_kernel_second_vs_eigen").c_str(), ss2, d2);
      const int lane = 17;
      std::cout << "p" << i + 1 << " lane17 first direct/sched=" << std::hex << b(d1.data()[lane])
                << "/" << b(s1.data()[lane]) << " second direct/sched=" << b(d2.data()[lane])
                << "/" << b(s2.data()[lane]) << std::dec << '\n';
      std::cout << "p" << i + 1 << " lane17 avx-tail=" << std::hex
                << b(eigen_row_tail_avx(s1, F.transpose(), 1)) << std::dec << '\n';
      dump_row_tail_terms(s1, F.transpose(), 1);
    }
    if (i == 1) {
      const Mat9 direct = eigen_chain(F, C);
      for (int lane = 0; lane < 81; ++lane) {
        const int row = lane % 9, col = lane / 9;
        std::cout << "p2_order lane=" << lane << " row=" << row << " col=" << col
                  << " " << find_order(F, C, row, col, direct.data()[lane]) << '\n';
      }
    }
    {
      Mat93 A, G; Eigen::Vector3f av, gv; Mat9 native_final, native_term2, native_term3;
      load(A, words(field(steps[i], "a"))); load(G, words(field(steps[i], "g")));
      const auto avw = words(field(covs[i], "accel_cov")); const auto gvw = words(field(covs[i], "gyro_cov"));
      for (int k = 0; k < 3; ++k) { av[k] = f(avw[k]); gv[k] = f(gvw[k]); }
      load(native_final, words(field(covs[i], "cov")));
      load(native_term2, words(field(covs[i], "term2")));
      load(native_term3, words(field(covs[i], "term3")));
      report((std::string("p") + std::to_string(i + 1) + " sequential_func").c_str(),
             sequential_func(F, C, A, G, av, gv), native_final);
      report((std::string("p") + std::to_string(i + 1) + " rowmajor_term2").c_str(),
             weighted_rowmajor_func(A, av), native_term2);
      report((std::string("p") + std::to_string(i + 1) + " rowmajor_term3").c_str(),
             weighted_rowmajor_func(G, gv), native_term3);
      report((std::string("p") + std::to_string(i + 1) + " weighted_eigen_term2").c_str(),
             weighted_eigen_term(A, av), native_term2);
      report((std::string("p") + std::to_string(i + 1) + " weighted_eigen_term3").c_str(),
             weighted_eigen_term(G, gv), native_term3);
      report((std::string("p") + std::to_string(i + 1) + " general_weighted_term2").c_str(),
             general_weighted_term(A, av), zero_assign_func(A, av));
      report((std::string("p") + std::to_string(i + 1) + " general_weighted_term3").c_str(),
             general_weighted_term(G, gv), zero_assign_func(G, gv));
      report((std::string("p") + std::to_string(i + 1) + " general_addcol_term2").c_str(),
             general_weighted_term_addcol(A, av), zero_assign_func(A, av));
      report((std::string("p") + std::to_string(i + 1) + " general_addcol_term3").c_str(),
             general_weighted_term_addcol(G, gv), zero_assign_func(G, gv));
      report((std::string("p") + std::to_string(i + 1) + " general_addcol_q_term2").c_str(),
             general_weighted_term_addcol_q(A, av), zero_assign_func(A, av));
      report((std::string("p") + std::to_string(i + 1) + " general_addcol_q_term3").c_str(),
             general_weighted_term_addcol_q(G, gv), zero_assign_func(G, gv));
      if (i == 0) {
        const Mat9 dbg = general_weighted_term_addcol(A, av);
        const float dp0 = std::fma(A(0, 0) * av[0], A(8, 0), 0.0f);
        const float dp1 = std::fma(A(0, 1) * av[1], A(8, 1), 0.0f);
        const float dp2 = std::fma(A(0, 2) * av[2], A(8, 2), 0.0f);
        std::cout << "p1 addcol_row0col8=" << std::hex << ::b(dbg(0, 8))
                  << " p=" << ::b(dp0) << "," << ::b(dp1) << "," << ::b(dp2)
                  << " sum=" << ::b((dp0 + dp1) + dp2) << std::dec << '\n';
      }
      report((std::string("p") + std::to_string(i + 1) + " addcol_term2").c_str(),
             weighted_accumulate_term_fma_addcol(A, av), native_term2);
      report((std::string("p") + std::to_string(i + 1) + " addcol_term3").c_str(),
             weighted_accumulate_term_fma_addcol(G, gv), native_term3);
      report((std::string("p") + std::to_string(i + 1) + " manual_loop_term2").c_str(),
             manual_outer_loop(A, av), native_term2);
      report((std::string("p") + std::to_string(i + 1) + " manual_loop_term3").c_str(),
             manual_outer_loop(G, gv), native_term3);
      report((std::string("p") + std::to_string(i + 1) + " manual_expr_term2").c_str(),
             manual_outer_expr(A, av), native_term2);
      report((std::string("p") + std::to_string(i + 1) + " manual_expr_term3").c_str(),
             manual_outer_expr(G, gv), native_term3);
      report((std::string("p") + std::to_string(i + 1) + " manual_direct_term2").c_str(),
             manual_outer_direct(A, av), native_term2);
      report((std::string("p") + std::to_string(i + 1) + " manual_direct_term3").c_str(),
             manual_outer_direct(G, gv), native_term3);
      report((std::string("p") + std::to_string(i + 1) + " expr_zero_term2").c_str(),
             expression_zero(A, av), native_term2);
      report((std::string("p") + std::to_string(i + 1) + " expr_zero_term3").c_str(),
             expression_zero(G, gv), native_term3);
      Mat9 zero2 = Mat9::Zero(); zero2 += A * av.asDiagonal() * A.transpose();
      Mat9 zero3 = Mat9::Zero(); zero3 += G * gv.asDiagonal() * G.transpose();
      Mat9 zero2_noalias = Mat9::Zero(); zero2_noalias.noalias() += A * av.asDiagonal() * A.transpose();
      Mat9 zero2_right = Mat9::Zero(); zero2_right += A * (av.asDiagonal() * A.transpose());
      Mat9 zero2_lazy = Mat9::Zero(); zero2_lazy += (A * av.asDiagonal()).lazyProduct(A.transpose());
      const Mat9 scalar2 = weighted_scalar_product(A, av);
      const Mat9 scalar3 = weighted_scalar_product(G, gv);
      const Mat9 add2 = weighted_add_product_fma_products(A, av);
      const Mat9 add3 = weighted_add_product_fma_products(G, gv);
      report((std::string("p") + std::to_string(i + 1) + " zero_add_term2").c_str(), zero2, native_term2);
      report((std::string("p") + std::to_string(i + 1) + " zero_add_term3").c_str(), zero3, native_term3);
      report((std::string("p") + std::to_string(i + 1) + " zero_noalias_term2").c_str(), zero2_noalias, native_term2);
      report((std::string("p") + std::to_string(i + 1) + " zero_right_term2").c_str(), zero2_right, native_term2);
      report((std::string("p") + std::to_string(i + 1) + " zero_lazy_term2").c_str(), zero2_lazy, native_term2);
      report((std::string("p") + std::to_string(i + 1) + " zero_add_vs_scalar2").c_str(), zero2, scalar2);
      report((std::string("p") + std::to_string(i + 1) + " zero_add_vs_scalar3").c_str(), zero3, scalar3);
      report((std::string("p") + std::to_string(i + 1) + " zero_add_vs_add2").c_str(), zero2, add2);
      report((std::string("p") + std::to_string(i + 1) + " zero_add_vs_add3").c_str(), zero3, add3);
      Mat9 scalar_sequential = eigen_chain(F, C);
      scalar_sequential = weighted_scalar_accumulate(scalar_sequential, A, av);
      scalar_sequential = weighted_scalar_accumulate(scalar_sequential, G, gv);
      Mat9 packet_sequential = eigen_chain(F, C);
      packet_sequential = weighted_packet_accumulate(packet_sequential, A, av);
      packet_sequential = weighted_packet_accumulate(packet_sequential, G, gv);
      Mat9 packet_after2 = eigen_chain(F, C);
      packet_after2 = weighted_packet_accumulate(packet_after2, A, av);
      Mat9 actual_after2 = eigen_chain(F, C);
      actual_after2 += A * av.asDiagonal() * A.transpose();
      report((std::string("p") + std::to_string(i + 1) + " packet_after2").c_str(), packet_after2, actual_after2);
      const Mat9 seq_prefix = sequential_prefix(F, C, A, av);
      report((std::string("p") + std::to_string(i + 1) + " seq_prefix_vs_actual_after2").c_str(),
             seq_prefix, actual_after2);
      const Mat9 seq_gonly = sequential_g_only(F, C, G, gv);
      const Mat9 expr_gonly = expression_accumulate(eigen_chain(F, C), G, gv);
      report((std::string("p") + std::to_string(i + 1) + " seq_gonly_vs_expr_gonly").c_str(),
             seq_gonly, expr_gonly);
      for (int lane = 0; lane < 81; ++lane) {
        if (::b(packet_after2.data()[lane]) != ::b(actual_after2.data()[lane])) {
          const int row = lane % 9, col = lane / 9;
          std::cout << "p" << i + 1 << " after2_variant lane=" << lane
                    << " row=" << row << " col=" << col
                    << " dest=" << std::hex << ::b(eigen_chain(F, C).data()[lane])
                    << " got=" << ::b(packet_after2.data()[lane])
                    << " want=" << ::b(actual_after2.data()[lane])
                    << " variant=" << find_weighted_accum_variant(A, av, row, col,
                                                                       eigen_chain(F, C).data()[lane],
                                                                       actual_after2.data()[lane])
                    << std::dec << '\n';
        }
      }
      Mat9 packet_after3 = packet_after2;
      packet_after3 = weighted_packet_accumulate(packet_after3, G, gv);
      if (i == 2 || i == 3 || i == 4 || i == 5 || i == 8) {
        const Mat9 z3 = zero_assign_func(G, gv);
        std::cout << "p" << i + 1 << " row8_g=";
        for (int col = 0; col < 9; ++col) {
          const int lane = 8 + 9 * col;
          std::cout << col << ":" << std::hex
                    << ::b(packet_after2.data()[lane]) << "/"
                    << ::b(z3.data()[lane]) << "/"
                    << ::b(native_term3.data()[lane]) << "/"
                    << ::b(packet_after3.data()[lane]) << "/"
                    << ::b(native_final.data()[lane]) << std::dec << ",";
        }
        std::cout << '\n';
      }
      Mat9 packet_a_scalar_g = eigen_chain(F, C);
      packet_a_scalar_g = weighted_packet_accumulate(packet_a_scalar_g, A, av);
      packet_a_scalar_g = weighted_scalar_accumulate(packet_a_scalar_g, G, gv);
      Mat9 scalar_a_packet_g = eigen_chain(F, C);
      scalar_a_packet_g = weighted_scalar_accumulate(scalar_a_packet_g, A, av);
      scalar_a_packet_g = weighted_packet_accumulate(scalar_a_packet_g, G, gv);
      Mat9 eigen_sequential = eigen_chain(F, C);
      eigen_sequential = weighted_eigen_accumulate(eigen_sequential, A, av);
      eigen_sequential = weighted_eigen_accumulate(eigen_sequential, G, gv);
      Mat9 packetA_eigenG = eigen_chain(F, C);
      packetA_eigenG = weighted_packet_accumulate(packetA_eigenG, A, av);
      packetA_eigenG = weighted_eigen_accumulate(packetA_eigenG, G, gv);
      Mat9 plain_packet = eigen_chain(F, C);
      plain_packet = weighted_plain_packet_accumulate(plain_packet, A, av);
      plain_packet = weighted_plain_packet_accumulate(plain_packet, G, gv);
      Mat9 plain_eigen = eigen_chain(F, C);
      plain_eigen = weighted_plain_eigen_accumulate(plain_eigen, A, av);
      plain_eigen = weighted_plain_eigen_accumulate(plain_eigen, G, gv);
      Mat9 plainA_eigenG = eigen_chain(F, C);
      plainA_eigenG = weighted_plain_packet_accumulate(plainA_eigenG, A, av);
      plainA_eigenG = weighted_plain_eigen_accumulate(plainA_eigenG, G, gv);
      Mat9 rowmajor_plain = eigen_chain(F, C);
      rowmajor_plain = weighted_rowmajor_accumulate(rowmajor_plain, A, av);
      rowmajor_plain = weighted_rowmajor_accumulate(rowmajor_plain, G, gv);
      Mat9 zeroadd_plain = eigen_chain(F, C);
      zeroadd_plain = weighted_zeroadd_accumulate(zeroadd_plain, A, av);
      zeroadd_plain = weighted_zeroadd_accumulate(zeroadd_plain, G, gv);
      Mat9 addcol_plain = eigen_chain(F, C);
      addcol_plain = weighted_accumulate_fma_addcol(addcol_plain, A, av);
      addcol_plain = weighted_accumulate_fma_addcol(addcol_plain, G, gv);
      Mat9 addcol_fused = eigen_chain(F, C);
      addcol_fused = weighted_accumulate_fma_addcol_fused(addcol_fused, A, av);
      addcol_fused = weighted_accumulate_fma_addcol_fused(addcol_fused, G, gv);
      Mat9 manual_seq = eigen_chain(F, C);
      manual_seq = manual_accumulate(manual_seq, A, av);
      manual_seq = manual_accumulate(manual_seq, G, gv);
      Mat9 expr_seq = eigen_chain(F, C);
      expr_seq = expression_accumulate(expr_seq, A, av);
      expr_seq = expression_accumulate(expr_seq, G, gv);
      Mat9 general_expr_seq = general_schedule_chain(F, C);
      general_expr_seq = expression_accumulate(general_expr_seq, A, av);
      general_expr_seq = expression_accumulate(general_expr_seq, G, gv);
      Mat9 general_packet_seq = general_schedule_chain(F, C);
      general_packet_seq = weighted_packet_accumulate(general_packet_seq, A, av);
      general_packet_seq = weighted_packet_accumulate(general_packet_seq, G, gv);
      Mat9 general_scalar_seq = general_schedule_chain(F, C);
      general_scalar_seq = general_weighted_accumulate(general_scalar_seq, A, av);
      general_scalar_seq = general_weighted_accumulate(general_scalar_seq, G, gv);
      Mat9 general_addcol_seq = general_schedule_chain(F, C);
      const Mat9 addcol_a = general_weighted_term_addcol(A, av);
      const Mat9 addcol_g = general_weighted_term_addcol(G, gv);
      for (int col = 0; col < 9; ++col) for (int row = 0; row < 9; ++row) {
        general_addcol_seq(row, col) += addcol_a(row, col);
        general_addcol_seq(row, col) += addcol_g(row, col);
      }
      Mat9 direct_sequential = eigen_chain(F, C);
      direct_sequential = weighted_direct_accumulate(direct_sequential, A, av);
      direct_sequential = weighted_direct_accumulate(direct_sequential, G, gv);
      report((std::string("p") + std::to_string(i + 1) + " scalar_sequential").c_str(), scalar_sequential, native_final);
      report((std::string("p") + std::to_string(i + 1) + " packet_sequential").c_str(), packet_sequential, native_final);
      report((std::string("p") + std::to_string(i + 1) + " packet_after3").c_str(), packet_after3, native_final);
      for (int lane = 0; lane < 81; ++lane) {
        if (::b(packet_after3.data()[lane]) != ::b(native_final.data()[lane])) {
          const int row = lane % 9, col = lane / 9;
          std::cout << "p" << i + 1 << " after3_variant lane=" << lane
                    << " row=" << row << " col=" << col
                    << " dest=" << std::hex << ::b(packet_after2.data()[lane])
                    << " got=" << ::b(packet_after3.data()[lane])
                    << " want=" << ::b(native_final.data()[lane])
                    << " variant=" << find_weighted_accum_variant(G, gv, row, col,
                                                                       packet_after2.data()[lane],
                                                                       native_final.data()[lane])
                    << std::dec << '\n';
        }
      }
      report((std::string("p") + std::to_string(i + 1) + " packetA_scalarG").c_str(), packet_a_scalar_g, native_final);
      report((std::string("p") + std::to_string(i + 1) + " scalarA_packetG").c_str(), scalar_a_packet_g, native_final);
      report((std::string("p") + std::to_string(i + 1) + " eigen_sequential").c_str(), eigen_sequential, native_final);
      report((std::string("p") + std::to_string(i + 1) + " packetA_eigenG").c_str(), packetA_eigenG, native_final);
      report((std::string("p") + std::to_string(i + 1) + " direct_sequential").c_str(), direct_sequential, native_final);
      report((std::string("p") + std::to_string(i + 1) + " plain_packet").c_str(), plain_packet, native_final);
      report((std::string("p") + std::to_string(i + 1) + " plain_eigen").c_str(), plain_eigen, native_final);
      report((std::string("p") + std::to_string(i + 1) + " plainA_eigenG").c_str(), plainA_eigenG, native_final);
      report((std::string("p") + std::to_string(i + 1) + " rowmajor_plain").c_str(), rowmajor_plain, native_final);
      report((std::string("p") + std::to_string(i + 1) + " zeroadd_plain").c_str(), zeroadd_plain, native_final);
      report((std::string("p") + std::to_string(i + 1) + " addcol_plain").c_str(), addcol_plain, native_final);
      report((std::string("p") + std::to_string(i + 1) + " addcol_fused").c_str(), addcol_fused, native_final);
      report((std::string("p") + std::to_string(i + 1) + " manual_seq").c_str(), manual_seq, native_final);
      report((std::string("p") + std::to_string(i + 1) + " expr_seq").c_str(), expr_seq, native_final);
      report((std::string("p") + std::to_string(i + 1) + " general_expr_seq").c_str(), general_expr_seq, native_final);
      report((std::string("p") + std::to_string(i + 1) + " general_packet_seq").c_str(), general_packet_seq, native_final);
      report((std::string("p") + std::to_string(i + 1) + " general_scalar_seq").c_str(), general_scalar_seq, native_final);
      report((std::string("p") + std::to_string(i + 1) + " general_addcol_seq").c_str(), general_addcol_seq, native_final);
      for (int lane = 0; lane < 81; ++lane) {
        if (::b(zero2.data()[lane]) != ::b(native_term2.data()[lane])) {
          const int row = lane % 9, col = lane / 9;
          std::cout << "p" << i + 1 << " zero2_variant lane=" << lane << " row=" << row << " col=" << col
                    << " got=" << std::hex << ::b(zero2.data()[lane]) << " variant="
                    << find_weighted_variant(A, av, row, col, zero2.data()[lane]) << std::dec << '\n';
        }
        if (::b(zero3.data()[lane]) != ::b(native_term3.data()[lane])) {
          const int row = lane % 9, col = lane / 9;
          std::cout << "p" << i + 1 << " zero3_variant lane=" << lane << " row=" << row << " col=" << col
                    << " got=" << std::hex << ::b(zero3.data()[lane]) << " variant="
                    << find_weighted_variant(G, gv, row, col, zero3.data()[lane]) << std::dec << '\n';
        }
      }
      Mat9 cov_before; load(cov_before, words(field(covs[i], "cov_before")));
      Mat9 sequential_cov = cov_before;
      sequential_cov = F * sequential_cov * F.transpose();
      sequential_cov += A * av.asDiagonal() * A.transpose();
      sequential_cov += G * gv.asDiagonal() * G.transpose();
      report((std::string("p") + std::to_string(i + 1) + " sequential_cov").c_str(), sequential_cov, native_final);
    }
    if (i == 0) {
      Mat93 A, G; Eigen::Vector3f av, gv; Mat9 final;
      load(A, words(field(steps[i], "a"))); load(G, words(field(steps[i], "g")));
      const auto avw = words(field(covs[i], "accel_cov")); const auto gvw = words(field(covs[i], "gyro_cov"));
      for (int k = 0; k < 3; ++k) { av[k] = f(avw[k]); gv[k] = f(gvw[k]); }
      load(final, words(field(covs[i], "cov")));
      report("p1 eigen_sum", eigen_sum(F, C, A, G, av, gv), final);
      const Mat9 t1 = eigen_chain(F, C);
      const Mat9 t2 = A * av.asDiagonal() * A.transpose();
      const Mat9 t3 = G * gv.asDiagonal() * G.transpose();
      std::cout << "flags inner_rowmajor=" << decltype((A * av.asDiagonal()).eval())::IsRowMajor
                << " outer_rowmajor=" << decltype((A * av.asDiagonal() * A.transpose()).eval())::IsRowMajor
                << '\n';
      using RhsMapper = Eigen::internal::const_blas_data_mapper<float, int, Eigen::RowMajor>;
      Eigen::internal::gemm_pack_rhs<float, int, RhsMapper, 4, Eigen::RowMajor> pack_rhs;
      float packed_rhs[27]{};
      RhsMapper rhs_mapper(A.data(), 9);
      pack_rhs(packed_rhs, rhs_mapper, 3, 9);
      std::cout << "packed_rhs=";
      for (int j = 0; j < 27; ++j) std::cout << std::hex << ::b(packed_rhs[j]) << ',';
      std::cout << std::dec << '\n';
      std::cout << "p1 t1_row8=";
      for (int col = 0; col < 9; ++col) std::cout << std::hex << ::b(t1(8, col)) << ',';
      std::cout << std::dec << '\n';
      Mat9 left_assoc = (t1 + t2) + t3;
      Mat9 right_assoc = t1 + (t2 + t3);
      Mat9 inplace = t1; inplace += t2; inplace += t3;
      Mat9 aliased = C;
      aliased = F * aliased * F.transpose() + A * av.asDiagonal() * A.transpose() +
                G * gv.asDiagonal() * G.transpose();
      Mat9 sequential = C;
      sequential = F * sequential * F.transpose();
      sequential += A * av.asDiagonal() * A.transpose();
      sequential += G * gv.asDiagonal() * G.transpose();
      Mat9 sequential_noalias = C;
      sequential_noalias.noalias() = F * sequential_noalias * F.transpose();
      sequential_noalias.noalias() += A * av.asDiagonal() * A.transpose();
      sequential_noalias.noalias() += G * gv.asDiagonal() * G.transpose();
      Mat9 seq_assign = C;
      seq_assign = F * seq_assign * F.transpose();
      seq_assign = seq_assign + A * av.asDiagonal() * A.transpose() +
                   G * gv.asDiagonal() * G.transpose();
      const Mat93 As = A * av.asDiagonal();
      const Mat93 Gs = G * gv.asDiagonal();
      using Mat93Row = Eigen::Matrix<float, 9, 3, Eigen::RowMajor>;
      const Mat93Row AsRow = A * av.asDiagonal();
      const Mat93Row GsRow = G * gv.asDiagonal();
      using Mat39 = Eigen::Matrix<float, 3, 9>;
      using Mat39Row = Eigen::Matrix<float, 3, 9, Eigen::RowMajor>;
      const Mat39 ATcol = A.transpose();
      const Mat39Row ATrow = A.transpose();
      const Mat39 GTcol = G.transpose();
      const Mat39Row GTrow = G.transpose();
      Mat9 sequential_scaled = C;
      sequential_scaled = F * sequential_scaled * F.transpose();
      sequential_scaled += As * A.transpose();
      sequential_scaled += Gs * G.transpose();
      Mat9 sequential_row = C;
      sequential_row = F * sequential_row * F.transpose();
      sequential_row += AsRow * A.transpose();
      sequential_row += GsRow * G.transpose();
      Mat9 sequential_explicit_rhs = C;
      sequential_explicit_rhs = F * sequential_explicit_rhs * F.transpose();
      sequential_explicit_rhs += As * ATcol;
      sequential_explicit_rhs += Gs * GTcol;
      Mat9 sequential_explicit_rhs_row = C;
      sequential_explicit_rhs_row = F * sequential_explicit_rhs_row * F.transpose();
      sequential_explicit_rhs_row += As * ATrow;
      sequential_explicit_rhs_row += Gs * GTrow;
      Mat9 after2 = t1;
      after2 += A * av.asDiagonal() * A.transpose();
      Mat9 zero_after2 = Mat9::Zero();
      zero_after2 += A * av.asDiagonal() * A.transpose();
      Mat9 after2_direct = t1 + t2;
      Mat9 after3_direct = after2_direct + t3;
      report("p1 zero_assign_func_term2", zero_assign_func(A, av), t2);
      report("p1 zero_add_func_term2", zero_add_func(A, av), t2);
      report("p1 scalar_accum_after2", weighted_scalar_accumulate(t1, A, av), after2);
      report("p1 packet_accum_after2", weighted_packet_accumulate(t1, A, av), after2);
      report("p1 after2_vs_direct", after2, after2_direct);
      report("p1 zero_after2_vs_term2", zero_after2, t2);
      std::cout << "p1 row8_after2=";
      for (int col = 0; col < 9; ++col) std::cout << std::hex << ::b(after2(8, col)) << ',';
      std::cout << " t2=";
      for (int col = 0; col < 9; ++col) std::cout << std::hex << ::b(t2(8, col)) << ',';
      std::cout << std::dec << '\n';
      for (int lane = 0; lane < 81; ++lane) {
        if (::b(after2.data()[lane]) != ::b(after2_direct.data()[lane]))
          std::cout << "p1 after2_diff lane=" << lane << " direct=" << std::hex
                    << ::b(after2_direct.data()[lane]) << " seq=" << ::b(after2.data()[lane]) << std::dec << '\n';
      }
      report("p1 left_assoc", left_assoc, final);
      report("p1 right_assoc", right_assoc, final);
      report("p1 inplace", inplace, final);
      report("p1 aliased", aliased, final);
      report("p1 sequential", sequential, final);
      report("p1 sequential_noalias", sequential_noalias, final);
      report("p1 seq_assign", seq_assign, final);
      report("p1 sequential_scaled", sequential_scaled, final);
      report("p1 sequential_row", sequential_row, final);
      report("p1 sequential_explicit_rhs", sequential_explicit_rhs, final);
      report("p1 sequential_explicit_rhs_row", sequential_explicit_rhs_row, final);
      report("p1 term1", t1, want);
      const Mat9 w2 = A * av.asDiagonal() * A.transpose();
      const Mat9 w3 = G * gv.asDiagonal() * G.transpose();
      Mat9 native_w2, native_w3;
      load(native_w2, words(field(covs[i], "term2")));
      load(native_w3, words(field(covs[i], "term3")));
      report("p1 term2", w2, native_w2);
      report("p1 term3", w3, native_w3);
      for (int lane : {62, 71, 80}) {
        std::cout << "p1 lane=" << lane << " t1/t2/t3=" << std::hex << ::b(t1.data()[lane]) << "/"
                  << ::b(t2.data()[lane]) << "/" << ::b(t3.data()[lane]) << " direct="
                  << ::b(left_assoc.data()[lane]) << " alias=" << ::b(aliased.data()[lane]) << " native="
                  << ::b(final.data()[lane]) << std::dec << '\n';
      }
      for (int lane = 0; lane < 81; ++lane) {
        if (::b(inplace.data()[lane]) != ::b(sequential.data()[lane]))
          std::cout << "p1 seq_diff lane=" << lane << " direct=" << std::hex << ::b(inplace.data()[lane])
                    << " seq=" << ::b(sequential.data()[lane]) << std::dec << '\n';
      }
    }
  }
}
