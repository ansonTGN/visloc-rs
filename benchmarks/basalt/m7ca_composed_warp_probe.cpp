// M7ca diagnostic replay of the pinned 20 SE2 increments through Eigen's
// AffineCompact2f *= Matrix3f product.  Increment bits are the recorded
// frame-0 -> frame-1 cam0 track-2 sequence; no patch/stream instrumentation
// is involved here.

#include <Eigen/Geometry>
#include <sophus/se2.hpp>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <iostream>

using Affine = Eigen::AffineCompact2f;

static float from_bits(std::uint32_t value) {
  float result = 0.0f;
  std::memcpy(&result, &value, sizeof(result));
  return result;
}

static std::uint32_t bits(float value) {
  std::uint32_t result = 0;
  std::memcpy(&result, &value, sizeof(result));
  return result;
}

static void print_bits(float value) {
  std::cout << "0x" << std::hex << std::setw(8) << std::setfill('0')
            << bits(value) << std::dec << std::setfill(' ');
}

struct TangentBits {
  std::uint32_t tx;
  std::uint32_t ty;
  std::uint32_t theta;
};

static void print_matrix(const char* label, const Affine& transform) {
  std::cout << label << '=';
  print_bits(transform.linear()(0, 0));
  std::cout << ',';
  print_bits(transform.linear()(0, 1));
  std::cout << ',';
  print_bits(transform.linear()(1, 0));
  std::cout << ',';
  print_bits(transform.linear()(1, 1));
  std::cout << ',';
  print_bits(transform.translation().x());
  std::cout << ',';
  print_bits(transform.translation().y());
  std::cout << '\n';
}

int main() {
  const TangentBits recorded[] = {
      {0xbeb28e40, 0xbe1adabe, 0xbb9ce858},
      {0x3ce12820, 0x3c65ae81, 0x3a9ce3d8},
      {0x3bd231f5, 0x3a8e8548, 0x3ad33e98},
      {0x3b0d8a18, 0x39772500, 0x3a04bb70},
      {0x3a2c8748, 0x38a7cb00, 0x3924e040},
      {0xbc6a7346, 0x3c3caeea, 0x3b9704a2},
      {0x3b085770, 0xbb2a77d4, 0xb8b79bc0},
      {0xba159230, 0x3a155be0, 0xb81a9e80},
      {0x393ca440, 0xb917ac40, 0x37592200},
      {0xb86ddc00, 0x38311a00, 0xb682a000},
      {0x3e5bb15e, 0x3d8bad19, 0xbc6e3b6a},
      {0xbdd326a0, 0x3ad60820, 0xb9fc3940},
      {0x3d4e2d10, 0xbbf5bb48, 0x3b3a9694},
      {0xbccec6c0, 0x3ba90ec0, 0xbb17b3fc},
      {0x3c5276da, 0xbb4111e0, 0x3abf9c20},
      {0xbe758c7b, 0xbd1b2fc4, 0x3b12b2a8},
      {0x3d91e281, 0x3beecf2a, 0xbc826c56},
      {0xbce5ac64, 0xb92ec0c0, 0x3bea2a84},
      {0x3c2f6de9, 0xb9e95350, 0xbb3feb72},
      {0xbb8701ec, 0x39921b40, 0x3a966db8},
  };

  Affine transform;
  transform.setIdentity();
  transform.translation() = Eigen::Vector2f(46.0f, 118.0f);
  std::size_t increment_index = 0;
  for (int level = 3; level >= 0; --level) {
    const float scale = static_cast<float>(1 << level);
    transform.translation() /= scale;
    for (int iteration = 0; iteration < 5; ++iteration) {
      const TangentBits& tangent_bits = recorded[increment_index++];
      const Eigen::Vector3f tangent(from_bits(tangent_bits.tx),
                                    from_bits(tangent_bits.ty),
                                    from_bits(tangent_bits.theta));
      transform *= Sophus::SE2f::exp(tangent).matrix();
      char label[32];
      std::snprintf(label, sizeof(label), "L%d.I%d.m", level, iteration);
      print_matrix(label, transform);
    }
    transform.translation() *= scale;
    char label[32];
    std::snprintf(label, sizeof(label), "L%d.world_m", level);
    print_matrix(label, transform);
  }
  print_matrix("final_m", transform);
  return 0;
}
