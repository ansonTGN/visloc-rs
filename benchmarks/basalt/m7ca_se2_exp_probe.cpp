// M7ca diagnostic probe for pinned Sophus::SE2f::exp operation order.
//
// The first 20 tangents are the recorded MH_01 frame-0 -> frame-1 cam0
// track-2 increments.  The probe intentionally calls the uninstrumented
// Sophus implementation and emits all rotation/translation bits.

#include <sophus/se2.hpp>

#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>

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

  for (std::size_t index = 0; index < sizeof(recorded) / sizeof(recorded[0]);
       ++index) {
    const Eigen::Vector3f tangent(from_bits(recorded[index].tx),
                                  from_bits(recorded[index].ty),
                                  from_bits(recorded[index].theta));
    const Sophus::SE2f update = Sophus::SE2f::exp(tangent);
    std::cout << "recorded[" << index << "] theta=";
    print_bits(tangent[2]);
    std::cout << " rotation=";
    print_bits(update.rotationMatrix()(0, 0));
    std::cout << ',';
    print_bits(update.rotationMatrix()(0, 1));
    std::cout << ',';
    print_bits(update.rotationMatrix()(1, 0));
    std::cout << ',';
    print_bits(update.rotationMatrix()(1, 1));
    std::cout << " translation=";
    print_bits(update.translation()[0]);
    std::cout << ',';
    print_bits(update.translation()[1]);
    std::cout << '\n';
  }

  // Values bracketing Eigen's float epsilon branch, plus signed and zero
  // cases.  Keep tx/ty nontrivial so both V(theta) rows are exercised.
  const std::uint32_t small_thetas[] = {
      0x00000000, 0x33000000, 0x33ffffff, 0x34000000, 0xb3000000,
      0xb3ffffff, 0xb4000000, 0x35000000,
  };
  for (std::size_t index = 0; index < sizeof(small_thetas) / sizeof(small_thetas[0]);
       ++index) {
    const Eigen::Vector3f tangent(from_bits(0x3f4ccccd),
                                  from_bits(0xbf19999a),
                                  from_bits(small_thetas[index]));
    const Sophus::SE2f update = Sophus::SE2f::exp(tangent);
    std::cout << "small[" << index << "] theta=";
    print_bits(tangent[2]);
    std::cout << " rotation=";
    print_bits(update.rotationMatrix()(0, 0));
    std::cout << ',';
    print_bits(update.rotationMatrix()(0, 1));
    std::cout << ',';
    print_bits(update.rotationMatrix()(1, 0));
    std::cout << ',';
    print_bits(update.rotationMatrix()(1, 1));
    std::cout << " translation=";
    print_bits(update.translation()[0]);
    std::cout << ',';
    print_bits(update.translation()[1]);
    std::cout << '\n';
  }
  return 0;
}
