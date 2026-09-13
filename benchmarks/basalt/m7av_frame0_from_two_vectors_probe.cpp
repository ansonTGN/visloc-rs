// M7av diagnostic: source-call frame-0 gravity alignment probe.
//
// The pinned Basalt no-time initialize() path calls
// Eigen::Quaternion<Scalar>::FromTwoVectors(data->accel, Vec3::UnitZ()).
// This probe uses the first MH_01 IMU packet at the frame-0 camera timestamp,
// applies the pinned calibration, and prints every intermediate used by the
// Eigen setFromTwoVectors implementation for Scalar=float and double.

#include <basalt/calibration/calib_bias.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>

namespace {

template <typename Scalar>
using Vec3 = Eigen::Matrix<Scalar, 3, 1>;

template <typename Scalar>
using Quaternion = Eigen::Quaternion<Scalar>;

template <typename Scalar>
using UInt = std::conditional_t<sizeof(Scalar) == 4, std::uint32_t,
                                std::uint64_t>;

template <typename Scalar>
UInt<Scalar> bits(Scalar value) {
  UInt<Scalar> result = 0;
  std::memcpy(&result, &value, sizeof(result));
  return result;
}

template <typename Scalar>
void print_vec(const char* name, const Vec3<Scalar>& value) {
  std::cout << name << " value=" << std::setprecision(20) << value.transpose()
            << " bits=0x" << std::hex << bits(value.x()) << ",0x"
            << bits(value.y()) << ",0x" << bits(value.z()) << std::dec
            << "\n";
}

template <typename Scalar>
void print_scalar(const char* name, Scalar value) {
  std::cout << name << " value=" << std::setprecision(20) << value << " bits=0x"
            << std::hex << bits(value) << std::dec << "\n";
}

template <typename Scalar>
void print_quat(const char* name, const Quaternion<Scalar>& value) {
  // Eigen stores coeffs as x,y,z,w; print both that order and wxyz explicitly.
  std::cout << name << " coeffs_xyzw=" << std::setprecision(20)
            << value.coeffs().transpose() << " wxyz=" << value.w() << ","
            << value.x() << "," << value.y() << "," << value.z() << " bits_xyzw=0x"
            << std::hex << bits(value.x()) << ",0x" << bits(value.y()) << ",0x"
            << bits(value.z()) << ",0x" << bits(value.w()) << std::dec << "\n";
}

template <typename Scalar>
void run(const char* label) {
  // MH_01 frame 0 is 1403636579763555584 ns. The first packet at or after
  // that timestamp is exactly this row. Values are first cast from the
  // EuRoC double queue to Scalar, as popFromImuDataQueue() does upstream.
  const Vec3<Scalar> raw(
      static_cast<Scalar>(8.0332807916666666),
      static_cast<Scalar>(-0.40861041666666664),
      static_cast<Scalar>(-2.40262925));

  basalt::CalibAccelBias<Scalar> calibration;
  Eigen::Matrix<Scalar, 9, 1> params;
  params << static_cast<Scalar>(-0.003025405479279035),
      static_cast<Scalar>(0.1200005286487319),
      static_cast<Scalar>(0.06708820471592454), static_cast<Scalar>(0),
      static_cast<Scalar>(0), static_cast<Scalar>(0), static_cast<Scalar>(0),
      static_cast<Scalar>(0), static_cast<Scalar>(0);
  calibration.getParam() = params;
  const Vec3<Scalar> accel = calibration.getCalibrated(raw);
  const Vec3<Scalar> target = Vec3<Scalar>::UnitZ();
  const Vec3<Scalar> v0 = accel.normalized();
  const Vec3<Scalar> v1 = target.normalized();
  const Scalar c = v1.dot(v0);
  const Vec3<Scalar> cross = v0.cross(v1);
  const Scalar one_plus_c = Scalar(1) + c;
  const Scalar two_times = one_plus_c * Scalar(2);
  const Scalar s = std::sqrt(two_times);
  const Scalar invs = Scalar(1) / s;
  const Vec3<Scalar> vec = cross * invs;
  const Quaternion<Scalar> source = Quaternion<Scalar>::FromTwoVectors(accel, target);
  const Scalar qnorm2 = source.squaredNorm();
  const Scalar qnorm = std::sqrt(qnorm2);
  const Quaternion<Scalar> normalized_source(source.coeffs() / qnorm);

  std::cout << "=== " << label << " ===\n";
  print_vec("raw", raw);
  print_vec("accel_calibrated", accel);
  print_vec("target", target);
  print_scalar("accel_squared_norm", accel.squaredNorm());
  print_scalar("accel_norm", accel.norm());
  print_vec("v0_normalized", v0);
  print_scalar("target_squared_norm", target.squaredNorm());
  print_scalar("target_norm", target.norm());
  print_vec("v1_normalized", v1);
  print_scalar("dot_c_v1_dot_v0", c);
  print_vec("cross_v0_cross_v1", cross);
  print_scalar("one_plus_c", one_plus_c);
  print_scalar("two_times_one_plus_c", two_times);
  print_scalar("sqrt_s", s);
  print_scalar("invs", invs);
  print_vec("vec_axis_times_invs", vec);
  print_quat("source_FromTwoVectors", source);
  print_scalar("source_squared_norm", qnorm2);
  print_scalar("source_norm", qnorm);
  print_quat("source_manually_normalized", normalized_source);
}

}  // namespace

int main() {
  run<float>("float");
  run<double>("double");
  return 0;
}
