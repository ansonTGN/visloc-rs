// Diagnostic-only direct IntegratedImuMeasurement<float>::predictState probe.
// This uses the pinned Basalt headers and the MH_01 frame-0->1 IMU packets,
// then prints the exact f32 delta and prediction operands/results.

#include <basalt/calibration/calib_bias.hpp>
#include <basalt/imu/preintegration.h>

#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace {

uint32_t bits(float value) {
  uint32_t result = 0;
  std::memcpy(&result, &value, sizeof(result));
  return result;
}

template <typename Derived>
void print_bits(const char* name, const Eigen::MatrixBase<Derived>& value) {
  std::cout << name << "=";
  for (Eigen::Index i = 0; i < value.size(); ++i) {
    if (i) std::cout << ",";
    std::cout << std::hex << bits(value.derived()(i));
  }
  std::cout << std::dec << "\n";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  using Scalar = float;
  using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
  const int64_t start_ns = 1403636579763555584LL;
  const int64_t end_ns = 1403636579813555456LL;

  basalt::CalibAccelBias<Scalar> accel_calib;
  Eigen::Matrix<Scalar, 9, 1> accel_param;
  accel_param << Scalar(-0.003025405479279035), Scalar(0.1200005286487319),
      Scalar(0.06708820471592454), Scalar(0), Scalar(0), Scalar(0),
      Scalar(0), Scalar(0), Scalar(0);
  accel_calib.getParam() = accel_param;
  basalt::CalibGyroBias<Scalar> gyro_calib;
  Eigen::Matrix<Scalar, 12, 1> gyro_param;
  gyro_param << Scalar(-0.002186848441668376), Scalar(0.020427823167917037),
      Scalar(0.07668367023977922), Scalar(0), Scalar(0), Scalar(0),
      Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0);
  gyro_calib.getParam() = gyro_param;

  const Vec3 accel_noise =
      Vec3::Constant(Scalar(0.016) * std::sqrt(Scalar(200)));
  const Vec3 gyro_noise =
      Vec3::Constant(Scalar(0.000282) * std::sqrt(Scalar(200)));
  const Vec3 accel_cov = accel_noise.array().square();
  const Vec3 gyro_cov = gyro_noise.array().square();

  basalt::IntegratedImuMeasurement<Scalar> measurement(
      start_ns, Vec3::Zero(), Vec3::Zero());
  std::ifstream input(argv[1]);
  std::string line;
  int count = 0;
  int64_t previous_data_t_ns = start_ns;
  Sophus::SO3<Scalar> manual_rotation;
  Vec3 manual_velocity = Vec3::Zero();
  Vec3 manual_position_left = Vec3::Zero();
  Vec3 manual_position_right = Vec3::Zero();
  Vec3 manual_position_source = Vec3::Zero();
  while (std::getline(input, line)) {
    if (line.empty() || line[0] == '#') continue;
    for (char& c : line) {
      if (c == ',') c = ' ';
    }
    std::stringstream row(line);
    int64_t t_ns = 0;
    double wx = 0, wy = 0, wz = 0, ax = 0, ay = 0, az = 0;
    if (!(row >> t_ns >> wx >> wy >> wz >> ax >> ay >> az)) return 3;
    if (t_ns <= start_ns || t_ns > end_ns) continue;
    basalt::ImuData<Scalar> data;
    data.t_ns = t_ns;
    data.gyro = gyro_calib.getCalibrated(
        Vec3(static_cast<Scalar>(wx), static_cast<Scalar>(wy),
             static_cast<Scalar>(wz)));
    data.accel = accel_calib.getCalibrated(
        Vec3(static_cast<Scalar>(ax), static_cast<Scalar>(ay),
             static_cast<Scalar>(az)));
    const Scalar dt = static_cast<Scalar>(t_ns - previous_data_t_ns) * Scalar(1e-9);
    const Vec3 omega = dt * data.gyro;
    const Scalar theta_sq = omega.squaredNorm();
    const Scalar theta = std::sqrt(theta_sq);
    const Scalar half_theta = Scalar(0.5) * theta;
    const Scalar sin_half_theta = std::sin(half_theta);
    const Scalar imag_factor = sin_half_theta / theta;
    const Scalar real_factor = std::cos(half_theta);
    const auto exp_q = Sophus::SO3<Scalar>::exp(omega).unit_quaternion();
    std::cout << "exp=";
    print_bits("q", exp_q.coeffs());
    std::cout << "omega=";
    print_bits("v", omega);
    std::cout << "theta_sq=" << std::hex << bits(theta_sq)
              << " theta=" << bits(theta)
              << " half=" << bits(half_theta)
              << " sin=" << bits(sin_half_theta)
              << " imag=" << bits(imag_factor)
              << " real=" << bits(real_factor) << std::dec << "\n";
    std::cout << "sample=" << (count + 1) << " gyro=";
    print_bits("g", data.gyro);
    std::cout << " ";
    print_bits("a", data.accel);
    measurement.integrate(data, accel_cov, gyro_cov);
    ++count;
    const auto manual_half =
        manual_rotation * Sophus::SO3<Scalar>::exp(Scalar(0.5) * dt * data.gyro);
    const Vec3 manual_accel_world = manual_half.matrix() * data.accel;
    const Vec3 manual_velocity_dt = manual_velocity * dt;
    const Vec3 manual_accel_term = Scalar(0.5) * manual_accel_world * dt * dt;
    manual_position_left =
        (manual_position_left + manual_velocity_dt) + manual_accel_term;
    manual_position_right =
        manual_position_right + (manual_velocity_dt + manual_accel_term);
    manual_position_source = manual_position_source + manual_velocity * dt +
                             Scalar(0.5) * manual_accel_world * dt * dt;
    manual_velocity += manual_accel_world * dt;
    manual_rotation =
        manual_rotation * Sophus::SO3<Scalar>::exp(dt * data.gyro);
    std::cout << "assoc=" << count << " ";
    print_bits("left", manual_position_left);
    std::cout << " ";
    print_bits("right", manual_position_right);
    std::cout << " ";
    print_bits("source", manual_position_source);
    const auto& step = measurement.getDeltaState();
    const auto step_q = step.T_w_i.so3().unit_quaternion();
    std::cout << "step=" << count << " t_ns=" << t_ns << " ";
    print_bits("p", step.T_w_i.translation());
    std::cout << " ";
    print_bits("q", step_q.coeffs());
    std::cout << " ";
    print_bits("v", step.vel_w_i);
    previous_data_t_ns = t_ns;
  }

  const auto& delta = measurement.getDeltaState();
  const auto delta_q = delta.T_w_i.so3().unit_quaternion();
  print_bits("delta_p", delta.T_w_i.translation());
  print_bits("delta_q", delta_q.coeffs());
  print_bits("delta_v", delta.vel_w_i);
  std::cout << "count=" << count << " dt_ns=" << measurement.get_dt_ns()
            << "\n";

  const Vec3 t0 = Vec3::Zero();
  const Vec3 v0 = Vec3::Zero();
  const Eigen::Quaternion<Scalar> q0(
      Scalar(0.5944822430610657), Scalar(-0.052778493613004684),
      Scalar(-0.8023747801780701), Scalar(0));
  const Sophus::SE3<Scalar> pose0(Sophus::SO3<Scalar>(q0), t0);
  const basalt::PoseVelState<Scalar> state0(start_ns, pose0, v0);
  basalt::PoseVelState<Scalar> state1;
  const Vec3 gravity(Scalar(0), Scalar(0), Scalar(-9.8100004196166992));
  measurement.predictState(state0, gravity, state1);
  const auto state1_q = state1.T_w_i.so3().unit_quaternion();
  print_bits("state1_p", state1.T_w_i.translation());
  print_bits("state1_q", state1_q.coeffs());
  print_bits("state1_v", state1.vel_w_i);
  return 0;
}
