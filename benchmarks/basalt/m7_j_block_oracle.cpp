// Diagnostic-only IMU factor intermediate probe.
//
// This translation unit includes the existing pinned-header oracle only for
// its input/calibration helpers; it does not touch Rust production sources or
// the pinned checkout.  It emits the exact f32 values at the intermediates
// immediately preceding the four Jacobian blocks.
#define main m7im_cov_ldlt_oracle_original_main
#include "m7im_cov_ldlt_oracle.cpp"
#undef main

#include <basalt/utils/sophus_utils.hpp>

#include <fstream>
#include <iostream>

namespace {

using Mat3 = Eigen::Matrix<Scalar, 3, 3>;

void write_named_vector(std::ostream& out, const char* name,
                        const Vec3& value) {
  out << name << "=";
  write_vector_bits(out, value);
  out << "\n";
}

template <typename Derived>
void write_named_matrix(std::ostream& out, const char* name,
                        const Eigen::MatrixBase<Derived>& value) {
  out << name << "=";
  write_matrix_bits(out, value);
  out << "\n";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "usage: m7_j_block_oracle <imu.csv> <states.tsv> <output>\n";
    return 2;
  }
  try {
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
    const auto samples = read_samples(argv[1], accel_calib, gyro_calib);
    const auto states = read_states(argv[2]);
    if (states.size() != 5) throw std::runtime_error("expected five states");

    const Vec3 accel_noise =
        Vec3::Constant(Scalar(0.016) * std::sqrt(Scalar(200)));
    const Vec3 gyro_noise =
        Vec3::Constant(Scalar(0.000282) * std::sqrt(Scalar(200)));
    const Vec3 accel_cov = accel_noise.array().square();
    const Vec3 gyro_cov = gyro_noise.array().square();
    const Vec3 gravity(Scalar(0), Scalar(0), Scalar(-9.8100004196166992));

    basalt::IntegratedImuMeasurement<Scalar> measurement(
        states[3].t_ns, states[3].bias_gyro, states[3].bias_accel);
    for (const Sample& sample : samples) {
      if (sample.t_ns <= states[3].t_ns || sample.t_ns > states[4].t_ns)
        continue;
      basalt::ImuData<Scalar> data;
      data.t_ns = sample.t_ns;
      data.gyro = sample.gyro;
      data.accel = sample.accel;
      measurement.integrate(data, accel_cov, gyro_cov);
    }

    Mat9 jac0 = Mat9::Zero();
    Mat9 jac1 = Mat9::Zero();
    Mat93 jac_bg = Mat93::Zero();
    Mat93 jac_ba = Mat93::Zero();
    const auto state0 = states[3].pose_vel_state();
    const auto state1 = states[4].pose_vel_state();
    const Vec9 raw = measurement.residual(
        state0, gravity, state1, states[3].bias_gyro, states[3].bias_accel,
        &jac0, &jac1, &jac_bg, &jac_ba);
    const Scalar dt = static_cast<Scalar>(measurement.get_dt_ns()) *
                      Scalar(1e-9);
    const Mat3 r0_inv = state0.T_w_i.so3().inverse().matrix();
    const Vec3 tmp =
        r0_inv * (state1.T_w_i.translation() - state0.T_w_i.translation() -
                  state0.vel_w_i * dt - Scalar(0.5) * gravity * dt * dt);
    const Vec3 tmp2 =
        r0_inv * (state1.vel_w_i - state0.vel_w_i - gravity * dt);
    const Vec3 rotation = raw.template segment<3>(3);
    Mat3 right_inv = Mat3::Zero();
    Mat3 left_inv = Mat3::Zero();
    Sophus::rightJacobianInvSO3(rotation, right_inv);
    Sophus::leftJacobianInvSO3(rotation, left_inv);

    std::ofstream out(argv[3]);
    if (!out) throw std::runtime_error("cannot open output");
    write_named_matrix(out, "r0_inv", r0_inv);
    write_named_vector(out, "tmp", tmp);
    write_named_vector(out, "tmp2", tmp2);
    write_named_vector(out, "rotation", rotation);
    write_named_matrix(out, "right_inv", right_inv);
    write_named_matrix(out, "left_inv", left_inv);
    write_named_matrix(out, "skew_tmp_r0_inv",
                       Sophus::SO3<Scalar>::hat(tmp) * r0_inv);
    write_named_matrix(out, "right_inv_r0_inv", right_inv * r0_inv);
    write_named_matrix(out, "skew_tmp2_r0_inv",
                       Sophus::SO3<Scalar>::hat(tmp2) * r0_inv);
    write_named_matrix(out, "jacobian_state0", jac0.block<3, 3>(0, 3));
    write_named_matrix(out, "jacobian_rotation", jac0.block<3, 3>(3, 3));
    write_named_matrix(out, "jacobian_velocity_rotation",
                       jac0.block<3, 3>(6, 3));
    write_named_vector(out, "raw_position", raw.template segment<3>(0));
    write_named_vector(out, "raw_rotation", raw.template segment<3>(3));
    write_named_vector(out, "raw_velocity", raw.template segment<3>(6));
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "m7_j_block_oracle failed: " << error.what() << "\n";
    return 1;
  }
}
