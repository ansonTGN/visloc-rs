// Diagnostic-only per-link IMU boundary audit for the pinned Basalt header.
//
// This is intentionally a small standalone oracle: it runs the exact
// IntegratedImuMeasurement<float> implementation used by basalt_vio, emits
// the corrected delta, raw residual/Jacobians, covariance, LDLT whitener,
// whitened rows, row products, and the four-link accumulated state system.
// Build/run from WSL with the pinned basalt/eigen/sophus header packages.

#include <basalt/calibration/calib_bias.hpp>
#include <basalt/imu/imu_types.h>
#include <basalt/imu/preintegration.h>

#include <Eigen/Core>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using Scalar = float;
using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
using VecN = Eigen::Matrix<Scalar, basalt::POSE_VEL_SIZE, 1>;
using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
using MatNN = Eigen::Matrix<Scalar, basalt::POSE_VEL_SIZE,
                             basalt::POSE_VEL_SIZE>;
using MatN3 = Eigen::Matrix<Scalar, basalt::POSE_VEL_SIZE, 3>;
using Mat30 = Eigen::Matrix<Scalar, basalt::POSE_VEL_SIZE, 30>;
using Mat3030 = Eigen::Matrix<Scalar, 30, 30>;
using Mat75 = Eigen::Matrix<Scalar, 75, 75>;
using Vec75 = Eigen::Matrix<Scalar, 75, 1>;

struct Sample {
  int64_t t_ns = 0;
  Vec3 gyro = Vec3::Zero();
  Vec3 accel = Vec3::Zero();
};

struct State {
  int frame = 0;
  int64_t t_ns = 0;
  Vec3 translation = Vec3::Zero();
  Eigen::Quaternion<Scalar> rotation =
      Eigen::Quaternion<Scalar>::Identity();
  Vec3 velocity = Vec3::Zero();
  Vec3 bias_gyro = Vec3::Zero();
  Vec3 bias_accel = Vec3::Zero();

  basalt::PoseVelState<Scalar> pose_vel_state() const {
    const Sophus::SO3<Scalar> so3(rotation);
    const Sophus::SE3<Scalar> pose(so3, translation);
    return basalt::PoseVelState<Scalar>(t_ns, pose, velocity);
  }
};

void replace_commas(std::string& line) {
  for (char& c : line) {
    if (c == ',') c = ' ';
  }
}

std::vector<Sample> read_samples(
    const std::string& csv_path,
    const basalt::CalibAccelBias<Scalar>& accel_calib,
    const basalt::CalibGyroBias<Scalar>& gyro_calib) {
  std::ifstream in(csv_path);
  if (!in) throw std::runtime_error("cannot open IMU CSV: " + csv_path);
  std::vector<Sample> samples;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    replace_commas(line);
    std::stringstream row(line);
    int64_t t_ns = 0;
    double wx = 0, wy = 0, wz = 0, ax = 0, ay = 0, az = 0;
    if (!(row >> t_ns >> wx >> wy >> wz >> ax >> ay >> az))
      throw std::runtime_error("malformed IMU CSV row");
    Sample sample;
    sample.t_ns = t_ns;
    // The pinned estimator queues ImuData<double> and narrows to float at
    // the estimator boundary. Preserve that conversion before calibration.
    const Vec3 gyro_raw(static_cast<Scalar>(wx), static_cast<Scalar>(wy),
                        static_cast<Scalar>(wz));
    const Vec3 accel_raw(static_cast<Scalar>(ax), static_cast<Scalar>(ay),
                         static_cast<Scalar>(az));
    sample.gyro = gyro_calib.getCalibrated(gyro_raw);
    sample.accel = accel_calib.getCalibrated(accel_raw);
    samples.push_back(sample);
  }
  return samples;
}

std::vector<State> read_states(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot open state TSV: " + path);
  std::vector<State> states;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    replace_commas(line);
    std::stringstream row(line);
    State state;
    Scalar qx = 0, qy = 0, qz = 0, qw = 1;
    if (!(row >> state.frame >> state.t_ns >> state.translation.x() >>
          state.translation.y() >> state.translation.z() >> qx >> qy >> qz >>
          qw >> state.velocity.x() >> state.velocity.y() >>
          state.velocity.z() >> state.bias_gyro.x() >> state.bias_gyro.y() >>
          state.bias_gyro.z() >> state.bias_accel.x() >> state.bias_accel.y() >>
          state.bias_accel.z()))
      throw std::runtime_error("malformed state TSV row");
    state.rotation = Eigen::Quaternion<Scalar>(qw, qx, qy, qz);
    states.push_back(state);
  }
  return states;
}

template <typename Derived>
void write_vector(std::ostream& out, const Eigen::MatrixBase<Derived>& value) {
  out << "[";
  for (Eigen::Index i = 0; i < value.size(); ++i) {
    if (i) out << ",";
    out << value.derived().coeff(i);
  }
  out << "]";
}

template <typename Derived>
void write_matrix(std::ostream& out, const Eigen::MatrixBase<Derived>& value) {
  out << "[";
  for (Eigen::Index row = 0; row < value.rows(); ++row) {
    if (row) out << ",";
    out << "[";
    for (Eigen::Index col = 0; col < value.cols(); ++col) {
      if (col) out << ",";
      out << value.derived()(row, col);
    }
    out << "]";
  }
  out << "]";
}

void write_state(std::ostream& out, const State& state) {
  out << "{\"frame\":" << state.frame << ",\"t_ns\":" << state.t_ns
      << ",\"translation\":";
  write_vector(out, state.translation);
  out << ",\"rotation_xyzw\":[" << state.rotation.x() << ","
      << state.rotation.y() << "," << state.rotation.z() << ","
      << state.rotation.w() << "],\"velocity\":";
  write_vector(out, state.velocity);
  out << ",\"bias_gyro\":";
  write_vector(out, state.bias_gyro);
  out << ",\"bias_accel\":";
  write_vector(out, state.bias_accel);
  out << "}";
}

void write_delta(std::ostream& out,
                 const basalt::IntegratedImuMeasurement<Scalar>& measurement) {
  const auto& delta = measurement.getDeltaState();
  const auto q = delta.T_w_i.so3().unit_quaternion();
  out << "{\"dt_ns\":" << delta.t_ns << ",\"position\":";
  write_vector(out, delta.T_w_i.translation());
  out << ",\"rotation_xyzw\":[" << q.x() << "," << q.y() << ","
      << q.z() << "," << q.w() << "],\"velocity\":";
  write_vector(out, delta.vel_w_i);
  out << ",\"d_state_d_ba\":";
  write_matrix(out, measurement.get_d_state_d_ba());
  out << ",\"d_state_d_bg\":";
  write_matrix(out, measurement.get_d_state_d_bg());
  out << "}";
}

struct LinkAudit {
  basalt::IntegratedImuMeasurement<Scalar> measurement;
  VecN raw = VecN::Zero();
  MatNN jac0 = MatNN::Zero();
  MatNN jac1 = MatNN::Zero();
  MatN3 jac_bg = MatN3::Zero();
  MatN3 jac_ba = MatN3::Zero();
  Mat30 whitened_jac = Mat30::Zero();
  VecN whitened_raw = VecN::Zero();
  Mat3030 link_h = Mat3030::Zero();
  Eigen::Matrix<Scalar, 30, 1> full_b =
      Eigen::Matrix<Scalar, 30, 1>::Zero();
};

LinkAudit integrate_link(const State& start, const State& end,
                         const std::vector<Sample>& samples,
                         const Vec3& accel_cov, const Vec3& gyro_cov,
                         const Vec3& gravity) {
  LinkAudit result;
  result.measurement = basalt::IntegratedImuMeasurement<Scalar>(
      start.t_ns, start.bias_gyro, start.bias_accel);
  for (const Sample& sample : samples) {
    if (sample.t_ns <= start.t_ns || sample.t_ns > end.t_ns) continue;
    basalt::ImuData<Scalar> data;
    data.t_ns = sample.t_ns;
    data.gyro = sample.gyro;
    data.accel = sample.accel;
    result.measurement.integrate(data, accel_cov, gyro_cov);
  }
  result.raw = result.measurement.residual(
      start.pose_vel_state(), gravity, end.pose_vel_state(), start.bias_gyro,
      start.bias_accel, &result.jac0, &result.jac1, &result.jac_bg,
      &result.jac_ba);
  const auto& sqrt_info = result.measurement.get_sqrt_cov_inv();
  Mat30 stacked = Mat30::Zero();
  stacked.leftCols<9>() = result.jac0;
  stacked.middleCols<3>(9) = result.jac_bg;
  stacked.middleCols<3>(12) = result.jac_ba;
  stacked.middleCols<9>(15) = result.jac1;
  result.whitened_raw = sqrt_info * result.raw;
  result.whitened_jac = sqrt_info * stacked;
  result.link_h = result.whitened_jac.transpose() * result.whitened_jac;
  result.full_b = result.whitened_jac.transpose() * result.whitened_raw;
  return result;
}

void add_link_to_global(Mat75& h, Vec75& b, const LinkAudit& link,
                        size_t from, size_t to) {
  const auto J = link.whitened_jac;
  const auto r = link.whitened_raw;
  const Eigen::Matrix<Scalar, 15, 15> h00 =
      J.leftCols(15).transpose() * J.leftCols(15);
  const Eigen::Matrix<Scalar, 15, 15> h01 =
      J.leftCols(15).transpose() * J.rightCols(15);
  const Eigen::Matrix<Scalar, 15, 15> h11 =
      J.rightCols(15).transpose() * J.rightCols(15);
  const Eigen::Matrix<Scalar, 15, 1> b0 =
      J.leftCols(15).transpose() * r;
  const Eigen::Matrix<Scalar, 15, 1> b1 =
      J.rightCols(15).transpose() * r;
  h.block<15, 15>(from * 15, from * 15) += h00;
  h.block<15, 15>(from * 15, to * 15) += h01;
  h.block<15, 15>(to * 15, from * 15) += h01.transpose();
  h.block<15, 15>(to * 15, to * 15) += h11;
  b.segment<15>(from * 15) += b0;
  b.segment<15>(to * 15) += b1;
  const Scalar dt = static_cast<Scalar>(link.measurement.get_dt_ns()) *
                    Scalar(1e-9);
  const Scalar gyro_w = Scalar(1.0e4) / std::sqrt(dt);
  const Scalar accel_w = Scalar(1.0e3) / std::sqrt(dt);
  for (int axis = 0; axis < 3; ++axis) {
    const size_t gi = from * 15 + 9 + axis;
    const size_t gj = to * 15 + 9 + axis;
    h(gi, gi) += gyro_w * gyro_w;
    h(gi, gj) -= gyro_w * gyro_w;
    h(gj, gi) -= gyro_w * gyro_w;
    h(gj, gj) += gyro_w * gyro_w;
    const size_t ai = from * 15 + 12 + axis;
    const size_t aj = to * 15 + 12 + axis;
    h(ai, ai) += accel_w * accel_w;
    h(ai, aj) -= accel_w * accel_w;
    h(aj, ai) -= accel_w * accel_w;
    h(aj, aj) += accel_w * accel_w;
  }
}

void write_link(std::ostream& out, const LinkAudit& link, const State& start,
                const State& end) {
  const auto& sqrt_info = link.measurement.get_sqrt_cov_inv();
  out << "{\"from_frame\":" << start.frame << ",\"to_frame\":"
      << end.frame << ",\"from_state\":";
  write_state(out, start);
  out << ",\"to_state\":";
  write_state(out, end);
  out << ",\"delta\":";
  write_delta(out, link.measurement);
  out << ",\"covariance\":";
  write_matrix(out, link.measurement.get_cov());
  out << ",\"sqrt_information\":";
  write_matrix(out, sqrt_info);
  out << ",\"raw_residual\":";
  write_vector(out, link.raw);
  out << ",\"jacobian_state0\":";
  write_matrix(out, link.jac0);
  out << ",\"jacobian_state1\":";
  write_matrix(out, link.jac1);
  out << ",\"jacobian_bg\":";
  write_matrix(out, link.jac_bg);
  out << ",\"jacobian_ba\":";
  write_matrix(out, link.jac_ba);
  out << ",\"whitened_residual\":";
  write_vector(out, link.whitened_raw);
  out << ",\"whitened_jacobian\":";
  write_matrix(out, link.whitened_jac);
  out << ",\"row_h\":";
  write_matrix(out, link.link_h);
  out << ",\"row_b\":";
  write_vector(out, link.full_b);
  out << ",\"objective\":"
      << Scalar(0.5) * link.whitened_raw.squaredNorm() << "}";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "usage: m7aq_upstream_imu_audit <imu.csv> <states.tsv> "
                 "<output.json>\n";
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
    std::vector<LinkAudit> links;
    links.reserve(4);
    for (size_t i = 0; i + 1 < states.size(); ++i) {
      links.push_back(integrate_link(states[i], states[i + 1], samples,
                                     accel_cov, gyro_cov, gravity));
    }
    Mat75 accumulated_h = Mat75::Zero();
    Vec75 accumulated_b = Vec75::Zero();
    for (size_t i = 0; i < links.size(); ++i)
      add_link_to_global(accumulated_h, accumulated_b, links[i], i, i + 1);

    std::ofstream out(argv[3]);
    if (!out) throw std::runtime_error("cannot open output");
    out << std::setprecision(17);
    out << "{\"schema\":\"basalt.imu_link_audit.v1\","
           "\"scalar\":\"float\",\"links\":[";
    for (size_t i = 0; i < links.size(); ++i) {
      if (i) out << ",";
      write_link(out, links[i], states[i], states[i + 1]);
    }
    out << "],\"accumulated_h\":";
    write_matrix(out, accumulated_h);
    out << ",\"accumulated_b\":";
    write_vector(out, accumulated_b);
    out << "}\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "m7aq upstream audit failed: " << error.what() << "\n";
    return 1;
  }
}
