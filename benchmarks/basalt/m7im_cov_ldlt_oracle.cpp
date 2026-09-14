// Diagnostic-only M7 IMU covariance/whitening oracle.
//
// This source is compiled against the pinned Basalt headers only.  It does
// not replace or modify the pinned checkout or any production Rust source.
// The packet records expose the exact covariance, Eigen LDLT contract, and
// square-root inverse used by IntegratedImuMeasurement<float>.  The final
// link (3 -> 4) also exposes the factor J/r/H/b boundary needed by the IMU
// agent after the covariance recursion reaches its 0/810 gate.

#include <basalt/calibration/calib_bias.hpp>
#include <basalt/imu/imu_types.h>
#include <basalt/imu/preintegration.h>

#include <Eigen/Core>

#include <cstdint>
#include <cmath>
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

using Scalar = float;
using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
using Vec9 = Eigen::Matrix<Scalar, basalt::POSE_VEL_SIZE, 1>;
using Mat9 = Eigen::Matrix<Scalar, basalt::POSE_VEL_SIZE,
                            basalt::POSE_VEL_SIZE>;
using Mat93 = Eigen::Matrix<Scalar, basalt::POSE_VEL_SIZE, 3>;
using Mat30 = Eigen::Matrix<Scalar, basalt::POSE_VEL_SIZE, 30>;
using Vec30 = Eigen::Matrix<Scalar, 30, 1>;
using Mat30x30 = Eigen::Matrix<Scalar, 30, 30>;

struct Sample {
  std::int64_t t_ns = 0;
  Vec3 gyro = Vec3::Zero();
  Vec3 accel = Vec3::Zero();
};

struct State {
  int frame = 0;
  std::int64_t t_ns = 0;
  Vec3 translation = Vec3::Zero();
  Eigen::Quaternion<Scalar> rotation = Eigen::Quaternion<Scalar>::Identity();
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
    std::int64_t t_ns = 0;
    double wx = 0, wy = 0, wz = 0, ax = 0, ay = 0, az = 0;
    if (!(row >> t_ns >> wx >> wy >> wz >> ax >> ay >> az))
      throw std::runtime_error("malformed IMU CSV row");
    Sample sample;
    sample.t_ns = t_ns;
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

std::string bits(Scalar value) {
  std::uint32_t raw = 0;
  static_assert(sizeof(raw) == sizeof(value));
  std::memcpy(&raw, &value, sizeof(raw));
  std::ostringstream out;
  out << std::hex << std::setw(8) << std::setfill('0') << raw;
  return out.str();
}

template <typename Derived>
void write_vector_bits(std::ostream& out,
                       const Eigen::MatrixBase<Derived>& value) {
  out << "[";
  for (Eigen::Index i = 0; i < value.size(); ++i) {
    if (i) out << ",";
    out << '"' << bits(static_cast<Scalar>(value.derived().coeff(i))) << '"';
  }
  out << "]";
}

template <typename Derived>
void write_matrix_bits(std::ostream& out,
                       const Eigen::MatrixBase<Derived>& value) {
  out << "{\"rows\":" << value.rows() << ",\"cols\":" << value.cols()
      << ",\"storage_order\":\"column_major\",\"bits_row_major\":[";
  for (Eigen::Index row = 0; row < value.rows(); ++row) {
    for (Eigen::Index col = 0; col < value.cols(); ++col) {
      if (row || col) out << ",";
      out << '"' << bits(static_cast<Scalar>(value.derived()(row, col)))
          << '"';
    }
  }
  out << "],\"bits_column_major\":[";
  for (Eigen::Index col = 0; col < value.cols(); ++col) {
    for (Eigen::Index row = 0; row < value.rows(); ++row) {
      if (col || row) out << ",";
      out << '"' << bits(static_cast<Scalar>(value.derived()(row, col)))
          << '"';
    }
  }
  out << "]}";
}

void write_ldlt(std::ostream& out, const Mat9& covariance,
                const Mat9& sqrt_information) {
  const auto ldlt = covariance.ldlt();
  const Mat9 matrix_l = ldlt.matrixL();
  const Mat9 matrix_u = ldlt.matrixU();
  Mat9 reconstructed = Mat9::Identity();
  reconstructed = ldlt.transpositionsP() * reconstructed;
  ldlt.matrixL().solveInPlace(reconstructed);
  Vec9 d_inv_sqrt;
  for (size_t i = 0; i < basalt::POSE_VEL_SIZE; ++i) {
    if (ldlt.vectorD()[i] < std::numeric_limits<Scalar>::min()) {
      d_inv_sqrt[i] = 0;
    } else {
      d_inv_sqrt[i] = Scalar(1.0) / sqrt(ldlt.vectorD()[i]);
    }
  }
  reconstructed = d_inv_sqrt.asDiagonal() * reconstructed;

  Mat9 solved = Mat9::Identity();
  ldlt.solveInPlace(solved);

  out << "{\"matrixLDLT\":";
  write_matrix_bits(out, ldlt.matrixLDLT());
  out << ",\"matrixL\":";
  write_matrix_bits(out, matrix_l);
  out << ",\"matrixU\":";
  write_matrix_bits(out, matrix_u);
  out << ",\"vectorD\":";
  write_vector_bits(out, ldlt.vectorD());
  out << ",\"D_inv_sqrt\":";
  write_vector_bits(out, d_inv_sqrt);
  out << ",\"transpositionsP\":[";
  for (size_t i = 0; i < basalt::POSE_VEL_SIZE; ++i) {
    if (i) out << ",";
    out << ldlt.transpositionsP().coeff(i);
  }
  out << "],\"isPositive\":" << (ldlt.isPositive() ? "true" : "false")
      << ",\"solve_identity\":";
  write_matrix_bits(out, solved);
  out << ",\"sqrt_reconstructed\":";
  write_matrix_bits(out, reconstructed);
  out << ",\"sqrt_cached\":";
  write_matrix_bits(out, sqrt_information);
  out << ",\"sqrt_reconstructed_exact\":"
      << (reconstructed == sqrt_information ? "true" : "false") << "}";
}

struct PacketRecord {
  std::int64_t sample_t_ns = 0;
  std::int64_t dt_ns = 0;
  Mat9 covariance_before = Mat9::Zero();
  Mat9 covariance_after = Mat9::Zero();
  Mat9 sqrt_information = Mat9::Zero();
  Mat9 ldlt_matrix = Mat9::Zero();
  Vec9 ldlt_d = Vec9::Zero();
  std::vector<int> pivots;
  Mat9 solve_identity = Mat9::Zero();
};

void write_packet(std::ostream& out, const PacketRecord& packet) {
  out << "{\"sample_t_ns\":" << packet.sample_t_ns << ",\"dt_ns\":"
      << packet.dt_ns << ",\"covariance_before\":";
  write_matrix_bits(out, packet.covariance_before);
  out << ",\"covariance_after\":";
  write_matrix_bits(out, packet.covariance_after);
  out << ",\"sqrt_information\":";
  write_matrix_bits(out, packet.sqrt_information);
  out << ",\"ldlt\":";
  write_ldlt(out, packet.covariance_after, packet.sqrt_information);
  out << "}";
}

void write_factor(std::ostream& out,
                  const basalt::IntegratedImuMeasurement<Scalar>& measurement,
                  const State& start, const State& end, const Vec3& gravity,
                  size_t packet_count) {
  Mat9 jac0 = Mat9::Zero();
  Mat9 jac1 = Mat9::Zero();
  Mat93 jac_bg = Mat93::Zero();
  Mat93 jac_ba = Mat93::Zero();
  const Vec9 raw = measurement.residual(
      start.pose_vel_state(), gravity, end.pose_vel_state(), start.bias_gyro,
      start.bias_accel, &jac0, &jac1, &jac_bg, &jac_ba);
  const auto& sqrt_information = measurement.get_sqrt_cov_inv();
  Mat30 stacked = Mat30::Zero();
  stacked.leftCols<9>() = jac0;
  stacked.middleCols<3>(9) = jac_bg;
  stacked.middleCols<3>(12) = jac_ba;
  stacked.middleCols<9>(15) = jac1;
  const Vec9 whitened_raw = sqrt_information * raw;
  const Mat30 whitened_jac = sqrt_information * stacked;
  const Mat30x30 row_h = whitened_jac.transpose() * whitened_jac;
  const Vec30 row_b = whitened_jac.transpose() * whitened_raw;
  out << "{\"from_frame\":" << start.frame << ",\"to_frame\":"
      << end.frame << ",\"packet_count\":" << packet_count
      << ",\"duration_ns\":" << measurement.get_dt_ns()
      << ",\"covariance\":";
  write_matrix_bits(out, measurement.get_cov());
  out << ",\"sqrt_information\":";
  write_matrix_bits(out, sqrt_information);
  out << ",\"raw_residual\":";
  write_vector_bits(out, raw);
  out << ",\"jacobian_state0\":";
  write_matrix_bits(out, jac0);
  out << ",\"jacobian_state1\":";
  write_matrix_bits(out, jac1);
  out << ",\"jacobian_bg\":";
  write_matrix_bits(out, jac_bg);
  out << ",\"jacobian_ba\":";
  write_matrix_bits(out, jac_ba);
  out << ",\"jacobian\":";
  write_matrix_bits(out, stacked);
  out << ",\"whitened_residual\":";
  write_vector_bits(out, whitened_raw);
  out << ",\"whitened_jacobian\":";
  write_matrix_bits(out, whitened_jac);
  out << ",\"row_h\":";
  write_matrix_bits(out, row_h);
  out << ",\"row_b\":";
  write_vector_bits(out, row_b);
  out << ",\"objective_bits\":\""
      << bits(Scalar(0.5) * whitened_raw.squaredNorm()) << "\"}";
}

void write_packet_array(std::ostream& out, const std::vector<PacketRecord>& xs) {
  out << "[";
  for (size_t i = 0; i < xs.size(); ++i) {
    if (i) out << ",";
    out << "{\"packet\":" << (i + 1) << ",\"record\":";
    write_packet(out, xs[i]);
    out << "}";
  }
  out << "]";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "usage: m7im_cov_ldlt_oracle <imu.csv> <states.tsv> "
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

    std::vector<PacketRecord> first_link_packets;
    basalt::IntegratedImuMeasurement<Scalar> frame4_measurement(
        states[3].t_ns, states[3].bias_gyro, states[3].bias_accel);
    for (size_t link = 0; link < 4; ++link) {
      basalt::IntegratedImuMeasurement<Scalar> measurement(
          states[link].t_ns, states[link].bias_gyro, states[link].bias_accel);
      std::int64_t previous_t_ns = states[link].t_ns;
      for (const Sample& sample : samples) {
        if (sample.t_ns <= states[link].t_ns ||
            sample.t_ns > states[link + 1].t_ns)
          continue;
        PacketRecord packet;
        packet.sample_t_ns = sample.t_ns;
        packet.dt_ns = sample.t_ns - previous_t_ns;
        previous_t_ns = sample.t_ns;
        packet.covariance_before = measurement.get_cov();
        basalt::ImuData<Scalar> data;
        data.t_ns = sample.t_ns;
        data.gyro = sample.gyro;
        data.accel = sample.accel;
        measurement.integrate(data, accel_cov, gyro_cov);
        packet.covariance_after = measurement.get_cov();
        packet.sqrt_information = measurement.get_sqrt_cov_inv();
        if (link == 0) first_link_packets.push_back(packet);
        if (link == 3) {
          // Re-run the exact frame-4 interval below from the same samples;
          // this assignment only keeps the compiler from eliding the final
          // link's audited integration in optimized diagnostic builds.
          frame4_measurement = measurement;
        }
      }
      if (link == 3) {
        frame4_measurement = measurement;
      }
    }
    if (first_link_packets.size() != 10)
      throw std::runtime_error("expected ten first-link IMU packets");

    // Reconstruct frame 3 -> 4 independently, so the factor record is
    // explicitly tied to the final ten-packet interval and not an alias of
    // the packet capture loop above.
    frame4_measurement = basalt::IntegratedImuMeasurement<Scalar>(
        states[3].t_ns, states[3].bias_gyro, states[3].bias_accel);
    size_t frame4_packet_count = 0;
    for (const Sample& sample : samples) {
      if (sample.t_ns <= states[3].t_ns || sample.t_ns > states[4].t_ns)
        continue;
      basalt::ImuData<Scalar> data;
      data.t_ns = sample.t_ns;
      data.gyro = sample.gyro;
      data.accel = sample.accel;
      frame4_measurement.integrate(data, accel_cov, gyro_cov);
      ++frame4_packet_count;
    }
    if (frame4_packet_count != 10)
      throw std::runtime_error("expected ten frame4 IMU packets");
    if (frame4_measurement.get_dt_ns() !=
        states[4].t_ns - states[3].t_ns)
      throw std::runtime_error("frame4 interval did not integrate ten packets");

    std::ofstream out(argv[3]);
    if (!out) throw std::runtime_error("cannot open output");
    out << "{\"schema\":\"basalt.m7im_cov_ldlt.v1\","
           "\"scalar\":\"float\",\"state_dim\":9,"
           "\"whitener_contract\":{"
           "\"decomposition\":\"Eigen Matrix<float,9,9>::ldlt()\","
           "\"storage_order\":\"column_major\","
           "\"sqrt_formula\":\"D^-1/2 * L^-1 * P\","
           "\"pivot_order\":\"ldlt.transpositionsP() applied before \","
           "\"pivot_zero_rule\":\"D[i] < numeric_limits<float>::min() => 0\","
           "\"solve_order\":\"Eigen LDLT in-place triangular solves\"},"
           "\"first_link_packets\":";
    write_packet_array(out, first_link_packets);
    out << ",\"frame4_factor\":";
    write_factor(out, frame4_measurement, states[3], states[4], gravity,
                 frame4_packet_count);
    out << "}\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "m7im oracle failed: " << error.what() << "\n";
    return 1;
  }
}
