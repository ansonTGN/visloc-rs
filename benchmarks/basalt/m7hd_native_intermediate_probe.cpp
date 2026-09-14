// Standalone native oracle driver for the first clean MH_01 IMU link.
// The build script supplies a generated copy of the pinned preintegration
// header with diagnostic hooks; the production header and binary are not
// modified.

#include <basalt/calibration/calib_bias.hpp>
#include <basalt/imu/imu_types.h>
#include <basalt/imu/preintegration.h>

#include <Eigen/Core>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using Scalar = float;
using Vec3 = Eigen::Matrix<Scalar, 3, 1>;

struct Sample {
  int64_t t_ns = 0;
  Vec3 gyro = Vec3::Zero();
  Vec3 accel = Vec3::Zero();
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
    const Vec3 gyro_raw(static_cast<Scalar>(wx), static_cast<Scalar>(wy),
                        static_cast<Scalar>(wz));
    const Vec3 accel_raw(static_cast<Scalar>(ax), static_cast<Scalar>(ay),
                         static_cast<Scalar>(az));
    Sample sample;
    sample.t_ns = t_ns;
    sample.gyro = gyro_calib.getCalibrated(gyro_raw);
    sample.accel = accel_calib.getCalibrated(accel_raw);
    samples.push_back(sample);
  }
  return samples;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "usage: m7hd_native_intermediate_probe <imu.csv> "
                 "<start_ns> <end_ns>\n";
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

    const int64_t start_ns = std::stoll(argv[2]);
    const int64_t end_ns = std::stoll(argv[3]);
    const auto samples = read_samples(argv[1], accel_calib, gyro_calib);

    const Vec3 accel_noise =
        Vec3::Constant(Scalar(0.016) * std::sqrt(Scalar(200)));
    const Vec3 gyro_noise =
        Vec3::Constant(Scalar(0.000282) * std::sqrt(Scalar(200)));
    const Vec3 accel_cov = accel_noise.array().square();
    const Vec3 gyro_cov = gyro_noise.array().square();

    basalt::IntegratedImuMeasurement<Scalar> measurement(
        start_ns, Vec3::Zero(), Vec3::Zero());
    std::size_t count = 0;
    for (const Sample& sample : samples) {
      if (sample.t_ns <= start_ns || sample.t_ns > end_ns) continue;
      basalt::ImuData<Scalar> data;
      data.t_ns = sample.t_ns;
      data.gyro = sample.gyro;
      data.accel = sample.accel;
      measurement.integrate(data, accel_cov, gyro_cov);
      ++count;
    }
    std::cout << "native_packets=" << count
              << " native_dt_ns=" << measurement.get_dt_ns() << '\n';
    if (count != 10 || measurement.get_dt_ns() != end_ns - start_ns)
      throw std::runtime_error("clean link did not contain ten packets");
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "m7hd native probe failed: " << error.what() << '\n';
    return 1;
  }
}
