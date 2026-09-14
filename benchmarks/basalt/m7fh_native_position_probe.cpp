// Diagnostic-only native per-packet position trace for the M7fh frame-3 link.
#include <basalt/calibration/calib_bias.hpp>
#include <basalt/imu/preintegration.h>

#include <bit>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

template <class Derived>
void dump(const char* name, const Eigen::MatrixBase<Derived>& v) {
  std::cout << name << "=";
  for (Eigen::Index i = 0; i < v.size(); ++i) {
    if (i) std::cout << ',';
    std::cout << std::hex << std::bit_cast<std::uint32_t>(v.derived()(i));
  }
  std::cout << std::dec << '\n';
}

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  using S = float;
  using V = Eigen::Matrix<S, 3, 1>;
  constexpr std::int64_t start = 1403636579913555456LL;
  constexpr std::int64_t end = 1403636579963555584LL;
  basalt::CalibAccelBias<S> accel_calib;
  Eigen::Matrix<S, 9, 1> accel_param;
  accel_param << S(-0.003025405479279035), S(0.1200005286487319),
      S(0.06708820471592454), S(0), S(0), S(0), S(0), S(0), S(0);
  accel_calib.getParam() = accel_param;
  basalt::CalibGyroBias<S> gyro_calib;
  Eigen::Matrix<S, 12, 1> gyro_param;
  gyro_param << S(-0.002186848441668376), S(0.020427823167917037),
      S(0.07668367023977922), S(0), S(0), S(0), S(0), S(0), S(0), S(0),
      S(0), S(0);
  gyro_calib.getParam() = gyro_param;
  const V accel_noise = V::Constant(S(0.016) * std::sqrt(S(200)));
  const V gyro_noise = V::Constant(S(0.000282) * std::sqrt(S(200)));
  const V accel_cov = accel_noise.array().square();
  const V gyro_cov = gyro_noise.array().square();
  basalt::IntegratedImuMeasurement<S> measurement(start, V::Zero(), V::Zero());
  std::ifstream input(argv[1]);
  std::string line;
  int count = 0;
  while (std::getline(input, line)) {
    if (line.empty() || line[0] == '#') continue;
    for (char& c : line) if (c == ',') c = ' ';
    std::stringstream row(line);
    std::int64_t t = 0;
    double wx = 0, wy = 0, wz = 0, ax = 0, ay = 0, az = 0;
    if (!(row >> t >> wx >> wy >> wz >> ax >> ay >> az)) return 3;
    if (t <= start || t > end) continue;
    basalt::ImuData<S> data;
    data.t_ns = t;
    data.gyro = gyro_calib.getCalibrated(V(static_cast<S>(wx), static_cast<S>(wy), static_cast<S>(wz)));
    data.accel = accel_calib.getCalibrated(V(static_cast<S>(ax), static_cast<S>(ay), static_cast<S>(az)));
    measurement.integrate(data, accel_cov, gyro_cov);
    ++count;
    dump("p", measurement.getDeltaState().T_w_i.translation());
    dump("v", measurement.getDeltaState().vel_w_i);
  }
  std::cout << "count=" << count << '\n';
  return 0;
}
