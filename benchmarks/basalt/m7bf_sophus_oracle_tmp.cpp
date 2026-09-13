#include <basalt/calibration/calibration.hpp>
#include <basalt/serialization/headers_serialization.h>
#include <sophus/se3.hpp>

#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>

namespace basalt {
template <class Scalar>
Sophus::SE3<Scalar> computeRelPose(
    const Sophus::SE3<Scalar>& T_w_i_h, const Sophus::SE3<Scalar>& T_i_c_h,
    const Sophus::SE3<Scalar>& T_w_i_t, const Sophus::SE3<Scalar>& T_i_c_t,
    Sophus::Matrix6<Scalar>* d_rel_d_h = nullptr,
    Sophus::Matrix6<Scalar>* d_rel_d_t = nullptr);
}

namespace {
uint32_t bits(float x) {
  uint32_t out;
  std::memcpy(&out, &x, sizeof(out));
  return out;
}
template <typename Derived>
void vec(const char* name, const Eigen::MatrixBase<Derived>& value) {
  std::cout << name << '=';
  for (Eigen::Index i = 0; i < value.size(); ++i) {
    if (i) std::cout << ',';
    std::cout << std::hex << bits(value.derived()(i));
  }
  std::cout << std::dec << '\n';
}
void pose(const char* name, const Sophus::SE3f& value) {
  vec(name, value.unit_quaternion().coeffs());
  std::string t = std::string(name) + "_t";
  vec(t.c_str(), value.translation());
}
}

int main() {
  basalt::Calibration<double> calib_d;
  std::ifstream input("/root/visloc-basalt-oracle-0f3b2b52/data/euroc_ds_calib.json");
  cereal::JSONInputArchive archive(input);
  archive(calib_d);
  const auto calib = calib_d.cast<float>();
  pose("calib0", calib.T_i_c[0]);
  pose("calib1", calib.T_i_c[1]);

  using SE3 = Sophus::SE3f;
  using Q = Eigen::Quaternionf;
  const SE3 host(
      Q(0.5944822430610657f, -0.052778493613004684f,
        -0.8023747801780701f, 0.0f),
      Eigen::Vector3f::Zero());
  const SE3 target(
      Q(0.5954498648643494f, -0.05392327159643173f,
        -0.801576554775238f, -0.002603980479761958f),
      Eigen::Vector3f(0.0003828657791018486f, -9.85765946097672e-5f,
                      -0.002031802199780941f));
  pose("host", host);
  pose("target", target);

  const auto target_camera_from_imu = calib.T_i_c[1].inverse();
  const auto target_imu_from_anchor_imu =
      Sophus::SE3f(target.so3().inverse() * host.so3(),
                   target.so3().inverse() *
                       (host.translation() - target.translation()));
  const auto target_camera_from_anchor_imu =
      target_camera_from_imu * target_imu_from_anchor_imu;
  const auto target_camera_from_anchor_camera =
      target_camera_from_anchor_imu * calib.T_i_c[0];
  pose("target_camera_from_imu", target_camera_from_imu);
  pose("target_imu_from_anchor_imu", target_imu_from_anchor_imu);
  pose("target_camera_from_anchor_imu", target_camera_from_anchor_imu);
  pose("target_camera_from_anchor_camera", target_camera_from_anchor_camera);

  Sophus::Matrix6<float> d_h;
  Sophus::Matrix6<float> d_t;
  const auto first = basalt::computeRelPose(
      host, calib.T_i_c[0], target, calib.T_i_c[1], &d_h, &d_t);
  pose("compute_rel_with_jac", first);
  const auto rel = basalt::computeRelPose(
      host, calib.T_i_c[0], target, calib.T_i_c[1]);
  pose("compute_rel", rel);
}
