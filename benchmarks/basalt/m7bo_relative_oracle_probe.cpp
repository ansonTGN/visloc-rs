#include <basalt/calibration/calibration.hpp>
#include <basalt/serialization/headers_serialization.h>
#include <basalt/utils/ba_utils.h>
#include <sophus/se3.hpp>

#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

namespace {

struct PoseStateLike {
  Sophus::SE3f pose_lin;
  Sophus::SE3f pose;
  bool linearized = true;
  const Sophus::SE3f& getPoseLin() const { return pose_lin; }
  const Sophus::SE3f& getPose() const { return pose; }
  bool isLinearized() const { return linearized; }
};

Sophus::SE3f runtime_like(const PoseStateLike& state_h,
                          const Sophus::SE3f& c0,
                          const PoseStateLike& state_t,
                          const Sophus::SE3f& c1) {
  Sophus::Matrix6<float> d_h;
  Sophus::Matrix6<float> d_t;
  Sophus::SE3f transform = basalt::computeRelPose(
      state_h.getPoseLin(), c0, state_t.getPoseLin(), c1, &d_h, &d_t);
  if (state_h.isLinearized() || state_t.isLinearized()) {
    transform = basalt::computeRelPose(state_h.getPose(), c0,
                                       state_t.getPose(), c1);
  }
  return transform;
}

uint32_t bits(float value) {
  uint32_t out;
  std::memcpy(&out, &value, sizeof(out));
  return out;
}

template <typename Derived>
void vec_bits(const char* name, const Eigen::MatrixBase<Derived>& value) {
  std::cout << name << "=";
  for (Eigen::Index i = 0; i < value.size(); ++i) {
    if (i) std::cout << ',';
    std::cout << std::hex << bits(static_cast<float>(value.derived()(i)));
  }
  std::cout << std::dec << '\n';
}

void pose_bits(const char* name, const Sophus::SE3f& value) {
  vec_bits(name, value.unit_quaternion().coeffs());
  const std::string translation_name = std::string(name) + "_t";
  vec_bits(translation_name.c_str(), value.translation());
}

template <typename Derived>
void vec_decimal(const char* name, const Eigen::MatrixBase<Derived>& value) {
  std::cout << std::setprecision(17) << name << "=" << value.transpose()
            << '\n';
}

}  // namespace

int main() {
  basalt::Calibration<double> calibration_double;
  std::ifstream input(
      "/root/visloc-basalt-oracle-0f3b2b52/data/euroc_ds_calib.json");
  cereal::JSONInputArchive archive(input);
  archive(calibration_double);
  const basalt::Calibration<float> calibration = calibration_double.cast<float>();

  std::cout << "probe=basalt_m7bo_relative_oracle_v1\n";
  std::cout << "compiler=" << __VERSION__ << "\n";
#ifdef __OPTIMIZE__
  std::cout << "__OPTIMIZE__=1\n";
#else
  std::cout << "__OPTIMIZE__=0\n";
#endif
#ifdef __FMA__
  std::cout << "__FMA__=1\n";
#else
  std::cout << "__FMA__=0\n";
#endif
#ifdef EIGEN_DONT_PARALLELIZE
  std::cout << "EIGEN_DONT_PARALLELIZE=1\n";
#else
  std::cout << "EIGEN_DONT_PARALLELIZE=0\n";
#endif
#ifdef EIGEN_INITIALIZE_MATRICES_BY_NAN
  std::cout << "EIGEN_INITIALIZE_MATRICES_BY_NAN=1\n";
#else
  std::cout << "EIGEN_INITIALIZE_MATRICES_BY_NAN=0\n";
#endif

  pose_bits("calib0", calibration.T_i_c[0]);
  pose_bits("calib1", calibration.T_i_c[1]);

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
  pose_bits("host", host);
  pose_bits("target", target);

  const SE3 target_camera_from_imu = calibration.T_i_c[1].inverse();
  const SE3 target_imu_from_anchor_imu =
      SE3(target.so3().inverse() * host.so3(),
          target.so3().inverse() *
              (host.translation() - target.translation()));
  const SE3 target_camera_from_anchor_imu =
      target_camera_from_imu * target_imu_from_anchor_imu;
  const SE3 target_camera_from_anchor_camera =
      target_camera_from_anchor_imu * calibration.T_i_c[0];
  pose_bits("target_camera_from_imu", target_camera_from_imu);
  pose_bits("target_imu_from_anchor_imu", target_imu_from_anchor_imu);
  pose_bits("target_camera_from_anchor_imu", target_camera_from_anchor_imu);
  pose_bits("target_camera_from_anchor_camera",
            target_camera_from_anchor_camera);

  const SE3 rel = basalt::computeRelPose(
      host, calibration.T_i_c[0], target, calibration.T_i_c[1]);
  pose_bits("compute_rel", rel);
  Sophus::Matrix6<float> d_h;
  Sophus::Matrix6<float> d_t;
  const SE3 rel_with_jac = basalt::computeRelPose(
      host, calibration.T_i_c[0], target, calibration.T_i_c[1], &d_h, &d_t);
  pose_bits("compute_rel_with_jac", rel_with_jac);
  PoseStateLike state_h{host, host, true};
  PoseStateLike state_t{target, target, true};
  pose_bits("runtime_like", runtime_like(state_h, calibration.T_i_c[0],
                                          state_t, calibration.T_i_c[1]));

  basalt::Keypoint<float> landmark;
  landmark.direction = Eigen::Vector2f(-0.39586612582206726f,
                                        -0.1799013614654541f);
  landmark.inv_dist = 0.14654140174388885f;
  const Eigen::Vector2f observation(27.31320571899414f,
                                    106.39038848876953f);
  Eigen::Vector2f residual;
  Eigen::Matrix<float, 4, 1> projection;
  Eigen::Matrix<float, 4, 1> host_point =
      basalt::StereographicParam<float>::unproject(landmark.direction);
  host_point[3] = landmark.inv_dist;
  const Eigen::Matrix<float, 4, 1> target_point = rel.matrix() * host_point;
  Eigen::Vector2f projected;
  calibration.intrinsics[1].project(target_point, projected);
  residual = projected - observation;
  projection.head<2>() = projected;
  projection[2] = target_point[3] / target_point.head<3>().norm();
  vec_decimal("host_point", host_point);
  vec_decimal("target_point", target_point);
  vec_decimal("landmark_direction", landmark.direction);
  std::cout << std::setprecision(17) << "landmark_inv_dist="
            << landmark.inv_dist << '\n';
  vec_decimal("observation", observation);
  vec_decimal("projection", projection);
  vec_decimal("residual", residual);
  vec_bits("projection_bits", projection);
  vec_bits("residual_bits", residual);

  const SE3 runtime_rel(
      Q(0.99995714426040649f, -0.0086768483743071556f,
        -0.0031813983805477619f, -0.00048895552754402161f),
      Eigen::Vector3f(-0.11029497534036636f, -0.0018457286059856415f,
                      -0.0010396065190434456f));
  pose_bits("runtime_rel_literal", runtime_rel);
  const Eigen::Matrix<float, 4, 1> runtime_target_point =
      runtime_rel.matrix() * host_point;
  Eigen::Vector2f runtime_projected;
  calibration.intrinsics[1].project(runtime_target_point, runtime_projected);
  const Eigen::Vector2f runtime_residual = runtime_projected - observation;
  vec_decimal("runtime_target_point", runtime_target_point);
  vec_decimal("runtime_projected", runtime_projected);
  vec_decimal("runtime_residual", runtime_residual);
  vec_bits("runtime_projected_bits", runtime_projected);
  vec_bits("runtime_residual_bits", runtime_residual);
}
