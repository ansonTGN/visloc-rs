// Diagnostic-only f32 visual-factor boundary probe for pinned Basalt/Sophus.
// Inputs are the authoritative frame-0/frame-1 state and track-1 fixture from
// target/m7at_current_upstream_f4.jsonl.  It keeps each Sophus inverse/product
// and Double-Sphere projection stage visible in the pinned Eigen operation
// order.

#include <sophus/se3.hpp>
#include <basalt/camera/double_sphere_camera.hpp>
#include <basalt/utils/ba_utils.h>

#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
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

void print_so3(const char* name, const Sophus::SO3f& value) {
  print_bits(name, value.unit_quaternion().coeffs());
}

void print_se3(const char* name, const Sophus::SE3f& value) {
  print_so3(name, value.so3());
  print_bits((std::string(name) + "_t").c_str(), value.translation());
}

}  // namespace

int main() {
  using Scalar = float;
  using Vec2 = Eigen::Matrix<Scalar, 2, 1>;
  using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
  using Mat2x3 = Eigen::Matrix<Scalar, 2, 3>;
  using Quaternion = Eigen::Quaternion<Scalar>;
  using SO3 = Sophus::SO3<Scalar>;
  using SE3 = Sophus::SE3<Scalar>;

  const Quaternion anchor_q(Scalar(0.5944822430610657),
                            Scalar(-0.052778493613004684),
                            Scalar(-0.8023747801780701), Scalar(0));
  const Quaternion target_q(Scalar(0.5954498648643494),
                            Scalar(-0.05392327159643173),
                            Scalar(-0.801576554775238),
                            Scalar(-0.002603980479761958));
  const Vec3 target_t(Scalar(0.0003828657791018486),
                      Scalar(-9.85765946097672e-5),
                      Scalar(-0.002031802199780941));
  const SE3 anchor(anchor_q, Vec3::Zero());
  const SE3 target(target_q, target_t);
  print_so3("anchor_state", anchor.so3());
  print_so3("target_state", target.so3());

  const Quaternion anchor_extrinsic_q(Scalar(0.7123125505904486),
                                      Scalar(-0.007239825785317818),
                                      Scalar(0.007541278561558601),
                                      Scalar(0.7017845426564943));
  const Quaternion target_extrinsic_q(Scalar(0.7115930283929829),
                                      Scalar(-0.0023360576185881625),
                                      Scalar(0.013000769689092388),
                                      Scalar(0.7024677108343111));
  const SE3 anchor_extrinsic(
      anchor_extrinsic_q,
      Vec3(Scalar(-0.016774788924641534), Scalar(-0.068938940687127),
           Scalar(0.005139123188382424)));
  const SE3 target_extrinsic(
      target_extrinsic_q,
      Vec3(Scalar(-0.01507436282032619), Scalar(0.0412627204046637),
           Scalar(0.00316287258752953)));
  print_so3("anchor_extrinsic", anchor_extrinsic.so3());
  print_so3("target_extrinsic", target_extrinsic.so3());

  const SE3 target_camera_from_imu = target_extrinsic.inverse();
  // This is Basalt's computeRelPose order, which keeps the IMU relative
  // translation as R_t^{-1} * (t_h - t_t) before the extrinsic products.
  const SO3 target_imu_rotation_inverse = target.so3().inverse();
  const SE3 target_imu_from_anchor_imu(
      target_imu_rotation_inverse * anchor.so3(),
      target_imu_rotation_inverse *
          (anchor.translation() - target.translation()));
  const SE3 target_camera_from_anchor_imu =
      target_camera_from_imu * target_imu_from_anchor_imu;
  const SE3 target_camera_from_anchor_camera =
      target_camera_from_anchor_imu * anchor_extrinsic;
  print_se3("target_camera_from_imu", target_camera_from_imu);
  print_se3("target_imu_from_anchor_imu", target_imu_from_anchor_imu);
  print_se3("target_camera_from_anchor_imu", target_camera_from_anchor_imu);
  print_se3("target_camera_from_anchor_camera", target_camera_from_anchor_camera);

  const Scalar u = Scalar(-0.39586612582206726);
  const Scalar v = Scalar(-0.1799013614654541);
  const Scalar rho = Scalar(0.14654140174388885);
  const Scalar x2 = u * u;
  const Scalar y2 = v * v;
  const Scalar r2 = x2 + y2;
  const Scalar norm_inv = Scalar(2) / (Scalar(1) + r2);
  const Vec3 bearing_unit(u * norm_inv, v * norm_inv, norm_inv - Scalar(1));
  Eigen::Matrix<Scalar, 4, 1> p_h;
  p_h.template head<3>() = bearing_unit;
  p_h[3] = rho;
  const Eigen::Matrix<Scalar, 4, 4> T_t_h = target_camera_from_anchor_camera.matrix();
  const Eigen::Matrix<Scalar, 4, 1> p_t = T_t_h * p_h;
  const Vec3 point = p_t.template head<3>();
  print_bits("bearing", bearing_unit);
  print_bits("p_h", p_h);
  print_bits("T_t_h", T_t_h);
  print_bits("p_t", p_t);
  print_bits("point", point);

  Eigen::Matrix<Scalar, 4, 2> source_jup;
  basalt::StereographicParam<Scalar>::unproject(Vec2(u, v), &source_jup);
  Eigen::Matrix<Scalar, 4, 3> source_jpp;
  source_jpp.setZero();
  source_jpp.template block<3, 2>(0, 0) =
      T_t_h.template topLeftCorner<3, 4>() * source_jup;
  source_jpp.col(2) = T_t_h.col(3);
  print_bits("source_Jup", source_jup);
  print_bits("source_Jpp", source_jpp);

  const Scalar fx = Scalar(361.6713883800533);
  const Scalar fy = Scalar(360.5856493689301);
  const Scalar cx = Scalar(379.40818394080869);
  const Scalar cy = Scalar(255.9772968522045);
  const Scalar xi = Scalar(-0.21300835384809328);
  const Scalar alpha = Scalar(0.5767008625037023);
  const Scalar xx = point.x() * point.x();
  const Scalar yy = point.y() * point.y();
  const Scalar zz = point.z() * point.z();
  const Scalar r2_point = xx + yy;
  const Scalar d1 = std::sqrt(r2_point + zz);
  const Scalar k = xi * d1 + point.z();
  const Scalar kk = k * k;
  const Scalar d2 = std::sqrt(r2_point + kk);
  const Scalar norm = alpha * d2 + (Scalar(1) - alpha) * k;
  const Scalar mx = point.x() / norm;
  const Scalar my = point.y() / norm;
  const Vec2 predicted(fx * mx + cx, fy * my + cy);
  const Vec2 observation(Scalar(27.31320571899414),
                         Scalar(106.39038848876953));
  const Vec2 raw = predicted - observation;

  // Keep the source pose/weight chain visible for the next fixed boundary:
  // linearizePoint returns the relative-pose block; LandmarkBlockAbsDynamic
  // then multiplies it by computeRelPose's host/target blocks and applies the
  // scalar Huber/observation whitening.
  Eigen::Matrix<Scalar, 4, 6> source_d_point_d_xi;
  source_d_point_d_xi.setZero();
  source_d_point_d_xi.template topLeftCorner<3, 3>() =
      Eigen::Matrix<Scalar, 3, 3>::Identity() * rho;
  source_d_point_d_xi.template topRightCorner<3, 3>() =
      -Sophus::SO3<Scalar>::hat(point);

  const Scalar norm_sq = norm * norm;
  const Scalar tt2 = xi * point.z() / d1 + Scalar(1);
  const Scalar d_norm_d_r2 =
      (xi * (Scalar(1) - alpha) / d1 +
       alpha * (xi * k / d1 + Scalar(1)) / d2) /
      norm_sq;
  const Scalar tmp2 =
      ((Scalar(1) - alpha) * tt2 + alpha * k * tt2 / d2) / norm_sq;
  Mat2x3 jacobian;
  jacobian(0, 0) = fx * (Scalar(1) / norm - xx * d_norm_d_r2);
  jacobian(1, 0) = -fy * point.x() * point.y() * d_norm_d_r2;
  jacobian(0, 1) = -fx * point.x() * point.y() * d_norm_d_r2;
  jacobian(1, 1) = fy * (Scalar(1) / norm - yy * d_norm_d_r2);
  jacobian(0, 2) = -fx * point.x() * tmp2;
  jacobian(1, 2) = -fy * point.y() * tmp2;
  print_bits("predicted", predicted);
  print_bits("raw", raw);
  print_bits("projection_jacobian", jacobian);

  basalt::DoubleSphereCamera<Scalar>::VecN params;
  params << fx, fy, cx, cy, xi, alpha;
  const basalt::DoubleSphereCamera<Scalar> camera(params);
  Vec2 camera_predicted;
  Eigen::Matrix<Scalar, 2, 4> camera_jacobian;
  camera.project(p_t, camera_predicted, &camera_jacobian);
  print_bits("camera_project", camera_predicted);
  print_bits("camera_project_jacobian", camera_jacobian);
  print_bits("assembled_source_landmark_jacobian",
             camera_jacobian * source_jpp);

  Eigen::Matrix<Scalar, 2, 6> source_pose_jacobian_from_chain;
  source_pose_jacobian_from_chain = camera_jacobian * source_d_point_d_xi;
  print_bits("source_d_point_d_xi", source_d_point_d_xi);
  print_bits("source_relative_pose_jacobian", source_pose_jacobian_from_chain);

  Eigen::Matrix<Scalar, 6, 6> source_d_rel_d_h;
  Eigen::Matrix<Scalar, 6, 6> source_d_rel_d_t;
  const SE3 source_relative_pose = basalt::computeRelPose(
      anchor, anchor_extrinsic, target, target_extrinsic,
      &source_d_rel_d_h, &source_d_rel_d_t);
  print_se3("source_relative_pose", source_relative_pose);
  const Eigen::Matrix<Scalar, 6, 6> source_adjoint = target_camera_from_anchor_imu.Adj();
  print_bits("source_relative_adjoint", source_adjoint);
  const Eigen::Matrix<Scalar, 3, 3> source_adjoint_cross =
      source_adjoint.template block<3, 3>(0, 3);
  print_bits("source_relative_adjoint_cross",
             source_adjoint_cross);
  print_bits("source_relative_adjoint_cross_times_anchor",
             (source_adjoint_cross * anchor.so3().inverse().matrix()).eval());
  print_bits("source_d_rel_d_h", source_d_rel_d_h);
  print_bits("source_d_rel_d_t", source_d_rel_d_t);
  print_bits("source_raw_d_rel_d_t", (-source_d_rel_d_t).eval());
  print_bits("source_anchor_pose_jacobian",
             source_pose_jacobian_from_chain * source_d_rel_d_h);
  print_bits("source_target_pose_jacobian",
             source_pose_jacobian_from_chain * source_d_rel_d_t);

  const Scalar residual_squared = raw.squaredNorm();
  const Scalar huber_parameter = Scalar(1);
  const Scalar huber_weight = residual_squared <=
                                      huber_parameter * huber_parameter
                                  ? Scalar(1)
                                  : huber_parameter / std::sqrt(residual_squared);
  const Scalar sqrt_weight = std::sqrt(huber_weight) / Scalar(0.5);
  print_bits("source_residual_squared", Eigen::Matrix<Scalar, 1, 1>(residual_squared));
  print_bits("source_huber_weight", Eigen::Matrix<Scalar, 1, 1>(huber_weight));
  print_bits("source_sqrt_weight", Eigen::Matrix<Scalar, 1, 1>(sqrt_weight));

  // LandmarkBlockAbsDynamic scales d_res_d_xi in place before multiplying
  // by the fixed 6x6 absolute-pose chains.  Keep this ordering explicit in
  // the probe so the Rust pose-J audit can distinguish weighted-J*product
  // from product*weighted-J, including signed-zero lanes.
  Eigen::Matrix<Scalar, 2, 6> source_weighted_relative_pose =
      source_pose_jacobian_from_chain;
  source_weighted_relative_pose *= sqrt_weight;
  print_bits("source_weighted_relative_pose", source_weighted_relative_pose);
  print_bits("source_weighted_anchor_pose_jacobian",
             source_weighted_relative_pose * source_d_rel_d_h);
  print_bits("source_weighted_target_pose_jacobian",
             source_weighted_relative_pose * source_d_rel_d_t);

  basalt::Keypoint<Scalar> keypoint;
  keypoint.direction << u, v;
  keypoint.inv_dist = rho;
  Vec2 source_residual;
  Eigen::Matrix<Scalar, 2, 6> source_pose_jacobian;
  Eigen::Matrix<Scalar, 2, 3> source_landmark_jacobian;
  const bool source_valid = basalt::linearizePoint(
      observation, keypoint, T_t_h, camera, source_residual,
      &source_pose_jacobian,
      &source_landmark_jacobian);
  std::cout << "source_valid=" << source_valid << "\n";
  print_bits("source_residual", source_residual);
  print_bits("source_pose_jacobian", source_pose_jacobian);
  print_bits("source_landmark_jacobian", source_landmark_jacobian);
  std::cout << std::setprecision(17);
  std::cout << "predicted_decimal=" << predicted.transpose() << "\n";
  std::cout << "raw_decimal=" << raw.transpose() << "\n";
  return 0;
}
