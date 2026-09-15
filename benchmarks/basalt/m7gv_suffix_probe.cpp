#include <basalt/calibration/calibration.hpp>
#include <basalt/serialization/headers_serialization.h>
#include <basalt/utils/ba_utils.h>
#include <sophus/se3.hpp>

#include <cstdint>
#include <cstring>
#include <array>
#include <fstream>
#include <iostream>

static uint32_t bits(float value) {
  uint32_t out;
  std::memcpy(&out, &value, sizeof(out));
  return out;
}
static float f32_from_bits(uint32_t value) {
  float out;
  std::memcpy(&out, &value, sizeof(out));
  return out;
}
static Sophus::SE3f raw_pose(std::array<uint32_t, 4> q_xyzw,
                             std::array<uint32_t, 3> t_xyz) {
  Sophus::SE3f pose;
  for (int i = 0; i < 4; ++i) pose.data()[i] = f32_from_bits(q_xyzw[i]);
  for (int i = 0; i < 3; ++i) pose.data()[4 + i] = f32_from_bits(t_xyz[i]);
  return pose;
}
template <typename Derived>
static void dump(const char* name, const Eigen::MatrixBase<Derived>& value) {
  std::cout << name << "=";
  for (Eigen::Index i = 0; i < value.size(); ++i) {
    if (i) std::cout << ',';
    std::cout << std::hex << bits(static_cast<float>(value.derived()(i)));
  }
  std::cout << std::dec << '\n';
}
static void dump_pose(const char* name, const Sophus::SE3f& pose) {
  dump(name, pose.unit_quaternion().coeffs());
  std::string tn = std::string(name) + "_t";
  dump(tn.c_str(), pose.translation());
}

int main() {
  basalt::Calibration<double> calibration_d;
  std::ifstream input("/root/visloc-basalt-oracle-0f3b2b52/data/euroc_ds_calib.json");
  cereal::JSONInputArchive archive(input);
  archive(calibration_d);
  const auto calibration = calibration_d.cast<float>();
  dump_pose("calib0", calibration.T_i_c[0]);
  dump_pose("calib1", calibration.T_i_c[1]);
  using SE3 = Sophus::SE3f;
  using Q = Eigen::Quaternionf;
  const SE3 states[] = {
      raw_pose({0xbd582e43, 0xbf4d686f, 0x00000000, 0x3f182ffd},
               {0x00000000, 0x00000000, 0x00000000}),
      raw_pose({0x00000000, 0x00000000, 0x00000000, 0x3f800000},
               {0x00000000, 0x00000000, 0x00000000}),
      raw_pose({0x00000000, 0x00000000, 0x00000000, 0x3f800000},
               {0x00000000, 0x00000000, 0x00000000}),
      raw_pose({0x00000000, 0x00000000, 0x00000000, 0x3f800000},
               {0x00000000, 0x00000000, 0x00000000}),
      raw_pose({0xbd5f79e6, 0xbf4f45a2, 0xbbb53f68, 0x3f159719},
               {0x3c02c75a, 0xbb0c3bda, 0xbcdc2b7f}),
  };
  // Frame 0 is the host and frame 4 is the target in this bounded audit.
  const SE3 host = states[0];
  const SE3 target = states[4];
  for (int cam = 0; cam < 2; ++cam) {
    Sophus::Matrix6<float> d_h;
    Sophus::Matrix6<float> d_t;
    const auto rel = basalt::computeRelPose(host, calibration.T_i_c[0], target,
                                            calibration.T_i_c[cam], &d_h, &d_t);
    dump_pose(cam == 0 ? "compute_rel_cam0" : "compute_rel_cam1", rel);
    const auto target_cam_from_imu = calibration.T_i_c[cam].inverse();
    const auto body = SE3(target.so3().inverse() * host.so3(),
                          target.so3().inverse() * (host.translation() - target.translation()));
    const auto prefix = target_cam_from_imu * body;
    dump_pose(cam == 0 ? "prefix_cam0" : "prefix_cam1", prefix);
    const auto suffix = prefix * calibration.T_i_c[0];
    dump_pose(cam == 0 ? "suffix_cam0" : "suffix_cam1", suffix);
    dump(cam == 0 ? "rotate_cam0" : "rotate_cam1",
         prefix.so3() * calibration.T_i_c[0].translation());
  }
}
