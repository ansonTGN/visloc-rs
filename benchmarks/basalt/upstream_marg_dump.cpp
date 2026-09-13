// Native Basalt MargData cereal bridge for the pinned oracle checkout.
//
// This is an evidence/interop tool, not production code. It deliberately
// keeps the native timestamp-keyed stored state (linearized branch, current
// branch, delta, and flag) instead of emitting only the effective state. The
// source checkout is selected by the build include paths; the commit string
// below is part of the output contract used by the comparison artifacts.
#include <basalt/serialization/headers_serialization.h>

// Parse the dependencies that contain third-party classes before redefining
// `private`; otherwise a dependency such as oneTBB would see its own private
// labels changed while this inspection-only include is expanded.
#include <basalt/imu/imu_types.h>
#include <basalt/optical_flow/optical_flow.h>
#include <basalt/utils/common_types.h>
#include <basalt/utils/sophus_utils.hpp>

// PoseStateWithLin and PoseVelBiasStateWithLin intentionally keep the stored
// current branch private. The native cereal packet contains that branch, so
// this reader exposes it for inspection without changing the pinned checkout.
// Keep this include after headers_serialization.h: its include guard prevents
// the macro from leaking into third-party headers that serialization pulls in.
#define private public
#include <basalt/utils/imu_types.h>
#undef private

#include <cereal/archives/binary.hpp>

#include <fstream>
#include <iomanip>
#include <iostream>

namespace {

template <class Scalar>
void json_se3(std::ostream& out, const Sophus::SE3<Scalar>& pose) {
  const auto q = pose.unit_quaternion();
  out << "{\"translation\":[" << pose.translation().x() << ','
      << pose.translation().y() << ',' << pose.translation().z()
      << "],\"quaternion_wxyz\":[" << q.w() << ',' << q.x() << ','
      << q.y() << ',' << q.z() << "]}";
}

template <class Derived>
void json_vector(std::ostream& out, const Eigen::MatrixBase<Derived>& vector) {
  out << '[';
  for (Eigen::Index index = 0; index < vector.size(); ++index) {
    if (index != 0) out << ',';
    out << vector(index);
  }
  out << ']';
}

void json_matrix(std::ostream& out, const Eigen::MatrixXd& matrix) {
  // Eigen::MatrixXd is column-major. The storage declaration is important:
  // this is the order in which Rust/Python consumers must compare native
  // abs_H, rather than an implicit row-major re-enumeration.
  out << "{\"rows\":" << matrix.rows() << ",\"cols\":"
      << matrix.cols() << ",\"storage\":\"column_major\",\"data\":[";
  for (Eigen::Index index = 0; index < matrix.size(); ++index) {
    if (index != 0) out << ',';
    out << matrix.data()[index];
  }
  out << "]}";
}

template <class Scalar>
void json_nav_state(std::ostream& out,
                    const basalt::PoseVelBiasState<Scalar>& state,
                    int64_t map_timestamp, bool timestamp_serialized) {
  // MargData's cereal serializer writes state_linearized.t_ns once, but does
  // not write state_current.t_ns. The map timestamp is therefore the
  // authoritative key for every branch; stored_timestamp_ns exposes the
  // decoded in-memory value without pretending that current.t_ns was on disk.
  out << "{\"timestamp_ns\":" << map_timestamp
      << ",\"stored_timestamp_ns\":" << state.t_ns
      << ",\"timestamp_serialized\":"
      << (timestamp_serialized ? "true" : "false") << ",\"pose\":";
  json_se3(out, state.T_w_i);
  out << ",\"velocity\":";
  json_vector(out, state.vel_w_i);
  out << ",\"gyro_bias\":";
  json_vector(out, state.bias_gyro);
  out << ",\"accel_bias\":";
  json_vector(out, state.bias_accel);
  out << '}';
}

void json_unavailable(std::ostream& out, const char* reason) {
  out << "{\"available\":false,\"reason\":\"" << reason << "\"}";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: upstream_marg_dump FILE.cereal\n";
    return 2;
  }

  std::ifstream input(argv[1], std::ios::binary);
  if (!input) {
    std::cerr << "cannot open " << argv[1] << '\n';
    return 3;
  }

  basalt::MargData data;
  cereal::BinaryInputArchive archive(input);
  archive(data);

  std::cout << std::setprecision(17);
  std::cout << "{\"bridge_schema\":\"basalt.native.margdata.bridge.v1\","
               "\"source\":{\"project\":\"basalt\","
               "\"commit\":\"0f3b2b52c807f70ff4e2973ce253c73329eea7bc\","
               "\"archive\":\"cereal.binary\"},"
               "\"native_cereal_order\":[\"aom\",\"abs_H\",\"abs_b\","
               "\"frame_poses\",\"frame_states\",\"kfs_all\","
               "\"kfs_to_marg\",\"use_imu\"],"
               "\"aom_serialized_order\":[\"total_size\",\"items\","
               "\"abs_order_map\"],"
               "\"timestamp_key\":\"nanoseconds\",\"aom\":{"
               "\"total_size\":"
            << data.aom.total_size << ",\"items\":" << data.aom.items
            << ",\"order\":[";

  bool first = true;
  for (const auto& [timestamp, block] : data.aom.abs_order_map) {
    if (!first) std::cout << ',';
    first = false;
    std::cout << "{\"timestamp_ns\":" << timestamp << ",\"offset\":"
              << block.first << ",\"dof\":" << block.second << '}';
  }
  std::cout << "]},\"abs_system\":{"
               "\"source\":\"native MargData.abs_H/abs_b\","
               "\"h\":";
  json_matrix(std::cout, data.abs_H);
  std::cout << ",\"b\":{\"available\":true,\"length\":"
            << data.abs_b.size() << ",\"data\":";
  json_vector(std::cout, data.abs_b);
  std::cout << "}},\"frame_poses\":[";

  first = true;
  for (const auto& [timestamp, pose] : data.frame_poses) {
    if (!first) std::cout << ',';
    first = false;
    std::cout << "{\"timestamp_ns\":" << timestamp
              << ",\"stored_timestamp_ns\":"
              << pose.pose_linearized.t_ns << ",\"pose_linearized\":";
    json_se3(std::cout, pose.pose_linearized.T_w_i);
    std::cout << ",\"pose_current\":";
    json_se3(std::cout, pose.T_w_i_current);
    std::cout << ",\"effective_pose\":";
    json_se3(std::cout, pose.getPose());
    std::cout << ",\"delta\":";
    json_vector(std::cout, pose.delta);
    std::cout << ",\"linearized\":"
              << (pose.linearized ? "true" : "false")
              << ",\"effective_branch\":\""
              << (pose.linearized ? "current" : "linearized") << "\"}";
  }

  std::cout << "],\"frame_states\":[";
  first = true;
  for (const auto& [timestamp, state] : data.frame_states) {
    if (!first) std::cout << ',';
    first = false;
    std::cout << "{\"timestamp_ns\":" << timestamp
              << ",\"stored_timestamp_ns\":"
              << state.state_linearized.t_ns << ",\"state_linearized\":";
    json_nav_state(std::cout, state.state_linearized, timestamp, true);
    std::cout << ",\"state_current\":";
    json_nav_state(std::cout, state.state_current, timestamp, false);
    std::cout << ",\"effective_state\":";
    json_nav_state(std::cout, state.getState(), timestamp, false);
    std::cout << ",\"delta\":";
    json_vector(std::cout, state.delta);
    std::cout << ",\"linearized\":"
              << (state.linearized ? "true" : "false")
              << ",\"effective_branch\":\""
              << (state.linearized ? "current" : "linearized") << "\"}";
  }

  std::cout << "],\"kfs_all\":[";
  first = true;
  for (const auto id : data.kfs_all) {
    if (!first) std::cout << ',';
    first = false;
    std::cout << id;
  }

  std::cout << "],\"kfs_to_marg\":[";
  first = true;
  for (const auto id : data.kfs_to_marg) {
    if (!first) std::cout << ',';
    first = false;
    std::cout << id;
  }

  std::cout << "],\"use_imu\":" << (data.use_imu ? "true" : "false")
            << ",\"rust_schema4_projection\":{\"frame_id_mapping\":";
  json_unavailable(std::cout,
                   "native MargData cereal keys frames by timestamp; Rust "
                   "schema4 requires an external timestamp_to_frame_id map");
  std::cout << ",\"aom_sqrt_jacobian\":";
  json_unavailable(std::cout,
                   "not serialized by native MargData cereal; only abs_H is "
                   "present");
  std::cout << ",\"aom_sqrt_rhs\":";
  json_unavailable(std::cout,
                   "not serialized by native MargData cereal; only abs_b is "
                   "present");
  std::cout << ",\"prior\":";
  json_unavailable(std::cout,
                   "Rust prior metadata is not a field of native MargData");
  std::cout << ",\"q2\":";
  json_unavailable(std::cout,
                   "Q2 projected rows are diagnostic capture data, not a "
                   "native MargData cereal field");
  std::cout << ",\"optical_flow\":";
  json_unavailable(std::cout,
                   "opt_flow_res is not serialized; image packets are "
                   "separate files");
  std::cout << "}}\n";
}
