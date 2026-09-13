// Diagnostic-only executable for the pinned upstream Basalt NFR mapper.
//
// This file is intentionally kept under benchmarks/ and is not part of the
// Rust estimator.  It links against the pinned upstream libbasalt.so and calls
// NfrMapper::processMargData followed by NfrMapper::extractNonlinearFactors on
// one serialized MargData record.  The output is a JSON object containing the
// values needed by the M8a/M8b port gate.

#include <basalt/calibration/calibration.hpp>
#include <basalt/io/marg_data_io.h>
#include <basalt/utils/filesystem.h>
#include <basalt/serialization/headers_serialization.h>
#include <basalt/utils/imu_types.h>
#include <basalt/utils/vio_config.h>
#include <basalt/vi_estimator/nfr_mapper.h>

#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>

#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include <fstream>
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using basalt::MargData;
using basalt::NfrMapper;

std::string json_string(const std::string& value) {
  std::ostringstream out;
  out << '"';
  for (const char ch : value) {
    if (ch == '\\' || ch == '"') out << '\\';
    if (ch == '\n') out << "\\n";
    else if (ch == '\r') out << "\\r";
    else if (ch == '\t') out << "\\t";
    else out << ch;
  }
  out << '"';
  return out.str();
}

template <class T>
void emit_scalar(std::ostream& out, const T value) {
  if (!std::isfinite(static_cast<double>(value))) {
    out << "null";
  } else {
    out << std::setprecision(17) << static_cast<double>(value);
  }
}

template <class Derived>
void emit_vector(std::ostream& out, const Eigen::MatrixBase<Derived>& value) {
  out << '[';
  for (Eigen::Index i = 0; i < value.size(); ++i) {
    if (i) out << ',';
    emit_scalar(out, value.derived().coeff(i));
  }
  out << ']';
}

template <class Derived>
void emit_matrix(std::ostream& out, const Eigen::MatrixBase<Derived>& value) {
  out << '[';
  for (Eigen::Index r = 0; r < value.rows(); ++r) {
    if (r) out << ',';
    out << '[';
    for (Eigen::Index c = 0; c < value.cols(); ++c) {
      if (c) out << ',';
      emit_scalar(out, value.derived().coeff(r, c));
    }
    out << ']';
  }
  out << ']';
}

void emit_ids(std::ostream& out, const std::set<int64_t>& ids) {
  out << '[';
  bool first = true;
  for (const int64_t id : ids) {
    if (!first) out << ',';
    first = false;
    out << id;
  }
  out << ']';
}

void emit_columns(std::ostream& out, const std::vector<int>& columns) {
  out << '[';
  for (size_t i = 0; i < columns.size(); ++i) {
    if (i) out << ',';
    out << columns[i];
  }
  out << ']';
}

void emit_aom(std::ostream& out, const basalt::AbsOrderMap& aom) {
  out << "{\"total_size\":" << aom.total_size << ",\"items\":"
      << aom.items << ",\"blocks\":[";
  bool first = true;
  for (const auto& [id, block] : aom.abs_order_map) {
    if (!first) out << ',';
    first = false;
    out << "{\"id\":" << id << ",\"offset\":" << block.first
        << ",\"size\":" << block.second << '}';
  }
  out << "]}";
}

void emit_pose(std::ostream& out, const Sophus::SE3d& pose) {
  out << "{\"translation\":";
  emit_vector(out, pose.translation());
  out << ",\"quaternion_xyzw\":[";
  emit_scalar(out, pose.unit_quaternion().x());
  out << ',';
  emit_scalar(out, pose.unit_quaternion().y());
  out << ',';
  emit_scalar(out, pose.unit_quaternion().z());
  out << ',';
  emit_scalar(out, pose.unit_quaternion().w());
  out << "] ,\"rotation\":";
  emit_matrix(out, pose.so3().matrix());
  out << '}';
}

void emit_pose_map(std::ostream& out,
                   const Eigen::aligned_map<int64_t,
                                             basalt::PoseStateWithLin<double>>&
                       values) {
  out << '[';
  bool first = true;
  for (const auto& [id, state] : values) {
    if (!first) out << ',';
    first = false;
    out << "{\"id\":" << id << ",\"t_ns\":" << state.getT_ns()
        << ",\"linearized\":" << (state.isLinearized() ? "true" : "false")
        << ",\"pose\":";
    emit_pose(out, state.getPose());
    out << '}';
  }
  out << ']';
}

void emit_state_map(std::ostream& out,
                    const Eigen::aligned_map<
                        int64_t, basalt::PoseVelBiasStateWithLin<double>>&
                        values) {
  out << '[';
  bool first = true;
  for (const auto& [id, state] : values) {
    if (!first) out << ',';
    first = false;
    const auto& s = state.getState();
    out << "{\"id\":" << id << ",\"t_ns\":" << state.getT_ns()
        << ",\"linearized\":" << (state.isLinearized() ? "true" : "false")
        << ",\"pose\":";
    emit_pose(out, s.T_w_i);
    out << ",\"velocity\":";
    emit_vector(out, s.vel_w_i);
    out << ",\"bias_gyro\":";
    emit_vector(out, s.bias_gyro);
    out << ",\"bias_accel\":";
    emit_vector(out, s.bias_accel);
    out << '}';
  }
  out << ']';
}

void emit_factors(std::ostream& out, const NfrMapper& mapper) {
  out << "{\"roll_pitch\":[";
  bool first = true;
  for (const auto& factor : mapper.roll_pitch_factors) {
    if (!first) out << ',';
    first = false;
    out << "{\"t_ns\":" << factor.t_ns << ",\"measurement_rotation\":";
    emit_matrix(out, factor.R_w_i_meas.matrix());
    out << ",\"information\":";
    emit_matrix(out, factor.cov_inv);
    out << '}';
  }
  out << "],\"relative_pose\":[";
  first = true;
  for (const auto& factor : mapper.rel_pose_factors) {
    if (!first) out << ',';
    first = false;
    out << "{\"t_i_ns\":" << factor.t_i_ns << ",\"t_j_ns\":"
        << factor.t_j_ns << ",\"measurement_translation\":";
    emit_vector(out, factor.T_i_j.translation());
    out << ",\"measurement_quaternion_xyzw\":[";
    emit_scalar(out, factor.T_i_j.unit_quaternion().x());
    out << ',';
    emit_scalar(out, factor.T_i_j.unit_quaternion().y());
    out << ',';
    emit_scalar(out, factor.T_i_j.unit_quaternion().z());
    out << ',';
    emit_scalar(out, factor.T_i_j.unit_quaternion().w());
    out << "] ,\"measurement_rotation\":";
    emit_matrix(out, factor.T_i_j.so3().matrix());
    out << ",\"information\":";
    emit_matrix(out, factor.cov_inv);
    out << '}';
  }
  out << "]}";
}

void load_marg(const std::string& path, MargData& data) {
  if (basalt::fs::is_directory(path)) {
    tbb::concurrent_bounded_queue<MargData::Ptr> queue;
    basalt::MargDataLoader loader;
    loader.out_marg_queue = &queue;
    loader.start(path);
    while (true) {
      MargData::Ptr next;
      queue.pop(next);
      if (!next) break;
      if (data.kfs_to_marg.empty()) data = *next;
    }
    return;
  }
  std::ifstream stream(path, std::ios::binary);
  if (!stream) throw std::runtime_error("cannot open MargData: " + path);
  cereal::BinaryInputArchive archive(stream);
  archive(data);
}

void emit_image_ids(std::ostream& out, const NfrMapper& mapper) {
  std::vector<int64_t> ids;
  ids.reserve(mapper.img_data.size());
  for (const auto& [id, _] : mapper.img_data) ids.push_back(id);
  std::sort(ids.begin(), ids.end());
  out << '[';
  for (size_t i = 0; i < ids.size(); ++i) {
    if (i) out << ',';
    out << ids[i];
  }
  out << ']';
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4 || argc > 5) {
    std::cerr << "usage: upstream_nfr_oracle CALIB CONFIG MARG [OUTPUT]\n";
    return 2;
  }

  const std::string calib_path = argv[1];
  const std::string config_path = argv[2];
  const std::string marg_path = argv[3];
  const std::string output_path = argc == 5 ? argv[4] : "";

  std::cerr << "[diag] load calibration\n";
  basalt::Calibration<double> calib;
  {
    std::ifstream stream(calib_path);
    if (!stream) throw std::runtime_error("cannot open calibration");
    cereal::JSONInputArchive archive(stream);
    archive(calib);
  }
  basalt::VioConfig config;
  config.load(config_path);

  std::cerr << "[diag] load marg\n";
  MargData data;
  load_marg(marg_path, data);
  std::cerr << "[diag] marg loaded cols=" << data.abs_H.cols()
            << " rows=" << data.abs_H.rows() << "\n";
  const MargData input = data;
  std::cerr << "[diag] construct mapper\n";
  // NfrMapper has Eigen's aligned operator new; make_shared may only provide
  // max_align_t on older libstdc++ versions used by the pinned oracle.
  NfrMapper::Ptr mapper(new NfrMapper(calib, config));

  std::cerr << "[diag] process\n";
  mapper->processMargData(data);
  std::cerr << "[diag] extract\n";
  const bool valid = mapper->extractNonlinearFactors(data);
  std::cerr << "[diag] extracted valid=" << valid << "\n";

  std::ostringstream out;
  out << std::setprecision(17);
  out << "{\"schema\":\"basalt.nfr_oracle.v1\",\"valid\":"
      << (valid ? "true" : "false") << ",\"input\":{";
  out << "\"aom\":";
  emit_aom(out, input.aom);
  out << ",\"abs_H_shape\":[" << input.abs_H.rows() << ','
      << input.abs_H.cols() << "] ,\"abs_H\":";
  emit_matrix(out, input.abs_H);
  out << ",\"abs_b\":";
  emit_vector(out, input.abs_b);
  out << ",\"matrix_semantics\":\"gram_and_gradient\"";
  out << ",\"frame_poses\":";
  emit_pose_map(out, input.frame_poses);
  out << ",\"frame_states\":";
  emit_state_map(out, input.frame_states);
  out << ",\"kfs_all\":";
  emit_ids(out, input.kfs_all);
  out << ",\"kfs_to_marg\":";
  emit_ids(out, input.kfs_to_marg);
  out << ",\"use_imu\":" << (input.use_imu ? "true" : "false") << "},";

  out << "\"output\":{";
  out << "\"aom\":";
  emit_aom(out, data.aom);
  out << ",\"abs_H_shape\":[" << data.abs_H.rows() << ','
      << data.abs_H.cols() << "] ,\"abs_H\":";
  emit_matrix(out, data.abs_H);
  out << ",\"abs_b\":";
  emit_vector(out, data.abs_b);
  out << ",\"matrix_semantics\":\"gram_and_gradient\"";
  std::vector<int> kept_columns;
  std::vector<int> marginalized_columns;
  for (const auto& [id, block] : input.aom.abs_order_map) {
    const bool kf = input.kfs_all.count(id) > 0;
    const int n = block.second;
    const int keep_n = n == basalt::POSE_VEL_BIAS_SIZE && kf
                           ? basalt::POSE_SIZE
                           : (n == basalt::POSE_SIZE ? n : 0);
    for (int i = 0; i < n; ++i) {
      if (i < keep_n)
        kept_columns.push_back(block.first + i);
      else
        marginalized_columns.push_back(block.first + i);
    }
  }
  out << ",\"kept_columns\":";
  emit_columns(out, kept_columns);
  out << ",\"marginalized_columns\":";
  emit_columns(out, marginalized_columns);
  out << ",\"frame_poses\":";
  emit_pose_map(out, data.frame_poses);
  out << ",\"frame_states\":";
  emit_state_map(out, data.frame_states);
  out << ",\"kfs_all\":";
  emit_ids(out, data.kfs_all);
  out << ",\"kfs_to_marg\":";
  emit_ids(out, data.kfs_to_marg);
  out << ",\"use_imu\":" << (data.use_imu ? "true" : "false")
      << ",\"rank_before\":" << input.abs_H.fullPivLu().rank()
      << ",\"rank_after\":" << data.abs_H.fullPivLu().rank() << ',';
  out << "\"factors\":";
  emit_factors(out, *mapper);
  out << ",\"image_ids\":";
  emit_image_ids(out, *mapper);
  out << "}}\n";

  if (output_path.empty()) {
    std::cout << out.str();
  } else {
    std::ofstream stream(output_path);
    if (!stream) throw std::runtime_error("cannot open output: " + output_path);
    stream << out.str();
  }
  return 0;
}
