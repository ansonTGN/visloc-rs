// Diagnostic oracle for the pinned Basalt NfrMapper global BA.
//
// This executable intentionally drives the upstream implementation instead of
// reimplementing the Schur equations.  The input is the portable M8DOPT1
// stream emitted by upstream_m8d_setup_opt_oracle.cpp (the checked-in 20-image
// setup artifact), together with either one pinned MargData packet or its
// directory of packets.
// It installs those poses/corners/tracks in NfrMapper, calls setup_opt(), and
// then mirrors only the surrounding optimize() loop so every intermediate
// value can be recorded.  The linearization, sparse accumulator, solve and
// point back-substitution are upstream symbols.

#include <basalt/calibration/calibration.hpp>
#include <basalt/io/marg_data_io.h>
#include <basalt/optimization/accumulator.h>
#include <basalt/serialization/headers_serialization.h>
#include <basalt/utils/imu_types.h>
#include <basalt/utils/vio_config.h>
#include <basalt/vi_estimator/nfr_mapper.h>

#include <Eigen/Dense>
#include <cereal/archives/json.hpp>
#include <cereal/archives/binary.hpp>
#include <sophus/se3.hpp>
#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr char kMagic[] = "M8DOPT1\n";

struct ImageId {
  int64_t frame_id = 0;
  uint64_t cam_id = 0;
  bool operator<(const ImageId& rhs) const {
    return frame_id < rhs.frame_id ||
           (frame_id == rhs.frame_id && cam_id < rhs.cam_id);
  }
};

struct SetupObservation {
  ImageId image;
  uint64_t feature_id = 0;
};

struct SetupTrack {
  int64_t id = 0;
  std::vector<SetupObservation> observations;
};

struct SetupPose {
  int64_t frame_id = 0;
  Sophus::SE3d pose;
};

template <class T>
void read_binary(std::istream& in, T& value) {
  in.read(reinterpret_cast<char*>(&value), sizeof(T));
  if (!in) throw std::runtime_error("truncated M8DOPT1 input");
}

void read_setup(const std::string& path,
                std::map<ImageId, Eigen::aligned_vector<Eigen::Vector2d>>& corners,
                Eigen::aligned_vector<SetupPose>& poses,
                std::vector<SetupTrack>& tracks) {
  std::ifstream in(path, std::ios::binary);
  if (!in) throw std::runtime_error("cannot open setup stream: " + path);

  char magic[sizeof(kMagic) - 1];
  in.read(magic, sizeof(magic));
  if (!in || std::string(magic, sizeof(magic)) !=
                 std::string(kMagic, sizeof(kMagic) - 1)) {
    throw std::runtime_error("invalid M8DOPT1 magic");
  }

  uint64_t image_count = 0;
  read_binary(in, image_count);
  for (uint64_t i = 0; i < image_count; ++i) {
    ImageId image;
    uint64_t corner_count = 0;
    read_binary(in, image.frame_id);
    read_binary(in, image.cam_id);
    read_binary(in, corner_count);
    auto& image_corners = corners[image];
    image_corners.resize(corner_count);
    for (auto& corner : image_corners) {
      read_binary(in, corner.x());
      read_binary(in, corner.y());
    }
  }

  uint64_t pose_count = 0;
  read_binary(in, pose_count);
  poses.reserve(pose_count);
  for (uint64_t i = 0; i < pose_count; ++i) {
    poses.emplace_back();
    SetupPose& pose = poses.back();
    Eigen::Vector3d translation;
    Eigen::Vector4d quaternion_xyzw;
    read_binary(in, pose.frame_id);
    for (int axis = 0; axis < 3; ++axis) read_binary(in, translation[axis]);
    for (int axis = 0; axis < 4; ++axis)
      read_binary(in, quaternion_xyzw[axis]);
    Eigen::Quaterniond quaternion(quaternion_xyzw[3], quaternion_xyzw[0],
                                  quaternion_xyzw[1], quaternion_xyzw[2]);
    pose.pose = Sophus::SE3d(Sophus::SO3d(quaternion), translation);
  }

  uint64_t track_count = 0;
  read_binary(in, track_count);
  tracks.resize(track_count);
  for (auto& track : tracks) {
    uint64_t observation_count = 0;
    read_binary(in, track.id);
    read_binary(in, observation_count);
    track.observations.resize(observation_count);
    for (auto& observation : track.observations) {
      read_binary(in, observation.image.frame_id);
      read_binary(in, observation.image.cam_id);
      read_binary(in, observation.feature_id);
    }
  }
}

void load_marg_directory(const std::string& path, basalt::NfrMapper& mapper) {
  if (!std::filesystem::is_directory(path)) {
    std::ifstream stream(path, std::ios::binary);
    if (!stream) throw std::runtime_error("cannot open MargData: " + path);
    basalt::MargData data;
    cereal::BinaryInputArchive archive(stream);
    archive(data);
    basalt::MargData::Ptr ptr(new basalt::MargData(std::move(data)));
    mapper.addMargData(ptr);
    return;
  }
  tbb::concurrent_bounded_queue<basalt::MargData::Ptr> queue;
  basalt::MargDataLoader loader;
  loader.out_marg_queue = &queue;
  loader.start(path);
  while (true) {
    basalt::MargData::Ptr next;
    queue.pop(next);
    if (!next) break;
    mapper.addMargData(next);
  }
}

void install_setup(
    const std::map<ImageId, Eigen::aligned_vector<Eigen::Vector2d>>& corners,
                  const Eigen::aligned_vector<SetupPose>& poses,
                  const std::vector<SetupTrack>& tracks,
                  basalt::NfrMapper& mapper) {
  for (const auto& [image, values] : corners) {
    basalt::KeypointsData data;
    data.corners = values;
    mapper.feature_corners.emplace(basalt::TimeCamId(image.frame_id,
                                                     image.cam_id),
                                   std::move(data));
  }

  for (const SetupPose& pose : poses) {
    mapper.frame_poses[pose.frame_id] =
        basalt::PoseStateWithLin<double>(pose.frame_id, pose.pose);
  }

  mapper.feature_tracks.clear();
  for (const SetupTrack& track : tracks) {
    basalt::FeatureTrack values;
    for (const SetupObservation& observation : track.observations) {
      values.emplace(
          basalt::TimeCamId(observation.image.frame_id,
                            observation.image.cam_id),
          static_cast<int>(observation.feature_id));
    }
    mapper.feature_tracks.emplace(track.id, std::move(values));
  }
}

void append_u64(std::vector<uint8_t>& bytes, uint64_t value) {
  for (int shift = 0; shift < 64; shift += 8)
    bytes.push_back(static_cast<uint8_t>(value >> shift));
}

void append_f64(std::vector<uint8_t>& bytes, double value) {
  uint64_t bits = 0;
  if (value == 0.0) {
    bits = 0;
  } else if (std::isnan(value)) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    std::memcpy(&bits, &nan, sizeof(bits));
  } else {
    std::memcpy(&bits, &value, sizeof(bits));
  }
  append_u64(bytes, bits);
}

void append_bool(std::vector<uint8_t>& bytes, bool value) {
  bytes.push_back(value ? 1 : 0);
}

uint64_t fnv1a(const std::vector<uint8_t>& bytes) {
  uint64_t hash = 1469598103934665603ULL;
  for (uint8_t byte : bytes) {
    hash ^= byte;
    hash *= 1099511628211ULL;
  }
  return hash;
}

std::vector<int64_t> sorted_landmark_ids(const basalt::NfrMapper& mapper) {
  std::vector<int64_t> ids;
  ids.reserve(mapper.lmdb.getLandmarks().size());
  for (const auto& [id, _] : mapper.lmdb.getLandmarks()) ids.push_back(id);
  std::sort(ids.begin(), ids.end());
  return ids;
}

uint64_t state_hash(
    const basalt::NfrMapper& mapper,
    const std::map<int64_t, SetupTrack>* setup_tracks = nullptr) {
  std::vector<uint8_t> bytes;
  append_u64(bytes, mapper.frame_poses.size());
  for (const auto& [id, state] : mapper.frame_poses) {
    append_u64(bytes, static_cast<uint64_t>(id));
    const Sophus::SE3d& pose = state.getPose();
    append_f64(bytes, pose.translation().x());
    append_f64(bytes, pose.translation().y());
    append_f64(bytes, pose.translation().z());
    append_f64(bytes, pose.unit_quaternion().x());
    append_f64(bytes, pose.unit_quaternion().y());
    append_f64(bytes, pose.unit_quaternion().z());
    append_f64(bytes, pose.unit_quaternion().w());
  }

  const auto ids = sorted_landmark_ids(mapper);
  append_u64(bytes, ids.size());
  for (const int64_t id : ids) {
    const auto& point = mapper.lmdb.getLandmark(id);
    append_u64(bytes, static_cast<uint64_t>(id));
    append_u64(bytes, static_cast<uint64_t>(point.host_kf_id.frame_id));
    append_u64(bytes, point.host_kf_id.cam_id);
    std::vector<basalt::TimeCamId> observation_ids;
    observation_ids.reserve(point.obs.size());
    for (const auto& [image, _] : point.obs) observation_ids.push_back(image);
    std::sort(observation_ids.begin(), observation_ids.end());
    // Upstream Keypoint does not retain the setup candidate pair after
    // initialization.  Canonicalize this metadata from the optimized
    // observation map so the Rust MapperLandmark hash covers the same state.
    const basalt::TimeCamId second = observation_ids.size() > 1
                                         ? observation_ids[1]
                                         : point.host_kf_id;
    const SetupTrack* setup_track = nullptr;
    if (setup_tracks) {
      auto track_it = setup_tracks->find(id);
      if (track_it != setup_tracks->end()) setup_track = &track_it->second;
    }
    append_u64(bytes, static_cast<uint64_t>(second.frame_id));
    append_u64(bytes, second.cam_id);
    append_f64(bytes, point.direction.x());
    append_f64(bytes, point.direction.y());
    append_f64(bytes, point.inv_dist);
    append_u64(bytes, observation_ids.size());
    for (const auto& image : observation_ids) {
      append_u64(bytes, static_cast<uint64_t>(image.frame_id));
      append_u64(bytes, image.cam_id);
      uint64_t feature_id = 0;
      if (setup_track) {
        auto setup_it = std::find_if(
            setup_track->observations.begin(), setup_track->observations.end(),
            [&](const SetupObservation& observation) {
              return observation.image.frame_id == image.frame_id &&
                     observation.image.cam_id == image.cam_id;
            });
        if (setup_it != setup_track->observations.end())
          feature_id = setup_it->feature_id;
      }
      append_u64(bytes, feature_id);
      append_f64(bytes, point.obs.at(image).x());
      append_f64(bytes, point.obs.at(image).y());
    }
  }
  return fnv1a(bytes);
}

struct Trial {
  double lambda = 0;
  double f_diff = 0;
  double vision = 0;
  double relative = 0;
  double roll_pitch = 0;
  double total = 0;
  double max_inc = 0;
  bool accepted = false;
};

struct Iteration {
  int index = 0;
  double vision = 0;
  double relative = 0;
  double roll_pitch = 0;
  double total = 0;
  Eigen::VectorXd hdiag;
  Eigen::VectorXd solve;
  std::vector<std::pair<int64_t, Eigen::Vector3d>> landmark_increments;
  std::vector<Trial> trials;
};

void emit_number(std::ostream& out, double value) {
  out << std::setprecision(17);
  if (std::isfinite(value))
    out << value;
  else
    out << "null";
}

void emit_vector(std::ostream& out, const Eigen::VectorXd& values) {
  out << '[';
  for (Eigen::Index i = 0; i < values.size(); ++i) {
    if (i) out << ',';
    emit_number(out, values[i]);
  }
  out << ']';
}

void emit_trial(std::ostream& out, const Trial& trial) {
  out << "{\"lambda\":";
  emit_number(out, trial.lambda);
  out << ",\"f_diff\":";
  emit_number(out, trial.f_diff);
  out << ",\"after_vision_cost\":";
  emit_number(out, trial.vision);
  out << ",\"after_relative_cost\":";
  emit_number(out, trial.relative);
  out << ",\"after_roll_pitch_cost\":";
  emit_number(out, trial.roll_pitch);
  out << ",\"after_total_cost\":";
  emit_number(out, trial.total);
  out << ",\"max_pose_increment\":";
  emit_number(out, trial.max_inc);
  out << ",\"accepted\":" << (trial.accepted ? "true" : "false") << '}';
}

void emit_iteration(std::ostream& out, const Iteration& iteration) {
  out << "{\"iteration\":" << iteration.index << ",\"vision_cost\":";
  emit_number(out, iteration.vision);
  out << ",\"relative_cost\":";
  emit_number(out, iteration.relative);
  out << ",\"roll_pitch_cost\":";
  emit_number(out, iteration.roll_pitch);
  out << ",\"total_cost\":";
  emit_number(out, iteration.total);
  out << ",\"h_diagonal\":";
  emit_vector(out, iteration.hdiag);
  out << ",\"pose_solve\":";
  emit_vector(out, iteration.solve);
  out << ",\"landmark_increments\":[";
  for (size_t i = 0; i < iteration.landmark_increments.size(); ++i) {
    if (i) out << ',';
    const auto& [id, increment] = iteration.landmark_increments[i];
    out << "[" << id << ", [";
    emit_number(out, increment.x());
    out << ',';
    emit_number(out, increment.y());
    out << ',';
    emit_number(out, increment.z());
    out << "]]";
  }
  out << "],\"trials\":[";
  for (size_t i = 0; i < iteration.trials.size(); ++i) {
    if (i) out << ',';
    emit_trial(out, iteration.trials[i]);
  }
  out << "]}";
}

void emit_factor_export(std::ostream& out, const basalt::NfrMapper& mapper) {
  out << "\"factor_export\":{\"relative_pose\":[";
  for (size_t i = 0; i < mapper.rel_pose_factors.size(); ++i) {
    if (i) out << ',';
    const auto& f = mapper.rel_pose_factors[i];
    out << "{\"from\":" << f.t_i_ns << ",\"to\":" << f.t_j_ns << ",\"translation\":[";
    emit_number(out, f.T_i_j.translation().x()); out << ','; emit_number(out, f.T_i_j.translation().y()); out << ','; emit_number(out, f.T_i_j.translation().z());
    out << "],\"rotation\":["; emit_number(out, f.T_i_j.unit_quaternion().x()); out << ','; emit_number(out, f.T_i_j.unit_quaternion().y()); out << ','; emit_number(out, f.T_i_j.unit_quaternion().z()); out << ','; emit_number(out, f.T_i_j.unit_quaternion().w());
    out << "],\"information\":[";
    for (int r = 0; r < 6; ++r) for (int c = 0; c < 6; ++c) { if (r || c) out << ','; emit_number(out, f.cov_inv(r,c)); }
    const auto& pi = mapper.frame_poses.at(f.t_i_ns).getPose();
    const auto& pj = mapper.frame_poses.at(f.t_j_ns).getPose();
    const auto res = basalt::relPoseError(f.T_i_j, pi, pj);
    out << "],\"cost\":"; emit_number(out, (res.transpose() * f.cov_inv * res)(0,0));
    out << "}";
  }
  out << "],\"roll_pitch\":[";
  for (size_t i = 0; i < mapper.roll_pitch_factors.size(); ++i) {
    if (i) out << ',';
    const auto& f = mapper.roll_pitch_factors[i];
    out << "{\"frame_id\":" << f.t_ns << ",\"measured_rotation\":[";
    const auto rm = f.R_w_i_meas.matrix();
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) { if (r || c) out << ','; emit_number(out, rm(r,c)); }
    out << "],\"information\":[";
    for (int r = 0; r < 2; ++r) for (int c = 0; c < 2; ++c) { if (r || c) out << ','; emit_number(out, f.cov_inv(r,c)); }
    const auto& p = mapper.frame_poses.at(f.t_ns).getPose();
    const auto res = basalt::rollPitchError(p, f.R_w_i_meas);
    out << "],\"cost\":"; emit_number(out, (res.transpose() * f.cov_inv * res)(0,0));
    out << "}";
  }
  out << "]},";
}

void emit_installed_pose_export(
    std::ostream& out,
    const std::map<int64_t, Sophus::SE3d>& installed_poses) {
  out << "\"installed_poses\":[";
  size_t index = 0;
  for (const auto& [frame_id, pose] : installed_poses) {
    if (index++) out << ',';
    const auto q = pose.unit_quaternion();
    out << "{\"frame_id\":" << frame_id << ",\"translation\":[";
    emit_number(out, pose.translation().x());
    out << ',';
    emit_number(out, pose.translation().y());
    out << ',';
    emit_number(out, pose.translation().z());
    out << "],\"quaternion_xyzw\":[";
    emit_number(out, q.x());
    out << ',';
    emit_number(out, q.y());
    out << ',';
    emit_number(out, q.z());
    out << ',';
    emit_number(out, q.w());
    out << "]}";
  }
  out << "],";
}

uint64_t trace_hash(const std::vector<Iteration>& trace) {
  std::vector<uint8_t> bytes;
  append_u64(bytes, trace.size());
  for (const Iteration& iteration : trace) {
    append_u64(bytes, static_cast<uint64_t>(iteration.index));
    append_f64(bytes, iteration.vision);
    append_f64(bytes, iteration.relative);
    append_f64(bytes, iteration.roll_pitch);
    append_f64(bytes, iteration.total);
    append_u64(bytes, iteration.hdiag.size());
    for (Eigen::Index i = 0; i < iteration.hdiag.size(); ++i)
      append_f64(bytes, iteration.hdiag[i]);
    append_u64(bytes, iteration.solve.size());
    for (Eigen::Index i = 0; i < iteration.solve.size(); ++i)
      append_f64(bytes, iteration.solve[i]);
    append_u64(bytes, iteration.landmark_increments.size());
    for (const auto& [id, increment] : iteration.landmark_increments) {
      append_u64(bytes, static_cast<uint64_t>(id));
      append_f64(bytes, increment.x());
      append_f64(bytes, increment.y());
      append_f64(bytes, increment.z());
    }
    append_u64(bytes, iteration.trials.size());
    for (const Trial& trial : iteration.trials) {
      append_f64(bytes, trial.lambda);
      append_f64(bytes, trial.f_diff);
      append_f64(bytes, trial.vision);
      append_f64(bytes, trial.relative);
      append_f64(bytes, trial.roll_pitch);
      append_f64(bytes, trial.total);
      append_f64(bytes, trial.max_inc);
      append_bool(bytes, trial.accepted);
    }
  }
  return fnv1a(bytes);
}

// Record the same point block that ScBundleAdjustmentBase::updatePoints uses,
// before handing the actual update to that upstream function.
void collect_landmark_increments(
    const Eigen::aligned_vector<basalt::ScBundleAdjustmentBase<double>::RelLinData>&
        rld_vec,
    const basalt::AbsOrderMap& aom,
    const Eigen::VectorXd& inc,
    std::vector<std::pair<int64_t, Eigen::Vector3d>>& output) {
  output.clear();
  for (const auto& rld : rld_vec) {
    for (const auto& [lm_id, lm_obs] : rld.lm_to_obs) {
      Eigen::Vector3d h_l_p_x = Eigen::Vector3d::Zero();
      for (const auto& [rel_idx, lm_idx] : lm_obs) {
        const auto& frld = rld.Hpppl.at(rel_idx);
        h_l_p_x += frld.Hpl.at(lm_idx).transpose() *
                   (rld.d_rel_d_h[rel_idx] *
                    inc.segment<6>(aom.abs_order_map.at(
                                       rld.order[rel_idx].first.frame_id)
                                       .first));
        h_l_p_x += frld.Hpl.at(lm_idx).transpose() *
                   (rld.d_rel_d_t[rel_idx] *
                    inc.segment<6>(aom.abs_order_map.at(
                                       rld.order[rel_idx].second.frame_id)
                                       .first));
      }
      // The expression above is intentionally expanded through the relative
      // pose blocks: NfrMapper uses RelLinData and updatePoints, not the
      // absolute helper.  `inc_l` is the exact upstream back-substitution
      // sign convention.
      const Eigen::Vector3d increment =
          -(rld.Hllinv.at(lm_id) * (rld.bl.at(lm_id) - h_l_p_x));
      output.emplace_back(lm_id, increment);
    }
  }
  std::sort(output.begin(), output.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });
}

struct Cost {
  double vision = 0;
  double relative = 0;
  double roll_pitch = 0;
  double total() const { return vision + relative + roll_pitch; }
};

Cost compute_cost(basalt::NfrMapper& mapper) {
  Cost cost;
  mapper.computeError(cost.vision);
  if (mapper.config.mapper_use_factors) {
    mapper.computeRelPose(cost.relative);
    mapper.computeRollPitch(cost.roll_pitch);
  }
  return cost;
}

basalt::AbsOrderMap make_aom(const basalt::NfrMapper& mapper) {
  basalt::AbsOrderMap aom;
  for (const auto& [frame_id, _] : mapper.frame_poses) {
    aom.abs_order_map[frame_id] =
        std::make_pair(static_cast<int>(aom.total_size), 6);
    aom.total_size += 6;
  }
  return aom;
}

void emit_trace(std::ostream& out, const Cost& initial, const Cost& final,
                double lambda, const std::vector<Iteration>& trace,
                const basalt::NfrMapper& mapper,
                const std::map<int64_t, SetupTrack>* setup_tracks,
                const std::map<int64_t, Sophus::SE3d>& installed_poses) {
  out << "{\"schema\":\"basalt-m8e-global-ba-oracle-v2\","
         "\"upstream_commit\":\"0f3b2b52c807f70ff4e2973ce253c73329eea7bc\","
         "\"initial_cost\":";
  emit_number(out, initial.total());
  out << ',';
  emit_installed_pose_export(out, installed_poses);
  emit_factor_export(out, mapper);
  out << "\"final_cost\":";
  emit_number(out, final.total());
  out << ",\"final_lambda\":";
  emit_number(out, lambda);
  out << ",\"iterations\":[";
  for (size_t i = 0; i < trace.size(); ++i) {
    if (i) out << ',';
    emit_iteration(out, trace[i]);
  }
  out << "],\"final_state_hash\":" << state_hash(mapper, setup_tracks)
      << ",\"trace_hash\":" << trace_hash(trace) << "}\n";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 5 || argc > 6) {
    std::cerr << "usage: upstream_m8e_global_ba_oracle CALIB CONFIG MARG_DIR "
                 "M8DOPT1 [OUTPUT]\n";
    return 2;
  }

  basalt::Calibration<double> calibration;
  {
    std::ifstream stream(argv[1]);
    if (!stream) throw std::runtime_error("cannot open calibration");
    cereal::JSONInputArchive archive(stream);
    archive(calibration);
  }
  basalt::VioConfig config;
  config.load(argv[2]);

  basalt::NfrMapper mapper(calibration, config);
  load_marg_directory(argv[3], mapper);

  std::map<ImageId, Eigen::aligned_vector<Eigen::Vector2d>> corners;
  Eigen::aligned_vector<SetupPose> poses;
  std::vector<SetupTrack> tracks;
  read_setup(argv[4], corners, poses, tracks);
  std::map<int64_t, SetupTrack> setup_tracks;
  for (const SetupTrack& track : tracks) setup_tracks.emplace(track.id, track);
  install_setup(corners, poses, tracks, mapper);
  std::map<int64_t, Sophus::SE3d> installed_poses;
  for (const auto& [frame_id, state] : mapper.frame_poses)
    installed_poses.emplace(frame_id, state.getPose());
  mapper.setup_opt();

  basalt::AbsOrderMap aom = make_aom(mapper);
  Cost cost = compute_cost(mapper);
  const Cost initial = cost;
  std::vector<Iteration> trace;
  double lambda = mapper.lambda;
  double lambda_vee = mapper.lambda_vee;

  for (int iteration = 0; iteration < 10; ++iteration) {
    double vision_error = 0;
    Eigen::aligned_vector<basalt::ScBundleAdjustmentBase<double>::RelLinData>
        rld_vec;
    mapper.linearizeHelper(rld_vec, mapper.lmdb.getObservations(),
                           vision_error);

    using Reduce = basalt::NfrMapper::MapperLinearizeAbsReduce<
        basalt::SparseHashAccumulator<double>>;
    Reduce lopt(aom, &mapper.frame_poses);
    tbb::parallel_reduce(
        tbb::blocked_range<decltype(rld_vec.cbegin())>(rld_vec.cbegin(),
                                                       rld_vec.cend()),
        lopt);
    if (config.mapper_use_factors) {
      tbb::parallel_reduce(
          tbb::blocked_range<decltype(mapper.roll_pitch_factors.cbegin())>(
              mapper.roll_pitch_factors.cbegin(),
              mapper.roll_pitch_factors.cend()),
          lopt);
      tbb::parallel_reduce(
          tbb::blocked_range<decltype(mapper.rel_pose_factors.cbegin())>(
              mapper.rel_pose_factors.cbegin(),
              mapper.rel_pose_factors.cend()),
          lopt);
    }

    Iteration record;
    record.index = iteration;
    record.vision = vision_error;
    record.relative = lopt.rel_error;
    record.roll_pitch = lopt.roll_pitch_error;
    record.total = record.vision + record.relative + record.roll_pitch;
    lopt.accum.iterative_solver = true;
    lopt.accum.setup_solver();
    record.hdiag = lopt.accum.Hdiagonal();

    bool step = false;
    bool converged = false;
    int max_iter = 10;
    while (!step && max_iter > 0 && !converged) {
      Eigen::VectorXd hdiag_lambda = record.hdiag * lambda;
      for (int i = 0; i < hdiag_lambda.size(); ++i)
        hdiag_lambda[i] = std::max(hdiag_lambda[i], mapper.min_lambda);
      const Eigen::VectorXd inc = lopt.accum.solve(&hdiag_lambda);
      record.solve = inc;
      const double max_inc = inc.array().abs().maxCoeff();
      if (max_inc < 1e-5) converged = true;

      mapper.backup();
      for (auto& [frame_id, state] : mapper.frame_poses) {
        const int index = aom.abs_order_map.at(frame_id).first;
        state.applyInc(-inc.segment<6>(index));
      }
      record.landmark_increments.clear();
      collect_landmark_increments(rld_vec, aom, inc,
                                  record.landmark_increments);
      for (const auto& rld : rld_vec)
        basalt::ScBundleAdjustmentBase<double>::updatePoints(
            aom, rld, inc, mapper.lmdb);

      Cost after = compute_cost(mapper);
      const double f_diff = cost.total() - after.total();
      const bool accepted = f_diff >= 0;
      record.trials.push_back(Trial{lambda, f_diff, after.vision,
                                    after.relative, after.roll_pitch,
                                    after.total(), max_inc, accepted});
      if (f_diff < 0) {
        lambda = std::min(mapper.max_lambda, lambda_vee * lambda);
        lambda_vee *= 2;
        mapper.restore();
      } else {
        cost = after;
        lambda = std::max(mapper.min_lambda, lambda / 3);
        lambda_vee = 2;
        step = true;
      }
      --max_iter;
    }
    trace.push_back(record);
    if (converged) break;
  }

  std::ofstream output;
  std::ostream* stream = &std::cout;
  if (argc == 6) {
    output.open(argv[5]);
    if (!output) throw std::runtime_error("cannot open oracle output");
    stream = &output;
  }
  emit_trace(*stream, initial, cost, lambda, trace, mapper, &setup_tracks,
             installed_poses);
  return 0;
}
