// Diagnostic oracle for the pinned Basalt NfrMapper::setup_opt contract.
//
// The binary input is produced by build_m8d_setup_fixture.py from the exact
// checked-in first-20 M8c corners and M8d exported tracks, plus frame poses
// emitted by upstream MargData.  This executable deliberately calls the
// upstream BundleAdjustmentBase::triangulate implementation rather than
// reimplementing DLT in the oracle.

#include <basalt/calibration/calibration.hpp>
#include <basalt/camera/stereographic_param.hpp>
#include <basalt/serialization/headers_serialization.h>
#include <basalt/utils/vio_config.h>
#include <basalt/vi_estimator/ba_base.h>

#include <cstdint>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <cereal/archives/json.hpp>

namespace {

constexpr char kMagic[] = "M8DOPT1\n";
constexpr double kCanonicalScale = 1e10;

struct ImageId {
  int64_t frame_id = 0;
  uint64_t cam_id = 0;
  bool operator<(const ImageId& rhs) const {
    return frame_id < rhs.frame_id ||
           (frame_id == rhs.frame_id && cam_id < rhs.cam_id);
  }
};

struct Observation {
  ImageId image;
  uint64_t feature_id = 0;
};

struct Track {
  uint64_t id = 0;
  std::vector<Observation> observations;
};

struct Pose {
  Sophus::SE3d value;
};

template <class T>
void read(std::istream& in, T& value) {
  in.read(reinterpret_cast<char*>(&value), sizeof(T));
  if (!in) throw std::runtime_error("truncated setup_opt input");
}

void read_input(
    const std::string& path,
    std::map<ImageId, std::vector<Eigen::Vector2d>>& corners,
    std::map<int64_t, Pose>& poses,
    std::vector<Track>& tracks) {
  std::ifstream in(path, std::ios::binary);
  if (!in) throw std::runtime_error("cannot open setup_opt input: " + path);
  char magic[sizeof(kMagic) - 1];
  in.read(magic, sizeof(magic));
  if (!in || std::string(magic, sizeof(magic)) !=
                 std::string(kMagic, sizeof(kMagic) - 1)) {
    throw std::runtime_error("invalid M8DOPT1 input magic");
  }

  uint64_t image_count = 0;
  read(in, image_count);
  for (uint64_t index = 0; index < image_count; ++index) {
    ImageId image;
    uint64_t corner_count = 0;
    read(in, image.frame_id);
    read(in, image.cam_id);
    read(in, corner_count);
    auto& values = corners[image];
    values.resize(corner_count);
    for (auto& corner : values) {
      read(in, corner.x());
      read(in, corner.y());
    }
  }

  uint64_t pose_count = 0;
  read(in, pose_count);
  for (uint64_t index = 0; index < pose_count; ++index) {
    int64_t frame_id = 0;
    Eigen::Vector3d translation;
    Eigen::Vector4d quaternion_xyzw;
    read(in, frame_id);
    for (int axis = 0; axis < 3; ++axis) read(in, translation[axis]);
    for (int axis = 0; axis < 4; ++axis) read(in, quaternion_xyzw[axis]);
    Eigen::Quaterniond quaternion(
        quaternion_xyzw[3], quaternion_xyzw[0], quaternion_xyzw[1],
        quaternion_xyzw[2]);
    poses.emplace(
        frame_id,
        Pose{Sophus::SE3d(Sophus::SO3d(quaternion), translation)});
  }

  uint64_t track_count = 0;
  read(in, track_count);
  tracks.resize(track_count);
  for (auto& track : tracks) {
    uint64_t observation_count = 0;
    read(in, track.id);
    read(in, observation_count);
    track.observations.resize(observation_count);
    for (auto& observation : track.observations) {
      read(in, observation.image.frame_id);
      read(in, observation.image.cam_id);
      read(in, observation.feature_id);
    }
  }
}

void append_u64(std::vector<uint8_t>& bytes, uint64_t value) {
  for (int shift = 0; shift < 64; shift += 8)
    bytes.push_back(static_cast<uint8_t>(value >> shift));
}

void append_i64(std::vector<uint8_t>& bytes, int64_t value) {
  append_u64(bytes, static_cast<uint64_t>(value));
}

int64_t quantize(double value) {
  return static_cast<int64_t>(std::llround(value * kCanonicalScale));
}

uint64_t fnv1a(const std::vector<uint8_t>& bytes) {
  uint64_t result = 1469598103934665603ULL;
  for (const uint8_t byte : bytes) {
    result ^= byte;
    result *= 1099511628211ULL;
  }
  return result;
}

std::string reason_name(const std::string& name) { return name; }

void json_image(std::ostream& out, const ImageId& image) {
  out << "{\"frame_id\":" << image.frame_id << ",\"cam_id\":"
      << image.cam_id << '}';
}

struct Accepted {
  uint64_t id = 0;
  ImageId host;
  ImageId second;
  Eigen::Vector2d direction;
  double inverse_distance = 0;
  const Track* track = nullptr;
};

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4 || argc > 5) {
    std::cerr << "usage: upstream_m8d_setup_opt_oracle CALIB CONFIG INPUT "
                 "[OUTPUT]\n";
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

  std::map<ImageId, std::vector<Eigen::Vector2d>> corners;
  std::map<int64_t, Pose> poses;
  std::vector<Track> tracks;
  read_input(argv[3], corners, poses, tracks);

  const double min_distance = config.mapper_min_triangulation_dist;
  const double min_distance2 = min_distance * min_distance;
  std::map<std::string, size_t> rejection_counts;
  std::vector<Accepted> accepted;
  size_t attempted_tracks = 0;
  size_t skipped_short = 0;
  size_t candidate_attempts = 0;
  size_t observation_count = 0;

  std::ostringstream details;
  details << '[';
  bool first_detail = true;
  for (const Track& track : tracks) {
    if (!first_detail) details << ',';
    first_detail = false;
    details << "{\"track_id\":" << track.id << ",\"observation_count\":"
            << track.observations.size();
    if (track.observations.empty()) {
      details << ",\"attempted\":false,\"accepted\":false}";
      continue;
    }
    details << ",\"host\":";
    json_image(details, track.observations.front().image);
    if (track.observations.size() < 2) {
      ++skipped_short;
      ++rejection_counts[reason_name("track_too_short")];
      details << ",\"attempted\":false,\"accepted\":false}";
      continue;
    }
    ++attempted_tracks;

    const Observation& host = track.observations.front();
    const auto corner_it = corners.find(host.image);
    const auto pose_it = poses.find(host.image.frame_id);
    const bool host_valid = corner_it != corners.end() &&
                            host.feature_id < corner_it->second.size() &&
                            pose_it != poses.end() &&
                            host.image.cam_id < calibration.intrinsics.size() &&
                            host.image.cam_id < calibration.T_i_c.size();
    if (!host_valid) {
      ++rejection_counts[reason_name("missing_input")];
      details << ",\"attempted\":true,\"accepted\":false}";
      continue;
    }

    Eigen::Vector4d host_bearing;
    const bool host_unprojected = calibration.intrinsics[host.image.cam_id]
                                      .unproject(corner_it->second[host.feature_id],
                                                 host_bearing);
    if (!host_unprojected) {
      ++rejection_counts[reason_name("unprojection_failed")];
      details << ",\"attempted\":true,\"accepted\":false}";
      continue;
    }
    const Sophus::SE3d T_w_h = pose_it->second.value *
                               calibration.T_i_c[host.image.cam_id];
    bool track_accepted = false;
    for (size_t candidate_index = 1;
         candidate_index < track.observations.size(); ++candidate_index) {
      ++candidate_attempts;
      const Observation& candidate = track.observations[candidate_index];
      const auto candidate_corner_it = corners.find(candidate.image);
      const auto candidate_pose_it = poses.find(candidate.image.frame_id);
      if (candidate_corner_it == corners.end() ||
          candidate.feature_id >= candidate_corner_it->second.size() ||
          candidate_pose_it == poses.end() ||
          candidate.image.cam_id >= calibration.intrinsics.size() ||
          candidate.image.cam_id >= calibration.T_i_c.size()) {
        ++rejection_counts[reason_name("missing_input")];
        continue;
      }
      Eigen::Vector4d candidate_bearing;
      if (!calibration.intrinsics[candidate.image.cam_id].unproject(
              candidate_corner_it->second[candidate.feature_id],
              candidate_bearing)) {
        ++rejection_counts[reason_name("unprojection_failed")];
        continue;
      }
      const Sophus::SE3d T_w_o = candidate_pose_it->second.value *
                                 calibration.T_i_c[candidate.image.cam_id];
      const Sophus::SE3d T_h_o = T_w_h.inverse() * T_w_o;
      if (T_h_o.translation().squaredNorm() < min_distance2) {
        ++rejection_counts[reason_name("baseline_too_small")];
        continue;
      }
      const Eigen::Vector4d point = basalt::BundleAdjustmentBase<double>::triangulate(
          host_bearing.template head<3>(), candidate_bearing.template head<3>(),
          T_h_o);
      if (!point.array().isFinite().all()) {
        ++rejection_counts[reason_name("triangulation_nonfinite")];
        continue;
      }
      if (point[3] <= 0) {
        ++rejection_counts[reason_name("inverse_distance_nonpositive")];
        continue;
      }
      if (point[3] > 2.0) {
        ++rejection_counts[reason_name("inverse_distance_too_large")];
        continue;
      }
      const Eigen::Vector2d direction =
          basalt::StereographicParam<double>::project(point);
      accepted.push_back(
          Accepted{track.id, host.image, candidate.image, direction, point[3], &track});
      track_accepted = true;
      details << ",\"attempted\":true,\"accepted\":true,\"second\":";
      json_image(details, candidate.image);
      details << ",\"direction\":[" << std::setprecision(17) << direction[0]
              << ',' << direction[1] << "],\"inverse_distance\":" << point[3]
              << '}';
      break;
    }
    if (!track_accepted) details << ",\"attempted\":true,\"accepted\":false}";
    else observation_count += track.observations.size();
  }
  details << ']';

  std::vector<uint8_t> canonical;
  for (const Accepted& landmark : accepted) {
    append_u64(canonical, landmark.id);
    append_i64(canonical, landmark.host.frame_id);
    append_u64(canonical, landmark.host.cam_id);
    append_i64(canonical, landmark.second.frame_id);
    append_u64(canonical, landmark.second.cam_id);
    append_i64(canonical, quantize(landmark.direction[0]));
    append_i64(canonical, quantize(landmark.direction[1]));
    append_i64(canonical, quantize(landmark.inverse_distance));
    append_u64(canonical, landmark.track->observations.size());
    for (const Observation& observation : landmark.track->observations) {
      append_i64(canonical, observation.image.frame_id);
      append_u64(canonical, observation.image.cam_id);
      append_u64(canonical, observation.feature_id);
      const auto image_it = corners.find(observation.image);
      const Eigen::Vector2d& pixel = image_it->second[observation.feature_id];
      append_i64(canonical, quantize(pixel[0]));
      append_i64(canonical, quantize(pixel[1]));
    }
  }

  std::ostringstream output;
  output << "{\"schema\":\"basalt-m8d-setup-opt-oracle-v1\","
         << "\"upstream_commit\":\"0f3b2b52c807f70ff4e2973ce253c73329eea7bc\","
         << "\"input_track_count\":" << tracks.size()
         << ",\"attempted_track_count\":" << attempted_tracks
         << ",\"accepted_track_count\":" << accepted.size()
         << ",\"skipped_short_track_count\":" << skipped_short
         << ",\"candidate_attempt_count\":" << candidate_attempts
         << ",\"observation_count\":" << observation_count
         << ",\"rejection_counts\":{";
  bool first_reason = true;
  for (const auto& [reason, count] : rejection_counts) {
    if (!first_reason) output << ',';
    first_reason = false;
    output << '"' << reason << "\":" << count;
  }
  output << "},\"canonical_hash\":" << fnv1a(canonical)
         << ",\"tracks\":" << details.str() << "}\n";

  if (argc == 5) {
    std::ofstream stream(argv[4]);
    if (!stream) throw std::runtime_error("cannot open oracle output");
    stream << output.str();
    std::ofstream canonical_stream(std::string(argv[4]) + ".canonical.bin",
                                    std::ios::binary);
    if (!canonical_stream)
      throw std::runtime_error("cannot open canonical oracle output");
    canonical_stream.write(
        reinterpret_cast<const char*>(canonical.data()), canonical.size());
  } else {
    std::cout << output.str();
  }
  return 0;
}
