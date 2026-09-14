// Diagnostic-only executable for the pinned upstream Basalt mapper frontend.
//
// This deliberately lives under benchmarks/ and is never linked into the Rust
// estimator.  It loads the real MH_01 MargData image records, drives the exact
// NfrMapper detect_keypoints()/match_stereo() path, and emits a raw JSON dump
// used by build_m8c_feature_fixture.py to make a compact deterministic fixture.
//
// The raw dump keeps all selected-image arrays so the Python step can retain
// complete data for the first TimeCamId and cryptographic summaries for the
// remaining TimeCamIds without depending on an upstream source edit.  The
// diagnostic bound is intentionally larger than the original seed fixture so
// a follow-up can exercise the first canonical 20 TimeCamIds from MargData.

#include <basalt/calibration/calibration.hpp>
#include <basalt/hash_bow/hash_bow.h>
#include <basalt/io/marg_data_io.h>
#include <basalt/serialization/headers_serialization.h>
#include <basalt/utils/keypoints.h>
#include <basalt/utils/vio_config.h>
#include <basalt/vi_estimator/nfr_mapper.h>

#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>

#include <cereal/archives/json.hpp>

#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include <algorithm>
#include <bitset>
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>


namespace {

using basalt::KeypointsData;
using basalt::MargData;
using basalt::NfrMapper;
using basalt::TimeCamId;

std::string json_string(const std::string& value) {
  std::ostringstream out;
  out << '"';
  for (const char ch : value) {
    if (ch == '\\' || ch == '"') out << '\\';
    if (ch == '\n')
      out << "\\n";
    else if (ch == '\r')
      out << "\\r";
    else if (ch == '\t')
      out << "\\t";
    else
      out << ch;
  }
  out << '"';
  return out.str();
}

template <class T>
void emit_scalar(std::ostream& out, const T value) {
  const double v = static_cast<double>(value);
  if (!std::isfinite(v))
    out << "null";
  else
    out << std::setprecision(17) << v;
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

void emit_scalars(std::ostream& out, const std::vector<double>& values) {
  out << '[';
  for (size_t i = 0; i < values.size(); ++i) {
    if (i) out << ',';
    emit_scalar(out, values[i]);
  }
  out << ']';
}

void emit_tcid(std::ostream& out, const TimeCamId& id) {
  out << "{\"frame_id\":" << id.frame_id << ",\"cam_id\":"
      << id.cam_id << '}';
}

void emit_descriptor_bytes(std::ostream& out,
                           const std::vector<std::bitset<256>>& descriptors) {
  // std::bitset's index 0 is the least-significant bit.  The fixture stores
  // eight such bits per byte, little-endian within each 32-byte descriptor.
  out << '[';
  for (size_t i = 0; i < descriptors.size(); ++i) {
    if (i) out << ',';
    out << '[';
    for (size_t byte = 0; byte < 32; ++byte) {
      if (byte) out << ',';
      unsigned int value = 0;
      for (size_t bit = 0; bit < 8; ++bit)
        if (descriptors[i][byte * 8 + bit]) value |= (1u << bit);
      out << value;
    }
    out << ']';
  }
  out << ']';
}

void emit_hashes(std::ostream& out,
                 const std::vector<basalt::FeatureHash>& hashes) {
  out << '[';
  for (size_t i = 0; i < hashes.size(); ++i) {
    if (i) out << ',';
    out << hashes[i].to_ulong();
  }
  out << ']';
}

void emit_bow(std::ostream& out, const basalt::HashBowVector& bow) {
  // HashBow::compute_bow stores entries from an unordered_map.  Preserve the
  // observed order in this raw diagnostic; the canonicalizer sorts by hash
  // and records that ordering gap explicitly.
  out << '[';
  for (size_t i = 0; i < bow.size(); ++i) {
    if (i) out << ',';
    out << "{\"hash\":" << bow[i].first.to_ulong() << ",\"weight\":";
    emit_scalar(out, bow[i].second);
    out << '}';
  }
  out << ']';
}

void emit_keypoints(std::ostream& out, const TimeCamId& tcid,
                    const KeypointsData& kd) {
  out << "{\"time_cam_id\":";
  emit_tcid(out, tcid);
  out << ",\"corner_count\":" << kd.corners.size() << ",\"corners_xy\":[";
  for (size_t i = 0; i < kd.corners.size(); ++i) {
    if (i) out << ',';
    emit_vector(out, kd.corners[i]);
  }
  out << "] ,\"corner_angles\":";
  emit_scalars(out, kd.corner_angles);
  out << ",\"descriptor_bytes\":";
  emit_descriptor_bytes(out, kd.corner_descriptors);
  out << ",\"corners_3d\":[";
  for (size_t i = 0; i < kd.corners_3d.size(); ++i) {
    if (i) out << ',';
    emit_vector(out, kd.corners_3d[i]);
  }
  out << "] ,\"hashes\":";
  emit_hashes(out, kd.hashes);
  out << ",\"bow_vector\":";
  emit_bow(out, kd.bow_vector);
  out << '}';
}

void emit_matches(std::ostream& out, const NfrMapper& mapper,
                  const std::vector<TimeCamId>& selected,
                  const basalt::VioConfig& config,
                  const basalt::Calibration<double>& calib) {
  std::set<TimeCamId> selected_set(selected.begin(), selected.end());

  const Sophus::SE3d T_0_1 =
      calib.T_i_c[0].inverse() * calib.T_i_c[1];
  Eigen::Matrix4d E;
  basalt::computeEssential(T_0_1, E);

  out << "[";
  bool first_pair = true;
  // Build the pair list from sorted selected IDs, matching NfrMapper's
  // stereo-only cam-0 -> cam-1 operation while making output deterministic.
  for (const TimeCamId& left : selected) {
    if (left.cam_id != 0) continue;
    const TimeCamId right(left.frame_id, 1);
    if (selected_set.count(right) == 0) continue;

    const KeypointsData& kd1 = mapper.feature_corners.at(left);
    const KeypointsData& kd2 = mapper.feature_corners.at(right);

    std::vector<std::pair<int, int>> raw;
    basalt::matchDescriptors(
        kd1.corner_descriptors, kd2.corner_descriptors, raw,
        config.mapper_max_hamming_distance,
        config.mapper_second_best_test_ratio);

    basalt::MatchData md;
    md.matches = raw;
    basalt::findInliersEssential(kd1, kd2, E, 1e-3, md);

    std::sort(raw.begin(), raw.end());
    std::sort(md.inliers.begin(), md.inliers.end());

    if (!first_pair) out << ',';
    first_pair = false;
    out << "{\"left\":";
    emit_tcid(out, left);
    out << ",\"right\":";
    emit_tcid(out, right);
    out << ",\"raw_mutual_hamming\":[";
    for (size_t i = 0; i < raw.size(); ++i) {
      if (i) out << ',';
      const int a = raw[i].first;
      const int b = raw[i].second;
      const int distance =
          static_cast<int>((kd1.corner_descriptors[a] ^
                            kd2.corner_descriptors[b])
                               .count());
      out << "{\"left_feature_id\":" << a
          << ",\"right_feature_id\":" << b << ",\"hamming\":"
          << distance << '}';
    }
    out << "] ,\"essential_inlier_ids\":[";
    for (size_t i = 0; i < md.inliers.size(); ++i) {
      if (i) out << ',';
      out << '[' << md.inliers[i].first << ',' << md.inliers[i].second << ']';
    }
    out << "] ,\"essential_inlier_count\":" << md.inliers.size()
        << ",\"mapper_feature_matches_stored\":";
    const auto stored = mapper.feature_matches.find(std::make_pair(left, right));
    out << (stored != mapper.feature_matches.end() ? "true" : "false");
    out << '}';
  }
  out << ']';
}

using QueryResults =
    std::map<TimeCamId, std::vector<std::pair<TimeCamId, double>>>;

QueryResults collect_query_candidates(const NfrMapper& mapper,
                                      const std::vector<TimeCamId>& selected,
                                      const basalt::VioConfig& config) {
  QueryResults queries;
  for (const TimeCamId& tcid : selected) {
    const KeypointsData& kd = mapper.feature_corners.at(tcid);
    std::vector<std::pair<TimeCamId, double>> results;
    mapper.hash_bow_database->querry_database(
        kd.bow_vector, config.mapper_num_frames_to_match, results,
        &tcid.frame_id);

    auto& candidates = queries[tcid];
    for (const auto& result : results) {
      if (result.first.frame_id != tcid.frame_id &&
          result.second > config.mapper_frames_to_match_threshold) {
        candidates.emplace_back(result);
      }
    }
    // HashBow's query traverses unordered containers.  Retain the semantic
    // candidate set in a deterministic TimeCamId/score order and document
    // the upstream bucket/iteration order as an intentional gap.
    std::sort(candidates.begin(), candidates.end(),
              [](const auto& left, const auto& right) {
                if (left.first != right.first)
                  return left.first < right.first;
                return left.second > right.second;
              });
  }
  return queries;
}

void emit_query_candidates(std::ostream& out, const QueryResults& queries) {
  out << '[';
  bool first_query = true;
  for (const auto& [tcid, candidates] : queries) {
    if (!first_query) out << ',';
    first_query = false;
    out << "{\"time_cam_id\":";
    emit_tcid(out, tcid);
    out << ",\"candidates\":[";
    for (size_t i = 0; i < candidates.size(); ++i) {
      if (i) out << ',';
      out << "{\"time_cam_id\":";
      emit_tcid(out, candidates[i].first);
      out << ",\"score\":";
      emit_scalar(out, candidates[i].second);
      out << '}';
    }
    out << "] ,\"candidate_count\":" << candidates.size() << '}';
  }
  out << ']';
}

void emit_temporal_matches(std::ostream& out, const NfrMapper& mapper,
                           const QueryResults& queries,
                           const basalt::VioConfig& config) {
  out << '[';
  bool first_pair = true;
  for (const auto& [left, candidates] : queries) {
    const KeypointsData& kd1 = mapper.feature_corners.at(left);
    for (const auto& [right, score] : candidates) {
      const KeypointsData& kd2 = mapper.feature_corners.at(right);
      std::vector<std::pair<int, int>> raw;
      basalt::matchDescriptors(kd1.corner_descriptors, kd2.corner_descriptors,
                               raw, 70, 1.2);
      std::sort(raw.begin(), raw.end());

      const auto stored =
          mapper.feature_matches.find(std::make_pair(left, right));
      const bool ransac_attempted =
          static_cast<int>(raw.size()) > config.mapper_min_matches;
      std::vector<std::pair<int, int>> inliers;
      if (stored != mapper.feature_matches.end()) {
        inliers = stored->second.inliers;
        std::sort(inliers.begin(), inliers.end());
      }

      const char* reject_stage = "accepted";
      if (!ransac_attempted)
        reject_stage = "raw_match_gate";
      else if (stored == mapper.feature_matches.end())
        reject_stage = "ransac_empty";

      if (!first_pair) out << ',';
      first_pair = false;
      out << "{\"left\":";
      emit_tcid(out, left);
      out << ",\"right\":";
      emit_tcid(out, right);
      out << ",\"bow_score\":";
      emit_scalar(out, score);
      out << ",\"raw_mutual_hamming\":[";
      for (size_t i = 0; i < raw.size(); ++i) {
        if (i) out << ',';
        const int a = raw[i].first;
        const int b = raw[i].second;
        const int distance = static_cast<int>(
            (kd1.corner_descriptors[a] ^ kd2.corner_descriptors[b]).count());
        out << "{\"left_feature_id\":" << a
            << ",\"right_feature_id\":" << b << ",\"hamming\":"
            << distance << '}';
      }
      out << "] ,\"raw_match_count\":" << raw.size()
          << ",\"ransac_attempted\":"
          << (ransac_attempted ? "true" : "false")
          << ",\"ransac_inlier_ids\":[";
      for (size_t i = 0; i < inliers.size(); ++i) {
        if (i) out << ',';
        out << '[' << inliers[i].first << ',' << inliers[i].second << ']';
      }
      out << "] ,\"ransac_inlier_count\":" << inliers.size()
          << ",\"mapper_feature_matches_stored\":"
          << (stored != mapper.feature_matches.end() ? "true" : "false")
          << ",\"reject_stage\":" << json_string(reject_stage) << '}';
    }
  }
  out << ']';
}

struct SeededTemporalResult {
  unsigned int seed = 0;
  int iterations = 0;
  bool model_found = false;
  bool accepted = false;
  Eigen::Matrix3d ransac_rotation = Eigen::Matrix3d::Zero();
  Eigen::Vector3d ransac_translation = Eigen::Vector3d::Zero();
  Eigen::Matrix3d refined_rotation = Eigen::Matrix3d::Zero();
  Eigen::Vector3d refined_translation = Eigen::Vector3d::Zero();
  std::vector<int> ransac_inliers;
  std::vector<int> refined_inliers;
};

// Golden-only copy of basalt::findInliersRansac with the upstream default
// unchanged.  The OpenGV problem constructor accepts randomSeed=false and
// seeds 12345; its public rng_alg_ is then overwritten here for fixed-seed
// fixture runs.  This is a diagnostic hook, not an upstream source edit.
SeededTemporalResult run_seeded_temporal(const KeypointsData& kd1,
                                         const KeypointsData& kd2,
                                         const std::vector<std::pair<int, int>>& matches,
                                         double threshold, int min_inliers,
                                         unsigned int seed) {
  SeededTemporalResult result;
  result.seed = seed;
  if (matches.size() < 8) return result;

  opengv::bearingVectors_t bearing_vectors1, bearing_vectors2;
  bearing_vectors1.reserve(matches.size());
  bearing_vectors2.reserve(matches.size());
  for (const auto& match : matches) {
    bearing_vectors1.push_back(kd1.corners_3d[match.first].head<3>());
    bearing_vectors2.push_back(kd2.corners_3d[match.second].head<3>());
  }

  opengv::relative_pose::CentralRelativeAdapter adapter(bearing_vectors1,
                                                         bearing_vectors2);
  using Problem = opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem;
  auto problem = std::make_shared<Problem>(
      adapter, Problem::STEWENIUS, false);
  problem->rng_alg_.seed(seed);
  // std::bind stores the generator by value in the OpenGV constructor, so
  // rebuild that public test hook after replacing rng_alg_; otherwise every
  // requested seed would silently retain the constructor's 12345 stream.
  problem->rng_gen_.reset(new std::function<int()>(
      std::bind(*problem->rng_dist_, problem->rng_alg_)));

  opengv::sac::Ransac<Problem> ransac;
  ransac.sac_model_ = problem;
  ransac.threshold_ = threshold;
  ransac.max_iterations_ = 100;
  if (!ransac.computeModel()) return result;

  result.model_found = true;
  result.iterations = ransac.iterations_;
  result.ransac_rotation = ransac.model_coefficients_.topLeftCorner<3, 3>();
  result.ransac_translation = ransac.model_coefficients_.topRightCorner<3, 1>();
  result.ransac_inliers = ransac.inliers_;

  adapter.sett12(result.ransac_translation);
  adapter.setR12(result.ransac_rotation);
  const opengv::transformation_t refined =
      opengv::relative_pose::optimize_nonlinear(adapter, ransac.inliers_);
  result.refined_rotation = refined.topLeftCorner<3, 3>();
  result.refined_translation = refined.topRightCorner<3, 1>();
  problem->selectWithinDistance(refined, threshold, result.refined_inliers);
  result.accepted = static_cast<int>(result.refined_inliers.size()) >= min_inliers;
  return result;
}

void emit_matrix3(std::ostream& out, const Eigen::Matrix3d& matrix) {
  emit_matrix(out, matrix);
}

void emit_seeded_temporal_oracle(std::ostream& out, const NfrMapper& mapper,
                                 const QueryResults& queries,
                                 const basalt::VioConfig& config) {
  struct Pair {
    TimeCamId left;
    TimeCamId right;
    double score;
    std::vector<std::pair<int, int>> raw;
  };
  std::vector<Pair> selected_pairs;
  for (const auto& [left, candidates] : queries) {
    const KeypointsData& kd1 = mapper.feature_corners.at(left);
    for (const auto& [right, score] : candidates) {
      const KeypointsData& kd2 = mapper.feature_corners.at(right);
      std::vector<std::pair<int, int>> raw;
      basalt::matchDescriptors(kd1.corner_descriptors, kd2.corner_descriptors,
                               raw, 70, 1.2);
      std::sort(raw.begin(), raw.end());
      if (raw.size() > static_cast<size_t>(config.mapper_min_matches)) {
        selected_pairs.push_back(Pair{left, right, score, std::move(raw)});
        if (selected_pairs.size() == 3) break;
      }
    }
    if (selected_pairs.size() == 3) break;
  }

  const unsigned int seeds[] = {12345u, 424242u, 7u};
  out << '[';
  bool first = true;
  for (const Pair& pair : selected_pairs) {
    const KeypointsData& kd1 = mapper.feature_corners.at(pair.left);
    const KeypointsData& kd2 = mapper.feature_corners.at(pair.right);
    for (unsigned int seed : seeds) {
      const SeededTemporalResult result = run_seeded_temporal(
          kd1, kd2, pair.raw, config.mapper_ransac_threshold,
          static_cast<int>(config.mapper_min_matches), seed);
      if (!first) out << ',';
      first = false;
      out << "{\"left\":";
      emit_tcid(out, pair.left);
      out << ",\"right\":";
      emit_tcid(out, pair.right);
      out << ",\"bow_score\":";
      emit_scalar(out, pair.score);
      out << ",\"seed\":" << seed
          << ",\"ransac_iterations\":" << result.iterations
          << ",\"model_found\":" << (result.model_found ? "true" : "false")
          << ",\"accepted\":" << (result.accepted ? "true" : "false")
          << ",\"ransac_model_rotation\":";
      emit_matrix3(out, result.ransac_rotation);
      out << ",\"ransac_model_translation\":";
      emit_vector(out, result.ransac_translation);
      out << ",\"refined_model_rotation\":";
      emit_matrix3(out, result.refined_rotation);
      out << ",\"refined_model_translation\":";
      emit_vector(out, result.refined_translation.normalized());
      out << ",\"ransac_inlier_ids\":[";
      for (size_t i = 0; i < result.ransac_inliers.size(); ++i) {
        if (i) out << ',';
        const auto& match = pair.raw[result.ransac_inliers[i]];
        out << '[' << match.first << ',' << match.second << ']';
      }
      out << "] ,\"refined_inlier_ids\":[";
      for (size_t i = 0; i < result.refined_inliers.size(); ++i) {
        if (i) out << ',';
        const auto& match = pair.raw[result.refined_inliers[i]];
        out << '[' << match.first << ',' << match.second << ']';
      }
      out << "] ,\"ransac_inlier_count\":" << result.ransac_inliers.size()
          << ",\"refined_inlier_count\":" << result.refined_inliers.size()
          << ",\"raw_match_count\":" << pair.raw.size() << '}';
    }
  }
  out << ']';
}

std::vector<basalt::MargData::Ptr> load_marg_packets(
    const std::string& path, std::vector<std::string>& packet_names) {
  tbb::concurrent_bounded_queue<basalt::MargData::Ptr> queue;
  basalt::MargDataLoader loader;
  loader.out_marg_queue = &queue;
  loader.start(path);

  std::vector<basalt::MargData::Ptr> packets;
  while (true) {
    basalt::MargData::Ptr next;
    queue.pop(next);
    if (!next) break;
    packets.emplace_back(std::move(next));
  }
  if (packets.empty())
    throw std::runtime_error("MargData directory is empty: " + path);

  packet_names.reserve(packets.size());
  for (const auto& packet : packets) {
    if (packet->kfs_to_marg.empty())
      packet_names.emplace_back("<unnamed>");
    else
      packet_names.emplace_back(
          std::to_string(*packet->kfs_to_marg.begin()) + ".cereal");
  }
  return packets;
}

void emit_config(std::ostream& out, const basalt::VioConfig& config) {
  out << "{\"mapper_detection_num_points\":"
      << config.mapper_detection_num_points
      << ",\"mapper_num_frames_to_match\":"
      << config.mapper_num_frames_to_match
      << ",\"mapper_frames_to_match_threshold\":";
  emit_scalar(out, config.mapper_frames_to_match_threshold);
  out << ",\"mapper_min_matches\":";
  emit_scalar(out, config.mapper_min_matches);
  out << ",\"mapper_ransac_threshold\":";
  emit_scalar(out, config.mapper_ransac_threshold);
  out << ",\"mapper_min_track_length\":";
  emit_scalar(out, config.mapper_min_track_length);
  out << ",\"mapper_max_hamming_distance\":";
  emit_scalar(out, config.mapper_max_hamming_distance);
  out << ",\"mapper_second_best_test_ratio\":";
  emit_scalar(out, config.mapper_second_best_test_ratio);
  out << ",\"mapper_bow_num_bits\":" << config.mapper_bow_num_bits
      << ",\"mapper_min_triangulation_dist\":";
  emit_scalar(out, config.mapper_min_triangulation_dist);
  out << ",\"mapper_no_factor_weights\":"
      << (config.mapper_no_factor_weights ? "true" : "false")
      << ",\"mapper_use_factors\":"
      << (config.mapper_use_factors ? "true" : "false")
      << ",\"mapper_use_lm\":"
      << (config.mapper_use_lm ? "true" : "false")
      << '}';
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 5 || argc > 6) {
    std::cerr << "usage: upstream_m8c_feature_oracle CALIB CONFIG MARG_DIR "
                 "OUTPUT [MAX_TCID=8]\n";
    return 2;
  }

  const std::string calib_path = argv[1];
  const std::string config_path = argv[2];
  const std::string marg_path = argv[3];
  const std::string output_path = argv[4];
  const size_t max_tcid = argc == 6 ? std::stoul(argv[5]) : 8;
  if (max_tcid < 2 || max_tcid > 20)
    throw std::runtime_error("MAX_TCID must be in [2,20]");

  basalt::Calibration<double> calib;
  {
    std::ifstream stream(calib_path);
    if (!stream) throw std::runtime_error("cannot open calibration");
    cereal::JSONInputArchive archive(stream);
    archive(calib);
  }
  basalt::VioConfig config;
  config.load(config_path);

  std::vector<std::string> packet_names;
  std::vector<basalt::MargData::Ptr> packets =
      load_marg_packets(marg_path, packet_names);

  NfrMapper::Ptr mapper(new NfrMapper(calib, config));
  for (auto& packet : packets) mapper->addMargData(packet);
  mapper->feature_corners.clear();
  mapper->feature_matches.clear();
  mapper->detect_keypoints();
  mapper->match_stereo();

  std::vector<TimeCamId> available;
  available.reserve(mapper->feature_corners.size());
  for (const auto& kv : mapper->feature_corners) available.push_back(kv.first);
  std::sort(available.begin(), available.end());
  if (available.size() < max_tcid)
    throw std::runtime_error("not enough detected TimeCamIds");
  const std::vector<TimeCamId> all_available = available;
  available.resize(max_tcid);
  const QueryResults query_candidates =
      collect_query_candidates(*mapper, available, config);
  // Execute the complete upstream temporal pass on the cumulative mapper.
  // The emitted query/match section is scoped to the canonical first-20
  // query IDs, while candidates may refer to any earlier mapper image.
  mapper->match_all();

  std::ofstream stream(output_path);
  if (!stream) throw std::runtime_error("cannot open output: " + output_path);
  stream << std::setprecision(17);
  stream << "{\"schema\":\"basalt.m8c_feature_oracle.raw.v1\"," 
            "\"upstream_commit\":\"0f3b2b52c807f70ff4e2973ce253c73329eea7bc\"," 
            "\"selected_marg_file\":"
         << json_string(packet_names.front()) << ",\"marg_packet_count\":"
         << packets.size() << ",\"marg_packet_files\":[";
  for (size_t i = 0; i < packet_names.size(); ++i) {
    if (i) stream << ',';
    stream << json_string(packet_names[i]);
  }
  stream << "] ,\"max_tcid\":" << max_tcid
         << ",\"config\":";
  emit_config(stream, config);
  stream << ",\"available_time_cam_ids\":[";
  for (size_t i = 0; i < all_available.size(); ++i) {
    if (i) stream << ',';
    emit_tcid(stream, all_available[i]);
  }
  stream << "] ,\"selected_time_cam_ids\":[";
  for (size_t i = 0; i < available.size(); ++i) {
    if (i) stream << ',';
    emit_tcid(stream, available[i]);
  }
  stream << "] ,\"features\":[";
  for (size_t i = 0; i < available.size(); ++i) {
    if (i) stream << ',';
    emit_keypoints(stream, available[i], mapper->feature_corners.at(available[i]));
  }
  stream << "] ,\"bow_query\":{\"num_results\":"
         << config.mapper_num_frames_to_match
         << ",\"score_threshold\":";
  emit_scalar(stream, config.mapper_frames_to_match_threshold);
  stream << ",\"ordering\":\"canonical TimeCamId/score; upstream unordered query order is not claimed\",\"queries\":";
  emit_query_candidates(stream, query_candidates);
  stream << ",\"temporal_matches\":";
  emit_temporal_matches(stream, *mapper, query_candidates, config);
  stream << ",\"seeded_temporal_oracle\":";
  emit_seeded_temporal_oracle(stream, *mapper, query_candidates, config);
  stream << "} ,\"stereo\":{\"essential_threshold\":0.001,\"pairs\":";
  emit_matches(stream, *mapper, available, config, calib);
  stream << ",\"T_0_1_translation\":";
  emit_vector(stream, (calib.T_i_c[0].inverse() * calib.T_i_c[1]).translation());
  stream << ",\"T_0_1_rotation\":";
  emit_matrix(stream,
              (calib.T_i_c[0].inverse() * calib.T_i_c[1]).so3().matrix());
  stream << "}}\n";
  return 0;
}
