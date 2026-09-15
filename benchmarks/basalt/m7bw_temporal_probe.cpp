// M7bw external native trace for MH_01 frame 0 -> frame 1, cam0 track 2.
// This duplicates only Basalt's public FrameToFrameOpticalFlow kernel so the
// pinned Eigen/Sophus intermediates can be compared with the Rust probe.

#include <basalt/image/image_pyr.h>
#include <basalt/optical_flow/patch.h>
#include <basalt/optical_flow/patterns.h>
#include <basalt/utils/vio_config.h>

#include <opencv2/imgcodecs.hpp>

#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

using Affine = Eigen::AffineCompact2f;
using Patch = basalt::OpticalFlowPatch<float, basalt::Pattern51<float>>;

static std::uint32_t bits(float value) {
  std::uint32_t result = 0;
  std::memcpy(&result, &value, sizeof(result));
  return result;
}

static void print_bits(float value) {
  std::cout << "0x" << std::hex << std::setw(8) << std::setfill('0')
            << bits(value) << std::dec << std::setfill(' ');
}

static basalt::ManagedImage<uint16_t> read_image(const std::string& path) {
  cv::Mat image = cv::imread(path, cv::IMREAD_UNCHANGED);
  if (image.empty() || image.type() != CV_8UC1) {
    throw std::runtime_error("expected 8-bit grayscale image: " + path);
  }
  basalt::ManagedImage<uint16_t> result(image.cols, image.rows);
  for (int y = 0; y < image.rows; ++y) {
    const auto* source = image.ptr<std::uint8_t>(y);
    auto* destination = result.RowPtr(y);
    for (int x = 0; x < image.cols; ++x) {
      destination[x] = static_cast<std::uint16_t>(source[x]) << 8;
    }
  }
  return result;
}

static void print_vector(const Patch::VectorP& vector) {
  for (int index = 0; index < Patch::PATTERN_SIZE; ++index) {
    if (index != 0) std::cout << ',';
    print_bits(vector[index]);
  }
  std::cout << '\n';
}

int main(int argc, char** argv) {
  if (argc < 3) return 2;
  const std::string root = argv[1];
  const std::string config_path = argv[2];
  basalt::VioConfig config;
  config.load(config_path);

  const std::int64_t timestamps[] = {1403636579763555584LL,
                                     1403636579813555456LL};
  std::ostringstream old_name;
  old_name << root << "/mav0/cam0/data/" << timestamps[0] << ".png";
  std::ostringstream current_name;
  current_name << root << "/mav0/cam0/data/" << timestamps[1] << ".png";
  basalt::ManagedImage<uint16_t> old_image = read_image(old_name.str());
  basalt::ManagedImage<uint16_t> current_image = read_image(current_name.str());
  basalt::ManagedImagePyr<uint16_t> old_pyramid, current_pyramid;
  old_pyramid.setFromImage(old_image, config.optical_flow_levels);
  current_pyramid.setFromImage(current_image, config.optical_flow_levels);

  const Eigen::Vector2f source(46.0f, 118.0f);
  Affine transform;
  transform.setIdentity();
  transform.translation() = source;
  std::cout << "source=";
  print_bits(source.x());
  std::cout << ',';
  print_bits(source.y());
  std::cout << '\n';

  for (int level = config.optical_flow_levels; level >= 0; --level) {
    const float scale = static_cast<float>(1 << level);
    transform.translation() /= scale;
    Patch patch(old_pyramid.lvl(level), source / scale);
    std::cout << "L" << level << ".mean=";
    print_bits(patch.mean);
    std::cout << " data0=";
    print_bits(patch.data[0]);
    std::cout << " jac0=";
    print_bits(patch.H_se2_inv_J_se2_T(0, 0));
    std::cout << " valid=" << patch.valid << '\n';
    for (int row = 0; row < 3; ++row) {
      std::cout << "L" << level << ".inv" << row << "=";
      for (int sample = 0; sample < Patch::PATTERN_SIZE; ++sample) {
        if (sample != 0) std::cout << ',';
        print_bits(patch.H_se2_inv_J_se2_T(row, sample));
      }
      std::cout << '\n';
    }

    for (int iteration = 0;
         iteration < config.optical_flow_max_iterations && patch.valid;
         ++iteration) {
      Patch::Matrix2P transformed =
          transform.linear().matrix() * Patch::pattern2;
      transformed.colwise() += transform.translation();
      Patch::VectorP residual;
      if (!patch.residual(current_pyramid.lvl(level), transformed, residual)) {
        throw std::runtime_error("native residual failed");
      }
      std::cout << "L" << level << ".I" << iteration << ".res=";
      print_vector(residual);
      std::cout << '\n';
      const Eigen::Vector3f increment = -patch.H_se2_inv_J_se2_T * residual;
      std::cout << "L" << level << ".I" << iteration << ".inc=";
      print_bits(increment[0]);
      std::cout << ',';
      print_bits(increment[1]);
      std::cout << ',';
      print_bits(increment[2]);
      std::cout << '\n';
      const Eigen::Matrix3f update = Sophus::SE2f::exp(increment).matrix();
      transform *= update;
      std::cout << "L" << level << ".I" << iteration << ".t=";
      print_bits(transform.translation().x());
      std::cout << ',';
      print_bits(transform.translation().y());
      std::cout << '\n';
    }
    transform.translation() *= scale;
    std::cout << "L" << level << ".world_t=";
    print_bits(transform.translation().x());
    std::cout << ',';
    print_bits(transform.translation().y());
    std::cout << '\n';
  }
  std::cout << "final=";
  print_bits(transform.translation().x());
  std::cout << ',';
  print_bits(transform.translation().y());
  std::cout << '\n';
  return 0;
}
