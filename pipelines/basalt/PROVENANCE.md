# Basalt provenance

This crate is a clean-room Rust implementation of the data contracts needed
for the Basalt VI-SLAM port. The upstream reference used for schema and model
semantics is:

- Repository: <https://github.com/VladyslavUsenko/basalt>
- Fixed reference: `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`
- Upstream license: BSD-3-Clause

No Basalt source files are copied into this crate. The implementation is
limited to compatible calibration parsing, coordinate conventions, timestamp
contracts, and the Double Sphere camera equations. Later ported algorithms
must retain this fixed-reference record and add their own provenance notes.

The M3a numerical primitives additionally follow the header-only dependency
used by that Basalt revision:

- Repository: <https://gitlab.com/VladyslavUsenko/basalt-headers>
- Fixed reference: `aa441ba3e51050c47ba1902537792a2e4db7e43d`
- Reference files: `include/basalt/image/image_pyr.h`,
  `include/basalt/optical_flow/patterns.h`, and
  `include/basalt/optical_flow/patch.h`

The M3b direct stream follows the Basalt-side ordering and default guards in
the same pinned repository:

- Reference files: `include/basalt/optical_flow/frame_to_frame_optical_flow.h`
  and `src/utils/keypoints.cpp`
- Ported contracts: forward/backward SE(2) IC, squared forward-backward
  threshold `0.04`, grid FAST replenishment, new-point stereo KLT, Double
  Sphere bearing essential residual threshold `0.005`, and observation emit
- This crate contains no copied Basalt source; the implementation is a
  clean-room Rust translation. Basalt's BSD-3-Clause provenance remains
  applicable to the referenced algorithm and configuration semantics.

The grid replenishment FAST detector is pinned to the OpenCV implementation
actually selected by the Basalt oracle build, rather than to a generic FAST
description:

- Basalt call site: `src/utils/keypoints.cpp` at
  `0f3b2b52c807f70ff4e2973ce253c73329eea7bc` (`cv::FAST` default
  `TYPE_9_16`, non-maximum suppression enabled).
- Oracle build manifest: `benchmarks/basalt/upstream_manifest.json`, vcpkg
  baseline `05442024c3fda64320bd25d2251cc9807b84fb6f` and pinned vcpkg
  revision `1e199d32ad53aab1defda61ce41c380302e3f95c`; the resolved package is
  OpenCV `4.12.0` (`opencv4` port-version 1).
- Authoritative implementation files: OpenCV
  `modules/features2d/src/fast.cpp` and `fast_score.cpp` from that resolved
  4.12.0 source. The Rust implementation preserves the 3-pixel
  sub-image border, strict nine-sample arc test, `cornerScore<16>` response,
  strict 8-neighbour NMS, raw `uint16 >> 8` conversion, and Basalt's
  response-only per-cell sort. It is a clean-room translation; OpenCV source
  is not copied into this crate and no OpenCV runtime is required.
- The FAST detector contribution in those OpenCV files is Copyright (c) 2006,
  2008 Edward Rosten and is distributed under the BSD-3-Clause license. The
  license notice and source are available at
  <https://github.com/opencv/opencv/blob/4.12.0/modules/features2d/src/fast_score.cpp>.

The M7a sensor-only EuRoC boundary follows the same pinned Basalt revision:

- Reference file: `include/basalt/io/dataset_io_euroc.h`
- Sensor paths: `mav0/cam0`, `mav0/cam1`, and `mav0/imu0` only; ground-truth
  trees are intentionally outside the reader contract.
- PNG conversion: upstream 8-bit camera samples are promoted as
  `uint16_t(value) << 8`, while native 16-bit grayscale samples are retained
  unchanged.
- The reader and CLI are clean-room Rust code; no upstream source is copied.

The M7k initialization/IMU parity audit uses the same immutable references:

- `src/vi_estimator/sqrt_keypoint_vio.cpp` at
  `0f3b2b52c807f70ff4e2973ce253c73329eea7bc` for first-camera
  accelerometer-to-`+Z` initialization, fixed gravity `(0,0,-9.81)`, zero
  velocity/bias seed, and initial square-root pose/bias weights.
- `include/basalt/calibration/calib_bias.hpp` and
  `include/basalt/imu/preintegration.h` at
  `aa441ba3e51050c47ba1902537792a2e4db7e43d` for static IMU calibration
  parameter layouts and preintegration conventions.

These references are BSD-3-Clause upstream provenance; the Rust code remains
a clean-room implementation. The active-window public factor keeps the M7h
`[rotation, velocity, position]` row contract, while the audited upstream
header's internal residual is `[position, rotation, velocity]`; this
permutation is an explicit compatibility boundary covered by tests.

The offline mapper follows the same pinned Basalt revision end to end:

- Reference files: `src/mapper.cpp`, `src/vi_estimator/nfr_mapper.cpp`,
  `include/basalt/vi_estimator/nfr_mapper.h`, `src/utils/keypoints.cpp`,
  `src/vi_estimator/ba_base.cpp`, and
  `src/vi_estimator/landmark_database.cpp`.
- The Rust lifecycle preserves MargData reduction and factor recovery, image
  retention, FAST/BRIEF and HashBoW matching, OpenGV-compatible temporal
  RANSAC, track building, stereographic landmark initialization, two global
  BA passes with the intervening outlier filter, and final pose/map export.
- `BundleAdjustmentBase::get_current_points` is represented literally: the
  visualization payload contains one world point for every host/target
  observation-index entry, including repeated coordinates for a landmark
  observed in multiple target images, and assigns the source ID `1` to every
  entry.
- The ground-truth-free command boundary is
  `examples/basalt_mapper_offline_demo.rs`. It accepts schema-4 MargData,
  calibration, and mapper configuration only, and writes parameterized
  landmarks, the accepted stereo/temporal match graph with its estimated
  transforms and raw/inlier feature IDs, observation-index world points,
  poses, and EuRoC/TUM trajectories. `--temporal-seed` exists for
  deterministic native-oracle and release fixtures; omitting it retains the
  pinned OpenGV wall-clock seed behavior.

The mapper implementation is a clean-room translation and contains no copied
Basalt source. The referenced Basalt algorithms remain subject to upstream's
BSD-3-Clause notice recorded above.
