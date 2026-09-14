# M8c upstream mapper feature oracle

This fixture is generated from the pinned upstream Basalt `NfrMapper`
frontend on real `MH_01_easy` images attached to ten continuous
MargData packets. It contains complete keypoints/rays/256-bit
descriptor bytes for the first selected `TimeCamId`; the remaining
19 images retain exact canonical SHA-256 summaries, 16-bit HashBoW
values, and canonical BoW entries.

Fixture: `benchmarks/basalt/m8c_feature_oracle20_ransac.json`

Fixture SHA-256: `01df890311a5e3cd3bf4b431d35e664df7b8de7908f59efb75aa43467a2b7bdd`

Upstream commit: `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`

## Exact pipeline and configuration

The diagnostic calls `NfrMapper::addMargData`, then the exact upstream
`detect_keypoints()` and `match_stereo()` methods. Detection uses
`detectKeypointsMapping`, `computeAngles(..., true)`, and
`computeDescriptors`; rays come from the configured camera
`GenericCamera::unproject`. HashBoW uses 16 bits. Stereo matching uses
the configured mutual Hamming threshold/ratio and essential residual
threshold `1e-3`, exactly as `NfrMapper::match_stereo`.

```json
{
  "mapper_detection_num_points": 800,
  "mapper_num_frames_to_match": 30,
  "mapper_frames_to_match_threshold": 0.04,
  "mapper_min_matches": 20,
  "mapper_ransac_threshold": 5e-05,
  "mapper_min_track_length": 5,
  "mapper_max_hamming_distance": 70,
  "mapper_second_best_test_ratio": 1.2,
  "mapper_bow_num_bits": 16,
  "mapper_min_triangulation_dist": 0.07,
  "mapper_no_factor_weights": false,
  "mapper_use_factors": true,
  "mapper_use_lm": true
}
```

## Full first-20 scope

The diagnostic loads all ten continuous numeric MargData packets via
the pinned `MargDataLoader`, calls `NfrMapper::addMargData` for each,
then runs `detect_keypoints`, `match_stereo`, and `match_all`. The
cumulative mapper contains 34 camera IDs; this fixture fixes the first
20 canonical IDs (ten stereo frames). It also records canonical BoW
query candidates and temporal raw/inlier/reject summaries. It also
contains a diagnostic-only explicit-seed OpenGV RANSAC oracle for
three representative raw>20 pairs across seeds 12345, 424242, and 7.

## Determinism and known upstream gap

`HashBow::compute_bow` inserts entries from a
`std::unordered_map<FeatureHash,double>`, and the mutual matcher uses
unordered-map iteration too. Their semantic sets are retained, but the
fixture canonicalizes BoW entries by numeric 16-bit hash and matches by
`(left_feature_id,right_feature_id)`. The fixture also sorts
`TimeCamId`s because mapper containers are unordered/concurrent. BoW
query candidates are likewise retained as a canonical semantic set,
not an upstream unordered traversal order. The full-array hashes use
the documented 1e-12 fixed-point float encoding to accommodate the
observed DS ray ulp difference; no source mutation or tuning was used.

The default `NfrMapper::match_all` path retains OpenGV's
`time(0)+clock()` seed inside the TBB loop, so default inlier
summaries remain observational. The diagnostic-only seeded hook
uses the literal `CentralRelativeAdapter`, STEWENIUS five-point
solver, sample size 8, 100-iteration/0.99 RANSAC, reprojection
threshold `5e-5`, `selectWithinDistance`, and Cayley/LM refinement.
Rust's seeded API is gated against those model/inlier records; the
only remaining gap is nondeterministic default-time-seed parity.

## Scope

No ground truth is read. The diagnostic source is under
`benchmarks/basalt/`; the pinned upstream checkout is untouched, and no
Rust production file is involved.
