# M8c upstream mapper feature oracle

This fixture is generated from the pinned upstream Basalt `NfrMapper`
frontend on real `MH_01_easy` images attached to ten continuous
MargData packets. It contains complete keypoints/rays/256-bit
descriptor bytes for the first selected `TimeCamId`; the remaining
19 images retain exact canonical SHA-256 summaries, 16-bit HashBoW
values, and canonical BoW entries.

Fixture: `benchmarks/basalt/m8c_feature_oracle20.json`

Fixture SHA-256: `7e05d367629ddbd68d32fbcafbc809507afb98c4917050a2f1e1191264b2f351`

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
query candidates and temporal raw/inlier/reject summaries.

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

OpenGV temporal RANSAC is a separate known gap: the pinned
`CentralRelativePoseSacProblem` uses its default `time(0)+clock()`
seed inside the upstream TBB loop, so inlier IDs/counts are observed
run summaries rather than deterministic values. Rust asserts the
exact raw mutual SHA, strict RANSAC gate, and reject stage; exact
OpenGV model/inlier parity remains open for the next task.

## Scope

No ground truth is read. The diagnostic source is under
`benchmarks/basalt/`; the pinned upstream checkout is untouched, and no
Rust production file is involved.
