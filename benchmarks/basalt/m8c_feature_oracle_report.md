# M8c upstream mapper feature oracle

This fixture is generated from the pinned upstream Basalt `NfrMapper`
frontend on real `MH_01_easy` images attached to the first available
MargData record. It contains complete keypoints/rays/256-bit descriptor
bytes for the first selected `TimeCamId`; later images retain counts,
SHA-256 summaries, 16-bit HashBoW values, and canonical BoW entries.

Fixture: `/mnt/c/Users/rsasa/Workspace/visloc-rs/benchmarks/basalt/m8c_feature_oracle.json`

Fixture SHA-256: `f85d985b93afc2a3b208502dda4c70584235d4b8a607759b962fc2afb8d10100`

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

## Bounded seed scope

The user-requested diagnostic bound is 2--8 `TimeCamId`s; this seed
therefore selects 8 IDs (four stereo frames) from the first MargData
record. That record has 16 detected IDs available, while the staged
MargData image directory has 17 frame records (34 camera IDs). The
M8c plan's first-20-image gate is intentionally **not claimed** by this
bounded artifact. The available range and the exact compile/run
commands are recorded in the fixture's `selection` and `provenance`
objects so a follow-up can widen only the selection bound.

## Determinism and known upstream gap

`HashBow::compute_bow` inserts entries from a
`std::unordered_map<FeatureHash,double>`, and the mutual matcher uses
unordered-map iteration too. Their semantic sets are retained, but the
fixture canonicalizes BoW entries by numeric 16-bit hash and matches by
`(left_feature_id,right_feature_id)`. The fixture also sorts
`TimeCamId`s because mapper containers are unordered/concurrent. The
upstream bucket/iteration order is therefore the exact documented gap;
no source mutation or tuning was used.

## Scope

No ground truth is read. The diagnostic source is under
`benchmarks/basalt/`; the pinned upstream checkout is untouched, and no
Rust production file is involved.
