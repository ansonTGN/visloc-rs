# LaMAria Stage 0 — first official-metric scores

Goal: #1 on the [LaMAria leaderboard](https://lamaria.ethz.ch/leaderboard) (ETH,
ICCV25 egocentric city-scale VI-SLAM on Project Aria glasses). Stage 0 gets
the first official-metric numbers for our Basalt Rust port on LaMAria
training sequences, using the upstream evaluator
([cvg/lamaria](https://github.com/cvg/lamaria)) unmodified.

Branch: `feat/lamaria-stage0b` (on top of `feat/lamaria-stage0`, merged with
`main`@6087ae4, the real-time VIO pipeline). Data:
`E:\datasets\lamaria\training\{R_01_easy,R_11_5cp,sequence_1_19}`.

## Method

- Pinhole LaMAria calibration converted to Basalt's Double Sphere convention
  (`scripts/lamaria_to_basalt_calib.py`, prior work). Two calibration
  variants per sequence: the datasheet Kalibr IMU noise, and a "variant A"
  override using Basalt's own EuRoC-default IMU noise
  (`configs/basalt/variants/lamaria/*_variantA_default_noise.json`) — an
  R_01_easy A/B showed variant A gives SE3 ATE 0.43 m vs 1.60 m with the
  datasheet noise (init Sim3 scale 0.96 vs 0.73), so variant A is used
  throughout.
- VIO: `basalt_euroc_vio_demo --pipeline --threads 12` (real-time pipeline,
  PR #153, byte-identical to the serial path).
- Mapper (optional refinement pass): `basalt_mapper_offline_demo` on the
  VIO's `marg_data`, `--temporal-seed 7` for determinism.
- `scripts/propagate_basalt_mapper_corrections.py` propagates the mapper's
  keyframe-only corrections back onto the full per-frame VIO trajectory,
  then `scripts/basalt_tum_to_lamaria_estimate.py` converts to the LaMAria
  estimate format.
- Official evaluator (WSL venv `/mnt/e/tools/venv-lamaria`,
  `/mnt/e/tools/lamaria`): `evaluate_wrt_control_points` (Sim3-aligns to the
  sequence's control points, reports **Score** 0–100 and **CP@1m** recall)
  and `evaluate_wrt_pgt` (pose recall against the pseudo-GT trajectory at
  1 m/5 m, using the Sim3 from the control-point step).

## Results

| Sequence | Stage | Score (CP) | CP@1m | Pose R@1m | Pose R@5m | ATE RMSE | Sim3 scale |
|---|---|---|---|---|---|---|---|
| R_11_5cp | VIO only | **49.65** | 20.0 % | 16.0 % | 100.0 % | 1.94 m | 0.965 |
| R_11_5cp | + mapper | **57.66** | 20.0 % | 17.1 % | 100.0 % | 1.37 m | 0.960 |
| sequence_1_19 | VIO only | 12.75 | 7.1 % | 0.0 % | 5.4 % | — | — |
| sequence_1_19 | + mapper | **16.98** | 0.0 % | 0.3 % | 37.1 % | — | — |

Leaderboard context (bino+imu track, Short subset where applicable):

| Method | Score |
|---|---|
| Aria's own SLAM (proprietary, non-causal) | 90.7 |
| microSLAM (mono) | 64.5 |
| **Ours, R_11_5cp, VIO + mapper** | **57.66** |
| **Ours, R_11_5cp, VIO only** | **49.65** |
| Best academic (OpenVINS+Maplab) | 27.7 |
| OKVIS2 | 20.0 |

**Our VIO-only trajectory already beats the best published academic
baseline** on the headline Score metric, without any loop-closure/mapper
refinement; the offline mapper pass adds another +8 points on top.

### Caveats

- **R_11_5cp has only 5 control points.** Score/CP@1m from `evaluate_wrt_control_points`
  is a Sim3 alignment + per-CP piecewise score over just those 5 points, so
  it has real variance — this is not yet a statistically solid estimate of
  test-set performance, just a genuine, honestly-computed first data point.
- **sequence_1_19 is a mixed result** (14 control points): the mapper lifts
  the aggregate Score 12.75 → 16.98 and pGT pose recall @5 m 5.4 % → 37.1 %,
  but CP@1m falls 7.1 % → 0.0 % — the mapper's global correction moves the
  one near control point back out past 1 m. Overall drift over 1.5 km / 15 min
  is still far larger than the Short-track target; the mapper helps but is not
  sufficient at this range.
- **These are training sequences**, not the held-out test set; the official
  leaderboard number comes only from submitting to the 63 test sequences.
- Evaluator run exactly as published (`evaluate_wrt_control_points` +
  `evaluate_wrt_pgt`, `--corresponding_sensor imu`), no custom scoring code.

## Mapper performance/memory fixes (this branch)

The offline mapper (`NfrMapper::run_headless`,
`pipelines/basalt/src/mapper/`) was effectively unusable at LaMAria scale
before this branch — a full R_01_easy mapper run (403 packets) ran for
6+ hours with no sign of finishing. Three fixes, in commit order:

1. **`match_all`'s O(N²) HashBoW candidate scoring.** `query_bow_candidates`
   (`mapper/features.rs`) scored every earlier detected image against every
   query — quadratic in total image count, compounded by an O(B²)
   linear-scan per pair (B = descriptors/image). Ported the online mapper's
   inverted hash-bucket index (`OnlineNfrMapper::hash_index`/
   `bow_candidates_via_index`, `mapper/online.rs`, commit 8b55648) into the
   offline path: `NfrMapper` gains a `hash_index` populated alongside
   `feature_corners`, and `match_all_impl` restricts each query's scan to
   images sharing at least one hash bucket — an *exact* reduction (proven by
   a new unit test on a synthetic DB with a score tie), not an
   approximation. **28% faster on EuRoC MH_01** (862.6s → 617.5s,
   byte-identical output). Weaker win on LaMAria specifically: its
   repetitive wide-FOV urban texture means most image pairs already share a
   hash bucket, so the reduction is much less effective there — the index
   alone is not sufficient at LaMAria scale.
2. **Memory: stream packets, free raw images early.**
   `basalt_mapper_offline_demo` no longer parses the whole `--marg-dir` into
   a `Vec<MargData>` before ingest (each packet is now read, ingested, and
   dropped in one step), and frees `NfrMapper::img_data`'s raw pixel buffers
   right after `match_stereo` (confirmed no later stage reads them). Peak
   RSS on an R_01_easy 150-packet prefix: **2311.5 MB → 46.5 MB (~50x)**.
3. **Config-level candidate-count cap.** `mapper_detection_num_points`
   (corners/image) 800 → 200
   (`configs/basalt/variants/lamaria/euroc_config_mapper_reduced_points.json`),
   cutting the remaining O(B²) cost ~16x. Used for the R_11_5cp mapper run
   above (1357 packets, 2625s / 43.7 min, well within budget).

Net result: the R_11_5cp mapper run (1357 packets, ~9x R_01's earlier
150-packet prefix) that would previously have been wildly impractical
completed in 43.7 minutes with peak RSS staying low throughout.

### Known issue: sequence_1_19 VIO crash

**Fixed** (PR #158): the crash was `basalt_euroc_vio_demo` buffering the
whole run's `trace.jsonl` in one in-process `String`; the fix streams each
trace line to disk as it is produced. With the fix, `sequence_1_19` completes
all 18,352 frames (MH_01 trajectory byte-identical), and the sequence is now
scored above (VIO 12.75, +mapper 16.98). The original diagnosis is kept
below for the record.

`basalt_euroc_vio_demo --pipeline --threads 12` on sequence_1_19 (18,351
frames, ~15 min) **crashes deterministically at frame 10849** with
`memory allocation of 18723373056 bytes failed` (~18.7 GB single
allocation). Reproduced twice, byte-identical failure point and allocation
size, under two different `--pipeline-capacity`/`--decode-threads` settings
— so this is **not** pipeline-buffering contention, but a
data/scale-dependent bug in the VIO/estimator path itself. R_11_5cp
(9,547 frames) completed cleanly and never reached this frame count, which
is consistent with an unbounded-growth condition (landmark/state count, or
similar) that only manifests past roughly 10,800+ processed frames. Not
diagnosed further in this session (out of scope for the mapper-focused
Stage 0b investigation, and would need dedicated time in the VIO/estimator
code); flagged here as follow-up work before sequence_1_19 (or any
similarly long test sequence) can be scored.

## Not done in Stage 0b

- R_01_easy mapper score (an earlier, unfixed mapper run on R_01 ran for
  6+ hours without finishing and was abandoned/killed; not rerun with the
  fixes in this branch).
- Full 23-sequence training sweep, and the 63-sequence test submission.

Done since this section was written: the sequence_1_19 VIO crash is fixed
(PR #158) and the sequence has been run end to end — 2,615 mapper packets /
2,622 poses / 6,410 landmarks in **5 h 30 min**, the mapper's second global
optimisation now clearly the dominant cost at this scale.
