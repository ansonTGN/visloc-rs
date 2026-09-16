# LaMAria plan — from first official-metric scores to the leaderboard

Goal: reach **#1 on the [LaMAria leaderboard](https://lamaria.ethz.ch/leaderboard)**
(ETH, egocentric city-scale VI-SLAM on Project Aria glasses) with this
repository's Basalt Rust port, while keeping it real time. This document is the
milestone/lever plan that follows Stage 0 (the first official-metric scores,
[`lamaria_stage0.md`](lamaria_stage0.md)). Numbers here come from artifacts
named inline; nothing is tuned against ground truth unless stated.

## 1. Benchmark and metric

- **Sensors**: Project Aria, 2 SLAM cameras (ASL release: pinhole-undistorted
  ~758×572, fx≈241 → ~115° HFOV) + IMU 1 kHz. Sequences run 0.5–2.9 km / 5–48
  min; the test set is 63 sequences (Short 18, Medium 10, Long 16, Low light 9,
  Moving platform 10).
- **Metric** (`cvg/lamaria` evaluator, unmodified): a robust Sim(3) alignment to
  surveyed control points, then a per-control-point piecewise score (20 points
  below 5 cm, decaying to 0 at 10 m). Report: **Score** 0–100, **CP@1m** recall,
  and `evaluate_wrt_pgt` pose recall at 1 m/5 m (pseudo-GT), using the same
  Sim(3). Submission is TUM `ns`, `world_from_imu`, all image timestamps, zipped
  as `slam/<seq>.txt`, with a 24 h submission rate limit.
- **Headlines on the leaderboard (bino+imu track, as recorded 2026-09-15)**:

  | Method | Short | Medium | Long | Low light | Moving |
  |---|---:|---:|---:|---:|---:|
  | Aria's own SLAM (proprietary, non-causal) | 90.7 | 78.5 | 70.9 | 84.2 | 55.0 |
  | microSLAM (mono) | 64.5 | 74.5 | 82.3 | 78.3 | 85.6 |
  | **Ours (training, R_11_5cp VIO+mapper)** | **57.66** | — | — | — | — |
  | Best academic (OpenVINS+Maplab) | 27.7 | 23.4 | 12.8 | 19.8 | 13.9 |
  | OKVIS2 | 20.0 | 11.6 | 2.6 | 14.5 | 4.7 |

- Test data must be **streamed and deleted** (~862 GB total; server throttles
  per connection — use `aria2c -x16`, ~57 MB/s, from the official file list).

## 2. Where we stand (2026-09-16)

Training-sequence results, all from the official evaluator
(`docs/lamaria_stage0.md`, PR #156):

| Sequence | Track | Stage | Score | CP@1m | Pose R@5m | ATE | Sim3 scale |
|---|---|---|---|---|---|---|---|
| R_11_5cp | training | VIO only | 49.65 | 20.0 % | 100 % | 1.94 m | 0.965 |
| R_11_5cp | training | + offline mapper | 57.66 | 20.0 % | 100 % | 1.37 m | 0.960 |
| sequence_1_19 | Short (1.5 km, 15 min) | VIO only | 12.75 | 7.1 % | 5.4 % | — | — |
| sequence_1_19 | Short (1.5 km, 15 min) | + mapper | *pending* | | | | |

Infrastructure already in place: real-time two-thread VIO pipeline (RTF 1.07
EuRoC / 1.10 R_01 near-uncontended, PR #153), online NFR mapper that keeps pace
with the VIO on EuRoC (8/11 vs ORB-SLAM3, PR #155), offline-mapper O(N²)
matching and RSS fixes (PR #156), and the streaming-trace fix that unblocked
long LaMAria sequences at 18k+ frames (PR #158, in review).

The two open gaps that the numbers expose:

1. **VIO-only drift over km / minutes** — sequence_1_19 VIO-only is 12.75
   (Pose R@5m 5.4 %): the mapper is not optional on the Short track, let alone
   Medium/Long.
2. **Mapper cost at city scale** — the online mapper runs ~2.4 s/packet vs the
   ~0.4 s keyframe interval (the real-time blocker), and the offline mapper's
   second global optimisation took 1,151 s alone for 1,357 packets (R_11_5cp),
   scaling roughly with packet count. Both must improve before a full test
   sweep (22 h of data) is affordable.

## 3. Milestones

Each milestone has an explicit exit criterion and is validated on **training**
sequences only (test scores are only obtainable by submission).

| # | Milestone | Exit criterion | Status |
|---|---|---|---|
| M0 | Stage 0: first official scores; mapper usable at scale | R_11_5cp 49.65 VIO / 57.66 mapper; sequence_1_19 VIO 12.75; mapper perf/memory fixes merged | **done** (PR #156) |
| M1 | Beat the best academic baseline on Short | mapper-scored sequence_1_19 (and ≥3 Short training sequences) ≥ 27.7, toward microSLAM's 64.5 | in progress |
| M2 | Medium/Long consistency | loop-closure/mapper holds at km scale; mean Short/Medium/Long Score clearly above 27.7; no >2 km drift blow-up | not started |
| M3 | Low-light robustness | low-light training sequences score comparably to Short; no tracking loss | not started |
| M4 | Moving-platform handling | moving-platform training sequences tracked without divergence | not started |
| M5 | Full test submission | all 63 sequences processed at ≥1× RT, `slam/<seq>.txt` built, submitted within the 24 h rate limit | not started |

## 4. Levers (a-priori, ranked)

These are the candidate changes, ordered by expected payoff per unit of risk.
None may be tuned against ground truth; any accuracy claim is a same-protocol
before/after on training sequences.

1. **Mapper throughput and scaling.** The online mapper's per-packet cost
   (~2.4 s vs the ~0.4 s keyframe interval) and the offline mapper's second
   global optimisation (1,151 s / 1,357 packets) are the gating costs for a
   full test sweep. Levers: parallelise per-pair matching/RANSAC and per-image
   detection, cap BoW candidates, and expose iteration/window caps for the
   second optimisation. Gate: deterministic output order and ATE within 5 % of
   the unoptimised baseline. `BASALT_ONLINE_MAPPER_TRACE=1` gives per-packet
   cost splits.
2. **Global consistency / loop closure at km scale.** sequence_1_19 VIO-only
   (1.5 km) loses the pGT recall entirely; the mapper plus loop closure must
   recover it. Levers: the inverted BoW index (already merged, but weaker on
   repetitive wide-FOV texture), loop-verification thresholds, and the
   persistent-map/pose-graph path explored on EuRoC
   ([`vi_slam_global_consistency_plan.md`](vi_slam_global_consistency_plan.md)).
3. **VIO tracking robustness.** Levers: wide-FOV optical-flow settings
   (`optical_flow_detection_grid_size`, pyramid `levels`, `pattern`,
   `vio_obs_std_dev`) — currently EuRoC values on a ~115° camera, so periphery
   KLT degradation is plausible and unverified; and initialization / accel-bias
   behaviour (R_01 accel bias converges to ~0.5 m/s², suspiciously absorbing
   init scale/gravity residual; check static-start handling).
4. **Online focal-length estimation.** Aria's focal length changes ~0.11 %
   over a session; the current calibration is fixed per sequence. An online
   focal estimate removes a systematic scale/drift source.
5. **Low-light and moving-platform robustness.** Distinct failure modes
   (feature starvation; tram/motion dynamics). Likely needs frontend-level
   work rather than mapper tuning.
6. **Runtime headroom.** 22 h of test data at ≥1× RT is required. VIO is
   already ~1.1× (near-uncontended); the mapper (lever 1) is the open item.

## 5. Test-set operational pipeline

1. Download one sequence with `aria2c -x16` from the official test file list.
2. Run VIO (`--pipeline --threads 12`, variant-A default IMU noise), then the
   mapper (`--temporal-seed 7`; reduced-points config if it exceeds ~1 h or
   ~12 GB RSS).
3. Propagate corrections (`scripts/propagate_basalt_mapper_corrections.py`) and
   convert to the submission format
   (`scripts/basalt_tum_to_lamaria_estimate.py`).
4. Delete the sequence data; add `slam/<seq>.txt` to the submission zip.
5. Submit via <https://lamaria.ethz.ch/login> (24 h rate limit).

## 6. Caveats and risks

- **R_11_5cp has only 5 control points**, so its Score/CP@1m has high variance
  and is a genuine-but-single data point, not a stable estimate.
- **All numbers to date are training sequences**, not the held-out test set;
  only a submission produces a leaderboard number.
- **Sim(3) alignment forgives global scale**, so metric-scale bias shows up as
  lower Score rather than as an obvious trajectory error.
- The evaluation is robust to a few outliers by construction; a single bad
  control point cannot dominate a sequence score.
- Compute budget: full test sweep is ~22 h of sensor data; mapper throughput
  (lever 1) must be resolved before it is affordable.

## See also

- [`lamaria_stage0.md`](lamaria_stage0.md) — method, results, mapper fixes.
- [`vi_slam_benchmarks.md`](vi_slam_benchmarks.md) — EuRoC VI-SLAM results.
- [`vi_slam_global_consistency_plan.md`](vi_slam_global_consistency_plan.md) —
  global-consistency evidence and staged plan.
- [`basalt_online_mapper_design.md`](basalt_online_mapper_design.md) — online
  mapper architecture.
