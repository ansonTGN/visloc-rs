# Map-matching localization for a prebuilt RNE map (plan)

Status: planning (2026-09-18). Scope: implement **map-matching relocalization** in
visloc-rs so a RoboSim/RNE episode can be localized against a map built from a
previous RNE episode. This is the "M2" of
`RoboSim:docs/VISLOC_NAVIGATION_PLAN.md`, but the deliverable lives in visloc-rs.

## 1. Problem and scope

Given:

- a **prebuilt map** built offline from an RNE episode (no ground truth used), and
- a **query episode** (different start pose, same scene) with cam0/cam1 + IMU,

produce, for each query camera frame, a 6-DoF pose in the **map frame**
(`map -> base_link`, plus its planar `Pose2d`), with bounded per-frame latency and
deterministic output. No temporal or odometry prior is assumed for the first
frame; a prior is available for subsequent frames.

In scope: descriptor-based 2D-3D matching against the map, geometric verification
(PnP RANSAC), retrieval/covisibility gating, pose-prior tracking mode, and
optional visual-inertial fusion of the map constraint.

Out of scope: online map *building* during localization, multi-robot maps,
appearance-invariant training, dense/direct odometry.

## 2. Survey — what the literature and OSS establish

### 2.1 The canonical prior-map localization pipeline

maplab's ROVIOLI is the closest match to our problem (VIO + prior map, online):

- Lynen et al., *Get Out of My Lab: Large-scale, Real-Time Visual-Inertial
  Localization*, RSS 2015.
- Schneider et al., *maplab: An Open Framework for Research in Visual-inertial
  Mapping and Localization*, RA-L 2018.

maplab's loop-closure / localization engine (documented workflow) is a three-stage
2D-3D scheme that we should copy almost verbatim:

1. **Nearest-neighbour search** in the projected descriptor space (inverted
   multi-index) → candidate 2D-3D correspondences.
2. **Covisibility filtering** using the map's covisibility graph, keeping only
   matches pointing to the most widely matched places.
3. **PnP + RANSAC** geometric verification of the 2D-3D matches.

The global pose is then fed into the estimator as a *localization constraint*
fused with odometry. Geneva et al., *Map-based Visual-Inertial Localization: A
Numerical Study*, ICRA 2022, compare the fusion choices and recommend: full joint
estimation for tiny workspaces; **Schmidt-Kalman** for small workspaces
(consistent); **measurement inflation** for large maps (efficient, slightly
inconsistent); and note **2D-3D landmark maps are the most accurate** vs
keyframe/2D-2D maps.

### 2.2 Hierarchical localization (retrieval + local matching)

The modern baseline for single-image localization against a large map:

- Sattler et al., *Benchmarking 6DOF Outdoor Visual Localization in Changing
  Conditions*, CVPR 2018 (benchmark; active-search baseline).
- Sattler et al., *Efficient & Effective Prioritized Matching for Large-Scale
  Image-Based Localization*, TPAMI 2017 (priority matching; disambiguation).
- Sarlin et al., *From Coarse to Fine: Robust Hierarchical Localization at Large
  Scale*, CVPR 2019 → **hloc**: image retrieval (NetVLAD-style) picks top-K
  reference images, then local features are matched query↔reference, then PnP.
- Sarlin et al., *Back to the Feature / PixLoc*, CVPR 2021: keep geometry
  classical, learn robust features, and refine the pose by **direct alignment of
  multiscale deep features** to the 3D model. Works from a coarse retrieval
  prior and can refine any other method's pose.

OSS: `cvg/Hierarchical-Localization`, `cvg/pixloc`, `colmap/colmap`,
`colmap/glomap`.

### 2.3 Local features, matchers, retrieval

- DeTone et al., **SuperPoint**, CVPRW 2018.
- Sarlin et al., **SuperGlue**, CVPR 2020; Lindenberger et al., **LightGlue**,
  ICCV 2023 (faster).
- Tyszkiewicz et al., **DISK**, NeurIPS 2020.
- VPR/retrieval: Arandjelović et al., **NetVLAD**, CVPR 2016; Berton et al.,
  **CosPlace**, CVPR 2022; **EigenPlaces**, ICCV 2023; **MegaLoc**, CVPRW 2025
  (SOTA on the LaMAR visual-localization benchmark when only the retrieval
  stage is swapped).
- Robust estimation: Fischler & Bolles, RANSAC, CACM 1981; Barath et al.,
  **MAGSAC++**, CVPR 2020; Larsson et al., **PoseLib** (minimal solvers + LO-RANSAC
  + non-linear refinement, `PoseLib/PoseLib`).

### 2.4 Learned coordinate/pose regression (alternative, not our baseline)

- Brachmann et al., **DSAC** (CVPR 2017) / **DSAC++** (CVPR 2018);
  **ACE: Accelerated Coordinate Encoding**, CVPR 2023.
  These regress scene coordinates and can localize without descriptors, but they
  require per-scene training and are a poor fit for a simulator-generated map we
  rebuild often. Keep as a future option only.

### 2.5 VIO-side map reuse (our own stack)

- Usenko et al., *Basalt: Visual-Inertial Mapping with Non-Linear Factor
  Recovery*, RA-L 2020 — the mapping layer uses **ORB keypoint matching +
  covisibility** for loop closures (distinct from the KLT VIO frontend). Visloc's
  NFR mapper is a port of this.
- Mur-Artal & Tardós, **ORB-SLAM2**, T-RO 2017; Campos et al., **ORB-SLAM3**,
  T-RO 2021 — DBoW2 place recognition + EPnP/RANSAC relocalization + multi-map;
  the origin of the "retrieval + PnP, then fuse" pattern for VI-SLAM.

## 3. What visloc-rs already has (reuse, do not rebuild)

| Capability | Location |
| --- | --- |
| Descriptor 2D-3D localization pipeline | `pipelines/localization` (`LocalizationPipeline`, `ImageLocalizer`, `localize_frame_with_descriptor_store`) |
| Pose-prior + radius submap selection | `LocalizationPrior`, `RadiusSubmapSelector`, `PendingSubmap`/`ProjectionCorrespondenceBuilder` |
| Landmark descriptors | `LandmarkDescriptorStore` (`crates/core/src/types/map.rs`) |
| Map/query IO | `crates/io`: `read_colmap_text_model`, `read_landmark_descriptors_txt`, `read_query_features_txt` |
| Feature extractors | `CornerFeatureExtractor`, `HogLikeFeatureExtractor`, `SuperPointOffnxExtractor`, `MultiScaleDeepExtractor`, `GlobalDescriptorOnnxExtractor` (`crates/vision/src/features`) |
| Matchers | `BruteForceMatcher`, `MutualSoftmaxMatcher` (LightGlue-style) |
| Odometry-side recovery | `OnlineSlamConfig::relocalization` (`FrameLocalizer`, pose-prior warm start) |
| SfM / COLMAP-port map builders | `examples/colmap_incremental_mapper.rs`, `examples/unordered_sfm_demo.rs`, `examples/sequential_sfm_demo.rs` |
| Basalt NFR map | `pipelines/basalt` (`basalt_mapper_offline_demo`) → `map.json` (NFR tracks), `points.json` (patches), `poses.json` (keyframes) |
| Prior plans to align with | `docs/openloris_map_relocalization_plan.md`, `docs/learned_retrieval_relocalization.md`, `docs/superpoint_lightglue_plan.md` |

`docs/openloris_map_relocalization_plan.md` already defines the exact shape we
need: *map from our own mapper + a `landmark_descriptors.txt` export + an example
that runs `localize_frame_with_descriptor_store` + an evaluation script.* We
should mirror it for RNE.

### Gap

The Basalt NFR map is a **relative track map** (per-track `direction`,
`inverse_distance`, host/second keyframe), and its patches are KLT patches, not
global descriptors. The localization pipeline wants
`VisualMap.landmarks[].descriptor` in a metric map frame. So the gap is the
**map representation bridge** and a **query feature/descriptor path**, not the
localizer itself.

## 4. Design

### 4.1 Map representation — two options

**Option A (recommended): descriptor map built by our SfM/COLMAP port.**
Build the map from the RNE episode's cam0 images with visloc's mapper, then
export `landmark_descriptors.txt` (representative observation per landmark). This
is exactly the OpenLORIS plan, reuses `ImageLocalizer` end-to-end, and gives us
COLMAP interoperability for free.

**Option B: Basalt NFR → `VisualMap` adapter.**
Project NFR tracks to metric landmarks using `poses.json`, attach each track's
`points.json` patch as an optional descriptor, and add a patch-correlation
descriptor/matcher. This reuses the M1 Basalt map but adds a new descriptor type
and matcher (more code, and the repetitive checker scene makes patch NN
ambiguous).

Recommendation: **Option A** for the first working localizer; keep Option B as a
follow-up only if we want to avoid SfM in the loop.

### 4.2 Algorithm (per query frame)

```
query image (cam0, optional cam1)
  -> local features + descriptors        (CornerFeatureExtractor/HOG first; SuperPoint optional)
  -> retrieval / prior submap selection  (RadiusSubmapSelector on pose prior; global descriptor for no-prior first frame)
  -> 2D-3D NN + ratio test               (BruteForceMatcher)
  -> covisibility filtering              (reject candidates not backed by a covisible cluster)
  -> PnP RANSAC + non-linear refinement  (PnPRansac; PoseLib-style LO-RANSAC if we need it)
  -> gates: min_inliers, min_inlier_ratio, max reprojection error
  -> pose in map frame (+ inlier stats, latency)
```

For the **no-prior first frame** use a global descriptor for top-K retrieval
(Phase L2). For all later frames feed the previous pose (plus IMU prediction) as a
`LocalizationPrior { pose, radius }` so the localizer only sees the local submap.

### 4.3 Visual-inertial fusion (for M3 navigation)

Two supported modes, matching Geneva et al.:

- **Measurement inflation** (default, simple): treat the map constraint as an
  independent pose measurement with inflated covariance; no map states in the
  estimator. Easy to bolt onto the existing online SLAM.
- **Schmidt-Kalman** (optional): keep map landmarks as non-updated "schmidt'ed"
  states for consistency in small workspaces.

The map→odom transform is then a filtered quantity; base_link pose is
`T_map_odom * T_odom_base`.

### 4.4 Phases

- **L0 — Map build + descriptor export.** Wire the RNE `mav0/` cam0 images into
  the existing mapper/COLMAP port; emit COLMAP text model + `landmark_descriptors.txt`
  + per-query `features.txt`. Deliverable: `scripts/export_rne_localization_map.py`
  (or a Rust example) + a documented map format.
- **L1 — Single-frame map matching (no prior).** New example
  `examples/localize_rne_map.rs` that loads map + descriptors, localizes each
  query frame, and writes `localization.tum` + stats. Reuse
  `localize_frame_with_descriptor_store`. Success metric: localization rate and
  pose error vs GT (GT post-hoc only).
- **L2 — Retrieval + covisibility gating.** Add a global-descriptor retrieval
  gate for the first frame (reuse `GlobalDescriptorOnnxExtractor`; A/B the
  incumbent `normalized_mean` vs EigenPlaces/MegaLoc) and covisibility filtering
  of candidate landmarks. This is where `docs/learned_retrieval_relocalization.md`
  says the win is, so measure it.
- **L3 — Tracking mode.** Per-frame localization with a pose-prior window
  (`RadiusSubmapSelector`), latency budget, and optional multi-frame smoothing.
  Deliverable: per-frame `map -> base_link` at camera rate with p50/p95 latency
  and a determinism check (bit-exact on replay).
- **L4 (optional) — Direct refinement / learned descriptors.** PixLoc-style
  feature-alignment refinement of the PnP pose; SuperPoint+LightGlue descriptors
  when HOG descriptor quality is the bottleneck (see `superpoint_lightglue_plan.md`).
- **L5 (optional) — Basalt NFR-native map.** Option B above.

## 5. Interfaces

Rust (new, additive):

- `scripts/export_rne_localization_map.py` — RNE `mav0/` → COLMAP model +
  `landmark_descriptors.txt` + query features.
- `examples/localize_rne_map.rs` — CLI:
  `--map-dir`, `--descriptors`, `--query-features`, `--query-images`,
  `--config`, `--prior-radius-m`, `--out-dir`; writes `localization.tum`,
  `localization_report.json`.
- Reuse `LocalizationConfig` for gates; no changes to `pipelines/localization`
  unless covisibility filtering needs a small `CandidateSelector` addition.

RoboSim integration: a `publish = false` adapter crate/example (never in
`rne_core`) that (i) captures a query episode to `mav0/`, (ii) invokes
`localize_rne_map`, and (iii) consumes `localization.tum` as `map -> base_link`
for `rne_nav`. Because it is an offline file hand-off, it stays deterministic and
does not couple RNE to visloc's build.

## 6. Evaluation protocol

Datasets: two RNE episodes in the same scene from different starts
(episode A = map; episode B = query, including a query whose start is *outside*
the mapped path to force retrieval). Optionally hold out every other keyframe
from the map build (OpenLORIS E2/E3 parity protocol) to get a clean held-out set.

Metrics:

1. **Localization rate** — fraction of queries producing a pose that passes the
   gates.
2. **Pose error** — translation (m) and rotation (deg) vs GT, at thresholds
   (e.g. 0.05 m / 2°, 0.25 m / 5°); report p50/p90/p95.
3. **Inlier stats** — inliers, inlier ratio, mean reprojection error.
4. **Latency** — per-frame p50/p95 wall time, and throughput vs camera rate.
5. **Determinism** — two runs bit-identical (`localization.tum` hash).
6. **Navigation effect (M3)** — ATE of the closed loop and goal-reaching success
   with map constraints on/off.

Negative controls: localize against a map built from a *different* scene
(should fail the gates); repeat the repetitive checker scene to expose aliasing
(expect this to be the hardest case and a good motivation for retrieval +
covisibility gating).

## 7. Risks and decision gates

- **Repetitive synthetic texture** (regular checkerboard) is a worst case for
  descriptor NN and will alias. Gate: if L1 localization rate is low, go to L2
  (covisibility retrieval) before touching descriptors.
- **Descriptor mismatch under large viewpoint/attitude change** was the binding
  constraint in the EuRoC cliff analysis (`superpoint_lightglue_plan.md`). Gate:
  if L2 gating is not enough, test SuperPoint+LightGlue (L4) as an A/B, matching
  the `learned_retrieval_relocalization.md` methodology (isolate retrieval recall,
  then end-to-end).
- **Map-region ambiguity** — add a `radius_m` prior and require a minimum
  covisible cluster size.
- **Frame convention** — RNE is Y-up, Basalt/visloc is Z-up; the map→RNE
  transform must be stored with the map and reused by localization and navigation
  (already flagged in the RoboSim plan).

## 8. References

Papers

- Fischler & Bolles, *Random Sample Consensus*, CACM 1981.
- Arandjelović et al., *NetVLAD*, CVPR 2016.
- Sattler et al., *Efficient & Effective Prioritized Matching*, TPAMI 2017.
- Brachmann et al., *DSAC*, CVPR 2017; *DSAC++*, CVPR 2018.
- Mur-Artal & Tardós, *ORB-SLAM2*, T-RO 2017.
- Wang et al., *VINS-Mono*, T-RO 2018.
- DeTone et al., *SuperPoint*, CVPRW 2018.
- Sattler et al., *Benchmarking 6DOF Outdoor Visual Localization*, CVPR 2018.
- Engel et al., *Direct Sparse Odometry*, T-RO 2018.
- Schneider et al., *maplab*, RA-L 2018.
- Sarlin et al., *From Coarse to Fine (hloc)*, CVPR 2019.
- Usenko et al., *Basalt: Visual-Inertial Mapping with Non-Linear Factor
  Recovery*, RA-L 2020.
- Barath et al., *MAGSAC++*, CVPR 2020.
- Sarlin et al., *SuperGlue*, CVPR 2020; Lindenberger et al., *LightGlue*,
  ICCV 2023.
- Tyszkiewicz et al., *DISK*, NeurIPS 2020.
- Sarlin et al., *Back to the Feature (PixLoc)*, CVPR 2021.
- Campos et al., *ORB-SLAM3*, T-RO 2021.
- Geneva et al., *Map-based Visual-Inertial Localization: A Numerical Study*,
  ICRA 2022.
- Sarlin et al., *LaMAR*, ECCV 2022.
- Berton et al., *CosPlace*, CVPR 2022; *EigenPlaces*, ICCV 2023; *MegaLoc*,
  CVPRW 2025.
- Brachmann et al., *ACE: Accelerated Coordinate Encoding*, CVPR 2023.

OSS

- `cvg/Hierarchical-Localization` (hloc), `cvg/pixloc`
- `colmap/colmap`, `colmap/glomap`
- `PoseLib/PoseLib`
- `ethz-asl/maplab`
- `VladyslavUsenko/basalt`
- `cvg/LightGlue`, `magicleap-ai/SuperGluePretrainedNetwork`
- `gmberton/MegaLoc`, `gmberton/CosPlace`, `gmberton/EigenPlaces`
- `princeton-vl/DROID-SLAM` (future dense option)

visloc-rs internal

- `docs/openloris_map_relocalization_plan.md` (the pattern to mirror)
- `docs/learned_retrieval_relocalization.md` (retrieval is the binding constraint)
- `docs/superpoint_lightglue_plan.md` (descriptor quality lever)
- `RoboSim:docs/VISLOC_NAVIGATION_PLAN.md` (M2/M3 integration)
