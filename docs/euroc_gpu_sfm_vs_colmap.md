# EuRoC GPU SfM vs COLMAP (CUDA)

A same-input, same-machine, back-to-back comparison of the visloc-rs GPU
SfM pipeline (`gsplat_euroc`, COLMAP-port mapper) against COLMAP 4.1 with
GPU feature extraction and GPU matching. The frames come from 8 EuRoC MAV
sequences, and accuracy is scored against the Vicon/Leica ground truth.

## Setup

- **Input:** 200 frames per sequence (cam0, every 4th frame from the
  start), undistorted to a pinhole camera with the EuRoC intrinsics.
  - `gsplat_euroc` writes these PNGs, and COLMAP reads the *same* files
    with the *same* fixed `PINHOLE` intrinsics.
  - Neither side refines intrinsics.
- **Hardware:** GTX 1660 Ti (6 GB), Windows 11. Both pipelines ran back to
  back in one session, via `scripts/benchmark_euroc_vs_colmap.py`.
- **COLMAP 4.1 (CUDA):**
  - `feature_extractor` with GPU on.
  - `sequential_matcher` (COLMAP's video preset) with GPU on.
  - `mapper` with defaults; focal, principal point and extra parameters
    are held fixed.
  - The largest model is scored.
- **visloc-rs:** the command is

  ```text
  gsplat_euroc --euroc <seq> --work <dir> --stride 4 --max-frames 200 --steps 0 \
    --gpu-sift --gpu-ba --mapper colmap-port --sift-l1-root \
    --sift-opt descriptor_magnification=3 --sift-opt max_orientations=2 \
    --keypoints 4000 --sift-opt prefer_larger_scale=1 \
    --keep-planar --verify-min-inliers 15 --register-gated
  ```

  - Features come from GPU SIFT (RootSIFT, COLMAP/VLFeat descriptor window,
    ≤4000 keypoints, larger scales kept first).
  - Matching is batched on the GPU: cross-checked, window 10 plus
    long-range skip pairs. It is overlapped with CPU two-view verification
    that ports COLMAP's E/F/H classification.
  - Mapping uses the faithful COLMAP incremental-mapper port
    (`visloc_slam::colmap_incremental`).
  - Near-static frames are gated out and then re-registered by PnP.
- **Metric:** Sim(3)-aligned RMSE of camera centres against ground truth,
  over the frames each method registered. The "Time" column is:
  - **COLMAP:** extract + match + map.
  - **visloc-rs:** everything up to the SfM result, including decoding,
    undistorting and writing the frames.

## Results

| Sequence | visloc-rs time | COLMAP time | visloc-rs reg. | COLMAP reg. | visloc-rs ATE | COLMAP ATE |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| MH_01_easy | **96 s** | 634 s | 182 | **200** | **0.35 cm** | **0.35 cm** |
| MH_03_medium | **113 s** | 340 s | 166 | **200** | **1.32 cm** | 2.61 cm |
| MH_05_difficult | **102 s** | 764 s | 194 | **200** | **2.58 cm** | 193.66 cm |
| V1_01_easy | **104 s** | 219 s | 199 | **200** | **2.40 cm** | 2.75 cm |
| V1_02_medium | **55 s** | 101 s | 198 | **200** | **1.76 cm** | 1.83 cm |
| V2_01_easy | **81 s** | 172 s | 199 | **200** | 3.30 cm | **1.00 cm** |
| V1_03_difficult | **45 s** | 93 s | 67 | **80** | 2.17 cm | **1.98 cm** |
| V2_03_difficult | **42 s** | 60 s | 118 | **180** | 3.37 cm | **2.85 cm** |

**Summary:**

- **Speed:** visloc-rs is faster on all 8 sequences, 1.4–7.5×.
- **Accuracy:** visloc-rs is more accurate on 4 of 8 (MH_03, MH_05, V1_01,
  V1_02) and equal on MH_01. COLMAP is more accurate on V2_01, V1_03 and
  V2_03.
- **Registration:** COLMAP registers more frames everywhere. The gap is
  small on the easy/medium sequences (182–199 vs 200) and large on
  V1_03/V2_03.

## MH_05 animation

![EuRoC MH_05: COLMAP vs visloc-rs trajectories, then the 3DGS flythrough](assets/euroc_mh05_vs_colmap.gif)

**Trajectories:** the COLMAP and visloc-rs trajectories are Sim(3)-aligned
to ground truth, top-down. The data comes from the COLMAP run of the table
above and the `gsplat_euroc` run below.

**Flythrough:**

- **Pipeline:** the same `gsplat_euroc` configuration trains a 3DGS scene
  for 7000 steps (215 s) and renders every registered camera with
  `--render-dir`.
  - Eval: PSNR 24.1, SSIM 0.876 on 25 held-out views.
- **Frames shown:** the animation starts at 15% of the sequence. The first
  frames are near-static close-ups during take-off, seen from few
  viewpoints, and render poorly.
- **Rebuild:**

  ```text
  gsplat_euroc --euroc <MH_05> --work <run> --stride 4 --max-frames 200 --steps 7000     <configuration above> --render-dir <run>/renders
  python scripts/make_euroc_vs_colmap_gif.py --euroc-root <EuRoC> --bench-out <out>     --run <run> --out docs/assets/euroc_mh05_vs_colmap.gif --colmap-time 764 --ours-time 102
  ```

## Caveats

- **COLMAP varies between runs; visloc-rs does not.**
  - In an earlier run on the same inputs, COLMAP scored V2_01 at 34.9 cm
    instead of 1.00 cm, and V1_02 at 1.75 cm.
  - MH_05 failed (about 200 cm) in both runs.
  - visloc-rs is deterministic (seeded RANSAC, fixed GPU batch order).
- **Blur breaks the reconstruction on V1_03 and V2_03.**
  - On V2_03 the pairs that cross the break carry about half COLMAP's
    inliers, so the model splits and only the largest part is scored.
  - `--merge-models` (a similarity estimated from cross-model matches)
    cannot bridge it, because the bridging frames are unregistered.
  - The remaining gap is SIFT quality on blurred frames.
- **The visloc-rs time includes writing the undistorted PNGs** that COLMAP
  then reads.
- **The visloc-rs times come from a rerun.** They were re-measured the same
  day, on the same machine, after the two-view RANSAC speed-ups (commits
  2e0b525 and 79acc65). Those commits leave every verification decision,
  registration and ATE unchanged; before them the times were 131 / 175 /
  132 / 157 / 81 / 117 / 68 / 64 s. The COLMAP numbers come from the
  back-to-back run.

## What moved the numbers

These are V1_02, 200 frames. The rows below "COLMAP features +
correspondences" all use the COLMAP-port mapper.

| Change | ATE |
| --- | ---: |
| Original `incremental_sfm` mapper | 8.13 cm |
| COLMAP features + correspondences → our old mapper | ~100 cm (scale collapse mid-growth) |
| COLMAP features + correspondences → COLMAP-port mapper | 2.12 cm |
| Our GPU SIFT → COLMAP-port mapper | 7.54 cm |
| + RootSIFT + planar pairs kept | 4.16 cm |
| + `descriptor_magnification=3` (COLMAP/VLFeat window) | 2.73 cm |
| + `max_orientations=2` | 2.38 cm |
| + 4000 keypoints, larger scales first | **1.68 cm** |

- **Mapper:** the old mapper's growth path is the defect. Seeded with
  COLMAP's poses, its BA reaches 2.15 cm.
- **PnP inlier floor:** the port's `abs_pose_min_num_inliers` must be
  COLMAP's 30. The port default of 8 is a rig-benchmark override, and it
  broke MH_01 (13 cm).

## Closing the registration and accuracy gap (2026-09-28)

COLMAP registers more frames on every sequence, and it is more accurate on
V2_01, V1_03 and V2_03. Most of the frame gap comes from the port mapper
splitting the sequence into several models; only the largest is scored.

| Sequence | Model sizes | COLMAP registered |
| --- | --- | ---: |
| MH_03 | 166, 45, 2 | 200 |
| V2_03 | 118, 56 | 180 |
| V1_03 | 67, 65, 31, 19 | 80 |

- **V2_03:** the split is at the badly blurred frame 56. Only 4 verified
  pairs cross it, with at most 41 matches.
- **V1_03:** two of the models overlap in time, sharing 35 verified pairs
  and 1,074 matches.

Nothing below moved the table without a regression somewhere. The options
were removed from `gsplat_euroc` on 2026-09-28; this table is their record.

| Change | Gains | Regressions |
| --- | --- | --- |
| `--merge-models` (Sim(3) weld + joint BA) | MH_03 199/200 | MH_03 ATE 1.32 → 2.04–2.95 cm; V2_03 not welded (1 link); V1_03 weld rejected (41/140 inliers) |
| `--register-gated-links 3` (gated frame → 3 kept frames per side) | MH_01 182 → 190; V1_02 198 → 200 | MH_03 ATE 1.32 → 13.1 cm (2 links: 3.70 cm) |
| `--keypoints 8000` (COLMAP's cap is 8,192) | V2_01 ATE 3.30 → 2.02 cm | MH_05 2.58 → 5.09 cm; V1_02 1.76 → 2.89 cm; fewer frames registered |
| `--polish` (re-triangulate all pairs + global BA) | MH_03 198/200 at 2.27 cm; V2_01, V1_02 and MH_05 200/200; V1_03 96 frames | MH_01 ATE 0.35 → 1.10 cm; V1_03 2.17 → 7.34 cm |

Three more attempts. The first two were also removed; the third remains
possible through `--import-features`:

- **Weak-link rescue.** `--rescue-weak 100 [--rescue-ratio 0.9]
  [--rescue-add-only]` re-matches nearby pairs that have fewer than 100
  verified matches, using a relaxed ratio and no cross-check.
  - Replacing the original pairs with the relaxed matches registers more
    frames but costs accuracy: V1_03 goes 67 → 105–107 frames (COLMAP: 80)
    at 8–10 cm, and V2_03 lands at 6.6–16 cm. MH_03 does improve, from
    1.32 to 1.07 cm.
  - Adding only the pairs that were missing changes almost nothing.
- **V2_01 scale drift.** Aligned on its own, each half of V2_01 fits
  ground truth to 0.5–0.7 cm, but the two halves' Sim(3) scales differ by 3%
  (2.2505 vs 2.1818).
  - Skip pairs out to 195 frames added 3,090 candidates, but none of the new
    long-range pairs passed verification (2,264 verified either way).
  - The sequence never revisits a view, so there is no loop to close the
    drift.

- **SuperPoint features.** `scripts/export_superpoint_bins.py` feeds
  `--import-features`, which now takes any descriptor length.
  - SuperPoint bridges the blur on both hard sequences:
    - V2_03 registers 172/200 frames in a single model (COLMAP: 180; SIFT:
      118).
    - V1_03 registers 118 (COLMAP: 80).
  - Accuracy drops: V2_03 goes to 4.37 cm (SIFT 3.37, COLMAP 2.85) and V1_03
    to 8.74 cm (SIFT 2.17, COLMAP 1.98). A likely cause is that the ONNX
    export returns integer-pixel keypoints. Refining them on the image with
    `cv2.cornerSubPix` (5×5 window, shifts under 1 px) did not settle it:
    - V2_03 got worse (5.03 cm at 172 frames).
    - V1_03 got better but registered fewer frames (3.46 cm at 98 frames).
    - So quantisation is not the main cause. SuperPoint's keypoint
      localisation is simply looser than SIFT's sub-pixel extrema.
- **Denser skip pairs on V2_01.** Skips every 10 frames out to 190 give
  200/200 frames at 2.90 cm, against 199 frames at 3.30 cm.

The remaining gap is accuracy on the blurred sequences and V2_01's scale
drift.

### Hybrid SP bridge / SP-seeded SIFT refinement (2026-09-28)

The idea: use SuperPoint only to *connect* the blur break (topology), keep
SIFT as the *geometry* everywhere else, hoping to get SP's connectivity at
SIFT's accuracy. Two designs, both opt-in flags on `gsplat_euroc`, iterated
on V2_03 and V1_03 (the fast, clearly-split sequences) before deciding.
Both are honest negatives; the flags and code were removed.

- **SP bridge correspondences, SIFT geometry.** For candidate pairs the
  SIFT-only colmap-port mapper's own split shows it needs (probed with one
  extra mapper run, then matched only for pairs crossing a model boundary or
  touching an unregistered frame), match+verify SuperPoint too and merge the
  result into the pair's SIFT matches as extra keypoints/tracks.
  - **Targeted (boundary pairs only), V2_03:** 2-74 of a few hundred
    boundary candidates verified, 3-42 frames gained SP keypoints — but the
    mapper's own split never changed (`[118, 56]` before and after), so ATE
    and registration were unchanged (3.37 cm, 118/200). The SP correspondences
    it finds at the actual break are too few for the port's registration
    threshold (`abs_pose_min_num_inliers=30`) to grow through, matching the
    doc's earlier note that only ~4 pairs with ≤41 matches cross V2_03's
    break.
  - **Blanket (any window pair below a SIFT-match threshold), V2_03:**
    connects into one model (`[176]` at threshold 50, `[172]`ish scale) but
    wrecks accuracy: 8.03 cm at 176/200 (threshold 50), 13.78 cm at 174/200
    (threshold 100), 16.08 cm at 118/200 in 2 models (threshold 30). Only an
    almost-no-op threshold (1: only pairs SIFT verified zero matches for)
    left accuracy untouched (3.90 cm) — and also left the split unchanged
    (117+67 frames, no merge).
  - V1_03 (targeted): the mapper's own 4-model split (`[67, 65, 31, 19]`)
    did not consolidate either; bridging *added* a 5th model
    (`[67, 65, 31, 29, 19]`) with the largest unchanged at 67/200, 2.08 cm
    (SIFT baseline: 2.17 cm, noise-level difference).
- **SP-seeded SIFT refinement.** Seed the colmap-port mapper's
  `Reconstruction` directly from a single-model SuperPoint run's poses
  (`init_sp_*.txt`, generated once from `--import-features` + `--mapper
  colmap-port`) instead of incremental registration, triangulate the *SIFT*
  correspondence graph through those fixed poses, then run global bundle
  adjustment (several filter/re-adjust rounds, then
  `iterative_global_refinement`) so SIFT's precision can correct SP's looser
  localisation. Needed making `colmap_incremental::pipeline::reconstruction_from_cache`
  `pub` (reverted with everything else).
  - V2_03: 172/200 registered (matches the SP seed's own topology; COLMAP:
    180, SIFT-only largest model: 118), but ATE only reaches 4.56-4.60 cm —
    barely different from feeding SuperPoint's own keypoints straight into
    the mapper (4.37 cm, see above) and well short of SIFT-only's 3.37 cm or
    COLMAP's 2.85 cm. Three rounds of filter+re-adjust before the final
    refinement changed almost nothing (4.60 -> 4.56 cm).
  - V1_03: 118/200 registered (SP topology; COLMAP: 80, SIFT-only: 67), ATE
    8.90 cm — same ballpark as SuperPoint-only (8.74 cm), far worse than
    SIFT-only (2.17 cm) or COLMAP (1.98 cm).
  - Conclusion: SuperPoint's own incremental solve apparently bakes pose
    error into the seed that one (gauge-fixed, monocular-scale-free) global
    BA pass does not fully correct, even when every observation driving that
    BA is SIFT-precision. Re-triangulating SIFT through SP's poses recovers
    SP's own accuracy level, not SIFT's — the connectivity SP provides and
    the precision SIFT provides don't compose by this route.

Both routes confirm the earlier finding stands: SuperPoint can reach the
frames SIFT's mapper leaves split or unregistered, but nothing tried so far
carries SIFT's precision across that bridge. The remaining gap on V1_03,
V2_03 and V2_01 is still open.

### Bisecting the gap: mapper vs. frontend (2026-09-28)

To localize where V2_01, V1_03 and V2_03's remaining gap lives, COLMAP's own
keypoints and verified two-view matches (from its `db.db`) were fed straight
into the colmap-port mapper two ways: **A1** (COLMAP features *and* COLMAP
matches, via `--import-colmap`, bypassing our matching/verification
entirely) and **A2** (COLMAP features, but our own matching/verification,
via `--import-features`), compared against **A3** (our full pipeline,
i.e. the table above).

| Sequence | A3 (ours) | A1 (COLMAP feat+match → our mapper) | A2 (COLMAP feat + our matching) | COLMAP |
| --- | ---: | ---: | ---: | ---: |
| V2_01 | 199/200, 3.30 cm | 198/200, **1.00 cm** | 196/200, 1.20 cm | 200/200, 1.00 cm |
| V1_03 | 67/200, 2.17 cm | 62/200, 2.96 cm | 60/200, 12.40 cm | 80/200, 1.98 cm |
| V2_03 | 118/200, 3.37 cm | 126/200, 6.95 cm | 176/200, **3.22 cm** | 180/200, 2.85 cm |

- **V2_01 and V2_03: the mapper is not the problem.** A1 on V2_01 ties
  COLMAP's own ATE almost exactly (1.00 vs 1.00 cm) using our mapper on
  COLMAP's exact features and matches. A2 on V2_03 (COLMAP's features
  through our own matching) nearly matches COLMAP's registration and
  accuracy (176/200 at 3.22 cm vs COLMAP's 180/200 at 2.85 cm). On both
  sequences the gap is SIFT feature quality: our GPU SIFT's own keypoints,
  not the mapper or the matcher, are what cost accuracy.
- **V1_03: the mapper itself is short of COLMAP.** A1 gives the port mapper
  the *exact same* features and matches COLMAP used (756 verified pairs) and
  it still only registers 62/200 at 2.96 cm, against COLMAP's 80/200 at 1.98
  cm on that identical input. This is a real mapper-vs-mapper gap (growth/
  registration-order/recovery), independent of the frontend. A2 makes it
  much worse (60/200, 12.40 cm) — our own matching on COLMAP's features
  introduces bad correspondences the verified-COLMAP-matches in A1 don't
  have. Not investigated further this round; the next step is diffing the
  port's register-next-image loop (registration trials per image, local BA
  after each registration, init-pair retries, `min_num_matches`,
  re-registration after global BA) against COLMAP's
  `incremental_pipeline.cc`/`incremental_mapper.cc` on the A1 inputs.

  A first diagnostic pass on A1 (COLMAP's own features and matches, so the
  input is byte-identical to what COLMAP itself grew to 80/200 in one
  model): the colmap-port mapper produces **4 separate models, sizes
  `[62, 60, 49, 25]`**, against COLMAP's single 80-frame model spanning
  frames 0-122 (continuously 0-59, then 63-69, 69-90, 119-122 — COLMAP
  itself leaves 60-62, 70-81 and 91-118 unregistered, so even COLMAP
  doesn't reach every frame, it just keeps growing *one* model through the
  gaps instead of abandoning it and starting over). So the gap isn't
  "our mapper can't register frame X" in isolation — several of COLMAP's
  registered frames do get registered by our port, just split across four
  separate reconstructions instead of accumulating into one. That points
  at the mapper's stall/restart behavior: once `find_next_images`'s
  candidate pool empties (every remaining unregistered image has either
  `< abs_pose_min_num_inliers` visible points or has exhausted
  `max_reg_trials`), `reconstruct_sub_model` ends and `run()` starts a
  fresh model from a new seed pair rather than ever revisiting the
  abandoned frontier. Whether COLMAP's own mapper hits the same stall
  condition but recovers (e.g. a different filtering/BA schedule leaves
  more points visible per candidate, so fewer images exhaust their trial
  budget before clearing 30 inliers) is the open question — it needs
  frame-by-frame visibility-count instrumentation on both sides, which
  wasn't done this round.

### GPU SIFT subpixel localization (2026-09-28)

Given A1/A2 point at feature quality on V2_01/V2_03, the GPU SIFT extrema
detector was audited against COLMAP's. Neither our CPU "legacy" DoG detector
(`detect_extremum` in `crates/vision/src/features/sift.rs`) nor its GPU port
(`crates/sift-gpu`) do any subpixel refinement — extrema stay on the
integer pixel/octave grid. COLMAP's SiftGPU (and VLFeat, which COLMAP's CPU
SIFT is based on) refine each extremum to subpixel (x, y, scale) precision
with a quadratic Newton fit, iterating into a neighbouring sample when the
fitted offset exceeds half a pixel. This is a real, previously-unexamined
gap, not on the earlier negative-lever list.

Implemented as an opt-in `--sift-opt subpixel_localization=1` flag (default
off, verified bit-identical to the existing GPU SIFT output when off) and
tried three variants on V2_01/V1_03/V2_03, all reverted after the results
below (the flag no longer exists in the tree):

| Variant | V2_01 ATE (reg.) | V1_03 ATE (reg.) | V2_03 ATE (reg.) |
| --- | ---: | ---: | ---: |
| Baseline (A3, no refinement) | 3.30 cm (199) | 2.17 cm (67) | 3.37 cm (118) |
| 2D (x,y) single-shot, clamp offset to ±0.6 | 1.53 cm (197) | 2.99 cm (99) | 16.56 cm (118) |
| 2D (x,y) single-shot, reject if offset > 0.5 | 3.91 cm (198) | **1.19 cm (62)** | 12.78 cm (118) |
| Full 3D (x,y,scale) iterative, VLFeat/Lowe-style (5 iters, neighbour-shift, refined contrast/edge retest) | 2.76 cm (197) | 2.89 cm (86) | 11.91 cm (119) |
| Full 3D + refine only extrema with `|v| > 2×contrast` | 2.75 cm (197) | 2.89 cm (86) | 11.91 cm (119) |

- **V2_01 improves under every variant** (3.30 → 1.53–3.91 cm; best with the
  permissive clamp policy), consistent with A1's finding that V2_01's gap is
  feature-localization quality — but no variant reaches COLMAP's 1.00 cm.
- **V2_03 regresses badly under every variant** (3.37 → 11.9–16.6 cm), even
  the theoretically-correct full iterative version with edge/contrast
  retests at the converged location, and even after gating refinement to
  only comfortably-above-threshold extrema (which changed nothing —
  ruling out marginal-contrast points as the cause). Four independent
  implementations landing in the same place is strong evidence this is a
  real property of V2_03's blur interacting with subpixel correction, not
  an implementation bug: refining keypoint positions on already-ambiguous,
  motion-blurred DoG surfaces moves points in ways that hurt the mapper's
  BA more than the integer-grid position did.
- **V1_03 is mixed**: worse under the clamp and iterative-3D variants,
  clearly better under the plain reject policy (2.17 → 1.19 cm, though at
  fewer registered frames, 67 → 62).
- No variant flips any of the three losing sequences into a win, and the
  best V2_01 result still trades off a large V2_03 regression under the
  same flag — the gate requires one fixed config across all 8 sequences, so
  none of these are adoptable as-is. Honest negative; recorded here, code
  reverted.

The remaining gap on V2_01 and V2_03 is confirmed to be GPU SIFT feature
quality (not the mapper or matcher), but subpixel localization alone isn't
the fix — something about it interacts badly with blur. V1_03 is a
separate, mapper-side gap.

### V1_03's 4-model split: not a mapper-logic bug (2026-09-28/29)

Followed up the previous round's open question — does COLMAP hit the same
stall on A1 (COLMAP's own features+matches) but recover? — with instrumented
event diffing on both sides (`VISLOC_DEBUG_BOUNDARY_FRAMES`/
`VISLOC_DEBUG_REG_FAIL` env-gated logging added to `pipeline.rs`/`mapper.rs`;
COLMAP re-run with its own verbose `LOG(INFO)` registration trace against the
identical A1 database).

- **The frontier stall is real but symmetric.** After registering frames
  0-59, both the port and COLMAP's own mapper fail to register frames 60/61/62
  (visible 3D points 21/12/0, all under `abs_pose_min_num_inliers=30`) —
  COLMAP's log shows the *same* three frames permanently unregistered in its
  final model. COLMAP's own sequential matcher adds long-range "skip" pairs
  (e.g. `58↔122`, 35 raw matches — quadratic-overlap bridge pairs, not just
  the window-10 neighbours), and COLMAP's mapper log shows it registers
  frame 122 next (not 60/61/62), then 121/120/119, then 82-90, then 63-69 —
  bridging the gap sideways through a different frame cluster it had not
  touched yet, all *inside the same reconstruction*.
- **The port's `find_next_images` finds the identical bridge candidate.**
  At the exact point the port's frontier stalls, image 122 is ranked the
  #1 (and only) candidate, with 31 visible points — matching COLMAP's own
  reported 32 at the equivalent point almost exactly. So `find_next_images`,
  the multi-model retry loop in `pipeline.rs`, and the visibility-propagation
  machinery in `observation_manager.rs` are correct, faithful ports; nothing
  here "abandons the frontier" via a control-flow bug. The stall triggers
  `reconstruct_sub_model`'s ordinary 2-consecutive-failure exit (also a
  faithful port of `incremental_pipeline.cc:628`), and `run()` correctly
  starts a fresh model from a new seed, same as COLMAP would if a *different*
  candidate had failed there.
- **The actual gap is in `RegisterNextImage`'s RANSAC, on that one marginal
  candidate.** `VISLOC_DEBUG_REG_FAIL` shows the port's P3P RANSAC finds only
  20/31 geometric inliers for image 122 (needs 30) where COLMAP succeeds.
  COLMAP's `EstimateAbsolutePose` uses `LORANSAC<P3PEstimator, EPnPEstimator>`
  (`optim/loransac.h`): once a P3P sample beats the running best, it
  recursively re-fits on the *growing* inlier set (up to 10 rounds) before
  scoring — our port did a single post-hoc refine only. Implemented COLMAP's
  loop in `crates/vision/src/ransac/mod.rs`'s `PnPRansac::search_best_pose`.
  First attempt reused the project's `DltPnP` as the non-minimal local
  refit step (COLMAP uses EPnP); this was a **real regression**, not
  neutral: `DltPnP`'s own module doc already warns it is "degenerate on
  coplanar points", and refitting on a locally-planar inlier window
  corrupted poses badly enough that Sim(3) ATE alignment failed outright
  (`ATE n/a`) on every EuRoC sequence tried. Re-tried using the existing
  `GaussNewtonPoseRefiner` (nonlinear, seeded from the current best pose)
  for the growing-inlier-set refit instead — numerically safe, all 22
  `ransac` unit tests still pass — but it **did not change V1_03's model
  split** (`[62, 60, 49, 25]` before and after): image 122 still tops out
  around 20/31 inliers even with iterative local optimization. It also
  measurably *hurt* accuracy on the untouched 62-frame block that both
  versions register identically (ATE 2.96 → 7.22 cm, same exact frame set),
  so **the LO-RANSAC change was reverted** (`ransac/mod.rs` is back to the
  original single-shot-refine behaviour; confirmed byte-identical output
  after revert: `[62, 60, 49, 25]`, ATE 2.96 cm, 2118 points).
- **Conclusion:** the ~35% "outlier" rate on image 122's correspondence set
  is not a RANSAC-robustness/marginal-recovery gap (COLMAP's own more
  powerful local optimizer doesn't close it either, going by our port's
  attempt at replicating it) — it points at the *correspondence set itself*
  (triangulated 3D point accuracy for the 0-59 block, or the specific 2D
  features on frame 122) being measurably worse than COLMAP's, i.e. the same
  family as the already-documented SIFT/triangulation-quality gap on
  blurred/transitional frames, not a mapper defect. Not closed this round;
  next steps for whoever picks this up: dump per-correspondence reprojection
  residuals for image 122 (is it a clean bimodal 20-good/11-bad split, or a
  smeared distribution suggesting systematic BA drift on the 0-59 block?),
  and compare the Sim(3)-aligned 3D positions of those tracks against
  COLMAP's own triangulation for the same points.
- **Debug logging kept** (env-gated, off by default, harmless):
  `VISLOC_DEBUG_REG_FAIL=1` prints the correspondence/inlier counts and
  reason for every failed `RegisterNextImage` attempt in
  `colmap_incremental/mapper.rs`.

## Reproduce

```text
cargo build --release -p visloc-gsplat-train --features gpu,euroc --example gsplat_euroc
python scripts/benchmark_euroc_vs_colmap.py V1_02_medium \
  --euroc-root <EuRoC dir with <seq>/mav0> --colmap <colmap.exe> \
  --ours-exe target/release/examples/gsplat_euroc --out-root <out> --tag _final \
  --ours-args "--mapper colmap-port --sift-l1-root --keep-planar --verify-min-inliers 15 --sift-opt descriptor_magnification=3 --sift-opt max_orientations=2 --keypoints 4000 --sift-opt prefer_larger_scale=1 --register-gated"
```
