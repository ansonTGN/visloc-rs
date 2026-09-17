# OpenLORIS map-based relocalization using the native COLMAP-port map

Goal: relocalize a single query image — no temporal or odometry prior — against
a map **built by our own mapper** (`examples/colmap_incremental_mapper`,
the COLMAP 64805cb port), using the existing
`visloc-localization` pipeline. This connects the mapper-parity work to the
localization stack; the existing demos only reuse external COLMAP models
(South Building) or 7-Scenes.

## What already exists (reused, not rebuilt)

- `pipelines/localization`: `VisualMap`, `LandmarkDescriptorStore`,
  `LocalizationPipeline` / `localize_frame_with_descriptor_store`,
  `BruteForceMatcher` (L2), `PnPRansac`.
- `crates/io`: `read_colmap_text_model`, `read_landmark_descriptors_txt`
  (`<landmark_id> <d...>`), `read_query_features_txt` (`x y <d...>`).
- Mapper output: `colmap-port-c2-v1/tier-*/.../model/model/0/{cameras,images,
  points3D}.txt` (COLMAP text).
- Inputs: `openloris-tier*-rig-manifest-v1.txt`; per-image keypoints from
  `colmap-prefix-parity-v1/tier-*/export/features/*_features.txt`; SIFT
  descriptors in the COLMAP DB (`descriptors` table, 128×uint8).

## Gap

The mapper export contains keypoints only, and no landmark descriptors, so the
localizer has nothing to match a query against. Also the mapper model uses its
own image ids, not the COLMAP DB ids.

## Design

1. **Descriptor export** (`scripts/export_openloris_localization_map.py`):
   - Map `landmark_id -> one representative map observation (model image_id,
     point2d_idx)`, then `model image_id -> name -> flat -> DB image_id` (via
     `image_aliases.tsv`) and read that descriptor from the DB blob.
   - Emit `landmark_descriptors.txt` (`<landmark_id> <128 floats>`), using
     **map observations only** (no held-out leakage).
   - Emit query feature files (`x y <128 floats>`) for the held-out frames from
     the DB descriptors + feature keypoints.
   - SIFT descriptors are uint8; the matcher is L2, so cast to f32 directly.
2. **Rust example** `examples/localize_openloris_map.rs`: load map +
   landmark descriptors + a query feature file + camera id, run
   `localize_frame_with_descriptor_store`, print pose, inlier count and
   reprojection error.
3. **Evaluation** (`scripts/eval_openloris_map_relocalization.py`): for each
   query, run the example, Sim(3)-align/compare its pose to the official GT
   (post-hoc only), report translation/rotation error, success rate and
   inlier stats.

## Experiments

- **E1 self-location sanity**: localize map images against the map (descriptor
  round-trip check); expect near-zero pose error for most.
- **E2 held-out (frame parity)**: map = frames 0..2499 restricted to *even*
  frames (drop odd images/observations from the model, keep landmarks with a
  remaining observation), queries = *odd* frames. Approximate (landmark
  positions still come from the full model); a fully clean variant would run
  the mapper on an even-frames-only manifest.
- **E3 (optional)**: rebuild the map with the mapper on even frames only and
  repeat E2; and/or query with a different sequence subset.

GT is used only by the post-map evaluation (E1/E2 scoring), never for
retrieval, matching or pose.

## Non-goals

- Not rebuilding the 7-Scenes benchmark; not changing retrieval (global
  descriptor) here — this is map reuse + single-image PnP relocalization.
- No README changes until results are reproducible and gated.
