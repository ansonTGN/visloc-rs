# Mapper memory: compact background-optimizer snapshot (2026-09-24)

## Problem

`OnlineNfrMapper::spawn_background_optimize` cloned the whole `NfrMapper` for
every periodic optimize. On MH_01 at kf5, the persistent mapper payload is at
least 128 MB (see `vi_slam_mapper_image_dedup_20260924.md`), so each optimize
roughly doubled it at peak.

The background pipeline is `build_tracks` → `setup_opt` → `optimize` →
`filter_outliers` → `optimize`. It reads only the following fields; the full
field-usage audit is in `session.rs`:

* the match graph (`feature_matches`)
* feature **pixel corners**, via `setup_opt` → triangulation `.corners`
* poses and factors
* calibration
* optimizer configuration and state

`build_tracks` always replaces `feature_tracks`. `setup_opt` always replaces
`lmdb` when calibration is present.

## Change (default)

`NfrMapper::optimizer_snapshot()` copies only those inputs. It keeps corners,
but not angles, descriptors, rays, hashes or BoW. It leaves out:

* `img_data`
* `hash_index`
* `feature_match_data`
* the old tracks and landmarks

Without calibration, `setup_opt` would keep the old `lmdb`, so in that case
the function falls back to a full clone. `spawn_background_optimize` now
uses the snapshot. The merge back into the live mapper is unchanged.

The test `mapper_online::tests::optimizer_snapshot_matches_full_clone_pipeline`
uses a synthetic state: 60 points projected into 6 translated frames, with
pixel noise and chained matches. It yields more than 30 landmarks, and BA
moves the poses. Running the full pipeline on the full clone and on the
snapshot gives identical poses, tracks, landmark database and optimizer
state. All 435 `visloc-basalt` tests pass.

## Measurement: MH_01, one run each, sequential

Output is in `/mnt/win/linux_data/visloc_compact_snapshot_probe_20260924`.

| | kf5 before | kf5 after | urgent before | urgent after |
|---|---|---|---|---|
| peak RSS (KiB) | 745672 | **617948 (−17%)** | 885164 | **829252 (−6%)** |
| SE3 ATE | 0.01703 | 0.01731 | 0.01515 | 0.01532 |

The "before" columns are the image-dedup binary. The original kf5 value, before
both changes, was 718196 KiB, so the net kf5 reduction is −14%.

Under urgent keyframes the remaining peak is dominated by the MargData queue
backlog, so the saving is smaller there. ATE differences are within
asynchronous run-to-run variation, and the optimizer results are identical by
test. Wall times were measured under a load average of about 10 to 14 and are
not compared.
