# Mapper speed: batched observation-index rebuild in filter_outliers (2026-09-24)

## Problem

In the MH_01 kf5 online timing breakdown, `optimize_filter_seconds` was
101.1 s over 4 background optimizes. The LM itself took 76.0 s. Each
`filter_outliers` call removes every outlier landmark or observation through
`NfrMapperLandmarkDb::remove_landmark` or `remove_observations`, and each of
those rebuilt the entire host/target observation index. On MH_01 that index
holds about 1.4 M observations, so the pass was quadratic.

## Change

`filter_outliers` in `mapper/session.rs` keeps the same ordered removal loop
but calls the raw removals. It then rebuilds the index once, and only if
anything was removed.

The final state is unchanged because:

* the index is a pure function of `landmarks` (`from_landmarks`);
* the raw removals read and write only `landmarks`;
* nothing reads the index during the loop.

The public per-removal methods are unchanged. The previous loop is kept as a
test-only `filter_outliers_per_removal_reference`.

The test `mapper_online::tests::filter_outliers_batched_rebuild_matches_per_removal_reference`
corrupts pixels in the synthetic BA fixture so that the pass removes both
whole landmarks and single observations. The resulting landmark database,
including the index, is identical to the reference. All 436 `visloc-basalt`
tests pass.

## Measurement: MH_01 kf5, one run

Output is in `/mnt/win/linux_data/visloc_filter_batch_probe_20260924`.

| | before (compact-snapshot run) | after |
|---|---|---|
| optimize_filter_seconds (4 calls) | 101.1 | **0.7** |
| optimize_total_seconds | 179.9 | **80.6** |
| optimize_max_seconds | 97.4 | 34.1 |
| total_wall − vio_wall (mapper tail) | 126.3 s | **44.2 s** |
| final landmarks | 14525 | 14527 |
| SE3 ATE | 0.01731 | 0.01662 |

VIO wall time is not comparable: 830 s before against 1071 s after, because a
concurrent build contended for CPU during the second run. ATE and landmark
differences are within asynchronous run-to-run variation, and the filter
itself is identical by test.
