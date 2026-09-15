# Clone-free MargData queue writer (2026-08-29)

## Scope

This records two behavior-preserving performance changes for the Rust VIO
path. The live all11x3 coordinator (PID 6232) and its two Rust workers were
not interrupted or rerun.

The existing retained-output profile identifies the packet path as a material
cost: the 80-frame run writes five MargData packets totaling about 89.7 MiB,
and the 800-frame schema-4 run writes 107 packets totaling about 1.68 GB.
`examples/basalt_euroc_vio_demo.rs` previously cloned every complete
`MargData` (including images and dense matrices) in `to_mapper_packet()` before
streaming it. The clone was unnecessary because the queue view only removes
the diagnostic `prior` field and changes `provenance_version`.

## Changes

- `pipelines/basalt/src/vio/margdata.rs:354-381` adds a borrowed
  `MapperPacketRef` with the same serde field order and skip rules.
- `pipelines/basalt/src/vio/margdata.rs:919-951` adds
  `MargData::write_mapper_packet_json`, which serializes that view directly
  and is a no-op for state-only records.
- `pipelines/basalt/src/vio/margdata.rs:459` centralizes the existing queue
  provenance string. `to_mapper_packet()` remains available and unchanged in
  its owned/API behavior.
- `examples/basalt_euroc_vio_demo.rs:98-108` now tests the predicate and
  writes the borrowed view, avoiding the packet clone on the hot path.
- `pipelines/basalt/src/vio/estimator.rs` adds an explicit
  `process_without_marg_data` mode. It keeps the solve, landmark writeback,
  square-root prior update, and window lifecycle, but skips only diagnostic
  MargData snapshots and the unused projected-row cache.
- `pipelines/basalt/src/adapter.rs` avoids raw u16
  image copies in that explicit mode. The demo selects it only when both
  `--no-marg-data` and `--no-trace` are present.
- `pipelines/basalt/src/vio/window.rs` adds `solve_without_factors`, avoiding
  the post-solve factor relinearization when the caller cannot consume the
  final factor snapshot. `WindowProblem::solve` also reuses LM's already
  accepted-state cost instead of rebuilding the complete factor stack solely
  to recompute it. The estimator now clones the complete factor-bearing
  `WindowProblem` only on frames whose solved window actually marginalizes;
  ordinary optimized frames snapshot only the pose/state vectors needed by
  the unchanged keyframe policy.
- After reanchoring and deferred landmark removal,
  `pipelines/basalt/src/vio/estimator.rs` prunes the world-point bridge cache
  to active landmark IDs. This removes one `Point3` per historical track from
  long-running estimators without changing any active factor or reanchor
  input.
- `pipelines/basalt/src/vio/landmarks.rs` adds upstream-shaped frame-history
  cleanup for `ObservationDb.observations`; the estimator removes entries for
  dropped full states and pose-only keyframes after the window shift, while
  observations of converted state→pose blocks remain available. The method
  shrinks the old vector high-water allocation only when it exceeds twice the
  live length, avoiding reallocations on small shifts.

No numeric operation, factor order, marginalization operation, or retained
output contract was changed. The normal estimator/adapter APIs still retain
their historical snapshots. The no-output mode returns a non-wire diagnostic
placeholder (`basalt-disabled`) and is selected only when both output streams
are explicitly disabled.

## Compute-side evidence

The existing source/profile audit identifies factor construction and repeated
LM linearization as the compute-side cost once packet I/O is disabled. Before
this change, every `WindowProblem::solve` performed a final factor
relinearization and a second full `cost()` relinearization after LM had
already returned the accepted-state cost. The no-output caller also discarded
the final factor vector, while still paying to build it. The new option removes
only those unused passes; all LM trial linearizations and accepted-state
writeback remain in place. The conditional problem snapshot removes another
full factor/row copy from ordinary frames. A static retention audit found that
`window_observations` and `track_history` follow the active window and that
`of_images` is not populated by the no-output adapter. `ObservationDb::observations`
now removes dropped-frame history after each window shift; it still retains
active-track bearings by design. A formal no-output timing run is intentionally
deferred until the shared phase-6 workers finish.

## Focused evidence

- `cargo fmt --all -- --check`: pass.
- `cargo test -p visloc-basalt --lib vio::margdata::tests -- --nocapture`:
  18 passed, 0 failed.
- `cargo test -p visloc-basalt --lib vio::margdata::tests::mapper_packet_streaming_writer_matches_owned_packet_bytes -- --nocapture`:
  1 passed, 0 failed. This compares every emitted byte, including a schema-4
  record with non-empty FEJ sidecars and a diagnostic prior, against the old
  owned `to_mapper_packet().to_json()` result.
- `cargo test -p visloc-basalt --lib vio::estimator::tests::no_marg_data_mode_preserves_solver_and_window_outputs -- --nocapture`:
  1 passed, 0 failed. Retained and no-output estimators agree on solver/window
  state and phase outputs over the synthetic multi-frame fixture.
- `cargo test -p visloc-basalt --lib vio::estimator::tests::synthetic_window_has_cross_blocks_and_shifts -- --nocapture`:
  1 passed, 0 failed.
- `cargo test -p visloc-basalt --lib vio::estimator::tests::world_cache_drops_tracks_removed_from_landmark_database -- --nocapture`:
  1 passed, 0 failed.
- `cargo test -p visloc-basalt --lib vio::landmarks::tests::frame_removal_prunes_only_dropped_observation_history -- --nocapture`:
  1 passed, 0 failed.
- `cargo check --example basalt_euroc_vio_demo`: pass.

The broad `vio::window::tests::` selection still reports two unrelated golden
fixture failures (`upstream_f32_sqrt_marginalization_matches_eigen_golden_boundaries`
and `upstream_f32_real_m6_packet_slice_matches_default_eigen`); the focused
solver/no-output tests above pass and neither failure exercises the new
dispatch.

The existing release executable was not changed in place; a release rebuild is
required before timing this source change. No full benchmark was started while
the all11x3 workers were live.

## Formal follow-up measurement

Build a fresh release executable in an isolated target directory, then compare
both engines with output disabled so wall time is not confounded by unequal
MargData serialization:

```powershell
$env:CARGO_TARGET_DIR = 'C:\Users\rsasa\Workspace\visloc-rs\target\m7im15_clonefree_perf_20260829'
& 'C:\Users\rsasa\.cargo\bin\cargo.exe' build --release --example basalt_euroc_vio_demo
```

Use `work/m7im15_run_clonefree_perf_20260829.py` for the existing Windows
process-tree monitor (it invokes the executable with `--no-marg-data --no-trace`):

```powershell
python work\m7im15_run_clonefree_perf_20260829.py --exe target\m7im15_clonefree_perf_20260829\release\examples\basalt_euroc_vio_demo.exe --dataset E:\datasets\euroc_mav\machine_hall\MH_01_easy --calibration target\euroc_ds_calib.json --config configs\basalt\euroc_config.json --out-dir target\m7im15_clonefree_mh01_80f_20260829 --max-frames 80 --log target\m7im15_clonefree_mh01_80f_20260829.log --result target\m7im15_clonefree_mh01_80f_20260829.json
```

For each 80f smoke and 400f gate, the monitored Rust command is:

```text
target\m7im15_clonefree_perf_20260829\release\examples\basalt_euroc_vio_demo.exe --euroc-dir E:\datasets\euroc_mav\machine_hall\MH_01_easy --calibration target\euroc_ds_calib.json --config configs\basalt\euroc_config.json --out-dir target\m7im15_clonefree_mh01_80f_20260829 --max-frames 80 --no-marg-data --no-trace
target\m7im15_clonefree_perf_20260829\release\examples\basalt_euroc_vio_demo.exe --euroc-dir E:\datasets\euroc_mav\machine_hall\MH_01_easy --calibration target\euroc_ds_calib.json --config configs\basalt\euroc_config.json --out-dir target\m7im15_clonefree_mh01_400f_20260829 --max-frames 400 --no-marg-data --no-trace
```

Use the pinned native command with the same dataset prefix and one worker:

```text
/root/visloc-basalt-oracle-0f3b2b52/build/core-relwithdebinfo/basalt_vio --dataset-path /mnt/e/datasets/euroc_mav/machine_hall/MH_01_easy --cam-calib /root/visloc-basalt-oracle-0f3b2b52/data/euroc_ds_calib.json --dataset-type euroc --config-path /root/visloc-basalt-oracle-0f3b2b52/data/euroc_config.json --show-gui 0 --save-trajectory euroc --max-frames 400 --num-threads 1
```

Record executable SHA-256, wall time, peak process-tree RSS, frame/IMU/
observation counts, and trajectory hashes. A retained-output run may be used
separately to measure the clone-free writer, but it must not replace the
no-output parity gate.

## Live-run exclusion

At the time of this audit, the shared phase-6 coordinator (PID 6232) and its
two Rust workers (PIDs 32512 and 40328) were still running the older release
binary `target/m7im15_q2_boundary_fresh_20260829/release/examples/basalt_euroc_vio_demo.exe`,
SHA-256
`84BDC0211BFFFD43D184806BEBFA6476244AC685731F89460BC0F896B7502081`.
Each worker was about 1.01 GB working set. The current estimator source was
already SHA-256
`BB8876186DAC34415293A326AA3B89C436EEE8E1E0BFFC0033F08A720A4351BE`,
so those workers predate the no-output path and are not a performance oracle
for this change. They were not interrupted or duplicated.

## Provenance

- Source `margdata.rs` SHA-256 after this change:
  `060F18DC1F81AC0A3603E41340647E41C25A4EC7A5A5E007E0E1C3701AAA1274`.
- Source `estimator.rs` SHA-256 after this change:
  `BB8876186DAC34415293A326AA3B89C436EEE8E1E0BFFC0033F08A720A4351BE`.
- Source `landmarks.rs` SHA-256 after this change:
  `CB1A04714E6ACDBD67F4674DEDCD72A0098326D76DD608951198DAC46D9A89EC`.
- Source `window.rs` SHA-256 after this change:
  `B2D8A25CAC8BF0A7F7B1F29461F885EC6335C66C775B4B692ADAF2B526807AE4`.
- Source `adapter.rs` SHA-256 after this change:
  `A4BE8856DC23B78FE183095B9458786D54FFD45C91D0CB8AF1C64D89CC15E98C`.
- Source `basalt_euroc_vio_demo.rs` SHA-256 after this change:
  `8F83EFE10DED07099D84D54D279D0FA841FA52D8F10A4732766629788D8F24FF`.
- Existing retained-output profile:
  `benchmarks/basalt/m7perf_readonly_profile_report.md`.
