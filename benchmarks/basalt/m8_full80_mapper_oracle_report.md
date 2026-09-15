# M8 full-80 mapper lifecycle oracle comparison

Date: 2026-08-24 (JST)  
Status: lifecycle execution and evidence capture complete; faithful parity is **not** met.

This report compares the Rust `basalt_offline_mapper` with a diagnostic-only
copy of the pinned upstream mapper. It is an oracle audit, not a production
parity claim. No upstream checkout was modified and no GT, result-path, or
trajectory alignment was used.

## Scope and provenance

- Upstream commit: `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.
- The Rust side replays the same 80-record MargData fixture
  (`frame_000000.json` through `frame_000079.json`): five mapper packets and
  75 state-only marginalization records. The upstream side replays the
  corresponding five-packet cereal cache plus `gt.cereal` and 12 images.
  The image/feature/track stream is structurally aligned, but the serialized
  frame poses are not numerically identical. This distinction is material for
  triangulation and is isolated below with a diagnostic native-pose injection;
  the raw comparison must not be presented as a same-numeric-input claim.
  The copied reproducibility input is in
  `target/m8_full80_upstream_input_20260824/`.
- Diagnostic source copies are
  `target/m8_upstream_mapper_lifecycle_oracle.cpp` and
  `target/m8_upstream_keypoints_seeded.cpp`. Their SHA-256 values are recorded
  in `target/m8_full80_parity_manifest_20260824.json`.
- Config and calibration hashes match between the Rust and upstream paths:
  `euroc_config.json` =
  `82937bd6493e592ef89572d31260c10f7437b4fb3ff1fda179375713966e34fa`;
  `euroc_ds_calib.json` =
  `ad8c5a18c48c55dacf61d18ebbc18cd7d4f3acccd5840bd5a5cd8646adb6271c`.

The upstream executable is a diagnostic link only. The original ninja link
line placed the diagnostic keypoints object after the OpenCV static archives,
which caused undefined OpenCV references. The verified fix places
`/tmp/m8_keypoints.o` immediately after `/tmp/m8_mapper.o`, before the static
archives, and runs from `build/core-relwithdebinfo`. The exact recipe and
hashes are in `target/m8_full80_upstream_oracle_20260824/`.

## Fixed diagnostic stage comparison

The fixed runs are diagnostic checks only.  The current comparison uses seed
12345 on both sides.  Rust reproduces GCC 11's `std::mt19937` and
`uniform_int_distribution<int>(0, INT_MAX)` stream; a 4,096-draw hash crosses
the 624-value twist boundary, while a forced-extrema probe covers generator
values `0xfffffffe` and `0xffffffff`.

| Stage | Upstream diagnostic | Rust diagnostic | Observation |
|---|---:|---:|---|
| ingest | 12 images, 12 poses, 35 relative + 5 roll/pitch factors | 12 poses, 24 processed images | Lifecycle reaches the same 12 timestamp window. |
| detect | 24 feature rows | 17,558 features over 24 images | Row count is not a feature-count equivalent. |
| stereo match | 3,020 raw / 1,950 inlier | 3,020 raw / 1,950 inlier | This boundary agrees. |
| temporal match | 38,137 raw / 36,921 inlier (cumulative diagnostic: 41,157 / 38,871) | 38,137 raw / 36,921 inlier (cumulative: 41,157 / 38,871) | All 264 pair keys and every ordered raw/inlier feature-ID list agree. |
| tracks | 658 tracks / 7,741 observations | 658 exported / 7,741 observations | Track graph cardinality agrees after using the identical OpenGV stream. |
| setup (raw pose inputs) | 649 landmarks / 7,684 observations | 649 landmarks / 7,694 observations | The 10-observation difference is real for the raw packet/cache inputs, but is not yet a triangulation-backend attribution because the 12 pose maps differ. |
| optimize 1 final cost | 111,226.822198 | 1,392,711.192220 | BA state/cost is not equivalent; setup geometry/observation parity comes first. |
| filter | 617 landmarks / 7,301 observations | 649 / 7,694 (threshold removes 0) | The fixed Rust audit uses the deliberately large outlier threshold; this row is not a filter-threshold equivalence claim. |
| optimize 2 final cost | 34,766.040704 | 1,392,120.344760 | Final state is not equivalent. |

The old report's apparent temporal raw-match discrepancy was a measurement
taxonomy error: upstream's `match_all` diagnostic walks the persistent
`feature_matches` map, which already contains the 3,020 raw stereo matches.
The diagnostic split hook now records `3,020 + 38,137 = 41,157` explicitly,
and Rust's candidate count (264) and temporal raw count (38,137) agree.  The
fresh seed-12345 Rust executable was built at 17:02:27 with SHA-256
`61BE806278974C9913EFD823F88AD8719F4E44BF2930A1C3FFE9716D528C092B`.
The full pair-key comparison reports zero missing pairs, zero ordered raw-list
differences, and zero ordered inlier-list differences across all 264 pairs.
The raw-input setup difference (7,684 vs 7,694) is not evidence of a
triangulation-backend difference until the pose maps are controlled. The old
fixed0 result is retained only as a historical control and is not used as
parity evidence.

Aligning tracks by their complete `(timestamp, camera, feature-id)`
observation sets gives all 658 tracks with zero missing sets and zero size
differences.  Sixteen triangulation acceptance flags differ: eight Rust-only
accepted tracks contain 52 observations and eight upstream-only accepted
tracks contain 42, exactly accounting for Rust's 10-observation excess while
leaving both sides at 649 landmarks.  The machine-readable mapping is
`target/m8_fixed12345_setup_track_compare_20260824.json`.

## Controlled native-pose boundary

The 12 setup poses differ materially before triangulation (raw-input maximum
translation difference about 0.1104 m; after anchoring frame zero, relative
translation difference about 0.0584 m and relative rotation difference about
0.7615 degrees). A diagnostic-only override injected the native
`setup_candidates` pose map by timestamp, resolving it to Rust's compact frame
IDs before feature detection and setup. This is a control experiment, not a
production behavior change.

Under that override, all 658 track observation sets remain aligned, accepted
flags are identical, and both sides produce 649 landmarks / 7,684
observations. Rust's setup reprojection error is `409345.7561790617`, versus
native setup vision error `409345.7561790651`. The earlier 16 accepted-track
swaps therefore came from the pose-input frontier; the controlled result does
not support replacing nalgebra's SVD with an Eigen JacobiSVD at this boundary.
Evidence is retained in
`target/m8_full80_rust_nativepose_fixed12345_candidates_20260824.json` and
`target/m8_full80_rust_nativepose_fixed12345_20260824/`.

The next control injected the native relative-pose and roll/pitch payloads as
well. The diagnostic dump contains 35 relative and 5 roll/pitch factors; after
timestamp-to-frame-ID alignment, every translation, quaternion, information
matrix element, and weight agrees (maximum absolute difference zero). With
both native poses and native factor payloads, Rust's initial BA cost is
`449528.1978063228`, versus the native setup total
`449528.1978063258`; this closes the factor-assembly boundary. The controlled
artifacts are `/tmp/m8_oracle_runs/ordered_setup_factors_fixed12345.jsonl`,
`/tmp/m8_rust_nativefactor_setup.json`, and
`/tmp/m8_rust_nativefactor_20260824/stage_report.json`. Their SHA-256
provenance is recorded in the parity manifest, together with the factor
diagnostic binary hash.

Using the source-equivalent outlier threshold `3.0` and ten requested
iterations (the upstream diagnostic's default), the corrected Rust optimize-1
final cost is `111226.8221981741` (native `111226.8221981867`). Both sides
then retain 617 landmarks / 7,301 observations, and optimize-2 ends at
`34766.0407042765` versus native `34766.0407042901`. The residual differences
are below `1.4e-8` in these totals. The earlier 1.49%/0.95% gap was a Rust LM
loop-order bug: Rust skipped a solve when `max_increment < 1e-5`, while the
source sets `converged` and still applies that final trial once. The corrected
source applies the trial and only stops the next outer iteration. The
threshold-3 corrected run is retained in
`/tmp/m8_rust_nativefactor10_fix_20260824/`; its release binary SHA-256 is
`e165d7d68b2141a39f6a7eb1231cfb38e8260fe027893b322f0098c06403f405`.

## Raw MargData packet/frame/state provenance boundary

The raw pose frontier is now isolated before feature detection and
triangulation. Diagnostic-only `before_add`/`after_add` snapshots record the
native cereal packet and Rust JSON packet, the frame/state tables, IEEE-754
bits, AOM shape/key sets, and the persistent mapper pose map. The complete
comparison is in
`target/m8_full80_pose_provenance_compare_20260824.json`; its packet captures
are `target/m8_full80_upstream_oracle_20260824/ordered_packet_poses_fixed12345.jsonl`
and `target/m8_full80_rust_packet_poses_fixed12345.jsonl`.

The two streams both contain five mapper packets. Packet index 0 has the same
key (`1403636580463555584`), frame timestamp sets, keyframe set, and AOM
dimensions, but the first numeric mismatch is already the `frame_poses` pose
at timestamp `1403636579763555584`. Values are serialized and compared as
`[tx, ty, tz, qw, qx, qy, qz]` bits:

```text
native = [1.432138740931066e-09, 5.275158088124954e-10,
          -3.191414021941341e-09, 0.5872793162936838,
          -0.050467833578458424, -0.8078013610800668,
          -0.0036004811778298684]
rust   = [-3.775514123560697e-09, 2.409871746600345e-10,
          3.5727507063931796e-10, 0.5833827955183675,
          -0.05227491934446851, -0.8105121164546499,
          -0.0013984923696407651]
```

All compared source pose/state bit records differ (`0/41` pose records and
`0/21` state records, with the union count reflecting later window-set
differences). The first packet-key/event mismatch is index 1: native
`1403636581163555584` versus Rust `1403636581863555584`; later
keyframe/marginalization sets also differ. After ingestion the timestamp key
sets still contain the same 12 poses, but the raw mapper maps have anchored
relative maxima of `0.6546206155 m` and `1.7395954595 deg`. This is a source
state/window frontier, not a quaternion-order or timestamp remapping issue;
these are pre-setup `after_add` values and must not be conflated with the
later setup-candidate control metric (`0.05836 m`, `0.76149 deg`).

The source provenance is explicit: Rust `emit_pre_marg` copies solved
`window_poses`/`window_states` in
`pipelines/basalt/src/vio/estimator.rs:1759-1935`, writes them through the
`serde_json` contract in `pipelines/basalt/src/vio/margdata.rs:366-377`, and
the CLI reads them at `examples/basalt_offline_mapper.rs:114-159`. The pinned
upstream `MargData` layout and cereal order are
`include/basalt/utils/imu_types.h:333-346,368-387`; its mapper copies
`getPose()` into the persistent map at
`src/vi_estimator/nfr_mapper.cpp:60-80`. No M7 source was changed for this
audit. The existing `MargData::roundtrip_hash` and streaming-writer tests
also verify exact Rust JSON value preservation; the observed mismatch is
therefore in the emitted M7 state/window payload, not a lossy JSON round trip.

The manifest now contains `m8_margdata_input_pose_bit_gate`: an uncontrolled
parity claim requires exact packet/event structure and exact pose/state bits.
The controlled native-pose+factor regression already passes through BA: the
initial, optimize-1, and optimize-2 cost absolute differences are
`3.0e-9`, `1.26e-8`, and `1.36e-8`, with filter landmark/observation counts
equal. Thus, when the gate inputs match, the mapper lifecycle is numerically
parity-exact; the remaining raw failure is inherited M7 VIO state and
keyframe scheduling.

The recorded gate can be rechecked without rerunning the 959 MB mapper input:

```text
python benchmarks/basalt/check_m8_pose_provenance.py
```

To regenerate the Rust packet snapshot, set
`BASALT_OFFLINE_MAPPER_DIAGNOSTIC_PACKET_POSES` to the recorded JSONL path
while running the existing fixed-seed mapper command (the diagnostic uses
`--num-opt-iter 0`, so it stops after ingest/frontend bookkeeping for this
boundary). The native counterpart is the packet diagnostic binary recorded
in the manifest with `--diagnostic-output` and
`--diagnostic-fixed-seed`; both commands preserve the same five-packet input
order and do not alter production state.

## Production repetitions (historical baseline)

Production runs keep each implementation's default time-derived seed. The
upstream production repetitions were run under WSL with `/usr/bin/time -v`;
repetitions 2 and 3 were concurrent, so their wall times are diagnostic only.
Rust wall time and process-tree RSS were measured on Windows. These resources
are therefore not a cross-platform performance gate.

| Implementation/run | temporal raw/inlier | tracks / observations | setup landmarks / observations | opt1 final cost | opt2 final cost | wall / peak RSS |
|---|---:|---:|---:|---:|---:|---:|
| upstream prod1 | 38,137 / 36,901 (cumulative 41,157 / 38,851) | 650 / 7,652 | 641 / 7,599 | 101,942.494831 | 34,320.156597 | 1.26 s / 151,544 kB |
| upstream prod2 | 38,137 / 36,877 (cumulative 41,157 / 38,827) | 651 / 7,652 | 643 / 7,602 | 98,864.288808 | 34,324.218067 | 4.81 s / 147,208 kB |
| upstream prod3 | 38,137 / 36,900 (cumulative 41,157 / 38,850) | 648 / 7,685 | 640 / 7,633 | 96,192.493091 | 34,892.948112 | 5.24 s / 154,104 kB |
| Rust prod1 | 38,137 / 36,915 (cumulative 41,157 / 38,865) | 654 / 7,719 | 645 / 7,672 | 1,435,488.917960 | 1,424,157.502853 | 24.1153 s / 41,766,912 B |
| Rust prod2 | 38,137 / 36,890 (cumulative 41,157 / 38,840) | 655 / 7,671 | 646 / 7,624 | 1,379,802.310717 | 1,379,558.136565 | 21.3735 s / 56,041,472 B |
| Rust prod3 | 38,137 / 36,885 (cumulative 41,157 / 38,835) | 640 / 7,577 | 631 / 7,530 | 1,345,776.138998 | 1,329,725.318246 | 21.3765 s / 49,418,240 B |

The production result directories are
`target/m8_full80_rust_prod1_20260824/`,
`target/m8_full80_rust_prod2_20260824/`, and
`target/m8_full80_rust_prod3_20260824/`. The upstream JSONL/time captures
are in `target/m8_full80_upstream_oracle_20260824/`.

The table above predates the LM final-trial correction and used the old Rust
audit settings (`--num-opt-iter 3 --outlier-threshold 1000000000`), so it is
retained as a historical baseline only. Corrected WSL repetitions use the
source-equivalent ten iterations and outlier threshold 3.0:

| Corrected run | temporal raw/inlier | tracks / observations | setup landmarks / observations | opt1 final cost | filter landmarks / observations | opt2 final cost | wall / peak RSS |
|---|---:|---:|---:|---:|---:|---:|---:|
| Rust prodfix1 | 38,137 / 36,920 | 648 / 7,643 | 639 / 7,596 | 1,341,622.143918 | 453 / 5,261 | 501,219.005445 | 43.84 s / 76,868 kB |
| Rust prodfix2 | 38,137 / 36,902 | 666 / 7,831 | 657 / 7,784 | 1,368,606.019852 | 489 / 5,872 | 727,093.974593 | 25.85 s / 76,692 kB |
| Rust prodfix3 | 38,137 / 36,870 | 652 / 7,681 | 641 / 7,622 | 1,364,974.401634 | 454 / 5,282 | 502,879.669536 | 22.83 s / 76,900 kB |

These corrected raw runs remain intentionally non-parity because the Rust
MargData pose map is not the native cereal pose map. Their artifacts are
`target/m8_full80_rust_prodfix1_20260824/` through
`target/m8_full80_rust_prodfix3_20260824/`; the executable hash is recorded in
the manifest.

## Reproduction

Rust production command (repeat with a distinct `--out-dir` for each run):

```text
target/release/examples/basalt_offline_mapper.exe \
  --marg-data target/basalt_m6_boundary_run80/marg_data \
  --calibration target/euroc_ds_calib.json \
  --config configs/basalt/euroc_config.json \
  --out-dir target/m8_full80_rust_prodN_20260824 \
  --num-opt-iter 10 --outlier-threshold 3
```

Upstream diagnostic command (WSL; add `--diagnostic-fixed-seed` only for the
fixed diagnostic run):

```text
LD_LIBRARY_PATH=/root/visloc-basalt-oracle-0f3b2b52/build/core-relwithdebinfo \
/tmp/m8_lifecycle --show-gui 0 \
  --cam-calib /root/visloc-basalt-oracle-0f3b2b52/data/euroc_ds_calib.json \
  --marg-data /tmp/m8_upstream_m6_input \
  --config-path /root/visloc-basalt-oracle-0f3b2b52/data/euroc_config.json \
  --diagnostic-output /tmp/m8_oracle_runs/upstream_prodN.jsonl
```

The lifecycle binary, object files, link recipe, JSONL captures, and SHA-256
records are retained only as diagnostic provenance. The upstream checkout was
not changed; the Rust mapper source includes the source-order LM final-trial
correction described above and must be rebuilt before treating the older
production repetition table as current.
