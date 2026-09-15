# M7bk Basalt hygiene audit

Audit snapshot: 2026-08-23 02:41 JST. This is a read-only source/report audit;
no build or test command was run. The worktree is shared, so hashes below are
the values observed at this snapshot and should be re-read after sibling edits
settle.

## Findings

### H1 — active M7 tests still print probe bit dumps

`pipelines/basalt/src/vio/aom.rs:2743,2752` print the raw/intermediate pose
bits in `m7_relative_pose_chain_matches_pinned_lanes`. The second probe test
prints more dumps at `aom.rs:2795,2798,2818,2832,2839` in
`m7bf_source_pose_chain_matches_authoritative_lanes`. These tests are inside
the normal `#[cfg(test)]` module but are not `#[ignore]`; an ordinary test run
therefore emits source/probe diagnostics. Remove the prints after the lanes
are pinned, or put the complete dump behind an explicit opt-in diagnostic
environment variable and document it as such.

### H2 — probe-only source helpers are compiled as production code

`pipelines/basalt/src/vio/aom.rs:314-366` defines `F32SourcePose`,
`source_quat_product_f32`, and `source_quat_inverse_f32` outside the test
module. A repository-wide call-site scan found `source_quat_inverse_f32` at
its definition only; `F32SourcePose` and `source_quat_product_f32` are used by
the M7bf test/probe path only. The live visual factor uses `F32Pose` and
`sophus_quat_product_f32` instead. Move this source-order comparator block into
the test module if it is still needed for one final audit, otherwise remove it;
do not leave probe-only helpers in the production compilation unit.

### H3 — an opt-in production trace remains in the estimator

`pipelines/basalt/src/vio/estimator.rs:1033-1128` checks
`VISLOC_BASALT_TRIANG_TRACE` and emits `M7_TRI`/`M7_TRI_BITS` through
`eprintln!` at `:1041` and `:1083`. The environment check prevents default
output, but the temporary M7 dump is still linked into the production path and
can expose full pose/extrinsic/bit payloads when inherited by a replay. Move it
to a dedicated ignored probe or a deliberately enabled diagnostic feature
after the triangulation frontier is closed. `window.rs:303-310,866-1180` is a
separate, explicitly documented `VISLOC_BASALT_DETAIL_TRACE` file trace; its
`:1176` write-error `eprintln!` is intentional and not counted as leakage.

### H4 — temporary/unregistered M7 probe files are outside provenance coverage

`benchmarks/basalt/m7bf_sophus_oracle_tmp.cpp:1` is explicitly named `_tmp`
and is a disposable Sophus source-order probe. `benchmarks/basalt/m7av_frame0_from_two_vectors_probe.cpp:1`
is a new diagnostic source probe. Neither appears in
`benchmarks/basalt/basalt_provenance_manifest_v1.json:229-272`. The current
`python benchmarks/basalt/verify_provenance.py` result is:

```text
FAIL generator/diagnostic coverage mismatch: missing=['benchmarks/basalt/m7av_frame0_from_two_vectors_probe.cpp', 'benchmarks/basalt/m7bf_sophus_oracle_tmp.cpp']
```

Delete the disposable `_tmp` source if its audit is complete, or retain it
under an explicit audit-only path. If `m7av` is retained, add it as a checked-
in generator with dependencies and a hash; do not silently leave either file
outside the manifest.

### H5 — ignored external tests have inconsistent input guards

The following ignored tests can be explicitly invoked without supplying all
external inputs, because they silently fall back to machine-specific/default
paths:

- `pipelines/basalt/tests/m8c_feature_parity.rs:26-28,188-190,544-551` uses
  `E:\datasets\...` and `target\...` defaults instead of requiring
  `VISLOC_BASALT_MH01_ROOT`, `VISLOC_BASALT_CALIBRATION`, and
  `VISLOC_BASALT_CONFIG`.
- `pipelines/basalt/tests/m8c_exact_bits_probe.rs:478-489` is ignored but
  defaults both `M8C_FEATURE_RAW_JSON` and `M8C_FEATURE_ORACLE_JSON`; a direct
  `--ignored` run can read an unintended stale `target` fixture.
- `pipelines/basalt/tests/m8d_setup_opt_oracle.rs:165-177` requires the oracle
  and input variables, but silently defaults calibration/config at `:170-174`.

Require every external path explicitly (or return a clear skip before opening
files). The existing guards are adequate in
`tests/mh01_stereo_golden.rs:184-202,352-365`,
`tests/m7bd_obs_pixel_exact.rs:12-25`, and the ignored source probe at
`src/mapper/features.rs:2325-2329`.

### H6 — source/report provenance is stale or incomplete

The static verifier also reports the following exact mismatches:

- `benchmarks/basalt/basalt_provenance_manifest_v1.json:206-220` records the
  AOM hash as `366545af...e26eb8`; the observed current
  `pipelines/basalt/src/vio/aom.rs` hash is
  `2a475649f36a23fee74e4419d3e48d2c47318874ef9d4af555dafa2e26e855fa`.
- `basalt_provenance_manifest_v1.json:247` records
  `m7aq_visual_factor_boundary_probe.cpp` as `f95837...ba2149`; observed
  current hash is `89acd3e35366f828f669a2f2d6e95c944610693dcb277f4f3871fa39d5f9b402`.
- `basalt_provenance_manifest_v1.json:5,12` still says the tree/hash basis was
  captured on 2026-08-22. The current slice is dated 2026-08-23.

The manifest has no source hash fields for the other changed M7 production
files. At this snapshot their SHA-256 values are:

| source | observed SHA-256 |
| --- | --- |
| `pipelines/basalt/src/camera.rs` | `6aa8fc0a3bd7e3d426001fd8b3d9d987638b3b0d440f75f68b54b7017885e9b9` |
| `pipelines/basalt/src/vio/aom.rs` | `2a475649f36a23fee74e4419d3e48d2c47318874ef9d4af555dafa2e26e855fa` |
| `pipelines/basalt/src/vio/window.rs` | `1da94c99a0813198f57984aa81e73ab5a999196408e103bbeeab7f044485ac67` |
| `pipelines/basalt/src/vio/landmarks.rs` | `22e52d06b555e58f64e56dbb1fa36184b647052309a8a6cbb60eef872a313a0e` |
| `pipelines/basalt/src/pyramid.rs` | `eddbcb22d2a73a07495a9ff9a3c9f581b14683f816fce88d5917dda7e7a4b2e2` |
| `pipelines/basalt/src/patch.rs` | `06adf1a96311f1ae79c0bbf3e412626ae5dd14936f849f01b3df16cc07c9f218` |
| `pipelines/basalt/src/vio/estimator.rs` | `87066e3d730a8d533ffefbbbf8d5dbb3e992e4e9b775da217c6250b57e49185c` |

Specifically, `basalt_provenance_manifest_v1.json:184,197,200,222-227`
classifies camera, patch, pyramid, estimator, landmarks, and window but does
not hash them, so their edits cannot be detected by the verifier. The source
hashes embedded in `benchmarks/basalt/m7an_f32_boundary_report.md:105-110`
are also stale: the report lists old estimator/window/AOM hashes
`A5D957...11AEF4`, `B7810F...961D72`, and `FC844D...2BF3D`, respectively.

Two report statements should be corrected along with the hashes:
`benchmarks/basalt/m7at_f32_window_exact_report.md:64` names
`M7_T_TRI_BITS`, while the current estimator label is `M7_TRI_BITS` at
`estimator.rs:1085`; and `benchmarks/basalt/m7ay_camera_unproject_fma_report.md:70`
says the temporary `VISLOC_BASALT_TRIANG_TRACE` dump was removed, while the
guard and both dumps remain at `estimator.rs:1033-1128`.

The complete verifier output at this snapshot is:

```text
FAIL hash mismatch for production source pipelines/basalt/src/vio/aom.rs
FAIL generator/diagnostic coverage mismatch: missing=['benchmarks/basalt/m7av_frame0_from_two_vectors_probe.cpp', 'benchmarks/basalt/m7bf_sophus_oracle_tmp.cpp'], extra=[]
FAIL hash mismatch for benchmarks/basalt/m7aq_visual_factor_boundary_probe.cpp
```

## Clean observations

- No literal `DBG`, `dbg!`, or temporary debug macro was found in the selected
  production/demo paths. The demo's `eprintln!` at
  `examples/basalt_euroc_vio_demo.rs:41,109` is error/progress reporting and
  `:156` is the requested summary; these are intentional CLI output. Its
  trace JSON is opt-out via `--no-trace`, not a hidden fixture.
- `tests/mh01_stereo_golden.rs:340` emits endpoint error statistics only from
  an explicitly ignored external test.
- The fixture-backed tests under `src/camera.rs`, `src/pyramid.rs`,
  `src/patch.rs`, and `src/vio/landmarks.rs` contain no `println!`/`eprintln!`
  calls in their active paths.

## Recommended cleanup and refresh order

1. Freeze the source snapshot and remove/move H1/H2 instrumentation. Decide
   whether H3's triangulation trace is deleted or retained as a dedicated,
   explicitly enabled audit probe. Remove `m7bf..._tmp.cpp`, or formally retain
   it as an audit-only artifact; register `m7av...probe.cpp` if retained.
2. Harden the ignored-test input contract (H5): require all dataset,
   calibration, config, raw-fixture, oracle, and executable paths instead of
   defaulting to `E:\...` or `target\...`.
3. After source edits are stable, refresh behavioral evidence from leaves to
   consumers: camera → pyramid/patch → landmarks/triangulation → AOM visual
   factors/solver → window assembly/marginalization → estimator/IMU schedule →
   `basalt_euroc_vio_demo` end-to-end traces. This prevents a downstream report
   from being refreshed against an older upstream operand.
4. Update `basalt_provenance_manifest_v1.json` in one pass: set the new capture
   date/basis, add source hashes for all seven changed files, refresh the AOM
   and M7aq probe hashes, and add/remove the M7av/M7bf generator records to
   match the retained files. Then run `refresh_provenance_hashes.py` (for the
   registered hash records) followed by `verify_provenance.py`.
5. Refresh source-hash-bearing reports first (`m7an`), then the camera/IMU/
   landmark/window comparison reports (`m7ay`, `m7at`, `m7az`, `m7bj`) and
   finally any end-to-end summaries. Each report should carry the final source
   hash set or manifest digest as well as artifact hashes. Re-run the final
   `rg` hygiene scan and provenance verifier before release; no report should
   be regenerated before the cleanup in steps 1–2 is complete.
