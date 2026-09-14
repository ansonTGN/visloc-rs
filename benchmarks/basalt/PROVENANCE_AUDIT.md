# Basalt provenance and redistribution audit

This 2026-08-30 metadata refresh covers the production Rust and diagnostic
generators present when the manifest was refreshed. A narrowly scoped exception
records explicitly retained audit source code under `target/` when a requested
replay depends on that exact source; the current exceptions are
`target/m7_householder_probe.cpp` and `target/m7_q2_gemm_probe.cpp`. Generated
target binaries, logs, traces, fixtures, and disassemblies remain transient and
are not registered as distribution payloads. The current direct-pack/Q2
campaign registry is
[`m11_current_golden_registry_20260830.json`](m11_current_golden_registry_20260830.json);
its referenced run outputs remain evidence, not bundled target artifacts. The
Cargo dependency/license inventory is now
[`cargo_license_inventory_v2.json`](cargo_license_inventory_v2.json); it
resolves all 119 lockfile package rows and binds the former unresolved rows to
exact crate, checksum, SPDX, license-file, and source evidence. The immutable
[`cargo_license_inventory_v1.json`](cargo_license_inventory_v1.json) is retained
as historical evidence and remains byte-unchanged. The
machine-readable source of truth is
[`basalt_provenance_manifest_v2.json`](basalt_provenance_manifest_v2.json),
the immutable frozen 2026-08-31 source/executable record (not a binding for
later source edits);
[`basalt_provenance_manifest_v1.json`](basalt_provenance_manifest_v1.json) is
retained as immutable historical evidence;
`generate_provenance_manifest_v2.py --verify-manifest` checks the current v2
binding without third-party Python packages. The legacy
[`verify_provenance.py`](verify_provenance.py) remains the v1/source audit and
validates the v2 inventory by default. Pass
`--cargo-license-inventory v1` when explicitly auditing the immutable v1
history file. The current v2 structural schema is
[`schemas/provenance_manifest_v2.schema.json`](schemas/provenance_manifest_v2.schema.json);
the v1 schema remains available for historical validation.

## Immutable references

| dependency | exact source commit | SPDX | source copied? | notice |
| --- | --- | --- | --- | --- |
| Basalt | `0f3b2b52c807f70ff4e2973ce253c73329eea7bc` | BSD-3-Clause | No; clean-room Rust/diagnostic code | Required if upstream source or binary is redistributed |
| basalt-headers | `aa441ba3e51050c47ba1902537792a2e4db7e43d` | BSD-3-Clause | No; clean-room Rust/diagnostic code | Required if headers/binaries are redistributed |
| OpenGV | `91f4b19c73450833a40e463ad3648aae80b3a7f3` | BSD-3-Clause | No; Rust mapper and probes call the API | Keep `License.txt` attribution for a bundled build |
| Sophus | `d0b7315a0d90fc6143defa54596a3a95d9fa10ec` (v1.24.6) | MIT | No; Rust Lie-group operations are a translation | Keep MIT notice if headers/binary are bundled |
| Eigen | `bc3b39870ecb690a623a3f49149a358b95c5781d` (v5.0.1) | MPL-2.0 plus listed third-party notices | No; only C++ diagnostics include Eigen | Keep Eigen COPYING files when Eigen is bundled |
| OpenCV | `cbee6841638edb6fbc8110df7cd52bb8e3d66211` (v4.12.0) | Apache-2.0; referenced FAST/corner files BSD-3-Clause | No; Rust FAST/corner code is a translation | Keep package and source-file notices if OpenCV is bundled |

The Basalt oracle checkout was checked at commit
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc`, tree
`b7afb830d82b45b8209cf784ad9744025d838411`.  The vcpkg resolution is pinned
to submodule commit `1e199d32ad53aab1defda61ce41c380302e3f95c` and baseline
`05442024c3fda64320bd25d2251cc9807b84fb6f`; the overlay ports pin
`basalt-headers` and OpenGV, while the baseline resolves Eigen 5.0.1,
OpenCV 4.12.0#1, and Sophus 1.24.6.  The registry tag commits and tree IDs
are recorded in the JSON manifest.  A materialized dependency checkout can be
verified with, for example:

The small upstream tag lookups used to resolve the immutable registry source
commits were necessary because the pinned Windows checkout contains the vcpkg
submodule as an unpopulated gitlink, not as redistributed dependency source.

```powershell
python benchmarks/basalt/verify_provenance.py `
  --checkout basalt=C:\path\to\basalt `
  --checkout basalt_headers=C:\path\to\basalt-headers `
  --checkout opengv=C:\path\to\opengv `
  --checkout sophus=C:\path\to\Sophus `
  --checkout eigen=C:\path\to\eigen `
  --checkout opencv=C:\path\to\opencv
```

## Translation and generator status

Every Rust source file under `pipelines/basalt/src` has an explicit
`translation_status`, upstream path references, and a SHA-256 binding in the
current v2 manifest. `clean_room_translation` is the current engineering provenance
classification: the implementation was written from behavior/contracts and
the audit found no copied upstream source subtree.  It is not a legal or
line-by-line authorship determination.  `clean_room_adapter` and
`clean_room_contract` identify boundaries and data contracts; `metadata_only`
is provenance metadata.  Every C++/fixture/script entry registered in that
snapshot has a SHA-256 hash,
an explicit `copied_upstream_source: false`, and the exact dependencies it
invokes. Production records carry a source SHA-256 and upstream source-path
mapping where applicable. The separate `retained_audit_sources` records
use the same hash/dependency contract for requested source exceptions; they do
not register generated results in `target/`.

If an authorized edit changes a diagnostic generator, run
`python benchmarks/basalt/refresh_provenance_hashes.py` once the worktree is
stable, followed by `python benchmarks/basalt/verify_provenance.py`.

## Release and GT firewall

`basalt_provenance_manifest_v2.json` is the immutable frozen 2026-08-31
source/executable record and has `ground_truth_artifacts: []`; its selected RC
executable and 52/80/400 certificate are evidence bindings, not bundled target
payloads. A new candidate may reuse the checked-in historical certificate only
as provenance history: source/input mismatches are emitted as warnings and
prevent frozen promotion until a fresh current-tree correctness run supplies a
replacement certificate.
`basalt_release_manifest_v1.json` contains source, provenance, license, and
validator metadata artifacts only and has `ground_truth_artifacts: []`. The
validator rejects GT-named
paths, `target/`, and `marg_data` in release artifact entries.  EuRoC
ground-truth metadata remains in the separate post-run dataset/evaluation
manifests and is explicitly excluded from this release manifest.

## Timing build provenance

`pipelines/basalt/Cargo.toml` declares `basalt-timing-breakdown` as an
opt-in feature and keeps the default feature set empty. The implementation in
`pipelines/basalt/src/timing.rs` makes the contract observable: the default
build's `TimingBreakdown::from_env()` is disabled without reading
`VISLOC_BASALT_TIMING_BREAKDOWN`, while a feature-enabled build accepts the
runtime switch only for the exact value `1`. `write_json` is available for the
feature-enabled sidecar and returns an explicit unsupported error in a default
build. Estimator construction uses `from_env`; it does not infer a feature
from the process environment.

Every Phase 6 plan and run must record the Cargo feature state, timing env
mode, and executable SHA-256 in its request identity and manifests. The
coordinator rejects an env-only timing request, an unknown/invalid state, or a
missing Rust executable content hash. This is a build-provenance binding, not
a claim that an arbitrary binary was inspected for Cargo metadata; the
declared state must be supplied with `--timing-feature enabled|disabled` and
verified against the binary/build record by the release harness.

## Legal uncertainty

This is a technical attribution record, not legal advice. No upstream source
is copied by the audited files, but a release owner must re-check transitive
vcpkg packages (including cereal, TBB, image codecs, and platform libraries),
the Cargo.lock dependency/license inventory, whether any future binary bundles
those packages, and whether future edits copy source rather than translate
behavior. `cargo_license_inventory_v2.json` is the current resolved technical
inventory; its explicit `ort` VCS dirtiness and winapi non-packaged-license
caveats still require legal review before redistribution. The immutable v1
inventory remains historical evidence, not the current resolution status.
Obtain legal review before a redistribution claim.
