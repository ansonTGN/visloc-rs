# Basalt upstream oracle (M0/M1)

The M0 execution/evaluation contract is in
`protocols/basalt_euroc_parity_v1.json`, and the literal-port scope is frozen
in `basalt_port_manifest_v1.json`.  The independent `harness.py` `run` and
post-exit `evaluate` commands use these contracts without modifying the
upstream-oracle scripts below.

For deterministic M10 matrix runs (all 11 EuRoC sequences × three repetitions
by default), use `batch.py`. It resolves the frozen machine-hall/vicon-room
dataset split, keeps compat and extended output namespaces separate, runs and
evaluates each cell through the GT-firewalled harness, and resumes only from
matching request/artifact hashes. See `m10_batch_orchestrator_report.md` for
the PowerShell command and resume/dry-run contract.
The pinned-input and machine-local dataset readiness check is recorded in
`m10_execution_readiness_report.md`.

This directory records the upstream oracle used for the faithful Basalt port. The
oracle is the upstream repository at commit
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc`; it is not the existing Rust
optical-flow/extended path.

## Provenance

`upstream_manifest.json` fixes the source commit/tree, vcpkg submodule commit,
upstream EuRoC config/calibration hashes, and the source files used for the
first parity audit. `configs/basalt/euroc_config.json` is byte-identical to the
fixed upstream LF checkout; the oracle still uses the upstream path so the
provenance remains explicit.

The checked-in
[`basalt_provenance_manifest_v2.json`](basalt_provenance_manifest_v2.json) is
the immutable frozen record of the 2026-08-31 release candidate; it is not a
binding for later source edits. New pending candidates use the checked-in
`release_inputs/` calibration and historical correctness certificate, and
report certificate warnings when that certificate does not bind the current
source bytes. A new frozen manifest requires a fresh executable and correctness
certificate for the current tree; warning-bearing candidates are rejected.
The v2 verifier is the standard-library generator's `--verify-manifest` mode;
the v1 manifest below is retained as immutable historical evidence.

The current Cargo dependency/license inventory is
[`cargo_license_inventory_v2.json`](cargo_license_inventory_v2.json). It
resolves all 119 lockfile package rows and binds each former v1 unresolved row
to crate, checksum, SPDX, license-file, and source evidence. The immutable
[`cargo_license_inventory_v1.json`](cargo_license_inventory_v1.json) remains
available as historical evidence; `verify_provenance.py` selects v2 by default
and accepts `--cargo-license-inventory v1` for that history check.

## Timing feature contract

Timing breakdown is a compile-time opt-in. The `visloc-basalt` crate declares
Cargo feature `basalt-timing-breakdown`, while its default feature set is
empty. In a default build, `TimingBreakdown::from_env()` returns the disabled
collector without reading `VISLOC_BASALT_TIMING_BREAKDOWN`; an environment
variable by itself therefore cannot turn timing on. A feature-enabled build
reads the runtime variable only when its exact value is `1` and otherwise
keeps the collector disabled. `TimingBreakdown::write_json()` writes the
versioned timing sidecar only in a feature-enabled build; the default build
returns an explicit unsupported error.

Phase 6 run plans bind the declared feature state (`enabled` or `disabled`),
the exact environment mode, and the Rust executable content SHA-256 into the
request/session/run manifests. A request with
`VISLOC_BASALT_TIMING_BREAKDOWN=1` must declare
`--timing-feature enabled`; missing, disabled, invalid, or unknown declarations
fail closed. Use `--timing-feature disabled` for the canonical no-timing
release binary and `--timing-feature enabled` only for a binary built with:

```powershell
cargo build -p visloc-basalt --example basalt_euroc_vio_demo --features basalt-timing-breakdown --release
```

The timing sidecar is diagnostic evidence and is not part of the formal lean
trajectory-only output policy.

## Phase 6 runtime methodology

`phase6_coordinator.py` is a direct-path coordinator.  Its formal all11×3
population is exactly 66 lean cells (11 sequences × 3 repetitions × the two
methods), with `workers=1`, engine `threads=1`, seed `7`, and a fresh output
root.  A canonical formal gate additionally requires a validated sensor-only
view and a complete strict GT-absence fingerprint; a physical direct dataset
root is recorded but is never canonical.  A representative MargData/trace run
is never substituted into that denominator: diagnostics must use a separately
named root and are reported in `diagnostic_aggregate`.

Each cell records its complete artifact inventory after engine exit and before
evaluation or pruning.  Lean success requires an explicit pre-prune absence
proof for MargData, trace, timing, and other diagnostic paths; a leaked file is
a denominator-preserving DNF and cannot be resumed as success.  Resume reuse
is bound to the session, sequence/repetition pair and method position, cache
context, and Rust executable content SHA-256.  A resumed session is
recovery-only unless historical pair-adjacent launch order is reconstructed;
its runtime gate is therefore noncanonical.

The only authoritative cross-method RSS domain is the Linux `/proc` process
tree.  The native WSL wrapper's inner Linux value is retained for that gate;
Windows psutil values are auxiliary and mixed domains are `not_evaluable`.
The default Rust profile is `msvc_windows`; with that profile the coordinator
can compare common outer wall time, but the RSS resource gate is deliberately
`not_evaluable` because the Rust process is measured by Windows.  The optional
`rust_wsl_linux` profile launches
`rust_wsl_runner.py` inside Ubuntu and records both methods in the same Linux
`/proc` domain.  It is fail-closed until a local provenance manifest binds the
Linux binary/config/calibration/wrapper and source SHA, and an exactness
certificate proves trajectory/lifecycle/output parity at 52, 80, and 400
frames.  The certificate also binds the MSVC control executable; it does not
replace the MSVC binary as the canonical numeric target.
Runtime ratios use the common coordinator-host process-tree wall metric when
available; native inner-Linux wall time remains diagnostic and mixed wall
domains are `not_evaluable`.  A mixed direct/staged sensor namespace also makes
the corresponding gate `not_evaluable`; those values are never silently
substituted.  Custom Rust commands must bind the content SHA of their actual
`argv[0]` to the recorded executable before spawn and on resume.

## Supported execution host

The upstream CMakeLists explicitly supports Linux and macOS only. Windows native
is therefore not an oracle target. This machine has Ubuntu-22.04 WSL2 and the
EuRoC dataset is visible as `/mnt/e/datasets/euroc_mav`.

Run the reproducible setup/build from PowerShell:

```powershell
wsl.exe -d Ubuntu-22.04 -- bash /mnt/c/Users/rsasa/Workspace/visloc-rs/benchmarks/basalt/setup_oracle_wsl.sh
```

The script installs only WSL packages, checks out the fixed source and vcpkg
commits, bootstraps vcpkg, configures the official `relwithdebinfo` preset, and
builds the upstream targets. It never changes the repository worktree and does
not add ground truth to the estimator command line.

After a successful build, run a headless MH_01 oracle (the output is outside
the repository by default):

```powershell
wsl.exe -d Ubuntu-22.04 -- bash /mnt/c/Users/rsasa/Workspace/visloc-rs/benchmarks/basalt/run_oracle_wsl.sh MH_01_easy
```

The run writes the upstream `trajectory.csv`, `marg_data/`, command line, and
stdout/stderr under `$HOME/basalt-oracle-results/<sequence>/`. Evaluation must
be a separate post-run process; `--save-groundtruth` is intentionally omitted.

The checked-in runner accepts `ORACLE_BUILD_DIR` when a previously built
`basalt_vio` lives outside the preset directory. The recorded MH_01 smoke used
the core-only fallback at
`$HOME/visloc-basalt-oracle-0f3b2b52/build/core-relwithdebinfo`; it omits the
optional RealSense package because the fixed vcpkg RealSense 2.51.1 source does
not compile with this host's GCC 11.4. This fallback is clearly marked in the
provenance manifest and must not be treated as an official full-manifest build.

If setup cannot complete because a dependency mirror or compiler is unavailable,
keep the configure/build log and record the failure in the manifest rather than
substituting published numbers for a local oracle.

## M1 diagnostic trace

The sensor-only 400-frame trace oracle is recorded in
`target/basalt_upstream_mh01_400f_core_trace_20260821T000012Z/`.  It contains
`trace.jsonl`, a flattened `trace.csv`, `trace_summary.json` (including the
first 20 states and frame-0/frame-80 checkpoints), the harness manifest, and
the exact diagnostic source diffs.  The concise checkpoint report is
`m1_trace_report.md`.  To validate/flatten another trace:

```powershell
python benchmarks/basalt/trace_report.py `
  --trace target/<run>/trace.jsonl `
  --csv target/<run>/trace.csv `
  --summary target/<run>/trace_summary.json `
  --run-manifest target/<run>/run_manifest.json
```

The upstream estimator is unchanged unless `BASALT_TRACE_JSONL` is set; the
instrumented snapshot only reads track/window/LM/marginalization state and
writes one JSON object after each `optimize_and_marg` call.

The first-20-frame IMU golden trace is in
`target/basalt_upstream_mh01_400f_core_imu_trace_20260821T000014Z/`.  Set
`BASALT_IMU_TRACE_JSONL` to capture one record per frame, including
preintegrated deltas, residual blocks, costs, and predicted/optimized states.
`imu_trace_report.py` validates the contiguous JSONL and emits its CSV and
summary artifacts.

For a compact first-interval oracle, `m1_imu_golden_frame1_2.md` documents
`target/basalt_upstream_imu_golden_frame1_2.json`. This JSON is generated by a
standalone diagnostic C++ call to the pinned `IntegratedImuMeasurement<float>`
implementation and includes the exact frame-1→2 calibrated samples, delta
state, full covariance/square-root information matrices, and residuals from
the two upstream optimized state records. The source, state input, and exact
WSL compile/run command are hash-recorded in `upstream_manifest.json`.
