# M7eh release test and residue audit

Date: 2026-08-23 JST  
Scope: current M7ec/M7dx/M7dk Rust source, the M7cm-cited external checks,
and the clean native M7cr CTest oracle. This was verification only: no
production source, fixture, benchmark, commit, or push was changed.

## Disposition

**Pass for release tests and runtime residue scan.** The current
`visloc-basalt` release library suite and focused M7/window checks are green.
All 51 source lines containing `M7`/`m7` are inside `#[cfg(test)]` modules;
there is no M7-named production branch, debug/probe call, or environment
branch in `pipelines/basalt/src`.

The provenance validator is **not green** against this dirty/untracked
workspace snapshot. It reports four existing manifest problems (the changed
`aom.rs` hash, two diagnostic-source hash mismatches, and a diagnostic
coverage list that has not been refreshed). No manifest refresh was performed
because this task did not authorize provenance/source edits.

## Commands and exact results

The host is Windows PowerShell; `cargo.exe` is not on the inherited PATH, so
the exact executable used below was
`C:\Users\rsasa\.cargo\bin\cargo.exe`.

| command | result |
| --- | --- |
| `cargo test --release -p visloc-basalt --lib` | **168 passed, 0 failed, 1 ignored** (169 tests discovered) |
| `cargo test --release -p visloc-basalt --lib vio::aom::tests::m7 -- --nocapture` | **12 passed, 0 failed** (157 filtered) |
| `cargo test --release -p visloc-basalt --lib vio::window::tests -- --nocapture` | **13 passed, 0 failed** (156 filtered) |
| `cargo test --release -p visloc-basalt --lib patch` | **7 passed, 0 failed** (162 filtered) |
| `cargo test --release -p visloc-basalt --lib update` | **8 passed, 0 failed** (161 filtered; substring filter also selects 3 cross-module tests) |
| `cargo test --release -p visloc-basalt --lib update::tests` | **5 passed, 0 failed** (164 filtered) |
| `cargo test --release -p visloc-basalt --test m7bw_cam0_temporal_exact` | **1 passed, 0 failed** |
| `cargo test --release -p visloc-basalt --test m7bd_obs_pixel_exact -- --ignored` with the three `VISLOC_BASALT_*` paths set to the local MH_01/calibration/config inputs | **1 passed, 0 failed** |

The focused AOM run includes both
`m7dx_clean_track1_relative_pose_jacobians_are_bitwise_exact` and
`m7ec_clean_track2_stereo_factor_is_bitwise_exact`. The 13 window tests are
the current M7dk full-pose-sidecar/window contract coverage; there is no
function named `m7dk` in the source. The retained M7dk replay artifacts remain
at `target/m7dk_fresh5_detail.jsonl` and
`target/m7dk_state_hb_comparison.json`.

The successful M7bd command used these exact PowerShell environment values:

```powershell
$env:VISLOC_BASALT_MH01_ROOT = 'E:\datasets\euroc_mav\machine_hall\MH_01_easy'
$env:VISLOC_BASALT_CALIBRATION = 'C:\Users\rsasa\Workspace\visloc-rs\target\euroc_ds_calib.json'
$env:VISLOC_BASALT_CONFIG = 'C:\Users\rsasa\Workspace\visloc-rs\target\euroc_config.json'
& 'C:\Users\rsasa\.cargo\bin\cargo.exe' test --release -p visloc-basalt --test m7bd_obs_pixel_exact -- --ignored
```

The M7cm-cited provenance checks were run as follows:

```powershell
$env:PYTEST_DISABLE_PLUGIN_AUTOLOAD = '1'
python -m pytest -q benchmarks/basalt/test_provenance.py
```

Result: **1 passed, 1 failed**. The failure is
`test_provenance_manifest_and_generator_hashes`, whose validator returned the
four failures listed below; the release-artifact ground-truth firewall test
passed. Without disabling the globally installed xonsh pytest plugin, the
same command aborts before collection with
`NoConsoleScreenBufferError` (non-interactive PowerShell), so the disabled
autoload result is the meaningful unit-test result.

```powershell
python benchmarks/basalt/verify_provenance.py --root .
```

Result: exit code 1, with exactly four failure lines:

```text
FAIL hash mismatch for production source pipelines/basalt/src/vio/aom.rs
FAIL generator/diagnostic coverage mismatch: missing=[m7av..., m7bf..., m7bo..., m7br..., m7bw..., m7ca..., m7cj..., m7de..., m7di..., pipelines/basalt/examples/m7bw..., pipelines/basalt/examples/m7ca...]
FAIL hash mismatch for benchmarks/basalt/m7aq_predict_velocity_probe.cpp
FAIL hash mismatch for benchmarks/basalt/m7aq_visual_factor_boundary_probe.cpp
```

The long coverage line is abbreviated above only for readability; the
validator itself emitted the complete missing-path list. These are metadata
hygiene failures, not release test failures.

M7cr’s cited clean native command was also discoverable in WSL and rerun
against the existing clean worktree:

```bash
ctest --test-dir /root/visloc-basalt-clean-m7cr-20260823/build/core-relwithdebinfo --output-on-failure
```

Result: **100% tests passed, 18/18**, total time 0.18 s. The worktree was
present and its CTest result matched the M7cr report. The M7cr manifest still
records the pinned commit
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc`, tree
`b7afb830d82b45b8209cf784ad9744025d838411`, and clean status.

## Residue scan

Commands:

```powershell
rg -n -i "m7" pipelines/basalt/src -g "*.rs"
rg -n "env::|VISLOC_|M8C_|eprintln!|println!" pipelines/basalt/src -g "*.rs"
rg -n -i "debug|probe|dump|diag" pipelines/basalt/src -g "*.rs"
```

Counts and classification:

| scan result | count / disposition |
| --- | --- |
| `M7`/`m7` lines | 51 total; **51/51 test-only** in `camera.rs`, `patch.rs`, `update.rs`, `vio/aom.rs`, `vio/landmarks.rs`, and the one window test comment |
| M7 runtime branches | **0** |
| M7 runtime debug/probe output calls | **0** |
| environment/output marker lines | 20 total |
| `euroc.rs` temp-dir environment use | 1; test helper under `#[cfg(test)]` |
| `mapper/features.rs` M8C env/output use | 10; all in `#[cfg(test)]`, and the fixed-model trace test is explicitly `#[ignore]` |
| `vio/aom.rs` env marker | 1 documentation reference to the opt-in iteration hook; no env read there |
| `vio/window.rs` detail-trace env/output use | 8; intentional opt-in `VISLOC_BASALT_DETAIL_ITERATIONS`, `VISLOC_BASALT_DETAIL_TRACE`, and optional frame filter. The normal estimator path returns before any file write when unset. |

The window trace is a gated audit artifact, not a temporary M7 branch: the
source documentation states that normal estimator runs do not write it, and
the implementation requires the explicit trace environment variables. The
mapper `M8C_FIXED_*` output is confined to the ignored diagnostic test. No
unconditional `println!`/`eprintln!`, `M7_*` env variable, `BA_REL_DIAG`,
`RUNTIME_REL`, `BASALT_TRACE_JSONL`, or `BASALT_IMU_TRACE_JSONL` marker was
found in the Rust production source.

## Source and artifact hashes

At audit time `git rev-parse HEAD` was
`e07e0c9fd1507a519097b4e0c7bac7ab2afe1977`; `pipelines/basalt/` itself is an
untracked workspace package, so the content hashes below—not the Git commit—
bind the audited source snapshot. The aggregate is SHA-256 of the UTF-8
newline-joined, lexicographically sorted records
`<file_sha256><two spaces><pipelines/basalt/src-relative-path>` for all Rust
files under `pipelines/basalt/src`:

```text
production Rust source files: 31
production_source_tree_sha256: 33171a50b4f5ae5294838e1e723d8a57f40b5dfb4f58264533002ec0df358d8b
```

Key source and fixture hashes:

| path | bytes | SHA-256 |
| --- | ---: | --- |
| `pipelines/basalt/src/vio/aom.rs` | 161819 | `1d6d7c2eb55266541f0c0cafa99ed4e8a029e866cd97c201a3328b2b557e18cb` |
| `pipelines/basalt/src/vio/window.rs` | 166166 | `5eebb8961e9f4f82fd4c89cce98358a5f73d9de310ebde4f08dadb6be828c3e9` |
| `pipelines/basalt/src/patch.rs` | 32193 | `ababf5d4ecbcd8866abdb6ed668bd2b6d79f2db0e2c39ef8a9e7d2c7ff338503` |
| `pipelines/basalt/src/camera.rs` | 13314 | `6aa8fc0a3bd7e3d426001fd8b3d9d987638b3b0d440f75f68b54b7017885e9b9` |
| `pipelines/basalt/src/update.rs` | 11821 | `20cc095adcadcae0208ead2632f677560d94d349720d5459acd349729cd7bd6d` |
| `pipelines/basalt/src/vio/landmarks.rs` | 54118 | `fdc3d7ccc14bfb0e1dc9381216b2a7dae22eb643c197d9ed955784bdfaa27c9b` |
| `pipelines/basalt/src/mapper/features.rs` | 93911 | `2ce39d5c76784fcbf248028a7fe5c6759e0e419817697943c08d66fe1a395e28` |
| `pipelines/basalt/tests/fixtures/m7dx_clean_track1_relpose_jacobian.json` | 1713 | `74bf73417d0fa8e49fc2a41f94821113f6b32383b1fe939effea0a8180c12462` |
| `pipelines/basalt/tests/fixtures/m7ec_clean_track2_stereo_factor.json` | 728 | `a31e13de0cfa90574d565fcdb21879fcac93a103bd0ff212a2fcfa58ea5f6023` |
| `target/m7dk_fresh5_detail.jsonl` | 71798793 | `30e5b069f880e019a12f987943fbf481825ca39ff69a608b68b5376bbd266c1b` |
| `target/m7dk_state_hb_comparison.json` | 35520 | `2fa74516b8c3af0a4aec823471b2ad809ab4847c4b4b27b36e4da7f66be00385` |

## Audit conclusion

Current M7ec and M7dx exact-factor tests pass in release mode, M7dk’s window
sidecar contract passes the complete window test group, the M7cm-cited M7bd
and M7bw external checks pass with the available MH_01 inputs, and the clean
M7cr CTest oracle remains 18/18. No unmistakable temporary M7 residue was
found, so no source cleanup was warranted. Provenance metadata should be
refreshed separately after the parent change set is stable; this audit did not
make that change.
