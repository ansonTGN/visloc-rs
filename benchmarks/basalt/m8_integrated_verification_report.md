# M8 integrated verification

Date: 2026-08-22  
Basalt source commit under test: `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`

## Result

The post-fix M8a/M8b/M8d/M8e portable gates are green. The M8c frontend,
descriptor/ray, seeded RANSAC, and pinned OpenGV residual probes are green.
The remaining strict real-image M8c failure is confined to the final seeded
OpenGV-refined model: RANSAC models/inlier sets and all traced residual bits
match, but the optimizer's final translation is not stable at the existing
`1e-3` model gate for every seed/camera pair. This is the known optimizer /
pipeline boundary documented by `m8c_opengv_authoritative_report.md`; no
source-faithful arithmetic regression was found and no speculative production
edit was made. The diagnostic sweep measured refined-model errors of
`1.5650749510155125e-3` (cam0-to-cam0, seed `424242`),
`5.722348987908271e-3` (cam0-to-cam1, seed `7`), and
`6.0380015182011e-2` (cam0-to-cam1, seed `424242`), while the corresponding
RANSAC/inlier sets remained exact.

The ignored external M8d C++ check could not run in this Windows workspace:
the wrapper reaches WSL but `/tmp/upstream_m8d_setup_opt_oracle` is absent.
The checked-in Rust/C++ parity artifact remains available and unchanged.

## Verification commands

| Command or fixture | Result |
| --- | --- |
| `cargo test -p visloc-basalt mapper --lib` | 26 passed, 0 failed, 1 ignored; covers M8a/M8b NFR, MargData processing, mapper-off, and global-BA checks |
| `cargo test -p visloc-basalt --test m8c_feature_seed` | 1 passed |
| `cargo test -p visloc-basalt --test m8d_track_oracle` | 1 passed |
| `cargo test -p visloc-basalt --test m6_packet_contract` | 1 passed |
| ignored `m8c_exact_bits_probe` with the checked-in raw/oracle fixtures | 1 passed; exact residual-bit probe |
| ignored `m8c_debug_fixed_oracle_lm_trace` with the checked-in raw fixture | 1 passed |
| ignored real-image M8c diagnostic sweep | completed; diagnostic mode intentionally bypasses strict model assertions |
| ignored real-image M8c strict sweep | reaches the known refined-model boundary; see Result above |
| ignored `m8d_setup_opt_oracle` | blocked only by the missing external WSL executable |
| `cargo check --all-targets` | passed |
| `cargo run --release -p visloc-basalt --example m8e_rust_fixture` | stable output; 10/10 trials accepted |

The all-target check required only a helper compile repair: the root
`examples/basalt_marg_process_dump.rs` now has a direct `serde_json`
dependency and formats `MargDataProcessError` before propagating it. This is
outside the production M8 path and does not alter mapper behavior.

## M8e summary, hashes, and metrics

The current summary is `benchmarks/basalt/m8e_global_ba_oracle_summary.json`
(SHA-256 `770C7F74A9A6054ACE3E68390E1EED16E1F7946D8CE3BABC252230335AA1A39C`).
Its embedded source/artifact hashes were rechecked:

| Artifact | SHA-256 |
| --- | --- |
| `pipelines/basalt/src/mapper/mod.rs` | `E439594D34E099E0FAB96F5EB057745ACA0BCBA157D67DBC193DDE40DA740898` |
| `benchmarks/basalt/upstream_m8e_global_ba_oracle.cpp` | `01DC005BAE02B232F32A0AE02778938DA95B14AE2C2545336A3A0DD5BCFBCF5B` |
| `target/m8e_factor_oracle_v2.json` | `7A6F548D74B1C617F3423ED332DE2219AD2ED70BA509349AF6302326C3A95071` |
| `target/m8e_hdiag_oracle_diag4.json` | `E6B62DF297AA204B00AF63DFBC144AFC25C2936A07F5411AAD0C5EA407B3895F` |
| `target/m8e_hdiag_oracle_diag4.log` | `45703F12ECDE0530400020B6BDD7A36FAE8B94861E85363E17877E88ECBF633D` |
| `target/m8e_rust_fixture_fixed.json` | `6AE616990AB9670B11B7194DC179A87E5B47773211D65E89C8B4A91B489A4F43` |
| `target/m8e_rust_hdiag_fixed.log` | `ACAA77793AE16E0E6C6CA995BA36B43808C54A636A1BB6DC59F61981B51D147C` |

The fixture is `target/m8d_setup_m7v80.bin`, with 83 pose blocks, 549
landmarks, and 5,687 observations. Upstream exact metrics are:

* initial cost `2711067.758715515`;
* final cost `93074.762625592528`;
* final lambda `1e-32`;
* final-state FNV-1a `12460248985570421363`;
* per-iteration-trace FNV-1a `7693564316440188179`.

The current Rust run reports initial cost `2711067.758715517`, final cost
`93074.76262564836`, and Hdiag[0] `24980552.89154232` versus upstream
`24980552.891542263`. The final-cost delta is `5.5837e-8`; all 10 trials are
accepted. Its current raw final-state/trace FNV values are
`16337076209544843136` / `1429234084729328639`. These raw state/trace hashes
are toolchain-sensitive; the summary's stable cost, count, acceptance, and
source/artifact hash gates pass.

## Mapper-off and diagnostic-hook audits

`raw_image_capture_leaves_mapper_off_vio_trace_identical` passes on the
deterministic VIO fixture. It compares plain processing with
`process_with_images` and asserts identical state/keyframe connectivity,
active counts, state trace, phases, window/fallback state, and MargData after
clearing image payloads. The mapper's empty/no-op and MargData non-mutation
tests also pass.

Temporary `M8C_*` reads are confined to `#[cfg(test)]` mapper tests and test /
benchmark harnesses. `M8E_*` reads occur only in the diagnostic benchmark
artifacts; no `M8C_*` or `M8E_*` environment hook is present in the production
library runtime. Existing generic `VISLOC_BASALT_DETAIL_*` and
`VISLOC_BASALT_TRIANG_TRACE` diagnostics are unrelated and were not changed.

No `M7` files, `HANDOFF_codex.md`, or `work/` files were modified by this
verification.
