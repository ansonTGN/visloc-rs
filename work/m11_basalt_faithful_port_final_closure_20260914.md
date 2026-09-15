# Basalt faithful Rust port — final closure

Date: 2026-09-14 JST
Status: **PASS_NATIVE_EQUIVALENT**
Completion: **100%**

## Scope and binding

This closes the faithful Rust port of upstream Basalt commit `0f3b2b52c807f70ff4e2973ce253c73329eea7bc` for EuRoC stereo-inertial VIO and the offline mapper.

- Authoritative worktree: `E:\visloc_archive\ssfm_runset_core_pr_20260819`
- Audited HEAD before this closure commit: `0734a881433615ff86ed2f55844aa66dbd249d5f`
- Production source tree SHA-256: `d7d39c379011800ad47908faf133ccc4173a9980b19d1475fc96cc7906eca1f5`
- Windows release executable: SHA-256 `941B740908B2BB448D039865F6461E86DDAFE4612E7635F50D44AAEB7C46DCDF`, 3,819,008 bytes
- Linux release executable: SHA-256 `A952F6D94304C387EB9A720B07E534A59F9DBDAB51AF55EC205CE44AC9B19F7F`, 3,962,664 bytes

## Requirement-by-requirement result

| Requirement | Result | Direct evidence |
|---|---|---|
| EuRoC stereo-inertial VIO functionality | Pass | Current full library suite: 374 passed, 0 failed, 59 ignored. Frontend, stereo/IMU ingestion, initialization, preintegration, ABS_QR LM, FEJ, square-root marginalization, lifecycle and lean output are implemented. |
| Native numerical behavior | Pass at the pinned clean dense oracle | Frame 4 iteration 0 is IEEE-754 f32 bit-exact at five pipeline boundaries. Each boundary has H 5625/5625 and b 75/75 exact. |
| Latest all11 formal GT-free validation | Pass | 22/22 selected cells succeeded. Every sequence passes ATE, coverage, RPE rotation/translation, Sim3 scale, runtime and sensor-only namespace gates. |
| Same-domain runtime/RSS | Pass | Current final Linux binary, three alternating repetitions: runtime median ratio 1.133199 and RSS median ratio 0.563384, limits 1.5. |
| Offline mapper from real native packet to final map | Pass, native-equivalent | Exact match graph: 120 pairs, 15,299 raw matches, 14,139 inliers, zero inlier-set mismatches, 397 tracks, 382 landmarks and 3,583 observations. All final points are within 1 mm of native. Points are not claimed bit-exact. |
| schema4 / FEJ / lifecycle | Pass exact | Windows/Linux 52, 80 and 400-frame lean replays have byte-exact CSV, TUM and lifecycle JSONL; forbidden diagnostic outputs are absent. |
| Release / golden / license / provenance | Pass | Final binary/source/input bindings, current golden registry, frozen provenance v2, active provenance v1 and selected license payload validate. License inventory resolves 119/119; legal clearance is not claimed. |
| External-SSD policy / change control | Pass | Bulk data, builds and runs stayed on E:. No push was performed; unrelated untracked files were preserved. |

## Formal all11 result

Artifact: `work/m11_phase6_latest_combined_all11x1_20260914.json`
SHA-256: `47AFEA5BD83EA10BFA985424D5F9121A1C4A3B687004613C994654CD9DF7E907`

- 22 selected native+Rust cells, 22 successes, no missing metrics.
- The artifact binds the selected run-manifest and evaluation hashes.
- All 11 sequences pass accuracy, coverage, RPE, scale, runtime and GT-free namespace gates.
- Its RSS field is deliberately `not_evaluable` because Windows Rust and Linux native RSS domains differ. It is not used for the RSS claim.

## Current-binary runtime and RSS

Artifact: `E:\visloc-rs-runs\basalt_goal_release_final_runtime3_20260914\phase6_gate_report.json`
SHA-256: `7F8D8F9EB93E0C7107F54633A04C0010D1601AAF2A759D3C0969462C6694C44B`

The run used workers=1, threads=1, seed=7, alternating order, the sensor-only GT firewall and six successful cells.

| Metric | Native | Rust | Ratio |
|---|---:|---:|---:|
| Runtime mean | 388.285 s | 436.724 s | — |
| Runtime median | 380.356 s | 431.019 s | 1.133199 |
| Runtime worst | 450.522 s | 449.650 s | — |
| RSS mean | 52,240,384 B | 29,545,813 B | — |
| RSS median | 52,441,088 B | 29,544,448 B | 0.563384 |
| RSS worst | 52,858,880 B | 29,679,616 B | — |

The earlier one-shot pair failed at runtime ratio 1.517863 and is intentionally preserved in `work/m11_release_final_runtime_pair_aggregate_20260914.json` (SHA-256 `B3348EF606E27662DE90394FB4E5009B2C1F0BD712940B8B13AED922266124BE`). The formal prescribed multi-repetition median gate above is the promotion result.

## Dense numerical parity

Artifact: `work/m11_absqr_dense_pipeline_current_release_20260914.json`
SHA-256: `B96FBD56E1DE88497EE38629D6FAEF7234432C12FFA084ACC404D1D4AAB7BB2E`

Against the clean pinned native oracle, `visual_total`, `imu_total`, `prior_before`, `prior_after` and `final` are all bit-exact: H 5625/5625 and b 75/75 at every stage.

## Offline mapper parity

Artifact: `work/m11_mapper_colpiv_fullv_parity_20260914.json`
SHA-256: `AF9FE51AE37017B926D4631F588294E02F8C6A4DCCEB37AC448EBE4971958DD4`

The OpenGV/Eigen-compatible five-point path produces the exact native match graph from the real frame51 schema4 packet. Final point coordinates are within 1 mm of native. This is a native-equivalence claim with a much tighter observed tolerance than the gate, not a bit-exact coordinate claim.

## Release evidence

- Cross-target exactness: `benchmarks/basalt/release_inputs/m11_rust_wsl_exactness_final2_20260914.json`, SHA-256 `19D0FE0A0568F609B3384394F2E22E43CDEF8FD85BFFB4D0E7CBD1997B0B56CF`
- Correctness certificate: `benchmarks/basalt/release_inputs/m11_release_candidate_final2_correctness_20260914.json`, SHA-256 `9F60E3DA48FF251F06C3727E21BA9F12F3C3174AED4D36AC2AF119B0F88A5573`
- Linux provenance: `benchmarks/basalt/release_inputs/m11_rust_wsl_provenance_final2_20260914.json`, SHA-256 `B8ADC1B8E40C735969571DF3DBA83BD4EAC7BE2997CC7BE4C9A431D83B4D2D0B`
- Golden registry: `benchmarks/basalt/m11_current_golden_registry_20260830.json`, SHA-256 `FCA72289AE4DF42C2BA03900498ED5DCD13FA16421C9B564FEBE79CA015503EC`
- Active provenance v1: SHA-256 `3FE589C4FBA81D8B6127FB1101E04EF05707436CC2A593717AF645E4E6F98DB2`
- Release manifest v1: SHA-256 `479757369DA4CD1AE3D15A9F80E1461C59BB0DD64AB9A829A15E4A54F6A8A825`
- Frozen provenance v2: SHA-256 `384FFF8DD3E1480B89CFC18AF6C9D8EF493606B6D2D81975C698D5BEBA9331B3`
- Cargo license inventory v2: SHA-256 `97B3C8DB56F8EBDA5E98975FEFE1EFCFAE51BAB19AC3D4F911ECF6CDF55A58EF`; 119/119 resolved, zero unresolved.
- Selected license graph: SHA-256 `A6779A901805284C7F0A529A9550BDF4315963D0C1DCCD3A17F02D143A51A790`
- Candidate license payload: SHA-256 `7865FDB77E8C2BB019F85EA6262E344EABEF6B34412D15DFF233376691A0FA9C`

Final checks all pass: `cargo test -p visloc-basalt --lib`, `cargo fmt --package visloc-basalt -- --check`, `git diff --check`, release-provenance freshness, provenance validation, and 15 provenance tests.

## Honest limitations

- Mapper coordinates are tolerance-equivalent within 1 mm, not bit-exact.
- The all11 aggregate's mixed-platform RSS cannot be compared directly; the separate same-domain gate provides the formal RSS evidence.
- The one-shot runtime failure remains recorded; the protocol-defined alternating three-repetition median passes.
- License metadata is complete, but legal clearance is outside this engineering audit.

No required implementation or validation work remains for the stated faithful-port objective. No push was performed.
