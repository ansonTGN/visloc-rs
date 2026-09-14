# M10 release-bundle blocker audit

Captured 2026-08-29 06:21 JST while the phase-6 coordinator and two engine
workers remained active. This was read-only; no heavy build, engine run,
timing benchmark, release archive, or pinned-checkout edit was performed.

## Current passes

- `python -m benchmarks.basalt.verify_provenance` — **PASS**.
- The lightweight provenance/GT-firewall suite — **27 passed, 0 failed**, with
  one existing pytest cache-permission warning.
- All 9 release artifact paths exist; `ground_truth_artifacts=[]` and the
  validator found zero forbidden artifact-path matches.
- Both NOTICE files exist and carry the recorded current hashes:
  `benchmarks/basalt/NOTICE` = 5,253 bytes,
  `307e6d77cd2b57c5548f3e650629d659c3aab62729f32bc87b5119b4a5d30841`; and
  `pipelines/basalt/NOTICE` = 10,486 bytes,
  `24e7aaee0388d60053e382fc60aa1bd241f8912599c7d5358ea57403cfb1484c`.

## Remaining release blockers

| ID | Status | Evidence / required owner action |
|---|---|---|
| Bundle content hashes | OPEN — reproducibility blocker | The 9 release records carry paths/kinds but **0** per-artifact SHA-256 or byte fields. Define a per-file inventory/archive contract and bind it externally. |
| Contract-input scope | OPEN — owner decision | The release list omits the port manifest, upstream manifest, parity protocol, and checked-in config. Include/hash them or explicitly declare external pinned inputs. |
| Test-content binding | OPEN — owner decision | `pipelines/basalt/tests` contributes 44 files / 273,787 bytes through one directory path, but none is directly provenance-listed or hashed. Add a test-tree inventory or document the exclusion. |
| Release schema | OPEN — owner decision | No dedicated `release_manifest_v1.schema.json` exists; release checks are hand-coded in `verify_provenance.py`. |
| Legal review | OPEN — legal | NOTICE metadata is present, but transitive vcpkg and binary redistribution obligations still require legal review. |

The release manifest itself is 1,671 bytes with SHA-256
`995056435ddabadcdd478c7e7e3a56df6c0a5141f0c14abf88c5d158d2b298bd`.
The repaired provenance manifest is 41,717 bytes with SHA-256
`5f872df75bf4a8a00fe5cfe7745593610ce4c296b24035e329c3ff30417ce77d`.
No release archive candidate was found in the repository scan for
release/bundle-named `.zip`, `.tar`, `.gz`, or `.7z` files.

## Frame-51 evidence scope

The new `work/m7im15_frame51_kf_counter_comparison_20260829.json`, matching
`.md`, and `work/m7im15_gdb_native_frame51_kf_counters_20260829.gdb` are
diagnostic evidence, not release artifacts or generator registrations. They
are outside the validator's inventory by both location (`work/`) and suffix;
the exact sizes and hashes are recorded in the companion JSON remediation
artifact.

## Caveats retained

The pinned WSL checkout `/root/visloc-basalt-oracle-0f3b2b52` is still dirty in
five diagnostic files. Any binary from it remains instrumented/dirty evidence,
not a clean-pinned oracle. The historical
`target/basalt_upstream_imu_golden_frame1_2.cpp` source hash remains intentionally
unbound to the current source. These conditions are disclosed rather than
silently relabeled.
