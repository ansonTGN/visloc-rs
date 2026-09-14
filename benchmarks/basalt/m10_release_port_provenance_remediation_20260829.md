# M10 provenance remediation

Captured 2026-08-29 06:15 JST. This is a metadata-only repair of the current
working-tree provenance manifest, followed by a scoped hash record for root's
frame-51 `estimator.rs` edit; no build, engine run, timing benchmark, oracle
binary, or pinned-checkout file was changed.

## Before/after

| Check | Before | After |
|---|---:|---:|
| Production Rust files | 31 listed / 32 actual; `mapper/session.rs` missing | 32 / 32; exact coverage |
| Generator/diagnostic files | 48 listed / 97 actual; 50 missing | 97 / 97; exact coverage |
| Incorrect generator entry | `m7im_cov_ldlt_report.md` listed | Removed from generator registry |
| Current source hash mismatches | 8 (one `aom.rs`, seven generators) | 0 |
| `verify_provenance` | FAIL | PASS: `Basalt provenance validation passed.` |
| Lightweight contract/GT-firewall tests | Not part of this remediation run | 27 passed, 0 failed, 1 cache warning |

The complete pre-repair mismatch values and all 50 exact paths are recorded in
the JSON companion. The repaired manifest is 41,639 bytes with SHA-256
`5f872df75bf4a8a00fe5cfe7745593610ce4c296b24035e329c3ff30417ce77d`.

## Scoped changes

- Registered `pipelines/basalt/src/mapper/session.rs` as a clean-room Basalt
  translation.
- Registered the 50 post-snapshot `.cpp`/`.py`/`.sh`/`.rs` diagnostic and
  generator files with current SHA-256 values, kinds, and upstream references.
- Removed the non-generator `benchmarks/basalt/m7im_cov_ldlt_report.md` entry.
- Refreshed the eight stale current-tree SHA-256 records only. Historical
  golden source/output bindings were not overwritten.

## Scoped follow-up refresh

Root subsequently changed `pipelines/basalt/src/vio/estimator.rs` for the
faithful frame-51 stereo-observation counter. The initial manifest had no
optional hash field for that port file, so this follow-up records the current
source hash `d898c63375a98b4ad58cab9d7206fe153baa86256ab60b9a7d9cda531a2294a7`
and leaves all other source records untouched. The manifest now has two hashed
production port records (`aom.rs` and `estimator.rs`) and remains validator-clean.

The new frame-51 evidence files are deliberately not generator registrations:
they are `work/m7im15_frame51_kf_counter_comparison_20260829.json` (2,530
bytes, SHA-256 `834cac4b80f0e7d730ad309fa32c0f9a60787dd41b468a881116f852de9fb65d`),
the matching `.md` (1,723 bytes, SHA-256
`667c10a51684ef54175779d0f2c83f2612eb7f796a36b3a46489f1ab1d610d0f`), and
`work/m7im15_gdb_native_frame51_kf_counters_20260829.gdb` (683 bytes, SHA-256
`2e6c89cd8f142f4cbfedd6d342095fd2e1633bae0285581cac0c7bf844c10021`). The
validator inventories only code/script suffixes below `benchmarks/basalt` and
`pipelines/basalt/examples`, so registering these `work/` JSON/MD/GDB evidence
files would be outside its contract.

Validation commands:

```text
python -m benchmarks.basalt.verify_provenance
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q benchmarks/basalt/test_provenance.py tests/test_basalt_parity_harness.py tests/test_basalt_batch.py tests/test_basalt_parity_evaluator.py tests/test_euroc_dataset_manifest.py
```

## Caveats retained

The pinned WSL checkout `/root/visloc-basalt-oracle-0f3b2b52` is still dirty in
five diagnostic files, so binaries from it remain instrumented/dirty evidence,
not clean-pinned oracle evidence. The historical
`target/basalt_upstream_imu_golden_frame1_2.cpp` source hash also remains
intentionally unbound to the current source. These caveats were not hidden or
reclassified by this repair.
