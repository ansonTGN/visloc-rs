# M10 release-manifest remediation

Captured 2026-08-29 06:32 JST. This remediation updates release/provenance
metadata and validator tests only. No engine run, heavy build, timing test,
release package, or pinned-checkout edit was performed.

## Exact before/after

| Contract | Before | After |
|---|---:|---:|
| Release artifact records | 9 path records, including 3 directories | 101 individual files |
| Per-artifact SHA-256/bytes | 0 / 0 | 101 / 101 |
| Contract inputs bound | 0 | 5 (port, upstream, protocol, config, provenance) |
| Individual test files bound | 0 of 44 | 44 of 44 |
| Dedicated release schema | absent | `schemas/release_manifest_v1.schema.json` present and artifact-bound |
| Release self-entry | listed (not hashable without recursion) | excluded with explicit `external_sha256` self-binding |
| Validator | prior blocker audit: path-only/incomplete | `Basalt provenance validation passed.` |
| Lightweight suite | prior suite: 27 passed | 28 passed, 0 failed, 1 cache warning |

The old release manifest was 1,671 bytes with SHA-256
`995056435ddabadcdd478c7e7e3a56df6c0a5141f0c14abf88c5d158d2b298bd`.
The repaired manifest is 23,313 bytes with SHA-256
`4c0310b2f3ef26302e636bec3930d5aed97f4038da25175cd6c4138400dd2007`.

## Inventory and validation

The deterministic inventory contains 32 Rust source files, 15 diagnostic
examples, 44 contract-test files, two NOTICE files, and the five required
contract inputs. Every record has `path`, `kind`, `bytes`, and lowercase
SHA-256; the validator recomputes both size and digest and reports zero
mismatches or duplicate paths.

The dedicated schema is
`benchmarks/basalt/schemas/release_manifest_v1.schema.json`, 3,006 bytes,
SHA-256
`a2cd19a366bc1dbf3468ed4f75b0eb3933d59ef7c2fd71131dc078993242b58b`.
`benchmarks/basalt/verify_provenance.py` now validates the schema contract,
all 101 file records, required contract-input inclusion, unique paths,
external self-binding, notices, and the release GT firewall.

Commands run:

```text
python -m benchmarks.basalt.verify_provenance
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q benchmarks/basalt/test_provenance.py tests/test_basalt_parity_harness.py tests/test_basalt_batch.py tests/test_basalt_parity_evaluator.py tests/test_euroc_dataset_manifest.py
```

The release manifest itself is metadata and is deliberately not one of its
own artifacts. Its ordinary final SHA-256 is recorded in this evidence and
must be captured externally by any packaging process; this avoids claiming a
mathematically impossible self-hash.

## Attribution and caveats

The two NOTICE files remain source-distribution attribution records. They do
not settle legal requirements for transitive vcpkg packages or binary
redistribution. The pinned WSL checkout remains dirty in five diagnostic
files, so its binaries are still labeled instrumented/dirty rather than
clean-pinned oracle evidence. The historical golden generator-source hash is
also intentionally preserved rather than rebound.
