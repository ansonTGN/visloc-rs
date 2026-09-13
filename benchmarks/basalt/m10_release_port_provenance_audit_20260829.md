# M10 release/port/provenance current-tree audit

Captured 2026-08-29 05:58 JST while the phase-6 coordinator (PID 6232) and
two engine workers (PIDs 8492 and 46744) were active. No build, engine run, or
timing benchmark was started. The complete machine-readable evidence is in
`m10_release_port_provenance_audit_20260829.json`.

## Results

| Area | Result | Evidence |
|---|---|---|
| Immutable Basalt/vcpkg anchors | PASS | Commit `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`, tree `b7afb830d82b45b8209cf784ad9744025d838411`, vcpkg `1e199d32ad53aab1defda61ce41c380302e3f95c` agree across port/provenance/release/protocol/NOTICE/upstream manifests. |
| Config/calibration/dataset binding | PASS | Config 2401 bytes, SHA-256 `82937bd6493e592ef89572d31260c10f7437b4fb3ff1fda179375713966e34fa`; external WSL calibration 5967 bytes, SHA-256 `ad8c5a18c48c55dacf61d18ebbc18cd7d4f3acccd5840bd5a5cd8646adb6271c`; frozen dataset has 11 sequences, CRC checked, and protocol SHA `7957f102ffa47a67c3509cdf77942d599e7b0f5f5f2bf452987f87a55df246e5`. |
| Clean source-scope provenance | PASS (HEAD) / CAVEAT (working tree) | All 20 `upstream_manifest.source_scope.key_file_sha256` values match `git show HEAD`; the WSL checkout is dirty in five diagnostic source files, so dirty native binaries are not clean-oracle evidence. |
| Toolchain | PASS (recorded contract) | WSL reports g++ 11.4.0, CMake 4.4.2, Ninja 1.10.1; vcpkg baseline `05442024c3fda64320bd25d2251cc9807b84fb6f`; package/config/gitmodules hashes match manifest. |
| NOTICE/license attribution | PASS (metadata) | Both NOTICE files exist and list all six pinned dependencies and required SPDX/attribution text. Transitive vcpkg/binary redistribution remains a legal-review item. |
| GT firewall | PASS | Frozen protocol booleans, release artifact existence/path scan, and lightweight Python contract suite: **25 passed** (one pytest cache warning). No non-test Rust GT reader found; the only Rust GT path literal is in a `#[cfg(test)]` fixture. |
| Golden references | PARTIAL | Historical M1 run/trace/IMU output hashes are present and exact after the one metadata correction below; the golden generator source and later M7 source/binary references have drifted. |
| Current provenance validator | FAIL (snapshot drift) | `python -m benchmarks.basalt.verify_provenance` reports one missing production file, 50 unregistered generators, one non-generator extra entry, and eight current source hash mismatches. |

## Scoped corrections

- Corrected the 63-hex `m1_imu_trace.run_manifest_sha256` in
  `upstream_manifest.json` to the exact 64-hex digest of its referenced run
  manifest: `8258ff5dc37fd8f58432b16fca2eb687fb7d0d7172e4402f36d8ebc455e7d0bf`.
- Clarified `PROVENANCE_AUDIT.md` so its generator coverage claim is explicitly
  scoped to the 2026-08-22 snapshot. No production numeric code or release
  artifact was changed.

## Exact unresolved items

The provenance manifest was captured before later phase-6 additions. Current
coverage is 32 Rust source files vs 31 listed (missing
`pipelines/basalt/src/mapper/session.rs`) and 97 generator/diagnostic files vs
48 listed (50 missing; the full exact path list is in the JSON artifact).
`benchmarks/basalt/m7im_cov_ldlt_report.md` is incorrectly listed as a
generator although the validator only inventories code/script extensions.

The recorded `pipelines/basalt/src/vio/aom.rs` hash is stale, as are the seven
post-snapshot generator hashes listed in the JSON artifact. Refresh those only
after phase-6 source edits stabilize. Do not overwrite historical golden
source/binary hashes without regenerating their outputs. The 8/22 readiness
report also records the old provenance manifest size/hash (22,178 bytes,
`dbb5ae1610406be879f5e021f9cfae0bc75a572e6c24bfd74691015367b0a57b`), while
the current manifest is 30,061 bytes (`e7bbdca7905b98f1e5479a18bba7ef7351188ab406ba2125782065b17066cb40`).

