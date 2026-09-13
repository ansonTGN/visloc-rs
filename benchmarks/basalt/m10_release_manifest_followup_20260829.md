# M10 release-manifest canonical runner follow-up

Status: **PASS** for the scoped metadata/provenance remediation; no timing,
heavy build, package/release, commit, or push was performed. Worker PID 41260
was left undisturbed.

The reusable no-MargData/no-trace runner is now canonical at
`benchmarks/basalt/run_clonefree_perf.py` (19,830 bytes,
`bc334b62590aa0aa7a69a1c3bbccaae5f5efea18648029182202db8935d56c18`). The
old `work/m7im15_run_clonefree_perf_20260829.py` path is only a 373-byte
compatibility wrapper and is excluded from the release inventory. The result
schema is bound at
`benchmarks/basalt/schemas/clonefree_perf_result_v1.schema.json` (10,104
bytes, `fa6f428ef052068d8d3531c8fa52b29192a3a2a0aef05feb71e7d763cd91c397`).

## Binding refresh

| inventory | bytes | SHA-256 | records |
|---|---:|---|---:|
| `basalt_release_manifest_v1.json` | 23,765 | `eaaac886d47cec9e9f77ba509226e4010639f4ea59d2f4613698c7163de52c88` | 103 artifacts, including 44 contract tests |
| `basalt_provenance_manifest_v1.json` | 42,261 | `6d9b24ff2a2fc056b1688412c3fdda1721693cb9b9d83bf4623a887fb4f62ac3` | 98 generators + 3 retained sources |

Both new release artifacts have exact bytes/SHA-256 bindings. The canonical
runner is in `checked_in_generator_hashes`; the result schema is in
`retained_audit_sources` with a release-contract retention note. The release
manifest does not depend on the evidence-only wrapper.

## Validation and input evidence

- `python -m benchmarks.basalt.verify_provenance`: **PASS**.
- Draft-2020-12 validation of the clone-free result schema: **0 errors**.
- Canonical runner `--help`: **PASS**.
- Non-running preflight: **PASS** for MH01, 3,682 cam0 frames and 7,370
  sensor/image files; all six sensor CSV/YAML hashes verified.
- Focused Python suite: **28 passed, 0 failed**, with one existing
  `PytestCacheWarning` caused by workspace cache permissions.

The 52-frame real-data equivalence remains bound to
`work/m7im15_nomarg_real52_equivalence_20260829.json` SHA-256
`716c99bc0e3655d1f43f84da8992236220761196db24b4365e47de936edc7975`:
52 frames, 512 IMU samples, 14,661 observations, and byte-exact TUM/CSV
trajectories. Its timing is explicitly unusable because workers were active.

The prepared future cells are MH01 80, 400, and full 3,682 frames. Each will
use a fresh output/log/result/run-manifest namespace and the exact argv flags
`--no-marg-data --no-trace`. The runner records the fresh executable SHA-256
before launch and rejects missing or stale inputs.

The frozen parity protocol requires staged sensor copies. These cells use the
verified direct sensor path only for isolated performance measurement; every
result records that deviation and must remain outside staged-copy parity
aggregates. Source-distribution NOTICE attribution and transitive binary/legal
review remain separate obligations.
