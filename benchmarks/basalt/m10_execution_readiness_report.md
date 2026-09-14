# M10 execution-readiness audit (2026-08-22)

This is a post-integration readiness audit for the GT-free batch runner. No
estimator, mapper, `HANDOFF_*`, or `work/` files were changed, and no real
EuRoC engine run was started. The only implementation changes in this audit
are harness/schema validation hardening in `benchmarks/basalt/`.

## Pinned upstream inputs

The pinned WSL checkout is present at
`/root/visloc-basalt-oracle-0f3b2b52` (the Windows-accessible equivalent is
`\\wsl$\Ubuntu-22.04\root\visloc-basalt-oracle-0f3b2b52`). Both upstream files
match `basalt_port_manifest_v1.json` and `euroc_dataset_manifest.json`:

| input | bytes | SHA-256 | result |
|---|---:|---|---|
| `data/euroc_ds_calib.json` | 5967 | `ad8c5a18c48c55dacf61d18ebbc18cd7d4f3acccd5840bd5a5cd8646adb6271c` | present/match |
| `data/euroc_config.json` | 2401 | `82937bd6493e592ef89572d31260c10f7437b4fb3ff1fda179375713966e34fa` | present/match |

The checked-in [configs/basalt/euroc_config.json](../../configs/basalt/euroc_config.json)
is 2401 bytes and has the same config SHA. Calibration is intentionally not
vendored: the frozen provenance binds the upstream BSD-3-Clause file and its
SHA, while the Windows UNC path above gives the engine an absolute, existing
copy without adding a duplicate artifact to this repository.

The release/provenance records bind upstream commit
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc`, tree
`b7afb830d82b45b8209cf784ad9744025d838411`, and BSD-3-Clause licensing.

The checked-in execution records are present with these current hashes:

| file | bytes | current SHA-256 |
|---|---:|---|
| `benchmarks/basalt/euroc_dataset_manifest.json` | 920070 | `a803db9e01f7a39f587cb9f87fd8601c8800455135ae7a2dc67f9e108003dd26` |
| `benchmarks/basalt/protocols/basalt_euroc_parity_v1.json` | 4348 | `7957f102ffa47a67c3509cdf77942d599e7b0f5f5f2bf452987f87a55df246e5` |
| `configs/basalt/euroc_config.json` | 2401 | `82937bd6493e592ef89572d31260c10f7437b4fb3ff1fda179375713966e34fa` |
| `benchmarks/basalt/basalt_provenance_manifest_v1.json` | 22178 | `dbb5ae1610406be879f5e021f9cfae0bc75a572e6c24bfd74691015367b0a57b` |

## Dataset split and integrity

`E:\datasets\euroc_mav` exists and all 11 frozen sequence roots are present.
The physical split is:

```text
MH_01_easy ... MH_05_difficult  -> E:\datasets\euroc_mav\machine_hall\<sequence>
V1_01_easy ... V2_03_difficult  -> E:\datasets\euroc_mav\vicon_room\sequences\<sequence>
```

The `all11` tree is also present as Windows junctions, each resolving to the
corresponding physical split root. All required sensor CSV/YAML SHA-256 values
and camera image counts match the frozen dataset manifest. Ground-truth CSV
hashes also match the manifest audit records; they are not staged by the
engine-facing harness.

The manifest was re-frozen through the canonical generator (full archive CRC
mode), not hand-edited:

```powershell
python benchmarks/basalt/euroc_dataset_manifest.py build `
  --dataset-root E:\datasets\euroc_mav `
  --output benchmarks\basalt\euroc_dataset_manifest.json
```

It now binds `benchmarks/basalt/protocols/basalt_euroc_parity_v1.json` at
`7957f102ffa47a67c3509cdf77942d599e7b0f5f5f2bf452987f87a55df246e5`. The
manifest remains `status=frozen`; all 11 sequence records, including archive
bytes/SHA/CRC, sensor and camera hashes, row counts, and timestamp ranges,
are unchanged from the prior freeze (zero invariant mismatches). Only the
manifest binding/top-level manifest hash changed.

The current protocol also matches the planned frozen parity gate: ordered
all11 coverage, three repetitions, mean/median/worst aggregation with failed
runs retained in the denominator, separate post-exit evaluation, and the
documented coverage, SE(3) ATE, RPE, scale-diagnostic, wall-time, and RSS
thresholds. No threshold tuning was introduced by this audit.

## Post-integration validation

The focused readiness suite was run with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`
because the installed xonsh pytest plugin requires an interactive Windows
console in this host. The exact command and result were:

```powershell
$env:PYTEST_DISABLE_PLUGIN_AUTOLOAD = '1'
python -m pytest -q `
  tests/test_basalt_batch.py `
  tests/test_basalt_parity_harness.py `
  tests/test_basalt_parity_evaluator.py `
  benchmarks/basalt/test_provenance.py `
  tests/test_euroc_dataset_manifest.py
# 27 passed
```

Additional checks passed:

- `python benchmarks\basalt\verify_provenance.py --root .` — provenance
  validation passed. The authorized hash refresh changed only the recorded
  SHA-256 values for the edited `batch.py`, `harness.py`, and `schema.py`
  generators. Their current recorded hashes are `batch.py` =
  `a67dfc4490f77de96ae7fbd8d476d6f918bff6ab9bb04e8e3f2ed51288096bf5`,
  `harness.py` =
  `6fa5733311f99d68763721b935eaad30a68ce7ff37e4cb1ead1473edd878db38`,
  and `schema.py` =
  `67cd3511fa7320de215157cfea721f5b6921575fff68fce4a540f4be729e3a53`.
- `python -m pytest -q tests/test_basalt_parity_harness.py` — **7 passed**;
  staged sensor-only input, post-exit evaluation, command GT-token rejection,
  runtime and peak process-tree RSS fields, and two-process CLI separation.
- `python -m pytest -q tests/test_basalt_parity_evaluator.py` — **4 passed**;
  33 paired baseline/port identities (11 sequences × 3 repetitions), all
  metric-specific gates, and missing-repetition rejection.
- `python -m pytest -q benchmarks/basalt/test_provenance.py` — **2 passed**;
  provenance generator hashes and release GT-artifact firewall.
- Schema adversarial smoke checks rejected both empty `minProperties` objects
  and invalid `$defs`-backed `additionalProperties`. Five tampered protocol
  variants were rejected for disabling pre-exit GT/shell/workspace/manifest
  firewall policies. A missing dataset manifest, a `../` GT path, and a
  manifest GT path redirected to a sensor CSV are also rejected during
  dry-run planning.
- The frozen compat contract reports **6 forbidden generic paths** and
  `velocity_seed=false`, `rebootstrap=false`, `sim3_welding=false`, with
  temporal/stereo FB threshold `0.04`.

`wsl.exe -d Ubuntu-22.04 -- ...` was used only to start the stopped pinned
checkout for a read-only file check. The UNC calibration path then resolved to
5967 bytes with SHA-256
`ad8c5a18c48c55dacf61d18ebbc18cd7d4f3acccd5840bd5a5cd8646adb6271c`.
The WSL distribution must be running before the real engine batch is started.

## Absolute GT-free batch command

The following PowerShell command uses the existing release executable, the
absolute Windows UNC calibration path, and the absolute checked-in config.
`harness run` changes the engine CWD, so the executable/calibration/config paths
must stay absolute; only the three documented placeholders are rendered per
cell.

```powershell
$exe = (Resolve-Path -LiteralPath 'target\release\examples\basalt_euroc_vio_demo.exe').Path
$calib = '\\wsl$\Ubuntu-22.04\root\visloc-basalt-oracle-0f3b2b52\data\euroc_ds_calib.json'
$config = (Resolve-Path -LiteralPath 'configs\basalt\euroc_config.json').Path
$argv = @(
  $exe,
  '--euroc-dir', '{input_root}',
  '--calibration', $calib,
  '--config', $config,
  '--out-dir', '{output_root}'
)
$argvJson = $argv | ConvertTo-Json -Compress

python -m benchmarks.basalt.batch `
  --protocol benchmarks\basalt\protocols\basalt_euroc_parity_v1.json `
  --dataset-root E:\datasets\euroc_mav `
  --dataset-manifest benchmarks\basalt\euroc_dataset_manifest.json `
  --output-root target\basalt_m10_readiness `
  --method visloc_basalt_compat `
  --profile basalt-compat `
  --command-json $argvJson `
  --trajectory trajectory.tum `
  --dry-run
```

The command was executed on this host. It resolved exactly **33 cells** in
protocol order (`MH_01_easy/r1` first, `V2_03_difficult/r3` last), with output
namespace
`visloc_basalt_compat/basalt-compat/<sequence>/r<repetition>`. Dry-run created
no output directory. It passes the engine's actual CLI flags (`--euroc-dir`,
`--calibration`, `--config`, `--out-dir`) and does not provide any GT path.
