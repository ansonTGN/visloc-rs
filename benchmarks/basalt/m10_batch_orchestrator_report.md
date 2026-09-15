# M10 batch orchestrator

`batch.py` is the deterministic matrix runner for the frozen
`basalt-euroc-parity-v1` contract. It coordinates the existing GT-firewalled
`harness.py`; it does not change estimator code or evaluation thresholds.

## Matrix and paths

The default matrix is all protocol sequences × three repetitions. Restrict it
with exact `--sequence` values or family prefixes (`--prefix MH`, `--prefix V1`,
and `--prefix V2`). Sequence resolution follows the dataset manifest's split:

```text
<dataset-root>/machine_hall/<MH_*> 
<dataset-root>/vicon_room/sequences/<V1_*/V2_*>
```

An explicit `all11` junction root and direct sequence root remain supported for
small fixtures. Each run is isolated under
`<output-root>/<method>/<profile>/<sequence>/r<repetition>/`, so compat and
extended results cannot share a namespace.

Example (PowerShell). Resolve every engine-side file to an absolute path:
`harness run` changes the estimator's working directory to its isolated
workspace, so relative executable/configuration paths are not portable. This
uses the pinned full EuRoC calibration through its existing WSL UNC path.

```powershell
$exe = (Resolve-Path "target\release\examples\basalt_euroc_vio_demo.exe").Path
$calib = '\\wsl$\Ubuntu-22.04\root\visloc-basalt-oracle-0f3b2b52\data\euroc_ds_calib.json'
$config = (Resolve-Path "configs\basalt\euroc_config.json").Path
$engineArgv = @(
  $exe,
  "--euroc-dir", "{input_root}",
  "--calibration", $calib,
  "--config", $config,
  "--out-dir", "{output_root}"
)
$engineArgvJson = $engineArgv | ConvertTo-Json -Compress

python -m benchmarks.basalt.batch `
  --protocol benchmarks\basalt\protocols\basalt_euroc_parity_v1.json `
  --dataset-root E:\datasets\euroc_mav `
  --dataset-manifest benchmarks\basalt\euroc_dataset_manifest.json `
  --output-root target\basalt_m10 `
  --method visloc_basalt_compat `
  --profile basalt-compat `
  --command-json $engineArgvJson `
  --trajectory trajectory.tum
```

Use `--dry-run` to print the resolved matrix without creating output or
starting a process. `--out` chooses a different final evaluation result path.

## Firewall and evaluation order

For every cell, the coordinator starts `python -m benchmarks.basalt.harness
run`. Harness stages only the protocol sensor paths and writes the run manifest
after the estimator exits. Only then does the coordinator resolve the
sequence's GT file and start a separate `harness evaluate` process. The GT path
is never part of the engine argv, environment, staged input, or run manifest.

## Resume and evidence

Each run manifest records a canonical request and SHA-256 fingerprint, source
tree identity, engine wall/RSS/status, and a hash/size inventory of all
non-manifest artifacts. Resume accepts a run only when the fingerprint,
protocol hash, command argv, staged input, source tree, and artifacts still
match. Legacy, incomplete, or mismatched directories fail closed. Evaluation
files are similarly bound to the manifest hash and request fingerprint.
The dataset manifest's protocol id, repository path, and SHA-256 binding are
also checked against the current protocol before planning, including dry-run.

The final `evaluation_result.json` uses the existing harness aggregate contract:
all successful, DNF, and failed cells remain in the matrix; failures stay in
the coverage/runtime denominator. `aggregate_results` is reused unchanged.

Focused fixture coverage is in `tests/test_basalt_batch.py` and exercises the
family split, ordered all11 × 3 dry-run, GT path/content isolation,
protocol-binding rejection, artifact-backed resume, and stale command
rejection.
