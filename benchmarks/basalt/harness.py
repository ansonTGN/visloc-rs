#!/usr/bin/env python3
"""Run and score Basalt parity trials with a ground-truth firewall.

The ``run`` subcommand is deliberately unable to receive a ground-truth path:
it stages only the sensor paths named by the protocol into a fresh engine
workspace, sanitizes GT-looking environment variables, and starts the
estimator with an argv vector (never through a shell).  It writes the run
manifest only after the estimator exits.

The ``evaluate`` subcommand is a separate process boundary.  It is the only
subcommand that accepts a ground-truth path and delegates the established
EuRoC SE(3)/Sim(3)/RPE implementation to ``scripts/evaluate_euroc_trajectory.py``.
Coverage uses the number of staged camera frames as its denominator, including
failed runs, while Sim(3) scale remains a diagnostic and never replaces metric
SE(3) scoring.
"""

from __future__ import annotations

import argparse
import csv
import datetime as _dt
import hashlib
import importlib.util
import json
import os
import platform
import shutil
import statistics
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Iterable, Sequence


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_PROTOCOL = (
    Path(__file__).resolve().parent / "protocols" / "basalt_euroc_parity_v1.json"
)
MANIFEST_SCHEMA_ID = "basalt.run_manifest.v1"
EVALUATION_SCHEMA_ID = "basalt.evaluation_result.v1"
PROTOCOL_ID = "basalt-euroc-parity-v1"
_UTC = getattr(_dt, "UTC", _dt.timezone.utc)
_GT_ENV_MARKERS = (
    "GROUND_TRUTH",
    "GROUNDTRUTH",
    "GT_PATH",
    "GT_FILE",
    "GT_CSV",
    "VISLOC_GT",
)
_GT_TOKEN_MARKERS = (
    "state_groundtruth_estimate0",
    "ground_truth",
    "ground-truth",
    "groundtruth",
)


def utc_now() -> str:
    return _dt.datetime.now(_UTC).replace(microsecond=0).isoformat().replace(
        "+00:00", "Z"
    )


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def sha256_tree(path: Path) -> str:
    """Hash file names, sizes, and bytes in deterministic relative order."""

    digest = hashlib.sha256()
    if not path.is_dir():
        raise ValueError(f"expected directory for tree hash: {path}")
    files = sorted(
        (item for item in path.rglob("*") if item.is_file() and not item.is_symlink()),
        key=lambda item: item.relative_to(path).as_posix(),
    )
    for item in files:
        relative = item.relative_to(path).as_posix().encode("utf-8")
        digest.update(relative)
        digest.update(b"\0")
        digest.update(str(item.stat().st_size).encode("ascii"))
        digest.update(b"\0")
        digest.update(bytes.fromhex(sha256_file(item)))
        digest.update(b"\n")
    return digest.hexdigest()


def _command_hash(command: Sequence[str]) -> str:
    payload = json.dumps(list(command), ensure_ascii=False, separators=(",", ":")).encode(
        "utf-8"
    )
    return hashlib.sha256(payload).hexdigest()


def _resolve_command_executable(command: Sequence[str]) -> Path | None:
    if not command:
        return None
    candidate = Path(str(command[0]))
    if candidate.is_file():
        return candidate.resolve()
    resolved = shutil.which(str(command[0]))
    return Path(resolved).resolve() if resolved else None


def _command_file_provenance(command: Sequence[str]) -> list[dict[str, str]]:
    """Hash command-referenced files without opening any GT path.

    The command firewall runs first.  This helper is therefore only called
    after explicit GT-looking argv tokens have been rejected.
    """

    records: list[dict[str, str]] = []
    seen: set[Path] = set()
    executable = _resolve_command_executable(command)
    candidates: list[Path] = []
    if executable is not None:
        candidates.append(executable)
    for token in command[1:]:
        candidate = Path(str(token))
        if candidate.is_file():
            candidates.append(candidate.resolve())
    for candidate in candidates:
        if candidate in seen or not candidate.is_file():
            continue
        seen.add(candidate)
        records.append({"path": str(candidate), "sha256": sha256_file(candidate)})
    return records


def _reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=_reject_duplicate_keys)
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def load_protocol(path: Path = DEFAULT_PROTOCOL) -> dict[str, Any]:
    protocol = read_json(path)
    if protocol.get("schema_version") != 1 or protocol.get("protocol_id") != PROTOCOL_ID:
        raise ValueError(f"unsupported Basalt protocol: {path}")
    dataset = protocol.get("dataset")
    if not isinstance(dataset, dict):
        raise ValueError("protocol must contain a dataset object")
    if dataset.get("ground_truth_available_to_runner") is not False:
        raise ValueError("protocol must declare ground_truth_available_to_runner=false")
    if dataset.get("ground_truth_materialized_before_engine_exit") is not False:
        raise ValueError("protocol must prohibit pre-exit ground truth materialization")
    sensor_paths = dataset.get("sensor_input_paths")
    if not isinstance(sensor_paths, list) or not sensor_paths or not all(
        isinstance(item, str) and item for item in sensor_paths
    ):
        raise ValueError("protocol must declare non-empty sensor_input_paths")
    ground_truth_paths = dataset.get("ground_truth_paths")
    if not isinstance(ground_truth_paths, list) or not ground_truth_paths or not all(
        isinstance(item, str) and item for item in ground_truth_paths
    ):
        raise ValueError("protocol must declare non-empty ground_truth_paths")
    policy = protocol.get("execution_policy")
    if not isinstance(policy, dict):
        raise ValueError("protocol must contain an execution_policy object")
    required_false = {
        "shell_execution": False,
        "ground_truth_path_is_never_passed_to_engine": True,
        "ground_truth_environment_keys_are_removed": True,
    }
    required_true = {
        "command_is_argv": True,
        "engine_workspace_is_staged_copy": True,
        "only_sensor_input_paths_are_staged": True,
        "manifest_written_after_engine_exit": True,
        "evaluation_must_be_a_separate_process": True,
    }
    for key, expected in (*required_false.items(), *required_true.items()):
        if policy.get(key) is not expected:
            if expected:
                raise ValueError(f"protocol must require execution_policy.{key}=true")
            raise ValueError(f"protocol must require execution_policy.{key}=false")
    if not isinstance(protocol.get("sequences"), list) or not protocol["sequences"]:
        raise ValueError("protocol has no sequences")
    return protocol


def protocol_sequence(protocol: dict[str, Any], sequence: str) -> dict[str, Any]:
    for item in protocol["sequences"]:
        if isinstance(item, str) and item == sequence:
            return {"id": item}
        if isinstance(item, dict) and item.get("id") == sequence:
            return item
    raise ValueError(f"sequence is not in protocol: {sequence}")


def _safe_relative(path: str | Path) -> Path:
    candidate = Path(str(path).replace("/", os.sep))
    if candidate.is_absolute() or candidate.drive:
        raise ValueError(f"input path must be relative: {path}")
    if any(part in ("", ".", "..") for part in candidate.parts):
        raise ValueError(f"input path escapes the staged root: {path}")
    return candidate


def _assert_not_nested(parent: Path, child: Path) -> None:
    parent_resolved = parent.resolve()
    child_resolved = child.resolve()
    if child_resolved == parent_resolved or parent_resolved in child_resolved.parents:
        raise ValueError(f"output directory must not be inside input directory: {child}")


def _copy_sensor_path(source_root: Path, destination_root: Path, relative: Path) -> int:
    """Copy one explicitly allowed path and return copied file count.

    Symlinks are rejected instead of followed: a symlink inside a nominally
    sensor-only path could otherwise make the GT file visible to the engine.
    Missing optional paths are skipped and recorded by the caller.
    """

    source = source_root / relative
    destination = destination_root / relative
    if not source.exists():
        return 0
    if source.is_symlink():
        raise ValueError(f"symlink is not allowed in staged input: {source}")
    if source.is_file():
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        return 1
    if not source.is_dir():
        raise ValueError(f"unsupported input path: {source}")
    copied = 0
    for item in sorted(source.rglob("*")):
        if item.is_symlink():
            raise ValueError(f"symlink is not allowed in staged input: {item}")
        if not item.is_file():
            continue
        item_relative = item.relative_to(source_root)
        target = destination_root / item_relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(item, target)
        copied += 1
    return copied


def stage_sensor_input(
    source_root: Path,
    destination_root: Path,
    allowed_paths: Iterable[str],
    frame_count_glob: str,
) -> dict[str, Any]:
    """Materialize a sensor-only input tree and return its provenance."""

    source_root = source_root.resolve()
    destination_root = destination_root.resolve()
    if not source_root.is_dir():
        raise ValueError(f"dataset sequence root does not exist: {source_root}")
    if destination_root.exists() and any(destination_root.iterdir()):
        raise ValueError(f"staged input directory is not empty: {destination_root}")
    destination_root.mkdir(parents=True, exist_ok=True)
    normalized_paths = [_safe_relative(path) for path in allowed_paths]
    missing: list[str] = []
    copied = 0
    for relative in normalized_paths:
        count = _copy_sensor_path(source_root, destination_root, relative)
        copied += count
        if count == 0:
            missing.append(relative.as_posix())
    if copied == 0:
        raise ValueError("protocol staged no sensor files")
    frame_pattern = _safe_relative(frame_count_glob)
    frame_files = [
        item
        for item in destination_root.glob(frame_pattern.as_posix())
        if item.is_file() and not item.is_symlink()
    ]
    return {
        "staged_root": str(destination_root),
        "source_root": str(source_root),
        "file_count": len(
            [item for item in destination_root.rglob("*") if item.is_file() and not item.is_symlink()]
        ),
        "frame_count": len(frame_files),
        "frame_count_glob": frame_pattern.as_posix(),
        "allowed_paths": [item.as_posix() for item in normalized_paths],
        "missing_paths": missing,
        "sha256": sha256_tree(destination_root),
    }


def _blocked_environment_keys(environment: dict[str, str]) -> list[str]:
    blocked = []
    for key in environment:
        upper = key.upper()
        if (
            upper in {"GT", "GROUNDTRUTH", "GROUND_TRUTH"}
            or upper.startswith("GT_")
            or upper.endswith("_GT")
            or any(marker in upper for marker in _GT_ENV_MARKERS)
        ):
            blocked.append(key)
    return sorted(blocked)


def sanitize_environment(base: dict[str, str] | None = None) -> tuple[dict[str, str], list[str]]:
    environment = dict(os.environ if base is None else base)
    blocked = _blocked_environment_keys(environment)
    for key in blocked:
        environment.pop(key, None)
    return environment, blocked


def _blocked_command_tokens(command: Sequence[str], ground_truth_relative: str) -> list[str]:
    gt_path = ground_truth_relative.replace("\\", "/").casefold()
    blocked: list[str] = []
    for token in command:
        normalized = str(token).replace("\\", "/").casefold()
        basename = normalized.rsplit("/", 1)[-1]
        is_gt_flag = normalized in {
            "--gt",
            "--ground-truth",
            "--groundtruth",
            "--ground-truth-path",
            "--groundtruth-path",
        }
        if is_gt_flag or normalized == gt_path or any(marker in normalized for marker in _GT_TOKEN_MARKERS):
            blocked.append(str(token))
        elif basename in {"gt.csv", "groundtruth.csv", "ground_truth.csv"}:
            blocked.append(str(token))
    return blocked


def render_command(command: Sequence[str], replacements: dict[str, str]) -> list[str]:
    rendered = []
    for token in command:
        value = str(token)
        for key, replacement in replacements.items():
            value = value.replace("{" + key + "}", replacement)
        rendered.append(value)
    if not rendered or not rendered[0]:
        raise ValueError("engine command is empty")
    return rendered


def _tree_rss_bytes(pid: int) -> int:
    try:
        import psutil  # type: ignore

        root = psutil.Process(pid)
        processes = [root, *root.children(recursive=True)]
        total = 0
        for process in processes:
            try:
                total += int(process.memory_info().rss)
            except (psutil.Error, OSError):
                continue
        return total
    except (ImportError, OSError):
        pass
    except Exception:
        # ``NoSuchProcess`` is expected during the final sample on fast
        # estimators.  The fallback monitor below is best-effort as well.
        pass
    # The repository's existing monitor has a Windows ctypes implementation;
    # use it when psutil is not installed, while retaining a zero-safe fallback.
    try:
        scripts = ROOT / "scripts"
        if str(scripts) not in sys.path:
            sys.path.insert(0, str(scripts))
        from benchmark_process_metrics import process_tree_rss  # type: ignore

        return int(process_tree_rss(pid))
    except (ImportError, OSError, RuntimeError, ValueError):
        return 0


def _run_process(
    command: Sequence[str],
    *,
    cwd: Path,
    environment: dict[str, str],
    log_path: Path,
    poll_seconds: float,
    timeout_seconds: float | None,
) -> dict[str, Any]:
    if poll_seconds <= 0:
        raise ValueError("poll_seconds must be positive")
    log_path.parent.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    started_utc = utc_now()
    peak_rss = 0
    timed_out = False
    returncode = -1
    with log_path.open("w", encoding="utf-8", newline="") as stream:
        stream.write("COMMAND_ARGV_JSON: " + json.dumps(list(command)) + "\n\n")
        stream.flush()
        try:
            process = subprocess.Popen(
                list(command),
                cwd=str(cwd),
                env=environment,
                stdout=stream,
                stderr=subprocess.STDOUT,
                shell=False,
            )
        except OSError as exc:
            stream.write(f"\nPROCESS_START_ERROR: {exc}\n")
            return {
                "returncode": -1,
                "wall_seconds": time.perf_counter() - started,
                "peak_process_tree_rss_bytes": 0,
                "started_utc": started_utc,
                "finished_utc": utc_now(),
                "timed_out": False,
                "start_error": str(exc),
            }
        while process.poll() is None:
            peak_rss = max(peak_rss, _tree_rss_bytes(process.pid))
            if timeout_seconds is not None and time.perf_counter() - started > timeout_seconds:
                timed_out = True
                process.terminate()
                try:
                    process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
                break
            time.sleep(poll_seconds)
        returncode = int(process.wait())
        peak_rss = max(peak_rss, _tree_rss_bytes(process.pid))
    return {
        "returncode": returncode,
        "wall_seconds": time.perf_counter() - started,
        "peak_process_tree_rss_bytes": peak_rss,
        "started_utc": started_utc,
        "finished_utc": utc_now(),
        "timed_out": timed_out,
    }


def _safe_output_candidate(root: Path, candidate: str) -> Path:
    relative = _safe_relative(candidate)
    result = (root / relative).resolve()
    if result != root.resolve() and root.resolve() not in result.parents:
        raise ValueError(f"trajectory candidate escapes engine output: {candidate}")
    return result


def _discover_trajectory(output_root: Path, candidates: Sequence[str]) -> Path | None:
    for candidate in candidates:
        path = _safe_output_candidate(output_root, candidate)
        if path.is_file() and not path.is_symlink():
            return path
    return None


def _engine_workspace_has_gt_name(workspace: Path) -> list[str]:
    found = []
    for item in workspace.rglob("*"):
        if not item.is_file() or item.is_symlink():
            continue
        name = item.relative_to(workspace).as_posix().casefold()
        if any(marker in name for marker in _GT_TOKEN_MARKERS):
            found.append(name)
    return sorted(found)


def run_engine(
    *,
    protocol_path: Path,
    dataset_root: Path,
    sequence: str,
    method: str,
    profile: str,
    output_dir: Path,
    command: Sequence[str],
    poll_seconds: float | None = None,
    timeout_seconds: float | None = None,
    trajectory_candidates: Sequence[str] | None = None,
) -> Path:
    """Execute one estimator run and return its post-exit manifest path."""

    protocol_path = protocol_path.resolve()
    protocol = load_protocol(protocol_path)
    sequence_info = protocol_sequence(protocol, sequence)
    if profile not in {"basalt-compat", "basalt-extended"}:
        raise ValueError(f"unsupported profile: {profile}")
    dataset_root = dataset_root.resolve()
    sequence_root = dataset_root / sequence
    if not sequence_root.is_dir():
        # Accept a sequence-root argument as a convenience, but keep the
        # protocol's normal dataset-root layout as the first interpretation.
        if dataset_root.name == sequence and dataset_root.is_dir():
            sequence_root = dataset_root
        else:
            raise ValueError(f"sequence root does not exist: {sequence_root}")
    output_dir = output_dir.resolve()
    _assert_not_nested(sequence_root, output_dir)
    # Keeping the run tree outside the dataset root prevents ``..`` traversal
    # from exposing another EuRoC sequence's ground-truth file to the engine.
    _assert_not_nested(dataset_root, output_dir)
    if output_dir.exists() and any(output_dir.iterdir()):
        raise ValueError(f"run output directory is not empty: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)
    workspace = output_dir / "engine_workspace"
    input_root = workspace / "input"
    engine_output = workspace / "output"
    workspace.mkdir(parents=True, exist_ok=True)
    engine_output.mkdir(parents=True, exist_ok=True)

    dataset = protocol["dataset"]
    forbidden_sensor_paths = [
        str(item)
        for item in dataset["sensor_input_paths"]
        if any(marker in str(item).casefold() for marker in _GT_TOKEN_MARKERS)
    ]
    if forbidden_sensor_paths:
        raise ValueError(
            "protocol sensor_input_paths overlap ground truth paths: "
            + repr(forbidden_sensor_paths)
        )
    input_record = stage_sensor_input(
        sequence_root,
        input_root,
        dataset["sensor_input_paths"],
        dataset["frame_count_glob"],
    )
    gt_relative = str(dataset["ground_truth_paths"][0])
    replacements = {
        "input_root": str(input_root),
        "output_root": str(engine_output),
        "output_dir": str(engine_output),
        "sequence": sequence,
        "run_id": output_dir.name,
        # Explicitly do not provide ground_truth or ground_truth_path keys.
    }
    rendered = render_command(command, replacements)
    blocked_tokens = _blocked_command_tokens(rendered, gt_relative)
    if blocked_tokens:
        raise ValueError(
            "engine command contains a ground-truth token: " + repr(blocked_tokens)
        )
    command_files = _command_file_provenance(rendered)
    command_executable = _resolve_command_executable(rendered)
    environment, blocked_environment = sanitize_environment()
    environment.update(
        {
            "VISLOC_BASALT_GT_FREE": "1",
            "VISLOC_BASALT_PROFILE": profile,
            "VISLOC_BASALT_SEQUENCE": sequence,
            "VISLOC_BASALT_INPUT_ROOT": str(input_root),
            "VISLOC_BASALT_OUTPUT_ROOT": str(engine_output),
        }
    )
    poll = (
        float(poll_seconds)
        if poll_seconds is not None
        else float(protocol["execution_policy"].get("resource_poll_seconds", 0.25))
    )
    log_path = workspace / "engine.log"
    execution = _run_process(
        rendered,
        cwd=workspace,
        environment=environment,
        log_path=log_path,
        poll_seconds=poll,
        timeout_seconds=timeout_seconds,
    )
    trajectory = _discover_trajectory(
        engine_output,
        trajectory_candidates
        or protocol.get("engine_output", {}).get(
            "trajectory_candidates",
            ["trajectory.tum", "slam_trajectory.csv", "trajectory.csv"],
        ),
    )
    gt_named_files = _engine_workspace_has_gt_name(workspace)
    firewall_ok = not gt_named_files
    status = "success" if execution["returncode"] == 0 and trajectory is not None and firewall_ok else "dnf"
    failure_reason = None
    if execution["returncode"] != 0:
        failure_reason = f"engine returncode {execution['returncode']}"
    elif trajectory is None:
        failure_reason = "trajectory artifact was not produced"
    elif not firewall_ok:
        failure_reason = "engine workspace contains a ground-truth-named artifact"
    trajectory_sha = sha256_file(trajectory) if trajectory is not None else None
    manifest = {
        "schema_version": 1,
        "schema_id": MANIFEST_SCHEMA_ID,
        "protocol_id": protocol["protocol_id"],
        "run_id": output_dir.name,
        "method": method,
        "profile": profile,
        "sequence": sequence,
        "protocol": {
            "path": str(protocol_path),
            "sha256": sha256_file(protocol_path),
        },
        "provenance": {
            "upstream": protocol.get("upstream", {}),
            "command_argv_sha256": _command_hash(rendered),
            "command_files": command_files,
            "command_executable": str(command_executable) if command_executable else None,
            "command_executable_sha256": (
                sha256_file(command_executable) if command_executable else None
            ),
            "python_executable": sys.executable,
            "python_executable_sha256": (
                sha256_file(Path(sys.executable)) if Path(sys.executable).is_file() else None
            ),
            "python_version": sys.version,
            "platform": platform.platform(),
            "cargo_lock_sha256": (
                sha256_file(ROOT / "Cargo.lock") if (ROOT / "Cargo.lock").is_file() else None
            ),
        },
        "ground_truth_firewall": {
            "ground_truth_available_to_engine": False,
            "ground_truth_path_passed_to_engine": False,
            "ground_truth_environment_keys_passed": False,
            "staged_input_excludes_ground_truth": True,
            "manifest_written_after_engine_exit": True,
            "blocked_environment_keys": blocked_environment,
            "blocked_command_tokens": [],
            "workspace_gt_named_artifacts": gt_named_files,
        },
        "input": {
            **input_record,
            "source_root_sha256": sha256_tree(sequence_root),
        },
        "execution": {
            **execution,
            "argv": rendered,
            "cwd": str(workspace),
            "resource_poll_seconds": poll,
            "stdout_log": str(log_path),
            "python": sys.version,
            "platform": platform.platform(),
        },
        "artifacts": {
            "trajectory": str(trajectory) if trajectory is not None else None,
            "trajectory_sha256": trajectory_sha,
            "stdout_log": str(log_path),
            "engine_output_root": str(engine_output),
        },
        "status": status,
        "failure_reason": failure_reason,
        "created_utc": utc_now(),
    }
    manifest_path = output_dir / "run_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest_path


def _load_trajectory_rows(path: Path) -> tuple[int, int]:
    """Return (all rows, successful rows) without consulting ground truth."""

    first = ""
    with path.open("r", encoding="utf-8", errors="replace") as stream:
        for line in stream:
            stripped = line.strip()
            if stripped and not stripped.startswith("#"):
                first = stripped
                break
    if not first:
        return 0, 0
    total = 0
    tracked = 0
    if "," in first:
        with path.open("r", encoding="utf-8", errors="replace", newline="") as stream:
            reader = csv.DictReader(stream)
            if reader.fieldnames is None:
                return 0, 0
            if "timestamp_ns" not in reader.fieldnames:
                raise ValueError("trajectory CSV is missing timestamp_ns")
            for row in reader:
                if not row or not row.get("timestamp_ns"):
                    continue
                total += 1
                success = row.get("tracking_success")
                if success is None or success.strip().casefold() in {"1", "true", "yes", "ok", "success"}:
                    tracked += 1
    else:
        with path.open("r", encoding="utf-8", errors="replace") as stream:
            for line in stream:
                fields = line.split()
                if not fields or fields[0].startswith("#"):
                    continue
                if len(fields) != 8:
                    raise ValueError("TUM trajectory row must have 8 fields")
                total += 1
                tracked += 1
    return total, tracked


def _load_euroc_evaluator() -> Any:
    path = ROOT / "scripts" / "evaluate_euroc_trajectory.py"
    spec = importlib.util.spec_from_file_location("visloc_euroc_trajectory_evaluator", path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load trajectory evaluator: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _distribution(values: Sequence[float], *, lower_is_better: bool = False) -> dict[str, Any]:
    if not values:
        return {"count": 0, "mean": None, "median": None, "worst": None}
    finite = [float(value) for value in values if value == value and abs(value) != float("inf")]
    if not finite:
        return {"count": 0, "mean": None, "median": None, "worst": None}
    return {
        "count": len(finite),
        "mean": statistics.fmean(finite),
        "median": statistics.median(finite),
        "worst": min(finite) if lower_is_better else max(finite),
    }


def evaluate_manifest(
    manifest_path: Path,
    ground_truth_path: Path,
    max_diff_ns: int,
    tum_time_unit: str = "ns",
) -> dict[str, Any]:
    """Score one run; this function is intended to run after engine exit."""

    manifest = read_json(manifest_path)
    firewall = manifest.get("ground_truth_firewall", {})
    required_false = (
        firewall.get("ground_truth_available_to_engine") is False
        and firewall.get("ground_truth_path_passed_to_engine") is False
        and firewall.get("ground_truth_environment_keys_passed") is False
    )
    if not required_false or firewall.get("staged_input_excludes_ground_truth") is not True:
        raise ValueError(f"manifest fails GT firewall: {manifest_path}")
    if manifest.get("schema_id") != MANIFEST_SCHEMA_ID:
        raise ValueError(f"unsupported manifest schema: {manifest_path}")
    if manifest.get("protocol_id") != PROTOCOL_ID:
        raise ValueError(f"manifest protocol mismatch: {manifest_path}")
    protocol_record = manifest.get("protocol", {})
    recorded_protocol_path = Path(str(protocol_record.get("path", "")))
    recorded_protocol_sha = protocol_record.get("sha256")
    if recorded_protocol_path.is_file() and recorded_protocol_sha:
        if sha256_file(recorded_protocol_path).casefold() != str(recorded_protocol_sha).casefold():
            raise ValueError(f"protocol artifact sha256 mismatch: {recorded_protocol_path}")
    trajectory_value = manifest.get("artifacts", {}).get("trajectory")
    execution = manifest.get("execution", {})
    expected_frames = int(manifest.get("input", {}).get("frame_count", 0))
    base: dict[str, Any] = {
        "manifest": str(manifest_path.resolve()),
        "manifest_sha256": sha256_file(manifest_path),
        "status": manifest.get("status", "failure"),
        "sequence": manifest.get("sequence"),
        "method": manifest.get("method"),
        "repetition": manifest.get("repetition"),
        "runtime": {
            "wall_seconds": float(execution.get("wall_seconds", 0.0)),
            "peak_process_tree_rss_bytes": int(execution.get("peak_process_tree_rss_bytes", 0)),
            "returncode": int(execution.get("returncode", -1)),
        },
        "coverage": {
            "expected_input_frames": expected_frames,
            "trajectory_rows": 0,
            "tracked_frames": 0,
            "failed_frames": expected_frames,
            "tracked_fraction": 0.0 if expected_frames else None,
        },
        "metrics": {},
        "failure_reason": manifest.get("failure_reason"),
    }
    if not trajectory_value:
        base["status"] = "dnf"
        base["failure_reason"] = base["failure_reason"] or "trajectory artifact missing"
        return base
    trajectory_path = Path(str(trajectory_value))
    if not trajectory_path.is_file():
        base["status"] = "dnf"
        base["failure_reason"] = "trajectory artifact does not exist"
        return base
    expected_trajectory_sha = manifest.get("artifacts", {}).get("trajectory_sha256")
    if expected_trajectory_sha and sha256_file(trajectory_path).casefold() != str(expected_trajectory_sha).casefold():
        base["status"] = "failure"
        base["failure_reason"] = "trajectory artifact sha256 mismatch"
        return base
    total_rows, tracked_rows = _load_trajectory_rows(trajectory_path)
    base["coverage"].update(
        {
            "trajectory_rows": total_rows,
            "tracked_frames": tracked_rows,
            "failed_frames": max(expected_frames - tracked_rows, 0),
            "tracked_fraction": tracked_rows / expected_frames if expected_frames else None,
        }
    )
    if int(execution.get("returncode", -1)) != 0:
        base["status"] = "dnf"
        base["failure_reason"] = base["failure_reason"] or "engine returned non-zero"
        return base
    evaluator = _load_euroc_evaluator()
    try:
        ground_truth = evaluator.load_ground_truth(ground_truth_path)
        estimate = evaluator.load_estimate(trajectory_path, tum_time_unit=tum_time_unit)
        score = evaluator.evaluate(ground_truth, estimate, max_diff_ns)
    except (OSError, ValueError, StopIteration) as exc:
        base["status"] = "dnf"
        base["failure_reason"] = f"trajectory evaluation failed: {exc}"
        return base
    base["status"] = "success"
    base["failure_reason"] = None
    base["metrics"] = {
        "ate_translation_se3_rmse_m": score["ate_translation_se3_m"]["rmse"],
        "ate_translation_sim3_rmse_m": score["ate_translation_sim3_m"]["rmse"],
        "rpe_translation_consecutive_rmse_m": score["rpe_translation_consecutive_m"]["rmse"],
        "rpe_translation_consecutive_sim3_rmse_m": score["rpe_translation_consecutive_sim3_m"]["rmse"],
        "rpe_rotation_consecutive_rmse_deg": score["rpe_rotation_consecutive_deg"]["rmse"],
        "sim3_scale_diagnostic": score["sim3_scale"],
        "associated_poses": score["associated_poses"],
        "association_ratio": score["association_ratio"],
    }
    base["association"] = {
        "associated_poses": score["associated_poses"],
        "estimate_poses": score["estimate_poses"],
        "association_ratio": score["association_ratio"],
        "max_association_delta_ns": score["max_association_delta_ns"],
        "max_diff_ns": max_diff_ns,
        "tum_time_unit": tum_time_unit,
    }
    return base


def aggregate_results(runs: Sequence[dict[str, Any]]) -> dict[str, Any]:
    metric_names = (
        "ate_translation_se3_rmse_m",
        "ate_translation_sim3_rmse_m",
        "rpe_translation_consecutive_rmse_m",
        "rpe_translation_consecutive_sim3_rmse_m",
        "rpe_rotation_consecutive_rmse_deg",
        "sim3_scale_diagnostic",
    )
    all_metric_names = (*metric_names, "coverage_tracked_fraction", "runtime_wall_seconds", "peak_process_tree_rss_bytes")
    successful = [run for run in runs if run.get("status") == "success"]
    completed = [run for run in runs if run.get("status") in {"success", "dnf", "failure"}]
    values: dict[str, list[float]] = {name: [] for name in all_metric_names}
    for run in successful:
        for name in metric_names:
            value = run.get("metrics", {}).get(name)
            if isinstance(value, (int, float)):
                values[name].append(float(value))
    for run in completed:
        coverage = run.get("coverage", {}).get("tracked_fraction")
        if isinstance(coverage, (int, float)):
            values["coverage_tracked_fraction"].append(float(coverage))
        runtime = run.get("runtime", {}).get("wall_seconds")
        rss = run.get("runtime", {}).get("peak_process_tree_rss_bytes")
        if isinstance(runtime, (int, float)):
            values["runtime_wall_seconds"].append(float(runtime))
        if isinstance(rss, (int, float)):
            values["peak_process_tree_rss_bytes"].append(float(rss))
    return {
        "run_count": len(runs),
        "completed_count": len(completed),
        "success_count": len(successful),
        "dnf_count": sum(run.get("status") == "dnf" for run in runs),
        "failure_count": sum(run.get("status") == "failure" for run in runs),
        "include_failures_in_denominator": True,
        "metrics": {
            name: _distribution(values[name], lower_is_better=name == "coverage_tracked_fraction")
            for name in all_metric_names
        },
    }


def evaluate_manifests(
    manifest_paths: Sequence[Path],
    ground_truth_path: Path,
    output_path: Path,
    max_diff_ns: int,
    tum_time_unit: str = "ns",
) -> Path:
    if not manifest_paths:
        raise ValueError("at least one manifest is required")
    if tum_time_unit not in {"ns", "s"}:
        raise ValueError("tum_time_unit must be ns or s")
    ground_truth_path = ground_truth_path.resolve()
    if not ground_truth_path.is_file():
        raise ValueError(f"ground truth does not exist: {ground_truth_path}")
    runs = [
        evaluate_manifest(path.resolve(), ground_truth_path, max_diff_ns, tum_time_unit)
        for path in manifest_paths
    ]
    result = {
        "schema_version": 1,
        "schema_id": EVALUATION_SCHEMA_ID,
        "protocol_id": PROTOCOL_ID,
        "ground_truth_used_after_engine_exit": True,
        "ground_truth": {
            "path": str(ground_truth_path),
            "sha256": sha256_file(ground_truth_path),
        },
        "evaluation_policy": {
            "max_association_delta_ns": max_diff_ns,
            "tum_time_unit": tum_time_unit,
            "alignment_primary": "SE(3)",
            "sim3_scale_is_diagnostic_only": True,
        },
        "runs": runs,
        "aggregate": aggregate_results(runs),
        "created_utc": utc_now(),
    }
    output_path = output_path.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return output_path


def _parse_command(args: argparse.Namespace) -> list[str]:
    if args.command_json is not None:
        value = json.loads(args.command_json)
        if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
            raise ValueError("--command-json must be a JSON array of strings")
        return list(value)
    command = list(args.command or [])
    if command and command[0] == "--":
        command.pop(0)
    if not command:
        raise ValueError("engine command is required")
    return command


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="subcommand", required=True)
    run = subparsers.add_parser("run", help="run an estimator with a GT-free staged input")
    run.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    run.add_argument("--dataset-root", type=Path, required=True)
    run.add_argument("--sequence", required=True)
    run.add_argument("--method", default="visloc_basalt_compat")
    run.add_argument("--profile", choices=("basalt-compat", "basalt-extended"), default="basalt-compat")
    run.add_argument("--output-dir", type=Path, required=True)
    run.add_argument("--command-json", help="JSON argv array; supports {input_root}, {output_root}, {sequence}")
    run.add_argument("--command", nargs=argparse.REMAINDER, help="argv after --command (never shell-parsed)")
    run.add_argument("--poll-seconds", type=float)
    run.add_argument("--timeout-seconds", type=float)
    run.add_argument("--trajectory", action="append", help="relative trajectory candidate in engine output")
    evaluate = subparsers.add_parser("evaluate", help="score manifests after engine exit in this process")
    evaluate.add_argument("--manifest", type=Path, action="append", required=True)
    evaluate.add_argument("--ground-truth", type=Path, required=True)
    evaluate.add_argument("--max-diff-ns", type=int, default=10_000_000)
    evaluate.add_argument("--tum-time-unit", choices=("ns", "s"), default="ns")
    evaluate.add_argument("--out", type=Path, required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        if args.subcommand == "run":
            if args.command_json is not None and args.command:
                parser.error("use only one of --command-json and --command")
            command = _parse_command(args)
            manifest = run_engine(
                protocol_path=args.protocol,
                dataset_root=args.dataset_root,
                sequence=args.sequence,
                method=args.method,
                profile=args.profile,
                output_dir=args.output_dir,
                command=command,
                poll_seconds=args.poll_seconds,
                timeout_seconds=args.timeout_seconds,
                trajectory_candidates=args.trajectory,
            )
            print(manifest)
            return 0 if read_json(manifest).get("status") == "success" else 1
        if args.max_diff_ns < 0:
            parser.error("--max-diff-ns must be non-negative")
        output = evaluate_manifests(
            args.manifest,
            args.ground_truth,
            args.out,
            args.max_diff_ns,
            args.tum_time_unit,
        )
        print(output)
        return 0
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        parser.error(str(exc))
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
