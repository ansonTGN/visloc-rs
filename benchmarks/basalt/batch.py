#!/usr/bin/env python3
"""Deterministic, failure-inclusive EuRoC batch orchestration for M10.

This module is intentionally a thin coordinator around :mod:`harness`.
Estimator processes are started only by ``harness run`` and receive a staged
sensor-only tree.  A ground-truth path is resolved and passed to
``harness evaluate`` only after the corresponding run process has exited.

The batch layout is stable and names the method/profile namespace explicitly::

    <output-root>/<method>/<profile>/<sequence>/r<repetition>/

Every completed run carries a request fingerprint and an artifact inventory.
Existing run directories are reused only when those records, the protocol,
the source tree, and every recorded artifact still match.  A stale or legacy
directory fails closed instead of being silently counted as a fresh run.

The runner has no threshold or tuning logic.  Metric aggregation is delegated
to ``harness.aggregate_results`` so DNF/failure rows remain in runtime and
coverage denominators.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import hashlib
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Iterable, Sequence

try:  # Supports both ``python -m benchmarks.basalt.batch`` and direct launch.
    from .harness import (
        EVALUATION_SCHEMA_ID,
        MANIFEST_SCHEMA_ID,
        PROTOCOL_ID,
        aggregate_results,
        load_protocol,
        read_json,
        render_command,
        sha256_file,
        sha256_tree,
        utc_now,
    )
except ImportError:  # pragma: no cover - direct script compatibility
    from harness import (  # type: ignore
        EVALUATION_SCHEMA_ID,
        MANIFEST_SCHEMA_ID,
        PROTOCOL_ID,
        aggregate_results,
        load_protocol,
        read_json,
        render_command,
        sha256_file,
        sha256_tree,
        utc_now,
    )


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_PROTOCOL = ROOT / "benchmarks" / "basalt" / "protocols" / "basalt_euroc_parity_v1.json"
DEFAULT_DATASET_MANIFEST = ROOT / "benchmarks" / "basalt" / "euroc_dataset_manifest.json"
BATCH_SCHEMA_ID = "basalt.batch_run.v1"
BATCH_VERSION = 1
_UTC = getattr(_dt, "UTC", _dt.timezone.utc)


class BatchError(ValueError):
    """Raised when a batch request or resumable artifact is unsafe."""


def _json_hash(value: Any) -> str:
    payload = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode(
        "utf-8"
    )
    return hashlib.sha256(payload).hexdigest()


def _write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _bind_document_content_hash(document: dict[str, Any]) -> dict[str, Any]:
    """Add a non-recursive canonical content hash to a result document."""

    batch = document.setdefault("batch", {})
    if not isinstance(batch, dict):
        raise BatchError("result batch metadata must be an object")
    batch.pop("content_sha256", None)
    batch["content_sha256"] = _json_hash(document)
    return document


def _verify_document_content_hash(document: dict[str, Any], path: Path) -> None:
    batch = document.get("batch")
    if not isinstance(batch, dict) or not isinstance(batch.get("content_sha256"), str):
        raise BatchError(f"result has no content hash; refusing stale reuse: {path}")
    expected = str(batch["content_sha256"])
    candidate = dict(document)
    candidate_batch = dict(batch)
    candidate_batch.pop("content_sha256", None)
    candidate["batch"] = candidate_batch
    if _json_hash(candidate).casefold() != expected.casefold():
        raise BatchError(f"result content hash mismatch: {path}")


def _flatten_values(values: Iterable[str] | None) -> list[str]:
    result: list[str] = []
    for value in values or ():
        for item in str(value).split(","):
            item = item.strip()
            if item:
                result.append(item)
    return result


def select_sequences(
    protocol_sequences: Sequence[str],
    *,
    sequences: Iterable[str] | None = None,
    prefixes: Iterable[str] | None = None,
) -> list[str]:
    """Select protocol sequences in frozen protocol order.

    ``sequences`` is exact (comma-separated values are accepted); ``prefixes``
    is case-insensitive and is useful for ``MH``, ``V1`` or ``V2`` subsets.
    With no selector all protocol sequences are selected.
    """

    ordered = list(protocol_sequences)
    if not ordered or len(set(ordered)) != len(ordered):
        raise BatchError("protocol sequence list must be non-empty and unique")
    exact = _flatten_values(sequences)
    prefix = _flatten_values(prefixes)
    unknown = [item for item in exact if item not in ordered]
    if unknown:
        raise BatchError(f"sequence is not in protocol: {unknown!r}")
    if not exact and not prefix:
        return ordered
    selected = [
        item
        for item in ordered
        if item in exact or any(item.casefold().startswith(value.casefold()) for value in prefix)
    ]
    if not selected:
        raise BatchError(f"selectors matched no protocol sequence: sequences={exact!r}, prefixes={prefix!r}")
    return selected


def _safe_component(value: str, label: str) -> str:
    value = str(value)
    if not value or value in {".", ".."} or Path(value).name != value:
        raise BatchError(f"{label} must be a single safe path component: {value!r}")
    if any(char in value for char in ("/", "\\", ":")):
        raise BatchError(f"{label} must not contain path separators: {value!r}")
    return value


def _manifest_sequence_map(manifest: dict[str, Any] | None) -> dict[str, dict[str, Any]]:
    if not manifest:
        return {}
    records = manifest.get("sequences", [])
    if not isinstance(records, list):
        raise BatchError("dataset manifest sequences must be an array")
    result: dict[str, dict[str, Any]] = {}
    for record in records:
        if isinstance(record, str):
            result[record] = {"id": record}
        elif isinstance(record, dict) and isinstance(record.get("id"), str):
            result[str(record["id"])] = record
    return result


def _resolve_manifest_protocol_path(value: str, manifest_path: Path) -> Path:
    """Resolve the manifest's repository-relative protocol binding."""

    candidate = Path(value).expanduser()
    if candidate.is_absolute() or candidate.drive:
        return candidate.resolve()
    # Frozen manifests store repository-relative paths.  A manifest copied to
    # another directory may instead use a path relative to its own location;
    # accept that only when it resolves to an existing file.
    repo_candidate = (ROOT / candidate).resolve()
    if repo_candidate.is_file():
        return repo_candidate
    return (manifest_path.parent / candidate).resolve()


def _validate_dataset_manifest_protocol_binding(
    manifest: dict[str, Any],
    *,
    manifest_path: Path,
    protocol_path: Path,
    protocol_sha256: str,
) -> None:
    """Fail closed when dataset metadata was frozen against another protocol."""

    binding = manifest.get("protocol")
    if not isinstance(binding, dict):
        raise BatchError(f"dataset manifest has no protocol path/SHA binding: {manifest_path}")
    binding_id = binding.get("id")
    if binding_id != PROTOCOL_ID:
        raise BatchError(
            f"dataset manifest protocol id mismatch: recorded={binding_id!r}, expected={PROTOCOL_ID!r}"
        )
    recorded_path = binding.get("path")
    if not isinstance(recorded_path, str) or not recorded_path:
        raise BatchError(f"dataset manifest protocol path binding is missing: {manifest_path}")
    resolved_recorded = _resolve_manifest_protocol_path(recorded_path, manifest_path)
    resolved_current = protocol_path.resolve()
    if resolved_recorded != resolved_current:
        raise BatchError(
            "dataset manifest protocol path mismatch: "
            f"recorded={resolved_recorded}, current={resolved_current}"
        )
    recorded_sha = binding.get("sha256")
    if not isinstance(recorded_sha, str) or recorded_sha.casefold() != protocol_sha256.casefold():
        raise BatchError(
            "dataset manifest protocol SHA mismatch: "
            f"recorded={recorded_sha!r}, current={protocol_sha256}"
        )


def _sequence_family(sequence: str, record: dict[str, Any] | None) -> str:
    family = record.get("family") if record else None
    if family in {"machine_hall", "vicon_room"}:
        return str(family)
    if sequence.casefold().startswith("mh"):
        return "machine_hall"
    if sequence.casefold().startswith(("v1", "v2")):
        return "vicon_room"
    raise BatchError(f"cannot infer EuRoC dataset family for {sequence}")


def resolve_sequence_root(
    dataset_root: Path,
    sequence: str,
    *,
    family: str | None = None,
    require_exists: bool = True,
) -> Path:
    """Resolve a sequence using the frozen machine-hall/vicon-room split.

    The normal base-root form is ``<base>/machine_hall/<MH>`` and
    ``<base>/vicon_room/sequences/<V*>``.  Explicit ``all11`` and direct
    sequence roots are accepted for fixtures and existing junction trees, but
    a flat ``<base>/<sequence>`` is only the final compatibility fallback.
    """

    root = Path(dataset_root).expanduser().resolve()
    sequence = _safe_component(sequence, "sequence")
    family = family or _sequence_family(sequence, None)
    if family not in {"machine_hall", "vicon_room"}:
        raise BatchError(f"unsupported EuRoC family: {family}")

    candidates: list[Path] = []
    if root.name.casefold() == sequence.casefold():
        candidates.append(root)
    elif root.name.casefold() == "all11":
        candidates.append(root / sequence)
        base = root.parent
        if family == "machine_hall":
            candidates.append(base / "machine_hall" / sequence)
        else:
            candidates.append(base / "vicon_room" / "sequences" / sequence)
    elif root.name.casefold() == "machine_hall":
        candidates.append(root / sequence)
    elif root.name.casefold() in {"vicon_room", "sequences"}:
        candidates.append(root / ("sequences" if root.name.casefold() == "vicon_room" else "") / sequence)
        if root.name.casefold() == "vicon_room":
            candidates.append(root / sequence)
    else:
        if family == "machine_hall":
            candidates.append(root / "machine_hall" / sequence)
        else:
            candidates.append(root / "vicon_room" / "sequences" / sequence)
        # An explicitly supplied all11/junction-compatible root can still
        # expose the sequence directly.  This is after the physical split.
        candidates.append(root / "all11" / sequence)
        # Tiny fixtures and legacy callers sometimes use a flat tree.
        candidates.append(root / sequence)

    unique: list[Path] = []
    seen: set[str] = set()
    for candidate in candidates:
        resolved = candidate.resolve()
        marker = os.path.normcase(str(resolved))
        if marker not in seen:
            seen.add(marker)
            unique.append(resolved)
    if require_exists:
        for candidate in unique:
            if candidate.is_dir():
                return candidate
        joined = ", ".join(str(item) for item in unique)
        raise BatchError(f"sequence root does not exist for {sequence}: {joined}")
    return unique[0] if unique else (root / sequence).resolve()


def _sequence_record(
    dataset_manifest: dict[str, Any] | None,
    sequence: str,
) -> dict[str, Any]:
    record = _manifest_sequence_map(dataset_manifest).get(sequence)
    return dict(record or {"id": sequence})


def _frame_count(record: dict[str, Any]) -> int:
    cameras = record.get("cameras")
    if isinstance(cameras, dict):
        cam0 = cameras.get("cam0")
        if isinstance(cam0, dict) and isinstance(cam0.get("row_count"), int):
            return max(0, int(cam0["row_count"]))
    counts = record.get("camera_counts")
    if isinstance(counts, dict):
        cam0 = counts.get("cam0")
        if isinstance(cam0, dict) and isinstance(cam0.get("csv_rows"), int):
            return max(0, int(cam0["csv_rows"]))
    return 0


def _ground_truth_relative_path(
    dataset_manifest: dict[str, Any] | None,
    sequence: str,
    protocol: dict[str, Any],
) -> str:
    record = _sequence_record(dataset_manifest, sequence)
    paths = protocol.get("dataset", {}).get("ground_truth_paths", [])
    if not isinstance(paths, list) or not paths or not isinstance(paths[0], str) or not paths[0]:
        raise BatchError("protocol has no ground_truth_paths")
    protocol_value = str(paths[0])
    manifest_value = record.get("ground_truth")
    if isinstance(manifest_value, dict) and isinstance(manifest_value.get("path"), str) and manifest_value["path"]:
        value = str(manifest_value["path"])
        if Path(value).as_posix() != Path(protocol_value).as_posix():
            raise BatchError(
                "dataset manifest ground-truth path mismatch: "
                f"recorded={value!r}, protocol={protocol_value!r}"
            )
    else:
        value = protocol_value
    relative = Path(value)
    if (
        not value
        or relative.is_absolute()
        or relative.drive
        or any(part in {"", ".", ".."} for part in relative.parts)
    ):
        raise BatchError(
            f"ground-truth path must be a safe sequence-relative path: {value!r}"
        )
    return relative.as_posix()


def _ground_truth_path(
    source_root: Path,
    dataset_manifest: dict[str, Any] | None,
    sequence: str,
    protocol: dict[str, Any],
) -> Path:
    relative = Path(_ground_truth_relative_path(dataset_manifest, sequence, protocol))
    root = source_root.resolve()
    candidate = (root / relative).resolve()
    if candidate != root and root not in candidate.parents:
        raise BatchError(f"ground-truth path escapes sequence root: {relative}")
    return candidate


def run_directory(output_root: Path, method: str, profile: str, sequence: str, repetition: int) -> Path:
    """Return the deterministic per-run output directory."""

    if repetition < 1:
        raise BatchError("repetition must be >= 1")
    return (
        Path(output_root).expanduser().resolve()
        / _safe_component(method, "method")
        / _safe_component(profile, "profile")
        / _safe_component(sequence, "sequence")
        / f"r{repetition}"
    )


def make_run_request(
    *,
    protocol_path: Path,
    protocol_sha256: str,
    dataset_manifest_path: Path | None,
    dataset_manifest_sha256: str | None,
    dataset_root: Path,
    source_root: Path,
    sequence: str,
    method: str,
    profile: str,
    repetition: int,
    command: Sequence[str],
    poll_seconds: float | None,
    timeout_seconds: float | None,
    trajectory_candidates: Sequence[str] | None,
    max_diff_ns: int,
    tum_time_unit: str,
) -> tuple[dict[str, Any], str]:
    """Build the immutable identity used to accept a resumable run."""

    request = {
        "schema_version": BATCH_VERSION,
        "schema_id": BATCH_SCHEMA_ID,
        "protocol_id": PROTOCOL_ID,
        "protocol_path": str(protocol_path.resolve()),
        "protocol_sha256": protocol_sha256,
        "dataset_manifest_path": str(dataset_manifest_path.resolve()) if dataset_manifest_path else None,
        "dataset_manifest_sha256": dataset_manifest_sha256,
        "dataset_root": str(dataset_root.resolve()),
        "source_root": str(source_root.resolve()),
        "sequence": sequence,
        "method": method,
        "profile": profile,
        "repetition": repetition,
        "command_template": [str(item) for item in command],
        "command_template_sha256": _json_hash([str(item) for item in command]),
        "poll_seconds": poll_seconds,
        "timeout_seconds": timeout_seconds,
        "trajectory_candidates": list(trajectory_candidates or []),
        "evaluation": {"max_diff_ns": max_diff_ns, "tum_time_unit": tum_time_unit},
    }
    return request, _json_hash(request)


def _artifact_inventory(run_dir: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    if not run_dir.is_dir():
        return records
    for path in sorted(run_dir.rglob("*"), key=lambda item: item.relative_to(run_dir).as_posix()):
        if not path.is_file() or path.is_symlink():
            continue
        relative = path.relative_to(run_dir).as_posix()
        if relative in {"run_manifest.json", "evaluation_result.json"}:
            continue
        records.append({"path": relative, "bytes": path.stat().st_size, "sha256": sha256_file(path)})
    return records


def _assert_artifact_inventory(run_dir: Path, records: Sequence[dict[str, Any]]) -> None:
    for record in records:
        relative = Path(str(record.get("path", "")))
        if not relative.parts or relative.is_absolute() or ".." in relative.parts:
            raise BatchError(f"invalid recorded artifact path in {run_dir}: {record!r}")
        path = (run_dir / relative).resolve()
        if path != run_dir.resolve() and run_dir.resolve() not in path.parents:
            raise BatchError(f"recorded artifact escapes run directory: {relative}")
        if not path.is_file() or path.is_symlink():
            raise BatchError(f"recorded artifact is missing: {path}")
        expected_sha = str(record.get("sha256", ""))
        expected_bytes = int(record.get("bytes", -1))
        if path.stat().st_size != expected_bytes or sha256_file(path).casefold() != expected_sha.casefold():
            raise BatchError(f"recorded artifact changed: {path}")


def _rendered_engine_command(command: Sequence[str], run_dir: Path, sequence: str) -> list[str]:
    workspace = run_dir / "engine_workspace"
    return render_command(
        command,
        {
            "input_root": str(workspace / "input"),
            "output_root": str(workspace / "output"),
            "output_dir": str(workspace / "output"),
            "sequence": sequence,
            "run_id": run_dir.name,
        },
    )


def _manifest_failure_run(manifest_path: Path, reason: str) -> dict[str, Any]:
    """Create the existing evaluation contract's DNF row without GT access."""

    manifest = read_json(manifest_path)
    execution = manifest.get("execution", {})
    input_record = manifest.get("input", {})
    expected_frames = int(input_record.get("frame_count", 0))
    return {
        "manifest": str(manifest_path.resolve()),
        "manifest_sha256": sha256_file(manifest_path),
        "status": "dnf",
        "sequence": manifest.get("sequence"),
        "method": manifest.get("method"),
        "profile": manifest.get("profile"),
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
        "failure_reason": reason,
    }


def _write_synthetic_failure_manifest(
    *,
    run_dir: Path,
    protocol_path: Path,
    protocol_sha256: str,
    source_root: Path,
    sequence: str,
    method: str,
    profile: str,
    repetition: int,
    request: dict[str, Any],
    request_sha256: str,
    command: Sequence[str],
    expected_frames: int,
    wall_seconds: float,
    reason: str,
) -> Path:
    run_dir.mkdir(parents=True, exist_ok=True)
    invocation_path = run_dir / "batch_invocation_failure.json"
    _write_json(invocation_path, {"command_template": list(command), "reason": reason})
    manifest_path = run_dir / "run_manifest.json"
    manifest = {
        "schema_version": 1,
        "schema_id": MANIFEST_SCHEMA_ID,
        "protocol_id": PROTOCOL_ID,
        "run_id": run_dir.name,
        "method": method,
        "profile": profile,
        "sequence": sequence,
        "repetition": repetition,
        "protocol": {"path": str(protocol_path.resolve()), "sha256": protocol_sha256},
        "ground_truth_firewall": {
            "ground_truth_available_to_engine": False,
            "ground_truth_path_passed_to_engine": False,
            "ground_truth_environment_keys_passed": False,
            "staged_input_excludes_ground_truth": True,
            "manifest_written_after_engine_exit": True,
        },
        "input": {
            "staged_root": str(run_dir / "engine_workspace" / "input"),
            "source_root": str(source_root.resolve()),
            "file_count": 0,
            "frame_count": expected_frames,
            "sha256": "0" * 64,
        },
        "execution": {
            "argv": _rendered_engine_command(command, run_dir, sequence),
            "cwd": str(run_dir.resolve()),
            "returncode": -1,
            "wall_seconds": max(0.0, wall_seconds),
            "peak_process_tree_rss_bytes": 0,
            "resource_poll_seconds": 0.25,
        },
        "artifacts": {
            "trajectory": None,
            "trajectory_sha256": None,
            "stdout_log": str(invocation_path),
        },
        "status": "dnf",
        "failure_reason": reason,
        "batch": {
            "schema_version": BATCH_VERSION,
            "schema_id": BATCH_SCHEMA_ID,
            "request": request,
            "request_sha256": request_sha256,
            "artifact_hashes": [],
        },
        "created_utc": utc_now(),
    }
    if source_root.is_dir():
        manifest["input"]["source_root_sha256"] = sha256_tree(source_root)
    _write_json(manifest_path, manifest)
    manifest["batch"]["artifact_hashes"] = _artifact_inventory(run_dir)
    _write_json(manifest_path, manifest)
    return manifest_path


def _enrich_manifest(
    manifest_path: Path,
    *,
    request: dict[str, Any],
    request_sha256: str,
    repetition: int,
    run_dir: Path,
) -> dict[str, Any]:
    manifest = read_json(manifest_path)
    if manifest.get("schema_id") != MANIFEST_SCHEMA_ID or manifest.get("protocol_id") != PROTOCOL_ID:
        raise BatchError(f"harness produced an unsupported manifest: {manifest_path}")
    if manifest.get("method") != request["method"] or manifest.get("profile") != request["profile"]:
        raise BatchError(f"harness manifest identity mismatch: {manifest_path}")
    if manifest.get("sequence") != request["sequence"]:
        raise BatchError(f"harness manifest sequence mismatch: {manifest_path}")
    manifest["repetition"] = repetition
    manifest["batch"] = {
        "schema_version": BATCH_VERSION,
        "schema_id": BATCH_SCHEMA_ID,
        "request": request,
        "request_sha256": request_sha256,
        "artifact_hashes": _artifact_inventory(run_dir),
    }
    _write_json(manifest_path, manifest)
    return manifest


def _invoke_harness_run(
    *,
    protocol_path: Path,
    source_root: Path,
    sequence: str,
    method: str,
    profile: str,
    run_dir: Path,
    command: Sequence[str],
    request: dict[str, Any],
    request_sha256: str,
    repetition: int,
    expected_frames: int,
    poll_seconds: float | None,
    timeout_seconds: float | None,
    trajectory_candidates: Sequence[str] | None,
) -> Path:
    run_dir.mkdir(parents=True, exist_ok=True)
    if any(run_dir.iterdir()):
        raise BatchError(f"run directory is not empty: {run_dir}")
    harness_command = [
        sys.executable,
        "-m",
        "benchmarks.basalt.harness",
        "run",
        "--protocol",
        str(protocol_path.resolve()),
        "--dataset-root",
        str(source_root.resolve()),
        "--sequence",
        sequence,
        "--method",
        method,
        "--profile",
        profile,
        "--output-dir",
        str(run_dir.resolve()),
        "--command-json",
        json.dumps(list(command), ensure_ascii=False),
    ]
    if poll_seconds is not None:
        harness_command.extend(["--poll-seconds", str(poll_seconds)])
    if timeout_seconds is not None:
        harness_command.extend(["--timeout-seconds", str(timeout_seconds)])
    for candidate in trajectory_candidates or ():
        harness_command.extend(["--trajectory", str(candidate)])
    started = time.perf_counter()
    try:
        completed = subprocess.run(
            harness_command,
            cwd=str(ROOT),
            capture_output=True,
            text=True,
            check=False,
            shell=False,
        )
        failure = None
    except OSError as exc:
        completed = None
        failure = str(exc)
    elapsed = time.perf_counter() - started
    invocation = {
        "argv": harness_command,
        "returncode": completed.returncode if completed is not None else -1,
        "wall_seconds": elapsed,
        "stdout": (completed.stdout if completed is not None else "")[-32768:],
        "stderr": (completed.stderr if completed is not None else "")[-32768:],
        "error": failure,
    }
    invocation_path = run_dir / "batch_invocation.json"
    _write_json(invocation_path, invocation)
    manifest_path = run_dir / "run_manifest.json"
    if not manifest_path.is_file():
        reason = "harness run produced no manifest"
        if failure:
            reason += f": {failure}"
        elif completed is not None and completed.returncode != 0:
            reason += f" (returncode {completed.returncode})"
        manifest_path = _write_synthetic_failure_manifest(
            run_dir=run_dir,
            protocol_path=protocol_path,
            protocol_sha256=request["protocol_sha256"],
            source_root=source_root,
            sequence=sequence,
            method=method,
            profile=profile,
            repetition=repetition,
            request=request,
            request_sha256=request_sha256,
            command=command,
            expected_frames=expected_frames,
            wall_seconds=elapsed,
            reason=reason,
        )
        # Preserve the outer invocation as an artifact too; the synthetic
        # helper writes its own failure record before this file is hashed.
        manifest = read_json(manifest_path)
        manifest["batch"]["artifact_hashes"] = _artifact_inventory(run_dir)
        _write_json(manifest_path, manifest)
        return manifest_path
    _enrich_manifest(
        manifest_path,
        request=request,
        request_sha256=request_sha256,
        repetition=repetition,
        run_dir=run_dir,
    )
    return manifest_path


def _validate_existing_run(
    *,
    manifest_path: Path,
    run_dir: Path,
    request: dict[str, Any],
    request_sha256: str,
    source_root: Path,
    command: Sequence[str],
    sequence: str,
) -> dict[str, Any]:
    manifest = read_json(manifest_path)
    batch = manifest.get("batch")
    if not isinstance(batch, dict) or batch.get("schema_id") != BATCH_SCHEMA_ID:
        raise BatchError(f"existing run has no batch identity; refusing stale reuse: {manifest_path}")
    if batch.get("request_sha256") != request_sha256 or batch.get("request") != request:
        raise BatchError(f"existing run request mismatch; refusing stale reuse: {manifest_path}")
    if manifest.get("repetition") != request["repetition"]:
        raise BatchError(f"existing run repetition mismatch: {manifest_path}")
    if manifest.get("method") != request["method"] or manifest.get("profile") != request["profile"]:
        raise BatchError(f"existing run method/profile mismatch: {manifest_path}")
    if manifest.get("sequence") != sequence:
        raise BatchError(f"existing run sequence mismatch: {manifest_path}")
    protocol = manifest.get("protocol", {})
    if protocol.get("sha256") != request["protocol_sha256"]:
        raise BatchError(f"existing run protocol hash mismatch: {manifest_path}")
    if not source_root.is_dir():
        raise BatchError(f"source dataset is missing; refusing stale reuse: {source_root}")
    if manifest.get("input", {}).get("source_root_sha256"):
        current_source_sha = sha256_tree(source_root)
        if current_source_sha.casefold() != str(manifest["input"]["source_root_sha256"]).casefold():
            raise BatchError(f"source dataset changed since run: {source_root}")
    staged_root = Path(str(manifest.get("input", {}).get("staged_root", "")))
    if staged_root.is_dir() and manifest.get("input", {}).get("sha256"):
        if sha256_tree(staged_root).casefold() != str(manifest["input"]["sha256"]).casefold():
            raise BatchError(f"staged input changed since run: {staged_root}")
    elif manifest.get("status") == "success":
        raise BatchError(f"staged input is missing; refusing stale reuse: {staged_root}")
    recorded = batch.get("artifact_hashes", [])
    if not isinstance(recorded, list):
        raise BatchError(f"existing run artifact inventory is invalid: {manifest_path}")
    _assert_artifact_inventory(run_dir, recorded)
    expected_argv = _rendered_engine_command(command, run_dir, sequence)
    if manifest.get("execution", {}).get("argv") != expected_argv:
        raise BatchError(f"existing run command mismatch: {manifest_path}")
    return manifest


def _evaluate_process(
    *,
    manifest_path: Path,
    ground_truth_path: Path,
    evaluation_path: Path,
    request: dict[str, Any],
    request_sha256: str,
    max_diff_ns: int,
    tum_time_unit: str,
) -> dict[str, Any]:
    if evaluation_path.is_file():
        document = read_json(evaluation_path)
        _verify_document_content_hash(document, evaluation_path)
        batch = document.get("batch")
        if not isinstance(batch, dict) or batch.get("request_sha256") != request_sha256:
            raise BatchError(f"existing evaluation request mismatch: {evaluation_path}")
        runs = document.get("runs")
        if not isinstance(runs, list) or len(runs) != 1:
            raise BatchError(f"existing per-run evaluation is malformed: {evaluation_path}")
        run = dict(runs[0])
        if run.get("manifest_sha256") != sha256_file(manifest_path):
            raise BatchError(f"existing evaluation references a changed manifest: {evaluation_path}")
        return run

    manifest = read_json(manifest_path)
    if not ground_truth_path.is_file():
        return _manifest_failure_run(manifest_path, f"ground truth does not exist after engine exit: {ground_truth_path}")
    command = [
        sys.executable,
        "-m",
        "benchmarks.basalt.harness",
        "evaluate",
        "--manifest",
        str(manifest_path.resolve()),
        "--ground-truth",
        str(ground_truth_path.resolve()),
        "--max-diff-ns",
        str(max_diff_ns),
        "--tum-time-unit",
        tum_time_unit,
        "--out",
        str(evaluation_path.resolve()),
    ]
    try:
        completed = subprocess.run(
            command,
            cwd=str(ROOT),
            capture_output=True,
            text=True,
            check=False,
            shell=False,
        )
    except OSError as exc:
        return _manifest_failure_run(manifest_path, f"post-exit evaluator could not start: {exc}")
    if not evaluation_path.is_file():
        return _manifest_failure_run(
            manifest_path,
            f"post-exit evaluator produced no result (returncode {completed.returncode})",
        )
    document = read_json(evaluation_path)
    if document.get("schema_id") != EVALUATION_SCHEMA_ID:
        return _manifest_failure_run(manifest_path, "post-exit evaluator produced an unsupported result")
    runs = document.get("runs")
    if not isinstance(runs, list) or len(runs) != 1:
        return _manifest_failure_run(manifest_path, "post-exit evaluator produced no single-run result")
    run = dict(runs[0])
    # Bind the evaluator artifact to the same request before the batch summary
    # is written.  The evaluator's own run manifest hash remains authoritative.
    document["batch"] = {
        "schema_version": BATCH_VERSION,
        "schema_id": BATCH_SCHEMA_ID,
        "request": request,
        "request_sha256": request_sha256,
        "evaluator_returncode": completed.returncode,
    }
    _bind_document_content_hash(document)
    _write_json(evaluation_path, document)
    return run


def _summary_request(
    *,
    protocol_path: Path,
    protocol_sha256: str,
    dataset_manifest_path: Path | None,
    dataset_manifest_sha256: str | None,
    dataset_root: Path,
    selected_sequences: Sequence[str],
    prefixes: Sequence[str],
    repetitions: int,
    method: str,
    profile: str,
    command: Sequence[str],
    poll_seconds: float | None,
    timeout_seconds: float | None,
    trajectory_candidates: Sequence[str] | None,
    max_diff_ns: int,
    tum_time_unit: str,
) -> tuple[dict[str, Any], str]:
    request = {
        "schema_version": BATCH_VERSION,
        "schema_id": BATCH_SCHEMA_ID,
        "protocol_id": PROTOCOL_ID,
        "protocol_path": str(protocol_path.resolve()),
        "protocol_sha256": protocol_sha256,
        "dataset_manifest_path": str(dataset_manifest_path.resolve()) if dataset_manifest_path else None,
        "dataset_manifest_sha256": dataset_manifest_sha256,
        "dataset_root": str(dataset_root.resolve()),
        "selected_sequences": list(selected_sequences),
        "prefixes": list(prefixes),
        "repetitions": repetitions,
        "method": method,
        "profile": profile,
        "command_template": list(command),
        "poll_seconds": poll_seconds,
        "timeout_seconds": timeout_seconds,
        "trajectory_candidates": list(trajectory_candidates or []),
        "evaluation": {"max_diff_ns": max_diff_ns, "tum_time_unit": tum_time_unit},
    }
    return request, _json_hash(request)


def run_batch(
    *,
    protocol_path: Path = DEFAULT_PROTOCOL,
    dataset_root: Path,
    dataset_manifest_path: Path | None = DEFAULT_DATASET_MANIFEST,
    output_root: Path,
    command: Sequence[str],
    method: str = "visloc_basalt_compat",
    profile: str = "basalt-compat",
    sequences: Iterable[str] | None = None,
    prefixes: Iterable[str] | None = None,
    repetitions: int | None = None,
    poll_seconds: float | None = None,
    timeout_seconds: float | None = None,
    trajectory_candidates: Sequence[str] | None = None,
    max_diff_ns: int | None = None,
    tum_time_unit: str = "ns",
    out_path: Path | None = None,
    dry_run: bool = False,
    resume: bool = True,
) -> Path | dict[str, Any]:
    """Run/evaluate the selected matrix, or return a dry-run plan."""

    protocol_path = Path(protocol_path).expanduser().resolve()
    dataset_root = Path(dataset_root).expanduser().resolve()
    protocol = load_protocol(protocol_path)
    if profile not in {"basalt-compat", "basalt-extended"}:
        raise BatchError(f"unsupported profile: {profile}")
    if repetitions is None:
        repetitions = int(protocol.get("evaluation_policy", {}).get("repetitions", 3))
    if repetitions < 1:
        raise BatchError("repetitions must be >= 1")
    if max_diff_ns is None:
        max_diff_ns = int(protocol.get("evaluation_policy", {}).get("max_association_delta_ns", 10_000_000))
    if max_diff_ns < 0:
        raise BatchError("max_diff_ns must be non-negative")
    if tum_time_unit not in {"ns", "s"}:
        raise BatchError("tum_time_unit must be ns or s")
    command = [str(item) for item in command]
    if not command:
        raise BatchError("engine command is required")
    protocol_sha = sha256_file(protocol_path)
    dataset_manifest: dict[str, Any] | None = None
    dataset_manifest_resolved: Path | None = None
    dataset_manifest_sha: str | None = None
    if dataset_manifest_path is not None:
        dataset_manifest_resolved = Path(dataset_manifest_path).expanduser().resolve()
        if dataset_manifest_resolved.is_file():
            dataset_manifest = read_json(dataset_manifest_resolved)
            if dataset_manifest.get("protocol_id") not in {None, PROTOCOL_ID}:
                raise BatchError(f"dataset manifest protocol mismatch: {dataset_manifest_resolved}")
            _validate_dataset_manifest_protocol_binding(
                dataset_manifest,
                manifest_path=dataset_manifest_resolved,
                protocol_path=protocol_path,
                protocol_sha256=protocol_sha,
            )
            dataset_manifest_sha = sha256_file(dataset_manifest_resolved)
        else:
            # A dry-run is still a readiness check: silently dropping the
            # frozen dataset manifest would produce a plausible 33-cell plan
            # with zero expected-frame metadata and no protocol binding.
            raise BatchError(f"dataset manifest does not exist: {dataset_manifest_resolved}")
    protocol_sequences = [str(item) for item in protocol.get("sequences", []) if isinstance(item, str)]
    selected = select_sequences(protocol_sequences, sequences=sequences, prefixes=prefixes)
    prefixes_flat = _flatten_values(prefixes)
    summary_request, summary_request_sha = _summary_request(
        protocol_path=protocol_path,
        protocol_sha256=protocol_sha,
        dataset_manifest_path=dataset_manifest_resolved,
        dataset_manifest_sha256=dataset_manifest_sha,
        dataset_root=dataset_root,
        selected_sequences=selected,
        prefixes=prefixes_flat,
        repetitions=repetitions,
        method=method,
        profile=profile,
        command=command,
        poll_seconds=poll_seconds,
        timeout_seconds=timeout_seconds,
        trajectory_candidates=trajectory_candidates,
        max_diff_ns=max_diff_ns,
        tum_time_unit=tum_time_unit,
    )
    plan: list[dict[str, Any]] = []
    records = _manifest_sequence_map(dataset_manifest)
    for sequence in selected:
        record = records.get(sequence, {})
        family = _sequence_family(sequence, record)
        # Validate the post-exit evaluator target during planning as well as
        # execution.  A malformed dataset manifest must not yield a plausible
        # dry-run plan that later evaluates outside the sequence root.
        _ground_truth_relative_path(dataset_manifest, sequence, protocol)
        source_root = resolve_sequence_root(dataset_root, sequence, family=family, require_exists=not dry_run)
        for repetition in range(1, repetitions + 1):
            run_dir = run_directory(output_root, method, profile, sequence, repetition)
            request, request_sha = make_run_request(
                protocol_path=protocol_path,
                protocol_sha256=protocol_sha,
                dataset_manifest_path=dataset_manifest_resolved,
                dataset_manifest_sha256=dataset_manifest_sha,
                dataset_root=dataset_root,
                source_root=source_root,
                sequence=sequence,
                method=method,
                profile=profile,
                repetition=repetition,
                command=command,
                poll_seconds=poll_seconds,
                timeout_seconds=timeout_seconds,
                trajectory_candidates=trajectory_candidates,
                max_diff_ns=max_diff_ns,
                tum_time_unit=tum_time_unit,
            )
            plan.append(
                {
                    "sequence": sequence,
                    "family": family,
                    "repetition": repetition,
                    "source_root": str(source_root),
                    "run_dir": str(run_dir),
                    "request_sha256": request_sha,
                    "expected_input_frames": _frame_count(record),
                }
            )
    if dry_run:
        return {
            "schema_version": BATCH_VERSION,
            "schema_id": BATCH_SCHEMA_ID,
            "protocol_id": PROTOCOL_ID,
            "request": summary_request,
            "request_sha256": summary_request_sha,
            "run_count": len(plan),
            "runs": plan,
            "dry_run": True,
        }

    output_root = Path(output_root).expanduser().resolve()
    eval_result_path = (
        Path(out_path).expanduser().resolve()
        if out_path is not None
        else output_root / _safe_component(method, "method") / _safe_component(profile, "profile") / "evaluation_result.json"
    )
    run_results: list[dict[str, Any]] = []
    ground_truth_sources: list[dict[str, Any]] = []
    ground_truth_seen: set[str] = set()
    for item in plan:
        sequence = str(item["sequence"])
        repetition = int(item["repetition"])
        source_root = Path(str(item["source_root"]))
        run_dir = Path(str(item["run_dir"]))
        request, request_sha = make_run_request(
            protocol_path=protocol_path,
            protocol_sha256=protocol_sha,
            dataset_manifest_path=dataset_manifest_resolved,
            dataset_manifest_sha256=dataset_manifest_sha,
            dataset_root=dataset_root,
            source_root=source_root,
            sequence=sequence,
            method=method,
            profile=profile,
            repetition=repetition,
            command=command,
            poll_seconds=poll_seconds,
            timeout_seconds=timeout_seconds,
            trajectory_candidates=trajectory_candidates,
            max_diff_ns=max_diff_ns,
            tum_time_unit=tum_time_unit,
        )
        manifest_path = run_dir / "run_manifest.json"
        if manifest_path.is_file():
            if not resume:
                raise BatchError(f"existing run found with --no-resume: {run_dir}")
            _validate_existing_run(
                manifest_path=manifest_path,
                run_dir=run_dir,
                request=request,
                request_sha256=request_sha,
                source_root=source_root,
                command=command,
                sequence=sequence,
            )
        elif run_dir.exists() and any(run_dir.iterdir()):
            raise BatchError(f"run directory has no manifest; refusing stale reuse: {run_dir}")
        else:
            manifest_path = _invoke_harness_run(
                protocol_path=protocol_path,
                source_root=source_root,
                sequence=sequence,
                method=method,
                profile=profile,
                run_dir=run_dir,
                command=command,
                request=request,
                request_sha256=request_sha,
                repetition=repetition,
                expected_frames=int(item["expected_input_frames"]),
                poll_seconds=poll_seconds,
                timeout_seconds=timeout_seconds,
                trajectory_candidates=trajectory_candidates,
            )
        gt_path = _ground_truth_path(source_root, dataset_manifest, sequence, protocol)
        evaluation_path = run_dir / "evaluation_result.json"
        run_result = _evaluate_process(
            manifest_path=manifest_path,
            ground_truth_path=gt_path,
            evaluation_path=evaluation_path,
            request=request,
            request_sha256=request_sha,
            max_diff_ns=max_diff_ns,
            tum_time_unit=tum_time_unit,
        )
        run_result["repetition"] = repetition
        run_result["artifact_hashes"] = read_json(manifest_path).get("batch", {}).get("artifact_hashes", [])
        if evaluation_path.is_file():
            run_result["evaluation_result_sha256"] = sha256_file(evaluation_path)
        run_results.append(run_result)
        if sequence not in ground_truth_seen:
            source_record = {"sequence": sequence, "path": str(gt_path)}
            if gt_path.is_file():
                source_record["sha256"] = sha256_file(gt_path)
            else:
                source_record["sha256"] = None
            ground_truth_sources.append(source_record)
            ground_truth_seen.add(sequence)

    result = {
        "schema_version": 1,
        "schema_id": EVALUATION_SCHEMA_ID,
        "protocol_id": PROTOCOL_ID,
        "ground_truth_used_after_engine_exit": True,
        "batch": {
            "schema_version": BATCH_VERSION,
            "schema_id": BATCH_SCHEMA_ID,
            "request": summary_request,
            "request_sha256": summary_request_sha,
            "run_count": len(run_results),
            "failure_policy": "record_dnf_and_keep_full_denominator",
        },
        "ground_truth_sources": ground_truth_sources,
        "runs": run_results,
        "aggregate": aggregate_results(run_results),
        "created_utc": utc_now(),
    }
    _bind_document_content_hash(result)
    if eval_result_path.exists() and resume:
        try:
            old = read_json(eval_result_path)
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            raise BatchError(f"existing batch evaluation is unreadable: {eval_result_path}: {exc}") from exc
        _verify_document_content_hash(old, eval_result_path)
        old_batch = old.get("batch", {})
        if isinstance(old_batch, dict) and old_batch.get("request_sha256") not in {None, summary_request_sha}:
            raise BatchError(f"existing batch evaluation request mismatch: {eval_result_path}")
    elif eval_result_path.exists() and not resume:
        raise BatchError(f"existing batch evaluation found with --no-resume: {eval_result_path}")
    _write_json(eval_result_path, result)
    return eval_result_path


def _parse_command(args: argparse.Namespace) -> list[str]:
    if args.command_json is not None:
        try:
            value = json.loads(args.command_json)
        except json.JSONDecodeError as exc:
            raise BatchError(f"--command-json is not valid JSON: {exc}") from exc
        if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
            raise BatchError("--command-json must be a JSON array of strings")
        return list(value)
    value = list(args.command or [])
    if value and value[0] == "--":
        value.pop(0)
    if not value:
        raise BatchError("engine command is required")
    return value


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    run = parser.add_argument_group("batch run")
    run.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    run.add_argument("--dataset-root", type=Path, required=True)
    run.add_argument("--dataset-manifest", type=Path, default=DEFAULT_DATASET_MANIFEST)
    run.add_argument("--output-root", type=Path, required=True)
    run.add_argument("--out", type=Path, help="batch evaluation_result.json path")
    run.add_argument("--sequence", "--sequences", action="append", help="exact sequence; repeat or comma-separate")
    run.add_argument(
        "--prefix",
        "--prefixes",
        "--sequence-prefix",
        action="append",
        help="sequence-name prefix, e.g. MH, V1, or V2",
    )
    run.add_argument("--all", action="store_true", help="explicitly request all protocol sequences")
    run.add_argument("--repetitions", type=int, help="repetitions (defaults to protocol, normally 3)")
    run.add_argument("--method", default="visloc_basalt_compat")
    run.add_argument("--profile", choices=("basalt-compat", "basalt-extended"), default="basalt-compat")
    run.add_argument("--command-json", help="engine argv JSON; supports {input_root}, {output_root}, {sequence}")
    run.add_argument("--command", nargs=argparse.REMAINDER, help="engine argv after --command")
    run.add_argument("--poll-seconds", type=float)
    run.add_argument("--timeout-seconds", type=float)
    run.add_argument("--trajectory", action="append", help="relative trajectory candidate in engine output")
    run.add_argument("--max-diff-ns", type=int)
    run.add_argument("--tum-time-unit", choices=("ns", "s"), default="ns")
    run.add_argument("--dry-run", action="store_true")
    run.add_argument("--no-resume", action="store_true", help="refuse existing run/evaluation artifacts")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        if args.all and (args.sequence or args.prefix):
            parser.error("--all cannot be combined with --sequence/--prefix")
        command = _parse_command(args)
        result = run_batch(
            protocol_path=args.protocol,
            dataset_root=args.dataset_root,
            dataset_manifest_path=args.dataset_manifest,
            output_root=args.output_root,
            out_path=args.out,
            command=command,
            method=args.method,
            profile=args.profile,
            sequences=None if args.all else args.sequence,
            prefixes=None if args.all else args.prefix,
            repetitions=args.repetitions,
            poll_seconds=args.poll_seconds,
            timeout_seconds=args.timeout_seconds,
            trajectory_candidates=args.trajectory,
            max_diff_ns=args.max_diff_ns,
            tum_time_unit=args.tum_time_unit,
            dry_run=args.dry_run,
            resume=not args.no_resume,
        )
        if isinstance(result, dict):
            print(json.dumps(result, indent=2, sort_keys=True))
        else:
            print(result)
        return 0
    except (BatchError, OSError, ValueError, json.JSONDecodeError) as exc:
        parser.error(str(exc))
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
