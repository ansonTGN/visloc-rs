"""Run the Rust Linux measurement twin under WSL with authoritative RSS.

This adapter is intentionally independent of the Windows/MSVC launcher.  The
Phase 6 coordinator uses it only when a Rust WSL executable, a local
provenance record, and a 52/80/400 exactness certificate are all bound.  The
engine receives a sensor-only dataset and the lean output flags; ground truth
is never passed through argv or the child environment.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import time
from pathlib import Path
from typing import Any, Mapping


POLL_SECONDS = 0.25
PROVENANCE_SCHEMA = "basalt.phase6.rust_wsl_provenance.v1"
EXACTNESS_SCHEMA = "basalt.phase6.rust_wsl_exactness.v1"
RUNTIME_PROFILE = "rust_wsl_linux"
RSS_DOMAIN = "linux-proc-process-tree"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _valid_sha(value: Any) -> bool:
    if not isinstance(value, str) or len(value) != 64:
        return False
    return all(char in "0123456789abcdefABCDEF" for char in value)


def is_gt_environment_key(key: str) -> bool:
    normalized = "".join(char.lower() if char.isalnum() else "_" for char in str(key)).strip("_")
    return normalized in {"gt", "gt_root", "euroc_gt"} or any(
        marker in normalized for marker in ("ground_truth", "groundtruth", "gt_path", "gt_file", "gt_csv")
    )


def filtered_environment() -> tuple[dict[str, str], list[str]]:
    environment = os.environ.copy()
    removed = sorted(
        [key for key in environment if is_gt_environment_key(key)], key=str.casefold
    )
    for key in removed:
        environment.pop(key, None)
    return environment, removed


def _contains_gt_token(value: Any) -> bool:
    lowered = str(value).casefold()
    return any(token in lowered for token in ("ground_truth", "groundtruth", "gt_path", "gt_file", "gt_csv"))


def _proc_ppid_rss(pid: int) -> tuple[int, int] | None:
    try:
        status = (Path("/proc") / str(pid) / "status").read_text(encoding="utf-8")
    except (FileNotFoundError, PermissionError, OSError):
        return None
    resident = 0
    for line in status.splitlines():
        if line.startswith("VmRSS:"):
            fields = line.split()
            if len(fields) >= 2:
                try:
                    resident = int(fields[1]) * 1024
                except ValueError:
                    resident = 0
            break
    try:
        stat = (Path("/proc") / str(pid) / "stat").read_text(encoding="utf-8")
        after_comm = stat.rsplit(") ", 1)[1].split()
        return int(after_comm[1]), resident
    except (FileNotFoundError, PermissionError, OSError, IndexError, ValueError):
        return None


def process_tree_rss(root_pid: int) -> int:
    table: dict[int, tuple[int, int]] = {}
    try:
        entries = list(Path("/proc").iterdir())
    except OSError:
        return 0
    for entry in entries:
        if entry.name.isdigit():
            record = _proc_ppid_rss(int(entry.name))
            if record is not None:
                table[int(entry.name)] = record
    descendants = {root_pid}
    changed = True
    while changed:
        changed = False
        for pid, (parent, _resident) in table.items():
            if parent in descendants and pid not in descendants:
                descendants.add(pid)
                changed = True
    return sum(table.get(pid, (0, 0))[1] for pid in descendants)


def _atomic_write_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def _load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise SystemExit(f"invalid JSON binding {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise SystemExit(f"JSON binding must be an object: {path}")
    return value


def _remote_binding(provenance: Mapping[str, Any], key: str, label: str) -> Mapping[str, Any]:
    value = provenance.get(key)
    if not isinstance(value, Mapping) or not isinstance(value.get("path"), str):
        raise SystemExit(f"Rust WSL provenance is missing {label} binding")
    if not str(value["path"]).startswith("/") or not _valid_sha(value.get("sha256")):
        raise SystemExit(f"Rust WSL provenance has an invalid {label} binding")
    try:
        size = int(value.get("bytes"))
    except (TypeError, ValueError) as exc:
        raise SystemExit(f"Rust WSL provenance has an invalid {label} size") from exc
    if size < 0:
        raise SystemExit(f"Rust WSL provenance has a negative {label} size")
    return {"path": str(value["path"]), "bytes": size, "sha256": str(value["sha256"]).casefold()}


def _validate_bindings(
    *,
    binary: Path,
    config: Path,
    calibration: Path,
    provenance_path: Path | None,
    certificate_path: Path | None,
) -> dict[str, Any]:
    if provenance_path is None or certificate_path is None:
        raise SystemExit("Rust WSL runner requires provenance and exactness certificate paths")
    provenance = _load_json(provenance_path)
    if provenance.get("schema_id") != PROVENANCE_SCHEMA or provenance.get("runtime_profile") != RUNTIME_PROFILE:
        raise SystemExit("Rust WSL provenance schema/profile mismatch")
    remote = {
        "binary": _remote_binding(provenance, "binary", "binary"),
        "config": _remote_binding(provenance, "config", "config"),
        "calibration": _remote_binding(provenance, "calibration", "calibration"),
    }
    for local, key in ((binary, "binary"), (config, "config"), (calibration, "calibration")):
        if str(local) != remote[key]["path"]:
            raise SystemExit(f"Rust WSL {key} path differs from provenance")
        if not local.is_file():
            raise SystemExit(f"Rust WSL {key} is missing: {local}")
        actual_sha = sha256(local)
        if actual_sha.casefold() != str(remote[key]["sha256"]).casefold() or local.stat().st_size != remote[key]["bytes"]:
            raise SystemExit(f"Rust WSL {key} content differs from provenance")
    source_sha = provenance.get("source_sha256")
    if source_sha is None and isinstance(provenance.get("source"), Mapping):
        source_sha = provenance["source"].get("sha256")
    if not _valid_sha(source_sha):
        raise SystemExit("Rust WSL provenance has no source SHA-256")
    target_triple = provenance.get("target_triple")
    if not isinstance(target_triple, str) or not target_triple.strip():
        raise SystemExit("Rust WSL provenance has no target triple")
    certificate = _load_json(certificate_path)
    if certificate.get("schema_id") != EXACTNESS_SCHEMA or str(certificate.get("status", "")).casefold() != "pass":
        raise SystemExit("Rust WSL exactness certificate is not a pass")
    if str(certificate.get("rust_wsl_provenance_sha256", "")).casefold() != sha256(provenance_path).casefold():
        raise SystemExit("Rust WSL exactness certificate is not bound to provenance")
    expected_cert = {
        "rust_wsl_executable_sha256": remote["binary"]["sha256"],
        "config_sha256": remote["config"]["sha256"],
        "calibration_sha256": remote["calibration"]["sha256"],
        "source_sha256": str(source_sha).casefold(),
    }
    for key, expected in expected_cert.items():
        if not _valid_sha(certificate.get(key)) or str(certificate[key]).casefold() != expected:
            raise SystemExit(f"Rust WSL certificate binding mismatch: {key}")
    if certificate.get("required_frames") != [52, 80, 400]:
        raise SystemExit("Rust WSL exactness certificate must cover frames 52, 80, and 400")
    frames = certificate.get("frames")
    if not isinstance(frames, Mapping):
        raise SystemExit("Rust WSL exactness certificate has no frame results")
    for frame in (52, 80, 400):
        item = frames.get(str(frame))
        if not isinstance(item, Mapping) or any(item.get(key) is not True for key in ("trajectory_exact", "lifecycle_exact", "forbidden_outputs_absent")):
            raise SystemExit(f"Rust WSL exactness certificate frame {frame} is incomplete")
    return {
        "binary": dict(remote["binary"]),
        "config": dict(remote["config"]),
        "calibration": dict(remote["calibration"]),
        "source_sha256": str(source_sha).casefold(),
        "target_triple": target_triple,
        "provenance_sha256": sha256(provenance_path),
        "exactness_certificate_sha256": sha256(certificate_path),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--input-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--monitor-json", type=Path, required=True)
    parser.add_argument("--provenance-json", type=Path, required=True)
    parser.add_argument("--exactness-certificate", type=Path, required=True)
    parser.add_argument("--max-frames", type=int)
    parser.add_argument("--num-threads", type=int, default=1)
    parser.add_argument("--poll-seconds", type=float, default=POLL_SECONDS)
    args = parser.parse_args()
    if args.max_frames is not None and args.max_frames < 1:
        raise SystemExit("--max-frames must be positive")
    if args.num_threads != 1:
        raise SystemExit("canonical Rust WSL mode requires --num-threads 1")
    if args.poll_seconds <= 0:
        raise SystemExit("--poll-seconds must be positive")
    if not args.input_root.joinpath("mav0").is_dir():
        raise SystemExit(f"Rust WSL sensor root is missing: {args.input_root / 'mav0'}")
    binary = args.binary
    if not binary.is_file() or not os.access(binary, os.X_OK):
        raise SystemExit(f"Rust WSL binary is not executable: {binary}")
    binding = _validate_bindings(
        binary=binary,
        config=args.config,
        calibration=args.calibration,
        provenance_path=args.provenance_json,
        certificate_path=args.exactness_certificate,
    )
    args.output_root.mkdir(parents=True, exist_ok=True)
    command = [
        str(binary),
        "--euroc-dir",
        str(args.input_root),
        "--calibration",
        str(args.calibration),
        "--config",
        str(args.config),
        "--out-dir",
        str(args.output_root),
        "--no-marg-data",
        "--no-trace",
    ]
    if args.max_frames is not None:
        command.extend(("--max-frames", str(args.max_frames)))
    if any(_contains_gt_token(token) for token in command):
        raise SystemExit("Rust WSL command contains a ground-truth token")
    environment, removed_gt_env_keys = filtered_environment()
    if any(is_gt_environment_key(key) for key in environment):
        raise SystemExit("ground-truth environment key survived Rust WSL sanitization")
    started = time.perf_counter()
    process = subprocess.Popen(command, cwd=args.output_root, env=environment)
    peak_rss = 0
    while True:
        peak_rss = max(peak_rss, process_tree_rss(process.pid))
        try:
            returncode = process.wait(timeout=args.poll_seconds)
            break
        except subprocess.TimeoutExpired:
            continue
    peak_rss = max(peak_rss, process_tree_rss(process.pid))
    finished = time.perf_counter()
    record = {
        "schema": "basalt.rust_wsl_monitor.v1",
        "runtime_profile": RUNTIME_PROFILE,
        "command": command,
        "returncode": returncode,
        "wall_seconds": finished - started,
        "peak_process_tree_rss_bytes": peak_rss,
        "resource_poll_seconds": args.poll_seconds,
        "platform": "wsl-linux",
        "output_policy": "trajectory_only_lean",
        "no_marg_data": True,
        "no_trace": True,
        "gt_environment_keys_removed": removed_gt_env_keys,
        "ground_truth_command_tokens": False,
        "rss_measurement_domain": RSS_DOMAIN,
        "rss_measurement_source": "rust_wsl_runner:/proc",
        "rss_authoritative": True,
        "output_root": str(args.output_root),
        "rust_wsl_binding": binding,
    }
    _atomic_write_json(args.monitor_json, record)
    return int(returncode)


if __name__ == "__main__":
    raise SystemExit(main())
