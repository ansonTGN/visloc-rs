"""Run the pinned Basalt native executable inside WSL with Linux RSS sampling.

``run_clonefree_perf.py`` is the public entry point.  This small adapter is
deliberately kept separate from the Windows runner so that the native process
tree (rather than only the ``wsl.exe`` shim) is measured.  It receives only
argv values, removes GT-looking environment variables, omits ``--marg-data``
and writes a monitor record atomically after the native process exits.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import time
from pathlib import Path
from typing import Any


UPSTREAM_COMMIT = "0f3b2b52c807f70ff4e2973ce253c73329eea7bc"
UPSTREAM_TREE_SHA1 = "b7afb830d82b45b8209cf784ad9744025d838411"
POLL_SECONDS = 0.1


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def is_gt_environment_key(key: str) -> bool:
    upper = key.upper()
    return (
        "GROUND_TRUTH" in upper
        or upper in {"GT", "GT_PATH", "GT_FILE", "GT_ROOT", "EUROC_GT", "EUROC_GT_PATH"}
        or upper.endswith(("_GT_PATH", "_GT_FILE", "_GT_ROOT"))
    )


def filtered_environment() -> tuple[dict[str, str], list[str]]:
    environment = os.environ.copy()
    removed = sorted(key for key in environment if is_gt_environment_key(key))
    for key in removed:
        environment.pop(key, None)
    return environment, removed


def _contains_gt_token(value: str) -> bool:
    lowered = str(value).casefold()
    return any(token in lowered for token in ("ground_truth", "groundtruth", "gt_path", "gt_file", "gt_csv"))


def _proc_ppid_rss(pid: int) -> tuple[int, int] | None:
    """Return ``(parent_pid, resident_bytes)`` from Linux ``/proc``."""

    try:
        status = (Path("/proc") / str(pid) / "status").read_text(encoding="utf-8")
    except (FileNotFoundError, PermissionError, OSError):
        return None
    rss = 0
    for line in status.splitlines():
        if line.startswith("VmRSS:"):
            fields = line.split()
            if len(fields) >= 2:
                try:
                    rss = int(fields[1]) * 1024
                except ValueError:
                    rss = 0
            break
    try:
        stat = (Path("/proc") / str(pid) / "stat").read_text(encoding="utf-8")
        after_comm = stat.rsplit(") ", 1)[1].split()
        return int(after_comm[1]), rss
    except (FileNotFoundError, PermissionError, OSError, IndexError, ValueError):
        return None


def process_tree_rss(root_pid: int) -> int:
    table: dict[int, tuple[int, int]] = {}
    proc_root = Path("/proc")
    try:
        entries = list(proc_root.iterdir())
    except OSError:
        return 0
    for entry in entries:
        if not entry.name.isdigit():
            continue
        record = _proc_ppid_rss(int(entry.name))
        if record is not None:
            table[int(entry.name)] = record
    descendants = {root_pid}
    changed = True
    while changed:
        changed = False
        for pid, (parent, _) in table.items():
            if parent in descendants and pid not in descendants:
                descendants.add(pid)
                changed = True
    return sum(table.get(pid, (0, 0))[1] for pid in descendants)


def _git_value(checkout: Path, *args: str) -> str:
    return subprocess.check_output(
        ["git", "-C", str(checkout), *args], text=True, stderr=subprocess.STDOUT
    ).strip()


def _atomic_write_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--input-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--monitor-json", type=Path, required=True)
    parser.add_argument("--checkout", type=Path, required=True)
    parser.add_argument(
        "--max-frames",
        type=int,
        help="optional frame limit; omit for a complete sequence run",
    )
    parser.add_argument("--num-threads", type=int, default=1)
    parser.add_argument("--poll-seconds", type=float, default=POLL_SECONDS)
    args = parser.parse_args()

    binary = args.binary
    config = args.config
    calibration = args.calibration
    input_root = args.input_root
    output_root = args.output_root
    checkout = args.checkout
    monitor_path = args.monitor_json
    for path, label in (
        (binary, "native binary"),
        (config, "native config"),
        (calibration, "native calibration"),
        (checkout, "native checkout"),
        (input_root / "mav0", "EuRoC sensor root"),
    ):
        if not path.exists():
            raise SystemExit(f"{label} is missing: {path}")
    if not binary.is_file() or not os.access(binary, os.X_OK):
        raise SystemExit(f"native binary is not executable: {binary}")
    if args.max_frames is not None and args.max_frames < 1:
        raise SystemExit("--max-frames must be positive")
    if args.num_threads != 1:
        raise SystemExit("canonical native mode requires --num-threads 1")
    if args.poll_seconds <= 0:
        raise SystemExit("--poll-seconds must be positive")
    output_root.mkdir(parents=True, exist_ok=True)

    commit = _git_value(checkout, "rev-parse", "HEAD")
    tree_sha1 = _git_value(checkout, "rev-parse", "HEAD^{tree}")
    if commit != UPSTREAM_COMMIT:
        raise SystemExit(f"native checkout commit is not pinned: {commit}")
    if tree_sha1 != UPSTREAM_TREE_SHA1:
        raise SystemExit(f"native checkout tree is not pinned: {tree_sha1}")
    try:
        dirty = _git_value(checkout, "status", "--short").splitlines()
    except subprocess.CalledProcessError as exc:
        raise SystemExit(f"unable to inspect native checkout status: {exc}") from exc
    if dirty:
        raise SystemExit(f"native checkout is dirty: {checkout}: {dirty[:8]}")

    command = [
        str(binary),
        "--dataset-path",
        str(input_root),
        "--cam-calib",
        str(calibration),
        "--dataset-type",
        "euroc",
        "--show-gui",
        "0",
        "--config-path",
        str(config),
        "--save-trajectory",
        "euroc",
        "--num-threads",
        "1",
    ]
    if args.max_frames is not None:
        command.extend(("--max-frames", str(args.max_frames)))
    if any(_contains_gt_token(token) for token in command):
        raise SystemExit("native command contains a ground-truth token")
    # Intentionally no --marg-data: this is the native no-output performance
    # mode.  Keeping the argv literal makes the omission auditable.
    environment, removed_gt_env_keys = filtered_environment()
    started = time.perf_counter()
    process = subprocess.Popen(command, cwd=output_root, env=environment)
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
        "schema": "basalt.native_wsl_monitor.v1",
        "command": command,
        "returncode": returncode,
        "wall_seconds": finished - started,
        "peak_process_tree_rss_bytes": peak_rss,
        "resource_poll_seconds": args.poll_seconds,
        "runtime_profile": "native_wsl_linux",
        "rss_measurement_domain": "linux-proc-process-tree",
        "rss_measurement_source": "native_wsl_runner:/proc",
        "rss_authoritative": True,
        "platform": "wsl-linux",
        "output_policy": "trajectory_only_lean",
        "no_marg_data": True,
        "no_trace": True,
        "gt_environment_keys_removed": removed_gt_env_keys,
        "ground_truth_command_tokens": False,
        "output_root": str(output_root),
        "native_binding": {
            "checkout": str(checkout),
            "commit": commit,
            "tree_sha1": tree_sha1,
            "checkout_dirty": bool(dirty),
            "dirty_paths": dirty,
            "binary": {"path": str(binary), "bytes": binary.stat().st_size, "sha256": sha256(binary)},
            "config": {"path": str(config), "bytes": config.stat().st_size, "sha256": sha256(config)},
            "calibration": {
                "path": str(calibration),
                "bytes": calibration.stat().st_size,
                "sha256": sha256(calibration),
            },
            "wrapper": {
                "path": str(Path(__file__).resolve()),
                "bytes": Path(__file__).stat().st_size,
                "sha256": sha256(Path(__file__).resolve()),
            },
        },
    }
    _atomic_write_json(monitor_path, record)
    return int(returncode)


if __name__ == "__main__":
    raise SystemExit(main())
