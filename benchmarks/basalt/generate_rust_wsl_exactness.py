"""Generate fail-closed Rust Windows/WSL exactness and provenance bindings."""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import subprocess
from pathlib import Path
from typing import Any


PROVENANCE_SCHEMA = "basalt.phase6.rust_wsl_provenance.v1"
EXACTNESS_SCHEMA = "basalt.phase6.rust_wsl_exactness.v1"
RUNTIME_PROFILE = "rust_wsl_linux"
REQUIRED_FRAMES = (52, 80, 400)
FORBIDDEN_OUTPUTS = ("trace.jsonl", "marg_data", "timing_breakdown.json")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def binding(path: Path, *, recorded_path: str | None = None) -> dict[str, Any]:
    resolved = path.resolve()
    if not resolved.is_file():
        raise ValueError(f"required file is missing: {resolved}")
    return {
        "path": recorded_path or str(resolved),
        "bytes": resolved.stat().st_size,
        "sha256": sha256(resolved),
    }


def tracked_source_binding(root: Path) -> dict[str, Any]:
    command = [
        "git",
        "ls-files",
        "--",
        "Cargo.lock",
        "Cargo.toml",
        "pipelines/basalt/Cargo.toml",
        "pipelines/basalt/src",
    ]
    result = subprocess.run(command, cwd=root, check=True, capture_output=True, text=True)
    paths = sorted(line for line in result.stdout.splitlines() if line)
    if not paths:
        raise ValueError("tracked Basalt source closure is empty")
    records = []
    aggregate = hashlib.sha256()
    for relative in paths:
        item = binding(root / relative)
        record = {
            "path": relative.replace("\\", "/"),
            "bytes": item["bytes"],
            "sha256": item["sha256"],
        }
        records.append(record)
        aggregate.update(
            f"{record['sha256']} {record['bytes']} {record['path']}\n".encode("utf-8")
        )
    return {
        "algorithm": "sha256-of-sorted-sha256-size-path-lines-v1",
        "sha256": aggregate.hexdigest(),
        "file_count": len(records),
        "files": records,
    }


def parse_summary(path: Path, expected_frames: int) -> dict[str, Any]:
    values: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        key, separator, value = line.partition("=")
        if separator:
            values[key] = value
    processed = int(values.get("frames_processed", "-1"))
    requested = int(values.get("frames_requested", "-1"))
    if processed != expected_frames or requested != expected_frames:
        raise ValueError(
            f"{path}: expected {expected_frames} frames, got requested={requested}, processed={processed}"
        )
    return {
        "frames_requested": requested,
        "frames_processed": processed,
        "observations_emitted": int(values["observations_emitted"]),
        "imu_samples_delivered": int(values["imu_samples_delivered"]),
        "sha256": sha256(path),
    }


def frame_result(runs: Path, frame: int) -> dict[str, Any]:
    windows = runs / f"windows{frame}"
    linux = runs / f"linux{frame}"
    windows_summary = parse_summary(windows / "summary.txt", frame)
    linux_summary = parse_summary(linux / "summary.txt", frame)
    if {
        key: windows_summary[key]
        for key in ("frames_requested", "frames_processed", "observations_emitted", "imu_samples_delivered")
    } != {
        key: linux_summary[key]
        for key in ("frames_requested", "frames_processed", "observations_emitted", "imu_samples_delivered")
    }:
        raise ValueError(f"frame {frame}: Windows/Linux summary counts differ")

    artifacts: dict[str, Any] = {}
    exact: dict[str, bool] = {}
    for name in ("trajectory.csv", "trajectory.tum", "lifecycle.jsonl"):
        windows_binding = binding(windows / name)
        linux_binding = binding(linux / name)
        matches = (
            windows_binding["bytes"] == linux_binding["bytes"]
            and windows_binding["sha256"] == linux_binding["sha256"]
        )
        if not matches:
            raise ValueError(f"frame {frame}: {name} differs between Windows and Linux")
        artifacts[name] = {"windows": windows_binding, "linux": linux_binding}
        exact[name] = matches

    forbidden_present = {
        platform: [name for name in FORBIDDEN_OUTPUTS if (directory / name).exists()]
        for platform, directory in (("windows", windows), ("linux", linux))
    }
    if any(forbidden_present.values()):
        raise ValueError(f"frame {frame}: forbidden outputs present: {forbidden_present}")
    return {
        "trajectory_exact": exact["trajectory.csv"] and exact["trajectory.tum"],
        "lifecycle_exact": exact["lifecycle.jsonl"],
        "forbidden_outputs_absent": True,
        "summaries": {"windows": windows_summary, "linux": linux_summary},
        "artifacts": artifacts,
        "forbidden_outputs": list(FORBIDDEN_OUTPUTS),
    }


def write_new_json(path: Path, value: dict[str, Any]) -> None:
    if path.exists():
        raise ValueError(f"refusing to overwrite: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8", newline="\n")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--runs", type=Path, required=True)
    parser.add_argument("--windows-executable", type=Path, required=True)
    parser.add_argument("--linux-executable", type=Path, required=True)
    parser.add_argument("--linux-executable-path", required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--linux-config-path", required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--linux-calibration-path", required=True)
    parser.add_argument("--provenance-output", type=Path, required=True)
    parser.add_argument("--certificate-output", type=Path, required=True)
    parser.add_argument("--target-triple", default="x86_64-unknown-linux-gnu")
    parser.add_argument("--rustc-version", required=True)
    args = parser.parse_args()

    root = args.root.resolve()
    runs = args.runs.resolve()
    source = tracked_source_binding(root)
    linux_binary = binding(args.linux_executable, recorded_path=args.linux_executable_path)
    config = binding(args.config, recorded_path=args.linux_config_path)
    calibration = binding(args.calibration, recorded_path=args.linux_calibration_path)
    windows_binary = binding(args.windows_executable)
    head = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=root, check=True, capture_output=True, text=True
    ).stdout.strip()
    dirty = subprocess.run(
        ["git", "status", "--porcelain=v1", "--untracked-files=no"],
        cwd=root,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.splitlines()
    allowed_dirty = {"benchmarks/basalt/generate_rust_wsl_exactness.py"}
    unexpected_dirty = [line for line in dirty if line[3:].replace("\\", "/") not in allowed_dirty]
    if unexpected_dirty:
        raise ValueError(f"unexpected tracked source changes: {unexpected_dirty}")

    created = dt.datetime.now(dt.timezone.utc).replace(microsecond=0).isoformat()
    provenance = {
        "schema_id": PROVENANCE_SCHEMA,
        "runtime_profile": RUNTIME_PROFILE,
        "created_utc": created,
        "binary": linux_binary,
        "config": config,
        "calibration": calibration,
        "source": source,
        "source_sha256": source["sha256"],
        "git": {"head": head, "unexpected_tracked_changes": []},
        "target_triple": args.target_triple,
        "toolchain": {"rustc": args.rustc_version},
        "build": {
            "profile": "release",
            "features": ["basalt-lm-workspace-reuse"],
            "rustflags": "-C target-feature=+avx2,+fma",
        },
        "wsl": {"rss_domain": "linux-proc-process-tree"},
    }
    provenance_output = args.provenance_output.resolve()
    write_new_json(provenance_output, provenance)

    frames = {str(frame): frame_result(runs, frame) for frame in REQUIRED_FRAMES}
    certificate = {
        "schema_id": EXACTNESS_SCHEMA,
        "status": "pass",
        "created_utc": created,
        "required_frames": list(REQUIRED_FRAMES),
        "rust_wsl_provenance_sha256": sha256(provenance_output),
        "rust_wsl_executable_sha256": linux_binary["sha256"],
        "rust_msvc_executable_sha256": windows_binary["sha256"],
        "config_sha256": config["sha256"],
        "calibration_sha256": calibration["sha256"],
        "source_sha256": source["sha256"],
        "windows_executable": windows_binary,
        "frames": frames,
    }
    certificate_output = args.certificate_output.resolve()
    write_new_json(certificate_output, certificate)
    print(f"provenance={provenance_output} sha256={sha256(provenance_output)}")
    print(f"certificate={certificate_output} sha256={sha256(certificate_output)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
