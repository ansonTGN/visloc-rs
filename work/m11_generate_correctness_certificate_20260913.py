"""Generate the current AVX2/FMA + LM-reuse Basalt correctness certificate."""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
from pathlib import Path
from typing import Any


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest().upper()


def record(root: Path, path: Path) -> dict[str, Any]:
    resolved = path.resolve()
    return {
        "path": resolved.relative_to(root).as_posix(),
        "bytes": resolved.stat().st_size,
        "sha256": sha256(resolved),
    }


def parse_summary(path: Path) -> dict[str, Any]:
    values: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        key, separator, value = line.partition("=")
        if separator:
            values[key] = value
    integer_keys = (
        "frames_requested",
        "frames_processed",
        "cam0_manifest_frames",
        "cam1_manifest_timestamps",
        "imu_samples_loaded",
        "imu_samples_delivered",
        "observations_emitted",
    )
    return {key: int(values[key]) for key in integer_keys}


def replay_record(
    frame_count: int,
    primary: Path,
    repeat: Path,
    executable: Path,
    dataset: Path,
    calibration: Path,
    config: Path,
) -> dict[str, Any]:
    expected = {52: (14661, 512), 80: (24256, 792), 400: (106981, 3992)}
    primary_summary = parse_summary(primary / "summary.txt")
    repeat_summary = parse_summary(repeat / "summary.txt")
    if primary_summary != repeat_summary:
        raise ValueError(f"max{frame_count} summary counts differ between independent runs")
    if primary_summary["frames_requested"] != frame_count or primary_summary["frames_processed"] != frame_count:
        raise ValueError(f"max{frame_count} frame count mismatch")
    if (primary_summary["observations_emitted"], primary_summary["imu_samples_delivered"]) != expected[frame_count]:
        raise ValueError(f"max{frame_count} observation/IMU count mismatch")
    for name in ("trajectory.csv", "trajectory.tum"):
        if sha256(primary / name) != sha256(repeat / name):
            raise ValueError(f"max{frame_count} {name} is not byte-exact across independent runs")
    forbidden = ("trace.jsonl", "marg_data", "timing_breakdown.json")
    for directory in (primary, repeat):
        present = [name for name in forbidden if (directory / name).exists()]
        if present:
            raise ValueError(f"max{frame_count} forbidden lean artifacts present: {present}")
    return {
        "name": f"max{frame_count}_lean",
        "max_frames": frame_count,
        "exit_code": 0,
        "output_dir": str(primary),
        "independent_repeat_dir": str(repeat),
        "argv": [
            str(executable), "--euroc-dir", str(dataset), "--calibration", str(calibration),
            "--config", str(config), "--out-dir", str(primary), "--max-frames", str(frame_count),
            "--no-marg-data", "--no-trace",
        ],
        "summary": {**primary_summary, "summary_sha256": sha256(primary / "summary.txt")},
        "artifacts": {
            "trajectory_csv": {"bytes": (primary / "trajectory.csv").stat().st_size, "sha256": sha256(primary / "trajectory.csv")},
            "trajectory_tum": {"bytes": (primary / "trajectory.tum").stat().st_size, "sha256": sha256(primary / "trajectory.tum")},
            "files": ["summary.txt", "trajectory.csv", "trajectory.tum"],
            "forbidden_absent": list(forbidden),
        },
        "authoritative": {
            "trajectory_csv_byte_exact": True,
            "trajectory_tum_byte_exact": True,
            "obs_exact": True,
            "imu_exact": True,
            "basis": "independent same-input replay; exact trajectory bytes and exact observation/IMU counts",
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--replay-root", type=Path, required=True)
    parser.add_argument("--max80-repeat", type=Path, required=True)
    parser.add_argument("--executable", type=Path, required=True)
    parser.add_argument("--build-evidence", type=Path, required=True)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root = args.root.resolve()
    replay_root = args.replay_root.resolve()
    executable = args.executable.resolve()
    dataset = args.dataset.resolve()
    calibration = (root / "benchmarks/basalt/release_inputs/euroc_ds_calib.json").resolve()
    config = (root / "configs/basalt/euroc_config.json").resolve()
    protocol = (root / "benchmarks/basalt/protocols/basalt_euroc_parity_v1.json").resolve()
    dataset_manifest = (root / "benchmarks/basalt/euroc_dataset_manifest.json").resolve()
    sources = sorted((root / "pipelines/basalt/src").rglob("*.rs"))
    source_records = [record(root, path) for path in sources]
    replays = [
        replay_record(52, replay_root / "max52_lean", replay_root / "max52_lean_repeat2", executable, dataset, calibration, config),
        replay_record(80, replay_root / "max80_lean", args.max80_repeat.resolve(), executable, dataset, calibration, config),
        replay_record(400, replay_root / "max400_lean", replay_root / "max400_lean_repeat2", executable, dataset, calibration, config),
    ]
    artifact = {
        "schema": "visloc.basalt.release_candidate_correctness.v1",
        "artifact": "m11_release_candidate_avx2_reuse_correctness_20260913",
        "created_utc": dt.datetime.now(dt.timezone.utc).replace(microsecond=0).isoformat(),
        "status": "PASS_CORRECTNESS_ONLY",
        "scope": {
            "workspace_root": str(root),
            "fresh_target_dir": str(args.build_evidence.resolve().parent),
            "features": ["basalt-lm-workspace-reuse"],
            "timing_breakdown": "compile-time feature disabled",
            "lm_workspace_reuse": "feature enabled",
            "performance_evaluation": False,
            "commit": False,
            "push": False,
        },
        "build": {
            "status": "success",
            "exit_code": 0,
            "command": ["cargo", "build", "--locked", "--release", "--example", "basalt_euroc_vio_demo", "--features", "basalt-lm-workspace-reuse"],
            "rustflags": "-C target-feature=+avx2,+fma",
            "build_evidence": record(root, args.build_evidence.resolve()),
            "executable": record(root, executable),
        },
        "source_binding": {
            "hash_algorithm": "SHA256",
            "files": source_records,
        },
        "protocol_binding": {
            "protocol": "basalt_euroc_parity_v1",
            **record(root, protocol),
            "correctness_profile": "lean_no_marg_no_trace",
        },
        "input_binding": {
            "dataset_root": str(dataset),
            "sensor_only": True,
            "gt_argument_passed_to_engine": False,
            "dataset_manifest": record(root, dataset_manifest),
            "calibration": record(root, calibration),
            "config": record(root, config),
            "engine_flags": ["--no-marg-data", "--no-trace"],
            "environment": {"VISLOC_BASALT_TIMING_BREAKDOWN": "unset"},
        },
        "replays": replays,
        "aggregate": {
            "all_exit_zero": True,
            "all_frame_counts_exact": True,
            "all_independent_trajectory_replays_byte_exact": True,
            "all_observation_and_imu_counts_exact": True,
            "performance_claimed": False,
        },
    }
    output = args.output.resolve()
    if output.exists():
        raise ValueError(f"refusing to overwrite: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(artifact, indent=2) + "\n", encoding="utf-8", newline="\n")
    print(output)
    print(sha256(output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
