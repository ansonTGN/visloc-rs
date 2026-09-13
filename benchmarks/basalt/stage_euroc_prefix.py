#!/usr/bin/env python3
"""Materialize a deterministic sensor-only EuRoC frame prefix for M1 runs."""

from __future__ import annotations

import argparse
import csv
import shutil
from pathlib import Path


def copy_prefix(sequence_root: Path, destination: Path, max_frames: int) -> int:
    source_mav0 = sequence_root / "mav0"
    if not source_mav0.is_dir() or max_frames <= 0:
        raise ValueError("expected a EuRoC sequence root and positive max_frames")
    if destination.exists() and any(destination.iterdir()):
        raise ValueError(f"destination is not empty: {destination}")
    destination.mkdir(parents=True, exist_ok=True)

    selected: dict[str, list[tuple[str, str]]] = {}
    for camera in ("cam0", "cam1"):
        camera_root = source_mav0 / camera
        csv_path = camera_root / "data.csv"
        with csv_path.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.reader(stream)
            rows = list(reader)
        if not rows:
            raise ValueError(f"empty camera index: {csv_path}")
        selected_rows = rows[: max_frames + 1]
        if len(selected_rows) != max_frames + 1:
            raise ValueError(f"camera index has fewer than {max_frames} frames: {csv_path}")
        selected[camera] = [tuple(row[:2]) for row in selected_rows[1:]]
        out_camera = destination / "mav0" / camera
        out_data = out_camera / "data"
        out_data.mkdir(parents=True, exist_ok=True)
        shutil.copy2(camera_root / "sensor.yaml", out_camera / "sensor.yaml")
        with (out_camera / "data.csv").open("w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream, lineterminator="\n")
            writer.writerows(selected_rows)
        for _, filename in selected[camera]:
            source_image = camera_root / "data" / filename
            if not source_image.is_file():
                raise ValueError(f"missing image referenced by {csv_path}: {source_image}")
            shutil.copy2(source_image, out_data / filename)

    imu_root = source_mav0 / "imu0"
    out_imu = destination / "mav0" / "imu0"
    out_imu.mkdir(parents=True, exist_ok=True)
    shutil.copy2(imu_root / "sensor.yaml", out_imu / "sensor.yaml")
    shutil.copy2(imu_root / "data.csv", out_imu / "data.csv")
    return max_frames


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sequence-root", type=Path, required=True)
    parser.add_argument("--destination", type=Path, required=True)
    parser.add_argument("--max-frames", type=int, required=True)
    args = parser.parse_args()
    frames = copy_prefix(args.sequence_root.resolve(), args.destination.resolve(), args.max_frames)
    print(f"staged sensor-only EuRoC prefix: {args.destination} ({frames} frames per camera)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
