#!/usr/bin/env python3
"""Run the pinned Basalt binary from the GT-free parity harness.

The upstream ``basalt_vio`` CLI writes ``trajectory.csv`` relative to its
current working directory.  The parity harness owns a separate engine output
directory, so this argv-only wrapper makes that directory the process cwd and
passes only the staged sensor tree, fixed upstream config, and fixed upstream
calibration to Basalt.  No shell is used.
"""

from __future__ import annotations

import argparse
import csv
import subprocess
from pathlib import Path


def normalize_trajectory(raw_path: Path, normalized_path: Path) -> None:
    """Map upstream EuRoC CSV names to the evaluator's neutral CSV schema."""

    with raw_path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.reader(stream))
    if len(rows) < 2 or len(rows[0]) < 8:
        raise RuntimeError(f"Basalt trajectory is empty or malformed: {raw_path}")
    normalized_path.parent.mkdir(parents=True, exist_ok=True)
    with normalized_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(
            ["timestamp_ns", "px", "py", "pz", "qw", "qx", "qy", "qz", "tracking_success"]
        )
        for row in rows[1:]:
            if len(row) < 8 or not row[0].strip():
                continue
            writer.writerow(
                [row[0], row[1], row[2], row[3], row[4], row[5], row[6], row[7], "1"]
            )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--input-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument(
        "--marg-data",
        type=Path,
        default=None,
        help="Optional MargData output outside the engine workspace; omitted for GT-firewall runs.",
    )
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--num-threads", type=int, default=0)
    args = parser.parse_args()

    input_root = args.input_root.resolve()
    output_root = args.output_root.resolve()
    binary = args.binary.resolve()
    config = args.config.resolve()
    calibration = args.calibration.resolve()
    if not (input_root / "mav0").is_dir():
        raise SystemExit(f"staged EuRoC input is missing mav0: {input_root}")
    if not binary.is_file():
        raise SystemExit(f"Basalt binary does not exist: {binary}")
    if not config.is_file() or not calibration.is_file():
        raise SystemExit("Basalt config/calibration is missing")
    output_root.mkdir(parents=True, exist_ok=True)

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
    ]
    if args.marg_data is not None:
        marg_data = args.marg_data.resolve()
        marg_data.mkdir(parents=True, exist_ok=True)
        command.extend(("--marg-data", str(marg_data)))
    if args.num_threads > 0:
        command.extend(("--num-threads", str(args.num_threads)))
    if args.max_frames > 0:
        command.extend(("--max-frames", str(args.max_frames)))

    completed = subprocess.run(command, cwd=output_root, check=False)
    raw_trajectory = output_root / "trajectory.csv"
    if completed.returncode == 0 and raw_trajectory.is_file():
        normalize_trajectory(raw_trajectory, output_root / "slam_trajectory.csv")
    return int(completed.returncode)


if __name__ == "__main__":
    raise SystemExit(main())
