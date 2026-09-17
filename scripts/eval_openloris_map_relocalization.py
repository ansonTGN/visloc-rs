#!/usr/bin/env python3
"""Evaluate OpenLORIS map-based relocalization.

Runs ``examples/localize_openloris_map`` in batch mode over a query-features
directory and compares each estimated pose to the query image's pose in a
reference (full) COLMAP text model, which is the map pose the localizer is
expected to recover. This is a GT-free self-consistency measure: the reference
pose is the mapper's own pose for that image, expressed in the same world frame
as the localization map.

Outputs translation error (camera-centre distance, metres), rotation error
(degrees) and the success rate, plus inlier/reprojection statistics.
"""

from __future__ import annotations

import argparse
import json
import statistics
import subprocess
import sys
from pathlib import Path

import numpy as np


def quat_to_R(q: tuple[float, float, float, float]) -> np.ndarray:
    w, x, y, z = q
    n = w * w + x * x + y * y + z * z
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array(
        [
            [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
            [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
            [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
        ]
    )


def load_reference(model_dir: Path) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """flat image name -> (camera centre, rotation matrix)."""
    out: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    for raw in (model_dir / "images.txt").read_text().splitlines():
        if not raw or raw.startswith("#"):
            continue
        f = raw.split()
        if len(f) != 10:
            continue
        q = (float(f[1]), float(f[2]), float(f[3]), float(f[4]))
        t = np.array([float(f[5]), float(f[6]), float(f[7])])
        R = quat_to_R(q)
        out[Path(f[9]).stem] = (-R.T @ t, R)
    return out


def rotation_angle_deg(Ra: np.ndarray, Rb: np.ndarray) -> float:
    cos = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--binary", type=Path, required=True)
    p.add_argument("--map-dir", type=Path, required=True)
    p.add_argument("--landmark-descriptors", type=Path, required=True)
    p.add_argument("--query-dir", type=Path, required=True)
    p.add_argument("--reference-model", type=Path, required=True)
    p.add_argument("--camera-id", type=int, default=1)
    p.add_argument("--out", type=Path)
    return p.parse_args()


def main() -> int:
    args = parse_args()
    reference = load_reference(args.reference_model)

    cmd = [
        str(args.binary),
        "--map-dir",
        str(args.map_dir),
        "--landmark-descriptors",
        str(args.landmark_descriptors),
        "--query-features-dir",
        str(args.query_dir),
        "--camera-id",
        str(args.camera_id),
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True, check=True)

    total = 0
    successes = 0
    t_errors: list[float] = []
    r_errors: list[float] = []
    inliers: list[int] = []
    reproj: list[float] = []
    failures: dict[str, int] = {}
    missing_ref = 0
    for line in proc.stdout.splitlines():
        parts = line.split()
        if not parts:
            continue
        name, rest = parts[0], parts[1:]
        if not rest:
            continue
        total += 1
        if rest[0] != "SUCCESS":
            reason = "unknown"
            for tok in rest:
                if tok.startswith("reason="):
                    reason = tok.split("=", 1)[1]
            failures[reason] = failures.get(reason, 0) + 1
            continue
        successes += 1
        values: dict[str, list[float]] = {}
        key = None
        for tok in rest[1:]:
            if "=" in tok:
                key, rhs = tok.split("=", 1)
                values[key] = []
                if rhs:
                    values[key].append(float(rhs))
            elif key is not None:
                values[key].append(float(tok))
        if name not in reference:
            missing_ref += 1
            continue
        t = np.array(values.get("t", [0.0, 0.0, 0.0]))
        q = values.get("q", [1.0, 0.0, 0.0, 0.0])
        c_est = -quat_to_R(tuple(q)).T @ t
        c_ref, R_ref = reference[name]
        t_errors.append(float(np.linalg.norm(c_est - c_ref)))
        r_errors.append(rotation_angle_deg(quat_to_R(tuple(q)), R_ref))
        inliers.append(int(values.get("inliers", [0])[0]))
        reproj.append(values.get("reproj", [float("nan")])[0])

    def pct(vals: list[float], p: float) -> float | None:
        return float(np.percentile(vals, p)) if vals else None

    summary = {
        "schema": "visloc_openloris_map_relocalization_eval_v1",
        "map_dir": str(args.map_dir),
        "query_dir": str(args.query_dir),
        "queries": total,
        "successes": successes,
        "success_rate": (successes / total) if total else 0.0,
        "failures": failures,
        "scored": len(t_errors),
        "missing_reference": missing_ref,
        "translation_error_m": {
            "mean": float(statistics.fmean(t_errors)) if t_errors else None,
            "median": float(statistics.median(t_errors)) if t_errors else None,
            "p95": pct(t_errors, 95),
            "max": max(t_errors) if t_errors else None,
        },
        "rotation_error_deg": {
            "mean": float(statistics.fmean(r_errors)) if r_errors else None,
            "median": float(statistics.median(r_errors)) if r_errors else None,
            "p95": pct(r_errors, 95),
            "max": max(r_errors) if r_errors else None,
        },
        "inliers": {
            "median": float(statistics.median(inliers)) if inliers else None,
            "min": min(inliers) if inliers else None,
        },
        "reprojection_px": {
            "median": float(statistics.median(reproj)) if reproj else None,
            "p95": pct(reproj, 95),
        },
        # success with the map pose recovered to < 0.5 m / < 2 deg
        "localization_rate_0p5m_2deg": (
            sum(1 for t, r in zip(t_errors, r_errors) if t < 0.5 and r < 2.0) / total
            if total
            else 0.0
        ),
    }
    if args.out:
        args.out.write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
