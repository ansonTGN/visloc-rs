#!/usr/bin/env python3
"""Generate the Basalt VI-SLAM README figures from measured run artifacts.

Two figures are produced from real EuRoC replay outputs (no synthetic data):

1. Top-down (X, Y) trajectory overlay: Rust `basalt_euroc_vio_demo` estimate
   (SE(3)-Umeyama-aligned to ground truth) vs native Basalt (same alignment)
   vs EuRoC Vicon/Leica ground truth, for one or two EuRoC sequences.
2. A bar chart of per-sequence ATE (translation RMSE, SE(3) alignment) for
   the Rust port vs native Basalt across all 11 EuRoC sequences, read from
   the all11 combined gate-report JSON.

The trajectory alignment (SE(3) Umeyama, no scale) mirrors
`scripts/evaluate_euroc_trajectory.py`, which produced the numbers reported
in the README table, so the plotted alignment matches the reported ATE.

Ground-truth EuRoC CSVs are read from a local dataset copy for plotting only;
this script does not write anything into release/benchmark manifests.

Usage::

    python3 scripts/plot_basalt_readme_figures.py \\
        --all11-json work/m11_phase6_latest_combined_all11x1_20260914.json \\
        --output-dir docs/assets \\
        --traj-seq MH_01_easy=<rust_tum>,<native_csv>,<gt_csv> \\
        --traj-seq V1_01_easy=<rust_tum>,<native_csv>,<gt_csv>

Optional dependencies: numpy, matplotlib. Asset-generation helper, not part
of the core build, test, or CI path.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--all11-json", type=Path, required=True, help="m11_phase6_latest_combined_all11x1 JSON")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--traj-seq",
        action="append",
        default=[],
        metavar="NAME=RUST_TUM,NATIVE_CSV,GT_CSV",
        help="one sequence to render as a trajectory overlay; repeatable",
    )
    return parser.parse_args()


def timestamp_ns(token: str) -> int:
    return int(token.strip().split(".", 1)[0])


def load_euroc_csv(path: Path) -> list[tuple[int, float, float, float]]:
    """Load `timestamp_ns,p_x,p_y,p_z,q_w,q_x,q_y,q_z,...` rows (native/GT convention)."""
    poses = []
    with path.open("r", encoding="utf-8", errors="replace", newline="") as stream:
        for row in csv.reader(stream):
            if not row or row[0].lstrip().startswith("#"):
                continue
            if len(row) < 4:
                continue
            ts = timestamp_ns(row[0])
            px, py, pz = float(row[1]), float(row[2]), float(row[3])
            poses.append((ts, px, py, pz))
    poses.sort(key=lambda pose: pose[0])
    return poses


def load_tum(path: Path) -> list[tuple[int, float, float, float]]:
    """Load `timestamp_s tx ty tz qx qy qz qw` TUM rows (Rust engine output)."""
    poses = []
    with path.open("r", encoding="utf-8", errors="replace") as stream:
        for line in stream:
            fields = line.split()
            if not fields or fields[0].startswith("#"):
                continue
            if len(fields) != 8:
                continue
            ts_ns = int(round(float(fields[0]) * 1e9))
            tx, ty, tz = float(fields[1]), float(fields[2]), float(fields[3])
            poses.append((ts_ns, tx, ty, tz))
    poses.sort(key=lambda pose: pose[0])
    return poses


def associate(gt, est, max_diff_ns: int = 10_000_000):
    import bisect

    gt_stamps = [pose[0] for pose in gt]
    pairs = []
    used_gt = set()
    for pose in est:
        idx = bisect.bisect_left(gt_stamps, pose[0])
        candidates = [c for c in (idx - 1, idx) if 0 <= c < len(gt)]
        if not candidates:
            continue
        best = min(candidates, key=lambda c: abs(gt_stamps[c] - pose[0]))
        delta = abs(gt_stamps[best] - pose[0])
        if delta <= max_diff_ns and best not in used_gt:
            used_gt.add(best)
            pairs.append((gt[best], pose))
    return pairs


def umeyama(src, dst, with_scale: bool):
    import numpy as np

    src_mean, dst_mean = src.mean(axis=0), dst.mean(axis=0)
    src_c, dst_c = src - src_mean, dst - dst_mean
    cov = src_c.T @ dst_c / len(src)
    u, singular, vt = np.linalg.svd(cov)
    sign = np.sign(np.linalg.det(vt.T @ u.T))
    correction = np.diag([1.0, 1.0, sign])
    rotation = vt.T @ correction @ u.T
    variance = float(np.square(src_c).sum() / len(src))
    scale = float((singular * np.asarray([1.0, 1.0, sign])).sum() / variance) if with_scale else 1.0
    translation = dst_mean - scale * rotation @ src_mean
    return scale, rotation, translation


def align_to_gt(gt, est):
    """SE(3) Umeyama-align `est` positions onto `gt` (no scale); returns Nx3 aligned array."""
    import numpy as np

    pairs = associate(gt, est)
    if len(pairs) < 3:
        raise ValueError("fewer than 3 associated poses")
    gt_xyz = np.asarray([[p[0][1], p[0][2], p[0][3]] for p in pairs])
    est_xyz = np.asarray([[p[1][1], p[1][2], p[1][3]] for p in pairs])
    _, rotation, translation = umeyama(est_xyz, gt_xyz, False)
    all_est_xyz = np.asarray([[pose[1], pose[2], pose[3]] for pose in est])
    aligned = (rotation @ all_est_xyz.T).T + translation
    return aligned


def plot_trajectory_figure(seq_name: str, rust_tum: Path, native_csv: Path, gt_csv: Path, output: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    gt = load_euroc_csv(gt_csv)
    rust = load_tum(rust_tum)
    native = load_euroc_csv(native_csv) if native_csv else None

    rust_aligned = align_to_gt(gt, rust)
    native_aligned = align_to_gt(gt, native) if native else None
    gt_xyz = np.asarray([[p[1], p[2], p[3]] for p in gt])

    fig, ax = plt.subplots(figsize=(6.4, 6.0))
    ax.plot(gt_xyz[:, 0], gt_xyz[:, 1], color="#2f3b52", linewidth=2.2, label="EuRoC ground truth", zorder=2)
    if native_aligned is not None:
        ax.plot(
            native_aligned[:, 0],
            native_aligned[:, 1],
            color="#f2994a",
            linewidth=1.5,
            linestyle="--",
            label="native Basalt (aligned)",
            zorder=3,
        )
    ax.plot(
        rust_aligned[:, 0],
        rust_aligned[:, 1],
        color="#2f80ed",
        linewidth=1.4,
        label="Rust port (aligned)",
        zorder=4,
    )
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"EuRoC {seq_name} — top-down trajectory (SE(3)-aligned)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, color="#d8dee9", linewidth=0.6)
    ax.legend(loc="best", frameon=True, framealpha=0.9)
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=150, facecolor="white")
    plt.close(fig)
    print(f"wrote {output}")


def plot_ate_bar_chart(all11_json: Path, output: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    with all11_json.open() as handle:
        data = json.load(handle)
    comparisons = data["gate_report"]["comparisons"]
    rows = []
    for comparison in comparisons:
        gates = comparison["gates"]["ate_translation_se3"]["observed"]
        rows.append((comparison["sequence"], gates["rust"], gates["native"]))
    rows.sort(key=lambda row: row[0])

    labels = [row[0].replace("_easy", "").replace("_medium", "").replace("_difficult", "") for row in rows]
    rust_vals = [row[1] for row in rows]
    native_vals = [row[2] for row in rows]

    x = np.arange(len(labels))
    width = 0.38
    fig, ax = plt.subplots(figsize=(10.5, 4.6))
    ax.bar(x - width / 2, rust_vals, width, label="Rust port", color="#2f80ed")
    ax.bar(x + width / 2, native_vals, width, label="native Basalt", color="#f2994a")
    ax.set_ylabel("ATE translation RMSE, SE(3) [m]")
    ax.set_title("All-11 EuRoC: Rust port vs native Basalt (lower is better)")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=30, ha="right")
    ax.grid(True, axis="y", color="#d8dee9", linewidth=0.6)
    ax.legend(loc="upper left", frameon=True, framealpha=0.9)
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=150, facecolor="white")
    plt.close(fig)
    print(f"wrote {output}")


def main() -> int:
    args = parse_args()
    try:
        import matplotlib  # noqa: F401
        import numpy  # noqa: F401
    except ImportError:
        print("matplotlib/numpy not available; install them first.", file=sys.stderr)
        return 2

    plot_ate_bar_chart(args.all11_json, args.output_dir / "basalt_all11_ate_bar.png")

    for spec in args.traj_seq:
        name, rest = spec.split("=", 1)
        rust_tum_s, native_csv_s, gt_csv_s = rest.split(",")
        plot_trajectory_figure(
            name,
            Path(rust_tum_s),
            Path(native_csv_s),
            Path(gt_csv_s),
            args.output_dir / f"basalt_{name.lower()}_trajectory.png",
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
