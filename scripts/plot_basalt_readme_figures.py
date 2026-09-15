#!/usr/bin/env python3
"""Generate the Basalt VI-SLAM README figures from measured run artifacts.

Two figures are produced from real EuRoC replay outputs (no synthetic data),
both comparing this repository's shipped result (official EuRoC calibration,
Basalt Rust-port VIO + the unchanged offline NFR mapper, called "visloc-rs"
below) against measured ORB-SLAM3 stereo-inertial:

1. A 2x3 grid of top-down (X, Y) trajectories (visloc-rs vs ORB-SLAM3 vs
   EuRoC ground truth, all SE(3)-Umeyama-aligned to ground truth) for six
   EuRoC sequences.
2. A two-series bar chart of full-trajectory SE(3) ATE (visloc-rs vs
   ORB-SLAM3) across all 11 EuRoC sequences.

The trajectory alignment (SE(3) Umeyama, no scale) mirrors
`scripts/evaluate_euroc_trajectory.py`, which produced the numbers reported
in the README table, so the plotted alignment matches the reported ATE.

Ground-truth EuRoC CSVs are read from a local dataset copy for plotting only;
this script does not write anything into release/benchmark manifests.

Usage::

    python3 scripts/plot_basalt_readme_figures.py \\
        --output-dir docs/assets \\
        --official-calib-summary-json <basalt_official_calib_run>/summary.json \\
        --orbslam3-summary-md <orbslam3_run>/summary.md \\
        --override V2_02_medium:full_ate_se3_rmse_m=0.0103 \\
        --hero-mapper-out-dir <basalt_official_calib_run>/mapper_out \\
        --hero-orbslam3-dir <orbslam3_run> \\
        --hero-gt-root <euroc_dataset_root>/all11

`--override` patches one sequence's mapper row after loading
`--official-calib-summary-json`; it exists because the driver's V2_02_medium
mapper stage in this repository's own 2026-09-15 run was killed by a wall-time
safety monitor artefact (the correct result came from a manual detached
rerun, see README). Omit `--official-calib-summary-json`/`--orbslam3-summary-md`
to skip the bar chart, or `--hero-mapper-out-dir`/`--hero-orbslam3-dir`/
`--hero-gt-root` to skip the hero trajectory grid.

Online-mapper mode (Stage 3 of docs/vi_slam_global_consistency_plan.md,
scripts/run_basalt_online_all11.py's output): pass
`--online-summary-json <online_run>/summary.json` (its own schema --
`rows[].online_se3_rmse_m`, `status`/`error` for not-run/failed rows,
distinct from `--official-calib-summary-json`'s offline schema) together
with `--orbslam3-summary-md` to draw a 2-series online-vs-ORB-SLAM3 bar
chart (`basalt_online_vs_orbslam3.png`), matching the offline chart's shape
but reading the online driver's numbers. Pass `--hero-online-run-dir
<online_run>/runs` (each `<run_dir>/<SEQ>/trajectory_online.tum` is already
the full-frame propagated trajectory -- no mapper_out/rerun-directory
naming to thread through, unlike the offline hero grid) together with
`--hero-orbslam3-dir`/`--hero-gt-root` for the online hero trajectory grid
(`basalt_online_vs_orbslam3_trajectories.png`). These are independent of,
and do not change, the offline `--official-calib-summary-json` /
`--hero-mapper-out-dir` outputs above.

Optional dependencies: numpy, matplotlib. Asset-generation helper, not part
of the core build, test, or CI path.
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
from pathlib import Path

# (EuRoC sequence, short label, mapper_out subdirectory, trajectory filename)
# for the hero trajectory grid. V2_02_medium uses the manual detached rerun
# directory because the driver's own attempt for that sequence was killed by
# a wall-time safety monitor artefact (see README / plan doc).
HERO_SEQUENCES = [
    ("MH_01_easy", "MH_01", "MH_01_easy", "trajectory_full_propagated.tum"),
    ("MH_03_medium", "MH_03", "MH_03_medium", "trajectory_full_propagated.tum"),
    ("V1_02_medium", "V1_02", "V1_02_medium", "trajectory_full_propagated.tum"),
    ("V1_03_difficult", "V1_03", "V1_03_difficult", "trajectory_full_propagated.tum"),
    ("V2_01_easy", "V2_01", "V2_01_easy", "trajectory_full_propagated.tum"),
    ("V2_02_medium", "V2_02", "V2_02_medium_rerun", "full_trajectory.tum"),
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--official-calib-summary-json",
        type=Path,
        default=None,
        help="run_basalt_official_calib_all11.py summary.json (VIO + mapper)",
    )
    parser.add_argument(
        "--orbslam3-summary-md",
        type=Path,
        default=None,
        help="ORB-SLAM3 same-protocol summary.md (measured column parsed from its table)",
    )
    parser.add_argument(
        "--override",
        action="append",
        default=[],
        metavar="SEQUENCE:field=value[,field=value...]",
        help="patch one sequence's mapper row after loading (see module docstring)",
    )
    parser.add_argument(
        "--hero-mapper-out-dir",
        type=Path,
        default=None,
        help="mapper_out/ root from the official-calibration all-11 run",
    )
    parser.add_argument(
        "--hero-orbslam3-dir",
        type=Path,
        default=None,
        help="ORB-SLAM3 run root containing <SEQ>/r1/f_orbslam3_<SEQ>_r1.txt",
    )
    parser.add_argument(
        "--hero-gt-root",
        type=Path,
        default=None,
        help="EuRoC dataset root containing <SEQ>/mav0/state_groundtruth_estimate0/data.csv",
    )
    parser.add_argument(
        "--online-summary-json",
        type=Path,
        default=None,
        help="run_basalt_online_all11.py summary.json (online mapper, its own schema)",
    )
    parser.add_argument(
        "--hero-online-run-dir",
        type=Path,
        default=None,
        help="run_basalt_online_all11.py's runs/ directory "
        "(<dir>/<SEQ>/trajectory_online.tum is already the full-frame propagated trajectory)",
    )
    return parser.parse_args()


def timestamp_ns(token: str) -> int:
    return int(token.strip().split(".", 1)[0])


def load_euroc_csv(path: Path) -> list[tuple[int, float, float, float]]:
    """Load `timestamp_ns,p_x,p_y,p_z,q_w,q_x,q_y,q_z,...` rows (EuRoC GT convention)."""
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
    """Load `timestamp_s tx ty tz qx qy qz qw` TUM rows (this repo's engine/mapper output)."""
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


def load_tum_ns(path: Path) -> list[tuple[int, float, float, float]]:
    """Load `timestamp_ns tx ty tz qx qy qz qw` TUM rows (ORB-SLAM3's own output convention:
    the first field is already nanoseconds, written as a float with a trailing '.000000')."""
    poses = []
    with path.open("r", encoding="utf-8", errors="replace") as stream:
        for line in stream:
            fields = line.split()
            if not fields or fields[0].startswith("#"):
                continue
            if len(fields) != 8:
                continue
            ts_ns = int(round(float(fields[0])))
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


def align_and_ate(gt, est):
    """SE(3)-Umeyama-align `est` onto `gt` (no scale, 10ms association).

    Returns `(aligned_xyz_for_every_est_pose, translation_ate_rmse_m)`; the
    RMSE is computed over the associated (nearest-timestamp, <=10ms) pairs
    only, matching `scripts/evaluate_euroc_trajectory.py`'s protocol.
    """
    import numpy as np

    pairs = associate(gt, est)
    if len(pairs) < 3:
        raise ValueError("fewer than 3 associated poses")
    gt_xyz = np.asarray([[p[0][1], p[0][2], p[0][3]] for p in pairs])
    est_xyz = np.asarray([[p[1][1], p[1][2], p[1][3]] for p in pairs])
    _, rotation, translation = umeyama(est_xyz, gt_xyz, False)
    aligned_assoc = (rotation @ est_xyz.T).T + translation
    residual = aligned_assoc - gt_xyz
    rmse = float(np.sqrt(np.mean(np.sum(residual**2, axis=1))))
    all_est_xyz = np.asarray([[pose[1], pose[2], pose[3]] for pose in est])
    aligned_all = (rotation @ all_est_xyz.T).T + translation
    return aligned_all, rmse


def plot_hero_trajectory_grid(
    mapper_out_dir: Path,
    orbslam3_dir: Path,
    gt_root: Path,
    output: Path,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    gt_color, orb_color, ours_color = "#1b1f27", "#c06b6b", "#2f6fed"

    fig, axes = plt.subplots(2, 3, figsize=(16.0, 9.0))
    for ax, (sequence, label, mapper_subdir, traj_name) in zip(axes.flat, HERO_SEQUENCES):
        gt = load_euroc_csv(gt_root / sequence / "mav0" / "state_groundtruth_estimate0" / "data.csv")
        ours = load_tum(mapper_out_dir / mapper_subdir / traj_name)
        orb = load_tum_ns(orbslam3_dir / sequence / "r1" / f"f_orbslam3_{sequence}_r1.txt")

        ours_aligned, ours_rmse = align_and_ate(gt, ours)
        orb_aligned, orb_rmse = align_and_ate(gt, orb)
        gt_xyz = np.asarray([[p[1], p[2], p[3]] for p in gt])

        ax.plot(gt_xyz[:, 0], gt_xyz[:, 1], color=gt_color, linewidth=2.0, zorder=2, label="EuRoC ground truth")
        ax.plot(orb_aligned[:, 0], orb_aligned[:, 1], color=orb_color, linewidth=1.4, zorder=3, label="ORB-SLAM3")
        ax.plot(ours_aligned[:, 0], ours_aligned[:, 1], color=ours_color, linewidth=1.5, zorder=4, label="visloc-rs")

        ax.set_title(f"{label} — ours {ours_rmse * 100:.1f} cm / ORB-SLAM3 {orb_rmse * 100:.1f} cm", fontsize=11)
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_xticks([])
        ax.set_yticks([])
        for spine in ax.spines.values():
            spine.set_color("#c7ccd4")
        ax.set_facecolor("white")

    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=3, frameon=False, bbox_to_anchor=(0.5, -0.02))
    fig.suptitle(
        "EuRoC top-down trajectories, SE(3)-aligned to ground truth: visloc-rs vs ORB-SLAM3",
        fontsize=13,
    )
    fig.patch.set_facecolor("white")
    fig.tight_layout(rect=(0, 0.04, 1, 0.96))
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=100, facecolor="white", bbox_inches="tight", pil_kwargs={"optimize": True})
    plt.close(fig)
    print(f"wrote {output}")


def parse_overrides(specs: list[str]) -> dict[str, dict[str, float]]:
    overrides: dict[str, dict[str, float]] = {}
    for spec in specs:
        sequence, _, fields = spec.partition(":")
        patch: dict[str, float] = {}
        for field in fields.split(","):
            key, _, value = field.partition("=")
            if key:
                patch[key.strip()] = float(value)
        overrides[sequence.strip()] = patch
    return overrides


def load_orbslam3_measured(summary_md: Path) -> dict[str, float]:
    """Parse the "ORB-SLAM3 measured median ATE-SE3" column out of its results table."""
    measured: dict[str, float] = {}
    pattern = re.compile(r"^\|\s*([A-Za-z0-9_]+)\s*\|\s*([0-9.]+)\s*\(")
    for line in summary_md.read_text(encoding="utf-8").splitlines():
        match = pattern.match(line)
        if match:
            measured[match.group(1)] = float(match.group(2))
    return measured


def load_official_calib_rows(summary_json: Path, overrides: dict[str, dict[str, float]]):
    import json

    with summary_json.open() as handle:
        data = json.load(handle)
    rows = []
    for row in data["rows"]:
        sequence = row["sequence"]
        patch = overrides.get(sequence, {})
        full_se3 = patch.get("full_ate_se3_rmse_m", row.get("full_ate_se3_rmse_m"))
        if full_se3 is None:
            # sequence's mapper stage failed in the driver and was not overridden
            continue
        rows.append((sequence, float(full_se3)))
    rows.sort(key=lambda row: row[0])
    return rows


def load_online_rows(summary_json: Path):
    """Load `run_basalt_online_all11.py`'s summary.json rows.

    Its schema differs from the offline driver's (`online_se3_rmse_m`, and a
    "not_run"/`error` row shape for sequences the sweep has not reached or
    that failed) -- see that script's `write_summary`.
    """
    import json

    with summary_json.open() as handle:
        data = json.load(handle)
    rows = []
    for row in data["rows"]:
        se3 = row.get("online_se3_rmse_m")
        if se3 is None:
            # not yet run, or failed -- see row.get("status")/row.get("error")
            continue
        rows.append((row["sequence"], float(se3)))
    rows.sort(key=lambda row: row[0])
    return rows


def plot_online_vs_orbslam3(
    online_summary_json: Path,
    orbslam3_summary_md: Path,
    output: Path,
) -> None:
    """Same 2-series bar-chart shape as `plot_ours_vs_orbslam3`, reading the
    online mapper driver's summary.json instead of the offline one."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    rows = load_online_rows(online_summary_json)
    orbslam3 = load_orbslam3_measured(orbslam3_summary_md)

    labels = [seq.replace("_easy", "").replace("_medium", "").replace("_difficult", "") for seq, _ in rows]
    online_vals = [se3 for _, se3 in rows]
    orb_vals = [orbslam3[seq] for seq, _ in rows]

    x = np.arange(len(labels))
    width = 0.36
    fig, ax = plt.subplots(figsize=(10.5, 4.6))
    ax.bar(x - width / 2, online_vals, width, label="visloc-rs (online VIO + mapper)", color="#2f6fed")
    ax.bar(x + width / 2, orb_vals, width, label="ORB-SLAM3 (measured)", color="#c0392b")
    ax.set_ylabel("Full-trajectory ATE translation RMSE, SE(3) [m]")
    ax.set_title("EuRoC: visloc-rs online VI-SLAM vs ORB-SLAM3 (lower is better)")
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


def plot_hero_trajectory_grid_online(
    run_dir: Path,
    orbslam3_dir: Path,
    gt_root: Path,
    output: Path,
) -> None:
    """Same hero-grid shape as `plot_hero_trajectory_grid`, reading the
    online driver's own `<run_dir>/<SEQ>/trajectory_online.tum` directly (it
    is already the full-frame propagated trajectory -- no mapper_out/rerun
    directory naming to thread through)."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    gt_color, orb_color, ours_color = "#1b1f27", "#c06b6b", "#2f6fed"

    fig, axes = plt.subplots(2, 3, figsize=(16.0, 9.0))
    for ax, (sequence, label, _mapper_subdir, _traj_name) in zip(axes.flat, HERO_SEQUENCES):
        gt = load_euroc_csv(gt_root / sequence / "mav0" / "state_groundtruth_estimate0" / "data.csv")
        ours = load_tum(run_dir / sequence / "trajectory_online.tum")
        orb = load_tum_ns(orbslam3_dir / sequence / "r1" / f"f_orbslam3_{sequence}_r1.txt")

        ours_aligned, ours_rmse = align_and_ate(gt, ours)
        orb_aligned, orb_rmse = align_and_ate(gt, orb)
        gt_xyz = np.asarray([[p[1], p[2], p[3]] for p in gt])

        ax.plot(gt_xyz[:, 0], gt_xyz[:, 1], color=gt_color, linewidth=2.0, zorder=2, label="EuRoC ground truth")
        ax.plot(orb_aligned[:, 0], orb_aligned[:, 1], color=orb_color, linewidth=1.4, zorder=3, label="ORB-SLAM3")
        ax.plot(
            ours_aligned[:, 0],
            ours_aligned[:, 1],
            color=ours_color,
            linewidth=1.5,
            zorder=4,
            label="visloc-rs (online)",
        )

        ax.set_title(f"{label} — ours {ours_rmse * 100:.1f} cm / ORB-SLAM3 {orb_rmse * 100:.1f} cm", fontsize=11)
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_xticks([])
        ax.set_yticks([])
        for spine in ax.spines.values():
            spine.set_color("#c7ccd4")
        ax.set_facecolor("white")

    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=3, frameon=False, bbox_to_anchor=(0.5, -0.02))
    fig.suptitle(
        "EuRoC top-down trajectories, SE(3)-aligned to ground truth: visloc-rs (online) vs ORB-SLAM3",
        fontsize=13,
    )
    fig.patch.set_facecolor("white")
    fig.tight_layout(rect=(0, 0.04, 1, 0.96))
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=100, facecolor="white", bbox_inches="tight", pil_kwargs={"optimize": True})
    plt.close(fig)
    print(f"wrote {output}")


def plot_ours_vs_orbslam3(
    official_calib_summary_json: Path,
    orbslam3_summary_md: Path,
    overrides: dict[str, dict[str, float]],
    output: Path,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    rows = load_official_calib_rows(official_calib_summary_json, overrides)
    orbslam3 = load_orbslam3_measured(orbslam3_summary_md)

    labels = [seq.replace("_easy", "").replace("_medium", "").replace("_difficult", "") for seq, _ in rows]
    ours_vals = [mapper for _, mapper in rows]
    orb_vals = [orbslam3[seq] for seq, _ in rows]

    x = np.arange(len(labels))
    width = 0.36
    fig, ax = plt.subplots(figsize=(10.5, 4.6))
    ax.bar(x - width / 2, ours_vals, width, label="visloc-rs (Basalt port + mapper)", color="#2f6fed")
    ax.bar(x + width / 2, orb_vals, width, label="ORB-SLAM3 (measured)", color="#c0392b")
    ax.set_ylabel("Full-trajectory ATE translation RMSE, SE(3) [m]")
    ax.set_title("EuRoC: visloc-rs vs ORB-SLAM3 (lower is better)")
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

    if args.official_calib_summary_json and args.orbslam3_summary_md:
        plot_ours_vs_orbslam3(
            args.official_calib_summary_json,
            args.orbslam3_summary_md,
            parse_overrides(args.override),
            args.output_dir / "basalt_official_calib_vs_orbslam3.png",
        )

    if args.hero_mapper_out_dir and args.hero_orbslam3_dir and args.hero_gt_root:
        plot_hero_trajectory_grid(
            args.hero_mapper_out_dir,
            args.hero_orbslam3_dir,
            args.hero_gt_root,
            args.output_dir / "basalt_vs_orbslam3_trajectories.png",
        )

    if args.online_summary_json and args.orbslam3_summary_md:
        plot_online_vs_orbslam3(
            args.online_summary_json,
            args.orbslam3_summary_md,
            args.output_dir / "basalt_online_vs_orbslam3.png",
        )

    if args.hero_online_run_dir and args.hero_orbslam3_dir and args.hero_gt_root:
        plot_hero_trajectory_grid_online(
            args.hero_online_run_dir,
            args.hero_orbslam3_dir,
            args.hero_gt_root,
            args.output_dir / "basalt_online_vs_orbslam3_trajectories.png",
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
