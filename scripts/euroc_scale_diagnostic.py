#!/usr/bin/env python3
"""Print SE(3) and Sim(3) ATE plus Umeyama scale for a TUM-format trajectory.

Reuses the same ground-truth loader, association, and Umeyama alignment as
``evaluate_euroc_trajectory.py`` so numbers are directly comparable to that
evaluator's SE(3) ATE. Additionally computes a scale-enabled Umeyama
alignment (estimate -> ground truth) and reports the resulting scale factor,
which is the metric-scale-bias diagnostic used in
docs/vi_slam_global_consistency_plan.md Stage 1.

Usage:
    python scripts/euroc_scale_diagnostic.py \
        --ground-truth-csv E:/datasets/euroc_mav/all11/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv \
        --trajectory E:/visloc-rs-runs/basalt_lc_ceiling_20260914/vio_marg/MH_01_easy/trajectory.tum \
        --max-diff-ns 10000000

Prints one line: SE3=<m> Sim3=<m> scale=<estimate-to-gt> n=<associated pairs>
"""

import argparse
import importlib.util
import sys
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
_EVAL_PATH = _HERE / "evaluate_euroc_trajectory.py"
_spec = importlib.util.spec_from_file_location("evaluate_euroc_trajectory", _EVAL_PATH)
_eval_mod = importlib.util.module_from_spec(_spec)
sys.modules["evaluate_euroc_trajectory"] = _eval_mod
_spec.loader.exec_module(_eval_mod)


def diagnose(ground_truth_csv: Path, trajectory: Path, max_diff_ns: int, tum_time_unit: str = "ns"):
    ground_truth = _eval_mod.load_ground_truth(ground_truth_csv)
    estimate = _eval_mod.load_estimate(trajectory, tum_time_unit=tum_time_unit)
    pairs = _eval_mod.associate(ground_truth, estimate, max_diff_ns)
    if len(pairs) < 3:
        raise ValueError("fewer than 3 associated poses")
    gt_positions = np.asarray([pair[0][1] for pair in pairs])
    est_positions = np.asarray([pair[1][1] for pair in pairs])

    # SE(3): rigid alignment, no scale (matches evaluate_euroc_trajectory.py).
    _, se3_rotation, se3_translation = _eval_mod.umeyama(est_positions, gt_positions, False)
    se3_aligned = (se3_rotation @ est_positions.T).T + se3_translation
    se3_ate = float(np.sqrt(np.mean(np.sum(np.square(se3_aligned - gt_positions), axis=1))))

    # Sim(3): similarity alignment (rotation + translation + scale).
    sim_scale, sim_rotation, sim_translation = _eval_mod.umeyama(est_positions, gt_positions, True)
    sim_aligned = (sim_scale * (sim_rotation @ est_positions.T).T) + sim_translation
    sim_ate = float(np.sqrt(np.mean(np.sum(np.square(sim_aligned - gt_positions), axis=1))))

    return {
        "n": len(pairs),
        "se3_ate_m": se3_ate,
        "sim3_ate_m": sim_ate,
        "sim3_scale_estimate_to_gt": sim_scale,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ground-truth-csv", type=Path, required=True)
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--max-diff-ns", type=int, default=10_000_000)
    parser.add_argument("--tum-time-unit", choices=["ns", "s"], default="ns")
    args = parser.parse_args()

    result = diagnose(args.ground_truth_csv, args.trajectory, args.max_diff_ns, args.tum_time_unit)
    print(
        "SE3={se3_ate_m:.6f} Sim3={sim3_ate_m:.6f} scale={sim3_scale_estimate_to_gt:.6f} n={n}".format(
            **result
        )
    )


if __name__ == "__main__":
    main()
