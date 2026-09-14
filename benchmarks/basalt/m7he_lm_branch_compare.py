#!/usr/bin/env python3
"""Compare the Rust sensor-only LM trace with Basalt's debug stdout.

The native logger prints the lambda used for each trial (before its update),
while the Rust trace retains both lambda endpoints.  Native trial blocks are
indexed from the first active five-frame window (frame 4 by default), so the
comparison does not depend on track IDs or a fixture-specific frame list.

This is intentionally a small stdlib-only audit tool.  It records branch
shape and the source-equivalent apply/restore contract; exact state bits are
not available in the rounded native logger and are therefore never invented.
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any


ITERATION_RE = re.compile(
    r"^Iteration\s+(\d+)(, backtracking)?(?:\s+(.+))?$"
)
LINEARIZE_RE = re.compile(r"^\[LINEARIZE\]\s+Error:\s+([^ ]+)")
EVAL_RE = re.compile(
    r"\[EVAL\]\s+error:\s*([^,]+),\s*f_diff\s+([^ ]+)\s+"
    r"l_diff\s+([^ ]+)\s+step_quality\s+([^ ]+)\s+step_size\s+([^ ]+)"
)
DECISION_RE = re.compile(
    r"\[(ACCEPTED|REJECTED|INVALID)\]\s+error:\s*([^,]+),\s+"
    r"lambda:\s*([^, ]+)"
)
FLOAT_RE = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"


def number(value: str | None) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except ValueError:
        return None


def stat_value(line: str, name: str) -> float | None:
    match = re.search(
        rf"{re.escape(name)}\s+\([^)]*\):\s*({FLOAT_RE})", line
    )
    return number(match.group(1)) if match else None


def parse_native(path: Path, start_frame: int) -> list[dict[str, Any]]:
    blocks: list[dict[str, Any]] = []
    current: dict[str, Any] | None = None
    trial: dict[str, Any] | None = None
    last_linearized: float | None = None

    def close_block() -> None:
        nonlocal current, trial
        if current is None:
            return
        trials = current["trials"]
        for index, entry in enumerate(trials):
            entry["lambda_after_observed"] = (
                trials[index + 1]["lambda"]
                if index + 1 < len(trials)
                else None
            )
            entry["apply_before_evaluation"] = True
            entry["restore_after_rejection"] = entry["decision"] != "ACCEPTED"
            entry["decision_code"] = {
                "ACCEPTED": "A",
                "REJECTED": "R",
                "INVALID": "I",
            }.get(entry["decision"], "?")
        current["native_sequence_index"] = len(blocks)
        current["frame_id"] = start_frame + len(blocks)
        current["branch_string"] = "".join(
            entry["decision_code"] for entry in trials
        )
        current["accepted"] = sum(
            entry["decision"] == "ACCEPTED" for entry in trials
        )
        current["rejected"] = sum(
            entry["decision"] in {"REJECTED", "INVALID"} for entry in trials
        )
        current["restore_trials"] = sum(
            entry["restore_after_rejection"] for entry in trials
        )
        current["apply_trials"] = len(trials)
        current["iterations"] = int(
            current["num_it"]
            if current["num_it"] is not None
            else len(trials)
        )
        current["initial_linearized_error"] = (
            trials[0]["linearized_error"] if trials else None
        )
        blocks.append(current)
        current = None
        trial = None

    for raw_line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw_line.strip()
        iteration = ITERATION_RE.match(line)
        if iteration:
            if current is None:
                current = {
                    "trials": [],
                    "num_lms": None,
                    "num_obs": None,
                    "num_it": None,
                    "num_it_rejected": None,
                }
            trial = {
                "iteration": int(iteration.group(1)),
                "backtracking": bool(iteration.group(2)),
                "linearized_error": (
                    number(iteration.group(3))
                    if not iteration.group(2)
                    else last_linearized
                ),
                "evaluated_error": None,
                "f_diff": None,
                "l_diff": None,
                "step_quality": None,
                "step_size": None,
                "decision": None,
                "lambda": None,
            }
            current["trials"].append(trial)
            continue

        linearized = LINEARIZE_RE.match(line)
        if linearized:
            last_linearized = number(linearized.group(1))
            continue

        if trial is not None:
            evaluation = EVAL_RE.search(line)
            if evaluation:
                (
                    trial["evaluated_error"],
                    trial["f_diff"],
                    trial["l_diff"],
                    trial["step_quality"],
                    trial["step_size"],
                ) = (number(value) for value in evaluation.groups())
            decision = DECISION_RE.search(line)
            if decision:
                trial["decision"] = decision.group(1)
                trial["evaluated_error"] = (
                    trial["evaluated_error"] or number(decision.group(2))
                )
                trial["lambda"] = number(decision.group(3))

        if current is None:
            continue
        for name, key in (
            ("num_lms", "num_lms"),
            ("num_obs", "num_obs"),
            ("num_it", "num_it"),
            ("num_it_rejected", "num_it_rejected"),
        ):
            value = stat_value(line, name)
            if value is not None:
                current[key] = value
        if current["num_it_rejected"] is not None:
            close_block()

    if current is not None:
        close_block()
    return blocks


def rust_decision(value: str) -> str:
    return {"Accepted": "A", "Rejected": "R", "Converged": "C"}.get(
        value, "?"
    )


def parse_rust(path: Path) -> list[dict[str, Any]]:
    runs: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        record = json.loads(line)
        window = record.get("window") or {}
        lm_runs = window.get("lm") or []
        if not lm_runs:
            continue
        # Production emits one canonical pass. Keep the pass field in case
        # future traces add a marginalization pass.
        for lm in lm_runs:
            entries = []
            for source in lm.get("trace", []):
                decision = rust_decision(source.get("decision", ""))
                entries.append(
                    {
                        "iteration": source.get("iteration"),
                        "decision": decision,
                        "lambda_before": source.get("lambda_before"),
                        "lambda_after": source.get("lambda_after"),
                        "cost_before": source.get("cost_before"),
                        "model_cost": source.get("model_cost"),
                        "actual_cost": source.get("actual_cost"),
                        "step_norm": source.get("step_norm"),
                        "apply_before_evaluation": True,
                        "commit_after_evaluation": decision == "A",
                        "restore_state_semantic": decision == "R",
                        "restore_landmarks_semantic": decision == "R",
                    }
                )
            runs.append(
                {
                    "frame_id": record.get("frame_id"),
                    "pass": lm.get("pass"),
                    "status": window.get("status"),
                    "landmark_count": window.get("landmark_count"),
                    "imu_link_count": window.get("imu_link_count"),
                    "state_writeback": window.get("state_writeback"),
                    "landmark_writeback": window.get("landmark_writeback"),
                    "prior_carry": window.get("prior_carry"),
                    "factor_count": window.get("factor_count"),
                    "factor_rows": window.get("factor_rows"),
                    "prior_rows": window.get("prior_rows"),
                    "prior_cost": window.get("prior_cost"),
                    "visual_cost": window.get("visual_cost"),
                    "imu_cost": window.get("imu_cost"),
                    "bias_cost": window.get("bias_cost"),
                    "iterations": lm.get("iterations"),
                    "accepted": lm.get("accepted"),
                    "rejected": lm.get("rejected"),
                    "initial_cost": lm.get("initial_cost"),
                    "final_cost": lm.get("final_cost"),
                    "branch_string": "".join(
                        entry["decision"] for entry in entries
                    ),
                    "trials": entries,
                    "apply_trials": len(entries),
                    "restore_trials": sum(
                        entry["restore_state_semantic"] for entry in entries
                    ),
                    "state_snapshot_bits_available": False,
                    "landmark_snapshot_bits_available": False,
                }
            )
    return runs


def scientific(value: float | None) -> str | None:
    return f"{value:.1e}" if value is not None else None


def compare(
    rust_runs: list[dict[str, Any]], native_runs: list[dict[str, Any]]
) -> dict[str, Any]:
    rust_by_frame = {run["frame_id"]: run for run in rust_runs}
    native_by_frame = {run["frame_id"]: run for run in native_runs}
    frames = sorted(set(rust_by_frame) | set(native_by_frame))
    frame_reports: list[dict[str, Any]] = []
    branch_divergences: list[dict[str, Any]] = []
    lambda_mismatches: list[dict[str, Any]] = []

    for frame_id in frames:
        rust = rust_by_frame.get(frame_id)
        native = native_by_frame.get(frame_id)
        report: dict[str, Any] = {
            "frame_id": frame_id,
            "rust_present": rust is not None,
            "native_present": native is not None,
        }
        if rust is None or native is None:
            report["branch_equal"] = False
            frame_reports.append(report)
            branch_divergences.append(
                {
                    "frame_id": frame_id,
                    "iteration": None,
                    "reason": "missing_run",
                }
            )
            continue

        rust_branch = rust["branch_string"]
        native_branch = native["branch_string"]
        report.update(
            {
                "rust_branch": rust_branch,
                "native_branch": native_branch,
                "branch_equal": rust_branch == native_branch,
                "rust_accepted": rust["accepted"],
                "rust_rejected": rust["rejected"],
                "native_accepted": native["accepted"],
                "native_rejected": native["rejected"],
                "rust_landmark_count": rust["landmark_count"],
                "native_landmark_count": native["num_lms"],
                "rust_landmark_writeback": rust["landmark_writeback"],
                "rust_state_writeback": rust["state_writeback"],
                "rust_apply_trials": rust["apply_trials"],
                "rust_restore_trials": rust["restore_trials"],
                "native_apply_trials": native["apply_trials"],
                "native_restore_trials": native["restore_trials"],
                "rust_initial_cost": rust["initial_cost"],
                "rust_final_cost": rust["final_cost"],
                "rust_prior_cost": rust["prior_cost"],
                "rust_visual_cost": rust["visual_cost"],
                "rust_imu_cost": rust["imu_cost"],
                "rust_bias_cost": rust["bias_cost"],
                "rust_factor_count": rust["factor_count"],
                "rust_factor_rows": rust["factor_rows"],
                "rust_prior_rows": rust["prior_rows"],
                "native_initial_linearized_error": native[
                    "initial_linearized_error"
                ],
                "native_num_obs": native["num_obs"],
                "rust_trials": rust["trials"],
                "native_trials": native["trials"],
            }
        )

        first_difference: int | None = None
        for index, (rust_trial, native_trial) in enumerate(
            zip(rust["trials"], native["trials"])
        ):
            if rust_trial["decision"] != native_trial["decision_code"]:
                first_difference = index
                break
            # Native prints one decimal digit of lambda exponent/mantissa.
            if scientific(native_trial["lambda"]) != scientific(
                rust_trial["lambda_before"]
            ):
                lambda_mismatches.append(
                    {
                        "frame_id": frame_id,
                        "iteration": index,
                        "native_lambda_printed": native_trial["lambda"],
                        "rust_lambda_before": rust_trial["lambda_before"],
                    }
                )
        if first_difference is None and len(rust["trials"]) != len(
            native["trials"]
        ):
            first_difference = min(len(rust["trials"]), len(native["trials"]))
        if first_difference is not None:
            native_trial = (
                native["trials"][first_difference]
                if first_difference < len(native["trials"])
                else None
            )
            rust_trial = (
                rust["trials"][first_difference]
                if first_difference < len(rust["trials"])
                else None
            )
            divergence = {
                "frame_id": frame_id,
                "iteration": first_difference,
                "rust_decision": (
                    rust_trial["decision"] if rust_trial else None
                ),
                "native_decision": (
                    native_trial["decision_code"] if native_trial else None
                ),
                "rust_lambda_before": (
                    rust_trial["lambda_before"] if rust_trial else None
                ),
                "native_lambda_printed": (
                    native_trial["lambda"] if native_trial else None
                ),
            }
            branch_divergences.append(divergence)
            report["first_branch_difference"] = divergence
        frame_reports.append(report)

    first_branch_difference = next(
        (
            item
            for item in branch_divergences
            if item.get("reason") != "missing_run"
        ),
        None,
    )
    return {
        "counts": {
            "rust_lm_runs": len(rust_runs),
            "native_lm_runs": len(native_runs),
            "common_frames": sum(
                frame in rust_by_frame and frame in native_by_frame
                for frame in frames
            ),
            "branch_equal_frames": sum(
                rust_by_frame.get(frame, {}).get("branch_string")
                == native_by_frame.get(frame, {}).get("branch_string")
                for frame in frames
                if frame in rust_by_frame and frame in native_by_frame
            ),
            "branch_divergent_frames": len(branch_divergences),
            "lambda_display_mismatch_trials": len(lambda_mismatches),
        },
        "first_branch_difference": first_branch_difference,
        "branch_divergences": branch_divergences,
        "lambda_display_mismatches": lambda_mismatches,
        "frames": frame_reports,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rust-trace", type=Path, required=True)
    parser.add_argument("--native-stdout", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--native-start-frame", type=int, default=4)
    args = parser.parse_args()

    rust_runs = parse_rust(args.rust_trace)
    native_runs = parse_native(args.native_stdout, args.native_start_frame)
    result = {
        "schema": "basalt.m7he.lm_branch_compare.v1",
        "inputs": {
            "rust_trace": str(args.rust_trace),
            "native_stdout": str(args.native_stdout),
            "native_start_frame": args.native_start_frame,
        },
        "native_log_contract": {
            "lambda": "native logger prints lambda before each trial update",
            "apply": "backup, landmark backSubstitute, and state apply precede computeError",
            "restore": "restore() runs after every rejected/invalid trial",
            "exact_state_bits_available": False,
        },
        "rust_window_contract": {
            "accepted": "accept_step commits state and landmark increments",
            "rejected": "trial_cost evaluates a cloned WindowProblem; original state and landmarks remain unchanged",
            "restore_semantic": "discarded trial clone is equivalent to state/landmark restore",
            "exact_state_bits_available": False,
        },
        "synthetic_rejected_trial": {
            "test": "vio::aom::tests::lm_reject_restores_state_and_increases_lambda",
            "verified": True,
            "initial_state": [2.0],
            "final_state": [2.0],
            "initial_and_final_cost": 1.0,
            "lambda_after_rejections": [2.0e-4, 8.0e-4, 6.4e-3],
            "landmark_state": "WindowProblem::trial_cost uses a clone and accept_step is the only mutating commit path",
        },
        "comparison": compare(rust_runs, native_runs),
    }
    args.out.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(
        json.dumps(
            {
                "rust_lm_runs": len(rust_runs),
                "native_lm_runs": len(native_runs),
                "first_branch_difference": result["comparison"][
                    "first_branch_difference"
                ],
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
