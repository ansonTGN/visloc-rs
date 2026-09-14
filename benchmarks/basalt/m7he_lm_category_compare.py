#!/usr/bin/env python3
"""Decompose one LM boundary into source assembly categories.

This is an audit-only companion to ``m7he_lm_branch_compare.py``.  Rust's
opt-in iteration detail trace contains the ABS_QR reduced row stack, while the
native GDB capture contains cumulative H/b at the visual, post-IMU,
post-damping/prior, and final boundaries.  The tool compares those stages in
binary32 bit space, reports the first exact mismatch, and constructs a
counterfactual H/b in which only Rust's IMU increment is replaced by the
native one.  The counterfactual is deliberately diagnostic data; it never
changes the estimator or LM schedule.

The native capture is independent of track IDs.  Its selected window is
identified by the active landmark count only because the pinned GDB run has
no public frame identifier at ``get_dense_H_b`` entry; the output records that
selection and must be paired with the native LM logger before interpretation.
"""

from __future__ import annotations

import argparse
import json
import struct
import sys
from pathlib import Path
from typing import Any


def f32(value: float) -> float:
    return struct.unpack("<f", struct.pack("<f", float(value)))[0]


def f32_bits(value: float) -> str:
    return "%08x" % struct.unpack("<I", struct.pack("<f", f32(value)))[0]


def f32_add(left: float, right: float) -> float:
    return f32(f32(left) + f32(right))


def f32_sub(left: float, right: float) -> float:
    return f32(f32(left) - f32(right))


def load_native_matrix(value: dict[str, Any]) -> list[list[float]]:
    rows = int(value["rows"])
    cols = int(value["cols"])
    bits = value["bits"]
    if len(bits) != rows * cols:
        raise ValueError(f"native matrix has {len(bits)} values, expected {rows * cols}")
    flat = [
        struct.unpack("<f", struct.pack("<I", int(item, 16)))[0]
        for item in bits
    ]
    return [[flat[col * rows + row] for col in range(cols)] for row in range(rows)]


def load_native_vector(value: dict[str, Any]) -> list[float]:
    rows = int(value["rows"])
    bits = value["bits"]
    if len(bits) != rows:
        raise ValueError(f"native vector has {len(bits)} values, expected {rows}")
    return [
        struct.unpack("<f", struct.pack("<I", int(item, 16)))[0]
        for item in bits
    ]


def zero_matrix(rows: int, cols: int) -> list[list[float]]:
    return [[0.0 for _ in range(cols)] for _ in range(rows)]


def zero_vector(rows: int) -> list[float]:
    return [0.0 for _ in range(rows)]


def add_matrix(dst: list[list[float]], src: list[list[float]]) -> None:
    for row in range(len(dst)):
        for col in range(len(dst[row])):
            dst[row][col] = f32_add(dst[row][col], src[row][col])


def subtract_matrix(left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
    return [
        [f32_sub(left[row][col], right[row][col]) for col in range(len(left[row]))]
        for row in range(len(left))
    ]


def add_vector(dst: list[float], src: list[float]) -> None:
    for index in range(len(dst)):
        dst[index] = f32_add(dst[index], src[index])


def subtract_vector(left: list[float], right: list[float]) -> list[float]:
    return [f32_sub(left[index], right[index]) for index in range(len(left))]


def normal_from_rows(
    rows: list[list[float]], rhs: list[float], start: int, end: int, cols: int
) -> tuple[list[list[float]], list[float]]:
    h = zero_matrix(cols, cols)
    b = zero_vector(cols)
    for row_index in range(start, end):
        row = rows[row_index]
        value = f32(rhs[row_index])
        for col in range(cols):
            b[col] = f32_add(b[col], f32(row[col]) * value)
            for other in range(cols):
                h[col][other] = f32_add(
                    h[col][other], f32(row[col]) * f32(row[other])
                )
    return h, b


def first_difference(
    left_h: list[list[float]],
    left_b: list[float],
    right_h: list[list[float]],
    right_b: list[float],
) -> dict[str, Any] | None:
    for row in range(len(left_h)):
        for col in range(len(left_h[row])):
            left = f32(left_h[row][col])
            right = f32(right_h[row][col])
            if f32_bits(left) != f32_bits(right):
                return {
                    "kind": "H",
                    "row": row,
                    "col": col,
                    "left": left,
                    "right": right,
                    "left_bits": f32_bits(left),
                    "right_bits": f32_bits(right),
                    "abs_diff": abs(left - right),
                }
    for row, (left, right) in enumerate(zip(left_b, right_b)):
        left = f32(left)
        right = f32(right)
        if f32_bits(left) != f32_bits(right):
            return {
                "kind": "b",
                "row": row,
                "left": left,
                "right": right,
                "left_bits": f32_bits(left),
                "right_bits": f32_bits(right),
                "abs_diff": abs(left - right),
            }
    return None


def max_abs_difference(
    left_h: list[list[float]],
    left_b: list[float],
    right_h: list[list[float]],
    right_b: list[float],
) -> dict[str, float]:
    h = max(
        (abs(left_h[row][col] - right_h[row][col])
         for row in range(len(left_h))
         for col in range(len(left_h[row]))),
        default=0.0,
    )
    b = max(
        (abs(left - right) for left, right in zip(left_b, right_b)),
        default=0.0,
    )
    return {"h": h, "b": b}


def rust_snapshots(path: Path, frame: int) -> dict[tuple[int, int, str], dict[str, Any]]:
    snapshots: dict[tuple[int, int, str], dict[str, Any]] = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        record = json.loads(line)
        if record.get("record") != "snapshot" or record.get("frame_id") != frame:
            continue
        key = (int(record["trial"]), int(record["iteration"]), record["phase"])
        snapshots[key] = record
    if not snapshots:
        raise ValueError(f"no Rust detail snapshots for frame {frame}")
    return snapshots


def rust_trace_record(path: Path, frame: int) -> dict[str, Any]:
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        record = json.loads(line)
        if record.get("frame_id") == frame:
            return record
    raise ValueError(f"no Rust sensor trace record for frame {frame}")


def visual_cost_from_detail(snapshot: dict[str, Any]) -> float | None:
    factors = snapshot.get("landmark_factors")
    if not isinstance(factors, list):
        return None
    ordered = sorted(factors, key=lambda factor: tuple(factor.get("row_span", [0, 0])))
    value = 0.0
    for factor in ordered:
        for observation in factor.get("observations", []):
            objective = observation.get("objective")
            if objective is not None:
                value = f32_add(value, float(objective))
    return value


def category_costs(snapshot: dict[str, Any]) -> dict[str, Any]:
    direct = snapshot.get("category_costs")
    if isinstance(direct, dict):
        return direct
    # Older detail files predate category_costs.  Visual objective is still
    # independently recoverable from the per-observation audit; the other
    # categories are intentionally marked unavailable instead of inferred from
    # a total and losing the source association.
    return {
        "prior": None,
        "visual": visual_cost_from_detail(snapshot),
        "imu": None,
        "bias": None,
        "total": snapshot.get("cost", {}).get("before"),
        "availability": "visual-only legacy detail trace",
    }


def rust_stage_systems(
    snapshot: dict[str, Any], window: dict[str, Any]
) -> dict[str, tuple[list[list[float]], list[float]]]:
    global_data = snapshot["global"]
    rows = [[f32(value) for value in row] for row in global_data["reduced_rows"]]
    rhs = [f32(value) for value in global_data["reduced_rhs"]]
    cols = len(rows[0]) if rows else len(global_data["b"])
    prior_rows = int(window.get("prior_factor_rows", 0))
    visual_rows = int(window.get("visual_factor_rows", 0))
    imu_rows = int(window.get("imu_factor_rows", 0))
    bias_rows = int(window.get("bias_factor_rows", 0))
    expected = prior_rows + visual_rows + imu_rows + bias_rows
    if expected != len(rows):
        raise ValueError(
            f"category row count {expected} != serialized row count {len(rows)}"
        )
    prior = normal_from_rows(rows, rhs, 0, prior_rows, cols)
    visual = normal_from_rows(rows, rhs, prior_rows, prior_rows + visual_rows, cols)
    imu_start = prior_rows + visual_rows
    imu = normal_from_rows(rows, rhs, imu_start, imu_start + imu_rows, cols)
    bias_start = imu_start + imu_rows
    bias = normal_from_rows(rows, rhs, bias_start, bias_start + bias_rows, cols)
    visual_imu = (zero_matrix(cols, cols), zero_vector(cols))
    add_matrix(visual_imu[0], visual[0])
    add_matrix(visual_imu[0], imu[0])
    add_vector(visual_imu[1], visual[1])
    add_vector(visual_imu[1], imu[1])
    full = (zero_matrix(cols, cols), zero_vector(cols))
    for category in (prior, visual, imu, bias):
        add_matrix(full[0], category[0])
        add_vector(full[1], category[1])
    return {
        "prior": prior,
        "visual": visual,
        "imu": imu,
        "bias": bias,
        "visual_imu": visual_imu,
        "full": full,
    }


def native_stages(path: Path) -> dict[str, tuple[list[list[float]], list[float]]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    captures = payload.get("captures", {})
    stages: dict[str, tuple[list[list[float]], list[float]]] = {}
    for name, capture in captures.items():
        stages[name] = (
            load_native_matrix(capture["h"]),
            load_native_vector(capture["b"]),
        )
    return stages


def compare_stages(
    rust: dict[str, tuple[list[list[float]], list[float]]],
    native: dict[str, tuple[list[list[float]], list[float]]],
) -> dict[str, Any]:
    # Native cumulative boundaries map to the corresponding source phase.
    mapping = {
        "visual": "visual",
        "visual_imu": "imu",
        "full": "full",
    }
    result: dict[str, Any] = {}
    for rust_name, native_name in mapping.items():
        if rust_name not in rust or native_name not in native:
            result[rust_name] = {"available": False}
            continue
        rust_h, rust_b = rust[rust_name]
        native_h, native_b = native[native_name]
        result[rust_name] = {
            "available": True,
            "shape": [len(rust_h), len(rust_h[0]) if rust_h else 0],
            "first_exact_difference": first_difference(
                rust_h, rust_b, native_h, native_b
            ),
            "max_abs_difference": max_abs_difference(
                rust_h, rust_b, native_h, native_b
            ),
        }
    if "imu" in native and "visual" in native:
        native_imu = (
            subtract_matrix(native["imu"][0], native["visual"][0]),
            subtract_vector(native["imu"][1], native["visual"][1]),
        )
        rust_imu = rust["imu"]
        result["imu_increment"] = {
            "available": True,
            "first_exact_difference": first_difference(
                rust_imu[0], rust_imu[1], native_imu[0], native_imu[1]
            ),
            "max_abs_difference": max_abs_difference(
                rust_imu[0], rust_imu[1], native_imu[0], native_imu[1]
            ),
        }
    return result


def counterfactual_imu_patch(
    rust: dict[str, tuple[list[list[float]], list[float]]],
    native: dict[str, tuple[list[list[float]], list[float]]],
) -> dict[str, Any]:
    if not {"full", "imu", "visual"}.issubset(rust) or not {"imu", "visual"}.issubset(native):
        return {"available": False, "reason": "missing category boundary"}
    native_increment = (
        subtract_matrix(native["imu"][0], native["visual"][0]),
        subtract_vector(native["imu"][1], native["visual"][1]),
    )
    rust_increment = rust["imu"]
    patched_h = [row[:] for row in rust["full"][0]]
    patched_b = rust["full"][1][:]
    delta_h = subtract_matrix(native_increment[0], rust_increment[0])
    delta_b = subtract_vector(native_increment[1], rust_increment[1])
    add_matrix(patched_h, delta_h)
    add_vector(patched_b, delta_b)
    result: dict[str, Any] = {
        "available": True,
        "meaning": "Rust full reduced H/b with only the IMU increment replaced by native cumulative(visual+IMU)-visual",
        "patched_first_difference_vs_unpatched": first_difference(
            rust["full"][0], rust["full"][1], patched_h, patched_b
        ),
        "patched_delta_max_abs": max_abs_difference(
            rust["full"][0], rust["full"][1], patched_h, patched_b
        ),
    }
    # Keep the counterfactual matrices out of the normal report by default;
    # they are available when a caller needs to feed them to a solver probe.
    return result


def decision_margin(
    rust_trace: dict[str, Any], native_stdout: Path, frame: int, iteration: int
) -> dict[str, Any]:
    # Reuse the established native parser so branch and rounding semantics do
    # not diverge between reports.
    sys.path.insert(0, str(Path(__file__).parent))
    import m7he_lm_branch_compare as branch_compare

    native_runs = branch_compare.parse_native(native_stdout, 4)
    native = next(run for run in native_runs if run["frame_id"] == frame)
    native_trial = native["trials"][iteration]
    rust_lm = (rust_trace.get("window") or {}).get("lm", [])[0]
    rust_trial = rust_lm["trace"][iteration]
    rust_before = f32(rust_trial["cost_before"])
    rust_actual = f32(rust_trial["actual_cost"])
    rust_model = f32(rust_trial["model_cost"])
    return {
        "frame_id": frame,
        "iteration": iteration,
        "native": {
            "decision": native_trial["decision_code"],
            "f_diff_printed": native_trial["f_diff"],
            "l_diff_printed": native_trial["l_diff"],
            "step_quality_printed": native_trial["step_quality"],
            "step_size_printed": native_trial["step_size"],
            "linearized_error_printed": native_trial["linearized_error"],
            "evaluated_error_printed": native_trial["evaluated_error"],
        },
        "rust": {
            "decision": rust_trial["decision"],
            "cost_before": rust_before,
            "model_cost": rust_model,
            "actual_cost": rust_actual,
            "actual_minus_before": f32_sub(rust_actual, rust_before),
            "model_minus_before": f32_sub(rust_model, rust_before),
            "step_norm": rust_trial["step_norm"],
        },
        "interpretation": "native logger rounds f_diff/l_diff; Rust values are serialized binary32 costs",
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rust-detail", type=Path, required=True)
    parser.add_argument("--rust-trace", type=Path, required=True)
    parser.add_argument("--native-categories", type=Path, required=True)
    parser.add_argument("--native-stdout", type=Path, required=True)
    parser.add_argument("--frame", type=int, default=8)
    parser.add_argument(
        "--stage-iteration",
        type=int,
        default=0,
        help="Rust iteration_start snapshot to compare with the native H/b capture",
    )
    parser.add_argument("--iteration", type=int, default=4)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()

    rust_record = rust_trace_record(args.rust_trace, args.frame)
    snapshots = rust_snapshots(args.rust_detail, args.frame)
    initial = snapshots[(0, 0, "iteration_start")]
    stage_snapshot = snapshots[(0, args.stage_iteration, "iteration_start")]
    window = rust_record["window"]
    rust_stages = rust_stage_systems(stage_snapshot, window)
    native = native_stages(args.native_categories)
    result = {
        "schema": "basalt.m7he.lm_category_compare.v1",
        "inputs": {
            "rust_detail": str(args.rust_detail),
            "rust_trace": str(args.rust_trace),
            "native_categories": str(args.native_categories),
            "native_stdout": str(args.native_stdout),
            "frame": args.frame,
            "stage_iteration": args.stage_iteration,
            "iteration": args.iteration,
        },
        "rust_initial_categories": category_costs(initial),
        "rust_stage_categories": category_costs(stage_snapshot),
        "rust_initial_row_partition": {
            key: window.get(key)
            for key in (
                "prior_factor_rows",
                "visual_factor_rows",
                "imu_factor_rows",
                "bias_factor_rows",
            )
        },
        "native_selection": json.loads(args.native_categories.read_text(encoding="utf-8")).get(
            "selected_landmark_count"
        ),
        "stage_comparison": compare_stages(rust_stages, native),
        "imu_hb_counterfactual": counterfactual_imu_patch(rust_stages, native),
        "decision_margin": decision_margin(
            rust_record, args.native_stdout, args.frame, args.iteration
        ),
        "rust_trial_category_costs": {
            f"iteration_{iteration}_{phase}": category_costs(snapshot)
            for (trial, iteration, phase), snapshot in sorted(snapshots.items())
            if trial == 0 and phase in {"iteration_start", "trial"}
        },
        "notes": [
            "H/b stage comparisons use f32 row outer products from the serialized Rust reduced row stack.",
            "Older detail traces expose visual objective per observation but not per-trial prior/IMU/bias costs; those fields remain null until category_costs is emitted.",
            "The IMU replacement is an H/b counterfactual only and does not claim a branch result without rerunning the solver with the override.",
        ],
    }
    args.out.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({
        "schema": result["schema"],
        "stage_comparison": result["stage_comparison"],
        "decision_margin": result["decision_margin"],
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
