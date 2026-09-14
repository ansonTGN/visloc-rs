"""Read-only M7 clean H/b versus Rust state/factor residual analysis.

This script consumes the already captured clean-native H/b and state records
plus the Rust JSONL detail snapshot.  It does not run Basalt, invoke a
compiler, or modify production sources.  The generated JSON is intentionally
small enough to review in a diff while retaining the exact mismatch counts,
factor partition, state-lane mapping, and pure-support residual metrics.
"""

from __future__ import annotations

import json
import math
import struct
from collections import Counter
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
HB_PATH = ROOT / "target" / "m7ct_clean_frame4_hb.json"
STATE_PATH = ROOT / "target" / "m7db_clean_frame4_states.json"
RUST_PATH = ROOT / "target" / "m7cm_fresh5_detail_20260823.jsonl"
OUT_PATH = ROOT / "target" / "m7de_hb_state_residual.json"


def f32_from_bits(value: str) -> float:
    return struct.unpack("<f", struct.pack("<I", int(value, 16)))[0]


def f32_bits(value: float) -> int:
    return struct.unpack("<I", struct.pack("<f", float(value)))[0]


def bits_array(value: np.ndarray) -> np.ndarray:
    flat = np.asarray(value).reshape(-1)
    return np.asarray([f32_bits(x) for x in flat], dtype=np.uint32).reshape(
        np.asarray(value).shape
    )


def finite(value: float) -> float:
    """JSON-safe float conversion for metrics (all inputs are finite here)."""

    value = float(value)
    if not math.isfinite(value):
        return value
    return value


def metrics(native: np.ndarray, predicted: np.ndarray, mask: np.ndarray) -> dict:
    n = np.asarray(native, dtype=np.float64)[mask]
    p = np.asarray(predicted, dtype=np.float64)[mask]
    delta = n - p
    native_bits = bits_array(n)
    predicted_bits = bits_array(p)
    mismatch = native_bits != predicted_bits
    max_index = int(np.argmax(np.abs(delta))) if delta.size else 0
    max_cell = np.argwhere(mask)[max_index].tolist() if delta.size else None
    native_norm = float(np.linalg.norm(n))
    return {
        "cells": int(delta.size),
        "bit_mismatches": int(np.count_nonzero(mismatch)),
        "nonzero_residual": int(np.count_nonzero(delta)),
        "max_abs": finite(np.max(np.abs(delta)) if delta.size else 0.0),
        "mean_abs": finite(np.mean(np.abs(delta)) if delta.size else 0.0),
        "l2": finite(np.linalg.norm(delta)),
        "relative_l2_to_native": finite(
            np.linalg.norm(delta) / native_norm if native_norm else 0.0
        ),
        "max_cell": max_cell,
        "max_native": finite(n[max_index]) if delta.size else 0.0,
        "max_predicted": finite(p[max_index]) if delta.size else 0.0,
        "max_delta_native_minus_predicted": finite(delta[max_index])
        if delta.size
        else 0.0,
    }


def mismatch_count(native: np.ndarray, predicted: np.ndarray, mask: np.ndarray) -> int:
    return int(np.count_nonzero(bits_array(native)[mask] != bits_array(predicted)[mask]))


def load_inputs() -> tuple[dict, dict, dict]:
    with HB_PATH.open(encoding="utf-8") as handle:
        hb = json.load(handle)
    with STATE_PATH.open(encoding="utf-8") as handle:
        state = json.load(handle)
    with RUST_PATH.open(encoding="utf-8") as handle:
        snapshots = [json.loads(line) for line in handle]
    detail = next(
        item
        for item in snapshots
        if item.get("record") == "snapshot"
        and item.get("iteration") == 0
        and item.get("trial") == 0
        and item.get("phase") == "iteration_start"
    )
    return hb, state, detail


def state_comparison(native_state: dict, detail: dict) -> tuple[list[dict], dict]:
    fields = [
        (
            "quaternion_xyzw",
            "quaternion_xyzw_bits",
            lambda rust: rust["pose"]["quaternion_xyzw"],
        ),
        (
            "translation_xyz",
            "translation_xyz_bits",
            lambda rust: rust["pose"]["translation"],
        ),
        ("velocity_xyz", "velocity_xyz_bits", lambda rust: rust["velocity"]),
        ("bias_gyro_xyz", "bias_gyro_xyz_bits", lambda rust: rust["bias_gyro"]),
        ("bias_accel_xyz", "bias_accel_xyz_bits", lambda rust: rust["bias_accel"]),
    ]
    mismatches: list[dict] = []
    for native_record, rust_record in zip(
        native_state["native_states"], detail["blocks"]["states"]
    ):
        for field, bits_name, rust_getter in fields:
            native_bits = native_record["state_linearized"][bits_name]
            rust_values = rust_getter(rust_record)
            rust_bits = [f"{f32_bits(value):08x}" for value in rust_values]
            for lane, (native_bit, rust_bit) in enumerate(zip(native_bits, rust_bits)):
                if native_bit.lower() == rust_bit.lower():
                    continue
                native_value = f32_from_bits(native_bit)
                rust_value = f32_from_bits(rust_bit)
                mismatches.append(
                    {
                        "frame_id": int(native_record["frame_id"]),
                        "field": field,
                        "lane": lane,
                        "native_bits": native_bit.lower(),
                        "rust_bits": rust_bit.lower(),
                        "native": native_value,
                        "rust": rust_value,
                        "delta": rust_value - native_value,
                        # The clean-state report uses the signed raw f32 bit
                        # delta for same-sign lanes (the four lanes here).
                        "delta_ulp": int(rust_bit, 16) - int(native_bit, 16),
                    }
                )
    return mismatches, {
        "total_lanes": 75,
        "bit_mismatches": len(mismatches),
        "mismatches": mismatches,
    }


def factor_partition(detail: dict, rows: np.ndarray, rhs: np.ndarray) -> tuple[dict, dict, dict]:
    # The first 15 rows are the LM damping rows.  The 61 landmark factors are
    # contiguous next, and the four 15-row IMU factors occupy the tail.
    partitions = {"damping": (0, 15), "visual": (15, 1183), "imu": (1183, 1243)}
    h_parts = {}
    b_parts = {}
    for name, (start, end) in partitions.items():
        h_parts[name] = rows[start:end].T @ rows[start:end]
        b_parts[name] = rows[start:end].T @ rhs[start:end]

    visual_factors = detail["landmark_factors"]
    visual_rows_concat = np.concatenate(
        [np.asarray(factor["reduced_rows"], dtype=np.float64) for factor in visual_factors],
        axis=0,
    )
    visual_rhs_concat = np.concatenate(
        [np.asarray(factor["reduced_rhs"], dtype=np.float64) for factor in visual_factors],
        axis=0,
    )
    target_observations = Counter()
    target_rows = Counter()
    target_factors = Counter()
    visual_factor_norms = []
    for factor in visual_factors:
        observations = factor["observations"]
        factor_rows = np.asarray(factor["reduced_rows"], dtype=np.float64)
        factor_rhs = np.asarray(factor["reduced_rhs"], dtype=np.float64)
        visual_factor_norms.append(
            {
                "track_id": int(factor["track_id"]),
                "row_span_visual": [
                    int(factor["row_span"][0]),
                    int(factor["row_span"][1]),
                ],
                "observations": len(observations),
                "H_fro": finite(np.linalg.norm(factor_rows.T @ factor_rows)),
                "b_l2": finite(np.linalg.norm(factor_rows.T @ factor_rhs)),
            }
        )
        seen_frames = set()
        for observation in observations:
            frame = int(observation["target_frame_id"])
            target_observations[frame] += 1
            target_rows[frame] += 2
            seen_frames.add(frame)
        for frame in seen_frames:
            target_factors[frame] += 1

    imu_factor_norms = []
    for factor_index in range(4):
        start = 1183 + factor_index * 15
        end = start + 15
        factor_rows = rows[start:end]
        factor_rhs = rhs[start:end]
        imu_factor_norms.append(
            {
                "factor_index": factor_index,
                "frame_pair": [factor_index, factor_index + 1],
                "global_row_span": [start, end],
                "rows": 15,
                "H_fro": finite(np.linalg.norm(factor_rows.T @ factor_rows)),
                "b_l2": finite(np.linalg.norm(factor_rows.T @ factor_rhs)),
            }
        )

    partition = {
        "damping": {
            "global_row_span": [0, 15],
            "rows": 15,
            "H_fro": finite(np.linalg.norm(h_parts["damping"])),
            "b_l2": finite(np.linalg.norm(b_parts["damping"])),
            "H_nonzero_cells": int(np.count_nonzero(h_parts["damping"])),
        },
        "visual": {
            "global_row_span": [15, 1183],
            "factor_count": len(visual_factors),
            "rows": int(sum(len(factor["reduced_rows"]) for factor in visual_factors)),
            "observations": int(sum(len(factor["observations"]) for factor in visual_factors)),
            "H_fro": finite(np.linalg.norm(h_parts["visual"])),
            "b_l2": finite(np.linalg.norm(b_parts["visual"])),
            "H_nonzero_cells": int(np.count_nonzero(h_parts["visual"])),
            "b_nonzero_lanes": int(np.count_nonzero(b_parts["visual"])),
            "target_frame_observations": {str(k): int(v) for k, v in sorted(target_observations.items())},
            "target_frame_rows": {str(k): int(v) for k, v in sorted(target_rows.items())},
            "target_frame_factor_counts": {str(k): int(v) for k, v in sorted(target_factors.items())},
            "factor_norm_summary": {
                "H_fro_min": finite(min(item["H_fro"] for item in visual_factor_norms)),
                "H_fro_max": finite(max(item["H_fro"] for item in visual_factor_norms)),
                "H_fro_mean": finite(np.mean([item["H_fro"] for item in visual_factor_norms])),
                "b_l2_min": finite(min(item["b_l2"] for item in visual_factor_norms)),
                "b_l2_max": finite(max(item["b_l2"] for item in visual_factor_norms)),
            },
        },
        "imu": {
            "global_row_span": [1183, 1243],
            "factor_count": 4,
            "rows": 60,
            "H_fro": finite(np.linalg.norm(h_parts["imu"])),
            "b_l2": finite(np.linalg.norm(b_parts["imu"])),
            "H_nonzero_cells": int(np.count_nonzero(h_parts["imu"])),
            "b_nonzero_lanes": int(np.count_nonzero(b_parts["imu"])),
            "factor_norms": imu_factor_norms,
        },
        "row_alignment": {
            "visual_global_start": 15,
            "visual_rows_max_abs_delta": finite(
                np.max(np.abs(rows[15:1183] - visual_rows_concat))
            ),
            "visual_rhs_max_abs_delta": finite(
                np.max(np.abs(rhs[15:1183] - visual_rhs_concat))
            ),
            "imu_tail_grouping": "global rows 1183:1243 grouped as four contiguous 15-row frame-pair factors",
        },
        "visual_factor_norms": visual_factor_norms,
    }
    # Keep the per-factor table useful for reviewers, but do not embed any
    # 75-column Jacobian payloads in the report artifact.
    return partition, h_parts, b_parts


def main() -> None:
    hb, native_state, detail = load_inputs()
    clean_h = np.asarray(
        [f32_from_bits(value) for value in hb["H_f32_bits"]], dtype=np.float64
    ).reshape(75, 75, order="F")
    clean_b = np.asarray(
        [f32_from_bits(value) for value in hb["b_f32_bits"]], dtype=np.float64
    )
    rust_h = np.asarray(detail["global"]["h"], dtype=np.float64)
    rust_b = np.asarray(detail["global"]["b"], dtype=np.float64)
    rows = np.asarray(detail["global"]["reduced_rows"], dtype=np.float64)
    rhs = np.asarray(detail["global"]["reduced_rhs"], dtype=np.float64)

    partition, h_parts, b_parts = factor_partition(detail, rows, rhs)
    state_mismatches, state_summary = state_comparison(native_state, detail)

    indices = np.arange(75)
    frame = indices // 15
    local = indices % 15
    frame_row, frame_col = np.meshgrid(frame, frame, indexing="ij")
    local_row, local_col = np.meshgrid(local, local, indexing="ij")
    pose_pose = (local_row < 6) & (local_col < 6)
    nonpose = ~pose_pose
    visual_only = pose_pose & (np.abs(frame_row - frame_col) > 1)
    mixed_pose = pose_pose & ~visual_only

    actual_h = {
        "visual_only_nonadjacent_pose": metrics(clean_h, rust_h, visual_only),
        "imu_only_nonpose": metrics(clean_h, rust_h, nonpose),
        "mixed_same_or_adjacent_pose": metrics(clean_h, rust_h, mixed_pose),
        "all": metrics(clean_h, rust_h, np.ones((75, 75), dtype=bool)),
    }
    reconstructed_h = {
        "visual_only_nonadjacent_pose": metrics(
            clean_h, h_parts["visual"], visual_only
        ),
        "imu_only_nonpose_after_damping": metrics(
            clean_h - h_parts["damping"], h_parts["imu"], nonpose
        ),
        # In pose blocks the Rust IMU contribution is subtracted as a
        # conditional visual residual; this is explicitly not a clean native
        # per-factor oracle because only aggregate clean H/b was captured.
        "pose_conditional_after_rust_imu_and_damping": metrics(
            clean_h - h_parts["imu"] - h_parts["damping"],
            h_parts["visual"],
            pose_pose,
        ),
    }

    local_semantic = local // 3
    b_groups = []
    for name, start, end in (
        ("pose6", 0, 6),
        ("velocity3", 6, 9),
        ("bias_gyro3", 9, 12),
        ("bias_accel3", 12, 15),
    ):
        lanes = np.concatenate(
            [np.arange(frame_id * 15 + start, frame_id * 15 + end) for frame_id in range(5)]
        )
        mask = np.zeros(75, dtype=bool)
        mask[lanes] = True
        b_groups.append(
            {
                "group": name,
                "lanes": int(len(lanes)),
                "actual": metrics(clean_b, rust_b, mask),
                "mismatch_lanes": mismatch_count(clean_b, rust_b, mask),
                "conditional_after_rust_imu": metrics(
                    clean_b - b_parts["imu"], b_parts["visual"], mask
                ),
            }
        )

    # The four state differences map to observations/IMU links at the factor
    # boundary.  Velocity is not a visual input; quaternion lanes are.
    target_rows = {
        int(frame_id): sum(
            2
            for factor in detail["landmark_factors"]
            for observation in factor["observations"]
            if int(observation["target_frame_id"]) == frame_id
        )
        for frame_id in range(5)
    }
    changed_lane_implications = []
    for mismatch in state_mismatches:
        frame_id = int(mismatch["frame_id"])
        visual_rows = target_rows[frame_id] if mismatch["field"] == "quaternion_xyzw" else 0
        if frame_id == 0:
            imu_factors = []
        elif frame_id == 4:
            imu_factors = [3]
        else:
            imu_factors = [frame_id - 1, frame_id]
        changed_lane_implications.append(
            {
                "frame_id": frame_id,
                "field": mismatch["field"],
                "lane": mismatch["lane"],
                "visual_target_rows": int(visual_rows),
                "imu_factor_indices": imu_factors,
                "imu_rows_exposed": int(15 * len(imu_factors)),
                "note": (
                    "visual reprojection input"
                    if visual_rows
                    else "no direct visual dependency; IMU/state only"
                ),
            }
        )

    changed_visual_rows = sum(
        2
        for factor in detail["landmark_factors"]
        for observation in factor["observations"]
        if int(observation["target_frame_id"]) in (2, 4)
    )

    mismatch_bits_h = bits_array(clean_h) != bits_array(rust_h)
    mismatch_bits_b = bits_array(clean_b) != bits_array(rust_b)
    state_block_mismatch = [
        [
            int(np.count_nonzero(mismatch_bits_h[i * 15 : (i + 1) * 15, j * 15 : (j + 1) * 15]))
            for j in range(5)
        ]
        for i in range(5)
    ]
    semantic_mismatch = [
        [
            int(
                np.count_nonzero(
                    mismatch_bits_h.reshape(5, 15, 5, 15)[
                        :, a * 3 : (a + 1) * 3, :, b * 3 : (b + 1) * 3
                    ]
                )
            )
            for b in range(4)
        ]
        for a in range(4)
    ]

    output = {
        "schema": "visloc.m7de_hb_state_residual.v1",
        "date": "2026-08-23 JST",
        "status": "read-only analysis; production source unchanged",
        "provenance": {
            "clean_hb": str(HB_PATH.relative_to(ROOT)).replace("\\", "/"),
            "clean_states": str(STATE_PATH.relative_to(ROOT)).replace("\\", "/"),
            "rust_detail": str(RUST_PATH.relative_to(ROOT)).replace("\\", "/"),
            "rust_snapshot": {
                "record": detail["record"],
                "phase": detail["phase"],
                "iteration": detail["iteration"],
                "trial": detail["trial"],
                "frame_id": detail["frame_id"],
                "factor_count": detail["factor_count"],
                "numeric_types": detail["numeric_types"],
            },
            "clean_h_layout": hb["dimensions"]["H"]["storage"],
            "comparison": "clean native binary32 versus Rust snapshot values cast to binary32",
        },
        "overall": {
            "H": {
                "cells": 5625,
                "bit_mismatches": int(np.count_nonzero(mismatch_bits_h)),
                "max_abs": finite(np.max(np.abs(clean_h - rust_h))),
                "l2": finite(np.linalg.norm(clean_h - rust_h)),
            },
            "b": {
                "lanes": 75,
                "bit_mismatches": int(np.count_nonzero(mismatch_bits_b)),
                "max_abs": finite(np.max(np.abs(clean_b - rust_b))),
                "l2": finite(np.linalg.norm(clean_b - rust_b)),
            },
            "H_state_block_mismatch_5x5": state_block_mismatch,
            "H_semantic_mismatch_4x4": semantic_mismatch,
            "b_semantic_groups": b_groups,
        },
        "state_comparison": {
            **state_summary,
            "changed_lane_factor_implications": changed_lane_implications,
        },
        "factor_partition": partition,
        "residuals": {
            "actual_clean_vs_rust": actual_h,
            "factor_reconstruction_support_exact": reconstructed_h,
            "b_groups": b_groups,
            "mask_definitions": {
                "visual_only_nonadjacent_pose": "pose6 x pose6 cells with |frame_row-frame_col| > 1; no IMU or damping row support",
                "imu_only_nonpose": "any cell with a velocity/gyro-bias/accel-bias local lane; visual rows have zero support there",
                "mixed_same_or_adjacent_pose": "remaining pose6 x pose6 cells (diagonal or adjacent frame pair)",
            },
        },
        "frontier": {
            "state_exactification": {
                "current_state_bit_mismatches": 4,
                "target_after_exactification": 0,
                "visual_rows_exposed_by_quaternion_lanes": int(changed_visual_rows),
                "visual_rows_total": 1168,
                "visual_row_exposure_fraction": finite(changed_visual_rows / 1168),
                "imu_rows_exposed_by_changed_lanes": 45,
                "imu_rows_total": 60,
                "imu_row_exposure_fraction": 0.75,
                "changed_imu_factors": [1, 2, 3],
                "interpretation": "Exacting the four native state lanes removes the only observed state-input gate; row exposure is a sensitivity bound, not a guaranteed H/b bit-mismatch reduction.",
            },
            "remaining_kernel_residual_floor_if_state_fix_only": {
                "visual_only_H_bit_mismatches": actual_h["visual_only_nonadjacent_pose"]["bit_mismatches"],
                "imu_only_nonpose_H_bit_mismatches": actual_h["imu_only_nonpose"]["bit_mismatches"],
                "mixed_pose_H_bit_mismatches": actual_h["mixed_same_or_adjacent_pose"]["bit_mismatches"],
                "pose_b_bit_mismatches": b_groups[0]["mismatch_lanes"],
                "imu_only_nonpose_b_bit_mismatches": sum(
                    item["mismatch_lanes"] for item in b_groups[1:]
                ),
                "note": "These are clean-vs-current-Rust residuals on disjoint supports. Native per-factor clean rows were not captured, so they quantify the remaining visual/IMU kernel boundary without claiming a post-fix exact count.",
            },
        },
    }

    OUT_PATH.write_text(json.dumps(output, indent=2, sort_keys=False) + "\n", encoding="utf-8")
    print(f"wrote {OUT_PATH}")


if __name__ == "__main__":
    main()
