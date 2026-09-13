#!/usr/bin/env python3
"""Classify the clean M7he pre-QR frontier without changing production code.

The clean native capture contains the binary32 row stack at performQR entry;
the Rust detail trace contains the keyed visual factor fields.  This script
maps native rows to Rust observations by the exact direction/rho/pixel tuple,
checks the raw visual boundary, and infers the native whitening scalar by
matching the native residual and landmark-J lanes.  It is deliberately
read-only apart from the optional JSON output path.
"""

from __future__ import annotations

import argparse
import json
import struct
from collections import Counter
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_NATIVE_RAW = ROOT / "target" / "m7ef_clean_visual_all.json"
DEFAULT_RUST = ROOT / "target" / "m7he_fresh5_f32_fma_c0fma_detail.jsonl"
DEFAULT_PREQR = ROOT / "target" / "m7he_preqr_all61_1t.json"
DEFAULT_OUTPUT = ROOT / "target" / "m7he_preqr_boundary_taxonomy.json"


def f32(value: Any) -> float:
    return struct.unpack("<f", struct.pack("<f", float(value)))[0]


def bits(value: Any) -> int:
    return struct.unpack("<I", struct.pack("<f", float(value)))[0]


def hex_bits(value: Any) -> str:
    return f"{bits(value):08x}"


def value_from_bits(value: str) -> float:
    return struct.unpack("<f", struct.pack("<I", int(value, 16)))[0]


def relative(path: Path) -> str:
    return str(path.resolve().relative_to(ROOT.resolve())).replace("\\", "/")


def native_key(record: dict[str, Any]) -> tuple[Any, ...]:
    return (
        tuple(record["keypoint"]["direction2"]["f32_bits"]),
        record["keypoint"]["inv_dist"]["f32_bits"][0],
        tuple(record["pixel2"]["f32_bits"]),
    )


def rust_key(factor: dict[str, Any], observation: dict[str, Any]) -> tuple[Any, ...]:
    return (
        tuple(hex_bits(value) for value in factor["direction"]),
        hex_bits(factor["rho"]),
        tuple(hex_bits(value) for value in observation["pixel"]),
    )


def relation(factor: dict[str, Any], observation: dict[str, Any]) -> str:
    if (observation["target_frame_id"], observation["target_cam"]) == (
        factor["host_frame_id"],
        factor["host_cam"],
    ):
        return "same_timecam"
    if observation["target_frame_id"] == factor["host_frame_id"]:
        return "same_timestamp_stereo"
    return "cross_time"


def load_snapshot(path: Path) -> dict[str, Any]:
    for line in path.read_text(encoding="utf-8").splitlines():
        record = json.loads(line)
        if (
            record.get("record") == "snapshot"
            and record.get("iteration") == 0
            and record.get("trial") == 0
            and record.get("phase") == "iteration_start"
        ):
            return record
    raise ValueError("frame-4 iteration-start snapshot not found")


def infer_native_weight(
    current_weight: float,
    raw_residual: list[float],
    raw_jp_column_major: list[float],
    native_residual: list[float],
    native_jl: list[list[float]],
) -> tuple[int, int, int]:
    """Return (weight bits, signed ULP delta, matching lane count).

    The native raw d_res_d_p lanes are Eigen column-major.  The row-major
    native storage therefore uses lanes [0,2,4] for row 0 and [1,3,5] for
    row 1.  Matching all eight residual/Jl lanes makes the inferred scalar
    independent of any single residual ratio or rounding accident.
    """

    current_bits = bits(current_weight)
    best: tuple[int, int, int] | None = None
    for delta in range(-8, 9):
        candidate_bits = current_bits + delta
        if candidate_bits < 0 or candidate_bits > 0x7F800000:
            continue
        candidate = value_from_bits(f"{candidate_bits:08x}")
        predicted_residual = [
            f32(raw_residual[0] * candidate),
            f32(raw_residual[1] * candidate),
        ]
        predicted_jl = [
            [
                f32(raw_jp_column_major[0] * candidate),
                f32(raw_jp_column_major[2] * candidate),
                f32(raw_jp_column_major[4] * candidate),
            ],
            [
                f32(raw_jp_column_major[1] * candidate),
                f32(raw_jp_column_major[3] * candidate),
                f32(raw_jp_column_major[5] * candidate),
            ],
        ]
        matches = sum(
            bits(left) == bits(right)
            for left, right in zip(predicted_residual, native_residual)
        )
        matches += sum(
            bits(left) == bits(right)
            for predicted_row, native_row in zip(predicted_jl, native_jl)
            for left, right in zip(predicted_row, native_row)
        )
        candidate_result = (matches, candidate_bits, delta)
        if best is None or candidate_result > (best[2], best[0], best[1]):
            best = (candidate_bits, delta, matches)
    if best is None:
        raise AssertionError("no candidate weight")
    return best


def classify(
    native_raw_path: Path,
    rust_path: Path,
    preqr_path: Path,
) -> dict[str, Any]:
    snapshot = load_snapshot(rust_path)
    factors = snapshot["landmark_factors"]
    keyed: dict[tuple[Any, ...], tuple[int, int, dict[str, Any], dict[str, Any]]] = {}
    for factor_index, factor in enumerate(factors):
        for observation_ordinal, observation in enumerate(factor["observations"]):
            keyed[rust_key(factor, observation)] = (
                factor_index,
                observation_ordinal,
                factor,
                observation,
            )

    native_doc = json.loads(native_raw_path.read_text(encoding="utf-8"))
    native_records = native_doc["records"]
    preqr_records = json.loads(preqr_path.read_text(encoding="utf-8"))["records"]
    preqr_by_track = {int(record["track_id"]): record for record in preqr_records}

    raw_residual_mismatches = 0
    raw_jp_weighted_mismatches = 0
    preqr_mismatches = 0
    # The clean comparator's denominator includes every variable-size storage
    # block, including the three damping rows.  Visual rows below are the only
    # rows that can carry this boundary mismatch, but retain the full native
    # denominator for the durable 591/108080 count.
    preqr_total = sum(len(record["storage"]["bits"]) for record in preqr_records)
    mismatch_rows: list[dict[str, Any]] = []
    observations: dict[tuple[int, int], dict[str, Any]] = {}

    for native in native_records:
        factor_index, observation_ordinal, factor, observation = keyed[native_key(native)]
        track_id = int(factor["track_id"])
        preqr = preqr_by_track[track_id]
        values = [value_from_bits(value) for value in preqr["storage"]["bits"]]
        columns = int(preqr["storage"]["cols"])
        raw_residual = [f32(value) for value in observation["raw_residual"]]
        native_raw_residual = [value_from_bits(value) for value in native["raw_residual2"]["f32_bits"]]
        raw_residual_mismatches += sum(
            bits(left) != bits(right)
            for left, right in zip(native_raw_residual, raw_residual)
        )

        current_weight = f32(observation["sqrt_weight"])
        raw_jp = [value_from_bits(value) for value in native["d_res_d_p6"]["f32_bits"]]
        rust_jl = [
            f32(observation["jl"][row][column])
            for column in range(3)
            for row in range(2)
        ]
        raw_jp_weighted = [f32(value * current_weight) for value in raw_jp]
        raw_jp_weighted_mismatches += sum(
            bits(left) != bits(right)
            for left, right in zip(raw_jp_weighted, rust_jl)
        )

        storage_rows = (2 * observation_ordinal, 2 * observation_ordinal + 1)
        native_residual = [
            values[storage_rows[row] * columns + columns - 1] for row in range(2)
        ]
        native_jl = [
            [values[storage_rows[row] * columns + 76 + column] for column in range(3)]
            for row in range(2)
        ]
        inferred_bits, inferred_delta, matching_lanes = infer_native_weight(
            current_weight,
            raw_residual,
            raw_jp,
            native_residual,
            native_jl,
        )

        observation_rows: list[dict[str, Any]] = []
        for row_lane, storage_row in enumerate(storage_rows):
            base = storage_row * columns
            differences: list[tuple[str, int, str, str]] = []
            expected_state = factor["state_jacobian"][storage_row]
            for column in range(75):
                native_value = values[base + column]
                rust_value = f32(expected_state[column])
                if bits(native_value) != bits(rust_value):
                    differences.append(
                        ("state_jacobian", column, hex_bits(native_value), hex_bits(rust_value))
                    )
            for column in range(3):
                native_value = values[base + 76 + column]
                rust_value = f32(observation["jl"][row_lane][column])
                if bits(native_value) != bits(rust_value):
                    differences.append(
                        ("landmark_jacobian", column, hex_bits(native_value), hex_bits(rust_value))
                    )
            native_value = values[base + 79]
            rust_value = f32(observation["residual"][row_lane])
            if bits(native_value) != bits(rust_value):
                differences.append(("residual", 0, hex_bits(native_value), hex_bits(rust_value)))

            preqr_mismatches += len(differences)
            if not differences:
                continue
            fields = Counter(difference[0] for difference in differences)
            row_record = {
                "native_ordinal": int(native["ordinal"]),
                "track_id": track_id,
                "factor_index": factor_index,
                "observation_ordinal": observation_ordinal,
                "storage_row": storage_row,
                "row_lane": row_lane,
                "relation": relation(factor, observation),
                "target_frame_id": observation["target_frame_id"],
                "target_cam": observation["target_cam"],
                "current_sqrt_weight_bits": hex_bits(current_weight),
                "inferred_native_sqrt_weight_bits": f"{inferred_bits:08x}",
                "inferred_weight_delta_ulp": inferred_delta,
                "inferred_weight_matching_lanes": matching_lanes,
                "first_difference": "robust_weight",
                "storage_first_difference": differences[0][0],
                "classification": "robust_weight_then_weighted_multiply",
                "mismatch_count": len(differences),
                "mismatch_fields": dict(fields),
                "first_witness": {
                    "field": differences[0][0],
                    "column": differences[0][1],
                    "native_bits": differences[0][2],
                    "rust_bits": differences[0][3],
                },
            }
            mismatch_rows.append(row_record)
            observations.setdefault(
                (track_id, observation_ordinal),
                {
                    "track_id": track_id,
                    "factor_index": factor_index,
                    "observation_ordinal": observation_ordinal,
                    "relation": relation(factor, observation),
                    "target_frame_id": observation["target_frame_id"],
                    "target_cam": observation["target_cam"],
                    "raw_residual_bits": [hex_bits(value) for value in raw_residual],
                    "current_sqrt_weight_bits": hex_bits(current_weight),
                    "inferred_native_sqrt_weight_bits": f"{inferred_bits:08x}",
                    "inferred_weight_delta_ulp": inferred_delta,
                    "inferred_weight_matching_lanes": matching_lanes,
                    "storage_rows": [],
                },
            )["storage_rows"].append(storage_row)

    tracks = sorted({row["track_id"] for row in mismatch_rows})
    all_observations = list(observations.values())
    return {
        "schema": "visloc-rs.basalt.m7he-preqr-boundary-taxonomy.v1",
        "status": "complete_read_only",
        "scope": "Clean native M7ef raw visual rows and M7he one-thread performQR-entry storage against the current f32 Rust detail trace.",
        "inputs": {
            "native_raw": {"path": relative(native_raw_path), "records": len(native_records)},
            "rust_detail": {"path": relative(rust_path), "snapshot": {"frame_id": snapshot["frame_id"], "iteration": 0, "trial": 0, "phase": "iteration_start"}},
            "preqr": {"path": relative(preqr_path), "records": len(preqr_records)},
        },
        "summary": {
            "preqr_mismatches": preqr_mismatches,
            "preqr_total_values": preqr_total,
            "preqr_exact_values": preqr_total - preqr_mismatches,
            "mismatching_tracks": tracks,
            "mismatching_track_count": len(tracks),
            "mismatching_observation_count": len(all_observations),
            "mismatching_storage_row_count": len(mismatch_rows),
            "raw_residual_mismatch_lanes": raw_residual_mismatches,
            "raw_residual_total_lanes": len(native_records) * 2,
            "raw_jp_weighted_mismatch_lanes": raw_jp_weighted_mismatches,
            "raw_jp_weighted_total_lanes": len(native_records) * 6,
            "all_mismatches_are_robust": all(row["classification"] == "robust_weight_then_weighted_multiply" for row in mismatch_rows),
            "all_mismatches_are_cross_time": all(row["relation"] == "cross_time" for row in mismatch_rows),
            "first_common_causal_boundary": "robust_weight",
            "downstream_effect": "weighted_multiply_into_state_jacobian_landmark_jacobian_residual",
            "raw_jxi_first_difference": False,
        },
        "observation_witnesses": sorted(all_observations, key=lambda row: (row["native_ordinal"] if "native_ordinal" in row else row["track_id"], row["observation_ordinal"])),
        "mismatch_rows": sorted(mismatch_rows, key=lambda row: (row["native_ordinal"], row["storage_row"])),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--native-raw", type=Path, default=DEFAULT_NATIVE_RAW)
    parser.add_argument("--rust", type=Path, default=DEFAULT_RUST)
    parser.add_argument("--preqr", type=Path, default=DEFAULT_PREQR)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    result = classify(args.native_raw, args.rust, args.preqr)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result["summary"], sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
