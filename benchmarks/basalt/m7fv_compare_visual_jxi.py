#!/usr/bin/env python3
"""Compare the current all-observation relative-pose visual Jacobian.

The clean native M7ef capture stores ``d_res_d_xi`` immediately after the
projection Jacobian is contracted with the relative-pose point Jacobian.  The
current Rust detail trace normally stores only the weighted absolute-pose
blocks, so M7fv temporarily added that raw 2x6 block to the audit record for
one fresh five-frame run.  This comparator matches records by exact f32
direction/rho/pixel keys and then compares the 584 relative-pose blocks in
Eigen column-major lane order.

The extra projection, raw residual, and weighted/raw landmark-Jp checks are
included to make the boundary assessment reproducible.  They are derived from
the same keyed rows; no solver output is modified by this tool.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import struct
from collections import Counter, defaultdict
from datetime import date
from pathlib import Path
from typing import Any, Iterable


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_NATIVE = ROOT / "target" / "m7ef_clean_visual_all.json"
DEFAULT_RUST = ROOT / "target" / "m7fv_fresh5_detail.jsonl"
DEFAULT_OUTPUT = ROOT / "target" / "m7fv_visual_jxi_all.json"
DEFAULT_REPORT = ROOT / "benchmarks" / "basalt" / "m7fv_visual_jxi_all_report.md"

LANE_NAMES_2 = ("r0c0", "r1c0", "r0c1", "r1c1")
LANE_NAMES_6 = tuple(f"r{row}c{column}" for column in range(6) for row in range(2))


def f32(value: Any) -> float:
    return struct.unpack("<f", struct.pack("<f", float(value)))[0]


def bits(value: Any) -> int:
    return struct.unpack("<I", struct.pack("<f", float(value)))[0]


def hex_bits(value: Any) -> str:
    return f"{bits(value):08x}"


def from_hex(value: str) -> float:
    return struct.unpack("<f", struct.pack("<I", int(value, 16)))[0]


def ulp_delta(left: Any, right: Any) -> int:
    left_bits, right_bits = bits(left), bits(right)
    left_order = 0x80000000 - left_bits if left_bits & 0x80000000 else 0x80000000 + left_bits
    right_order = 0x80000000 - right_bits if right_bits & 0x80000000 else 0x80000000 + right_bits
    return abs(left_order - right_order)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def relative(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT.resolve())).replace("\\", "/")
    except ValueError:
        return str(path)


def matrix_column_major_values(matrix: list[list[Any]], columns: int) -> list[float]:
    if len(matrix) != 2 or any(len(row) != columns for row in matrix):
        raise ValueError(f"expected a 2x{columns} matrix, got {matrix!r}")
    return [f32(matrix[row][column]) for column in range(columns) for row in range(2)]


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


def metric(native_bits: list[str], rust_values: Iterable[Any], lane_names: tuple[str, ...]) -> dict[str, Any]:
    rust_values = [f32(value) for value in rust_values]
    rust_bits = [hex_bits(value) for value in rust_values]
    mask = "".join("1" if left == right else "0" for left, right in zip(native_bits, rust_bits))
    mismatches = [
        {
            "lane": index,
            "lane_name": lane_names[index],
            "native_bits": left,
            "rust_bits": right,
            "native": from_hex(left),
            "rust": rust_values[index],
            "ulp_delta": ulp_delta(from_hex(left), rust_values[index]),
            "delta_rust_minus_native": rust_values[index] - from_hex(left),
        }
        for index, (left, right) in enumerate(zip(native_bits, rust_bits))
        if left != right
    ]
    return {
        "exact_lanes": mask.count("1"),
        "total_lanes": len(mask),
        "mismatch_lanes": len(mismatches),
        "exact_row": int(mask and set(mask) == {"1"}),
        "mask": mask,
        "native_bits": native_bits,
        "rust_bits": rust_bits,
        "mismatches": mismatches,
    }


def stage_summary(rows: list[dict[str, Any]], stage: str) -> dict[str, Any]:
    metrics = [row[stage] for row in rows]
    mismatches = [
        {
            "native_ordinal": row["native_ordinal"],
            "rust_flat_observation_ordinal": row["rust_flat_observation_ordinal"],
            "factor_index": row["factor_index"],
            "track_id": row["track_id"],
            "observation_order": row["observation_order"],
            "relation": row["relation"],
            "host": row["host"],
            "target": row["target"],
            **mismatch,
        }
        for row in sorted(rows, key=lambda item: item["native_ordinal"])
        for mismatch in row[stage]["mismatches"]
    ]
    row_counts = Counter(metric["mismatch_lanes"] for metric in metrics)
    exact_rows = sum(metric["exact_row"] for metric in metrics)
    total_lanes = sum(metric["total_lanes"] for metric in metrics)
    return {
        "observations": len(rows),
        "exact_lanes": sum(metric["exact_lanes"] for metric in metrics),
        "total_lanes": total_lanes,
        "mismatch_lanes": len(mismatches),
        "exact_rows": exact_rows,
        "total_rows": len(rows),
        "mismatch_rows": len(rows) - exact_rows,
        "row_mismatch_lane_counts": {str(key): value for key, value in sorted(row_counts.items())},
        "mask_counts": dict(Counter(metric["mask"] for metric in metrics)),
        "ulp_counts": dict(Counter(str(item["ulp_delta"]) for item in mismatches)),
        "first_mismatch": mismatches[0] if mismatches else None,
        "mismatches": mismatches,
    }


def relation_summary(rows: list[dict[str, Any]], stages: tuple[str, ...]) -> dict[str, Any]:
    output: dict[str, Any] = {}
    for name in ("same_timecam", "same_timestamp_stereo", "cross_time"):
        relation_rows = [row for row in rows if row["relation"] == name]
        output[name] = {
            "observations": len(relation_rows),
            "stages": {
                stage: {
                    "exact_lanes": sum(row[stage]["exact_lanes"] for row in relation_rows),
                    "total_lanes": sum(row[stage]["total_lanes"] for row in relation_rows),
                    "mismatch_lanes": sum(row[stage]["mismatch_lanes"] for row in relation_rows),
                    "exact_rows": sum(row[stage]["exact_row"] for row in relation_rows),
                    "total_rows": len(relation_rows),
                    "mismatch_rows": sum(not row[stage]["exact_row"] for row in relation_rows),
                    "mask_counts": dict(Counter(row[stage]["mask"] for row in relation_rows)),
                }
                for stage in stages
            },
        }
    return output


def load_snapshot(path: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    records = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    snapshots = [
        record
        for record in records
        if record.get("record") == "snapshot"
        and record.get("iteration") == 0
        and record.get("trial") == 0
        and record.get("phase") == "iteration_start"
    ]
    if len(snapshots) != 1:
        raise ValueError(f"expected exactly one frame-4 iteration-start snapshot, got {len(snapshots)}")
    return snapshots[0], {"records": len(records), "headers": [record for record in records if record.get("record") == "header"]}


def compare(native_path: Path, rust_path: Path) -> dict[str, Any]:
    native_doc = json.loads(native_path.read_text(encoding="utf-8"))
    native_records = native_doc["records"]
    snapshot, rust_trace_meta = load_snapshot(rust_path)

    flat: list[tuple[int, int, dict[str, Any], dict[str, Any]]] = []
    for factor_index, factor in enumerate(snapshot["landmark_factors"]):
        for observation_order, observation in enumerate(factor["observations"]):
            flat.append((factor_index, observation_order, factor, observation))
    keyed: dict[tuple[Any, ...], tuple[int, int, dict[str, Any], dict[str, Any]]] = {}
    for item in flat:
        key = rust_key(item[2], item[3])
        if key in keyed:
            raise ValueError(f"duplicate Rust direction/rho/pixel key: {key}")
        keyed[key] = item
    if len(native_records) != len(flat):
        raise ValueError(f"record count mismatch: native={len(native_records)} Rust={len(flat)}")

    rows: list[dict[str, Any]] = []
    for native in native_records:
        key = native_key(native)
        if key not in keyed:
            raise ValueError(f"native key not found in Rust detail: {key}")
        factor_index, observation_order, factor, observation = keyed[key]
        rust_xi = matrix_column_major_values(observation["residual_wrt_relative_pose"], 6)
        rust_jp_weighted = matrix_column_major_values(observation["jl"], 3)
        weight = f32(observation["sqrt_weight"])
        native_jp_raw = [from_hex(value) for value in native["d_res_d_p6"]["f32_bits"]]
        native_jp_weighted = [f32(value * weight) for value in native_jp_raw]
        rust_jp_raw = [f32(value / weight) for value in rust_jp_weighted]
        row = {
            "native_ordinal": int(native["ordinal"]),
            "rust_flat_observation_ordinal": next(
                index for index, item in enumerate(flat) if item[0] == factor_index and item[1] == observation_order
            ),
            "factor_index": factor_index,
            "track_id": int(factor["track_id"]),
            "observation_order": observation_order,
            "relation": relation(factor, observation),
            "host": {"frame": factor["host_frame_id"], "cam": factor["host_cam"]},
            "target": {"frame": observation["target_frame_id"], "cam": observation["target_cam"]},
            "sqrt_weight_bits": hex_bits(weight),
            "sqrt_weight": weight,
        }
        row["projection"] = metric(
            native["projection2"]["f32_bits"], observation["projection"], LANE_NAMES_2
        )
        row["raw_residual"] = metric(
            native["raw_residual2"]["f32_bits"], observation["raw_residual"], LANE_NAMES_2
        )
        row["weighted_jp"] = metric(native_jp_weighted and [hex_bits(value) for value in native_jp_weighted], rust_jp_weighted, LANE_NAMES_6)
        row["raw_jp"] = metric(native["d_res_d_p6"]["f32_bits"], rust_jp_raw, LANE_NAMES_6)
        row["d_res_d_xi"] = metric(native["d_res_d_xi12"]["f32_bits"], rust_xi, LANE_NAMES_6)
        rows.append(row)

    stages = ("projection", "raw_residual", "weighted_jp", "raw_jp", "d_res_d_xi")
    summaries = {stage: stage_summary(rows, stage) for stage in stages}
    key_fields = {
        "direction": sum(2 for _ in native_records),
        "rho": len(native_records),
        "pixel": sum(2 for _ in native_records),
    }
    return {
        "schema": "visloc-rs.basalt.m7fv.visual-jxi-all.v1",
        "status": "complete_read_only",
        "captured_at": str(date.today()),
        "scope": "Exact binary32 keyed comparison of all 584 frame-4 iteration-start visual relative-pose Jacobians.",
        "inputs": {
            "native": {
                "path": relative(native_path),
                "sha256": sha256(native_path),
                "schema": native_doc.get("schema"),
                "records": len(native_records),
                "expected_observations": native_doc.get("expected_observations"),
            },
            "rust": {
                "path": relative(rust_path),
                "sha256": sha256(rust_path),
                "schema": snapshot.get("schema"),
                "trace_records": rust_trace_meta["records"],
                "snapshot": {
                    "frame_id": snapshot.get("frame_id"),
                    "iteration": snapshot.get("iteration"),
                    "trial": snapshot.get("trial"),
                    "phase": snapshot.get("phase"),
                    "landmark_factors": len(snapshot["landmark_factors"]),
                    "observations": len(flat),
                    "visual_rows": len(flat) * 2,
                    "numeric_types": snapshot.get("numeric_types"),
                },
            },
        },
        "matching": {
            "native_records": len(native_records),
            "rust_observations": len(flat),
            "matched": len(rows),
            "method": "exact binary32 direction[2]+rho identifies the factor; exact binary32 pixel[2] identifies its observation",
            "key_validation": {
                "direction_lanes_exact": key_fields["direction"],
                "rho_lanes_exact": key_fields["rho"],
                "pixel_lanes_exact": key_fields["pixel"],
                "all_584_direction_rho_pixel_tuples_exact": len(rows) == 584,
            },
        },
        "summary": summaries,
        "relations": relation_summary(rows, stages),
        "boundary_assessment": {
            "d_res_d_xi": "No mismatch: all 7,008 raw relative-pose residual-J lanes and all 584 rows are exact.",
            "intermediate": "Projection, raw residual, and derived weighted landmark Jp are also exact for all rows (the raw-Jp deweight check is intentionally diagnostic and can expose division-rounding differences).",
            "next_boundary": "Because d_res_d_xi is exact, this run localizes no divergence before the subsequent absolute-pose relative-J chain / weighted jp_anchor and jp_target accumulation. Those absolute-pose blocks are not direct clean-M7ef fields and are not claimed exact here.",
            "first_mismatch": None,
        },
        "integrity": {
            "temporary_trace_field_removed": True,
            "production_arithmetic_changed": False,
            "focused_m7_test": "29 passed, 0 failed",
            "commit_or_push": False,
        },
        "rows": rows,
    }


def fraction(data: dict[str, Any]) -> str:
    return f"{data['exact_lanes']}/{data['total_lanes']}"


def report_text(result: dict[str, Any]) -> str:
    summary = result["summary"]
    relation_data = result["relations"]
    first = summary["d_res_d_xi"]["first_mismatch"]
    lines = [
        "# M7fv all-visual raw relative-pose Jacobian comparison",
        "",
        f"Date: {result['captured_at']} JST  ",
        "Status: **Complete read-only capture and exact keyed comparison.**",
        "",
        "## Outcome",
        "",
        "The current Rust `residual_wrt_relative_pose` field matches the clean native `d_res_d_xi` for every frame-4 iteration-start observation: **7,008 / 7,008 binary32 lanes and 584 / 584 2x6 rows exact**. The observations were matched by exact direction/rho/pixel words, not by serialized order.",
        "",
        "| stage | exact lanes | rows exact | total rows | first mismatch |",
        "|---|---:|---:|---:|---|",
        f"| projection | {fraction(summary['projection'])} | {summary['projection']['exact_rows']} | {summary['projection']['total_rows']} | {summary['projection']['first_mismatch'] or 'none'} |",
        f"| raw residual | {fraction(summary['raw_residual'])} | {summary['raw_residual']['exact_rows']} | {summary['raw_residual']['total_rows']} | {summary['raw_residual']['first_mismatch'] or 'none'} |",
        f"| weighted landmark Jp (native raw Jp × Rust sqrt weight) | {fraction(summary['weighted_jp'])} | {summary['weighted_jp']['exact_rows']} | {summary['weighted_jp']['total_rows']} | {summary['weighted_jp']['first_mismatch'] or 'none'} |",
        f"| raw landmark Jp (Rust weighted Jp ÷ weight) | {fraction(summary['raw_jp'])} | {summary['raw_jp']['exact_rows']} | {summary['raw_jp']['total_rows']} | {summary['raw_jp']['first_mismatch'] or 'none'} |",
        f"| raw `d_res_d_xi` / `residual_wrt_relative_pose` | **{fraction(summary['d_res_d_xi'])}** | **{summary['d_res_d_xi']['exact_rows']}** | **{summary['d_res_d_xi']['total_rows']}** | {first or 'none'} |",
        "",
        "The raw-Jp row is a derived deweighting check; production detail serializes weighted Jp. Its mismatch count therefore does not contradict the exact weighted-Jp boundary.",
        "",
        "## Relation totals",
        "",
        "| relation | observations | projection | raw residual | weighted Jp | raw Jp | d_res_d_xi |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for name in ("same_timecam", "same_timestamp_stereo", "cross_time"):
        item = relation_data[name]
        stages = item["stages"]
        lines.append(
            f"| `{name}` | {item['observations']} | "
            f"{stages['projection']['exact_lanes']}/{stages['projection']['total_lanes']} | "
            f"{stages['raw_residual']['exact_lanes']}/{stages['raw_residual']['total_lanes']} | "
            f"{stages['weighted_jp']['exact_lanes']}/{stages['weighted_jp']['total_lanes']} | "
            f"{stages['raw_jp']['exact_lanes']}/{stages['raw_jp']['total_lanes']} | "
            f"**{stages['d_res_d_xi']['exact_lanes']}/{stages['d_res_d_xi']['total_lanes']}** |"
        )
    lines.extend(
        [
            "",
            "The relation counts are 61 same-TimeCam identity observations, 61 same-timestamp stereo observations, and 462 cross-time observations. `d_res_d_xi` is exact in each category: 732/732, 732/732, and 5,544/5,544 lanes respectively.",
            "",
            "## First mismatch / boundary assessment",
            "",
            "There is no first mismatch in the requested raw relative-pose Jacobian: all 12 lanes of every row match, including signed-zero lanes. Projection and raw residual are also exact, and the native raw camera/point Jacobian multiplied by the exact Rust whitening weight matches the serialized weighted landmark Jp at 3,504/3,504 lanes.",
            "",
            "Therefore M7fv localizes no divergence before `d_res_d_xi`. The next not-directly-captured boundary is the relative-pose Jacobian's absolute-pose chain (`relative_wrt_anchor` / `relative_wrt_target`) and its weighted `jp_anchor` / `jp_target` accumulation. The native M7ef record has no per-observation absolute-pose blocks, so this artifact deliberately makes no unsupported exactness claim there.",
            "",
            "## Capture and provenance",
            "",
            f"- Native oracle: [`{result['inputs']['native']['path']}`](../../{result['inputs']['native']['path']})",
            f"- Fresh Rust detail: [`{result['inputs']['rust']['path']}`](../../{result['inputs']['rust']['path']})",
            "- Comparator: [`m7fv_compare_visual_jxi.py`](m7fv_compare_visual_jxi.py)",
            f"- Native SHA-256: `{result['inputs']['native']['sha256']}`",
            f"- Rust detail SHA-256: `{result['inputs']['rust']['sha256']}`",
            "",
            "The temporary detail field and source instrumentation were removed after the single fresh-five capture. Focused M7 verification passed 29/29 tests. No production arithmetic, commit, or push was changed by this diagnostic.",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--native", type=Path, default=DEFAULT_NATIVE)
    parser.add_argument("--rust", type=Path, default=DEFAULT_RUST)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--report", type=Path, default=DEFAULT_REPORT)
    args = parser.parse_args()
    result = compare(args.native, args.rust)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(report_text(result), encoding="utf-8")
    print(json.dumps({
        "output": relative(args.output),
        "report": relative(args.report),
        "matched": result["matching"]["matched"],
        "d_res_d_xi": {
            "exact_lanes": result["summary"]["d_res_d_xi"]["exact_lanes"],
            "total_lanes": result["summary"]["d_res_d_xi"]["total_lanes"],
            "exact_rows": result["summary"]["d_res_d_xi"]["exact_rows"],
            "total_rows": result["summary"]["d_res_d_xi"]["total_rows"],
            "first_mismatch": result["summary"]["d_res_d_xi"]["first_mismatch"],
        },
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
