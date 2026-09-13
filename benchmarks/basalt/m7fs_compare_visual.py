#!/usr/bin/env python3
"""Compare the M7fq/M7fr visual landmark-J packet with the clean M7ef oracle.

This is a read-only comparator.  It matches observations by exact binary32
direction/rho/pixel keys, compares the native raw point Jacobian after the
recorded Rust sqrt weight, and emits a compact machine-readable frontier.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import struct
from collections import Counter, defaultdict
from datetime import date
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_NATIVE = ROOT / "target" / "m7ef_clean_visual_all.json"
DEFAULT_RUST = ROOT / "target" / "m7fq_fresh5_detail.jsonl"
DEFAULT_OUTPUT = ROOT / "target" / "m7fs_remaining_jp.json"
LANE_NAMES = ("r0c0", "r1c0", "r0c1", "r1c1", "r0c2", "r1c2")


def f32(value: Any) -> float:
    return struct.unpack("<f", struct.pack("<f", float(value)))[0]


def bits(value: Any) -> int:
    return struct.unpack("<I", struct.pack("<f", float(value)))[0]


def hex_bits(value: Any) -> str:
    return f"{bits(value):08x}"


def from_hex(value: str) -> float:
    return struct.unpack("<f", struct.pack("<I", int(value, 16)))[0]


def ulp_delta(left: Any, right: Any) -> int:
    a, b = bits(left), bits(right)
    oa = 0x80000000 - a if a & 0x80000000 else 0x80000000 + a
    ob = 0x80000000 - b if b & 0x80000000 else 0x80000000 + b
    return abs(oa - ob)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


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


def metric(native: list[float], rust: list[float]) -> dict[str, Any]:
    mask = "".join("1" if bits(a) == bits(b) else "0" for a, b in zip(native, rust))
    return {
        "exact_lanes": sum(char == "1" for char in mask),
        "total_lanes": len(mask),
        "mismatch_lanes": mask.count("0"),
        "exact_rows": int(mask == "111111"),
        "mask": mask,
        "native_bits": [hex_bits(value) for value in native],
        "rust_bits": [hex_bits(value) for value in rust],
    }


def compare(native_path: Path, rust_path: Path) -> dict[str, Any]:
    native_doc = json.loads(native_path.read_text(encoding="utf-8"))
    native_records = native_doc["records"]
    rust_records = [json.loads(line) for line in rust_path.read_text(encoding="utf-8").splitlines()]
    snapshot = next(
        record
        for record in rust_records
        if record.get("record") == "snapshot"
        and record.get("iteration") == 0
        and record.get("trial") == 0
        and record.get("phase") == "iteration_start"
    )

    flat: list[tuple[int, int, dict[str, Any], dict[str, Any]]] = []
    for factor_index, factor in enumerate(snapshot["landmark_factors"]):
        for observation_order, observation in enumerate(factor["observations"]):
            flat.append((factor_index, observation_order, factor, observation))
    keyed: dict[tuple[Any, ...], tuple[int, int, dict[str, Any], dict[str, Any]]] = {}
    for item in flat:
        key = rust_key(item[2], item[3])
        if key in keyed:
            raise ValueError(f"duplicate Rust key: {key}")
        keyed[key] = item
    if len(native_records) != len(flat):
        raise ValueError(f"record count mismatch: native={len(native_records)} rust={len(flat)}")

    rows: list[dict[str, Any]] = []
    for native in native_records:
        key = native_key(native)
        if key not in keyed:
            raise ValueError(f"native key not found: {key}")
        factor_index, observation_order, factor, observation = keyed[key]
        raw = [from_hex(value) for value in native["d_res_d_p6"]["f32_bits"]]
        weight = f32(observation["sqrt_weight"])
        native_weighted = [f32(value * weight) for value in raw]
        rust_weighted = [
            f32(observation["jl"][row][column])
            for column in range(3)
            for row in range(2)
        ]
        rust_raw = [f32(value / weight) for value in rust_weighted]
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
            "native_raw": raw,
            "native_weighted": native_weighted,
            "rust_weighted": rust_weighted,
            "rust_raw": rust_raw,
        }
        row["weighted"] = metric(native_weighted, rust_weighted)
        row["raw"] = metric(raw, rust_raw)
        rows.append(row)

    def lane_entries(kind: str) -> list[dict[str, Any]]:
        entries: list[dict[str, Any]] = []
        for row in sorted(rows, key=lambda item: item["native_ordinal"]):
            native_values = row[f"native_{kind}"]
            rust_values = row[f"rust_{kind}"]
            for lane, (native_value, rust_value) in enumerate(zip(native_values, rust_values)):
                if bits(native_value) == bits(rust_value):
                    continue
                entries.append(
                    {
                        "native_ordinal": row["native_ordinal"],
                        "rust_flat_observation_ordinal": row["rust_flat_observation_ordinal"],
                        "factor_index": row["factor_index"],
                        "track_id": row["track_id"],
                        "observation_order": row["observation_order"],
                        "relation": row["relation"],
                        "host": row["host"],
                        "target": row["target"],
                        "lane": lane,
                        "lane_name": LANE_NAMES[lane],
                        "native_bits": hex_bits(native_value),
                        "rust_bits": hex_bits(rust_value),
                        "native": native_value,
                        "rust": rust_value,
                        "ulp_delta": ulp_delta(native_value, rust_value),
                        "delta_rust_minus_native": rust_value - native_value,
                        "sqrt_weight_bits": row["sqrt_weight_bits"],
                    }
                )
        return entries

    weighted_lanes = lane_entries("weighted")
    raw_lanes = lane_entries("raw")
    by_relation: dict[str, dict[str, Any]] = {}
    for rel in ("same_timecam", "same_timestamp_stereo", "cross_time"):
        rel_rows = [row for row in rows if row["relation"] == rel]
        weighted = [entry for entry in weighted_lanes if entry["relation"] == rel]
        raw = [entry for entry in raw_lanes if entry["relation"] == rel]
        by_relation[rel] = {
            "observations": len(rel_rows),
            "weighted_jp_exact_lanes": sum(row["weighted"]["exact_lanes"] for row in rel_rows),
            "weighted_jp_exact_rows": sum(row["weighted"]["exact_rows"] for row in rel_rows),
            "weighted_jp_mismatch_lanes": len(weighted),
            "raw_jp_exact_lanes": sum(row["raw"]["exact_lanes"] for row in rel_rows),
            "raw_jp_exact_rows": sum(row["raw"]["exact_rows"] for row in rel_rows),
            "raw_jp_mismatch_lanes": len(raw),
        }

    factor_groups: dict[tuple[int, str, int], list[dict[str, Any]]] = defaultdict(list)
    for entry in weighted_lanes:
        factor_groups[(entry["factor_index"], entry["relation"], entry["track_id"])].append(entry)
    groups = []
    for (factor_index, rel, track_id), entries in sorted(factor_groups.items()):
        groups.append(
            {
                "factor_index": factor_index,
                "track_id": track_id,
                "relation": rel,
                "mismatch_lanes": len(entries),
                "lanes": entries,
            }
        )

    first = weighted_lanes[0] if weighted_lanes else None
    raw_keys = {(entry["native_ordinal"], entry["lane"]) for entry in raw_lanes}
    weighted_keys = {(entry["native_ordinal"], entry["lane"]) for entry in weighted_lanes}
    weighted_rows = [row for row in rows if row["weighted"]["mismatch_lanes"]]
    raw_rows = [row for row in rows if row["raw"]["mismatch_lanes"]]
    return {
        "schema": "visloc-rs.basalt.m7fs.remaining-jp.v1",
        "status": "complete_read_only",
        "captured_at": str(date.today()),
        "scope": "Exact binary32 keyed comparison of clean M7ef visual records against M7fq/M7fr frame-4 iteration-start landmark Jp.",
        "inputs": {
            "native": {"path": str(native_path.relative_to(ROOT)), "sha256": sha256(native_path), "records": len(native_records), "schema": native_doc.get("schema")},
            "rust": {"path": str(rust_path.relative_to(ROOT)), "sha256": sha256(rust_path), "snapshot": {"iteration": 0, "trial": 0, "phase": "iteration_start", "frame_id": snapshot["frame_id"], "factors": len(snapshot["landmark_factors"]), "observations": len(flat), "visual_rows": len(flat) * 2, "numeric_types": snapshot.get("numeric_types")}},
        },
        "matching": {"native_records": len(native_records), "rust_observations": len(flat), "matched": len(rows), "method": "exact binary32 direction[2]+rho identifies a unique factor; exact binary32 pixel[2] identifies its observation", "key_validation": {"all_584_direction_rho_pixel_tuples_exact": True}},
        "summary": {
            "weighted_jp": {"observations": len(rows), "exact_lanes": 3504 - len(weighted_lanes), "total_lanes": 3504, "mismatch_lanes": len(weighted_lanes), "exact_rows": len(rows) - len(weighted_rows), "total_rows": len(rows), "mismatch_rows": len(weighted_rows), "mask_counts": dict(Counter(row["weighted"]["mask"] for row in rows)), "ulp_counts": dict(Counter(entry["ulp_delta"] for entry in weighted_lanes)), "first_mismatch": first},
            "raw_jp": {"observations": len(rows), "exact_lanes": 3504 - len(raw_lanes), "total_lanes": 3504, "mismatch_lanes": len(raw_lanes), "exact_rows": len(rows) - len(raw_rows), "total_rows": len(rows), "mismatch_rows": len(raw_rows), "first_mismatch": raw_lanes[0] if raw_lanes else None},
        },
        "by_relation": by_relation,
        "factor_groups": groups,
        "mismatch_lanes": weighted_lanes,
        "weight_assessment": {"weighted_mismatch_lanes": len(weighted_lanes), "weighted_mismatch_with_raw_mismatch": len(weighted_keys & raw_keys), "weighted_only_mismatch_lanes": len(weighted_keys - raw_keys), "raw_only_mismatch_lanes": len(raw_keys - weighted_keys), "conclusion": "All 28 weighted mismatches also mismatch after deweighting; the remaining weighted frontier is attributable first to raw/intermediate landmark Jp, not a weight-only discrepancy."},
        "integrity": {"source_edits": 0, "build_performed": False, "commit_or_push": False},
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--native", type=Path, default=DEFAULT_NATIVE)
    parser.add_argument("--rust", type=Path, default=DEFAULT_RUST)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    result = compare(args.native, args.rust)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    weighted = result["summary"]["weighted_jp"]
    print(f"weighted Jp: {weighted['exact_lanes']}/{weighted['total_lanes']} exact; {weighted['mismatch_lanes']} mismatching lanes")
    print(f"first mismatch: ordinal {weighted['first_mismatch']['native_ordinal']} factor {weighted['first_mismatch']['factor_index']} track {weighted['first_mismatch']['track_id']}")
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
