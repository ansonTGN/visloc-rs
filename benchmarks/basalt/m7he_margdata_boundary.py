#!/usr/bin/env python3
"""Summarize the generic 80-frame sqrt/FEJ/MargData boundary.

The estimator keeps a diagnostic MargData record for every eligible window
frame, but Basalt's mapper queue receives only records with a non-empty
``kfs_to_marg`` list.  This tool checks that distinction, the mixed AOM/FEJ
wire contract, and the algebraic ``J.T @ J``/``J.T @ r`` relationship without
depending on MH01 track IDs or a fixed packet count.

An optional upstream projection directory (the ``up_*.json`` files emitted by
``upstream_marg_dump.cpp``) is compared by normalized state timestamps.  The
upstream projection intentionally supplies structure only; its numeric
trajectory is a separate oracle boundary.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any


FRAME_PERIOD_NS = 50_000_000
DEFAULT_BASE_TIMESTAMP_NS = 1_403_636_579_763_555_584


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def frame_id(timestamp_ns: int, base_timestamp_ns: int) -> int | None:
    delta = timestamp_ns - base_timestamp_ns
    nearest = round(delta / FRAME_PERIOD_NS)
    return nearest if abs(delta - nearest * FRAME_PERIOD_NS) <= 1_000 else None


def matrix_values(data: dict[str, Any]) -> list[list[float]]:
    rows = int(data["rows"])
    cols = int(data["cols"])
    values = [float(value) for value in data["data"]]
    if len(values) != rows * cols:
        raise ValueError("matrix payload length does not match shape")
    # MatrixData is nalgebra column-major on disk.
    return [[values[column * rows + row] for column in range(cols)] for row in range(rows)]


def max_normal_equation_error(packet: dict[str, Any]) -> dict[str, float | None]:
    sqrt_j = matrix_values(packet["aom_sqrt_jacobian"])
    sqrt_r = [float(value) for value in packet["aom_sqrt_rhs"]]
    abs_h_data = packet.get("aom_abs_h")
    abs_b_data = packet.get("aom_abs_b")
    if abs_h_data is None or abs_b_data is None:
        return {"max_h": None, "max_b": None}
    abs_h = matrix_values(abs_h_data)
    abs_b = [float(value) for value in abs_b_data]
    rows = len(sqrt_j)
    cols = len(sqrt_j[0]) if rows else 0
    if len(abs_h) != cols or len(abs_h[0]) != cols or len(abs_b) != cols:
        raise ValueError("absolute system shape does not match square-root AOM")
    max_h = 0.0
    max_b = 0.0
    for left in range(cols):
        for right in range(cols):
            expected = sum(sqrt_j[row][left] * sqrt_j[row][right] for row in range(rows))
            max_h = max(max_h, abs(expected - abs_h[left][right]))
        expected_b = sum(sqrt_j[row][left] * sqrt_r[row] for row in range(rows))
        max_b = max(max_b, abs(expected_b - abs_b[left]))
    return {"max_h": max_h, "max_b": max_b}


def packet_contract(packet: dict[str, Any], path: Path) -> dict[str, Any]:
    aom = packet.get("aom_order", [])
    aom_order = [[int(block["frame_id"]), int(block["offset"]), int(block["dof"])] for block in aom]
    offsets_contiguous = all(
        int(left["offset"]) + int(left["dof"]) == int(right["offset"])
        for left, right in zip(aom, aom[1:])
    )
    aom_dof = sum(int(block["dof"]) for block in aom)
    sqrt_shape = [int(packet["aom_sqrt_jacobian"]["rows"]), int(packet["aom_sqrt_jacobian"]["cols"])]
    abs_shape = None
    if packet.get("aom_abs_h") is not None:
        abs_shape = [int(packet["aom_abs_h"]["rows"]), int(packet["aom_abs_h"]["cols"])]
    state_ids = [int(state["frame_id"]) for state in packet.get("frame_states", [])]
    pose_ids = [int(pose["frame_id"]) for pose in packet.get("frame_poses", [])]
    image_keys = [
        (int(image["frame_id"]), int(image["timestamp_ns"]), int(image["camera_id"]))
        for image in packet.get("of_images", [])
    ]
    row_counts = [int(value) for value in packet.get("row_counts", [])]
    normal = max_normal_equation_error(packet)
    return {
        "file": str(path),
        "file_sha256": sha256(path),
        "schema_version": int(packet.get("schema_version", 0)),
        "provenance_version": packet.get("provenance_version"),
        "sqrt_shape": sqrt_shape,
        "abs_shape": abs_shape,
        "sqrt_rhs_len": len(packet.get("aom_sqrt_rhs", [])),
        "row_counts": row_counts,
        "row_count_sum": sum(row_counts),
        "aom_dof": aom_dof,
        "aom_order": aom_order,
        "aom_offsets_contiguous": offsets_contiguous,
        "aom_last_end": (int(aom[-1]["offset"]) + int(aom[-1]["dof"])) if aom else 0,
        "frame_pose_ids": pose_ids,
        "frame_state_ids": state_ids,
        "fej_flags": [bool(state.get("linearized", False)) for state in packet.get("frame_states", [])],
        "keyframes": [int(value) for value in packet.get("kfs_all", packet.get("keyframes", []))],
        "kfs_to_marg": [int(value) for value in packet.get("kfs_to_marg", [])],
        "image_count": len(image_keys),
        "image_keys_sorted": image_keys == sorted(image_keys),
        "image_frame_ids": sorted({key[0] for key in image_keys}),
        "used_imu": bool(packet.get("used_imu", False)),
        "diagnostic_prior_present": packet.get("prior") is not None,
        "normal_equation_max_abs": normal,
        "normal_equation_within_f32_roundoff": (
            normal["max_h"] is not None
            and normal["max_b"] is not None
            and float(normal["max_h"]) <= 2e-6
            and float(normal["max_b"]) <= 2e-6
        ),
    }


def upstream_candidates(path: Path, base_timestamp_ns: int) -> list[dict[str, Any]]:
    candidates = []
    for item in sorted(path.glob("up_*.json")):
        packet = read_json(item)
        states = [
            frame_id(int(state["timestamp_ns"]), base_timestamp_ns)
            for state in packet.get("frame_states", [])
        ]
        candidates.append({"path": item, "packet": packet, "state_ids": states})
    return candidates


def upstream_match(contract: dict[str, Any], candidates: list[dict[str, Any]], base_timestamp_ns: int) -> dict[str, Any] | None:
    if not candidates:
        return None
    states = contract["frame_state_ids"]
    exact = [candidate for candidate in candidates if candidate["state_ids"] == states]
    candidate = exact[0] if exact else min(
        candidates,
        key=lambda item: abs((item["state_ids"][-1] or 0) - (states[-1] if states else 0)),
    )
    packet = candidate["packet"]
    upstream_aom = [
        [frame_id(int(block["timestamp_ns"]), base_timestamp_ns), int(block["offset"]), int(block["dof"])]
        for block in packet.get("aom_order", [])
    ]
    # The compact contract stores only scalar summaries; compare the fields
    # that are independent of matrix numeric trajectory here.
    upstream_states = [frame_id(int(state["timestamp_ns"]), base_timestamp_ns) for state in packet.get("frame_states", [])]
    upstream_kfs = [frame_id(int(value), base_timestamp_ns) for value in packet.get("kfs_all", [])]
    upstream_targets = [frame_id(int(value), base_timestamp_ns) for value in packet.get("kfs_to_marg", [])]
    return {
        "file": str(candidate["path"]),
        "selection": "exact_state_timestamps" if exact else "nearest_state_timestamp",
        "state_ids_equal": states == upstream_states,
        "fej_flags_equal": contract["fej_flags"] == [bool(state.get("linearized", False)) for state in packet.get("frame_states", [])],
        "keyframes_equal": contract["keyframes"] == upstream_kfs,
        "kfs_to_marg_equal": contract["kfs_to_marg"] == upstream_targets,
        "aom_order_present": bool(upstream_aom),
        "aom_order_equal": contract["aom_order"] == upstream_aom,
        "used_imu_equal": contract["used_imu"] == bool(packet.get("use_imu", False)),
        "upstream_prior_serialized": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rust-trace", type=Path, required=True)
    parser.add_argument("--rust-marg-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--upstream-dir", type=Path)
    parser.add_argument("--base-timestamp-ns", type=int, default=DEFAULT_BASE_TIMESTAMP_NS)
    args = parser.parse_args()

    trace = [json.loads(line) for line in args.rust_trace.read_text(encoding="utf-8").splitlines() if line.strip()]
    window_rows = [row for row in trace if row.get("window", {}).get("attempted")]
    marginal_rows = [row for row in trace if row.get("marginalization") is not None]
    queue_rows = [row for row in trace if row.get("kfs_to_marg")]
    packet_paths = sorted(args.rust_marg_dir.glob("frame_*.json"))
    packets = [packet_contract(read_json(path), path) for path in packet_paths]
    upstream = upstream_candidates(args.upstream_dir, args.base_timestamp_ns) if args.upstream_dir else []
    for packet in packets:
        packet["upstream_structure"] = upstream_match(packet, upstream, args.base_timestamp_ns)

    output = {
        "schema": "basalt.m7he.margdata_boundary.v1",
        "inputs": {
            "rust_trace": str(args.rust_trace),
            "rust_trace_sha256": sha256(args.rust_trace),
            "rust_marg_dir": str(args.rust_marg_dir),
            "upstream_dir": str(args.upstream_dir) if args.upstream_dir else None,
        },
        "trace": {
            "record_count": len(trace),
            "frame_ids": [int(row["frame_id"]) for row in trace],
            "window_attempted_count": len(window_rows),
            "marginalization_record_count": len(marginal_rows),
            "queue_packet_count": len(queue_rows),
            "queue_packet_frames": [int(row["frame_id"]) for row in queue_rows],
            "all_window_status_success": all(row["window"].get("status") == "success" for row in window_rows),
        },
        "packets": packets,
        "invariants": {
            "packet_count_matches_queue": len(packet_paths) == len(queue_rows),
            "all_schema_v3": all(packet["schema_version"] == 3 for packet in packets),
            "all_aom_contiguous": all(packet["aom_offsets_contiguous"] for packet in packets),
            "all_aom_end_matches_cols": all(packet["aom_last_end"] == packet["sqrt_shape"][1] for packet in packets),
            "all_sqrt_rhs_shapes": all(packet["sqrt_rhs_len"] == packet["sqrt_shape"][0] for packet in packets),
            "all_row_counts_match_sqrt_rows": all(packet["row_count_sum"] == packet["sqrt_shape"][0] for packet in packets),
            "all_abs_shapes_match_aom": all(packet["abs_shape"] == [packet["sqrt_shape"][1], packet["sqrt_shape"][1]] for packet in packets),
            "all_images_canonical": all(packet["image_keys_sorted"] for packet in packets),
            "all_packets_used_imu": all(packet["used_imu"] for packet in packets),
            "all_mapper_packets_prior_free": all(not packet["diagnostic_prior_present"] for packet in packets),
            "all_normal_equations_f32_roundoff": all(packet["normal_equation_within_f32_roundoff"] for packet in packets),
        },
        "notes": [
            "The trace includes state-only MargData diagnostics; only non-empty kfs_to_marg records are mapper packets.",
            "The packet JSON is the queue-facing view and therefore omits Rust's carried diagnostic prior.",
            "AOM/FEJ/target equality against upstream is structural; numeric H/b trajectory equality is a later boundary.",
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(args.output), "packets": len(packets), "queue_frames": output["trace"]["queue_packet_frames"]}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
