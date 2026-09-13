#!/usr/bin/env python3
"""Compare actual Rust mapper packets with pinned Basalt MargData packets.

The upstream C++ dump is intentionally a small JSON projection of its Cereal
packet (``upstream_marg_dump.cpp``).  Rust stores matrices in nalgebra's
column-major iterator order while the dump writes Eigen matrices row-major;
the comparison below accounts for that representation difference.

This is a GT-free packet oracle.  It compares the event selected by the same
state timestamps, then reports structural parity and numeric H/b differences.
The latter are evidence, not a tolerance gate: the Rust direct-KLT and solver
inputs are not yet the pinned upstream frontend/ABS-QR trajectory.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


FRAME_PERIOD_NS = 50_000_000
DEFAULT_BASE_TIMESTAMP_NS = 1_403_636_579_763_555_584


def read(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def frame_id(timestamp_ns: int, base_timestamp_ns: int) -> int | None:
    delta = timestamp_ns - base_timestamp_ns
    nearest = round(delta / FRAME_PERIOD_NS)
    # EuRoC's decimal timestamps are represented as integer nanoseconds and
    # alternate by roughly 128 ns around the ideal 50 ms grid.
    if abs(delta - nearest * FRAME_PERIOD_NS) > 1_000:
        return None
    return nearest


def ids(values: list[int], base_timestamp_ns: int) -> list[int | str]:
    out: list[int | str] = []
    for value in values:
        out.append(frame_id(int(value), base_timestamp_ns))
    return out


def upstream_candidates(upstream_dir: Path, base_timestamp_ns: int) -> list[tuple[Path, dict[str, Any]]]:
    candidates = []
    for path in sorted(upstream_dir.glob("up_*.json")):
        data = read(path)
        states = [frame_id(int(item["timestamp_ns"]), base_timestamp_ns) for item in data["frame_states"]]
        candidates.append((path, {"data": data, "state_ids": states}))
    if not candidates:
        raise SystemExit(f"no upstream up_*.json packets in {upstream_dir}")
    return candidates


def select_upstream(
    rust_state_ids: list[int],
    candidates: list[tuple[Path, dict[str, Any]]],
) -> tuple[Path, dict[str, Any], str]:
    exact = [item for item in candidates if item[1]["state_ids"] == rust_state_ids]
    if exact:
        path, item = exact[0]
        return path, item["data"], "exact_state_timestamps"
    target = rust_state_ids[-1] if rust_state_ids else 0
    path, item = min(
        candidates,
        key=lambda candidate: abs((candidate[1]["state_ids"][-1] or 0) - target),
    )
    return path, item["data"], "nearest_state_timestamp"


def rust_aom(data: dict[str, Any]) -> list[list[Any]]:
    return [[int(block["frame_id"]), int(block["offset"]), int(block["dof"])] for block in data["aom_order"]]


def upstream_aom(data: dict[str, Any], base_timestamp_ns: int) -> list[list[Any]]:
    return [
        [frame_id(int(block["timestamp_ns"]), base_timestamp_ns), int(block["offset"]), int(block["dof"])]
        for block in data["aom_order"]
    ]


def matrix_diff(rust: dict[str, Any] | None, upstream: dict[str, Any], rhs: list[float], upstream_rhs: list[float]) -> dict[str, Any]:
    if rust is None:
        return {"shape_equal": False, "reason": "Rust packet has no aom_abs_h"}
    rows = int(rust["rows"])
    cols = int(rust["cols"])
    same_shape = rows == int(upstream["rows"]) and cols == int(upstream["cols"])
    if not same_shape:
        return {
            "shape_equal": False,
            "rust_shape": [rows, cols],
            "upstream_shape": [int(upstream["rows"]), int(upstream["cols"])],
        }
    rust_data = [float(value) for value in rust["data"]]
    upstream_data = [float(value) for value in upstream["data"]]
    diffs = []
    for row in range(rows):
        for col in range(cols):
            # nalgebra Matrix::iter() is column-major; the C++ helper emits
            # Eigen's row-major traversal.
            diffs.append(rust_data[col * rows + row] - upstream_data[row * cols + col])
    bdiffs = [float(a) - float(b) for a, b in zip(rhs, upstream_rhs)]
    return {
        "shape_equal": True,
        "max_abs_h": max((abs(value) for value in diffs), default=0.0),
        "rms_h": math.sqrt(sum(value * value for value in diffs) / max(1, len(diffs))),
        "max_abs_b": max((abs(value) for value in bdiffs), default=0.0),
        "rms_b": math.sqrt(sum(value * value for value in bdiffs) / max(1, len(bdiffs))),
    }


def frame_table_diff(
    rust_items: list[dict[str, Any]],
    upstream_items: list[dict[str, Any]],
    base_timestamp_ns: int,
) -> dict[str, Any]:
    """Compare pose values for frame IDs common after timestamp normalization.

    Quaternions are sign-aligned before differencing because q and -q encode
    the same pose.  IDs/counts are reported separately so a KF-policy
    difference is not hidden by a numeric comparison on an arbitrary row.
    """
    upstream_by_id = {
        frame_id(int(item["timestamp_ns"]), base_timestamp_ns): item
        for item in upstream_items
    }
    diffs: list[float] = []
    matched = 0
    for rust_item in rust_items:
        rust_id = int(rust_item["frame_id"])
        upstream_item = upstream_by_id.get(rust_id)
        if upstream_item is None:
            continue
        rust_pose = [float(value) for value in rust_item["pose"]]
        upstream_pose = [
            float(upstream_item[key])
            for key in ("tx", "ty", "tz", "qw", "qx", "qy", "qz")
        ]
        quaternion_dot = sum(
            rust_pose[index] * upstream_pose[index] for index in range(3, 7)
        )
        if quaternion_dot < 0.0:
            upstream_pose[3:] = [-value for value in upstream_pose[3:]]
        diffs.extend(
            rust_pose[index] - upstream_pose[index] for index in range(7)
        )
        matched += 1
    return {
        "rust_count": len(rust_items),
        "upstream_count": len(upstream_items),
        "matched_count": matched,
        "max_abs_pose": max((abs(value) for value in diffs), default=0.0),
        "rms_pose": math.sqrt(sum(value * value for value in diffs) / max(1, len(diffs))),
    }


def compare_packet(
    rust_path: Path,
    rust: dict[str, Any],
    upstream_path: Path,
    upstream: dict[str, Any],
    base_timestamp_ns: int,
) -> dict[str, Any]:
    rust_states = [int(item["frame_id"]) for item in rust["frame_states"]]
    up_states = [frame_id(int(item["timestamp_ns"]), base_timestamp_ns) for item in upstream["frame_states"]]
    rust_kfs = [int(value) for value in rust["kfs_all"]]
    up_kfs = ids([int(value) for value in upstream["kfs_all"]], base_timestamp_ns)
    rust_km = [int(value) for value in rust["kfs_to_marg"]]
    up_km = ids([int(value) for value in upstream["kfs_to_marg"]], base_timestamp_ns)
    rust_pose_ids = [int(item["frame_id"]) for item in rust["frame_poses"]]
    up_pose_ids = [frame_id(int(item["timestamp_ns"]), base_timestamp_ns) for item in upstream["frame_poses"]]
    rust_state_flags = [bool(item["linearized"]) for item in rust["frame_states"]]
    up_state_flags = [bool(item["linearized"]) for item in upstream["frame_states"]]
    rust_images = sorted({int(item["frame_id"]) for item in rust.get("of_images", [])})
    comparisons = {
        "aom_order": rust_aom(rust) == upstream_aom(upstream, base_timestamp_ns),
        "frame_pose_ids": rust_pose_ids == up_pose_ids,
        "frame_state_ids": rust_states == up_states,
        "fej_flags": rust_state_flags == up_state_flags,
        "kfs_all": rust_kfs == up_kfs,
        "kfs_to_marg": rust_km == up_km,
        "targets_pose_selection": [int(value) for value in rust["marginalization"]["poses_to_marg"]] == rust_km,
        "images_keyframe_ids": rust_images == sorted(rust_kfs),
        "image_count_is_two_per_kf": len(rust.get("of_images", [])) == 2 * len(rust_kfs),
        "used_imu": bool(rust["used_imu"]) == bool(upstream["use_imu"]),
        # This input directory is the mapper-writer output.  Upstream
        # MargData has no carried Rust square-root prior, and the queue writer
        # omits the diagnostic-only field via MargData::to_mapper_packet().
        "source_packet_prior_present": rust.get("prior") is not None,
        "mapper_writer_prior_present": rust.get("prior") is not None,
        "prior_wire_contract": rust.get("prior") is None,
    }
    groups = [int(value) for value in rust.get("row_counts", [])]
    rust_h = rust.get("aom_abs_h")
    return {
        "rust_file": str(rust_path),
        "upstream_file": str(upstream_path),
        "rust": {
            "aom_order": rust_aom(rust),
            "sqrt_jacobian_shape": [int(rust["aom_sqrt_jacobian"]["rows"]), int(rust["aom_sqrt_jacobian"]["cols"])],
            "row_counts": groups,
            "row_count_sum": sum(groups),
            "marginalization": rust["marginalization"],
            "kfs_all": rust_kfs,
            "kfs_to_marg": rust_km,
            "frame_poses": rust_pose_ids,
            "frame_states": rust_states,
            "fej_flags": rust_state_flags,
            "image_frame_ids": rust_images,
            "image_count": len(rust.get("of_images", [])),
            "used_imu": bool(rust["used_imu"]),
            "packet_prior_present": rust.get("prior") is not None,
        },
        "upstream": {
            "aom_order": upstream_aom(upstream, base_timestamp_ns),
            "abs_h_shape": [int(upstream["abs_h"]["rows"]), int(upstream["abs_h"]["cols"])],
            "kfs_all": up_kfs,
            "kfs_to_marg": up_km,
            "frame_poses": up_pose_ids,
            "frame_states": up_states,
            "fej_flags": up_state_flags,
            "image_frame_ids_expected": sorted(up_kfs),
            "image_count_expected": 2 * len(up_kfs),
            "used_imu": bool(upstream["use_imu"]),
            "prior_serialized": False,
        },
        "comparisons": comparisons,
        "frame_tables": {
            "poses": frame_table_diff(
                rust["frame_poses"], upstream["frame_poses"], base_timestamp_ns
            ),
            "states": frame_table_diff(
                rust["frame_states"], upstream["frame_states"], base_timestamp_ns
            ),
        },
        "numeric": matrix_diff(
            rust_h,
            upstream["abs_h"],
            [float(value) for value in rust.get("aom_abs_b", [])],
            [float(value) for value in upstream.get("abs_b", [])],
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rust-dir", type=Path, required=True)
    parser.add_argument("--upstream-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--base-timestamp-ns", type=int, default=DEFAULT_BASE_TIMESTAMP_NS)
    args = parser.parse_args()

    candidates = upstream_candidates(args.upstream_dir, args.base_timestamp_ns)
    records = []
    for rust_path in sorted(args.rust_dir.glob("frame_*.json")):
        rust = read(rust_path)
        rust_state_ids = [int(item["frame_id"]) for item in rust["frame_states"]]
        upstream_path, upstream, selection = select_upstream(rust_state_ids, candidates)
        record = compare_packet(rust_path, rust, upstream_path, upstream, args.base_timestamp_ns)
        record["selection"] = selection
        records.append(record)

    output = {
        "schema": "basalt.m6_actual_packet_oracle.v1",
        "upstream": "basalt-0f3b2b52",
        "base_timestamp_ns": args.base_timestamp_ns,
        "frame_period_ns": FRAME_PERIOD_NS,
        "packet_count": len(records),
        "records": records,
        "notes": [
            "Rows/groups are Rust diagnostic groups; pinned MargData serializes abs_H/abs_b, not factor groups.",
            "H/b differences are reported without tolerance relaxation because Rust direct-KLT/solver trajectories differ from the pinned C++ run.",
            "The mapper-writer input is expected to omit the Rust-only carried prior; in-process diagnostic retention is covered by the MargData unit test.",
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"packet_count": len(records), "output": str(args.output)}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
