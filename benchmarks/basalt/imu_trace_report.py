#!/usr/bin/env python3
"""Validate and flatten the diagnostic Basalt IMU golden trace.

The trace is produced by the opt-in ``BASALT_IMU_TRACE_JSONL`` hook in the
fixed upstream estimator.  It records one frame-level record (frame 0 has no
interval) and keeps the preintegration, residual, cost, and state values
computed by the upstream code.  This tool does not run the estimator or use
ground truth.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import statistics
from pathlib import Path
from typing import Any


TRACE_SCHEMA_ID = "basalt.imu_trace.v1"
SUMMARY_SCHEMA_ID = "basalt.imu_trace_summary.v1"

SCALAR_FIELDS = (
    "frame_index",
    "timestamp_ns",
    "interval_present",
    "interval_start_ns",
    "interval_end_ns",
    "dt_ns",
    "imu_sample_count",
    "first_sample_t_ns",
    "last_sample_t_ns",
    "kf_decision",
    "lm_iterations",
    "lm_rejected_iterations",
    "lm_lambda_final",
    "whitened_cost",
    "bias_walk_cost_gyro",
    "bias_walk_cost_accel",
    "bias_walk_cost_total",
)

VECTOR_FIELDS = {
    "bias_linearization_gyro": 3,
    "bias_linearization_accel": 3,
    "gravity": 3,
    "delta_position": 3,
    "delta_rotation_xyzw": 4,
    "delta_velocity": 3,
    "raw_residual_position": 3,
    "raw_residual_rotation": 3,
    "raw_residual_velocity": 3,
    "whitened_residual_position": 3,
    "whitened_residual_rotation": 3,
    "whitened_residual_velocity": 3,
    "bias_walk_residual_gyro": 3,
    "bias_walk_residual_accel": 3,
}

STATE_VECTOR_FIELDS = {
    "pose_t": 3,
    "pose_q_xyzw": 4,
    "velocity": 3,
    "bias_gyro": 3,
    "bias_accel": 3,
}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_trace(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}:{line_number}: invalid JSON: {exc}") from exc
            if not isinstance(record, dict) or record.get("trace_schema") != TRACE_SCHEMA_ID:
                raise ValueError(f"{path}:{line_number}: unexpected trace schema")
            records.append(record)
    if not records:
        raise ValueError(f"trace is empty: {path}")
    indexes = [int(record["frame_index"]) for record in records]
    expected = list(range(indexes[0], indexes[0] + len(indexes)))
    if indexes != expected:
        raise ValueError(f"frame indexes are not contiguous: {indexes!r}")
    for record in records:
        for field in SCALAR_FIELDS:
            if field not in record:
                raise ValueError(f"frame {record['frame_index']}: missing {field}")
        for field, size in VECTOR_FIELDS.items():
            value = record.get(field)
            if value is not None and (not isinstance(value, list) or len(value) != size):
                raise ValueError(f"frame {record['frame_index']}: bad {field}")
        for state_name in ("predicted_state", "optimized_state"):
            state = record.get(state_name)
            if state is not None:
                for field, size in STATE_VECTOR_FIELDS.items():
                    value = state.get(field)
                    if not isinstance(value, list) or len(value) != size:
                        raise ValueError(
                            f"frame {record['frame_index']}: bad {state_name}.{field}"
                        )
    return records


def flatten_record(record: dict[str, Any]) -> dict[str, Any]:
    row: dict[str, Any] = {field: record.get(field) for field in SCALAR_FIELDS}
    for field, size in VECTOR_FIELDS.items():
        value = record.get(field)
        for index in range(size):
            row[f"{field}_{index}"] = None if value is None else value[index]
    for state_name in ("predicted_state", "optimized_state"):
        state = record.get(state_name)
        row[f"{state_name}_t_ns"] = None if state is None else state.get("t_ns")
        for field, size in STATE_VECTOR_FIELDS.items():
            value = None if state is None else state.get(field)
            for index in range(size):
                row[f"{state_name}_{field}_{index}"] = (
                    None if value is None else value[index]
                )
    return row


def csv_fields() -> list[str]:
    fields = list(SCALAR_FIELDS)
    fields.extend(
        f"{field}_{index}"
        for field, size in VECTOR_FIELDS.items()
        for index in range(size)
    )
    for state_name in ("predicted_state", "optimized_state"):
        fields.append(f"{state_name}_t_ns")
        fields.extend(
            f"{state_name}_{field}_{index}"
            for field, size in STATE_VECTOR_FIELDS.items()
            for index in range(size)
        )
    return fields


def write_csv(records: list[dict[str, Any]], path: Path) -> None:
    fields = csv_fields()
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        for record in records:
            writer.writerow(flatten_record(record))


def artifact(path_value: str | None) -> dict[str, Any] | None:
    if not path_value:
        return None
    path = Path(path_value)
    item: dict[str, Any] = {"path": str(path), "exists": path.is_file()}
    if path.is_file():
        item["sha256"] = sha256_file(path)
        item["bytes"] = path.stat().st_size
    return item


def manifest_summary(path_value: str | None) -> dict[str, Any] | None:
    if not path_value:
        return None
    path = Path(path_value)
    manifest = json.loads(path.read_text(encoding="utf-8"))
    execution = manifest.get("execution", {})
    return {
        "path": str(path),
        "sha256": sha256_file(path),
        "run_id": manifest.get("run_id"),
        "method": manifest.get("method"),
        "status": manifest.get("status"),
        "sequence": manifest.get("sequence"),
        "input_frame_count": manifest.get("input", {}).get("frame_count"),
        "input_sha256": manifest.get("input", {}).get("sha256"),
        "execution": {
            "returncode": execution.get("returncode"),
            "wall_seconds": execution.get("wall_seconds"),
            "peak_process_tree_rss_bytes": execution.get(
                "peak_process_tree_rss_bytes"
            ),
            "timed_out": execution.get("timed_out"),
        },
        "ground_truth_firewall": manifest.get("ground_truth_firewall"),
    }


def build_summary(
    records: list[dict[str, Any]],
    trace_path: Path,
    manifest_path: str | None,
    header_path: str | None,
    source_path: str | None,
    header_diff_path: str | None,
    source_diff_path: str | None,
    binary_path: str | None,
) -> dict[str, Any]:
    interval_records = [record for record in records if record["interval_present"]]
    dt_values = [record["dt_ns"] for record in interval_records]
    sample_counts = [record["imu_sample_count"] for record in interval_records]
    return {
        "summary_schema": SUMMARY_SCHEMA_ID,
        "trace_schema": TRACE_SCHEMA_ID,
        "trace": {
            "path": str(trace_path),
            "sha256": sha256_file(trace_path),
            "record_count": len(records),
            "first_frame_index": records[0]["frame_index"],
            "last_frame_index": records[-1]["frame_index"],
            "interval_record_count": len(interval_records),
            "dt_ns_unique": sorted(set(dt_values)),
            "sample_count_unique": sorted(set(sample_counts)),
            "dt_ns_min": min(dt_values) if dt_values else None,
            "dt_ns_max": max(dt_values) if dt_values else None,
            "sample_count_min": min(sample_counts) if sample_counts else None,
            "sample_count_max": max(sample_counts) if sample_counts else None,
            "whitened_cost_mean": (
                statistics.fmean(record["whitened_cost"] for record in interval_records)
                if interval_records
                else None
            ),
            "bias_walk_cost_total_mean": (
                statistics.fmean(
                    record["bias_walk_cost_total"] for record in interval_records
                )
                if interval_records
                else None
            ),
        },
        "semantics": {
            "sample_count": "number of upstream integrate() calls in the camera interval; an interpolated endpoint sample is counted as one",
            "delta_state": "IntegratedImuMeasurement.getDeltaState() after all raw samples for the interval",
            "bias_linearization": "bias passed to the upstream IntegratedImuMeasurement constructor",
            "raw_residual": "IntegratedImuMeasurement.residual() position/rotation/velocity blocks at final optimized states before marginalization",
            "whitened_cost": "0.5 * ||sqrt_cov_inv * raw_residual||^2",
            "bias_walk_cost": "0.5 * ||bias_weight_sqrt / sqrt(dt) * (bg_start-bg_end or ba_start-ba_end)||^2",
            "state_pair": "predictedState(start_state, gravity) versus final optimized end state before marginalization",
            "ground_truth": "sensor-only run; GT was not staged or passed to the engine",
        },
        "checkpoints": {
            "frame0": records[0],
            "frame19": records[-1],
            "initial20": records[:20],
        },
        "artifacts": {
            "run_manifest": manifest_summary(manifest_path),
            "instrumented_header": artifact(header_path),
            "instrumented_source": artifact(source_path),
            "header_diff": artifact(header_diff_path),
            "source_diff": artifact(source_diff_path),
            "binary": artifact(binary_path),
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", required=True, type=Path)
    parser.add_argument("--csv", required=True, type=Path)
    parser.add_argument("--summary", required=True, type=Path)
    parser.add_argument("--run-manifest")
    parser.add_argument("--instrumented-header")
    parser.add_argument("--instrumented-source")
    parser.add_argument("--header-diff")
    parser.add_argument("--source-diff")
    parser.add_argument("--binary")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    records = read_trace(args.trace)
    write_csv(records, args.csv)
    summary = build_summary(
        records,
        args.trace,
        args.run_manifest,
        args.instrumented_header,
        args.instrumented_source,
        args.header_diff,
        args.source_diff,
        args.binary,
    )
    args.summary.parent.mkdir(parents=True, exist_ok=True)
    args.summary.write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(args.summary)
    print(
        f"records={len(records)} intervals={summary['trace']['interval_record_count']} "
        f"dt_ns={summary['trace']['dt_ns_unique']} "
        f"sample_counts={summary['trace']['sample_count_unique']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
