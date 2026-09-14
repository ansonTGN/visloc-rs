#!/usr/bin/env python3
"""Validate and flatten a diagnostic Basalt VIO trace.

The trace is emitted by an opt-in ``BASALT_TRACE_JSONL`` hook in the
upstream estimator.  This utility deliberately only consumes trace records;
it does not run the estimator or any ground-truth evaluator.  The summary
keeps the first 20 records and the complete frame-0/frame-80 records so that
the initial state and the requested stereo checkpoints remain reviewable
without loading the full JSONL file.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable


TRACE_SCHEMA_ID = "basalt.vio_trace.v1"
SUMMARY_SCHEMA_ID = "basalt.vio_trace_summary.v1"

SCALAR_FIELDS = (
    "frame_index",
    "timestamp_ns",
    "cam0_tracks",
    "cam1_tracks",
    "cam0_new_tracks",
    "cam1_new_tracks",
    "cam0_lost_tracks",
    "cam1_lost_tracks",
    "connected_cam0",
    "unconnected_cam0",
    "connected_observations_total",
    "kf_decision",
    "kf_count",
    "window_states",
    "window_poses",
    "landmarks",
    "new_landmarks",
    "lost_landmarks",
    "lm_optimize_event",
    "lm_iterations",
    "lm_rejected_iterations",
    "lm_lambda_final",
    "marg_event",
    "marginalized_kfs",
    "marginalized_states",
    "marginalized_landmarks",
)

VECTOR_FIELDS = {
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


def read_records(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}:{line_number}: invalid JSON: {exc}") from exc
            if not isinstance(record, dict):
                raise ValueError(f"{path}:{line_number}: expected JSON object")
            if record.get("trace_schema") != TRACE_SCHEMA_ID:
                raise ValueError(
                    f"{path}:{line_number}: unexpected trace_schema "
                    f"{record.get('trace_schema')!r}"
                )
            records.append(record)

    if not records:
        raise ValueError(f"trace is empty: {path}")

    frame_indices = [int(record["frame_index"]) for record in records]
    expected = list(range(frame_indices[0], frame_indices[0] + len(frame_indices)))
    if frame_indices != expected:
        raise ValueError(
            "frame_index values must be contiguous and ordered; "
            f"got first={frame_indices[:5]!r}, last={frame_indices[-5:]!r}"
        )
    return records


def _finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def validate_record_shape(record: dict[str, Any]) -> None:
    required = set(SCALAR_FIELDS) | set(VECTOR_FIELDS)
    missing = sorted(name for name in required if name not in record)
    if missing:
        raise ValueError(
            f"frame {record.get('frame_index')}: missing fields: {', '.join(missing)}"
        )
    for name in SCALAR_FIELDS:
        if name in {"kf_decision", "lm_optimize_event", "marg_event"}:
            if not isinstance(record[name], bool):
                raise ValueError(f"frame {record['frame_index']}: {name} is not boolean")
        elif not isinstance(record[name], (int, float)) or isinstance(record[name], bool):
            raise ValueError(f"frame {record['frame_index']}: {name} is not numeric")
    for name, size in VECTOR_FIELDS.items():
        vector = record[name]
        if not isinstance(vector, list) or len(vector) != size:
            raise ValueError(
                f"frame {record['frame_index']}: {name} must have {size} values"
            )
        if not all(_finite(value) for value in vector):
            raise ValueError(f"frame {record['frame_index']}: {name} has non-finite values")


def flatten_record(record: dict[str, Any]) -> dict[str, Any]:
    row: dict[str, Any] = {name: record[name] for name in SCALAR_FIELDS}
    for name, size in VECTOR_FIELDS.items():
        for index in range(size):
            row[f"{name}_{index}"] = record[name][index]
    return row


def write_csv(records: Iterable[dict[str, Any]], path: Path) -> None:
    fields = list(SCALAR_FIELDS)
    fields.extend(
        f"{name}_{index}"
        for name, size in VECTOR_FIELDS.items()
        for index in range(size)
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        for record in records:
            writer.writerow(flatten_record(record))


def _stats(records: list[dict[str, Any]], field: str) -> dict[str, float | int]:
    values = [float(record[field]) for record in records]
    return {
        "min": min(values),
        "max": max(values),
        "mean": sum(values) / len(values),
    }


def artifact(path_value: str | None) -> dict[str, Any] | None:
    if not path_value:
        return None
    path = Path(path_value)
    item: dict[str, Any] = {"path": str(path)}
    if path.is_file():
        item["exists"] = True
        item["sha256"] = sha256_file(path)
        item["bytes"] = path.stat().st_size
    else:
        item["exists"] = False
    return item


def run_manifest_summary(path_value: str | None) -> dict[str, Any] | None:
    if not path_value:
        return None
    path = Path(path_value)
    manifest: dict[str, Any] = json.loads(path.read_text(encoding="utf-8"))
    execution = manifest.get("execution", {})
    return {
        "path": str(path),
        "sha256": sha256_file(path),
        "schema_id": manifest.get("schema_id"),
        "run_id": manifest.get("run_id"),
        "method": manifest.get("method"),
        "profile": manifest.get("profile"),
        "sequence": manifest.get("sequence"),
        "status": manifest.get("status"),
        "input": {
            "frame_count": manifest.get("input", {}).get("frame_count"),
            "sha256": manifest.get("input", {}).get("sha256"),
            "ground_truth_named_artifacts": manifest.get(
                "ground_truth_firewall", {}
            ).get("workspace_gt_named_artifacts", []),
        },
        "execution": {
            "returncode": execution.get("returncode"),
            "wall_seconds": execution.get("wall_seconds"),
            "peak_process_tree_rss_bytes": execution.get(
                "peak_process_tree_rss_bytes"
            ),
            "timed_out": execution.get("timed_out"),
            "started_utc": execution.get("started_utc"),
            "finished_utc": execution.get("finished_utc"),
        },
        "ground_truth_firewall": manifest.get("ground_truth_firewall"),
    }


def evaluation_summary(path_value: str | None) -> dict[str, Any] | None:
    if not path_value:
        return None
    path = Path(path_value)
    result: dict[str, Any] = json.loads(path.read_text(encoding="utf-8"))
    runs = result.get("runs", [])
    run = runs[0] if runs else {}
    return {
        "path": str(path),
        "sha256": sha256_file(path),
        "schema_id": result.get("schema_id"),
        "ground_truth_used_after_engine_exit": result.get(
            "ground_truth_used_after_engine_exit"
        ),
        "ground_truth_sha256": result.get("ground_truth", {}).get("sha256"),
        "status": run.get("status"),
        "coverage": run.get("coverage"),
        "metrics": run.get("metrics"),
        "runtime": run.get("runtime"),
        "association": run.get("association"),
    }


def build_summary(
    records: list[dict[str, Any]],
    trace_path: Path,
    run_manifest_path: str | None,
    header_path: str | None,
    source_path: str | None,
    header_diff_path: str | None,
    source_diff_path: str | None,
    binary_path: str | None,
    evaluation_path: str | None,
) -> dict[str, Any]:
    frame0 = next((record for record in records if record["frame_index"] == 0), None)
    frame80 = next((record for record in records if record["frame_index"] == 80), None)
    kf_frames = [record["frame_index"] for record in records if record["kf_decision"]]
    marg_frames = [record["frame_index"] for record in records if record["marg_event"]]
    return {
        "summary_schema": SUMMARY_SCHEMA_ID,
        "trace_schema": TRACE_SCHEMA_ID,
        "trace": {
            "path": str(trace_path),
            "sha256": sha256_file(trace_path),
            "record_count": len(records),
            "first_frame_index": records[0]["frame_index"],
            "last_frame_index": records[-1]["frame_index"],
            "first_timestamp_ns": records[0]["timestamp_ns"],
            "last_timestamp_ns": records[-1]["timestamp_ns"],
            "timestamp_delta_ns": records[-1]["timestamp_ns"] - records[0]["timestamp_ns"],
            "kf_decision_count": len(kf_frames),
            "kf_decision_frames": kf_frames,
            "marg_event_count": len(marg_frames),
            "marg_event_frames": marg_frames,
            "field_stats": {
                field: _stats(records, field)
                for field in (
                    "cam0_tracks",
                    "cam1_tracks",
                    "cam0_new_tracks",
                    "cam1_new_tracks",
                    "cam0_lost_tracks",
                    "cam1_lost_tracks",
                    "connected_observations_total",
                    "kf_count",
                    "window_states",
                    "window_poses",
                    "landmarks",
                    "lm_iterations",
                    "lm_rejected_iterations",
                    "lm_lambda_final",
                )
            },
        },
        "semantics": {
            "track_counts": "unique current KeypointId values per camera after optical-flow measurement; new/lost are set differences from the previous frame",
            "connected_observations_total": "sum of connected observations used by the estimator for the current frame",
            "kf_decision": "the upstream take_kf decision at measure() entry, before optimization/marginalization",
            "window_states": "number of active state slots after optimize_and_marg for the current frame",
            "window_poses": "number of active keyframe poses after optimize_and_marg for the current frame",
            "lm_iterations": "accepted plus rejected LM iterations reported by the upstream optimizer for this frame",
            "marg_event": "true when upstream marginalize() entered its actual marginalization path for this frame",
            "ground_truth": "the run is sensor-only; the harness stages no state_groundtruth_estimate0 and passes no GT path to the engine",
        },
        "checkpoints": {
            "frame0": frame0,
            "frame80": frame80,
            "initial20": records[:20],
        },
        "artifacts": {
            "run_manifest": run_manifest_summary(run_manifest_path),
            "evaluation_result": evaluation_summary(evaluation_path),
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
    parser.add_argument("--run-manifest", type=str)
    parser.add_argument("--instrumented-header", type=str)
    parser.add_argument("--instrumented-source", type=str)
    parser.add_argument("--header-diff", type=str)
    parser.add_argument("--source-diff", type=str)
    parser.add_argument("--binary", type=str)
    parser.add_argument("--evaluation-result", type=str)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    records = read_records(args.trace)
    for record in records:
        validate_record_shape(record)
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
        args.evaluation_result,
    )
    args.summary.parent.mkdir(parents=True, exist_ok=True)
    args.summary.write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(args.summary)
    print(f"records={len(records)} kf_decisions={len(summary['trace']['kf_decision_frames'])} marg_events={len(summary['trace']['marg_event_frames'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
