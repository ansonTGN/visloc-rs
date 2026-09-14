#!/usr/bin/env python3
"""Diff the opt-in M7al Basalt iteration-oracle artifacts.

The two runtimes use different frame identifiers (Rust uses the local frame
index while Basalt uses the nanosecond timestamp) and Basalt emits landmark
linearization/QR/back-substitution records beside its snapshots.  This tool
normalizes those two details, then walks one frame's snapshots in a fixed
order and reports the first structural or numeric difference above the
explicit absolute/relative tolerance.

It is deliberately a report tool: it never changes solver thresholds or
rewrites either artifact.
"""

from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Any


SCHEMA = "basalt.vio_iteration.v1"


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            try:
                record = json.loads(line)
            except json.JSONDecodeError as error:
                raise ValueError(f"{path}:{line_number}: invalid JSON: {error}") from error
            if record.get("schema") != SCHEMA:
                raise ValueError(f"{path}:{line_number}: unexpected schema")
            records.append(record)
    return records


def snapshot_index(records: list[dict[str, Any]]) -> dict[tuple[int, int, str], dict[str, Any]]:
    return {
        (int(record["iteration"]), int(record["trial"]), str(record["phase"])): record
        for record in records
        if record.get("record") == "snapshot"
    }


def record_index(
    records: list[dict[str, Any]], record_name: str
) -> dict[tuple[int, int], dict[int, dict[str, Any]]]:
    indexed: dict[tuple[int, int], dict[int, dict[str, Any]]] = defaultdict(dict)
    for record in records:
        if record.get("record") == record_name:
            bucket = indexed[(int(record["iteration"]), int(record["trial"]))]
            track_id = int(record["track_id"])
            # A TBB worker can finish a diagnostic write after the enclosing
            # phase has advanced its trace context.  Such late records have
            # the same (iteration, trial, track) key but a duplicate row span
            # and must not replace the first record from the solve that owns
            # the block.  Worker emission order itself remains intentionally
            # unordered; canonical comparisons below use track IDs/row spans.
            bucket.setdefault(track_id, record)
    return indexed


def number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def finite(value: Any) -> bool:
    return number(value) and math.isfinite(float(value))


class Diff:
    def __init__(self, abs_tol: float, rel_tol: float) -> None:
        self.abs_tol = abs_tol
        self.rel_tol = rel_tol
        self.first_control: dict[str, Any] | None = None
        self.first_payload: dict[str, Any] | None = None
        self.structural: list[dict[str, Any]] = []
        self.order_observations: list[dict[str, Any]] = []
        self.padding_observations: list[dict[str, Any]] = []

    def structural_difference(
        self,
        path: str,
        kind: str,
        upstream: Any,
        rust: Any,
        *,
        control: bool = False,
    ) -> None:
        entry = {"path": path, "kind": kind, "upstream": upstream, "rust": rust}
        self.structural.append(entry)
        if control:
            if self.first_control is None:
                self.first_control = entry
        elif self.first_payload is None:
            self.first_payload = entry

    def numeric_difference(self, path: str, upstream: Any, rust: Any) -> None:
        if not finite(upstream) or not finite(rust):
            if upstream != rust:
                self.structural_difference(path, "non_finite_or_type", upstream, rust)
            return
        upstream_float = float(upstream)
        rust_float = float(rust)
        absolute = abs(upstream_float - rust_float)
        relative = absolute / max(abs(upstream_float), abs(rust_float), 1.0e-300)
        if absolute > self.abs_tol and relative > self.rel_tol:
            entry = {
                "path": path,
                "kind": "numeric",
                "upstream": upstream,
                "rust": rust,
                "absolute": absolute,
                "relative": relative,
                "abs_tol": self.abs_tol,
                "rel_tol": self.rel_tol,
            }
            if self.first_payload is None:
                self.first_payload = entry

    def scalar(self, path: str, upstream: Any, rust: Any) -> None:
        if number(upstream) and number(rust):
            self.numeric_difference(path, upstream, rust)
        elif upstream != rust:
            self.structural_difference(path, "value", upstream, rust)

    def shape(self, path: str, upstream: Any, rust: Any) -> None:
        if isinstance(upstream, list) and isinstance(rust, list):
            if len(upstream) != len(rust):
                self.structural_difference(path + ".length", "length", len(upstream), len(rust))
        else:
            self.structural_difference(path, "not_array", type(upstream).__name__, type(rust).__name__)


def frame_timestamp_map(snapshot: dict[str, Any], source: str) -> dict[Any, int]:
    result: dict[Any, int] = {}
    for block in snapshot.get("aom", []):
        timestamp = int(block.get("timestamp_ns", block.get("frame_id", 0)))
        frame_id = block.get("frame_id")
        result[frame_id] = timestamp
    if source == "upstream":
        return {timestamp: timestamp for timestamp in result.values()}
    return result


def timestamp(value: Any, mapping: dict[Any, int], explicit: Any = None) -> Any:
    if explicit is not None:
        return explicit
    return mapping.get(value, value)


def normalize_landmarks(snapshot: dict[str, Any], source: str) -> dict[int, dict[str, Any]]:
    mapping = frame_timestamp_map(snapshot, source)
    result: dict[int, dict[str, Any]] = {}
    for landmark in snapshot.get("landmarks", []):
        normalized = dict(landmark)
        normalized["host_frame_id"] = timestamp(
            landmark.get("host_frame_id"), mapping, landmark.get("host_timestamp_ns")
        )
        observations = []
        for observation in landmark.get("observations", []):
            item = dict(observation)
            item["target_frame_id"] = timestamp(
                observation.get("target_frame_id"),
                mapping,
                observation.get("target_timestamp_ns"),
            )
            observations.append(item)
        normalized["observations"] = observations
        result[int(landmark["track_id"])] = normalized
    return result


def normalize_blocks(snapshot: dict[str, Any], source: str) -> dict[tuple[str, int], dict[str, Any]]:
    mapping = frame_timestamp_map(snapshot, source)
    result: dict[tuple[str, int], dict[str, Any]] = {}
    blocks = snapshot.get("blocks", {})
    for kind in ("poses", "states"):
        for block in blocks.get(kind, []):
            key = (kind[:-1], timestamp(block.get("frame_id"), mapping, block.get("timestamp_ns")))
            result[key] = block
    return result


def compare_common(diff: Diff, path: str, upstream: Any, rust: Any, skip: set[str] | None = None) -> None:
    skip = skip or set()
    if number(upstream) and number(rust):
        diff.numeric_difference(path, upstream, rust)
    elif isinstance(upstream, dict) and isinstance(rust, dict):
        for key in upstream:
            if key in skip:
                continue
            if key not in rust:
                diff.structural_difference(f"{path}.{key}", "missing_in_rust", True, False)
                continue
            compare_common(diff, f"{path}.{key}", upstream[key], rust[key], skip)
    elif isinstance(upstream, list) and isinstance(rust, list):
        if len(upstream) != len(rust):
            diff.structural_difference(f"{path}.length", "length", len(upstream), len(rust))
            return
        for index, (left, right) in enumerate(zip(upstream, rust)):
            compare_common(diff, f"{path}[{index}]", left, right, skip)
    elif upstream != rust:
        diff.structural_difference(path, "value", upstream, rust)


def compare_matrix_with_zero_tail(
    diff: Diff, path: str, upstream: Any, rust: Any
) -> None:
    """Compare QR rows while treating an all-zero trailing state tail as padding.

    The final Basalt iteration may have a reduced active AOM width after its
    window bookkeeping, while the Rust diagnostic keeps the fixed frame-4
    width.  The extra Rust columns are only a representation difference when
    every value in that tail is zero; the populated common prefix remains a
    normal scalar comparison.
    """
    if not isinstance(upstream, list) or not isinstance(rust, list):
        compare_common(diff, path, upstream, rust)
        return
    if len(upstream) != len(rust):
        diff.structural_difference(f"{path}.length", "length", len(upstream), len(rust))
        return
    for row_index, (upstream_row, rust_row) in enumerate(zip(upstream, rust)):
        if not isinstance(upstream_row, list) or not isinstance(rust_row, list):
            compare_common(diff, f"{path}[{row_index}]", upstream_row, rust_row)
            continue
        common_width = min(len(upstream_row), len(rust_row))
        compare_common(
            diff,
            f"{path}[{row_index}][:common]",
            upstream_row[:common_width],
            rust_row[:common_width],
        )
        if len(upstream_row) == len(rust_row):
            continue
        if len(upstream_row) > common_width:
            tail = upstream_row[common_width:]
            wider = "upstream"
        else:
            tail = rust_row[common_width:]
            wider = "rust"
        nonzero_tail = [value for value in tail if not finite(value) or abs(float(value)) > diff.abs_tol]
        if nonzero_tail:
            diff.structural_difference(
                f"{path}[{row_index}].padding",
                "nonzero_padding",
                upstream_row,
                rust_row,
            )
        else:
            diff.padding_observations.append(
                {
                    "path": f"{path}[{row_index}]",
                    "wider": wider,
                    "upstream_width": len(upstream_row),
                    "rust_width": len(rust_row),
                }
            )


def compare_aom(diff: Diff, upstream: dict[str, Any], rust: dict[str, Any]) -> None:
    rust_mapping = frame_timestamp_map(rust, "rust")
    left = upstream.get("aom", [])
    right = rust.get("aom", [])
    if len(left) != len(right):
        diff.structural_difference("aom.length", "length", len(left), len(right))
        return
    for index, (aom_upstream, aom_rust) in enumerate(zip(left, right)):
        path = f"aom[{index}]"
        diff.scalar(f"{path}.kind", aom_upstream.get("kind"), aom_rust.get("kind"))
        diff.scalar(
            f"{path}.timestamp_ns",
            aom_upstream.get("timestamp_ns"),
            rust_mapping.get(aom_rust.get("frame_id"), aom_rust.get("timestamp_ns")),
        )
        for field in ("offset", "dof"):
            diff.scalar(f"{path}.{field}", aom_upstream.get(field), aom_rust.get(field))


def compare_blocks(diff: Diff, upstream: dict[str, Any], rust: dict[str, Any]) -> None:
    left = normalize_blocks(upstream, "upstream")
    right = normalize_blocks(rust, "rust")
    if set(left) != set(right):
        diff.structural_difference("blocks.keys", "key_set", sorted(left), sorted(right))
        return
    for key in sorted(left):
        # Upstream and Rust intentionally expose different optional matrix
        # fields.  Compare all common state/pose scalars in source order.
        upstream_block = dict(left[key])
        rust_block = dict(right[key])
        # The key already canonicalizes Rust's local frame index to the
        # upstream nanosecond timestamp.
        upstream_block["frame_id"] = key[1]
        rust_block["frame_id"] = key[1]
        compare_common(
            diff,
            f"blocks[{key[0]},{key[1]}]",
            upstream_block,
            rust_block,
            {"matrix"},
        )


def compare_landmarks(diff: Diff, upstream: dict[str, Any], rust: dict[str, Any]) -> None:
    left = normalize_landmarks(upstream, "upstream")
    right = normalize_landmarks(rust, "rust")
    if set(left) != set(right):
        diff.structural_difference("landmarks.track_id", "key_set", sorted(left), sorted(right))
        return
    for track_id in sorted(left):
        a = left[track_id]
        b = right[track_id]
        path = f"landmarks[track_id={track_id}]"
        for field in ("host_frame_id", "host_cam"):
            diff.scalar(f"{path}.{field}", a.get(field), b.get(field))
        for field in ("direction", "rho"):
            compare_common(diff, f"{path}.{field}", a.get(field), b.get(field))
        if len(a.get("observations", [])) != len(b.get("observations", [])):
            diff.structural_difference(
                f"{path}.observations.length",
                "length",
                len(a.get("observations", [])),
                len(b.get("observations", [])),
            )
            continue
        for index, (obs_a, obs_b) in enumerate(zip(a.get("observations", []), b.get("observations", []))):
            obs_path = f"{path}.observations[{index}]"
            for field in ("target_frame_id", "target_cam", "pixel"):
                compare_common(diff, f"{obs_path}.{field}", obs_a.get(field), obs_b.get(field))


def compare_visual_factors(
    diff: Diff,
    upstream: dict[str, Any],
    rust: dict[str, Any],
    upstream_linearization: dict[int, dict[str, Any]],
    upstream_qr: dict[int, dict[str, Any]],
) -> None:
    rust_factors = {int(item["track_id"]): item for item in rust.get("landmark_factors", [])}
    if set(upstream_linearization) != set(rust_factors):
        diff.structural_difference(
            "landmark_factors.track_id", "key_set", sorted(upstream_linearization), sorted(rust_factors)
        )
        return
    for track_id in sorted(upstream_linearization):
        linear = upstream_linearization[track_id]
        factor = rust_factors[track_id]
        path = f"landmark_factors[track_id={track_id}]"
        for field in ("direction", "rho"):
            compare_common(diff, f"{path}.{field}", linear.get(field), factor.get(field))
        observations_a = linear.get("observations", [])
        observations_b = factor.get("observations", [])
        if len(observations_a) != len(observations_b):
            diff.structural_difference(
                f"{path}.observations.length", "length", len(observations_a), len(observations_b)
            )
            continue
        for index, (obs_a, obs_b) in enumerate(zip(observations_a, observations_b)):
            obs_path = f"{path}.observations[{index}]"
            # These are the same weighted ABS_QR inputs on both sides.
            for field in ("raw_residual", "huber_weight", "sqrt_weight"):
                compare_common(diff, f"{obs_path}.{field}", obs_a.get(field), obs_b.get(field))
            compare_common(
                diff,
                f"{obs_path}.landmark_jacobian",
                obs_a.get("landmark_jacobian"),
                obs_b.get("jl"),
            )
            # For a same-frame stereo observation, upstream accumulates host
            # and target contributions into the same absolute AOM block.
            same_frame = obs_a.get("target_frame_id") == linear.get("host_frame_id")
            if same_frame:
                host = obs_a.get("abs_host_jac", [])
                target = obs_a.get("abs_target_jac", [])
                combined = [
                    [left + right for left, right in zip(host_row, target_row)]
                    for host_row, target_row in zip(host, target)
                ]
                rust_combined = [
                    [left + right for left, right in zip(host_row, target_row)]
                    for host_row, target_row in zip(
                        obs_b.get("jp_anchor", []), obs_b.get("jp_target", [])
                    )
                ]
                compare_common(diff, f"{obs_path}.abs_same_frame_jac", combined, rust_combined)
            else:
                compare_common(
                    diff,
                    f"{obs_path}.abs_host_jac",
                    obs_a.get("abs_host_jac"),
                    obs_b.get("jp_anchor"),
                )
                compare_common(
                    diff,
                    f"{obs_path}.abs_target_jac",
                    obs_a.get("abs_target_jac"),
                    obs_b.get("jp_target"),
                )
        qr = upstream_qr.get(track_id)
        if qr is None:
            diff.structural_difference(f"{path}.qr", "missing_upstream_qr", True, False)
            continue
        compare_matrix_with_zero_tail(
            diff, f"{path}.reduced_rows", qr.get("reduced_rows"), factor.get("reduced_rows")
        )
        compare_common(
            diff, f"{path}.reduced_rhs", qr.get("reduced_rhs"), factor.get("reduced_rhs")
        )
        if qr.get("row_span", [None, None])[1] != factor.get("row_span", [None, None])[1]:
            diff.structural_difference(
                f"{path}.row_span.count",
                "row_count",
                qr.get("row_span", [None, None])[1],
                factor.get("row_span", [None, None])[1],
            )

    # Basalt's landmark container is an aligned_unordered_map and TBB may
    # visit blocks in different orders.  Keep the observed sequence in the
    # artifact for audit, but do not classify a permutation as a semantic
    # numeric divergence: payloads above were compared by canonical ID.
    upstream_order = [track for track, record in sorted(upstream_qr.items(), key=lambda item: item[1]["row_span"][0])]
    rust_order = [
        int(record["track_id"])
        for record in sorted(rust_factors.values(), key=lambda item: item["row_span"][0])
    ]
    if upstream_order != rust_order:
        diff.order_observations.append(
            {"path": "landmark_qr.order", "upstream": upstream_order, "rust": rust_order}
        )


def compare_global(diff: Diff, upstream: dict[str, Any], rust: dict[str, Any]) -> None:
    for field in ("h", "b"):
        compare_common(diff, f"global.{field}", upstream.get("global", {}).get(field), rust.get("global", {}).get(field))
    # Raw Q2 stacks depend on unordered upstream traversal.  Dimensions are
    # still checked, while scalar comparison is done per track above.
    upstream_rows = upstream.get("global", {}).get("reduced_rows") or []
    rust_rows = rust.get("global", {}).get("reduced_rows") or []
    upstream_shape = [len(upstream_rows), len(upstream_rows[0]) if upstream_rows else 0]
    rust_shape = [len(rust_rows), len(rust_rows[0]) if rust_rows else 0]
    if upstream_shape != rust_shape:
        diff.structural_difference("global.reduced_rows.shape", "shape", upstream_shape, rust_shape)


def compare_cost_and_lm(diff: Diff, upstream: dict[str, Any], rust: dict[str, Any]) -> None:
    for field in ("lambda", "lambda_after"):
        diff.scalar(field, upstream.get(field), rust.get(field))
    compare_common(diff, "cost", upstream.get("cost"), rust.get("cost"))
    diff.scalar("decision", upstream.get("decision"), rust.get("decision"))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--upstream", type=Path, required=True)
    parser.add_argument("--rust", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--abs-tol", type=float, default=1.0e-6)
    parser.add_argument("--rel-tol", type=float, default=1.0e-6)
    args = parser.parse_args()

    upstream_records = load_jsonl(args.upstream)
    rust_records = load_jsonl(args.rust)
    upstream_snapshots = snapshot_index(upstream_records)
    rust_snapshots = snapshot_index(rust_records)
    diff = Diff(args.abs_tol, args.rel_tol)
    factor_diff = Diff(args.abs_tol, args.rel_tol)
    if set(upstream_snapshots) != set(rust_snapshots):
        diff.structural_difference(
            "snapshots.keys", "key_set", sorted(upstream_snapshots), sorted(rust_snapshots)
            , control=True
        )

    upstream_linearizations = record_index(upstream_records, "landmark_linearization")
    upstream_qrs = record_index(upstream_records, "landmark_qr")
    summaries = []
    for key in sorted(set(upstream_snapshots) & set(rust_snapshots)):
        up = upstream_snapshots[key]
        ru = rust_snapshots[key]
        iteration, trial, phase = key
        # Structural and numeric walk order is deliberate: ordering metadata,
        # state/landmark values, raw visual rows, QR rows, global rows, then LM
        # decision/cost.  The first entry is therefore actionable rather than
        # an arbitrary JSON key order.
        compare_aom(diff, up, ru)
        compare_blocks(diff, up, ru)
        compare_landmarks(diff, up, ru)
        # Landmark linearization/QR records are emitted once per solve
        # iteration, before the trial mutates landmark values.  Compare them
        # against the matching iteration-start snapshot only.
        if phase == "iteration_start":
            compare_visual_factors(
                diff,
                up,
                ru,
                upstream_linearizations.get((iteration, trial), {}),
                upstream_qrs.get((iteration, trial), {}),
            )
            compare_visual_factors(
                factor_diff,
                up,
                ru,
                upstream_linearizations.get((iteration, trial), {}),
                upstream_qrs.get((iteration, trial), {}),
            )
        compare_global(diff, up, ru)
        compare_cost_and_lm(diff, up, ru)
        summaries.append(
            {
                "iteration": iteration,
                "trial": trial,
                "phase": phase,
                "upstream_decision": up.get("decision"),
                "rust_decision": ru.get("decision"),
                "upstream_cost": up.get("cost"),
                "rust_cost": ru.get("cost"),
            }
        )

    result = {
        "schema": "basalt.vio_iteration_diff.v1",
        "upstream": str(args.upstream),
        "rust": str(args.rust),
        "tolerance": {"abs": args.abs_tol, "rel": args.rel_tol},
        "first_difference": diff.first_control or diff.first_payload,
        "first_control_flow_difference": diff.first_control,
        "first_payload_difference": diff.first_payload,
        "first_factor_difference": factor_diff.first_payload,
        "structural_difference_count": len(diff.structural),
        "structural_difference_preview": diff.structural[:32],
        "order_observations": diff.order_observations[:8],
        "padding_observations": diff.padding_observations[:32],
        "ignored_order_sensitive_fields": ["global.reduced_rows", "global.reduced_rhs"],
        "snapshot_count": len(summaries),
        "snapshots": summaries,
        "upstream_header": next((r for r in upstream_records if r.get("record") == "header"), None),
        "rust_header": next((r for r in rust_records if r.get("record") == "header"), None),
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(
        json.dumps(
            {
                "first_difference": result["first_difference"],
                "first_control_flow_difference": result["first_control_flow_difference"],
                "first_payload_difference": result["first_payload_difference"],
                "snapshot_count": len(summaries),
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
