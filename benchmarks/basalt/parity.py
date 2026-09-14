"""Paired Basalt-vs-Rust parity evaluation for frozen 3-repetition evidence."""

from __future__ import annotations

import argparse
import json
import math
import re
import statistics
from pathlib import Path
from typing import Any

from .harness import DEFAULT_PROTOCOL, PROTOCOL_ID, load_protocol, read_json, sha256_file
from .schema import validate_document, validate_document_file


EVALUATION_SCHEMA = "evaluation_result_v1.schema.json"
RUN_SCHEMA = "run_manifest_v1.schema.json"
REPETITIONS = 3

_METRICS = {
    "coverage": ("coverage", "tracked_fraction"),
    "ate_se3": ("metrics", "ate_translation_se3_rmse_m"),
    "rpe_translation": ("metrics", "rpe_translation_consecutive_rmse_m"),
    "rpe_rotation": ("metrics", "rpe_rotation_consecutive_rmse_deg"),
    "scale": ("metrics", "sim3_scale_diagnostic"),
    "runtime": ("runtime", "wall_seconds"),
    "rss": ("runtime", "peak_process_tree_rss_bytes"),
}


def _finite(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    result = float(value)
    return result if math.isfinite(result) else None


def _metric(run: dict[str, Any], name: str) -> float | None:
    section, key = _METRICS[name]
    return _finite(run.get(section, {}).get(key))


def _run_identity(run: dict[str, Any]) -> tuple[str, int]:
    sequence = run.get("sequence")
    if not isinstance(sequence, str) or not sequence:
        raise ValueError("evaluation run has no sequence")
    repetition = run.get("repetition")
    if repetition is None:
        manifest = run.get("manifest")
        if isinstance(manifest, str) and Path(manifest).is_file():
            source = read_json(Path(manifest))
            repetition = source.get("repetition")
            if repetition is None:
                repetition = source.get("run", {}).get("repetition")
        if repetition is None:
            match = re.search(r"(?:^|[_-])r(?:ep)?(\d+)(?:$|[_-])", str(manifest or ""), re.IGNORECASE)
            if match:
                repetition = int(match.group(1))
    if isinstance(repetition, bool) or not isinstance(repetition, int) or repetition < 1:
        raise ValueError(f"evaluation run {sequence!r} has no repetition identity")
    return sequence, repetition


def _load_evaluation(path: Path) -> dict[str, Any]:
    result = validate_document_file(path, EVALUATION_SCHEMA)
    if result.get("protocol_id") != PROTOCOL_ID:
        raise ValueError(f"evaluation protocol mismatch: {path}")
    return result


def _index(result: dict[str, Any], label: str) -> dict[tuple[str, int], dict[str, Any]]:
    indexed: dict[tuple[str, int], dict[str, Any]] = {}
    for run in result["runs"]:
        identity = _run_identity(run)
        if identity in indexed:
            raise ValueError(f"{label}: duplicate run identity {identity!r}")
        indexed[identity] = run
    return indexed


def _gate_reason(name: str, base: float, port: float, gates: dict[str, Any]) -> str | None:
    if name == "coverage":
        drop = float(gates["coverage_max_drop"])
        if port < base - drop:
            return f"coverage {port:.6g} is below upstream {base:.6g} by more than {drop:.6g}"
        return None
    if name == "scale":
        absolute = float(gates["scale_abs_diff_max"])
        relative = float(gates["scale_relative_diff_max"])
        limit = max(absolute, abs(base) * relative)
        if abs(port - base) > limit:
            return f"scale difference {abs(port - base):.6g} exceeds {limit:.6g}"
        return None
    ratio_key = {
        "ate_se3": "ate_se3_ratio_max",
        "rpe_translation": "rpe_translation_ratio_max",
        "rpe_rotation": "rpe_rotation_ratio_max",
        "runtime": "runtime_ratio_max",
        "rss": "rss_ratio_max",
    }[name]
    floor_key = {
        "ate_se3": "ate_se3_additive_floor_m",
        "rpe_translation": "rpe_translation_additive_floor_m",
        "rpe_rotation": "rpe_rotation_additive_floor_deg",
        "runtime": None,
        "rss": None,
    }[name]
    ratio_limit = base * float(gates[ratio_key])
    additive_limit = base + (float(gates[floor_key]) if floor_key else 0.0)
    limit = max(ratio_limit, additive_limit)
    if port > limit:
        return f"{name} {port:.6g} exceeds upstream {base:.6g} limit {limit:.6g}"
    return None


def _aggregate(values: list[float], *, higher_is_better: bool = False) -> dict[str, float | None]:
    if not values:
        return {"mean": None, "median": None, "worst": None}
    return {
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "worst": max(values) if higher_is_better else max(values),
    }


def evaluate_paired_parity(
    upstream_evaluation: Path,
    port_evaluation: Path,
    protocol_path: Path = DEFAULT_PROTOCOL,
) -> dict[str, Any]:
    """Compare two validated evaluation results and return all gate reasons."""

    protocol = load_protocol(protocol_path)
    gates = protocol.get("parity_gates")
    if not isinstance(gates, dict):
        raise ValueError("protocol has no parity_gates")
    upstream = _load_evaluation(upstream_evaluation)
    port = _load_evaluation(port_evaluation)
    upstream_runs = _index(upstream, "upstream")
    port_runs = _index(port, "port")
    expected = {(sequence, repetition) for sequence in protocol["sequences"] for repetition in range(1, REPETITIONS + 1)}
    reasons: list[str] = []
    if set(upstream_runs) != expected:
        reasons.append(f"upstream identities differ from expected set: missing={sorted(expected - set(upstream_runs))}, extra={sorted(set(upstream_runs) - expected)}")
    if set(port_runs) != expected:
        reasons.append(f"port identities differ from expected set: missing={sorted(expected - set(port_runs))}, extra={sorted(set(port_runs) - expected)}")

    pairs: list[dict[str, Any]] = []
    sequence_rows: list[dict[str, Any]] = []
    for sequence in protocol["sequences"]:
        sequence_pairs: list[dict[str, Any]] = []
        for repetition in range(1, REPETITIONS + 1):
            identity = (sequence, repetition)
            base_run = upstream_runs.get(identity)
            port_run = port_runs.get(identity)
            pair_reasons: list[str] = []
            if base_run is None or port_run is None:
                pair_reasons.append("missing paired run")
            else:
                if base_run.get("status") != "success":
                    pair_reasons.append(f"upstream status is {base_run.get('status')}")
                if port_run.get("status") != "success":
                    pair_reasons.append(f"port status is {port_run.get('status')}")
                if base_run.get("status") == "success" and port_run.get("status") == "success":
                    for name in _METRICS:
                        base_value = _metric(base_run, name)
                        port_value = _metric(port_run, name)
                        if base_value is None or port_value is None:
                            pair_reasons.append(f"missing/non-finite {name}")
                            continue
                        reason = _gate_reason(name, base_value, port_value, gates)
                        if reason:
                            pair_reasons.append(reason)
            row = {
                "sequence": sequence,
                "repetition": repetition,
                "pass": not pair_reasons,
                "reasons": pair_reasons,
                "upstream_status": base_run.get("status") if base_run else "missing",
                "port_status": port_run.get("status") if port_run else "missing",
            }
            if base_run and port_run:
                row["metrics"] = {
                    name: {"upstream": _metric(base_run, name), "port": _metric(port_run, name)}
                    for name in _METRICS
                }
            pairs.append(row)
            sequence_pairs.append(row)
            if pair_reasons:
                reasons.extend(
                    f"{sequence} r{repetition}: {reason}" for reason in pair_reasons
                )

        aggregate_reasons: list[str] = []
        if any(not row["pass"] for row in sequence_pairs):
            aggregate_reasons.append("one or more repetitions failed")
        sequence_row: dict[str, Any] = {"sequence": sequence, "pass": not aggregate_reasons, "reasons": aggregate_reasons}
        for name in _METRICS:
            base_values = [_metric(upstream_runs[(sequence, r)], name) for r in range(1, REPETITIONS + 1) if (sequence, r) in upstream_runs and upstream_runs[(sequence, r)].get("status") == "success"]
            port_values = [_metric(port_runs[(sequence, r)], name) for r in range(1, REPETITIONS + 1) if (sequence, r) in port_runs and port_runs[(sequence, r)].get("status") == "success"]
            if len(base_values) == REPETITIONS and len(port_values) == REPETITIONS and all(v is not None for v in (*base_values, *port_values)):
                base_median = statistics.median(v for v in base_values if v is not None)
                port_median = statistics.median(v for v in port_values if v is not None)
                sequence_row[name] = {"upstream_median": base_median, "port_median": port_median}
                reason = _gate_reason(name, base_median, port_median, gates)
                if reason and reason not in sequence_row["reasons"]:
                    sequence_row["reasons"].append(f"3-repetition median: {reason}")
                    sequence_row["pass"] = False
            else:
                sequence_row[name] = {"upstream_median": None, "port_median": None}
        sequence_rows.append(sequence_row)

    suite: dict[str, Any] = {"pass": True, "reasons": [], "metrics": {}}
    for name in _METRICS:
        base_values = [row[name]["upstream_median"] for row in sequence_rows if row[name]["upstream_median"] is not None]
        port_values = [row[name]["port_median"] for row in sequence_rows if row[name]["port_median"] is not None]
        suite["metrics"][name] = {
            "upstream_median": statistics.median(base_values) if base_values else None,
            "port_median": statistics.median(port_values) if port_values else None,
            "upstream_worst": max(base_values) if base_values else None,
            "port_worst": max(port_values) if port_values else None,
        }
        if len(base_values) != len(protocol["sequences"]) or len(port_values) != len(protocol["sequences"]):
            suite["pass"] = False
            suite["reasons"].append(f"incomplete 3-repetition aggregate for {name}")
        else:
            reason = _gate_reason(name, suite["metrics"][name]["upstream_median"], suite["metrics"][name]["port_median"], gates)
            if reason:
                suite["pass"] = False
                suite["reasons"].append(f"suite median: {reason}")
    if reasons:
        suite["pass"] = False
    reasons.extend(suite["reasons"])
    passed = not reasons and all(row["pass"] for row in sequence_rows)
    return {
        "schema_version": 1,
        "schema_id": "basalt.paired_parity_result.v1",
        "protocol_id": protocol["protocol_id"],
        "protocol_sha256": sha256_file(protocol_path),
        "upstream_evaluation": str(upstream_evaluation.resolve()),
        "port_evaluation": str(port_evaluation.resolve()),
        "parity_gates": gates,
        "pass": passed,
        "reasons": reasons,
        "pairs": pairs,
        "sequences": sequence_rows,
        "suite": suite,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--upstream-evaluation", type=Path, required=True)
    parser.add_argument("--port-evaluation", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        result = evaluate_paired_parity(args.upstream_evaluation, args.port_evaluation, args.protocol)
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print(args.out)
        return 0 if result["pass"] else 1
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"parity evaluation failed: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
