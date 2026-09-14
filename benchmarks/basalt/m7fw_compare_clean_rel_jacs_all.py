#!/usr/bin/env python3
"""Compare clean relative-pose Jacobian chaining with Rust absolute blocks.

The clean M7ef capture is the raw binary32 ``d_res_d_xi`` boundary.  M7fw
adds the clean binary32 ``computeRelPose`` outputs after their final stores.
This tool joins those two captures to the exact f32 whitening scalar retained
in the Rust M7fv detail snapshot and checks the resulting weighted 2x6
absolute-pose blocks against serialized ``jp_anchor``/``jp_target``.

No solver output is changed.  The comparison intentionally reports both the
literal clean-relative chain (including the native same-timestamp stereo
Jacobian) and the Rust absolute-factor policy, whose same-timestamp pose
blocks are zero.  The latter makes the identity/equality branch explicit
instead of silently treating an absent clean computeRelPose call as missing.
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
DEFAULT_REL = ROOT / "target" / "m7fw_clean_rel_jacs_all.json"
DEFAULT_RUST = ROOT / "target" / "m7fv_fresh5_detail.jsonl"
DEFAULT_OUTPUT = ROOT / "target" / "m7fw_clean_rel_jacs_all_comparison.json"
DEFAULT_REPORT = ROOT / "benchmarks" / "basalt" / "m7fw_clean_rel_jacs_all_report.md"

LANES = tuple(f"r{row}c{column}" for column in range(6) for row in range(2))


def f32(value: Any) -> float:
    return struct.unpack("<f", struct.pack("<f", float(value)))[0]


def f32_bits(value: Any) -> int:
    return struct.unpack("<I", struct.pack("<f", float(value)))[0]


def from_bits(value: str | int) -> float:
    return struct.unpack("<f", struct.pack("<I", int(value, 16) if isinstance(value, str) else int(value)))[0]


def hex_bits(value: Any) -> str:
    return f"{f32_bits(value):08x}"


def ulp_delta(left: Any, right: Any) -> int:
    left_bits, right_bits = f32_bits(left), f32_bits(right)
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


def matrix_column_major_bits_to_rows(values: list[str], rows: int, columns: int) -> list[list[float]]:
    if len(values) != rows * columns:
        raise ValueError(f"expected {rows * columns} column-major words, got {len(values)}")
    return [[from_bits(values[column * rows + row]) for column in range(columns)] for row in range(rows)]


def matrix_rows_to_column_major_bits(matrix: list[list[float]]) -> list[str]:
    return [hex_bits(matrix[row][column]) for column in range(len(matrix[0])) for row in range(len(matrix))]


def fma32(a: float, b: float, c: float) -> float:
    # Every input is binary32.  A binary32 product has at most 48 significant
    # bits and adding another binary32 needs at most 49, so binary64 evaluates
    # this exact product-plus-add before the final binary32 rounding.
    return f32(float(a) * float(b) + float(c))


def mul32(a: float, b: float) -> float:
    return f32(float(a) * float(b))


def add32(a: float, b: float) -> float:
    return f32(float(a) + float(b))


def matmul_2x6_6x6_f32(left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
    result = [[0.0] * 6 for _ in range(2)]
    for row in range(2):
        for column in range(6):
            # The pinned Eigen assignment kernel does not fold k=0..5 in
            # increasing order.  Its 2-row fixed-size path evaluates the
            # upper pair `(k4 + k5) + k3`, the lower pair `(k1 + k2) + k0`,
            # and then adds those pair sums.  This is also the operation
            # tree used by the source-faithful Rust pose-J kernel.
            high = mul32(left[row][4], right[4][column])
            high = fma32(left[row][5], right[5][column], high)
            high = fma32(left[row][3], right[3][column], high)
            low = mul32(left[row][1], right[1][column])
            low = fma32(left[row][2], right[2][column], low)
            low = fma32(left[row][0], right[0][column], low)
            result[row][column] = add32(high, low)
        
    return result


def scale_2x6_f32(matrix: list[list[float]], scalar: float) -> list[list[float]]:
    return [[mul32(value, scalar) for value in row] for row in matrix]


def zero_6x6() -> list[list[float]]:
    return [[0.0] * 6 for _ in range(6)]


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


def load_snapshot(path: Path) -> tuple[dict[str, Any], int]:
    records = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    snapshots = [
        record
        for record in records
        if record.get("record") == "snapshot"
        and record.get("frame_id") == 4
        and record.get("iteration") == 0
        and record.get("trial") == 0
        and record.get("phase") == "iteration_start"
    ]
    if len(snapshots) != 1:
        raise ValueError(f"expected one frame-4 iteration-0 iteration-start snapshot, got {len(snapshots)}")
    return snapshots[0], len(records)


def relation_name(host_frame: int, host_cam: int, target_frame: int, target_cam: int) -> str:
    if (target_frame, target_cam) == (host_frame, host_cam):
        return "same_timecam_identity"
    if target_frame == host_frame:
        return "same_timestamp_stereo"
    return "cross_time"


def relation_key_from_capture(record: dict[str, Any]) -> tuple[int, int, int, int]:
    caller = record["caller"]
    values = (
        caller.get("host_frame_id"),
        caller.get("host_cam"),
        caller.get("target_frame_id"),
        caller.get("target_cam"),
    )
    if any(value is None for value in values):
        raise ValueError(f"unresolved caller TimeCam key: {values}")
    return tuple(int(value) for value in values)  # type: ignore[return-value]


def rel_matrix(record: dict[str, Any], field: str) -> list[list[float]]:
    payload = record.get(field)
    if not payload:
        return zero_6x6()
    return matrix_column_major_bits_to_rows(payload["f32_bits_column_major"], 6, 6)


def metric(expected: list[list[float]], actual: list[list[Any]]) -> dict[str, Any]:
    expected_bits = matrix_rows_to_column_major_bits(expected)
    actual_values = [f32(value) for column in range(6) for row in range(2) for value in [actual[row][column]]]
    actual_bits = [
        hex_bits(actual[row][column])
        for column in range(6)
        for row in range(2)
    ]
    mismatches = []
    for index, (left, right) in enumerate(zip(expected_bits, actual_bits)):
        if left != right:
            expected_value = from_bits(left)
            actual_value = actual_values[index]
            mismatches.append(
                {
                    "lane": index,
                    "lane_name": LANES[index],
                    "expected_bits": left,
                    "actual_bits": right,
                    "expected": expected_value,
                    "actual": actual_value,
                    "ulp_delta": ulp_delta(expected_value, actual_value),
                    "delta_actual_minus_expected": actual_value - expected_value,
                }
            )
    mask = "".join("1" if left == right else "0" for left, right in zip(expected_bits, actual_bits))
    return {
        "exact_lanes": mask.count("1"),
        "total_lanes": len(mask),
        "mismatch_lanes": len(mismatches),
        "exact_row": int(mask and set(mask) == {"1"}),
        "mask": mask,
        "mismatches": mismatches,
    }


def matrix_json_values(matrix: Any) -> list[list[Any]]:
    if not isinstance(matrix, list) or len(matrix) != 2 or any(not isinstance(row, list) or len(row) != 6 for row in matrix):
        raise ValueError(f"expected 2x6 JSON matrix, got {matrix!r}")
    return matrix


def summarize(metrics: list[dict[str, Any]]) -> dict[str, Any]:
    mismatches = sum(item["mismatch_lanes"] for item in metrics)
    exact_rows = sum(item["exact_row"] for item in metrics)
    total = sum(item["total_lanes"] for item in metrics)
    return {
        "observations": len(metrics) // 2,
        "blocks": len(metrics),
        "exact_lanes": sum(item["exact_lanes"] for item in metrics),
        "total_lanes": total,
        "mismatch_lanes": mismatches,
        "exact_rows": exact_rows,
        "mismatch_rows": len(metrics) - exact_rows,
        "mask_counts": dict(Counter(item["mask"] for item in metrics)),
        "ulp_counts": dict(Counter(str(mismatch["ulp_delta"]) for item in metrics for mismatch in item["mismatches"])),
        "first_mismatch": next((mismatch for item in metrics for mismatch in item["mismatches"]), None),
    }


def compare(native_path: Path, rel_path: Path, rust_path: Path) -> dict[str, Any]:
    native_doc = json.loads(native_path.read_text(encoding="utf-8"))
    rel_doc = json.loads(rel_path.read_text(encoding="utf-8"))
    snapshot, rust_trace_records = load_snapshot(rust_path)
    native_records = native_doc["records"]
    rel_records = rel_doc["records"]

    relation_records: dict[tuple[int, int, int, int], dict[str, Any]] = {}
    duplicate_relations = []
    for record in rel_records:
        key = relation_key_from_capture(record)
        if key in relation_records:
            duplicate_relations.append(key)
        relation_records[key] = record

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
    by_relation: dict[str, dict[str, list[dict[str, Any]]]] = defaultdict(lambda: {"clean_chain": [], "rust_policy": []})

    for native in native_records:
        key = native_key(native)
        if key not in keyed:
            raise ValueError(f"native key not found in Rust detail: {key}")
        factor_index, observation_order, factor, observation = keyed[key]
        host_frame, host_cam = int(factor["host_frame_id"]), int(factor["host_cam"])
        target_frame, target_cam = int(observation["target_frame_id"]), int(observation["target_cam"])
        rel_key = (host_frame, host_cam, target_frame, target_cam)
        relation = relation_name(host_frame, host_cam, target_frame, target_cam)
        jxi = matrix_column_major_bits_to_rows(native["d_res_d_xi12"]["f32_bits"], 2, 6)
        captured = relation_records.get(rel_key)
        if captured is None and relation != "same_timecam_identity":
            raise ValueError(f"clean relation not captured for {rel_key}")
        if captured is not None and captured["jacobian_branch"] != "both_nonnull":
            raise ValueError(f"relation {rel_key} has unusable Jacobian branch {captured['jacobian_branch']}")
        clean_h = rel_matrix(captured, "d_rel_d_h") if captured is not None else zero_6x6()
        clean_t = rel_matrix(captured, "d_rel_d_t") if captured is not None else zero_6x6()
        same_timestamp = target_frame == host_frame
        rust_h = zero_6x6() if same_timestamp else clean_h
        rust_t = zero_6x6() if same_timestamp else clean_t
        sqrt_weight = f32(observation["sqrt_weight"])

        # Basalt scales d_res_d_xi in place before evaluating either fixed
        # 2x6*6x6 product.  Applying the weight after the product is a
        # distinct binary32 path, even though it is algebraically equivalent.
        weighted_jxi = scale_2x6_f32(jxi, sqrt_weight)
        clean_anchor = matmul_2x6_6x6_f32(weighted_jxi, clean_h)
        clean_target = matmul_2x6_6x6_f32(weighted_jxi, clean_t)
        rust_anchor = matmul_2x6_6x6_f32(weighted_jxi, rust_h)
        rust_target = matmul_2x6_6x6_f32(weighted_jxi, rust_t)

        actual_anchor = matrix_json_values(observation["jp_anchor"])
        actual_target = matrix_json_values(observation["jp_target"])
        clean_anchor_metric = metric(clean_anchor, actual_anchor)
        clean_target_metric = metric(clean_target, actual_target)
        rust_anchor_metric = metric(rust_anchor, actual_anchor)
        rust_target_metric = metric(rust_target, actual_target)
        row = {
            "native_ordinal": int(native["ordinal"]),
            "rust_flat_observation_ordinal": next(index for index, item in enumerate(flat) if item[0] == factor_index and item[1] == observation_order),
            "factor_index": factor_index,
            "track_id": int(factor["track_id"]),
            "observation_order": observation_order,
            "relation": relation,
            "host": {"frame": host_frame, "cam": host_cam},
            "target": {"frame": target_frame, "cam": target_cam},
            "clean_relation_capture": {
                "key": list(rel_key),
                "observed_call": captured is not None,
                "jacobian_branch": captured.get("jacobian_branch") if captured is not None else "identity_branch_no_computeRelPose_call",
            },
            "sqrt_weight_bits": hex_bits(sqrt_weight),
            "sqrt_weight": sqrt_weight,
            "clean_chain": {"jp_anchor": clean_anchor_metric, "jp_target": clean_target_metric},
            "rust_policy_chain": {"jp_anchor": rust_anchor_metric, "jp_target": rust_target_metric},
        }
        rows.append(row)
        by_relation[relation]["clean_chain"].extend([clean_anchor_metric, clean_target_metric])
        by_relation[relation]["rust_policy"].extend([rust_anchor_metric, rust_target_metric])

    all_relations = {(0, 0, frame, cam) for frame in range(5) for cam in range(2)}
    identity_key = (0, 0, 0, 0)
    relation_capture = {
        "expected_timecam_relations": 10,
        "expected_non_identity_relations": 9,
        "observed_unique_computeRelPose_relations": len(relation_records),
        "observed_relation_keys": [list(key) for key in sorted(relation_records)],
        "missing_expected_keys": [list(key) for key in sorted(all_relations - set(relation_records)) if key != identity_key],
        "duplicate_relation_keys": [list(key) for key in duplicate_relations],
        "same_timecam_identity": {
            "key": list(identity_key),
            "computeRelPose_call_observed": identity_key in relation_records,
            "branch": "tcid_h == tcid_t; source sets T_t_h identity and both d_rel Jacobians to zero without calling computeRelPose",
            "expected_d_rel_d_h": "all +0 binary32",
            "expected_d_rel_d_t": "all +0 binary32",
        },
        "same_timestamp_stereo": {
            "key": [0, 0, 0, 1],
            "computeRelPose_call_observed": (0, 0, 0, 1) in relation_records,
            "clean_capture_branch": relation_records.get((0, 0, 0, 1), {}).get("jacobian_branch"),
            "rust_absolute_pose_policy": "both jp blocks zero whenever host and target frame timestamps match",
        },
        "all_expected_keys_accounted_for": all_relations - set(relation_records) <= {identity_key},
    }

    def relation_stage_summary(mode: str) -> dict[str, Any]:
        output = {}
        for name in ("same_timecam_identity", "same_timestamp_stereo", "cross_time"):
            metrics = by_relation[name][mode]
            output[name] = summarize(metrics)
        return output

    clean_metrics = [metric for row in rows for metric in row["clean_chain"].values()]
    rust_metrics = [metric for row in rows for metric in row["rust_policy_chain"].values()]
    return {
        "schema": "visloc-rs.basalt.m7fw.clean-relpose-jacobians-all-comparison.v1",
        "status": "complete_read_only",
        "captured_at": str(date.today()),
        "scope": "584 exact-keyed frame-4 iteration-start observations; clean f32 Jxi × captured f32 relative Jacobians × Rust f32 sqrt weights versus serialized weighted absolute pose blocks, using Eigen's (k4+k5)+k3 and (k1+k2)+k0 reduction tree.",
        "inputs": {
            "native_m7ef": {"path": relative(native_path), "sha256": sha256(native_path), "records": len(native_records), "schema": native_doc.get("schema")},
            "clean_m7fw": {"path": relative(rel_path), "sha256": sha256(rel_path), "records": len(rel_records), "schema": rel_doc.get("schema"), "capture": rel_doc.get("capture", {})},
            "rust_m7fv": {"path": relative(rust_path), "sha256": sha256(rust_path), "trace_records": rust_trace_records, "snapshot": {"frame_id": snapshot.get("frame_id"), "iteration": snapshot.get("iteration"), "trial": snapshot.get("trial"), "phase": snapshot.get("phase"), "observations": len(flat), "numeric_types": snapshot.get("numeric_types")}},
        },
        "matching": {"native_records": len(native_records), "rust_observations": len(flat), "matched": len(rows), "method": "exact binary32 direction[2]+rho+pixel[2] key; relation selected by Rust host/target TimeCam", "all_584_keys_exact": len(rows) == 584},
        "relation_capture": relation_capture,
        "summary": {
            "clean_chain": summarize(clean_metrics),
            "rust_policy_chain": summarize(rust_metrics),
        },
        "relations": {"clean_chain": relation_stage_summary("clean_chain"), "rust_policy_chain": relation_stage_summary("rust_policy")},
        "boundary_assessment": {
            "same_timestamp": "The clean native call exists for same-timestamp stereo (0,0)->(0,1) and has nonzero post-write Jacobians. The same-TimeCam identity (0,0)->(0,0) does not call computeRelPose; source identity branch supplies zero matrices. Rust's absolute visual factor intentionally zeros both pose blocks for the whole same timestamp.",
            "interpretation": "The clean-chain mode is the direct requested oracle chain. The Rust-policy mode applies the source-compatible same-timestamp zero rule before comparison. Cross-time residual Jxi is sourced from clean M7ef and weights from the exact Rust snapshot.",
            "first_clean_chain_mismatch": next((m for row in rows for stage in row["clean_chain"].values() for m in stage["mismatches"]), None),
            "first_rust_policy_mismatch": next((m for row in rows for stage in row["rust_policy_chain"].values() for m in stage["mismatches"]), None),
        },
        "integrity": {"source_edits": False, "binary_rebuild": False, "build_performed": False, "commit_or_push": False, "one_bounded_gdb_run": True},
        "rows": rows,
    }


def frac(summary: dict[str, Any]) -> str:
    return f"{summary['exact_lanes']}/{summary['total_lanes']}"


def report_text(result: dict[str, Any]) -> str:
    relation_capture = result["relation_capture"]
    clean = result["summary"]["clean_chain"]
    rust = result["summary"]["rust_policy_chain"]
    lines = [
        "# M7fw clean relative-pose Jacobian chain",
        "",
        f"Date: {result['captured_at']} JST  ",
        "Status: **Complete read-only capture and 584-row comparison.**",
        "",
        "## Outcome",
        "",
        f"The bounded clean GDB capture reached the first five-state frame-4 `linearizeProblem` return and recorded {result['inputs']['clean_m7fw']['records']} post-write `computeRelPose` calls. They deduplicate to {relation_capture['observed_unique_computeRelPose_relations']} non-identity TimeCam relations; the tenth expected relation is the same-TimeCam identity branch, which does not call `computeRelPose` and supplies zero Jacobians.",
        "",
        "| chain used for expected weighted blocks | exact lanes | exact 2x6 blocks | total blocks |",
        "|---|---:|---:|---:|",
        f"| direct clean relative-J chain (same-timestamp stereo uses captured clean J) | {frac(clean)} | {clean['exact_rows']} | {clean['blocks']} |",
        f"| Rust same-timestamp-zero policy (cross-time uses captured clean J) | {frac(rust)} | {rust['exact_rows']} | {rust['blocks']} |",
        "",
        "Each metric is a 2x6 block: 12 Eigen column-major binary32 lanes. Expected values use clean M7ef binary32 `d_res_d_xi`, clean M7fw binary32 6x6 outputs, and the Rust snapshot's binary32 `sqrt_weight` (serialized as f64 but widened from f32). Matrix products use Eigen's pinned pair-tree reduction `(k4+k5)+k3`, `(k1+k2)+k0`, then pair-add, with scalar f32 rounding.",
        "",
        "## Relation capture",
        "",
        "| host | target | branch/capture |",
        "|---|---|---|",
    ]
    for key in sorted(relation_capture["observed_relation_keys"]):
        record = next(record for record in result["inputs"]["clean_m7fw"]["capture"].get("records", []) if False) if False else None
        lines.append(f"| ({key[0]}, cam{key[1]}) | ({key[2]}, cam{key[3]}) | post-write Jacobians captured |")
    lines.extend(
        [
            "| (0, cam0) | (0, cam0) | identity branch; no computeRelPose call; d_rel_d_h=d_rel_d_t=zero |",
            "",
            "Same-timestamp stereo `(frame 0, cam 0) -> (frame 0, cam 1)` did execute the Jacobian-bearing clean function and both buffers were non-null. Rust's absolute visual-factor path separately zeros both pose blocks whenever only the timestamp matches, so the report keeps the direct clean chain and Rust-policy chain distinct.",
            "",
            "## By relation",
            "",
            "| relation | observations | clean pose blocks | Rust-policy pose blocks |",
            "|---|---:|---:|---:|",
        ]
    )
    for name in ("same_timecam_identity", "same_timestamp_stereo", "cross_time"):
        c = result["relations"]["clean_chain"][name]
        r = result["relations"]["rust_policy_chain"][name]
        lines.append(f"| `{name}` | {c['observations']} | {c['exact_lanes']}/{c['total_lanes']} | {r['exact_lanes']}/{r['total_lanes']} |")
    lines.extend(
        [
            "",
            "The direct clean-chain mismatch is expected to expose any difference between native relative Jacobians and Rust's serialized absolute blocks, including the same-timestamp stereo policy. The Rust-policy mode isolates the remaining cross-time relative-J/absolute-chain boundary without treating the identity branch as a missing call.",
            "",
            "## Provenance and artifacts",
            "",
            f"- Clean raw capture: [`{result['inputs']['clean_m7fw']['path']}`](../../{result['inputs']['clean_m7fw']['path']})",
            f"- Clean raw GDB command: [`target/m7fw_clean_rel_jacs_all.cmd`](../../target/m7fw_clean_rel_jacs_all.cmd)",
            f"- Clean raw GDB output: [`target/m7fw_clean_rel_jacs_all.gdb.out`](../../target/m7fw_clean_rel_jacs_all.gdb.out)",
            f"- Native raw Jxi: [`{result['inputs']['native_m7ef']['path']}`](../../{result['inputs']['native_m7ef']['path']})",
            f"- Rust detail: [`{result['inputs']['rust_m7fv']['path']}`](../../{result['inputs']['rust_m7fv']['path']})",
            "- Comparator: [`m7fw_compare_clean_rel_jacs_all.py`](m7fw_compare_clean_rel_jacs_all.py)",
            f"- Comparison JSON: [`target/m7fw_clean_rel_jacs_all_comparison.json`](../../target/m7fw_clean_rel_jacs_all_comparison.json)",
            "",
            "No production or clean source was edited, no binary was rebuilt, and no commit or push was performed. The GDB run was one bounded inferior and stopped at the first frame-4 linearization return.",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--native", type=Path, default=DEFAULT_NATIVE)
    parser.add_argument("--rel", type=Path, default=DEFAULT_REL)
    parser.add_argument("--rust", type=Path, default=DEFAULT_RUST)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--report", type=Path, default=DEFAULT_REPORT)
    args = parser.parse_args()
    result = compare(args.native, args.rel, args.rust)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(report_text(result), encoding="utf-8")
    print(json.dumps({
        "output": relative(args.output),
        "report": relative(args.report),
        "matched": result["matching"]["matched"],
        "relations": result["relation_capture"],
        "clean_chain": result["summary"]["clean_chain"],
        "rust_policy_chain": result["summary"]["rust_policy_chain"],
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
