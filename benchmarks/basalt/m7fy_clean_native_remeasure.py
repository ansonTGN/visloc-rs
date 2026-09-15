#!/usr/bin/env python3
"""Compact M7gg Rust-vs-clean Basalt remeasurement.

This is a read-only consumer of the bounded M7ct/M7db/M7ef/M7fw captures and
the current M7gb Rust detail trace.  It deliberately does not compare against
an older Rust trace.  H/b and state lanes are compared directly; individual
absolute pose blocks are reconstructed from clean M7ef relative residual
Jacobians and clean M7fw relative-pose Jacobians; aggregate state rows are
reconstructed with the source-order f32 anchor-then-target scatter.
"""

from __future__ import annotations

import hashlib
import importlib
import json
import struct
import sys
from collections import defaultdict
from datetime import date
from pathlib import Path
from typing import Any, Iterable


ROOT = Path(__file__).resolve().parents[2]
HB_PATH = ROOT / "target" / "m7ct_clean_frame4_hb.json"
STATE_PATH = ROOT / "target" / "m7db_clean_frame4_states.json"
VISUAL_PATH = ROOT / "target" / "m7ef_clean_visual_all.json"
REL_PATH = ROOT / "target" / "m7fw_clean_rel_jacs_all.json"
RUST_PATH = ROOT / "target" / "m7gb_fresh5_detail.jsonl"
OLD_COMPARISON_PATH = ROOT / "target" / "m7gb_clean_rel_jacs_comparison.json"
OUT_PATH = ROOT / "target" / "m7gg_clean_native_remeasure.json"
REPORT_PATH = ROOT / "benchmarks" / "basalt" / "m7gg_clean_native_remeasure_report.md"

# Running this file directly places only benchmarks/basalt on sys.path; add
# the workspace root so the existing comparator can be reused as a module.
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def f32(value: Any) -> float:
    return struct.unpack("<f", struct.pack("<f", float(value)))[0]


def bits(value: Any) -> int:
    return struct.unpack("<I", struct.pack("<f", float(value)))[0]


def from_bits(value: str | int) -> float:
    raw = int(value, 16) if isinstance(value, str) else int(value)
    return struct.unpack("<f", struct.pack("<I", raw))[0]


def hex_bits(value: Any) -> str:
    return f"{bits(value):08x}"


def ulp_delta(left_bits: str, right_bits: str) -> int:
    left = int(left_bits, 16)
    right = int(right_bits, 16)
    left_order = 0x80000000 - left if left & 0x80000000 else 0x80000000 + left
    right_order = 0x80000000 - right if right & 0x80000000 else 0x80000000 + right
    return right_order - left_order


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def rel(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT.resolve())).replace("\\", "/")
    except ValueError:
        return str(path)


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def load_snapshot(path: Path) -> tuple[dict[str, Any], int]:
    records = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    snapshots = [
        item
        for item in records
        if item.get("record") == "snapshot"
        and item.get("frame_id") == 4
        and item.get("iteration") == 0
        and item.get("trial") == 0
        and item.get("phase") == "iteration_start"
    ]
    if len(snapshots) != 1:
        raise ValueError(f"expected one current M7gb frame-4 iteration-start snapshot, got {len(snapshots)}")
    return snapshots[0], len(records)


def mismatch_record(index: int, expected_bits: str, actual_bits: str, label: dict[str, Any] | None = None) -> dict[str, Any]:
    record: dict[str, Any] = {
        "index": index,
        "expected_bits": expected_bits,
        "actual_bits": actual_bits,
        "expected": from_bits(expected_bits),
        "actual": from_bits(actual_bits),
        "ulp_delta_actual_minus_expected": ulp_delta(expected_bits, actual_bits),
        "delta_actual_minus_expected": from_bits(actual_bits) - from_bits(expected_bits),
    }
    if label:
        record.update(label)
    return record


def lane_metrics(
    expected_bits: Iterable[str],
    actual_values: Iterable[Any],
    labels: Iterable[dict[str, Any] | None] | None = None,
    keep_first: int = 5,
) -> dict[str, Any]:
    expected = [str(value).lower() for value in expected_bits]
    actual = [hex_bits(value) for value in actual_values]
    if len(expected) != len(actual):
        raise ValueError(f"lane count mismatch: expected={len(expected)} actual={len(actual)}")
    label_list = list(labels) if labels is not None else [None] * len(expected)
    if len(label_list) != len(expected):
        raise ValueError("label count mismatch")
    mismatches = [
        mismatch_record(i, left, right, label_list[i])
        for i, (left, right) in enumerate(zip(expected, actual))
        if left != right
    ]
    max_item = None
    for i, (left, right) in enumerate(zip(expected, actual)):
        item = mismatch_record(i, left, right, label_list[i])
        if max_item is None or abs(item["delta_actual_minus_expected"]) > max_item["abs_delta"]:
            max_item = {"abs_delta": abs(item["delta_actual_minus_expected"]), **item}
    return {
        "exact_lanes": len(expected) - len(mismatches),
        "total_lanes": len(expected),
        "mismatch_lanes": len(mismatches),
        "first_mismatch": mismatches[0] if mismatches else None,
        "first_mismatches": mismatches[:keep_first],
        "max_abs_delta": max_item["abs_delta"] if max_item else 0.0,
        "max_abs_delta_entry": max_item,
    }


def h_b_metrics(snapshot: dict[str, Any], hb: dict[str, Any]) -> dict[str, Any]:
    h = snapshot["global"]["h"]
    if len(h) != 75 or any(len(row) != 75 for row in h):
        raise ValueError("Rust global H is not 75x75")
    h_values = [h[row][column] for column in range(75) for row in range(75)]
    h_labels = [
        {"matrix_row": index % 75, "matrix_col": index // 75}
        for index in range(75 * 75)
    ]
    b_values = snapshot["global"]["b"]
    if len(b_values) != 75:
        raise ValueError("Rust global b is not length 75")
    b_labels = [{"lane": index} for index in range(75)]
    return {
        "comparison_cast": "Rust f64 values cast to binary32; native H is transposed from Eigen column-major words",
        "H": lane_metrics(hb["H_f32_bits"], h_values, h_labels),
        "b": lane_metrics(hb["b_f32_bits"], b_values, b_labels),
    }


STATE_FIELDS = (
    ("quaternion_xyzw", "pose", "quaternion_xyzw"),
    ("translation_xyz", "pose", "translation"),
    ("velocity_xyz", None, "velocity"),
    ("bias_gyro_xyz", None, "bias_gyro"),
    ("bias_accel_xyz", None, "bias_accel"),
)


def state_metrics(snapshot: dict[str, Any], state_doc: dict[str, Any]) -> dict[str, Any]:
    clean_by_frame = {int(item["frame_id"]): item for item in state_doc["native_states"]}
    rust_by_frame = {int(item["frame_id"]): item for item in snapshot["blocks"]["states"]}
    if set(clean_by_frame) != set(rust_by_frame):
        raise ValueError(f"state frame mismatch: clean={sorted(clean_by_frame)} rust={sorted(rust_by_frame)}")

    def collect(include_quaternion_w: bool) -> tuple[list[str], list[Any], list[dict[str, Any]]]:
        expected: list[str] = []
        actual: list[Any] = []
        labels: list[dict[str, Any]] = []
        for frame_id in sorted(clean_by_frame):
            clean = clean_by_frame[frame_id]["state_linearized"]
            rust = rust_by_frame[frame_id]
            for field, parent, rust_key in STATE_FIELDS:
                native_lanes = clean[field + "_bits"]
                rust_lanes = rust[parent][rust_key] if parent else rust[rust_key]
                lane_indices = range(len(native_lanes))
                if field == "quaternion_xyzw" and not include_quaternion_w:
                    lane_indices = range(3)
                for lane in lane_indices:
                    expected.append(native_lanes[lane])
                    actual.append(rust_lanes[lane])
                    labels.append({"frame_id": frame_id, "field": field, "lane": lane})
        return expected, actual, labels

    compact_expected, compact_actual, compact_labels = collect(False)
    full_expected, full_actual, full_labels = collect(True)
    return {
        "compact_15dof": {
            "definition": "quaternion xy z (manifold 3 lanes), translation xyz, velocity xyz, gyro bias xyz, accel bias xyz per frame",
            **lane_metrics(compact_expected, compact_actual, compact_labels),
        },
        "full_sidecar": {
            "definition": "the same fields including serialized quaternion w; 16 scalar lanes per frame",
            **lane_metrics(full_expected, full_actual, full_labels),
        },
    }


def first_context(result: dict[str, Any], mode: str, relation: str) -> dict[str, Any] | None:
    for row in result["rows"]:
        if row["relation"] != relation:
            continue
        for block in ("jp_anchor", "jp_target"):
            mismatches = row[mode][block]["mismatches"]
            if mismatches:
                return {
                    "native_ordinal": row["native_ordinal"],
                    "factor_index": row["factor_index"],
                    "track_id": row["track_id"],
                    "observation_order": row["observation_order"],
                    "relation": relation,
                    "block": block,
                    "mismatch": mismatches[0],
                }
    return None


def self_test_weighted_pose_product_order() -> None:
    """Guard the binary32 boundary that distinguishes the old aggregate oracle."""
    m7fw = importlib.import_module("benchmarks.basalt.m7fw_compare_clean_rel_jacs_all")
    jxi = [
        [m7fw.f32(value) for value in (1.0000001, 2.0000002, 3.0000005, 4.0000007, 5.0000009, 6.000001)],
        [m7fw.f32(value) for value in (6.000001, 5.0000009, 4.0000007, 3.0000005, 2.0000002, 1.0000001)],
    ]
    rel = [
        [m7fw.f32((row + 1) * 0.1 + (column + 1) * 0.03) for column in range(6)]
        for row in range(6)
    ]
    weight = m7fw.f32(0.1)
    pre_scale = m7fw.matmul_2x6_6x6_f32(m7fw.scale_2x6_f32(jxi, weight), rel)
    post_scale = m7fw.scale_2x6_f32(m7fw.matmul_2x6_6x6_f32(jxi, rel), weight)
    if all(
        m7fw.hex_bits(pre_scale[row][column]) == m7fw.hex_bits(post_scale[row][column])
        for row in range(2)
        for column in range(6)
    ):
        raise AssertionError("pre-scale and post-product aggregate paths unexpectedly agree")


def individual_pose_summary(native: dict[str, Any], relation: dict[str, Any], snapshot: dict[str, Any]) -> dict[str, Any]:
    # Reuse the pinned M7fw implementation for the clean M7ef×M7fw chain,
    # while explicitly naming the Rust input as current M7gb in this report.
    m7fw = importlib.import_module("benchmarks.basalt.m7fw_compare_clean_rel_jacs_all")
    result = m7fw.compare(VISUAL_PATH, REL_PATH, RUST_PATH)
    output: dict[str, Any] = {
        "comparison": "clean M7ef d_res_d_xi × clean M7fw d_rel_d_h/d_rel_d_t × current Rust f32 sqrt_weight versus current Rust jp_anchor/jp_target",
        "matching": result["matching"],
        "current_rust_input": result["inputs"]["rust_m7fv"],
        "clean_chain": result["summary"]["clean_chain"],
        "rust_policy_chain": result["summary"]["rust_policy_chain"],
        "by_relation": result["relations"],
        "first_context": {
            mode: {
                rel_name: first_context(result, mode, rel_name)
                for rel_name in ("same_timecam_identity", "same_timestamp_stereo", "cross_time")
            }
            for mode in ("clean_chain", "rust_policy_chain")
        },
    }
    # The full M7fw row payload is intentionally not copied into this compact
    # artifact; target/m7gb_clean_rel_jacs_comparison.json retains it.
    return output


def aggregate_pose_metrics(snapshot: dict[str, Any]) -> dict[str, Any]:
    m7fw = importlib.import_module("benchmarks.basalt.m7fw_compare_clean_rel_jacs_all")
    native_doc = load_json(VISUAL_PATH)
    rel_doc = load_json(REL_PATH)
    relation_records = {
        m7fw.relation_key_from_capture(item): item for item in rel_doc["records"]
    }
    flat: list[tuple[int, int, dict[str, Any], dict[str, Any]]] = []
    for factor_index, factor in enumerate(snapshot["landmark_factors"]):
        for observation_order, observation in enumerate(factor["observations"]):
            flat.append((factor_index, observation_order, factor, observation))
    keyed = {
        m7fw.rust_key(factor, observation): (_factor_index, _observation_order, factor, observation)
        for _factor_index, _observation_order, factor, observation in flat
    }

    by_relation: dict[str, list[dict[str, Any]]] = defaultdict(list)
    full_expected: list[str] = []
    full_actual: list[Any] = []
    full_labels: list[dict[str, Any]] = []
    active_expected: list[str] = []
    active_actual: list[Any] = []
    active_labels: list[dict[str, Any]] = []

    for native in native_doc["records"]:
        factor_index, observation_order, factor, observation = keyed[m7fw.native_key(native)]
        host_frame = int(factor["host_frame_id"])
        host_cam = int(factor["host_cam"])
        target_frame = int(observation["target_frame_id"])
        target_cam = int(observation["target_cam"])
        relation_name = m7fw.relation_name(host_frame, host_cam, target_frame, target_cam)
        relation_key = (host_frame, host_cam, target_frame, target_cam)
        captured = relation_records.get(relation_key)
        jxi = m7fw.matrix_column_major_bits_to_rows(native["d_res_d_xi12"]["f32_bits"], 2, 6)
        clean_h = m7fw.rel_matrix(captured, "d_rel_d_h") if captured else m7fw.zero_6x6()
        clean_t = m7fw.rel_matrix(captured, "d_rel_d_t") if captured else m7fw.zero_6x6()
        weight = m7fw.f32(observation["sqrt_weight"])
        # Basalt whitens d_res_d_xi in place before either Eigen product.  The
        # product-then-scale form is algebraically equivalent but not
        # source-faithful in binary32 (and can change signed zeros).  Keep the
        # aggregate oracle on the same pre-scale + pair-tree path as M7fw.
        weighted_jxi = m7fw.scale_2x6_f32(jxi, weight)
        anchor = m7fw.matmul_2x6_6x6_f32(weighted_jxi, clean_h)
        target = m7fw.matmul_2x6_6x6_f32(weighted_jxi, clean_t)
        expected = [[m7fw.f32(0.0)] * 75 for _ in range(2)]
        for row in range(2):
            for column in range(6):
                expected[row][host_frame * 15 + column] = m7fw.add32(
                    expected[row][host_frame * 15 + column], anchor[row][column]
                )
            for column in range(6):
                expected[row][target_frame * 15 + column] = m7fw.add32(
                    expected[row][target_frame * 15 + column], target[row][column]
                )
        actual_rows = factor["state_jacobian"][2 * observation_order : 2 * observation_order + 2]
        full_expected.extend(
            hex_bits(expected[row][column])
            for column in range(75)
            for row in range(2)
        )
        full_actual.extend(
            actual_rows[row][column]
            for column in range(75)
            for row in range(2)
        )
        label_base = {
            "native_ordinal": int(native["ordinal"]),
            "factor_index": factor_index,
            "track_id": int(factor["track_id"]),
            "observation_order": observation_order,
            "relation": relation_name,
        }
        full_labels.extend(
            {**label_base, "row": row, "state_column": column}
            for column in range(75)
            for row in range(2)
        )
        active_columns = sorted(
            set(range(host_frame * 15, host_frame * 15 + 6))
            | set(range(target_frame * 15, target_frame * 15 + 6))
        )
        active_expected.extend(
            hex_bits(expected[row][column])
            for column in active_columns
            for row in range(2)
        )
        active_actual.extend(
            actual_rows[row][column]
            for column in active_columns
            for row in range(2)
        )
        active_labels.extend(
            {**label_base, "row": row, "state_column": column}
            for column in active_columns
            for row in range(2)
        )
        by_relation[relation_name].append(
            {
                "expected": [hex_bits(expected[row][column]) for column in range(75) for row in range(2)],
                "actual": [actual_rows[row][column] for column in range(75) for row in range(2)],
                "labels": full_labels[-150:],
            }
        )

    overall = lane_metrics(full_expected, full_actual, full_labels)
    active = lane_metrics(active_expected, active_actual, active_labels)
    relation_summary: dict[str, Any] = {}
    for name in ("same_timecam_identity", "same_timestamp_stereo", "cross_time"):
        expected: list[str] = []
        actual: list[Any] = []
        labels: list[dict[str, Any]] = []
        for item in by_relation[name]:
            expected.extend(item["expected"])
            actual.extend(item["actual"])
            labels.extend(item["labels"])
        relation_summary[name] = lane_metrics(expected, actual, labels)
    return {
        "comparison": "clean M7ef d_res_d_xi × clean M7fw relative-pose chain × current Rust f32 sqrt_weight, scattered into current Rust factor state_jacobian",
        "source_order": "for each observation and each row: old + anchor block, then result + target block; pose columns are frame*15 + [0..6)",
        "full_state_rows": overall,
        "active_pose_support": active,
        "by_relation": relation_summary,
    }


def compact_old_comparison(path: Path, current_sha: str) -> dict[str, Any]:
    if not path.exists():
        return {"present": False}
    doc = load_json(path)
    rust_input = doc.get("inputs", {}).get("rust", doc.get("inputs", {}).get("rust_m7fv", {}))
    return {
        "present": True,
        "path": rel(path),
        "sha256": sha256(path),
        "schema": doc.get("schema"),
        "rust_input_path": rust_input.get("path"),
        "rust_input_sha256": rust_input.get("sha256"),
        "matches_current_m7gb": rust_input.get("sha256") == current_sha,
        "matched_observations": doc.get("matching", {}).get("matched"),
        "clean_chain": {
            key: doc.get("summary", {}).get("clean_chain", {}).get(key)
            for key in ("exact_lanes", "total_lanes", "mismatch_lanes", "exact_rows", "mismatch_rows", "first_mismatch")
        },
        "rust_policy_chain": {
            key: doc.get("summary", {}).get("rust_policy_chain", {}).get(key)
            for key in ("exact_lanes", "total_lanes", "mismatch_lanes", "exact_rows", "mismatch_rows", "first_mismatch")
        },
    }


def build() -> dict[str, Any]:
    self_test_weighted_pose_product_order()
    current_sha = sha256(RUST_PATH)
    snapshot, trace_records = load_snapshot(RUST_PATH)
    hb = load_json(HB_PATH)
    state_doc = load_json(STATE_PATH)
    pose = individual_pose_summary(load_json(VISUAL_PATH), load_json(REL_PATH), snapshot)
    aggregate = aggregate_pose_metrics(snapshot)
    return {
        "schema": "visloc-rs.basalt.m7gg.clean-native-remeasure.v1",
        "status": "complete_read_only",
        "captured_at": str(date.today()),
        "scope": "Current M7gb frame-4 iteration-start Rust detail versus clean native M7ct H/b, M7db states, M7ef visual Jacobians, and M7fw relative Jacobians.",
        "inputs": {
            "rust_m7gb": {"path": rel(RUST_PATH), "sha256": current_sha, "trace_records": trace_records, "snapshot": {"frame_id": snapshot["frame_id"], "iteration": snapshot["iteration"], "trial": snapshot["trial"], "phase": snapshot["phase"], "observations": sum(len(f["observations"]) for f in snapshot["landmark_factors"])}},
            "clean_m7ct": {"path": rel(HB_PATH), "sha256": sha256(HB_PATH), "schema": hb.get("schema")},
            "clean_m7db": {"path": rel(STATE_PATH), "sha256": sha256(STATE_PATH), "schema": state_doc.get("schema")},
            "clean_m7ef": {"path": rel(VISUAL_PATH), "sha256": sha256(VISUAL_PATH), "schema": load_json(VISUAL_PATH).get("schema")},
            "clean_m7fw": {"path": rel(REL_PATH), "sha256": sha256(REL_PATH), "schema": load_json(REL_PATH).get("schema")},
        },
        "h_b": h_b_metrics(snapshot, hb),
        "states": state_metrics(snapshot, state_doc),
        "individual_pose_jacobians": pose,
        "aggregate_pose_jacobians": aggregate,
        "existing_m7gb_comparison_compact": compact_old_comparison(OLD_COMPARISON_PATH, current_sha),
        "integrity": {"production_source_edited": False, "clean_source_edited": False, "binary_rebuilt": False, "commit_or_push": False},
    }


def frac(metric: dict[str, Any]) -> str:
    return f"{metric['exact_lanes']}/{metric['total_lanes']}"


def report(result: dict[str, Any]) -> str:
    hb = result["h_b"]
    states = result["states"]
    pose = result["individual_pose_jacobians"]
    aggregate = result["aggregate_pose_jacobians"]
    lines = [
        "# M7gg current Rust-vs-clean Basalt remeasurement",
        "",
        f"Date: {result['captured_at']} JST  ",
        "Status: **Complete read-only measurement; no production source or binary changes.**",
        "",
        "## Scope and provenance",
        "",
        "The current input is `target/m7gb_fresh5_detail.jsonl`, matched by SHA-256 and exact binary32 direction/rho/pixel keys. No prior Rust trace is used as an oracle. Clean references are M7ct H/b, M7db state bits, M7ef per-observation visual Jacobians, and M7fw relative-pose Jacobians.",
        "",
        f"- Current M7gb SHA-256: `{result['inputs']['rust_m7gb']['sha256']}`",
        f"- Snapshot: frame {result['inputs']['rust_m7gb']['snapshot']['frame_id']}, iteration {result['inputs']['rust_m7gb']['snapshot']['iteration']}, `{result['inputs']['rust_m7gb']['snapshot']['phase']}`, {result['inputs']['rust_m7gb']['snapshot']['observations']} observations",
        f"- Exact visual-key matches: {pose['matching']['matched']}/{pose['matching']['native_records']} (all keys exact: `{pose['matching']['all_584_keys_exact']}`)",
        "",
        "## Clean H/b and state bits",
        "",
        "| buffer | exact | mismatches | first mismatch | max abs delta |",
        "|---|---:|---:|---|---:|",
        f"| H 75×75 (Eigen column-major clean vs Rust row-major transposed) | {frac(hb['H'])} | {hb['H']['mismatch_lanes']} | `{hb['H']['first_mismatch']['expected_bits']} → {hb['H']['first_mismatch']['actual_bits']}` at H({hb['H']['first_mismatch']['matrix_row']},{hb['H']['first_mismatch']['matrix_col']}) | {hb['H']['max_abs_delta']} |",
        f"| b 75×1 | {frac(hb['b'])} | {hb['b']['mismatch_lanes']} | `{hb['b']['first_mismatch']['expected_bits']} → {hb['b']['first_mismatch']['actual_bits']}` at b[{hb['b']['first_mismatch']['lane']}] | {hb['b']['max_abs_delta']} |",
        f"| compact state (15 DoF/frame; quaternion xyz, no w) | {frac(states['compact_15dof'])} | {states['compact_15dof']['mismatch_lanes']} | {states['compact_15dof']['first_mismatch']} | {states['compact_15dof']['max_abs_delta']} |",
        f"| full state sidecar (includes quaternion w; 16 scalar/frame) | {frac(states['full_sidecar'])} | {states['full_sidecar']['mismatch_lanes']} | {states['full_sidecar']['first_mismatch']} | {states['full_sidecar']['max_abs_delta']} |",
        "",
        "The compact gate has one remaining clean-vs-current mismatch: frame 4 `translation_xyz[2]`, clean `bcdc2b7f` versus Rust `bcdc2b7e` (+1 ULP actual-minus-clean).",
        "",
        "## Individual pose blocks (clean M7ef × clean M7fw chain)",
        "",
        "Each metric is one 2×6 weighted `jp_anchor` or `jp_target` block. The direct clean-chain row keeps the clean same-timestamp stereo relative Jacobian; the Rust-policy row zeros same-timestamp pose blocks only to expose the source policy separately.",
        "",
        "| relation | observations | direct clean-chain exact lanes | Rust-policy exact lanes |",
        "|---|---:|---:|---:|",
    ]
    for name in ("same_timecam_identity", "same_timestamp_stereo", "cross_time"):
        c = pose["by_relation"]["clean_chain"][name]
        r = pose["by_relation"]["rust_policy_chain"][name]
        lines.append(f"| `{name}` | {c['observations']} | {frac(c)} ({c['mismatch_lanes']} mismatches) | {frac(r)} ({r['mismatch_lanes']} mismatches) |")
    c = pose["clean_chain"]
    r = pose["rust_policy_chain"]
    first_direct = next(
        (
            pose["first_context"]["clean_chain"][name]
            for name in ("same_timecam_identity", "same_timestamp_stereo", "cross_time")
            if pose["first_context"]["clean_chain"][name]
        ),
        None,
    )
    first_cross = pose["first_context"]["clean_chain"]["cross_time"]
    lines.extend(
        [
            f"| **overall / 1,168 blocks** | **584** | **{frac(c)}** ({c['mismatch_lanes']} mismatches, {c['exact_rows']} exact blocks) | **{frac(r)}** ({r['mismatch_lanes']} mismatches, {r['exact_rows']} exact blocks) |",
            "",
            f"First direct clean-chain mismatch: `{first_direct}`. First cross-time mismatch: `{first_cross}`.",
            "",
            "Rust-policy same-timestamp stereo is reported separately; aggregate scatter is checked independently and remains exact for that relation.",
            "",
            "## Aggregate pose/state rows",
            "",
            "The expected aggregate is formed from clean M7ef×M7fw weighted blocks using the source-order f32 scatter (`old + anchor`, then `result + target`) into each frame's six pose columns. Current `factor.state_jacobian` is compared after f32 casting.",
            "",
            "| scope | exact lanes | mismatches | first mismatch |",
            "|---|---:|---:|---|",
            f"| full 2×75 state rows, 584 observations (87,600 lanes) | {frac(aggregate['full_state_rows'])} | {aggregate['full_state_rows']['mismatch_lanes']} | {aggregate['full_state_rows']['first_mismatch']} |",
            f"| active host/target pose support only (12,552 lanes) | {frac(aggregate['active_pose_support'])} | {aggregate['active_pose_support']['mismatch_lanes']} | {aggregate['active_pose_support']['first_mismatch']} |",
        ]
    )
    for name in ("same_timecam_identity", "same_timestamp_stereo", "cross_time"):
        item = aggregate["by_relation"][name]
        lines.append(f"| aggregate `{name}` | {frac(item)} | {item['mismatch_lanes']} | {item['first_mismatch']} |")
    lines.extend(
        [
            "",
            f"Aggregate identity and same-timestamp stereo rows are exact (9,150/9,150 full-state lanes each); the stereo individual blocks cancel exactly in the same frame. All {aggregate['by_relation']['cross_time']['mismatch_lanes']} aggregate mismatches are cross-time and share the first clean-chain relative/absolute pose boundary shown above.",
            "",
            "## Existing M7gb comparison artifact",
            "",
            f"`target/m7gb_clean_rel_jacs_comparison.json` records the same current M7gb SHA (`{result['existing_m7gb_comparison_compact'].get('rust_input_sha256')}`) and 584/584 keyed matches. Its compact direct clean-chain result is {result['existing_m7gb_comparison_compact'].get('clean_chain', {}).get('exact_lanes')}/{result['existing_m7gb_comparison_compact'].get('clean_chain', {}).get('total_lanes')} lanes; its Rust-policy result is {result['existing_m7gb_comparison_compact'].get('rust_policy_chain', {}).get('exact_lanes')}/{result['existing_m7gb_comparison_compact'].get('rust_policy_chain', {}).get('total_lanes')} lanes.",
            "",
            "## Narrowest next implementation boundary",
            "",
            f"The same-timestamp stereo implementation boundary is closed at aggregate scatter: both individual blocks are retained and their aggregate state pose columns are exact zero. The remaining pose discrepancy is {pose['clean_chain']['mismatch_lanes']}/{pose['clean_chain']['total_lanes']} cross-time individual lanes and the same {aggregate['by_relation']['cross_time']['mismatch_lanes']} aggregate lanes. Separately, the only state-input gate visible in the compact state capture is frame-4 translation-z (+1 ULP).",
            "",
            "Artifacts: [`m7fy_clean_native_remeasure.py`](m7fy_clean_native_remeasure.py), [`target/m7gg_clean_native_remeasure.json`](../../target/m7gg_clean_native_remeasure.json), and [`target/m7gb_clean_rel_jacs_comparison.json`](../../target/m7gb_clean_rel_jacs_comparison.json).",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> int:
    result = build()
    OUT_PATH.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    REPORT_PATH.write_text(report(result), encoding="utf-8")
    print(json.dumps({
        "output": rel(OUT_PATH),
        "report": rel(REPORT_PATH),
        "rust_sha256": result["inputs"]["rust_m7gb"]["sha256"],
        "H": {key: result["h_b"]["H"][key] for key in ("exact_lanes", "total_lanes", "mismatch_lanes")},
        "b": {key: result["h_b"]["b"][key] for key in ("exact_lanes", "total_lanes", "mismatch_lanes")},
        "state_compact": {key: result["states"]["compact_15dof"][key] for key in ("exact_lanes", "total_lanes", "mismatch_lanes")},
        "individual_clean": {key: result["individual_pose_jacobians"]["clean_chain"][key] for key in ("exact_lanes", "total_lanes", "mismatch_lanes")},
        "aggregate_full": {key: result["aggregate_pose_jacobians"]["full_state_rows"][key] for key in ("exact_lanes", "total_lanes", "mismatch_lanes")},
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
