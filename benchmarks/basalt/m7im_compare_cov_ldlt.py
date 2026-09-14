#!/usr/bin/env python3
"""Bit-level comparison for the M7 covariance/LDLT/whitening boundary.

The oracle JSON is produced by m7im_cov_ldlt_oracle.sh.  The comparator also
accepts the existing Rust m7aq four-link audit and the diagnostic Rust
m7hd_step_probe stdout.  A legacy audit is intentionally reported as a
comparison result (rather than silently treated as authoritative) because it
may have been generated before the current pinned Eigen covariance GEMM
schedule was fixed.
"""

from __future__ import annotations

import argparse
import json
import re
import struct
from pathlib import Path
from typing import Any


def f32_bits(value: Any) -> int:
    return struct.unpack("<I", struct.pack("<f", float(value)))[0]


def first_diff(left: list[int], right: list[int]) -> dict[str, Any] | None:
    for index, (lhs, rhs) in enumerate(zip(left, right)):
        if lhs != rhs:
            return {"index": index, "left": f"{lhs:08x}", "right": f"{rhs:08x}"}
    if len(left) != len(right):
        return {"index": min(len(left), len(right)), "left": None, "right": None}
    return None


def bits_field(value: Any) -> list[int]:
    """Read an oracle bit object or legacy decimal nested matrix/vector."""
    if isinstance(value, dict):
        return [int(item, 16) for item in value["bits_row_major"]]
    if isinstance(value, list):
        result: list[int] = []
        for item in value:
            if isinstance(item, list):
                result.extend(bits_field(item))
            elif isinstance(item, str):
                result.append(int(item, 16))
            else:
                result.append(f32_bits(item))
        return result
    raise TypeError(type(value).__name__)


def oracle_field(oracle: dict[str, Any], field: str) -> list[int]:
    return bits_field(oracle["frame4_factor"][field])


def rust_field(link: dict[str, Any], field: str) -> list[int]:
    return bits_field(link[field])


def compare(name: str, left: list[int], right: list[int],
            source_field: str | None = None) -> dict[str, Any]:
    mismatch = sum(lhs != rhs for lhs, rhs in zip(left, right))
    result: dict[str, Any] = {
        "field": name,
        "left_count": len(left),
        "right_count": len(right),
        "mismatch_count": mismatch + (abs(len(left) - len(right))),
        "first_diff": first_diff(left, right),
        "exact": mismatch == 0 and len(left) == len(right),
    }
    if source_field is not None:
        # Keep the selected scalar boundary explicit.  In particular, the
        # injected Rust audit also carries widened f64 `raw_residual` and
        # `jacobian` fields; those are deliberately not this comparison.
        result["source_field"] = source_field
    return result


INJECTED_RUST_FIELDS = {
    "covariance": "covariance",
    "sqrt_information": "sqrt_information",
    "raw_residual": "raw_residual_f32_exact",
    "jacobian": "jacobian_f32_exact",
    "whitened_residual": "whitened_residual",
    "whitened_jacobian": "whitened_jacobian",
    "row_h": "row_h",
    "row_b": "row_b",
}


def select_rust_frame4_fields(rust: dict[str, Any]) -> tuple[
        dict[str, Any], dict[str, str], dict[str, Any]]:
    """Select only fields at the injected Rust f32 boundary.

    The m7aq/m7im injected audit stores both widened diagnostic values
    (`raw_residual`, `jacobian`) and the exact f32 values emitted by the
    injected evaluator.  Comparing the widened values can manufacture a
    false factor mismatch, so fail closed if either exact field is absent.
    """
    if "links" in rust:
        links = rust["links"]
        if not isinstance(links, list) or len(links) <= 3:
            raise ValueError("injected Rust schema has no frame-3 -> frame-4 link")
        link = links[3]
        missing = [name for name in INJECTED_RUST_FIELDS.values() if name not in link]
        if missing:
            raise ValueError(
                "injected Rust link is missing exact f32 fields: " + ", ".join(missing))
        return link, dict(INJECTED_RUST_FIELDS), {
            "mode": "injected_links",
            "link_index": 3,
            "from_frame": link.get("from_frame"),
            "to_frame": link.get("to_frame"),
            "scalar_contract": "rust-f32-exact",
        }

    # A compact direct schema is accepted only when it advertises the same
    # exact fields; do not silently fall back to legacy decimal f64 fields.
    if "frame4_factor" in rust:
        factor = rust["frame4_factor"]
        direct_fields = dict(INJECTED_RUST_FIELDS)
        missing = [name for name in direct_fields.values() if name not in factor]
        if missing:
            raise ValueError(
                "direct Rust schema is missing exact f32 fields: " + ", ".join(missing))
        return factor, direct_fields, {
            "mode": "direct_frame4_factor",
            "scalar_contract": "rust-f32-exact",
        }

    raise ValueError("unsupported Rust factor schema (expected links or frame4_factor)")


def parse_kv(line: str) -> dict[str, str]:
    return dict(re.findall(r"(?:^| )([A-Za-z_][A-Za-z0-9_]*)=([^ ]+)", line))


def rust_packet_covariance(path: Path | None) -> list[list[int]] | None:
    if path is None or not path.exists():
        return None
    try:
        parsed = json.loads(path.read_text())
    except json.JSONDecodeError:
        parsed = None
    if isinstance(parsed, dict) and "first_link_packets" in parsed:
        records = parsed["first_link_packets"]
        return [
            [int(item, 16) for item in record["record"]["covariance_after"]["bits_column_major"]]
            for record in records[:10]
        ]
    result: list[list[int]] = []
    for line in path.read_text().splitlines():
        if not line.startswith("RUST_M7HD_STEP"):
            continue
        fields = parse_kv(line)
        if "cov" in fields:
            result.append([int(item, 16) for item in fields["cov"].split(",")])
    return result or None


def native_packet_covariance(path: Path | None) -> list[list[int]] | None:
    if path is None or not path.exists():
        return None
    result: list[list[int]] = []
    for line in path.read_text().splitlines():
        if not line.startswith("M7HD_NATIVE_COV"):
            continue
        fields = parse_kv(line)
        result.append([int(item, 16) for item in fields["cov"].split(",")])
    return result or None


def packet_summary(oracle: dict[str, Any], records: list[list[int]] | None,
                   label: str) -> dict[str, Any]:
    expected = [
        [int(item, 16) for item in packet["record"]["covariance_after"]["bits_column_major"]]
        for packet in oracle["first_link_packets"][:10]
    ]
    if records is None:
        return {"label": label, "status": "not_comparable", "expected_packets": len(expected)}
    records = records[:10]
    diffs = [compare(f"packet_{i + 1}.covariance", expected[i], actual)
             for i, actual in enumerate(records[:len(expected)])]
    total = sum(item["mismatch_count"] for item in diffs)
    first = next((item for item in diffs if not item["exact"]), None)
    return {
        "label": label,
        "status": "exact" if not first and len(records) == len(expected) else "mismatch",
        "expected_packets": len(expected),
        "actual_packets": len(records),
        "total_bit_mismatches": total,
        "first_mismatch": first,
        "per_packet": diffs,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--oracle", type=Path,
                        default=Path("target/m7im_cov_ldlt_oracle_20260824.json"))
    parser.add_argument("--rust", type=Path,
                        default=Path("target/m7aq_rust_imu_audit_after_fma.json"))
    parser.add_argument("--rust-packets", type=Path,
                        default=Path("target/m7hd_step_probe_after_fma.stdout"))
    parser.add_argument("--native-packets", type=Path,
                        default=Path("target/m7hd_native_intermediate.jsonl"))
    parser.add_argument("--report", type=Path,
                        default=Path("target/m7im_cov_ldlt_comparison_20260824.json"))
    args = parser.parse_args()

    oracle = json.loads(args.oracle.read_text())
    rust = json.loads(args.rust.read_text()) if args.rust.exists() else None
    report: dict[str, Any] = {
        "schema": "basalt.m7im_cov_ldlt_comparison.v1",
        "oracle": str(args.oracle),
        "rust": str(args.rust),
        "contract": oracle["whitener_contract"],
        "first_link_packet_comparisons": [],
        "frame4_factor_comparisons": [],
    }

    report["first_link_packet_comparisons"].append(
        packet_summary(oracle, native_packet_covariance(args.native_packets), "native_trace"))
    report["first_link_packet_comparisons"].append(
        packet_summary(oracle, rust_packet_covariance(args.rust_packets), "rust_trace"))

    if rust is None:
        report["frame4_factor_comparisons"].append({"status": "not_comparable"})
    else:
        try:
            link, rust_fields, selection = select_rust_frame4_fields(rust)
        except (KeyError, TypeError, ValueError) as error:
            report["frame4_factor_selection"] = {"status": "rejected", "error": str(error)}
            report["frame4_factor_comparisons"].append({"status": "unsupported_rust_schema"})
        else:
            report["frame4_factor_selection"] = {
                **selection,
                "fields": rust_fields,
            }
            for field, selected_field in rust_fields.items():
                report["frame4_factor_comparisons"].append(
                    compare(field, oracle_field(oracle, field),
                            rust_field(link, selected_field), selected_field))

    packet_status = [item["status"] for item in report["first_link_packet_comparisons"]]
    factor = report["frame4_factor_comparisons"]
    report["status"] = "exact" if all(status == "exact" for status in packet_status) and factor and all(
        item.get("exact", False) for item in factor) else "mismatch_or_pending"
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({
        "status": report["status"],
        "packet": [(x["label"], x["status"], x.get("total_bit_mismatches"))
                   for x in report["first_link_packet_comparisons"]],
        "frame4_first": next((x for x in factor if not x.get("exact", False)), None),
        "report": str(args.report),
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
