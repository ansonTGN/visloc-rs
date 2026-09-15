#!/usr/bin/env python3
"""Compare the opt-in Rust/native covariance assignment-term dumps.

The covariance observer intentionally reports a different (diagnostic) term
schedule.  This comparator therefore consumes only ``*_M7HD_ASSIGN`` records,
which contain the production ``noise * variance * noise.transpose()`` term.
It accepts either the Rust ``RUST_M7HD_ASSIGN`` prefix or a native oracle
prefix such as ``M7HD_NATIVE_ASSIGN``/``M7HD_ZERO_ASSIGN``.  Records are
matched by kind (accel/gyro) and ordinal, so the tool is independent of frame
or fixture IDs.
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any


FIELD_RE = re.compile(r"(?:^| )([A-Za-z_][A-Za-z0-9_]*)=([^ ]+)")
PREFIX_RE = re.compile(r"^(?:RUST_M7HD_ASSIGN|M7HD_(?:NATIVE|ZERO)_ASSIGN)\b")


def fields(line: str) -> dict[str, str]:
    return {name: value for name, value in FIELD_RE.findall(line)}


def words(value: str) -> list[int]:
    return [int(item, 16) for item in value.split(",") if item]


def read_records(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if not PREFIX_RE.match(line):
            continue
        item = fields(line)
        if "term" not in item or "kind" not in item:
            continue
        item["term_words"] = words(item["term"])
        item["noise_words"] = words(item["noise"]) if "noise" in item else None
        item["variance_bits"] = int(item["variance"], 0) if "variance" in item else None
        records.append(item)
    return records


def grouped(records: list[dict[str, Any]]) -> dict[str, list[dict[str, Any]]]:
    result: dict[str, list[dict[str, Any]]] = {}
    for item in records:
        result.setdefault(item["kind"], []).append(item)
    return result


def compare(native: list[dict[str, Any]], rust: list[dict[str, Any]]) -> dict[str, Any]:
    native_by_kind = grouped(native)
    rust_by_kind = grouped(rust)
    kinds = sorted(set(native_by_kind) | set(rust_by_kind))
    output: dict[str, Any] = {"kinds": {}, "native_records": len(native), "rust_records": len(rust)}
    for kind in kinds:
        left = native_by_kind.get(kind, [])
        right = rust_by_kind.get(kind, [])
        rows: list[dict[str, Any]] = []
        for ordinal in range(max(len(left), len(right))):
            n = left[ordinal] if ordinal < len(left) else None
            r = right[ordinal] if ordinal < len(right) else None
            row: dict[str, Any] = {"ordinal": ordinal, "present_native": n is not None, "present_rust": r is not None}
            if n is not None and r is not None:
                nterm = n["term_words"]
                rterm = r["term_words"]
                diffs = [
                    (index, rv, nv)
                    for index, (rv, nv) in enumerate(zip(rterm, nterm))
                    if rv != nv
                ]
                row["term_mismatches"] = len(diffs) + abs(len(rterm) - len(nterm))
                row["term_first"] = (
                    {"lane": lane, "rust": f"{rv:08x}", "native": f"{nv:08x}"}
                    if diffs
                    else None
                )
                if n.get("noise_words") is not None and r.get("noise_words") is not None:
                    nd = n["noise_words"]
                    rd = r["noise_words"]
                    row["noise_mismatches"] = sum(a != b for a, b in zip(rd, nd)) + abs(len(rd) - len(nd))
                if n.get("variance_bits") is not None and r.get("variance_bits") is not None:
                    row["variance_equal"] = n["variance_bits"] == r["variance_bits"]
            else:
                row["term_mismatches"] = None
            rows.append(row)
        output["kinds"][kind] = rows
    output["term_mismatches"] = sum(
        row["term_mismatches"] or 0
        for rows in output["kinds"].values()
        for row in rows
    )
    output["exact"] = output["term_mismatches"] == 0 and all(
        row["present_native"] and row["present_rust"]
        for rows in output["kinds"].values()
        for row in rows
    )
    return output


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--native", type=Path, required=True)
    parser.add_argument("--rust", type=Path, required=True)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    result = compare(read_records(args.native), read_records(args.rust))
    serialized = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.out:
        args.out.write_text(serialized, encoding="utf-8")
    print(serialized, end="")
    return 0 if result["exact"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
