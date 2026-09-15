#!/usr/bin/env python3
"""Compare a native ABS_QR dense pipeline capture with a Rust prefix trace.

The native capture stores a 3*u64 header followed by column-major f32 H and b.
The Rust JSONL stores the same values as hexadecimal IEEE-754 bit patterns.
This verifier deliberately compares bits, not parsed floating-point values.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import struct
from pathlib import Path
from typing import Any


STAGES = ("visual_total", "imu_total", "prior_before", "prior_after", "final")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_native_stage(path: Path) -> tuple[int, int, list[str], list[str]]:
    payload = path.read_bytes()
    if len(payload) < 24:
        raise ValueError(f"native stage is shorter than its header: {path}")
    rows, cols, rhs_size = struct.unpack_from("<QQQ", payload)
    word_count = rows * cols + rhs_size
    expected_size = 24 + 4 * word_count
    if len(payload) != expected_size:
        raise ValueError(
            f"native stage size mismatch: {path}: {len(payload)} != {expected_size}"
        )
    words = struct.unpack_from(f"<{word_count}I", payload, 24)
    h_size = rows * cols
    return rows, cols, [f"{x:08x}" for x in words[:h_size]], [
        f"{x:08x}" for x in words[h_size:]
    ]


def read_rust_event(path: Path, frame_id: int, iteration: int) -> tuple[dict[str, Any], dict[str, Any]]:
    header: dict[str, Any] | None = None
    stages: dict[str, Any] = {}
    event_id: int | None = None
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            record = json.loads(line)
            if (
                record.get("record") == "header"
                and record.get("frame_id") == frame_id
                and record.get("iteration") == iteration
                and record.get("trial") == 0
            ):
                if header is not None:
                    raise ValueError(
                        f"multiple matching Rust headers at {path}:{line_number}"
                    )
                header = record
                event_id = record.get("event_id")
                continue
            if header is None or record.get("event_id") != event_id:
                continue
            stage = record.get("stage")
            if stage in STAGES:
                if stage in stages:
                    raise ValueError(f"duplicate Rust stage {stage!r} at {path}:{line_number}")
                stages[stage] = record
    if header is None:
        raise ValueError(
            f"no Rust event for frame_id={frame_id}, iteration={iteration}, trial=0"
        )
    missing = [stage for stage in STAGES if stage not in stages]
    if missing:
        raise ValueError(f"Rust event is missing stages: {', '.join(missing)}")
    return header, stages


def compare_words(native: list[str], rust: list[str]) -> dict[str, Any]:
    if len(native) != len(rust):
        return {
            "native_count": len(native),
            "rust_count": len(rust),
            "exact_count": 0,
            "mismatch_count": max(len(native), len(rust)),
            "first_mismatch": {"reason": "length_mismatch"},
        }
    mismatches = [index for index, pair in enumerate(zip(native, rust)) if pair[0] != pair[1]]
    first = None
    if mismatches:
        index = mismatches[0]
        first = {"index": index, "native_bits": native[index], "rust_bits": rust[index]}
    return {
        "native_count": len(native),
        "rust_count": len(rust),
        "exact_count": len(native) - len(mismatches),
        "mismatch_count": len(mismatches),
        "first_mismatch": first,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--native-root", type=Path, required=True)
    parser.add_argument("--native-binding", type=Path)
    parser.add_argument("--rust-jsonl", type=Path, required=True)
    parser.add_argument("--rust-executable", type=Path)
    parser.add_argument("--frame-id", type=int, default=4)
    parser.add_argument("--iteration", type=int, default=0)
    parser.add_argument("--out-json", type=Path, required=True)
    parser.add_argument("--out-md", type=Path, required=True)
    args = parser.parse_args()

    native_root = args.native_root.resolve()
    native_stage_root = (
        native_root / "pipeline" if (native_root / "pipeline").is_dir() else native_root
    )
    rust_jsonl = args.rust_jsonl.resolve()
    header, rust_stages = read_rust_event(rust_jsonl, args.frame_id, args.iteration)
    comparisons: list[dict[str, Any]] = []
    for stage in STAGES:
        native_path = native_stage_root / f"{stage}.bin"
        rows, cols, native_h, native_b = read_native_stage(native_path)
        rust_record = rust_stages[stage]
        rust_h = rust_record["global_h"]
        rust_b = rust_record["global_b"]
        if (rows, cols) != (rust_h["rows"], rust_h["cols"]):
            raise ValueError(f"H shape mismatch at {stage}")
        if len(native_b) != rust_b["rows"] or rust_b["cols"] != 1:
            raise ValueError(f"b shape mismatch at {stage}")
        comparisons.append(
            {
                "stage": stage,
                "native_bin": str(native_path),
                "native_bin_sha256": sha256(native_path),
                "h": compare_words(native_h, rust_h["bits"]),
                "b": compare_words(native_b, rust_b["bits"]),
            }
        )

    passed = all(
        comparison[component]["mismatch_count"] == 0
        for comparison in comparisons
        for component in ("h", "b")
    )
    native_final_metadata = native_stage_root / "final.json"
    artifact: dict[str, Any] = {
        "schema": "visloc-rs.basalt.absqr-dense-pipeline-compare.v1",
        "status": "PASS" if passed else "FAIL",
        "comparison": "native and Rust IEEE-754 f32 bits; column-major H then b",
        "event": {
            "frame_id": args.frame_id,
            "iteration": args.iteration,
            "trial": 0,
            "rust_event_id": header["event_id"],
            "state_dof": header["state_dof"],
            "visual_factor_count": header["visual_factor_count"],
            "factor_order_fingerprint": header["factor_order_fingerprint"],
        },
        "native": {
            "root": str(native_root),
            "stage_root": str(native_stage_root),
            "run_uuid": (
                json.loads(native_final_metadata.read_text(encoding="utf-8")).get("run_uuid")
                if native_final_metadata.is_file()
                else None
            ),
        },
        "rust": {
            "jsonl": str(rust_jsonl),
            "jsonl_sha256": sha256(rust_jsonl),
        },
        "stages": comparisons,
    }
    if args.native_binding:
        binding = args.native_binding.resolve()
        artifact["native"]["binding"] = str(binding)
        artifact["native"]["binding_sha256"] = sha256(binding)
    if args.rust_executable:
        executable = args.rust_executable.resolve()
        artifact["rust"]["executable"] = str(executable)
        artifact["rust"]["executable_bytes"] = executable.stat().st_size
        artifact["rust"]["executable_sha256"] = sha256(executable)

    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_md.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(artifact, indent=2) + "\n", encoding="utf-8")
    rows = [
        "# ABS_QR dense pipeline comparison",
        "",
        f"Status: **{artifact['status']}**",
        "",
        f"Event: frame {args.frame_id}, iteration {args.iteration}, trial 0; "
        f"state DOF {header['state_dof']}; visual factors {header['visual_factor_count']}.",
        "",
        "| Stage | H exact | b exact |",
        "|---|---:|---:|",
    ]
    for comparison in comparisons:
        h = comparison["h"]
        b = comparison["b"]
        rows.append(
            f"| {comparison['stage']} | {h['exact_count']}/{h['native_count']} | "
            f"{b['exact_count']}/{b['native_count']} |"
        )
    rows.extend(
        [
            "",
            "The verifier compares the serialized f32 bit patterns directly; no decimal parsing or tolerance is used.",
            "",
        ]
    )
    args.out_md.write_text("\n".join(rows), encoding="utf-8")
    print(json.dumps({"status": artifact["status"], "stages": comparisons}, indent=2))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
