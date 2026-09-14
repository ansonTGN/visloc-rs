#!/usr/bin/env python3
"""Validate the recorded M8 MargData provenance/gate artifact.

This is intentionally a lightweight audit gate.  It does not silently turn a
raw M7 input mismatch into a mapper pass: the raw fixture must be reported as
failed, while the controlled native-pose+factor mapper regression must remain
within its recorded numerical tolerance.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "target" / "m8_full80_parity_manifest_20260824.json"
COMPARISON = ROOT / "target" / "m8_full80_pose_provenance_compare_20260824.json"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=MANIFEST)
    parser.add_argument("--comparison", type=Path, default=COMPARISON)
    parser.add_argument(
        "--require-pass",
        action="store_true",
        help="require the raw input gate to pass (future clean fixture use)",
    )
    args = parser.parse_args()

    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    comparison = json.loads(args.comparison.read_text(encoding="utf-8"))
    provenance = manifest["raw_pose_provenance"]
    gate = manifest["input_pose_parity_gate"]
    controlled = gate["controlled_mapper_regression"]

    recorded_hash = provenance["comparison_artifact_sha256"].lower()
    actual_hash = sha256(args.comparison)
    if actual_hash != recorded_hash:
        raise SystemExit(
            f"comparison artifact hash mismatch: recorded={recorded_hash} actual={actual_hash}"
        )

    alignment = comparison["packet_alignment"]
    mismatch = alignment["first_numeric_field_mismatch"]
    if mismatch["packet_index"] != 0 or mismatch["collection"] != "frame_poses":
        raise SystemExit("unexpected first M8 raw pose mismatch boundary")
    if alignment["first_packet_key_mismatch_index"] != 1:
        raise SystemExit("unexpected M8 packet event-key boundary")

    if comparison["post_add_mapper_pose_map"]["timestamp_key_sets_equal"] is not True:
        raise SystemExit("post-add mapper timestamp keys are not aligned")

    cost_limits = {
        "initial_cost_abs_diff": 1.0e-6,
        "optimize_1_abs_diff": 1.0e-6,
        "optimize_2_abs_diff": 1.0e-6,
    }
    cost_diffs = {
        "initial_cost_abs_diff": controlled["initial_cost_abs_diff"],
        "optimize_1_abs_diff": controlled["optimize_1_abs_diff"],
        "optimize_2_abs_diff": controlled["optimize_2_abs_diff"],
    }
    for name, value in cost_diffs.items():
        if value > cost_limits[name]:
            raise SystemExit(f"controlled mapper regression exceeded {name}: {value}")
    if not controlled["filter_landmarks_equal"] or not controlled["filter_observations_equal"]:
        raise SystemExit("controlled mapper filter cardinalities diverged")

    raw_status = gate["status"]
    if args.require_pass:
        if raw_status != "pass":
            raise SystemExit(f"raw input pose gate is not passing: {raw_status}")
        if alignment["first_numeric_field_mismatch"] is not None:
            raise SystemExit("raw input pose gate claims pass but has a numeric mismatch")
        if alignment["source_frame_pose_entries_exact"] != alignment["source_frame_pose_entries_compared"]:
            raise SystemExit("raw input pose gate claims pass but pose bits differ")
        if alignment["source_frame_state_entries_exact"] != alignment["source_frame_state_entries_compared"]:
            raise SystemExit("raw input pose gate claims pass but state bits differ")
        if any(
            not all(
                row[field]
                for field in (
                    "packet_key_equal",
                    "kfs_all_equal",
                    "kfs_to_marg_equal",
                    "frame_pose_timestamp_set_equal",
                    "frame_state_timestamp_set_equal",
                    "metadata_equal",
                    "aom_order_equal",
                )
            )
            for row in alignment["packet_index_comparison"]
        ):
            raise SystemExit("raw input pose gate claims pass but packet structure differs")
    if not args.require_pass and raw_status != "raw_fixture_fails_inherited_m7_state":
        raise SystemExit(f"unexpected raw input pose gate status: {raw_status}")

    print(
        "M8 pose provenance gate: "
        f"raw={raw_status}; controlled_mapper_regression=PASS; "
        f"first_mismatch={mismatch['collection']}.{mismatch['field']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
