"""Build a hash-bound Phase 6 aggregate from a base plan plus clean replacements."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from benchmarks.basalt import phase6_coordinator as phase6


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"JSON root is not an object: {path}")
    return value


def parse_replacement(value: str) -> tuple[str, Path]:
    sequence, separator, raw_root = value.partition("=")
    if not separator or not sequence or not raw_root:
        raise argparse.ArgumentTypeError("replacement must be SEQUENCE=RUN_ROOT")
    return sequence, Path(raw_root).resolve()


def cell_from_document(
    document: dict[str, Any],
    run_dir: Path,
    base_namespace: dict[str, Any],
) -> phase6.Cell:
    manifest_path = run_dir / "run_manifest.json"
    evaluation_path = run_dir / "evaluation_result.json"
    if not manifest_path.is_file() or not evaluation_path.is_file():
        raise ValueError(f"selected cell is incomplete: {run_dir}")
    manifest = load_json(manifest_path)
    if manifest.get("status") != "success":
        raise ValueError(f"selected cell is not successful: {run_dir}")
    for key in ("method", "sequence", "repetition"):
        if manifest.get(key) != document.get(key):
            raise ValueError(f"selected cell {key} mismatch: {run_dir}")
    if manifest.get("no_marg_data") is not True or manifest.get("no_trace") is not True:
        raise ValueError(f"selected cell is not lean: {run_dir}")
    firewall = manifest.get("ground_truth_firewall", {})
    if firewall.get("strict_gt_absence_verified") is not True:
        raise ValueError(f"selected cell lacks strict GT firewall proof: {run_dir}")

    request = copy.deepcopy(manifest.get("request", {}))
    request["input_namespace"] = copy.deepcopy(base_namespace)
    request["output_policy"] = phase6.OUTPUT_POLICY_LEAN
    request["formal_denominator"] = True
    return phase6.Cell(
        method=str(document["method"]),
        sequence=str(document["sequence"]),
        family=str(document["family"]),
        repetition=int(document["repetition"]),
        run_dir=run_dir,
        source_root=Path(document["source_root"]),
        source_root_resolved=Path(document["source_root_resolved"]),
        command=tuple(document["command"]),
        request=request,
        request_sha256=str(manifest.get("request_sha256", document["request_sha256"])),
        expected_frames=int(document["expected_input_frames"]),
        sensor_fingerprint=document.get("sensor_fingerprint"),
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-plan", type=Path, required=True)
    parser.add_argument("--base-run-root", type=Path, required=True)
    parser.add_argument("--replacement", action="append", type=parse_replacement, default=[])
    parser.add_argument("--protocol", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    plan_path = args.base_plan.resolve()
    base_root = args.base_run_root.resolve()
    protocol_path = args.protocol.resolve()
    output_path = args.output.resolve()
    plan = load_json(plan_path)
    replacements = dict(args.replacement)
    known_sequences = set(plan.get("request", {}).get("sequences", []))
    unknown = sorted(set(replacements) - known_sequences)
    if unknown:
        raise ValueError(f"replacement sequences are not in the base plan: {unknown}")
    base_namespace = plan.get("request", {}).get("input_namespace")
    if not isinstance(base_namespace, dict):
        raise ValueError("base plan has no input namespace")

    cells: list[phase6.Cell] = []
    selection: list[dict[str, Any]] = []
    results: list[dict[str, Any]] = []
    for document in plan.get("cells", []):
        sequence = str(document["sequence"])
        method = str(document["method"])
        repetition = int(document["repetition"])
        root = replacements.get(sequence, base_root)
        run_dir = root / method / sequence / f"r{repetition}"
        cell = cell_from_document(document, run_dir, base_namespace)
        cells.append(cell)
        manifest_path = run_dir / "run_manifest.json"
        evaluation_path = run_dir / "evaluation_result.json"
        selection.append(
            {
                "method": method,
                "sequence": sequence,
                "repetition": repetition,
                "source": "replacement" if sequence in replacements else "base",
                "run_dir": str(run_dir),
                "run_manifest_sha256": sha256(manifest_path),
                "evaluation_result_sha256": sha256(evaluation_path),
            }
        )
        results.append({"status": "success", "run_dir": str(run_dir)})

    selected_plan = copy.deepcopy(plan)
    selected_plan["cells"] = cells
    gate = phase6.aggregate_gate_report(
        selected_plan,
        results,
        protocol_path=protocol_path,
        output_root=output_path.parent,
    )
    artifact = {
        "schema_id": "basalt.phase6.corrected_aggregate.v1",
        "base_plan": {"path": str(plan_path), "sha256": sha256(plan_path)},
        "base_run_root": str(base_root),
        "replacements": {key: str(value) for key, value in sorted(replacements.items())},
        "selection": selection,
        "selection_count": len(selection),
        "gate_report": gate,
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(artifact, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(output_path)
    print(sha256(output_path))
    print(gate.get("overall_status"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
