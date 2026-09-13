"""Focused tests for the non-destructive current provenance v2 generator."""

from __future__ import annotations

import hashlib
import json
import os
import shutil
import sys
import tempfile
from pathlib import Path

import pytest

from benchmarks.basalt.generate_provenance_manifest_v2 import (
    ROOT,
    ManifestError,
    build_candidate,
    validate_candidate,
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _current_test_certificate(
    executable: Path,
    directory: Path,
    *,
    features: list[str] | None = None,
    timing_feature: str = "disabled",
    lm_workspace_reuse: str = "disabled",
) -> Path:
    """Create a synthetic certificate bound to current bytes for contract tests."""

    template = json.loads(
        (
            ROOT
            / "benchmarks/basalt/release_inputs/"
            "m11_release_candidate_final2_correctness_20260831.json"
        ).read_text(encoding="utf-8")
    )
    template["scope"]["features"] = sorted(features or [])
    template["scope"]["timing_breakdown"] = f"compile-time feature {timing_feature}"
    template["scope"]["lm_workspace_reuse"] = f"feature {lm_workspace_reuse}"
    pending = build_candidate(ROOT, captured_at="2026-09-13T00:00:00+00:00")
    current = pending["current_binding"]
    executable_relative = executable.relative_to(ROOT).as_posix()
    template["build"]["executable"] = {
        "path": executable_relative,
        "bytes": executable.stat().st_size,
        "sha256": _sha256(executable),
    }
    template["source_binding"]["files"] = [
        {
            "path": record["path"],
            "bytes": record["bytes"],
            "sha256": record["sha256"],
        }
        for record in current["source"]["files"]
    ]
    contracts = current["contracts"]
    for name in ("config", "calibration", "dataset_manifest"):
        template["input_binding"][name] = {
            key: contracts[name][key] for key in ("path", "bytes", "sha256")
        }
    template["protocol_binding"].update(
        {key: contracts["protocol"][key] for key in ("path", "bytes", "sha256")}
    )
    path = directory / "current_correctness.json"
    path.write_text(json.dumps(template, indent=2) + "\n", encoding="utf-8", newline="\n")
    return path


def test_v2_candidate_binds_current_sources_and_preserves_v1():
    v1 = ROOT / "benchmarks/basalt/basalt_provenance_manifest_v1.json"
    before = (v1.read_bytes(), _sha256(v1))
    candidate = build_candidate(ROOT, captured_at="2026-08-31T00:00:00+00:00")

    assert validate_candidate(ROOT, candidate, require_executable=False) == []
    assert candidate["status"] == "current_candidate_not_frozen"
    assert candidate["supersedes"]["path"] == "benchmarks/basalt/basalt_provenance_manifest_v1.json"
    assert candidate["supersedes"]["immutable"] is True
    assert candidate["current_binding"]["source"]["file_count"] == 33
    assert candidate["current_binding"]["tests"]["file_count"] > 0
    assert candidate["current_binding"]["build"]["status"] == "pending_executable"
    assert candidate["current_binding"]["license"]["package_count"] == 119
    assert (v1.read_bytes(), _sha256(v1)) == before


def test_v2_executable_binding_requires_explicit_feature_and_hashes():
    relative = Path("work") / f"v2_provenance_test_{os.getpid()}.exe"
    # Keep the temporary evidence inside the repository so the same safe
    # repository-relative path checks used in production are exercised.
    path = ROOT / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b"v2 executable test evidence")
    try:
        with pytest.raises(ManifestError, match="timing-feature"):
            build_candidate(ROOT, executable=relative)
        candidate = build_candidate(
            ROOT,
            executable=relative,
            timing_feature="disabled",
            captured_at="2026-08-31T00:00:00+00:00",
        )
        assert validate_candidate(ROOT, candidate, require_executable=True) == []
        assert candidate["current_binding"]["build"]["status"] == "bound"
        assert candidate["current_binding"]["build"]["executable"]["sha256"] == _sha256(path)
        candidate["current_binding"]["build"]["executable"]["sha256"] = "0" * 64
        assert any("hash mismatch" in error for error in validate_candidate(ROOT, candidate, require_executable=True))
    finally:
        path.unlink(missing_ok=True)


def test_v2_cli_candidate_is_new_file_and_pending_verify_is_explicit():
    output = ROOT / "work" / f"v2_provenance_cli_candidate_{os.getpid()}.json"
    command = [
        sys.executable,
        str(ROOT / "benchmarks/basalt/generate_provenance_manifest_v2.py"),
        "--root",
        str(ROOT),
        "--output",
        str(output),
        "--captured-at",
        "2026-08-31T00:00:00+00:00",
        "--verify",
        "--allow-pending",
    ]
    import subprocess

    try:
        completed = subprocess.run(command, check=False, capture_output=True, text=True)
        assert completed.returncode == 0, completed.stderr
        payload = json.loads(output.read_text(encoding="utf-8"))
        assert payload["status"] == "current_candidate_not_frozen"
        assert payload["current_binding"]["build"]["executable"] is None
        # The generator is intentionally non-overwriting, so a second write to
        # the same candidate path fails rather than silently changing a provenance file.
        completed = subprocess.run(command, check=False, capture_output=True, text=True)
        assert completed.returncode != 0
        assert "refusing to overwrite" in completed.stderr
    finally:
        output.unlink(missing_ok=True)


def test_v2_frozen_rc_binds_exactness_certificate_and_rejects_tamper():
    target_root = ROOT / "target"
    target_root.mkdir(parents=True, exist_ok=True)
    directory = Path(tempfile.mkdtemp(prefix="provenance_v2_test_", dir=target_root))
    try:
        executable_path = directory / "basalt_test_executable.bin"
        executable_path.write_bytes(b"synthetic executable binding")
        build_evidence_path = directory / "build.txt"
        build_evidence_path.write_text("synthetic build evidence\n", encoding="utf-8")
        correctness_path = _current_test_certificate(executable_path, directory)
        executable = executable_path.relative_to(ROOT)
        build_evidence = build_evidence_path.relative_to(ROOT)
        correctness = correctness_path.relative_to(ROOT)
        candidate = build_candidate(
            ROOT,
            captured_at="2026-09-13T00:00:00+00:00",
            executable=executable,
            build_evidence=build_evidence,
            correctness_artifact=correctness,
            timing_feature="disabled",
            compiler="rustc test fixture; host=x86_64-pc-windows-msvc",
            features=[],
            lm_workspace_reuse="disabled",
            freeze=True,
        )
        assert candidate["status"] == "frozen"
        assert validate_candidate(ROOT, candidate, require_executable=True) == []
        assert {
            item["max_frames"]
            for item in candidate["current_binding"]["correctness"]["replays"]
        } == {52, 80, 400}
        candidate["current_binding"]["correctness"]["replays"][0]["max_frames"] = 51
        errors = validate_candidate(ROOT, candidate, require_executable=True)
        assert any("52/80/400" in error for error in errors)

        candidate["current_binding"]["correctness"]["replays"][0]["max_frames"] = 52
        candidate["current_binding"]["correctness"]["certificate_warnings"] = ["stale"]
        errors = validate_candidate(ROOT, candidate, require_executable=True)
        assert any("current sources and inputs" in error for error in errors)
        candidate["current_binding"]["correctness"]["certificate_warnings"] = []

        candidate["current_binding"]["contracts"]["benchmark_readme"]["sha256"] = "0" * 64
        errors = validate_candidate(ROOT, candidate, require_executable=True)
        assert any("hash mismatch" in error and "README.md" in error for error in errors)

        candidate["current_binding"]["contracts"]["benchmark_readme"]["sha256"] = _sha256(
            ROOT / "benchmarks/basalt/README.md"
        )
        candidate["current_binding"]["license"]["sha256"] = "0" * 64
        errors = validate_candidate(ROOT, candidate, require_executable=True)
        assert any(
            "hash mismatch" in error and "cargo_license_inventory_v2" in error
            for error in errors
        )
    finally:
        shutil.rmtree(directory)


def test_v2_frozen_rc_accepts_workspace_reuse_when_certificate_matches():
    target_root = ROOT / "target"
    target_root.mkdir(parents=True, exist_ok=True)
    directory = Path(tempfile.mkdtemp(prefix="provenance_v2_reuse_test_", dir=target_root))
    try:
        executable_path = directory / "basalt_test_executable.bin"
        executable_path.write_bytes(b"synthetic workspace-reuse executable binding")
        correctness_path = _current_test_certificate(
            executable_path,
            directory,
            features=["basalt-lm-workspace-reuse"],
            lm_workspace_reuse="enabled",
        )
        candidate = build_candidate(
            ROOT,
            captured_at="2026-09-13T00:00:00+00:00",
            executable=executable_path.relative_to(ROOT),
            correctness_artifact=correctness_path.relative_to(ROOT),
            timing_feature="disabled",
            compiler="rustc test fixture; host=x86_64-pc-windows-msvc",
            features=["basalt-lm-workspace-reuse"],
            lm_workspace_reuse="enabled",
            freeze=True,
        )
        assert validate_candidate(ROOT, candidate, require_executable=True) == []
        assert candidate["current_binding"]["build"]["lm_workspace_reuse"] == "enabled"
        assert candidate["current_binding"]["correctness"]["scope"] == {
            "features": ["basalt-lm-workspace-reuse"],
            "timing_breakdown": "compile-time feature disabled",
            "lm_workspace_reuse": "feature enabled",
            "performance_evaluation": False,
        }
    finally:
        shutil.rmtree(directory)
