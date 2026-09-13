"""Focused tests for the non-destructive current provenance v2 generator."""

from __future__ import annotations

import hashlib
import json
import os
import sys
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
    executable = Path("target/m11_release_candidate_final2_ms_20260831/release/examples/basalt_euroc_vio_demo.exe")
    # The captured RC build evidence uses the actual existing stderr path.
    build_evidence = Path("work/m11_release_candidate_final2_build_20260831.err")
    candidate = build_candidate(
        ROOT,
        captured_at="2026-08-31T00:00:00+00:00",
        executable=executable,
        build_evidence=build_evidence,
        timing_feature="disabled",
        compiler="rustc 1.94.0; commit=4a4ef493e3a1488c6e321570238084b38948f6db; host=x86_64-pc-windows-msvc; LLVM=21.1.8",
        features=[],
        lm_workspace_reuse="disabled",
        freeze=True,
    )
    assert candidate["status"] == "frozen"
    assert validate_candidate(ROOT, candidate, require_executable=True) == []
    assert {item["max_frames"] for item in candidate["current_binding"]["correctness"]["replays"]} == {
        52,
        80,
        400,
    }
    candidate["current_binding"]["correctness"]["replays"][0]["max_frames"] = 51
    errors = validate_candidate(ROOT, candidate, require_executable=True)
    assert any("52/80/400" in error for error in errors)

    candidate["current_binding"]["correctness"]["replays"][0]["max_frames"] = 52
    candidate["current_binding"]["contracts"]["benchmark_readme"]["sha256"] = "0" * 64
    errors = validate_candidate(ROOT, candidate, require_executable=True)
    assert any("hash mismatch" in error and "README.md" in error for error in errors)

    candidate["current_binding"]["contracts"]["benchmark_readme"]["sha256"] = _sha256(
        ROOT / "benchmarks/basalt/README.md"
    )
    candidate["current_binding"]["license"]["sha256"] = "0" * 64
    errors = validate_candidate(ROOT, candidate, require_executable=True)
    assert any("hash mismatch" in error and "cargo_license_inventory_v2" in error for error in errors)
