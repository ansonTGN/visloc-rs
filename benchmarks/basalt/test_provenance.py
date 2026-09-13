"""Regression tests for the Basalt provenance/release audit."""

from pathlib import Path
import subprocess
import sys

from benchmarks.basalt.verify_provenance import (
    validate,
    validate_cargo_license_inventory,
)


ROOT = Path(__file__).resolve().parents[2]


def test_release_provenance_refresh_is_current():
    result = subprocess.run(
        [
            sys.executable,
            str(ROOT / "benchmarks/basalt/refresh_release_provenance.py"),
            "--check",
            "--root",
            str(ROOT),
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_provenance_manifest_and_generator_hashes():
    assert validate(ROOT) == []


def test_release_manifest_contains_no_ground_truth_artifacts():
    release = ROOT / "benchmarks/basalt/basalt_release_manifest_v1.json"
    assert release.is_file()
    import json

    payload = json.loads(release.read_text(encoding="utf-8"))
    assert payload["ground_truth_artifacts"] == []
    assert payload["ground_truth_firewall"]["release_artifacts_contain_ground_truth"] is False


def test_release_manifest_binds_every_file_and_contract_input():
    import hashlib
    import json

    release = ROOT / "benchmarks/basalt/basalt_release_manifest_v1.json"
    payload = json.loads(release.read_text(encoding="utf-8"))
    schema = json.loads(
        (ROOT / "benchmarks/basalt/schemas/release_manifest_v1.schema.json").read_text(
            encoding="utf-8"
        )
    )
    assert schema["$id"].endswith("/release-manifest-v1.schema.json")
    assert set(schema["$defs"]["artifact"]["required"]) == {
        "path",
        "kind",
        "bytes",
        "sha256",
    }
    artifacts = payload["artifacts"]
    assert payload["schema_path"] == "benchmarks/basalt/schemas/release_manifest_v1.schema.json"
    assert payload["self_binding"]["mode"] == "external_sha256"
    assert all({"path", "kind", "bytes", "sha256"} <= set(item) for item in artifacts)
    assert all(item["path"] != "benchmarks/basalt/basalt_release_manifest_v1.json" for item in artifacts)
    expected_contracts = set(payload["contract_inputs"].values())
    listed = {item["path"] for item in artifacts}
    assert expected_contracts <= listed
    assert {
        "benchmarks/basalt/schemas/clonefree_perf_result_v1.schema.json",
        "benchmarks/basalt/run_clonefree_perf.py",
        "benchmarks/basalt/native_wsl_runner.py",
        "benchmarks/basalt/test_clonefree_perf.py",
    } <= listed
    expected_pipeline_tests = {
        path.relative_to(ROOT).as_posix()
        for path in (ROOT / "pipelines/basalt/tests").rglob("*")
        if path.is_file()
    }
    listed_contract_tests = {
        item["path"] for item in artifacts if item["kind"] == "contract_test"
    }
    assert listed_contract_tests == {
        *expected_pipeline_tests,
        "benchmarks/basalt/test_clonefree_perf.py",
    }
    for item in artifacts:
        path = ROOT / item["path"]
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        assert item["bytes"] == path.stat().st_size
        assert item["sha256"] == digest


def test_vcpkg_gitlink_is_index_bound_and_excluded_from_content_inventory():
    import json

    upstream = json.loads(
        (ROOT / "benchmarks/basalt/upstream_manifest.json").read_text(encoding="utf-8")
    )
    record = upstream["repository"]["submodules"]["thirdparty/vcpkg"]
    assert record["gitlink_mode"] == "160000"
    assert record["binding"] == "git_index_oid"
    assert record["content_inventory"] == "excluded_submodule_gitlink"
    assert record["license_spdx"] == "MIT"

    release = json.loads(
        (ROOT / "benchmarks/basalt/basalt_release_manifest_v1.json").read_text(
            encoding="utf-8"
        )
    )
    binding = release["gitlink_bindings"]["thirdparty/vcpkg"]
    assert binding["mode"] == "160000"
    assert binding["oid"] == record["commit"]
    assert binding["inventory"] == "excluded_from_content_artifacts"
    assert "thirdparty/vcpkg" in release["excluded_artifacts"]
    assert "thirdparty/vcpkg/**" in release["excluded_artifacts"]
    assert not any(
        item["path"] == "thirdparty/vcpkg" for item in release["artifacts"]
    )

    verifier = (ROOT / "benchmarks/basalt/verify_manifest.ps1").read_text(
        encoding="utf-8"
    )
    assert "ls-files" in verifier
    assert "gitlink_mode" in verifier
    assert "materialized HEAD" in verifier


def _tampered_cargo_inventory(mutate):
    import json
    import tempfile

    source = ROOT / "benchmarks/basalt/cargo_license_inventory_v2.json"
    payload = json.loads(source.read_text(encoding="utf-8"))
    mutate(payload)
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        suffix=".json",
        prefix="cargo_license_inventory_v2_tampered_",
        dir=ROOT / "work",
        delete=False,
    ) as stream:
        destination = Path(stream.name)
        json.dump(payload, stream)
    try:
        return validate_cargo_license_inventory(ROOT, "v2", destination)
    finally:
        destination.unlink(missing_ok=True)


def test_cargo_license_v2_rejects_crate_checksum_tamper():
    errors = _tampered_cargo_inventory(
        lambda payload: payload["packages"][0]["crate"].update(
            sha256="0" * 64
        ),
    )
    assert any("crate.sha256" in error or "crate.sha256 mismatch" in error for error in errors)


def test_cargo_license_v2_rejects_license_hash_tamper():
    errors = _tampered_cargo_inventory(
        lambda payload: payload["packages"][0]["license_files"][0].update(
            sha256="0" * 64
        ),
    )
    assert any("license_files[0].evidence.sha256" in error for error in errors)


def test_cargo_license_v2_rejects_unresolved_count_tamper():
    errors = _tampered_cargo_inventory(
        lambda payload: payload.update(licenses_unresolved=1),
    )
    assert any("licenses_unresolved must be zero" in error for error in errors)


def test_cargo_license_v2_rejects_overlay_source_tamper():
    errors = _tampered_cargo_inventory(
        lambda payload: payload["packages"][0]["upstream"].update(
            cargo_toml_evidence=(
                "work/m11_cargo_license_network_resolution_20260830_evidence/"
                "cc-1.2.61.api.json"
            )
        ),
    )
    assert any("cargo_toml_evidence" in error for error in errors)


def test_cargo_license_v1_history_remains_compatible():
    assert validate_cargo_license_inventory(ROOT, "v1") == []
