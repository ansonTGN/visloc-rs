"""Focused tests for non-Cargo supplemental source coverage."""

from __future__ import annotations

import importlib.util
import json
import shutil
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("verify_supplemental_source_dependencies.py")
SPEC = importlib.util.spec_from_file_location("supplemental_source_dependencies", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)

ROOT = Path(__file__).resolve().parents[2]
SOURCE_MANIFEST = ROOT / "benchmarks" / "basalt" / "supplemental_source_dependencies_v1.json"
AFFECTED_SOURCE = ROOT / "pipelines" / "basalt" / "src" / "update.rs"


class SupplementalSourceDependencyTests(unittest.TestCase):
    def _temporary_fixture(self, *, payload_status: str | None = None) -> tuple[tempfile.TemporaryDirectory[str], Path, dict]:
        holder = tempfile.TemporaryDirectory()
        base = Path(holder.name)
        manifest_path = base / "supplemental.json"
        manifest = json.loads(SOURCE_MANIFEST.read_text(encoding="utf-8"))
        if payload_status is not None:
            evidence = manifest["dependencies"][0]["license_evidence"]
            evidence["payload_status"] = payload_status
            for key in ("payload_path", "payload_bytes", "payload_sha256"):
                evidence.pop(key, None)
        manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
        affected = base / "pipelines" / "basalt" / "src" / "update.rs"
        affected.parent.mkdir(parents=True)
        shutil.copyfile(AFFECTED_SOURCE, affected)
        return holder, base, manifest

    def test_current_candidate_has_verified_persisted_payload(self) -> None:
        result = MODULE.validate_manifest(ROOT)
        self.assertEqual(result["status"], MODULE.PASS_STATUS)
        self.assertTrue(result["release_ready"])
        self.assertTrue(result["payload_persisted"])
        self.assertTrue(result["payload_file_verified"])
        self.assertEqual(result["known_source_hashes_verified_count"], 4)
        self.assertEqual(result["known_source_file_count"], 4)
        self.assertTrue(result["known_license_hash_verified"])
        self.assertEqual(result["affected_production_file_hashes_verified_count"], 1)
        self.assertEqual(result["errors"], [])

    def test_current_manifest_binds_sinf_to_production_se2_path(self) -> None:
        manifest = json.loads(SOURCE_MANIFEST.read_text(encoding="utf-8"))
        dependency = manifest["dependencies"][0]
        self.assertEqual(
            dependency["covered_functions"]["sinf"],
            MODULE.EXPECTED_PRODUCTION_SCOPE,
        )
        affected = dependency["affected_production_files"][0]
        self.assertEqual(affected["function_scope"]["sinf"], "production")

    def test_test_only_sinf_scope_is_rejected(self) -> None:
        holder, base, manifest = self._temporary_fixture()
        try:
            dependency = manifest["dependencies"][0]
            dependency["covered_functions"]["sinf"] = "test-only"
            dependency["affected_production_files"][0]["function_scope"]["sinf"] = (
                "test_only_until_production_promotion"
            )
            manifest_path = base / "supplemental.json"
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            result = MODULE.validate_manifest(base, manifest_path)
        finally:
            holder.cleanup()
        self.assertEqual(result["status"], MODULE.FAIL_STATUS)
        self.assertFalse(result["release_ready"])
        self.assertTrue(any("covered_functions.sinf" in item for item in result["errors"]))
        self.assertTrue(any("function_scope.sinf" in item for item in result["errors"]))

    def test_known_upstream_hash_tamper_fails(self) -> None:
        holder, base, manifest = self._temporary_fixture()
        try:
            manifest["dependencies"][0]["source_hashes"][0]["sha256"] = "0" * 64
            manifest_path = base / "supplemental.json"
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            result = MODULE.validate_manifest(base, manifest_path)
        finally:
            holder.cleanup()
        self.assertEqual(result["status"], MODULE.FAIL_STATUS)
        self.assertTrue(any("known immutable hash" in item for item in result["errors"]))

    def test_affected_production_file_tamper_fails(self) -> None:
        holder, base, _ = self._temporary_fixture()
        try:
            affected = base / "pipelines" / "basalt" / "src" / "update.rs"
            affected.write_bytes(affected.read_bytes() + b"\n")
            result = MODULE.validate_manifest(base, base / "supplemental.json")
        finally:
            holder.cleanup()
        self.assertEqual(result["status"], MODULE.FAIL_STATUS)
        self.assertTrue(any("affected_production_files[0] SHA-256 mismatch" in item for item in result["errors"]))

    def test_forged_persisted_status_without_payload_fails_closed(self) -> None:
        holder, base, _ = self._temporary_fixture(payload_status="RELEASE_PAYLOAD_PERSISTED")
        try:
            result = MODULE.validate_manifest(base, base / "supplemental.json")
        finally:
            holder.cleanup()
        self.assertEqual(result["status"], MODULE.FAIL_STATUS)
        self.assertFalse(result["release_ready"])
        self.assertTrue(any("payload_path" in item for item in result["errors"]))

    def test_tampered_persisted_payload_fails_closed(self) -> None:
        holder, base, manifest = self._temporary_fixture(payload_status="RELEASE_PAYLOAD_PERSISTED")
        try:
            payload = base / "payload" / "LICENSE"
            payload.parent.mkdir()
            payload.write_bytes(b"tampered license text\n")
            evidence = manifest["dependencies"][0]["license_evidence"]
            evidence["payload_path"] = "payload/LICENSE"
            # A self-consistent but wrong file/hash pair must not promote the
            # metadata: the payload is bound to the immutable known LICENSE
            # bytes/hash, not merely to its own declarations.
            evidence["payload_bytes"] = payload.stat().st_size
            evidence["payload_sha256"] = MODULE.sha256_file(payload)
            manifest_path = base / "supplemental.json"
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            result = MODULE.validate_manifest(base, manifest_path)
        finally:
            holder.cleanup()
        self.assertEqual(result["status"], MODULE.FAIL_STATUS)
        self.assertFalse(result["release_ready"])
        self.assertTrue(any("payload_bytes does not match" in item for item in result["errors"]))
        self.assertTrue(any("payload_sha256 does not match" in item for item in result["errors"]))
        self.assertTrue(any("license_evidence.payload SHA-256 mismatch" in item for item in result["errors"]))


if __name__ == "__main__":
    unittest.main()
