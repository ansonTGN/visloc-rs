"""Focused tests for selected-profile license payload staging."""

from __future__ import annotations

import importlib.util
import io
import tarfile
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("bundle_selected_license_payload.py")
SPEC = importlib.util.spec_from_file_location("selected_license_payload", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class SelectedLicensePayloadTests(unittest.TestCase):
    @staticmethod
    def _report_with_supplemental(supplemental: dict | None) -> dict:
        rows = [
            {
                "name": f"demo-{index}",
                "version": "1.0.0",
                "source": f"registry+https://example/{index}",
            }
            for index in range(66)
        ]
        return {
            "status": "pass_selected_graph_reconciled",
            "scope": {
                "target": MODULE.EXPECTED_TARGET,
                "example": MODULE.EXPECTED_EXAMPLE,
                "no_default_features": True,
            },
            "reconciliation": {
                "selected_package_count": 66,
                "selected_registry_package_count": 56,
                "selected_local_package_count": 10,
                "selected_inventory_missing_count": 0,
                "license_text_missing_from_known_local_cache_count": 0,
            },
            "supplemental_source_dependencies": supplemental,
            "selected_packages": rows,
        }

    def test_stale_cargo_only_report_is_rejected(self) -> None:
        report = self._report_with_supplemental(None)
        with self.assertRaisesRegex(MODULE.PayloadError, "missing supplemental"):
            MODULE._validate_selected_report(report)

    def test_pending_supplemental_report_is_rejected(self) -> None:
        report = self._report_with_supplemental(
            {
                "status": "pending_supplemental_license_payload",
                "release_ready": False,
            }
        )
        with self.assertRaisesRegex(MODULE.PayloadError, "not release-ready"):
            MODULE._validate_selected_report(report)

    def test_ready_supplemental_report_is_accepted(self) -> None:
        report = self._report_with_supplemental(
            {
                "status": "pass_supplemental_source_dependency",
                "release_ready": True,
                "manual_source_dependency_count": 1,
                "known_source_hashes_verified_count": 4,
                "known_source_file_count": 4,
                "known_license_hash_verified": True,
                "affected_production_file_hashes_verified_count": 1,
                "affected_production_file_count": 1,
                "payload_persisted": True,
            }
        )
        MODULE._validate_selected_report(report)

    def test_build_stages_current_verified_supplement(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp) / "candidate"
            root = Path(__file__).resolve().parents[2]
            manifest = MODULE.build_payload(
                root=root,
                report_path=root / "work" / "m11_selected_license_graph_20260913.json",
                output_root=output,
                cargo_lock_path=root / "Cargo.lock",
            )
            self.assertTrue((output / "candidate_license_payload_manifest.json").is_file())
            self.assertEqual(manifest["status"], "pass_candidate_selected_license_payload_staged")
            self.assertEqual(manifest["verification"]["package_count"], 66)
            self.assertEqual(manifest["verification"]["missing_license_text_count"], 0)

    def test_rendered_manifest_distinguishes_referenced_aor_from_output_files(self) -> None:
        markdown = MODULE._render_markdown(
            {
                "status": "pass_candidate_selected_license_payload_staged",
                "captured_at": "2026-09-07T00:00:00+09:00",
                "schema": MODULE.SCHEMA,
                "scope": {
                    "target": MODULE.EXPECTED_TARGET,
                    "example": MODULE.EXPECTED_EXAMPLE,
                },
                "verification": {
                    "package_count": 66,
                    "registry_package_count": 56,
                    "local_package_count": 10,
                    "payload_file_count": 1,
                    "source_destination_hash_mismatch_count": 0,
                    "cargo_lock_identity_verified_count": 66,
                    "cargo_checksum_verified_count": 0,
                    "crate_archive_verified_count": 0,
                    "crate_archive_unavailable_count": 66,
                    "cargo_checksum_unavailable_count": 66,
                },
                "supplemental_source_dependencies": {
                    "manual_source_dependency_count": 1,
                    "status": "pass_supplemental_source_dependency",
                    "release_ready": True,
                    "known_source_hashes_verified_count": 4,
                    "known_source_file_count": 4,
                    "affected_production_file_hashes_verified_count": 1,
                    "affected_production_file_count": 1,
                    "known_license_hash_verified": True,
                    "payload_persisted": True,
                    "payload_file_verified": True,
                },
                "supplemental_source_coverage": {
                    "license_payload_in_output_files": False,
                },
                "files": [],
            }
        )
        self.assertIn("referenced/verified: **true**", markdown)
        self.assertIn("included in this candidate's `files`: **false**", markdown)
        self.assertIn("not a self-contained combined release", markdown)

    def test_lock_parser_preserves_registry_identity_and_checksum(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "Cargo.lock"
            path.write_text(
                """version = 4\n\n[[package]]\nname = \"demo\"\nversion = \"1.0.0\"\nsource = \"registry+https://github.com/rust-lang/crates.io-index\"\nchecksum = \"abc\"\n""",
                encoding="utf-8",
            )
            records = MODULE._lock_packages(path)
        self.assertEqual(records[("demo", "1.0.0", "registry+https://github.com/rust-lang/crates.io-index")]["checksum"], "abc")

    def test_registry_identity_and_report_hash_are_verified_without_archive(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            base = Path(temp)
            registry_root = base / ".cargo" / "registry" / "src" / "index.local"
            package_dir = registry_root / "demo-1.0.0"
            package_dir.mkdir(parents=True)
            manifest = package_dir / "Cargo.toml"
            manifest.write_text('[package]\nname = "demo"\nversion = "1.0.0"\n', encoding="utf-8")
            license_path = package_dir / "LICENSE-MIT"
            license_path.write_text("demo license\n", encoding="utf-8")
            license_hash = MODULE.sha256_file(license_path)
            row = {
                "name": "demo",
                "version": "1.0.0",
                "source": "registry+https://github.com/rust-lang/crates.io-index",
                "kind": "registry",
                "license": {
                    "license_expression": "MIT",
                    "inventory_basis": "v1_base",
                    "inventory_status": "v1_base_resolved",
                    "registry_cache_source_dir": str(package_dir),
                    "registry_cache_license_files": [
                        {"path": str(license_path), "bytes": license_path.stat().st_size, "sha256": license_hash}
                    ],
                },
            }
            lock = {
                ("demo", "1.0.0", "registry+https://github.com/rust-lang/crates.io-index"): {
                    "name": "demo",
                    "version": "1.0.0",
                    "source": "registry+https://github.com/rust-lang/crates.io-index",
                    "checksum": "abc",
                }
            }
            package, destinations = MODULE._verify_registry_package(row, lock, [registry_root], base, {})
        self.assertEqual(package["cargo_lock"]["identity_verified"], True)
        self.assertEqual(package["cargo_checksum_sidecar"]["status"], "unavailable")
        self.assertEqual(package["crate_archive"]["status"], "unavailable")
        self.assertEqual(len(destinations), 1)

    def test_missing_license_evidence_fails_closed(self) -> None:
        row = {
            "name": "demo",
            "version": "1.0.0",
            "license": {"registry_cache_source_dir": "C:/cache/demo-1.0.0", "registry_cache_license_files": []},
        }
        with self.assertRaises(MODULE.PayloadError):
            MODULE._registry_license_sources(row, Path.cwd(), Path("C:/cache/demo-1.0.0"))

    def test_copy_spec_round_trips_source_hash(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            base = Path(temp)
            source = base / "LICENSE"
            source.write_bytes(b"payload\n")
            spec = MODULE._new_file_spec(
                source=source,
                destination="licenses/demo/LICENSE",
                source_package="demo",
                source_version="1.0.0",
                spdx_expression="MIT",
                role="registry_license_text",
                package_identity="demo 1.0.0",
                expected_bytes=source.stat().st_size,
                expected_sha256=MODULE.sha256_file(source),
                source_kind="registry_cache_evidence",
            )
            output = base / "out"
            MODULE._copy_specs(output, [spec])
            destination = output / "licenses" / "demo" / "LICENSE"
            self.assertEqual(MODULE.sha256_file(destination), spec["source_sha256"])
            self.assertTrue(spec["source_destination_hash_match"])

    def test_ambiguous_archive_evidence_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            first = Path(temp) / "one.crate"
            second = Path(temp) / "two.crate"
            first.write_bytes(b"same")
            second.write_bytes(b"same")
            with self.assertRaises(MODULE.PayloadError):
                MODULE._verify_archive([first, second], MODULE.sha256_file(first))

    def test_tampered_cache_license_does_not_match_archive_member(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            base = Path(temp)
            source_dir = base / "demo-1.0.0"
            source_dir.mkdir()
            cached_license = source_dir / "LICENSE-MIT"
            cached_license.write_bytes(b"tampered cache text\n")
            archive = base / "demo-1.0.0.crate"
            with tarfile.open(archive, "w:gz") as tar:
                data = b"archive text\n"
                info = tarfile.TarInfo("demo-1.0.0/LICENSE-MIT")
                info.size = len(data)
                tar.addfile(info, io.BytesIO(data))
            license_record = {
                "source_path": str(cached_license),
                "source_bytes": cached_license.stat().st_size,
                "source_sha256": MODULE.sha256_file(cached_license),
                "relative_path": "LICENSE-MIT",
            }
            with self.assertRaises(MODULE.PayloadError):
                MODULE._verify_archive([archive], MODULE.sha256_file(archive), [license_record], source_dir)

    def test_archive_traversal_and_duplicate_members_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            base = Path(temp)
            source_dir = base / "demo-1.0.0"
            source_dir.mkdir()
            cached_license = source_dir / "LICENSE-MIT"
            cached_license.write_bytes(b"license\n")
            traversal = base / "traversal.crate"
            with tarfile.open(traversal, "w:gz") as tar:
                data = b"bad\n"
                info = tarfile.TarInfo("../LICENSE-MIT")
                info.size = len(data)
                tar.addfile(info, io.BytesIO(data))
            record = {
                "source_path": str(cached_license),
                "source_bytes": cached_license.stat().st_size,
                "source_sha256": MODULE.sha256_file(cached_license),
                "relative_path": "LICENSE-MIT",
            }
            with self.assertRaises(MODULE.PayloadError):
                MODULE._verify_archive([traversal], MODULE.sha256_file(traversal), [record], source_dir)

            duplicate = base / "duplicate.crate"
            with tarfile.open(duplicate, "w:gz") as tar:
                data = b"license\n"
                for _ in range(2):
                    info = tarfile.TarInfo("demo-1.0.0/LICENSE-MIT")
                    info.size = len(data)
                    tar.addfile(info, io.BytesIO(data))
            with self.assertRaises(MODULE.PayloadError):
                MODULE._verify_archive([duplicate], MODULE.sha256_file(duplicate), [record], source_dir)


if __name__ == "__main__":
    unittest.main()
