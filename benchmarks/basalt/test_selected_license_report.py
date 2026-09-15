"""Small pure-function tests for the selected Cargo license report."""

from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("generate_selected_license_report.py")
SPEC = importlib.util.spec_from_file_location("selected_license_report", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class SelectedLicenseReportTests(unittest.TestCase):
    def test_reachable_graph_includes_dev_and_build_edges(self) -> None:
        nodes = {
            "root": {
                "dependencies": [
                    {"pkg": "runtime", "dep_kinds": [{"kind": None}]},
                    {"pkg": "dev", "dep_kinds": [{"kind": "dev"}]},
                ]
            },
            "runtime": {"dependencies": [{"pkg": "build", "dep_kinds": [{"kind": "build"}]}]},
            "dev": {"dependencies": []},
            "build": {"dependencies": []},
            "disconnected": {"dependencies": []},
        }
        self.assertEqual(
            MODULE.reachable_package_ids(nodes, "root"),
            {"root", "runtime", "dev", "build"},
        )

    def test_overlay_payload_marks_upstream_only_license(self) -> None:
        overlay = {
            "license_files": [
                {
                    "path": "LICENSE-MIT",
                    "evidence": "missing/LICENSE-MIT",
                    "packaged": False,
                }
            ]
        }
        files, status = MODULE._overlay_payload(overlay, Path.cwd())
        self.assertEqual(status, "v2_license_evidence_path_missing")
        self.assertFalse(files[0]["evidence_exists"])

    def test_path_key_distinguishes_sources(self) -> None:
        a = {"name": "demo", "version": "1.0.0", "source": "registry+https://example"}
        b = {"name": "demo", "version": "1.0.0", "source": "git+https://example"}
        self.assertNotEqual(MODULE._path_key(a), MODULE._path_key(b))

    def test_workspace_metadata_matches_local_inventory_row(self) -> None:
        package = {"name": "visloc-rs", "version": "0.1.0", "source": None}
        base = {("visloc-rs", "0.1.0", "local-workspace"): {"license": "MIT OR Apache-2.0"}}
        key, row, overlay = MODULE._find_inventory_match(package, base, {})
        self.assertEqual(key, ("visloc-rs", "0.1.0", "local-workspace"))
        self.assertEqual(row["license"], "MIT OR Apache-2.0")
        self.assertIsNone(overlay)

    def test_overall_status_keeps_pending_supplemental_coverage_nonrelease(self) -> None:
        self.assertEqual(
            MODULE._overall_report_status(True, "pending_supplemental_license_payload"),
            "pending_supplemental_license_payload",
        )
        self.assertEqual(
            MODULE._overall_report_status(False, "pass_supplemental_source_dependency"),
            "fail_graph_reconciliation",
        )
        self.assertEqual(
            MODULE._overall_report_status(True, "pass_supplemental_source_dependency"),
            "pass_selected_graph_reconciled",
        )


if __name__ == "__main__":
    unittest.main()
