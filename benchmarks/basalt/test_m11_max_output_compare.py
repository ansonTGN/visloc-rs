"""Negative and contract tests for the bounded 52/80 output comparator."""

from __future__ import annotations

import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = ROOT / "work" / "m11_max52_output_compare_20260907.py"
SPEC = importlib.util.spec_from_file_location("m11_max_output_compare", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _summary(frame_count: int) -> str:
    return (
        "sensor_only=true\n"
        f"frames_requested={frame_count}\n"
        f"frames_processed={frame_count}\n"
        "observations_emitted=3\n"
        "imu_samples_delivered=4\n"
    )


def _packet(value: float = 1.0, schema_version: int = 4) -> dict:
    return {
        "schema_version": schema_version,
        "provenance_version": "basalt-0f3b2b52-mapper-packet-v1",
        "fej_complete": True,
        "used_imu": True,
        "kfs_all": [0, 7],
        "kfs_to_marg": [0],
        "frame_poses": [
            {"frame_id": index, "pose": [value + index, 0.0, 0.0]}
            for index in range(7)
        ],
        "frame_poses_fej": {
            str(index): {"pose": [value + index, 0.0, 0.0]}
            for index in range(7)
        },
        "frame_states": [
            {"frame_id": index, "state": [value + index, 0.0]}
            for index in range(3)
        ],
        "frame_states_fej": {
            str(index): {"state": [value + index, 0.0]}
            for index in range(3)
        },
        "aom_order": [
            {"frame_id": index, "offset": index, "dof": 1, "kind": "pose"}
            for index in range(9)
        ],
        "aom_abs_h": {"rows": 72, "cols": 72, "data": [value] * (72 * 72)},
        "aom_abs_b": [value] * 72,
    }


def _write_full(root: Path, frame_count: int = 52, packet: dict | None = None) -> None:
    root.mkdir(parents=True)
    (root / "summary.txt").write_text(_summary(frame_count), encoding="utf-8")
    (root / "trajectory.csv").write_text("0,0,0\n", encoding="utf-8")
    (root / "trajectory.tum").write_text("0 0 0 0 0 0 0 1\n", encoding="utf-8")
    (root / "trace.jsonl").write_text('{"frame":0}\n', encoding="utf-8")
    marg = root / "marg_data"
    marg.mkdir()
    final_packet = marg / f"frame_{frame_count - 1:06d}.json"
    final_packet.write_text(
        json.dumps(packet if packet is not None else _packet(), sort_keys=True),
        encoding="utf-8",
    )


def _write_lean(root: Path, frame_count: int = 52) -> None:
    root.mkdir(parents=True)
    (root / "summary.txt").write_text(_summary(frame_count), encoding="utf-8")
    (root / "trajectory.csv").write_text("0,0,0\n", encoding="utf-8")
    (root / "trajectory.tum").write_text("0 0 0 0 0 0 0 1\n", encoding="utf-8")


class MaxOutputCompareTests(unittest.TestCase):
    def _roots(self, frame_count: int = 52) -> tuple[tempfile.TemporaryDirectory[str], Path, Path, Path]:
        holder = tempfile.TemporaryDirectory()
        base = Path(holder.name)
        linux_full = base / "linux-full"
        linux_lean = base / "linux-lean"
        msvc_full = base / "msvc-full"
        _write_full(linux_full, frame_count)
        _write_lean(linux_lean, frame_count)
        _write_full(msvc_full, frame_count)
        return holder, linux_full, linux_lean, msvc_full

    def test_valid_52_contract_passes(self) -> None:
        holder, linux_full, linux_lean, msvc_full = self._roots()
        try:
            result = MODULE.compare_outputs(linux_full, linux_lean, msvc_full, 52)
        finally:
            holder.cleanup()
        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["expected_frames"], 52)
        self.assertTrue(result["external_engine_exit_contract"]["status"] == "NOT_INFERRED")

    def test_valid_80_contract_uses_frame_000079_marg_packet(self) -> None:
        holder, linux_full, linux_lean, msvc_full = self._roots(80)
        try:
            result = MODULE.compare_outputs(linux_full, linux_lean, msvc_full, 80)
        finally:
            holder.cleanup()
        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["linux_full"]["marg"]["expected_final"], "marg_data/frame_000079.json")

    def test_cli_writes_summary_and_accepts_expected_frames(self) -> None:
        holder, linux_full, linux_lean, msvc_full = self._roots(80)
        try:
            output = Path(holder.name) / "compare.json"
            rc = MODULE.main(
                [
                    "--linux-full",
                    str(linux_full),
                    "--linux-lean",
                    str(linux_lean),
                    "--msvc-full",
                    str(msvc_full),
                    "--summary",
                    str(output),
                    "--expected-frames",
                    "80",
                ]
            )
            result = json.loads(output.read_text(encoding="utf-8"))
        finally:
            holder.cleanup()
        self.assertEqual(rc, 0)
        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["expected_frames"], 80)

    def test_missing_required_full_file_cannot_compare_equal(self) -> None:
        holder, linux_full, linux_lean, msvc_full = self._roots()
        try:
            (linux_full / "trajectory.csv").unlink()
            result = MODULE.compare_outputs(linux_full, linux_lean, msvc_full)
        finally:
            holder.cleanup()
        self.assertEqual(result["status"], "FAIL")
        self.assertFalse(result["linux_full"]["valid"])
        self.assertFalse(result["comparisons"]["linux_full_vs_linux_lean_trajectory.csv"]["exact"])
        self.assertTrue(any("trajectory.csv" in item for item in result["linux_full"]["errors"]))

    def test_empty_required_lean_file_cannot_compare_equal(self) -> None:
        holder, linux_full, linux_lean, msvc_full = self._roots()
        try:
            (linux_lean / "trajectory.tum").write_text("", encoding="utf-8")
            result = MODULE.compare_outputs(linux_full, linux_lean, msvc_full)
        finally:
            holder.cleanup()
        self.assertEqual(result["status"], "FAIL")
        self.assertFalse(result["linux_lean"]["valid"])
        self.assertFalse(result["comparisons"]["linux_full_vs_linux_lean_trajectory.tum"]["exact"])

    def test_missing_summary_values_fail_closed(self) -> None:
        holder, linux_full, linux_lean, msvc_full = self._roots()
        try:
            (linux_full / "summary.txt").write_text("sensor_only=true\n", encoding="utf-8")
            result = MODULE.compare_outputs(linux_full, linux_lean, msvc_full)
        finally:
            holder.cleanup()
        self.assertEqual(result["status"], "FAIL")
        self.assertFalse(result["linux_full"]["summary_complete"])
        self.assertTrue(any("frames_requested" in item for item in result["linux_full"]["errors"]))

    def test_bad_frame_count_fails_closed(self) -> None:
        holder, linux_full, linux_lean, msvc_full = self._roots()
        try:
            (linux_full / "summary.txt").write_text(_summary(80), encoding="utf-8")
            result = MODULE.compare_outputs(linux_full, linux_lean, msvc_full, 52)
        finally:
            holder.cleanup()
        self.assertEqual(result["status"], "FAIL")
        self.assertTrue(any("expected frame count 52" in item for item in result["linux_full"]["errors"]))

    def test_wrong_marg_schema_fails_closed(self) -> None:
        holder, linux_full, linux_lean, msvc_full = self._roots()
        try:
            packet_path = msvc_full / "marg_data" / "frame_000051.json"
            packet = json.loads(packet_path.read_text(encoding="utf-8"))
            packet["schema_version"] = 3
            packet_path.write_text(json.dumps(packet), encoding="utf-8")
            result = MODULE.compare_outputs(linux_full, linux_lean, msvc_full)
        finally:
            holder.cleanup()
        self.assertEqual(result["status"], "FAIL")
        self.assertFalse(result["msvc_full"]["valid"])
        self.assertTrue(any("schema_version must be 4" in item for item in result["msvc_full"]["errors"]))

    def test_wrong_marg_shape_fails_closed(self) -> None:
        holder, linux_full, linux_lean, msvc_full = self._roots()
        try:
            packet_path = msvc_full / "marg_data" / "frame_000051.json"
            packet = json.loads(packet_path.read_text(encoding="utf-8"))
            packet["frame_poses"].pop()
            packet_path.write_text(json.dumps(packet), encoding="utf-8")
            result = MODULE.compare_outputs(linux_full, linux_lean, msvc_full)
        finally:
            holder.cleanup()
        self.assertEqual(result["status"], "FAIL")
        self.assertFalse(result["msvc_full"]["valid"])
        self.assertTrue(any("frame_poses length must equal 7" in item for item in result["msvc_full"]["errors"]))

    def test_unexpected_lean_trace_sidecar_fails_closed(self) -> None:
        holder, linux_full, linux_lean, msvc_full = self._roots()
        try:
            (linux_lean / "trace.jsonl").write_text('{"unexpected":true}\n', encoding="utf-8")
            result = MODULE.compare_outputs(linux_full, linux_lean, msvc_full)
        finally:
            holder.cleanup()
        self.assertEqual(result["status"], "FAIL")
        self.assertFalse(result["linux_lean"]["valid"])
        self.assertTrue(any("forbidden lean sidecar" in item for item in result["linux_lean"]["errors"]))

    def test_unexpected_lean_marg_inventory_fails_closed(self) -> None:
        holder, linux_full, linux_lean, msvc_full = self._roots()
        try:
            marg = linux_lean / "marg_data"
            marg.mkdir()
            (marg / "frame_000051.json").write_text("{}", encoding="utf-8")
            result = MODULE.compare_outputs(linux_full, linux_lean, msvc_full)
        finally:
            holder.cleanup()
        self.assertEqual(result["status"], "FAIL")
        self.assertTrue(any("forbidden lean sidecar" in item for item in result["linux_lean"]["errors"]))

    def test_numeric_marg_mismatch_is_reported(self) -> None:
        holder, linux_full, linux_lean, msvc_full = self._roots()
        try:
            packet_path = msvc_full / "marg_data" / "frame_000051.json"
            packet = json.loads(packet_path.read_text(encoding="utf-8"))
            packet["aom_abs_h"]["data"][0] = 2.0
            packet_path.write_text(json.dumps(packet), encoding="utf-8")
            result = MODULE.compare_outputs(linux_full, linux_lean, msvc_full)
        finally:
            holder.cleanup()
        comparison = result["comparisons"]["linux_full_vs_msvc_marg_inventory"]
        self.assertEqual(result["status"], "FAIL")
        self.assertFalse(comparison["exact"])
        self.assertIn("$.marg_data/frame_000051.json.aom_abs_h.data[0]", comparison["first_difference"]["path"])

    def test_marg_metadata_mismatch_is_reported(self) -> None:
        holder, linux_full, linux_lean, msvc_full = self._roots()
        try:
            packet_path = msvc_full / "marg_data" / "frame_000051.json"
            packet = json.loads(packet_path.read_text(encoding="utf-8"))
            packet["provenance_version"] = "different-provenance"
            packet_path.write_text(json.dumps(packet), encoding="utf-8")
            result = MODULE.compare_outputs(linux_full, linux_lean, msvc_full)
        finally:
            holder.cleanup()
        comparison = result["comparisons"]["linux_full_vs_msvc_marg_inventory"]
        self.assertEqual(result["status"], "FAIL")
        self.assertFalse(comparison["exact"])
        self.assertEqual(comparison["first_difference"]["reason"], "value")

    def test_extra_full_marg_packet_is_not_ignored(self) -> None:
        holder, linux_full, linux_lean, msvc_full = self._roots()
        try:
            extra = msvc_full / "marg_data" / "frame_000050.json"
            extra.write_text(json.dumps(_packet()), encoding="utf-8")
            result = MODULE.compare_outputs(linux_full, linux_lean, msvc_full)
        finally:
            holder.cleanup()
        comparison = result["comparisons"]["linux_full_vs_msvc_marg_inventory"]
        self.assertEqual(result["status"], "FAIL")
        self.assertFalse(comparison["exact"])
        self.assertIn("marg_data/frame_000050.json", comparison["first_difference"]["right_only"])

    def test_nonfinite_marg_number_fails_closed(self) -> None:
        holder, linux_full, linux_lean, msvc_full = self._roots()
        try:
            packet_path = msvc_full / "marg_data" / "frame_000051.json"
            packet = json.loads(packet_path.read_text(encoding="utf-8"))
            packet["aom_abs_b"][0] = float("nan")
            packet_path.write_text(json.dumps(packet), encoding="utf-8")
            result = MODULE.compare_outputs(linux_full, linux_lean, msvc_full)
        finally:
            holder.cleanup()
        self.assertEqual(result["status"], "FAIL")
        self.assertFalse(result["msvc_full"]["valid"])
        self.assertTrue(any("non-finite" in item for item in result["msvc_full"]["errors"]))


if __name__ == "__main__":
    unittest.main()
