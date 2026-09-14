#!/usr/bin/env python3
"""Fail-closed comparison of paired Basalt 52/80-frame output roots.

The comparator validates each root before comparing it.  It never treats two
missing or empty files, incomplete summaries, invalid MargData, or an absent
lean sidecar as an equality witness.  Process exit codes are an external
runner contract and are intentionally not inferred from output files; the
caller must record those codes separately.

The output destination is named with --summary.  --expected-frames accepts
the bounded protocol values 52 and 80 and defaults to 52 for compatibility
with the historical max52 invocation.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any


EXPECTED_FRAME_COUNTS = (52, 80)
SUMMARY_KEYS = (
    "sensor_only",
    "frames_requested",
    "frames_processed",
    "observations_emitted",
    "imu_samples_delivered",
)
INTEGER_SUMMARY_KEYS = SUMMARY_KEYS[1:]
FULL_REQUIRED_FILES = ("summary.txt", "trajectory.csv", "trajectory.tum", "trace.jsonl")
LEAN_REQUIRED_FILES = ("summary.txt", "trajectory.csv", "trajectory.tum")
LEAN_ALLOWED_FILES = frozenset(LEAN_REQUIRED_FILES)
LEAN_FORBIDDEN_TOKENS = ("marg", "trace", "timing")
MARG_REQUIRED_KEYS = (
    "schema_version",
    "provenance_version",
    "fej_complete",
    "used_imu",
    "kfs_all",
    "kfs_to_marg",
    "frame_poses",
    "frame_poses_fej",
    "frame_states",
    "frame_states_fej",
    "aom_order",
    "aom_abs_h",
    "aom_abs_b",
)
EXPECTED_MARG_SHAPES = {
    "frame_poses": 7,
    "frame_poses_fej": 7,
    "frame_states": 3,
    "frame_states_fej": 3,
    "aom_order": 9,
    "aom_abs_b": 72,
    "aom_abs_h_rows": 72,
    "aom_abs_h_cols": 72,
    "aom_abs_h_data": 72 * 72,
}


def _regular_file(path: Path) -> bool:
    try:
        return path.is_file() and not path.is_symlink()
    except OSError:
        return False


def _nonempty_file(path: Path) -> bool:
    if not _regular_file(path):
        return False
    try:
        return path.stat().st_size > 0
    except OSError:
        return False


def sha256(path: Path) -> str | None:
    """Return a regular-file digest, or None for a missing/symlinked file."""

    if not _regular_file(path):
        return None
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
    except OSError:
        return None
    return digest.hexdigest()


def _file_info(path: Path) -> dict[str, Any]:
    regular = _regular_file(path)
    size: int | None = None
    if regular:
        try:
            size = path.stat().st_size
        except OSError:
            regular = False
    return {
        "path": str(path),
        "sha256": sha256(path),
        "bytes": size,
        "present": regular,
        "nonempty": regular and size is not None and size > 0,
    }


def first_line_difference(left: Path, right: Path) -> dict[str, Any] | None:
    left_nonempty = _nonempty_file(left)
    right_nonempty = _nonempty_file(right)
    if not left_nonempty or not right_nonempty:
        return {
            "reason": "missing_file",
            "left_exists": _regular_file(left),
            "right_exists": _regular_file(right),
            "left_nonempty": left_nonempty,
            "right_nonempty": right_nonempty,
        }
    try:
        left_lines = left.read_text(encoding="utf-8", errors="replace").splitlines()
        right_lines = right.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError as exc:
        return {"reason": "unreadable_file", "error": str(exc)}
    for index, (left_line, right_line) in enumerate(zip(left_lines, right_lines), start=1):
        if left_line != right_line:
            return {
                "line": index,
                "left": left_line[:512],
                "right": right_line[:512],
                "reason": "line_value",
            }
    if len(left_lines) != len(right_lines):
        return {
            "line": min(len(left_lines), len(right_lines)) + 1,
            "left_lines": len(left_lines),
            "right_lines": len(right_lines),
            "reason": "line_count",
        }
    return None


def summary(path: Path) -> dict[str, str]:
    """Parse key/value lines without making validity claims.

    Callers that make a comparison must use _validate_summary, which rejects
    missing keys, duplicates, empty values, and malformed required counts.
    """

    result: dict[str, str] = {}
    if not _regular_file(path):
        return result
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return result
    for line in lines:
        key, separator, value = line.partition("=")
        if separator:
            result[key.strip()] = value.strip()
    return result


def _validate_summary(path: Path, expected_frames: int) -> tuple[dict[str, str], list[str]]:
    errors: list[str] = []
    values: dict[str, str] = {}
    if not _nonempty_file(path):
        errors.append(f"summary is missing or empty: {path}")
        return values, errors
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError as exc:
        errors.append(f"summary is unreadable: {path}: {exc}")
        return values, errors
    for line_number, raw_line in enumerate(lines, start=1):
        if not raw_line.strip():
            continue
        key, separator, value = raw_line.partition("=")
        key = key.strip()
        value = value.strip()
        if not separator or not key or not value:
            errors.append(f"summary line {line_number} is not a non-empty key=value record")
            continue
        if key in values:
            errors.append(f"summary key is duplicated: {key}")
            continue
        values[key] = value
    for key in SUMMARY_KEYS:
        if key not in values:
            errors.append(f"summary is missing required key: {key}")
    if values.get("sensor_only") != "true":
        errors.append("summary sensor_only must be exactly true")
    for key in INTEGER_SUMMARY_KEYS:
        raw_value = values.get(key)
        if raw_value is None:
            continue
        try:
            parsed = int(raw_value, 10)
        except ValueError:
            errors.append(f"summary {key} is not an integer: {raw_value!r}")
            continue
        if parsed < 0:
            errors.append(f"summary {key} must be non-negative")
        if key in ("frames_requested", "frames_processed") and parsed != expected_frames:
            errors.append(f"summary {key} must equal expected frame count {expected_frames}: {parsed}")
    return values, errors


def _validate_finite_numbers(value: Any, path: str, errors: list[str]) -> None:
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return
    if isinstance(value, (int, float)):
        try:
            finite = math.isfinite(value)
        except (OverflowError, TypeError):
            finite = False
        if not finite:
            errors.append(f"MargData numeric value is non-finite at {path}")
        return
    if isinstance(value, list):
        for index, item in enumerate(value):
            _validate_finite_numbers(item, f"{path}[{index}]", errors)
        return
    if isinstance(value, dict):
        for key, item in value.items():
            _validate_finite_numbers(item, f"{path}.{key}", errors)
        return
    errors.append(f"MargData contains unsupported JSON value at {path}")


def _validate_numeric_sequence(value: Any, path: str, errors: list[str]) -> None:
    if not isinstance(value, list):
        errors.append(f"MargData numeric sequence must be a list at {path}")
        return
    for index, item in enumerate(value):
        item_path = f"{path}[{index}]"
        if isinstance(item, list):
            _validate_numeric_sequence(item, item_path, errors)
        elif isinstance(item, bool) or not isinstance(item, (int, float)):
            errors.append(f"MargData numeric sequence contains non-number at {item_path}")
        else:
            try:
                finite = math.isfinite(item)
            except (OverflowError, TypeError):
                finite = False
            if not finite:
                errors.append(f"MargData numeric sequence is non-finite at {item_path}")


def _validate_integer_list(value: Any, path: str, errors: list[str]) -> None:
    if not isinstance(value, list):
        errors.append(f"MargData integer sequence must be a list at {path}")
        return
    for index, item in enumerate(value):
        if isinstance(item, bool) or not isinstance(item, int):
            errors.append(f"MargData integer sequence contains non-integer at {path}[{index}]")


def _shape_descriptor(value: Any) -> Any:
    if isinstance(value, dict):
        return {
            "type": "object",
            "keys": {key: _shape_descriptor(value[key]) for key in sorted(value)},
        }
    if isinstance(value, list):
        distinct_shapes = {
            json.dumps(_shape_descriptor(item), sort_keys=True, separators=(",", ":"))
            for item in value
        }
        return {
            "type": "array",
            "length": len(value),
            "item_shapes": [json.loads(item) for item in sorted(distinct_shapes)],
        }
    if isinstance(value, bool):
        return {"type": "bool"}
    if isinstance(value, int):
        return {"type": "integer"}
    if isinstance(value, float):
        return {"type": "number"}
    if isinstance(value, str):
        return {"type": "string"}
    if value is None:
        return {"type": "null"}
    return {"type": type(value).__name__}


def _numeric_records(value: Any, path: str = "$") -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    if isinstance(value, dict):
        for key in sorted(value):
            records.extend(_numeric_records(value[key], f"{path}.{key}"))
    elif isinstance(value, list):
        for index, item in enumerate(value):
            records.extend(_numeric_records(item, f"{path}[{index}]"))
    elif isinstance(value, (int, float)) and not isinstance(value, bool):
        records.append({"path": path, "type": type(value).__name__, "value": value})
    return records


def _json_digest(value: Any) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _safe_len(value: Any) -> int | None:
    return len(value) if isinstance(value, (list, dict, str)) else None


def _marg_packet_compact(path: Path) -> tuple[dict[str, Any], dict[str, Any] | None, list[str]]:
    errors: list[str] = []
    compact: dict[str, Any] = {
        "path": str(path),
        "status": "MISSING",
        "bytes": None,
        "sha256": None,
    }
    if not _nonempty_file(path):
        errors.append(f"MargData packet is missing or empty: {path}")
        return compact, None, errors
    compact["bytes"] = path.stat().st_size
    compact["sha256"] = sha256(path)
    try:
        packet = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        errors.append(f"MargData packet is not valid JSON: {path}: {exc}")
        compact["status"] = "INVALID"
        compact["errors"] = errors
        return compact, None, errors
    if not isinstance(packet, dict):
        errors.append(f"MargData packet must be a JSON object: {path}")
        compact["status"] = "INVALID"
        compact["errors"] = errors
        return compact, None, errors
    for key in MARG_REQUIRED_KEYS:
        if key not in packet:
            errors.append(f"MargData packet is missing required key {key}: {path}")
    if packet.get("schema_version") != 4:
        errors.append(f"MargData schema_version must be 4: {path}")
    if not isinstance(packet.get("provenance_version"), str) or not packet.get("provenance_version"):
        errors.append(f"MargData provenance_version must be non-empty: {path}")
    for key in ("fej_complete", "used_imu"):
        if packet.get(key) is not True:
            errors.append(f"MargData {key} must be true: {path}")
    for key in ("kfs_all", "kfs_to_marg", "frame_poses", "frame_states", "aom_order", "aom_abs_b"):
        value = packet.get(key)
        if not isinstance(value, list) or not value:
            errors.append(f"MargData {key} must be a non-empty list: {path}")
    for key in ("frame_poses_fej", "frame_states_fej"):
        value = packet.get(key)
        if not isinstance(value, (dict, list)) or not value:
            errors.append(f"MargData {key} must be a non-empty map/list: {path}")
    for key in ("frame_poses", "frame_poses_fej", "frame_states", "frame_states_fej", "aom_order", "aom_abs_b"):
        value = packet.get(key)
        expected_length = EXPECTED_MARG_SHAPES[key]
        if isinstance(value, (dict, list)) and len(value) != expected_length:
            errors.append(f"MargData {key} length must equal {expected_length}: {path}")
    if (
        isinstance(packet.get("frame_poses"), list)
        and isinstance(packet.get("frame_poses_fej"), (dict, list))
        and len(packet["frame_poses"]) != len(packet["frame_poses_fej"])
    ):
        errors.append(f"MargData frame_poses/frame_poses_fej lengths differ: {path}")
    if (
        isinstance(packet.get("frame_states"), list)
        and isinstance(packet.get("frame_states_fej"), (dict, list))
        and len(packet["frame_states"]) != len(packet["frame_states_fej"])
    ):
        errors.append(f"MargData frame_states/frame_states_fej lengths differ: {path}")
    hessian = packet.get("aom_abs_h")
    if not isinstance(hessian, dict):
        errors.append(f"MargData aom_abs_h must be an object: {path}")
    else:
        rows = hessian.get("rows")
        cols = hessian.get("cols")
        data = hessian.get("data")
        if isinstance(rows, bool) or not isinstance(rows, int) or rows <= 0:
            errors.append(f"MargData aom_abs_h.rows must be a positive integer: {path}")
        if isinstance(cols, bool) or not isinstance(cols, int) or cols <= 0:
            errors.append(f"MargData aom_abs_h.cols must be a positive integer: {path}")
        if not isinstance(data, list):
            errors.append(f"MargData aom_abs_h.data must be a list: {path}")
        elif isinstance(rows, int) and isinstance(cols, int) and rows > 0 and cols > 0:
            if len(data) != rows * cols:
                errors.append(
                    f"MargData aom_abs_h.data length must equal rows*cols ({rows * cols}): {path}"
                )
        if rows != EXPECTED_MARG_SHAPES["aom_abs_h_rows"]:
            errors.append(f"MargData aom_abs_h.rows must equal 72: {path}")
        if cols != EXPECTED_MARG_SHAPES["aom_abs_h_cols"]:
            errors.append(f"MargData aom_abs_h.cols must equal 72: {path}")
        if isinstance(data, list) and len(data) != EXPECTED_MARG_SHAPES["aom_abs_h_data"]:
            errors.append(f"MargData aom_abs_h.data length must equal 5184: {path}")
        _validate_numeric_sequence(data, "$.aom_abs_h.data", errors)
    _validate_integer_list(packet.get("kfs_all"), "$.kfs_all", errors)
    _validate_integer_list(packet.get("kfs_to_marg"), "$.kfs_to_marg", errors)
    _validate_numeric_sequence(packet.get("aom_abs_b"), "$.aom_abs_b", errors)
    _validate_finite_numbers(packet, "$", errors)
    view = {key: packet.get(key) for key in MARG_REQUIRED_KEYS}
    numeric_records = _numeric_records(packet)
    compact.update(
        {
            "status": "PRESENT" if not errors else "INVALID",
            "schema_version": packet.get("schema_version"),
            "provenance_version": packet.get("provenance_version"),
            "fej_complete": packet.get("fej_complete"),
            "used_imu": packet.get("used_imu"),
            "frame_poses_len": _safe_len(packet.get("frame_poses")),
            "frame_poses_fej_len": _safe_len(packet.get("frame_poses_fej")),
            "frame_states_len": _safe_len(packet.get("frame_states")),
            "frame_states_fej_len": _safe_len(packet.get("frame_states_fej")),
            "aom_order_len": _safe_len(packet.get("aom_order")),
            "aom_abs_b_len": _safe_len(packet.get("aom_abs_b")),
            "aom_abs_h_rows": hessian.get("rows") if isinstance(hessian, dict) else None,
            "aom_abs_h_cols": hessian.get("cols") if isinstance(hessian, dict) else None,
            "aom_abs_h_data_len": _safe_len(hessian.get("data")) if isinstance(hessian, dict) else None,
            "shape_sha256": _json_digest(_shape_descriptor(view)),
            "numeric_sha256": _json_digest(numeric_records),
            "numeric_value_count": len(numeric_records),
        }
    )
    if errors:
        compact["errors"] = errors
    return compact, packet, errors


def _marg_inventory(
    root: Path,
    expected_frames: int,
) -> tuple[dict[str, Any], list[str]]:
    errors: list[str] = []
    inventory: dict[str, Any] = {}
    marg_dir = root / "marg_data"
    if not marg_dir.is_dir() or marg_dir.is_symlink():
        errors.append(f"full output MargData directory is missing or symlinked: {marg_dir}")
        return inventory, errors
    try:
        entries = sorted(marg_dir.rglob("*"))
    except OSError as exc:
        errors.append(f"cannot enumerate MargData directory {marg_dir}: {exc}")
        return inventory, errors
    regular_files: list[Path] = []
    for entry in entries:
        if entry.is_symlink():
            errors.append(f"MargData inventory contains a symlink: {entry}")
        elif entry.is_file():
            regular_files.append(entry)
    if not regular_files:
        errors.append(f"full output MargData inventory is empty: {marg_dir}")
    for packet_path in regular_files:
        relative = packet_path.relative_to(root).as_posix()
        if packet_path.suffix.lower() != ".json":
            errors.append(f"MargData inventory contains a non-JSON file: {relative}")
            continue
        compact, _packet, packet_errors = _marg_packet_compact(packet_path)
        inventory[relative] = compact
        errors.extend(packet_errors)
    expected_final = f"marg_data/frame_{expected_frames - 1:06d}.json"
    if expected_final not in inventory:
        errors.append(f"full output is missing expected final MargData packet: {expected_final}")
    return inventory, errors


def marg_compact(path: Path, expected_frames: int = 52) -> dict[str, Any]:
    """Return a compact MargData record for compatibility with old callers."""

    compact, _packet, errors = _marg_packet_compact(path)
    if errors:
        compact["errors"] = errors
    return compact


def _all_root_files(root: Path) -> tuple[list[str], list[str]]:
    files: list[str] = []
    errors: list[str] = []
    if not root.is_dir() or root.is_symlink():
        return files, [f"output root is missing or symlinked: {root}"]
    try:
        entries = sorted(root.rglob("*"))
    except OSError as exc:
        return files, [f"cannot enumerate output root {root}: {exc}"]
    for entry in entries:
        if entry.is_symlink():
            errors.append(f"output inventory contains a symlink: {entry.relative_to(root).as_posix()}")
        elif entry.is_file():
            files.append(entry.relative_to(root).as_posix())
    return files, errors


def root_compact(root: Path, expected_frames: int = 52, role: str = "full") -> dict[str, Any]:
    if expected_frames not in EXPECTED_FRAME_COUNTS:
        raise ValueError(f"expected_frames must be one of {EXPECTED_FRAME_COUNTS}")
    if role not in ("full", "lean"):
        raise ValueError("role must be full or lean")
    root = Path(root)
    errors: list[str] = []
    all_files, inventory_errors = _all_root_files(root)
    errors.extend(inventory_errors)
    required_files = FULL_REQUIRED_FILES if role == "full" else LEAN_REQUIRED_FILES
    for name in required_files:
        if not _nonempty_file(root / name):
            errors.append(f"{role} required file is missing or empty: {name}")
    summary_data, summary_errors = _validate_summary(root / "summary.txt", expected_frames)
    errors.extend(summary_errors)
    file_records = {
        name: _file_info(root / name)
        for name in ("trajectory.csv", "trajectory.tum", "summary.txt", "trace.jsonl")
    }
    marg: dict[str, Any]
    if role == "full":
        marg_inventory, marg_errors = _marg_inventory(root, expected_frames)
        errors.extend(marg_errors)
        marg = {
            "status": "PRESENT" if not marg_errors else "INVALID",
            "inventory": marg_inventory,
            "expected_final": f"marg_data/frame_{expected_frames - 1:06d}.json",
        }
    else:
        marg = {"status": "ABSENT", "inventory": {}, "expected_final": None}
        for relative in all_files:
            if relative not in LEAN_ALLOWED_FILES:
                lower = relative.lower()
                if any(token in lower for token in LEAN_FORBIDDEN_TOKENS):
                    errors.append(f"forbidden lean sidecar: {relative}")
                else:
                    errors.append(f"unexpected lean output file: {relative}")
    return {
        "root": str(root),
        "role": role,
        "expected_frames": expected_frames,
        "valid": not errors,
        "errors": errors,
        "summary": {key: summary_data.get(key) for key in SUMMARY_KEYS},
        "summary_complete": not summary_errors,
        "files": file_records,
        "marg": marg,
        "inventory_files": all_files,
    }


def _file_exact(left: Path, right: Path) -> bool:
    left_hash = sha256(left)
    right_hash = sha256(right)
    return (
        _nonempty_file(left)
        and _nonempty_file(right)
        and left_hash is not None
        and right_hash is not None
        and left_hash == right_hash
    )


def _file_comparison(left: Path, right: Path) -> dict[str, Any]:
    return {
        "exact": _file_exact(left, right),
        "first_difference": first_line_difference(left, right),
    }


def _summary_comparison(left: dict[str, Any], right: dict[str, Any]) -> dict[str, Any]:
    exact = (
        left["valid"]
        and right["valid"]
        and left["summary_complete"]
        and right["summary_complete"]
        and left["summary"] == right["summary"]
    )
    return {
        "exact": exact,
        "left": left["summary"],
        "right": right["summary"],
    }


def _first_json_difference(left: Any, right: Any, path: str = "$") -> dict[str, Any] | None:
    if isinstance(left, bool) or isinstance(right, bool):
        if type(left) is not type(right) or left != right:
            return {"path": path, "reason": "value", "left": left, "right": right}
        return None
    if type(left) is not type(right):
        return {
            "path": path,
            "reason": "type",
            "left_type": type(left).__name__,
            "right_type": type(right).__name__,
        }
    if isinstance(left, dict):
        left_keys = set(left)
        right_keys = set(right)
        if left_keys != right_keys:
            return {
                "path": path,
                "reason": "keys",
                "left_only": sorted(left_keys - right_keys),
                "right_only": sorted(right_keys - left_keys),
            }
        for key in sorted(left_keys):
            difference = _first_json_difference(left[key], right[key], f"{path}.{key}")
            if difference is not None:
                return difference
        return None
    if isinstance(left, list):
        if len(left) != len(right):
            return {"path": path, "reason": "length", "left": len(left), "right": len(right)}
        for index, (left_item, right_item) in enumerate(zip(left, right)):
            difference = _first_json_difference(left_item, right_item, f"{path}[{index}]")
            if difference is not None:
                return difference
        return None
    if left != right:
        return {"path": path, "reason": "value", "left": left, "right": right}
    return None


def _marg_view(packet: dict[str, Any]) -> dict[str, Any]:
    return packet


def _read_packet(path: Path) -> dict[str, Any] | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def _marg_comparison(
    left: dict[str, Any],
    right: dict[str, Any],
) -> dict[str, Any]:
    left_inventory = left["marg"]["inventory"]
    right_inventory = right["marg"]["inventory"]
    left_names = sorted(left_inventory)
    right_names = sorted(right_inventory)
    first_difference: dict[str, Any] | None = None
    exact = left["valid"] and right["valid"] and left_names == right_names
    if left_names != right_names:
        first_difference = {
            "reason": "inventory",
            "left_only": sorted(set(left_names) - set(right_names)),
            "right_only": sorted(set(right_names) - set(left_names)),
        }
    if exact:
        for relative in left_names:
            left_info = left_inventory[relative]
            right_info = right_inventory[relative]
            left_packet = _read_packet(Path(left["root"]) / relative)
            right_packet = _read_packet(Path(right["root"]) / relative)
            packet_difference = None
            if left_packet is not None and right_packet is not None:
                packet_difference = _first_json_difference(
                    _marg_view(left_packet),
                    _marg_view(right_packet),
                    f"$.{relative}",
                )
            if (
                left_info.get("status") != "PRESENT"
                or right_info.get("status") != "PRESENT"
                or left_info.get("shape_sha256") != right_info.get("shape_sha256")
                or left_info.get("numeric_sha256") != right_info.get("numeric_sha256")
                or left_packet is None
                or right_packet is None
                or packet_difference is not None
            ):
                exact = False
                if left_packet is None or right_packet is None:
                    first_difference = {
                        "path": relative,
                        "reason": "invalid_packet",
                        "left_status": left_info.get("status"),
                        "right_status": right_info.get("status"),
                    }
                else:
                    first_difference = packet_difference
                break
    return {
        "exact": exact,
        "left_inventory": left_names,
        "right_inventory": right_names,
        "first_difference": first_difference,
    }


def compare_outputs(
    linux_full_root: Path,
    linux_lean_root: Path,
    msvc_full_root: Path,
    expected_frames: int = 52,
) -> dict[str, Any]:
    linux_full = root_compact(linux_full_root, expected_frames, "full")
    linux_lean = root_compact(linux_lean_root, expected_frames, "lean")
    msvc_full = root_compact(msvc_full_root, expected_frames, "full")
    comparisons: dict[str, Any] = {}
    for name in ("trajectory.csv", "trajectory.tum"):
        comparisons[f"linux_full_vs_linux_lean_{name}"] = _file_comparison(
            Path(linux_full_root) / name,
            Path(linux_lean_root) / name,
        )
        comparisons[f"linux_full_vs_msvc_full_{name}"] = _file_comparison(
            Path(linux_full_root) / name,
            Path(msvc_full_root) / name,
        )
    lean_trace = Path(linux_lean_root) / "trace.jsonl"
    comparisons["linux_full_vs_linux_lean_trace.jsonl"] = {
        "exact": (
            linux_full["valid"]
            and linux_lean["valid"]
            and _nonempty_file(Path(linux_full_root) / "trace.jsonl")
            and not _regular_file(lean_trace)
        ),
        "expected_absent_in_lean": True,
        "first_difference": first_line_difference(Path(linux_full_root) / "trace.jsonl", lean_trace),
    }
    comparisons["linux_full_vs_msvc_full_trace.jsonl"] = _file_comparison(
        Path(linux_full_root) / "trace.jsonl",
        Path(msvc_full_root) / "trace.jsonl",
    )
    comparisons["linux_full_vs_linux_lean_summary_counts"] = _summary_comparison(linux_full, linux_lean)
    comparisons["linux_full_vs_msvc_full_summary_counts"] = _summary_comparison(linux_full, msvc_full)
    comparisons["linux_full_vs_msvc_marg_inventory"] = _marg_comparison(linux_full, msvc_full)
    comparison_exact = all(item.get("exact") is True for item in comparisons.values())
    result = {
        "schema": "visloc.basalt.m11.max_output_compare.v2",
        "expected_frames": expected_frames,
        "linux_full": linux_full,
        "linux_lean": linux_lean,
        "msvc_full": msvc_full,
        "comparisons": comparisons,
        "external_engine_exit_contract": {
            "status": "NOT_INFERRED",
            "note": "Process exit codes must be established by the runner; output presence/equality cannot prove zero engine RC.",
        },
        "status": "PASS" if comparison_exact else "FAIL",
    }
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--linux-full", required=True, type=Path)
    parser.add_argument("--linux-lean", required=True, type=Path)
    parser.add_argument("--msvc-full", required=True, type=Path)
    parser.add_argument("--summary", required=True, type=Path)
    parser.add_argument(
        "--expected-frames",
        type=int,
        choices=EXPECTED_FRAME_COUNTS,
        default=52,
        help="expected frames in each summary and final MargData packet (default: 52)",
    )
    args = parser.parse_args(argv)
    result = compare_outputs(
        args.linux_full,
        args.linux_lean,
        args.msvc_full,
        args.expected_frames,
    )
    args.summary.parent.mkdir(parents=True, exist_ok=True)
    args.summary.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(
        json.dumps(
            {
                "status": result["status"],
                "expected_frames": args.expected_frames,
                "trajectory_csv_exact": result["comparisons"]["linux_full_vs_msvc_full_trajectory.csv"]["exact"],
                "trajectory_tum_exact": result["comparisons"]["linux_full_vs_msvc_full_trajectory.tum"]["exact"],
                "trace_exact": result["comparisons"]["linux_full_vs_msvc_full_trace.jsonl"]["exact"],
                "marg_inventory_exact": result["comparisons"]["linux_full_vs_msvc_marg_inventory"]["exact"],
                "summary": str(args.summary),
                "external_engine_exit_contract": "NOT_INFERRED",
            },
            sort_keys=True,
        )
    )
    return 0 if result["status"] == "PASS" else 8


if __name__ == "__main__":
    raise SystemExit(main())
