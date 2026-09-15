#!/usr/bin/env python3
"""Create an all11 sensor-only EuRoC view using same-volume hardlinks.

The physical dataset and the destination are intentionally machine-local.  The
main path only accepts the manifest's physical roots under E:\\datasets\\euroc_mav
and the fixed destination E:\\datasets\\euroc_mav\\sensor_hardlink_all11.  A
destination that already exists is never repaired in place: it must match the
source tree and every source/destination ``(st_dev, st_ino, st_size)`` tuple, or
the command fails without modifying it.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import shutil
import stat
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable


ALLOWED_DATASET_BASE = Path(r"E:\datasets\euroc_mav")
EXPECTED_DESTINATION = ALLOWED_DATASET_BASE / "sensor_hardlink_all11"
DEFAULT_MANIFEST = Path("benchmarks/basalt/euroc_dataset_manifest.json")
DEFAULT_ARTIFACT = Path("work/m11_hardlink_sensor_view_creation_20260830.json")
SENSORS = ("cam0", "cam1", "imu0")
CAMERAS = ("cam0", "cam1")
FORBIDDEN_PATH_TOKENS = ("groundtruth", "ground_truth", "state_groundtruth", ".zip")
REPARSE_ATTRIBUTE = getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400)


class SensorViewError(RuntimeError):
    """A validation or safe-staging failure."""


# Compatibility aliases for the earlier test/helper spelling.  The canonical
# public names remain ``SensorViewError`` and ``create_sensor_view``.
ViewError = SensorViewError


@dataclass(frozen=True)
class TreeEntry:
    rel: str
    kind: str
    source: Path
    size: int = 0
    st_dev: int = 0
    st_ino: int = 0


def _norm(path: Path) -> str:
    return os.path.normcase(os.path.abspath(os.fspath(path)))


def _is_under(path: Path, parent: Path) -> bool:
    try:
        return os.path.commonpath([_norm(path), _norm(parent)]) == _norm(parent)
    except ValueError:
        return False


def _is_reparse(path: Path, st: os.stat_result | None = None) -> bool:
    if path.is_symlink():
        return True
    if st is None:
        st = os.stat(path, follow_symlinks=False)
    return bool(getattr(st, "st_file_attributes", 0) & REPARSE_ATTRIBUTE)


def _stat_regular(path: Path, *, label: str) -> os.stat_result:
    try:
        st = os.stat(path, follow_symlinks=False)
    except OSError as exc:
        raise SensorViewError(f"cannot stat {label}: {path}: {exc}") from exc
    if _is_reparse(path, st):
        raise SensorViewError(f"reparse point/symlink is forbidden for {label}: {path}")
    if not stat.S_ISREG(st.st_mode):
        raise SensorViewError(f"expected regular file for {label}: {path}")
    return st


def _stat_directory(path: Path, *, label: str) -> os.stat_result:
    try:
        st = os.stat(path, follow_symlinks=False)
    except OSError as exc:
        raise SensorViewError(f"cannot stat {label}: {path}: {exc}") from exc
    if _is_reparse(path, st):
        raise SensorViewError(f"reparse point/symlink is forbidden for {label}: {path}")
    if not stat.S_ISDIR(st.st_mode):
        raise SensorViewError(f"expected directory for {label}: {path}")
    return st


def _forbidden_rel(rel: str) -> str | None:
    lowered = rel.casefold()
    for token in FORBIDDEN_PATH_TOKENS:
        if token in lowered:
            return token
    return None


def _add_entry(entries: dict[str, TreeEntry], entry: TreeEntry) -> None:
    if _forbidden_rel(entry.rel):
        raise SensorViewError(f"forbidden GT/zip path in sensor view: {entry.rel}")
    old = entries.get(entry.rel)
    if old is not None and old.kind != entry.kind:
        raise SensorViewError(f"tree entry kind collision at {entry.rel}")
    entries[entry.rel] = entry


def _scan_tree(root: Path, destination_rel: str, entries: dict[str, TreeEntry]) -> int:
    """Scan a sensor directory, rejecting non-regular/reparse entries."""

    _stat_directory(root, label="sensor source")
    _add_entry(entries, TreeEntry(destination_rel, "dir", root))
    reparse_count = 0

    def visit(source_dir: Path, rel_dir: str) -> None:
        nonlocal reparse_count
        try:
            children = sorted(os.scandir(source_dir), key=lambda item: item.name)
        except OSError as exc:
            raise SensorViewError(f"cannot enumerate sensor directory: {source_dir}: {exc}") from exc
        for child in children:
            child_path = Path(child.path)
            try:
                st = child.stat(follow_symlinks=False)
            except OSError as exc:
                raise SensorViewError(f"cannot stat sensor entry: {child_path}: {exc}") from exc
            if _is_reparse(child_path, st):
                reparse_count += 1
                raise SensorViewError(f"reparse point/symlink is forbidden: {child_path}")
            rel = f"{rel_dir}/{child.name}" if rel_dir else child.name
            if stat.S_ISDIR(st.st_mode):
                _add_entry(entries, TreeEntry(rel, "dir", child_path))
                visit(child_path, rel)
            elif stat.S_ISREG(st.st_mode):
                _add_entry(
                    entries,
                    TreeEntry(
                        rel,
                        "file",
                        child_path,
                        size=st.st_size,
                        st_dev=st.st_dev,
                        st_ino=st.st_ino,
                    ),
                )
            else:
                raise SensorViewError(f"non-regular sensor entry is forbidden: {child_path}")

    visit(root, destination_rel)
    return reparse_count


def _source_sequence_records(manifest: dict[str, Any], *, strict_all11: bool) -> list[dict[str, Any]]:
    records = manifest.get("sequences")
    if not isinstance(records, list) or not records:
        raise SensorViewError("dataset manifest has no sequences")
    if strict_all11:
        order = manifest.get("protocol", {}).get("sequence_order")
        if not isinstance(order, list) or len(order) != 11:
            raise SensorViewError("manifest protocol does not define the frozen all11 sequence order")
        ids = [record.get("id") for record in records if isinstance(record, dict)]
        if ids != order or len(records) != 11:
            raise SensorViewError(f"manifest sequence order/count is not all11: {ids!r}")
    result: list[dict[str, Any]] = []
    seen: set[str] = set()
    for record in records:
        if not isinstance(record, dict):
            raise SensorViewError("manifest sequence record is not an object")
        sequence = record.get("id")
        physical = record.get("physical_root")
        if not isinstance(sequence, str) or not sequence or sequence in seen:
            raise SensorViewError(f"invalid or duplicate sequence id: {sequence!r}")
        if not isinstance(physical, dict) or not isinstance(physical.get("path"), str):
            raise SensorViewError(f"sequence has no physical_root.path: {sequence}")
        seen.add(sequence)
        result.append(record)
    return result


def _manifest_bound_files(
    records: Iterable[dict[str, Any]], *, strict_all11: bool
) -> tuple[list[dict[str, Any]], dict[str, str]]:
    """Return manifest-bound files and their expected SHA-256 values."""

    bound: list[dict[str, Any]] = []
    expected: dict[str, str] = {}
    for record in records:
        sequence = str(record["id"])
        file_hashes = record.get("file_hashes", {})
        if not isinstance(file_hashes, dict):
            if strict_all11:
                raise SensorViewError(f"missing file_hashes for {sequence}")
            continue
        csv_hashes = file_hashes.get("csv_sha256", {})
        yaml_hashes = file_hashes.get("sensor_yaml_sha256", {})
        for role, values in (("csv", csv_hashes), ("sensor_yaml", yaml_hashes)):
            if not isinstance(values, dict):
                if strict_all11:
                    raise SensorViewError(f"missing {role} hashes for {sequence}")
                continue
            for relative, digest in values.items():
                if not isinstance(relative, str) or not isinstance(digest, str):
                    raise SensorViewError(f"invalid manifest file hash record for {sequence}")
                key = f"{sequence}/{relative}"
                if key in expected:
                    raise SensorViewError(f"duplicate manifest-bound file: {key}")
                expected[key] = digest
                bound.append(
                    {
                        "sequence": sequence,
                        "relative_path": relative,
                        "role": role,
                        "manifest_sha256": digest,
                    }
                )
    if strict_all11 and len(bound) != 77:
        raise SensorViewError(f"expected 77 manifest-bound files, got {len(bound)}")
    return bound, expected


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _verify_manifest_bound_sources(
    bound: list[dict[str, Any]], records_by_id: dict[str, dict[str, Any]]
) -> tuple[list[dict[str, Any]], int, int]:
    source_results: list[dict[str, Any]] = []
    gt_count = 0
    sensor_count = 0
    for item in bound:
        sequence = item["sequence"]
        relative = item["relative_path"]
        source = Path(records_by_id[sequence]["physical_root"]["path"]) / Path(relative)
        st = _stat_regular(source, label=f"manifest-bound {sequence}/{relative}")
        digest = _sha256(source)
        matches = digest == item["manifest_sha256"]
        if not matches:
            raise SensorViewError(
                f"manifest-bound SHA mismatch for {sequence}/{relative}: "
                f"expected {item['manifest_sha256']}, got {digest}"
            )
        is_gt = "groundtruth" in relative.casefold() or "ground_truth" in relative.casefold()
        if is_gt:
            gt_count += 1
        else:
            sensor_count += 1
        item_result = dict(item)
        item_result.update({"bytes": st.st_size, "source_present": True, "source_sha256_match": True})
        source_results.append(item_result)
    return source_results, sensor_count, gt_count


def _validate_camera_pngs(
    records: list[dict[str, Any]], entries: dict[str, TreeEntry]
) -> list[dict[str, Any]]:
    results: list[dict[str, Any]] = []
    for record in records:
        sequence = str(record["id"])
        source_root = Path(record["physical_root"]["path"])
        for camera in CAMERAS:
            camera_root = source_root / "mav0" / camera
            csv_path = camera_root / "data.csv"
            data_dir = camera_root / "data"
            _stat_regular(csv_path, label=f"{sequence} {camera} data.csv")
            _stat_directory(data_dir, label=f"{sequence} {camera} data")
            with csv_path.open("r", encoding="utf-8", newline="") as stream:
                rows = list(csv.reader(stream))
            if not rows or len(rows[0]) < 2:
                raise SensorViewError(f"invalid camera CSV header: {csv_path}")
            referenced = {row[1] for row in rows[1:] if len(row) >= 2 and row[1]}
            if len(referenced) != len(rows) - 1:
                raise SensorViewError(f"duplicate/invalid camera CSV image references: {csv_path}")
            pngs = set()
            for path in sorted(data_dir.iterdir()):
                st = os.stat(path, follow_symlinks=False)
                if _is_reparse(path, st) or not stat.S_ISREG(st.st_mode):
                    raise SensorViewError(f"camera data contains non-regular/reparse entry: {path}")
                if path.suffix.casefold() != ".png":
                    raise SensorViewError(f"camera data contains non-PNG file: {path}")
                pngs.add(path.name)
            if pngs != referenced:
                missing = sorted(referenced - pngs)[:3]
                extra = sorted(pngs - referenced)[:3]
                raise SensorViewError(
                    f"camera PNG set mismatch for {sequence}/{camera}: missing={missing}, extra={extra}"
                )
            destination_prefix = f"{sequence}/mav0/{camera}/data"
            destination_pngs = {
                rel.rsplit("/", 1)[-1]
                for rel, entry in entries.items()
                if entry.kind == "file" and rel.startswith(destination_prefix + "/")
            }
            results.append(
                {
                    "sequence": sequence,
                    "camera": camera,
                    "csv_rows": len(rows) - 1,
                    "referenced_pngs": len(referenced),
                    "source_pngs": len(pngs),
                    "staged_pngs": len(destination_pngs),
                    "exact": pngs == referenced and destination_pngs == referenced,
                }
            )
    return results


def _build_expected_entries(records: list[dict[str, Any]]) -> tuple[dict[str, TreeEntry], int]:
    entries: dict[str, TreeEntry] = {}
    reparse_count = 0
    for record in records:
        sequence = str(record["id"])
        source_root = Path(record["physical_root"]["path"])
        _stat_directory(source_root, label=f"physical sequence {sequence}")
        _add_entry(entries, TreeEntry(sequence, "dir", source_root))
        mav0 = source_root / "mav0"
        _stat_directory(mav0, label=f"{sequence}/mav0")
        _add_entry(entries, TreeEntry(f"{sequence}/mav0", "dir", mav0))
        for sensor in SENSORS:
            sensor_root = mav0 / sensor
            reparse_count += _scan_tree(sensor_root, f"{sequence}/mav0/{sensor}", entries)
    return entries, reparse_count


def _scan_destination(root: Path) -> tuple[dict[str, tuple[str, os.stat_result]], int]:
    """Scan destination entries without following links."""

    _stat_directory(root, label="destination")
    found: dict[str, tuple[str, os.stat_result]] = {}
    reparse_count = 0

    def visit(directory: Path, rel_dir: str) -> None:
        nonlocal reparse_count
        try:
            children = sorted(os.scandir(directory), key=lambda item: item.name)
        except OSError as exc:
            raise SensorViewError(f"cannot enumerate destination: {directory}: {exc}") from exc
        for child in children:
            path = Path(child.path)
            try:
                st = child.stat(follow_symlinks=False)
            except OSError as exc:
                raise SensorViewError(f"cannot stat destination entry: {path}: {exc}") from exc
            if _is_reparse(path, st):
                reparse_count += 1
                raise SensorViewError(f"destination contains reparse point/symlink: {path}")
            rel = f"{rel_dir}/{child.name}" if rel_dir else child.name
            if _forbidden_rel(rel):
                raise SensorViewError(f"destination contains forbidden GT/zip path: {rel}")
            if stat.S_ISDIR(st.st_mode):
                found[rel] = ("dir", st)
                visit(path, rel)
            elif stat.S_ISREG(st.st_mode):
                found[rel] = ("file", st)
            else:
                raise SensorViewError(f"destination contains non-regular entry: {path}")

    visit(root, "")
    return found, reparse_count


def _validate_tree(
    destination: Path, expected: dict[str, TreeEntry], *, compare_identity: bool
) -> dict[str, Any]:
    found, reparse_count = _scan_destination(destination)
    expected_kinds = {rel: entry.kind for rel, entry in expected.items()}
    missing = sorted(set(expected_kinds) - set(found))
    extra = sorted(set(found) - set(expected_kinds))
    kind_mismatches = sorted(
        rel for rel in set(expected_kinds).intersection(found) if expected_kinds[rel] != found[rel][0]
    )
    identity_mismatches: list[str] = []
    hardlink_count = 0
    logical_bytes = 0
    for rel, entry in expected.items():
        if entry.kind != "file" or rel not in found:
            continue
        logical_bytes += entry.size
        _, destination_stat = found[rel]
        if compare_identity and (
            destination_stat.st_dev != entry.st_dev
            or destination_stat.st_ino != entry.st_ino
            or destination_stat.st_size != entry.size
        ):
            identity_mismatches.append(rel)
        if (
            destination_stat.st_dev == entry.st_dev
            and destination_stat.st_ino == entry.st_ino
            and destination_stat.st_size == entry.size
        ):
            hardlink_count += 1
    if missing or extra or kind_mismatches or identity_mismatches:
        details = {
            "missing": missing[:8],
            "extra": extra[:8],
            "kind_mismatches": kind_mismatches[:8],
            "identity_mismatches": identity_mismatches[:8],
            "missing_count": len(missing),
            "extra_count": len(extra),
            "kind_mismatch_count": len(kind_mismatches),
            "identity_mismatch_count": len(identity_mismatches),
        }
        raise SensorViewError(f"destination tree is not exact: {json.dumps(details, sort_keys=True)}")
    return {
        "entries": len(found),
        "directories": sum(kind == "dir" for kind, _ in found.values()),
        "files": sum(kind == "file" for kind, _ in found.values()),
        "logical_file_bytes": logical_bytes,
        "hardlink_identity_exact_files": hardlink_count,
        "reparse_points": reparse_count,
    }


def _validate_allowed_paths(
    records: list[dict[str, Any]], destination: Path, *, enforce: bool
) -> None:
    if not enforce:
        return
    if _norm(destination) != _norm(EXPECTED_DESTINATION):
        raise SensorViewError(f"destination is outside the fixed allowed path: {destination}")
    if not _is_under(destination.parent, ALLOWED_DATASET_BASE):
        raise SensorViewError(f"destination parent is outside allowed dataset root: {destination.parent}")
    for record in records:
        source = Path(record["physical_root"]["path"])
        if not source.is_absolute() or not _is_under(source, ALLOWED_DATASET_BASE):
            raise SensorViewError(f"manifest physical root is outside allowed dataset root: {source}")
        if _is_under(destination, source) or _is_under(source, destination):
            raise SensorViewError(f"source/destination overlap: {source} vs {destination}")


def _same_volume(records: list[dict[str, Any]], destination_parent: Path) -> dict[str, Any]:
    parent_stat = os.stat(destination_parent, follow_symlinks=False)
    devices = {str(parent_stat.st_dev)}
    source_devices: dict[str, int] = {}
    for record in records:
        sequence = str(record["id"])
        source = Path(record["physical_root"]["path"])
        source_stat = _stat_directory(source, label=f"physical sequence {sequence}")
        source_devices[sequence] = source_stat.st_dev
        devices.add(str(source_stat.st_dev))
    if len(devices) != 1:
        raise SensorViewError(f"source/destination are not on one volume: {source_devices}, dest={parent_stat.st_dev}")
    return {"destination_parent_st_dev": parent_stat.st_dev, "source_st_dev": source_devices}


def _metadata_capacity(destination_parent: Path, file_count: int, directory_count: int) -> dict[str, Any]:
    usage = shutil.disk_usage(destination_parent)
    estimate = 1024 * 1024 + file_count * 4096 + directory_count * 1024
    required = estimate * 2
    if usage.free < required:
        raise SensorViewError(
            f"insufficient destination metadata headroom: free={usage.free}, required={required}"
        )
    return {
        "free_bytes_before": usage.free,
        "metadata_estimate_bytes": estimate,
        "required_headroom_bytes": required,
        "sufficient": True,
    }


def _stage_hardlinks(temp_root: Path, expected: dict[str, TreeEntry]) -> int:
    for rel, entry in sorted(expected.items()):
        destination = temp_root / Path(rel)
        if entry.kind == "dir":
            destination.mkdir(parents=True, exist_ok=True)
            continue
        destination.parent.mkdir(parents=True, exist_ok=True)
        try:
            os.link(entry.source, destination)
        except OSError as exc:
            raise SensorViewError(f"hardlink failed for {entry.source} -> {destination}: {exc}") from exc
    return sum(entry.kind == "file" for entry in expected.values())


def _camera_layout_checks(destination: Path, camera_results: list[dict[str, Any]]) -> None:
    for item in camera_results:
        camera_root = destination / item["sequence"] / "mav0" / item["camera"]
        csv_path = camera_root / "data.csv"
        data_dir = camera_root / "data"
        _stat_regular(csv_path, label="staged camera data.csv")
        _stat_directory(data_dir, label="staged camera data")
        png_count = sum(1 for path in data_dir.iterdir() if path.suffix.casefold() == ".png")
        if png_count != item["referenced_pngs"]:
            raise SensorViewError(f"staged PNG count mismatch: {camera_root}")


def create_sensor_view(
    manifest_path: Path,
    destination: Path,
    *,
    artifact_path: Path | None = None,
    enforce_allowed_paths: bool = True,
    strict_all11: bool = True,
) -> dict[str, Any]:
    started = time.perf_counter()
    manifest_path = manifest_path.resolve()
    destination = Path(destination)
    if not manifest_path.is_file():
        raise SensorViewError(f"manifest does not exist: {manifest_path}")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    records = _source_sequence_records(manifest, strict_all11=strict_all11)
    records_by_id = {str(record["id"]): record for record in records}
    _validate_allowed_paths(records, destination, enforce=enforce_allowed_paths)
    if destination.exists() and not destination.is_dir():
        raise SensorViewError(f"destination exists but is not a directory: {destination}")
    if not destination.parent.is_dir():
        raise SensorViewError(f"destination parent does not exist: {destination.parent}")
    volume = _same_volume(records, destination.parent)
    bound, expected_hashes = _manifest_bound_files(records, strict_all11=strict_all11)
    bound_results, sensor_bound_count, gt_bound_count = _verify_manifest_bound_sources(bound, records_by_id)
    expected, source_reparse_count = _build_expected_entries(records)
    camera_results = _validate_camera_pngs(records, expected)
    if not all(item["exact"] for item in camera_results):
        raise SensorViewError("camera PNG exactness check failed")
    capacity = _metadata_capacity(
        destination.parent,
        file_count=sum(entry.kind == "file" for entry in expected.values()),
        directory_count=sum(entry.kind == "dir" for entry in expected.values()),
    )
    scan_seconds = time.perf_counter() - started

    mode = "created"
    temp_root: Path | None = None
    try:
        if destination.exists():
            validation = _validate_tree(destination, expected, compare_identity=True)
            mode = "idempotent_existing"
        else:
            temp_root = Path(
                tempfile.mkdtemp(prefix=f".{destination.name}.tmp-", dir=os.fspath(destination.parent))
            )
            link_count = _stage_hardlinks(temp_root, expected)
            validation = _validate_tree(temp_root, expected, compare_identity=True)
            if validation["hardlink_identity_exact_files"] != link_count:
                raise SensorViewError("staged hardlink identity count mismatch before rename")
            if destination.exists():
                raise SensorViewError(f"destination appeared during staging; refusing overwrite: {destination}")
            os.replace(temp_root, destination)
            temp_root = None
            validation = _validate_tree(destination, expected, compare_identity=True)
    except Exception:
        if temp_root is not None and temp_root.exists():
            shutil.rmtree(temp_root)
        raise

    _camera_layout_checks(destination, camera_results)
    destination_stat = os.stat(destination, follow_symlinks=False)
    capacity_after = shutil.disk_usage(destination.parent)
    gt_destination_paths = [
        f"{item['sequence']}/{item['relative_path']}"
        for item in bound_results
        if item["role"] == "csv" and ("groundtruth" in item["relative_path"].casefold() or "ground_truth" in item["relative_path"].casefold())
    ]
    destination_found, destination_reparse = _scan_destination(destination)
    if destination_reparse or source_reparse_count:
        raise SensorViewError("reparse point count is nonzero")
    if any(path in destination_found for path in gt_destination_paths):
        raise SensorViewError("ground-truth bound path unexpectedly exists in destination")
    destination_paths = sorted(destination_found)
    gt_tokens = [path for path in destination_paths if _forbidden_rel(path)]
    if gt_tokens:
        raise SensorViewError(f"destination has forbidden GT/zip tokens: {gt_tokens[:3]}")
    hardlink_files = validation["hardlink_identity_exact_files"]
    expected_files = validation["files"]
    if hardlink_files != expected_files:
        raise SensorViewError(f"not all destination files are exact hardlinks: {hardlink_files}/{expected_files}")
    artifact = {
        "artifact": "work/m11_hardlink_sensor_view_creation_20260830.json",
        "captured_at": "2026-08-30",
        "status": "PASS",
        "mode": mode,
        "constraints": {
            "coordinator_touched": False,
            "existing_sensor_only_or_physical_data_modified": False,
            "network": False,
            "submodule_checkout": False,
            "data_bytes_copied": 0,
            "atomic_rename": mode == "created",
        },
        "paths": {
            "manifest": str(manifest_path),
            "manifest_sha256": _sha256(manifest_path),
            "dataset_base": str(ALLOWED_DATASET_BASE),
            "destination": str(destination),
            "destination_parent": str(destination.parent),
            "allowed_absolute_paths_checked": enforce_allowed_paths,
            "sequence_count": len(records),
            "sequence_ids": [str(record["id"]) for record in records],
        },
        "volume": volume,
        "metadata_capacity": {
            **capacity,
            "free_bytes_after": capacity_after.free,
            "headroom_after_sufficient": capacity_after.free >= capacity["required_headroom_bytes"],
        },
        "manifest_binding": {
            "bound_file_count": len(bound_results),
            "bound_source_files_present": sum(item["source_present"] for item in bound_results),
            "bound_source_hashes_exact": sum(item["source_sha256_match"] for item in bound_results),
            "sensor_bound_file_count": sensor_bound_count,
            "ground_truth_bound_file_count": gt_bound_count,
            "destination_sensor_bound_files": sensor_bound_count,
            "destination_ground_truth_bound_paths_absent": len(gt_destination_paths),
            "ground_truth_destination_paths": gt_destination_paths,
            "files": bound_results,
        },
        "tree": {
            "source_sensor_reparse_points": source_reparse_count,
            "destination_reparse_points": destination_reparse,
            "destination_gt_token_paths": len(gt_tokens),
            "destination_entries": validation["entries"],
            "destination_directories": validation["directories"],
            "destination_files": validation["files"],
            "destination_logical_file_bytes": validation["logical_file_bytes"],
            "hardlink_identity_exact_files": hardlink_files,
            "hardlink_identity_check": "os.stat(st_dev, st_ino, st_size) exact for every staged file",
            "extra_or_missing_entries": 0,
            "gt_zip_other_forbidden": True,
        },
        "camera_pngs": {
            "camera_count": len(camera_results),
            "all_camera_pngs_exact": all(item["exact"] for item in camera_results),
            "total_pngs": sum(item["source_pngs"] for item in camera_results),
            "total_staged_pngs": sum(item["staged_pngs"] for item in camera_results),
            "per_camera": camera_results,
        },
        "timing_seconds": {"preflight_scan": round(scan_seconds, 6)},
        "identity": {
            "destination_st_dev": destination_stat.st_dev,
            "destination_st_ino": destination_stat.st_ino,
            "source_data_bytes_referenced": sum(entry.size for entry in expected.values() if entry.kind == "file"),
            "data_bytes_copied": 0,
            "os_link_calls": expected_files if mode == "created" else 0,
        },
    }
    if artifact_path is not None:
        artifact_path.parent.mkdir(parents=True, exist_ok=True)
        artifact_path.write_text(json.dumps(artifact, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return artifact


# Backward-compatible public name used by the coordinator test fixtures.  The
# historical helper accepted one-sequence manifests, while the canonical CLI
# remains strict about the frozen all-11 manifest.
def create_view(
    manifest_path: Path,
    destination: Path,
    **kwargs: Any,
) -> dict[str, Any]:
    kwargs.setdefault("strict_all11", False)
    kwargs.setdefault("enforce_allowed_paths", False)
    result = create_sensor_view(manifest_path, destination, **kwargs)
    # Preserve the compact fields exposed by the old one-sequence helper;
    # canonical callers should use the nested schema returned above.
    result.setdefault("resumed", result.get("mode") == "idempotent_existing")
    paths = result.get("paths", {})
    binding = result.get("manifest_binding", {})
    tree = result.get("tree", {})
    identity = result.get("identity", {})
    result.setdefault("sequence_count", paths.get("sequence_count"))
    result.setdefault("sensor_file_count", tree.get("destination_files"))
    result.setdefault("manifest_bound_files", binding.get("bound_file_count"))
    result.setdefault("hardlink_identity_exact_count", tree.get("hardlink_identity_exact_files"))
    result.setdefault("sensor_bytes_copied", identity.get("data_bytes_copied"))
    result.setdefault("hash_exact_count", tree.get("hardlink_identity_exact_files"))
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--destination", type=Path, default=EXPECTED_DESTINATION)
    parser.add_argument("--artifact", type=Path, default=DEFAULT_ARTIFACT)
    args = parser.parse_args()
    try:
        artifact = create_sensor_view(
            args.manifest,
            args.destination,
            artifact_path=args.artifact,
            enforce_allowed_paths=True,
            strict_all11=True,
        )
    except (OSError, SensorViewError, json.JSONDecodeError) as exc:
        print(f"sensor-only hardlink view failed: {exc}")
        return 1
    print(
        f"sensor-only hardlink view {artifact['mode']}: {artifact['paths']['destination']} "
        f"({artifact['tree']['destination_files']} files, {artifact['camera_pngs']['total_pngs']} PNGs, "
        f"data_bytes_copied={artifact['identity']['data_bytes_copied']})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
