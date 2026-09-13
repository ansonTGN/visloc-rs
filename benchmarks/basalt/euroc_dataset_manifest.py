#!/usr/bin/env python3
"""Build and validate the frozen, sensor-auditable EuRoC input manifest.

The manifest deliberately records ground-truth metadata for the *post-run*
evaluator, but the ground-truth firewall says that an estimator receives only
the sensor paths named by the parity protocol.  This module has no dependency
on the estimator and performs all checks from the bytes on the local machine.

The default paths are the machine-local bindings used by the parity runs:
``E:\\datasets\\euroc_mav`` and ``E:\\datasets\\euroc_mav\\all11``.  They are
metadata, not portable dataset paths.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import shutil
import subprocess
import sys
import zipfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any, Iterable, Sequence


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_PROTOCOL = ROOT / "benchmarks" / "basalt" / "protocols" / "basalt_euroc_parity_v1.json"
DEFAULT_OUTPUT = ROOT / "benchmarks" / "basalt" / "euroc_dataset_manifest.json"
DEFAULT_DATASET_BASE = Path(r"E:\datasets\euroc_mav")
SCHEMA_VERSION = 1
SCHEMA_ID = "basalt.euroc_dataset_manifest.v1"
MANIFEST_ID = "basalt-euroc-dataset-freeze-v1"
PROTOCOL_ID = "basalt-euroc-parity-v1"
ARCHIVE_WORKERS = 4

SEQUENCES: tuple[str, ...] = (
    "MH_01_easy",
    "MH_02_easy",
    "MH_03_medium",
    "MH_04_difficult",
    "MH_05_difficult",
    "V1_01_easy",
    "V1_02_medium",
    "V1_03_difficult",
    "V2_01_easy",
    "V2_02_medium",
    "V2_03_difficult",
)
MACHINE_HALL = frozenset(SEQUENCES[:5])
VICON_ROOM = frozenset(SEQUENCES[5:])
REQUIRED_CSVS: tuple[str, ...] = (
    "mav0/cam0/data.csv",
    "mav0/cam1/data.csv",
    "mav0/imu0/data.csv",
    "mav0/state_groundtruth_estimate0/data.csv",
)
REQUIRED_SENSOR_YAMLS: tuple[str, ...] = (
    "mav0/cam0/sensor.yaml",
    "mav0/cam1/sensor.yaml",
    "mav0/imu0/sensor.yaml",
)

# These values are copied only as official source metadata.  Archive hashes,
# byte counts, row counts, timestamps, and local outer-file checks below are
# always recomputed from the local files by build/validate.
OFFICIAL_BITSTREAMS: tuple[dict[str, Any], ...] = (
    {
        "id": "7b2419c1-62b5-4714-b7f8-485e5fe3e5fe",
        "name": "machine_hall.zip",
        "size_bytes": 12683729426,
        "md5": "363f5c2502b469cdd97ef85997714806",
        "family": "machine_hall",
        "local_outer_role": "not_bound",
    },
    {
        "id": "02ecda9a-298f-498b-970c-b7c44334d880",
        "name": "vicon_room1.zip",
        "size_bytes": 6042263426,
        "md5": "5ce06b405827e453a82523d3ca9c2fd0",
        "family": "vicon_room1",
        "local_outer_role": "exact_local_outer",
    },
    {
        "id": "ea12bc01-3677-4b4c-853d-87c7870b8c44",
        "name": "vicon_room2.zip",
        "size_bytes": 6013384949,
        "md5": "c6347f4e0476aaa9a43a919c163c49c5",
        "family": "vicon_room2",
        "local_outer_role": "rejected_local_outer",
        "rejected_local_md5": "f332544ffdd89b742db02877ce566fd2",
    },
)


class ManifestError(ValueError):
    """Raised when a dataset or manifest violates the freeze contract."""


def _sha256_file(path: Path, *, chunk_size: int = 8 * 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            block = stream.read(chunk_size)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def _md5_file(path: Path, *, chunk_size: int = 8 * 1024 * 1024) -> str:
    digest = hashlib.md5()
    with path.open("rb") as stream:
        while True:
            block = stream.read(chunk_size)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def _json_sha256(path: Path) -> str:
    return _sha256_file(path) if path.is_file() else ""


def _absolute(path: Path) -> Path:
    return path.expanduser().resolve()


def _path_string(path: Path) -> str:
    # Keep native Windows paths in the machine-local manifest.  A POSIX host
    # still gets deterministic native paths for its own supplied root.
    return str(_absolute(path))


def _normalise_dataset_roots(dataset_root: Path) -> tuple[Path, Path]:
    """Return ``(dataset_base, unified_root)`` for either accepted CLI form."""

    supplied = _absolute(dataset_root)
    if supplied.name.casefold() == "all11":
        return supplied.parent, supplied
    return supplied, supplied / "all11"


def _load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ManifestError(f"cannot read JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ManifestError(f"JSON root must be an object: {path}")
    return value


def _load_protocol(path: Path) -> dict[str, Any]:
    protocol = _load_json(path)
    if protocol.get("protocol_id") != PROTOCOL_ID:
        raise ManifestError(f"protocol id mismatch: {path}")
    dataset = protocol.get("dataset")
    if not isinstance(dataset, dict):
        raise ManifestError("protocol has no dataset section")
    if dataset.get("ground_truth_available_to_runner") is not False:
        raise ManifestError("protocol must declare ground_truth_available_to_runner=false")
    if dataset.get("ground_truth_materialized_before_engine_exit") is not False:
        raise ManifestError("protocol must prohibit pre-exit ground truth materialization")
    execution = protocol.get("execution_policy")
    if not isinstance(execution, dict) or execution.get("evaluation_must_be_a_separate_process") is not True:
        raise ManifestError("protocol must require separate-process evaluation")
    sequences = protocol.get("sequences")
    if not isinstance(sequences, list) or tuple(sequences) != SEQUENCES:
        raise ManifestError(f"protocol sequence set/order is not frozen EuRoC all11: {path}")
    return protocol


def _load_upstream() -> dict[str, Any]:
    protocol = _load_protocol(DEFAULT_PROTOCOL)
    upstream = dict(protocol["upstream"])
    port_path = ROOT / "benchmarks" / "basalt" / "basalt_port_manifest_v1.json"
    if port_path.is_file():
        port = _load_json(port_path)
        port_upstream = port.get("upstream", {})
        if isinstance(port_upstream, dict):
            upstream["tree_sha1"] = port_upstream.get("tree_sha1")
            for key in ("config", "calibration"):
                if isinstance(port_upstream.get(key), dict):
                    upstream[key] = dict(port_upstream[key])
        upstream["port_manifest"] = {
            "path": "benchmarks/basalt/basalt_port_manifest_v1.json",
            "sha256": _json_sha256(port_path),
        }
    return upstream


def _require_file(path: Path, description: str) -> Path:
    if not path.is_file():
        raise ManifestError(f"missing {description}: {path}")
    return path


def _timestamp_rows(path: Path, *, minimum_columns: int, label: str) -> tuple[list[int], int]:
    """Parse one EuRoC CSV and return timestamps plus data-row count."""

    _require_file(path, label)
    timestamps: list[int] = []
    row_count = 0
    try:
        with path.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.reader(stream)
            try:
                header = next(reader)
            except StopIteration as exc:
                raise ManifestError(f"empty CSV: {path}") from exc
            if not header or not header[0].lstrip().startswith("#timestamp"):
                raise ManifestError(f"unexpected timestamp header in {path}: {header!r}")
            for line_number, row in enumerate(reader, start=2):
                if not row or not any(cell.strip() for cell in row):
                    continue
                if len(row) < minimum_columns:
                    raise ManifestError(
                        f"short row in {path} at line {line_number}: {len(row)} < {minimum_columns}"
                    )
                try:
                    timestamp = int(row[0].strip())
                except ValueError as exc:
                    raise ManifestError(f"non-integer timestamp in {path} at line {line_number}") from exc
                timestamps.append(timestamp)
                row_count += 1
    except UnicodeDecodeError as exc:
        raise ManifestError(f"CSV is not UTF-8: {path}") from exc
    if not timestamps:
        raise ManifestError(f"CSV has no data rows: {path}")
    if any(current <= previous for previous, current in zip(timestamps, timestamps[1:])):
        raise ManifestError(f"timestamps are not strictly increasing: {path}")
    return timestamps, row_count


def _timestamp_record(
    path: Path,
    timestamps: Sequence[int],
    row_count: int,
    *,
    display_path: Path | None = None,
) -> dict[str, Any]:
    first_ns = int(timestamps[0])
    last_ns = int(timestamps[-1])
    return {
        "path": (display_path if display_path is not None else path).as_posix(),
        "bytes": path.stat().st_size,
        "sha256": _sha256_file(path),
        "row_count": row_count,
        "first_timestamp_ns": first_ns,
        "last_timestamp_ns": last_ns,
        "timestamp_range_ns": {"first": first_ns, "last": last_ns},
    }


def _camera_record(sequence_root: Path, camera: str) -> tuple[dict[str, Any], list[int]]:
    camera_root = sequence_root / "mav0" / camera
    csv_path = _require_file(camera_root / "data.csv", f"{camera} CSV")
    sensor_path = _require_file(camera_root / "sensor.yaml", f"{camera} sensor.yaml")
    timestamps, row_count = _timestamp_rows(csv_path, minimum_columns=2, label=f"{camera} CSV")
    references: list[str] = []
    with csv_path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.reader(stream)
        next(reader, None)
        for row in reader:
            if row and any(cell.strip() for cell in row):
                if len(row) < 2:
                    raise ManifestError(f"missing image filename in {csv_path}")
                references.append(row[1].strip())
    if len(references) != row_count or len(set(references)) != row_count:
        raise ManifestError(f"camera CSV has duplicate/missing image references: {csv_path}")
    png_files = sorted(
        item for item in (camera_root / "data").iterdir()
        if item.is_file() and item.suffix.casefold() == ".png"
    ) if (camera_root / "data").is_dir() else []
    png_names = {item.name for item in png_files}
    if len(png_names) != len(png_files):
        raise ManifestError(f"duplicate camera image names: {camera_root / 'data'}")
    if set(references) != png_names:
        missing = sorted(set(references) - png_names)
        extra = sorted(png_names - set(references))
        raise ManifestError(f"{camera} CSV/image mismatch; missing={missing[:3]} extra={extra[:3]}")
    record = _timestamp_record(
        csv_path,
        timestamps,
        row_count,
        display_path=csv_path.relative_to(sequence_root),
    )
    record.update(
        {
            "camera": camera,
            "png_count": len(png_files),
            "csv_row_count": row_count,
            "sensor_yaml": {
                "path": sensor_path.relative_to(sequence_root).as_posix(),
                "bytes": sensor_path.stat().st_size,
                "sha256": _sha256_file(sensor_path),
            },
            "image_reference_count": len(references),
        }
    )
    return record, timestamps


def _archive_record(path: Path, *, check_crc: bool) -> dict[str, Any]:
    _require_file(path, "sequence archive")
    crc_status = "skipped"
    bad_entry: str | None = None
    crc_checker = "not_run"
    crc_exit_code: int | None = None
    entry_count = 0
    try:
        # ``unzip -t`` uses the platform's mature streaming ZIP checker and is
        # substantially faster on the multi-gigabyte EuRoC image archives.
        # The standard-library path remains the portable fallback and is used
        # by focused tests/minimal hosts without an unzip executable.
        with zipfile.ZipFile(path, "r") as archive:
            infos = archive.infolist()
            entry_count = len(infos)
            if not infos:
                raise ManifestError(f"archive has no entries: {path}")
        if check_crc:
            unzip = shutil.which("unzip")
            if unzip:
                crc_checker = "unzip"
                completed = subprocess.run(
                    [unzip, "-t", "-q", str(path)],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    check=False,
                    text=True,
                )
                crc_exit_code = completed.returncode
                if completed.returncode != 0:
                    detail = (completed.stderr or completed.stdout).strip()
                    raise ManifestError(
                        f"ZIP CRC mismatch in {path}: checker=unzip "
                        f"exit_code={completed.returncode} {detail[:240]}"
                    )
                crc_status = "passed"
            else:
                crc_checker = "python_zipfile"
                crc_exit_code = 0
                with zipfile.ZipFile(path, "r") as archive:
                    bad_entry = archive.testzip()
                crc_status = "passed" if bad_entry is None else "failed"
    except (OSError, zipfile.BadZipFile, zipfile.LargeZipFile) as exc:
        raise ManifestError(f"cannot inspect ZIP archive {path}: {exc}") from exc
    if check_crc and bad_entry is not None:
        raise ManifestError(f"ZIP CRC mismatch in {path}: {bad_entry}")
    return {
        "path": _path_string(path),
        "machine_local": True,
        "bytes": path.stat().st_size,
        "sha256": _sha256_file(path),
        "format": "zip",
        "entry_count": entry_count,
        "crc_check": {
            "status": crc_status,
            "bad_entry": bad_entry,
            "checker": crc_checker,
            "exit_code": crc_exit_code,
        },
    }


def _archive_path(base: Path, sequence: str) -> tuple[Path, str]:
    if sequence in MACHINE_HALL:
        return base / "machine_hall" / sequence / f"{sequence}.zip", "machine_hall_sequence_zip"
    if sequence in VICON_ROOM:
        return (
            base / "vicon_room" / "sequence_archives_verified" / f"{sequence}.zip",
            "vicon_verified_sequence_zip",
        )
    raise ManifestError(f"unknown protocol sequence: {sequence}")


def _local_binding(path: Path, *, role: str, expected_md5: str | None = None) -> dict[str, Any]:
    result: dict[str, Any] = {
        "path": _path_string(path),
        "machine_local": True,
        "portable": False,
        "role": role,
    }
    if not path.is_file():
        result.update({"status": "not_present", "bound": False})
        return result
    actual_md5 = _md5_file(path)
    result.update(
        {
            "status": "present",
            "bound": role == "exact_local_outer" and expected_md5 == actual_md5,
            "bytes": path.stat().st_size,
            "md5": actual_md5,
        }
    )
    if expected_md5 is not None:
        result["matches_official_md5"] = actual_md5 == expected_md5
    return result


def _official_source(base: Path) -> dict[str, Any]:
    vicon_root = base / "vicon_room"
    snapshot: dict[str, Any] = {}
    for relative in ("work/ethz_item.json", "work/ethz_original_bs.json"):
        path = ROOT / relative
        snapshot[relative] = {"path": relative, "sha256": _json_sha256(path)}
    bitstreams: list[dict[str, Any]] = []
    for item in OFFICIAL_BITSTREAMS:
        record = dict(item)
        role = str(item["local_outer_role"])
        if item["family"] == "vicon_room1":
            local_path = vicon_root / "vicon_room1.zip.assembled.part"
            record["local_binding"] = _local_binding(
                local_path, role=role, expected_md5=str(item["md5"])
            )
        elif item["family"] == "vicon_room2":
            local_path = vicon_root / "vicon_room2.zip.assembled.part"
            binding = _local_binding(local_path, role=role, expected_md5=str(item["md5"]))
            if local_path.is_file():
                binding["matches_rejected_local_md5"] = binding.get("md5") == item["rejected_local_md5"]
                # This outer assembly is intentionally never used as an input
                # archive; only verified per-sequence zips are bound below.
                binding["bound"] = False
                binding["status"] = "rejected_local_assembly"
            record["local_binding"] = binding
            record["rejection"] = {
                "status": "rejected",
                "reason": "local assembled outer MD5 differs from the official bitstream MD5",
                "expected_local_assembled_md5": item["rejected_local_md5"],
                "never_bind_as_sequence_archive": True,
            }
        else:
            machine_outer = base / "machine_hall.zip"
            record["local_binding"] = _local_binding(machine_outer, role=role)
            record["rejection"] = {
                "status": "not_bound",
                "reason": "freeze binds the five verified per-sequence MH zips, not an unmaterialized outer file",
            }
        bitstreams.append(record)
    return {
        "dataset_name": "EuRoC MAV official ASL release",
        "doi": "10.3929/ethz-b-000690084",
        "doi_url": "https://doi.org/10.3929/ethz-b-000690084",
        "ethz_handle": "20.500.11850/690084",
        "item_title": "The EuRoC micro aerial vehicle datasets",
        "publisher": "ETH Zurich",
        "metadata_snapshots": snapshot,
        "official_outer_bitstreams": bitstreams,
    }


def _sequence_record(
    sequence: str,
    *,
    base: Path,
    unified_root: Path,
    check_archive_crc: bool,
) -> dict[str, Any]:
    family = "machine_hall" if sequence in MACHINE_HALL else "vicon_room"
    if family == "machine_hall":
        physical = base / "machine_hall" / sequence
    else:
        physical = base / "vicon_room" / "sequences" / sequence
    unified = unified_root / sequence
    if not physical.is_dir():
        raise ManifestError(f"missing physical sequence root: {physical}")
    if not unified.is_dir():
        raise ManifestError(f"missing unified sequence binding: {unified}")
    physical = _absolute(physical)
    unified = _absolute(unified)
    # Read from the unified path, which is the runner's binding.  The physical
    # path is retained for archive provenance and an explicit comparison.
    cam0, cam0_ts = _camera_record(unified, "cam0")
    cam1, cam1_ts = _camera_record(unified, "cam1")
    imu_path = _require_file(unified / "mav0" / "imu0" / "data.csv", "IMU CSV")
    imu_ts, imu_rows = _timestamp_rows(imu_path, minimum_columns=7, label="IMU CSV")
    gt_path = _require_file(
        unified / "mav0" / "state_groundtruth_estimate0" / "data.csv", "ground-truth CSV"
    )
    gt_ts, gt_rows = _timestamp_rows(gt_path, minimum_columns=2, label="ground-truth CSV")
    archive_path, archive_binding = _archive_path(base, sequence)
    archive = _archive_record(archive_path, check_crc=check_archive_crc)

    intersection = sorted(set(cam0_ts).intersection(cam1_ts))
    intersection_bytes = ("\n".join(str(value) for value in intersection) + "\n").encode("ascii")
    intersection_hash = hashlib.sha256(intersection_bytes).hexdigest()
    cam0_only = sorted(set(cam0_ts).difference(cam1_ts))
    cam1_only = sorted(set(cam1_ts).difference(cam0_ts))
    imu_record = _timestamp_record(
        imu_path,
        imu_ts,
        imu_rows,
        display_path=imu_path.relative_to(unified),
    )
    gt_record = _timestamp_record(
        gt_path,
        gt_ts,
        gt_rows,
        display_path=gt_path.relative_to(unified),
    )
    yaml_hashes = {
        relative: _sha256_file(unified / relative)
        for relative in REQUIRED_SENSOR_YAMLS
    }
    csv_hashes = {
        relative: _sha256_file(unified / relative)
        for relative in REQUIRED_CSVS
    }
    archive_crc_passed = archive["crc_check"]["status"] == "passed"
    # A diagnostic build still performs all structural, timestamp, count, and
    # byte-hash checks.  It deliberately records the CRC as *not checked*,
    # rather than treating a skipped check as a pass.  The full freeze path
    # below still requires ``archive_crc_passed``.
    result_checks = {
        "physical_root_exists": physical.is_dir(),
        "unified_root_exists": unified.is_dir(),
        "required_sensor_paths": True,
        "camera_csv_image_references_exact": True,
        "camera_timestamps_strictly_increasing": True,
        "imu_timestamps_strictly_increasing": True,
        "ground_truth_timestamps_strictly_increasing": True,
        "archive_zip_readable": archive["crc_check"]["status"] in {"passed", "skipped"},
        "archive_zip_crc_checked": check_archive_crc,
        "archive_zip_crc": archive_crc_passed,
        "archive_is_bound_sequence_zip": archive_binding in {
            "machine_hall_sequence_zip",
            "vicon_verified_sequence_zip",
        },
    }
    required_checks = {
        key: value
        for key, value in result_checks.items()
        if key not in {"archive_zip_crc_checked", "archive_zip_crc"}
    }
    if check_archive_crc:
        required_checks["archive_zip_crc"] = archive_crc_passed
    integrity_passed = all(required_checks.values())
    if not integrity_passed:
        raise ManifestError(f"integrity checks failed for {sequence}: {result_checks}")
    if check_archive_crc:
        freeze_note = (
            "prior frozen valid sequence zip; CRC passed"
            if sequence == "V2_02_medium"
            else "verified sequence archive; entry CRC passed"
        )
        archive_rule = "bound sequence archive is a readable ZIP with passing entry CRC"
    else:
        freeze_note = "diagnostic archive inspection; entry CRC not checked"
        archive_rule = "bound sequence archive is a readable ZIP; entry CRC not checked (diagnostic only)"
    return {
        "id": sequence,
        "family": family,
        "physical_root": {
            "path": _path_string(physical),
            "machine_local": True,
            "portable": False,
            "binding": "physical_sequence_root",
        },
        "unified_root": {
            "path": _path_string(unified),
            "machine_local": True,
            "portable": False,
            "binding": "all11_junction_or_local_binding",
            "resolved_path": _path_string(unified),
            "resolves_to_physical_root": unified.resolve() == physical.resolve(),
        },
        "archive": {
            **archive,
            "binding": archive_binding,
            "freeze_note": freeze_note,
        },
        # Direct aliases make the frozen bytes/hash contract easy to consume
        # without requiring callers to know the nested archive representation.
        "archive_bytes": archive["bytes"],
        "archive_sha256": archive["sha256"],
        "cameras": {"cam0": cam0, "cam1": cam1},
        "camera_counts": {
            "cam0": {"png": cam0["png_count"], "csv_rows": cam0["csv_row_count"]},
            "cam1": {"png": cam1["png_count"], "csv_rows": cam1["csv_row_count"]},
        },
        "imu": imu_record,
        "ground_truth": {
            **gt_record,
            "available_to_post_exit_evaluator": True,
            "available_to_engine": False,
        },
        "imu_row_count": imu_rows,
        "imu_first_timestamp_ns": imu_ts[0],
        "imu_last_timestamp_ns": imu_ts[-1],
        "ground_truth_row_count": gt_rows,
        "ground_truth_first_timestamp_ns": gt_ts[0],
        "ground_truth_last_timestamp_ns": gt_ts[-1],
        "stereo": {
            "timestamp_intersection_ns": intersection,
            "timestamp_intersection_count": len(intersection),
            "timestamp_intersection_first_ns": intersection[0] if intersection else None,
            "timestamp_intersection_last_ns": intersection[-1] if intersection else None,
            "timestamp_intersection_sha256": intersection_hash,
            "cam0_only_count": len(cam0_only),
            "cam1_only_count": len(cam1_only),
            "stereo_pair_count": len(intersection),
            "exact": True,
        },
        "file_hashes": {
            "csv_sha256": csv_hashes,
            "sensor_yaml_sha256": yaml_hashes,
        },
        "integrity": {
            "passed": integrity_passed,
            "archive_crc_checked": check_archive_crc,
            "rules": [
                "physical and unified sequence roots exist",
                "all protocol sensor CSV/YAML paths exist",
                "camera CSV rows and PNG names match exactly",
                "all sensor and GT timestamps are strictly increasing",
                "camera stereo pairs are the exact timestamp-set intersection",
                archive_rule,
            ],
            "results": result_checks,
        },
    }


def _parallel_sequence_records(
    *,
    base: Path,
    unified_root: Path,
    check_archive_crc: bool,
) -> list[dict[str, Any]]:
    """Recompute archives in bounded parallelism, retaining protocol order.

    Archive SHA-256 and CRC verification are both streaming operations.  A
    fixed worker bound avoids an unbounded process/thread fan-out while
    ``executor.map``'s ordered result contract keeps the manifest byte-stable
    for a given set of inputs.
    """

    worker_count = min(ARCHIVE_WORKERS, len(SEQUENCES))

    def recompute(sequence: str) -> dict[str, Any]:
        return _sequence_record(
            sequence,
            base=base,
            unified_root=unified_root,
            check_archive_crc=check_archive_crc,
        )

    with ThreadPoolExecutor(max_workers=worker_count, thread_name_prefix="euroc-manifest") as executor:
        return list(executor.map(recompute, SEQUENCES))


def _build_manifest(
    dataset_root: Path,
    *,
    protocol_path: Path = DEFAULT_PROTOCOL,
    check_archive_crc: bool = True,
) -> dict[str, Any]:
    base, unified = _normalise_dataset_roots(dataset_root)
    protocol = _load_protocol(_absolute(protocol_path))
    sequences = _parallel_sequence_records(
        base=base,
        unified_root=unified,
        check_archive_crc=check_archive_crc,
    )
    sequence_results = {sequence["id"]: sequence["integrity"]["passed"] for sequence in sequences}
    return {
        "schema_version": SCHEMA_VERSION,
        "schema_id": SCHEMA_ID,
        "manifest_id": MANIFEST_ID,
        "protocol_id": PROTOCOL_ID,
        "status": "frozen" if check_archive_crc else "unverified_archive_crc",
        "purpose": "M0 deterministic EuRoC dataset freeze for GT-free Basalt parity runs",
        "upstream": _load_upstream(),
        "protocol": {
            "path": "benchmarks/basalt/protocols/basalt_euroc_parity_v1.json",
            "sha256": _sha256_file(_absolute(protocol_path)),
            "id": PROTOCOL_ID,
            "sequence_order": list(SEQUENCES),
        },
        "source": _official_source(base),
        "roots": {
            "dataset_base": {
                "path": _path_string(base),
                "machine_local": True,
                "portable": False,
                "binding": "machine_local_dataset_base",
            },
            "unified": {
                "path": _path_string(unified),
                "machine_local": True,
                "portable": False,
                "binding": "all11_sequence_junction_tree",
            },
            "physical_machine_hall": {
                "path": _path_string(base / "machine_hall"),
                "machine_local": True,
                "portable": False,
                "binding": "physical_sequence_roots",
            },
            "physical_vicon_sequences": {
                "path": _path_string(base / "vicon_room" / "sequences"),
                "machine_local": True,
                "portable": False,
                "binding": "physical_sequence_roots",
            },
            "verified_vicon_archives": {
                "path": _path_string(base / "vicon_room" / "sequence_archives_verified"),
                "machine_local": True,
                "portable": False,
                "binding": "verified_sequence_archive_store",
            },
        },
        "sequences": sequences,
        "integrity": {
            "rule_set": "basalt-euroc-dataset-freeze-v1",
            "passed": all(sequence_results.values()),
            "archive_crc_checked": check_archive_crc,
            "protocol_sequence_confirmation": {
                "expected_count": len(SEQUENCES),
                "manifest_count": len(sequences),
                "all_protocol_sequences_present": tuple(item["id"] for item in sequences) == SEQUENCES,
                "sequence_results": sequence_results,
            },
            "rules": [
                "protocol id and ordered all11 sequence set are frozen",
                "every archive byte count and SHA256 is recomputed from the local archive",
                (
                    "every bound archive has passing ZIP entry CRC"
                    if check_archive_crc
                    else "bound archives are readable ZIPs; entry CRC was not checked (diagnostic only)"
                ),
                "camera PNG/CSV counts, references, and timestamp ranges are recomputed",
                "IMU and GT row counts/timestamp ranges are recomputed",
                "all four CSV and three sensor.yaml SHA256 values are recomputed",
                "camera timestamp intersection and stereo pair count are exact",
            ],
        },
        "ground_truth_firewall": {
            "declaration": "GT is audit metadata only; estimator processes receive no GT path or bytes.",
            "ground_truth_available_to_engine": False,
            "ground_truth_path_passed_to_engine": False,
            "ground_truth_environment_keys_passed": False,
            "staged_input_excludes_ground_truth": True,
            "manifest_written_after_engine_exit": True,
            "evaluation_must_be_a_separate_process": True,
            "dataset_manifest_contains_gt_metadata_for_post_exit_evaluation": True,
            "protocol_ground_truth_available_to_runner": False,
        },
    }


def _write_json(path: Path, value: dict[str, Any]) -> None:
    path = _absolute(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8", newline="\n")


def build_manifest(
    dataset_root: Path = DEFAULT_DATASET_BASE,
    *,
    output: Path = DEFAULT_OUTPUT,
    protocol_path: Path = DEFAULT_PROTOCOL,
    check_archive_crc: bool = True,
) -> dict[str, Any]:
    """Recompute and write the deterministic manifest, returning its object."""

    manifest = _build_manifest(
        dataset_root,
        protocol_path=protocol_path,
        check_archive_crc=check_archive_crc,
    )
    _write_json(output, manifest)
    return manifest


def _expect_equal(label: str, actual: Any, expected: Any, errors: list[str]) -> None:
    if actual != expected:
        errors.append(f"{label}: manifest={expected!r}, recomputed={actual!r}")


def _validate_sequence(manifest_sequence: dict[str, Any], recomputed: dict[str, Any], errors: list[str]) -> None:
    sequence = manifest_sequence.get("id")
    for key in ("id", "family", "archive_bytes", "archive_sha256", "camera_counts", "imu_row_count", "ground_truth_row_count"):
        _expect_equal(f"{sequence}.{key}", recomputed.get(key), manifest_sequence.get(key), errors)
    for key in ("imu_first_timestamp_ns", "imu_last_timestamp_ns", "ground_truth_first_timestamp_ns", "ground_truth_last_timestamp_ns"):
        _expect_equal(f"{sequence}.{key}", recomputed.get(key), manifest_sequence.get(key), errors)
    _expect_equal(f"{sequence}.stereo", recomputed.get("stereo"), manifest_sequence.get("stereo"), errors)
    _expect_equal(f"{sequence}.file_hashes", recomputed.get("file_hashes"), manifest_sequence.get("file_hashes"), errors)
    for camera in ("cam0", "cam1"):
        _expect_equal(
            f"{sequence}.cameras.{camera}",
            recomputed.get("cameras", {}).get(camera),
            manifest_sequence.get("cameras", {}).get(camera),
            errors,
        )
    manifest_archive = manifest_sequence.get("archive", {})
    recomputed_archive = recomputed.get("archive", {})
    for key in ("bytes", "sha256", "entry_count", "binding"):
        _expect_equal(
            f"{sequence}.archive.{key}",
            recomputed_archive.get(key),
            manifest_archive.get(key),
            errors,
        )
    _expect_equal(
        f"{sequence}.archive.crc_check",
        recomputed_archive.get("crc_check"),
        manifest_archive.get("crc_check"),
        errors,
    )
    _expect_equal(
        f"{sequence}.integrity.archive_crc_checked",
        recomputed.get("integrity", {}).get("archive_crc_checked"),
        manifest_sequence.get("integrity", {}).get("archive_crc_checked"),
        errors,
    )
    recomputed_results = recomputed.get("integrity", {}).get("results", {})
    manifest_results = manifest_sequence.get("integrity", {}).get("results", {})
    for key in (
        "archive_zip_readable",
        "archive_zip_crc_checked",
        "archive_zip_crc",
        "archive_is_bound_sequence_zip",
    ):
        _expect_equal(
            f"{sequence}.integrity.results.{key}",
            recomputed_results.get(key),
            manifest_results.get(key),
            errors,
        )
    if manifest_sequence.get("integrity", {}).get("passed") is not True:
        errors.append(f"{sequence}.integrity.passed is not true")


def validate_manifest(
    manifest_path: Path = DEFAULT_OUTPUT,
    *,
    dataset_root: Path | None = None,
    protocol_path: Path = DEFAULT_PROTOCOL,
    check_archive_crc: bool = True,
) -> dict[str, Any]:
    """Recompute all frozen fields and raise :class:`ManifestError` on drift."""

    manifest_path = _absolute(manifest_path)
    manifest = _load_json(manifest_path)
    errors: list[str] = []
    _expect_equal("schema_version", manifest.get("schema_version"), SCHEMA_VERSION, errors)
    _expect_equal("schema_id", manifest.get("schema_id"), SCHEMA_ID, errors)
    _expect_equal("manifest_id", manifest.get("manifest_id"), MANIFEST_ID, errors)
    _expect_equal("protocol_id", manifest.get("protocol_id"), PROTOCOL_ID, errors)
    if manifest.get("status") != "frozen":
        errors.append(f"status must be frozen, got {manifest.get('status')!r}")
    try:
        protocol = _load_protocol(_absolute(protocol_path))
    except ManifestError as exc:
        errors.append(str(exc))
        protocol = {}
    if protocol:
        protocol_record = manifest.get("protocol", {})
        _expect_equal("protocol.id", protocol_record.get("id"), PROTOCOL_ID, errors)
        _expect_equal("protocol.sequence_order", protocol_record.get("sequence_order"), list(SEQUENCES), errors)
        _expect_equal(
            "protocol.sha256",
            protocol_record.get("sha256"),
            _sha256_file(_absolute(protocol_path)),
            errors,
        )
    base, unified = _normalise_dataset_roots(
        dataset_root if dataset_root is not None else Path(manifest.get("roots", {}).get("dataset_base", {}).get("path", ""))
    )
    manifest_sequences = manifest.get("sequences")
    if not isinstance(manifest_sequences, list):
        errors.append("sequences must be an array")
        manifest_sequences = []
    by_id = {item.get("id"): item for item in manifest_sequences if isinstance(item, dict)}
    _expect_equal("sequence count", len(manifest_sequences), len(SEQUENCES), errors)
    _expect_equal("sequence ids", tuple(by_id), SEQUENCES, errors)
    present_sequences = [sequence for sequence in SEQUENCES if sequence in by_id]
    with ThreadPoolExecutor(
        max_workers=min(ARCHIVE_WORKERS, len(present_sequences) or 1),
        thread_name_prefix="euroc-validate",
    ) as executor:
        futures = {
            sequence: executor.submit(
                _sequence_record,
                sequence,
                base=base,
                unified_root=unified,
                check_archive_crc=check_archive_crc,
            )
            for sequence in present_sequences
        }
        # Consume futures in frozen protocol order, even though the archive
        # streams complete independently.  This keeps diagnostics stable and
        # avoids returning a validator result whose ordering depends on I/O.
        for sequence in SEQUENCES:
            if sequence not in futures:
                errors.append(f"missing protocol sequence: {sequence}")
                continue
            try:
                _validate_sequence(by_id[sequence], futures[sequence].result(), errors)
            except ManifestError as exc:
                errors.append(f"{sequence}: {exc}")
    firewall = manifest.get("ground_truth_firewall", {})
    firewall_false = (
        "ground_truth_available_to_engine",
        "ground_truth_path_passed_to_engine",
        "ground_truth_environment_keys_passed",
    )
    for key in firewall_false:
        if firewall.get(key) is not False:
            errors.append(f"ground_truth_firewall.{key} must be false")
    for key in ("staged_input_excludes_ground_truth", "manifest_written_after_engine_exit", "evaluation_must_be_a_separate_process"):
        if firewall.get(key) is not True:
            errors.append(f"ground_truth_firewall.{key} must be true")
    integrity = manifest.get("integrity", {})
    if integrity.get("passed") is not True:
        errors.append("integrity.passed is not true")
    protocol_confirmation = integrity.get("protocol_sequence_confirmation", {})
    if protocol_confirmation.get("all_protocol_sequences_present") is not True:
        errors.append("integrity does not confirm all protocol sequences")
    _expect_equal(
        "integrity.archive_crc_checked",
        check_archive_crc,
        integrity.get("archive_crc_checked"),
        errors,
    )
    if errors:
        raise ManifestError("manifest validation failed:\n- " + "\n- ".join(errors))
    return {
        "status": "valid",
        "schema_id": SCHEMA_ID,
        "manifest": str(manifest_path),
        "sequence_count": len(SEQUENCES),
        "archive_crc_checked": check_archive_crc,
        "ground_truth_firewall": "pass",
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    build = subparsers.add_parser("build", help="recompute and write the frozen manifest")
    build.add_argument(
        "--dataset-root",
        "--dataset-base",
        dest="dataset_root",
        type=Path,
        default=DEFAULT_DATASET_BASE,
        help="machine-local dataset base or its all11 unified root",
    )
    build.add_argument("--unified-root", type=Path, help=argparse.SUPPRESS)
    build.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    build.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    build.add_argument(
        "--skip-archive-crc",
        action="store_true",
        help="diagnostic only; writes status=unverified_archive_crc (not a freeze)",
    )
    validate = subparsers.add_parser("validate", help="recompute and validate a frozen manifest")
    validate.add_argument("--manifest", type=Path, default=DEFAULT_OUTPUT)
    validate.add_argument(
        "--dataset-root",
        "--dataset-base",
        dest="dataset_root",
        type=Path,
        help="machine-local dataset base or its all11 unified root (defaults to manifest binding)",
    )
    validate.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    validate.add_argument("--skip-archive-crc", action="store_true", help=argparse.SUPPRESS)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "build":
            dataset_root = args.dataset_root
            if args.unified_root is not None:
                # Internal/test convenience while preserving the one-root CLI.
                dataset_root = args.dataset_root
                base, _ = _normalise_dataset_roots(dataset_root)
                manifest = _build_manifest(
                    dataset_root,
                    protocol_path=args.protocol,
                    check_archive_crc=not args.skip_archive_crc,
                )
                manifest["roots"]["unified"]["path"] = _path_string(args.unified_root)
                _write_json(args.output, manifest)
            else:
                manifest = build_manifest(
                    dataset_root,
                    output=args.output,
                    protocol_path=args.protocol,
                    check_archive_crc=not args.skip_archive_crc,
                )
            print(json.dumps({"status": manifest["status"], "output": str(_absolute(args.output)), "sequences": len(manifest["sequences"])}, sort_keys=True))
            return 0 if manifest["status"] == "frozen" else 2
        result = validate_manifest(
            args.manifest,
            dataset_root=args.dataset_root,
            protocol_path=args.protocol,
            check_archive_crc=not args.skip_archive_crc,
        )
        print(json.dumps(result, sort_keys=True))
        return 0
    except ManifestError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
