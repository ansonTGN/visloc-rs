#!/usr/bin/env python3
"""Strict decoder for Basalt's native ``images/*.cereal`` archives.

The pinned Basalt 0f3b2b52 writer uses cereal 1.3.2's non-portable binary
archive.  This module intentionally decodes only that ABI: little-endian
x86-64 arithmetic values, cereal's uint64 size tags, and cereal's uint32
shared-pointer IDs.  It emits the same normalized envelope consumed by
``native_schema4_bridge.py``.  It does not attempt to reconstruct fields that
Basalt's OpticalFlowResult serializer omits (notably ``pyramid_levels``).

The decoder is deliberately fail-closed.  It checks archive names, pointer
identity/type, container limits, map ordering, dimensions, exact EOF, input
and result timestamps, image hashes, and camera cardinality before emitting a
manifest.  The output stores image bytes as base64 rather than converting the
16-bit samples through a lossy image format.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import re
import struct
import sys
from pathlib import Path
from typing import Any, Callable


SCHEMA = "basalt.native.optical_flow.archive.v1"
ARCHIVE_KIND = "native_cereal_images"
PINNED_BASALT_COMMIT = "0f3b2b52c807f70ff4e2973ce253c73329eea7bc"
PINNED_CEREAL_VERSION = "1.3.2"
MAX_RECORDS = 100_000
MAX_CAMERAS = 16
MAX_OBSERVATIONS_PER_CAMERA = 1_000_000
MAX_IMAGE_PIXELS = 64 * 1024 * 1024
MAX_IMAGE_WIDTH = 16 * 1024
MAX_IMAGE_HEIGHT = 16 * 1024
POINTER_MSB = 0x80000000
POINTER_ID_MASK = 0x7FFFFFFF
ARCHIVE_NAME_RE = re.compile(r"^-?[0-9]+\.cereal$")


class DecodeError(ValueError):
    """Raised for an archive that is not the pinned native ABI."""


class _Reader:
    def __init__(self, data: bytes, source: str) -> None:
        self.data = data
        self.source = source
        self.pos = 0
        self.pointers: dict[int, tuple[str, Any]] = {}
        self.new_pointer_count = 0
        self.alias_pointer_count = 0

    def _take(self, length: int, what: str) -> bytes:
        if length < 0 or self.pos + length > len(self.data):
            raise DecodeError(
                f"{self.source}: truncated {what} at offset {self.pos} "
                f"(need {length}, have {len(self.data) - self.pos})"
            )
        start = self.pos
        self.pos += length
        return self.data[start:self.pos]

    def _unpack(self, fmt: str, what: str) -> Any:
        return struct.unpack("<" + fmt, self._take(struct.calcsize("<" + fmt), what))[0]

    def u32(self, what: str = "uint32") -> int:
        return int(self._unpack("I", what))

    def u64(self, what: str = "uint64") -> int:
        return int(self._unpack("Q", what))

    def i64(self, what: str = "int64") -> int:
        return int(self._unpack("q", what))

    def f32(self, what: str = "float32") -> tuple[float, int]:
        raw = self._take(4, what)
        value = float(struct.unpack("<f", raw)[0])
        bits = int.from_bytes(raw, "little")
        if not math.isfinite(value):
            raise DecodeError(f"{self.source}: non-finite {what} at offset {self.pos - 4}")
        return value, bits

    def f64(self, what: str = "float64") -> float:
        value = float(self._unpack("d", what))
        if not math.isfinite(value):
            raise DecodeError(f"{self.source}: non-finite {what} at offset {self.pos - 8}")
        return value

    def bytes(self, length: int, what: str) -> bytes:
        return self._take(length, what)

    def count(self, what: str, maximum: int, minimum: int = 0) -> int:
        value = self.u64(f"{what} size tag")
        if value < minimum or value > maximum:
            raise DecodeError(
                f"{self.source}: {what} size {value} outside [{minimum}, {maximum}]"
            )
        return value

    def shared(
        self,
        what: str,
        kind: str,
        decoder: Callable[[], Any],
        required: bool = False,
    ) -> Any:
        encoded_id = self.u32(f"{what} shared-pointer ID")
        if encoded_id == 0:
            if required:
                raise DecodeError(f"{self.source}: required {what} shared pointer is null")
            return None

        pointer_id = encoded_id & POINTER_ID_MASK
        is_new = bool(encoded_id & POINTER_MSB)
        if pointer_id == 0:
            raise DecodeError(f"{self.source}: invalid {what} shared-pointer ID 0x{encoded_id:08x}")

        if is_new:
            if pointer_id in self.pointers:
                raise DecodeError(
                    f"{self.source}: duplicate new {what} shared-pointer ID {pointer_id}"
                )
            value = decoder()
            self.pointers[pointer_id] = (kind, value)
            self.new_pointer_count += 1
            return value

        existing = self.pointers.get(pointer_id)
        if existing is None:
            raise DecodeError(
                f"{self.source}: {what} references unknown shared-pointer ID {pointer_id}"
            )
        existing_kind, value = existing
        if existing_kind != kind:
            raise DecodeError(
                f"{self.source}: {what} pointer ID {pointer_id} has type "
                f"{existing_kind}, expected {kind}"
            )
        self.alias_pointer_count += 1
        return value


def _fnv1a_u16_payload(payload: bytes) -> int:
    value = 1469598103934665603
    for byte in payload:
        value = (value ^ byte) * 1099511628211 & ((1 << 64) - 1)
    return value


def _sha256(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _read_affine(reader: _Reader, label: str) -> tuple[list[float], list[str]]:
    # basalt::serialization/eigen_io.h iterates fixed-size matrices as
    # m(row, col), so this is the logical row-major [m00,m01,m02,m10,m11,m12]
    # order despite Eigen's default column-major storage.
    values: list[float] = []
    bits: list[str] = []
    for index in range(6):
        value, raw_bits = reader.f32(f"{label}[{index}]")
        values.append(value)
        bits.append(f"{raw_bits:08x}")
    return values, bits


def _read_map(reader: _Reader, camera_index: int) -> list[dict[str, Any]]:
    count = reader.count(
        f"observations[{camera_index}]", MAX_OBSERVATIONS_PER_CAMERA
    )
    result: list[dict[str, Any]] = []
    previous_key: int | None = None
    for item_index in range(count):
        key = reader.u64(f"observations[{camera_index}][{item_index}] keypoint ID")
        if previous_key is not None and key <= previous_key:
            raise DecodeError(
                f"{reader.source}: observations[{camera_index}] map keys are not strictly ordered "
                f"at item {item_index}: {previous_key} then {key}"
            )
        previous_key = key
        affine, affine_bits = _read_affine(
            reader, f"observations[{camera_index}][{item_index}].affine_2x3"
        )
        result.append(
            {
                "track_id": key,
                "camera_id": camera_index,
                "xy": [affine[2], affine[5]],
                "affine_2x3": affine,
                "affine_2x3_bits": affine_bits,
            }
        )
    return result


def _read_image(reader: _Reader) -> dict[str, Any]:
    # MargDataSaver serializes ManagedImage<uint16_t>::w/h as native size_t.
    # The pinned producer is x86-64 Linux, where size_t is 8-byte little endian.
    width = reader.u64("ManagedImage<uint16_t>.w")
    height = reader.u64("ManagedImage<uint16_t>.h")
    if width == 0 or height == 0:
        raise DecodeError(f"{reader.source}: zero-sized ManagedImage is not accepted")
    if width > MAX_IMAGE_WIDTH or height > MAX_IMAGE_HEIGHT:
        raise DecodeError(
            f"{reader.source}: image dimensions {width}x{height} exceed decoder limits"
        )
    pixels = width * height
    if pixels > MAX_IMAGE_PIXELS:
        raise DecodeError(
            f"{reader.source}: image pixel count {pixels} exceeds decoder limit"
        )
    payload = reader.bytes(pixels * 2, "ManagedImage<uint16_t> payload")
    return {
        "width": width,
        "height": height,
        "data": base64.b64encode(payload).decode("ascii"),
        "sample_hash": f"{_fnv1a_u16_payload(payload):016x}",
        "payload_sha256": _sha256(payload),
    }


def _read_input(reader: _Reader) -> dict[str, Any]:
    timestamp = reader.i64("OpticalFlowInput.t_ns")
    image_count = reader.count("OpticalFlowInput.img_data", MAX_CAMERAS, minimum=1)
    images: list[dict[str, Any]] = []
    for camera_index in range(image_count):
        exposure = reader.f64(f"OpticalFlowInput.img_data[{camera_index}].exposure")
        image = reader.shared(
            f"OpticalFlowInput.img_data[{camera_index}].img",
            "ManagedImage<uint16_t>",
            lambda: _read_image(reader),
            required=True,
        )
        image_record = dict(image)
        image_record.update(
            {
                "timestamp_ns": timestamp,
                "camera_id": camera_index,
                "exposure": exposure,
            }
        )
        images.append(image_record)
    return {"timestamp_ns": timestamp, "images": images}


def _read_result(reader: _Reader) -> dict[str, Any]:
    timestamp = reader.i64("OpticalFlowResult.t_ns")
    camera_count = reader.count("OpticalFlowResult.observations", MAX_CAMERAS, minimum=1)
    observations: list[dict[str, Any]] = []
    for camera_index in range(camera_count):
        observations.extend(_read_map(reader, camera_index))

    input_images = reader.shared(
        "OpticalFlowResult.input_images",
        "OpticalFlowInput",
        lambda: _read_input(reader),
        required=True,
    )
    if input_images["timestamp_ns"] != timestamp:
        raise DecodeError(
            f"{reader.source}: OpticalFlowInput.t_ns {input_images['timestamp_ns']} "
            f"does not match OpticalFlowResult.t_ns {timestamp}"
        )
    if len(input_images["images"]) != camera_count:
        raise DecodeError(
            f"{reader.source}: observations camera count {camera_count} != "
            f"input image count {len(input_images['images'])}"
        )

    return {
        "timestamp_ns": timestamp,
        "observations": observations,
        "images": input_images["images"],
        "camera_count": camera_count,
    }


def _decode_record(data: bytes, source: str, filename: str) -> tuple[dict[str, Any], dict[str, int]]:
    reader = _Reader(data, source)
    result = reader.shared(
        "OpticalFlowResult",
        "OpticalFlowResult",
        lambda: _read_result(reader),
        required=True,
    )
    if reader.pos != len(data):
        raise DecodeError(
            f"{source}: trailing bytes after OpticalFlowResult at offset {reader.pos} "
            f"({len(data) - reader.pos} bytes)"
        )
    expected_name = f"{result['timestamp_ns']}.cereal"
    if filename != expected_name:
        raise DecodeError(
            f"{source}: filename {filename!r} does not match serialized timestamp "
            f"{result['timestamp_ns']} (expected {expected_name!r})"
        )
    record = {
        "timestamp_ns": result["timestamp_ns"],
        "input_timestamp_ns": result["timestamp_ns"],
        "source_file": filename,
        "source_sha256": _sha256(data),
        "observations": result["observations"],
        "images": result["images"],
    }
    counters = {
        "new_shared_pointers": reader.new_pointer_count,
        "aliased_shared_pointers": reader.alias_pointer_count,
    }
    return record, counters


def _camera_digest(records: list[dict[str, Any]], camera_id: int) -> str:
    rows: list[dict[str, Any]] = []
    for record in records:
        for image in record["images"]:
            if image["camera_id"] != camera_id:
                continue
            rows.append(
                {
                    "timestamp_ns": record["timestamp_ns"],
                    "camera_id": camera_id,
                    "width": image["width"],
                    "height": image["height"],
                    "sample_hash": image["sample_hash"].lower(),
                    "payload_sha256": image["payload_sha256"].lower(),
                }
            )
    canonical = json.dumps(
        rows, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")
    return _sha256(canonical)


def _validate_archive_directory(path: Path) -> list[Path]:
    try:
        root = path.resolve(strict=True)
    except OSError as error:
        raise DecodeError(f"archive root {path}: cannot resolve: {error}") from error
    if not root.is_dir():
        raise DecodeError(f"archive root {root} is not a directory")
    entries = sorted(root.iterdir(), key=lambda item: item.name)
    if not entries:
        raise DecodeError(f"archive root {root} contains no files")
    files: list[Path] = []
    seen_timestamps: set[int] = set()
    for entry in entries:
        if entry.is_symlink():
            raise DecodeError(f"archive root {root}: symlink entry {entry.name!r} is forbidden")
        if not entry.is_file():
            raise DecodeError(f"archive root {root}: non-file entry {entry.name!r} is forbidden")
        if ARCHIVE_NAME_RE.fullmatch(entry.name) is None:
            raise DecodeError(
                f"archive root {root}: unexpected entry {entry.name!r}; "
                "only <signed decimal timestamp>.cereal is accepted"
            )
        timestamp_text = entry.name[:-7]
        timestamp = int(timestamp_text)
        if timestamp in seen_timestamps:
            raise DecodeError(f"archive root {root}: duplicate timestamp {timestamp}")
        seen_timestamps.add(timestamp)
        files.append(entry)
    if len(files) > MAX_RECORDS:
        raise DecodeError(f"archive root {root}: {len(files)} records exceed decoder limit")
    return files


def decode_archive(archive_dir: Path) -> dict[str, Any]:
    files = _validate_archive_directory(archive_dir)
    root = archive_dir.resolve()
    records: list[dict[str, Any]] = []
    pointer_counters = {"new_shared_pointers": 0, "aliased_shared_pointers": 0}
    camera_count: int | None = None
    total_observations = 0
    total_image_payload_bytes = 0

    for path in files:
        try:
            data = path.read_bytes()
        except OSError as error:
            raise DecodeError(f"{path}: cannot read: {error}") from error
        record, counters = _decode_record(data, str(path), path.name)
        record_camera_count = len(record["images"])
        if camera_count is None:
            camera_count = record_camera_count
        elif record_camera_count != camera_count:
            raise DecodeError(
                f"{path}: camera count {record_camera_count} differs from prior "
                f"records' {camera_count}"
            )
        for image in record["images"]:
            total_image_payload_bytes += image["width"] * image["height"] * 2
        total_observations += len(record["observations"])
        for key in pointer_counters:
            pointer_counters[key] += counters[key]
        records.append(record)

    assert camera_count is not None
    camera_ids = list(range(camera_count))
    camera_hashes = {
        str(camera_id): _camera_digest(records, camera_id) for camera_id in camera_ids
    }
    return {
        "schema": SCHEMA,
        "archive_kind": ARCHIVE_KIND,
        "source_root": str(root),
        "source": {
            "project": "basalt",
            "archive": "cereal.binary.images",
            "writer": "basalt::MargDataSaver::save_image_func",
            "pinned_basalt_commit": PINNED_BASALT_COMMIT,
            "pinned_cereal_version": PINNED_CEREAL_VERSION,
            "binary_abi": "x86-64 little-endian non-portable cereal BinaryArchive",
        },
        "decoder": {
            "name": "native_images_cereal_decoder.py",
            "version": 1,
            "pinned_basalt_commit": PINNED_BASALT_COMMIT,
            "pinned_cereal_version": PINNED_CEREAL_VERSION,
            "size_type_bytes": 8,
            "native_size_t_bytes": 8,
            "endianness": "little",
        },
        "serialized_layout": [
            "shared_ptr<OpticalFlowResult>: uint32 pointer id, then data for a new id",
            "OpticalFlowResult: int64 t_ns, vector<aligned_map<KeypointId,AffineCompact2f>> observations, shared_ptr<OpticalFlowInput>",
            "observation map: uint64 size, repeated uint64 KeypointId + six float32 matrix values in row/column logical order",
            "OpticalFlowInput: int64 t_ns, vector<ImageData> img_data",
            "ImageData: float64 exposure, shared_ptr<ManagedImage<uint16_t>>",
            "ManagedImage<uint16_t>: native size_t width, native size_t height, width*height raw uint16 bytes",
        ],
        "omitted_native_fields": {
            "pyramid_levels": {
                "available": False,
                "reason": "OpticalFlowResult::serialize in pinned src/io/marg_data_io.cpp serializes t_ns, observations, and input_images only",
            }
        },
        "camera_ids": camera_ids,
        "camera_count": camera_count,
        "record_count": len(records),
        "observation_count": total_observations,
        "image_payload_bytes": total_image_payload_bytes,
        "pointer_counts": pointer_counters,
        "camera_hashes": camera_hashes,
        "records": records,
    }


def _fixture_bytes() -> bytes:
    """Build a tiny exact cereal binary fixture without filesystem I/O."""
    out = bytearray()
    pack = lambda fmt, *values: out.extend(struct.pack("<" + fmt, *values))
    timestamp = 1403636579763555584
    pack("I", POINTER_MSB | 1)
    pack("q", timestamp)
    pack("Q", 2)  # observations cameras
    pack("Q", 1)
    pack("Q", 3)
    pack("6f", 1.0, 0.0, 10.25, 0.0, 1.0, 20.5)
    pack("Q", 1)
    pack("Q", 3)
    pack("6f", 0.5, 0.25, 11.25, -0.25, 0.5, 21.5)
    pack("I", POINTER_MSB | 2)
    pack("q", timestamp)
    pack("Q", 2)  # ImageData entries
    pack("d", 0.01)
    pack("I", POINTER_MSB | 3)
    pack("Q", 2)
    pack("Q", 2)
    out.extend(struct.pack("<4H", 0, 1, 0x1234, 0xFFFF))
    pack("d", 0.02)
    pack("I", 3)  # exact shared pointer alias
    return bytes(out)


def run_self_test() -> dict[str, Any]:
    payload = _fixture_bytes()
    record, counters = _decode_record(
        payload,
        "synthetic/1403636579763555584.cereal",
        "1403636579763555584.cereal",
    )
    if len(record["observations"]) != 2:
        raise AssertionError("fixture observation count")
    if len(record["images"]) != 2:
        raise AssertionError("fixture image count")
    if record["images"][0]["data"] != record["images"][1]["data"]:
        raise AssertionError("shared image alias was not preserved")
    if counters != {"new_shared_pointers": 3, "aliased_shared_pointers": 1}:
        raise AssertionError(f"unexpected pointer counters: {counters}")

    malformed: dict[str, str] = {}
    for name, bad_payload in {
        "truncated": payload[:-1],
        "trailing_byte": payload + b"\\x00",
    }.items():
        try:
            _decode_record(
                bad_payload,
                f"synthetic/{name}.cereal",
                "1403636579763555584.cereal",
            )
        except DecodeError as error:
            malformed[name] = str(error)
        else:  # pragma: no cover - the assertion is the test
            raise AssertionError(f"malformed fixture {name} was accepted")

    return {
        "status": "PASS",
        "fixture_bytes": len(payload),
        "record_timestamp_ns": record["timestamp_ns"],
        "observation_count": len(record["observations"]),
        "image_count": len(record["images"]),
        "shared_pointer_counters": counters,
        "malformed_rejections": sorted(malformed),
        "layout": {
            "result_pointer_id": "0x80000001",
            "input_pointer_id": "0x80000002",
            "image_pointer_id": "0x80000003",
            "image_alias_id": "0x00000003",
        },
    }


def _write_manifest(path: Path, document: dict[str, Any], overwrite: bool) -> None:
    if path.exists() and not overwrite:
        raise DecodeError(f"output {path} already exists; pass --overwrite to replace it")
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = json.dumps(document, indent=2, sort_keys=False, ensure_ascii=False) + "\n"
    path.write_text(encoded, encoding="utf-8", newline="\n")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive-dir", type=Path, help="native images/ directory")
    parser.add_argument("--output", type=Path, help="normalized JSON manifest path")
    parser.add_argument("--overwrite", action="store_true", help="allow replacing --output")
    parser.add_argument("--self-test", action="store_true", help="run in-memory ABI tests")
    args = parser.parse_args(argv)

    try:
        if args.self_test:
            if args.archive_dir is not None or args.output is not None:
                raise DecodeError("--self-test cannot be combined with --archive-dir/--output")
            print(json.dumps(run_self_test(), indent=2, sort_keys=True))
            return 0
        if args.archive_dir is None or args.output is None:
            parser.error("--archive-dir and --output are required unless --self-test is used")
        document = decode_archive(args.archive_dir)
        _write_manifest(args.output, document, args.overwrite)
        print(
            json.dumps(
                {
                    "status": "PASS",
                    "output": str(args.output),
                    "record_count": document["record_count"],
                    "camera_count": document["camera_count"],
                    "observation_count": document["observation_count"],
                    "image_payload_bytes": document["image_payload_bytes"],
                },
                sort_keys=True,
            )
        )
        return 0
    except (DecodeError, OSError, AssertionError) as error:
        print(f"native_images_cereal_decoder: ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
