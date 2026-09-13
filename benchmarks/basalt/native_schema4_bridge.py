#!/usr/bin/env python3
"""Project the pinned native cereal MargData bridge into a Rust schema-4 view.

The native cereal reader remains ``upstream_marg_dump.cpp``.  This adapter is
deliberately an interop boundary: values present in the native packet are
copied without effective-branch reconstruction, while fields that cereal does
not contain are represented by explicit unavailable records.  An incomplete
projection is never presented as a deserializable Rust ``MargData`` value.

An optional ``--optical-flow-archive`` input joins a normalized dump of the
separate native ``images/*.cereal`` records.  It may also point directly at a
raw native ``images`` directory: the pinned-ABI Python decoder is used in that
case, and this layer still verifies the normalized record, camera, payload,
and timestamp/frame-map contracts before merging it.

The v2 companion profile exposes two contracts.  ``native-compatible`` means
every native field and same-run sidecar is exact; only the native-absent
``kf_to_marg`` destination/reanchor relation is optional.  The separate Rust
extension namespace reports FEJ tangent/layout mapping and Rust reanchor
semantics explicitly, and never treats raw native frame-state bits as a Rust
value.  ``--strict-native-compatible`` gates the former; ``--strict-schema4``
continues to gate the complete Rust extension profile.  The v3 extension
boundary additionally requires same-event raw f32 bits, source-row
permutations, and explicit reanchor pairs before emitting a Rust record.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import importlib.util
import json
import math
import os
import posixpath
import re
import subprocess
import tempfile
import struct
import sys
from pathlib import Path
from typing import Any, Iterable, Mapping


ADAPTER_SCHEMA = "basalt.rust.schema4.native_adapter.v1"
OPTICAL_FLOW_ARCHIVE_SCHEMA = "basalt.native.optical_flow.archive.v1"
OPTICAL_FLOW_MERGE_SCHEMA = "basalt.rust.schema4.native_optical_flow_merge.v1"
RUST_EXTENSION_MAPPING_SCHEMA = "basalt.rust.schema4.extension.mapping.v1"
RUST_SCHEMA4_EXTENSION_SCHEMA = "basalt.native.schema4.rust_extension_capture.v3"
RUST_SCHEMA4_ADAPTER_SCHEMA = "basalt.rust.schema4.native_adapter.v3"
SCHEMA4 = 4
COMPANION_SCHEMA = "basalt.native.schema4.runtime_companion.v1"
COMPANION_PROFILE_SCHEMA = "basalt.rust.schema4.native_companion_profile.v1"
COMPANION_MAGIC = b"BSC4CMP1"
COMPANION_FILE_HEADER_SIZE = 22
COMPANION_SECTION_HEADER_SIZE = 68
COMPANION_TAG_NAMES = {
    1: "AOM_PRE",
    3: "Q2_FINAL",
    5: "PARTITION",
    6: "PRIOR_INPUT_METADATA",
    13: "SELECTION_DIAGNOSTIC",
    15: "PRIOR_INPUT",
}
# This is the evidence matrix from the native companion contract.  Keep it
# explicit: a packet can satisfy the small packet/AOM/Q2 binding while still
# lacking source rows or the post-marginalization lifecycle.  The full strict
# profile must never silently collapse those two levels into one boolean.
COMPANION_FIELD_MATRIX = (
    "MargData.aom",
    "MargData.abs_H",
    "MargData.abs_b",
    "MargData.frame_poses",
    "MargData.frame_states",
    "MargData.kfs_all",
    "MargData.kfs_to_marg",
    "MargData.use_imu",
    "MargData.opt_flow_res",
    "MargLinData.is_sqrt",
    "MargLinData.order",
    "MargLinData.H and MargLinData.b",
    "prior current delta",
    "prior FEJ point / anchor",
    "Q2Jp/Q2r final",
    "visual Q2 source",
    "visual factor identity and observation rows",
    "IMU Q2 source",
    "pose damping Q2 rows",
    "Q2 row topology",
    "AOM pre/post and marginal partition",
    "kf_to_marg equivalent",
    "selection denominators/scores",
    "lost_landmarks",
    "internal new prior",
    "PoseStateWithLin backup_*",
    "optical-flow observations/images",
)
_WORKSPACE = Path(__file__).resolve().parents[2]
_VALIDATOR_PATH = _WORKSPACE / "work" / "basalt_margdata_bridge_validate_20260829.py"


class AdapterError(ValueError):
    """Malformed input or an intentionally rejected incomplete projection."""


def _validator():
    spec = importlib.util.spec_from_file_location("native_bridge_validator", _VALIDATOR_PATH)
    if spec is None or spec.loader is None:
        raise AdapterError(f"cannot load bridge validator: {_VALIDATOR_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _unavailable(reason: str) -> dict[str, Any]:
    return {"available": False, "reason": reason}


def _canonical(value: Any) -> bytes:
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")


def _roundtrip_exact(value: Any) -> tuple[bool, str, str]:
    """Check JSON parse/serialize stability and return both canonical hashes."""
    before = _canonical(value)
    decoded = json.loads(before.decode("utf-8"))
    after = _canonical(decoded)
    return (
        before == after,
        hashlib.sha256(before).hexdigest(),
        hashlib.sha256(after).hexdigest(),
    )


def _sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _hex_sha256(value: Any, path: str) -> str:
    if not isinstance(value, str) or len(value) != 64:
        raise AdapterError(f"{path}: expected a 64-character SHA-256 hex string")
    lowered = value.lower()
    if any(char not in "0123456789abcdef" for char in lowered):
        raise AdapterError(f"{path}: invalid SHA-256 hex string")
    return lowered


def _companion_require(condition: bool, message: str) -> None:
    if not condition:
        raise AdapterError(f"companion: {message}")


def _companion_integer(value: Any, path: str, minimum: int | None = None) -> int:
    _companion_require(
        isinstance(value, int) and not isinstance(value, bool),
        f"{path}: expected integer",
    )
    if minimum is not None:
        _companion_require(value >= minimum, f"{path}: expected >= {minimum}")
    return value


def _companion_integer_list(value: Any, path: str) -> list[int]:
    _companion_require(isinstance(value, list), f"{path}: expected integer array")
    result = [_companion_integer(item, f"{path}[{index}]") for index, item in enumerate(value)]
    _companion_require(result == sorted(set(result)), f"{path}: expected sorted unique values")
    return result


def _companion_f32_values(payload: bytes, count: int, path: str) -> list[float]:
    _companion_require(count >= 0, f"{path}: negative element count")
    expected = count * 4
    _companion_require(
        len(payload) == expected,
        f"{path}: expected {expected} payload bytes, got {len(payload)}",
    )
    values = list(struct.unpack("<" + "f" * count, payload)) if count else []
    _companion_require(
        all(math.isfinite(value) for value in values),
        f"{path}: NaN/Inf is forbidden in a strict companion section",
    )
    return values


def _companion_json_payload(payload: bytes, path: str) -> dict[str, Any]:
    try:
        value = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise AdapterError(f"companion: {path}: invalid UTF-8 JSON: {error}") from error
    _companion_require(isinstance(value, dict), f"{path}: expected JSON object")
    return value


def _companion_read_aom(payload: bytes, path: str) -> dict[str, Any]:
    _companion_require(len(payload) >= 24, f"{path}: truncated AOM header")
    total_size, items, order_count = struct.unpack_from("<QQQ", payload, 0)
    expected = 24 + order_count * 24
    _companion_require(len(payload) == expected, f"{path}: AOM byte length mismatch")
    order: list[dict[str, int]] = []
    offset = 24
    previous_timestamp: int | None = None
    previous_end = 0
    for index in range(order_count):
        timestamp, block_offset, dof = struct.unpack_from("<qQQ", payload, offset)
        offset += 24
        _companion_require(dof > 0, f"{path}[{index}].dof: expected positive value")
        if previous_timestamp is not None:
            _companion_require(
                timestamp > previous_timestamp,
                f"{path}: timestamps are not strictly increasing",
            )
        _companion_require(
            block_offset == previous_end,
            f"{path}[{index}]: non-contiguous offset {block_offset}, expected {previous_end}",
        )
        _companion_require(
            block_offset + dof <= total_size,
            f"{path}[{index}]: block exceeds total_size",
        )
        order.append({"timestamp_ns": timestamp, "offset": block_offset, "dof": dof})
        previous_timestamp = timestamp
        previous_end = block_offset + dof
    _companion_require(previous_end == total_size, f"{path}: blocks do not cover total_size")
    return {"total_size": total_size, "items": items, "order": order}


def _companion_validate_aom_manifest(
    manifest_aom: Any, binary_aom: Mapping[str, Any], path: str = "aom_binding"
) -> None:
    _companion_require(isinstance(manifest_aom, dict), f"{path}: expected object")
    total_size = _companion_integer(manifest_aom.get("total_size"), f"{path}.total_size", 0)
    items = _companion_integer(manifest_aom.get("items"), f"{path}.items", 0)
    order = manifest_aom.get("order")
    _companion_require(isinstance(order, list), f"{path}.order: expected array")
    _companion_require(
        {"total_size": total_size, "items": items, "order": order}
        == dict(binary_aom),
        f"{path}: manifest does not match AOM_PRE payload",
    )


def _companion_parse_binary(
    manifest: Mapping[str, Any], binary_path: Path
) -> tuple[dict[int, dict[str, Any]], str]:
    try:
        raw = binary_path.read_bytes()
    except OSError as error:
        raise AdapterError(f"companion: cannot read binary {binary_path}: {error}") from error
    _companion_require(len(raw) >= COMPANION_FILE_HEADER_SIZE, "binary header is truncated")
    magic, version, scalar, layout, section_count, header_bytes, reserved = struct.unpack_from(
        "<8sHBBHII", raw, 0
    )
    _companion_require(magic == COMPANION_MAGIC, f"binary magic {magic!r} is not BSC4CMP1")
    _companion_require(version == 1, f"unsupported binary version {version}")
    _companion_require(scalar == 1, f"strict v1 requires f32 scalar code 1, got {scalar}")
    _companion_require(layout == 1, f"strict v1 requires column-major layout code 1, got {layout}")
    _companion_require(reserved == 0, "binary reserved word is non-zero")
    _companion_require(section_count > 0, "binary has no sections")
    expected_header = COMPANION_FILE_HEADER_SIZE + COMPANION_SECTION_HEADER_SIZE * section_count
    _companion_require(header_bytes == expected_header, "binary header_bytes is inconsistent")
    _companion_require(len(raw) >= header_bytes, "binary ends inside section headers")
    manifest_format = manifest.get("binary_format")
    _companion_require(isinstance(manifest_format, dict), "binary_format: expected object")
    _companion_require(manifest_format.get("magic") == "BSC4CMP1", "manifest binary magic mismatch")
    _companion_require(manifest_format.get("version") == 1, "manifest binary version mismatch")
    _companion_require(manifest_format.get("endianness") == "little", "manifest endianness mismatch")
    _companion_require(manifest_format.get("header_bytes") == header_bytes, "manifest header_bytes mismatch")

    manifest_sections = manifest.get("sections")
    _companion_require(isinstance(manifest_sections, list), "sections: expected array")
    _companion_require(
        len(manifest_sections) == section_count,
        "manifest section count does not match binary",
    )
    headers: list[dict[str, Any]] = []
    offset = COMPANION_FILE_HEADER_SIZE
    seen: set[int] = set()
    for index in range(section_count):
        tag, flags, rows, cols, element_count, payload_bytes, digest = struct.unpack_from(
            "<HHQQQQ32s", raw, offset
        )
        offset += COMPANION_SECTION_HEADER_SIZE
        _companion_require(tag in COMPANION_TAG_NAMES, f"unknown v1 section tag {tag}")
        _companion_require(tag not in seen, f"duplicate section tag {tag}")
        seen.add(tag)
        metadata = manifest_sections[index]
        _companion_require(isinstance(metadata, dict), f"sections[{index}]: expected object")
        _companion_require(metadata.get("tag") == tag, f"sections[{index}].tag mismatch")
        for key, expected in (
            ("flags", flags),
            ("rows", rows),
            ("cols", cols),
            ("element_count", element_count),
            ("payload_bytes", payload_bytes),
            ("byte_offset", header_bytes if index == 0 else None),
        ):
            if key == "byte_offset":
                continue
            _companion_require(metadata.get(key) == expected, f"sections[{index}].{key} mismatch")
        headers.append(
            {
                "tag": tag,
                "flags": flags,
                "rows": rows,
                "cols": cols,
                "element_count": element_count,
                "payload_bytes": payload_bytes,
                "digest": digest.hex(),
                "manifest": metadata,
            }
        )

    payload_offset = header_bytes
    sections: dict[int, dict[str, Any]] = {}
    for header in headers:
        payload_bytes = int(header["payload_bytes"])
        end = payload_offset + payload_bytes
        _companion_require(end <= len(raw), f"section {header['tag']} payload is truncated")
        payload = raw[payload_offset:end]
        metadata = header["manifest"]
        _companion_require(
            metadata.get("byte_offset") == payload_offset,
            f"section {header['tag']} byte_offset mismatch",
        )
        digest = _sha256_bytes(payload)
        _companion_require(
            digest == str(header["digest"]).lower(),
            f"section {header['tag']} payload SHA-256 mismatch",
        )
        _companion_require(
            digest == _hex_sha256(metadata.get("payload_sha256"), f"sections[{header['tag']}].payload_sha256"),
            f"section {header['tag']} manifest payload SHA-256 mismatch",
        )
        parsed = dict(header)
        parsed["payload"] = payload
        parsed["payload_offset"] = payload_offset
        sections[header["tag"]] = parsed
        payload_offset = end
    _companion_require(payload_offset == len(raw), "binary has trailing bytes after sections")

    if 1 in sections:
        aom = _companion_read_aom(sections[1]["payload"], "AOM_PRE")
        _companion_validate_aom_manifest(manifest.get("aom_binding"), aom)
        sections[1]["parsed"] = aom
    if 3 in sections:
        q2 = sections[3]
        q2_shape = manifest.get("row_topology_binding", {}).get("q2_shape")
        _companion_require(isinstance(q2_shape, dict), "row_topology_binding.q2_shape: expected object")
        for key, expected in (("rows", q2["rows"]), ("cols", q2["cols"]), ("rhs_size", q2["rows"]), ("layout", "column_major"), ("scalar", "f32")):
            if key in ("rows", "cols"):
                _companion_require(q2_shape.get(key) == expected, f"q2_shape.{key} mismatch")
        rhs_size = _companion_integer(q2_shape.get("rhs_size"), "q2_shape.rhs_size", 0)
        _companion_require(q2["element_count"] == q2["rows"] * q2["cols"] + rhs_size, "Q2_FINAL element count mismatch")
        values = _companion_f32_values(q2["payload"], q2["element_count"], "Q2_FINAL")
        matrix_count = q2["rows"] * q2["cols"]
        sections[3]["parsed"] = {
            "rows": q2["rows"],
            "cols": q2["cols"],
            "rhs_size": rhs_size,
            "jacobian": values[:matrix_count],
            "rhs": values[matrix_count:],
        }
    for tag in (5, 6, 13):
        if tag in sections:
            sections[tag]["parsed"] = _companion_json_payload(
                sections[tag]["payload"], COMPANION_TAG_NAMES[tag]
            )
    if 15 in sections:
        prior = sections[15]
        metadata = sections.get(6, {}).get("parsed")
        _companion_require(isinstance(metadata, dict), "PRIOR_INPUT requires PRIOR_INPUT_METADATA")
        rows = _companion_integer(metadata.get("rows"), "PRIOR_INPUT_METADATA.rows", 1)
        cols = _companion_integer(metadata.get("cols"), "PRIOR_INPUT_METADATA.cols", 1)
        rhs_size = _companion_integer(metadata.get("rhs_size"), "PRIOR_INPUT_METADATA.rhs_size", 0)
        _companion_require((rows, cols) == (prior["rows"], prior["cols"]), "PRIOR_INPUT shape mismatch")
        _companion_require(prior["element_count"] == rows * cols + rhs_size, "PRIOR_INPUT element count mismatch")
        values = _companion_f32_values(prior["payload"], prior["element_count"], "PRIOR_INPUT")
        matrix_count = rows * cols
        sections[15]["parsed"] = {
            "rows": rows,
            "cols": cols,
            "rhs_size": rhs_size,
            "jacobian": values[:matrix_count],
            "rhs": values[matrix_count:],
        }
    return sections, _sha256_bytes(raw)


def _companion_validate_owner(
    manifest: Mapping[str, Any], companion_path: Path, q2: Mapping[str, Any]
) -> dict[str, Any]:
    topology = manifest.get("row_topology_binding")
    _companion_require(isinstance(topology, dict), "row_topology_binding: expected object")
    owner_name = topology.get("source_owner")
    _companion_require(isinstance(owner_name, str) and owner_name, "row_topology_binding.source_owner is missing")
    _companion_require(not Path(owner_name).is_absolute(), "source_owner must be relative")
    companion_root = companion_path.resolve().parent
    owner_path = (companion_root / owner_name).resolve()
    _companion_require(owner_path.parent == companion_root, "source_owner escapes companion directory")
    try:
        owner = json.loads(owner_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise AdapterError(f"companion: cannot read source owner {owner_path}: {error}") from error
    _companion_require(isinstance(owner, dict), "source owner must be a JSON object")
    _companion_require(owner.get("schema") == "basalt.native.schema4.q2_source_owner.v1", "source owner schema mismatch")
    _companion_require(owner.get("run_uuid") == manifest.get("run_uuid"), "source owner run_uuid mismatch")
    for key, expected in (("rows", q2["rows"]), ("cols", q2["cols"]), ("rhs_size", q2["rhs_size"])):
        _companion_require(owner.get(key) == expected, f"source owner {key} does not match Q2_FINAL")
    _companion_require(owner.get("aom_total_size") == manifest["aom_binding"]["total_size"], "source owner AOM width mismatch")
    _companion_require(owner.get("aom_items") == manifest["aom_binding"]["items"], "source owner AOM item count mismatch")
    _companion_require(owner.get("aom_order") == manifest["aom_binding"]["order"], "source owner AOM order mismatch")
    for key in ("visual_rows", "imu_rows", "damping_rows", "prior_rows"):
        _companion_integer(owner.get(key), f"source_owner.{key}", 0)
    _companion_require(
        sum(owner[key] for key in ("visual_rows", "imu_rows", "damping_rows", "prior_rows")) == q2["rows"],
        "source owner row categories do not cover Q2_FINAL",
    )
    factors = owner.get("visual_factors")
    _companion_require(isinstance(factors, list), "source owner visual_factors must be an array")
    previous = 0
    ordinals: set[int] = set()
    for index, factor in enumerate(factors):
        _companion_require(isinstance(factor, dict), f"source_owner.visual_factors[{index}] must be an object")
        ordinal = _companion_integer(factor.get("ordinal"), f"source_owner.visual_factors[{index}].ordinal", 0)
        _companion_require(ordinal not in ordinals, "source owner visual ordinal is duplicated")
        ordinals.add(ordinal)
        start = _companion_integer(factor.get("row_start"), f"source_owner.visual_factors[{index}].row_start", 0)
        count = _companion_integer(factor.get("row_count"), f"source_owner.visual_factors[{index}].row_count", 1)
        _companion_require(start == previous, "source owner visual row spans have a gap or overlap")
        previous = start + count
    _companion_require(previous == owner["visual_rows"], "source owner visual spans do not cover visual rows")
    return {
        "path": str(owner_path),
        "sha256": _sha256_file(owner_path),
        "data": owner,
    }


def _companion_validate_selection(
    manifest: Mapping[str, Any], selection: Mapping[str, Any]
) -> None:
    identity = manifest.get("packet_identity")
    _companion_require(isinstance(identity, dict), "packet_identity: expected object")
    _companion_require(selection.get("kfs_all") == identity.get("kfs_all"), "selection kfs_all mismatch")
    _companion_require(selection.get("kfs_to_marg") == identity.get("kfs_to_marg"), "selection kfs_to_marg mismatch")
    q2_boundary = identity.get(
        "q2_last_state_to_marg_timestamp_ns",
        identity.get("event_state_timestamp_ns"),
    )
    _companion_require(
        selection.get("last_state_to_marg") == q2_boundary,
        "selection Q2 last-state timestamp mismatch",
    )
    _companion_require(selection.get("is_lin_sqrt") is True, "selection is_lin_sqrt is not true")
    _companion_require(selection.get("marg_is_sqrt") is True, "selection marg_is_sqrt is not true")


def _companion_sidecar_path(companion_path: Path, name: Any, label: str) -> Path:
    """Resolve a same-run sidecar without allowing path or symlink escapes."""
    _companion_require(isinstance(name, str) and name, f"{label}: missing filename")
    candidate = (companion_path.resolve().parent / name).resolve()
    root = companion_path.resolve().parent
    _companion_require(candidate.parent == root, f"{label}: path escapes companion directory")
    _companion_require(not candidate.is_symlink(), f"{label}: symlink is forbidden")
    _companion_require(candidate.is_file(), f"{label}: regular file is required")
    return candidate


def _companion_read_json_file(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise AdapterError(f"companion: cannot read {label} {path}: {error}") from error
    _companion_require(isinstance(value, dict), f"{label}: expected JSON object")
    return value


def _companion_validate_event_aom(value: Any, path: str) -> dict[str, Any]:
    _companion_require(isinstance(value, dict), f"{path}: expected object")
    total_size = _companion_integer(value.get("total_size"), f"{path}.total_size", 0)
    items = _companion_integer(value.get("items"), f"{path}.items", 0)
    order = value.get("order")
    _companion_require(isinstance(order, list), f"{path}.order: expected array")
    normalized: list[dict[str, int]] = []
    previous_timestamp: int | None = None
    previous_end = 0
    for index, item in enumerate(order):
        _companion_require(isinstance(item, dict), f"{path}.order[{index}]: expected object")
        timestamp = _companion_integer(item.get("timestamp_ns"), f"{path}.order[{index}].timestamp_ns")
        offset = _companion_integer(item.get("offset"), f"{path}.order[{index}].offset", 0)
        dof = _companion_integer(item.get("dof"), f"{path}.order[{index}].dof", 1)
        if previous_timestamp is not None:
            _companion_require(timestamp > previous_timestamp, f"{path}.order is not timestamp ordered")
        _companion_require(offset == previous_end, f"{path}.order[{index}] has a gap or overlap")
        _companion_require(offset + dof <= total_size, f"{path}.order[{index}] exceeds total_size")
        normalized.append({"timestamp_ns": timestamp, "offset": offset, "dof": dof})
        previous_timestamp = timestamp
        previous_end = offset + dof
    _companion_require(items == len(normalized), f"{path}.items disagrees with order length")
    _companion_require(previous_end == total_size, f"{path}.order does not cover total_size")
    return {"total_size": total_size, "items": items, "order": normalized}


def _companion_validate_prior_transition_pair(
    manifest: Mapping[str, Any],
    companion_path: Path,
    full_event: Mapping[str, Any],
) -> dict[str, Any]:
    identity = manifest["packet_identity"]
    transition = full_event.get("prior_transition")
    _companion_require(isinstance(transition, dict), "full_event.prior_transition is missing")
    _companion_require(
        transition.get("schema") == "basalt.native.schema4.prior_transition.v1",
        "full_event.prior_transition schema mismatch",
    )
    _companion_require(transition.get("event_bound") is True, "prior transition is not event-bound")
    pre_name = transition.get("pre_shift_file")
    post_name = transition.get("post_shift_file")
    pre_path = _companion_sidecar_path(companion_path, pre_name, "prior pre-shift sidecar")
    post_path = _companion_sidecar_path(companion_path, post_name, "prior post-shift sidecar")
    pre = _companion_read_json_file(pre_path, "prior pre-shift metadata")
    post = _companion_read_json_file(post_path, "prior post-shift metadata")
    for phase, metadata, expected_name in (("pre_shift", pre, pre_name), ("post_shift", post, post_name)):
        _companion_require(metadata.get("schema") == "basalt.native.schema4.prior_transition.v1", f"prior {phase} schema mismatch")
        _companion_require(metadata.get("phase") == phase, f"prior {phase} phase mismatch")
        _companion_require(metadata.get("run_uuid") == manifest["run_uuid"], f"prior {phase} run_uuid mismatch")
        _companion_require(metadata.get("event_ordinal") == identity["event_ordinal"], f"prior {phase} event ordinal mismatch")
        _companion_require(metadata.get("primary_kf_timestamp_ns") == identity["primary_kf_timestamp_ns"], f"prior {phase} primary KF mismatch")
        _companion_require(metadata.get("event_state_timestamp_ns") == identity["event_state_timestamp_ns"], f"prior {phase} state timestamp mismatch")
        _companion_require(metadata.get("packet_filename") == identity["packet_filename"], f"prior {phase} packet filename mismatch")
        _companion_require(metadata.get("kfs_all") == identity["kfs_all"], f"prior {phase} kfs_all mismatch")
        _companion_require(metadata.get("kfs_to_marg") == identity["kfs_to_marg"], f"prior {phase} kfs_to_marg mismatch")
        _companion_require(metadata.get("binary") == expected_name.replace(".json", ".bin"), f"prior {phase} binary reference mismatch")
        rows = _companion_integer(metadata.get("rows"), f"prior {phase}.rows", 1)
        cols = _companion_integer(metadata.get("cols"), f"prior {phase}.cols", 1)
        rhs_size = _companion_integer(metadata.get("rhs_size"), f"prior {phase}.rhs_size", 0)
        delta_size = _companion_integer(metadata.get("delta_size"), f"prior {phase}.delta_size", 0)
        _companion_require(rows == cols == rhs_size == delta_size, f"prior {phase} shape/delta mismatch")
        _companion_require(metadata.get("is_sqrt") is True, f"prior {phase} is_sqrt is not true")
        _companion_require(isinstance(metadata.get("order"), list), f"prior {phase}.order is missing")
        _companion_integer_list(
            [int(item["timestamp_ns"]) for item in metadata["order"]],
            f"prior {phase}.order timestamps",
        )
        binary = _companion_sidecar_path(companion_path, metadata.get("binary"), f"prior {phase} binary")
        expected_size = 24 + (rows * cols + rhs_size + delta_size) * 4
        _companion_require(binary.stat().st_size == expected_size, f"prior {phase} binary length mismatch")
        _companion_require(len(metadata["order"]) > 0, f"prior {phase}.order is empty")
    pre_binary = _companion_sidecar_path(companion_path, pre["binary"], "prior pre-shift binary")
    post_binary = _companion_sidecar_path(companion_path, post["binary"], "prior post-shift binary")
    pre_raw = pre_binary.read_bytes()
    post_raw = post_binary.read_bytes()
    _companion_require(pre_raw[:24] == post_raw[:24], "prior pre/post headers differ")
    rows, cols, rhs_size = struct.unpack_from("<QQQ", pre_raw, 0)
    _companion_require(len(pre_raw) == len(post_raw), "prior pre/post payload length differs")
    _companion_require(
        pre_raw[24 : 24 + rows * cols * 4] == post_raw[24 : 24 + rows * cols * 4],
        "prior pre/post H payload differs",
    )
    delta_offset = 24 + (rows * cols + rhs_size) * 4
    _companion_require(pre_raw[delta_offset:] == post_raw[delta_offset:], "prior pre/post delta payload differs")
    return {
        "pre_metadata": pre,
        "post_metadata": post,
        "pre_path": str(pre_path),
        "post_path": str(post_path),
        "pre_sha256": _sha256_file(pre_path),
        "post_sha256": _sha256_file(post_path),
        "pre_binary_sha256": _sha256_file(pre_binary),
        "post_binary_sha256": _sha256_file(post_binary),
        "event_bound": True,
        "order_exact": True,
        "delta_exact": True,
    }


def _companion_validate_full_event(
    manifest: Mapping[str, Any],
    companion_path: Path,
    q2: Mapping[str, Any],
    owner: Mapping[str, Any],
    packet_binding: Mapping[str, Any],
) -> dict[str, Any] | None:
    """Validate the expanded same-event native lifecycle/source sidecars."""
    root = companion_path.resolve().parent
    full_path = root / "event0_full.json"
    if not full_path.is_file():
        return None
    full_event = _companion_read_json_file(full_path, "full event sidecar")
    identity = manifest["packet_identity"]
    _companion_require(full_event.get("schema") == "basalt.native.schema4.full_event.v1", "full event schema mismatch")
    _companion_require(full_event.get("schema_version") == 1, "unsupported full event schema version")
    _companion_require(full_event.get("run_uuid") == manifest["run_uuid"], "full event run_uuid mismatch")
    for key in ("event_ordinal", "primary_kf_timestamp_ns", "event_state_timestamp_ns", "packet_filename"):
        _companion_require(full_event.get(key) == identity[key], f"full event {key} mismatch")
    _companion_require(full_event.get("packet_sha256") == packet_binding.get("computed_sha256"), "full event packet SHA mismatch")
    for key in ("kfs_all", "kfs_to_marg"):
        _companion_require(full_event.get(key) == identity[key], f"full event {key} mismatch")
    kfs_all = _companion_integer_list(full_event.get("kfs_all"), "full_event.kfs_all")
    kfs_to_marg = _companion_integer_list(full_event.get("kfs_to_marg"), "full_event.kfs_to_marg")
    _companion_require(kfs_to_marg and set(kfs_to_marg).issubset(kfs_all), "full event selected KFs are invalid")
    for key in ("kfs_after_selection", "poses_to_marg", "states_to_marg_all", "states_to_marg_vel_bias", "lost_landmarks"):
        _companion_integer_list(full_event.get(key), f"full_event.{key}")
    selection = full_event.get("selection_diagnostic")
    _companion_require(isinstance(selection, dict), "full event selection diagnostic is missing")
    records = selection.get("records")
    _companion_require(isinstance(records, list) and records, "full event selection records are missing")
    for index, record in enumerate(records):
        _companion_require(isinstance(record, dict), f"full event selection record {index} is malformed")
        for key in ("iteration", "candidate_order", "candidate_kf_timestamp_ns"):
            _companion_integer(record.get(key), f"full_event.selection_diagnostic.records[{index}].{key}", 0)
        _companion_require(record.get("phase") in ("ratio", "fallback"), f"full event selection record {index} phase is invalid")
        _companion_require(isinstance(record.get("eligible"), bool), f"full event selection record {index} eligibility is invalid")
        for key in ("connected", "num_points_kf", "ratio_bits", "score_bits", "denom_bits"):
            if record.get(key) is not None:
                _companion_integer(record[key], f"full_event.selection_diagnostic.records[{index}].{key}", 0)
    _companion_require(selection.get("selected_kfs") == kfs_to_marg, "full event selected KFs disagree with diagnostic")
    partition = full_event.get("partition")
    _companion_require(isinstance(partition, dict), "full event partition is missing")
    aom_pre = _companion_validate_event_aom(partition.get("aom_pre"), "full_event.partition.aom_pre")
    aom_post = _companion_validate_event_aom(partition.get("aom_post"), "full_event.partition.aom_post")
    _companion_require(aom_pre == manifest["aom_binding"], "full event AOM pre disagrees with companion")
    idx_keep = _companion_integer_list(partition.get("idx_to_keep"), "full_event.partition.idx_to_keep")
    idx_marg = _companion_integer_list(partition.get("idx_to_marg"), "full_event.partition.idx_to_marg")
    _companion_require(not set(idx_keep).intersection(idx_marg), "full event index partition overlaps")
    _companion_require(sorted(idx_keep + idx_marg) == list(range(q2["cols"])), "full event index partition does not cover Q2 columns")
    _companion_require(isinstance(full_event.get("kf_transition"), dict), "full event KF transition is missing")
    transition = full_event["kf_transition"]
    _companion_require(transition.get("operation") == "remove_selected_hosts_and_observations", "unexpected native KF transition operation")
    _companion_require(transition.get("source_kfs") == kfs_to_marg, "native KF transition source mismatch")
    _companion_require(transition.get("destination_kfs") is None, "native KF transition destination must remain absent")
    owner_data = owner.get("data", {})
    prior_pair = _companion_validate_prior_transition_pair(manifest, companion_path, full_event)

    def validate_source_blob(name: Any, label: str) -> tuple[Path, bytes]:
        path = _companion_sidecar_path(companion_path, name, label)
        return path, path.read_bytes()

    visual_name = owner_data.get("visual_source_file")
    visual_path, visual_raw = validate_source_blob(visual_name, "visual Q2 source")
    visual_source = owner_data.get("visual_source")
    _companion_require(isinstance(visual_source, list), "source owner visual_source is missing")
    factors = owner_data.get("visual_factors")
    _companion_require(isinstance(factors, list) and len(factors) == len(visual_source), "visual source/factor count mismatch")
    visual_spans: list[tuple[int, int]] = []
    for index, item in enumerate(visual_source):
        _companion_require(isinstance(item, dict), f"visual_source[{index}] is malformed")
        for key in ("ordinal", "landmark_id", "storage_rows", "storage_cols", "padding_idx", "landmark_idx", "res_idx", "storage_byte_offset", "storage_byte_length"):
            _companion_integer(item.get(key), f"visual_source[{index}].{key}", 0)
        _companion_require(item["ordinal"] == index, f"visual_source[{index}] ordinal mismatch")
        _companion_require(item["landmark_id"] == factors[index].get("landmark_id"), f"visual_source[{index}] landmark mismatch")
        offset = item["storage_byte_offset"]
        length = item["storage_byte_length"]
        _companion_require(length == item["storage_rows"] * item["storage_cols"] * 4, f"visual_source[{index}] byte length mismatch")
        _companion_require(offset + length <= len(visual_raw), f"visual_source[{index}] exceeds source blob")
        digest = _hex_sha256(item.get("storage_sha256"), f"visual_source[{index}].storage_sha256")
        _companion_require(digest == _sha256_bytes(visual_raw[offset : offset + length]), f"visual_source[{index}] payload digest mismatch")
        observations = item.get("observations")
        _companion_require(isinstance(observations, list) and observations, f"visual_source[{index}] observations are missing")
        for obs_index, observation in enumerate(observations):
            _companion_require(isinstance(observation, dict), f"visual_source[{index}].observations[{obs_index}] is malformed")
            for key in ("frame_id", "cam_id", "storage_row_start", "storage_row_count"):
                _companion_integer(observation.get(key), f"visual_source[{index}].observations[{obs_index}].{key}", 0)
            _companion_require(isinstance(observation.get("in_aom"), bool), f"visual_source[{index}].observations[{obs_index}].in_aom is invalid")
        visual_spans.append((offset, length))
    _companion_require(visual_spans == [(0 if index == 0 else sum(length for _, length in visual_spans[:index]), length) for index, (_, length) in enumerate(visual_spans)], "visual source blobs are not contiguous")

    imu_name = owner_data.get("imu_source_file")
    imu_path, imu_raw = validate_source_blob(imu_name, "IMU Q2 source")
    imu_source = owner_data.get("imu_source")
    _companion_require(isinstance(imu_source, list), "source owner imu_source is missing")
    previous_end = 0
    for index, item in enumerate(imu_source):
        _companion_require(isinstance(item, dict), f"imu_source[{index}] is malformed")
        for key in ("ordinal", "start_timestamp_ns", "end_timestamp_ns", "start_offset", "end_offset", "row_start", "row_count", "matrix_rows", "matrix_cols", "rhs_size", "byte_offset", "byte_length"):
            _companion_integer(item.get(key), f"imu_source[{index}].{key}", 0)
        _companion_require(item["ordinal"] == index, f"imu_source[{index}] ordinal mismatch")
        _companion_require(item["byte_offset"] == previous_end, f"imu_source[{index}] blob has a gap or overlap")
        _companion_require(item["byte_length"] == (item["matrix_rows"] * item["matrix_cols"] + item["rhs_size"]) * 4, f"imu_source[{index}] byte length mismatch")
        _companion_require(item["byte_offset"] + item["byte_length"] <= len(imu_raw), f"imu_source[{index}] exceeds source blob")
        digest = _hex_sha256(item.get("sha256"), f"imu_source[{index}].sha256")
        _companion_require(digest == _sha256_bytes(imu_raw[item["byte_offset"] : item["byte_offset"] + item["byte_length"]]), f"imu_source[{index}] payload digest mismatch")
        previous_end = item["byte_offset"] + item["byte_length"]
    _companion_require(previous_end == len(imu_raw), "IMU source blob has trailing bytes")
    return {
        "path": str(full_path),
        "sha256": _sha256_file(full_path),
        "data": full_event,
        "prior_pair": prior_pair,
        "visual_source": {"path": str(visual_path), "sha256": _sha256_file(visual_path), "factor_count": len(visual_source)},
        "imu_source": {"path": str(imu_path), "sha256": _sha256_file(imu_path), "block_count": len(imu_source)},
        "aom_pre_exact": True,
        "aom_post_exact": True,
        "partition_exact": True,
        "selection_exact": True,
        "lost_landmarks_exact": True,
        "native_transition": {"operation": transition["operation"], "destination": "native_absent"},
    }


def _companion_field_matrix(
    manifest: Mapping[str, Any],
    native_doc: Mapping[str, Any],
    sections: Mapping[int, Mapping[str, Any]],
    owner: Mapping[str, Any],
    partition: Mapping[str, Any] | None,
    optical_flow_archive: Mapping[str, Any] | None,
    optical_flow_source_root: Path | None,
    full_event: Mapping[str, Any] | None = None,
) -> dict[str, dict[str, Any]]:
    """Classify each contract field from evidence actually validated here.

    The manifest's optional ``availability`` object is descriptive metadata;
    it is intentionally not trusted to upgrade a missing capture.  This
    function only reports an exact field when the corresponding native packet,
    companion section, or same-run image archive was checked by the bridge.
    """
    owner_data = owner.get("data") if isinstance(owner, Mapping) else None
    if not isinstance(owner_data, Mapping):
        owner_data = {}
    selection = sections.get(13, {}).get("parsed")
    selection_exact = isinstance(selection, Mapping)
    q2_exact = isinstance(sections.get(3, {}).get("parsed"), Mapping)
    prior_exact = isinstance(sections.get(15, {}).get("parsed"), Mapping)
    aom_exact = isinstance(sections.get(1, {}).get("parsed"), Mapping)
    partition_exact = not _companion_partition_missing(partition)
    packet_exact = bool(native_doc.get("frame_poses")) and bool(native_doc.get("frame_states"))
    image_exact = optical_flow_archive is not None and optical_flow_source_root is not None
    full_data = full_event.get("data") if isinstance(full_event, Mapping) else None
    full_exact = isinstance(full_event, Mapping) and isinstance(full_data, Mapping)
    prior_pair_exact = full_exact and full_event.get("prior_pair", {}).get("event_bound") is True
    visual_source_exact = full_exact and isinstance(full_event.get("visual_source"), Mapping)
    imu_source_exact = full_exact and isinstance(full_event.get("imu_source"), Mapping)
    visual_factors = owner_data.get("visual_factors")
    visual_factor_meta = isinstance(visual_factors, list) and all(
        isinstance(item, Mapping) and "landmark_id" in item and "row_start" in item
        for item in visual_factors
    )
    visual_storage_values = isinstance(visual_factors, list) and all(
        isinstance(item, Mapping) and all(
            key in item for key in ("storage_rows", "storage_cols", "padding_idx", "landmark_idx", "res_idx")
        )
        for item in visual_factors
    )
    damping_absent = owner_data.get("damping_rows") == 0

    def exact(status: str, evidence: list[str], capture_site: str, reason: str | None = None) -> dict[str, Any]:
        result: dict[str, Any] = {
            "status": status,
            "evidence": evidence,
            "capture_site": capture_site,
        }
        if reason is not None:
            result["reason"] = reason
        return result

    matrix: dict[str, dict[str, Any]] = {
        "MargData.aom": exact(
            "exact_direct_native", ["native packet", "AOM_PRE"],
            "MargData::aom and companion tag AOM_PRE",
        ) if aom_exact else exact(
            "missing", [], "MargData::aom", "AOM_PRE was not validated",
        ),
        "MargData.abs_H": exact(
            "exact_direct_native", ["native packet abs_H"],
            "MargData cereal packet serializer",
        ) if native_doc.get("abs_system") else exact(
            "missing", [], "MargData cereal packet", "native abs_H is absent",
        ),
        "MargData.abs_b": exact(
            "exact_direct_native", ["native packet abs_b"],
            "MargData cereal packet serializer",
        ) if native_doc.get("abs_system") else exact(
            "missing", [], "MargData cereal packet", "native abs_b is absent",
        ),
        "MargData.frame_poses": exact(
            "exact_direct_native", ["native packet frame_poses"],
            "MargData cereal packet serializer",
        ) if packet_exact else exact(
            "missing", [], "MargData cereal packet", "frame pose table is absent",
        ),
        "MargData.frame_states": exact(
            "exact_direct_native", ["native packet frame_states"],
            "MargData cereal packet serializer",
        ) if packet_exact else exact(
            "missing", [], "MargData cereal packet", "frame state table is absent",
        ),
        "MargData.kfs_all": exact(
            "exact_direct_native", ["packet_identity.kfs_all", "native packet kfs_all"],
            "MargData cereal packet serializer",
        ),
        "MargData.kfs_to_marg": exact(
            "exact_direct_native", ["packet_identity.kfs_to_marg", "native packet kfs_to_marg"],
            "MargData cereal packet serializer",
        ),
        "MargData.use_imu": exact(
            "exact_direct_native", ["native packet use_imu", "selection branch"],
            "MargData cereal packet serializer",
        ),
        "MargData.opt_flow_res": exact(
            "exact_image_archive", ["same-run images archive", "timestamp/frame-map"],
            "MargDataSaver images/<timestamp>.cereal side archive",
        ) if image_exact else exact(
            "missing", [], "MargDataSaver images/<timestamp>.cereal", "same-run image archive is not bound",
        ),
        "MargLinData.is_sqrt": exact(
            "exact_companion", ["SELECTION_DIAGNOSTIC.is_lin_sqrt", "SELECTION_DIAGNOSTIC.marg_is_sqrt"],
            "Q2 boundary selection diagnostic",
        ) if selection_exact else exact(
            "missing", [], "Q2 boundary selection diagnostic", "runtime branch marker is absent",
        ),
        "MargLinData.order": exact(
            "exact_companion", ["event0_full.partition.aom_post", "event0_prior_pre_shift.json", "event0_prior_post_shift.json"],
            "linearization_abs_qr.cpp prior-row source",
        ) if prior_pair_exact else exact(
            "partial", ["AOM_PRE only"],
            "linearization_abs_qr.cpp prior-row source",
            "event-bound pre/post prior sidecars are absent",
        ),
        "MargLinData.H and MargLinData.b": exact(
            "raw_native_exact", ["PRIOR_INPUT_METADATA", "PRIOR_INPUT"],
            "linearization_abs_qr.cpp prior-row source",
            "raw native prior source is not a Rust PriorData semantic adapter",
        ) if prior_exact else exact(
            "missing", [], "linearization_abs_qr.cpp prior-row source", "PRIOR_INPUT is absent",
        ),
        "prior current delta": exact(
            "exact_companion", ["event0_prior_pre_shift.bin", "event0_prior_post_shift.bin"],
            "sqrt_keypoint_vio.cpp prior delta shift",
        ) if prior_pair_exact else exact(
            "missing", [], "sqrt_keypoint_vio.cpp prior delta shift", "event-bound prior delta was not captured",
        ),
        "prior FEJ point / anchor": exact(
            "native_raw_exact_unmapped", ["native frame state linearized branches"],
            "PoseStateWithLin frame-state packet fields",
            "the native anchor bits are captured exactly, but no deterministic Rust schema4 tangent/layout adapter exists; the Rust extension mapping remains a separate typed status",
        ) if packet_exact else exact(
            "missing", [], "PoseStateWithLin frame-state packet fields",
            "native frame-state anchors are absent",
        ),
        "Q2Jp/Q2r final": exact(
            "exact_companion", ["Q2_FINAL section", "Q2 shape binding"],
            "LinearizationAbsQR::get_dense_Q2Jp_Q2r caller hook",
        ) if q2_exact else exact(
            "missing", [], "LinearizationAbsQR::get_dense_Q2Jp_Q2r", "Q2_FINAL is absent",
        ),
        "visual Q2 source": exact(
            "exact_companion", ["visual_qr_storage.bin", "q2_source_owner.visual_source"],
            "LinearizationAbsQR landmark owner hook",
        ) if visual_source_exact and visual_storage_values else exact(
            "partial", ["visual factor shape metadata"],
            "LinearizationAbsQR landmark owner hook",
            "validated visual source values are absent",
        ) if visual_storage_values else exact(
            "missing", [], "LinearizationAbsQR landmark owner hook", "visual storage metadata is absent",
        ),
        "visual factor identity and observation rows": exact(
            "exact_companion", ["q2_source_owner.visual_source host/observations", "visual row spans"],
            "LinearizationAbsQR landmark owner hook",
        ) if visual_source_exact and visual_factor_meta else exact(
            "partial", ["landmark IDs", "visual row spans"],
            "LinearizationAbsQR landmark owner hook",
            "host/target/camera observation identities are absent",
        ) if visual_factor_meta else exact(
            "missing", [], "LinearizationAbsQR landmark owner hook", "visual factor metadata is absent",
        ),
        "IMU Q2 source": exact(
            "exact_companion", ["imu_q2_source.bin", "q2_source_owner.imu_source"],
            "LinearizationAbsQR IMU owner hook",
        ) if imu_source_exact else exact(
            "partial", ["aggregate IMU row count"],
            "LinearizationAbsQR IMU owner hook",
            "per-ImuBlock Jp/r and timestamp pair are absent",
        ) if owner_data.get("imu_rows", 0) else exact(
            "exact_absent", ["owner imu_rows=0"],
            "LinearizationAbsQR IMU owner hook",
        ),
        "pose damping Q2 rows": exact(
            "exact_absent", ["owner damping_rows=0"],
            "LinearizationAbsQR pose damping row source",
        ) if damping_absent else exact(
            "missing", [], "LinearizationAbsQR pose damping row source", "damping rows are present but raw diagonal is not captured",
        ),
        "Q2 row topology": exact(
            "exact_companion", ["q2_source_owner.row_topology", "visual/IMU source spans"],
            "LinearizationAbsQR::get_dense_Q2Jp_Q2r",
        ) if full_exact and q2_exact else exact(
            "partial", ["visual row spans", "aggregate category counts"],
            "LinearizationAbsQR::get_dense_Q2Jp_Q2r",
            "source spans are not individually recorded",
        ) if q2_exact else exact(
            "missing", [], "LinearizationAbsQR::get_dense_Q2Jp_Q2r", "Q2 topology is absent",
        ),
        "AOM pre/post and marginal partition": exact(
            "exact_companion", ["event0_full.partition.aom_pre", "event0_full.partition.aom_post", "idx_to_keep", "idx_to_marg"],
            "sqrt_keypoint_vio.cpp selection/partition hooks",
        ) if full_exact and partition_exact and aom_exact else exact(
            "partial", ["AOM_PRE", "PARTITION target sets"],
            "sqrt_keypoint_vio.cpp selection/partition hooks",
            "post-removal AOM and exact index partition are absent",
        ) if partition_exact and aom_exact else exact(
            "missing", [], "sqrt_keypoint_vio.cpp selection/partition hooks", "partition evidence is incomplete",
        ),
        "kf_to_marg equivalent": exact(
            "native_absent_optional", ["event0_full.kf_transition", "kfs_to_marg"],
            "sqrt_keypoint_vio.cpp selection/removal transition",
            "native removeKeyframes has no destination/reanchor pair; excluded from native-compatible required fields",
        ),
        "selection denominators/scores": exact(
            "exact_companion", ["event0_full.selection_diagnostic.records"],
            "sqrt_keypoint_vio.cpp keyframe selection loop",
        ) if full_exact else exact(
            "missing", [], "sqrt_keypoint_vio.cpp keyframe selection loop", "candidate counts, ratios, and scores are not in the captured diagnostic",
        ),
        "lost_landmarks": exact(
            "exact_companion", ["event0_full.lost_landmarks"],
            "sqrt_keypoint_vio.cpp marginalize(lost_landmaks)",
        ) if full_exact else exact(
            "missing", [], "sqrt_keypoint_vio.cpp marginalize(lost_landmaks)", "lost landmark IDs are not captured",
        ),
        "internal new prior": exact(
            "exact_companion", ["event0_prior_pre_shift", "event0_prior_post_shift", "event0_full.prior_transition"],
            "sqrt_keypoint_vio.cpp prior output transition",
        ) if prior_pair_exact else exact(
            "missing", [], "sqrt_keypoint_vio.cpp prior output transition", "event-bound pre-shift/delta/post-shift prior sections are absent",
        ),
        "PoseStateWithLin backup_*": exact(
            "not_required", [], "PoseStateWithLin rollback-only members",
            "backup values are not part of the packet contract",
        ),
        "optical-flow observations/images": exact(
            "exact_image_archive", ["same-run images archive", "raw-u16 payload hashes"],
            "MargDataSaver separate images archive",
        ) if image_exact else exact(
            "missing", [], "MargDataSaver separate images archive", "same-run image archive is not bound",
        ),
    }
    _companion_require(set(matrix) == set(COMPANION_FIELD_MATRIX), "field matrix implementation is incomplete")
    return matrix


def _companion_partition_missing(partition: Any) -> list[str]:
    if not isinstance(partition, dict):
        return ["marginalization.targets_complete"]
    if partition.get("state") not in ("captured", "complete"):
        return ["marginalization.targets_complete"]
    required = (
        "poses_to_marg",
        "states_to_marg_all",
        "states_to_marg_vel_bias",
    )
    missing = [f"companion.partition.{name}" for name in required if name not in partition]
    if "last_state_to_marg" not in partition and "event_state_timestamp_ns" not in partition:
        missing.append("companion.partition.last_state_to_marg")
    return missing


def _companion_validate_partition(
    partition: Any, manifest: Mapping[str, Any], q2: Mapping[str, Any]
) -> dict[str, Any]:
    _companion_require(isinstance(partition, dict), "partition sidecar must be a JSON object")
    _companion_require(
        partition.get("schema") == "basalt.native.schema4.partition.v1",
        "partition sidecar schema mismatch",
    )
    _companion_require(
        partition.get("run_uuid") == manifest.get("run_uuid"),
        "partition sidecar run_uuid mismatch",
    )
    identity = manifest["packet_identity"]
    for key in ("event_ordinal", "primary_kf_timestamp_ns", "event_state_timestamp_ns"):
        if key in partition:
            _companion_require(
                partition[key] == identity[key],
                f"partition sidecar {key} mismatch",
            )
    state_timestamp = partition.get(
        "last_state_to_marg", partition.get("event_state_timestamp_ns")
    )
    _companion_require(
        state_timestamp == identity["event_state_timestamp_ns"],
        "partition sidecar last-state timestamp mismatch",
    )
    _companion_require(
        partition.get("kfs_all") == identity["kfs_all"],
        "partition sidecar kfs_all mismatch",
    )
    _companion_require(
        partition.get("kfs_to_marg") == identity["kfs_to_marg"],
        "partition sidecar kfs_to_marg mismatch",
    )
    if "packet_filename" in partition:
        _companion_require(
            partition.get("packet_filename") == identity["packet_filename"],
            "partition sidecar packet_filename mismatch",
        )
    shape = manifest["row_topology_binding"]["q2_shape"]
    for name, expected in (
        ("q2_rows", q2["rows"]),
        ("q2_cols", q2["cols"]),
        ("q2_rhs_size", q2["rhs_size"]),
    ):
        _companion_require(partition.get(name) == expected, f"partition sidecar {name} mismatch")
    _companion_require(
        partition.get("q2_cols") == shape["cols"],
        "partition sidecar Q2 shape disagrees with manifest",
    )
    _companion_require(partition.get("state") in ("captured", "complete"), "partition sidecar is not captured")
    for name in (
        "idx_to_keep", "idx_to_marg", "poses_to_marg", "states_to_marg_all",
        "states_to_marg_vel_bias", "lost_landmarks",
    ):
        if name in partition:
            _companion_integer_list(partition[name], f"partition.{name}")
    missing = _companion_partition_missing(partition)
    _companion_require(not missing, "partition sidecar missing fields: " + ", ".join(missing))
    normalized = dict(partition)
    # The native hook names this same state boundary
    # ``event_state_timestamp_ns``; normalize it for the bridge profile while
    # retaining the original field in the attested payload.
    normalized.setdefault("last_state_to_marg", state_timestamp)
    return normalized


def _companion_validate_packet_identity(
    manifest: Mapping[str, Any], native_packet_path: Path | None
) -> dict[str, Any]:
    identity = manifest.get("packet_identity")
    _companion_require(isinstance(identity, dict), "packet_identity: expected object")
    filename = identity.get("packet_filename")
    _companion_require(isinstance(filename, str) and Path(filename).name == filename, "packet_identity.packet_filename must be a basename")
    declared = identity.get("packet_sha256")
    _companion_require(declared is not None, "packet_identity.packet_sha256 is required for strict binding")
    declared = _hex_sha256(declared, "packet_identity.packet_sha256")
    if native_packet_path is None:
        return {"available": False, "reason": "--native-packet is required for strict companion binding", "declared_sha256": declared}
    try:
        resolved = native_packet_path.resolve(strict=True)
    except OSError as error:
        raise AdapterError(f"companion: native packet cannot be resolved: {error}") from error
    _companion_require(resolved.is_file(), "native packet path is not a file")
    _companion_require(resolved.name == filename, "native packet filename does not match companion identity")
    computed = _sha256_file(resolved)
    _companion_require(declared == computed, "native packet SHA-256 disagrees with companion")
    return {
        "available": True,
        "path": str(resolved),
        "declared_sha256": declared,
        "computed_sha256": computed,
        "attestation": "computed from the same-run native cereal packet",
    }


BOUNDING_SCHEMA = "basalt.native.schema4.binding.v1"
BOUNDING_SCHEMA_V2 = "basalt.native.schema4.binding.v2"

# Native MargData does not carry a Rust-only reanchor destination relation.  A
# native FEJ anchor *does* exist in the frame-state packet, so its raw native
# bits belong to the native-compatible contract; only the absent Rust tangent
# adapter belongs to the extension profile.  Keep the two namespaces explicit
# instead of making a native packet fail a contract that it cannot satisfy by
# construction.
NATIVE_COMPATIBILITY_EXCLUDED_FIELDS = frozenset({
    "kf_to_marg equivalent",
})
RUST_EXTENSION_FIELDS = frozenset({
    "prior FEJ point / anchor",
    "kf_to_marg equivalent",
})


def _bound_wsl_realpath(path_text: str, path: str) -> str:
    """Require a regular, non-symlink WSL file and return its real path."""
    for test_args, condition in ((["test", "-f", path_text], "regular file"),
                                 (["test", "!", "-L", path_text], "non-symlink")):
        try:
            completed = subprocess.run(
                ["wsl.exe", "--", *test_args],
                check=False,
                capture_output=True,
                text=True,
                timeout=30,
            )
        except (OSError, subprocess.SubprocessError) as error:
            raise AdapterError(f"companion: cannot validate WSL path {path_text}: {error}") from error
        _companion_require(completed.returncode == 0, f"{path} must be a {condition} WSL path")
    try:
        completed = subprocess.run(
            ["wsl.exe", "--", "realpath", "-e", "--", path_text],
            check=False,
            capture_output=True,
            text=True,
            timeout=30,
        )
    except (OSError, subprocess.SubprocessError) as error:
        raise AdapterError(f"companion: cannot resolve WSL path {path_text}: {error}") from error
    _companion_require(completed.returncode == 0, f"cannot resolve WSL path {path_text}")
    resolved = completed.stdout.rstrip("\r\n")
    _companion_require(resolved.startswith("/"), f"WSL realpath for {path_text} is malformed")
    return resolved


def _bound_sha256(path_text: Any, path: str) -> tuple[str, str]:
    """Hash a declared artifact path, including a pinned WSL path.

    Strict provenance cannot trust a digest copied into a manifest.  Native
    binaries in this workspace live in WSL, so Windows bridge invocations use
    ``wsl.exe sha256sum`` with an argument vector (never a shell command).
    """
    _companion_require(isinstance(path_text, str) and path_text, f"{path}.path is missing")
    if path_text.startswith("/") and os.name == "nt":
        resolved_wsl = _bound_wsl_realpath(path_text, path)
        try:
            completed = subprocess.run(
                ["wsl.exe", "--", "sha256sum", "--", resolved_wsl],
                check=False,
                capture_output=True,
                text=True,
                timeout=60,
            )
        except (OSError, subprocess.SubprocessError) as error:
            raise AdapterError(f"companion: cannot hash WSL path {path_text}: {error}") from error
        _companion_require(completed.returncode == 0, f"cannot hash WSL path {path_text}")
        match = re.search(r"(?im)^([0-9a-f]{64})\s+", completed.stdout)
        _companion_require(match is not None, f"WSL hash output for {path_text} is malformed")
        # Keep the POSIX spelling in the attestation.  ``Path('/tmp/...')``
        # becomes ``\\tmp\\...`` on the Windows host and must not be fed back
        # to Win32 path operations or used as a misleading local path.
        return resolved_wsl, match.group(1).lower()
    candidate = Path(path_text)
    _companion_require(candidate.is_absolute(), f"{path}.path must be absolute")
    try:
        resolved = candidate.resolve(strict=True)
    except OSError as error:
        raise AdapterError(f"companion: cannot resolve {path}.{path_text}: {error}") from error
    _companion_require(resolved.is_file(), f"{path}.path is not a regular file")
    _companion_require(not candidate.is_symlink(), f"{path}.path must not be a symlink")
    return str(resolved), _sha256_file(resolved)


def _bound_git_output(root_text: str, args: list[str], label: str) -> str:
    """Run a read-only git query against the declared source root."""
    _companion_require(isinstance(root_text, str) and root_text.startswith("/"), f"{label}.root must be an absolute WSL path")
    command = ["wsl.exe", "--", "git", "-C", root_text, *args] if os.name == "nt" else ["git", "-C", root_text, *args]
    try:
        completed = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=30,
        )
    except (OSError, subprocess.SubprocessError) as error:
        raise AdapterError(f"companion: source identity query failed ({label}): {error}") from error
    _companion_require(completed.returncode == 0, f"source identity query failed ({label})")
    # Porcelain status uses the first column for the index state; a leading
    # space is meaningful.  Remove only transport line endings, never call
    # ``strip()`` on the complete output.
    return completed.stdout.rstrip("\r\n")


def _companion_validate_binding_manifest(
    binding_path: Path,
    binding: Mapping[str, Any],
    manifest: Mapping[str, Any],
    companion_path: Path,
    native_packet_path: Path | None,
    optical_flow_archive_path: Path | None,
    optical_flow_archive: Mapping[str, Any] | None,
    packet_binding: Mapping[str, Any],
) -> dict[str, Any]:
    """Recompute every strict provenance digest and source identity."""
    binding_schema = binding.get("schema")
    _companion_require(
        binding_schema in (BOUNDING_SCHEMA, BOUNDING_SCHEMA_V2),
        "binding manifest schema mismatch",
    )
    full_binding = binding_schema == BOUNDING_SCHEMA_V2
    _companion_require(binding.get("run_uuid") == manifest.get("run_uuid"), "binding run_uuid mismatch")
    allowed_roots = binding.get("allowed_roots")
    _companion_require(isinstance(allowed_roots, dict), "binding allowed_roots is missing")
    workspace_root_text = allowed_roots.get("workspace")
    native_source_root_text = allowed_roots.get("native_source")
    configuration_root_text = allowed_roots.get("configuration")
    _companion_require(
        isinstance(workspace_root_text, str) and Path(workspace_root_text).is_absolute(),
        "binding allowed_roots.workspace must be an absolute local path",
    )
    workspace_root = Path(workspace_root_text).resolve(strict=True)
    _companion_require(workspace_root.is_dir(), "binding allowed_roots.workspace is not a directory")
    binding_resolved = binding_path.resolve(strict=True)
    _companion_require(not binding_resolved.is_symlink(), "binding manifest must not be a symlink")
    try:
        binding_resolved.relative_to(workspace_root)
    except ValueError as error:
        raise AdapterError("companion: binding manifest escapes workspace root") from error
    _companion_require(
        isinstance(native_source_root_text, str) and native_source_root_text.startswith("/"),
        "binding allowed_roots.native_source must be an absolute WSL path",
    )
    _companion_require(
        isinstance(configuration_root_text, str) and configuration_root_text.startswith("/"),
        "binding allowed_roots.configuration must be an absolute WSL path",
    )
    files = binding.get("files")
    _companion_require(isinstance(files, dict), "binding files must be an object")
    expected_hashes = {
        "native_executable": manifest.get("native_executable_sha256"),
        "native_library": manifest.get("native_library_sha256"),
        "source_manifest": manifest.get("source_manifest_sha256"),
        "config": manifest.get("configuration", {}).get("config_sha256"),
        "calibration": manifest.get("configuration", {}).get("calibration_sha256"),
        "input": manifest.get("configuration", {}).get("input_sha256"),
    }
    required_file_keys = (
        "native_executable", "native_library", "source_manifest", "config",
        "calibration", "input", "native_packet", "images_archive",
        "companion_manifest", "companion_binary", "source_owner",
    )
    if full_binding:
        required_file_keys += (
            "full_event", "prior_pre_metadata", "prior_pre_binary",
            "prior_post_metadata", "prior_post_binary", "visual_source",
            "imu_source",
        )
    bound: dict[str, dict[str, Any]] = {}
    for key in required_file_keys:
        record = files.get(key)
        _companion_require(isinstance(record, dict), f"binding files.{key} is missing")
        declared = _hex_sha256(record.get("sha256"), f"binding files.{key}.sha256")
        resolved_text, computed = _bound_sha256(record.get("path"), f"binding files.{key}")
        _companion_require(declared == computed, f"binding files.{key} digest disagrees with real path")
        expected = expected_hashes.get(key)
        if expected is not None:
            _companion_require(declared == str(expected).lower(), f"binding files.{key} disagrees with companion declaration")
        is_wsl_path = isinstance(record.get("path"), str) and str(record["path"]).startswith("/") and os.name == "nt"
        bound[key] = {"path": resolved_text, "sha256": computed, "size": None if is_wsl_path else Path(resolved_text).stat().st_size}

    companion_root = companion_path.resolve().parent
    run_root = companion_root.parent
    for key in (
        "native_packet", "images_archive", "companion_manifest", "companion_binary",
        "source_owner", "full_event", "prior_pre_metadata", "prior_pre_binary",
        "prior_post_metadata", "prior_post_binary", "visual_source", "imu_source",
    ):
        if key not in files:
            continue
        declared_path = Path(str(files[key]["path"]))
        if not declared_path.is_absolute() or (declared_path.drive and declared_path.drive != run_root.drive):
            resolved = declared_path.resolve()
        else:
            resolved = declared_path.resolve()
        try:
            resolved.relative_to(run_root)
        except ValueError as error:
            raise AdapterError(f"companion: binding files.{key} escapes run root") from error
        _companion_require(not resolved.is_symlink(), f"binding files.{key} must not be a symlink")

    for key in ("source_manifest", "input"):
        local_path = Path(bound[key]["path"]).resolve(strict=True)
        try:
            local_path.relative_to(workspace_root)
        except ValueError as error:
            raise AdapterError(f"companion: binding files.{key} escapes workspace root") from error

    _companion_require(Path(bound["companion_manifest"]["path"]).resolve() == companion_path.resolve(), "binding companion_manifest path mismatch")
    _companion_require(Path(bound["companion_binary"]["path"]).resolve() == (companion_root / str(manifest["binary"])).resolve(), "binding companion_binary path mismatch")
    owner_name = manifest["row_topology_binding"]["source_owner"]
    _companion_require(Path(bound["source_owner"]["path"]).resolve() == (companion_root / owner_name).resolve(), "binding source_owner path mismatch")
    if native_packet_path is not None:
        _companion_require(Path(bound["native_packet"]["path"]).resolve() == native_packet_path.resolve(), "binding native_packet path mismatch")
    if optical_flow_archive_path is not None:
        _companion_require(Path(bound["images_archive"]["path"]).resolve() == optical_flow_archive_path.resolve(), "binding images_archive path mismatch")
    _companion_require(bound["native_packet"]["sha256"] == packet_binding.get("computed_sha256"), "binding native_packet digest mismatch")
    if full_binding:
        expected_sidecars = {
            "full_event": "event0_full.json",
            "prior_pre_metadata": "event0_prior_pre_shift.json",
            "prior_pre_binary": "event0_prior_pre_shift.bin",
            "prior_post_metadata": "event0_prior_post_shift.json",
            "prior_post_binary": "event0_prior_post_shift.bin",
            "visual_source": "visual_qr_storage.bin",
            "imu_source": "imu_q2_source.bin",
        }
        for key, basename in expected_sidecars.items():
            _companion_require(
                Path(bound[key]["path"]).resolve() == (run_root / "companion" / basename).resolve(),
                f"binding {key} path mismatch",
            )

    source_manifest_path = Path(bound["source_manifest"]["path"])
    try:
        source_manifest = json.loads(source_manifest_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise AdapterError(f"companion: cannot read bound source manifest: {error}") from error
    _companion_require(isinstance(source_manifest, dict), "bound source manifest must be an object")
    _companion_require(source_manifest.get("pinned_commit") == manifest.get("pinned_commit"), "bound source manifest commit mismatch")
    build = source_manifest.get("build")
    _companion_require(isinstance(build, dict), "bound source manifest build record is missing")
    _companion_require(str(build.get("binary_sha256", "")).lower() == bound["native_executable"]["sha256"], "source manifest executable digest mismatch")
    _companion_require(str(build.get("library_sha256", "")).lower() == bound["native_library"]["sha256"], "source manifest library digest mismatch")
    source_files = source_manifest.get("source_files")
    _companion_require(isinstance(source_files, dict) and source_files, "bound source manifest source_files is missing")
    source_root = source_manifest.get("temporary_tree")
    _companion_require(isinstance(source_root, str) and source_root.startswith("/"), "bound source manifest temporary_tree is missing")
    source_file_attestations: dict[str, str] = {}
    for relative, expected in source_files.items():
        _hex_sha256(expected, f"source_manifest.source_files.{relative}")
        relative_text = str(relative)
        relative_norm = posixpath.normpath(relative_text)
        _companion_require(
            not relative_text.startswith("/")
            and relative_norm not in ("..", ".")
            and not relative_norm.startswith("../"),
            f"source_manifest.source_files.{relative}: path escapes source root",
        )
        source_file_path = source_root.rstrip("/") + "/" + relative_norm
        _, computed = _bound_sha256(source_file_path, f"source_manifest.source_files.{relative}")
        _companion_require(computed == str(expected).lower(), f"source manifest source file digest mismatch: {relative}")
        source_file_attestations[str(relative)] = computed

    source_identity = binding.get("source_identity")
    _companion_require(isinstance(source_identity, dict), "binding source_identity is missing")
    _companion_require(source_identity.get("root") == source_root, "binding source root mismatch")
    _companion_require(source_identity.get("pinned_commit") == manifest.get("pinned_commit"), "binding source commit mismatch")
    _companion_require(_bound_git_output(source_root, ["rev-parse", "HEAD"], "HEAD") == source_identity.get("head"), "binding source HEAD mismatch")
    _companion_require(_bound_git_output(source_root, ["rev-parse", "HEAD^{tree}"], "tree") == source_identity.get("base_tree"), "binding source tree mismatch")
    status = _bound_git_output(source_root, ["status", "--porcelain=v1", "--ignore-submodules=all"], "status")
    status_lines = source_identity.get("status_lines")
    _companion_require(status.splitlines() == status_lines, "binding source status mismatch")
    _companion_require(source_identity.get("diagnostic_patches") == source_manifest.get("diagnostic_patches"), "binding diagnostic patch map mismatch")
    status_file_hashes = source_identity.get("status_file_hashes")
    _companion_require(isinstance(status_file_hashes, dict), "binding status_file_hashes is missing")
    status_paths = []
    for line in status_lines:
        _companion_require(isinstance(line, str) and len(line) >= 4, "binding source status line is malformed")
        relative = line[3:]
        relative_norm = posixpath.normpath(relative)
        _companion_require(
            relative_norm == relative
            and not relative.startswith("/")
            and relative_norm not in ("..", ".")
            and not relative_norm.startswith("../"),
            f"binding source status path escapes source root: {relative}",
        )
        status_paths.append(relative)
    _companion_require(set(status_file_hashes) == set(status_paths), "binding source status file allowlist mismatch")
    for relative in status_paths:
        expected_status_sha = _hex_sha256(
            status_file_hashes[relative], f"source_identity.status_file_hashes.{relative}"
        )
        _, computed_status_sha = _bound_sha256(
            source_root.rstrip("/") + "/" + relative,
            f"source_identity.status_file_hashes.{relative}",
        )
        _companion_require(
            computed_status_sha == expected_status_sha,
            f"source status file digest mismatch: {relative}",
        )
    # Native executable/library are WSL artifacts from this exact temporary
    # source tree.  Resolve and constrain them before accepting their hashes;
    # a valid digest for an unrelated file is not a valid build binding.
    source_root_norm = posixpath.normpath(source_root)
    native_source_norm = posixpath.normpath(native_source_root_text)
    _companion_require(source_root_norm == native_source_norm, "binding native source root disagrees with source manifest")
    for key in ("native_executable", "native_library"):
        artifact_path = bound[key]["path"]
        _companion_require(artifact_path.startswith("/"), f"binding {key} is not a WSL path")
        try:
            common = posixpath.commonpath([source_root_norm, posixpath.normpath(artifact_path)])
        except ValueError as error:
            raise AdapterError(f"companion: binding {key} path namespace mismatch") from error
        _companion_require(common == source_root_norm, f"binding {key} escapes bound source root")

    configuration_norm = posixpath.normpath(configuration_root_text)
    for key in ("config", "calibration"):
        artifact_path = bound[key]["path"]
        _companion_require(artifact_path.startswith("/"), f"binding {key} is not a WSL path")
        try:
            common = posixpath.commonpath([configuration_norm, posixpath.normpath(artifact_path)])
        except ValueError as error:
            raise AdapterError(f"companion: binding {key} path namespace mismatch") from error
        _companion_require(common == configuration_norm, f"binding {key} escapes configuration root")

    archive_binding = bound["images_archive"]
    if optical_flow_archive is not None:
        declared_archive_sha = optical_flow_archive.get("archive_sha256")
        if declared_archive_sha is not None:
            _companion_require(str(declared_archive_sha).lower() == archive_binding["sha256"], "images archive declared digest mismatch")
    return {
        "schema": binding_schema,
        "path": str(binding_path.resolve()),
        "sha256": _sha256_file(binding_path.resolve()),
        "run_uuid": manifest["run_uuid"],
        "files": bound,
        "source_identity": {
            "root": source_root,
            "head": source_identity["head"],
            "base_tree": source_identity["base_tree"],
            "status_lines": list(source_identity["status_lines"]),
            "status_file_hashes": {
                str(relative): str(status_file_hashes[relative]).lower()
                for relative in status_paths
            },
            "diagnostic_patches": dict(source_identity["diagnostic_patches"]),
            "source_files": source_file_attestations,
        },
        "attestation": "all declared hashes recomputed from real paths; source commit/tree/status/patch map checked",
    }


def _validate_companion(
    manifest: Mapping[str, Any],
    companion_path: Path,
    native_doc: Mapping[str, Any],
    native_packet_path: Path | None,
    optical_flow_archive: Mapping[str, Any] | None,
    optical_flow_source_root: Path | None = None,
    partition_path: Path | None = None,
    binding_manifest: Mapping[str, Any] | None = None,
    binding_manifest_path: Path | None = None,
    optical_flow_archive_path: Path | None = None,
) -> dict[str, Any]:
    _companion_require(manifest.get("schema") == COMPANION_SCHEMA, "manifest schema mismatch")
    _companion_require(manifest.get("schema_version") == 1, "unsupported manifest schema version")
    for key in ("run_uuid", "pinned_commit", "native_executable_sha256", "native_library_sha256", "source_manifest_sha256"):
        value = manifest.get(key)
        _companion_require(isinstance(value, str) and value, f"manifest {key} is missing")
    configuration = manifest.get("configuration")
    _companion_require(isinstance(configuration, dict), "configuration: expected object")
    for key in ("config_sha256", "calibration_sha256", "input_sha256"):
        _hex_sha256(configuration.get(key), f"configuration.{key}")
    _hex_sha256(manifest.get("native_executable_sha256"), "native_executable_sha256")
    _hex_sha256(manifest.get("native_library_sha256"), "native_library_sha256")
    _hex_sha256(manifest.get("source_manifest_sha256"), "source_manifest_sha256")
    identity = manifest.get("packet_identity")
    _companion_require(isinstance(identity, dict), "packet_identity: expected object")
    filename = identity.get("packet_filename")
    _companion_require(
        isinstance(filename, str) and Path(filename).name == filename,
        "packet_identity.packet_filename must be a basename",
    )
    _companion_integer(identity.get("event_ordinal"), "packet_identity.event_ordinal", 0)
    primary = _companion_integer(identity.get("primary_kf_timestamp_ns"), "packet_identity.primary_kf_timestamp_ns")
    event_state = _companion_integer(identity.get("event_state_timestamp_ns"), "packet_identity.event_state_timestamp_ns")
    kfs_all = _companion_integer_list(identity.get("kfs_all"), "packet_identity.kfs_all")
    kfs_to_marg = _companion_integer_list(identity.get("kfs_to_marg"), "packet_identity.kfs_to_marg")
    _companion_require(kfs_to_marg, "packet_identity.kfs_to_marg must be non-empty for a strict packet")
    _companion_require(primary in kfs_to_marg, "primary KF is not in kfs_to_marg")
    _companion_require(set(kfs_to_marg).issubset(kfs_all), "kfs_to_marg is not a subset of kfs_all")
    packet_binding = _companion_validate_packet_identity(manifest, native_packet_path)
    _companion_require(manifest.get("pinned_commit") == native_doc["source"]["commit"], "pinned commit disagrees with native packet")
    _companion_require(kfs_all == native_doc["kfs_all"], "kfs_all disagrees with native packet")
    _companion_require(kfs_to_marg == native_doc["kfs_to_marg"], "kfs_to_marg disagrees with native packet")
    _companion_require(primary == kfs_to_marg[0], "primary KF must be the first selected KF")
    _companion_require(event_state in [entry["timestamp_ns"] for entry in native_doc["frame_states"]], "event state timestamp is absent from native frame_states")

    binary_name = manifest.get("binary")
    _companion_require(
        isinstance(binary_name, str) and binary_name and Path(binary_name).name == binary_name,
        "binary must be a basename",
    )
    companion_root = companion_path.resolve().parent
    binary_path = (companion_root / binary_name).resolve()
    _companion_require(binary_path.parent == companion_root, "binary escapes companion directory")
    sections, binary_sha = _companion_parse_binary(manifest, binary_path)
    _companion_require(1 in sections, "required AOM_PRE section is missing")
    _companion_require(3 in sections, "required Q2_FINAL section is missing")
    _companion_require(13 in sections, "required SELECTION_DIAGNOSTIC section is missing")
    _companion_validate_selection(manifest, sections[13]["parsed"])
    q2 = sections[3]["parsed"]
    _companion_require(q2["cols"] == manifest["aom_binding"]["total_size"], "Q2 columns disagree with AOM width")
    _companion_require(sections[1]["parsed"] == manifest["aom_binding"], "AOM_PRE does not match manifest AOM")
    owner = _companion_validate_owner(manifest, companion_path, q2)
    partition = sections.get(5, {}).get("parsed")
    missing = _companion_partition_missing(partition)
    partition_source: dict[str, Any] | None = None
    if missing and partition_path is not None:
        try:
            partition_candidate = json.loads(partition_path.read_text(encoding="utf-8"))
        except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
            raise AdapterError(f"companion: cannot read partition sidecar {partition_path}: {error}") from error
        partition = _companion_validate_partition(partition_candidate, manifest, q2)
        missing = _companion_partition_missing(partition)
        partition_source = {
            "path": str(partition_path.resolve()),
            "sha256": _sha256_file(partition_path),
        }
    elif not missing:
        partition = _companion_validate_partition(partition, manifest, q2)
        partition_source = {"source": "embedded companion PARTITION section"}
    if packet_binding.get("available") is not True:
        missing.append("packet_identity.packet_sha256")
    if optical_flow_archive is None:
        missing.append("optical_flow_archive")
    else:
        archive_source = optical_flow_archive.get("source")
        _companion_require(isinstance(archive_source, dict), "optical-flow source metadata is missing")
        archive_commit = archive_source.get("pinned_basalt_commit")
        if archive_commit is not None:
            _companion_require(
                archive_commit == manifest["pinned_commit"],
                "optical-flow archive pinned commit disagrees with companion",
            )
        archive_timestamps = sorted(
            int(record["timestamp_ns"])
            for record in optical_flow_archive.get("records", [])
        )
        _companion_require(
            archive_timestamps == kfs_all,
            "optical-flow archive timestamps do not exactly cover packet kfs_all",
        )
        if optical_flow_source_root is not None:
            source_root = optical_flow_source_root.resolve()
            _companion_require(source_root.is_dir(), "optical-flow source root is not a directory")
            if native_packet_path is not None:
                packet_root = native_packet_path.resolve().parent
                _companion_require(
                    source_root == packet_root / "images",
                    "optical-flow source root is not the same-run native images directory",
                )
    provenance_binding: dict[str, Any] | None = None
    provenance_missing: list[str] = []
    if binding_manifest is None or binding_manifest_path is None:
        provenance_missing.append("provenance_binding")
    else:
        provenance_binding = _companion_validate_binding_manifest(
            binding_manifest_path,
            binding_manifest,
            manifest,
            companion_path,
            native_packet_path,
            optical_flow_archive_path,
            optical_flow_archive,
            packet_binding,
        )
    full_event: dict[str, Any] | None = None
    full_event_path: Path | None = None
    if provenance_binding is not None and provenance_binding.get("schema") == BOUNDING_SCHEMA_V2:
        bound_full = provenance_binding.get("files", {}).get("full_event")
        if isinstance(bound_full, Mapping):
            full_event_path = Path(str(bound_full.get("path")))
    elif (companion_path.resolve().parent / "event0_full.json").is_file():
        full_event_path = companion_path.resolve().parent / "event0_full.json"
    if full_event_path is not None:
        full_event = _companion_validate_full_event(
            manifest,
            companion_path,
            q2,
            owner,
            packet_binding,
        )
    field_matrix = _companion_field_matrix(
        manifest,
        native_doc,
        sections,
        owner,
        partition,
        optical_flow_archive,
        optical_flow_source_root,
        full_event,
    )
    exact_states = {
        "exact_direct_native",
        "exact_companion",
        "exact_image_archive",
        "exact_absent",
        "raw_native_exact",
        "native_raw_exact_unmapped",
    }
    native_compatible_missing = [
        field for field in COMPANION_FIELD_MATRIX
        if field not in NATIVE_COMPATIBILITY_EXCLUDED_FIELDS
        if field_matrix[field]["status"] not in exact_states
        and field_matrix[field]["status"] != "not_required"
    ]
    # Rust-only semantics are deliberately not inferred from a raw native
    # capture.  They require an explicit typed adapter status.  This keeps a
    # native FEJ anchor (whose raw bits are exact) distinct from a Rust
    # tangent/layout FEJ value, and keeps the absent native reanchor relation
    # optional for the native profile while still visible to Rust consumers.
    rust_extension_missing = [
        field for field in RUST_EXTENSION_FIELDS
        if field_matrix[field]["status"] != "rust_extension_exact"
    ]
    rust_extension_missing.sort()
    field_matrix_missing = list(dict.fromkeys(native_compatible_missing + rust_extension_missing))
    binding_missing = list(dict.fromkeys(missing))
    all_missing = list(dict.fromkeys(binding_missing + provenance_missing + field_matrix_missing))
    native_profile_missing = list(dict.fromkeys(
        binding_missing + provenance_missing + native_compatible_missing
    ))
    native_profile_accepted = not native_profile_missing
    rust_extension_accepted = not all_missing
    # The native eight-field packet and images are direct evidence.  The
    # remaining runtime claims are explicitly raw-native where Rust has no
    # corresponding field (for example prior FEJ tangent points and native
    # damping categories); they are not silently relabelled as Rust values.
    runtime_fields = [
        "MargLinData.is_sqrt",
        "MargLinData.order",
        "MargLinData.H_b",
        "prior.current_delta",
        "prior.fej_point.raw_anchor",
        "Q2Jp_Q2r",
        "visual_factor_identity",
        "IMU_source_metadata",
        "pose_damping_metadata",
        "Q2_row_topology",
        "AOM_pre_partition",
        "keyframe_selection",
        "selection_diagnostics",
        "lost_landmarks",
        "internal_prior_transition",
    ]
    profile = {
        "schema": COMPANION_PROFILE_SCHEMA,
        # ``accepted`` is the complete strict profile.  Keep the smaller
        # packet/AOM/Q2 binding visible separately so a useful capture is not
        # mistaken for a complete 27-field schema-4 reproduction.
        "accepted": rust_extension_accepted,
        "native_compatible_accepted": native_profile_accepted,
        "minimal_bundle_accepted": not binding_missing,
        "strict_profile_status": "FULL_ACCEPTED" if rust_extension_accepted else "FULL_INCOMPLETE",
        "native_compatible_profile_status": (
            "NATIVE_COMPATIBLE_ACCEPTED"
            if native_profile_accepted else "NATIVE_COMPATIBLE_INCOMPLETE"
        ),
        "rust_extension_profile_status": (
            "RUST_EXTENSION_ACCEPTED"
            if rust_extension_accepted else "RUST_EXTENSION_INCOMPLETE"
        ),
        "minimal_profile_status": "MINIMAL_ACCEPTED" if not binding_missing else "MINIMAL_INCOMPLETE",
        "provenance_status": "VERIFIED" if provenance_binding is not None else "MISSING",
        "profile_kind": "attested_native_packet_plus_runtime_companion",
        "field_matrix_schema": "native.schema4.runtime_companion.field_matrix.v1",
        "field_matrix_complete": not field_matrix_missing,
        "field_matrix": field_matrix,
        "field_matrix_missing": field_matrix_missing,
        "native_compatible_required_fields": [
            field for field in COMPANION_FIELD_MATRIX
            if field not in NATIVE_COMPATIBILITY_EXCLUDED_FIELDS
            and field_matrix[field]["status"] != "not_required"
        ],
        "native_compatible_missing_fields": native_compatible_missing,
        "rust_extension_fields": sorted(RUST_EXTENSION_FIELDS),
        "rust_extension_missing_fields": rust_extension_missing,
        "rust_extension_contract": {
            "namespace": "rust.schema4.extension.v1",
            "fields": {
                "prior FEJ point / anchor": {
                    "status": "mapping_unavailable",
                    "native_status": field_matrix["prior FEJ point / anchor"]["status"],
                    "type": "rust_extension_only",
                    "scope": "PriorData.fej_point reduced vector; direct per-frame FEJ sidecars are mapped separately",
                    "reason": "native frame-state branches have a deterministic timestamp/wire/chart mapping, but a same-run Rust reduced-prior oracle is required before accepting PriorData.fej_point",
                },
                "kf_to_marg equivalent": {
                    "status": "native_absent",
                    "native_status": field_matrix["kf_to_marg equivalent"]["status"],
                    "type": "rust_extension_only",
                    "reason": "native removeKeyframes has no destination/reanchor pair",
                },
            },
        },
        "native_compatibility_contract": {
            "namespace": "native.schema4.compatible.v1",
            "excluded_optional_fields": sorted(NATIVE_COMPATIBILITY_EXCLUDED_FIELDS),
            "rule": "only semantics absent from native MargData are optional; raw native values remain required and exact",
        },
        "native_absent_optional_fields": [
            field for field in NATIVE_COMPATIBILITY_EXCLUDED_FIELDS
            if field_matrix[field]["status"] == "native_absent_optional"
        ],
        "run_uuid": manifest["run_uuid"],
        "event_ordinal": identity["event_ordinal"],
        "primary_kf_timestamp_ns": primary,
        "event_state_timestamp_ns": event_state,
        "selected_kf_timestamps": kfs_to_marg,
        "packet_binding": packet_binding,
        "native_packet_fields": [
            "aom", "abs_H", "abs_b", "frame_poses", "frame_states",
            "kfs_all", "kfs_to_marg", "use_imu",
        ],
        "native_packet_binding": {
            "run_uuid": manifest["run_uuid"],
            "event_state_timestamp_ns": event_state,
            "selected_kf_timestamps": kfs_to_marg,
            "packet_sha256": packet_binding.get("computed_sha256"),
            "packet_filename": filename,
        },
        "companion_manifest_sha256": _sha256_file(companion_path),
        "companion_binary_sha256": binary_sha,
        "source_owner": owner,
        "partition": partition,
        "partition_source": partition_source,
        "q2": {
            "rows": q2["rows"],
            "cols": q2["cols"],
            "rhs_size": q2["rhs_size"],
            "section_sha256": sections[3]["digest"],
            "scalar": "f32",
            "layout": "column_major",
        },
        "images_binding": (
            {
                "run_uuid": manifest["run_uuid"],
                "event_state_timestamp_ns": event_state,
                "selected_kf_timestamps": kfs_to_marg,
                "kfs_all": kfs_all,
                "source_root": str(optical_flow_source_root.resolve())
                if optical_flow_source_root is not None else None,
                "record_count": len(optical_flow_archive["records"])
                if optical_flow_archive is not None else 0,
                "camera_ids": list(optical_flow_archive["camera_ids"])
                if optical_flow_archive is not None else [],
                "attestation": "validated native images archive; timestamps and source root bound to packet event",
            }
            if optical_flow_archive is not None else None
        ),
        "runtime_fields_attested": runtime_fields,
        "binding_missing_fields": binding_missing,
        "provenance_missing_fields": provenance_missing,
        "provenance_binding": provenance_binding,
        "missing_fields": all_missing,
        "raw_native_non_rust_semantics": [
            "prior.fej_point.raw_anchor",
            "IMU_source_metadata",
            "pose_damping_metadata",
            "kf_to_marg_pair_relation unless an explicit pair table is captured",
        ],
    }
    # Keep raw matrix/vector bytes out of the JSON result.  They remain
    # cryptographically attested by each section digest and are available in
    # the companion binary; returning them here would make the bridge result
    # non-serializable (bytes) and would duplicate a potentially large Q2.
    public_sections: dict[int, dict[str, Any]] = {}
    for tag, section in sections.items():
        public_section = {
            key: value
            for key, value in section.items()
            if key != "payload"
        }
        parsed = section.get("parsed")
        if tag in (3, 15) and isinstance(parsed, dict):
            public_section["parsed"] = {
                key: value
                for key, value in parsed.items()
                if key not in ("jacobian", "rhs")
            }
        public_sections[tag] = public_section
    return {
        "manifest": manifest,
        "sections": public_sections,
        "owner": owner,
        "packet_binding": packet_binding,
        "profile": profile,
    }


def _u64_hash(value: Any, path: str) -> int:
    if isinstance(value, bool):
        raise AdapterError(f"{path}: expected an unsigned 64-bit hash")
    if isinstance(value, int):
        result = value
    elif isinstance(value, str):
        text = value.strip().lower()
        if text.startswith("0x"):
            text = text[2:]
        if not text or any(char not in "0123456789abcdef" for char in text):
            raise AdapterError(f"{path}: expected hexadecimal hash")
        result = int(text, 16)
    else:
        raise AdapterError(f"{path}: expected an unsigned 64-bit hash")
    if result < 0 or result >= 1 << 64:
        raise AdapterError(f"{path}: hash is outside unsigned 64-bit range")
    return result


def _fnv1a_u16_le(samples: Iterable[int]) -> int:
    value = 1469598103934665603
    for sample in samples:
        for byte in struct.pack("<H", sample):
            value = (value ^ byte) * 1099511628211 & ((1 << 64) - 1)
    return value


def _decode_u16_samples(value: Any, path: str) -> list[int]:
    if isinstance(value, str):
        try:
            raw = base64.b64decode(value, validate=True)
        except Exception as error:  # pragma: no cover - exact exception varies by Python
            raise AdapterError(f"{path}: invalid base64 image payload: {error}") from error
        if len(raw) % 2:
            raise AdapterError(f"{path}: base64 image payload has an odd byte count")
        return [int.from_bytes(raw[index:index + 2], "little")
                for index in range(0, len(raw), 2)]
    if not isinstance(value, list):
        raise AdapterError(f"{path}: expected u16 array or little-endian base64 string")
    result: list[int] = []
    for index, sample in enumerate(value):
        if isinstance(sample, bool) or not isinstance(sample, int):
            raise AdapterError(f"{path}[{index}]: expected integer u16 sample")
        if sample < 0 or sample > 0xFFFF:
            raise AdapterError(f"{path}[{index}]: sample outside u16 range")
        result.append(sample)
    return result


def _finite_float(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise AdapterError(f"{path}: expected a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise AdapterError(f"{path}: expected a finite number")
    return result


def _positive_int(value: Any, path: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise AdapterError(f"{path}: expected a positive integer")
    return value


def _nonnegative_int(value: Any, path: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise AdapterError(f"{path}: expected a non-negative integer")
    return value


def _safe_archive_path(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value:
        raise AdapterError(f"{path}: expected a relative archive filename")
    normalized = value.replace("\\", "/")
    candidate = Path(normalized)
    if candidate.is_absolute() or ":" in normalized:
        raise AdapterError(f"{path}: absolute archive paths are forbidden")
    parts = normalized.split("/")
    if any(part in ("", ".", "..") for part in parts):
        raise AdapterError(f"{path}: archive path must not contain empty, '.', or '..' components")
    if not normalized.lower().endswith(".cereal"):
        raise AdapterError(f"{path}: expected a .cereal archive filename")
    return normalized


def _load_authoritative_frame_map(path: Path | None) -> dict[int, int]:
    """Load a one-to-one timestamp→frame map without silently overwriting rows."""
    if path is None:
        return {}
    try:
        with path.open("r", encoding="utf-8") as stream:
            value = json.load(stream)
    except (OSError, json.JSONDecodeError) as error:
        raise AdapterError(f"frame-map: cannot read {path}: {error}") from error
    if isinstance(value, dict) and "timestamp_to_frame_id" in value:
        value = value["timestamp_to_frame_id"]
    entries: list[tuple[Any, Any]] = []
    if isinstance(value, list):
        for index, item in enumerate(value):
            if not isinstance(item, dict):
                raise AdapterError(f"frame-map[{index}]: expected timestamp entry object")
            entries.append((item.get("timestamp_ns", item.get("timestamp")),
                            item.get("frame_id", item.get("id"))))
    elif isinstance(value, dict):
        entries = list(value.items())
    else:
        raise AdapterError("frame-map: expected object or timestamp entry array")
    result: dict[int, int] = {}
    reverse: dict[int, int] = {}
    for index, (timestamp, frame_id) in enumerate(entries):
        if isinstance(timestamp, bool):
            raise AdapterError(f"frame-map[{index}]: timestamp must be an integer")
        try:
            timestamp_i = int(timestamp)
        except (TypeError, ValueError) as error:
            raise AdapterError(f"frame-map[{index}]: invalid timestamp") from error
        if isinstance(frame_id, dict):
            frame_id = frame_id.get("frame_id", frame_id.get("id"))
        if isinstance(frame_id, bool) or not isinstance(frame_id, int) or frame_id < 0:
            raise AdapterError(f"frame-map[{index}]: frame ID must be a non-negative integer")
        if timestamp_i in result:
            raise AdapterError(f"frame-map: duplicate timestamp {timestamp_i}")
        if frame_id in reverse:
            raise AdapterError(
                f"frame-map: frame ID {frame_id} is ambiguous for timestamps "
                f"{reverse[frame_id]} and {timestamp_i}"
            )
        result[timestamp_i] = frame_id
        reverse[frame_id] = timestamp_i
    return result


def _camera_digest(records: Iterable[Mapping[str, Any]], camera_id: int) -> str:
    rows: list[dict[str, Any]] = []
    for record in records:
        timestamp = int(record["timestamp_ns"])
        for image in record["images"]:
            if int(image["camera_id"]) != camera_id:
                continue
            rows.append({
                "timestamp_ns": timestamp,
                "camera_id": camera_id,
                "width": int(image["width"]),
                "height": int(image["height"]),
                "sample_hash": f"{_u64_hash(image['sample_hash'], 'camera image sample_hash'):016x}",
                "payload_sha256": str(image["payload_sha256"]).lower(),
            })
    return _sha256_bytes(_canonical(rows))


def _load_optical_flow_archive(path: Path) -> tuple[dict[str, Any], Path | None, str]:
    """Load a normalized dump of native images/*.cereal records.

    A directory with ``manifest.json`` is treated as a pre-decoded normalized
    envelope.  A raw directory containing only timestamp-named ``.cereal``
    files is decoded by ``native_images_cereal_decoder.py``.  Both paths feed
    the same strict validation below; source files are checked against their
    SHA-256 values before any merge.
    """
    source_root: Path | None = None
    if path.is_dir():
        manifest = path / "manifest.json"
        if not manifest.is_file():
            decoder_path = Path(__file__).with_name("native_images_cereal_decoder.py")
            if not decoder_path.is_file():
                raise AdapterError(
                    f"optical-flow archive directory {path} has no manifest.json and "
                    f"the pinned decoder is missing at {decoder_path}"
                )
            try:
                spec = importlib.util.spec_from_file_location(
                    "native_images_cereal_decoder_for_bridge", decoder_path
                )
                if spec is None or spec.loader is None:
                    raise AdapterError(f"cannot load pinned decoder {decoder_path}")
                decoder = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(decoder)
                document = decoder.decode_archive(path)
            except AdapterError:
                raise
            except Exception as error:  # pragma: no cover - exact decoder errors vary
                raise AdapterError(
                    f"optical-flow archive directory {path}: pinned cereal decode failed: {error}"
                ) from error
            return document, path.resolve(), f"native cereal decoder: {path}"
        input_path = manifest
        source_root = path
    else:
        input_path = path
    try:
        with input_path.open("r", encoding="utf-8") as stream:
            document = json.load(stream)
    except (OSError, json.JSONDecodeError) as error:
        raise AdapterError(f"optical-flow archive: cannot read {input_path}: {error}") from error
    if not isinstance(document, dict):
        raise AdapterError("optical-flow archive: expected a JSON object")
    declared_root = document.get("source_root")
    if declared_root is not None:
        declared_path = Path(str(declared_root))
        source_root = declared_path if declared_path.is_absolute() else input_path.parent / declared_path
    source = str(input_path)
    return document, source_root, source


def _validate_optical_flow_archive(
    document: Mapping[str, Any], source_root: Path | None = None,
) -> dict[str, Any]:
    if document.get("schema") != OPTICAL_FLOW_ARCHIVE_SCHEMA:
        raise AdapterError(
            f"optical-flow archive schema: expected {OPTICAL_FLOW_ARCHIVE_SCHEMA!r}"
        )
    source_value = document.get("source", {})
    if not isinstance(source_value, dict):
        raise AdapterError("optical-flow archive source: expected an object")
    archive_kind = document.get("archive_kind", "normalized_native_cereal_images")
    if not isinstance(archive_kind, str) or not archive_kind:
        raise AdapterError("optical-flow archive archive_kind: expected a non-empty string")
    camera_ids_value = document.get("camera_ids")
    if not isinstance(camera_ids_value, list) or not camera_ids_value:
        raise AdapterError("optical-flow archive camera_ids: expected a non-empty array")
    camera_ids = [_nonnegative_int(value, f"camera_ids[{index}]")
                  for index, value in enumerate(camera_ids_value)]
    if len(set(camera_ids)) != len(camera_ids):
        raise AdapterError("optical-flow archive camera_ids: duplicate camera ID")
    camera_count = _positive_int(document.get("camera_count"), "camera_count")
    if camera_count != len(camera_ids):
        raise AdapterError(
            f"optical-flow archive camera_count {camera_count} != camera_ids length {len(camera_ids)}"
        )
    camera_ids = sorted(camera_ids)
    records_value = document.get("records")
    if not isinstance(records_value, list) or not records_value:
        raise AdapterError("optical-flow archive records: expected a non-empty array")
    if "record_count" in document:
        record_count = _nonnegative_int(document["record_count"], "record_count")
        if record_count != len(records_value):
            raise AdapterError(
                f"optical-flow archive record_count {record_count} != records length {len(records_value)}"
            )

    normalized_records: list[dict[str, Any]] = []
    timestamps: set[int] = set()
    observation_keys: set[tuple[int, int, int]] = set()
    source_files: set[str] = set()
    affine_bits_count = 0
    for record_index, record_value in enumerate(records_value):
        path = f"records[{record_index}]"
        if not isinstance(record_value, dict):
            raise AdapterError(f"{path}: expected an object")
        timestamp = record_value.get("timestamp_ns")
        if isinstance(timestamp, bool) or not isinstance(timestamp, int):
            raise AdapterError(f"{path}.timestamp_ns: expected an integer")
        if timestamp in timestamps:
            raise AdapterError(f"{path}.timestamp_ns: duplicate timestamp {timestamp}")
        timestamps.add(timestamp)
        input_timestamp = record_value.get("input_timestamp_ns", timestamp)
        if isinstance(input_timestamp, bool) or not isinstance(input_timestamp, int):
            raise AdapterError(f"{path}.input_timestamp_ns: expected an integer")
        if input_timestamp != timestamp:
            raise AdapterError(
                f"{path}.input_timestamp_ns {input_timestamp} != result timestamp {timestamp}"
            )
        source_file = record_value.get("source_file")
        normalized_source_file: str | None = None
        if source_file is not None:
            normalized_source_file = _safe_archive_path(source_file, f"{path}.source_file")
            if normalized_source_file in source_files:
                raise AdapterError(f"{path}.source_file: duplicate archive filename")
            source_files.add(normalized_source_file)
        source_sha = record_value.get("source_sha256")
        if source_sha is not None:
            source_sha = _hex_sha256(source_sha, f"{path}.source_sha256")
        if source_root is not None:
            if normalized_source_file is None or source_sha is None:
                raise AdapterError(
                    f"{path}: source_file and source_sha256 are required for a materialized archive"
                )
            source_path = (source_root / normalized_source_file).resolve()
            root_resolved = source_root.resolve()
            if root_resolved != source_path and root_resolved not in source_path.parents:
                raise AdapterError(f"{path}.source_file: archive path escapes source root")
            if not source_path.is_file():
                raise AdapterError(f"{path}.source_file: missing source archive {source_path}")
            actual_source_sha = _sha256_file(source_path)
            if actual_source_sha != source_sha:
                raise AdapterError(
                    f"{path}.source_file: SHA-256 mismatch {actual_source_sha} != {source_sha}"
                )

        observations_value = record_value.get("observations")
        if not isinstance(observations_value, list):
            raise AdapterError(f"{path}.observations: expected an array")
        observations: list[dict[str, Any]] = []
        for observation_index, observation_value in enumerate(observations_value):
            observation_path = f"{path}.observations[{observation_index}]"
            if not isinstance(observation_value, dict):
                raise AdapterError(f"{observation_path}: expected an object")
            track_id = _nonnegative_int(observation_value.get("track_id"),
                                        f"{observation_path}.track_id")
            camera_id = _nonnegative_int(observation_value.get("camera_id"),
                                         f"{observation_path}.camera_id")
            if camera_id not in camera_ids:
                raise AdapterError(f"{observation_path}.camera_id: unknown camera {camera_id}")
            key = (timestamp, track_id, camera_id)
            if key in observation_keys:
                raise AdapterError(f"{observation_path}: duplicate (timestamp,track,camera) key")
            observation_keys.add(key)
            xy = observation_value.get("xy")
            if xy is None:
                xy = [observation_value.get("x"), observation_value.get("y")]
            if not isinstance(xy, list) or len(xy) != 2:
                raise AdapterError(f"{observation_path}.xy: expected two coordinates")
            xy_normalized = [
                _finite_float(xy[0], f"{observation_path}.xy[0]"),
                _finite_float(xy[1], f"{observation_path}.xy[1]"),
            ]
            if "xy" in observation_value and (
                "x" in observation_value or "y" in observation_value
            ):
                explicit_xy = [
                    _finite_float(observation_value.get("x"), f"{observation_path}.x"),
                    _finite_float(observation_value.get("y"), f"{observation_path}.y"),
                ]
                if any(
                    struct.pack(">d", left) != struct.pack(">d", right)
                    for left, right in zip(xy_normalized, explicit_xy)
                ):
                    raise AdapterError(f"{observation_path}: xy and x/y disagree")
            affine = observation_value.get("affine_2x3")
            affine_normalized: list[float] | None = None
            if affine is not None:
                if not isinstance(affine, list) or len(affine) != 6:
                    raise AdapterError(f"{observation_path}.affine_2x3: expected six values")
                affine_normalized = [
                    _finite_float(value, f"{observation_path}.affine_2x3[{index}]")
                    for index, value in enumerate(affine)
                ]
            affine_bits = observation_value.get("affine_2x3_bits")
            affine_bits_normalized: list[str] | None = None
            if affine_bits is not None:
                if not isinstance(affine_bits, list) or len(affine_bits) != 6:
                    raise AdapterError(
                        f"{observation_path}.affine_2x3_bits: expected six 32-bit words"
                    )
                if affine_normalized is None:
                    raise AdapterError(
                        f"{observation_path}.affine_2x3_bits requires affine_2x3"
                    )
                affine_bits_normalized = []
                for bit_index, bit_value in enumerate(affine_bits):
                    if not isinstance(bit_value, str) or len(bit_value) != 8:
                        raise AdapterError(
                            f"{observation_path}.affine_2x3_bits[{bit_index}]: expected 8 hex digits"
                        )
                    lowered = bit_value.lower()
                    if any(char not in "0123456789abcdef" for char in lowered):
                        raise AdapterError(
                            f"{observation_path}.affine_2x3_bits[{bit_index}]: invalid hex word"
                        )
                    affine_bits_normalized.append(lowered)
                expected_bits = [
                    f"{struct.unpack('<I', struct.pack('<f', value))[0]:08x}"
                    for value in affine_normalized
                ]
                if affine_bits_normalized != expected_bits:
                    raise AdapterError(
                        f"{observation_path}.affine_2x3_bits disagrees with affine_2x3 raw bits"
                    )
                affine_bits_count += 1
            levels = observation_value.get("pyramid_levels")
            levels_normalized: list[int] | None = None
            if levels is not None:
                if not isinstance(levels, list):
                    raise AdapterError(f"{observation_path}.pyramid_levels: expected an array")
                levels_normalized = [
                    _nonnegative_int(value, f"{observation_path}.pyramid_levels[{index}]")
                    for index, value in enumerate(levels)
                ]
            observations.append({
                "track_id": track_id,
                "camera_id": camera_id,
                "xy": xy_normalized,
                **({"affine_2x3": affine_normalized} if affine_normalized is not None else {}),
                **({"affine_2x3_bits": affine_bits_normalized}
                   if affine_bits_normalized is not None else {}),
                **({"pyramid_levels": levels_normalized} if levels_normalized is not None else {}),
            })

        images_value = record_value.get("images")
        if not isinstance(images_value, list):
            raise AdapterError(f"{path}.images: expected an array")
        if len(images_value) != camera_count:
            raise AdapterError(
                f"{path}.images: camera count {len(images_value)} != expected {camera_count}"
            )
        images: list[dict[str, Any]] = []
        image_cameras: set[int] = set()
        for image_index, image_value in enumerate(images_value):
            image_path = f"{path}.images[{image_index}]"
            if not isinstance(image_value, dict):
                raise AdapterError(f"{image_path}: expected an object")
            image_timestamp = image_value.get("timestamp_ns")
            if isinstance(image_timestamp, bool) or not isinstance(image_timestamp, int):
                raise AdapterError(f"{image_path}.timestamp_ns: expected an integer")
            if image_timestamp != timestamp:
                raise AdapterError(
                    f"{image_path}.timestamp_ns {image_timestamp} != record timestamp {timestamp}"
                )
            camera_id = _nonnegative_int(image_value.get("camera_id"),
                                         f"{image_path}.camera_id")
            if camera_id not in camera_ids:
                raise AdapterError(f"{image_path}.camera_id: unknown camera {camera_id}")
            if camera_id in image_cameras:
                raise AdapterError(f"{image_path}.camera_id: duplicate camera image")
            image_cameras.add(camera_id)
            width = _positive_int(image_value.get("width"), f"{image_path}.width")
            height = _positive_int(image_value.get("height"), f"{image_path}.height")
            raw_samples = image_value.get("samples_u16")
            raw_data = image_value.get("data")
            if raw_samples is None and raw_data is None:
                raise AdapterError(f"{image_path}: samples_u16/data is required")
            samples = _decode_u16_samples(
                raw_samples if raw_samples is not None else raw_data,
                f"{image_path}.samples_u16",
            )
            if raw_samples is not None and raw_data is not None:
                if samples != _decode_u16_samples(raw_data, f"{image_path}.data"):
                    raise AdapterError(f"{image_path}: samples_u16 and data disagree")
            expected_length = width * height
            if len(samples) != expected_length:
                raise AdapterError(
                    f"{image_path}: sample count {len(samples)} != width*height {expected_length}"
                )
            expected_sample_hash = _fnv1a_u16_le(samples)
            supplied_sample_hash = _u64_hash(
                image_value.get("sample_hash"), f"{image_path}.sample_hash"
            )
            if supplied_sample_hash != expected_sample_hash:
                raise AdapterError(
                    f"{image_path}.sample_hash mismatch: "
                    f"{supplied_sample_hash:016x} != {expected_sample_hash:016x}"
                )
            payload_sha = _sha256_bytes(b"".join(struct.pack("<H", sample) for sample in samples))
            supplied_payload_sha = _hex_sha256(
                image_value.get("payload_sha256"), f"{image_path}.payload_sha256"
            )
            if supplied_payload_sha != payload_sha:
                raise AdapterError(
                    f"{image_path}.payload_sha256 mismatch: {supplied_payload_sha} != {payload_sha}"
                )
            exposure = image_value.get("exposure")
            exposure_normalized = (
                _finite_float(exposure, f"{image_path}.exposure") if exposure is not None else None
            )
            images.append({
                "timestamp_ns": timestamp,
                "camera_id": camera_id,
                "width": width,
                "height": height,
                "samples_u16": samples,
                "sample_hash": expected_sample_hash,
                "payload_sha256": payload_sha,
                **({"exposure": exposure_normalized} if exposure_normalized is not None else {}),
            })
        if image_cameras != set(camera_ids):
            missing = sorted(set(camera_ids) - image_cameras)
            extra = sorted(image_cameras - set(camera_ids))
            raise AdapterError(
                f"{path}.images: camera set mismatch missing={missing} extra={extra}"
            )
        normalized_records.append({
            "timestamp_ns": timestamp,
            "input_timestamp_ns": input_timestamp,
            "source_file": normalized_source_file,
            "source_sha256": source_sha,
            "observations": observations,
            "images": sorted(images, key=lambda image: int(image["camera_id"])),
        })

    camera_hashes_value = document.get("camera_hashes")
    if not isinstance(camera_hashes_value, dict):
        raise AdapterError("optical-flow archive camera_hashes: required object")
    expected_camera_hash_keys = {str(camera_id) for camera_id in camera_ids}
    if set(camera_hashes_value) != expected_camera_hash_keys:
        raise AdapterError(
            "optical-flow archive camera_hashes keys do not match camera_ids"
        )
    actual_camera_hashes: dict[str, str] = {}
    for camera_id in camera_ids:
        supplied = _hex_sha256(
            camera_hashes_value[str(camera_id)], f"camera_hashes[{camera_id}]"
        )
        actual = _camera_digest(normalized_records, camera_id)
        if supplied != actual:
            raise AdapterError(
                f"camera {camera_id} aggregate hash mismatch: {supplied} != {actual}"
            )
        actual_camera_hashes[str(camera_id)] = actual

    return {
        "schema": OPTICAL_FLOW_ARCHIVE_SCHEMA,
        "archive_kind": archive_kind,
        "source": source_value,
        "camera_ids": camera_ids,
        "camera_count": camera_count,
        "record_count": len(normalized_records),
        "records": normalized_records,
        "camera_hashes": actual_camera_hashes,
        "affine_bits_count": affine_bits_count,
    }


def _merge_optical_flow_archive(
    projection: Mapping[str, Any],
    document: Mapping[str, Any],
    frame_map: Mapping[int, int],
    source_root: Path | None = None,
    archive_source: str = "normalized optical-flow archive",
) -> tuple[dict[str, Any], dict[str, Any]]:
    normalized = _validate_optical_flow_archive(document, source_root)
    if not frame_map:
        raise AdapterError(
            "optical-flow merge: an authoritative timestamp-to-frame-ID map is required"
        )
    record_frame_ids: dict[int, int] = {}
    for record in normalized["records"]:
        timestamp = int(record["timestamp_ns"])
        if timestamp not in frame_map:
            raise AdapterError(
                f"optical-flow merge: missing frame ID for timestamp {timestamp}"
            )
        frame_id = int(frame_map[timestamp])
        if frame_id in record_frame_ids.values():
            previous = next(
                key for key, value in record_frame_ids.items() if value == frame_id
            )
            raise AdapterError(
                f"optical-flow merge: frame ID {frame_id} is ambiguous for timestamps "
                f"{previous} and {timestamp}"
            )
        record_frame_ids[timestamp] = frame_id

    observations: list[dict[str, Any]] = []
    images: list[dict[str, Any]] = []
    records: list[dict[str, Any]] = []
    for record in normalized["records"]:
        timestamp = int(record["timestamp_ns"])
        frame_id = record_frame_ids[timestamp]
        raw_observations: list[dict[str, Any]] = []
        for observation in record["observations"]:
            xy = list(observation["xy"])
            item = {
                "frame_id": frame_id,
                "track_id": int(observation["track_id"]),
                "camera_id": int(observation["camera_id"]),
                "x": xy[0],
                "y": xy[1],
            }
            observations.append(item)
            raw_observations.append({
                "track_id": int(observation["track_id"]),
                "camera_id": int(observation["camera_id"]),
                "xy": xy,
                **({"affine_2x3": list(observation["affine_2x3"])}
                   if "affine_2x3" in observation else {}),
                **({"affine_2x3_bits": list(observation["affine_2x3_bits"])}
                   if "affine_2x3_bits" in observation else {}),
                **({"pyramid_levels": list(observation["pyramid_levels"])}
                   if "pyramid_levels" in observation else {}),
            })
        raw_images: list[dict[str, Any]] = []
        for image in record["images"]:
            item = {
                "frame_id": frame_id,
                "timestamp_ns": timestamp,
                "camera_id": int(image["camera_id"]),
                "width": int(image["width"]),
                "height": int(image["height"]),
                "data": list(image["samples_u16"]),
            }
            images.append(item)
            raw_images.append({
                "timestamp_ns": timestamp,
                "camera_id": int(image["camera_id"]),
                "width": int(image["width"]),
                "height": int(image["height"]),
                "samples_u16": list(image["samples_u16"]),
                "sample_hash": f"{int(image['sample_hash']):016x}",
                "payload_sha256": str(image["payload_sha256"]),
                **({"exposure": image["exposure"]} if "exposure" in image else {}),
            })
        records.append({
            "timestamp_ns": timestamp,
            "frame_id": frame_id,
            "input_timestamp_ns": int(record["input_timestamp_ns"]),
            "source_file": record.get("source_file"),
            "source_sha256": record.get("source_sha256"),
            "observations": raw_observations,
            "images": raw_images,
        })

    merged_projection = dict(projection)
    merged_projection["of_observations"] = sorted(
        observations,
        key=lambda item: (
            int(item["frame_id"]), int(item["track_id"]), int(item["camera_id"])
        ),
    )
    merged_projection["of_images"] = sorted(
        images,
        key=lambda item: (
            int(item["frame_id"]), int(item["timestamp_ns"]), int(item["camera_id"])
        ),
    )
    merge_metadata = {
        "schema": OPTICAL_FLOW_MERGE_SCHEMA,
        "source_archive": archive_source,
        "archive_schema": normalized["schema"],
        "archive_kind": normalized["archive_kind"],
        "source": normalized["source"],
        "camera_ids": list(normalized["camera_ids"]),
        "camera_count": int(normalized["camera_count"]),
        "record_count": int(normalized["record_count"]),
        "observation_count": len(observations),
        "image_count": len(images),
        "camera_hashes": dict(normalized["camera_hashes"]),
        "affine_bits_count": int(normalized["affine_bits_count"]),
        "timestamp_to_frame_id": {
            str(timestamp): record_frame_ids[timestamp]
            for timestamp in sorted(record_frame_ids)
        },
        "records": records,
        "lossless_native_fields": [
            "OpticalFlowResult.t_ns",
            "OpticalFlowResult.observations (including affine_2x3, raw affine_2x3_bits, and optional pyramid_levels)",
            "OpticalFlowInput.t_ns",
            "OpticalFlowInput.img_data raw uint16 samples/payload hashes/exposure/camera dimensions",
        ],
        "rust_projection_note": (
            "of_observations retains mapper x/y; full native affine/pyramid/exposure "
            "values remain in this merge envelope because Rust OfObservationData/OfImageData "
            "do not have those native fields."
        ),
    }
    return merged_projection, merge_metadata


def _map_values(
    values: Iterable[int], frame_map: Mapping[int, int], field: str,
) -> tuple[list[int] | None, list[str]]:
    missing = sorted({int(value) for value in values if int(value) not in frame_map})
    if missing:
        return None, [f"{field} timestamp-to-frame-id entries: {missing}"]
    return [int(frame_map[int(value)]) for value in values], []


def _mapped_aom_order(
    doc: Mapping[str, Any], frame_map: Mapping[int, int], unavailable: list[str],
) -> list[dict[str, Any]] | dict[str, Any]:
    result: list[dict[str, Any]] = []
    for block in doc["aom"]["order"]:
        timestamp = int(block["timestamp_ns"])
        dof = int(block["dof"])
        kind = {6: "pose", 15: "state"}.get(dof)
        if kind is None:
            unavailable.append(f"aom_order block kind for dof {dof} at {timestamp}")
        result.append({
            "timestamp_ns": timestamp,
            "frame_id": frame_map.get(timestamp),
            "offset": int(block["offset"]),
            "dof": dof,
            "kind": kind or "unknown",
        })
    if any(item["frame_id"] is None for item in result):
        unavailable.append("aom_order.frame_id: timestamp-to-frame-id mapping incomplete")
    return result


def _rust_frame_states(
    doc: Mapping[str, Any],
    frame_map: Mapping[int, int],
    kfs_all: Iterable[int],
) -> list[dict[str, Any]]:
    """Convert native effective states to Rust's compatibility table."""
    entries = list(doc["frame_states"])
    latest = max((int(entry["timestamp_ns"]) for entry in entries), default=None)
    keyframe_timestamps = {int(timestamp) for timestamp in kfs_all}
    return [
        {
            "frame_id": int(frame_map[int(entry["timestamp_ns"])]),
            "timestamp_ns": int(entry["timestamp_ns"]),
            "pose": list(entry["effective_state"]["pose"]["translation"])
            + list(entry["effective_state"]["pose"]["quaternion_wxyz"]),
            "velocity": list(entry["effective_state"]["velocity"]),
            "gyro_bias": list(entry["effective_state"]["gyro_bias"]),
            "accel_bias": list(entry["effective_state"]["accel_bias"]),
            "linearized": bool(entry["linearized"]),
            "is_keyframe": int(entry["timestamp_ns"]) in keyframe_timestamps,
            "is_latest": int(entry["timestamp_ns"]) == latest,
        }
        for entry in entries
    ]


def _f32_value(value: Any, path: str) -> float:
    """Return the exact f32->f64 value used by the UpstreamF32 chart."""
    try:
        candidate = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise AdapterError(f"{path}: expected finite numeric value") from error
    if not math.isfinite(candidate):
        raise AdapterError(f"{path}: expected finite numeric value")
    try:
        return struct.unpack("<f", struct.pack("<f", candidate))[0]
    except (OverflowError, struct.error) as error:
        raise AdapterError(f"{path}: value is not representable as f32") from error


def _f32_bits(value: float) -> str:
    return struct.pack("<f", float(value)).hex()


def _native_pose_wire(pose: Mapping[str, Any], path: str) -> dict[str, Any]:
    translation = pose.get("translation")
    quaternion = pose.get("quaternion_wxyz")
    if not isinstance(translation, list) or len(translation) != 3:
        raise AdapterError(f"{path}.translation: expected three values")
    if not isinstance(quaternion, list) or len(quaternion) != 4:
        raise AdapterError(f"{path}.quaternion_wxyz: expected four values")
    return {
        "translation": [float(value) for value in translation],
        "quaternion_wxyz": [float(value) for value in quaternion],
    }


def _native_nav_wire(state: Mapping[str, Any], path: str) -> dict[str, Any]:
    pose = state.get("pose")
    velocity = state.get("velocity")
    gyro_bias = state.get("gyro_bias")
    accel_bias = state.get("accel_bias")
    if not isinstance(pose, Mapping):
        raise AdapterError(f"{path}.pose: expected an object")
    if not all(isinstance(value, list) and len(value) == 3 for value in (
        velocity, gyro_bias, accel_bias,
    )):
        raise AdapterError(f"{path}: expected three-vector navigation components")
    return {
        "timestamp_ns": int(state["timestamp_ns"]),
        "stored_timestamp_ns": int(state["stored_timestamp_ns"]),
        "timestamp_serialized": bool(state["timestamp_serialized"]),
        "pose": _native_pose_wire(pose, f"{path}.pose"),
        "velocity": [float(value) for value in velocity],
        "gyro_bias": [float(value) for value in gyro_bias],
        "accel_bias": [float(value) for value in accel_bias],
    }


def _flatten_pose_upstream_f32(pose: Mapping[str, Any], path: str) -> list[float]:
    wire = _native_pose_wire(pose, path)
    # Rust flatten_pose_with_mode stores translation followed by quaternion
    # xyz.  The quaternion w lane remains in the sidecar and is not a prior
    # tangent coordinate.
    values = wire["translation"] + wire["quaternion_wxyz"][1:]
    return [_f32_value(value, f"{path}[{index}]") for index, value in enumerate(values)]


def _pose_wire_upstream_f32(pose: Mapping[str, Any], path: str) -> dict[str, Any]:
    wire = _native_pose_wire(pose, path)
    return {
        "translation": [
            _f32_value(value, f"{path}.translation[{index}]")
            for index, value in enumerate(wire["translation"])
        ],
        "quaternion_wxyz": [
            _f32_value(value, f"{path}.quaternion_wxyz[{index}]")
            for index, value in enumerate(wire["quaternion_wxyz"])
        ],
    }


def _flatten_nav_upstream_f32(state: Mapping[str, Any], path: str) -> list[float]:
    wire = _native_nav_wire(state, path)
    values = (
        wire["pose"]["translation"]
        + wire["pose"]["quaternion_wxyz"][1:]
        + wire["velocity"]
        + wire["gyro_bias"]
        + wire["accel_bias"]
    )
    return [_f32_value(value, f"{path}[{index}]") for index, value in enumerate(values)]


def _nav_wire_upstream_f32(state: Mapping[str, Any], path: str) -> dict[str, Any]:
    wire = _native_nav_wire(state, path)
    result = dict(wire)
    result["pose"] = _pose_wire_upstream_f32(state["pose"], f"{path}.pose")
    for name in ("velocity", "gyro_bias", "accel_bias"):
        result[name] = [
            _f32_value(value, f"{path}.{name}[{index}]")
            for index, value in enumerate(wire[name])
        ]
    return result


def _safe_companion_child(root: Path, name: str, label: str) -> Path:
    if not isinstance(name, str) or not name or Path(name).name != name:
        raise AdapterError(f"{label}: expected a basename-only sidecar name")
    candidate = (root / name).resolve()
    if candidate.parent != root.resolve():
        raise AdapterError(f"{label}: sidecar escapes companion directory")
    return candidate


def _read_json_object(path: Path, label: str) -> dict[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as stream:
            value = json.load(stream)
    except (OSError, json.JSONDecodeError) as error:
        raise AdapterError(f"{label}: cannot read {path}: {error}") from error
    if not isinstance(value, dict):
        raise AdapterError(f"{label}: expected a JSON object")
    return value


def _mapping_prior_candidate(
    doc: Mapping[str, Any],
    frame_map: Mapping[int, int],
    companion_manifest: Mapping[str, Any] | None,
    companion_path: Path | None,
    pose_by_timestamp: Mapping[int, Mapping[str, Any]],
    state_by_timestamp: Mapping[int, Mapping[str, Any]],
) -> dict[str, Any]:
    """Build a bound candidate for Rust PriorData.fej_point.

    The candidate is intentionally not promoted to an exact Rust value.  A
    native companion captures the reduced prior order and native linearized
    anchors, but only a same-run Rust MargData with a serialized prior can
    prove that the two representations use identical membership and chart
    semantics.
    """
    unavailable = {
        "status": "mapping_unavailable",
        "reason": "event-bound reduced-prior order/anchors are unavailable",
    }
    if companion_manifest is None or companion_path is None:
        unavailable["reason"] = "same-run companion manifest is unavailable"
        return unavailable
    root = companion_path.resolve().parent
    full_path = root / "event0_full.json"
    if not full_path.is_file():
        unavailable["reason"] = "event0_full.json is unavailable"
        return unavailable
    full_event = _read_json_object(full_path, "full event sidecar")
    if full_event.get("schema") != "basalt.native.schema4.full_event.v1":
        raise AdapterError("full event sidecar: unexpected schema")
    if full_event.get("run_uuid") != companion_manifest.get("run_uuid"):
        raise AdapterError("full event sidecar: run UUID disagrees with companion")
    companion_identity = companion_manifest.get("packet_identity")
    if not isinstance(companion_identity, Mapping):
        raise AdapterError("companion manifest: packet identity is missing")
    if int(full_event.get("event_ordinal", -1)) != int(
        companion_identity.get("event_ordinal", -2)
    ):
        raise AdapterError("full event sidecar: event ordinal disagrees with companion")
    if int(full_event.get("event_state_timestamp_ns", -1)) != int(
        companion_identity.get("event_state_timestamp_ns", -2)
    ):
        raise AdapterError("full event sidecar: state timestamp disagrees with companion")
    transition = full_event.get("prior_transition")
    if not isinstance(transition, Mapping):
        unavailable["reason"] = "full event has no prior transition"
        return unavailable
    if transition.get("phase") not in (None, "post_shift"):
        raise AdapterError("full event sidecar: unsupported prior transition phase")
    post_name = transition.get("post_shift_file")
    if not isinstance(post_name, str):
        unavailable["reason"] = "full event has no post-shift prior sidecar"
        return unavailable
    prior_path = _safe_companion_child(root, post_name, "prior post-shift sidecar")
    if not prior_path.is_file():
        unavailable["reason"] = "post-shift prior sidecar is unavailable"
        return unavailable
    prior = _read_json_object(prior_path, "prior post-shift sidecar")
    if prior.get("schema") != "basalt.native.schema4.prior_transition.v1":
        raise AdapterError("prior post-shift sidecar: unexpected schema")
    if prior.get("phase") != "post_shift":
        raise AdapterError("prior post-shift sidecar: expected post_shift phase")
    for name in ("run_uuid", "packet_filename"):
        if prior.get(name) != full_event.get(name):
            raise AdapterError(f"prior post-shift sidecar: {name} disagrees with full event")
    if int(prior.get("event_ordinal", -1)) != int(full_event["event_ordinal"]):
        raise AdapterError("prior post-shift sidecar: event ordinal disagrees with full event")
    if int(prior.get("primary_kf_timestamp_ns", -1)) != int(full_event["primary_kf_timestamp_ns"]):
        raise AdapterError("prior post-shift sidecar: primary KF disagrees with full event")
    if int(prior.get("event_state_timestamp_ns", -1)) != int(full_event["event_state_timestamp_ns"]):
        raise AdapterError("prior post-shift sidecar: state timestamp disagrees with full event")
    order = prior.get("order")
    if not isinstance(order, list) or not order:
        unavailable["reason"] = "post-shift prior order is empty"
        return unavailable
    expected_cols = int(prior.get("cols", -1))
    expected_rows = int(prior.get("rows", -1))
    if expected_cols < 0 or expected_rows < 0 or int(prior.get("rhs_size", -1)) != expected_rows:
        raise AdapterError("prior post-shift sidecar: invalid dimensions")
    blocks: list[dict[str, Any]] = []
    vector: list[float] = []
    cursor = 0
    for index, item in enumerate(order):
        if not isinstance(item, Mapping):
            raise AdapterError(f"prior post-shift order[{index}]: expected an object")
        timestamp = int(item.get("timestamp_ns", -1))
        dof = int(item.get("dof", -1))
        offset = int(item.get("offset", -1))
        if offset != cursor or dof not in (6, 15):
            raise AdapterError(f"prior post-shift order[{index}]: non-contiguous or invalid block")
        if dof == 6 and timestamp in pose_by_timestamp:
            source_kind = "pose"
            values = _flatten_pose_upstream_f32(
                pose_by_timestamp[timestamp]["pose_linearized"],
                f"frame_poses[{timestamp}].pose_linearized",
            )
        elif dof == 6 and timestamp in state_by_timestamp:
            source_kind = "state_pose"
            values = _flatten_nav_upstream_f32(
                state_by_timestamp[timestamp]["state_linearized"],
                f"frame_states[{timestamp}].state_linearized",
            )[:6]
        elif dof == 15 and timestamp in state_by_timestamp:
            source_kind = "state"
            values = _flatten_nav_upstream_f32(
                state_by_timestamp[timestamp]["state_linearized"],
                f"frame_states[{timestamp}].state_linearized",
            )
        else:
            raise AdapterError(
                f"prior post-shift order[{index}]: no matching native linearized {dof}-DOF block at {timestamp}"
            )
        if len(values) != dof:
            raise AdapterError(f"prior post-shift order[{index}]: flattened block has wrong length")
        frame_id = frame_map.get(timestamp)
        if frame_id is None:
            raise AdapterError(f"prior post-shift order[{index}]: timestamp has no frame mapping")
        blocks.append({
            "index": index,
            "timestamp_ns": timestamp,
            "frame_id": int(frame_id),
            "offset": offset,
            "dof": dof,
            "source_kind": source_kind,
            "scalar_mode": "UpstreamF32",
            "values": values,
            "f32_bits": [_f32_bits(value) for value in values],
        })
        vector.extend(values)
        cursor += dof
    if cursor != expected_cols or len(vector) != expected_cols:
        raise AdapterError(
            f"prior post-shift order: block sum {cursor} does not equal cols {expected_cols}"
        )
    return {
        "status": "candidate_unproven",
        "reason": (
            "native event-bound post-shift order and linearized anchors produce a deterministic "
            "UpstreamF32 candidate; same-run Rust PriorData.fej_point oracle is still required"
        ),
        "source": {
            "full_event": str(full_path),
            "prior_sidecar": str(prior_path),
            "run_uuid": prior["run_uuid"],
            "event_ordinal": int(prior["event_ordinal"]),
            "packet_filename": prior["packet_filename"],
            "phase": prior.get("phase"),
            "order_sha256": _sha256_bytes(_canonical(order)),
        },
        "scalar_mode": "UpstreamF32",
        "cols": expected_cols,
        "rows": expected_rows,
        "order": blocks,
        "values": vector,
        "f32_bits": [_f32_bits(value) for value in vector],
        "oracle": {
            "status": "missing_same_run_rust_prior",
            "required": "Rust MargData.prior.fej_point plus run/event/packet identity",
        },
    }


def _compare_fej_sidecars(
    mapping: Mapping[str, Any], rust_margdata: Mapping[str, Any], rust_path: Path | None,
) -> dict[str, Any]:
    """Compare direct FEJ sidecars without treating an unbound file as oracle.

    Native frame tables are commonly serialized as f64 while the Rust
    ``UpstreamF32`` path stores the source float widened to f64 in JSON.  The
    comparison therefore operates on f32 bit patterns after applying the
    explicit native-to-Rust wire conversion.
    """
    result: dict[str, Any] = {
        "status": "unproven_unbound",
        "exact": False,
        "same_event_identity": False,
        "rust_path": str(rust_path) if rust_path is not None else None,
        "reason": "Rust MargData lacks same-run companion identity in this comparison",
        "first_mismatch": None,
        "compared_lanes": 0,
    }
    native_event = mapping.get("event_identity")
    rust_identity = rust_margdata.get("native_companion_identity")
    if isinstance(native_event, Mapping) and isinstance(rust_identity, Mapping):
        result["same_event_identity"] = dict(native_event) == dict(rust_identity)
    native_poses_raw = mapping.get("frame_poses_fej", {})
    rust_poses_raw = rust_margdata.get("frame_poses_fej", {})
    native_states_raw = mapping.get("frame_states_fej", {})
    rust_states_raw = rust_margdata.get("frame_states_fej", {})

    def index_sidecars(value: Any, label: str) -> dict[str, Mapping[str, Any]]:
        if isinstance(value, Mapping):
            items = value.items()
        elif isinstance(value, list):
            items = []
            for index, item in enumerate(value):
                if not isinstance(item, Mapping) or "timestamp_ns" not in item:
                    raise AdapterError(f"{label}[{index}]: missing timestamp")
                items.append((str(int(item["timestamp_ns"])), item))
        else:
            raise AdapterError(f"{label}: expected object or array")
        result: dict[str, Mapping[str, Any]] = {}
        for key, item in items:
            if not isinstance(item, Mapping):
                raise AdapterError(f"{label}[{key}]: expected object")
            normalized = str(int(key))
            if normalized in result:
                raise AdapterError(f"{label}: duplicate timestamp {normalized}")
            result[normalized] = item
        return result

    try:
        native_poses = index_sidecars(native_poses_raw, "native frame_poses_fej")
        rust_poses = index_sidecars(rust_poses_raw, "Rust frame_poses_fej")
        native_states = index_sidecars(native_states_raw, "native frame_states_fej")
        rust_states = index_sidecars(rust_states_raw, "Rust frame_states_fej")
    except AdapterError as error:
        result["reason"] = str(error)
        return result
    if not native_poses or not rust_poses or not native_states or not rust_states:
        result["reason"] = "one side lacks direct FEJ pose/state sidecars"
        return result

    def check_lane(
        table: str, timestamp: str, branch: str, component: str, lane: int,
        native_value: Any, rust_value: Any,
    ) -> bool:
        native_float = _f32_value(native_value, f"native {table}[{timestamp}]")
        rust_float = _f32_value(rust_value, f"Rust {table}[{timestamp}]")
        result["compared_lanes"] += 1
        native_bits = _f32_bits(native_float)
        rust_bits = _f32_bits(rust_float)
        if native_bits == rust_bits:
            return True
        if result["first_mismatch"] is None:
            result["first_mismatch"] = {
                "table": table,
                "timestamp_ns": int(timestamp),
                "lane": lane,
                "stage": branch,
                "component": component,
                "native_bits": native_bits,
                "rust_bits": rust_bits,
            }
        return False

    def rust_pose_lane(value: Any, component: str, lane: int) -> float:
        if isinstance(value, Mapping):
            vector = value.get(component)
            if not isinstance(vector, list):
                raise AdapterError(f"Rust FEJ pose: {component} is not an array")
            return float(vector[lane])
        if isinstance(value, list):
            index = lane if component == "translation" else 3 + lane
            return float(value[index])
        raise AdapterError("Rust FEJ pose: unsupported wire shape")

    def rust_state_lane(value: Any, parent: str, component: str, lane: int) -> float:
        if not isinstance(value, Mapping):
            raise AdapterError("Rust FEJ state: expected an object")
        if parent:
            parent_value = value.get(parent)
            if isinstance(parent_value, list):
                index = lane if component == "translation" else 3 + lane
                return float(parent_value[index])
            if not isinstance(parent_value, Mapping):
                raise AdapterError(f"Rust FEJ state: {parent} is not an object/array")
            vector = parent_value.get(component)
        else:
            vector = value.get(component)
        if not isinstance(vector, list):
            raise AdapterError(f"Rust FEJ state: {parent + '.' if parent else ''}{component} is not an array")
        return float(vector[lane])

    def check_pose_table() -> bool:
        ok = True
        for timestamp in sorted(native_poses, key=int):
            native_entry = native_poses[timestamp]
            rust_entry = rust_poses.get(str(timestamp))
            if not isinstance(rust_entry, Mapping):
                result["status"] = "bound_mismatch" if result["same_event_identity"] else "unproven_unbound"
                if result["first_mismatch"] is None:
                    result["first_mismatch"] = {
                        "table": "frame_poses_fej", "timestamp_ns": int(timestamp),
                        "lane": None, "stage": "pose", "component": None, "kind": "missing",
                    }
                return False
            if bool(native_entry.get("linearized")) != bool(rust_entry.get("linearized")):
                ok = False
                if result["first_mismatch"] is None:
                    result["first_mismatch"] = {
                        "table": "frame_poses_fej", "timestamp_ns": int(timestamp),
                        "lane": None, "stage": "linearized", "component": None,
                        "kind": "flag",
                    }
            native_wire = native_entry.get("rust_wire_f32")
            if not isinstance(native_wire, Mapping):
                raise AdapterError(f"native frame_poses_fej[{timestamp}]: missing rust_wire_f32")
            for branch in ("pose_linearized", "pose_current"):
                native_branch = native_wire[branch]
                rust_branch = rust_entry.get(branch)
                for component, width in (("translation", 3), ("quaternion_wxyz", 4)):
                    for lane in range(width):
                        try:
                            same = check_lane(
                                "frame_poses_fej", timestamp, branch, component, lane,
                                native_branch[component][lane],
                                rust_pose_lane(rust_branch, component, lane),
                            )
                        except (KeyError, IndexError, TypeError, ValueError, AdapterError):
                            same = False
                            if result["first_mismatch"] is None:
                                result["first_mismatch"] = {
                                    "table": "frame_poses_fej", "timestamp_ns": int(timestamp),
                                    "lane": lane, "stage": branch, "component": component,
                                    "kind": "malformed",
                                }
                        ok = same and ok
            native_delta = native_entry.get("delta")
            rust_delta = rust_entry.get("delta")
            if not isinstance(native_delta, list) or not isinstance(rust_delta, list) or len(native_delta) != len(rust_delta):
                ok = False
                continue
            for lane, (native_value, rust_value) in enumerate(zip(native_delta, rust_delta)):
                ok = check_lane(
                    "frame_poses_fej", timestamp, "delta", "delta", lane,
                    native_value, rust_value,
                ) and ok
        return ok

    def check_state_table() -> bool:
        ok = True
        fields = (
            ("pose", "translation", 3), ("pose", "quaternion_wxyz", 4),
            ("", "velocity", 3), ("", "gyro_bias", 3), ("", "accel_bias", 3),
        )
        for timestamp in sorted(native_states, key=int):
            native_entry = native_states[timestamp]
            rust_entry = rust_states.get(str(timestamp))
            if not isinstance(rust_entry, Mapping):
                if result["first_mismatch"] is None:
                    result["first_mismatch"] = {
                        "table": "frame_states_fej", "timestamp_ns": int(timestamp),
                        "lane": None, "stage": "state", "component": None, "kind": "missing",
                    }
                return False
            if bool(native_entry.get("linearized")) != bool(rust_entry.get("linearized")):
                ok = False
                if result["first_mismatch"] is None:
                    result["first_mismatch"] = {
                        "table": "frame_states_fej", "timestamp_ns": int(timestamp),
                        "lane": None, "stage": "linearized", "component": None,
                        "kind": "flag",
                    }
            native_wire = native_entry.get("rust_wire_f32")
            if not isinstance(native_wire, Mapping):
                raise AdapterError(f"native frame_states_fej[{timestamp}]: missing rust_wire_f32")
            for branch in ("state_linearized", "state_current"):
                native_branch = native_wire[branch]
                rust_branch = rust_entry.get(branch)
                for parent, component, width in fields:
                    for lane in range(width):
                        try:
                            native_vector = native_branch[parent][component] if parent else native_branch[component]
                            same = check_lane(
                                "frame_states_fej", timestamp, branch,
                                f"{parent + '.' if parent else ''}{component}", lane,
                                native_vector[lane],
                                rust_state_lane(rust_branch, parent, component, lane),
                            )
                        except (KeyError, IndexError, TypeError, ValueError, AdapterError):
                            same = False
                            if result["first_mismatch"] is None:
                                result["first_mismatch"] = {
                                    "table": "frame_states_fej", "timestamp_ns": int(timestamp),
                                    "lane": lane, "stage": branch,
                                    "component": f"{parent + '.' if parent else ''}{component}",
                                    "kind": "malformed",
                                }
                        ok = same and ok
            native_delta = native_entry.get("delta")
            rust_delta = rust_entry.get("delta")
            if not isinstance(native_delta, list) or not isinstance(rust_delta, list) or len(native_delta) != len(rust_delta):
                ok = False
                continue
            for lane, (native_value, rust_value) in enumerate(zip(native_delta, rust_delta)):
                ok = check_lane(
                    "frame_states_fej", timestamp, "delta", "delta", lane,
                    native_value, rust_value,
                ) and ok
        return ok

    poses_ok = check_pose_table()
    states_ok = check_state_table()
    result["sidecar_counts"] = {
        "native_poses": len(native_poses), "rust_poses": len(rust_poses),
        "native_states": len(native_states), "rust_states": len(rust_states),
    }
    if not poses_ok or not states_ok:
        result["reason"] = (
            "one or more FEJ lanes differ; the supplied Rust packet is not a "
            "same-run oracle (or has a different wire shape)"
        )
        result["status"] = "bound_mismatch" if result["same_event_identity"] else "unproven_unbound"
        return result
    result["exact"] = bool(result["same_event_identity"])
    result["status"] = "exact_same_event" if result["exact"] else "unproven_unbound"
    result["reason"] = (
        "all direct FEJ sidecars match, but no same-run identity was supplied"
        if not result["exact"] else "same-run direct FEJ sidecars match bitwise"
    )
    return result


def _compare_prior_fej_candidate(
    mapping: Mapping[str, Any], rust_margdata: Mapping[str, Any], rust_path: Path | None,
) -> dict[str, Any]:
    """Compare the event-bound reduced FEJ candidate with Rust PriorData.

    A matching vector is still untrusted unless the Rust packet carries the
    same companion identity.  This prevents a structurally identical prior
    from another max-frame run from being promoted as proof.
    """
    candidate = mapping.get("prior_fej_point")
    result: dict[str, Any] = {
        "status": "unproven_unbound",
        "exact": False,
        "same_event_identity": False,
        "rust_path": str(rust_path) if rust_path is not None else None,
        "reason": "Rust MargData lacks same-run companion identity",
        "first_mismatch": None,
        "compared_lanes": 0,
    }
    if not isinstance(candidate, Mapping) or candidate.get("status") != "candidate_unproven":
        result["status"] = "mapping_unavailable"
        result["reason"] = "event-bound reduced-prior candidate is unavailable"
        return result
    native_event = mapping.get("event_identity")
    rust_identity = rust_margdata.get("native_companion_identity")
    if isinstance(native_event, Mapping) and isinstance(rust_identity, Mapping):
        result["same_event_identity"] = dict(native_event) == dict(rust_identity)
    prior = rust_margdata.get("prior")
    if not isinstance(prior, Mapping):
        result["status"] = "missing_same_run_rust_prior"
        result["reason"] = "Rust MargData has no PriorData.prior field"
        return result
    candidate_order = candidate.get("order")
    candidate_values = candidate.get("values")
    if not isinstance(candidate_order, list) or not isinstance(candidate_values, list):
        result["status"] = "mapping_unavailable"
        result["reason"] = "candidate prior has no order/vector"
        return result
    rust_fej_point = prior.get("fej_point")
    if not isinstance(rust_fej_point, list):
        result["status"] = "missing_same_run_rust_prior"
        result["reason"] = "Rust PriorData has no fej_point vector"
        return result
    rust_frame_ids = prior.get("frame_ids")
    rust_block_kinds = prior.get("block_kinds")
    expected_frame_ids = [int(block["frame_id"]) for block in candidate_order]
    expected_kinds = [
        "state_pose" if block["source_kind"] == "state_pose"
        else "state" if block["source_kind"] == "state" else "pose"
        for block in candidate_order
    ]
    if rust_frame_ids != expected_frame_ids or rust_block_kinds != expected_kinds:
        result["status"] = "bound_mismatch" if result["same_event_identity"] else "unproven_unbound"
        result["reason"] = "Rust prior membership/order does not match event-bound candidate"
        result["first_mismatch"] = {
            "stage": "prior_membership",
            "candidate_frame_ids": expected_frame_ids,
            "rust_frame_ids": rust_frame_ids,
            "candidate_block_kinds": expected_kinds,
            "rust_block_kinds": rust_block_kinds,
        }
        return result
    rust_jacobian = prior.get("jacobian")
    rust_cols = rust_jacobian.get("cols") if isinstance(rust_jacobian, Mapping) else None
    if int(rust_cols or -1) != len(candidate_values) or len(rust_fej_point) != len(candidate_values):
        result["status"] = "bound_mismatch" if result["same_event_identity"] else "unproven_unbound"
        result["reason"] = "Rust prior fej_point dimension does not match candidate order"
        result["first_mismatch"] = {
            "stage": "prior_shape",
            "candidate_cols": len(candidate_values),
            "rust_cols": rust_cols,
            "rust_fej_len": len(rust_fej_point),
        }
        return result
    for lane, (candidate_value, rust_value) in enumerate(zip(candidate_values, rust_fej_point)):
        result["compared_lanes"] += 1
        candidate_bits = _f32_bits(_f32_value(candidate_value, f"candidate fej_point[{lane}]"))
        rust_bits = _f32_bits(_f32_value(rust_value, f"Rust fej_point[{lane}]"))
        if candidate_bits != rust_bits:
            result["status"] = "bound_mismatch" if result["same_event_identity"] else "unproven_unbound"
            result["reason"] = "candidate and Rust PriorData.fej_point differ"
            result["first_mismatch"] = {
                "stage": "prior_fej_point",
                "lane": lane,
                "candidate_bits": candidate_bits,
                "rust_bits": rust_bits,
            }
            return result
    if result["same_event_identity"]:
        result["status"] = "exact_same_event"
        result["exact"] = True
        result["reason"] = "event-bound candidate matches Rust PriorData.fej_point bitwise"
    else:
        result["status"] = "unproven_unbound"
        result["reason"] = "candidate matches vector/order but Rust packet identity is absent"
    return result


def _companion_sections_for_extension(
    manifest: Mapping[str, Any], companion_path: Path,
) -> tuple[dict[int, dict[str, Any]], str, Path]:
    """Re-open the event binary for the strict Rust extension boundary.

    The public companion result intentionally omits dense values.  A v3
    extension capture may refer to those values by section tag and digest, so
    the converter re-reads the already validated same-event binary here.
    """
    root = companion_path.resolve().parent
    binary_name = manifest.get("binary")
    binary_path = _safe_companion_child(root, binary_name, "companion binary")
    _companion_require(binary_path.is_file(), "companion binary is unavailable")
    sections, binary_sha = _companion_parse_binary(manifest, binary_path)
    return sections, binary_sha, binary_path


def _extension_require_f32_vector(
    value: Any, bits: Any, count: int, path: str,
) -> tuple[list[float], list[str]]:
    """Validate a captured f32 vector without manufacturing missing lanes."""
    _companion_require(isinstance(value, list), f"{path}: expected f32 values")
    _companion_require(len(value) == count, f"{path}: expected {count} values")
    _companion_require(isinstance(bits, list), f"{path}_f32_bits: expected raw bit words")
    _companion_require(len(bits) == count, f"{path}_f32_bits: expected {count} words")
    values: list[float] = []
    normalized_bits: list[str] = []
    for index, (raw_value, raw_bits) in enumerate(zip(value, bits)):
        number = _f32_value(raw_value, f"{path}[{index}]")
        _companion_require(
            isinstance(raw_bits, str) and re.fullmatch(r"[0-9a-fA-F]{8}", raw_bits) is not None,
            f"{path}_f32_bits[{index}]: expected eight hex digits",
        )
        expected_bits = _f32_bits(number)
        _companion_require(
            raw_bits.lower() == expected_bits,
            f"{path}[{index}] disagrees with its captured f32 bits",
        )
        values.append(number)
        normalized_bits.append(expected_bits)
    return values, normalized_bits


def _extension_order_key(order: Iterable[Mapping[str, Any]], path: str) -> list[dict[str, int]]:
    normalized: list[dict[str, int]] = []
    cursor = 0
    previous_timestamp: int | None = None
    for index, item in enumerate(order):
        _companion_require(isinstance(item, Mapping), f"{path}[{index}]: expected object")
        timestamp = _companion_integer(item.get("timestamp_ns"), f"{path}[{index}].timestamp_ns")
        offset = _companion_integer(item.get("offset"), f"{path}[{index}].offset", 0)
        dof = _companion_integer(item.get("dof"), f"{path}[{index}].dof", 1)
        _companion_require(dof in (6, 15), f"{path}[{index}].dof: expected 6 or 15")
        _companion_require(offset == cursor, f"{path}[{index}]: order has a gap or overlap")
        if previous_timestamp is not None:
            _companion_require(timestamp > previous_timestamp, f"{path}: timestamps are not increasing")
        normalized.append({"timestamp_ns": timestamp, "offset": offset, "dof": dof})
        previous_timestamp = timestamp
        cursor += dof
    return normalized


def _companion_prior_order_for_extension(
    companion_path: Path, manifest: Mapping[str, Any],
) -> tuple[list[dict[str, int]] | None, str | None]:
    """Return the event-bound post-shift prior order used as a binding oracle."""
    root = companion_path.resolve().parent
    full_path = root / "event0_full.json"
    if not full_path.is_file():
        return None, None
    full_event = _read_json_object(full_path, "full event sidecar")
    transition = full_event.get("prior_transition")
    if not isinstance(transition, Mapping):
        return None, None
    name = transition.get("post_shift_file")
    if not isinstance(name, str):
        return None, None
    sidecar_path = _safe_companion_child(root, name, "prior post-shift sidecar")
    if not sidecar_path.is_file():
        return None, None
    sidecar = _read_json_object(sidecar_path, "prior post-shift sidecar")
    order = sidecar.get("order")
    if not isinstance(order, list):
        return None, None
    return _extension_order_key(order, "prior post-shift order"), _sha256_bytes(
        _canonical(_extension_order_key(order, "prior post-shift order"))
    )


def _validate_rust_extension_capture(
    capture: Mapping[str, Any],
    manifest: Mapping[str, Any],
    companion_path: Path,
    native_packet_path: Path | None,
    frame_map: Mapping[int, int],
    sections: Mapping[int, Mapping[str, Any]],
    owner: Mapping[str, Any] | None,
    native_doc: Mapping[str, Any] | None = None,
    native_doc_path: Path | None = None,
) -> dict[str, Any]:
    """Validate a same-event native probe carrying Rust-only schema-4 data.

    A capture is accepted only when every value that the Rust record needs is
    tied to the companion event, raw f32 words, and an explicit source row or
    block mapping.  This is intentionally stricter than the native-compatible
    profile.  The only exception to a destination pair is an explicitly
    proven native deletion-only transition: native cereal records selected
    source KFs but the pinned ``remove_selected_hosts_and_observations`` path
    has no destination/reanchor relation, so the Rust wire carries an empty
    ``kf_to_marg`` vector and the adapter envelope preserves the native-absent
    proof.  No destination is inferred.
    """
    _companion_require(capture.get("schema") == RUST_SCHEMA4_EXTENSION_SCHEMA, "Rust extension schema mismatch")
    _companion_require(capture.get("schema_version") == 3, "unsupported Rust extension schema version")
    source = capture.get("source")
    _companion_require(isinstance(source, Mapping), "Rust extension source metadata is missing")
    _companion_require(
        source.get("kind") == "native_internal_same_event",
        "Rust extension source must be a native same-event internal capture",
    )
    for key in ("native_executable_sha256", "native_library_sha256"):
        _hex_sha256(source.get(key), f"Rust extension source.{key}")
        _companion_require(
            source[key] == manifest[key],
            f"Rust extension source.{key} disagrees with companion manifest",
        )
    capture_method = source.get("capture_method")
    _companion_require(
        capture_method == "native_frame_state_maps_f32_projection",
        "Rust extension source.capture_method must be the verified native frame-state f32 projection",
    )
    native_doc_sha = _hex_sha256(
        source.get("native_bridge_sha256"),
        "Rust extension source.native_bridge_sha256",
    )
    _companion_require(
        native_doc_path is not None,
        "Rust extension source.native_bridge_sha256 requires the native bridge input path",
    )
    _companion_require(
        native_doc_sha == _sha256_file(native_doc_path.resolve()),
        "Rust extension source.native_bridge_sha256 disagrees with the native bridge input",
    )
    _companion_require(
        isinstance(native_doc, Mapping),
        "native bridge input is required to recompute the FEJ frame-state projection",
    )
    native_state_map_sha = _hex_sha256(
        source.get("native_frame_state_map_sha256"),
        "Rust extension source.native_frame_state_map_sha256",
    )
    expected_state_map_sha = _sha256_bytes(
        _canonical({
            "frame_poses": native_doc.get("frame_poses"),
            "frame_states": native_doc.get("frame_states"),
        })
    )
    _companion_require(
        native_state_map_sha == expected_state_map_sha,
        "Rust extension source.native_frame_state_map_sha256 disagrees with the native frame-state maps",
    )

    identity = capture.get("event_identity")
    _companion_require(isinstance(identity, Mapping), "Rust extension event_identity is missing")
    manifest_identity = manifest.get("packet_identity")
    _companion_require(isinstance(manifest_identity, Mapping), "companion packet identity is missing")
    for key in (
        "run_uuid", "event_ordinal", "primary_kf_timestamp_ns",
        "event_state_timestamp_ns", "packet_filename", "packet_sha256",
        "frame_map_sha256",
    ):
        _companion_require(key in identity, f"Rust extension event_identity.{key} is missing")
    _companion_require(identity["run_uuid"] == manifest["run_uuid"], "Rust extension run UUID mismatch")
    for key in ("event_ordinal", "primary_kf_timestamp_ns", "event_state_timestamp_ns", "packet_filename"):
        _companion_require(
            identity[key] == manifest_identity[key],
            f"Rust extension event identity {key} disagrees with companion",
        )
    _hex_sha256(identity["packet_sha256"], "Rust extension event_identity.packet_sha256")
    _companion_require(
        identity["packet_sha256"] == manifest_identity.get("packet_sha256"),
        "Rust extension packet SHA disagrees with companion identity",
    )
    frame_map_sha = _hex_sha256(
        identity["frame_map_sha256"],
        "Rust extension event_identity.frame_map_sha256",
    )
    _companion_require(
        frame_map_sha == _frame_map_binding_sha(frame_map),
        "Rust extension frame-map SHA disagrees with the conversion map",
    )
    if native_packet_path is not None:
        _companion_require(
            _sha256_file(native_packet_path.resolve()) == identity["packet_sha256"],
            "Rust extension packet SHA disagrees with native packet",
        )

    _companion_require(3 in sections and 15 in sections, "Rust extension requires Q2_FINAL and PRIOR_INPUT sections")
    sqrt = capture.get("sqrt_system")
    _companion_require(isinstance(sqrt, Mapping), "Rust extension sqrt_system is missing")
    for key, expected in (("source_section_tag", 3), ("rows", sections.get(3, {}).get("rows")),
                          ("cols", sections.get(3, {}).get("cols")),
                          ("rhs_size", sections.get(3, {}).get("parsed", {}).get("rhs_size"))):
        _companion_require(sqrt.get(key) == expected, f"Rust extension sqrt_system.{key} disagrees with Q2_FINAL")
    _companion_require(sqrt.get("scalar") == "f32", "Rust extension sqrt_system.scalar must be f32")
    _companion_require(sqrt.get("layout") == "column_major", "Rust extension sqrt_system.layout must be column_major")
    _hex_sha256(sqrt.get("section_sha256"), "Rust extension sqrt_system.section_sha256")
    _companion_require(
        sqrt["section_sha256"] == sections[3]["digest"],
        "Rust extension sqrt_system section digest disagrees with Q2_FINAL",
    )

    row_counts = capture.get("row_counts")
    _companion_require(
        isinstance(row_counts, list) and len(row_counts) == 4,
        "Rust extension row_counts must contain [prior, visual, imu, bias]",
    )
    row_counts = [_companion_integer(value, f"Rust extension row_counts[{index}]", 0)
                  for index, value in enumerate(row_counts)]
    _companion_require(sum(row_counts) == int(sqrt["rows"]), "Rust extension row_counts do not cover sqrt rows")
    if owner is not None:
        owner_data = owner.get("data", {})
        _companion_require(isinstance(owner_data, Mapping), "source owner data is malformed")
        _companion_require(row_counts[0] == int(owner_data.get("prior_rows", -1)), "Rust extension prior row count disagrees with owner")
        _companion_require(row_counts[1] == int(owner_data.get("visual_rows", -1)), "Rust extension visual row count disagrees with owner")
        _companion_require(
            row_counts[2] + row_counts[3] == int(owner_data.get("imu_rows", -1)) + int(owner_data.get("damping_rows", -1)),
            "Rust extension IMU/bias row counts disagree with native owner",
        )

    segments = sqrt.get("row_segments")
    _companion_require(isinstance(segments, list) and len(segments) == 4,
                       "Rust extension sqrt_system.row_segments must contain four entries")
    expected_names = ["prior", "visual", "imu", "bias"]
    normalized_segments: list[dict[str, int | str]] = []
    for index, (item, name, count) in enumerate(zip(segments, expected_names, row_counts)):
        _companion_require(isinstance(item, Mapping), f"Rust extension row_segments[{index}] is malformed")
        _companion_require(item.get("category") == name, f"Rust extension row_segments[{index}] category/order mismatch")
        source_start = _companion_integer(item.get("source_start"), f"Rust extension row_segments[{index}].source_start", 0)
        target_start = _companion_integer(item.get("target_start"), f"Rust extension row_segments[{index}].target_start", 0)
        segment_count = _companion_integer(item.get("count"), f"Rust extension row_segments[{index}].count", 0)
        _companion_require(segment_count == count, f"Rust extension row_segments[{index}] count disagrees with row_counts")
        expected_target = sum(row_counts[:index])
        _companion_require(target_start == expected_target, f"Rust extension row_segments[{index}] target order is not canonical")
        normalized_segments.append({
            "category": name,
            "source_start": source_start,
            "target_start": target_start,
            "count": segment_count,
        })
    source_spans = sorted(normalized_segments, key=lambda item: int(item["source_start"]))
    source_cursor = 0
    for item in source_spans:
        _companion_require(int(item["source_start"]) == source_cursor, "Rust extension source row segments have a gap or overlap")
        source_cursor += int(item["count"])
    _companion_require(source_cursor == int(sqrt["rows"]), "Rust extension source row segments do not cover Q2 rows")

    prior = capture.get("prior")
    _companion_require(isinstance(prior, Mapping), "Rust extension prior capture is missing")
    for key, expected in (("source_section_tag", 15), ("rows", sections.get(15, {}).get("rows")),
                          ("cols", sections.get(15, {}).get("cols")),
                          ("rhs_size", sections.get(15, {}).get("parsed", {}).get("rhs_size"))):
        _companion_require(prior.get(key) == expected, f"Rust extension prior.{key} disagrees with PRIOR_INPUT")
    _companion_require(prior.get("scalar") == "f32", "Rust extension prior.scalar must be f32")
    _companion_require(prior.get("layout") == "column_major", "Rust extension prior.layout must be column_major")
    _hex_sha256(prior.get("section_sha256"), "Rust extension prior.section_sha256")
    _companion_require(prior["section_sha256"] == sections[15]["digest"],
                       "Rust extension prior section digest disagrees with PRIOR_INPUT")
    _companion_require(isinstance(prior.get("order"), list), "Rust extension prior.order is missing")
    prior_order = _extension_order_key(prior["order"], "Rust extension prior.order")
    _companion_require(sum(item["dof"] for item in prior_order) == int(prior["cols"]),
                       "Rust extension prior order does not cover prior columns")
    native_order, native_order_sha = _companion_prior_order_for_extension(companion_path, manifest)
    order_binding = prior.get("order_binding")
    _companion_require(isinstance(order_binding, Mapping), "Rust extension prior.order_binding is missing")
    _hex_sha256(order_binding.get("native_order_sha256"), "Rust extension prior.order_binding.native_order_sha256")
    _companion_require(native_order is not None and native_order_sha is not None,
                       "event-bound native prior order sidecar is unavailable")
    _companion_require(order_binding["native_order_sha256"] == native_order_sha,
                       "Rust extension prior order binding SHA disagrees with native event")
    _companion_require(prior_order == native_order, "Rust extension prior order disagrees with native event order")
    for item in prior_order:
        timestamp = int(item["timestamp_ns"])
        mapped = frame_map.get(timestamp)
        _companion_require(mapped is not None, f"Rust extension prior order timestamp {timestamp} has no frame ID")
    for index, original in enumerate(prior["order"]):
        if "frame_id" in original:
            _companion_require(
                int(original["frame_id"]) == int(frame_map[int(prior_order[index]["timestamp_ns"])]),
                f"Rust extension prior frame ID disagrees at timestamp {prior_order[index]['timestamp_ns']}",
            )
    fej_values, fej_bits = _extension_require_f32_vector(
        prior.get("fej_point"), prior.get("fej_point_f32_bits"), int(prior["cols"]),
        "Rust extension prior.fej_point",
    )
    _companion_require(
        prior.get("capture_method") == capture_method,
        "Rust extension prior.capture_method disagrees with source",
    )
    pose_entries = native_doc.get("frame_poses")
    state_entries = native_doc.get("frame_states")
    _companion_require(isinstance(pose_entries, list), "native bridge frame_poses are missing")
    _companion_require(isinstance(state_entries, list), "native bridge frame_states are missing")
    pose_by_timestamp = {
        int(entry["timestamp_ns"]): entry for entry in pose_entries
        if isinstance(entry, Mapping) and "timestamp_ns" in entry
    }
    state_by_timestamp = {
        int(entry["timestamp_ns"]): entry for entry in state_entries
        if isinstance(entry, Mapping) and "timestamp_ns" in entry
    }
    # Rust PriorData.block_kinds is not a free-standing native column label:
    # every prior block must resolve to the corresponding Rust frame table.
    # A native 6-DoF prefix sourced from a 15-DoF state is represented
    # explicitly as ``state_pose``; it never creates a duplicate pose table or
    # widens the prior block to fifteen columns.
    expected_block_kinds: list[str] = []
    for index, item in enumerate(prior_order):
        timestamp = int(item["timestamp_ns"])
        dof = int(item["dof"])
        if dof == 6:
            if timestamp in pose_by_timestamp:
                expected_block_kinds.append("pose")
            elif timestamp in state_by_timestamp:
                expected_block_kinds.append("state_pose")
            else:
                raise AdapterError(
                    f"Rust extension prior order[{index}] 6-DoF block has no matching native pose/state table"
                )
        else:
            _companion_require(
                timestamp in state_by_timestamp,
                f"Rust extension prior order[{index}] 15-DoF block has no matching native state table",
            )
            expected_block_kinds.append("state")
    declared_block_kinds = prior.get("block_kinds")
    if declared_block_kinds is not None:
        _companion_require(
            isinstance(declared_block_kinds, list)
            and len(declared_block_kinds) == len(expected_block_kinds),
            "Rust extension prior.block_kinds must contain one explicit kind per prior block",
        )
        _companion_require(
            declared_block_kinds == expected_block_kinds,
            "Rust extension prior.block_kinds disagrees with native frame tables/order",
        )
    expected_candidate = _mapping_prior_candidate(
        native_doc,
        frame_map,
        manifest,
        companion_path,
        pose_by_timestamp,
        state_by_timestamp,
    )
    _companion_require(
        expected_candidate.get("status") == "candidate_unproven",
        "native frame-state FEJ projection could not be recomputed",
    )
    _companion_require(
        prior.get("capture_order_sha256") == expected_candidate["source"]["order_sha256"],
        "Rust extension prior.capture_order_sha256 disagrees with event-bound candidate",
    )
    _companion_require(
        fej_bits == expected_candidate.get("f32_bits"),
        "Rust extension prior.fej_point differs from the event-bound native frame-state projection",
    )

    relation = capture.get("kf_to_marg")
    _companion_require(isinstance(relation, Mapping), "Rust extension kf_to_marg capture is missing")
    relation_status = relation.get("status")
    pairs = relation.get("pairs")
    normalized_pairs: list[dict[str, int]] = []
    native_absent_relation: dict[str, Any] | None = None
    if relation_status == "native_absent":
        _companion_require(isinstance(pairs, list) and not pairs,
                           "native-absent kf_to_marg must not contain fabricated pairs")
        _companion_require(relation.get("destination_kfs") is None,
                           "native-absent kf_to_marg destination_kfs must be null")
        source_timestamps = relation.get("source_timestamps_ns")
        _companion_require(
            isinstance(source_timestamps, list)
            and all(isinstance(value, int) and not isinstance(value, bool) for value in source_timestamps),
            "native-absent kf_to_marg source_timestamps_ns is malformed",
        )
        selected = [int(value) for value in manifest_identity["kfs_to_marg"]]
        _companion_require(source_timestamps == selected,
                           "native-absent kf_to_marg source timestamps disagree with selected KFs")
        operation = relation.get("operation")
        _companion_require(operation == "remove_selected_hosts_and_observations",
                           "native-absent kf_to_marg operation is not deletion-only")
        proof = relation.get("proof")
        _companion_require(isinstance(proof, Mapping),
                           "native-absent kf_to_marg requires full-event proof metadata")
        full_path = _safe_companion_child(companion_path.resolve().parent, "event0_full.json", "full event sidecar")
        _companion_require(full_path.is_file(), "native-absent kf_to_marg full-event proof is unavailable")
        full_event = _read_json_object(full_path, "full event sidecar")
        transition = full_event.get("kf_transition")
        _companion_require(isinstance(transition, Mapping), "native-absent kf_to_marg transition is missing")
        _companion_require(full_event.get("schema") == "basalt.native.schema4.full_event.v1",
                           "native-absent kf_to_marg full-event schema mismatch")
        for key in ("run_uuid", "event_ordinal", "primary_kf_timestamp_ns", "event_state_timestamp_ns", "packet_filename"):
            expected = manifest.get(key) if key == "run_uuid" else manifest_identity.get(key)
            _companion_require(full_event.get(key) == expected,
                               f"native-absent kf_to_marg full-event {key} mismatch")
        _companion_require(full_event.get("packet_sha256") == manifest_identity.get("packet_sha256"),
                           "native-absent kf_to_marg full-event packet SHA mismatch")
        _companion_require(transition.get("operation") == operation,
                           "native-absent kf_to_marg transition operation mismatch")
        _companion_require(transition.get("source_kfs") == selected,
                           "native-absent kf_to_marg transition source mismatch")
        _companion_require(transition.get("destination_kfs") is None,
                           "native-absent kf_to_marg transition unexpectedly has destinations")
        _companion_require(proof.get("full_event_sha256") == _sha256_file(full_path),
                           "native-absent kf_to_marg full-event proof SHA mismatch")
        _companion_require(proof.get("packet_sha256") == manifest_identity.get("packet_sha256"),
                           "native-absent kf_to_marg proof packet SHA mismatch")
        native_absent_relation = {
            "status": "native_absent",
            "pairs": [],
            "source_timestamps_ns": selected,
            "destination_kfs": None,
            "operation": operation,
            "reason": str(relation.get("reason", "native deletion-only transition has no destination/reanchor pair")),
            "proof": {
                "full_event_sha256": _sha256_file(full_path),
                "packet_sha256": str(manifest_identity["packet_sha256"]),
            },
        }
    else:
        _companion_require(relation_status == "captured", "Rust extension kf_to_marg status is unsupported")
        _companion_require(isinstance(pairs, list) and pairs, "Rust extension kf_to_marg.pairs is empty")
        sources: set[int] = set()
        destinations: set[int] = set()
        for index, item in enumerate(pairs):
            _companion_require(isinstance(item, Mapping), f"Rust extension kf_to_marg.pairs[{index}] is malformed")
            source_timestamp = _companion_integer(item.get("source_timestamp_ns"), f"Rust extension kf_to_marg.pairs[{index}].source_timestamp_ns")
            destination_timestamp = _companion_integer(item.get("destination_timestamp_ns"), f"Rust extension kf_to_marg.pairs[{index}].destination_timestamp_ns")
            _companion_require(source_timestamp in manifest_identity["kfs_to_marg"],
                               f"Rust extension kf_to_marg source {source_timestamp} is not selected")
            _companion_require(destination_timestamp in frame_map,
                               f"Rust extension kf_to_marg destination {destination_timestamp} has no frame ID")
            _companion_require(source_timestamp not in sources, "Rust extension kf_to_marg has duplicate source")
            _companion_require(destination_timestamp not in destinations, "Rust extension kf_to_marg has duplicate destination")
            sources.add(source_timestamp)
            destinations.add(destination_timestamp)
            source_frame_id = int(frame_map[source_timestamp])
            destination_frame_id = int(frame_map[destination_timestamp])
            if "source_frame_id" in item:
                _companion_require(int(item["source_frame_id"]) == source_frame_id, "Rust extension source frame ID disagrees")
            if "destination_frame_id" in item:
                _companion_require(int(item["destination_frame_id"]) == destination_frame_id, "Rust extension destination frame ID disagrees")
            normalized_pairs.append({
                "source_timestamp_ns": source_timestamp,
                "destination_timestamp_ns": destination_timestamp,
                "source_frame_id": source_frame_id,
                "destination_frame_id": destination_frame_id,
            })
        _companion_require(sources == set(int(value) for value in manifest_identity["kfs_to_marg"]),
                           "Rust extension kf_to_marg does not cover selected KFs exactly")

    return {
        "schema": RUST_SCHEMA4_EXTENSION_SCHEMA,
        "schema_version": 3,
        "source": dict(source),
        "event_identity": {key: identity[key] for key in (
            "run_uuid", "event_ordinal", "primary_kf_timestamp_ns",
            "event_state_timestamp_ns", "packet_filename", "packet_sha256",
        )} | {"frame_map_sha256": frame_map_sha},
        "sqrt_system": {
            "source_section_tag": 3,
            "rows": int(sqrt["rows"]),
            "cols": int(sqrt["cols"]),
            "rhs_size": int(sqrt["rhs_size"]),
            "scalar": "f32",
            "layout": "column_major",
            "section_sha256": str(sqrt["section_sha256"]).lower(),
            "row_segments": normalized_segments,
        },
        "row_counts": row_counts,
        "prior": {
            "source_section_tag": 15,
            "rows": int(prior["rows"]),
            "cols": int(prior["cols"]),
            "rhs_size": int(prior["rhs_size"]),
            "scalar": "f32",
            "layout": "column_major",
            "section_sha256": str(prior["section_sha256"]).lower(),
            "block_kinds": expected_block_kinds,
            "order": [
                dict(item, frame_id=int(frame_map[int(item["timestamp_ns"])]))
                for item in prior_order
            ],
            "fej_point": fej_values,
            "fej_point_f32_bits": fej_bits,
            "order_binding": {
                "native_order_sha256": native_order_sha,
                "semantic": "event-bound post-shift prior order",
            },
            "capture_method": capture_method,
            "capture_order_sha256": expected_candidate["source"]["order_sha256"],
        },
        "kf_to_marg": native_absent_relation or {
            "status": "captured",
            "pairs": normalized_pairs,
        },
    }


def _extension_candidate_summary(
    result: Mapping[str, Any], manifest: Mapping[str, Any] | None,
    sections: Mapping[int, Mapping[str, Any]] | None,
) -> dict[str, Any]:
    """Expose exact native evidence and the remaining Rust-only requirements."""
    q2 = sections.get(3, {}) if sections is not None else {}
    prior = sections.get(15, {}) if sections is not None else {}
    full = result.get("companion_validation")
    full_data = full.get("data") if isinstance(full, Mapping) else None
    transition = full_data.get("kf_transition") if isinstance(full_data, Mapping) else None
    companion_validation = result.get("companion_validation")
    owner = companion_validation.get("owner") if isinstance(companion_validation, Mapping) else None
    return {
        "schema": RUST_SCHEMA4_EXTENSION_SCHEMA,
        "status": "native_evidence_only",
        "event_identity_bound": bool(manifest is not None),
        "sqrt_system": {
            "status": "exact_companion_section" if q2.get("parsed") else "unavailable",
            "source_section_tag": 3,
            "rows": q2.get("rows"),
            "cols": q2.get("cols"),
            "rhs_size": q2.get("parsed", {}).get("rhs_size") if isinstance(q2.get("parsed"), Mapping) else None,
            "requires_extension_row_segments": True,
        },
        "prior": {
            "matrix_rhs_status": "exact_companion_section" if prior.get("parsed") else "unavailable",
            "source_section_tag": 15,
            "fej_point_status": "requires_same_event_extension_capture",
            "order_status": "requires_explicit_native_order_binding",
        },
        "row_metadata": {
            "status": "requires_same_event_extension_capture",
            "native_owner_status": "exact" if isinstance(owner, Mapping) else "unavailable",
            "native_row_order": "visual,imu,damping,prior",
            "rust_row_order": "prior,visual,imu,bias",
        },
        "kf_to_marg": {
            "status": "native_absent" if isinstance(transition, Mapping) and transition.get("destination_kfs") is None else "requires_same_event_extension_capture",
            "reason": "native cereal/full event records removal source KFs but no destination/reanchor pair",
        },
        "unavailable_fields": [
            "prior.fej_point",
            "sqrt_system.row_segments",
            "row_counts[prior,visual,imu,bias]",
            "kf_to_marg.destination",
        ],
    }


def _map_extension_targets(
    full_event: Mapping[str, Any], frame_map: Mapping[int, int],
) -> dict[str, list[int]]:
    result: dict[str, list[int]] = {}
    for key in ("poses_to_marg", "states_to_marg_all", "states_to_marg_vel_bias"):
        mapped, missing = _map_values(full_event.get(key, []), frame_map, f"full_event.{key}")
        if missing:
            raise AdapterError("; ".join(missing))
        result[key] = mapped or []
    lost = full_event.get("lost_landmarks", [])
    _companion_require(isinstance(lost, list), "full_event.lost_landmarks is missing")
    result["lost_landmarks"] = [_companion_integer(value, f"full_event.lost_landmarks[{i}]", 0)
                                 for i, value in enumerate(lost)]
    return result


def _rust_pose_array(value: Mapping[str, Any], path: str) -> list[float]:
    wire = _native_pose_wire(value, path)
    return [_f32_value(component, f"{path}.{name}[{index}]")
            for name, components in (("translation", wire["translation"]),
                                     ("quaternion_wxyz", wire["quaternion_wxyz"]))
            for index, component in enumerate(components)]


def _rust_nav_data(value: Mapping[str, Any], path: str) -> dict[str, list[float]]:
    wire = _native_nav_wire(value, path)
    return {
        "pose": _rust_pose_array(wire["pose"], f"{path}.pose"),
        "velocity": [_f32_value(v, f"{path}.velocity[{i}]") for i, v in enumerate(wire["velocity"])],
        "gyro_bias": [_f32_value(v, f"{path}.gyro_bias[{i}]") for i, v in enumerate(wire["gyro_bias"])],
        "accel_bias": [_f32_value(v, f"{path}.accel_bias[{i}]") for i, v in enumerate(wire["accel_bias"])],
    }


def _reorder_column_major_rows(
    values: list[float], rows: int, cols: int,
    segments: Iterable[Mapping[str, Any]],
) -> list[float]:
    source_to_target: dict[int, int] = {}
    for segment in segments:
        source_start = int(segment["source_start"])
        target_start = int(segment["target_start"])
        count = int(segment["count"])
        for index in range(count):
            source_to_target[source_start + index] = target_start + index
    if len(source_to_target) != rows or set(source_to_target) != set(range(rows)):
        raise AdapterError("Rust extension row mapping is not a complete permutation")
    output = [0.0] * (rows * cols)
    for column in range(cols):
        for source_row in range(rows):
            output[source_to_target[source_row] + column * rows] = values[source_row + column * rows]
    return output


def _frame_map_binding_sha(frame_map: Mapping[int, int]) -> str:
    """Hash the canonical timestamp-to-frame-ID map used by a conversion."""
    entries = [
        {"timestamp_ns": int(timestamp), "frame_id": int(frame_id)}
        for timestamp, frame_id in sorted(frame_map.items())
    ]
    return _sha256_bytes(_canonical(entries))


def build_rust_schema4_record(
    result: Mapping[str, Any],
    doc: Mapping[str, Any],
    frame_map: Mapping[int, int],
    extension_capture: Mapping[str, Any] | None = None,
    extension_path: Path | None = None,
    companion_manifest: Mapping[str, Any] | None = None,
    companion_path: Path | None = None,
    native_packet_path: Path | None = None,
    optical_flow_archive: Mapping[str, Any] | None = None,
    native_doc_path: Path | None = None,
) -> dict[str, Any]:
    """Build one deserializable Rust MargData only from a complete v3 capture."""
    companion = result.get("companion_validation")
    manifest = companion_manifest
    sections: dict[int, dict[str, Any]] | None = None
    native_binary_sha = None
    if isinstance(manifest, Mapping) and companion_path is not None:
        sections, native_binary_sha, _ = _companion_sections_for_extension(manifest, companion_path)
    candidate = _extension_candidate_summary(result, manifest, sections)
    incomplete: list[str] = []
    if extension_capture is None:
        incomplete.extend([
            "extension_capture",
            "prior.fej_point",
            "sqrt_system.row_segments",
            "row_counts[prior,visual,imu,bias]",
            "kf_to_marg.destination",
        ])
        return {
            "schema": RUST_SCHEMA4_ADAPTER_SCHEMA,
            "schema_version": 3,
            "status": "RUST_SCHEMA4_INCOMPLETE",
            "extension_capture": None,
            "extension_capture_path": str(extension_path) if extension_path else None,
            "native_binary_sha256": native_binary_sha,
            "candidate": candidate,
            "unavailable_fields": list(dict.fromkeys(incomplete)),
            "rust_margdata": None,
            "roundtrip": {"exact": True, "status": "not_applicable_incomplete"},
        }
    if not isinstance(manifest, Mapping) or companion_path is None or sections is None:
        raise AdapterError("Rust extension capture requires a validated same-event companion")
    owner = companion.get("owner") if isinstance(companion, Mapping) else None
    normalized = _validate_rust_extension_capture(
        extension_capture, manifest, companion_path, native_packet_path,
        frame_map, sections, owner if isinstance(owner, Mapping) else None,
        doc, native_doc_path,
    )
    if optical_flow_archive is None:
        incomplete.append("optical_flow_archive_for_mapper_ingress")
    full_event = companion.get("data") if isinstance(companion, Mapping) else None
    if not isinstance(full_event, Mapping) and companion_path is not None:
        full_path = _safe_companion_child(
            companion_path.resolve().parent, "event0_full.json", "full event sidecar"
        )
        if full_path.is_file():
            full_event = _read_json_object(full_path, "full event sidecar")
    if not isinstance(full_event, Mapping):
        incomplete.append("event-bound marginalization targets")
    if incomplete:
        return {
            "schema": RUST_SCHEMA4_ADAPTER_SCHEMA,
            "schema_version": 3,
            "status": "RUST_SCHEMA4_INCOMPLETE",
            "extension_capture": normalized,
            "extension_capture_path": str(extension_path) if extension_path else None,
            "native_binary_sha256": native_binary_sha,
            "candidate": candidate,
            "unavailable_fields": list(dict.fromkeys(incomplete)),
            "rust_margdata": None,
            "roundtrip": {"exact": True, "status": "not_applicable_incomplete"},
        }

    q2 = sections[3]["parsed"]
    prior_raw = sections[15]["parsed"]
    q2_jacobian = [_f32_value(v, f"Q2_FINAL.jacobian[{i}]") for i, v in enumerate(q2["jacobian"])]
    q2_rhs = [_f32_value(v, f"Q2_FINAL.rhs[{i}]") for i, v in enumerate(q2["rhs"])]
    q2_jacobian = _reorder_column_major_rows(
        q2_jacobian, int(q2["rows"]), int(q2["cols"]), normalized["sqrt_system"]["row_segments"],
    )
    q2_rhs = _reorder_column_major_rows(
        q2_rhs, int(q2["rows"]), 1, normalized["sqrt_system"]["row_segments"],
    )
    aom_order: list[dict[str, Any]] = []
    for index, block in enumerate(doc["aom"]["order"]):
        timestamp = int(block["timestamp_ns"])
        if timestamp not in frame_map:
            raise AdapterError(f"native AOM block {index} has no frame ID")
        dof = int(block["dof"])
        kind = "pose" if dof == 6 else "state" if dof == 15 else None
        if kind is None:
            raise AdapterError(f"native AOM block {index} has unsupported DOF {dof}")
        aom_order.append({"frame_id": int(frame_map[timestamp]), "offset": int(block["offset"]), "dof": dof, "kind": kind})
    frame_poses: list[dict[str, Any]] = []
    frame_poses_fej: dict[str, Any] = {}
    for index, entry in enumerate(doc["frame_poses"]):
        timestamp = int(entry["timestamp_ns"])
        frame_id = int(frame_map[timestamp])
        linearized = bool(entry["linearized"])
        pose_linearized = _rust_pose_array(entry["pose_linearized"], f"frame_poses[{index}].pose_linearized")
        pose_current = _rust_pose_array(entry["pose_current"], f"frame_poses[{index}].pose_current")
        delta = [_f32_value(v, f"frame_poses[{index}].delta[{i}]") for i, v in enumerate(entry["delta"])]
        effective = pose_current if linearized else pose_linearized
        frame_poses.append({"frame_id": frame_id, "timestamp_ns": timestamp, "pose": effective, "is_keyframe": timestamp in doc["kfs_all"]})
        frame_poses_fej[str(timestamp)] = {
            "pose_linearized": pose_linearized,
            "pose_current": pose_current,
            "delta": delta,
            "linearized": linearized,
        }
    frame_states: list[dict[str, Any]] = []
    frame_states_fej: dict[str, Any] = {}
    for index, entry in enumerate(doc["frame_states"]):
        timestamp = int(entry["timestamp_ns"])
        frame_id = int(frame_map[timestamp])
        linearized = bool(entry["linearized"])
        state_linearized = _rust_nav_data(entry["state_linearized"], f"frame_states[{index}].state_linearized")
        state_current = _rust_nav_data(entry["state_current"], f"frame_states[{index}].state_current")
        delta = [_f32_value(v, f"frame_states[{index}].delta[{i}]") for i, v in enumerate(entry["delta"])]
        effective = state_current if linearized else state_linearized
        frame_states.append({
            "frame_id": frame_id, "timestamp_ns": timestamp,
            **effective, "linearized": linearized,
            "is_keyframe": timestamp in doc["kfs_all"],
            "is_latest": timestamp == max(int(item["timestamp_ns"]) for item in doc["frame_states"]),
        })
        frame_states_fej[str(timestamp)] = {
            "state_linearized": state_linearized,
            "state_current": state_current,
            "delta": delta,
            "linearized": linearized,
        }
    targets = _map_extension_targets(full_event, frame_map)
    prior_order = normalized["prior"]["order"]
    prior_frame_ids = [int(item["frame_id"]) for item in prior_order]
    prior_kinds = list(normalized["prior"]["block_kinds"])
    native_h = doc["abs_system"]["h"]
    aom_abs_h = {
        "rows": int(native_h["rows"]), "cols": int(native_h["cols"]),
        "data": [_f32_value(v, f"abs_H[{i}]") for i, v in enumerate(native_h["data"])],
    }
    aom_abs_b = [_f32_value(v, f"abs_b[{i}]") for i, v in enumerate(doc["abs_system"]["b"]["data"])]
    rust_record: dict[str, Any] = {
        "schema_version": 4,
        "aom_sqrt_jacobian": {"rows": int(q2["rows"]), "cols": int(q2["cols"]), "data": q2_jacobian},
        "aom_sqrt_rhs": q2_rhs,
        "aom_abs_h": aom_abs_h,
        "aom_abs_b": aom_abs_b,
        "frame_poses": frame_poses,
        "frame_states": frame_states,
        "keyframes": [int(frame_map[int(timestamp)]) for timestamp in doc["kfs_all"]],
        "kf_to_marg": [
            (int(item["source_frame_id"]), int(item["destination_frame_id"]))
            for item in normalized["kf_to_marg"]["pairs"]
        ],
        "kfs_all": [int(frame_map[int(timestamp)]) for timestamp in doc["kfs_all"]],
        "kfs_to_marg": [int(frame_map[int(timestamp)]) for timestamp in doc["kfs_to_marg"]],
        "aom_order": aom_order,
        "marginalization": targets,
        "prior": {
            "frame_ids": prior_frame_ids,
            "block_kinds": prior_kinds,
            "jacobian": {
                "rows": int(prior_raw["rows"]), "cols": int(prior_raw["cols"]),
                "data": [_f32_value(v, f"PRIOR_INPUT.jacobian[{i}]") for i, v in enumerate(prior_raw["jacobian"])],
            },
            "rhs": [_f32_value(v, f"PRIOR_INPUT.rhs[{i}]") for i, v in enumerate(prior_raw["rhs"])],
            "fej_point": list(normalized["prior"]["fej_point"]),
        },
        "row_counts": list(normalized["row_counts"]),
        "of_observations": list(result["schema4_projection"].get("of_observations", [])),
        "of_images": list(result["schema4_projection"].get("of_images", [])),
        "frame_poses_fej": frame_poses_fej,
        "frame_states_fej": frame_states_fej,
        "fej_complete": True,
        "used_imu": bool(doc["use_imu"]),
        "provenance_version": "basalt-" + str(doc["source"]["commit"])[:12] + "-native-schema4-v3",
    }
    encoded = _canonical(rust_record)
    return {
        "schema": RUST_SCHEMA4_ADAPTER_SCHEMA,
        "schema_version": 3,
        "status": "RUST_SCHEMA4_READY",
        "extension_capture": normalized,
        "extension_capture_path": str(extension_path) if extension_path else None,
        "native_binary_sha256": native_binary_sha,
        "candidate": candidate,
        "native_transition": normalized["kf_to_marg"],
        "unavailable_fields": [],
        "rust_margdata": rust_record,
        "roundtrip": {
            "exact": _canonical(json.loads(encoded.decode("utf-8"))) == encoded,
            "canonical_sha256": _sha256_bytes(encoded),
        },
    }


def _build_rust_extension_mapping(
    doc: Mapping[str, Any],
    frame_map: Mapping[int, int],
    mapping_source: str,
    companion_manifest: Mapping[str, Any] | None,
    companion_path: Path | None,
    rust_margdata: Mapping[str, Any] | None = None,
    rust_margdata_path: Path | None = None,
) -> dict[str, Any]:
    pose_by_timestamp: dict[int, Mapping[str, Any]] = {}
    state_by_timestamp: dict[int, Mapping[str, Any]] = {}
    frame_poses: list[dict[str, Any]] = []
    frame_states: list[dict[str, Any]] = []
    for index, entry in enumerate(doc["frame_poses"]):
        timestamp = int(entry["timestamp_ns"])
        if timestamp in pose_by_timestamp:
            raise AdapterError(f"frame_poses[{index}]: duplicate timestamp")
        frame_id = frame_map.get(timestamp)
        if frame_id is None:
            raise AdapterError(f"frame_poses[{index}]: timestamp has no frame mapping")
        pose_by_timestamp[timestamp] = entry
        frame_poses.append({
            "timestamp_ns": timestamp,
            "frame_id": int(frame_id),
            "rust_key": str(timestamp),
            "pose_linearized": _native_pose_wire(entry["pose_linearized"], f"frame_poses[{index}].pose_linearized"),
            "pose_current": _native_pose_wire(entry["pose_current"], f"frame_poses[{index}].pose_current"),
            "rust_wire_f32": {
                "pose_linearized": _pose_wire_upstream_f32(
                    entry["pose_linearized"], f"frame_poses[{index}].pose_linearized"
                ),
                "pose_current": _pose_wire_upstream_f32(
                    entry["pose_current"], f"frame_poses[{index}].pose_current"
                ),
            },
            "delta": [float(value) for value in entry["delta"]],
            "linearized": bool(entry["linearized"]),
            "effective_branch": str(entry["effective_branch"]),
            "source": "native.frame_poses",
        })
    for index, entry in enumerate(doc["frame_states"]):
        timestamp = int(entry["timestamp_ns"])
        if timestamp in state_by_timestamp:
            raise AdapterError(f"frame_states[{index}]: duplicate timestamp")
        frame_id = frame_map.get(timestamp)
        if frame_id is None:
            raise AdapterError(f"frame_states[{index}]: timestamp has no frame mapping")
        state_by_timestamp[timestamp] = entry
        frame_states.append({
            "timestamp_ns": timestamp,
            "frame_id": int(frame_id),
            "rust_key": str(timestamp),
            "state_linearized": _native_nav_wire(entry["state_linearized"], f"frame_states[{index}].state_linearized"),
            "state_current": _native_nav_wire(entry["state_current"], f"frame_states[{index}].state_current"),
            "rust_wire_f32": {
                "state_linearized": _nav_wire_upstream_f32(
                    entry["state_linearized"], f"frame_states[{index}].state_linearized"
                ),
                "state_current": _nav_wire_upstream_f32(
                    entry["state_current"], f"frame_states[{index}].state_current"
                ),
            },
            "delta": [float(value) for value in entry["delta"]],
            "linearized": bool(entry["linearized"]),
            "effective_branch": str(entry["effective_branch"]),
            "source": "native.frame_states",
        })
    event_identity: dict[str, Any] | None = None
    if companion_manifest is not None:
        identity = companion_manifest.get("packet_identity")
        if not isinstance(identity, Mapping):
            identity = {}
        event_identity = {
            "run_uuid": companion_manifest.get("run_uuid"),
            "event_ordinal": identity.get("event_ordinal"),
            "event_state_timestamp_ns": identity.get("event_state_timestamp_ns"),
            "primary_kf_timestamp_ns": identity.get("primary_kf_timestamp_ns"),
            "packet_filename": identity.get("packet_filename"),
            "packet_sha256": identity.get("packet_sha256"),
            "frame_map_sha256": _frame_map_binding_sha(frame_map),
        }
    mapping: dict[str, Any] = {
        "schema": RUST_EXTENSION_MAPPING_SCHEMA,
        "status": "deterministic_frame_sidecars_mapped",
        "timestamp_key": "nanoseconds",
        "frame_id_source": mapping_source,
        "wire_order": {
            "pose": ["translation.x", "translation.y", "translation.z", "quaternion.w", "quaternion.x", "quaternion.y", "quaternion.z"],
            "nav": ["pose.translation.x", "pose.translation.y", "pose.translation.z", "pose.quaternion.w", "pose.quaternion.x", "pose.quaternion.y", "pose.quaternion.z", "velocity.x", "velocity.y", "velocity.z", "gyro_bias.x", "gyro_bias.y", "gyro_bias.z", "accel_bias.x", "accel_bias.y", "accel_bias.z"],
            "prior_fej_point": "translation[3], quaternion.xyz[3], then velocity/bias components for nav blocks",
        },
        "chart": "Basalt left-local chart; stored prior coordinates use UpstreamF32 quaternion xyz, not SE3 log",
        "event_identity": event_identity,
        "frame_sidecars": {
            "status": "deterministic_exact",
            "pose_count": len(frame_poses),
            "state_count": len(frame_states),
            "pose_timestamps_ns": [entry["timestamp_ns"] for entry in frame_poses],
            "state_timestamps_ns": [entry["timestamp_ns"] for entry in frame_states],
        },
        "frame_poses_fej": frame_poses,
        "frame_states_fej": frame_states,
    }
    mapping["prior_fej_point"] = _mapping_prior_candidate(
        doc, frame_map, companion_manifest, companion_path,
        pose_by_timestamp, state_by_timestamp,
    )
    if rust_margdata is not None:
        mapping["direct_fej_oracle"] = _compare_fej_sidecars(
            mapping, rust_margdata, rust_margdata_path,
        )
        mapping["prior_fej_oracle"] = _compare_prior_fej_candidate(
            mapping, rust_margdata, rust_margdata_path,
        )
        mapping["prior_fej_point"]["oracle"] = mapping["prior_fej_oracle"]
    else:
        mapping["direct_fej_oracle"] = {
            "status": "not_supplied",
            "exact": False,
            "same_event_identity": False,
            "reason": "no Rust MargData oracle was supplied",
        }
        mapping["prior_fej_oracle"] = {
            "status": "not_supplied",
            "exact": False,
            "same_event_identity": False,
            "reason": "no Rust MargData prior oracle was supplied",
        }
    return mapping


def adapt_native_bridge(
    doc: Mapping[str, Any],
    frame_map: Mapping[int, int] | None = None,
    mapping_source: str = "unavailable",
    optical_flow_archive: Mapping[str, Any] | None = None,
    optical_flow_source_root: Path | None = None,
    optical_flow_source: str = "normalized optical-flow archive",
    companion_manifest: Mapping[str, Any] | None = None,
    companion_path: Path | None = None,
    native_packet_path: Path | None = None,
    partition_path: Path | None = None,
    binding_manifest: Mapping[str, Any] | None = None,
    binding_manifest_path: Path | None = None,
    optical_flow_archive_path: Path | None = None,
    rust_margdata: Mapping[str, Any] | None = None,
    rust_margdata_path: Path | None = None,
    rust_extension_capture: Mapping[str, Any] | None = None,
    rust_extension_capture_path: Path | None = None,
    native_doc_path: Path | None = None,
) -> dict[str, Any]:
    """Build a schema-4-shaped projection without synthesizing native fields."""
    validator = _validator()
    validator.validate_bridge(doc)
    frame_map = {int(key): int(value) for key, value in (frame_map or {}).items()}
    projection = validator.project_for_rust(doc, frame_map, mapping_source)
    unavailable: list[str] = []

    all_timestamps = [
        int(entry["timestamp_ns"])
        for entry in list(doc["frame_poses"]) + list(doc["frame_states"])
    ]
    if not frame_map:
        unavailable.append("timestamp_to_frame_id")
    elif any(timestamp not in frame_map for timestamp in all_timestamps):
        unavailable.append("timestamp_to_frame_id: incomplete for pose/state tables")

    mapped_kfs_all, missing = _map_values(doc["kfs_all"], frame_map, "kfs_all")
    unavailable.extend(missing)
    mapped_kfs_to_marg, missing = _map_values(doc["kfs_to_marg"], frame_map, "kfs_to_marg")
    unavailable.extend(missing)
    if mapped_kfs_all is None:
        mapped_kfs_all = _unavailable("native keyframe timestamps have no complete frame map")
    if mapped_kfs_to_marg is None:
        mapped_kfs_to_marg = _unavailable("native marginalization timestamps have no complete frame map")

    aom_order = _mapped_aom_order(doc, frame_map, unavailable)
    mapping_complete = not any(
        timestamp not in frame_map for timestamp in all_timestamps
    ) and all(item.get("frame_id") is not None for item in aom_order)

    # The bridge already checked effective-branch equality.  Keep both FEJ
    # branches and deltas; only the compatibility vectors need frame IDs.
    frame_poses = projection["frame_poses"] if mapping_complete else _unavailable(
        "native pose rows cannot become Rust FramePoseData without frame IDs"
    )
    frame_states = _rust_frame_states(
        doc, frame_map, doc["kfs_all"]
    ) if mapping_complete else _unavailable(
        "native state rows cannot become Rust FrameStateData without frame IDs"
    )

    # These are the exact native fields that are not in Basalt MargData cereal.
    # Keep the names stable: downstream gates consume unavailable_fields.
    unavailable.extend([
        "aom_sqrt_jacobian",
        "aom_sqrt_rhs",
        "prior",
        "q2",
        "kf_to_marg",
        "marginalization.targets_complete",
        "row_counts",
        "of_observations",
        "of_images",
    ])
    unavailable = list(dict.fromkeys(unavailable))

    schema4_projection = {
        "schema_version": SCHEMA4,
        "aom_sqrt_jacobian": _unavailable(
            "native MargData cereal serializes abs_H, not Rust square-root Jacobian"
        ),
        "aom_sqrt_rhs": _unavailable(
            "native MargData cereal serializes abs_b, not Rust square-root RHS"
        ),
        "aom_abs_h": doc["abs_system"]["h"],
        "aom_abs_b": list(doc["abs_system"]["b"]["data"]),
        "frame_poses": frame_poses,
        "frame_states": frame_states,
        "keyframes": mapped_kfs_all,
        "kf_to_marg": _unavailable(
            "native kfs_to_marg is a timestamp list; Rust kf_to_marg requires explicit pairs"
        ),
        "kfs_all": list(doc["kfs_all"]),
        "kfs_to_marg": list(doc["kfs_to_marg"]),
        "aom_order": aom_order,
        "marginalization": _unavailable(
            "native cereal lacks the complete Rust MarginalizationTargets payload"
        ),
        "prior": _unavailable("prior metadata is not a native MargData cereal field"),
        "row_counts": _unavailable("native cereal has no Rust row-count categories"),
        "of_observations": _unavailable(
            "native optical-flow observations are not serialized in MargData"
        ),
        "of_images": _unavailable(
            "native image packets are separate from MargData cereal"
        ),
        "frame_poses_fej": projection["frame_poses_fej"],
        "frame_states_fej": projection["frame_states_fej"],
        "fej_complete": bool(doc["frame_poses"] is not None and doc["frame_states"] is not None),
        "used_imu": bool(doc["use_imu"]),
        "provenance_version": (
            "basalt-native-cereal-bridge-"
            + str(doc["source"]["commit"])[:12]
        ),
    }
    optical_flow_merge: dict[str, Any] | None = None
    if optical_flow_archive is not None:
        schema4_projection, optical_flow_merge = _merge_optical_flow_archive(
            schema4_projection,
            optical_flow_archive,
            frame_map,
            optical_flow_source_root,
            optical_flow_source,
        )
        unavailable = [
            field for field in unavailable
            if field not in {"of_observations", "of_images"}
        ]
    companion_result: dict[str, Any] | None = None
    if companion_manifest is not None:
        if companion_path is None:
            raise AdapterError("companion manifest requires its filesystem path")
        companion_result = _validate_companion(
            companion_manifest,
            companion_path,
            doc,
            native_packet_path,
            optical_flow_archive,
            optical_flow_source_root,
            partition_path,
            binding_manifest,
            binding_manifest_path,
            optical_flow_archive_path,
        )
    rust_extension_mapping = _build_rust_extension_mapping(
        doc,
        frame_map,
        mapping_source,
        companion_manifest,
        companion_path,
        rust_margdata,
        rust_margdata_path,
    ) if mapping_complete else {
        "schema": RUST_EXTENSION_MAPPING_SCHEMA,
        "status": "mapping_unavailable",
        "reason": "timestamp-to-frame-ID mapping is incomplete",
        "frame_sidecars": {"status": "mapping_unavailable"},
        "prior_fej_point": {
            "status": "mapping_unavailable",
            "reason": "frame sidecar mapping is incomplete",
        },
    }
    if companion_result is not None:
        # Keep the native field matrix conservative: the whole prior FEJ field
        # is not exact until a same-run Rust prior vector is present.  Expose
        # the narrower mapping result so consumers can use the direct FEJ
        # sidecars without accidentally accepting PriorData.fej_point.
        extension_contract = companion_result["profile"]["rust_extension_contract"]
        fej_contract = extension_contract["fields"]["prior FEJ point / anchor"]
        fej_contract["frame_sidecars"] = {
            "status": rust_extension_mapping.get("frame_sidecars", {}).get("status"),
            "pose_count": rust_extension_mapping.get("frame_sidecars", {}).get("pose_count", 0),
            "state_count": rust_extension_mapping.get("frame_sidecars", {}).get("state_count", 0),
        }
        fej_contract["prior_fej_point"] = {
            "status": rust_extension_mapping.get("prior_fej_point", {}).get("status"),
            "cols": rust_extension_mapping.get("prior_fej_point", {}).get("cols"),
            "oracle_status": rust_extension_mapping.get("prior_fej_point", {}).get("oracle", {}).get("status")
            if isinstance(rust_extension_mapping.get("prior_fej_point"), Mapping) else None,
            "comparison_status": rust_extension_mapping.get("prior_fej_oracle", {}).get("status"),
        }
        companion_result["profile"]["rust_extension_mapping_summary"] = {
            "schema": RUST_EXTENSION_MAPPING_SCHEMA,
            "frame_sidecars_status": rust_extension_mapping.get("frame_sidecars", {}).get("status"),
            "prior_fej_point_status": rust_extension_mapping.get("prior_fej_point", {}).get("status"),
            "direct_fej_oracle_status": rust_extension_mapping.get("direct_fej_oracle", {}).get("status"),
            "prior_fej_oracle_status": rust_extension_mapping.get("prior_fej_oracle", {}).get("status"),
        }
    exact, before_hash, after_hash = _roundtrip_exact(schema4_projection)
    mapping_roundtrip, mapping_before_hash, mapping_after_hash = _roundtrip_exact(
        rust_extension_mapping
    )
    result = {
        "adapter_schema": ADAPTER_SCHEMA,
        "source_bridge_schema": doc["bridge_schema"],
        "source": doc["source"],
        "rust_schema4_compatible": not unavailable,
        "rust_margdata_deserializable": not unavailable,
        "unavailable_fields": unavailable,
        "mapping_source": mapping_source,
        "timestamp_to_frame_id": {
            str(timestamp): frame_id for timestamp, frame_id in sorted(frame_map.items())
        },
        "native_cereal_order": list(doc["native_cereal_order"]),
        "native_aom_order": list(doc["aom_serialized_order"]),
        "native_payload": {
            "aom_total_size": int(doc["aom"]["total_size"]),
            "aom_items": int(doc["aom"]["items"]),
            "pose_count": len(doc["frame_poses"]),
            "state_count": len(doc["frame_states"]),
            "use_imu": bool(doc["use_imu"]),
        },
        "roundtrip": {
            "exact": exact,
            "canonical_sha256_before": before_hash,
            "canonical_sha256_after": after_hash,
            "note": "JSON semantic values, including both FEJ branches/deltas/flags and native abs_H/abs_b, survive parse/serialize unchanged.",
        },
        "rust_extension_mapping_roundtrip": {
            "exact": mapping_roundtrip,
            "canonical_sha256_before": mapping_before_hash,
            "canonical_sha256_after": mapping_after_hash,
            "note": "Timestamp/frame, wire-f32 sidecars, reduced-prior order, candidate lanes, and oracle status survive JSON round-trip unchanged.",
        },
        "optical_flow_merge": optical_flow_merge,
        "strict_schema4_profile": (
            companion_result["profile"] if companion_result is not None else None
        ),
        "companion_validation": companion_result,
        "rust_extension_mapping": rust_extension_mapping,
        "schema4_projection": schema4_projection,
    }
    if companion_path is not None:
        result["companion_manifest_path"] = str(companion_path.resolve())
    result["rust_schema4_extension"] = build_rust_schema4_record(
        result,
        doc,
        frame_map,
        rust_extension_capture,
        rust_extension_capture_path,
        companion_manifest,
        companion_path,
        native_packet_path,
        optical_flow_archive,
        native_doc_path,
    )
    return result


def _self_test_optical_flow_archive() -> dict[str, Any]:
    def image(timestamp: int, camera_id: int, samples: list[int]) -> dict[str, Any]:
        payload = b"".join(struct.pack("<H", sample) for sample in samples)
        return {
            "timestamp_ns": timestamp,
            "camera_id": camera_id,
            "width": 2,
            "height": 1,
            "samples_u16": samples,
            "sample_hash": f"{_fnv1a_u16_le(samples):016x}",
            "payload_sha256": _sha256_bytes(payload),
            "exposure": 0.01 * (camera_id + 1),
        }

    records = [
        {
            "timestamp_ns": 10,
            "input_timestamp_ns": 10,
            "source_file": "synthetic/10.cereal",
            "observations": [
                {
                    "track_id": 7,
                    "camera_id": 0,
                    "xy": [1.5, 2.5],
                    "affine_2x3": [1.0, 0.0, 1.5, 0.0, 1.0, 2.5],
                    "pyramid_levels": [0, 1],
                },
                {"track_id": 7, "camera_id": 1, "xy": [3.5, 4.5]},
            ],
            "images": [image(10, 0, [1, 2]), image(10, 1, [3, 4])],
        },
        {
            "timestamp_ns": 20,
            "input_timestamp_ns": 20,
            "source_file": "synthetic/20.cereal",
            "observations": [
                {"track_id": 8, "camera_id": 0, "xy": [5.5, 6.5]},
            ],
            "images": [image(20, 0, [5, 6]), image(20, 1, [7, 8])],
        },
    ]
    archive: dict[str, Any] = {
        "schema": OPTICAL_FLOW_ARCHIVE_SCHEMA,
        "archive_kind": "synthetic_normalized_fixture",
        "source": {
            "project": "basalt",
            "archive": "cereal.binary.images",
            "pinned_basalt_commit": "fixture",
            "synthetic": True,
        },
        "camera_ids": [0, 1],
        "camera_count": 2,
        "record_count": len(records),
        "records": records,
    }
    archive["camera_hashes"] = {
        str(camera_id): _camera_digest(records, camera_id)
        for camera_id in archive["camera_ids"]
    }
    return archive


def _self_test_companion() -> dict[str, Any]:
    """Exercise strict native-companion binding without a native toolchain.

    This fixture intentionally models the wire contract, not Rust semantic
    reconstruction.  It proves that a companion profile is accepted only
    after packet, selection, owner, Q2, partition, and image evidence all
    agree; the production native capture uses the same parser.
    """
    validator = _validator()
    native_doc = validator._self_test_document()
    archive = _self_test_optical_flow_archive()

    def json_bytes(value: Mapping[str, Any]) -> bytes:
        return _canonical(dict(value))

    def f32_bytes(values: Iterable[float]) -> bytes:
        values = list(values)
        return struct.pack("<" + "f" * len(values), *values) if values else b""

    aom_payload = struct.pack("<QQQ", 21, 2, 2)
    aom_payload += struct.pack("<qQQ", 10, 0, 6)
    aom_payload += struct.pack("<qQQ", 20, 6, 15)
    q2_rows, q2_cols, q2_rhs = 2, 21, 2
    q2_payload = f32_bytes([0.0] * (q2_rows * q2_cols + q2_rhs))
    selection = {
        "kfs_all": [10, 20],
        "kfs_to_marg": [10],
        "last_state_to_marg": 10,
        "is_lin_sqrt": True,
        "marg_is_sqrt": True,
    }
    partition = {
        "schema": "basalt.native.schema4.partition.v1",
        "run_uuid": "synthetic-run",
        "event_ordinal": 0,
        "primary_kf_timestamp_ns": 10,
        "event_state_timestamp_ns": 20,
        "kfs_all": [10, 20],
        "kfs_to_marg": [10],
        "q2_rows": q2_rows,
        "q2_cols": q2_cols,
        "q2_rhs_size": q2_rhs,
        "state": "captured",
        "idx_to_keep": [6, 7, 8],
        "idx_to_marg": [0, 1, 2, 3, 4, 5],
        "poses_to_marg": [10],
        "states_to_marg_all": [20],
        "states_to_marg_vel_bias": [20],
        "last_state_to_marg": 20,
    }
    prior_metadata = {
        "rows": 2,
        "cols": 2,
        "rhs_size": 2,
        "layout": "column_major",
        "scalar": "f32",
    }
    prior_payload = f32_bytes([0.0] * 6)
    sections = [
        {"tag": 1, "flags": 0, "rows": 0, "cols": 0,
         "element_count": 9, "semantic": "AOM_PRE", "payload": aom_payload},
        {"tag": 3, "flags": 0, "rows": q2_rows, "cols": q2_cols,
         "element_count": 44, "semantic": "Q2_FINAL", "payload": q2_payload},
        {"tag": 13, "flags": 0, "rows": 0, "cols": 0,
         "element_count": 0, "semantic": "SELECTION_DIAGNOSTIC",
         "payload": json_bytes(selection)},
        {"tag": 5, "flags": 0, "rows": 0, "cols": 0,
         "element_count": 0, "semantic": "PARTITION",
         "payload": json_bytes(partition)},
        {"tag": 6, "flags": 0, "rows": 0, "cols": 0,
         "element_count": 0, "semantic": "PRIOR_INPUT_METADATA",
         "payload": json_bytes(prior_metadata)},
        {"tag": 15, "flags": 0, "rows": 2, "cols": 2,
         "element_count": 6, "semantic": "PRIOR_INPUT", "payload": prior_payload},
    ]

    with tempfile.TemporaryDirectory(prefix="native-schema4-companion-") as temporary:
        root = Path(temporary)
        binary_path = root / "event0_10.companion.bin"
        manifest_path = root / "event0_10.companion.json"
        packet_path = root / "10.cereal"
        packet_path.write_bytes(b"synthetic native packet")
        header_bytes = COMPANION_FILE_HEADER_SIZE + COMPANION_SECTION_HEADER_SIZE * len(sections)
        binary = bytearray(struct.pack(
            "<8sHBBHII", COMPANION_MAGIC, 1, 1, 1, len(sections), header_bytes, 0
        ))
        payload_offset = header_bytes
        manifest_sections: list[dict[str, Any]] = []
        for section in sections:
            payload = section["payload"]
            binary.extend(struct.pack(
                "<HHQQQQ32s", section["tag"], section["flags"], section["rows"],
                section["cols"], section["element_count"], len(payload),
                bytes.fromhex(_sha256_bytes(payload)),
            ))
            manifest_sections.append({
                "tag": section["tag"], "flags": section["flags"],
                "rows": section["rows"], "cols": section["cols"],
                "element_count": section["element_count"],
                "payload_bytes": len(payload), "byte_offset": payload_offset,
                "payload_sha256": _sha256_bytes(payload),
                "semantic": section["semantic"],
            })
            payload_offset += len(payload)
        for section in sections:
            binary.extend(section["payload"])
        binary_path.write_bytes(binary)
        owner = {
            "schema": "basalt.native.schema4.q2_source_owner.v1",
            "run_uuid": "synthetic-run",
            "rows": q2_rows, "cols": q2_cols, "rhs_size": q2_rhs,
            "visual_rows": 2, "imu_rows": 0, "damping_rows": 0, "prior_rows": 0,
            "aom_total_size": 21, "aom_items": 2,
            "aom_order": [
                {"timestamp_ns": 10, "offset": 0, "dof": 6},
                {"timestamp_ns": 20, "offset": 6, "dof": 15},
            ],
            "visual_factors": [{"ordinal": 0, "row_start": 0, "row_count": 2}],
        }
        (root / "q2_source_owner.json").write_bytes(_canonical(owner) + b"\n")
        packet_sha = _sha256_file(packet_path)
        manifest = {
            "schema": COMPANION_SCHEMA,
            "schema_version": 1,
            "run_uuid": "synthetic-run",
            "pinned_commit": "fixture",
            "native_executable_sha256": "0" * 64,
            "native_library_sha256": "1" * 64,
            "source_manifest_sha256": "2" * 64,
            "configuration": {
                "config_sha256": "3" * 64,
                "calibration_sha256": "4" * 64,
                "input_sha256": "5" * 64,
            },
            "binary": binary_path.name,
            "binary_format": {
                "magic": "BSC4CMP1", "version": 1, "endianness": "little",
                "header_bytes": header_bytes,
            },
            "packet_identity": {
                "event_ordinal": 0,
                "primary_kf_timestamp_ns": 10,
                "event_state_timestamp_ns": 20,
                "q2_last_state_to_marg_timestamp_ns": 10,
                "packet_filename": packet_path.name,
                "packet_sha256": packet_sha,
                "kfs_all": [10, 20], "kfs_to_marg": [10],
            },
            "aom_binding": {
                "total_size": 21, "items": 2,
                "order": [
                    {"timestamp_ns": 10, "offset": 0, "dof": 6},
                    {"timestamp_ns": 20, "offset": 6, "dof": 15},
                ],
            },
            "row_topology_binding": {
                "q2_shape": {"rows": q2_rows, "cols": q2_cols, "rhs_size": q2_rhs,
                             "layout": "column_major", "scalar": "f32"},
                "source_owner": "q2_source_owner.json",
            },
            "sections": manifest_sections,
        }
        manifest_path.write_bytes(_canonical(manifest) + b"\n")
        result = adapt_native_bridge(
            native_doc, {10: 100, 20: 200}, "self-test map", archive, None,
            "self-test archive", manifest, manifest_path, packet_path,
        )
        profile = result["strict_schema4_profile"]
        if profile is None or not profile["minimal_bundle_accepted"]:
            raise AdapterError(f"self-test: minimal companion binding rejected: {profile}")
        if profile["accepted"] or profile["field_matrix_complete"]:
            raise AdapterError("self-test: incomplete runtime field matrix was accepted")
        if profile["native_compatible_profile_status"] != "NATIVE_COMPATIBLE_INCOMPLETE":
            raise AdapterError("self-test: incomplete native-compatible profile was accepted")
        if "kf_to_marg equivalent" in profile["native_compatible_required_fields"]:
            raise AdapterError("self-test: native-absent KF relation remained required")
        if "prior FEJ point / anchor" not in profile["native_compatible_required_fields"]:
            raise AdapterError("self-test: raw native FEJ anchor was removed from native contract")
        if profile["rust_extension_contract"]["fields"]["prior FEJ point / anchor"]["status"] != "mapping_unavailable":
            raise AdapterError("self-test: FEJ mapping boundary was not typed")
        if "prior current delta" not in profile["field_matrix_missing"]:
            raise AdapterError("self-test: missing prior delta was not reported")
        if profile["packet_binding"]["computed_sha256"] != packet_sha:
            raise AdapterError("self-test: packet SHA binding changed")
        if profile["q2"]["rows"] != q2_rows or profile["q2"]["cols"] != q2_cols:
            raise AdapterError("self-test: Q2 shape binding changed")
        companion_extension = result["rust_schema4_extension"]
        if companion_extension["schema_version"] != 3 or companion_extension["status"] != "RUST_SCHEMA4_INCOMPLETE":
            raise AdapterError("self-test: companion without v3 extension was accepted")
        expected_extension_missing = {
            "extension_capture",
            "prior.fej_point",
            "sqrt_system.row_segments",
            "row_counts[prior,visual,imu,bias]",
            "kf_to_marg.destination",
        }
        if not expected_extension_missing.issubset(set(companion_extension["unavailable_fields"])):
            raise AdapterError("self-test: v3 extension missing-field set changed")

        tamper_packet_dir = root / "tampered"
        tamper_packet_dir.mkdir()
        tamper_packet = tamper_packet_dir / packet_path.name
        tamper_packet.write_bytes(b"tampered native packet")
        try:
            adapt_native_bridge(
                native_doc, {10: 100, 20: 200}, "self-test map", archive, None,
                "self-test archive", manifest, manifest_path, tamper_packet,
            )
        except AdapterError:
            pass
        else:
            raise AdapterError("self-test: packet tamper was accepted")

        tamper_manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        tamper_manifest["row_topology_binding"]["q2_shape"]["rows"] = q2_rows + 1
        tamper_manifest_path = root / "tampered.companion.json"
        tamper_manifest_path.write_bytes(_canonical(tamper_manifest) + b"\n")
        try:
            adapt_native_bridge(
                native_doc, {10: 100, 20: 200}, "self-test map", archive, None,
                "self-test archive", tamper_manifest, tamper_manifest_path, packet_path,
            )
        except AdapterError:
            pass
        else:
            raise AdapterError("self-test: Q2-shape tamper was accepted")

        path_manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        path_manifest["binary"] = "../event0_10.companion.bin"
        path_manifest_path = root / "path-tampered.companion.json"
        path_manifest_path.write_bytes(_canonical(path_manifest) + b"\n")
        try:
            adapt_native_bridge(
                native_doc, {10: 100, 20: 200}, "self-test map", archive, None,
                "self-test archive", path_manifest, path_manifest_path, packet_path,
            )
        except AdapterError:
            pass
        else:
            raise AdapterError("self-test: companion binary path escape was accepted")

    return {
        "ok": True,
        "test": "strict-native-companion-profile-roundtrip-and-tamper-rejection",
        "profile_schema": COMPANION_PROFILE_SCHEMA,
        "accepted": False,
        "minimal_bundle_accepted": True,
        "field_matrix_complete": False,
        "field_matrix_fail_closed": True,
        "packet_binding": True,
        "image_timestamp_binding": True,
        "q2_owner_shape_binding": True,
        "partition_binding": True,
        "tamper_rejects": ["packet_sha256", "q2_shape", "binary_path_escape"],
    }


def _self_test() -> dict[str, Any]:
    validator = _validator()
    doc = validator._self_test_document()
    result = adapt_native_bridge(doc, {10: 100, 20: 200}, "self-test map")
    if not result["roundtrip"]["exact"]:
        raise AdapterError("self-test: projection JSON round-trip changed")
    if result["rust_schema4_compatible"]:
        raise AdapterError("self-test: incomplete native packet was marked compatible")
    required = {"aom_sqrt_jacobian", "aom_sqrt_rhs", "prior", "q2"}
    if not required.issubset(result["unavailable_fields"]):
        raise AdapterError("self-test: unavailable required fields were not reported")
    extension = result["rust_schema4_extension"]
    if extension["schema"] != RUST_SCHEMA4_ADAPTER_SCHEMA:
        raise AdapterError("self-test: Rust schema4 adapter envelope is not v3")
    if extension["schema_version"] != 3 or extension["status"] != "RUST_SCHEMA4_INCOMPLETE":
        raise AdapterError("self-test: incomplete native packet was not rejected by v3 extension")
    if "extension_capture" not in extension["unavailable_fields"]:
        raise AdapterError("self-test: missing same-event extension capture was not reported")
    frame_map_sha = _frame_map_binding_sha({10: 100, 20: 200})
    if frame_map_sha != _frame_map_binding_sha({20: 200, 10: 100}):
        raise AdapterError("self-test: frame-map binding hash is order-dependent")
    if frame_map_sha == _frame_map_binding_sha({10: 100, 20: 201}):
        raise AdapterError("self-test: frame-map binding hash ignored a frame ID change")
    pose = result["schema4_projection"]["frame_poses_fej"]["10"]
    state = result["schema4_projection"]["frame_states_fej"]["20"]
    if pose["pose_current"][0] != 9.0 or state["state_current"]["pose"][0] != 2.0:
        raise AdapterError("self-test: stored current FEJ branch was changed")
    no_map = adapt_native_bridge(doc)
    if "timestamp_to_frame_id" not in no_map["unavailable_fields"]:
        raise AdapterError("self-test: missing timestamp map was accepted")

    archive = _self_test_optical_flow_archive()
    merged = adapt_native_bridge(
        doc,
        {10: 100, 20: 200},
        "self-test map",
        archive,
        None,
        "self-test normalized image archive",
    )
    if not merged["roundtrip"]["exact"]:
        raise AdapterError("self-test: merged projection JSON round-trip changed")
    if merged["rust_schema4_compatible"]:
        raise AdapterError("self-test: native-only fields disappeared after image merge")
    if "of_observations" in merged["unavailable_fields"] or "of_images" in merged["unavailable_fields"]:
        raise AdapterError("self-test: validated optical-flow archive remained unavailable")
    merged_projection = merged["schema4_projection"]
    if len(merged_projection["of_observations"]) != 3:
        raise AdapterError("self-test: merged observation count mismatch")
    if len(merged_projection["of_images"]) != 4:
        raise AdapterError("self-test: merged image count mismatch")
    merge_metadata = merged["optical_flow_merge"]
    if merge_metadata["record_count"] != 2 or merge_metadata["camera_count"] != 2:
        raise AdapterError("self-test: merged archive cardinality mismatch")
    if merge_metadata["records"][0]["observations"][0].get("affine_2x3") is None:
        raise AdapterError("self-test: native affine observation sidecar was dropped")
    if merge_metadata["records"][0]["images"][0].get("exposure") is None:
        raise AdapterError("self-test: native image exposure sidecar was dropped")

    def expect_reject(label: str, candidate: Mapping[str, Any], frame_map: Mapping[int, int] = {10: 100, 20: 200}) -> None:
        try:
            adapt_native_bridge(
                doc,
                frame_map,
                "self-test map",
                candidate,
                None,
                "self-test malformed image archive",
            )
        except AdapterError:
            return
        raise AdapterError(f"self-test: malformed optical-flow archive accepted: {label}")

    duplicate_record = json.loads(json.dumps(archive))
    duplicate_record["records"].append(json.loads(json.dumps(archive["records"][0])))
    expect_reject("duplicate timestamp", duplicate_record)
    bad_sample_hash = json.loads(json.dumps(archive))
    bad_sample_hash["records"][0]["images"][0]["sample_hash"] = "0000000000000000"
    expect_reject("sample hash mismatch", bad_sample_hash)
    bad_camera_hash = json.loads(json.dumps(archive))
    bad_camera_hash["camera_hashes"]["1"] = "0" * 64
    expect_reject("camera aggregate hash mismatch", bad_camera_hash)
    bad_camera_count = json.loads(json.dumps(archive))
    bad_camera_count["records"][0]["images"] = bad_camera_count["records"][0]["images"][:1]
    expect_reject("camera count mismatch", bad_camera_count)
    expect_reject("missing timestamp mapping", archive, {10: 100})
    expect_reject("ambiguous frame mapping", archive, {10: 100, 20: 100})
    companion = _self_test_companion()
    return {
        "ok": True,
        "test": "native-schema4-projection-roundtrip-and-fail-closed",
        "roundtrip_exact": result["roundtrip"]["exact"],
        "unavailable_fields": result["unavailable_fields"],
        "stored_current_preserved": True,
        "strict_compatibility": False,
        "rust_schema4_extension": {
            "schema": extension["schema"],
            "schema_version": extension["schema_version"],
            "status": extension["status"],
            "fail_closed_missing": extension["unavailable_fields"],
            "roundtrip_exact": extension["roundtrip"]["exact"],
        },
        "multi_archive": {
            "roundtrip_exact": merged["roundtrip"]["exact"],
            "record_count": merge_metadata["record_count"],
            "observation_count": merge_metadata["observation_count"],
            "image_count": merge_metadata["image_count"],
            "camera_count": merge_metadata["camera_count"],
            "rejects_duplicate_missing_ambiguous_and_hash_mismatch": True,
            "native_affine_exposure_sidecars_preserved": True,
        },
        "strict_companion_profile": companion,
    }


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, help="native bridge v1 JSON")
    parser.add_argument("--frame-map", type=Path, help="timestamp-to-frame-ID JSON")
    parser.add_argument(
        "--optical-flow-archive",
        "--images-archive",
        dest="optical_flow_archive",
        type=Path,
        help="normalized JSON, a directory with manifest.json, or a raw pinned native images/*.cereal directory",
    )
    parser.add_argument(
        "--companion",
        type=Path,
        help="same-run native runtime companion manifest (.companion.json)",
    )
    parser.add_argument(
        "--native-packet",
        type=Path,
        help="same-run native MargData cereal packet named by the companion manifest",
    )
    parser.add_argument(
        "--partition",
        type=Path,
        help="same-run native partition sidecar captured after Q2 index selection",
    )
    parser.add_argument(
        "--binding-manifest",
        type=Path,
        help="strict provenance manifest whose artifact paths are re-hashed before acceptance",
    )
    parser.add_argument(
        "--rust-margdata",
        type=Path,
        help="optional Rust schema-4 MargData JSON for an explicitly unbound FEJ oracle comparison",
    )
    parser.add_argument(
        "--rust-extension-capture",
        type=Path,
        help="same-event native schema4-v3 extension capture with FEJ/prior/row/reanchor bindings",
    )
    parser.add_argument(
        "--rust-schema4-output",
        type=Path,
        help="write a complete Rust MargData JSON only when the v3 extension capture is fully bound",
    )
    parser.add_argument("--output", type=Path, help="write adapter envelope JSON")
    parser.add_argument("--roundtrip", action="store_true", help="require exact adapter projection round-trip")
    parser.add_argument("--strict-schema4", action="store_true", help="return failure when any field is unavailable")
    parser.add_argument(
        "--strict-native-compatible",
        action="store_true",
        help="require the native-compatible profile; Rust-only extension fields remain separate",
    )
    parser.add_argument(
        "--strict-rust-schema4",
        action="store_true",
        help="require a complete same-event Rust schema4 extension capture",
    )
    parser.add_argument("--self-test", action="store_true", help="run bounded in-memory tests")
    args = parser.parse_args(list(argv) if argv is not None else None)
    try:
        if args.self_test:
            print(json.dumps(_self_test(), sort_keys=True))
            return 0
        if args.input is None:
            raise AdapterError("--input is required unless --self-test is used")
        validator = _validator()
        with args.input.open("r", encoding="utf-8") as stream:
            document = json.load(stream)
        frame_map = (
            _load_authoritative_frame_map(args.frame_map)
            if args.optical_flow_archive is not None
            else validator._load_frame_map(args.frame_map)
        )
        optical_flow_archive = None
        optical_flow_source_root = None
        optical_flow_source = "unavailable"
        if args.optical_flow_archive is not None:
            optical_flow_archive, optical_flow_source_root, optical_flow_source = (
                _load_optical_flow_archive(args.optical_flow_archive)
            )
        companion_manifest = None
        if args.companion is not None:
            try:
                with args.companion.open("r", encoding="utf-8") as stream:
                    companion_manifest = json.load(stream)
            except (OSError, json.JSONDecodeError) as error:
                raise AdapterError(
                    f"companion manifest: cannot read {args.companion}: {error}"
                ) from error
            if not isinstance(companion_manifest, dict):
                raise AdapterError("companion manifest: expected a JSON object")
        binding_manifest = None
        if args.binding_manifest is not None:
            try:
                with args.binding_manifest.open("r", encoding="utf-8") as stream:
                    binding_manifest = json.load(stream)
            except (OSError, json.JSONDecodeError) as error:
                raise AdapterError(
                    f"binding manifest: cannot read {args.binding_manifest}: {error}"
                ) from error
            if not isinstance(binding_manifest, dict):
                raise AdapterError("binding manifest: expected a JSON object")
        rust_margdata = None
        if args.rust_margdata is not None:
            try:
                with args.rust_margdata.open("r", encoding="utf-8") as stream:
                    rust_margdata = json.load(stream)
            except (OSError, json.JSONDecodeError) as error:
                raise AdapterError(
                    f"Rust MargData oracle: cannot read {args.rust_margdata}: {error}"
                ) from error
            if not isinstance(rust_margdata, dict):
                raise AdapterError("Rust MargData oracle: expected a JSON object")
        rust_extension_capture = None
        if args.rust_extension_capture is not None:
            try:
                with args.rust_extension_capture.open("r", encoding="utf-8") as stream:
                    rust_extension_capture = json.load(stream)
            except (OSError, json.JSONDecodeError) as error:
                raise AdapterError(
                    f"Rust extension capture: cannot read {args.rust_extension_capture}: {error}"
                ) from error
            if not isinstance(rust_extension_capture, dict):
                raise AdapterError("Rust extension capture: expected a JSON object")
        result = adapt_native_bridge(
            document,
            frame_map,
            "external timestamp-to-frame-ID map" if args.frame_map else "unavailable",
            optical_flow_archive,
            optical_flow_source_root,
            optical_flow_source,
            companion_manifest,
            args.companion,
            args.native_packet,
            args.partition,
            binding_manifest,
            args.binding_manifest,
            args.optical_flow_archive,
            rust_margdata,
            args.rust_margdata,
            rust_extension_capture,
            args.rust_extension_capture,
            args.input,
        )
        if args.roundtrip and not result["roundtrip"]["exact"]:
            raise AdapterError("adapter projection JSON round-trip was not exact")
        if args.output is not None:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_bytes(_canonical(result) + b"\n")
        rust_extension = result["rust_schema4_extension"]
        if args.rust_schema4_output is not None:
            if rust_extension["status"] != "RUST_SCHEMA4_READY" or rust_extension["rust_margdata"] is None:
                raise AdapterError(
                    "--rust-schema4-output requires a complete same-event Rust schema4 extension capture"
                )
            args.rust_schema4_output.parent.mkdir(parents=True, exist_ok=True)
            args.rust_schema4_output.write_bytes(
                _canonical(rust_extension["rust_margdata"]) + b"\n"
            )
        summary = {
            "ok": True,
            "adapter_schema": ADAPTER_SCHEMA,
            "projection_written": str(args.output) if args.output else None,
            "rust_schema4_written": str(args.rust_schema4_output)
            if args.rust_schema4_output else None,
            "roundtrip_exact": result["roundtrip"]["exact"],
            "rust_schema4_compatible": result["rust_schema4_compatible"],
            "unavailable_fields": result["unavailable_fields"],
            "optical_flow_merged": result["optical_flow_merge"] is not None,
            "strict_schema4_profile_accepted": bool(
                result["strict_schema4_profile"]
                and result["strict_schema4_profile"]["accepted"]
            ),
            "native_compatible_profile_accepted": bool(
                result["strict_schema4_profile"]
                and result["strict_schema4_profile"]["native_compatible_accepted"]
            ),
            "strict_schema4_profile_status": (
                result["strict_schema4_profile"]["strict_profile_status"]
                if result["strict_schema4_profile"] else "MISSING_COMPANION"
            ),
            "native_compatible_profile_status": (
                result["strict_schema4_profile"]["native_compatible_profile_status"]
                if result["strict_schema4_profile"] else "MISSING_COMPANION"
            ),
            "rust_extension_mapping_status": result["rust_extension_mapping"].get("status"),
            "rust_extension_frame_sidecars_status": result["rust_extension_mapping"].get(
                "frame_sidecars", {}
            ).get("status"),
            "rust_extension_prior_fej_point_status": result["rust_extension_mapping"].get(
                "prior_fej_point", {}
            ).get("status"),
            "rust_extension_direct_fej_oracle_status": result["rust_extension_mapping"].get(
                "direct_fej_oracle", {}
            ).get("status"),
            "strict_schema4_minimal_bundle_status": (
                result["strict_schema4_profile"]["minimal_profile_status"]
                if result["strict_schema4_profile"] else "MISSING_COMPANION"
            ),
            "strict_schema4_provenance_status": (
                result["strict_schema4_profile"]["provenance_status"]
                if result["strict_schema4_profile"] else "MISSING_COMPANION"
            ),
            "strict_schema4_profile_missing_fields": (
                result["strict_schema4_profile"]["missing_fields"]
                if result["strict_schema4_profile"] else ["companion"]
            ),
            "native_compatible_profile_missing_fields": (
                result["strict_schema4_profile"]["native_compatible_missing_fields"]
                if result["strict_schema4_profile"] else ["companion"]
            ),
            "rust_schema4_extension_status": rust_extension["status"],
            "rust_schema4_extension_missing_fields": rust_extension["unavailable_fields"],
            "rust_schema4_extension_roundtrip_exact": rust_extension["roundtrip"]["exact"],
        }
        print(json.dumps(summary, sort_keys=True))
        if args.strict_schema4 and (
            result["strict_schema4_profile"] is None
            or not result["strict_schema4_profile"]["accepted"]
        ):
            return 2
        if args.strict_native_compatible and (
            result["strict_schema4_profile"] is None
            or not result["strict_schema4_profile"]["native_compatible_accepted"]
        ):
            return 2
        if args.strict_rust_schema4 and rust_extension["status"] != "RUST_SCHEMA4_READY":
            return 2
        return 0
    except (AdapterError, OSError, json.JSONDecodeError, ValueError) as error:
        print(json.dumps({"ok": False, "error": str(error)}), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
