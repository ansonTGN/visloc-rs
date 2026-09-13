"""Validate non-Cargo third-party source dependencies used by Basalt.

The selected Cargo graph cannot discover manually translated source embedded in
production files.  This focused, read-only verifier binds the checked-in
supplemental manifest to the current affected file and to the known immutable
Arm Optimized Routines v26.01 response hashes.  It deliberately does not
download or copy license payloads.

Exact final-workflow invocation::

    python benchmarks/basalt/verify_supplemental_source_dependencies.py \
        --root C:/Users/rsasa/Workspace/visloc-rs \
        [--payload-root C:/path/to/reviewed/payload]

The current source manifest binds a persisted AOR license payload.  Missing,
tampered, or stale source/payload bytes still fail closed.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import sys
from pathlib import Path
from typing import Any, Mapping


SCHEMA_ID = "visloc-rs.basalt.supplemental-source-dependencies.v1"
DEFAULT_MANIFEST = Path("benchmarks/basalt/supplemental_source_dependencies_v1.json")
EXPECTED_ID = "arm-optimized-routines-cosf-v26.01"
EXPECTED_REPOSITORY = "https://github.com/ARM-software/optimized-routines"
EXPECTED_TAG = "v26.01"
EXPECTED_COMMIT = "649ccc8411e7c4862965710f25c570d854a1483f"
EXPECTED_LICENSE = "MIT OR Apache-2.0 WITH LLVM-exception"
EXPECTED_PRODUCTION_SCOPE = "production finite-angle Se2::exp path"
PASS_STATUS = "pass_supplemental_source_dependency"
PENDING_STATUS = "pending_supplemental_license_payload"
FAIL_STATUS = "fail_supplemental_source_dependency"
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")

KNOWN_SOURCE_FILES: dict[str, tuple[int, str]] = {
    "math/cosf.c": (
        1658,
        "84068b509762ae5c3ba4f1488f73cce099ffbbf1df3f410aa1c7538562d41b4c",
    ),
    "math/sincosf.h": (
        4246,
        "ae5b8962056f72fa9f844b980bed0c50a3612f96e7a17629eb10e6c9b428463b",
    ),
    "math/sincosf_data.c": (
        1613,
        "9887da2fb996658a88e98ec7ea5142be2b4a8db5650f7fd3290228d67d405cdb",
    ),
    "math/sinf.c": (
        1748,
        "5b4961963b7b687b96c53db34ce2009feaa93f78cd9acf7b355fd9040b66f52f",
    ),
}
KNOWN_LICENSE_FILE = (
    13491,
    "650afbf29f214451e02241adc42534e82c9d6ae2b38e2444b92b5a1ffcaf9346",
)
PERSISTED_PAYLOAD_STATUSES = {
    "RELEASE_PAYLOAD_PERSISTED",
    "verified_metadata_with_release_payload",
}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _display_path(path: Path) -> str:
    return path.resolve().as_posix()


def _load_json(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def _relative_file(root: Path, raw: Any, errors: list[str], label: str) -> Path | None:
    if not isinstance(raw, str) or not raw:
        errors.append(f"{label} must be a non-empty relative path")
        return None
    candidate = Path(raw.replace("\\", "/"))
    if candidate.is_absolute() or ".." in candidate.parts:
        errors.append(f"{label} escapes the repository root: {raw!r}")
        return None
    lexical = root / candidate
    cursor = root
    for part in candidate.parts:
        cursor /= part
        if cursor.is_symlink():
            errors.append(f"{label} traverses a symlink: {raw!r}")
            return None
    resolved = lexical.resolve()
    try:
        resolved.relative_to(root)
    except ValueError:
        errors.append(f"{label} escapes the repository root: {raw!r}")
        return None
    return resolved


def _verify_local_file(
    root: Path,
    record: Mapping[str, Any],
    errors: list[str],
    label: str,
) -> bool:
    path = _relative_file(root, record.get("path"), errors, f"{label}.path")
    expected_bytes = record.get("bytes")
    expected_sha256 = record.get("sha256")
    if not isinstance(expected_bytes, int) or expected_bytes < 0:
        errors.append(f"{label}.bytes must be a non-negative integer")
    if not isinstance(expected_sha256, str) or not SHA256_RE.fullmatch(expected_sha256.lower()):
        errors.append(f"{label}.sha256 must be a lowercase 64-hex SHA-256")
    if path is None or not isinstance(expected_bytes, int) or not isinstance(expected_sha256, str):
        return False
    if path.is_symlink() or not path.is_file():
        errors.append(f"{label} file is missing or symlinked: {path}")
        return False
    actual_bytes = path.stat().st_size
    actual_sha256 = sha256_file(path)
    if actual_bytes != expected_bytes:
        errors.append(f"{label} byte mismatch: {actual_bytes} != {expected_bytes}")
    if actual_sha256.lower() != expected_sha256.lower():
        errors.append(f"{label} SHA-256 mismatch: {actual_sha256} != {expected_sha256}")
    return actual_bytes == expected_bytes and actual_sha256.lower() == expected_sha256.lower()


def validate_manifest(
    root: Path,
    manifest_path: Path | None = None,
    payload_root: Path | None = None,
) -> dict[str, Any]:
    """Return a structured validation result; pending payload is not a pass."""

    root = root.resolve()
    payload_root = (payload_root or root).resolve()
    path = (manifest_path or root / DEFAULT_MANIFEST).resolve()
    result: dict[str, Any] = {
        "schema": "visloc.basalt.supplemental-source-dependencies.check.v1",
        "manifest_path": _display_path(path),
        "status": FAIL_STATUS,
        "release_ready": False,
        "payload_root": _display_path(payload_root),
        "errors": [],
    }
    errors: list[str] = result["errors"]
    if path.is_symlink() or not path.is_file():
        errors.append(f"supplemental manifest is missing or symlinked: {path}")
        result["status"] = FAIL_STATUS
        return result
    try:
        payload = _load_json(path)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        errors.append(f"cannot load supplemental manifest: {exc}")
        return result
    result["manifest_bytes"] = path.stat().st_size
    result["manifest_sha256"] = sha256_file(path)
    if payload.get("schema_id") != SCHEMA_ID:
        errors.append(f"schema_id mismatch: {payload.get('schema_id')!r}")
    if payload.get("schema_version") != 1:
        errors.append(f"schema_version mismatch: {payload.get('schema_version')!r}")
    policy = payload.get("coverage_policy")
    if not isinstance(policy, dict):
        errors.append("coverage_policy must be an object")
    else:
        if not isinstance(policy.get("final_release_gate_rule"), str) or not policy["final_release_gate_rule"]:
            errors.append("coverage_policy.final_release_gate_rule is missing")
        if policy.get("rebind_at_freeze") is not True:
            errors.append("coverage_policy.rebind_at_freeze must be true")
    dependencies = payload.get("dependencies")
    if not isinstance(dependencies, list) or len(dependencies) != 1:
        errors.append("supplemental dependencies must contain exactly one AOR record")
        return result
    dependency = dependencies[0]
    if not isinstance(dependency, dict):
        errors.append("supplemental dependency record must be an object")
        return result
    result["manual_source_dependency_count"] = 1
    if dependency.get("id") != EXPECTED_ID:
        errors.append(f"unexpected supplemental dependency id: {dependency.get('id')!r}")
    if dependency.get("repository") != EXPECTED_REPOSITORY:
        errors.append("AOR repository is not the expected immutable repository")
    if dependency.get("tag") != EXPECTED_TAG:
        errors.append("AOR tag is not v26.01")
    if dependency.get("commit") != EXPECTED_COMMIT:
        errors.append("AOR full commit does not match the known v26.01 pin")
    if dependency.get("license_spdx") != EXPECTED_LICENSE:
        errors.append("AOR SPDX expression mismatch")
    covered_functions = dependency.get("covered_functions")
    if not isinstance(covered_functions, dict):
        errors.append("covered_functions must be an object")
    else:
        for function_name in ("cosf", "sinf"):
            if covered_functions.get(function_name) != EXPECTED_PRODUCTION_SCOPE:
                errors.append(
                    f"covered_functions.{function_name} must bind the finite-angle Se2::exp production path"
                )
    source_paths = dependency.get("source_paths")
    if source_paths != sorted(KNOWN_SOURCE_FILES):
        errors.append(f"AOR source_paths mismatch: {source_paths!r}")
    source_hashes = dependency.get("source_hashes")
    seen_sources: set[str] = set()
    source_hashes_verified = 0
    if not isinstance(source_hashes, list):
        errors.append("source_hashes must be a list")
        source_hashes = []
    for index, item in enumerate(source_hashes):
        label = f"source_hashes[{index}]"
        if not isinstance(item, dict):
            errors.append(f"{label} must be an object")
            continue
        source_path = item.get("path")
        if not isinstance(source_path, str) or source_path in seen_sources:
            errors.append(f"{label}.path is missing or duplicated")
            continue
        seen_sources.add(source_path)
        expected = KNOWN_SOURCE_FILES.get(source_path)
        if expected is None:
            errors.append(f"{label}.path is not one of the known AOR files: {source_path}")
            continue
        expected_url = f"https://raw.githubusercontent.com/ARM-software/optimized-routines/{EXPECTED_COMMIT}/{source_path}"
        if item.get("url") != expected_url:
            errors.append(f"{label}.url does not bind the expected immutable commit")
        if item.get("http_status") != 200:
            errors.append(f"{label}.http_status must be 200")
        bytes_ok = item.get("bytes") == expected[0]
        hash_ok = str(item.get("sha256", "")).lower() == expected[1]
        if not bytes_ok:
            errors.append(f"{label}.bytes does not match known immutable bytes")
        if not hash_ok:
            errors.append(f"{label}.sha256 does not match known immutable hash")
        if item.get("url") == expected_url and item.get("http_status") == 200 and bytes_ok and hash_ok:
            source_hashes_verified += 1
    if seen_sources != set(KNOWN_SOURCE_FILES):
        errors.append(f"source_hashes are incomplete: {sorted(seen_sources)!r}")
    license_evidence = dependency.get("license_evidence")
    license_hash_verified = False
    if not isinstance(license_evidence, dict):
        errors.append("license_evidence must be an object")
        license_evidence = {}
    expected_license_url = f"https://raw.githubusercontent.com/ARM-software/optimized-routines/{EXPECTED_COMMIT}/LICENSE"
    if license_evidence.get("path") != "LICENSE":
        errors.append("license_evidence.path must be LICENSE")
    if license_evidence.get("url") != expected_license_url:
        errors.append("license_evidence.url does not bind the expected immutable commit")
    if license_evidence.get("http_status") != 200:
        errors.append("license_evidence.http_status must be 200")
    if license_evidence.get("bytes") != KNOWN_LICENSE_FILE[0]:
        errors.append("license_evidence.bytes does not match known immutable bytes")
    license_hash_matches = str(license_evidence.get("sha256", "")).lower() == KNOWN_LICENSE_FILE[1]
    if not license_hash_matches:
        errors.append("license_evidence.sha256 does not match known immutable hash")
    if (
        license_evidence.get("path") == "LICENSE"
        and license_evidence.get("url") == expected_license_url
        and license_evidence.get("http_status") == 200
        and license_evidence.get("bytes") == KNOWN_LICENSE_FILE[0]
        and license_hash_matches
    ):
        license_hash_verified = True
    payload_status = license_evidence.get("payload_status")
    if not isinstance(payload_status, str) or not payload_status:
        errors.append("license_evidence.payload_status is missing")
        payload_status = "unknown"
    notice_evidence = dependency.get("notice_evidence")
    if not isinstance(notice_evidence, dict):
        errors.append("notice_evidence must be an object")
    elif notice_evidence.get("http_status") != 404:
        errors.append("notice_evidence must preserve the exact root NOTICE 404 observation")
    affected = dependency.get("affected_production_files")
    affected_verified = 0
    if not isinstance(affected, list) or not affected:
        errors.append("affected_production_files must be a non-empty list")
        affected = []
    for index, item in enumerate(affected):
        if not isinstance(item, dict):
            errors.append(f"affected_production_files[{index}] must be an object")
            continue
        function_scope = item.get("function_scope")
        if not isinstance(function_scope, dict):
            errors.append(f"affected_production_files[{index}].function_scope must be an object")
        elif function_scope.get("sinf") != "production":
            errors.append(
                f"affected_production_files[{index}].function_scope.sinf must be production"
            )
        if _verify_local_file(root, item, errors, f"affected_production_files[{index}]"):
            affected_verified += 1
    payload_path_raw = license_evidence.get("payload_path")
    payload_file_verified = False
    if payload_status in PERSISTED_PAYLOAD_STATUSES:
        if license_evidence.get("payload_bytes") != KNOWN_LICENSE_FILE[0]:
            errors.append("license_evidence.payload_bytes does not match the known immutable LICENSE bytes")
        if str(license_evidence.get("payload_sha256", "")).lower() != KNOWN_LICENSE_FILE[1]:
            errors.append("license_evidence.payload_sha256 does not match the known immutable LICENSE hash")
        payload_path = _relative_file(
            payload_root,
            payload_path_raw,
            errors,
            "license_evidence.payload_path",
        )
        payload_file_verified = _verify_local_file(
            payload_root,
            {
                "path": payload_path_raw,
                "bytes": KNOWN_LICENSE_FILE[0],
                "sha256": KNOWN_LICENSE_FILE[1],
            },
            errors,
            "license_evidence.payload",
        ) if payload_path is not None else False
    result.update(
        {
            "known_source_file_count": len(KNOWN_SOURCE_FILES),
            "known_source_hashes_verified_count": source_hashes_verified,
            "known_license_hash_verified": license_hash_verified,
            "affected_production_file_count": len(affected),
            "affected_production_file_hashes_verified_count": affected_verified,
            "license_payload_status": payload_status,
            "payload_path": payload_path_raw,
            "payload_file_verified": payload_file_verified,
            "payload_persisted": payload_status in PERSISTED_PAYLOAD_STATUSES and payload_file_verified,
        }
    )
    if not errors:
        if result["payload_persisted"]:
            result["status"] = PASS_STATUS
            result["release_ready"] = True
        else:
            result["status"] = PENDING_STATUS
            result["release_ready"] = False
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--manifest", type=Path, default=None)
    parser.add_argument("--payload-root", type=Path, default=None)
    args = parser.parse_args(argv)
    result = validate_manifest(
        args.root.resolve(),
        args.manifest.resolve() if args.manifest else None,
        args.payload_root.resolve() if args.payload_root else None,
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == PASS_STATUS else 1


if __name__ == "__main__":
    raise SystemExit(main())
