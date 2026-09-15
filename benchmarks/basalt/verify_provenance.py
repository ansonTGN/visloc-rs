"""Validate the checked-in Basalt provenance and release metadata.

The validator deliberately uses only the Python standard library.  It checks
the immutable upstream anchors recorded by the M0 manifest, exact hashes of
checked-in oracle/fixture generators, coverage of production Rust and
diagnostic source files, license/SPDX fields, and the release GT firewall.
The current Cargo license inventory is v2 by default; ``--cargo-license-inventory
v1`` explicitly audits the immutable v1 history file.  The v2 overlay is
checked against its v1 package base and all recorded crate/source/license
evidence.

With ``--checkout name=path`` it additionally verifies a materialized checkout
against its commit/tree and every listed upstream source path.  No checkout is
required for normal CI because the pinned oracle is intentionally external to
the repository.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
import sys
import tarfile
from pathlib import Path
from typing import Any, Iterable


SCHEMA_ID = "visloc-rs.basalt.provenance.v1"
RELEASE_SCHEMA_ID = "visloc-rs.basalt.release.v1"
COMMIT_RE = re.compile(r"^[0-9a-f]{40}$")
TREE_RE = COMMIT_RE
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
SPDX_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9+.\-]*$")
GT_FRAGMENTS = (
    "state_groundtruth",
    "groundtruth",
    "ground_truth",
    "/target/",
    "\\target\\",
    "marg_data",
)
CARGO_LICENSE_V1_PATH = "benchmarks/basalt/cargo_license_inventory_v1.json"
CARGO_LICENSE_V2_PATH = "benchmarks/basalt/cargo_license_inventory_v2.json"
CARGO_LICENSE_V1_SCHEMA_ID = "visloc-rs.cargo-license.v1"
CARGO_LICENSE_V2_SCHEMA_ID = "visloc-rs.cargo-license.v2"
CARGO_LICENSE_SOURCE = "registry+https://github.com/rust-lang/crates.io-index"
CARGO_LICENSE_PACKAGE_KEY_RE = re.compile(r"^.+ .+$")
SPDX_EXPRESSION_RE = re.compile(
    r"^[A-Za-z0-9][A-Za-z0-9+.\-]*(?:\s+(?:OR|AND|WITH)\s+[A-Za-z0-9][A-Za-z0-9+.\-]*)*$"
)


def _load(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _git(root: Path, *args: str) -> str:
    completed = subprocess.run(
        ["git", "-C", str(root), *args],
        check=True,
        capture_output=True,
        text=True,
    )
    return completed.stdout.strip()


def _iter_paths(root: Path, repo_root: Path, suffixes: Iterable[str]) -> set[str]:
    suffix_set = set(suffixes)
    return {
        path.relative_to(repo_root).as_posix()
        for path in root.rglob("*")
        if path.is_file() and path.suffix in suffix_set
    }


def _is_path_like(value: str) -> bool:
    return not value.startswith("/") and ".." not in Path(value).parts


def _check_checkout(
    name: str, checkout: Path, record: dict[str, Any], errors: list[str]
) -> None:
    if not checkout.is_dir():
        errors.append(f"checkout {name} does not exist: {checkout}")
        return
    try:
        actual_commit = _git(checkout, "rev-parse", "HEAD")
        actual_tree = _git(checkout, "rev-parse", "HEAD^{tree}")
    except (OSError, subprocess.CalledProcessError) as exc:
        errors.append(f"checkout {name} is not a readable git checkout: {exc}")
        return
    if actual_commit != record["commit"]:
        errors.append(
            f"checkout {name} commit mismatch: {actual_commit} != {record['commit']}"
        )
    if actual_tree != record["tree_sha1"]:
        errors.append(
            f"checkout {name} tree mismatch: {actual_tree} != {record['tree_sha1']}"
        )
    for source_path in record.get("source_paths", []):
        if not _is_path_like(source_path):
            errors.append(f"{name} source path is not repository-relative: {source_path}")
            continue
        if not (checkout / Path(source_path)).is_file():
            errors.append(f"{name} source path is missing: {source_path}")


def _validate_upstreams(
    root: Path,
    manifest: dict[str, Any],
    errors: list[str],
    checkouts: dict[str, Path],
) -> None:
    upstreams = manifest.get("upstreams")
    if not isinstance(upstreams, dict) or not upstreams:
        errors.append("upstreams must be a non-empty object")
        return
    for name, record in upstreams.items():
        if not isinstance(record, dict):
            errors.append(f"upstreams.{name} must be an object")
            continue
        for field, pattern in (("commit", COMMIT_RE), ("tree_sha1", TREE_RE)):
            value = record.get(field)
            if not isinstance(value, str) or not pattern.fullmatch(value):
                errors.append(f"upstreams.{name}.{field} is not a 40-hex pin")
        license_spdx = record.get("license_spdx")
        if not isinstance(license_spdx, str) or not SPDX_RE.fullmatch(license_spdx):
            errors.append(f"upstreams.{name}.license_spdx is missing/invalid")
        if record.get("redistribution_notice_required") is not True:
            errors.append(f"upstreams.{name}.redistribution_notice_required must be true")
        if not isinstance(record.get("license_file"), str):
            errors.append(f"upstreams.{name}.license_file is missing")
        source_paths = record.get("source_paths")
        if not isinstance(source_paths, list) or not source_paths:
            errors.append(f"upstreams.{name}.source_paths must be non-empty")
        else:
            for source_path in source_paths:
                if not isinstance(source_path, str) or not _is_path_like(source_path):
                    errors.append(f"{name} source path is not repository-relative: {source_path}")
        if name in checkouts:
            _check_checkout(name, checkouts[name], record, errors)

    upstream_manifest_path = root / "benchmarks/basalt/upstream_manifest.json"
    if upstream_manifest_path.is_file() and isinstance(upstreams.get("basalt"), dict):
        old = _load(upstream_manifest_path)
        pinned = upstreams["basalt"]
        for field in ("commit", "tree_sha1"):
            if old.get("repository", {}).get(field) != pinned[field]:
                errors.append(
                    f"upstream_manifest.json repository.{field} disagrees with provenance manifest"
                )
        old_vcpkg = old.get("repository", {}).get("submodules", {}).get("thirdparty/vcpkg", {})
        vcpkg = manifest.get("vcpkg_resolution", {})
        if old_vcpkg.get("commit") != vcpkg.get("commit"):
            errors.append("upstream_manifest.json vcpkg submodule pin disagrees")
        old_baseline = old.get("build_contract", {}).get("vcpkg_baseline")
        if old_baseline != vcpkg.get("baseline"):
            errors.append("upstream_manifest.json vcpkg baseline disagrees")
        for package, version in vcpkg.get("registry_ports", {}).items():
            if not isinstance(package, str) or not package or not isinstance(version, str) or not version:
                errors.append("vcpkg_resolution.registry_ports contains an invalid package/version")


def _validate_port_files(root: Path, manifest: dict[str, Any], errors: list[str]) -> None:
    records = manifest.get("port_files")
    if not isinstance(records, dict) or not records:
        errors.append("port_files must be a non-empty object")
        return
    actual = _iter_paths(root / "pipelines/basalt/src", root, (".rs",))
    listed = set(records)
    if actual != listed:
        errors.append(
            "production Rust coverage mismatch: "
            f"missing={sorted(actual - listed)}, extra={sorted(listed - actual)}"
        )
    for path, record in records.items():
        if not isinstance(record, dict):
            errors.append(f"port_files.{path} must be an object")
            continue
        if record.get("translation_status") not in {
            "clean_room_translation",
            "clean_room_adapter",
            "clean_room_contract",
            "crate_boundary",
            "metadata_only",
        }:
            errors.append(f"port_files.{path} has invalid translation_status")
        refs = record.get("upstream_refs")
        if not isinstance(refs, list) or not refs:
            errors.append(f"port_files.{path} has no upstream_refs")
        elif any(ref not in manifest["upstreams"] for ref in refs):
            errors.append(f"port_files.{path} has an unknown upstream_ref")
        expected_hash = record.get("sha256")
        if expected_hash is not None:
            if not isinstance(expected_hash, str) or not SHA256_RE.fullmatch(expected_hash):
                errors.append(f"port_files.{path}.sha256 is invalid")
            elif (root / path).is_file() and _sha256(root / path) != expected_hash:
                errors.append(f"hash mismatch for production source {path}")
            elif not (root / path).is_file():
                errors.append(f"listed production source is missing: {path}")
        source_mapping = record.get("upstream_source_paths")
        if source_mapping is not None:
            if not isinstance(source_mapping, dict) or not source_mapping:
                errors.append(f"port_files.{path}.upstream_source_paths must be a non-empty object")
            else:
                for upstream, source_paths in source_mapping.items():
                    if upstream not in manifest["upstreams"]:
                        errors.append(
                            f"port_files.{path}.upstream_source_paths has an unknown upstream: {upstream}"
                        )
                    if not isinstance(source_paths, list) or not source_paths:
                        errors.append(
                            f"port_files.{path}.upstream_source_paths.{upstream} must be non-empty"
                        )
                    elif any(
                        not isinstance(source_path, str) or not _is_path_like(source_path)
                        for source_path in source_paths
                    ):
                        errors.append(
                            f"port_files.{path}.upstream_source_paths.{upstream} has an invalid path"
                        )


def _validate_retained_audit_sources(
    root: Path, manifest: dict[str, Any], errors: list[str]
) -> None:
    records = manifest.get("retained_audit_sources")
    if not isinstance(records, dict) or not records:
        errors.append("retained_audit_sources must be a non-empty object")
        return
    for path, record in records.items():
        if not isinstance(path, str) or not _is_path_like(path):
            errors.append(f"retained_audit_sources has an invalid repository path: {path}")
            continue
        if not isinstance(record, dict):
            errors.append(f"retained_audit_sources.{path} must be an object")
            continue
        expected = record.get("sha256")
        if not isinstance(expected, str) or not SHA256_RE.fullmatch(expected):
            errors.append(f"retained_audit_sources.{path}.sha256 is invalid")
        elif (root / path).is_file() and _sha256(root / path) != expected:
            errors.append(f"hash mismatch for retained audit source {path}")
        elif not (root / path).is_file():
            errors.append(f"listed retained audit source is missing: {path}")
        if not isinstance(record.get("kind"), str) or not record["kind"]:
            errors.append(f"retained_audit_sources.{path}.kind is missing")
        if record.get("copied_upstream_source") is not False:
            errors.append(f"{path} must explicitly declare copied_upstream_source=false")
        refs = record.get("upstream_refs")
        if not isinstance(refs, list) or not refs:
            errors.append(f"retained_audit_sources.{path} has no upstream_refs")
        elif any(ref not in manifest["upstreams"] for ref in refs):
            errors.append(f"retained_audit_sources.{path} has an unknown upstream_ref")
        if not isinstance(record.get("retention"), str) or not record["retention"]:
            errors.append(f"retained_audit_sources.{path}.retention is missing")


def _validate_generator_hashes(root: Path, manifest: dict[str, Any], errors: list[str]) -> None:
    records = manifest.get("checked_in_generator_hashes")
    if not isinstance(records, dict) or not records:
        errors.append("checked_in_generator_hashes must be a non-empty object")
        return
    actual: set[str] = set()
    actual |= {
        path
        for path in _iter_paths(
            root / "benchmarks/basalt", root, (".cpp", ".cc", ".py", ".ps1", ".sh")
        )
        if Path(path).name not in {
            "test_provenance.py",
            "verify_provenance.py",
            "refresh_provenance_hashes.py",
        }
    }
    actual |= _iter_paths(root / "pipelines/basalt/examples", root, (".cpp", ".cc", ".rs", ".py", ".ps1", ".sh"))
    actual.discard("benchmarks/basalt/verify_provenance.py")
    actual.discard("benchmarks/basalt/refresh_provenance_hashes.py")
    listed = set(records)
    if actual != listed:
        errors.append(
            "generator/diagnostic coverage mismatch: "
            f"missing={sorted(actual - listed)}, extra={sorted(listed - actual)}"
        )
    for path, record in records.items():
        if not isinstance(record, dict):
            errors.append(f"checked_in_generator_hashes.{path} must be an object")
            continue
        expected = record.get("sha256")
        if not isinstance(expected, str) or not SHA256_RE.fullmatch(expected):
            errors.append(f"checked_in_generator_hashes.{path}.sha256 is invalid")
        elif (root / path).is_file() and _sha256(root / path) != expected:
            errors.append(f"hash mismatch for {path}")
        elif not (root / path).is_file():
            errors.append(f"listed generator is missing: {path}")
        if record.get("copied_upstream_source") is not False:
            errors.append(f"{path} must explicitly declare copied_upstream_source=false")
        refs = record.get("upstream_refs")
        if not isinstance(refs, list) or any(ref not in manifest["upstreams"] for ref in refs):
            errors.append(f"{path} has invalid upstream_refs")


def _inventory_file(root: Path, value: Any, label: str, errors: list[str], *, directory: bool = False) -> Path | None:
    """Resolve a repository-relative inventory evidence path and check its type."""

    if not isinstance(value, str) or not _is_path_like(value):
        errors.append(f"{label} must be a repository-relative path")
        return None
    path = root / Path(value)
    try:
        path.resolve().relative_to(root.resolve())
    except ValueError:
        errors.append(f"{label} escapes repository root: {value}")
        return None
    if directory:
        if not path.is_dir():
            errors.append(f"{label} directory is missing: {value}")
    elif not path.is_file():
        errors.append(f"{label} file is missing: {value}")
    return path


def _check_file_binding(
    root: Path,
    value: Any,
    expected_bytes: Any,
    expected_sha256: Any,
    label: str,
    errors: list[str],
) -> Path | None:
    path = _inventory_file(root, value, f"{label}.path", errors)
    if path is None or not path.is_file():
        return path
    if expected_bytes is not None:
        if isinstance(expected_bytes, bool) or not isinstance(expected_bytes, int) or expected_bytes < 0:
            errors.append(f"{label}.bytes is invalid")
        elif path.stat().st_size != expected_bytes:
            errors.append(f"{label}.bytes mismatch")
    if not isinstance(expected_sha256, str) or not SHA256_RE.fullmatch(expected_sha256):
        errors.append(f"{label}.sha256 is invalid")
    elif _sha256(path) != expected_sha256:
        errors.append(f"{label}.sha256 mismatch")
    return path


def _check_spdx_expression(value: Any, label: str, errors: list[str]) -> None:
    if not isinstance(value, str) or not SPDX_EXPRESSION_RE.fullmatch(value):
        errors.append(f"{label} is not a valid SPDX expression")


def _archive_member_sha256(
    crate_path: Path, member_basename: str, expected: str, label: str, errors: list[str]
) -> None:
    """Hash one named file in a crate without extracting or mutating the archive."""

    try:
        with tarfile.open(crate_path, mode="r:*") as archive:
            members = [
                member
                for member in archive.getmembers()
                if member.isfile() and member.name.rsplit("/", 1)[-1] == member_basename
            ]
            if len(members) != 1:
                errors.append(
                    f"{label} archive member count for {member_basename!r} is {len(members)}"
                )
                return
            stream = archive.extractfile(members[0])
            if stream is None:
                errors.append(f"{label} archive member is unreadable: {member_basename}")
                return
            digest = hashlib.sha256()
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
            if digest.hexdigest() != expected:
                errors.append(f"{label} archive hash mismatch for {member_basename}")
    except (OSError, tarfile.TarError) as exc:
        errors.append(f"{label} crate archive cannot be inspected: {exc}")


def _inventory_key(record: Any) -> str | None:
    if not isinstance(record, dict):
        return None
    name = record.get("name")
    version = record.get("version")
    if not isinstance(name, str) or not isinstance(version, str) or not name or not version:
        return None
    return f"{name} {version}"


def _validate_cargo_v1_payload(payload: dict[str, Any], errors: list[str], prefix: str) -> None:
    if payload.get("schema_version") != 1:
        errors.append(f"{prefix}.schema_version must be 1")
    if payload.get("schema_id") != CARGO_LICENSE_V1_SCHEMA_ID:
        errors.append(f"{prefix}.schema_id mismatch")
    if payload.get("manifest_id") != "cargo-license-inventory-v1":
        errors.append(f"{prefix}.manifest_id mismatch")
    packages = payload.get("packages")
    if not isinstance(packages, list):
        errors.append(f"{prefix}.packages must be a list")
        return
    keys: list[str] = []
    for index, package in enumerate(packages):
        key = _inventory_key(package)
        if key is None:
            errors.append(f"{prefix}.packages[{index}] has an invalid name/version")
            continue
        keys.append(key)
        if not isinstance(package.get("source"), str) or not package["source"]:
            errors.append(f"{prefix}.packages[{index}].source is missing")
        checksum = package.get("checksum")
        if isinstance(package.get("source"), str) and package["source"].startswith("registry+"):
            if not isinstance(checksum, str) or not SHA256_RE.fullmatch(checksum):
                errors.append(f"{prefix}.packages[{index}].checksum is invalid")
        elif checksum is not None and (not isinstance(checksum, str) or not SHA256_RE.fullmatch(checksum)):
            errors.append(f"{prefix}.packages[{index}].checksum is invalid")
        if not isinstance(package.get("license"), str) or not package["license"]:
            errors.append(f"{prefix}.packages[{index}].license is missing")
    if len(keys) != len(set(keys)):
        errors.append(f"{prefix}.packages contains duplicate name/version keys")
    package_count = payload.get("package_count")
    if isinstance(package_count, bool) or not isinstance(package_count, int) or package_count != len(packages):
        errors.append(f"{prefix}.package_count does not match packages")
    resolved = payload.get("licenses_resolved")
    unresolved = payload.get("licenses_unresolved")
    if any(isinstance(value, bool) or not isinstance(value, int) or value < 0 for value in (resolved, unresolved)):
        errors.append(f"{prefix}.license counts are invalid")
    elif resolved + unresolved != len(packages):
        errors.append(f"{prefix}.license counts do not cover packages")
    unresolved_packages = payload.get("unresolved_packages")
    if not isinstance(unresolved_packages, list) or any(
        not isinstance(item, str) or not CARGO_LICENSE_PACKAGE_KEY_RE.fullmatch(item)
        for item in unresolved_packages
    ):
        errors.append(f"{prefix}.unresolved_packages is invalid")
    elif isinstance(unresolved, int) and len(unresolved_packages) != unresolved:
        errors.append(f"{prefix}.unresolved_packages count mismatch")


def _validate_cargo_license_v1(root: Path, payload: dict[str, Any], errors: list[str]) -> None:
    _validate_cargo_v1_payload(payload, errors, "cargo license v1")


def _validate_cargo_license_v2(root: Path, payload: dict[str, Any], errors: list[str]) -> None:
    prefix = "cargo license v2"
    if payload.get("schema_version") != 2:
        errors.append(f"{prefix}.schema_version must be 2")
    if payload.get("schema_id") != CARGO_LICENSE_V2_SCHEMA_ID:
        errors.append(f"{prefix}.schema_id mismatch")
    if payload.get("manifest_id") != "cargo-license-inventory-v2":
        errors.append(f"{prefix}.manifest_id mismatch")
    if payload.get("record_model") != "immutable_v1_package_set_plus_resolved_evidence_overlay":
        errors.append(f"{prefix}.record_model mismatch")

    supersedes = payload.get("supersedes")
    base_path: Path | None = None
    base_payload: dict[str, Any] | None = None
    if not isinstance(supersedes, dict):
        errors.append(f"{prefix}.supersedes must be an object")
    else:
        if supersedes.get("path") != CARGO_LICENSE_V1_PATH:
            errors.append(f"{prefix}.supersedes.path must name the v1 inventory")
        if supersedes.get("history_preserved") is not True:
            errors.append(f"{prefix}.supersedes.history_preserved must be true")
        base_path = _check_file_binding(
            root,
            supersedes.get("path"),
            None,
            supersedes.get("sha256"),
            f"{prefix}.supersedes",
            errors,
        )
        if base_path is not None and base_path.is_file():
            try:
                base_payload = _load(base_path)
            except (OSError, ValueError, json.JSONDecodeError) as exc:
                errors.append(f"{prefix} cannot load base v1 inventory: {exc}")
            else:
                _validate_cargo_license_v1(root, base_payload, errors)

    basis = payload.get("basis")
    if not isinstance(basis, dict):
        errors.append(f"{prefix}.basis must be an object")
    else:
        if basis.get("base_package_set") != CARGO_LICENSE_V1_PATH:
            errors.append(f"{prefix}.basis.base_package_set mismatch")
        if basis.get("lockfile_version") != 3:
            errors.append(f"{prefix}.basis.lockfile_version must be 3")
        if not isinstance(basis.get("evidence_policy"), str) or not basis["evidence_policy"]:
            errors.append(f"{prefix}.basis.evidence_policy is missing")
        lock_path = _check_file_binding(
            root,
            basis.get("lockfile"),
            None,
            basis.get("lockfile_sha256"),
            f"{prefix}.basis.lockfile",
            errors,
        )
        if lock_path is None:
            pass
        base_count = base_payload.get("package_count") if isinstance(base_payload, dict) else None
        base_unresolved = base_payload.get("licenses_unresolved") if isinstance(base_payload, dict) else None
        if basis.get("base_package_count") != base_count:
            errors.append(f"{prefix}.basis.base_package_count mismatch")
        if basis.get("base_unresolved_count") != base_unresolved:
            errors.append(f"{prefix}.basis.base_unresolved_count mismatch")

    package_count = payload.get("package_count")
    if isinstance(package_count, bool) or not isinstance(package_count, int) or package_count < 0:
        errors.append(f"{prefix}.package_count is invalid")
    if payload.get("licenses_resolved") != package_count:
        errors.append(f"{prefix}.licenses_resolved must equal package_count")
    if payload.get("licenses_unresolved") != 0:
        errors.append(f"{prefix}.licenses_unresolved must be zero")
    if payload.get("unresolved_packages") != []:
        errors.append(f"{prefix}.unresolved_packages must be empty")

    base_by_key: dict[str, dict[str, Any]] = {}
    unresolved_keys: set[str] = set()
    if isinstance(base_payload, dict) and isinstance(base_payload.get("packages"), list):
        base_by_key = {
            key: package
            for package in base_payload["packages"]
            if (key := _inventory_key(package)) is not None
        }
        unresolved = base_payload.get("unresolved_packages", [])
        if isinstance(unresolved, list):
            unresolved_keys = set(item for item in unresolved if isinstance(item, str))

    overlay = payload.get("packages")
    if not isinstance(overlay, list):
        errors.append(f"{prefix}.packages must be a list")
        overlay = []
    if isinstance(basis, dict) and basis.get("overlay_count") != len(overlay):
        errors.append(f"{prefix}.basis.overlay_count mismatch")
    if isinstance(package_count, int) and not isinstance(package_count, bool) and package_count != len(base_by_key):
        errors.append(f"{prefix}.package_count does not match base package set")
    overlay_keys: set[str] = set()
    for index, package in enumerate(overlay):
        label = f"{prefix}.packages[{index}]"
        key = _inventory_key(package)
        if key is None:
            errors.append(f"{label} has an invalid name/version")
            continue
        if key in overlay_keys:
            errors.append(f"{label} duplicates overlay key {key}")
        overlay_keys.add(key)
        if key not in unresolved_keys:
            errors.append(f"{label} is not one of the v1 unresolved rows: {key}")
        base = base_by_key.get(key)
        for field in ("source", "checksum"):
            if not isinstance(package.get(field), str) or package.get(field) != (base or {}).get(field):
                errors.append(f"{label}.{field} does not match v1 base")
        if not isinstance(package.get("license"), str) or not package["license"]:
            errors.append(f"{label}.license is missing")
        _check_spdx_expression(package.get("spdx"), f"{label}.spdx", errors)
        status = package.get("status")
        if not isinstance(status, str) or not status.startswith("resolved"):
            errors.append(f"{label}.status is not resolved")

        crate = package.get("crate")
        if not isinstance(crate, dict):
            errors.append(f"{label}.crate must be an object")
            crate = {}
        crate_path = _check_file_binding(
            root,
            crate.get("path"),
            crate.get("bytes"),
            crate.get("sha256"),
            f"{label}.crate",
            errors,
        )
        if crate.get("inventory_checksum_match") is not True:
            errors.append(f"{label}.crate.inventory_checksum_match must be true")
        if isinstance(crate.get("sha256"), str) and crate.get("sha256") != package.get("checksum"):
            errors.append(f"{label}.crate.sha256 does not equal package checksum")

        registry = package.get("registry")
        if not isinstance(registry, dict):
            errors.append(f"{label}.registry must be an object")
            registry = {}
        registry_kind = registry.get("kind")
        if registry_kind == "crates_io_api":
            api_path = _check_file_binding(
                root,
                registry.get("api_evidence"),
                None,
                registry.get("api_sha256"),
                f"{label}.registry.api_evidence",
                errors,
            )
            if registry.get("api_checksum_match") is not True:
                errors.append(f"{label}.registry.api_checksum_match must be true")
            if api_path is not None and api_path.is_file():
                try:
                    api_payload = _load(api_path)
                    api_version = api_payload.get("version", {})
                    if not isinstance(api_version, dict):
                        raise ValueError("version is not an object")
                    if api_version.get("crate") != package.get("name") or api_version.get("num") != package.get("version"):
                        errors.append(f"{label}.registry API name/version mismatch")
                    if api_version.get("checksum") != package.get("checksum"):
                        errors.append(f"{label}.registry API checksum mismatch")
                    if api_version.get("license") != package.get("license"):
                        errors.append(f"{label}.registry API license mismatch")
                except (OSError, ValueError, json.JSONDecodeError) as exc:
                    errors.append(f"{label}.registry API evidence is invalid: {exc}")
        elif registry_kind == "local_registry_cache":
            if registry.get("api_captured") is not False:
                errors.append(f"{label}.registry.api_captured must be false for local evidence")
            for field in ("api_note",):
                if not isinstance(registry.get(field), str) or not registry[field]:
                    errors.append(f"{label}.registry.{field} is missing")
        else:
            errors.append(f"{label}.registry.kind is unsupported")

        upstream = package.get("upstream")
        if not isinstance(upstream, dict):
            errors.append(f"{label}.upstream must be an object")
            upstream = {}
        for field in ("repository", "commit_url"):
            if not isinstance(upstream.get(field), str) or not upstream[field]:
                errors.append(f"{label}.upstream.{field} is missing")
        if registry_kind != "local_registry_cache" and (
            not isinstance(upstream.get("cargo_toml_url"), str) or not upstream["cargo_toml_url"]
        ):
            errors.append(f"{label}.upstream.cargo_toml_url is missing")
        commit = upstream.get("commit")
        if not isinstance(commit, str) or not COMMIT_RE.fullmatch(commit):
            errors.append(f"{label}.upstream.commit is not a 40-hex pin")
        else:
            if commit not in upstream.get("commit_url", ""):
                errors.append(f"{label}.upstream.commit_url is not bound to commit")
            if registry_kind != "local_registry_cache" and commit not in upstream.get("cargo_toml_url", ""):
                errors.append(f"{label}.upstream.cargo_toml_url is not bound to commit")
        cargo_path = _check_file_binding(
            root,
            upstream.get("cargo_toml_evidence"),
            None,
            upstream.get("cargo_toml_sha256"),
            f"{label}.upstream.cargo_toml_evidence",
            errors,
        )
        if cargo_path is not None and crate_path is not None:
            archive_cargo_sha = upstream.get("archive_cargo_toml_orig_sha256")
            if archive_cargo_sha is not None:
                if not isinstance(archive_cargo_sha, str) or not SHA256_RE.fullmatch(archive_cargo_sha):
                    errors.append(f"{label}.upstream.archive_cargo_toml_orig_sha256 is invalid")
                else:
                    _archive_member_sha256(
                        crate_path,
                        "Cargo.toml.orig",
                        archive_cargo_sha,
                        f"{label}.upstream",
                        errors,
                    )
                    if upstream.get("archive_orig_matches_upstream") is True and archive_cargo_sha != upstream.get("cargo_toml_sha256"):
                        errors.append(f"{label}.upstream archive/source Cargo.toml hashes disagree")
            elif upstream.get("archive_orig_matches_upstream") is True:
                errors.append(f"{label}.upstream archive_orig_matches_upstream lacks archive hash")
        vcs_info = upstream.get("cargo_vcs_info_evidence")
        if vcs_info is not None:
            _inventory_file(root, vcs_info, f"{label}.upstream.cargo_vcs_info_evidence", errors)
        elif registry_kind == "local_registry_cache" and not isinstance(upstream.get("source_basis"), str):
            errors.append(f"{label}.upstream.source_basis is missing for local evidence")

        license_files = package.get("license_files")
        if not isinstance(license_files, list) or not license_files:
            errors.append(f"{label}.license_files must be a non-empty list")
            license_files = []
        for file_index, license_file in enumerate(license_files):
            file_label = f"{label}.license_files[{file_index}]"
            if not isinstance(license_file, dict):
                errors.append(f"{file_label} must be an object")
                continue
            if not isinstance(license_file.get("path"), str) or not license_file["path"]:
                errors.append(f"{file_label}.path is missing")
            evidence_path = _check_file_binding(
                root,
                license_file.get("evidence"),
                None,
                license_file.get("sha256"),
                f"{file_label}.evidence",
                errors,
            )
            archive_sha = license_file.get("archive_sha256")
            packaged = license_file.get("packaged")
            if packaged is not True and packaged is not False:
                errors.append(f"{file_label}.packaged must be boolean")
            if packaged is True:
                if not isinstance(archive_sha, str) or not SHA256_RE.fullmatch(archive_sha):
                    errors.append(f"{file_label}.archive_sha256 is required for packaged license")
                elif crate_path is not None:
                    _archive_member_sha256(crate_path, license_file.get("path", ""), archive_sha, file_label, errors)
            elif packaged is False and archive_sha is not None:
                errors.append(f"{file_label}.archive_sha256 must be null for non-packaged license")
            if evidence_path is None:
                continue
            url = license_file.get("url")
            if url is not None and (not isinstance(url, str) or (isinstance(commit, str) and commit not in url)):
                errors.append(f"{file_label}.url is not bound to upstream commit")

    if overlay_keys != unresolved_keys:
        errors.append(
            f"{prefix}.overlay keys do not exactly resolve v1 unresolved rows: "
            f"missing={sorted(unresolved_keys - overlay_keys)}, extra={sorted(overlay_keys - unresolved_keys)}"
        )

    evidence = payload.get("evidence")
    if not isinstance(evidence, dict):
        errors.append(f"{prefix}.evidence must be an object")
    else:
        for key in ("network_resolution_artifact", "local_resolution_artifact"):
            record = evidence.get(key)
            if not isinstance(record, dict):
                errors.append(f"{prefix}.evidence.{key} must be an object")
                continue
            _check_file_binding(
                root,
                record.get("path"),
                record.get("bytes"),
                record.get("sha256"),
                f"{prefix}.evidence.{key}",
                errors,
            )
        _inventory_file(root, evidence.get("evidence_root"), f"{prefix}.evidence.evidence_root", errors, directory=True)
    notice_files = payload.get("notice_files")
    if not isinstance(notice_files, list) or not notice_files:
        errors.append(f"{prefix}.notice_files must be a non-empty list")
    else:
        for index, notice in enumerate(notice_files):
            _inventory_file(root, notice, f"{prefix}.notice_files[{index}]", errors)


def validate_cargo_license_inventory(
    root: Path,
    inventory_version: str = "v2",
    inventory_path: Path | None = None,
) -> list[str]:
    """Validate the current Cargo license inventory or an explicitly selected history file.

    The default is the promoted v2 overlay.  ``inventory_version='v1'`` is retained
    for audit-history compatibility; tests may pass ``inventory_path`` to exercise
    tamper cases without modifying a checked-in inventory.
    """

    if inventory_version not in {"v1", "v2"}:
        return [f"unsupported cargo license inventory version: {inventory_version}"]
    path = inventory_path or root / (
        CARGO_LICENSE_V1_PATH if inventory_version == "v1" else CARGO_LICENSE_V2_PATH
    )
    errors: list[str] = []
    try:
        payload = _load(path)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        return [f"cannot load cargo license inventory {inventory_version}: {exc}"]
    if inventory_version == "v1":
        _validate_cargo_license_v1(root, payload, errors)
    else:
        _validate_cargo_license_v2(root, payload, errors)
    return errors


def _contains_gt_path(value: Any, fragments: tuple[str, ...]) -> bool:
    if isinstance(value, str):
        folded = value.casefold()
        return any(fragment.casefold() in folded for fragment in fragments)
    if isinstance(value, list):
        return any(_contains_gt_path(item, fragments) for item in value)
    if isinstance(value, dict):
        return any(_contains_gt_path(item, fragments) for item in value.values())
    return False


def _validate_release_schema(
    root: Path, release_path: Path, release: dict[str, Any], errors: list[str]
) -> None:
    """Validate the dedicated release-inventory contract before its contents."""

    schema_ref = release.get("schema_path")
    expected_schema_ref = "benchmarks/basalt/schemas/release_manifest_v1.schema.json"
    if schema_ref != expected_schema_ref:
        errors.append("release manifest schema_path mismatch")
        return
    schema_path = root / schema_ref
    if not schema_path.is_file():
        errors.append(f"release manifest schema is missing: {schema_ref}")
        return
    try:
        schema = _load(schema_path)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        errors.append(f"cannot load release manifest schema: {exc}")
        return
    if schema.get("$id") != "https://visloc-rs.local/schemas/basalt/release-manifest-v1.schema.json":
        errors.append("release manifest schema $id mismatch")
    required = schema.get("required")
    if not isinstance(required, list):
        errors.append("release manifest schema required must be a list")
    else:
        missing = sorted(set(required) - set(release))
        if missing:
            errors.append(f"release manifest is missing schema-required fields: {missing}")
    artifact_schema = schema.get("$defs", {}).get("artifact", {})
    artifact_required = artifact_schema.get("required") if isinstance(artifact_schema, dict) else None
    if not isinstance(artifact_required, list) or set(artifact_required) != {
        "path",
        "kind",
        "bytes",
        "sha256",
    }:
        errors.append("release manifest schema artifact definition must require path/kind/bytes/sha256")

    if release.get("schema_version") != 1:
        errors.append("release manifest schema_version must be 1")
    if release.get("schema_id") != RELEASE_SCHEMA_ID:
        errors.append("release manifest schema_id mismatch")
    if release.get("manifest_id") != "basalt-release-v1":
        errors.append("release manifest manifest_id mismatch")
    self_binding = release.get("self_binding")
    try:
        release_rel = release_path.relative_to(root).as_posix()
    except ValueError:
        release_rel = ""
    if not isinstance(self_binding, dict):
        errors.append("release manifest self_binding must be an object")
    else:
        if self_binding.get("path") != release_rel:
            errors.append("release manifest self_binding.path must name the release manifest")
        if self_binding.get("mode") != "external_sha256":
            errors.append("release manifest self_binding.mode must be external_sha256")
        if not isinstance(self_binding.get("note"), str) or not self_binding["note"]:
            errors.append("release manifest self_binding.note is missing")

    contract_inputs = release.get("contract_inputs")
    required_contracts = {
        "port_manifest",
        "upstream_manifest",
        "protocol",
        "config",
        "provenance_manifest",
    }
    contract_paths: set[str] = set()
    if not isinstance(contract_inputs, dict):
        errors.append("release manifest contract_inputs must be an object")
    else:
        missing_contracts = sorted(required_contracts - set(contract_inputs))
        if missing_contracts:
            errors.append(f"release manifest contract_inputs missing: {missing_contracts}")
        for name in required_contracts:
            value = contract_inputs.get(name)
            if not isinstance(value, str) or not _is_path_like(value):
                errors.append(f"release manifest contract_inputs.{name} is invalid")
            else:
                contract_paths.add(value)
        if contract_inputs.get("provenance_manifest") != release.get("provenance_manifest"):
            errors.append("release manifest contract_inputs.provenance_manifest disagrees")

    artifacts = release.get("artifacts")
    artifact_paths: list[str] = []
    if not isinstance(artifacts, list) or not artifacts:
        return
    for artifact in artifacts:
        if not isinstance(artifact, dict):
            errors.append("release manifest artifact must be an object")
            continue
        path = artifact.get("path")
        if not isinstance(path, str) or not _is_path_like(path):
            errors.append("release manifest artifact path is invalid")
            continue
        artifact_paths.append(path)
        if not isinstance(artifact.get("kind"), str) or not artifact["kind"]:
            errors.append(f"release manifest artifact kind is missing: {path}")
        size = artifact.get("bytes")
        if isinstance(size, bool) or not isinstance(size, int) or size < 0:
            errors.append(f"release manifest artifact bytes is invalid: {path}")
        digest = artifact.get("sha256")
        if not isinstance(digest, str) or not re.fullmatch(r"[0-9a-f]{64}", digest):
            errors.append(f"release manifest artifact sha256 is invalid: {path}")
        artifact_file = root / Path(path)
        if not artifact_file.is_file():
            errors.append(f"release manifest artifact must be a file: {path}")
            continue
        if isinstance(size, int) and not isinstance(size, bool) and size != artifact_file.stat().st_size:
            errors.append(f"release manifest artifact byte mismatch: {path}")
        if isinstance(digest, str) and re.fullmatch(r"[0-9a-f]{64}", digest):
            if _sha256(artifact_file) != digest:
                errors.append(f"release manifest artifact hash mismatch: {path}")
    if len(artifact_paths) != len(set(artifact_paths)):
        errors.append("release manifest artifact paths must be unique")
    if release_rel and release_rel in artifact_paths:
        errors.append("release manifest must use self_binding instead of listing itself as an artifact")
    for required_path in sorted(contract_paths):
        if required_path not in artifact_paths:
            errors.append(f"release manifest contract input is not artifact-bound: {required_path}")


def _validate_release(root: Path, manifest: dict[str, Any], errors: list[str]) -> None:
    release_path = root / manifest.get("release", {}).get(
        "manifest_path", "benchmarks/basalt/basalt_release_manifest_v1.json"
    )
    if not release_path.is_file():
        errors.append(f"release manifest is missing: {release_path}")
        return
    release = _load(release_path)
    _validate_release_schema(root, release_path, release, errors)
    if release.get("schema_id") != RELEASE_SCHEMA_ID:
        errors.append("release manifest schema_id mismatch")
    if release.get("upstream_commit") != manifest["upstreams"]["basalt"]["commit"]:
        errors.append("release manifest upstream commit mismatch")
    if release.get("ground_truth_artifacts") != []:
        errors.append("release manifest must have ground_truth_artifacts=[]")
    firewall = release.get("ground_truth_firewall", {})
    if firewall.get("release_artifacts_contain_ground_truth") is not False:
        errors.append("release GT firewall must say release_artifacts_contain_ground_truth=false")
    if firewall.get("ground_truth_is_post_run") is not True:
        errors.append("release GT firewall must say ground_truth_is_post_run=true")
    if not isinstance(firewall.get("forbidden_artifact_path_fragments"), list) or not firewall[
        "forbidden_artifact_path_fragments"
    ]:
        errors.append("release GT firewall must define forbidden artifact path fragments")
    artifacts = release.get("artifacts", [])
    fragments = tuple(firewall.get("forbidden_artifact_path_fragments", GT_FRAGMENTS))
    if not isinstance(artifacts, list) or not artifacts:
        errors.append("release manifest artifacts must be non-empty")
    elif _contains_gt_path(artifacts, fragments):
        errors.append("release manifest artifact paths contain a forbidden GT/target fragment")
    artifact_paths = {
        artifact.get("path")
        for artifact in artifacts
        if isinstance(artifact, dict) and isinstance(artifact.get("path"), str)
    }
    for artifact in artifacts if isinstance(artifacts, list) else []:
        if not isinstance(artifact, dict) or not isinstance(artifact.get("path"), str):
            errors.append("release manifest has an artifact without a path")
            continue
        artifact_path = root / artifact["path"]
        if not artifact_path.exists():
            errors.append(f"release artifact is missing: {artifact['path']}")
        if artifact["path"] not in artifact_paths:
            errors.append(f"release artifact path indexing failed: {artifact['path']}")
    expected_notice_files = manifest.get("release", {}).get("notice_files", [])
    for notice in expected_notice_files:
        if not (root / notice).is_file():
            errors.append(f"notice file is missing: {notice}")
        if notice not in artifact_paths:
            errors.append(f"notice file is not artifact-bound: {notice}")


def validate(
    root: Path,
    checkouts: dict[str, Path] | None = None,
    cargo_inventory_version: str = "v2",
) -> list[str]:
    """Return validation errors; an empty list means the audit passes."""

    checkouts = checkouts or {}
    path = root / "benchmarks/basalt/basalt_provenance_manifest_v1.json"
    errors: list[str] = []
    try:
        manifest = _load(path)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        return [f"cannot load provenance manifest: {exc}"]
    if manifest.get("schema_id") != SCHEMA_ID:
        errors.append("provenance manifest schema_id mismatch")
    if manifest.get("schema_version") != 1:
        errors.append("provenance manifest schema_version must be 1")
    _validate_upstreams(root, manifest, errors, checkouts)
    _validate_port_files(root, manifest, errors)
    _validate_generator_hashes(root, manifest, errors)
    _validate_retained_audit_sources(root, manifest, errors)
    _validate_release(root, manifest, errors)
    errors.extend(validate_cargo_license_inventory(root, cargo_inventory_version))
    return errors


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument(
        "--checkout",
        action="append",
        default=[],
        metavar="NAME=PATH",
        help="verify a materialized upstream checkout (repeatable)",
    )
    parser.add_argument(
        "--cargo-license-inventory",
        choices=("v2", "v1"),
        default="v2",
        help="select the current v2 Cargo license inventory (or immutable v1 history)",
    )
    args = parser.parse_args(argv)
    checkouts: dict[str, Path] = {}
    for item in args.checkout:
        if "=" not in item:
            parser.error("--checkout must be NAME=PATH")
        name, raw_path = item.split("=", 1)
        checkouts[name] = Path(raw_path).resolve()
    errors = validate(args.root.resolve(), checkouts, args.cargo_license_inventory)
    if errors:
        for error in errors:
            print(f"FAIL {error}", file=sys.stderr)
        return 1
    print("Basalt provenance validation passed.")
    if checkouts:
        print("Checked out upstream commits/trees and source paths:", ", ".join(sorted(checkouts)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
