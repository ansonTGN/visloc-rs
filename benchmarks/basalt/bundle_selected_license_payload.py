"""Stage the selected-profile Cargo license texts into a fresh candidate root.

The input is the locked/offline selected-graph report produced by
``generate_selected_license_report.py``.  This bundler copies only the 56
selected registry package license sets, the 10 local-package root license
texts, and the two checked-in Basalt attribution notices.  Before any staging,
it also requires the independent supplemental non-Cargo source verifier to
prove the manually translated AOR source and persisted license payload.  It
never downloads or edits a source, inventory, cache, or existing destination;
the current candidate therefore remains pending/non-release and stages no
payload.

The output is a candidate payload manifest, not legal clearance and not a
complete native/runtime SBOM.  Registry source-cache files are explicitly
labelled as cache evidence.  Cargo.lock identity is required; optional
``.cargo-checksum.json`` and ``.crate`` evidence is verified when present and
reported as unavailable when the local cache does not contain it.
"""

from __future__ import annotations

import argparse
import datetime as _datetime
import hashlib
import json
import os
import posixpath
import re
import shutil
import tarfile
from pathlib import Path
from typing import Any, Iterable, Mapping

try:
    from verify_supplemental_source_dependencies import (
        DEFAULT_MANIFEST as DEFAULT_SUPPLEMENTAL_SOURCE_MANIFEST,
        PASS_STATUS as SUPPLEMENTAL_PASS_STATUS,
        validate_manifest as validate_supplemental_manifest,
    )
except ImportError:  # Importable both as a script and as benchmarks.basalt.*.
    from benchmarks.basalt.verify_supplemental_source_dependencies import (
        DEFAULT_MANIFEST as DEFAULT_SUPPLEMENTAL_SOURCE_MANIFEST,
        PASS_STATUS as SUPPLEMENTAL_PASS_STATUS,
        validate_manifest as validate_supplemental_manifest,
    )


SCHEMA = "visloc.basalt.selected_license_payload.v1"
DEFAULT_REPORT = "work/m11_selected_license_graph_20260907.json"
DEFAULT_OUTPUT_NAME = "candidate_license_payload_manifest"
UPSTREAM_COMMIT = "0f3b2b52c807f70ff4e2973ce253c73329eea7bc"
NOTICE_PATHS = (
    "benchmarks/basalt/NOTICE",
    "pipelines/basalt/NOTICE",
)
EXPECTED_TARGET = "x86_64-pc-windows-msvc"
EXPECTED_EXAMPLE = "basalt_euroc_vio_demo"
EXPECTED_SELECTED = 66
EXPECTED_REGISTRY = 56
EXPECTED_LOCAL = 10


class PayloadError(RuntimeError):
    """Raised for an ambiguous, missing, or mismatched payload input."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _display_path(path: Path) -> str:
    return path.resolve().as_posix()


def _is_under(path: Path, parent: Path) -> bool:
    try:
        path.resolve().relative_to(parent.resolve())
    except ValueError:
        return False
    return True


def _load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise PayloadError(f"cannot load JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise PayloadError(f"expected a JSON object: {path}")
    return value


def _toml_string(text: str, key: str) -> str | None:
    match = re.search(rf'(?m)^{re.escape(key)}\s*=\s*"((?:\\.|[^"])*)"\s*$', text)
    if match is None:
        return None
    raw = match.group(1)
    try:
        return str(json.loads('"' + raw + '"'))
    except json.JSONDecodeError:
        return raw.replace('\\"', '"').replace('\\\\', '\\')


def _package_manifest_identity(path: Path, workspace_version: str | None = None) -> tuple[str, str]:
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise PayloadError(f"cannot read package manifest {path}: {exc}") from exc
    try:
        import tomllib  # type: ignore[import-not-found]
    except ModuleNotFoundError:
        tomllib = None  # type: ignore[assignment]
    if tomllib is not None:
        try:
            parsed = tomllib.loads(text)
            package = parsed.get("package")
            if isinstance(package, dict) and isinstance(package.get("name"), str):
                if isinstance(package.get("version"), str):
                    return str(package["name"]), str(package["version"])
                if workspace_version and (
                    package.get("version.workspace") is True
                    or isinstance(package.get("version"), dict)
                ):
                    return str(package["name"]), workspace_version
        except Exception as exc:  # pragma: no cover - fallback handles old Python/odd manifests
            raise PayloadError(f"cannot parse package manifest {path}: {exc}") from exc
    section_match = re.search(r"(?ms)^\[package\]\r?\n(.*?)(?=^\[|\Z)", text)
    if section_match is None:
        raise PayloadError(f"package table missing from {path}")
    section = section_match.group(1)
    name = _toml_string(section, "name")
    version = _toml_string(section, "version")
    if not version and workspace_version and re.search(r"(?m)^version\.workspace\s*=\s*true\s*$", section):
        version = workspace_version
    if not name or not version:
        raise PayloadError(f"literal package name/version missing from {path}")
    return name, version


def _lock_packages(path: Path) -> dict[tuple[str, str, str], dict[str, Any]]:
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise PayloadError(f"cannot read Cargo.lock {path}: {exc}") from exc
    parts = re.split(r"(?m)^\[\[package\]\]\r?\n", text)[1:]
    if not parts:
        raise PayloadError(f"Cargo.lock has no package entries: {path}")
    packages: dict[tuple[str, str, str], dict[str, Any]] = {}
    for part in parts:
        name = _toml_string(part, "name")
        version = _toml_string(part, "version")
        if not name or not version:
            continue
        source = _toml_string(part, "source")
        checksum = _toml_string(part, "checksum")
        key = (name, version, source or "")
        if key in packages:
            raise PayloadError(f"ambiguous duplicate Cargo.lock package identity: {key}")
        packages[key] = {
            "name": name,
            "version": version,
            "source": source,
            "checksum": checksum,
        }
    if not packages:
        raise PayloadError(f"Cargo.lock package parsing produced no records: {path}")
    return packages


def _identity(package: Mapping[str, Any]) -> tuple[str, str, str]:
    source = package.get("source")
    return (str(package.get("name", "")), str(package.get("version", "")), "" if source is None else str(source))


def _safe_component(value: str) -> str:
    result = re.sub(r"[^A-Za-z0-9._-]+", "_", value)
    if result in {"", ".", ".."}:
        raise PayloadError(f"unsafe empty destination component: {value!r}")
    return result


def _safe_relative(value: str) -> str:
    parts = [part for part in value.replace("\\", "/").split("/") if part not in {"", "."}]
    if not parts or any(part == ".." for part in parts):
        raise PayloadError(f"unsafe relative payload path: {value!r}")
    return "/".join(_safe_component(part) for part in parts)


def _default_registry_roots() -> list[Path]:
    cargo_home = Path(os.environ.get("CARGO_HOME", Path.home() / ".cargo"))
    source_root = cargo_home / "registry" / "src"
    if not source_root.is_dir():
        return []
    return sorted((child for child in source_root.iterdir() if child.is_dir()), key=lambda item: item.name.casefold())


def _registry_root_for(source_dir: Path, configured: Iterable[Path]) -> Path:
    source_dir = source_dir.resolve()
    source_root = source_dir.parent
    configured_roots = [item.resolve() for item in configured]
    if configured_roots and source_root not in configured_roots:
        raise PayloadError(f"registry source directory is outside configured cache roots: {source_dir}")
    if source_root.name.startswith("index.") and source_root.parent.name == "src" and source_root.parent.parent.name == "registry":
        return source_root
    raise PayloadError(f"registry source directory does not have a Cargo registry shape: {source_dir}")


def _registry_archive_candidates(source_dir: Path, registry_root: Path, name: str, version: str) -> list[Path]:
    archive_name = f"{name}-{version}.crate"
    cache_parent = registry_root.parent / "cache"
    candidates: list[Path] = []
    if not cache_parent.is_dir():
        return candidates
    for cache_root in sorted((item for item in cache_parent.iterdir() if item.is_dir()), key=lambda item: item.name.casefold()):
        candidate = cache_root / archive_name
        if candidate.is_file():
            candidates.append(candidate.resolve())
    return candidates


def _verify_file_source(path: Path, expected_bytes: Any, expected_sha256: Any, *, label: str) -> dict[str, Any]:
    path = path.resolve()
    if path.is_symlink() or not path.is_file():
        raise PayloadError(f"missing or symlinked {label}: {path}")
    if not isinstance(expected_sha256, str) or len(expected_sha256) != 64:
        raise PayloadError(f"missing expected SHA-256 for {label}: {path}")
    actual_bytes = path.stat().st_size
    actual_sha256 = sha256_file(path)
    if expected_bytes is not None and int(expected_bytes) != actual_bytes:
        raise PayloadError(f"byte mismatch for {label}: {path} ({actual_bytes} != {expected_bytes})")
    if actual_sha256.lower() != expected_sha256.lower():
        raise PayloadError(f"SHA-256 mismatch for {label}: {path}")
    return {
        "source_path": _display_path(path),
        "source_bytes": actual_bytes,
        "source_sha256": actual_sha256,
        "source_hash_verified_against_report": True,
    }


def _verify_checksum_sidecar(path: Path, expected_checksum: str, license_files: list[dict[str, Any]], source_dir: Path) -> dict[str, Any]:
    if not path.is_file():
        return {"path": _display_path(path), "status": "unavailable"}
    data = _load_json(path)
    package_checksum = data.get("package")
    if package_checksum != expected_checksum:
        raise PayloadError(f".cargo-checksum.json package checksum mismatch: {path}")
    file_hashes = data.get("files")
    if not isinstance(file_hashes, dict):
        raise PayloadError(f".cargo-checksum.json files map missing: {path}")
    for license_file in license_files:
        relative = Path(license_file["source_path"]).resolve().relative_to(source_dir.resolve()).as_posix()
        expected = file_hashes.get(relative)
        if not isinstance(expected, str) or expected.lower() != license_file["source_sha256"].lower():
            raise PayloadError(f"license file checksum missing/mismatched in {path}: {relative}")
    return {
        "path": _display_path(path),
        "status": "verified",
        "package_checksum": package_checksum,
        "license_file_count_verified": len(license_files),
    }


def _safe_archive_member_name(name: str) -> str:
    normalized = posixpath.normpath(name.replace("\\", "/"))
    if not normalized or normalized == "." or normalized.startswith("/") or any(part == ".." for part in normalized.split("/")):
        raise PayloadError(f"unsafe .crate archive member path: {name!r}")
    return normalized


def _verify_archive(
    candidates: list[Path],
    expected_checksum: str,
    license_files: list[dict[str, Any]] | None = None,
    source_dir: Path | None = None,
) -> dict[str, Any]:
    if len(candidates) > 1:
        raise PayloadError("ambiguous .crate archive evidence: " + ", ".join(_display_path(item) for item in candidates))
    if not candidates:
        return {"status": "unavailable", "candidate_count": 0}
    archive = candidates[0]
    actual = sha256_file(archive)
    if actual.lower() != expected_checksum.lower():
        raise PayloadError(f".crate archive checksum mismatch: {archive}")
    if not license_files or source_dir is None:
        return {
            "status": "verified_checksum_only_not_bound_to_license_text",
            "candidate_count": 1,
            "path": _display_path(archive),
            "bytes": archive.stat().st_size,
            "sha256": actual,
        }
    source_dir = source_dir.resolve()
    expected_members: dict[str, dict[str, Any]] = {}
    for license_file in license_files:
        relative = str(license_file.get("relative_path", "")).replace("\\", "/")
        member_name = _safe_archive_member_name(f"{source_dir.name}/{relative}")
        if member_name in expected_members:
            raise PayloadError(f"duplicate expected archive license member: {member_name}")
        expected_members[member_name] = license_file
    try:
        with tarfile.open(archive, mode="r:*") as tar:
            members_by_name: dict[str, list[tarfile.TarInfo]] = {}
            for member in tar.getmembers():
                safe_name = _safe_archive_member_name(member.name)
                members_by_name.setdefault(safe_name, []).append(member)
            for member_name, license_file in expected_members.items():
                matches = members_by_name.get(member_name, [])
                if len(matches) != 1:
                    raise PayloadError(f"archive license member missing/duplicate: {member_name}")
                member = matches[0]
                if not member.isfile() or member.issym() or member.islnk():
                    raise PayloadError(f"archive license member is not a regular file: {member_name}")
                stream = tar.extractfile(member)
                if stream is None:
                    raise PayloadError(f"cannot read archive license member: {member_name}")
                data = stream.read()
                member_sha256 = hashlib.sha256(data).hexdigest()
                if len(data) != int(license_file["source_bytes"]) or member_sha256.lower() != str(license_file["source_sha256"]).lower():
                    raise PayloadError(f"archive/cache license content mismatch: {member_name}")
    except tarfile.TarError as exc:
        raise PayloadError(f"cannot inspect .crate archive {archive}: {exc}") from exc
    return {
        "status": "verified_license_members",
        "candidate_count": 1,
        "path": _display_path(archive),
        "bytes": archive.stat().st_size,
        "sha256": actual,
        "license_member_count_verified": len(expected_members),
    }


def _registry_license_sources(
    row: Mapping[str, Any], root: Path, source_dir: Path
) -> tuple[list[tuple[Path, dict[str, Any], str]], str]:
    license_info = row.get("license")
    if not isinstance(license_info, dict):
        raise PayloadError(f"license record missing for {row.get('name')} {row.get('version')}")
    v2_files = license_info.get("v2_license_files")
    cache_files = license_info.get("registry_cache_license_files")
    if isinstance(v2_files, list) and v2_files:
        result: list[tuple[Path, dict[str, Any], str]] = []
        for item in v2_files:
            if not isinstance(item, dict) or not item.get("evidence"):
                raise PayloadError(f"ambiguous v2 license evidence for {row.get('name')} {row.get('version')}")
            result.append((root / str(item["evidence"]), item, "v2_inventory_evidence"))
        return result, "v2_inventory_evidence"
    if not isinstance(cache_files, list) or not cache_files:
        raise PayloadError(f"missing license text evidence for {row.get('name')} {row.get('version')}")
    result = []
    for item in cache_files:
        if not isinstance(item, dict) or not item.get("path"):
            raise PayloadError(f"ambiguous cache license evidence for {row.get('name')} {row.get('version')}")
        source = Path(str(item["path"]))
        if not _is_under(source, source_dir):
            raise PayloadError(f"registry license path escapes source package: {source}")
        result.append((source, item, "registry_cache_evidence"))
    return result, "registry_cache_evidence"


def _new_file_spec(
    *,
    source: Path,
    destination: str,
    source_package: str,
    source_version: str,
    spdx_expression: str,
    role: str,
    package_identity: str | None,
    expected_bytes: Any = None,
    expected_sha256: Any = None,
    source_kind: str,
) -> dict[str, Any]:
    measured = _verify_file_source(source, expected_bytes, expected_sha256, label=role)
    return {
        **measured,
        "destination_path": destination,
        "source_package": source_package,
        "source_version_or_commit": source_version,
        "spdx_expression": spdx_expression,
        "role": role,
        "source_kind": source_kind,
        "applies_to_packages": [package_identity] if package_identity else [],
        "spdx_expressions_by_package": {package_identity: spdx_expression} if package_identity else {},
    }


def _add_file_spec(file_map: dict[str, dict[str, Any]], spec: dict[str, Any]) -> dict[str, Any]:
    key = os.path.normcase(str(Path(spec["source_path"]).resolve()))
    existing = file_map.get(key)
    if existing is None:
        file_map[key] = spec
        return spec
    if existing["source_sha256"] != spec["source_sha256"] or existing["source_bytes"] != spec["source_bytes"]:
        raise PayloadError(f"same source file has conflicting measurements: {spec['source_path']}")
    for identity in spec.get("applies_to_packages", []):
        if identity not in existing["applies_to_packages"]:
            existing["applies_to_packages"].append(identity)
    existing_map = existing.setdefault("spdx_expressions_by_package", {})
    existing_map.update(spec.get("spdx_expressions_by_package", {}))
    return existing


def _validate_selected_report(report: Mapping[str, Any]) -> None:
    scope = report.get("scope")
    recon = report.get("reconciliation")
    if report.get("status") != "pass_selected_graph_reconciled":
        raise PayloadError(f"selected report is not a passing graph reconciliation: {report.get('status')}")
    if not isinstance(scope, dict) or scope.get("target") != EXPECTED_TARGET or not scope.get("no_default_features"):
        raise PayloadError("selected report profile is not x86_64-pc-windows-msvc/no-default-features")
    if scope.get("example") != EXPECTED_EXAMPLE:
        raise PayloadError(f"selected report example is not {EXPECTED_EXAMPLE}: {scope.get('example')}")
    if not isinstance(recon, dict):
        raise PayloadError("selected report reconciliation section is missing")
    supplemental = report.get("supplemental_source_dependencies")
    if not isinstance(supplemental, Mapping):
        raise PayloadError("selected report is missing supplemental non-Cargo source coverage")
    if supplemental.get("status") != SUPPLEMENTAL_PASS_STATUS or supplemental.get("release_ready") is not True:
        raise PayloadError(
            "selected report supplemental non-Cargo source coverage is not release-ready: "
            f"{supplemental.get('status')}"
        )
    if supplemental.get("manual_source_dependency_count") != 1:
        raise PayloadError("selected report supplemental source dependency count is not exactly one")
    if supplemental.get("known_source_hashes_verified_count") != supplemental.get("known_source_file_count"):
        raise PayloadError("selected report supplemental upstream source hashes are incomplete")
    if supplemental.get("known_license_hash_verified") is not True:
        raise PayloadError("selected report supplemental license hash is not verified")
    if supplemental.get("affected_production_file_hashes_verified_count") != supplemental.get("affected_production_file_count"):
        raise PayloadError("selected report supplemental affected production-file hashes are incomplete")
    if supplemental.get("payload_persisted") is not True:
        raise PayloadError("selected report supplemental license payload is not persisted")
    expected = {
        "selected_package_count": EXPECTED_SELECTED,
        "selected_registry_package_count": EXPECTED_REGISTRY,
        "selected_local_package_count": EXPECTED_LOCAL,
        "selected_inventory_missing_count": 0,
        "license_text_missing_from_known_local_cache_count": 0,
    }
    for key, value in expected.items():
        if recon.get(key) != value:
            raise PayloadError(f"selected report {key} is {recon.get(key)!r}, expected {value!r}")
    rows = report.get("selected_packages")
    if not isinstance(rows, list) or len(rows) != EXPECTED_SELECTED:
        raise PayloadError("selected report package list is missing or has the wrong count")
    identities = [_identity(row) for row in rows if isinstance(row, dict)]
    if len(identities) != EXPECTED_SELECTED or len(set(identities)) != EXPECTED_SELECTED:
        raise PayloadError("selected report package identities are missing or ambiguous")


def _verify_registry_package(
    row: Mapping[str, Any],
    lock_packages: Mapping[tuple[str, str, str], Mapping[str, Any]],
    configured_registry_roots: list[Path],
    root: Path,
    file_map: dict[str, dict[str, Any]],
) -> tuple[dict[str, Any], list[str]]:
    name = str(row.get("name", ""))
    version = str(row.get("version", ""))
    source = row.get("source")
    key = _identity(row)
    lock = lock_packages.get(key)
    if lock is None or not lock.get("checksum"):
        raise PayloadError(f"Cargo.lock registry identity/checksum missing: {key}")
    source_dir_raw = row.get("license", {}).get("registry_cache_source_dir") if isinstance(row.get("license"), dict) else None
    if not source_dir_raw:
        raise PayloadError(f"registry source cache directory missing: {key}")
    source_dir = Path(str(source_dir_raw)).resolve()
    if source_dir.name != f"{name}-{version}" or not source_dir.is_dir():
        raise PayloadError(f"registry source directory missing/ambiguous for {key}: {source_dir}")
    registry_root = _registry_root_for(source_dir, configured_registry_roots)
    manifest_path = source_dir / "Cargo.toml"
    manifest_identity = _package_manifest_identity(manifest_path)
    if manifest_identity != (name, version):
        raise PayloadError(f"registry Cargo.toml identity mismatch for {key}: {manifest_identity}")
    expression = row.get("license", {}).get("license_expression") if isinstance(row.get("license"), dict) else None
    if not isinstance(expression, str) or not expression or expression == "UNRESOLVED":
        raise PayloadError(f"selected registry license expression is unresolved: {key}")
    sources, source_kind = _registry_license_sources(row, root, source_dir)
    if len({str(item[0].resolve()) for item in sources}) != len(sources):
        raise PayloadError(f"duplicate/ambiguous license paths for {key}")
    license_records: list[dict[str, Any]] = []
    destinations: list[str] = []
    measured_license_files: list[dict[str, Any]] = []
    for source_path, expected, evidence_kind in sources:
        relative = source_path.resolve().relative_to(source_dir).as_posix() if evidence_kind == "registry_cache_evidence" else Path(str(expected.get("path", source_path.name))).as_posix()
        destination = "licenses/registry/" + _safe_component(f"{name}-{version}") + "/" + _safe_relative(relative)
        spec = _new_file_spec(
            source=source_path,
            destination=destination,
            source_package=name,
            source_version=version,
            spdx_expression=expression,
            role="registry_license_text",
            package_identity=f"{name} {version}",
            expected_bytes=expected.get("bytes"),
            expected_sha256=expected.get("sha256"),
            source_kind=source_kind,
        )
        stored = _add_file_spec(file_map, spec)
        license_records.append({"destination_path": stored["destination_path"], "source_path": stored["source_path"]})
        destinations.append(stored["destination_path"])
        if evidence_kind == "registry_cache_evidence":
            measured_license_files.append(
                {
                    "source_path": stored["source_path"],
                    "source_sha256": stored["source_sha256"],
                    "source_bytes": stored["source_bytes"],
                    "relative_path": relative,
                }
            )
    checksum: dict[str, Any] = {
        "path": _display_path(source_dir / ".cargo-checksum.json"),
        "status": "not_applicable_v2_evidence" if source_kind != "registry_cache_evidence" else "unavailable",
    }
    if source_kind == "registry_cache_evidence":
        checksum = _verify_checksum_sidecar(source_dir / ".cargo-checksum.json", str(lock["checksum"]), measured_license_files, source_dir)
    archive = _verify_archive(
        _registry_archive_candidates(source_dir, registry_root, name, version),
        str(lock["checksum"]),
        measured_license_files if source_kind == "registry_cache_evidence" else None,
        source_dir if source_kind == "registry_cache_evidence" else None,
    )
    package_record = {
        "name": name,
        "version": version,
        "kind": "registry",
        "source": source,
        "spdx_expression": expression,
        "inventory_basis": row.get("license", {}).get("inventory_basis"),
        "inventory_status": row.get("license", {}).get("inventory_status"),
        "manifest_path": _display_path(manifest_path),
        "cargo_lock": {**dict(lock), "identity_verified": True},
        "cargo_checksum_sidecar": checksum,
        "crate_archive": archive,
        "license_files": license_records,
        "source_evidence_kind": source_kind,
    }
    return package_record, destinations


def _workspace_file_specs(
    row: Mapping[str, Any], root: Path, file_map: dict[str, dict[str, Any]], local_identities: list[str]
) -> list[str]:
    info = row.get("license")
    if not isinstance(info, dict):
        raise PayloadError(f"local package license record missing: {row.get('name')} {row.get('version')}")
    expression = info.get("license_expression")
    if not isinstance(expression, str) or not expression or expression == "UNRESOLVED":
        raise PayloadError(f"local package license expression is unresolved: {row.get('name')} {row.get('version')}")
    records = info.get("workspace_license_files")
    if not isinstance(records, list) or not records:
        raise PayloadError(f"local package license text list is missing: {row.get('name')} {row.get('version')}")
    destinations: list[str] = []
    identity = f"{row.get('name')} {row.get('version')}"
    for expected in records:
        if not isinstance(expected, dict) or not expected.get("path"):
            raise PayloadError(f"ambiguous workspace license record: {identity}")
        relative = _safe_relative(str(expected["path"]))
        source = (root / relative).resolve()
        if not _is_under(source, root):
            raise PayloadError(f"workspace license path escapes root: {source}")
        spec = _new_file_spec(
            source=source,
            destination="licenses/workspace/" + relative,
            source_package="workspace-root",
            source_version="0.1.0",
            spdx_expression=expression,
            role="workspace_license_text",
            package_identity=identity,
            expected_bytes=expected.get("bytes"),
            expected_sha256=expected.get("sha256"),
            source_kind="workspace_root",
        )
        stored = _add_file_spec(file_map, spec)
        destinations.append(stored["destination_path"])
    local_identities.append(identity)
    return destinations


def _attribution_specs(root: Path, file_map: dict[str, dict[str, Any]]) -> list[str]:
    destinations: list[str] = []
    for relative in NOTICE_PATHS:
        source = (root / relative).resolve()
        if not source.is_file() or source.is_symlink():
            raise PayloadError(f"required Basalt attribution notice missing: {source}")
        text = source.read_text(encoding="utf-8", errors="replace")
        if UPSTREAM_COMMIT not in text or "BSD-3-Clause" not in text:
            raise PayloadError(f"Basalt attribution/commit check failed: {source}")
        spec = _new_file_spec(
            source=source,
            destination="licenses/basalt/" + _safe_relative(relative),
            source_package="basalt-upstream",
            source_version=UPSTREAM_COMMIT,
            spdx_expression="BSD-3-Clause",
            role="upstream_attribution_notice",
            package_identity=None,
            expected_bytes=None,
            expected_sha256=sha256_file(source),
            source_kind="checked_in_notice",
        )
        stored = _add_file_spec(file_map, spec)
        destinations.append(stored["destination_path"])
    return destinations


def _copy_specs(output_root: Path, specs: Iterable[dict[str, Any]]) -> None:
    for spec in sorted(specs, key=lambda item: item["destination_path"]):
        destination = output_root / Path(spec["destination_path"])
        if destination.exists() or destination.is_symlink():
            raise PayloadError(f"destination unexpectedly exists: {destination}")
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(Path(spec["source_path"]), destination)
        if destination.stat().st_size != spec["source_bytes"] or sha256_file(destination).lower() != spec["source_sha256"].lower():
            raise PayloadError(f"source/destination hash mismatch after copy: {destination}")
        spec["destination_path_absolute"] = _display_path(destination)
        spec["destination_bytes"] = destination.stat().st_size
        spec["destination_sha256"] = sha256_file(destination)
        spec["source_destination_hash_match"] = True


def _render_markdown(manifest: Mapping[str, Any]) -> str:
    verification = manifest["verification"]
    supplemental = manifest.get("supplemental_source_dependencies", {})
    if not isinstance(supplemental, Mapping):
        supplemental = {}
    supplemental_coverage = manifest.get("supplemental_source_coverage", {})
    if not isinstance(supplemental_coverage, Mapping):
        supplemental_coverage = {}
    lines = [
        "# Candidate selected-profile license payload",
        "",
        f"Status: **{manifest['status']}**  ",
        f"Captured: `{manifest['captured_at']}`  ",
        f"Schema: `{manifest['schema']}`",
        "",
        "This is a candidate selected-profile license payload, not legal clearance, a final release archive, or a native SDK/CRT SBOM.",
        "",
        "## Scope and verification",
        "",
        f"- Selected profile: `{manifest['scope']['target']}`, `{manifest['scope']['example']}`, `--no-default-features`.",
        f"- Packages: **{verification['package_count']}** ({verification['registry_package_count']} registry + {verification['local_package_count']} local).",
        f"- Unique payload files: **{verification['payload_file_count']}**; source/destination hash mismatches: **{verification['source_destination_hash_mismatch_count']}**.",
        f"- Cargo.lock identities verified: **{verification['cargo_lock_identity_verified_count']}**; .cargo-checksum verified: **{verification['cargo_checksum_verified_count']}**; .crate verified: **{verification['crate_archive_verified_count']}**.",
        f"- Optional archive evidence unavailable: **{verification['crate_archive_unavailable_count']}**; optional .cargo-checksum evidence unavailable: **{verification['cargo_checksum_unavailable_count']}**.",
        "",
        "## Supplemental non-Cargo source coverage",
        "",
        f"- Manual source dependencies: **{supplemental.get('manual_source_dependency_count', 0)}**; status: `{supplemental.get('status', 'missing')}`; release-ready: **{str(bool(supplemental.get('release_ready', False))).lower()}**.",
        f"- Known upstream source hashes verified: **{supplemental.get('known_source_hashes_verified_count', 0)}/{supplemental.get('known_source_file_count', 0)}**; affected production-file hashes verified: **{supplemental.get('affected_production_file_hashes_verified_count', 0)}/{supplemental.get('affected_production_file_count', 0)}**.",
        f"- AOR license hash verified: **{str(bool(supplemental.get('known_license_hash_verified', False))).lower()}**; persisted payload: **{str(bool(supplemental.get('payload_persisted', False))).lower()}**.",
        f"- AOR license payload referenced/verified: **{str(bool(supplemental.get('payload_file_verified', False))).lower()}**; included in this candidate's `files`: **{str(bool(supplemental_coverage.get('license_payload_in_output_files', False))).lower()}**. This candidate is not a self-contained combined release.",
        "",
        "## Payload files",
        "",
        "| Destination | Source package/version | SPDX | Bytes | SHA-256 |",
        "|---|---|---|---:|---|",
    ]
    for item in manifest["files"]:
        lines.append(
            f"| `{item['destination_path']}` | `{item['source_package']} {item['source_version_or_commit']}` | `{item['spdx_expression']}` | {item['source_bytes']} | `{item['source_sha256']}` |"
        )
    lines.extend(
        [
            "",
            "## Provenance caveats",
            "",
            "- Registry license texts are copied from the local Cargo source cache and are hash-verified against the selected graph report; they are not silently promoted to authoritative v2 inventory evidence.",
            "- Cargo.lock identity is required for every selected package. Optional `.cargo-checksum.json` and `.crate` files are verified when present; `.crate` license members are matched read-only by safe `<name-version>/<path>` tar member and bytes/hash, while unavailable archive evidence remains explicitly recorded.",
            "- The two checked-in Basalt NOTICE files are included for the pinned BSD-3-Clause attribution and commit `" + UPSTREAM_COMMIT + "`.",
            "- Excluded `ort`, `ort-sys`, `shlex`, `socks`, winapi rows, vcpkg, and non-Cargo linker/SDK/system-runtime inputs are not copied by this selected-profile payload.",
            "- No legal clearance, final release promotion, or native dependency closure is implied.",
            "",
        ]
    )
    return "\n".join(lines)


def build_payload(
    *,
    root: Path,
    report_path: Path,
    output_root: Path,
    cargo_lock_path: Path,
    registry_src_roots: Iterable[Path] | None = None,
    supplemental_source_manifest_path: Path | None = None,
    supplemental_payload_root: Path | None = None,
) -> dict[str, Any]:
    root = root.resolve()
    report_path = report_path.resolve()
    output_root = output_root.resolve()
    cargo_lock_path = cargo_lock_path.resolve()
    supplemental_manifest_path = (
        supplemental_source_manifest_path or root / DEFAULT_SUPPLEMENTAL_SOURCE_MANIFEST
    ).resolve()
    if output_root.exists():
        raise PayloadError(f"candidate output root must be fresh and absent: {output_root}")
    supplemental_source_dependencies = validate_supplemental_manifest(
        root, supplemental_manifest_path, supplemental_payload_root
    )
    if supplemental_source_dependencies.get("status") != SUPPLEMENTAL_PASS_STATUS:
        raise PayloadError(
            "supplemental source coverage is not release-ready: "
            f"{supplemental_source_dependencies.get('status')}; no candidate payload was staged"
        )
    report = _load_json(report_path)
    _validate_selected_report(report)
    embedded_supplemental = report["supplemental_source_dependencies"]
    for key in (
        "manifest_path",
        "manifest_sha256",
        "status",
        "payload_root",
        "payload_path",
        "payload_file_verified",
        "payload_persisted",
    ):
        if embedded_supplemental.get(key) != supplemental_source_dependencies.get(key):
            raise PayloadError(
                "selected report supplemental coverage is stale for "
                f"{key}: {embedded_supplemental.get(key)!r} != {supplemental_source_dependencies.get(key)!r}"
            )
    lock_packages = _lock_packages(cargo_lock_path)
    configured_roots = [item.resolve() for item in (registry_src_roots or _default_registry_roots())]
    rows = report["selected_packages"]
    file_map: dict[str, dict[str, Any]] = {}
    package_records: list[dict[str, Any]] = []
    local_identities: list[str] = []
    for row in rows:
        if not isinstance(row, dict):
            raise PayloadError("selected package row is not an object")
        kind = row.get("kind")
        identity = _identity(row)
        lock = lock_packages.get(identity)
        if lock is None:
            raise PayloadError(f"Cargo.lock identity missing for selected package: {identity}")
        if kind == "registry":
            package_record, _ = _verify_registry_package(row, lock_packages, configured_roots, root, file_map)
            package_records.append(package_record)
        elif kind == "local_path":
            manifest_raw = row.get("manifest")
            if not isinstance(manifest_raw, str) or not manifest_raw:
                raise PayloadError(f"local package manifest missing: {identity}")
            manifest_path = (root / manifest_raw).resolve()
            if not _is_under(manifest_path, root) or not manifest_path.is_file():
                raise PayloadError(f"local package manifest escapes/missing root: {manifest_path}")
            if _package_manifest_identity(manifest_path, workspace_version=str(row.get("version"))) != (str(row.get("name")), str(row.get("version"))):
                raise PayloadError(f"local Cargo.toml identity mismatch: {identity}")
            destinations = _workspace_file_specs(row, root, file_map, local_identities)
            package_records.append(
                {
                    "name": row.get("name"),
                    "version": row.get("version"),
                    "kind": "local_path",
                    "source": None,
                    "spdx_expression": row["license"]["license_expression"],
                    "inventory_basis": row["license"].get("inventory_basis"),
                    "inventory_status": row["license"].get("inventory_status"),
                    "manifest_path": _display_path(manifest_path),
                    "cargo_lock": {**dict(lock), "identity_verified": True},
                    "license_files": [{"destination_path": item} for item in destinations],
                    "source_evidence_kind": "workspace_root",
                }
            )
        else:
            raise PayloadError(f"unexpected selected package kind: {kind!r} ({identity})")
    if len(local_identities) != EXPECTED_LOCAL:
        raise PayloadError(f"local package count changed during staging: {len(local_identities)}")
    attribution_destinations = _attribution_specs(root, file_map)
    output_root.mkdir(parents=True, exist_ok=False)
    _copy_specs(output_root, file_map.values())
    files = sorted(file_map.values(), key=lambda item: item["destination_path"])
    checksum_verified = sum(item.get("cargo_checksum_sidecar", {}).get("status") == "verified" for item in package_records)
    checksum_unavailable = sum(item.get("cargo_checksum_sidecar", {}).get("status") == "unavailable" for item in package_records)
    archive_verified = sum(item.get("crate_archive", {}).get("status") == "verified_license_members" for item in package_records)
    archive_unavailable = sum(item.get("crate_archive", {}).get("status") == "unavailable" for item in package_records)
    caveats = [
        "Registry license files are local Cargo source-cache evidence, copied with source/destination hashes; they are not authoritative v2 inventory records unless separately promoted.",
        "The two Basalt NOTICE files are included for attribution only; this payload does not assert that vcpkg, ONNX, SDK, CRT, linker, or other native artifacts are shipped.",
        "Excluded inventory rows are not copied: ort, ort-sys, shlex, socks, winapi, both winapi GNU target rows, and the remaining unused rows.",
        "Candidate selected-profile payload only; no legal clearance, final release archive, or release promotion is implied.",
    ]
    if archive_unavailable:
        caveats.insert(1, "No selected .crate archive was available locally; archive-to-license-member binding is therefore unavailable for those rows.")
    if checksum_unavailable:
        caveats.insert(2, "No selected .cargo-checksum.json sidecar was available locally; Cargo.lock identities and source/report license hashes were verified.")
    manifest: dict[str, Any] = {
        "schema": SCHEMA,
        "status": "pass_candidate_selected_license_payload_staged",
        "captured_at": _datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "scope": {
            "target": report["scope"]["target"],
            "root_package": report["scope"]["root_package"],
            "example": report["scope"]["example"],
            "no_default_features": True,
            "selected_report_path": _display_path(report_path),
            "supplemental_source_manifest_path": _display_path(supplemental_manifest_path),
            "supplemental_payload_root": (
                _display_path(supplemental_payload_root)
                if supplemental_payload_root
                else None
            ),
            "output_root": _display_path(output_root),
        },
        "selected_report": {
            "path": _display_path(report_path),
            "bytes": report_path.stat().st_size,
            "sha256": sha256_file(report_path),
            "status": report.get("status"),
            "metadata_sha256": report.get("metadata", {}).get("metadata_sha256"),
            "source_inventory_v1": report.get("inventory_inputs", {}).get("v1"),
            "source_inventory_v2": report.get("inventory_inputs", {}).get("v2"),
        },
        "supplemental_source_dependencies": supplemental_source_dependencies,
        "supplemental_source_coverage": {
            "manual_source_dependency_count": supplemental_source_dependencies.get("manual_source_dependency_count", 0),
            "license_payload_path": supplemental_source_dependencies.get("payload_path"),
            "license_payload_file_verified": supplemental_source_dependencies.get("payload_file_verified", False),
            "license_payload_in_output_files": False,
            "combined_release_self_contained": False,
            "final_promotion_blocked": True,
            "final_promotion_blocker": "supplemental AOR license payload is referenced separately and is not included in this Cargo-only candidate output",
        },
        "cargo_lock": {
            "path": _display_path(cargo_lock_path),
            "bytes": cargo_lock_path.stat().st_size,
            "sha256": sha256_file(cargo_lock_path),
            "package_count": len(lock_packages),
        },
        "attribution": {
            "upstream_repository": "https://github.com/VladyslavUsenko/basalt",
            "commit": UPSTREAM_COMMIT,
            "license": "BSD-3-Clause",
            "notice_files": attribution_destinations,
        },
        "packages": sorted(package_records, key=lambda item: (str(item["name"]), str(item["version"]), str(item["source"] or ""))),
        "files": files,
        "verification": {
            "package_count": len(package_records),
            "registry_package_count": sum(item["kind"] == "registry" for item in package_records),
            "local_package_count": sum(item["kind"] == "local_path" for item in package_records),
            "payload_file_count": len(files),
            "source_destination_hash_mismatch_count": sum(not item.get("source_destination_hash_match", False) for item in files),
            "cargo_lock_identity_verified_count": sum(item.get("cargo_lock", {}).get("identity_verified") is True for item in package_records),
            "cargo_checksum_verified_count": checksum_verified,
            "cargo_checksum_unavailable_count": checksum_unavailable,
            "crate_archive_verified_count": archive_verified,
            "crate_archive_unavailable_count": archive_unavailable,
            "missing_license_text_count": 0,
            "ambiguous_license_text_count": 0,
        },
        "caveats": caveats,
    }
    manifest_path = output_root / f"{DEFAULT_OUTPUT_NAME}.json"
    markdown_path = output_root / f"{DEFAULT_OUTPUT_NAME}.md"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    markdown_path.write_text(_render_markdown(manifest), encoding="utf-8")
    manifest_sha = sha256_file(manifest_path)
    (output_root / f"{DEFAULT_OUTPUT_NAME}.sha256").write_text(f"{manifest_sha}  {manifest_path.name}\n", encoding="ascii")
    manifest["output_manifest"] = {
        "path": _display_path(manifest_path),
        "bytes": manifest_path.stat().st_size,
        "sha256": manifest_sha,
        "markdown_path": _display_path(markdown_path),
        "markdown_bytes": markdown_path.stat().st_size,
        "markdown_sha256": sha256_file(markdown_path),
    }
    return manifest


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--report", type=Path, default=None)
    parser.add_argument("--cargo-lock", type=Path, default=None)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--registry-src-root", action="append", default=None)
    parser.add_argument("--supplemental-source-manifest", type=Path, default=None)
    parser.add_argument("--supplemental-payload-root", type=Path, default=None)
    args = parser.parse_args(argv)
    root = args.root.resolve()
    report_path = (args.report or root / DEFAULT_REPORT).resolve()
    cargo_lock_path = (args.cargo_lock or root / "Cargo.lock").resolve()
    registry_roots = [Path(item).resolve() for item in args.registry_src_root] if args.registry_src_root else None
    try:
        manifest = build_payload(
            root=root,
            report_path=report_path,
            output_root=args.output_root,
            cargo_lock_path=cargo_lock_path,
            registry_src_roots=registry_roots,
            supplemental_source_manifest_path=(
                args.supplemental_source_manifest.resolve()
                if args.supplemental_source_manifest
                else None
            ),
            supplemental_payload_root=(
                args.supplemental_payload_root.resolve()
                if args.supplemental_payload_root
                else None
            ),
        )
    except (OSError, PayloadError, ValueError) as exc:
        print(f"ERROR: {exc}", file=os.sys.stderr)
        return 2
    print(json.dumps({"status": manifest["status"], "output_manifest": manifest.get("output_manifest"), "verification": manifest["verification"]}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
