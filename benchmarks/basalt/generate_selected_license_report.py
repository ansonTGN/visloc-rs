"""Reconcile a selected Cargo resolve graph with the Basalt license inventory.

This tool is deliberately narrower than a release SBOM.  It asks Cargo for the
locked, offline, target-filtered graph for the canonical root package/example
profile, traverses the complete resolve graph (including root dev edges), and
joins the result with the immutable v1 package set plus the v2 evidence
overlay.  It also checks the local Cargo registry cache for named license text
files, without downloading or copying anything.  The cache check is evidence
only: files found there are not promoted into the checked-in inventory.

The selected graph is not the same thing as a final binary/native linkage
closure.  Cargo ``links`` packages are reported explicitly; non-Cargo linker,
SDK, system-runtime, and vcpkg inputs remain outside this graph and are
reported as such instead of being invented.
"""

from __future__ import annotations

import argparse
import datetime as _datetime
import hashlib
import json
import os
import subprocess
import sys
from collections import deque
from pathlib import Path
from typing import Any, Iterable, Mapping

try:
    from verify_supplemental_source_dependencies import (
        DEFAULT_MANIFEST as DEFAULT_SUPPLEMENTAL_SOURCE_MANIFEST,
        FAIL_STATUS as SUPPLEMENTAL_FAIL_STATUS,
        PASS_STATUS as SUPPLEMENTAL_PASS_STATUS,
        PENDING_STATUS as SUPPLEMENTAL_PENDING_STATUS,
        validate_manifest as validate_supplemental_manifest,
    )
except ImportError:  # Importable both as a script and as benchmarks.basalt.*.
    from benchmarks.basalt.verify_supplemental_source_dependencies import (
        DEFAULT_MANIFEST as DEFAULT_SUPPLEMENTAL_SOURCE_MANIFEST,
        FAIL_STATUS as SUPPLEMENTAL_FAIL_STATUS,
        PASS_STATUS as SUPPLEMENTAL_PASS_STATUS,
        PENDING_STATUS as SUPPLEMENTAL_PENDING_STATUS,
        validate_manifest as validate_supplemental_manifest,
    )


SCHEMA = "visloc.basalt.selected_license_graph.v1"
DEFAULT_TARGET = "x86_64-pc-windows-msvc"
DEFAULT_ROOT_PACKAGE = "visloc-rs"
DEFAULT_EXAMPLE = "basalt_euroc_vio_demo"
DEFAULT_V1 = "benchmarks/basalt/cargo_license_inventory_v1.json"
DEFAULT_V2 = "benchmarks/basalt/cargo_license_inventory_v2.json"
COMMON_LICENSE_NAMES = {
    "LICENSE",
    "LICENSE.APACHE",
    "LICENSE.BSD",
    "LICENSE.ISC",
    "LICENSE.MD",
    "LICENSE.TXT",
    "LICENSE-APACHE",
    "LICENSE-BSD",
    "LICENSE-ISC",
    "LICENSE-MIT",
    "COPYING",
    "COPYING.APACHE",
    "COPYING.BSD",
    "COPYING.ISC",
    "COPYING.MD",
    "COPYING.MIT",
    "COPYING.TXT",
    "NOTICE",
    "NOTICE.MD",
    "NOTICE.TXT",
}


class ReportError(RuntimeError):
    """A fail-closed input or graph error."""


def _overall_report_status(cargo_graph_ok: bool, supplemental_status: str) -> str:
    """Combine independent Cargo and non-Cargo source coverage states."""

    if not cargo_graph_ok:
        return "fail_graph_reconciliation"
    if supplemental_status == SUPPLEMENTAL_PASS_STATUS:
        return "pass_selected_graph_reconciled"
    if supplemental_status == SUPPLEMENTAL_PENDING_STATUS:
        return SUPPLEMENTAL_PENDING_STATUS
    return SUPPLEMENTAL_FAIL_STATUS


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_json_sha256(value: Any) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ReportError(f"cannot load JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ReportError(f"expected JSON object: {path}")
    return value


def _path_key(row: Mapping[str, Any]) -> tuple[str, str, str]:
    return (str(row.get("name", "")), str(row.get("version", "")), str(row.get("source", "")))


def _display_name(row: Mapping[str, Any]) -> str:
    return f"{row.get('name', '?')} {row.get('version', '?')}"


def _inventory_maps(
    inventory_v1: Mapping[str, Any], inventory_v2: Mapping[str, Any]
) -> tuple[dict[tuple[str, str, str], dict[str, Any]], dict[tuple[str, str, str], dict[str, Any]]]:
    base_rows = inventory_v1.get("packages", [])
    overlay_rows = inventory_v2.get("packages", [])
    if not isinstance(base_rows, list) or not isinstance(overlay_rows, list):
        raise ReportError("license inventories must contain package arrays")
    base = {_path_key(row): dict(row) for row in base_rows if isinstance(row, dict)}
    overlay = {_path_key(row): dict(row) for row in overlay_rows if isinstance(row, dict)}
    return base, overlay


def _find_inventory_match(
    package: Mapping[str, Any],
    base_map: Mapping[tuple[str, str, str], Mapping[str, Any]],
    overlay_map: Mapping[tuple[str, str, str], Mapping[str, Any]],
) -> tuple[tuple[str, str, str] | None, Mapping[str, Any] | None, Mapping[str, Any] | None]:
    """Join metadata rows, including Cargo's source-less workspace IDs.

    Cargo metadata uses ``source=null`` for path packages, whereas the v1
    inventory records the same workspace packages as ``local-workspace``.
    Registry rows must still match their exact source, name, and version.
    """

    name = str(package.get("name", ""))
    version = str(package.get("version", ""))
    source_raw = package.get("source")
    source = "" if source_raw is None else str(source_raw)
    exact = (name, version, source)
    if exact in base_map:
        return exact, base_map[exact], overlay_map.get(exact)
    candidates = [
        key
        for key in base_map
        if key[0] == name
        and key[1] == version
        and source == ""
        and key[2] in {"", "local-workspace"}
    ]
    if len(candidates) == 1:
        key = candidates[0]
        return key, base_map[key], overlay_map.get(key)
    return None, None, None


def cargo_metadata(
    root: Path,
    cargo: str,
    target: str,
    metadata_json: Path | None = None,
) -> tuple[dict[str, Any], list[str], str]:
    if metadata_json is not None:
        metadata = load_json(metadata_json)
        return metadata, ["metadata-json", str(metadata_json)], sha256_file(metadata_json)
    command = [
        cargo,
        "metadata",
        "--format-version",
        "1",
        "--locked",
        "--offline",
        "--filter-platform",
        target,
        "--manifest-path",
        str(root / "Cargo.toml"),
        "--no-default-features",
    ]
    process = subprocess.run(
        command,
        cwd=root,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    if process.returncode != 0:
        detail = process.stderr.strip()[-4000:]
        raise ReportError(f"cargo metadata failed ({process.returncode}): {detail}")
    try:
        metadata = json.loads(process.stdout)
    except json.JSONDecodeError as exc:
        raise ReportError(f"cargo metadata returned invalid JSON: {exc}") from exc
    if not isinstance(metadata, dict):
        raise ReportError("cargo metadata returned a non-object")
    return metadata, command, canonical_json_sha256(metadata)


def _package_maps(metadata: Mapping[str, Any]) -> tuple[dict[str, dict[str, Any]], dict[str, dict[str, Any]]]:
    packages = metadata.get("packages")
    resolve = metadata.get("resolve")
    nodes = resolve.get("nodes") if isinstance(resolve, dict) else None
    if not isinstance(packages, list) or not isinstance(nodes, list):
        raise ReportError("cargo metadata must contain packages and resolve.nodes")
    package_map = {str(row["id"]): row for row in packages if isinstance(row, dict) and "id" in row}
    node_map = {str(row["id"]): row for row in nodes if isinstance(row, dict) and "id" in row}
    if not package_map or not node_map:
        raise ReportError("cargo metadata resolve graph is empty")
    return package_map, node_map


def find_root_id(
    metadata: Mapping[str, Any], root: Path, root_package: str
) -> tuple[str, dict[str, Any], dict[str, dict[str, Any]], dict[str, dict[str, Any]]]:
    package_map, node_map = _package_maps(metadata)
    resolve = metadata.get("resolve")
    root_id = str(resolve.get("root")) if isinstance(resolve, dict) and resolve.get("root") else ""
    if root_id not in package_map or root_id not in node_map:
        expected_manifest = (root / "Cargo.toml").resolve()
        matches = [
            package_id
            for package_id, row in package_map.items()
            if row.get("name") == root_package
            and Path(str(row.get("manifest_path", ""))).resolve() == expected_manifest
            and package_id in node_map
        ]
        if len(matches) != 1:
            raise ReportError(f"cannot identify unique root package {root_package!r}: {matches}")
        root_id = matches[0]
    return root_id, package_map[root_id], package_map, node_map


def _dependency_id(edge: Mapping[str, Any]) -> str | None:
    value = edge.get("pkg") or edge.get("id")
    return str(value) if value is not None else None


def reachable_package_ids(node_map: Mapping[str, Mapping[str, Any]], root_id: str) -> set[str]:
    """Return all resolve nodes reachable from the root, including dev/build edges."""

    if root_id not in node_map:
        raise ReportError(f"root package is absent from resolve nodes: {root_id}")
    seen: set[str] = set()
    queue: deque[str] = deque([root_id])
    while queue:
        package_id = queue.popleft()
        if package_id in seen:
            continue
        seen.add(package_id)
        for edge in node_map[package_id].get("dependencies", []):
            # Cargo metadata v1 exposes the resolved package IDs in
            # ``dependencies`` as strings and the richer edge kind/target
            # records separately in ``deps``.  Accept both forms so the
            # traversal cannot silently collapse to the root node.
            if isinstance(edge, str):
                dependency_id = edge
            elif isinstance(edge, dict):
                dependency_id = _dependency_id(edge)
            else:
                dependency_id = None
            if dependency_id and dependency_id not in seen:
                queue.append(dependency_id)
    return seen


def _relative_manifest(root: Path, manifest_path: str | None) -> str | None:
    if not manifest_path:
        return None
    try:
        return Path(manifest_path).resolve().relative_to(root.resolve()).as_posix()
    except ValueError:
        return str(Path(manifest_path).resolve()).replace("\\", "/")


def _registry_roots(explicit: Iterable[str] | None) -> list[Path]:
    if explicit:
        return [Path(item).expanduser().resolve() for item in explicit]
    cargo_home = Path(os.environ.get("CARGO_HOME", Path.home() / ".cargo"))
    source_root = cargo_home / "registry" / "src"
    if not source_root.is_dir():
        return []
    return sorted(
        (child for child in source_root.iterdir() if child.is_dir()),
        key=lambda path: str(path).casefold(),
    )


def _cache_package_dir(name: str, version: str, roots: Iterable[Path]) -> Path | None:
    directory_name = f"{name}-{version}"
    for source_root in roots:
        candidate = source_root / directory_name
        if candidate.is_dir():
            return candidate
    return None


def _license_file_records(directory: Path, *, relative_root: Path | None = None) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    try:
        entries = sorted(directory.iterdir(), key=lambda path: path.name.casefold())
    except OSError:
        return records
    for path in entries:
        if not path.is_file():
            continue
        upper_name = path.name.upper()
        if upper_name not in COMMON_LICENSE_NAMES and not (
            upper_name.startswith("LICENSE")
            or upper_name.startswith("COPYING")
            or upper_name.startswith("NOTICE")
        ):
            continue
        relative = path.relative_to(relative_root).as_posix() if relative_root else str(path).replace("\\", "/")
        try:
            records.append(
                {
                    "path": relative,
                    "bytes": path.stat().st_size,
                    "sha256": sha256_file(path),
                }
            )
        except OSError:
            continue
    return records


def _workspace_license_records(root: Path) -> list[dict[str, Any]]:
    return _license_file_records(root, relative_root=root)


def _overlay_payload(
    overlay: Mapping[str, Any], root: Path
) -> tuple[list[dict[str, Any]], str]:
    raw_files = overlay.get("license_files", [])
    files: list[dict[str, Any]] = []
    for raw in raw_files if isinstance(raw_files, list) else []:
        if not isinstance(raw, dict):
            continue
        item = dict(raw)
        evidence = item.get("evidence")
        item["evidence_exists"] = bool(evidence and (root / str(evidence)).is_file())
        files.append(item)
    if not files:
        return files, "v2_license_evidence_missing"
    if not all(bool(item.get("evidence_exists")) for item in files):
        return files, "v2_license_evidence_path_missing"
    if any(item.get("packaged") is False for item in files):
        return files, "v2_upstream_only_license_text"
    return files, "v2_license_evidence_present"


def _payload_for_package(
    package: Mapping[str, Any],
    inventory: Mapping[str, Any] | None,
    overlay: Mapping[str, Any] | None,
    root: Path,
    registry_roots: list[Path],
) -> dict[str, Any]:
    name = str(package.get("name", ""))
    version = str(package.get("version", ""))
    source = package.get("source")
    expression = (inventory or {}).get("license") or package.get("license")
    result: dict[str, Any] = {
        "license_expression": expression,
        "inventory_basis": "none",
        "inventory_status": "not_in_registry_inventory",
        "v2_license_files": [],
        "registry_cache_license_files": [],
        "payload_status": "unknown",
    }
    if inventory is not None:
        result["inventory_basis"] = "v2_overlay" if overlay is not None else "v1_base"
        result["inventory_status"] = (overlay or {}).get("status", "v1_base_resolved")
    if source is None:
        workspace_files = _workspace_license_records(root)
        result["workspace_license_files"] = workspace_files
        result["payload_status"] = (
            "workspace_license_files_present" if workspace_files else "workspace_license_files_missing"
        )
        return result
    if overlay is not None:
        evidence, status = _overlay_payload(overlay, root)
        result["v2_license_files"] = evidence
        result["payload_status"] = status
    cache_dir = _cache_package_dir(name, version, registry_roots)
    if cache_dir is not None:
        result["registry_cache_source_dir"] = str(cache_dir).replace("\\", "/")
        result["registry_cache_license_files"] = _license_file_records(cache_dir)
    if overlay is None:
        result["payload_status"] = (
            "cache_only_not_in_v2_inventory"
            if result["registry_cache_license_files"]
            else "license_text_not_in_v2_or_local_cache"
        )
    return result


def _package_kind(package: Mapping[str, Any]) -> str:
    source = package.get("source")
    if source is None:
        return "local_path"
    if str(source).startswith("registry+"):
        return "registry"
    if str(source).startswith("git+"):
        return "git"
    return "other"


def _target_kinds(package: Mapping[str, Any]) -> list[str]:
    kinds: set[str] = set()
    for target in package.get("targets", []) if isinstance(package.get("targets"), list) else []:
        if not isinstance(target, dict):
            continue
        for kind in target.get("kind", []) if isinstance(target.get("kind"), list) else []:
            kinds.add(str(kind))
    return sorted(kinds)


def _build_script_link_details(package: Mapping[str, Any]) -> dict[str, Any] | None:
    """Inspect literal build-script link directives without executing a build."""

    targets = package.get("targets", [])
    if not isinstance(targets, list):
        return None
    for target in targets:
        if not isinstance(target, dict) or "custom-build" not in target.get("kind", []):
            continue
        source_path = target.get("src_path")
        if not source_path:
            return {"status": "custom_build_target_has_no_source_path"}
        path = Path(str(source_path))
        try:
            text = path.read_text(encoding="utf-8", errors="replace")
        except OSError as exc:
            return {
                "status": "custom_build_source_unreadable",
                "path": str(path).replace("\\", "/"),
                "error": str(exc),
            }
        directives = [
            line.strip()
            for line in text.splitlines()
            if "cargo:rustc-link" in line or "cargo:rustc-cdylib-link-arg" in line
        ]
        return {
            "status": "no_literal_rustc_link_directive" if not directives else "literal_rustc_link_directive_found",
            "path": str(path).replace("\\", "/"),
            "literal_link_directives": directives,
        }
    return None


def build_report(
    *,
    root: Path,
    cargo: str,
    target: str,
    root_package: str,
    example: str,
    inventory_v1_path: Path,
    inventory_v2_path: Path,
    metadata_json: Path | None = None,
    registry_src_roots: Iterable[str] | None = None,
    supplemental_source_manifest_path: Path | None = None,
    supplemental_payload_root: Path | None = None,
) -> dict[str, Any]:
    root = root.resolve()
    supplemental_manifest_path = (
        supplemental_source_manifest_path or root / DEFAULT_SUPPLEMENTAL_SOURCE_MANIFEST
    ).resolve()
    supplemental_source_dependencies = validate_supplemental_manifest(
        root, supplemental_manifest_path, supplemental_payload_root
    )
    inventory_v1 = load_json(inventory_v1_path)
    inventory_v2 = load_json(inventory_v2_path)
    base_map, overlay_map = _inventory_maps(inventory_v1, inventory_v2)
    metadata, command, metadata_digest = cargo_metadata(root, cargo, target, metadata_json)
    root_id, root_record, package_map, node_map = find_root_id(metadata, root, root_package)
    selected_ids = reachable_package_ids(node_map, root_id)
    registry_roots = _registry_roots(registry_src_roots)

    selected_keys: set[tuple[str, str, str]] = set()
    selected_inventory_keys: set[tuple[str, str, str]] = set()
    selected_packages: list[dict[str, Any]] = []
    native_packages: list[dict[str, Any]] = []
    for package_id in sorted(selected_ids, key=lambda item: (str(package_map[item].get("name")), item)):
        package = package_map[package_id]
        key = (str(package.get("name", "")), str(package.get("version", "")), str(package.get("source", "")))
        selected_keys.add(key)
        inventory_key, base, overlay = _find_inventory_match(package, base_map, overlay_map)
        if inventory_key is not None:
            selected_inventory_keys.add(inventory_key)
        inventory = overlay or base
        package_result: dict[str, Any] = {
            "id": package_id,
            "name": package.get("name"),
            "version": package.get("version"),
            "source": package.get("source"),
            "kind": _package_kind(package),
            "manifest": _relative_manifest(root, package.get("manifest_path")),
            "target_kinds": _target_kinds(package),
            "inventory_key": list(inventory_key) if inventory_key is not None else None,
            "license": _payload_for_package(package, inventory, overlay, root, registry_roots),
        }
        if package.get("links") is not None:
            package_result["links"] = package.get("links")
            build_script = _build_script_link_details(package)
            if build_script is not None:
                package_result["build_script_link_check"] = build_script
            native_packages.append(
                {
                    "name": package.get("name"),
                    "version": package.get("version"),
                    "source": package.get("source"),
                    "links": package.get("links"),
                    "kind": _package_kind(package),
                    "target_kinds": _target_kinds(package),
                    "build_script_link_check": build_script,
                }
            )
        selected_packages.append(package_result)

    unused_packages: list[dict[str, Any]] = []
    for key, row in sorted(base_map.items(), key=lambda item: item[0]):
        if key in selected_inventory_keys:
            continue
        unused_packages.append(
            {
                "name": row.get("name"),
                "version": row.get("version"),
                "source": row.get("source"),
                "license": row.get("license"),
                "inventory_status": (overlay_map.get(key) or {}).get("status", "v1_base_resolved"),
                "reason": "not reachable from canonical root resolve graph under target/no-default-features",
            }
        )

    selected_registry = [row for row in selected_packages if row["kind"] == "registry"]
    selected_registry_inventory_missing = [
        f"{row['name']} {row['version']} ({row['source']})"
        for row in selected_registry
        if row.get("inventory_key") is None
    ]
    selected_local_inventory_missing = [
        f"{row['name']} {row['version']} ({row['manifest']})"
        for row in selected_packages
        if row["kind"] == "local_path" and row.get("inventory_key") is None
    ]
    selected_inventory_missing = selected_registry_inventory_missing + selected_local_inventory_missing
    special_names = {
        "ort",
        "ort-sys",
        "shlex",
        "socks",
        "winapi",
        "winapi-i686-pc-windows-gnu",
        "winapi-x86_64-pc-windows-gnu",
    }
    special = []
    for row in sorted(base_map.values(), key=lambda item: (str(item.get("name")), str(item.get("version")))):
        if row.get("name") not in special_names:
            continue
        key = _path_key(row)
        selected = key in selected_inventory_keys
        special.append(
            {
                "name": row.get("name"),
                "version": row.get("version"),
                "source": row.get("source"),
                "license": row.get("license"),
                "selected": selected,
                "inventory_status": (overlay_map.get(key) or {}).get("status", "v1_base_resolved"),
                "reason": (
                    "reachable from canonical root/example resolve graph"
                    if selected
                    else "not reachable under x86_64-pc-windows-msvc and --no-default-features"
                ),
            }
        )

    payload_status_counts: dict[str, int] = {}
    missing_authoritative_payload: list[dict[str, Any]] = []
    cache_missing: list[dict[str, Any]] = []
    for row in selected_packages:
        payload = row["license"]
        status = str(payload.get("payload_status"))
        payload_status_counts[status] = payload_status_counts.get(status, 0) + 1
        if row["kind"] == "registry" and payload.get("inventory_basis") == "v1_base":
            missing_authoritative_payload.append(
                {
                    "name": row["name"],
                    "version": row["version"],
                    "license": payload.get("license_expression"),
                    "cache_status": status,
                    "cache_license_files": payload.get("registry_cache_license_files", []),
                }
            )
        if row["kind"] == "registry" and not payload.get("registry_cache_license_files") and not payload.get("v2_license_files"):
            cache_missing.append(
                {
                    "name": row["name"],
                    "version": row["version"],
                    "license": payload.get("license_expression"),
                }
            )

    unresolved_v1 = {str(item) for item in inventory_v1.get("unresolved_packages", [])}
    selected_overlay_names = [
        _display_name(row)
        for row in selected_packages
        if row["license"].get("inventory_basis") == "v2_overlay"
    ]
    selected_base_names = [
        _display_name(row)
        for row in selected_packages
        if row["license"].get("inventory_basis") == "v1_base"
    ]
    native_names = ", ".join(
        f"{row['name']} {row['version']}" for row in native_packages
    )
    native_classification = (
        "No Cargo package declares a links namespace in the selected graph."
        if not native_packages
        else "Cargo links namespaces requiring separate native-link review: " + native_names + "."
    )
    cargo_graph_ok = (
        len(selected_ids) == 66
        and not selected_inventory_missing
        and not selected_local_inventory_missing
    )
    cargo_graph_status = (
        "pass_selected_graph_reconciled" if cargo_graph_ok else "fail_graph_reconciliation"
    )
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "captured_at": _datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "status": _overall_report_status(
            cargo_graph_ok, str(supplemental_source_dependencies.get("status"))
        ),
        "cargo_graph_status": cargo_graph_status,
        "supplemental_source_dependencies": supplemental_source_dependencies,
        "scope": {
            "root_package": root_package,
            "root_manifest": _relative_manifest(root, root_record.get("manifest_path")),
            "example": example,
            "target": target,
            "features": [],
            "no_default_features": True,
            "resolve_edge_policy": "traverse every Cargo resolve edge, including root dev/build edges; target filtering is delegated to Cargo",
            "supplemental_source_manifest": _relative_manifest(
                root, str(supplemental_manifest_path)
            ),
            "supplemental_payload_root": (
                str(supplemental_payload_root.resolve()).replace("\\", "/")
                if supplemental_payload_root
                else None
            ),
        },
        "metadata": {
            "command": command,
            "metadata_sha256": metadata_digest,
            "package_count_in_metadata": len(package_map),
            "resolve_node_count": len(node_map),
            "root_id": root_id,
            "reachable_package_count": len(selected_ids),
        },
        "inventory_inputs": {
            "v1": {
                "path": str(inventory_v1_path).replace("\\", "/"),
                "bytes": inventory_v1_path.stat().st_size,
                "sha256": sha256_file(inventory_v1_path),
                "package_count": inventory_v1.get("package_count"),
            },
            "v2": {
                "path": str(inventory_v2_path).replace("\\", "/"),
                "bytes": inventory_v2_path.stat().st_size,
                "sha256": sha256_file(inventory_v2_path),
                "package_count": inventory_v2.get("package_count"),
                "licenses_resolved": inventory_v2.get("licenses_resolved"),
                "licenses_unresolved": inventory_v2.get("licenses_unresolved"),
                "overlay_count": len(overlay_map),
            },
        },
        "reconciliation": {
            "selected_package_count": len(selected_packages),
            "selected_local_package_count": sum(row["kind"] == "local_path" for row in selected_packages),
            "selected_registry_package_count": len(selected_registry),
            "selected_v2_overlay_count": len(selected_overlay_names),
            "selected_v1_base_count": len(selected_base_names),
            "selected_workspace_inventory_count": sum(
                row["kind"] == "local_path" and row.get("inventory_key") is not None
                for row in selected_packages
            ),
            "selected_registry_inventory_count": sum(
                row["kind"] == "registry" and row.get("inventory_key") is not None
                for row in selected_packages
            ),
            "selected_inventory_missing_count": len(selected_inventory_missing),
            "selected_inventory_missing": selected_inventory_missing,
            "selected_registry_inventory_missing": selected_registry_inventory_missing,
            "selected_local_inventory_missing_count": len(selected_local_inventory_missing),
            "selected_local_inventory_missing": selected_local_inventory_missing,
            "unused_inventory_count": len(unused_packages),
            "v1_unresolved_selected": sorted(
                name for name in selected_base_names if name in unresolved_v1
            ),
            "special_package_status": special,
            "excluded_special_dependencies_are_current_executable_blockers": False,
            "excluded_special_dependency_policy": "Caveats for special rows not reachable from this selected graph are informational for this executable; revisit only when a feature/profile or final linker record selects them.",
            "payload_status_counts": payload_status_counts,
            "authoritative_license_payload_missing_count": len(missing_authoritative_payload),
            "authoritative_license_payload_missing": missing_authoritative_payload,
            "license_text_missing_from_known_local_cache_count": len(cache_missing),
            "license_text_missing_from_known_local_cache": cache_missing,
        },
        "selected_packages": selected_packages,
        "unused_inventory_packages": unused_packages,
        "native_dependency_check": {
            "cargo_links_packages": native_packages,
            "cargo_links_package_count": len(native_packages),
            "literal_native_link_directive_count": sum(
                len((row.get("build_script_link_check") or {}).get("literal_link_directives", []))
                for row in native_packages
            ),
            "classification": native_classification,
            "non_cargo_linker_sdk_system_inputs": "not represented by Cargo metadata; obtain from final build/link provenance separately",
            "vcpkg_in_selected_cargo_graph": False,
            "vcpkg_note": "vcpkg is not a Cargo package ID and is not reached by this selected graph. Do not treat the pinned native vcpkg oracle as a dependency of this executable without final linker evidence.",
            "selected_profile_impact": "No excluded vcpkg/ONNX/network package is a blocker for the canonical no-default-features graph; final native linkage still requires a separate build-input record.",
        },
        "protocol_and_mapper_boundary": {
            "protocol_path": "benchmarks/basalt/protocols/basalt_euroc_parity_v1.json",
            "protocol_requirement": "11 sequences x 2 methods x 3 repetitions = 66 VIO cells; protocol metrics are trajectory/coverage/runtime/RSS and it specifies no 52/80/400 mapper gate.",
            "vio_twin_control_records": "The existing 52/80/400 records are bounded VIO/control evidence, not mapper-specific obligations.",
            "mapper_obligations": [
                "real native packet/image archive/companion to Rust mapper end-to-end binding",
                "FEJ/schema4 field closure or an explicit fail-closed exclusion manifest",
                "dense H/b/J/RHS/FEJ/Q2 and per-iteration optimization equivalence where claimed",
                "complete keyframe/landmark lifecycle and native final map/trajectory comparison",
            ],
            "mapper_authority": [
                "pipelines/basalt/MAPPER_PROVENANCE.md",
                "work/m11_mapper_completion_audit_20260903.json",
            ],
        },
        "limitations": [
            "Registry license files found in the local Cargo cache are evidence-only and are not part of the v2 inventory or a release bundle.",
            "The v1 base records license expressions/checksums but do not carry per-license-text evidence paths for its 108 previously resolved rows; those selected rows are listed explicitly.",
            "Cargo metadata does not prove which native linker, SDK, system runtime, or vcpkg artifacts enter a final executable.",
            "This report performs no network access, Cargo build, engine run, formal run, legal clearance, or release promotion.",
        ],
    }
    return report


def _short_source(source: Any) -> str:
    if source is None:
        return "local"
    if str(source).startswith("registry+"):
        return "registry"
    return str(source)


def render_markdown(report: Mapping[str, Any]) -> str:
    scope = report["scope"]
    metadata = report["metadata"]
    recon = report["reconciliation"]
    supplemental = report.get("supplemental_source_dependencies", {})
    if not isinstance(supplemental, Mapping):
        supplemental = {}
    supplemental_errors = supplemental.get("errors", [])
    if not isinstance(supplemental_errors, list):
        supplemental_errors = [str(supplemental_errors)]
    lines = [
        "# Selected Cargo license graph",
        "",
        f"Status: **{report['status']}**  ",
        f"Captured: `{report['captured_at']}`  ",
        f"Schema: `{report['schema']}`",
        "",
        "This is a locked/offline Cargo graph reconciliation for the canonical "
        f"`{scope['root_package']}` / `{scope['example']}` profile. It is a "
        "license payload checklist, not legal clearance or a final binary/native SBOM.",
        "",
        "## Graph and inventory counts",
        "",
        f"- Cargo reachable packages: **{metadata['reachable_package_count']}** "
        f"({recon['selected_local_package_count']} local, {recon['selected_registry_package_count']} registry)",
        f"- v2 overlay rows selected: **{recon['selected_v2_overlay_count']}**; v1-base rows selected: **{recon['selected_v1_base_count']}** "
        f"({recon['selected_registry_inventory_count']} registry + {recon['selected_workspace_inventory_count']} workspace)",
        f"- Inventory rows unused by this profile: **{recon['unused_inventory_count']}**",
        f"- Selected registry rows lacking authoritative v2 per-file license evidence (cache texts listed below): **{recon['authoritative_license_payload_missing_count']}**",
        f"- Selected registry rows with no named license text in the known local cache: **{recon['license_text_missing_from_known_local_cache_count']}**",
        "",
        "## Supplemental non-Cargo source coverage",
        "",
        f"- Manual source dependencies: **{supplemental.get('manual_source_dependency_count', 0)}**; status: `{supplemental.get('status', 'missing')}`; release-ready: **{str(bool(supplemental.get('release_ready', False))).lower()}**.",
        f"- Known upstream source hashes verified: **{supplemental.get('known_source_hashes_verified_count', 0)}/{supplemental.get('known_source_file_count', 0)}**; affected production-file hashes verified: **{supplemental.get('affected_production_file_hashes_verified_count', 0)}/{supplemental.get('affected_production_file_count', 0)}**.",
        f"- AOR license hash verified: **{str(bool(supplemental.get('known_license_hash_verified', False))).lower()}**; persisted payload: **{str(bool(supplemental.get('payload_persisted', False))).lower()}**.",
        "",
        "Metadata command:",
        "",
        "```text",
        " ".join(str(part) for part in metadata["command"]),
        "```",
        "",
        "## Selected 66-package payload checklist",
        "",
        "`v2_evidence_present` means an evidence path from the v2 overlay exists. "
        "`cache_only_not_in_v2_inventory` means a local cache license file was found "
        "but is not an authoritative v2 payload. `workspace_license_files_present` "
        "refers to the root MIT/Apache texts used by local path packages.",
        "",
        "| Package | Kind | License | Inventory | Payload status | License text evidence |",
        "|---|---|---|---|---|---|",
    ]
    if supplemental_errors:
        lines.insert(
            lines.index("Metadata command:"),
            "- Supplemental verification errors: "
            + "; ".join(f"`{item}`" for item in supplemental_errors),
        )
    for row in report["selected_packages"]:
        payload = row["license"]
        evidence = payload.get("v2_license_files") or payload.get("registry_cache_license_files") or payload.get("workspace_license_files") or []
        evidence_names = ", ".join(str(item.get("path")) for item in evidence[:4]) or "MISSING"
        if len(evidence) > 4:
            evidence_names += f" (+{len(evidence) - 4})"
        lines.append(
            f"| `{row['name']} {row['version']}` | {row['kind']} | `{payload.get('license_expression')}` | "
            f"{payload.get('inventory_basis')} / {payload.get('inventory_status')} | "
            f"`{payload.get('payload_status')}` | `{evidence_names}` |"
        )
    lines.extend(
        [
            "",
            "## Unused inventory rows",
            "",
            "These are present in the 119-row workspace inventory but are not reachable "
            "from the selected target-filtered root graph:",
            "",
            ", ".join(f"`{row['name']} {row['version']}`" for row in report["unused_inventory_packages"]),
            "",
            "## Special rows requested for review",
            "",
            "| Package | Selected | Inventory status | Exact graph result |",
            "|---|---:|---|---|",
        ]
    )
    for row in recon["special_package_status"]:
        lines.append(
            f"| `{row['name']} {row['version']}` | {str(row['selected']).lower()} | `{row['inventory_status']}` | {row['reason']} |"
        )
    lines.extend(
        [
            "",
            "## Native dependency boundary",
            "",
            f"Cargo `links` packages in the selected graph: **{report['native_dependency_check']['cargo_links_package_count']}**.",
            "",
        ]
    )
    if report["native_dependency_check"]["cargo_links_packages"]:
        lines.append("| Package | `links` value | Kind |")
        lines.append("|---|---|---|")
        for row in report["native_dependency_check"]["cargo_links_packages"]:
            lines.append(f"| `{row['name']} {row['version']}` | `{row['links']}` | {row['kind']} |")
        lines.append("")
    lines.extend(
        [
            f"Cargo `links` packages: **{report['native_dependency_check']['cargo_links_package_count']}**; literal native link directives observed: **{report['native_dependency_check']['literal_native_link_directive_count']}**. Cargo metadata does not represent linker/SDK/system-runtime inputs; those must be bound from the final build. vcpkg is not a selected Cargo package, so this report does not infer that it applies to this example.",
            "",
            "## Protocol and mapper boundary",
            "",
            "The frozen protocol [`basalt_euroc_parity_v1.json`](../benchmarks/basalt/protocols/basalt_euroc_parity_v1.json) specifies 11 sequences, 2 methods, and 3 repetitions (66 VIO cells) plus trajectory/coverage/runtime/RSS metrics. It does **not** specify 52/80/400 as mapper gates. Those records are VIO/control evidence.",
            "",
            "Mapper obligations are recorded in [`MAPPER_PROVENANCE.md`](../pipelines/basalt/MAPPER_PROVENANCE.md) and [`m11_mapper_completion_audit_20260903.json`](m11_mapper_completion_audit_20260903.json): real native packet/image/companion → Rust mapper E2E, FEJ/schema4 closure, dense optimization field equivalence where claimed, complete keyframe/landmark lifecycle, and native final map/trajectory comparison.",
            "",
            "## Actionable missing payload",
            "",
            "- Add authoritative per-license-text records for the selected v1-base registry rows listed in the JSON; local-cache files are not yet release payloads.",
            "- Persist the complete applicable AOR license text in the reviewed release payload, then rerun `verify_supplemental_source_dependencies.py`; the current supplemental status is pending/non-release until that payload is present.",
            "- Bind the selected Cargo `links` packages (if any) and the final linker/SDK/system-runtime/native inputs; do not substitute the unrelated pinned vcpkg oracle.",
            "- Keep 52/80/400 as VIO twin/control evidence and produce mapper-specific evidence under the mapper obligations above.",
            "- Preserve v1/v2 inventories unchanged; this report only reconciles the selected profile.",
        ]
    )
    return "\n".join(lines) + "\n"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--cargo", default="cargo")
    parser.add_argument("--target", default=DEFAULT_TARGET)
    parser.add_argument("--root-package", default=DEFAULT_ROOT_PACKAGE)
    parser.add_argument("--example", default=DEFAULT_EXAMPLE)
    parser.add_argument("--inventory-v1", type=Path, default=None)
    parser.add_argument("--inventory-v2", type=Path, default=None)
    parser.add_argument("--metadata-json", type=Path, default=None)
    parser.add_argument("--registry-src-root", action="append", default=None)
    parser.add_argument("--supplemental-source-manifest", type=Path, default=None)
    parser.add_argument("--supplemental-payload-root", type=Path, default=None)
    parser.add_argument("--output-json", type=Path, default=None)
    parser.add_argument("--output-md", type=Path, default=None)
    args = parser.parse_args(argv)
    root = args.root.resolve()
    inventory_v1 = (args.inventory_v1 or root / DEFAULT_V1).resolve()
    inventory_v2 = (args.inventory_v2 or root / DEFAULT_V2).resolve()
    report = build_report(
        root=root,
        cargo=args.cargo,
        target=args.target,
        root_package=args.root_package,
        example=args.example,
        inventory_v1_path=inventory_v1,
        inventory_v2_path=inventory_v2,
        metadata_json=args.metadata_json.resolve() if args.metadata_json else None,
        registry_src_roots=args.registry_src_root,
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
    if args.output_json is None and args.output_md is None:
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0 if report["status"] == "pass_selected_graph_reconciled" else 1
    if args.output_json is not None:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if args.output_md is not None:
        args.output_md.parent.mkdir(parents=True, exist_ok=True)
        args.output_md.write_text(render_markdown(report), encoding="utf-8")
    return 0 if report["status"] == "pass_selected_graph_reconciled" else 1


if __name__ == "__main__":
    raise SystemExit(main())
