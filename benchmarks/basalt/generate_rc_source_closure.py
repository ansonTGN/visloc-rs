"""Generate a bounded source/dependency closure for the Basalt RC example.

The historical Basalt provenance generator records only ``pipelines/basalt``.
This companion tool binds the complete *local* Cargo path-dependency closure
needed by the canonical root example, while keeping registry sources out of
the checkout snapshot.  It is intentionally read-only with respect to source
inputs: it hashes files and writes only an explicitly requested new manifest.

The default scope is the release command used by the RC preflight:

    cargo build --locked --release --no-default-features \
      --manifest-path Cargo.toml --example basalt_euroc_vio_demo

Cargo's registry resolution is represented by ``Cargo.lock``; this tool does
not walk ``target/`` or ``work/`` and does not copy a Cargo registry.  Local
path dependencies are resolved from their manifests, with optional local
dependencies activated only when the selected package feature graph enables
them.  Literal ``include_str!`` and ``include_bytes!`` paths are hashed too.
Dynamic include expressions are reported as unresolved instead of silently
being omitted.
"""

from __future__ import annotations

import argparse
import ast
import datetime as _datetime
import hashlib
import json
import os
import shutil
import subprocess
import re
from collections import defaultdict, deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping

try:  # Python 3.11+ stdlib; tomli keeps the read-only scanner usable on 3.10.
    import tomllib
except ModuleNotFoundError:  # pragma: no cover - exercised only on Python 3.10.
    try:
        import tomli as tomllib
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "generate_rc_source_closure.py requires Python 3.11+ or the tomli package"
        ) from exc


ROOT = Path(__file__).resolve().parents[2]
SCHEMA_ID = "visloc.basalt.rc_source_closure.v1"
DEFAULT_EXAMPLE = "basalt_euroc_vio_demo"
DEFAULT_TARGET = "x86_64-pc-windows-msvc"
UPSTREAM_NATIVE_COMMIT = "0f3b2b52c807f70ff4e2973ce253c73329eea7bc"
IGNORED_ROOTS = {".git", ".codex", ".agents", "target", "work"}
CONFIG_CANDIDATES = (".cargo/config.toml", ".cargo/config")
TOOLCHAIN_CANDIDATES = ("rust-toolchain.toml", "rust-toolchain")
INCLUDE_RE = re.compile(r"\b(include|include_str|include_bytes)\s*!\s*\(")
BUILD_READ_RE = re.compile(
    r"\b(?:std::)?fs::(?:read(?:_to_string)?|read_dir|canonicalize|metadata|symlink_metadata)\b"
    r"|\b(?:std::)?fs::File::open\b|\bFile::open\b"
)
BUILD_WRITE_RE = re.compile(
    r"\b(?:std::)?fs::(?:write|create_dir|create_dir_all|copy|rename|remove_file|remove_dir_all)\b"
    r"|\b(?:std::)?fs::File::create\b|\bFile::create\b"
    r"|\b(?:std::)?fs::OpenOptions::new\b|\bOpenOptions::new\b"
)


class ClosureError(ValueError):
    """The requested closure cannot be represented safely."""


@dataclass(frozen=True)
class Dependency:
    name: str
    path: Path | None
    optional: bool
    default_features: bool
    features: tuple[str, ...]
    kind: str
    target_condition: str | None
    package_name: str | None


@dataclass
class Package:
    manifest: Path
    root: Path
    name: str
    data: dict[str, Any]
    dependencies: list[Dependency]


def sha256(path: Path) -> str:
    """Hash exact checked-out bytes, matching the existing v2 tool."""

    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _relative(root: Path, path: Path) -> str:
    try:
        return path.resolve().relative_to(root.resolve()).as_posix()
    except ValueError as exc:
        raise ClosureError(f"path is outside repository root: {path}") from exc


def _is_ignored_relative(relative: str) -> bool:
    parts = Path(relative).parts
    return bool(parts) and parts[0] in IGNORED_ROOTS


def _checked_file(root: Path, path: Path, *, label: str = "file") -> Path:
    resolved = path.resolve()
    relative = _relative(root, resolved)
    if _is_ignored_relative(relative):
        raise ClosureError(f"{label} is under an excluded tree: {relative}")
    if not resolved.is_file():
        raise ClosureError(f"{label} is missing: {relative}")
    return resolved


def _walk_files(root: Path) -> list[Path]:
    """Walk one package-owned directory without entering bulk output trees."""

    if not root.is_dir():
        return []
    result: list[Path] = []
    for directory, directory_names, file_names in _os_walk(root):
        directory_names[:] = sorted(
            name for name in directory_names if name not in IGNORED_ROOTS
        )
        for name in sorted(file_names):
            result.append(Path(directory) / name)
    return result


def _os_walk(root: Path):
    # Kept behind a tiny wrapper so the exclusion policy is obvious at the
    # call site and no glob can accidentally enumerate target/work.
    import os

    return os.walk(root, topdown=True, followlinks=False)


def _record(root: Path, path: Path, role: str, records: dict[str, dict[str, Any]]) -> dict[str, Any]:
    resolved = _checked_file(root, path, label=role)
    relative = _relative(root, resolved)
    existing = records.get(relative)
    if existing is None:
        existing = {
            "path": relative,
            "bytes": resolved.stat().st_size,
            "sha256": sha256(resolved),
            "roles": [role],
        }
        records[relative] = existing
    elif role not in existing["roles"]:
        existing["roles"].append(role)
        existing["roles"].sort()
    return dict(existing)


def _canonical_digest(records: Iterable[Mapping[str, Any]]) -> str:
    lines = sorted(
        f"{record['path']}|{str(record['sha256']).lower()}" for record in records
    )
    return hashlib.sha256(("\n".join(lines) + "\n").encode("utf-8")).hexdigest()


def _load_toml(path: Path) -> dict[str, Any]:
    try:
        value = tomllib.loads(path.read_text(encoding="utf-8"))
    except (OSError, tomllib.TOMLDecodeError) as exc:
        raise ClosureError(f"cannot load Cargo manifest {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ClosureError(f"Cargo manifest is not an object: {path}")
    return value


def _as_string_list(value: Any) -> tuple[str, ...]:
    if not isinstance(value, list):
        return ()
    return tuple(str(item) for item in value if isinstance(item, str))


def _dependency_tables(data: Mapping[str, Any], *, include_dev: bool) -> Iterable[tuple[str, str | None, Mapping[str, Any]]]:
    kinds = ["dependencies", "build-dependencies"]
    if include_dev:
        kinds.append("dev-dependencies")
    for kind in kinds:
        table = data.get(kind)
        if isinstance(table, Mapping):
            yield kind, None, table
    for key, target_table in data.items():
        if not isinstance(key, str) or not key.startswith("target."):
            continue
        if not isinstance(target_table, Mapping):
            continue
        for kind in kinds:
            table = target_table.get(kind)
            if isinstance(table, Mapping):
                yield kind, key[len("target.") :], table


def _dependency_from_value(
    owner: Path,
    name: str,
    value: Any,
    kind: str,
    target_condition: str | None,
    workspace_dependencies: Mapping[str, Any],
) -> Dependency:
    if isinstance(value, str):
        value = {"version": value}
    elif not isinstance(value, Mapping):
        value = {}
    if value.get("workspace") is True and name in workspace_dependencies:
        workspace_value = workspace_dependencies[name]
        if isinstance(workspace_value, Mapping):
            value = dict(workspace_value) | dict(value)
        elif isinstance(workspace_value, str):
            value = {"version": workspace_value, **dict(value)}
    raw_path = value.get("path")
    path: Path | None = None
    if isinstance(raw_path, str):
        path = (owner / raw_path).resolve()
    return Dependency(
        name=name,
        path=path,
        optional=bool(value.get("optional", False)),
        default_features=bool(value.get("default-features", True)),
        features=_as_string_list(value.get("features")),
        kind=kind,
        target_condition=target_condition,
        package_name=str(value["package"]) if isinstance(value.get("package"), str) else None,
    )


def _load_package(
    root: Path,
    manifest: Path,
    root_data: Mapping[str, Any],
    *,
    include_dev: bool,
) -> Package:
    manifest = _checked_file(root, manifest, label="Cargo.toml")
    data = _load_toml(manifest)
    package_data = data.get("package")
    if not isinstance(package_data, Mapping) or not isinstance(package_data.get("name"), str):
        raise ClosureError(f"Cargo.toml has no package.name: {_relative(root, manifest)}")
    workspace = root_data.get("workspace", {})
    workspace_dependencies = workspace.get("dependencies", {}) if isinstance(workspace, Mapping) else {}
    if not isinstance(workspace_dependencies, Mapping):
        workspace_dependencies = {}
    dependencies = [
        _dependency_from_value(
            manifest.parent,
            name,
            value,
            kind,
            target_condition,
            workspace_dependencies,
        )
        for kind, target_condition, table in _dependency_tables(data, include_dev=include_dev)
        for name, value in table.items()
        if isinstance(name, str)
    ]
    return Package(manifest, manifest.parent, str(package_data["name"]), data, dependencies)


def _path_manifest(root: Path, dependency: Dependency) -> Path:
    if dependency.path is None:
        raise ClosureError(f"dependency is not a local path dependency: {dependency.name}")
    path = dependency.path
    if path.is_dir():
        path = path / "Cargo.toml"
    return _checked_file(root, path, label=f"path dependency {dependency.name} Cargo.toml")


def _feature_table(package: Package) -> Mapping[str, Any]:
    value = package.data.get("features", {})
    return value if isinstance(value, Mapping) else {}


def _patch_tables(data: Mapping[str, Any]) -> list[str]:
    """Return Cargo patch/replace tables that need resolver authority."""

    present: list[str] = []
    for key in ("patch", "replace"):
        value = data.get(key)
        if isinstance(value, Mapping) and value:
            present.append(key)
    return present


def _expand_features(
    package: Package,
    requested: set[str],
) -> tuple[set[str], set[str], dict[str, set[str]], list[str]]:
    """Expand package feature aliases and return enabled local dependency names."""

    table = _feature_table(package)
    local_names = {dep.name for dep in package.dependencies if dep.path is not None}
    optional_names = {
        dep.name for dep in package.dependencies if dep.path is not None and dep.optional
    }
    active = set(requested)
    activated_dependencies: set[str] = set()
    dependency_features: dict[str, set[str]] = defaultdict(set)
    errors: list[str] = []
    queue = deque(active)
    while queue:
        feature = queue.popleft()
        if feature == "default":
            entries = table.get("default", [])
        else:
            entries = table.get(feature)
        if entries is None:
            if feature not in optional_names and feature != "default":
                errors.append(f"{package.name}: unknown requested feature {feature!r}")
            continue
        if not isinstance(entries, list):
            errors.append(f"{package.name}: feature {feature!r} is not an array")
            continue
        for item in entries:
            if not isinstance(item, str):
                continue
            if item.startswith("dep:"):
                dep_name = item[4:]
                if dep_name in local_names:
                    activated_dependencies.add(dep_name)
                continue
            if "/" in item:
                dep_name, dep_feature = item.split("/", 1)
                conditional = dep_name.endswith("?")
                dep_name = dep_name[:-1] if conditional else dep_name
                if dep_name in local_names:
                    if not conditional or dep_name in activated_dependencies or dep_name not in optional_names:
                        activated_dependencies.add(dep_name)
                        dependency_features[dep_name].add(dep_feature)
                continue
            if item in table:
                if item not in active:
                    active.add(item)
                    queue.append(item)
            elif item in optional_names:
                # Cargo exposes an implicit feature for optional dependencies
                # unless the feature is referenced through dep:foo.
                activated_dependencies.add(item)
    return active, activated_dependencies, dependency_features, errors


def _rust_string_end(text: str, start: int) -> tuple[str | None, int]:
    if start >= len(text):
        return None, start
    if text[start] == '"':
        index = start + 1
        escaped = False
        while index < len(text):
            char = text[index]
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                token = text[start : index + 1]
                try:
                    value = ast.literal_eval(token)
                except (SyntaxError, ValueError):
                    return None, index + 1
                return value if isinstance(value, str) else None, index + 1
            index += 1
        return None, len(text)
    if text[start] != "r":
        return None, start
    index = start + 1
    while index < len(text) and text[index] == "#":
        index += 1
    if index >= len(text) or text[index] != '"':
        return None, start
    hashes = index - start - 1
    content_start = index + 1
    terminator = '"' + ("#" * hashes)
    content_end = text.find(terminator, content_start)
    if content_end < 0:
        return None, len(text)
    return text[content_start:content_end], content_end + len(terminator)


def _mask_rust_non_code(text: str) -> str:
    """Mask comments and literals so include! names in prose are ignored."""

    chars = list(text)
    index = 0
    length = len(text)
    while index < length:
        if text.startswith("//", index):
            end = text.find("\n", index)
            end = length if end < 0 else end
            for offset in range(index, end):
                chars[offset] = " "
            index = end
            continue
        if text.startswith("/*", index):
            depth = 1
            end = index + 2
            while end < length and depth:
                if text.startswith("/*", end):
                    depth += 1
                    end += 2
                elif text.startswith("*/", end):
                    depth -= 1
                    end += 2
                else:
                    end += 1
            for offset in range(index, min(end, length)):
                if text[offset] != "\n":
                    chars[offset] = " "
            index = end
            continue
        if text[index] in {'"', "'"}:
            quote = text[index]
            end = index + 1
            escaped = False
            while end < length:
                if escaped:
                    escaped = False
                elif text[end] == "\\":
                    escaped = True
                elif text[end] == quote:
                    end += 1
                    break
                end += 1
            for offset in range(index, min(end, length)):
                if text[offset] != "\n":
                    chars[offset] = " "
            index = end
            continue
        if text[index] == "r" and index + 1 < length:
            look = index + 1
            while look < length and text[look] == "#":
                look += 1
            if look < length and text[look] == '"':
                hashes = look - index - 1
                terminator = '"' + ("#" * hashes)
                end = text.find(terminator, look + 1)
                end = length if end < 0 else end + len(terminator)
                for offset in range(index, min(end, length)):
                    if text[offset] != "\n":
                        chars[offset] = " "
                index = end
                continue
        index += 1
    return "".join(chars)


def _skip_space_and_comments(text: str, start: int) -> int:
    index = start
    while index < len(text):
        if text[index].isspace():
            index += 1
        elif text.startswith("//", index):
            newline = text.find("\n", index)
            index = len(text) if newline < 0 else newline + 1
        elif text.startswith("/*", index):
            end = text.find("*/", index + 2)
            index = len(text) if end < 0 else end + 2
        else:
            break
    return index


def _scan_includes(text: str) -> list[dict[str, Any]]:
    masked = _mask_rust_non_code(text)
    references: list[dict[str, Any]] = []
    for match in INCLUDE_RE.finditer(masked):
        start = _skip_space_and_comments(text, match.end())
        literal, end = _rust_string_end(text, start)
        line = text.count("\n", 0, match.start()) + 1
        kind = match.group(1)
        if literal is None:
            snippet = text[start : min(len(text), start + 160)].split(")", 1)[0].strip()
            references.append(
                {"macro": kind, "line": line, "literal_path": None, "expression": snippet}
            )
        else:
            references.append(
                {"macro": kind, "line": line, "literal_path": literal, "expression": None}
            )
        # ``end`` is only used to make the intent clear: regex iteration is
        # over masked source and naturally finds subsequent macros.
        _ = end
    return references


def _scan_build_script_reads(text: str) -> list[dict[str, Any]]:
    """Find build-script source reads that are not literal include! macros."""

    masked = _mask_rust_non_code(text)
    return [
        {
            "line": text.count("\n", 0, match.start()) + 1,
            "expression": text[match.start() : match.end()],
            "reason": "dynamic_build_script_filesystem_read",
        }
        for match in BUILD_READ_RE.finditer(masked)
    ]


def _scan_build_script_writes(text: str) -> list[dict[str, Any]]:
    """Find build-script writes/generation that is not in the source closure."""

    masked = _mask_rust_non_code(text)
    return [
        {
            "line": text.count("\n", 0, match.start()) + 1,
            "expression": text[match.start() : match.end()],
            "reason": "dynamic_build_script_generated_input",
        }
        for match in BUILD_WRITE_RE.finditer(masked)
    ]


def _package_build_script(package: Package) -> Path | None:
    package_data = package.data.get("package", {})
    configured = package_data.get("build") if isinstance(package_data, Mapping) else None
    if configured is False:
        return None
    if isinstance(configured, str):
        return (package.root / configured).resolve()
    candidate = package.root / "build.rs"
    return candidate.resolve() if candidate.is_file() else None


def _source_paths(package: Package) -> list[Path]:
    paths = _walk_files(package.root / "src")
    targets: list[Path] = []
    for key in ("lib",):
        table = package.data.get(key)
        if isinstance(table, Mapping) and isinstance(table.get("path"), str):
            targets.append((package.root / str(table["path"])).resolve())
    bins = package.data.get("bin", [])
    if isinstance(bins, Mapping):
        bins = [bins]
    if isinstance(bins, list):
        targets.extend(
            (package.root / str(item["path"])).resolve()
            for item in bins
            if isinstance(item, Mapping) and isinstance(item.get("path"), str)
        )
    result = {path.resolve() for path in paths}
    result.update(targets)
    return sorted(result, key=lambda path: path.as_posix().casefold())


def _example_path(package: Package, example_name: str) -> Path:
    examples = package.data.get("example", [])
    if isinstance(examples, Mapping):
        examples = [examples]
    if isinstance(examples, list):
        for item in examples:
            if isinstance(item, Mapping) and item.get("name") == example_name:
                path = item.get("path")
                if isinstance(path, str):
                    return (package.root / path).resolve()
    return (package.root / "examples" / f"{example_name}.rs").resolve()


def _dependency_summary(root: Path, dependency: Dependency) -> dict[str, Any]:
    result: dict[str, Any] = {
        "name": dependency.name,
        "kind": dependency.kind,
        "optional": dependency.optional,
        "default_features": dependency.default_features,
        "features": list(dependency.features),
    }
    if dependency.target_condition is not None:
        result["target_condition"] = dependency.target_condition
    if dependency.package_name is not None:
        result["package_name"] = dependency.package_name
    if dependency.path is not None:
        result["path"] = _relative(root, dependency.path)
    return result


def _cargo_metadata_cross_check(
    root: Path,
    *,
    root_package: str,
    expected_manifests: Iterable[str],
    target: str,
    features: Iterable[str],
    no_default_features: bool,
) -> dict[str, Any]:
    """Compare the static local graph with Cargo's locked resolved graph."""

    cargo_value = os.environ.get("CARGO")
    cargo = Path(cargo_value) if cargo_value else None
    if cargo is None or not cargo.is_file():
        found = shutil.which("cargo")
        cargo = Path(found) if found else None
    if cargo is None or not cargo.is_file():
        # rustup's per-user bin is commonly not on PATH in Windows service
        # shells; probing this exact location keeps the check read-only and
        # avoids searching target/work or inventing a toolchain selection.
        for candidate in (Path.home() / ".cargo" / "bin" / "cargo.exe", Path.home() / ".cargo" / "bin" / "cargo"):
            if candidate.is_file():
                cargo = candidate
                break
    command = [
        str(cargo or "cargo"),
        "metadata",
        "--format-version",
        "1",
        "--locked",
        "--offline",
        "--filter-platform",
        target,
        "--manifest-path",
        str(root / "Cargo.toml"),
    ]
    if no_default_features:
        command.append("--no-default-features")
    feature_list = sorted(set(features))
    if feature_list:
        command.extend(["--features", ",".join(feature_list)])
    try:
        completed = subprocess.run(command, check=False, capture_output=True, text=True)
    except OSError as exc:
        return {
            "status": "unavailable",
            "required_before_rc_freeze": True,
            "command": command,
            "error": str(exc),
        }
    if completed.returncode != 0:
        return {
            "status": "failed",
            "required_before_rc_freeze": True,
            "command": command,
            "returncode": completed.returncode,
            "stderr": completed.stderr.strip()[-2000:],
        }
    try:
        metadata = json.loads(completed.stdout)
    except json.JSONDecodeError as exc:
        return {
            "status": "invalid_output",
            "required_before_rc_freeze": True,
            "command": command,
            "error": str(exc),
        }
    packages = metadata.get("packages", [])
    resolve = metadata.get("resolve")
    if not isinstance(packages, list) or not isinstance(resolve, Mapping):
        return {
            "status": "invalid_output",
            "required_before_rc_freeze": True,
            "command": command,
            "error": "Cargo metadata has no packages/resolve objects",
        }
    by_id = {
        item.get("id"): item
        for item in packages
        if isinstance(item, Mapping) and isinstance(item.get("id"), str)
    }
    nodes = {
        item.get("id"): item
        for item in resolve.get("nodes", [])
        if isinstance(item, Mapping) and isinstance(item.get("id"), str)
    }
    root_id = resolve.get("root")
    reachable: set[str] = set()
    pending: deque[str] = deque([root_id] if isinstance(root_id, str) else [])
    while pending:
        package_id = pending.popleft()
        if package_id in reachable:
            continue
        reachable.add(package_id)
        node = nodes.get(package_id)
        if not isinstance(node, Mapping):
            continue
        for dependency in node.get("deps", []):
            if isinstance(dependency, Mapping) and isinstance(dependency.get("pkg"), str):
                pending.append(dependency["pkg"])
    expected = set(expected_manifests)
    reachable_local: set[str] = set()
    reachable_names: set[str] = set()
    for package_id in reachable:
        package = by_id.get(package_id)
        if not isinstance(package, Mapping):
            continue
        manifest_path = package.get("manifest_path")
        if not isinstance(manifest_path, str):
            continue
        try:
            relative = _relative(root, Path(manifest_path))
        except ClosureError:
            continue
        reachable_local.add(relative)
        if isinstance(package.get("name"), str):
            reachable_names.add(package["name"])
    missing = sorted(expected - reachable_local)
    extra = sorted(reachable_local - expected)
    return {
        "status": "pass" if not missing and not extra else "mismatch",
        "required_before_rc_freeze": True,
        "command": command,
        "root_package": root_package,
        "reachable_package_count": len(reachable),
        "reachable_local_manifests": sorted(reachable_local),
        "reachable_local_packages": sorted(reachable_names),
        "expected_local_manifests": sorted(expected),
        "missing_from_cargo_resolution": missing,
        "unexpected_local_in_cargo_resolution": extra,
        "note": "Cargo metadata resolves registry versions and active features; compiler dep-info is still required for exact cfg/build-script input binding.",
    }


def build_closure(
    root: Path = ROOT,
    *,
    example: str = DEFAULT_EXAMPLE,
    features: Iterable[str] = (),
    no_default_features: bool = True,
    target: str = DEFAULT_TARGET,
    include_dev_dependencies: bool = True,
    cargo_metadata_cross_check: bool = False,
    captured_at: str | None = None,
) -> dict[str, Any]:
    """Build a current-byte closure without building or mutating Cargo state."""

    root = root.resolve()
    feature_list = sorted(set(features))
    root_manifest = _checked_file(root, root / "Cargo.toml", label="workspace Cargo.toml")
    root_data = _load_toml(root_manifest)
    root_package = _load_package(
        root, root_manifest, root_data, include_dev=include_dev_dependencies
    )
    example_path = _example_path(root_package, example)
    records: dict[str, dict[str, Any]] = {}
    issues: list[str] = []
    patch_tables = _patch_tables(root_data)
    if patch_tables:
        issues.append(
            "Cargo {} table(s) are present but are not hand-expanded; use the full "
            "locked Cargo metadata cross-check before accepting this closure".format(
                ", ".join(patch_tables)
            )
        )
    package_states: dict[Path, set[str]] = {root_manifest: set(feature_list)}
    if not no_default_features:
        package_states[root_manifest].add("default")
    package_cache: dict[Path, Package] = {root_manifest: root_package}
    queue = deque([root_manifest])
    active_packages: list[Package] = []
    package_feature_errors: list[str] = []
    package_edge_state: dict[Path, tuple[list[Dependency], list[Dependency]]] = {}

    while queue:
        manifest = queue.popleft()
        package = package_cache.get(manifest)
        if package is None:
            package = _load_package(root, manifest, root_data, include_dev=include_dev_dependencies)
            package_cache[manifest] = package
        active, enabled_optional, requested_dependency_features, errors = _expand_features(
            package, package_states[manifest]
        )
        package_states[manifest] = active
        package_feature_errors.extend(errors)
        active_edges: list[Dependency] = []
        inactive_edges: list[Dependency] = []
        for dependency in package.dependencies:
            if dependency.path is None:
                continue
            enabled = not dependency.optional or dependency.name in enabled_optional
            if not enabled:
                inactive_edges.append(dependency)
                continue
            active_edges.append(dependency)
            try:
                child_manifest = _path_manifest(root, dependency)
            except ClosureError as exc:
                issues.append(str(exc))
                continue
            child = package_cache.get(child_manifest)
            if child is None:
                try:
                    child = _load_package(
                        root,
                        child_manifest,
                        root_data,
                        # Dev-dependencies are available to the selected root
                        # example, but a local dependency's dev graph is not
                        # compiled as part of that dependency.
                        include_dev=False,
                    )
                except ClosureError as exc:
                    issues.append(str(exc))
                    continue
                package_cache[child_manifest] = child
            if dependency.package_name is not None and dependency.package_name != child.name:
                issues.append(
                    f"dependency {package.name}:{dependency.name} names package "
                    f"{dependency.package_name!r}, manifest says {child.name!r}"
                )
            child_features = set(dependency.features)
            child_features.update(requested_dependency_features.get(dependency.name, set()))
            if dependency.default_features:
                child_features.add("default")
            prior = package_states.setdefault(child_manifest, set())
            if not child_features.issubset(prior):
                prior.update(child_features)
                queue.append(child_manifest)
            elif child_manifest not in [item.manifest for item in active_packages] and child_manifest not in queue:
                queue.append(child_manifest)
        package_edge_state[manifest] = (active_edges, inactive_edges)
        if package not in active_packages:
            active_packages.append(package)

    # Root and each active local package manifest/source/build script.
    root_inputs: dict[str, Any] = {}
    root_inputs["cargo_toml"] = _record(root, root_manifest, "workspace_manifest", records)
    cargo_lock = root / "Cargo.lock"
    if cargo_lock.is_file():
        root_inputs["cargo_lock"] = _record(root, cargo_lock, "cargo_lock", records)
    else:
        issues.append("Cargo.lock is missing")
        root_inputs["cargo_lock"] = None

    toolchain_records: list[dict[str, Any]] = []
    for relative in TOOLCHAIN_CANDIDATES:
        path = root / relative
        if path.is_file():
            toolchain_records.append(_record(root, path, "rust_toolchain", records))
    root_inputs["rust_toolchain"] = {
        "expected": list(TOOLCHAIN_CANDIDATES),
        "present": toolchain_records,
    }
    config_records: list[dict[str, Any]] = []
    for relative in CONFIG_CANDIDATES:
        path = root / relative
        if path.is_file():
            config_records.append(_record(root, path, "cargo_config", records))
    root_inputs["cargo_config"] = {
        "expected": list(CONFIG_CANDIDATES),
        "present": config_records,
    }

    try:
        root_inputs["example"] = _record(root, example_path, "selected_example", records)
    except ClosureError as exc:
        issues.append(str(exc))
        root_inputs["example"] = None

    package_entries: list[dict[str, Any]] = []
    source_scan_paths: list[tuple[Path, str]] = []
    for package in sorted(active_packages, key=lambda item: _relative(root, item.manifest)):
        package_records: list[dict[str, Any]] = []
        try:
            package_records.append(_record(root, package.manifest, "package_manifest", records))
        except ClosureError as exc:
            issues.append(str(exc))
        for source_path in _source_paths(package):
            try:
                package_records.append(_record(root, source_path, "package_source", records))
                source_scan_paths.append((source_path, "package_source"))
            except ClosureError as exc:
                issues.append(str(exc))
        build_script = _package_build_script(package)
        build_record = None
        if build_script is not None:
            issues.append(
                f"{_relative(root, build_script)}: build.rs execution and generated/output inputs "
                "are not statically modeled; bind fresh compiler dep-info and build outputs"
            )
            try:
                build_record = _record(root, build_script, "build_script", records)
                package_records.append(build_record)
                source_scan_paths.append((build_script, "build_script"))
            except ClosureError as exc:
                issues.append(str(exc))
        if package.manifest == root_manifest and root_inputs["example"] is not None:
            source_scan_paths.append((example_path, "selected_example"))
        active_edges, inactive_edges = package_edge_state.get(package.manifest, ([], []))
        package_entries.append(
            {
                "name": package.name,
                "manifest": _relative(root, package.manifest),
                "root": _relative(root, package.root),
                "active_features": sorted(package_states.get(package.manifest, set())),
                "files": sorted(package_records, key=lambda item: item["path"]),
                "active_local_path_dependencies": [
                    _dependency_summary(root, dependency) for dependency in active_edges
                ],
                "inactive_optional_local_path_dependencies": [
                    _dependency_summary(root, dependency) for dependency in inactive_edges
                ],
                "registry_dependencies": sorted(
                    dependency.name
                    for dependency in package.dependencies
                    if dependency.path is None and dependency.kind != "dev"
                ),
                "root_example_dev_registry_dependencies": sorted(
                    dependency.name
                    for dependency in package.dependencies
                    if dependency.path is None and dependency.kind == "dev"
                )
                if package.manifest == root_manifest
                else [],
            }
        )

    include_references: list[dict[str, Any]] = []
    unresolved_includes: list[dict[str, Any]] = []
    build_script_reads: list[dict[str, Any]] = []
    build_script_writes: list[dict[str, Any]] = []
    scanned: set[Path] = set()
    for source_path, source_role in source_scan_paths:
        if source_path in scanned:
            continue
        scanned.add(source_path)
        try:
            text = source_path.read_text(encoding="utf-8")
        except (OSError, UnicodeError) as exc:
            issues.append(f"cannot scan {source_path}: {exc}")
            continue
        try:
            source_relative = _relative(root, source_path)
        except ClosureError as exc:
            issues.append(str(exc))
            continue
        if source_role == "build_script":
            for read in _scan_build_script_reads(text):
                build_script_reads.append({"from": source_relative, **read})
            for write in _scan_build_script_writes(text):
                build_script_writes.append({"from": source_relative, **write})
        for reference in _scan_includes(text):
            entry = {"from": source_relative, "role": source_role, **reference}
            literal = reference.get("literal_path")
            if not isinstance(literal, str):
                unresolved_includes.append({**entry, "reason": "dynamic_include_expression"})
                continue
            target_path = (source_path.parent / literal).resolve()
            try:
                target_relative = _relative(root, target_path)
            except ClosureError:
                unresolved_includes.append({**entry, "reason": "outside_repository"})
                continue
            if _is_ignored_relative(target_relative):
                unresolved_includes.append({**entry, "reason": "excluded_bulk_tree"})
                continue
            if not target_path.is_file():
                unresolved_includes.append({**entry, "reason": "missing_include_input", "resolved_path": target_relative})
                continue
            include_role = "build_script_include" if source_role == "build_script" else "compile_time_include"
            include_record = _record(root, target_path, include_role, records)
            include_references.append({**entry, "resolved_path": target_relative, "sha256": include_record["sha256"], "bytes": include_record["bytes"]})
            if reference["macro"] == "include":
                # An included Rust source file may contain another include!;
                # enqueue it so nested literal inputs are not silently missed.
                source_scan_paths.append((target_path, "included_source"))

    issues.extend(package_feature_errors)
    issues.extend(
        f"{item['from']}:{item['line']}: {item['reason']}"
        for item in build_script_reads
    )
    issues.extend(
        f"{item['from']}:{item['line']}: {item['reason']}"
        for item in build_script_writes
    )
    issues.extend(
        f"{item['from']}:{item['line']}: {item['reason']}"
        for item in unresolved_includes
    )
    closure_files = [records[path] for path in sorted(records)]
    if cargo_metadata_cross_check:
        metadata_check = _cargo_metadata_cross_check(
            root,
            root_package=root_package.name,
            expected_manifests=(package["manifest"] for package in package_entries),
            target=target,
            features=feature_list,
            no_default_features=no_default_features,
        )
    else:
        metadata_command = [
            "cargo",
            "metadata",
            "--format-version",
            "1",
            "--locked",
            "--offline",
            "--filter-platform",
            target,
        ]
        if no_default_features:
            metadata_command.append("--no-default-features")
        if feature_list:
            metadata_command.extend(["--features", ",".join(feature_list)])
        metadata_check = {
            "status": "not_invoked",
            "required_before_rc_freeze": True,
            "command": metadata_command,
            "note": "Invoke with --cargo-metadata-cross-check and compare compiler dep-info before freezing.",
        }
    manifest: dict[str, Any] = {
        "schema_id": SCHEMA_ID,
        "schema_version": 1,
        "captured_at": captured_at or _datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        # "static_complete" means every input discoverable by this bounded
        # scanner resolved and hashed.  It is deliberately still a candidate:
        # only Cargo/compiler dep-info from the actual RC build can establish
        # the executable's exact active input set.
        "status": "static_complete_candidate" if not issues else "incomplete_candidate",
        "scope": {
            "root_package": root_package.name,
            "example": example,
            "example_path": root_inputs["example"]["path"] if root_inputs["example"] else None,
            "target": target,
            "features": feature_list,
            "no_default_features": no_default_features,
            "include_dev_dependencies": include_dev_dependencies,
            "root_example_dev_dependencies_included": include_dev_dependencies,
            "build_contract": ["--locked", "--release", "--no-default-features" if no_default_features else "--features=" + ",".join(feature_list)],
        },
        "authoritative_upstream": {
            "native_oracle_commit": UPSTREAM_NATIVE_COMMIT,
            "role": "pinned native comparison oracle only; not mixed into Rust closure",
        },
        "root_inputs": root_inputs,
        "local_path_dependency_graph": {
            "resolution": "Cargo.toml path keys recursively resolved; active optional edges follow selected package features",
            "package_count": len(package_entries),
            "packages": package_entries,
        },
        "compile_time_inputs": {
            "literal_macro_count": len(include_references),
            "references": sorted(include_references, key=lambda item: (item["from"], item["line"], item["macro"])),
            "unresolved": sorted(unresolved_includes, key=lambda item: (item["from"], item["line"], item["reason"])),
            "build_script_filesystem_reads": sorted(
                build_script_reads, key=lambda item: (item["from"], item["line"])
            ),
            "build_script_filesystem_writes": sorted(
                build_script_writes, key=lambda item: (item["from"], item["line"])
            ),
        },
        "closure": {
            "hash_algorithm": "sha256",
            "hash_basis": "exact checked-out bytes; no line-ending normalization",
            "canonical_record": "relative POSIX path|lowercase SHA256, newline-joined and sorted",
            "file_count": len(closure_files),
            "sha256": _canonical_digest(closure_files),
            "files": closure_files,
            "excluded_roots": sorted(IGNORED_ROOTS),
        },
        "executable_binding": {
            "status": "not_bound",
            "path": None,
            "sha256": None,
            "required_after_source_freeze": True,
            "note": "Bind the fresh E:-hosted release example only after the final source closure, compiler identity, features, and compiler dep-info agree.",
        },
        "validation": {
            "status": "pass_static_scan" if not issues else "fail_static_scan",
            "errors": sorted(set(issues)),
            "dynamic_include_count": sum(item["reason"] == "dynamic_include_expression" for item in unresolved_includes),
            "outside_repository_include_count": sum(item["reason"] == "outside_repository" for item in unresolved_includes),
            "missing_include_count": sum(item["reason"] == "missing_include_input" for item in unresolved_includes),
            "excluded_tree_include_count": sum(item["reason"] == "excluded_bulk_tree" for item in unresolved_includes),
            "build_script_filesystem_read_count": len(build_script_reads),
            "build_script_filesystem_write_count": len(build_script_writes),
            "cargo_metadata_cross_check": metadata_check,
            "compiler_dep_info_cross_check": {
                "status": "not_available_before_build",
                "required_before_rc_freeze": True,
                "note": "Bind compiler-generated example/build-script dep-info (.d) from the fresh RC build; historical target/work dep-info is not authoritative.",
            },
        },
        "linux_twin_binding": {
            "status": "required_before_final_rss_gate",
            "runtime_profile": "rust_wsl_linux",
            "source_closure_sha256": None,
            "build_inputs": None,
            "exactness_certificate": None,
            "note": "This Windows/MSVC source closure does not bind a Linux twin executable or its build inputs; provide a separately hashed Linux closure/provenance record before claiming same-domain RSS parity.",
        },
        "limitations": [
            "Registry crate source trees are not copied or hashed; Cargo.lock is bound as the resolution anchor.",
            "Cargo [patch]/[replace] tables are fail-closed rather than hand-expanded; a full locked Cargo metadata cross-check is required when present.",
            "Target-specific local path edges are recorded from all manifest target tables; cfg evaluation is not performed.",
            "Rust cfg(test) and feature-gated source is conservatively scanned, so include inputs may exceed the release executable's active subset.",
            "Dynamic include!/include_str!/include_bytes! expressions, build.rs execution, filesystem reads, and generated outputs cannot be resolved statically and fail closed.",
            "A dirty shared worktree is not a release snapshot; freeze this byte manifest from an isolated source closure before RC build.",
            "Static manifest parsing and include scanning do not prove the compiler's active cfg/feature input set; compare Cargo metadata and fresh compiler dep-info before freezing.",
            "A Linux rust_wsl_linux closure/executable/exactness binding is still required before any authoritative RSS parity claim.",
        ],
    }
    return manifest


def validate_manifest(root: Path, manifest: Mapping[str, Any]) -> list[str]:
    """Re-hash a closure manifest and return deterministic validation errors."""

    root = root.resolve()
    errors: list[str] = []
    closure = manifest.get("closure")
    if not isinstance(closure, Mapping):
        return ["closure is missing"]
    files = closure.get("files")
    if not isinstance(files, list):
        return ["closure.files is not a list"]
    seen: set[str] = set()
    for record in files:
        if not isinstance(record, Mapping):
            errors.append("closure file record is not an object")
            continue
        relative = record.get("path")
        if not isinstance(relative, str):
            errors.append("closure file has no path")
            continue
        if relative in seen:
            errors.append(f"duplicate closure path: {relative}")
        seen.add(relative)
        try:
            path = _checked_file(root, root / Path(relative), label="closure file")
        except ClosureError as exc:
            errors.append(str(exc))
            continue
        actual_sha = sha256(path)
        if actual_sha.casefold() != str(record.get("sha256", "")).casefold():
            errors.append(f"hash mismatch: {relative}")
        if path.stat().st_size != record.get("bytes"):
            errors.append(f"byte count mismatch: {relative}")
    if closure.get("file_count") != len(files):
        errors.append("closure file_count does not match files")
    if closure.get("sha256") != _canonical_digest(files):
        errors.append("closure sha256 does not match listed files")
    if manifest.get("schema_id") != SCHEMA_ID:
        errors.append("schema_id mismatch")
    return sorted(set(errors))


def render_markdown(manifest: Mapping[str, Any]) -> str:
    scope = manifest.get("scope", {})
    graph = manifest.get("local_path_dependency_graph", {})
    closure = manifest.get("closure", {})
    validation = manifest.get("validation", {})
    packages = graph.get("packages", []) if isinstance(graph, Mapping) else []
    lines = [
        "# RC source/dependency closure",
        "",
        f"Status: **{manifest.get('status', 'unknown')}**  ",
        f"Captured: `{manifest.get('captured_at')}`  ",
        f"Schema: `{manifest.get('schema_id')}`",
        "",
        "This is a source-binding candidate for the canonical Basalt EuRoC example. It does not freeze an executable or run the engine.",
        "",
        "## Scope",
        "",
        f"- Root package/example: `{scope.get('root_package')}` / `{scope.get('example')}` (`{scope.get('example_path')}`)",
        f"- Target: `{scope.get('target')}`; features: `{', '.join(scope.get('features', [])) or 'none'}`; no-default-features: `{scope.get('no_default_features')}`",
        f"- Local Cargo path packages: **{graph.get('package_count', 0)}**",
        f"- Closure files: **{closure.get('file_count', 0)}**; canonical SHA-256: `{closure.get('sha256')}`",
        f"- Native comparison oracle pin: `{manifest.get('authoritative_upstream', {}).get('native_oracle_commit')}` (comparison only)",
        "",
        "## Local package coverage",
        "",
        "| Package | Manifest | Active features | Files |",
        "|---|---|---|---:|",
    ]
    for package in packages:
        lines.append(
            f"| `{package.get('name')}` | `{package.get('manifest')}` | `{', '.join(package.get('active_features', [])) or 'none'}` | {len(package.get('files', []))} |"
        )
    lines.extend(
        [
            "",
            "Every active local path package contributes its manifest, package `src/**`, custom target source paths, and `build.rs` when present. The selected root example includes root dev-dependencies; dev-dependencies of packages consumed as libraries are excluded. Optional local edges not enabled by the selected feature graph are listed in the JSON (the canonical graph excludes disabled optional packages).",
            "",
            "## Root/build inputs",
            "",
            "The closure binds `Cargo.toml`, `Cargo.lock`, the present root `rust-toolchain*` file(s), the present root `.cargo/config*` file(s), and the selected example. Missing `Cargo.lock` or example inputs fail the candidate.",
            "",
            f"Literal compile-time include references: **{manifest.get('compile_time_inputs', {}).get('literal_macro_count', 0)}**; unresolved references: **{len(manifest.get('compile_time_inputs', {}).get('unresolved', []))}**.",
            "",
            "## Validation and limits",
            "",
            f"Re-hash validation at generation: **{validation.get('status')}**; errors: `{len(validation.get('errors', []))}`.",
            "",
            "The generator excludes `.git`, `.codex`, `.agents`, `target`, and `work` from traversal. Registry crate source is not copied; `Cargo.lock` is the registry-resolution anchor. Target-specific cfg selection and cfg(test)/feature reachability are intentionally conservative. Dynamic include expressions and build-script filesystem reads are fail-closed limitations.",
            "",
            "The Linux `rust_wsl_linux` source/build/exactness binding remains required before a same-domain RSS parity claim; this Windows/MSVC closure contains no Linux executable binding.",
            "",
            "This record is a candidate only. Freeze it from an isolated source/dependency closure after focused integration tests and before the E:-hosted RC build.",
            "",
        ]
    )
    return "\n".join(lines)


def _write_new(path: Path, text: str) -> None:
    if path.exists():
        raise ClosureError(f"refusing to overwrite existing output: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8", newline="\n")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--example", default=DEFAULT_EXAMPLE)
    parser.add_argument("--features", default="", help="comma-separated root feature names")
    feature_mode = parser.add_mutually_exclusive_group()
    feature_mode.add_argument("--no-default-features", dest="no_default_features", action="store_true", default=True)
    feature_mode.add_argument("--default-features", dest="no_default_features", action="store_false")
    parser.add_argument("--target", default=DEFAULT_TARGET)
    parser.add_argument(
        "--cargo-metadata-cross-check",
        action="store_true",
        help="run locked/offline Cargo metadata and compare reachable local packages",
    )
    dev_mode = parser.add_mutually_exclusive_group()
    dev_mode.add_argument(
        "--include-dev-dependencies",
        dest="include_dev_dependencies",
        action="store_true",
        default=True,
        help="include root example dev-dependencies (the canonical example behavior)",
    )
    dev_mode.add_argument(
        "--exclude-dev-dependencies",
        dest="include_dev_dependencies",
        action="store_false",
        help="omit root example dev-dependencies for a library-only closure",
    )
    parser.add_argument("--captured-at")
    parser.add_argument("--output", type=Path, default=Path("-"), help="new JSON path, or - for stdout")
    parser.add_argument("--markdown-output", type=Path)
    parser.add_argument("--verify-manifest", type=Path)
    args = parser.parse_args(argv)
    root = args.root.resolve()
    try:
        if args.verify_manifest is not None:
            path = args.verify_manifest if args.verify_manifest.is_absolute() else root / args.verify_manifest
            # A small generated report is commonly kept under ``work``.  Read
            # that explicitly named file without walking or hashing the
            # excluded bulk tree; closure entries themselves still reject
            # target/work paths during validation.
            path = path.resolve()
            _relative(root, path)
            if not path.is_file():
                raise ClosureError(f"closure manifest is missing: {path}")
            manifest = json.loads(path.read_text(encoding="utf-8"))
            errors = validate_manifest(root, manifest)
            if errors:
                for error in errors:
                    print(f"FAIL {error}")
                return 1
            print(f"verified {path.resolve()}")
            return 0
        features = [item.strip() for item in args.features.split(",") if item.strip()]
        manifest = build_closure(
            root,
            example=args.example,
            features=features,
            no_default_features=args.no_default_features,
            target=args.target,
            include_dev_dependencies=args.include_dev_dependencies,
            cargo_metadata_cross_check=args.cargo_metadata_cross_check,
            captured_at=args.captured_at,
        )
        if args.output == Path("-"):
            print(json.dumps(manifest, indent=2))
        else:
            output = args.output if args.output.is_absolute() else root / args.output
            _write_new(output.resolve(), json.dumps(manifest, indent=2) + "\n")
            print(f"wrote {output.resolve()}")
        if args.markdown_output is not None:
            markdown = args.markdown_output if args.markdown_output.is_absolute() else root / args.markdown_output
            _write_new(markdown.resolve(), render_markdown(manifest))
            print(f"wrote {markdown.resolve()}")
        return 0 if manifest["validation"]["status"] == "pass_static_scan" else 1
    except (ClosureError, OSError, json.JSONDecodeError) as exc:
        print(f"FAIL {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
