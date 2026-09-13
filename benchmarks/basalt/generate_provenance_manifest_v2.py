"""Generate and verify a current Basalt provenance-manifest v2 candidate.

The v1 manifests are historical records and are intentionally never edited by
this module.  A v2 candidate is assembled from the files that exist at the
time of invocation, so it must be generated again after the release source
and executable are frozen.  The default CLI writes JSON to stdout; ``--output``
is an explicit candidate path and refuses to overwrite an existing file.

The generator is deliberately standard-library-only.  It binds production
Rust, Rust tests, protocol/config/calibration/dataset inputs, Phase 6 tools,
the v2 Cargo license inventory, the current golden registry, and (when
provided) an executable plus its build evidence and timing-feature state.
"""

from __future__ import annotations

import argparse
import copy
import datetime as _datetime
import hashlib
import json
import os
from pathlib import Path
from typing import Any, Iterable


ROOT = Path(__file__).resolve().parents[2]
V1_MANIFEST = Path("benchmarks/basalt/basalt_provenance_manifest_v1.json")
V2_SCHEMA = Path("benchmarks/basalt/schemas/provenance_manifest_v2.schema.json")

_REQUIRED_CONTRACTS = {
    "config": Path("configs/basalt/euroc_config.json"),
    "calibration": Path("target/euroc_ds_calib.json"),
    "protocol": Path("benchmarks/basalt/protocols/basalt_euroc_parity_v1.json"),
    "dataset_manifest": Path("benchmarks/basalt/euroc_dataset_manifest.json"),
    "workspace_cargo_toml": Path("Cargo.toml"),
    "basalt_cargo_toml": Path("pipelines/basalt/Cargo.toml"),
    "cargo_lock": Path("Cargo.lock"),
    "coordinator": Path("benchmarks/basalt/phase6_coordinator.py"),
    "native_runner": Path("benchmarks/basalt/native_wsl_runner.py"),
    "rust_runner": Path("benchmarks/basalt/rust_wsl_runner.py"),
    "golden_registry": Path("benchmarks/basalt/m11_current_golden_registry_20260830.json"),
    "license_inventory": Path("benchmarks/basalt/cargo_license_inventory_v2.json"),
    "correctness_certificate": Path("work/m11_release_candidate_final2_correctness_20260831.json"),
    "benchmark_readme": Path("benchmarks/basalt/README.md"),
    "provenance_audit": Path("benchmarks/basalt/PROVENANCE_AUDIT.md"),
    "provenance_generator": Path("benchmarks/basalt/generate_provenance_manifest_v2.py"),
    "provenance_schema": Path("benchmarks/basalt/schemas/provenance_manifest_v2.schema.json"),
    "provenance_tests": Path("benchmarks/basalt/test_provenance_v2.py"),
    "benchmark_notice": Path("benchmarks/basalt/NOTICE"),
    "pipeline_notice": Path("pipelines/basalt/NOTICE"),
}


class ManifestError(ValueError):
    """A candidate cannot be safely generated or verified."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _repo_relative(root: Path, path: Path) -> str:
    try:
        return path.resolve().relative_to(root.resolve()).as_posix()
    except ValueError as exc:
        raise ManifestError(f"path is outside repository root: {path}") from exc


def _checked_file(root: Path, relative: Path | str) -> Path:
    raw = Path(relative)
    path = raw if raw.is_absolute() else root / raw
    path = path.resolve()
    if not path.is_file():
        raise ManifestError(f"required file is missing: {relative}")
    _repo_relative(root, path)
    return path


def _file_record(root: Path, path: Path, *, role: str | None = None) -> dict[str, Any]:
    path = _checked_file(root, path)
    record: dict[str, Any] = {
        "path": _repo_relative(root, path),
        "bytes": path.stat().st_size,
        "sha256": _sha256(path),
    }
    if role is not None:
        record["role"] = role
    return record


def _files_under(root: Path, directory: Path, suffixes: Iterable[str] | None = None) -> list[Path]:
    base = _checked_file(root, directory) if directory.is_file() else (root / directory).resolve()
    if not base.is_dir():
        raise ManifestError(f"required directory is missing: {directory}")
    allowed = set(suffixes) if suffixes is not None else None
    paths = [path for path in base.rglob("*") if path.is_file()]
    if allowed is not None:
        paths = [path for path in paths if path.suffix in allowed]
    return sorted(paths, key=lambda path: _repo_relative(root, path))


def _canonical_tree_digest(records: list[dict[str, Any]]) -> str:
    lines = [f"{record['path']}|{record['sha256'].lower()}" for record in records]
    payload = ("\n".join(sorted(lines)) + "\n").encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise ManifestError(f"cannot load JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ManifestError(f"JSON root is not an object: {path}")
    return value


def _source_binding(root: Path) -> dict[str, Any]:
    paths = _files_under(root, Path("pipelines/basalt/src"), (".rs",))
    files = [_file_record(root, path, role="production_rust") for path in paths]
    return {
        "root": "pipelines/basalt/src",
        "file_count": len(files),
        "hash_algorithm": "sha256",
        "hash_basis": "exact checked-out bytes; no line-ending normalization",
        "canonical_record": "relative POSIX path|lowercase SHA256, newline-joined and sorted",
        "source_tree_sha256": _canonical_tree_digest(files),
        "files": files,
    }


def _test_binding(root: Path) -> dict[str, Any]:
    paths = _files_under(root, Path("pipelines/basalt/tests"))
    files = [_file_record(root, path, role="rust_test_or_fixture") for path in paths]
    return {
        "root": "pipelines/basalt/tests",
        "file_count": len(files),
        "hash_algorithm": "sha256",
        "hash_basis": "exact checked-out bytes; no line-ending normalization",
        "files": files,
    }


def _contract_binding(root: Path) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for name, relative in _REQUIRED_CONTRACTS.items():
        role = {
            "config": "config",
            "calibration": "calibration",
            "protocol": "protocol",
            "dataset_manifest": "dataset_manifest",
            "coordinator": "phase6_coordinator",
            "native_runner": "phase6_native_runner",
            "rust_runner": "phase6_rust_runner",
            "golden_registry": "current_golden_registry",
            "license_inventory": "cargo_license_inventory_v2",
            "correctness_certificate": "correctness_certificate",
            "benchmark_readme": "benchmark_readme",
            "provenance_audit": "provenance_audit",
            "provenance_generator": "provenance_generator",
            "provenance_schema": "provenance_schema",
            "provenance_tests": "provenance_tests",
        }.get(name, "build_input")
        result[name] = _file_record(root, _checked_file(root, relative), role=role)
    return result


def _history_binding(root: Path, base_path: Path) -> dict[str, Any]:
    base = _checked_file(root, base_path)
    old = _load_json(base)
    return {
        "path": _repo_relative(root, base),
        "bytes": base.stat().st_size,
        "sha256": _sha256(base),
        "schema_id": old.get("schema_id"),
        "immutable": True,
        "retained_sections": [
            "upstreams",
            "vcpkg_resolution",
            "m7im_verification",
            "current_metadata",
            "release",
        ],
        "policy": "v1 and its M1/M7 records remain historical; v2 never rewrites them",
    }


def _license_summary(root: Path, record: dict[str, Any]) -> dict[str, Any]:
    inventory = _load_json(_checked_file(root, _REQUIRED_CONTRACTS["license_inventory"]))
    if inventory.get("schema_id") != "visloc-rs.cargo-license.v2":
        raise ManifestError("Cargo license inventory is not schema v2")
    return {
        "path": record["path"],
        "bytes": record["bytes"],
        "sha256": record["sha256"],
        "schema_id": inventory.get("schema_id"),
        "package_count": inventory.get("package_count"),
        "licenses_resolved": inventory.get("licenses_resolved"),
        "licenses_unresolved": inventory.get("licenses_unresolved"),
        "v1_history_preserved": inventory.get("supersedes", {}).get("history_preserved") is True,
        "legal_clearance_claimed": False,
    }


def _golden_summary(record: dict[str, Any]) -> dict[str, Any]:
    return {
        "path": record["path"],
        "bytes": record["bytes"],
        "sha256": record["sha256"],
        "status": "current_registry_candidate",
        "historical_records_immutable": True,
        "dense_abs_h_b_numeric_claim": False,
    }


def _normal_path(value: Any) -> str:
    return Path(str(value).replace("\\", "/")).as_posix().lower()


def _correctness_summary(
    root: Path,
    record: dict[str, Any],
    contracts: dict[str, Any],
    executable: dict[str, Any] | None,
) -> dict[str, Any]:
    """Bind the selected RC certificate and its 52/80/400 exactness claims."""

    certificate = _load_json(_checked_file(root, record["path"]))
    if certificate.get("schema") != "visloc.basalt.release_candidate_correctness.v1":
        raise ManifestError("correctness certificate schema is not v1")
    if certificate.get("status") != "PASS_CORRECTNESS_ONLY":
        raise ManifestError("correctness certificate is not PASS_CORRECTNESS_ONLY")
    scope = certificate.get("scope", {})
    if scope.get("features") != []:
        raise ManifestError("correctness certificate feature list is not empty")
    if scope.get("timing_breakdown") != "compile-time feature disabled":
        raise ManifestError("correctness certificate is not timing-off")
    if scope.get("lm_workspace_reuse") != "feature disabled":
        raise ManifestError("correctness certificate is not LM-reuse-off")

    certificate_executable = certificate.get("build", {}).get("executable", {})
    if not isinstance(certificate_executable, dict):
        raise ManifestError("correctness certificate has no executable binding")
    if executable is not None:
        if _normal_path(certificate_executable.get("path")) != _normal_path(executable.get("path")):
            raise ManifestError("correctness certificate executable path differs from selected executable")
        if certificate_executable.get("sha256", "").lower() != executable.get("sha256", "").lower():
            raise ManifestError("correctness certificate executable hash differs from selected executable")

    certificate_warnings: list[str] = []
    source_binding = certificate.get("source_binding", {})
    source_files = source_binding.get("files", []) if isinstance(source_binding, dict) else []
    if not isinstance(source_files, list) or not source_files:
        raise ManifestError("correctness certificate has no source binding")
    for source_record in source_files:
        if isinstance(source_record, dict):
            errors = _record_errors(root, source_record)
            if errors:
                certificate_warnings.extend(errors)

    input_binding = certificate.get("input_binding", {})
    input_map = {
        "config": "config",
        "calibration": "calibration",
        "dataset_manifest": "dataset_manifest",
        "protocol": "protocol",
    }
    if not isinstance(input_binding, dict):
        raise ManifestError("correctness certificate has no input binding")
    for certificate_name, contract_name in input_map.items():
        certificate_record = (
            certificate.get("protocol_binding", {})
            if certificate_name == "protocol"
            else input_binding.get(certificate_name, {})
        )
        if not isinstance(certificate_record, dict):
            raise ManifestError(f"correctness certificate missing input {certificate_name}")
        if certificate_record.get("sha256", "").lower() != contracts[contract_name]["sha256"].lower():
            certificate_warnings.append(
                f"certificate input hash differs from current contract: {certificate_name}"
            )

    expected_frames = (52, 80, 400)
    replays = certificate.get("replays", [])
    if not isinstance(replays, list):
        raise ManifestError("correctness certificate replays is not a list")
    replay_summary: list[dict[str, Any]] = []
    for frame_count in expected_frames:
        matches = [entry for entry in replays if isinstance(entry, dict) and entry.get("max_frames") == frame_count]
        if len(matches) != 1:
            raise ManifestError(f"correctness certificate must contain one max{frame_count} replay")
        replay = matches[0]
        if replay.get("exit_code") != 0:
            raise ManifestError(f"correctness max{frame_count} replay did not exit successfully")
        summary = replay.get("summary", {})
        if summary.get("frames_requested") != frame_count or summary.get("frames_processed") != frame_count:
            raise ManifestError(f"correctness max{frame_count} frame count is not exact")
        authoritative = replay.get("authoritative", {})
        for key in ("trajectory_csv_byte_exact", "trajectory_tum_byte_exact", "obs_exact", "imu_exact"):
            if authoritative.get(key) is not True:
                raise ManifestError(f"correctness max{frame_count} missing exactness claim: {key}")
        replay_summary.append(
            {
                "name": replay.get("name"),
                "max_frames": frame_count,
                "exit_code": replay.get("exit_code"),
                "frames_processed": summary.get("frames_processed"),
                "observations_emitted": summary.get("observations_emitted"),
                "imu_samples_delivered": summary.get("imu_samples_delivered"),
                "summary_sha256": summary.get("summary_sha256"),
                "trajectory_csv": replay.get("artifacts", {}).get("trajectory_csv"),
                "trajectory_tum": replay.get("artifacts", {}).get("trajectory_tum"),
                "forbidden_absent": replay.get("artifacts", {}).get("forbidden_absent", []),
                "exact": {
                    "trajectory_csv": authoritative.get("trajectory_csv_byte_exact"),
                    "trajectory_tum": authoritative.get("trajectory_tum_byte_exact"),
                    "observations": authoritative.get("obs_exact"),
                    "imu": authoritative.get("imu_exact"),
                },
            }
        )
    return {
        **record,
        "schema": certificate.get("schema"),
        "status": certificate.get("status"),
        "scope": {
            "features": scope.get("features"),
            "timing_breakdown": scope.get("timing_breakdown"),
            "lm_workspace_reuse": scope.get("lm_workspace_reuse"),
            "performance_evaluation": scope.get("performance_evaluation"),
        },
        "executable": {
            "path": certificate_executable.get("path"),
            "bytes": certificate_executable.get("bytes"),
            "sha256": certificate_executable.get("sha256"),
        },
        "source_binding_file_count": len(source_files),
        "certificate_warnings": certificate_warnings,
        "replays": replay_summary,
        "aggregate": certificate.get("aggregate", {}),
    }


def _build_binding(
    root: Path,
    source: dict[str, Any],
    contracts: dict[str, Any],
    *,
    executable: Path | str | None,
    build_evidence: Path | str | None,
    timing_feature: str | None,
    compiler: str | None,
    features: list[str] | None,
    lm_workspace_reuse: str | None,
    build_command: str | None,
    freeze: bool,
) -> dict[str, Any]:
    binding: dict[str, Any] = {
        "status": "pending_executable" if executable is None else "pending_feature_binding",
        "required_for_promotion": [
            "executable_content_sha256",
            "source_tree_sha256",
            "Cargo.toml_sha256",
            "Cargo.lock_sha256",
            "config_sha256",
            "calibration_sha256",
            "protocol_sha256",
            "dataset_manifest_sha256",
            "timing_feature_state",
        ],
        "source_tree_sha256": source["source_tree_sha256"],
        "cargo_toml_sha256": contracts["workspace_cargo_toml"]["sha256"],
        "cargo_lock_sha256": contracts["cargo_lock"]["sha256"],
        "input_sha256": {
            name: contracts[name]["sha256"]
            for name in ("config", "calibration", "protocol", "dataset_manifest")
        },
        "timing_feature": {
            "name": "basalt-timing-breakdown",
            "state": timing_feature if timing_feature in {"enabled", "disabled"} else "unknown",
            "env_is_not_a_substitute_for_compile_feature": True,
        },
        "compiler": {
            "rustc": compiler or "unknown",
            "target": "x86_64-pc-windows-msvc",
            "profile": "release",
        },
        "cargo_features": sorted(features or []),
        "lm_workspace_reuse": lm_workspace_reuse or "unknown",
        "build_command": build_command,
    }
    if executable is not None:
        exe = _file_record(root, Path(executable), role="selected_release_executable")
        binding["executable"] = exe
        if timing_feature not in {"enabled", "disabled"}:
            raise ManifestError("--timing-feature enabled or disabled is required with --executable")
        if freeze and not compiler:
            raise ManifestError("--compiler is required when freezing a provenance manifest")
        if freeze and lm_workspace_reuse not in {"enabled", "disabled"}:
            raise ManifestError("--lm-reuse enabled or disabled is required when freezing")
        if build_evidence is not None:
            binding["build_evidence"] = _file_record(
                root, Path(build_evidence), role="release_build_evidence"
            )
        binding["status"] = "bound"
    else:
        binding["executable"] = None
        binding["build_evidence"] = None
    return binding


def build_candidate(
    root: Path = ROOT,
    *,
    base_manifest: Path | str = V1_MANIFEST,
    captured_at: str | None = None,
    executable: Path | str | None = None,
    build_evidence: Path | str | None = None,
    timing_feature: str | None = None,
    compiler: str | None = None,
    features: list[str] | None = None,
    lm_workspace_reuse: str | None = None,
    build_command: str | None = None,
    correctness_artifact: Path | str | None = None,
    freeze: bool = False,
) -> dict[str, Any]:
    """Build a v2 candidate from current bytes without writing anything."""

    root = root.resolve()
    base_path = Path(base_manifest)
    if not base_path.is_absolute():
        base_path = root / base_path
    base_path = _checked_file(root, base_path)
    base = _load_json(base_path)
    source = _source_binding(root)
    tests = _test_binding(root)
    contracts = _contract_binding(root)
    history = _history_binding(root, base_path)
    correctness_path = correctness_artifact or _REQUIRED_CONTRACTS["correctness_certificate"]
    correctness_record = _file_record(root, _checked_file(root, correctness_path), role="correctness_certificate")
    status = "frozen" if freeze else "current_candidate_not_frozen"
    if freeze and executable is None:
        raise ManifestError("--executable is required when freezing a provenance manifest")
    candidate: dict[str, Any] = {
        "schema_version": 2,
        "schema_id": "visloc-rs.basalt.provenance.v2",
        "manifest_id": "basalt-provenance-v2-frozen" if freeze else "basalt-provenance-v2-candidate",
        "captured_at": captured_at
        or _datetime.datetime.now(_datetime.timezone.utc).replace(microsecond=0).isoformat(),
        "status": status,
        "supersedes": history,
        "upstream_identity": {
            "commit": base.get("upstreams", {}).get("basalt", {}).get("commit"),
            "tree_sha1": base.get("upstreams", {}).get("basalt", {}).get("tree_sha1"),
            "license_spdx": base.get("upstreams", {}).get("basalt", {}).get("license_spdx"),
            "vcpkg": copy.deepcopy(base.get("vcpkg_resolution", {})),
        },
        "current_binding": {
            "source": source,
            "tests": tests,
            "contracts": contracts,
            "build": _build_binding(
                root,
                source,
                contracts,
                executable=executable,
                build_evidence=build_evidence,
                timing_feature=timing_feature,
                compiler=compiler,
                features=features,
                lm_workspace_reuse=lm_workspace_reuse,
                build_command=build_command,
                freeze=freeze,
            ),
            "golden": _golden_summary(contracts["golden_registry"]),
            "license": _license_summary(root, contracts["license_inventory"]),
            "correctness": _correctness_summary(
                root,
                correctness_record,
                contracts,
                (
                    None
                    if not freeze or executable is None
                    else _file_record(root, Path(executable), role="selected_release_executable")
                ),
            ),
        },
        "artifact_policy": {
            "current_v2_is_candidate_until_source_and_executable_are_frozen": not freeze,
            "historical_v1_is_immutable": True,
            "selected_executable_is_evidence_not_distribution_payload": True,
            "ground_truth_artifacts": [],
            "ground_truth_evaluation": "post_run_only",
        },
    }
    return candidate


def _record_errors(root: Path, record: dict[str, Any], *, expected_role: str | None = None) -> list[str]:
    errors: list[str] = []
    relative = record.get("path")
    if not isinstance(relative, str):
        return ["record path is not a string"]
    try:
        path = _checked_file(root, relative)
    except ManifestError as exc:
        return [str(exc)]
    actual_sha = _sha256(path)
    actual_bytes = path.stat().st_size
    if str(record.get("sha256", "")).lower() != actual_sha.lower():
        errors.append(f"hash mismatch: {relative}")
    if record.get("bytes") != actual_bytes:
        errors.append(f"byte-size mismatch: {relative}")
    if expected_role is not None and record.get("role") != expected_role:
        errors.append(f"role mismatch: {relative}")
    return errors


def validate_candidate(
    root: Path,
    candidate: dict[str, Any],
    *,
    require_executable: bool = False,
) -> list[str]:
    """Return errors for a generated candidate; an empty list means valid."""

    root = root.resolve()
    errors: list[str] = []
    if candidate.get("schema_version") != 2:
        errors.append("schema_version must be 2")
    if candidate.get("schema_id") != "visloc-rs.basalt.provenance.v2":
        errors.append("schema_id mismatch")
    status = candidate.get("status")
    if status not in {"current_candidate_not_frozen", "frozen"}:
        errors.append("status must be current_candidate_not_frozen or frozen")
    supersedes = candidate.get("supersedes")
    if not isinstance(supersedes, dict) or supersedes.get("immutable") is not True:
        errors.append("v1 supersedes binding must be immutable")
    else:
        errors.extend(_record_errors(root, supersedes))

    current = candidate.get("current_binding")
    if not isinstance(current, dict):
        return errors + ["current_binding must be an object"]
    source = current.get("source")
    if not isinstance(source, dict):
        errors.append("current_binding.source must be an object")
    else:
        files = source.get("files")
        if not isinstance(files, list) or source.get("file_count") != len(files):
            errors.append("source file_count/files mismatch")
        else:
            for record in files:
                if isinstance(record, dict):
                    errors.extend(_record_errors(root, record, expected_role="production_rust"))
            if source.get("source_tree_sha256") != _canonical_tree_digest(files):
                errors.append("source_tree_sha256 does not match listed source files")
        actual_source_paths = {
            _repo_relative(root, path)
            for path in _files_under(root, Path("pipelines/basalt/src"), (".rs",))
        }
        listed_source_paths = {
            record.get("path") for record in files if isinstance(record, dict)
        }
        if actual_source_paths != listed_source_paths:
            errors.append("production Rust source coverage is incomplete")

    tests = current.get("tests")
    if not isinstance(tests, dict):
        errors.append("current_binding.tests must be an object")
    else:
        records = tests.get("files")
        if not isinstance(records, list) or tests.get("file_count") != len(records):
            errors.append("test file_count/files mismatch")
        else:
            for record in records:
                if isinstance(record, dict):
                    errors.extend(_record_errors(root, record, expected_role="rust_test_or_fixture"))

    contracts = current.get("contracts")
    if not isinstance(contracts, dict):
        errors.append("current_binding.contracts must be an object")
    else:
        for name in _REQUIRED_CONTRACTS:
            record = contracts.get(name)
            if not isinstance(record, dict):
                errors.append(f"missing contract record: {name}")
            else:
                errors.extend(_record_errors(root, record))

    build = current.get("build")
    if not isinstance(build, dict):
        errors.append("current_binding.build must be an object")
    else:
        feature = build.get("timing_feature", {})
        feature_state = feature.get("state") if isinstance(feature, dict) else None
        executable = build.get("executable")
        if executable is None:
            if require_executable:
                errors.append("selected executable is required for promotion")
        elif not isinstance(executable, dict):
            errors.append("build.executable must be a record or null")
        else:
            errors.extend(_record_errors(root, executable, expected_role="selected_release_executable"))
            if feature_state not in {"enabled", "disabled"}:
                errors.append("executable binding has unknown timing feature state")
            if build.get("status") != "bound":
                errors.append("executable binding status is not bound")
        if build.get("status") == "bound" and executable is None:
            errors.append("bound build is missing executable")
        if executable is not None:
            evidence = build.get("build_evidence")
            if evidence is not None and isinstance(evidence, dict):
                errors.extend(_record_errors(root, evidence, expected_role="release_build_evidence"))

        compiler = build.get("compiler")
        if not isinstance(compiler, dict) or not isinstance(compiler.get("rustc"), str):
            errors.append("build.compiler binding is missing")
        if not isinstance(build.get("cargo_features"), list):
            errors.append("build.cargo_features binding is missing")
        if build.get("lm_workspace_reuse") not in {"enabled", "disabled", "unknown"}:
            errors.append("build.lm_workspace_reuse binding is invalid")

        if status == "frozen":
            if build.get("status") != "bound" or executable is None:
                errors.append("frozen manifest requires a bound executable")
            if not isinstance(compiler, dict) or compiler.get("rustc") in {None, "", "unknown"}:
                errors.append("frozen manifest requires compiler identity")
            if build.get("lm_workspace_reuse") not in {"enabled", "disabled"}:
                errors.append("frozen manifest requires LM workspace feature state")

    golden = current.get("golden")
    if not isinstance(golden, dict):
        errors.append("current_binding.golden must be an object")
    else:
        errors.extend(_record_errors(root, golden))
    license_record = current.get("license")
    if not isinstance(license_record, dict):
        errors.append("current_binding.license must be an object")
    else:
        errors.extend(_record_errors(root, license_record))
        if license_record.get("package_count") != 119:
            errors.append("Cargo license package_count must be 119")
        if license_record.get("licenses_resolved") != 119:
            errors.append("Cargo license licenses_resolved must be 119")
        if license_record.get("licenses_unresolved") != 0:
            errors.append("Cargo license licenses_unresolved must be zero")
    correctness = current.get("correctness")
    if not isinstance(correctness, dict):
        errors.append("current_binding.correctness must be an object")
    else:
        errors.extend(_record_errors(root, correctness, expected_role="correctness_certificate"))
        if correctness.get("schema") != "visloc.basalt.release_candidate_correctness.v1":
            errors.append("correctness certificate schema mismatch")
        if correctness.get("status") != "PASS_CORRECTNESS_ONLY":
            errors.append("correctness certificate status mismatch")
        if status == "frozen":
            replay_frames = {
                replay.get("max_frames")
                for replay in correctness.get("replays", [])
                if isinstance(replay, dict)
            }
            if replay_frames != {52, 80, 400}:
                errors.append("frozen manifest correctness coverage must be exactly 52/80/400")
    if candidate.get("artifact_policy", {}).get("ground_truth_artifacts") != []:
        errors.append("ground_truth_artifacts must be empty")
    policy_candidate = candidate.get("artifact_policy", {}).get(
        "current_v2_is_candidate_until_source_and_executable_are_frozen"
    )
    if policy_candidate is not (status != "frozen"):
        errors.append("artifact policy candidate/frozen state mismatch")
    return errors


def _write_new(path: Path, payload: dict[str, Any]) -> None:
    if path.exists():
        raise ManifestError(f"refusing to overwrite existing candidate: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = json.dumps(payload, indent=2, sort_keys=False) + "\n"
    path.write_text(encoded, encoding="utf-8", newline="\n")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--output", type=Path, default=Path("-"), help="new candidate path, or - for stdout")
    parser.add_argument("--base-manifest", type=Path, default=V1_MANIFEST)
    parser.add_argument("--captured-at")
    parser.add_argument("--executable", type=Path)
    parser.add_argument("--build-evidence", type=Path)
    parser.add_argument("--timing-feature", choices=("enabled", "disabled"))
    parser.add_argument("--compiler", help="captured rustc/compiler identity for an executable binding")
    parser.add_argument(
        "--features",
        default="",
        help="comma-separated Cargo feature names (empty means --no-default-features)",
    )
    parser.add_argument("--lm-reuse", choices=("enabled", "disabled", "unknown"))
    parser.add_argument("--build-command")
    parser.add_argument("--correctness-artifact", type=Path)
    parser.add_argument(
        "--freeze",
        action="store_true",
        help="emit a frozen release binding; requires executable/compiler/feature evidence",
    )
    parser.add_argument(
        "--verify",
        action="store_true",
        help="verify the generated candidate; executable is required unless --allow-pending is set",
    )
    parser.add_argument("--allow-pending", action="store_true", help="allow a candidate without an executable")
    parser.add_argument(
        "--verify-manifest",
        type=Path,
        help="verify an existing v2 manifest instead of constructing a new candidate",
    )
    args = parser.parse_args(argv)
    root = args.root.resolve()
    try:
        if args.verify_manifest is not None:
            manifest_path = args.verify_manifest
            if not manifest_path.is_absolute():
                manifest_path = root / manifest_path
            manifest = _load_json(_checked_file(root, manifest_path))
            errors = validate_candidate(
                root,
                manifest,
                require_executable=not args.allow_pending,
            )
            if errors:
                for error in errors:
                    print(f"FAIL {error}", file=os.sys.stderr)
                return 1
            print(f"verified {manifest_path.resolve()}")
            return 0
        features = [feature.strip() for feature in args.features.split(",") if feature.strip()]
        candidate = build_candidate(
            root,
            base_manifest=args.base_manifest,
            captured_at=args.captured_at,
            executable=args.executable,
            build_evidence=args.build_evidence,
            timing_feature=args.timing_feature,
            compiler=args.compiler,
            features=features,
            lm_workspace_reuse=args.lm_reuse,
            build_command=args.build_command,
            correctness_artifact=args.correctness_artifact,
            freeze=args.freeze,
        )
        if args.verify:
            errors = validate_candidate(
                root,
                candidate,
                require_executable=not args.allow_pending,
            )
            if errors:
                for error in errors:
                    print(f"FAIL {error}", file=os.sys.stderr)
                return 1
        if args.output == Path("-"):
            print(json.dumps(candidate, indent=2))
        else:
            output = args.output if args.output.is_absolute() else root / args.output
            _write_new(output.resolve(), candidate)
            print(f"wrote {output.resolve()}")
        return 0
    except ManifestError as exc:
        print(f"FAIL {exc}", file=os.sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
