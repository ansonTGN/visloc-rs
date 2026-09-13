"""Cache-aware, direct-path EuRoC all11 x 1 coordinator.

This coordinator is intentionally separate from :mod:`batch`.  ``batch``
implements the frozen M10 staged-input contract; this module implements the
Phase 6 matrix contract, where every cell opens the already-existing EuRoC
sequence (including an ``all11`` junction tree) directly.  No sensor bytes
are copied into a cell directory.

The module has two useful properties for long experiments:

* a plan is deterministic and can be written without creating an output tree;
* each successful cell has a request fingerprint and atomically-written
  manifest/evaluation documents, so a later invocation skips only a matching
  completed cell.

Ground truth is resolved only after the engine process exits.  It is never a
command argument or environment value for the engine process.  Strict Phase 6
mode uses a sensor-only view: each sequence directory has only
``mav0/cam0``, ``mav0/cam1``, and ``mav0/imu0`` directory junctions.  The
post-exit evaluator resolves GT from the manifest's physical sequence root;
that path is deliberately absent from the engine command, environment, and
request document.

Formal Phase 6 cells use the ``trajectory_only_lean`` output policy: native
does not receive ``--marg-data`` and Rust receives ``--no-marg-data
--no-trace``.  A requested representative is explicitly marked as a
separate ``representative_diagnostic`` cell and is never silently mixed into
the formal lean policy.
"""

from __future__ import annotations

import argparse
import csv
import datetime as _dt
import hashlib
import json
import os
import platform
import re
import shutil
import signal
import stat
from concurrent.futures import ThreadPoolExecutor, as_completed
import subprocess
import sys
import time
import uuid
from copy import deepcopy
from dataclasses import dataclass, replace
from pathlib import Path
from statistics import median
from typing import Any, Iterable, Mapping, Sequence

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_PROTOCOL = ROOT / "benchmarks" / "basalt" / "protocols" / "basalt_euroc_parity_v1.json"
DEFAULT_DATASET_MANIFEST = ROOT / "benchmarks" / "basalt" / "euroc_dataset_manifest.json"
DEFAULT_CONFIG = ROOT / "configs" / "basalt" / "euroc_config.json"
DEFAULT_CALIBRATION = ROOT / "benchmarks" / "basalt" / "release_inputs" / "euroc_ds_calib.json"
DEFAULT_RUST_EXECUTABLE = ROOT / "target" / "release" / "examples" / "basalt_euroc_vio_demo.exe"
DEFAULT_NATIVE_ORACLE_ROOT = "/root/visloc-basalt-clean-m7cr-20260823"
DEFAULT_NATIVE_BUILD = f"{DEFAULT_NATIVE_ORACLE_ROOT}/build/core-relwithdebinfo/basalt_vio"
DEFAULT_NATIVE_CONFIG = f"{DEFAULT_NATIVE_ORACLE_ROOT}/data/euroc_config.json"
DEFAULT_NATIVE_CALIBRATION = f"{DEFAULT_NATIVE_ORACLE_ROOT}/data/euroc_ds_calib.json"
DEFAULT_NATIVE_WRAPPER = ROOT / "benchmarks" / "basalt" / "native_wsl_runner.py"
DEFAULT_RUST_WSL_EXECUTABLE = "/root/visloc-rs/target/release/examples/basalt_euroc_vio_demo"
DEFAULT_RUST_WSL_CONFIG = "/root/visloc-rs/configs/basalt/euroc_config.json"
DEFAULT_RUST_WSL_CALIBRATION = "/root/visloc-rs/target/euroc_ds_calib.json"
DEFAULT_RUST_WSL_WRAPPER = ROOT / "benchmarks" / "basalt" / "rust_wsl_runner.py"

COORDINATOR_SCHEMA_ID = "basalt.phase6.coordinator.v1"
RUN_SCHEMA_ID = "basalt.phase6.run.v1"
EVALUATION_SCHEMA_ID = "basalt.phase6.evaluation.v1"
SESSION_SCHEMA_ID = "basalt.phase6.session.v1"
RUNTIME_CONTEXT_SCHEMA_ID = "basalt.phase6.runtime_context.v1"
SESSION_FILE_NAME = "phase6_session.json"
TIMING_FEATURE_NAME = "basalt-timing-breakdown"
TIMING_ENV_NAME = "VISLOC_BASALT_TIMING_BREAKDOWN"
COORDINATOR_VERSION = 1
PROTOCOL_ID = "basalt-euroc-parity-v1"
METHOD_NATIVE = "native_core"
METHOD_RUST = "rust_current"
METHODS = (METHOD_NATIVE, METHOD_RUST)
OUTPUT_POLICY_LEAN = "trajectory_only_lean"
OUTPUT_POLICY_DIAGNOSTIC = "representative_diagnostic"
DEFAULT_REPETITIONS = 1
DEFAULT_POLL_SECONDS = 0.25
DEFAULT_TIMEOUT_SECONDS = 12 * 60 * 60
DEFAULT_SEED = 7
GT_MARKERS = (
    "state_groundtruth_estimate0",
    "ground_truth",
    "ground-truth",
    "groundtruth",
    "gt_path",
    "gt_file",
    "gt_csv",
)
SENSOR_ONLY_COMPONENTS = ("cam0", "cam1", "imu0")
SENSOR_VIEW_MODE_DIRECT = "direct"
SENSOR_VIEW_MODE_JUNCTION = "junction"
SENSOR_VIEW_MODE_HARDLINK = "hardlink_staged"
NATIVE_MONITOR_SCHEMA = "basalt.native_wsl_monitor.v1"
RUST_WSL_MONITOR_SCHEMA = "basalt.rust_wsl_monitor.v1"
RUST_WSL_PROVENANCE_SCHEMA = "basalt.phase6.rust_wsl_provenance.v1"
RUST_WSL_EXACTNESS_SCHEMA = "basalt.phase6.rust_wsl_exactness.v1"
RUST_RUNTIME_PROFILE_MSVC = "msvc_windows"
RUST_RUNTIME_PROFILE_WSL = "rust_wsl_linux"
RUST_RUNTIME_PROFILES = (RUST_RUNTIME_PROFILE_MSVC, RUST_RUNTIME_PROFILE_WSL)
NATIVE_UPSTREAM_TREE_SHA1 = "b7afb830d82b45b8209cf784ad9744025d838411"
PROTOCOL_SEQUENCE_IDS = (
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
FORMAL_MATRIX_CELL_COUNT = len(METHODS) * len(PROTOCOL_SEQUENCE_IDS) * DEFAULT_REPETITIONS
RESUME_IDENTITY_SCHEMA_ID = "basalt.phase6.resume_identity.v1"
INPUT_NAMESPACE_SCHEMA_ID = "basalt.phase6.input_namespace.v1"
RSS_AUTHORITATIVE_DOMAIN = "linux-proc-process-tree"
# Both engines are launched by this Windows coordinator.  This is the only
# common wall-clock domain available when native runs through the WSL shim and
# Rust runs as an MSVC process.  Native inner-Linux wall time remains a
# diagnostic field; it must not be compared directly with Rust wall time.
RUNTIME_COMMON_MEASUREMENT_DOMAIN = "windows-coordinator-process-tree-command"
_UTC = getattr(_dt, "UTC", _dt.timezone.utc)


class CoordinatorError(ValueError):
    """Raised for an unsafe or internally inconsistent coordinator request."""


def utc_now() -> str:
    return _dt.datetime.now(_UTC).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_hash(value: Any) -> str:
    payload = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _valid_sha256(value: Any) -> bool:
    return isinstance(value, str) and re.fullmatch(r"[0-9a-fA-F]{64}", value) is not None


def _resume_identity(request: Mapping[str, Any]) -> dict[str, Any]:
    """Return the per-cell identity that makes a resume safe.

    ``request_sha256`` already covers this structure, but keeping the fields
    explicit in both the request and the terminal manifest makes a stale
    artifact auditable without having to trust a hash alone.  The Rust SHA is
    session-wide (and is therefore also present on native cells in a paired
    plan), while the pair id/position are cell-specific.
    """

    runtime = request.get("runtime_context")
    runtime = runtime if isinstance(runtime, Mapping) else {}
    timing = request.get("timing_feature_binding")
    timing = timing if isinstance(timing, Mapping) else {}
    rust_sha = request.get("rust_executable_sha256")
    if rust_sha is None:
        rust_sha = timing.get("rust_executable_sha256")
    return {
        "schema_id": RESUME_IDENTITY_SCHEMA_ID,
        "session_id": runtime.get("session_id"),
        "pair_id": runtime.get("pair_id"),
        "pair_position": runtime.get("pair_position"),
        "pair_size": runtime.get("pair_size"),
        "method_order": runtime.get("method_order"),
        "cache_context_id": runtime.get("cache_context_id"),
        "rust_runtime_profile": request.get(
            "rust_runtime_profile", RUST_RUNTIME_PROFILE_MSVC
        ),
        "rss_gate_candidate_eligible": request.get(
            "rss_gate_candidate_eligible", False
        ),
        "rust_executable_sha256": rust_sha,
    }


def _input_namespace(request: Mapping[str, Any]) -> dict[str, Any]:
    """Describe and bind the complete input population used by the gate."""

    dataset = request.get("dataset")
    dataset = dataset if isinstance(dataset, Mapping) else {}
    namespace = request.get("input_namespace")
    namespace = namespace if isinstance(namespace, Mapping) else {}
    sensor_only = bool(request.get("sensor_only_input_view", dataset.get("sensor_only_input_view", False)))
    mode = str(
        request.get(
            "sensor_input_view_mode",
            dataset.get(
                "sensor_input_view_mode",
                SENSOR_VIEW_MODE_JUNCTION if sensor_only else SENSOR_VIEW_MODE_DIRECT,
            ),
        )
    )
    staged = bool(request.get("staged_sensor_only", dataset.get("staged_sensor_only", False)))
    direct = bool(request.get("direct_dataset_no_sensor_copy", True))
    fingerprints = namespace.get("sensor_view_fingerprints", dataset.get("sensor_view_fingerprints", {}))
    if not isinstance(fingerprints, Mapping):
        fingerprints = {}
    fingerprints = {
        str(sequence): str(fingerprint).casefold()
        for sequence, fingerprint in sorted(fingerprints.items(), key=lambda item: str(item[0]))
        if _valid_sha256(fingerprint)
    }
    return {
        "schema_id": INPUT_NAMESPACE_SCHEMA_ID,
        "mode": "sensor_only" if sensor_only else "direct",
        "sensor_input_view_mode": mode,
        "direct_dataset_no_sensor_copy": direct,
        "staged_sensor_only": staged,
        "sensor_view_fingerprints": fingerprints,
        "strict_gt_absence_verified": bool(namespace.get("strict_gt_absence_verified", False)),
        "gt_absence_fingerprint": namespace.get("gt_absence_fingerprint"),
    }


def _runtime_session_id(value: Any) -> str:
    """Validate the opaque id that binds one formal matrix session."""

    text = str(value).strip()
    if re.fullmatch(r"[0-9a-fA-F]{32,64}", text) is None:
        raise CoordinatorError("runtime session id must be a 32-64 character hexadecimal value")
    return text.casefold()


def _load_session_record(output_root: Path) -> dict[str, Any] | None:
    path = Path(output_root) / SESSION_FILE_NAME
    if not path.is_file():
        return None
    try:
        document = _load_json(path)
    except (CoordinatorError, OSError, ValueError, json.JSONDecodeError) as exc:
        raise CoordinatorError(f"invalid phase6 session record: {path}: {exc}") from exc
    if document.get("schema_id") != SESSION_SCHEMA_ID:
        raise CoordinatorError(f"phase6 session schema mismatch: {path}")
    session_id = document.get("session_id")
    if session_id is None:
        raise CoordinatorError(f"phase6 session has no session_id: {path}")
    document["session_id"] = _runtime_session_id(session_id)
    cache_context_id = document.get("cache_context_id")
    if not _valid_sha256(cache_context_id):
        raise CoordinatorError(f"phase6 session has no valid cache_context_id: {path}")
    return document


def _timing_env_mode(environment: Mapping[str, Any] | None = None) -> str:
    """Return the exact timing env contract used by ``TimingBreakdown``."""

    values = os.environ if environment is None else environment
    raw = values.get(TIMING_ENV_NAME)
    if raw is None:
        return "unset"
    normalized = str(raw).strip().casefold()
    if normalized == "1":
        return "requested"
    if normalized in {"", "0", "false", "off", "no"}:
        return "disabled"
    return "invalid"


def _normalize_timing_feature_state(
    value: Any,
    *,
    env_mode: str,
) -> str:
    """Resolve a declared Cargo feature state and enforce env-only timing."""

    if env_mode == "invalid":
        raise CoordinatorError(
            f"{TIMING_ENV_NAME} must be unset/0 or exactly 1; refusing an ambiguous value"
        )
    if value is None:
        state = "unknown" if env_mode == "requested" else "disabled"
    elif isinstance(value, bool):
        state = "enabled" if value else "disabled"
    else:
        state = str(value).strip().casefold()
    if state not in {"enabled", "disabled"}:
        raise CoordinatorError(
            f"Cargo feature {TIMING_FEATURE_NAME} state must be enabled or disabled, got {value!r}"
        )
    if env_mode == "requested" and state != "enabled":
        raise CoordinatorError(
            f"{TIMING_ENV_NAME}=1 requires Cargo feature {TIMING_FEATURE_NAME}=enabled; "
            f"declared state is {state}"
        )
    return state


def _validate_timing_feature_binding(
    binding: Any,
    *,
    method: str,
    environment: Mapping[str, Any],
) -> None:
    """Validate the timing feature/env contract again immediately before spawn."""

    if not isinstance(binding, Mapping) or binding.get("feature") != TIMING_FEATURE_NAME:
        raise CoordinatorError("missing or unknown Cargo timing feature binding")
    state = binding.get("state")
    if state not in {"enabled", "disabled"}:
        raise CoordinatorError("unknown Cargo timing feature state")
    current_mode = _timing_env_mode(environment)
    if current_mode == "invalid":
        raise CoordinatorError(
            f"{TIMING_ENV_NAME} has an invalid value at engine launch"
        )
    if current_mode != binding.get("environment_mode"):
        raise CoordinatorError(
            f"{TIMING_ENV_NAME} mode changed after planning; refusing stale timing feature binding"
        )
    if current_mode == "requested" and state != "enabled":
        raise CoordinatorError(
            f"{TIMING_ENV_NAME}=1 requires Cargo feature {TIMING_FEATURE_NAME}=enabled"
        )
    if method == METHOD_RUST and not _valid_sha256(binding.get("rust_executable_sha256")):
        raise CoordinatorError(
            f"Rust timing feature binding has no executable SHA-256 for {TIMING_FEATURE_NAME}"
        )


def atomic_write_json(path: Path, value: Any) -> None:
    """Write JSON using a same-directory temporary file and ``os.replace``."""

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.{uuid.uuid4().hex}.tmp")
    try:
        temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        # Windows rejects ``fsync`` on a read-only descriptor (errno 9).
        # Re-open read/write so the atomic durability step works on both
        # Windows and POSIX hosts.
        with temporary.open("r+b") as stream:
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def _safe_component(value: str, label: str) -> str:
    value = str(value)
    if not value or value in {".", ".."} or Path(value).name != value:
        raise CoordinatorError(f"{label} must be one path component: {value!r}")
    if any(char in value for char in ("/", "\\", ":")):
        raise CoordinatorError(f"{label} must not contain path separators: {value!r}")
    return value


def family_for_sequence(sequence: str) -> str:
    value = str(sequence).strip()
    lowered = value.casefold()
    if lowered.startswith("mh_"):
        return "machine_hall"
    if lowered.startswith("v1_") or lowered.startswith("v2_"):
        return "vicon_room"
    raise CoordinatorError(f"cannot infer EuRoC family for {sequence!r}")


def family_selector(value: str) -> set[str]:
    """Resolve MH/V1/V2 and physical-family selectors to sequence prefixes."""

    selected: set[str] = set()
    for raw in str(value).split(","):
        token = raw.strip().casefold()
        if not token:
            continue
        if token in {"mh", "machine_hall", "machine-hall"}:
            selected.add("mh")
        elif token in {"v1", "v1_", "vicon1", "vicon_room1", "vicon-room1"}:
            selected.add("v1")
        elif token in {"v2", "v2_", "vicon2", "vicon_room2", "vicon-room2"}:
            selected.add("v2")
        elif token in {"v", "vicon", "vicon_room", "vicon-room"}:
            selected.update({"v1", "v2"})
        else:
            raise CoordinatorError(f"unsupported sequence family selector: {raw!r}")
    return selected


def _lexical_absolute(path: Path) -> Path:
    """Make a path absolute without resolving Windows junctions."""

    path = Path(path).expanduser()
    return path if path.is_absolute() else Path.cwd() / path


def _canonical_executable(value: Path | str, *, allow_wsl_path: bool = False) -> str:
    """Bind an executable to an absolute path without changing WSL paths."""

    raw = str(value)
    if allow_wsl_path and raw.startswith("/"):
        return raw
    path = _lexical_absolute(Path(raw)).resolve(strict=False)
    if not path.is_absolute():
        raise CoordinatorError(f"executable path is not absolute: {value!r}")
    return str(path)


def _normalize_rust_runtime_profile(value: Any) -> str:
    """Normalize the Rust execution target used by a Phase 6 plan.

    ``msvc_windows`` is the production/default target.  ``rust_wsl_linux`` is
    an explicitly separate measurement twin: it never silently substitutes for
    the MSVC executable and can only become a resource-gate candidate after a
    bound exactness certificate is supplied.
    """

    if value is None:
        return RUST_RUNTIME_PROFILE_MSVC
    normalized = str(value).strip().casefold()
    aliases = {
        "msvc": RUST_RUNTIME_PROFILE_MSVC,
        "windows": RUST_RUNTIME_PROFILE_MSVC,
        "windows-msvc": RUST_RUNTIME_PROFILE_MSVC,
        "rust_msvc_windows": RUST_RUNTIME_PROFILE_MSVC,
        "wsl": RUST_RUNTIME_PROFILE_WSL,
        "linux": RUST_RUNTIME_PROFILE_WSL,
        "wsl-linux": RUST_RUNTIME_PROFILE_WSL,
        "rust-wsl-linux": RUST_RUNTIME_PROFILE_WSL,
    }
    normalized = aliases.get(normalized, normalized)
    if normalized not in RUST_RUNTIME_PROFILES:
        raise CoordinatorError(
            f"unsupported Rust runtime profile {value!r}; expected one of {RUST_RUNTIME_PROFILES}"
        )
    return normalized


def _wsl_path(value: Path | str) -> str:
    """Return a stable WSL spelling for a Windows or already-WSL path."""

    raw = str(value)
    if raw.startswith("/"):
        return raw
    return windows_to_wsl(Path(raw))


def resolve_direct_sequence_root(
    dataset_root: Path,
    sequence: str,
    *,
    family: str | None = None,
    require_exists: bool = True,
) -> Path:
    """Resolve a sequence while preserving the lexical junction path.

    Unlike ``Path.resolve()``, the returned path is the path passed to the
    engine.  This matters for ``<base>/all11/<sequence>`` junctions: the
    manifest records both this direct path and its physical target, but the
    engine receives the direct path and no staged sensor tree is created.
    """

    root = _lexical_absolute(Path(dataset_root))
    sequence = _safe_component(sequence, "sequence")
    family = family or family_for_sequence(sequence)
    if family not in {"machine_hall", "vicon_room"}:
        raise CoordinatorError(f"unsupported family {family!r}")

    candidates: list[Path] = []
    name = root.name.casefold()
    if name == sequence.casefold():
        candidates.append(root)
    elif name == "all11":
        candidates.append(root / sequence)
        base = root.parent
        if family == "machine_hall":
            candidates.append(base / "machine_hall" / sequence)
        else:
            candidates.append(base / "vicon_room" / "sequences" / sequence)
    elif name == "machine_hall":
        candidates.append(root / sequence)
    elif name == "vicon_room":
        candidates.extend((root / "sequences" / sequence, root / sequence))
    elif name == "sequences":
        candidates.append(root / sequence)
    else:
        candidates.extend(
            (
                root / ("machine_hall" if family == "machine_hall" else "vicon_room") / ("" if family == "machine_hall" else "sequences") / sequence,
                root / "all11" / sequence,
                root / sequence,
            )
        )
    # The conditional expression above is readable for normal paths, but the
    # all11 branch needs explicit handling to avoid string/Path precedence.
    if name == "all11":
        base = root.parent
        candidates = [root / sequence]
        if family == "machine_hall":
            candidates.append(base / "machine_hall" / sequence)
        else:
            candidates.append(base / "vicon_room" / "sequences" / sequence)

    unique: list[Path] = []
    seen: set[str] = set()
    for candidate in candidates:
        candidate = _lexical_absolute(candidate)
        key = os.path.normcase(str(candidate))
        if key not in seen:
            seen.add(key)
            unique.append(candidate)
    if require_exists:
        for candidate in unique:
            if candidate.is_dir():
                return candidate
        raise CoordinatorError(
            f"sequence root does not exist for {sequence}: {', '.join(map(str, unique))}"
        )
    return unique[0]


def _flatten(values: Iterable[str] | None) -> list[str]:
    flattened: list[str] = []
    for value in values or ():
        flattened.extend(item.strip() for item in str(value).split(",") if item.strip())
    return flattened


def select_sequences(
    protocol_sequences: Sequence[str],
    *,
    sequences: Iterable[str] | None = None,
    families: Iterable[str] | None = None,
) -> list[str]:
    ordered = list(protocol_sequences)
    if not ordered or len(ordered) != len(set(ordered)):
        raise CoordinatorError("protocol sequence list must be non-empty and unique")
    exact = _flatten(sequences)
    unknown = [item for item in exact if item not in ordered]
    if unknown:
        raise CoordinatorError(f"sequences are not in protocol: {unknown!r}")
    family_prefixes: set[str] = set()
    for family in _flatten(families):
        family_prefixes.update(family_selector(family))
    if not exact and not family_prefixes:
        return ordered
    selected = []
    for sequence in ordered:
        lowered = sequence.casefold()
        prefix = "mh" if lowered.startswith("mh_") else "v1" if lowered.startswith("v1_") else "v2" if lowered.startswith("v2_") else ""
        if sequence in exact or prefix in family_prefixes:
            selected.append(sequence)
    if not selected:
        raise CoordinatorError("sequence/family selectors matched no protocol sequence")
    return selected


def _load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(Path(path).read_text(encoding="utf-8"))
    except OSError as exc:
        raise CoordinatorError(f"cannot read JSON {path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise CoordinatorError(f"invalid JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise CoordinatorError(f"expected JSON object: {path}")
    return value


def _manifest_records(manifest: Mapping[str, Any] | None) -> dict[str, dict[str, Any]]:
    records = (manifest or {}).get("sequences", [])
    output: dict[str, dict[str, Any]] = {}
    for item in records if isinstance(records, list) else []:
        if isinstance(item, dict) and isinstance(item.get("id"), str):
            output[str(item["id"])] = item
    return output


def _expected_frames(record: Mapping[str, Any]) -> int:
    cameras = record.get("cameras")
    if isinstance(cameras, Mapping):
        cam0 = cameras.get("cam0")
        if isinstance(cam0, Mapping):
            for key in ("row_count", "csv_row_count"):
                if isinstance(cam0.get(key), int):
                    return int(cam0[key])
    counts = record.get("camera_counts")
    if isinstance(counts, Mapping) and isinstance(counts.get("cam0"), Mapping):
        value = counts["cam0"].get("csv_rows")
        if isinstance(value, int):
            return int(value)
    return 0


def _sensor_fingerprint(record: Mapping[str, Any]) -> str | None:
    hashes = record.get("file_hashes")
    if not isinstance(hashes, Mapping):
        return None
    selected: dict[str, Any] = {}
    for group in ("csv_sha256", "sensor_yaml_sha256"):
        values = hashes.get(group)
        if isinstance(values, Mapping):
            selected[group] = {
                str(path): str(value)
                for path, value in sorted(values.items())
                if "groundtruth" not in str(path).casefold()
            }
    return canonical_hash(selected) if selected else None


def windows_to_wsl(path: Path | str) -> str:
    value = str(path)
    match = re.match(r"^([A-Za-z]):[\\/](.*)$", value)
    if match:
        return f"/mnt/{match.group(1).lower()}/{match.group(2).replace(chr(92), '/')}"
    if value.startswith("\\\\wsl$\\"):
        parts = value.lstrip("\\").split("\\")
        if len(parts) >= 2:
            return "/" + "/".join(parts[2:])
    return value.replace("\\", "/")


def _contains_gt_token(value: str) -> bool:
    lowered = str(value).casefold()
    return any(marker in lowered for marker in GT_MARKERS)


def _normalize_sensor_input_view_mode(
    value: str | None,
    *,
    sensor_only_input_view: bool,
) -> tuple[str, bool]:
    """Normalize the engine-visible sensor view contract.

    ``hardlink_staged`` and ``junction`` are strict sensor-only namespaces;
    selecting either mode implicitly enables the sensor-only firewall.  A
    legacy direct run keeps the explicit ``direct`` mode and does not inspect
    a manifest physical root.
    """

    if value is None or not str(value).strip():
        return (SENSOR_VIEW_MODE_JUNCTION if sensor_only_input_view else SENSOR_VIEW_MODE_DIRECT), bool(
            sensor_only_input_view
        )
    mode = str(value).strip().casefold()
    aliases = {
        "hardlink": SENSOR_VIEW_MODE_HARDLINK,
        "hardlink-staged": SENSOR_VIEW_MODE_HARDLINK,
        "hardlink_staged": SENSOR_VIEW_MODE_HARDLINK,
        "junction": SENSOR_VIEW_MODE_JUNCTION,
        "direct": SENSOR_VIEW_MODE_DIRECT,
    }
    mode = aliases.get(mode, mode)
    if mode not in {SENSOR_VIEW_MODE_DIRECT, SENSOR_VIEW_MODE_JUNCTION, SENSOR_VIEW_MODE_HARDLINK}:
        raise CoordinatorError(
            "sensor input view mode must be direct, junction, or hardlink_staged"
        )
    if mode == SENSOR_VIEW_MODE_DIRECT and sensor_only_input_view:
        raise CoordinatorError("direct sensor input view cannot enable sensor-only mode")
    if mode != SENSOR_VIEW_MODE_DIRECT:
        return mode, True
    return mode, False


def _sensor_mode_for_root(
    dataset_root: Path,
    value: str | None,
    *,
    sensor_only_input_view: bool,
) -> tuple[str, bool]:
    """Normalize a view mode, inferring the canonical hardlink namespace.

    The command-line contract historically required only
    ``--sensor-only-input-view``.  Keep that spelling compatible while making
    the explicitly named ``sensor_hardlink_all11`` root select the hardlink
    validator automatically.  Other roots retain the legacy junction default.
    """

    if value is None and sensor_only_input_view:
        if Path(dataset_root).name.casefold() == "sensor_hardlink_all11":
            value = SENSOR_VIEW_MODE_HARDLINK
    return _normalize_sensor_input_view_mode(value, sensor_only_input_view=sensor_only_input_view)


def _is_reparse_point(path: Path, metadata: os.stat_result | None = None) -> bool:
    """Return whether *path* is a symlink/junction or Windows reparse point."""

    try:
        if _is_directory_link(path):
            return True
    except OSError:
        return True
    try:
        metadata = metadata or path.lstat()
    except OSError:
        return True
    # FILE_ATTRIBUTE_REPARSE_POINT is 0x400.  ``st_file_attributes`` is
    # Windows-specific, so use getattr for POSIX test fixtures.
    return bool(getattr(metadata, "st_file_attributes", 0) & 0x400)


def _is_directory_link(path: Path) -> bool:
    """Return whether *path* is a directory junction/symlink.

    Python 3.12 exposes ``Path.is_junction`` on Windows.  The symlink fallback
    keeps the structural validator testable on POSIX without weakening the
    Windows junction requirement used by the real Phase 6 view.
    """

    try:
        return bool(path.is_junction() or path.is_symlink())
    except OSError:
        return False


def _directory_link_target(path: Path) -> Path:
    """Read and normalize a directory-link target without traversing it."""

    try:
        raw = os.readlink(path)
    except OSError:
        return _lexical_absolute(path.resolve(strict=False))
    # Windows junctions commonly return a ``\\?\\``-prefixed target.
    if raw.startswith("\\\\?\\"):
        raw = raw[4:]
    target = Path(raw)
    if not target.is_absolute():
        target = path.parent / target
    return _lexical_absolute(target)


def _physical_root_from_record(record: Mapping[str, Any]) -> Path:
    """Return the manifest-bound physical sequence root, without reading GT."""

    physical = record.get("physical_root")
    value = physical.get("path") if isinstance(physical, Mapping) else None
    if not isinstance(value, str) or not value.strip():
        raise CoordinatorError("sensor-only runs require manifest physical_root.path")
    raw_root = Path(value)
    if not raw_root.is_absolute():
        raise CoordinatorError("manifest physical_root.path must be absolute")
    root = _lexical_absolute(raw_root)
    if not root.is_dir():
        raise CoordinatorError(f"manifest physical_root.path is unavailable: {root}")
    if _contains_gt_token(str(root)):
        raise CoordinatorError("manifest physical_root.path contains a ground-truth token")
    return root


def _sensor_target_candidates(source_root: Path, sequence: str, record: Mapping[str, Any], sensor: str) -> dict[str, Path]:
    """Build the only legal sensor junction targets for one view entry."""

    candidates: dict[str, Path] = {}
    # The canonical view is sibling to ``all11`` and targets that direct
    # junction tree.  A physical target is also accepted for portable views.
    direct_root = source_root.parent / "all11" / sequence / "mav0" / sensor
    candidates["direct_all11"] = _lexical_absolute(direct_root)
    physical_root = _physical_root_from_record(record)
    if physical_root.name.casefold() != sequence.casefold():
        raise CoordinatorError(
            f"manifest physical_root.path sequence mismatch: {physical_root} != {sequence}"
        )
    candidates["physical_root"] = _lexical_absolute(physical_root / "mav0" / sensor)
    return candidates


def _sensor_file_inventory(root: Path, *, label: str) -> tuple[set[str], dict[str, tuple[Path, os.stat_result]]]:
    """Collect a sensor tree without following links or reparse points."""

    root = _lexical_absolute(root)
    if not root.is_dir() or _is_directory_link(root):
        raise CoordinatorError(f"{label} must be a real directory: {root}")
    directories: set[str] = set()
    files: dict[str, tuple[Path, os.stat_result]] = {}
    pending = [root]
    while pending:
        current = pending.pop()
        try:
            entries = sorted(current.iterdir(), key=lambda item: item.name.casefold())
        except OSError as exc:
            raise CoordinatorError(f"cannot enumerate {label}: {current}: {exc}") from exc
        for entry in entries:
            relative = entry.relative_to(root).as_posix()
            if _contains_gt_token(relative):
                raise CoordinatorError(f"ground-truth token in {label}: {entry}")
            if entry.name.casefold().endswith(".zip"):
                raise CoordinatorError(f"ZIP/archive entry is not allowed in {label}: {entry}")
            try:
                no_follow_metadata = entry.lstat()
            except OSError as exc:
                raise CoordinatorError(f"cannot lstat {label}: {entry}: {exc}") from exc
            if _is_reparse_point(entry, no_follow_metadata):
                raise CoordinatorError(f"reparse/symlink is not allowed in {label}: {entry}")
            try:
                metadata = entry.stat()
            except OSError as exc:
                raise CoordinatorError(f"cannot stat {label}: {entry}: {exc}") from exc
            if stat.S_ISDIR(metadata.st_mode):
                directories.add(relative)
                pending.append(entry)
            elif stat.S_ISREG(metadata.st_mode):
                files[relative] = (entry, metadata)
            else:
                raise CoordinatorError(f"non-regular entry in {label}: {entry}")
    return directories, files


def _sensor_manifest_hashes(record: Mapping[str, Any], sequence: str) -> dict[str, str]:
    hashes = record.get("file_hashes")
    if not isinstance(hashes, Mapping):
        raise CoordinatorError(f"sensor-only input view requires file_hashes for {sequence}")
    selected: dict[str, str] = {}
    for group in ("csv_sha256", "sensor_yaml_sha256"):
        values = hashes.get(group)
        if not isinstance(values, Mapping):
            raise CoordinatorError(f"sensor-only input view requires {group} for {sequence}")
        for relative, digest in values.items():
            relative = str(relative).replace("\\", "/")
            if any(relative.startswith(f"mav0/{sensor}/") for sensor in SENSOR_ONLY_COMPONENTS):
                if not isinstance(digest, str) or not re.fullmatch(r"[0-9a-fA-F]{64}", digest):
                    raise CoordinatorError(f"invalid sensor hash in manifest for {sequence}: {relative}")
                selected[relative] = digest.casefold()
    expected = {
        *(f"mav0/{sensor}/data.csv" for sensor in SENSOR_ONLY_COMPONENTS),
        *(f"mav0/{sensor}/sensor.yaml" for sensor in SENSOR_ONLY_COMPONENTS),
    }
    if set(selected) != expected:
        raise CoordinatorError(
            f"manifest sensor hash set mismatch for {sequence}: "
            f"expected={sorted(expected)!r} actual={sorted(selected)!r}"
        )
    return selected


def _validate_hardlink_staged_sequence_root(
    source_root: Path,
    sequence: str,
    record: Mapping[str, Any],
) -> dict[str, Any]:
    """Validate a GT-hidden view whose sensor files are hardlinks.

    Physical roots are used only for this preflight comparison and the
    post-exit GT evaluator.  They are deliberately absent from the returned
    request-facing evidence and therefore cannot enter an engine argv/env.
    """

    root = _lexical_absolute(source_root)
    sequence = _safe_component(sequence, "sequence")
    if not root.is_dir() or _is_reparse_point(root):
        raise CoordinatorError(f"hardlink-staged view root must be a real directory: {root}")
    # The hardlink namespace is deliberately narrow.  Permit a partial view
    # for focused tests/smoke selectors, but reject any unrelated root entry so
    # a ZIP, GT tree, or reparse point cannot be hidden beside a selected cell.
    for root_entry in sorted(root.iterdir(), key=lambda item: item.name.casefold()):
        if _contains_gt_token(root_entry.name) or root_entry.name.casefold().endswith(".zip"):
            raise CoordinatorError(f"forbidden GT/ZIP entry in hardlink-staged view: {root_entry}")
        if _is_reparse_point(root_entry):
            raise CoordinatorError(f"reparse point in hardlink-staged view: {root_entry}")
        if not root_entry.is_dir() or root_entry.name not in PROTOCOL_SEQUENCE_IDS:
            raise CoordinatorError(f"unexpected hardlink-staged root entry: {root_entry}")
    sequence_root = root / sequence
    if not sequence_root.is_dir() or _is_reparse_point(sequence_root):
        raise CoordinatorError(f"hardlink-staged sequence root must be a real directory: {sequence_root}")
    sequence_entries = sorted((item.name for item in sequence_root.iterdir()), key=str.casefold)
    if sequence_entries != ["mav0"]:
        raise CoordinatorError(f"hardlink-staged sequence has unexpected entries: {sequence_root}: {sequence_entries!r}")
    mav0 = sequence_root / "mav0"
    if not mav0.is_dir() or _is_reparse_point(mav0):
        raise CoordinatorError(f"hardlink-staged mav0 must be a real directory: {mav0}")
    entries = sorted(mav0.iterdir(), key=lambda item: item.name.casefold())
    names = [item.name for item in entries]
    if names != sorted(SENSOR_ONLY_COMPONENTS):
        raise CoordinatorError(f"hardlink-staged mav0 entries mismatch: {mav0}: {names!r}")
    if any(_contains_gt_token(name) for name in (*sequence_entries, *names)):
        raise CoordinatorError(f"hardlink-staged view contains a ground-truth token: {sequence_root}")

    physical_root = _physical_root_from_record(record)
    if physical_root.name.casefold() != sequence.casefold():
        raise CoordinatorError(f"manifest physical_root.path sequence mismatch: {physical_root} != {sequence}")
    try:
        if root.resolve(strict=False) == physical_root.resolve(strict=False):
            raise CoordinatorError("hardlink-staged engine root must differ from physical root")
    except OSError:
        pass
    expected_hashes = _sensor_manifest_hashes(record, sequence)
    total_files = 0
    total_bytes = 0
    image_files = 0
    # The strict fingerprint is a three-sensor contract.  Keep file-level
    # counts for audit detail, but expose the identity/volume/hash counters at
    # sensor-directory granularity so a multi-file camera tree cannot make a
    # valid hardlink view fail the canonical gate.
    identity_file_count = 0
    same_volume_file_count = 0
    identity_sensor_count = 0
    same_volume_sensor_count = 0
    hash_sensor_count = 0
    manifest_hash_count = 0
    sensors: dict[str, dict[str, Any]] = {}
    for sensor in SENSOR_ONLY_COMPONENTS:
        source_sensor = physical_root / "mav0" / sensor
        view_sensor = mav0 / sensor
        source_dirs, source_files = _sensor_file_inventory(source_sensor, label=f"physical {sequence}/{sensor}")
        view_dirs, view_files = _sensor_file_inventory(view_sensor, label=f"staged {sequence}/{sensor}")
        if source_dirs != view_dirs or set(source_files) != set(view_files):
            raise CoordinatorError(f"hardlink-staged tree mismatch for {sequence}/{sensor}")
        tree_digest = hashlib.sha256()
        sensor_bytes = 0
        sensor_images = 0
        sensor_manifest_hash_count = 0
        manifest_bound_files: list[dict[str, Any]] = []
        for relative in sorted(source_files, key=str.casefold):
            source_path, source_metadata = source_files[relative]
            view_path, view_metadata = view_files[relative]
            if source_metadata.st_dev != view_metadata.st_dev:
                raise CoordinatorError(f"hardlink-staged volume mismatch: {source_path} -> {view_path}")
            if source_metadata.st_ino != view_metadata.st_ino:
                raise CoordinatorError(f"hardlink-staged file identity mismatch: {source_path} -> {view_path}")
            if source_metadata.st_size != view_metadata.st_size:
                raise CoordinatorError(f"hardlink-staged size mismatch: {source_path} -> {view_path}")
            manifest_key = f"mav0/{sensor}/{relative}"
            expected = expected_hashes.get(manifest_key)
            # Hash only the six manifest-bound sensor files.  Image bytes are
            # bound by exact hardlink identity/size and camera set/count; a
            # full PNG hash scan would turn every plan into a multi-GB job.
            digest = sha256_file(source_path) if expected is not None else None
            if expected is not None:
                manifest_hash_count += 1
                sensor_manifest_hash_count += 1
                if digest.casefold() != expected:
                    raise CoordinatorError(f"manifest sensor hash mismatch: {sequence}/{manifest_key}")
                manifest_bound_files.append(
                    {
                        "relative": manifest_key,
                        "bytes": source_metadata.st_size,
                        "st_dev": source_metadata.st_dev,
                        "st_ino": source_metadata.st_ino,
                        "sha256": digest,
                    }
                )
            tree_digest.update(relative.encode("utf-8"))
            tree_digest.update(b"\0")
            tree_digest.update(str(source_metadata.st_size).encode("ascii"))
            tree_digest.update(b"\0")
            tree_digest.update(str(source_metadata.st_dev).encode("ascii"))
            tree_digest.update(b"\0")
            tree_digest.update(str(source_metadata.st_ino).encode("ascii"))
            tree_digest.update(b"\0")
            tree_digest.update((digest or "unhashed").encode("ascii"))
            tree_digest.update(b"\n")
            total_files += 1
            total_bytes += source_metadata.st_size
            sensor_bytes += source_metadata.st_size
            identity_file_count += 1
            same_volume_file_count += 1
            if sensor in {"cam0", "cam1"} and Path(relative).suffix.casefold() == ".png":
                if not relative.startswith("data/"):
                    raise CoordinatorError(f"camera PNG is outside data/ in {sequence}/{sensor}: {relative}")
                image_files += 1
                sensor_images += 1
            elif sensor in {"cam0", "cam1"} and relative.startswith("data/"):
                raise CoordinatorError(f"camera data contains a non-PNG file in {sequence}/{sensor}: {relative}")
        if sensor_manifest_hash_count != 2:
            raise CoordinatorError(
                f"manifest-bound sensor hash count mismatch for {sequence}/{sensor}: "
                f"{sensor_manifest_hash_count} != 2"
            )
        identity_sensor_count += 1
        same_volume_sensor_count += 1
        hash_sensor_count += 1
        sensors[sensor] = {
            "files": len(source_files),
            "bytes": sensor_bytes,
            "camera_images": sensor_images,
            "hardlink_identity_exact": True,
            "hardlink_same_volume": True,
            "manifest_hash_exact": True,
            "manifest_bound_file_count": sensor_manifest_hash_count,
            "tree_sha256": tree_digest.hexdigest(),
            "manifest_bound_files": manifest_bound_files,
        }
    camera_expected: dict[str, int] = {}
    camera_counts = record.get("camera_counts")
    cameras = record.get("cameras")
    for camera in ("cam0", "cam1"):
        count: Any = None
        if isinstance(camera_counts, Mapping) and isinstance(camera_counts.get(camera), Mapping):
            count = camera_counts[camera].get("png")
        if count is None and isinstance(cameras, Mapping) and isinstance(cameras.get(camera), Mapping):
            count = cameras[camera].get("png_count", cameras[camera].get("image_reference_count"))
        if count is not None:
            if isinstance(count, bool) or not isinstance(count, int) or count < 0:
                raise CoordinatorError(f"invalid manifest camera PNG count for {sequence}/{camera}")
            camera_expected[camera] = count
            if sensors[camera]["camera_images"] != count:
                raise CoordinatorError(
                    f"camera PNG count mismatch for {sequence}/{camera}: "
                    f"{sensors[camera]['camera_images']} != {count}"
                )
    if len(camera_expected) not in (0, 2):
        raise CoordinatorError(f"manifest camera PNG counts must cover cam0/cam1 for {sequence}")
    if manifest_hash_count != 6:
        raise CoordinatorError(f"manifest-bound sensor hash count mismatch: {manifest_hash_count} != 6")
    structure = {
        "mode": SENSOR_VIEW_MODE_HARDLINK,
        "sequence": sequence,
        "sequence_entries": sequence_entries,
        "mav0_entries": names,
        "sensors": sensors,
        "sensor_file_count": total_files,
        "camera_image_file_count": image_files,
        "camera_png_counts": {
            camera: sensors[camera]["camera_images"] for camera in ("cam0", "cam1")
        },
        "manifest_camera_png_counts": camera_expected,
        "manifest_bound_file_count": manifest_hash_count,
        "sensor_dir_count": len(sensors),
        "hardlink_identity_exact_count": identity_sensor_count,
        "hardlink_same_volume_count": same_volume_sensor_count,
        "hash_exact_count": hash_sensor_count,
        "hardlink_target_sensors": sorted(SENSOR_ONLY_COMPONENTS),
        "manifest_bound_target_files": sorted(expected_hashes),
        "hardlink_identity_exact_file_count": identity_file_count,
        "hardlink_same_volume_file_count": same_volume_file_count,
        "staged_data_bytes": 0,
    }
    return {
        "sensor_only_input_view": True,
        "sensor_input_view_mode": SENSOR_VIEW_MODE_HARDLINK,
        "sensor_bytes_copied": 0,
        "staged_sensor_only": True,
        "view_fingerprint": canonical_hash(structure),
        "gt_entry_absence": {
            "sensor_input_view_mode": SENSOR_VIEW_MODE_HARDLINK,
            "sequence_entries": sequence_entries,
            "mav0_entries": names,
            "forbidden_gt_names": [],
            "regular_sensor_copies": 0,
            "junction_count": 0,
            "hardlink_count": identity_sensor_count,
            "hardlink_identity_exact_count": identity_sensor_count,
            "hardlink_same_volume_count": same_volume_sensor_count,
            "hash_exact_count": hash_sensor_count,
            "hardlink_target_sensors": sorted(SENSOR_ONLY_COMPONENTS),
            "manifest_bound_target_files": sorted(expected_hashes),
            "sensor_file_count": total_files,
            "camera_image_file_count": image_files,
            "camera_png_counts": {
                camera: sensors[camera]["camera_images"] for camera in ("cam0", "cam1")
            },
            "manifest_camera_png_counts": camera_expected,
            "manifest_bound_file_count": manifest_hash_count,
            "sensor_dir_count": len(sensors),
            "hardlink_identity_exact_file_count": identity_file_count,
            "hardlink_same_volume_file_count": same_volume_file_count,
            "manifest_hash_exact_file_count": manifest_hash_count,
            "source_sensor_bytes": total_bytes,
            "staged_data_bytes": 0,
            "sensor_bytes_copied": 0,
            "zero_duplicated_data_bytes": True,
            "hash_validation": "source_sha256_bound_by_identical_st_dev_st_ino",
        },
        "structure": structure,
    }


def _validate_sensor_only_sequence_root(
    source_root: Path,
    sequence: str,
    record: Mapping[str, Any],
    mode: str = SENSOR_VIEW_MODE_JUNCTION,
) -> dict[str, Any]:
    """Validate the engine-visible sensor-only namespace for one sequence.

    The validator examines only the view directories and junction metadata; it
    never enumerates or hashes the physical GT tree.  Any extra entry,
    regular sensor copy, GT-looking name, missing sensor link, or junction
    escape fails closed before an engine can start.
    """

    mode, _ = _normalize_sensor_input_view_mode(mode, sensor_only_input_view=True)
    if mode == SENSOR_VIEW_MODE_HARDLINK:
        return _validate_hardlink_staged_sequence_root(source_root, sequence, record)
    if mode != SENSOR_VIEW_MODE_JUNCTION:
        raise CoordinatorError(f"unsupported sensor-only validator mode: {mode!r}")
    root = _lexical_absolute(source_root)
    sequence_root = root / _safe_component(sequence, "sequence")
    if not sequence_root.is_dir() or _is_directory_link(sequence_root):
        raise CoordinatorError(f"sensor-only sequence root must be a real directory: {sequence_root}")
    sequence_entries = sorted((item.name for item in sequence_root.iterdir()), key=str.casefold)
    if sequence_entries != ["mav0"]:
        raise CoordinatorError(f"sensor-only sequence has unexpected entries: {sequence_root}: {sequence_entries!r}")
    mav0 = sequence_root / "mav0"
    if not mav0.is_dir() or _is_directory_link(mav0):
        raise CoordinatorError(f"sensor-only mav0 must be a real directory: {mav0}")
    entries = sorted(mav0.iterdir(), key=lambda item: item.name.casefold())
    names = [item.name for item in entries]
    if names != sorted(SENSOR_ONLY_COMPONENTS):
        raise CoordinatorError(f"sensor-only mav0 entries mismatch: {mav0}: {names!r}")
    if any(_contains_gt_token(name) for name in (*sequence_entries, *names)):
        raise CoordinatorError(f"sensor-only view contains a ground-truth token: {sequence_root}")

    link_evidence: list[dict[str, Any]] = []
    for entry in entries:
        sensor = entry.name
        if not _is_directory_link(entry):
            raise CoordinatorError(f"sensor-only sensor must be a directory junction: {entry}")
        target = _directory_link_target(entry)
        if _contains_gt_token(str(target)):
            raise CoordinatorError(f"sensor junction target contains a ground-truth token: {entry} -> {target}")
        if not target.is_dir():
            raise CoordinatorError(f"sensor junction target is unavailable: {entry} -> {target}")
        candidates = _sensor_target_candidates(root, sequence, record, sensor)
        target_key = os.path.normcase(str(target).rstrip("\\/"))
        target_kind = None
        for kind, candidate in candidates.items():
            candidate_key = os.path.normcase(str(candidate).rstrip("\\/"))
            if target_key == candidate_key:
                target_kind = kind
                break
        if target_kind is None:
            raise CoordinatorError(f"sensor junction escapes the bound source roots: {entry} -> {target}")
        link_evidence.append(
            {
                "name": sensor,
                "link_type": "junction" if entry.is_junction() else "symlink",
                "target_kind": target_kind,
                "target_relative": f"mav0/{sensor}",
            }
        )
    structure = {
        "sequence": sequence,
        "sequence_entries": sequence_entries,
        "mav0_entries": names,
        "links": link_evidence,
        "forbidden_gt_names": [],
        "regular_sensor_copies": 0,
    }
    return {
        "sensor_only_input_view": True,
        "sensor_input_view_mode": SENSOR_VIEW_MODE_JUNCTION,
        "sensor_bytes_copied": 0,
        "staged_sensor_only": False,
        "view_fingerprint": canonical_hash(structure),
        "gt_entry_absence": {
            "sequence_entries": sequence_entries,
            "mav0_entries": names,
            "forbidden_gt_names": [],
            "regular_sensor_copies": 0,
            "junction_count": len(link_evidence),
        },
        "structure": structure,
    }


def _strict_gt_absence_fingerprint(
    *,
    sensor_only_input_view: bool,
    sensor_input_view_mode: str,
    sequences: Sequence[str],
    sensor_fingerprints: Mapping[str, Any],
    sensor_absence_evidence: Mapping[str, Any],
) -> tuple[bool, str | None]:
    """Validate and bind the complete sensor-only/GT-absence evidence set."""

    if not sensor_only_input_view or sensor_input_view_mode not in {
        SENSOR_VIEW_MODE_JUNCTION,
        SENSOR_VIEW_MODE_HARDLINK,
    }:
        return False, None
    if not sequences:
        return False, None
    bound: dict[str, Any] = {}
    for sequence in sequences:
        fingerprint = sensor_fingerprints.get(sequence)
        absence = sensor_absence_evidence.get(sequence)
        if not _valid_sha256(fingerprint) or not isinstance(absence, Mapping):
            return False, None
        if absence.get("forbidden_gt_names") != [] or absence.get("regular_sensor_copies") != 0:
            return False, None
        if sensor_input_view_mode == SENSOR_VIEW_MODE_JUNCTION:
            if absence.get("junction_count") != len(SENSOR_ONLY_COMPONENTS):
                return False, None
        else:
            if (
                absence.get("hardlink_identity_exact_count") != len(SENSOR_ONLY_COMPONENTS)
                or absence.get("hardlink_same_volume_count") != len(SENSOR_ONLY_COMPONENTS)
                or absence.get("hash_exact_count") != len(SENSOR_ONLY_COMPONENTS)
                or absence.get("sensor_dir_count") != len(SENSOR_ONLY_COMPONENTS)
                or absence.get("hardlink_target_sensors") != sorted(SENSOR_ONLY_COMPONENTS)
                or absence.get("manifest_bound_target_files")
                != sorted(
                    f"mav0/{sensor}/{filename}"
                    for sensor in SENSOR_ONLY_COMPONENTS
                    for filename in ("data.csv", "sensor.yaml")
                )
                or absence.get("manifest_bound_file_count")
                != len(SENSOR_ONLY_COMPONENTS) * 2
                or absence.get("hardlink_identity_exact_file_count")
                != absence.get("sensor_file_count")
                or absence.get("hardlink_same_volume_file_count")
                != absence.get("sensor_file_count")
                or absence.get("manifest_hash_exact_file_count")
                != len(SENSOR_ONLY_COMPONENTS) * 2
                or absence.get("zero_duplicated_data_bytes") is not True
            ):
                return False, None
        bound[sequence] = {
            "sensor_view_fingerprint": str(fingerprint).casefold(),
            "gt_entry_absence": dict(absence),
        }
    return True, canonical_hash(
        {
            "schema_id": INPUT_NAMESPACE_SCHEMA_ID,
            "sensor_input_view_mode": sensor_input_view_mode,
            "sequences": bound,
        }
    )


def _collect_sensor_namespace_evidence(
    dataset_root: Path,
    sequences: Sequence[str],
    records: Mapping[str, Any],
    sensor_input_view_mode: str,
) -> tuple[dict[str, str], dict[str, dict[str, Any]], dict[str, dict[str, Any]]]:
    """Validate every selected sensor-only sequence before request hashing."""

    fingerprints: dict[str, str] = {}
    absence: dict[str, dict[str, Any]] = {}
    evidence_by_sequence: dict[str, dict[str, Any]] = {}
    for sequence in sequences:
        record = records.get(sequence)
        if not isinstance(record, Mapping):
            raise CoordinatorError(f"sensor-only input view has no manifest record for {sequence}")
        evidence = _validate_sensor_only_sequence_root(
            dataset_root,
            sequence,
            record,
            sensor_input_view_mode,
        )
        fingerprint = evidence.get("view_fingerprint")
        gt_absence = evidence.get("gt_entry_absence")
        if not _valid_sha256(fingerprint) or not isinstance(gt_absence, Mapping):
            raise CoordinatorError(f"sensor-only input view has incomplete validation evidence for {sequence}")
        fingerprints[sequence] = str(fingerprint).casefold()
        absence[sequence] = dict(gt_absence)
        evidence_by_sequence[sequence] = evidence
    return fingerprints, absence, evidence_by_sequence


def _validate_engine_command(command: Sequence[str]) -> None:
    for token in command:
        if "{ground_truth" in str(token).casefold() or "{gt" in str(token).casefold():
            raise CoordinatorError("engine command cannot contain a ground-truth placeholder")
        if _contains_gt_token(token):
            raise CoordinatorError(f"engine command contains a ground-truth token: {token!r}")


def _normalize_representative(value: str | None) -> str | None:
    """Return a canonical representative id, rejecting ambiguous requests."""

    if value is None or not value.strip():
        return None
    match = re.fullmatch(r"(native_core|rust_current):([^:]+):r([1-9][0-9]*)", value.strip())
    if match is None:
        raise CoordinatorError(
            "representative must be METHOD:SEQUENCE:rN, for example "
            "rust_current:MH_01_easy:r1"
        )
    return f"{match.group(1)}:{match.group(2)}:r{int(match.group(3))}"


def _policy_flags(output_policy: str) -> tuple[bool, bool]:
    """Return the explicit no-MargData/no-trace contract for a policy."""

    if output_policy == OUTPUT_POLICY_LEAN:
        return True, True
    if output_policy == OUTPUT_POLICY_DIAGNOSTIC:
        return False, False
    raise CoordinatorError(f"unsupported output policy: {output_policy!r}")


def _validate_output_policy_command(method: str, command: Sequence[str], output_policy: str) -> None:
    """Fail closed when a command violates its two-engine output contract."""

    no_marg_data, no_trace = _policy_flags(output_policy)
    tokens = [str(token).casefold() for token in command]
    option_names = {token.split("=", 1)[0] for token in tokens}
    rust_wsl_wrapper = method == METHOD_RUST and any(
        Path(token).name.casefold() == "rust_wsl_runner.py" for token in tokens
    )

    def option_positions(name: str) -> list[int]:
        return [index for index, token in enumerate(tokens) if token == name or token.startswith(name + "=")]

    def option_has_value(name: str) -> bool:
        positions = option_positions(name)
        if len(positions) != 1:
            return False
        index = positions[0]
        token = tokens[index]
        if token.startswith(name + "="):
            return bool(token.split("=", 1)[1].strip())
        return index + 1 < len(tokens) and not tokens[index + 1].startswith("-")

    if output_policy == OUTPUT_POLICY_DIAGNOSTIC:
        if rust_wsl_wrapper:
            raise CoordinatorError("rust_wsl_linux supports only trajectory-only lean output")
        if method == METHOD_NATIVE:
            if not option_has_value("--marg-data"):
                raise CoordinatorError(
                    "native diagnostic command must contain exactly one --marg-data path"
                )
            if {"--no-marg-data", "--no-trace"} & option_names:
                raise CoordinatorError(
                    "native diagnostic command must not contain lean output flags"
                )
        if method == METHOD_RUST and ({"--no-marg-data", "--no-trace"} & option_names):
            raise CoordinatorError(
                "rust_current diagnostic command must omit --no-marg-data and --no-trace"
            )
        return
    if output_policy != OUTPUT_POLICY_LEAN:
        return
    if rust_wsl_wrapper:
        # The Linux twin wrapper owns the inner Rust argv and unconditionally
        # adds both lean flags.  Its outer WSL command therefore carries the
        # policy by adapter identity rather than duplicating inner argv.
        return
    if no_marg_data and "--marg-data" in option_names:
        raise CoordinatorError(f"{method} lean command must not contain --marg-data")
    if method == METHOD_RUST:
        if no_marg_data and "--no-marg-data" not in option_names:
            raise CoordinatorError("rust_current lean command must contain --no-marg-data")
        if no_trace and "--no-trace" not in option_names:
            raise CoordinatorError("rust_current lean command must contain --no-trace")


@dataclass(frozen=True)
class MethodSpec:
    method: str
    command_template: tuple[str, ...] | None
    executable: str | None
    executable_kind: str
    runtime_profile: str = RUST_RUNTIME_PROFILE_MSVC
    runner_config: str | None = None
    runner_calibration: str | None = None
    runner_wrapper: str | None = None
    provenance_path: str | None = None
    exactness_certificate_path: str | None = None


@dataclass(frozen=True)
class Cell:
    method: str
    sequence: str
    family: str
    repetition: int
    run_dir: Path
    source_root: Path
    source_root_resolved: Path
    command: tuple[str, ...]
    request: dict[str, Any]
    request_sha256: str
    expected_frames: int
    sensor_fingerprint: str | None

    def as_dict(self) -> dict[str, Any]:
        dataset_request = self.request.get("dataset") if isinstance(self.request.get("dataset"), Mapping) else {}
        return {
            "method": self.method,
            "sequence": self.sequence,
            "family": self.family,
            "repetition": self.repetition,
            "run_dir": str(self.run_dir),
            "source_root": str(self.source_root),
            "source_root_resolved": str(self.source_root_resolved),
            "command": list(self.command),
            "request_sha256": self.request_sha256,
            "output_policy": self.request.get("output_policy", OUTPUT_POLICY_LEAN),
            "no_marg_data": self.request.get("no_marg_data", True),
            "no_trace": self.request.get("no_trace", True),
            "representative_diagnostic": self.request.get("representative_diagnostic", False),
            "runtime_context": self.request.get("runtime_context"),
            "resume_identity": self.request.get("resume_identity"),
            "executable_binding": self.request.get("executable_binding"),
            "command_executable_binding": self.request.get("command_executable_binding"),
            "timing_feature_binding": self.request.get("timing_feature_binding"),
            "rust_runtime_profile": self.request.get(
                "rust_runtime_profile", RUST_RUNTIME_PROFILE_MSVC
            ),
            "rust_wsl_binding": self.request.get("rust_wsl_binding"),
            "rss_gate_candidate_eligible": self.request.get(
                "rss_gate_candidate_eligible", False
            ),
            "rust_executable_sha256": self.request.get("rust_executable_sha256"),
            "formal_denominator": self.request.get("formal_denominator", True),
            "input_namespace": self.request.get("input_namespace"),
            "expected_input_frames": self.expected_frames,
            "sensor_fingerprint": self.sensor_fingerprint,
            "sensor_only_input_view": self.request.get("sensor_only_input_view", False),
            "sensor_input_view_mode": self.request.get("sensor_input_view_mode", SENSOR_VIEW_MODE_DIRECT),
            "sensor_bytes_copied": self.request.get("sensor_bytes_copied", 0),
            "staged_sensor_only": self.request.get("staged_sensor_only", False),
            "sensor_view_fingerprint": dataset_request.get("sensor_view_fingerprint"),
            "gt_entry_absence": dataset_request.get("gt_entry_absence"),
        }


def _parse_template(raw: str | Sequence[str] | None) -> tuple[str, ...] | None:
    if raw is None:
        return None
    if isinstance(raw, str):
        try:
            value = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise CoordinatorError(f"command JSON is invalid: {exc}") from exc
    else:
        value = list(raw)
    if not isinstance(value, list) or not all(isinstance(item, str) for item in value) or not value:
        raise CoordinatorError("command template must be a non-empty JSON string array")
    return tuple(value)


def native_available(native_executable: str | Path | None = None, *, probe_wsl: bool = True) -> bool:
    """Return whether the pinned native core can be launched on this host."""

    requested = str(native_executable) if native_executable is not None else None
    if requested:
        path = Path(requested)
        if path.exists() and path.is_file():
            return True
        # A WSL-side executable is represented by wsl.exe plus a Linux path.
        if probe_wsl and shutil.which("wsl.exe"):
            try:
                result = subprocess.run(
                    ["wsl.exe", "-d", "Ubuntu-22.04", "--", "test", "-x", requested],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=5,
                    check=False,
                )
                return result.returncode == 0
            except (OSError, subprocess.TimeoutExpired):
                return False
        return False
    if shutil.which("wsl.exe") is None:
        return False
    if not probe_wsl:
        return True
    try:
        result = subprocess.run(
            ["wsl.exe", "-d", "Ubuntu-22.04", "--", "test", "-x", DEFAULT_NATIVE_BUILD],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=5,
            check=False,
        )
        return result.returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


def _method_available(spec: MethodSpec, command: Sequence[str] | None = None) -> bool:
    """Check a method without requiring a command-template executable path."""

    if spec.command_template is not None and command:
        executable = Path(str(command[0]))
        if executable.is_file() or shutil.which(str(command[0])):
            return True
        if str(command[0]).casefold().endswith("wsl.exe"):
            return native_available(spec.executable)
        return False
    if spec.executable_kind in {"native_wsl", "rust_wsl"}:
        return native_available(spec.executable)
    return Path(str(spec.executable)).is_file()


def _default_method_specs(
    *,
    rust_executable: Path | str | None,
    rust_runtime_profile: str = RUST_RUNTIME_PROFILE_MSVC,
    rust_wsl_executable: str | None = None,
    rust_wsl_config: str | None = None,
    rust_wsl_calibration: str | None = None,
    rust_wsl_wrapper: Path | str | None = None,
    rust_wsl_provenance_path: Path | str | None = None,
    rust_wsl_exactness_certificate_path: Path | str | None = None,
    native_executable: Path | str | None,
    rust_command: str | Sequence[str] | None,
    native_command: str | Sequence[str] | None,
) -> dict[str, MethodSpec]:
    rust_runtime_profile = _normalize_rust_runtime_profile(rust_runtime_profile)
    rust_template = _parse_template(rust_command)
    native = _canonical_executable(native_executable or DEFAULT_NATIVE_BUILD, allow_wsl_path=True)
    native_kind = "native_wsl" if native.startswith("/") else "native"
    if rust_runtime_profile == RUST_RUNTIME_PROFILE_WSL:
        if rust_template is not None:
            raise CoordinatorError(
                "rust_wsl_linux uses the built-in Linux monitor runner; custom Rust commands are not supported"
            )
        rust = _canonical_executable(
            rust_wsl_executable or DEFAULT_RUST_WSL_EXECUTABLE,
            allow_wsl_path=True,
        )
        if not rust.startswith("/"):
            raise CoordinatorError("rust_wsl_linux executable must be an absolute WSL path")
        runner_config = str(rust_wsl_config or DEFAULT_RUST_WSL_CONFIG)
        runner_calibration = str(rust_wsl_calibration or DEFAULT_RUST_WSL_CALIBRATION)
        runner_wrapper = str(
            _lexical_absolute(Path(rust_wsl_wrapper or DEFAULT_RUST_WSL_WRAPPER))
        )
        for value, label in (
            (runner_config, "Rust WSL config"),
            (runner_calibration, "Rust WSL calibration"),
        ):
            if not value.startswith("/"):
                raise CoordinatorError(f"{label} must be an absolute WSL path")
        return {
            METHOD_NATIVE: MethodSpec(
                METHOD_NATIVE,
                _parse_template(native_command),
                native,
                native_kind,
            ),
            METHOD_RUST: MethodSpec(
                METHOD_RUST,
                None,
                rust,
                "rust_wsl",
                runtime_profile=rust_runtime_profile,
                runner_config=runner_config,
                runner_calibration=runner_calibration,
                runner_wrapper=runner_wrapper,
                provenance_path=(
                    str(_lexical_absolute(Path(rust_wsl_provenance_path)))
                    if rust_wsl_provenance_path is not None
                    else None
                ),
                exactness_certificate_path=(
                    str(_lexical_absolute(Path(rust_wsl_exactness_certificate_path)))
                    if rust_wsl_exactness_certificate_path is not None
                    else None
                ),
            ),
        }
    if rust_template is not None:
        # A custom Rust command is source-bound to the executable it actually
        # launches.  Canonicalize argv[0] in the template itself: workers run
        # with ``cwd=attempt_dir``, so retaining a caller-relative token would
        # make a valid plan launch a different file (or fail to launch).
        rendered_argv0 = _resolve_command_executable(rust_template[0])
        if rendered_argv0 is not None:
            rust_template = (str(rendered_argv0), *rust_template[1:])
        if rust_executable is None:
            rust = str(rendered_argv0) if rendered_argv0 is not None else _canonical_executable(DEFAULT_RUST_EXECUTABLE)
        else:
            rust = _canonical_executable(rust_executable)
    else:
        rust = _canonical_executable(rust_executable or DEFAULT_RUST_EXECUTABLE)
    return {
        METHOD_NATIVE: MethodSpec(METHOD_NATIVE, _parse_template(native_command), native, native_kind),
        METHOD_RUST: MethodSpec(
            METHOD_RUST,
            rust_template,
            rust,
            "rust",
            runtime_profile=RUST_RUNTIME_PROFILE_MSVC,
        ),
    }


def _native_checkout_root(executable: str) -> str:
    marker = "/build/"
    root = executable.split(marker, 1)[0] if marker in executable else ""
    if not root:
        raise CoordinatorError(f"cannot infer native checkout root from executable: {executable}")
    return root


def _native_config_for_executable(executable: str) -> str:
    return f"{_native_checkout_root(executable)}/data/euroc_config.json"


def _native_calibration_for_executable(executable: str) -> str:
    return f"{_native_checkout_root(executable)}/data/euroc_ds_calib.json"


def _local_file_binding(path: Path | str, *, required: bool = False) -> dict[str, Any]:
    """Return a content binding for a host-visible executable or wrapper."""

    absolute = _lexical_absolute(Path(path))
    if not absolute.is_file():
        if required:
            raise CoordinatorError(f"bound file is unavailable: {absolute}")
        return {"path": str(absolute), "bytes": None, "sha256": None, "available": False}
    return {
        "path": str(absolute),
        "bytes": absolute.stat().st_size,
        "sha256": sha256_file(absolute),
        "available": True,
    }


def _local_file_binding_matches_current(binding: Any) -> bool:
    """Verify a persisted host-file binding before reusing a completed cell."""

    if not isinstance(binding, Mapping):
        return False
    path_value = binding.get("path")
    if (
        not isinstance(path_value, str)
        or binding.get("available") is not True
        or not _valid_sha256(binding.get("sha256"))
    ):
        return False
    path = Path(path_value)
    try:
        return (
            path.is_file()
            and int(binding.get("bytes")) == path.stat().st_size
            and str(binding["sha256"]).casefold() == sha256_file(path).casefold()
        )
    except (OSError, TypeError, ValueError):
        return False


def _wsl_file_binding(document: Mapping[str, Any], key: str, *, label: str) -> dict[str, Any]:
    """Validate one remote WSL file binding from a Rust provenance record."""

    value = document.get(key)
    if not isinstance(value, Mapping):
        raise CoordinatorError(f"Rust WSL provenance is missing {label} binding")
    path = value.get("path")
    if not isinstance(path, str) or not path.startswith("/"):
        raise CoordinatorError(f"Rust WSL {label} path must be absolute")
    digest = value.get("sha256")
    if not _valid_sha256(digest):
        raise CoordinatorError(f"Rust WSL {label} binding has no valid SHA-256")
    try:
        size = int(value.get("bytes"))
    except (TypeError, ValueError) as exc:
        raise CoordinatorError(f"Rust WSL {label} binding has no valid byte size") from exc
    if size < 0:
        raise CoordinatorError(f"Rust WSL {label} binding has a negative byte size")
    return {"path": path, "bytes": size, "sha256": str(digest).casefold()}


def _rust_wsl_provenance_binding(
    *,
    provenance_path: str | None,
    certificate_path: str | None,
    executable: str,
    config: str,
    calibration: str,
    wrapper: str,
    required: bool,
) -> dict[str, Any]:
    """Load the immutable Rust Linux twin binding and exactness certificate.

    Remote WSL files cannot be hashed with the Windows ``Path`` API.  The
    provenance manifest therefore carries their Linux paths, sizes and hashes;
    the WSL runner rechecks those values immediately before spawning Rust.  A
    local copy of each manifest is itself content-bound here so a plan cannot
    silently use stale certificate metadata.
    """

    if not provenance_path or not certificate_path:
        if required:
            raise CoordinatorError(
                "rust_wsl_linux requires provenance and 52/80/400 exactness certificate bindings"
            )
        return {
            "profile": RUST_RUNTIME_PROFILE_WSL,
            "rss_candidate_eligible": False,
            "eligibility_reason": "missing_rust_wsl_provenance_or_exactness_certificate",
        }
    provenance_file = _local_file_binding(provenance_path, required=required)
    certificate_file = _local_file_binding(certificate_path, required=required)
    if provenance_file.get("available") is not True or certificate_file.get("available") is not True:
        if required:
            raise CoordinatorError("Rust WSL provenance/certificate file is unavailable")
        return {
            "profile": RUST_RUNTIME_PROFILE_WSL,
            "rss_candidate_eligible": False,
            "eligibility_reason": "unavailable_rust_wsl_provenance_or_exactness_certificate",
            "provenance_file": provenance_file,
            "exactness_certificate": certificate_file,
        }
    provenance = _load_json(Path(str(provenance_file["path"])))
    if provenance.get("schema_id") != RUST_WSL_PROVENANCE_SCHEMA:
        raise CoordinatorError("Rust WSL provenance schema mismatch")
    if provenance.get("runtime_profile") != RUST_RUNTIME_PROFILE_WSL:
        raise CoordinatorError("Rust WSL provenance runtime profile mismatch")
    binary = _wsl_file_binding(provenance, "binary", label="binary")
    config_binding = _wsl_file_binding(provenance, "config", label="config")
    calibration_binding = _wsl_file_binding(provenance, "calibration", label="calibration")
    for expected, actual, label in (
        (executable, binary["path"], "binary"),
        (config, config_binding["path"], "config"),
        (calibration, calibration_binding["path"], "calibration"),
    ):
        if str(expected) != str(actual):
            raise CoordinatorError(
                f"Rust WSL provenance {label} path does not match the requested runner path"
            )
    source = provenance.get("source")
    source_sha = provenance.get("source_sha256")
    if source_sha is None and isinstance(source, Mapping):
        source_sha = source.get("sha256")
    if not _valid_sha256(source_sha):
        raise CoordinatorError("Rust WSL provenance has no valid source SHA-256")
    target_triple = provenance.get("target_triple")
    if not isinstance(target_triple, str) or not target_triple.strip():
        raise CoordinatorError("Rust WSL provenance has no target triple")
    certificate = _load_json(Path(str(certificate_file["path"])))
    if certificate.get("schema_id") != RUST_WSL_EXACTNESS_SCHEMA:
        raise CoordinatorError("Rust WSL exactness certificate schema mismatch")
    if str(certificate.get("status", "")).casefold() != "pass":
        raise CoordinatorError("Rust WSL exactness certificate is not a pass")
    if certificate.get("rust_wsl_provenance_sha256", "").casefold() != str(provenance_file["sha256"]).casefold():
        raise CoordinatorError("Rust WSL exactness certificate is not bound to the provenance file")
    expected_bindings = {
        "rust_wsl_executable_sha256": binary["sha256"],
        "config_sha256": config_binding["sha256"],
        "calibration_sha256": calibration_binding["sha256"],
        "source_sha256": str(source_sha).casefold(),
    }
    for key, expected in expected_bindings.items():
        actual = certificate.get(key)
        if not _valid_sha256(actual) or str(actual).casefold() != expected:
            raise CoordinatorError(f"Rust WSL exactness certificate binding mismatch: {key}")
    if not _valid_sha256(certificate.get("rust_msvc_executable_sha256")):
        raise CoordinatorError("Rust WSL exactness certificate has no MSVC control SHA-256")
    required_frames = (52, 80, 400)
    if certificate.get("required_frames") != list(required_frames):
        raise CoordinatorError("Rust WSL exactness certificate must cover frames 52, 80, and 400")
    frame_results = certificate.get("frames")
    if not isinstance(frame_results, Mapping):
        raise CoordinatorError("Rust WSL exactness certificate has no per-frame results")
    for frame in required_frames:
        result = frame_results.get(str(frame))
        if not isinstance(result, Mapping):
            raise CoordinatorError(f"Rust WSL exactness certificate is missing frame {frame}")
        for key in ("trajectory_exact", "lifecycle_exact", "forbidden_outputs_absent"):
            if result.get(key) is not True:
                raise CoordinatorError(
                    f"Rust WSL exactness certificate frame {frame} is not exact: {key}"
                )
    runner_binding = _local_file_binding(wrapper, required=required)
    if runner_binding.get("available") is not True:
        raise CoordinatorError("Rust WSL runner wrapper is unavailable")
    return {
        "profile": RUST_RUNTIME_PROFILE_WSL,
        "rss_candidate_eligible": True,
        "eligibility_reason": "exactness_certificate_and_remote_bindings_present",
        "binary": binary,
        "config": config_binding,
        "calibration": calibration_binding,
        "source_sha256": str(source_sha).casefold(),
        "target_triple": target_triple,
        "toolchain": provenance.get("toolchain"),
        "build": provenance.get("build"),
        "wsl": provenance.get("wsl"),
        "provenance_file": provenance_file,
        "exactness_certificate": certificate_file,
        "exactness_certificate_status": "pass",
        "exactness_required_frames": list(required_frames),
        "rust_msvc_executable_sha256": str(
            certificate["rust_msvc_executable_sha256"]
        ).casefold(),
        "runner": runner_binding,
    }


def _validate_rust_wsl_request_binding(request: Mapping[str, Any]) -> None:
    """Recheck local Rust WSL manifest/certificate files before every launch."""

    binding = request.get("rust_wsl_binding")
    if not isinstance(binding, Mapping) or binding.get("rss_candidate_eligible") is not True:
        raise CoordinatorError(
            "rust_wsl_linux is not an RSS candidate without a valid exactness certificate"
        )
    for key in ("provenance_file", "exactness_certificate", "runner"):
        if not _local_file_binding_matches_current(binding.get(key)):
            raise CoordinatorError(f"Rust WSL {key} binding changed before launch")


def _resolve_command_executable(token: Any) -> Path | None:
    """Resolve a rendered argv[0] to a local file without executing it."""

    if not isinstance(token, str) or not token.strip():
        return None
    candidate = Path(token)
    if candidate.is_file():
        return _lexical_absolute(candidate).resolve(strict=False)
    located = shutil.which(token)
    if located:
        path = Path(located)
        if path.is_file():
            return _lexical_absolute(path).resolve(strict=False)
    return None


def _command_executable_binding(command: Sequence[str], *, required: bool) -> dict[str, Any]:
    """Return a content binding for the executable actually present in argv[0]."""

    if not command:
        raise CoordinatorError("engine command must contain an executable")
    resolved = _resolve_command_executable(command[0])
    if resolved is None:
        if required:
            raise CoordinatorError(f"engine command executable is unavailable: {command[0]!r}")
        return {
            "path": str(command[0]),
            "bytes": None,
            "sha256": None,
            "available": False,
            "argv0": str(command[0]),
        }
    binding = _local_file_binding(resolved, required=required)
    binding["argv0"] = str(command[0])
    return binding


def _validate_rust_command_binding(
    command: Sequence[str],
    bound_executable: Any,
    *,
    required: bool,
) -> dict[str, Any]:
    """Require rendered Rust argv[0] content to equal the bound executable."""

    actual = _command_executable_binding(command, required=required)
    actual_sha = actual.get("sha256")
    bound_sha = bound_executable.get("sha256") if isinstance(bound_executable, Mapping) else None
    if required and (not _valid_sha256(actual_sha) or not _valid_sha256(bound_sha)):
        raise CoordinatorError("Rust command argv[0] and bound executable require valid content SHA-256 bindings")
    if required and _valid_sha256(actual_sha) and _valid_sha256(bound_sha):
        if str(actual_sha).casefold() != str(bound_sha).casefold():
            raise CoordinatorError(
                "Rust command argv[0] content SHA-256 does not match the bound executable"
            )
        if actual.get("bytes") != bound_executable.get("bytes"):
            raise CoordinatorError("Rust command argv[0] byte length does not match the bound executable")
    return actual


def _revalidate_sensor_view_before_launch(
    cell: Cell,
    dataset_manifest: Mapping[str, Any] | None,
) -> dict[str, Any] | None:
    """Recheck the planned sensor namespace immediately before an engine call.

    Planning records the complete view fingerprint, but links/files can be
    changed between planning and a worker launch.  Revalidate every selected
    sequence and the manifest content, then compare the resulting aggregate
    fingerprint and this cell's evidence with the immutable plan.  No
    physical-root path is added to the engine request or command.
    """

    request = cell.request
    if request.get("sensor_only_input_view") is not True:
        return None
    dataset_request = request.get("dataset")
    if not isinstance(dataset_request, Mapping):
        raise CoordinatorError("sensor-only launch is missing its dataset binding")
    dataset_root_value = dataset_request.get("root")
    manifest_path_value = dataset_request.get("manifest_path")
    manifest_sha = dataset_request.get("manifest_sha256")
    if not isinstance(dataset_root_value, str) or not dataset_root_value:
        raise CoordinatorError("sensor-only launch is missing its engine-visible dataset root")
    if not isinstance(manifest_path_value, str) or not manifest_path_value:
        raise CoordinatorError("sensor-only launch is missing its manifest path binding")
    manifest_path = _lexical_absolute(Path(manifest_path_value))
    if not manifest_path.is_file() or not _valid_sha256(manifest_sha):
        raise CoordinatorError("sensor-only manifest binding is unavailable or invalid at launch")
    current_manifest_sha = sha256_file(manifest_path)
    if current_manifest_sha.casefold() != str(manifest_sha).casefold():
        raise CoordinatorError("sensor-only manifest fingerprint changed between planning and launch")
    try:
        current_manifest = _load_json(manifest_path)
    except (CoordinatorError, OSError, ValueError, json.JSONDecodeError) as exc:
        raise CoordinatorError(f"sensor-only manifest cannot be reloaded at launch: {exc}") from exc
    records = _manifest_records(current_manifest)
    namespace = request.get("input_namespace")
    if not isinstance(namespace, Mapping):
        raise CoordinatorError("sensor-only launch is missing its input namespace binding")
    expected_fingerprints = namespace.get("sensor_view_fingerprints")
    if not isinstance(expected_fingerprints, Mapping) or not expected_fingerprints:
        raise CoordinatorError("sensor-only launch is missing per-sequence view fingerprints")
    sequences = sorted(str(sequence) for sequence in expected_fingerprints)
    mode = str(request.get("sensor_input_view_mode", SENSOR_VIEW_MODE_JUNCTION))
    dataset_root = _lexical_absolute(Path(dataset_root_value))
    current_fingerprints: dict[str, str] = {}
    current_absence: dict[str, dict[str, Any]] = {}
    current_evidence: dict[str, dict[str, Any]] = {}
    for sequence in sequences:
        record = records.get(sequence)
        if not isinstance(record, Mapping):
            raise CoordinatorError(f"sensor-only manifest record disappeared before launch: {sequence}")
        evidence = _validate_sensor_only_sequence_root(dataset_root, sequence, record, mode)
        fingerprint = evidence.get("view_fingerprint")
        absence = evidence.get("gt_entry_absence")
        if not _valid_sha256(fingerprint) or not isinstance(absence, Mapping):
            raise CoordinatorError(f"sensor-only launch evidence is incomplete for {sequence}")
        current_fingerprints[sequence] = str(fingerprint).casefold()
        current_absence[sequence] = dict(absence)
        current_evidence[sequence] = evidence
    expected_fingerprint_map = {
        str(sequence): str(fingerprint).casefold()
        for sequence, fingerprint in expected_fingerprints.items()
    }
    if current_fingerprints != expected_fingerprint_map:
        raise CoordinatorError("sensor view fingerprint changed between planning and launch")
    current_strict, current_gt_fingerprint = _strict_gt_absence_fingerprint(
        sensor_only_input_view=True,
        sensor_input_view_mode=mode,
        sequences=sequences,
        sensor_fingerprints=current_fingerprints,
        sensor_absence_evidence=current_absence,
    )
    expected_strict = bool(request.get("strict_gt_absence_verified", namespace.get("strict_gt_absence_verified", False)))
    expected_gt_fingerprint = request.get("gt_absence_fingerprint", namespace.get("gt_absence_fingerprint"))
    if current_strict is not expected_strict or current_gt_fingerprint != expected_gt_fingerprint:
        raise CoordinatorError("strict GT-absence fingerprint changed between planning and launch")
    cell_evidence = dataset_request.get("sensor_view_evidence")
    if not isinstance(cell_evidence, Mapping):
        raise CoordinatorError("sensor-only launch is missing per-cell view evidence")
    current_cell_evidence = current_evidence.get(cell.sequence)
    if current_cell_evidence != dict(cell_evidence):
        raise CoordinatorError("sensor view evidence changed between planning and launch")
    return current_cell_evidence


def _wsl_capture(*arguments: str, timeout: float = 10.0) -> str:
    """Run a short read-only command in the pinned WSL distribution."""

    try:
        result = subprocess.run(
            ["wsl.exe", "-d", "Ubuntu-22.04", "--", *arguments],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise CoordinatorError(f"native WSL provenance probe failed: {exc}") from exc
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip()
        raise CoordinatorError(f"native WSL provenance probe failed ({result.returncode}): {detail[-500:]}")
    return result.stdout.strip()


def _native_provenance(spec: MethodSpec, expected_commit: str | None = None) -> dict[str, Any] | None:
    """Bind a WSL native spec to a clean checkout and immutable file hashes."""

    if spec.executable_kind != "native_wsl" or not spec.executable:
        return None
    executable = str(spec.executable)
    root = _native_checkout_root(executable)
    config = _native_config_for_executable(executable)
    calibration = _native_calibration_for_executable(executable)
    wrapper = _lexical_absolute(DEFAULT_NATIVE_WRAPPER)
    if not wrapper.is_file():
        raise CoordinatorError(f"native WSL wrapper is unavailable: {wrapper}")
    head = _wsl_capture("git", "-C", root, "rev-parse", "HEAD")
    status = _wsl_capture("git", "-C", root, "status", "--porcelain")
    if status:
        raise CoordinatorError(f"native checkout is dirty: {root}: {status.splitlines()[:8]!r}")
    if expected_commit and head.casefold() != str(expected_commit).casefold():
        raise CoordinatorError(f"native checkout commit mismatch: {head} != {expected_commit}")

    def digest(path: str) -> str:
        output = _wsl_capture("sha256sum", path)
        value = output.split()[0] if output.split() else ""
        if not re.fullmatch(r"[0-9a-fA-F]{64}", value):
            raise CoordinatorError(f"invalid native hash for {path}: {output!r}")
        return value.lower()

    return {
        "checkout_root": root,
        "git_head": head,
        "git_tree_sha1": NATIVE_UPSTREAM_TREE_SHA1,
        "git_status_clean": True,
        "executable": executable,
        "executable_sha256": digest(executable),
        "config": config,
        "config_sha256": digest(config),
        "calibration": calibration,
        "calibration_sha256": digest(calibration),
        "wrapper_path": str(wrapper),
        "wrapper_bytes": wrapper.stat().st_size,
        "wrapper_sha256": sha256_file(wrapper),
    }


def _render_template(template: Sequence[str], values: Mapping[str, str]) -> tuple[str, ...]:
    rendered = tuple(str(token).format_map(dict(values)) for token in template)
    _validate_engine_command(rendered)
    return rendered


def _default_command(
    spec: MethodSpec,
    *,
    source_root: Path,
    engine_output: Path,
    config: Path,
    calibration: Path,
    max_frames: int | None,
    threads: int,
    seed: int,
    sequence: str,
    output_policy: str = OUTPUT_POLICY_LEAN,
    poll_seconds: float = DEFAULT_POLL_SECONDS,
) -> tuple[str, ...]:
    no_marg_data, _no_trace = _policy_flags(output_policy)
    if spec.executable_kind == "native_wsl":
        if threads != 1:
            raise CoordinatorError("canonical native WSL wrapper requires threads=1")
        native_executable = str(spec.executable)
        args = [
            "wsl.exe",
            "-d",
            "Ubuntu-22.04",
            "--",
            "python3",
            windows_to_wsl(DEFAULT_NATIVE_WRAPPER),
            "--binary",
            native_executable,
            "--config",
            _native_config_for_executable(native_executable),
            "--calibration",
            _native_calibration_for_executable(native_executable),
            "--input-root",
            windows_to_wsl(source_root),
            "--output-root",
            windows_to_wsl(engine_output),
            "--monitor-json",
            windows_to_wsl(engine_output / "native_monitor.json"),
            "--checkout",
            _native_checkout_root(native_executable),
            "--num-threads",
            "1",
            "--poll-seconds",
            str(poll_seconds),
        ]
        if max_frames is not None:
            args.extend(("--max-frames", str(max_frames)))
        return tuple(args)
    if spec.executable_kind == "rust_wsl":
        if threads != 1:
            raise CoordinatorError("canonical Rust WSL monitor requires threads=1")
        wrapper = spec.runner_wrapper or str(DEFAULT_RUST_WSL_WRAPPER)
        config_wsl = spec.runner_config or DEFAULT_RUST_WSL_CONFIG
        calibration_wsl = spec.runner_calibration or DEFAULT_RUST_WSL_CALIBRATION
        monitor_path = engine_output / "rust_wsl_monitor.json"
        args = [
            "wsl.exe",
            "-d",
            "Ubuntu-22.04",
            "--",
            "python3",
            _wsl_path(wrapper),
            "--binary",
            str(spec.executable),
            "--config",
            str(config_wsl),
            "--calibration",
            str(calibration_wsl),
            "--input-root",
            _wsl_path(source_root),
            "--output-root",
            _wsl_path(engine_output),
            "--monitor-json",
            _wsl_path(monitor_path),
            "--poll-seconds",
            str(poll_seconds),
            "--num-threads",
            "1",
        ]
        if spec.provenance_path is not None:
            args.extend(("--provenance-json", _wsl_path(spec.provenance_path)))
        if spec.exactness_certificate_path is not None:
            args.extend(("--exactness-certificate", _wsl_path(spec.exactness_certificate_path)))
        if max_frames is not None:
            args.extend(("--max-frames", str(max_frames)))
        return tuple(args)
    if spec.method == METHOD_NATIVE:
        args = [
            str(spec.executable),
            "--dataset-path",
            str(source_root),
            "--cam-calib",
            str(calibration),
            "--dataset-type",
            "euroc",
            "--config-path",
            str(config),
            "--show-gui",
            "0",
            "--save-trajectory",
            "euroc",
            "--num-threads",
            str(threads),
        ]
        if not no_marg_data:
            args[args.index("--show-gui") : args.index("--show-gui")] = [
                "--marg-data",
                str(engine_output / "marg_data"),
            ]
        if max_frames is not None:
            args.extend(("--max-frames", str(max_frames)))
        return tuple(args)
    # The current Rust demo intentionally has no seed/thread command flags.
    # The deterministic policy is still injected into its environment and
    # recorded in the manifest; command-template users can bind {seed}/{threads
    # } to a newer executable that exposes those controls.
    args = [
        str(spec.executable),
        "--euroc-dir",
        str(source_root),
        "--calibration",
        str(calibration),
        "--config",
        str(config),
        "--out-dir",
        str(engine_output),
    ]
    if no_marg_data:
        args.extend(("--no-marg-data", "--no-trace"))
    if max_frames is not None:
        args.extend(("--max-frames", str(max_frames)))
    return tuple(args)


def _cell_method_order(method_ids: Sequence[str], sequence_index: int, repetition: int) -> tuple[str, ...]:
    """Alternate paired method order by sequence/repetition parity.

    The canonical pair is native then Rust.  XOR parity ensures that neither
    engine is always the later/cold-cache process across repetitions while
    retaining a stable sequence-major cell set.
    """

    canonical = tuple(method for method in (METHOD_NATIVE, METHOD_RUST) if method in method_ids)
    if len(canonical) == 2 and (sequence_index + repetition) % 2:
        return (canonical[1], canonical[0])
    return canonical


def _command_values(
    *,
    source_root: Path,
    engine_output: Path,
    config: Path,
    calibration: Path,
    max_frames: int | None,
    threads: int,
    seed: int,
    sequence: str,
    method: str,
) -> dict[str, str]:
    values = {
        "dataset": str(source_root),
        "source_root": str(source_root),
        "output_root": str(engine_output),
        "engine_output": str(engine_output),
        "sequence": sequence,
        "max_frames": str(max_frames or 0),
        "threads": str(threads),
        "seed": str(seed),
        "config": str(config),
        "calibration": str(calibration),
        "config_native": DEFAULT_NATIVE_CONFIG,
        "calibration_native": DEFAULT_NATIVE_CALIBRATION,
        "method": method,
    }
    return values


def _run_dir(output_root: Path, method: str, sequence: str, repetition: int) -> Path:
    return output_root / _safe_component(method, "method") / _safe_component(sequence, "sequence") / f"r{repetition}"


def build_plan(
    *,
    dataset_root: Path,
    output_root: Path,
    protocol_path: Path = DEFAULT_PROTOCOL,
    dataset_manifest_path: Path | None = DEFAULT_DATASET_MANIFEST,
    methods: Iterable[str] = METHODS,
    sequences: Iterable[str] | None = None,
    families: Iterable[str] | None = None,
    repetitions: int = DEFAULT_REPETITIONS,
    max_frames: int | None = None,
    config_path: Path = DEFAULT_CONFIG,
    calibration_path: Path = DEFAULT_CALIBRATION,
    rust_executable: Path | str | None = None,
    rust_runtime_profile: str = RUST_RUNTIME_PROFILE_MSVC,
    rust_wsl_executable: str | None = None,
    rust_wsl_config: str | None = None,
    rust_wsl_calibration: str | None = None,
    rust_wsl_wrapper: Path | str | None = None,
    rust_wsl_provenance_path: Path | str | None = None,
    rust_wsl_exactness_certificate_path: Path | str | None = None,
    native_executable: Path | str | None = None,
    rust_command: str | Sequence[str] | None = None,
    native_command: str | Sequence[str] | None = None,
    threads: int = 1,
    workers: int = 1,
    seed: int = DEFAULT_SEED,
    timeout_seconds: float = DEFAULT_TIMEOUT_SECONDS,
    poll_seconds: float = DEFAULT_POLL_SECONDS,
    require_sources: bool = False,
    sensor_only_input_view: bool = False,
    sensor_input_view_mode: str | None = None,
    representative: str | None = None,
    run_session_id: str | None = None,
    timing_feature: str | bool | None = None,
    fresh_root: bool | None = None,
) -> dict[str, Any]:
    if repetitions < 1:
        raise CoordinatorError("repetitions must be >= 1")
    if max_frames is not None and max_frames < 1:
        raise CoordinatorError("max_frames must be positive when provided")
    if threads < 1:
        raise CoordinatorError("threads must be >= 1")
    if workers < 1:
        raise CoordinatorError("workers must be >= 1")
    if timeout_seconds <= 0 or poll_seconds <= 0:
        raise CoordinatorError("timeout_seconds and poll_seconds must be positive")
    rust_runtime_profile = _normalize_rust_runtime_profile(rust_runtime_profile)
    protocol_path = _lexical_absolute(Path(protocol_path))
    protocol = _load_json(protocol_path)
    if protocol.get("protocol_id") != PROTOCOL_ID:
        raise CoordinatorError(f"protocol id mismatch: {protocol_path}")
    protocol_sequences = [str(item) for item in protocol.get("sequences", []) if isinstance(item, str)]
    selected_sequences = select_sequences(protocol_sequences, sequences=sequences, families=families)
    dataset_root = _lexical_absolute(Path(dataset_root))
    output_root = _lexical_absolute(Path(output_root))
    persisted_session = _load_session_record(output_root)
    if fresh_root is None:
        # This is intentionally checked before any coordinator artifact is
        # created.  A caller may still inspect/reuse a non-empty root, but the
        # formal runtime gate must then remain non-canonical.
        fresh_root = (
            persisted_session.get("fresh_output_root") is True
            if persisted_session is not None
            else not output_root.exists()
        )
    elif not isinstance(fresh_root, bool):
        raise CoordinatorError("fresh_root must be a boolean when supplied")
    config_path = _lexical_absolute(Path(config_path))
    calibration_path = _lexical_absolute(Path(calibration_path))
    dataset_manifest: dict[str, Any] | None = None
    manifest_sha: str | None = None
    if dataset_manifest_path is not None:
        dataset_manifest_path = _lexical_absolute(Path(dataset_manifest_path))
        if dataset_manifest_path.is_file():
            dataset_manifest = _load_json(dataset_manifest_path)
            manifest_sha = sha256_file(dataset_manifest_path)
        elif require_sources:
            raise CoordinatorError(f"dataset manifest does not exist: {dataset_manifest_path}")
    sensor_input_view_mode, sensor_only_input_view = _sensor_mode_for_root(
        dataset_root,
        sensor_input_view_mode,
        sensor_only_input_view=bool(sensor_only_input_view),
    )
    records = _manifest_records(dataset_manifest)
    if sensor_only_input_view and not records:
        raise CoordinatorError("sensor-only input view requires a dataset manifest with sequence records")
    if sensor_only_input_view:
        for sequence in selected_sequences:
            record = records.get(sequence)
            if not isinstance(record, Mapping):
                raise CoordinatorError(f"sensor-only input view has no manifest record for {sequence}")
            _physical_root_from_record(record)
            ground_truth = record.get("ground_truth")
            gt_hash = ground_truth.get("sha256") if isinstance(ground_truth, Mapping) else None
            if not isinstance(gt_hash, str) or not re.fullmatch(r"[0-9a-fA-F]{64}", gt_hash):
                raise CoordinatorError(f"sensor-only input view requires a manifest-bound ground-truth hash for {sequence}")
    protocol_sha = sha256_file(protocol_path)
    config_sha = sha256_file(config_path) if config_path.is_file() else None
    calibration_sha = sha256_file(calibration_path) if calibration_path.is_file() else None
    method_ids = list(dict.fromkeys(str(item) for item in methods))
    unknown_methods = [item for item in method_ids if item not in METHODS]
    if unknown_methods:
        raise CoordinatorError(f"unsupported methods: {unknown_methods!r}")
    expected_formal_cell_count = len(method_ids) * len(selected_sequences) * repetitions
    full_formal_matrix = (
        set(method_ids) == set(METHODS)
        and len(method_ids) == len(METHODS)
        and selected_sequences == list(PROTOCOL_SEQUENCE_IDS)
        and repetitions == DEFAULT_REPETITIONS
    )
    if full_formal_matrix and expected_formal_cell_count != FORMAL_MATRIX_CELL_COUNT:
        raise CoordinatorError(
            "formal all11x1 matrix has an invalid expected cell count; refusing a partial denominator"
        )
    representative = _normalize_representative(representative)
    if representative is not None:
        # A diagnostic cell must not silently replace one of the formal lean
        # denominator cells.  Keep the coordinator formal-only and require a
        # separately named output root/run for retained MargData/trace.
        raise CoordinatorError(
            "representative diagnostics require a separate run/output root; "
            "the formal phase6 plan is lean-only"
        )
    specs = _default_method_specs(
        rust_executable=rust_executable,
        rust_runtime_profile=rust_runtime_profile,
        rust_wsl_executable=rust_wsl_executable,
        rust_wsl_config=rust_wsl_config,
        rust_wsl_calibration=rust_wsl_calibration,
        rust_wsl_wrapper=rust_wsl_wrapper,
        rust_wsl_provenance_path=rust_wsl_provenance_path,
        rust_wsl_exactness_certificate_path=rust_wsl_exactness_certificate_path,
        native_executable=native_executable,
        rust_command=rust_command,
        native_command=native_command,
    )
    native_provenance = None
    if METHOD_NATIVE in method_ids and specs[METHOD_NATIVE].executable_kind == "native_wsl" and (
        specs[METHOD_NATIVE].command_template is None or require_sources
    ) and (sensor_only_input_view or require_sources):
        upstream = protocol.get("upstream")
        expected_commit = upstream.get("commit") if isinstance(upstream, Mapping) else None
        native_provenance = _native_provenance(specs[METHOD_NATIVE], expected_commit)
    rust_wsl_binding: dict[str, Any] | None = None
    if METHOD_RUST in method_ids and rust_runtime_profile == RUST_RUNTIME_PROFILE_WSL:
        rust_spec = specs[METHOD_RUST]
        rust_wsl_binding = _rust_wsl_provenance_binding(
            provenance_path=rust_spec.provenance_path,
            certificate_path=rust_spec.exactness_certificate_path,
            executable=str(rust_spec.executable),
            config=str(rust_spec.runner_config),
            calibration=str(rust_spec.runner_calibration),
            wrapper=str(rust_spec.runner_wrapper),
            required=require_sources,
        )
        rust_executable_binding = rust_wsl_binding.get("binary") if rust_wsl_binding else None
    else:
        rust_executable_binding = (
            _local_file_binding(specs[METHOD_RUST].executable, required=require_sources)
            if METHOD_RUST in method_ids
            else None
        )
    if METHOD_RUST in method_ids and require_sources:
        if not _local_file_binding_matches_current(rust_executable_binding):
            raise CoordinatorError(
                "rust_current requires an available executable with a valid content SHA-256 binding"
            )
    timing_env_mode = _timing_env_mode()
    timing_feature_state = _normalize_timing_feature_state(
        timing_feature,
        env_mode=timing_env_mode,
    )
    session_id = "plan-unbound" if run_session_id is None else _runtime_session_id(run_session_id)
    if METHOD_NATIVE not in method_ids:
        native_executable_binding = None
    elif native_provenance is not None:
        native_executable_binding = {
            "kind": "native_wsl",
            "path": specs[METHOD_NATIVE].executable,
            "sha256": native_provenance.get("executable_sha256"),
            "bytes": None,
            "available": True,
            "wrapper_path": native_provenance.get("wrapper_path"),
            "wrapper_sha256": native_provenance.get("wrapper_sha256"),
        }
    elif specs[METHOD_NATIVE].executable and not str(specs[METHOD_NATIVE].executable).startswith("/"):
        native_executable_binding = _local_file_binding(
            specs[METHOD_NATIVE].executable,
            required=require_sources,
        )
    else:
        native_executable_binding = {
            "kind": "native_wsl",
            "path": specs[METHOD_NATIVE].executable,
            "sha256": None,
            "bytes": None,
            "available": False,
            "wrapper_path": str(DEFAULT_NATIVE_WRAPPER),
            "wrapper_sha256": None,
        }
    if require_sources and METHOD_NATIVE in method_ids and not _valid_sha256(
        native_executable_binding.get("sha256")
        if isinstance(native_executable_binding, Mapping)
        else None
    ):
        raise CoordinatorError(
            "native_core requires an available executable with a valid content SHA-256 binding"
        )
    timing_feature_binding = {
        "feature": TIMING_FEATURE_NAME,
        "state": timing_feature_state,
        "environment": TIMING_ENV_NAME,
        "environment_mode": timing_env_mode,
        "environment_requested": timing_env_mode == "requested",
        "rust_executable_sha256": (
            rust_executable_binding.get("sha256")
            if isinstance(rust_executable_binding, Mapping)
            else None
        ),
    }
    if require_sources and METHOD_RUST in method_ids and not _valid_sha256(
        timing_feature_binding["rust_executable_sha256"]
    ):
        raise CoordinatorError(
            f"timing feature binding requires a Rust executable SHA-256 for {TIMING_FEATURE_NAME}"
        )
    if sensor_only_input_view:
        (
            sensor_fingerprints,
            sensor_absence_evidence,
            sensor_view_evidence_by_sequence,
        ) = _collect_sensor_namespace_evidence(
            dataset_root,
            selected_sequences,
            records,
            sensor_input_view_mode,
        )
    else:
        sensor_fingerprints = {}
        sensor_absence_evidence = {}
        sensor_view_evidence_by_sequence = {}
    strict_gt_absence_verified, gt_absence_fingerprint = _strict_gt_absence_fingerprint(
        sensor_only_input_view=sensor_only_input_view,
        sensor_input_view_mode=sensor_input_view_mode,
        sequences=selected_sequences,
        sensor_fingerprints=sensor_fingerprints,
        sensor_absence_evidence=sensor_absence_evidence,
    )
    input_namespace = {
        "schema_id": INPUT_NAMESPACE_SCHEMA_ID,
        "mode": "sensor_only" if sensor_only_input_view else "direct",
        "sensor_input_view_mode": sensor_input_view_mode,
        "direct_dataset_no_sensor_copy": True,
        "staged_sensor_only": sensor_input_view_mode == SENSOR_VIEW_MODE_HARDLINK,
        "sensor_view_fingerprints": dict(sorted(sensor_fingerprints.items())),
        "strict_gt_absence_verified": strict_gt_absence_verified,
        "gt_absence_fingerprint": gt_absence_fingerprint,
    }
    cache_context = {
        "schema_id": RUNTIME_CONTEXT_SCHEMA_ID,
        "session_id": session_id,
        "protocol_sha256": protocol_sha,
        "dataset_root": str(dataset_root),
        "dataset_manifest_sha256": manifest_sha,
        "config_sha256": config_sha,
        "calibration_sha256": calibration_sha,
        "methods": method_ids,
        "sequences": selected_sequences,
        "repetitions": repetitions,
        "max_frames": max_frames,
        "threads": threads,
        "workers": workers,
        "seed": seed,
        "timeout_seconds": timeout_seconds,
        "poll_seconds": poll_seconds,
        "output_policy": OUTPUT_POLICY_LEAN,
        "rust_executable_binding": rust_executable_binding,
        "native_executable_binding": native_executable_binding,
        "sensor_only_input_view": sensor_only_input_view,
        "sensor_input_view_mode": sensor_input_view_mode,
        "sensor_bytes_copied": 0,
        "timing_feature_binding": timing_feature_binding,
        "rust_runtime_profile": rust_runtime_profile,
        "rust_wsl_binding": rust_wsl_binding,
        "rss_gate_candidate_eligible": bool(
            rust_runtime_profile == RUST_RUNTIME_PROFILE_WSL
            and isinstance(rust_wsl_binding, Mapping)
            and rust_wsl_binding.get("rss_candidate_eligible") is True
        ),
        "fresh_output_root": bool(fresh_root),
        "formal_expected_cell_count": expected_formal_cell_count,
        "full_formal_matrix": full_formal_matrix,
        "input_namespace": input_namespace,
    }
    cache_context_id = canonical_hash(cache_context)
    cells: list[Cell] = []
    sequence_positions = {sequence: index for index, sequence in enumerate(selected_sequences)}
    for method in method_ids:
        spec = specs[method]
        for sequence in selected_sequences:
            sequence_index = sequence_positions[sequence]
            family = family_for_sequence(sequence)
            source_root = resolve_direct_sequence_root(
                dataset_root,
                sequence,
                family=family,
                require_exists=require_sources or sensor_only_input_view,
            )
            source_resolved = source_root.resolve(strict=False)
            record = records.get(sequence, {})
            expected_frames = _expected_frames(record)
            sensor_fingerprint = _sensor_fingerprint(record)
            if sensor_only_input_view:
                sensor_view_evidence = sensor_view_evidence_by_sequence.get(sequence)
                if sensor_view_evidence is None:
                    sensor_view_evidence = _validate_sensor_only_sequence_root(
                        dataset_root,
                        sequence,
                        record,
                        sensor_input_view_mode,
                    )
                    sensor_view_evidence_by_sequence[sequence] = sensor_view_evidence
            else:
                sensor_view_evidence = None
            if sensor_view_evidence is not None:
                sensor_fingerprints[sequence] = str(sensor_view_evidence["view_fingerprint"])
                sensor_absence_evidence[sequence] = sensor_view_evidence["gt_entry_absence"]
            for repetition in range(1, repetitions + 1):
                run_dir = _run_dir(output_root, method, sequence, repetition)
                cell_id = f"{method}:{sequence}:r{repetition}"
                is_representative = cell_id == representative
                output_policy = OUTPUT_POLICY_DIAGNOSTIC if is_representative else OUTPUT_POLICY_LEAN
                no_marg_data, no_trace = _policy_flags(output_policy)
                method_order = _cell_method_order(method_ids, sequence_index, repetition)
                method_rank = method_order.index(method)
                pair_id = f"{sequence}:r{repetition}"
                pair_position = method_rank + 1
                runtime_context = {
                    "schema_id": RUNTIME_CONTEXT_SCHEMA_ID,
                    "session_id": session_id,
                    "pair_id": pair_id,
                    "pair_position": pair_position,
                    "pair_position_label": (
                        "first" if pair_position == 1 and len(method_order) > 1
                        else "second" if pair_position == 2 and len(method_order) > 1
                        else "only"
                    ),
                    "pair_size": len(method_order),
                    "method_order": list(method_order),
                    "cache_context_id": cache_context_id,
                }
                resume_identity = {
                    "schema_id": RESUME_IDENTITY_SCHEMA_ID,
                    "session_id": session_id,
                    "pair_id": pair_id,
                    "pair_position": pair_position,
                    "pair_size": len(method_order),
                    "method_order": list(method_order),
                    "cache_context_id": cache_context_id,
                    "rust_runtime_profile": rust_runtime_profile,
                    "rss_gate_candidate_eligible": bool(
                        rust_runtime_profile == RUST_RUNTIME_PROFILE_WSL
                        and isinstance(rust_wsl_binding, Mapping)
                        and rust_wsl_binding.get("rss_candidate_eligible") is True
                    ),
                    "rust_executable_sha256": timing_feature_binding.get("rust_executable_sha256"),
                }
                # The request binds only the engine-visible source root.  In
                # sensor-only mode the manifest physical root remains a
                # post-exit evaluator concern and is never copied into this
                # request or the engine command.
                if method == METHOD_NATIVE and native_provenance is not None:
                    method_config = {
                        "path": native_provenance["config"],
                        "sha256": native_provenance["config_sha256"],
                    }
                    method_calibration = {
                        "path": native_provenance["calibration"],
                        "sha256": native_provenance["calibration_sha256"],
                    }
                else:
                    method_config = {"path": str(config_path), "sha256": config_sha}
                    method_calibration = {"path": str(calibration_path), "sha256": calibration_sha}
                request = {
                    "schema_id": COORDINATOR_SCHEMA_ID,
                    "protocol": {"path": str(protocol_path), "sha256": protocol_sha},
                    "dataset": {
                        "root": str(dataset_root),
                        "sequence": sequence,
                        "family": family,
                        "direct_source_root": str(source_root),
                        "resolved_source_root": str(source_resolved),
                        "manifest_path": str(dataset_manifest_path) if dataset_manifest_path else None,
                        "manifest_sha256": manifest_sha,
                        "sensor_fingerprint": sensor_fingerprint,
                        "sensor_only_input_view": sensor_only_input_view,
                        "sensor_input_view_mode": sensor_input_view_mode,
                        "sensor_bytes_copied": 0,
                        "staged_sensor_only": sensor_input_view_mode == SENSOR_VIEW_MODE_HARDLINK,
                        "sensor_view_fingerprint": sensor_view_evidence["view_fingerprint"] if sensor_view_evidence else None,
                         "gt_entry_absence": sensor_view_evidence["gt_entry_absence"] if sensor_view_evidence else None,
                         "sensor_view_evidence": sensor_view_evidence,
                    },
                    "method": method,
                    "repetition": repetition,
                    "max_frames": max_frames,
                    "config": method_config,
                    "calibration": method_calibration,
                    "executable": spec.executable,
                     "native_provenance": native_provenance if method == METHOD_NATIVE else None,
                    "executable_binding": (
                        native_executable_binding if method == METHOD_NATIVE else rust_executable_binding
                    ),
                    "command_template": list(spec.command_template) if spec.command_template is not None else f"default:{method}",
                    "output_policy": output_policy,
                    "no_marg_data": no_marg_data,
                    "no_trace": no_trace,
                     "representative_diagnostic": is_representative,
                     "runtime_context": runtime_context,
                     "resume_identity": resume_identity,
                     "formal_denominator": not is_representative,
                    "input_namespace": input_namespace,
                    "strict_gt_absence_verified": strict_gt_absence_verified,
                    "gt_absence_fingerprint": gt_absence_fingerprint,
                    "timing_feature_binding": timing_feature_binding,
                    "rust_runtime_profile": rust_runtime_profile,
                    "rust_wsl_binding": rust_wsl_binding,
                    "rss_gate_candidate_eligible": bool(
                        rust_runtime_profile == RUST_RUNTIME_PROFILE_WSL
                        and isinstance(rust_wsl_binding, Mapping)
                        and rust_wsl_binding.get("rss_candidate_eligible") is True
                    ),
                    "rust_executable_sha256": (
                        rust_executable_binding.get("sha256")
                        if method == METHOD_RUST and isinstance(rust_executable_binding, Mapping)
                        else None
                    ),
                     "threads": threads,
                     "workers": workers,
                     "seed": seed,
                    "timeout_seconds": timeout_seconds,
                    "poll_seconds": poll_seconds,
                    "direct_dataset_no_sensor_copy": True,
                    "sensor_only_input_view": sensor_only_input_view,
                    "sensor_input_view_mode": sensor_input_view_mode,
                    "sensor_bytes_copied": 0,
                    "staged_sensor_only": sensor_input_view_mode == SENSOR_VIEW_MODE_HARDLINK,
                     "cell_order": {
                        "policy": "sequence_major_repetition_major_method_parity_v1",
                        "sequence_index": sequence_index,
                        "repetition": repetition,
                        "method_order": list(method_order),
                        "method_rank": method_rank,
                         "parity": (sequence_index + repetition) % 2,
                     },
                     "cell_order_policy_id": "sequence_major_repetition_major_method_parity_v1",
                    "retention": (
                        "trajectory_summary_evaluation_manifest_logs_marg_data_trace"
                        if is_representative
                        else "trajectory_summary_evaluation_manifest_logs"
                    ),
                }
                # Engine-output path is stable within an attempt only; command
                # tokens are rendered again by run_cell once an attempt number
                # is known.  The plan uses a stable placeholder path for audit.
                preview_output = run_dir / "attempt-01" / "engine_output"
                values = _command_values(
                    source_root=source_root,
                    engine_output=preview_output,
                    config=config_path,
                    calibration=calibration_path,
                    max_frames=max_frames,
                    threads=threads,
                    seed=seed,
                    sequence=sequence,
                    method=method,
                )
                if spec.command_template is None:
                    command = _default_command(
                        spec,
                        source_root=source_root,
                        engine_output=preview_output,
                        config=config_path,
                        calibration=calibration_path,
                        max_frames=max_frames,
                        threads=threads,
                        seed=seed,
                        sequence=sequence,
                        output_policy=output_policy,
                        poll_seconds=poll_seconds,
                    )
                else:
                    command = _render_template(spec.command_template, values)
                _validate_engine_command(command)
                _validate_output_policy_command(method, command, output_policy)
                command_executable_binding = None
                if method == METHOD_RUST:
                    if spec.executable_kind == "rust_wsl":
                        command_executable_binding = {
                            "kind": "rust_wsl_runner",
                            "argv0": str(command[0]),
                            "runtime_profile": RUST_RUNTIME_PROFILE_WSL,
                            "rust_executable_sha256": (
                                rust_executable_binding.get("sha256")
                                if isinstance(rust_executable_binding, Mapping)
                                else None
                            ),
                        }
                    else:
                        command_executable_binding = _validate_rust_command_binding(
                            command,
                            rust_executable_binding,
                            required=require_sources,
                        )
                request["command_executable_binding"] = command_executable_binding
                request_sha = canonical_hash(request)
                cells.append(
                    Cell(
                        method=method,
                        sequence=sequence,
                        family=family,
                        repetition=repetition,
                        run_dir=run_dir,
                        source_root=source_root,
                        source_root_resolved=source_resolved,
                        command=command,
                        request=request,
                        request_sha256=request_sha,
                        expected_frames=expected_frames,
                        sensor_fingerprint=sensor_fingerprint,
                    )
                )
    # Build order is part of the reproducibility contract.  Sorting after the
    # existing construction keeps the cell set and per-cell request hashes
    # unchanged while making workers=1 execution sequence-major.
    cells.sort(
        key=lambda cell: (
            int(cell.request["cell_order"]["sequence_index"]),
            cell.repetition,
            int(cell.request["cell_order"]["method_rank"]),
        )
    )
    if METHOD_NATIVE in method_ids and METHOD_RUST in method_ids:
        for method in (METHOD_NATIVE, METHOD_RUST):
            formal_cells = [
                cell
                for cell in cells
                if cell.method == method and cell.request.get("output_policy") == OUTPUT_POLICY_LEAN
            ]
            if not formal_cells:
                raise CoordinatorError(
                    f"formal output policy requires at least one {method} trajectory-only cell"
                )
    formal_cells = [
        cell
        for cell in cells
        if cell.request.get("output_policy", OUTPUT_POLICY_LEAN) == OUTPUT_POLICY_LEAN
    ]
    diagnostic_cells = [cell for cell in cells if cell not in formal_cells]
    if len(formal_cells) != expected_formal_cell_count or diagnostic_cells:
        raise CoordinatorError(
            "formal phase6 plan must contain exactly its lean denominator and no inline diagnostic cells"
        )
    if full_formal_matrix and len(formal_cells) != FORMAL_MATRIX_CELL_COUNT:
        raise CoordinatorError(
            f"formal all11x1 plan must contain exactly {FORMAL_MATRIX_CELL_COUNT} lean cells"
        )
    plan_request = {
        "schema_id": COORDINATOR_SCHEMA_ID,
        "protocol_path": str(protocol_path),
        "protocol_sha256": protocol_sha,
        "dataset_root": str(dataset_root),
        "dataset_manifest_path": str(dataset_manifest_path) if dataset_manifest_path else None,
        "dataset_manifest_sha256": manifest_sha,
        "output_root": str(output_root),
        "methods": method_ids,
        "sequences": selected_sequences,
        "repetitions": repetitions,
        "max_frames": max_frames,
        "config_path": str(config_path),
        "config_sha256": config_sha,
        "calibration_path": str(calibration_path),
        "calibration_sha256": calibration_sha,
        "threads": threads,
        "workers": workers,
        "seed": seed,
        "timeout_seconds": timeout_seconds,
        "poll_seconds": poll_seconds,
        "output_policy": OUTPUT_POLICY_LEAN,
        "no_marg_data": True,
        "no_trace": True,
        "representative": representative,
        "representative_output_policy": OUTPUT_POLICY_DIAGNOSTIC if representative else None,
        "formal_cell_count": len(formal_cells),
        "formal_expected_cell_count": expected_formal_cell_count,
        "formal_matrix": {
            "full_all11": full_formal_matrix,
            "formal_repetitions": DEFAULT_REPETITIONS,
            "expected_cell_count": expected_formal_cell_count,
            "actual_cell_count": len(formal_cells),
            "required_full_count": FORMAL_MATRIX_CELL_COUNT,
            "diagnostic_cells_in_formal": 0,
            "diagnostic_policy": "extra_cell_in_separate_output_root_and_aggregate",
        },
        "input_namespace": input_namespace,
        "run_session_id": session_id,
        "timing_feature_binding": timing_feature_binding,
        "rust_runtime_profile": rust_runtime_profile,
        "rust_wsl_binding": rust_wsl_binding,
        "rss_gate_candidate_eligible": bool(
            rust_runtime_profile == RUST_RUNTIME_PROFILE_WSL
            and isinstance(rust_wsl_binding, Mapping)
            and rust_wsl_binding.get("rss_candidate_eligible") is True
        ),
        "runtime_context": {
            "schema_id": RUNTIME_CONTEXT_SCHEMA_ID,
            "session_id": session_id,
            "cache_context": cache_context,
            "cache_context_id": cache_context_id,
            "pairing": "sequence/repetition pair with explicit method position",
        },
        "cell_order_policy": {
            "id": "sequence_major_repetition_major_method_parity_v1",
            "sequence_order": "selected_protocol_order",
            "repetition_order": "ascending",
            "canonical_method_order": [METHOD_NATIVE, METHOD_RUST],
            "parity": "reverse canonical method order when (sequence_index + repetition) % 2 == 1",
            "workers_1_execution": "deterministic sequential plan order",
            "workers_gt_1_runtime_gate": "noncanonical_parallel_order",
        },
        "runtime_gate_canonical": (
            workers == 1
            and threads == 1
            and seed == DEFAULT_SEED
            and bool(fresh_root)
            and sensor_only_input_view
            and strict_gt_absence_verified
        ),
        "resource_gate_canonical": (
            workers == 1
            and threads == 1
            and seed == DEFAULT_SEED
            and bool(fresh_root)
            and sensor_only_input_view
            and strict_gt_absence_verified
            and set(method_ids) == set(METHODS)
            and rust_runtime_profile == RUST_RUNTIME_PROFILE_WSL
            and isinstance(rust_wsl_binding, Mapping)
            and rust_wsl_binding.get("rss_candidate_eligible") is True
        ),
        "runtime_gate_requirements": {
            "workers": 1,
            "threads": 1,
            "seed": DEFAULT_SEED,
            "fresh_output_root": True,
            "sensor_namespace_must_be_uniform": True,
            "sensor_only_input_view": True,
            "strict_gt_absence_verified": True,
            "rust_runtime_profile": rust_runtime_profile,
            "rss_authoritative_domain": RSS_AUTHORITATIVE_DOMAIN,
            "rss_gate_candidate_eligible": bool(
                rust_runtime_profile == RUST_RUNTIME_PROFILE_WSL
                and isinstance(rust_wsl_binding, Mapping)
                and rust_wsl_binding.get("rss_candidate_eligible") is True
            ),
        },
        "workers_policy": "canonical_sequential" if workers == 1 else "noncanonical_parallel",
        "fresh_output_root": bool(fresh_root),
        "formal_method_policies": {
            method: {
                "output_policy": OUTPUT_POLICY_LEAN,
                "no_marg_data": True,
                "no_trace": True,
            }
            for method in method_ids
        },
        "executable_bindings": {
            METHOD_RUST: rust_executable_binding,
            METHOD_NATIVE: native_executable_binding,
        },
        "direct_dataset_no_sensor_copy": True,
        "sensor_only_input_view": sensor_only_input_view,
        "sensor_input_view_mode": sensor_input_view_mode,
        "sensor_bytes_copied": 0,
        "staged_sensor_only": sensor_input_view_mode == SENSOR_VIEW_MODE_HARDLINK,
        "sensor_view_fingerprints": sensor_fingerprints,
        "sensor_view_gt_entry_absence": sensor_absence_evidence,
        "strict_gt_absence_verified": strict_gt_absence_verified,
        "gt_absence_fingerprint": gt_absence_fingerprint,
        "native_provenance": native_provenance,
        "retention": "trajectory_summary_evaluation_manifest_logs",
    }
    return {
        "schema_id": COORDINATOR_SCHEMA_ID,
        "schema_version": COORDINATOR_VERSION,
        "request": plan_request,
        "request_sha256": canonical_hash(plan_request),
        "runtime_gate_canonical": (
            False if persisted_session is not None else plan_request["runtime_gate_canonical"]
        ),
        "resource_gate_canonical": (
            False if persisted_session is not None else plan_request["resource_gate_canonical"]
        ),
        "runtime_gate_canonical_override": (
            False if persisted_session is not None else None
        ),
        "runtime_gate_canonical_override_reason": (
            "resumed_session_pair_adjacency_not_reconstructible"
            if persisted_session is not None
            else None
        ),
        "resumed_existing_session": persisted_session is not None,
        "run_count": len(cells),
        "cells": cells,
        "methods": {
            method: {
                "executable": specs[method].executable,
                "kind": specs[method].executable_kind,
                "binding": plan_request.get("executable_bindings", {}).get(method),
            }
            for method in method_ids
        },
        "storage_policy": {
            "per_cell_sensor_copy_bytes": 0,
            "direct_dataset_paths": True,
            "sensor_only_input_view": sensor_only_input_view,
            "sensor_input_view_mode": sensor_input_view_mode,
            "sensor_bytes_copied": 0,
            "staged_sensor_only": sensor_input_view_mode == SENSOR_VIEW_MODE_HARDLINK,
            "gt_entry_absence_evidence": sensor_only_input_view,
            "strict_gt_absence_verified": strict_gt_absence_verified,
            "gt_absence_fingerprint": gt_absence_fingerprint,
            "retained_default": ["trajectory", "summary", "evaluation", "manifest", "stdout.log", "stderr.log"],
            "optional_representative_retains": ["marg_data", "trace"],
            "formal_output_policy": OUTPUT_POLICY_LEAN,
            "formal_no_marg_data": True,
            "formal_no_trace": True,
            "formal_cell_count": len(formal_cells),
            "formal_expected_cell_count": expected_formal_cell_count,
            "formal_full_all11": full_formal_matrix,
            "formal_repetitions": DEFAULT_REPETITIONS,
            "representative": representative,
            "representative_separate_from_formal": True,
            "diagnostic_requires_separate_root": True,
            "diagnostic_aggregate_separate": True,
            "input_namespace": input_namespace,
            "fresh_output_root": bool(fresh_root),
            "cell_workers": workers,
        },
    }


def build_diagnostic_plan(
    formal_plan: Mapping[str, Any],
    *,
    representative: str,
    diagnostic_output_root: Path,
    specs: Mapping[str, MethodSpec],
    config_path: Path,
    calibration_path: Path,
    max_frames: int | None,
    threads: int,
    seed: int,
    poll_seconds: float = DEFAULT_POLL_SECONDS,
) -> dict[str, Any]:
    """Build one explicitly separate diagnostic cell from a formal plan.

    The formal plan is deliberately immutable from this helper's perspective:
    it remains the complete lean denominator, while the returned plan has one
    full-output diagnostic cell rooted elsewhere.  Keeping this conversion
    explicit prevents a representative from silently replacing a formal
    repetition and gives dry-run callers an auditable second plan.
    """

    representative = _normalize_representative(representative)
    if representative is None:
        raise CoordinatorError("a diagnostic plan requires an explicit representative")
    diagnostic_output_root = _lexical_absolute(Path(diagnostic_output_root))
    formal_request = formal_plan.get("request")
    if not isinstance(formal_request, Mapping):
        raise CoordinatorError("formal plan has no request for diagnostic separation")
    formal_output_root = _lexical_absolute(Path(str(formal_request.get("output_root", ""))))
    if diagnostic_output_root == formal_output_root:
        raise CoordinatorError("diagnostic output root must differ from the formal output root")

    match = re.fullmatch(r"(native_core|rust_current):([^:]+):r([1-9][0-9]*)", representative)
    if match is None:
        raise CoordinatorError("diagnostic representative has an invalid canonical id")
    method, sequence, repetition_text = match.groups()
    repetition = int(repetition_text)
    if method not in specs:
        raise CoordinatorError(f"diagnostic method is not present in the formal plan: {method}")
    matching = [
        cell
        for cell in formal_plan.get("cells", [])
        if isinstance(cell, Cell)
        and cell.method == method
        and cell.sequence == sequence
        and cell.repetition == repetition
    ]
    if len(matching) != 1:
        raise CoordinatorError(
            f"diagnostic representative is not exactly one formal cell: {representative}"
        )
    formal_cell = matching[0]
    spec = specs[method]
    diagnostic_run_dir = _run_dir(diagnostic_output_root, method, sequence, repetition)
    diagnostic_engine_output = diagnostic_run_dir / "attempt-01" / "engine_output"
    values = _command_values(
        source_root=formal_cell.source_root,
        engine_output=diagnostic_engine_output,
        config=config_path,
        calibration=calibration_path,
        max_frames=max_frames,
        threads=threads,
        seed=seed,
        sequence=sequence,
        method=method,
    )
    if spec.command_template is None:
        command = _default_command(
            spec,
            source_root=formal_cell.source_root,
            engine_output=diagnostic_engine_output,
            config=config_path,
            calibration=calibration_path,
            max_frames=max_frames,
            threads=threads,
            seed=seed,
            sequence=sequence,
            output_policy=OUTPUT_POLICY_DIAGNOSTIC,
            poll_seconds=poll_seconds,
        )
    else:
        command = _render_template(spec.command_template, values)
    _validate_engine_command(command)
    _validate_output_policy_command(method, command, OUTPUT_POLICY_DIAGNOSTIC)

    # A diagnostic run has its own session/cache identity.  It may share input
    # bytes with the formal run, but never its output root or resumable cell
    # identity.
    diagnostic_session_id = canonical_hash(
        {
            "formal_request_sha256": formal_plan.get("request_sha256"),
            "diagnostic_output_root": str(diagnostic_output_root),
            "representative": representative,
        }
    )
    diagnostic_cache_context = {
        "schema_id": RUNTIME_CONTEXT_SCHEMA_ID,
        "session_id": diagnostic_session_id,
        "formal_request_sha256": formal_plan.get("request_sha256"),
        "diagnostic_output_root": str(diagnostic_output_root),
        "output_policy": OUTPUT_POLICY_DIAGNOSTIC,
        "method": method,
        "sequence": sequence,
        "repetition": repetition,
    }
    diagnostic_cache_context_id = canonical_hash(diagnostic_cache_context)
    diagnostic_runtime_context = {
        "schema_id": RUNTIME_CONTEXT_SCHEMA_ID,
        "session_id": diagnostic_session_id,
        "pair_id": f"diagnostic:{sequence}:r{repetition}",
        "pair_position": 1,
        "pair_position_label": "only",
        "pair_size": 1,
        "method_order": [method],
        "cache_context_id": diagnostic_cache_context_id,
    }
    request = deepcopy(formal_cell.request)
    request.update(
        {
            "output_policy": OUTPUT_POLICY_DIAGNOSTIC,
            "no_marg_data": False,
            "no_trace": False,
            "representative_diagnostic": True,
            "formal_denominator": False,
            "diagnostic_separate_from_formal": True,
            "diagnostic_output_root": str(diagnostic_output_root),
            "runtime_context": diagnostic_runtime_context,
            "cell_order_policy_id": "diagnostic_single_cell_v1",
            "cell_order": {
                "policy": "diagnostic_single_cell_v1",
                "sequence_index": 0,
                "repetition": repetition,
                "method_order": [method],
                "method_rank": 0,
                "parity": 0,
            },
            "retention": "trajectory_summary_evaluation_manifest_logs_marg_data_trace",
        }
    )
    request["resume_identity"] = _resume_identity(request)
    diagnostic_cell = replace(
        formal_cell,
        run_dir=diagnostic_run_dir,
        command=command,
        request=request,
        request_sha256=canonical_hash(request),
    )

    plan_request = deepcopy(dict(formal_request))
    plan_request.update(
        {
            "output_root": str(diagnostic_output_root),
            "methods": [method],
            "sequences": [sequence],
            "repetitions": 1,
            "output_policy": OUTPUT_POLICY_DIAGNOSTIC,
            "no_marg_data": False,
            "no_trace": False,
            "representative": representative,
            "representative_output_policy": OUTPUT_POLICY_DIAGNOSTIC,
            "formal_cell_count": 0,
            "formal_expected_cell_count": 0,
            "formal_matrix": {
                "full_all11": False,
                "formal_repetitions": DEFAULT_REPETITIONS,
                "expected_cell_count": 0,
                "actual_cell_count": 0,
                "required_full_count": FORMAL_MATRIX_CELL_COUNT,
                "diagnostic_cells_in_formal": 0,
                "diagnostic_policy": "diagnostic_plan_is_separate_from_formal",
            },
            "run_session_id": diagnostic_session_id,
            "runtime_context": {
                "schema_id": RUNTIME_CONTEXT_SCHEMA_ID,
                "session_id": diagnostic_session_id,
                "cache_context": diagnostic_cache_context,
                "cache_context_id": diagnostic_cache_context_id,
                "pairing": "diagnostic single cell; excluded from formal denominator",
            },
            "cell_order_policy": {
                "id": "diagnostic_single_cell_v1",
                "sequence_order": "single representative",
                "repetition_order": "single representative",
                "canonical_method_order": [method],
                "workers_1_execution": "single diagnostic cell",
                "workers_gt_1_runtime_gate": "noncanonical parallel order",
            },
            "runtime_gate_canonical": False,
            "resource_gate_canonical": False,
            "workers_policy": "diagnostic_separate",
            "fresh_output_root": not diagnostic_output_root.exists(),
            "formal_method_policies": {
                method: {
                    "output_policy": OUTPUT_POLICY_DIAGNOSTIC,
                    "no_marg_data": False,
                    "no_trace": False,
                }
            },
            "retention": "trajectory_summary_evaluation_manifest_logs_marg_data_trace",
            "diagnostic_only": True,
            "diagnostic_separate_from_formal": True,
            "diagnostic_output_root": str(diagnostic_output_root),
        }
    )
    return {
        "schema_id": COORDINATOR_SCHEMA_ID,
        "schema_version": COORDINATOR_VERSION,
        "request": plan_request,
        "request_sha256": canonical_hash(plan_request),
        "runtime_gate_canonical": False,
        "resource_gate_canonical": False,
        "run_count": 1,
        "formal_run_count": 0,
        "diagnostic_run_count": 1,
        "diagnostic_only": True,
        "diagnostic_separate_from_formal": True,
        "cells": [diagnostic_cell],
        "methods": {
            method: {
                "executable": specs[method].executable,
                "kind": specs[method].executable_kind,
                "binding": plan_request.get("executable_bindings", {}).get(method),
            }
        },
        "storage_policy": {
            "formal_cell_count": 0,
            "diagnostic_cell_count": 1,
            "formal_output_policy": OUTPUT_POLICY_LEAN,
            "diagnostic_output_policy": OUTPUT_POLICY_DIAGNOSTIC,
            "diagnostic_separate_from_formal": True,
            "diagnostic_output_root": str(diagnostic_output_root),
            "retained_default": [
                "trajectory",
                "summary",
                "evaluation",
                "manifest",
                "stdout.log",
                "stderr.log",
                "marg_data",
                "trace",
            ],
        },
    }


def plan_document(plan: Mapping[str, Any]) -> dict[str, Any]:
    cells = plan.get("cells", [])
    return {
        "schema_id": plan["schema_id"],
        "schema_version": plan["schema_version"],
        "request": plan["request"],
        "request_sha256": plan["request_sha256"],
        "run_count": plan["run_count"],
        "runtime_gate_canonical": _plan_runtime_gate_canonical(plan),
        "resource_gate_canonical": bool(plan.get("resource_gate_canonical", False)),
        "runtime_gate_canonical_override": plan.get("runtime_gate_canonical_override"),
        "runtime_gate_canonical_override_reason": plan.get("runtime_gate_canonical_override_reason"),
        "storage_policy": plan["storage_policy"],
        "cells": [cell.as_dict() if isinstance(cell, Cell) else cell for cell in cells],
        "dry_run": True,
    }


def _attempt_dir(run_dir: Path) -> Path:
    attempts = []
    if run_dir.is_dir():
        for item in run_dir.glob("attempt-*"):
            match = re.fullmatch(r"attempt-(\d+)", item.name)
            if match and item.is_dir():
                attempts.append(int(match.group(1)))
    return run_dir / f"attempt-{max(attempts, default=0) + 1:02d}"


def _archive_failure_marker(run_dir: Path, attempt_dir: Path) -> None:
    """Retain a prior terminal failure marker before a resumable retry."""

    marker = run_dir / "failure.json"
    if not marker.is_file():
        return
    try:
        document = _load_json(marker)
    except (CoordinatorError, OSError, ValueError, json.JSONDecodeError):
        document = {"raw_marker": marker.read_text(encoding="utf-8", errors="replace")}
    history = run_dir / "failure_history"
    archive = history / f"{attempt_dir.name}.json"
    suffix = 1
    while archive.exists():
        archive = history / f"{attempt_dir.name}.{suffix}.json"
        suffix += 1
    atomic_write_json(archive, document)


def _recover_stale_running_marker(run_dir: Path) -> None:
    """Clear only an orphaned coordinator marker before allocating an attempt.

    Older coordinator revisions wrote ``running.json`` before launching the
    child and did not include a PID.  Such a marker is safe to clear when the
    coordinator is invoked again; all attempt directories remain untouched.
    Current markers include both coordinator and engine PIDs, and an active
    PID fails closed instead of allowing two engines to write the same cell.
    """

    marker = run_dir / "running.json"
    if not marker.is_file():
        return
    try:
        document = _load_json(marker)
    except (CoordinatorError, OSError, ValueError, json.JSONDecodeError):
        marker.unlink()
        return
    pids = []
    for key in ("engine_pid", "coordinator_pid"):
        value = document.get(key)
        if isinstance(value, int) and value > 0:
            pids.append(value)
    for pid in pids:
        try:
            import psutil  # type: ignore

            if psutil.pid_exists(pid):
                raise CoordinatorError(f"cell has an active process (pid={pid}): {run_dir}")
        except ImportError:
            if pid != os.getpid():
                try:
                    os.kill(pid, 0)
                except OSError:
                    continue
                raise CoordinatorError(f"cell has an active process (pid={pid}): {run_dir}")
    marker.unlink()


def _safe_relative(path: Path, root: Path) -> str:
    return path.relative_to(root).as_posix()


def _artifact_inventory(root: Path) -> dict[str, dict[str, Any]]:
    inventory: dict[str, dict[str, Any]] = {}
    if not root.is_dir():
        return inventory
    for item in sorted((item for item in root.rglob("*") if item.is_file()), key=lambda p: p.as_posix()):
        relative = _safe_relative(item, root)
        inventory[relative] = {"bytes": item.stat().st_size, "sha256": sha256_file(item)}
    return inventory


def _diagnostic_artifact_paths(inventory: Mapping[str, Any]) -> list[str]:
    """List diagnostic artifacts that a lean cell must not emit before prune."""

    forbidden: list[str] = []
    for relative in inventory:
        path = str(relative).replace("\\", "/")
        lowered = path.casefold()
        basename = lowered.rsplit("/", 1)[-1]
        if basename == "native_monitor.json":
            # The native WSL monitor is the required Linux resource evidence,
            # not engine diagnostic output.
            continue
        components = set(lowered.split("/"))
        if (
            {"marg_data", "marg-data", "trace", "traces", "timing", "timings"} & components
            or "marg_data" in basename
            or "marg-data" in basename
            or basename.startswith("trace")
            or ".trace" in basename
            or basename.startswith("timing")
            or ".timing" in basename
            or "diagnostic" in basename
        ):
            forbidden.append(path)
    return sorted(forbidden)


def _pre_prune_absence_evidence(
    output_policy: str,
    inventory: Mapping[str, Any],
    diagnostic_paths: Sequence[str],
) -> dict[str, Any]:
    """Describe the output-policy check made before any retention pruning."""

    lean = output_policy == OUTPUT_POLICY_LEAN
    return {
        "checked": True,
        "phase": "after_engine_exit_before_evaluation_and_prune",
        "policy": output_policy,
        "inventory_complete": True,
        "inventory_file_count": len(inventory),
        "diagnostic_paths": list(diagnostic_paths),
        "expected_absent": lean,
        "proved_absent": bool(lean and not diagnostic_paths),
    }


def _process_tree_rss(pid: int) -> int:
    try:
        import psutil  # type: ignore

        try:
            process = psutil.Process(pid)
        except psutil.NoSuchProcess:
            return 0
        try:
            processes = [process, *process.children(recursive=True)]
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            return 0
        total = 0
        for child in processes:
            try:
                total += int(child.memory_info().rss)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        return total
    except (ImportError, OSError):
        return 0


def _terminate_process(process: subprocess.Popen[str]) -> None:
    if process.poll() is not None:
        return
    if os.name == "nt":
        try:
            subprocess.run(
                ["taskkill", "/PID", str(process.pid), "/T", "/F"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=10,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            process.kill()
    else:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except (OSError, ProcessLookupError):
            process.kill()


def _is_ground_truth_environment_key(key: Any) -> bool:
    """Recognize forbidden GT env names without broad substring false positives."""

    normalized = re.sub(r"[^a-z0-9]+", "_", str(key).casefold()).strip("_")
    if normalized in {"gt", "gt_root", "euroc_gt"}:
        return True
    return any(
        marker in normalized
        for marker in ("ground_truth", "groundtruth", "gt_path", "gt_file", "gt_csv")
    )


def _ground_truth_environment_keys(environment: Mapping[str, Any]) -> list[str]:
    return sorted(
        (str(key) for key in environment if _is_ground_truth_environment_key(key)),
        key=str.casefold,
    )


def deterministic_environment(seed: int, threads: int) -> dict[str, str]:
    values = os.environ.copy()
    # Remove common GT-looking variables inherited from CI or a user's shell.
    # The exact GT/GT_ROOT/EUROC_GT spellings are included so casing and
    # separator variants cannot silently supply evaluator data to the engine.
    for key in list(values):
        if _is_ground_truth_environment_key(key):
            values.pop(key, None)
    if _ground_truth_environment_keys(values):
        raise CoordinatorError("ground-truth environment key survived engine environment sanitization")
    values.update(
        {
            "PYTHONHASHSEED": str(seed),
            "OMP_NUM_THREADS": str(threads),
            "OPENBLAS_NUM_THREADS": str(threads),
            "MKL_NUM_THREADS": str(threads),
            "RAYON_NUM_THREADS": str(threads),
            "TBB_NUM_THREADS": str(threads),
            "VISLOC_BASALT_SEED": str(seed),
            "VISLOC_BASALT_TEMPORAL_SEED": str(seed),
            "VISLOC_BASALT_THREADS": str(threads),
        }
    )
    return values


def _find_trajectory(engine_output: Path, candidates: Sequence[str]) -> Path | None:
    for candidate in candidates:
        path = engine_output / candidate
        if path.is_file():
            return path
    for candidate in ("trajectory.csv", "trajectory.tum", "trajectory.txt"):
        path = engine_output / candidate
        if path.is_file():
            return path
    return None


def _find_trajectories(engine_output: Path, candidates: Sequence[str]) -> list[Path]:
    found: list[Path] = []
    seen: set[Path] = set()
    for name in (*candidates, "trajectory.tum", "trajectory.csv", "trajectory.txt"):
        path = engine_output / name
        if path.is_file() and path not in seen:
            seen.add(path)
            found.append(path)
    return found


def _trajectory_candidates_for_cell(
    engine_output: Path,
    attempt_dir: Path,
    candidates: Sequence[str],
    *,
    native: bool,
) -> list[Path]:
    found = _find_trajectories(engine_output, candidates)
    if native:
        # Basalt's ``--save-trajectory euroc`` writes trajectory.csv in the
        # process cwd, while MargData is written below --marg-data.
        for path in _find_trajectories(attempt_dir, candidates):
            if path not in found:
                found.append(path)
    return found


def _validate_native_monitor(
    monitor_path: Path,
    *,
    engine_output: Path,
    expected_provenance: Mapping[str, Any] | None,
    expected_max_frames: int | None,
    expected_poll_seconds: float | None = None,
) -> dict[str, Any]:
    """Load and fail closed on the native wrapper's Linux process metrics."""

    monitor_path = _lexical_absolute(monitor_path)
    engine_output = _lexical_absolute(engine_output)
    try:
        monitor_path.resolve(strict=False).relative_to(engine_output.resolve(strict=False))
    except ValueError as exc:
        raise CoordinatorError("native monitor path escapes engine output") from exc
    if not monitor_path.is_file():
        raise CoordinatorError(f"native monitor is missing: {monitor_path}")
    monitor = _load_json(monitor_path)
    if monitor.get("schema") != NATIVE_MONITOR_SCHEMA:
        raise CoordinatorError(f"native monitor schema mismatch: {monitor.get('schema')!r}")
    if monitor.get("platform") != "wsl-linux":
        raise CoordinatorError("native monitor is not a Linux/WSL process-tree record")
    if monitor.get("output_policy") != OUTPUT_POLICY_LEAN or monitor.get("no_marg_data") is not True or monitor.get("no_trace") is not True:
        raise CoordinatorError("native monitor output policy is not trajectory-only lean")
    if monitor.get("ground_truth_command_tokens") is not False:
        raise CoordinatorError("native monitor did not prove GT-free native argv")
    try:
        returncode = int(monitor.get("returncode"))
        wall_seconds = float(monitor.get("wall_seconds"))
        peak_rss = int(monitor.get("peak_process_tree_rss_bytes"))
        poll_seconds = float(monitor.get("resource_poll_seconds"))
    except (TypeError, ValueError) as exc:
        raise CoordinatorError("native monitor has malformed resource metrics") from exc
    if returncode != 0 or not (wall_seconds >= 0.0) or not (peak_rss >= 0) or not (poll_seconds > 0.0):
        raise CoordinatorError("native monitor has invalid returncode/resource metrics")
    if expected_poll_seconds is not None and poll_seconds != float(expected_poll_seconds):
        raise CoordinatorError("native monitor poll interval does not match the plan")
    command = monitor.get("command")
    if not isinstance(command, list) or not command or not all(isinstance(token, str) for token in command):
        raise CoordinatorError("native monitor command is missing or malformed")
    _validate_engine_command(command)
    if "--marg-data" in {token.casefold().split("=", 1)[0] for token in command}:
        raise CoordinatorError("native monitor command contains --marg-data")
    try:
        trajectory_index = command.index("--save-trajectory")
    except ValueError as exc:
        raise CoordinatorError("native monitor command omitted --save-trajectory euroc") from exc
    if trajectory_index + 1 >= len(command) or command[trajectory_index + 1].casefold() != "euroc":
        raise CoordinatorError("native monitor command must use --save-trajectory euroc")
    if expected_max_frames is not None:
        try:
            index = command.index("--max-frames")
        except ValueError as exc:
            raise CoordinatorError("native monitor command omitted requested --max-frames") from exc
        if index + 1 >= len(command) or command[index + 1] != str(expected_max_frames):
            raise CoordinatorError("native monitor max-frames does not match request")
    binding = monitor.get("native_binding")
    if not isinstance(binding, Mapping):
        raise CoordinatorError("native monitor native_binding is missing")
    if binding.get("checkout_dirty") is not False or binding.get("dirty_paths") != []:
        raise CoordinatorError("native monitor reports a dirty checkout")
    if expected_provenance is not None:
        checks = (
            ("commit", binding.get("commit"), expected_provenance.get("git_head")),
            ("tree_sha1", binding.get("tree_sha1"), expected_provenance.get("git_tree_sha1")),
            ("binary.sha256", (binding.get("binary") or {}).get("sha256"), expected_provenance.get("executable_sha256")),
            ("config.sha256", (binding.get("config") or {}).get("sha256"), expected_provenance.get("config_sha256")),
            ("calibration.sha256", (binding.get("calibration") or {}).get("sha256"), expected_provenance.get("calibration_sha256")),
            ("wrapper.sha256", (binding.get("wrapper") or {}).get("sha256"), expected_provenance.get("wrapper_sha256")),
        )
        for label, actual, expected in checks:
            if not isinstance(expected, str) or str(actual).casefold() != expected.casefold():
                raise CoordinatorError(f"native monitor binding mismatch for {label}")
    return monitor


def _validate_rust_wsl_monitor(
    monitor_path: Path,
    *,
    engine_output: Path,
    expected_binding: Mapping[str, Any] | None,
    expected_max_frames: int | None,
    expected_poll_seconds: float,
) -> dict[str, Any]:
    """Validate the Linux process-tree record emitted by the Rust WSL twin."""

    monitor_path = _lexical_absolute(monitor_path)
    engine_output = _lexical_absolute(engine_output)
    try:
        monitor_path.resolve(strict=False).relative_to(engine_output.resolve(strict=False))
    except ValueError as exc:
        raise CoordinatorError("Rust WSL monitor path escapes engine output") from exc
    if not monitor_path.is_file():
        raise CoordinatorError(f"Rust WSL monitor is missing: {monitor_path}")
    monitor = _load_json(monitor_path)
    if monitor.get("schema") != RUST_WSL_MONITOR_SCHEMA:
        raise CoordinatorError(f"Rust WSL monitor schema mismatch: {monitor.get('schema')!r}")
    if monitor.get("platform") != "wsl-linux" or monitor.get("runtime_profile") != RUST_RUNTIME_PROFILE_WSL:
        raise CoordinatorError("Rust WSL monitor is not a rust_wsl_linux record")
    if (
        monitor.get("output_policy") != OUTPUT_POLICY_LEAN
        or monitor.get("no_marg_data") is not True
        or monitor.get("no_trace") is not True
    ):
        raise CoordinatorError("Rust WSL monitor output policy is not trajectory-only lean")
    if monitor.get("ground_truth_command_tokens") is not False:
        raise CoordinatorError("Rust WSL monitor did not prove GT-free argv")
    try:
        returncode = int(monitor.get("returncode"))
        wall_seconds = float(monitor.get("wall_seconds"))
        peak_rss = int(monitor.get("peak_process_tree_rss_bytes"))
        poll_seconds = float(monitor.get("resource_poll_seconds"))
    except (TypeError, ValueError) as exc:
        raise CoordinatorError("Rust WSL monitor has malformed resource metrics") from exc
    if (
        returncode != 0
        or wall_seconds < 0.0
        or peak_rss < 0
        or poll_seconds <= 0.0
        or poll_seconds != float(expected_poll_seconds)
    ):
        raise CoordinatorError("Rust WSL monitor resource metrics/poll do not match the plan")
    if monitor.get("rss_measurement_domain") != RSS_AUTHORITATIVE_DOMAIN:
        raise CoordinatorError("Rust WSL monitor RSS domain is not Linux /proc")
    if monitor.get("rss_measurement_source") != "rust_wsl_runner:/proc":
        raise CoordinatorError("Rust WSL monitor RSS source is not rust_wsl_runner:/proc")
    if monitor.get("rss_authoritative") is not True:
        raise CoordinatorError("Rust WSL monitor did not mark Linux RSS authoritative")
    command = monitor.get("command")
    if not isinstance(command, list) or not command or not all(isinstance(token, str) for token in command):
        raise CoordinatorError("Rust WSL monitor command is missing or malformed")
    _validate_engine_command(command)
    lowered = {token.casefold().split("=", 1)[0] for token in command}
    if "--marg-data" in lowered or "--no-marg-data" not in lowered or "--no-trace" not in lowered:
        raise CoordinatorError("Rust WSL monitor command is not lean/no-output")
    if expected_max_frames is not None:
        try:
            index = command.index("--max-frames")
        except ValueError as exc:
            raise CoordinatorError("Rust WSL monitor omitted requested --max-frames") from exc
        if index + 1 >= len(command) or command[index + 1] != str(expected_max_frames):
            raise CoordinatorError("Rust WSL monitor max-frames does not match request")
    actual = monitor.get("rust_wsl_binding")
    if not isinstance(actual, Mapping) or not isinstance(expected_binding, Mapping):
        raise CoordinatorError("Rust WSL monitor binding is missing")
    for label, expected, observed in (
        ("binary.sha256", (expected_binding.get("binary") or {}).get("sha256"), (actual.get("binary") or {}).get("sha256")),
        ("config.sha256", (expected_binding.get("config") or {}).get("sha256"), (actual.get("config") or {}).get("sha256")),
        ("calibration.sha256", (expected_binding.get("calibration") or {}).get("sha256"), (actual.get("calibration") or {}).get("sha256")),
        ("source_sha256", expected_binding.get("source_sha256"), actual.get("source_sha256")),
        ("provenance_file.sha256", (expected_binding.get("provenance_file") or {}).get("sha256"), actual.get("provenance_sha256")),
        ("exactness_certificate.sha256", (expected_binding.get("exactness_certificate") or {}).get("sha256"), actual.get("exactness_certificate_sha256")),
    ):
        if not _valid_sha256(expected) or str(observed).casefold() != str(expected).casefold():
            raise CoordinatorError(f"Rust WSL monitor binding mismatch for {label}")
    if actual.get("target_triple") != expected_binding.get("target_triple"):
        raise CoordinatorError("Rust WSL monitor target-triple binding mismatch")
    return monitor


def _summary_path(engine_output: Path) -> Path | None:
    for name in ("summary.txt", "summary.json", "run_summary.json", "result.json"):
        path = engine_output / name
        if path.is_file():
            return path
    return None


def _prune_success_output(
    retention_root: Path,
    *,
    representative: bool,
    trajectories: Sequence[Path],
    summary: Path | None = None,
    preserve_logs: bool = True,
    extra_keep: Sequence[Path] = (),
) -> None:
    """Apply default retention below the current attempt only.

    Native Basalt writes ``trajectory.csv`` and diagnostic ``stats_*`` files
    in the process cwd, while Rust writes its files below ``engine_output``.
    Pruning from the attempt root handles both layouts and never traverses
    outside the current cell attempt.
    """

    if representative:
        return
    keep = {path.resolve(strict=False) for path in trajectories}
    if summary is None:
        summary = _summary_path(retention_root)
    if summary is None:
        for candidate in retention_root.rglob("*"):
            if candidate.is_file() and candidate.name.casefold() in {
                "summary.txt",
                "summary.json",
                "run_summary.json",
                "result.json",
            }:
                summary = candidate
                break
    if summary is not None:
        keep.add(summary.resolve(strict=False))
    keep.update(path.resolve(strict=False) for path in extra_keep if path.is_file())
    if preserve_logs:
        keep.update(
            path.resolve(strict=False)
            for path in retention_root.rglob("*")
            if path.is_file() and path.name.casefold() in {"stdout.log", "stderr.log"}
        )
    for path in sorted((item for item in retention_root.rglob("*") if item.is_file()), reverse=True):
        if path.resolve(strict=False) not in keep:
            path.unlink()
    for directory in sorted((item for item in retention_root.rglob("*") if item.is_dir()), reverse=True):
        try:
            directory.rmdir()
        except OSError:
            pass


def _ground_truth_path(
    source_root: Path,
    record: Mapping[str, Any],
    *,
    sensor_only_input_view: bool = False,
) -> Path:
    """Resolve and hash-bind GT only for post-engine evaluation.

    Sensor-only runs must use the manifest's physical root.  The engine-visible
    view is intentionally not searched because it must not contain a GT entry.
    Legacy direct fixtures retain the historical source-root fallback when the
    strict flag is not enabled.
    """

    ground_truth = record.get("ground_truth")
    relative = ground_truth.get("path") if isinstance(ground_truth, Mapping) else None
    if not isinstance(relative, str) or not relative:
        relative = "mav0/state_groundtruth_estimate0/data.csv"
    relative_path = Path(relative)
    if relative_path.is_absolute() or ".." in relative_path.parts:
        raise CoordinatorError(f"ground-truth manifest path must be relative: {relative!r}")

    if sensor_only_input_view:
        physical_root = _physical_root_from_record(record)
        source_resolved = _lexical_absolute(source_root).resolve(strict=False)
        physical_resolved = physical_root.resolve(strict=False)
        if source_resolved == physical_resolved:
            raise CoordinatorError("sensor-only evaluator root must differ from the engine-visible view")
        path = (physical_root / relative_path).resolve(strict=False)
        try:
            path.relative_to(physical_resolved)
        except ValueError as exc:
            raise CoordinatorError(f"ground truth escapes manifest physical root: {path}") from exc
        expected_hash = ground_truth.get("sha256") if isinstance(ground_truth, Mapping) else None
        if not isinstance(expected_hash, str) or not re.fullmatch(r"[0-9a-fA-F]{64}", expected_hash):
            raise CoordinatorError("sensor-only evaluator requires a manifest-bound ground-truth SHA-256")
        if not path.is_file():
            raise CoordinatorError(f"ground truth is missing for post-exit evaluation: {path}")
        actual_hash = sha256_file(path)
        if actual_hash.casefold() != expected_hash.casefold():
            raise CoordinatorError(
                f"ground-truth hash mismatch for post-exit evaluation: {path} ({actual_hash} != {expected_hash})"
            )
        return path

    path = (source_root / relative_path).resolve(strict=False)
    if not path.is_file():
        raise CoordinatorError(f"ground truth is missing for post-exit evaluation: {path}")
    return path


def _evaluate_after_exit(
    *,
    trajectory: Path,
    ground_truth: Path,
    evaluation_path: Path,
    max_diff_ns: int,
    tum_time_unit: str,
    engine_exit_observed: bool = True,
) -> dict[str, Any]:
    if not engine_exit_observed:
        raise CoordinatorError("post-exit evaluation cannot start before engine exit")
    raw_path = evaluation_path.with_name(f".{evaluation_path.stem}.raw.{os.getpid()}.json")
    normalized_path: Path | None = None
    evaluation_trajectory = trajectory
    try:
        first_data_line = next(
            (
                line.strip()
                for line in trajectory.read_text(encoding="utf-8", errors="replace").splitlines()
                if line.strip() and not line.lstrip().startswith("#")
            ),
            "",
        )
        first_header = next(
            (
                line.strip()
                for line in trajectory.read_text(encoding="utf-8", errors="replace").splitlines()
                if line.strip()
            ),
            "",
        )
        # Native Basalt emits EuRoC CSV columns named p_RS_R_*; the shared
        # evaluator deliberately accepts TUM or visloc CSV instead.  Convert
        # this standard native form only after engine exit, preserving the
        # original CSV as the retained artifact.
        if "," in first_data_line or "," in first_header:
            header = first_header.lstrip("#").strip().split(",")
            native_fields = {field.strip().casefold() for field in header}
            if "p_rs_r_x [m]" in native_fields or "tx" in native_fields:
                normalized_path = evaluation_path.with_name(f".{trajectory.stem}.normalized.tum")
                with trajectory.open("r", encoding="utf-8", errors="replace", newline="") as stream:
                    rows = csv.reader(stream)
                    raw_header = next(rows, [])
                    normalized_header = [field.lstrip("#").strip().casefold() for field in raw_header]
                    indices = {field: index for index, field in enumerate(normalized_header)}
                    timestamp_key = "timestamp [ns]" if "timestamp [ns]" in indices else "timestamp_ns"
                    if timestamp_key not in indices:
                        timestamp_key = "timestamp"
                    position_keys = (
                        ("p_rs_r_x [m]", "p_rs_r_y [m]", "p_rs_r_z [m]", "q_rs_w []", "q_rs_x []", "q_rs_y []", "q_rs_z []")
                        if "p_rs_r_x [m]" in indices
                        else ("tx", "ty", "tz", "qw", "qx", "qy", "qz")
                    )
                    required_keys = (timestamp_key, *position_keys)
                    if any(key not in indices for key in required_keys):
                        raise ValueError("trajectory CSV does not contain recognizable pose columns")
                    with normalized_path.open("w", encoding="utf-8") as output:
                        for row in rows:
                            if not row or len(row) <= max(indices[key] for key in required_keys):
                                continue
                            timestamp = float(row[indices[timestamp_key]]) * (1.0e-9 if timestamp_key != "timestamp" or abs(float(row[indices[timestamp_key]])) > 1.0e12 else 1.0)
                            values = [row[indices[key]] for key in position_keys]
                            output.write(f"{timestamp:.18f} {' '.join(values)}\n")
                evaluation_trajectory = normalized_path
    except (OSError, ValueError, StopIteration):
        normalized_path = None
    command = [
        sys.executable,
        str(ROOT / "scripts" / "evaluate_euroc_trajectory.py"),
        "--ground-truth-csv",
        str(ground_truth),
        "--trajectory",
        str(evaluation_trajectory),
        "--max-diff-ns",
        str(max_diff_ns),
        "--tum-time-unit",
        _trajectory_time_unit(evaluation_trajectory, tum_time_unit),
        "--out-json",
        str(raw_path),
    ]
    try:
        result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=300, check=False)
        if result.returncode != 0:
            raise CoordinatorError(
                f"post-exit evaluator failed ({result.returncode}): {result.stderr.strip()[-1000:]}"
            )
        payload = _load_json(raw_path)
        payload["schema_id"] = EVALUATION_SCHEMA_ID
        payload["coordinator_schema_version"] = 1
        payload["ground_truth_used_after_engine_exit"] = True
        payload["ground_truth"] = {"path": str(ground_truth), "sha256": sha256_file(ground_truth)}
        payload["evaluator_command"] = command
        payload["input_trajectory"] = str(trajectory)
        for run in payload.get("runs", []):
            if isinstance(run, dict):
                run["trajectory"] = str(trajectory.resolve(strict=False))
        atomic_write_json(evaluation_path, payload)
        return payload
    finally:
        if raw_path.exists():
            raw_path.unlink()
        if normalized_path is not None and normalized_path.exists():
            normalized_path.unlink()


def _trajectory_time_unit(path: Path, configured: str) -> str:
    """Infer the common seconds-form TUM output used by the Rust demo."""

    if path.suffix.casefold() == ".tum":
        return "s"
    try:
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            token = line.split(",", 1)[0].split()[0]
            # Nanosecond TUM files are integer-like 1e18 values.  A decimal
            # token in a whitespace trajectory is conventionally seconds.
            if "." in token and abs(float(token)) < 1.0e12:
                return "s"
            break
    except (OSError, ValueError, IndexError):
        pass
    return configured


def _matching_completed(cell: Cell, manifest_path: Path, evaluation_path: Path) -> bool:
    if not manifest_path.is_file() or not evaluation_path.is_file():
        return False
    try:
        manifest = _load_json(manifest_path)
        if manifest.get("status") != "success" or manifest.get("request_sha256") != cell.request_sha256:
            return False
        for key in ("output_policy", "no_marg_data", "no_trace"):
            if manifest.get(key) != cell.request.get(key):
                return False
        persisted_request = manifest.get("request")
        if not isinstance(persisted_request, Mapping):
            return False
        for key in (
            "runtime_context",
            "executable_binding",
            "command_executable_binding",
            "rust_executable_sha256",
            "timing_feature_binding",
            "rust_runtime_profile",
            "rust_wsl_binding",
            "rss_gate_candidate_eligible",
        ):
            if persisted_request.get(key) != cell.request.get(key):
                return False
        expected_resume_identity = _resume_identity(cell.request)
        persisted_resume_identity = manifest.get("resume_identity")
        if not isinstance(persisted_resume_identity, Mapping):
            # Artifacts produced before explicit session/pair/SHA binding are
            # never eligible for a formal resume.
            return False
        if dict(persisted_resume_identity) != expected_resume_identity:
            return False
        if cell.method == METHOD_RUST:
            if cell.request.get("rust_runtime_profile", RUST_RUNTIME_PROFILE_MSVC) == RUST_RUNTIME_PROFILE_WSL:
                _validate_rust_wsl_request_binding(cell.request)
            elif not _local_file_binding_matches_current(cell.request.get("executable_binding")):
                return False
        if cell.method == METHOD_RUST:
            execution = manifest.get("execution")
            argv = execution.get("argv") if isinstance(execution, Mapping) else None
            if not isinstance(argv, list) or not argv:
                return False
            if cell.request.get("rust_runtime_profile", RUST_RUNTIME_PROFILE_MSVC) == RUST_RUNTIME_PROFILE_WSL:
                execution_monitor = execution.get("rust_wsl_monitor") if isinstance(execution, Mapping) else None
                if not isinstance(execution_monitor, str):
                    return False
                _validate_rust_wsl_monitor(
                    manifest_path.parent / execution_monitor,
                    engine_output=manifest_path.parent / str(
                        (manifest.get("attempt") or {}).get("engine_output", "")
                    ),
                    expected_binding=cell.request.get("rust_wsl_binding"),
                    expected_max_frames=cell.request.get("max_frames"),
                    expected_poll_seconds=float(cell.request.get("poll_seconds", DEFAULT_POLL_SECONDS)),
                )
            else:
                actual_command_binding = _validate_rust_command_binding(
                    argv,
                    cell.request.get("executable_binding"),
                    required=True,
                )
                planned_command_binding = cell.request.get("command_executable_binding")
                if not isinstance(planned_command_binding, Mapping):
                    return False
                if (
                    actual_command_binding.get("sha256") != planned_command_binding.get("sha256")
                    or actual_command_binding.get("bytes") != planned_command_binding.get("bytes")
                ):
                    return False
        artifacts = manifest.get("artifacts", {})
        if not isinstance(artifacts, Mapping):
            return False
        if artifacts.get("pre_prune_inventory_complete") is not True:
            return False
        if cell.request.get("output_policy", OUTPUT_POLICY_LEAN) == OUTPUT_POLICY_LEAN:
            if artifacts.get("pre_prune_absence_proof") is not True:
                return False
            if artifacts.get("pre_prune_diagnostic_artifacts") != []:
                return False
        for relative, metadata in artifacts.get("retained_files", {}).items():
            path = manifest_path.parent / str(relative)
            if not path.is_file() or not isinstance(metadata, Mapping):
                return False
            if metadata.get("sha256") != sha256_file(path):
                return False
        return True
    except (OSError, ValueError, json.JSONDecodeError):
        return False


def run_cell(
    cell: Cell,
    *,
    protocol_path: Path,
    dataset_manifest: Mapping[str, Any] | None,
    config_path: Path,
    calibration_path: Path,
    spec: MethodSpec,
    max_frames: int | None,
    threads: int,
    seed: int,
    timeout_seconds: float,
    poll_seconds: float,
    trajectory_candidates: Sequence[str] = ("trajectory.tum", "trajectory.csv", "trajectory.txt"),
    max_diff_ns: int = 10_000_000,
    tum_time_unit: str = "ns",
    representative: bool = False,
    preserve_logs: bool = True,
    resume: bool = True,
    native_is_available: bool | None = None,
    session_reused: bool = False,
) -> dict[str, Any]:
    run_dir = cell.run_dir
    sensor_only_input_view = bool(cell.request.get("sensor_only_input_view"))
    sensor_input_view_mode = str(
        cell.request.get(
            "sensor_input_view_mode",
            SENSOR_VIEW_MODE_JUNCTION if sensor_only_input_view else SENSOR_VIEW_MODE_DIRECT,
        )
    )
    output_policy = str(cell.request.get("output_policy", OUTPUT_POLICY_LEAN))
    no_marg_data, no_trace = _policy_flags(output_policy)
    dataset_request = cell.request.get("dataset") if isinstance(cell.request.get("dataset"), Mapping) else {}
    sensor_bytes_copied = int(cell.request.get("sensor_bytes_copied", dataset_request.get("sensor_bytes_copied", 0)) or 0)
    native_wrapper_mode = cell.method == METHOD_NATIVE and spec.command_template is None and spec.executable_kind == "native_wsl"
    rust_wsl_wrapper_mode = cell.method == METHOD_RUST and spec.command_template is None and spec.executable_kind == "rust_wsl"
    linux_wrapper_mode = native_wrapper_mode or rust_wsl_wrapper_mode
    manifest_path = run_dir / "run_manifest.json"
    evaluation_path = run_dir / "evaluation_result.json"
    _revalidate_sensor_view_before_launch(cell, dataset_manifest)
    if resume and _matching_completed(cell, manifest_path, evaluation_path):
        return {
            "method": cell.method,
            "sequence": cell.sequence,
            "repetition": cell.repetition,
            "status": "skipped",
            "skip_reason": "matching_completed_manifest",
            "run_dir": str(run_dir),
            "request_sha256": cell.request_sha256,
            "executable_binding": cell.request.get("executable_binding"),
            "timing_feature_binding": cell.request.get("timing_feature_binding"),
            "rust_runtime_profile": cell.request.get("rust_runtime_profile", RUST_RUNTIME_PROFILE_MSVC),
            "rust_wsl_binding": cell.request.get("rust_wsl_binding"),
            "rss_gate_candidate_eligible": cell.request.get("rss_gate_candidate_eligible", False),
            "output_policy": output_policy,
            "no_marg_data": no_marg_data,
            "no_trace": no_trace,
            "runtime_context": cell.request.get("runtime_context"),
            "resume_identity": cell.request.get("resume_identity", _resume_identity(cell.request)),
            "resume_context": {
                "session_reused": bool(session_reused),
                "cell_reused": True,
            },
            "sensor_input_view_mode": sensor_input_view_mode,
            "sensor_bytes_copied": sensor_bytes_copied,
            "staged_sensor_only": sensor_input_view_mode == SENSOR_VIEW_MODE_HARDLINK,
        }

    run_dir.mkdir(parents=True, exist_ok=True)
    _recover_stale_running_marker(run_dir)
    attempt_dir = _attempt_dir(run_dir)
    _archive_failure_marker(run_dir, attempt_dir)
    engine_output = attempt_dir / "engine_output"
    attempt_dir.mkdir(parents=True, exist_ok=False)
    running_marker = run_dir / "running.json"
    failure_marker = run_dir / "failure.json"
    attempt_values = _command_values(
        source_root=cell.source_root,
        engine_output=engine_output,
        config=config_path,
        calibration=calibration_path,
        max_frames=max_frames,
        threads=threads,
        seed=seed,
        sequence=cell.sequence,
        method=cell.method,
    )
    if spec.command_template is None:
        command = _default_command(
            spec,
            source_root=cell.source_root,
            engine_output=engine_output,
            config=config_path,
            calibration=calibration_path,
            max_frames=max_frames,
            threads=threads,
            seed=seed,
            sequence=cell.sequence,
            output_policy=output_policy,
            poll_seconds=poll_seconds,
        )
    else:
        command = _render_template(spec.command_template, attempt_values)
    _validate_engine_command(command)
    _validate_output_policy_command(cell.method, command, output_policy)
    if cell.method == METHOD_RUST:
        if rust_wsl_wrapper_mode:
            _validate_rust_wsl_request_binding(cell.request)
            planned_command_binding = cell.request.get("command_executable_binding")
            if not isinstance(planned_command_binding, Mapping):
                raise CoordinatorError("Rust WSL command is missing its runner binding")
        else:
            actual_command_binding = _validate_rust_command_binding(
                command,
                cell.request.get("executable_binding"),
                required=True,
            )
            planned_command_binding = cell.request.get("command_executable_binding")
            if isinstance(planned_command_binding, Mapping) and (
                actual_command_binding.get("sha256") != planned_command_binding.get("sha256")
                or actual_command_binding.get("bytes") != planned_command_binding.get("bytes")
            ):
                raise CoordinatorError(
                    "Rust command argv[0] content changed between planning and launch"
                )
    inherited_ground_truth_environment_keys = _ground_truth_environment_keys(os.environ)
    env = deterministic_environment(seed, threads)
    remaining_ground_truth_environment_keys = _ground_truth_environment_keys(env)
    if remaining_ground_truth_environment_keys:
        raise CoordinatorError(
            "ground-truth environment keys survived engine environment sanitization: "
            + ", ".join(remaining_ground_truth_environment_keys)
        )
    _validate_timing_feature_binding(
        cell.request.get("timing_feature_binding"),
        method=cell.method,
        environment=env,
    )
    # Repeat the check after attempt setup and immediately before spawn to
    # close the link/file TOCTOU window opened by planning and preparation.
    _revalidate_sensor_view_before_launch(cell, dataset_manifest)
    executable_available = (
        native_is_available
        if cell.method == METHOD_NATIVE and native_is_available is not None
        else _method_available(spec, command)
    )
    base_manifest = {
        "schema_id": RUN_SCHEMA_ID,
        "schema_version": 1,
        "protocol_id": PROTOCOL_ID,
        "run_id": f"{cell.method}:{cell.sequence}:r{cell.repetition}",
        "method": cell.method,
        "sequence": cell.sequence,
        "family": cell.family,
        "repetition": cell.repetition,
        "request": cell.request,
        "request_sha256": cell.request_sha256,
        "executable_binding": cell.request.get("executable_binding"),
        "timing_feature_binding": cell.request.get("timing_feature_binding"),
        "rust_runtime_profile": cell.request.get("rust_runtime_profile", RUST_RUNTIME_PROFILE_MSVC),
        "rust_wsl_binding": cell.request.get("rust_wsl_binding"),
        "rss_gate_candidate_eligible": cell.request.get("rss_gate_candidate_eligible", False),
        "runtime_context": cell.request.get("runtime_context"),
        "resume_identity": cell.request.get("resume_identity", _resume_identity(cell.request)),
        "formal_denominator": bool(cell.request.get("formal_denominator", output_policy == OUTPUT_POLICY_LEAN)),
        "input_namespace": cell.request.get("input_namespace", _input_namespace(cell.request)),
        "resume_context": {
            "session_reused": bool(session_reused),
            "cell_reused": False,
        },
        "output_policy": output_policy,
        "no_marg_data": no_marg_data,
        "no_trace": no_trace,
        "representative_diagnostic": output_policy == OUTPUT_POLICY_DIAGNOSTIC,
        "input": {
            "mode": "sensor_only" if sensor_only_input_view else "direct",
            "direct_source_root": str(cell.source_root),
            "resolved_source_root": str(cell.source_root_resolved),
            "junction_or_symlink": str(cell.source_root) != str(cell.source_root_resolved),
            "expected_cam0_frames": cell.expected_frames,
            "sensor_fingerprint": cell.sensor_fingerprint,
            "per_cell_sensor_copy": False,
            "sensor_only_input_view": sensor_only_input_view,
            "sensor_input_view_mode": sensor_input_view_mode,
            "sensor_bytes_copied": sensor_bytes_copied,
            "staged_sensor_only": sensor_input_view_mode == SENSOR_VIEW_MODE_HARDLINK,
            "sensor_view_fingerprint": dataset_request.get("sensor_view_fingerprint"),
            "sensor_view_evidence": dataset_request.get("sensor_view_evidence"),
            "gt_entry_absence": dataset_request.get("gt_entry_absence"),
            "input_namespace": cell.request.get("input_namespace", _input_namespace(cell.request)),
        },
        "ground_truth_firewall": {
            "ground_truth_path_passed_to_engine": False,
            "ground_truth_environment_keys_passed": False,
            "ground_truth_environment_keys_removed": inherited_ground_truth_environment_keys,
            "ground_truth_environment_keys_remaining": remaining_ground_truth_environment_keys,
            "engine_command_contains_ground_truth": False,
            "direct_source_contains_gt_tree": not sensor_only_input_view,
            "sensor_only_gt_entry_absent": sensor_only_input_view,
            "strict_gt_absence_verified": bool(cell.request.get("strict_gt_absence_verified", False)),
            "gt_absence_fingerprint": cell.request.get("gt_absence_fingerprint"),
            "sensor_input_view_mode": sensor_input_view_mode,
            "post_exit_ground_truth_source": "manifest.physical_root" if sensor_only_input_view else "source_root",
            "post_exit_ground_truth_hash_bound": sensor_only_input_view,
            "evaluation_started_after_engine_exit": True,
        },
        "determinism": {
            "seed": seed,
            "threads": threads,
            "environment_seed_keys": ["PYTHONHASHSEED", "VISLOC_BASALT_SEED", "VISLOC_BASALT_TEMPORAL_SEED"],
            "environment_thread_keys": ["OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "RAYON_NUM_THREADS", "TBB_NUM_THREADS", "VISLOC_BASALT_THREADS"],
            "seed_command_bound": any("--seed" in token or "{seed}" in token for token in command),
            "threads_command_bound": any("--num-threads" in token or "{threads}" in token for token in command),
            "rust_current_cli_seed_thread_support": False if cell.method == METHOD_RUST else None,
        },
        "attempt": {"path": str(attempt_dir), "engine_output": str(engine_output)},
    }
    atomic_write_json(running_marker, {**base_manifest, "status": "running", "created_utc": utc_now()})
    started_utc = utc_now()
    started = time.perf_counter()
    rss_peak = 0
    returncode: int | None = None
    timed_out = False
    stdout_path = attempt_dir / "stdout.log"
    stderr_path = attempt_dir / "stderr.log"
    if not executable_available:
        unavailable_inventory = _artifact_inventory(attempt_dir)
        unavailable_diagnostic_artifacts = _diagnostic_artifact_paths(unavailable_inventory)
        failure = {
            **base_manifest,
            "status": "dnf",
            "failure_reason": "executable_unavailable",
            "created_utc": utc_now(),
            "artifacts": {
                "attempt_root": _safe_relative(attempt_dir, run_dir),
                "engine_output_root": _safe_relative(engine_output, run_dir),
                "pre_prune_inventory_complete": True,
                "pre_prune_inventory": unavailable_inventory,
                "pre_prune_diagnostic_artifacts": unavailable_diagnostic_artifacts,
                "pre_prune_absence_proof": bool(
                    output_policy != OUTPUT_POLICY_LEAN or not unavailable_diagnostic_artifacts
                ),
                "pre_prune_absence_evidence": _pre_prune_absence_evidence(
                    output_policy,
                    unavailable_inventory,
                    unavailable_diagnostic_artifacts,
                ),
                "retained_files": {},
            },
        }
        atomic_write_json(failure_marker, failure)
        atomic_write_json(manifest_path, failure)
        running_marker.unlink(missing_ok=True)
        return {
            "method": cell.method,
            "sequence": cell.sequence,
            "repetition": cell.repetition,
            "status": "dnf",
            "failure_reason": "executable_unavailable",
            "run_dir": str(run_dir),
            "executable_binding": cell.request.get("executable_binding"),
            "timing_feature_binding": cell.request.get("timing_feature_binding"),
            "runtime_context": cell.request.get("runtime_context"),
            "resume_context": {
                "session_reused": bool(session_reused),
                "cell_reused": False,
            },
            "sensor_input_view_mode": sensor_input_view_mode,
        }
    try:
        attempt_dir.mkdir(parents=True, exist_ok=True)
        engine_output.mkdir(parents=True, exist_ok=True)
        with stdout_path.open("w", encoding="utf-8", errors="replace") as stdout, stderr_path.open("w", encoding="utf-8", errors="replace") as stderr:
            process = subprocess.Popen(
                list(command),
                cwd=str(attempt_dir),
                env=env,
                stdout=stdout,
                stderr=stderr,
                text=True,
                start_new_session=(os.name != "nt"),
            )
            running_document = _load_json(running_marker)
            running_document["coordinator_pid"] = os.getpid()
            running_document["engine_pid"] = process.pid
            atomic_write_json(running_marker, running_document)
            while True:
                returncode = process.poll()
                rss_peak = max(rss_peak, _process_tree_rss(process.pid))
                elapsed = time.perf_counter() - started
                if returncode is not None:
                    break
                if elapsed >= timeout_seconds:
                    timed_out = True
                    _terminate_process(process)
                    returncode = process.wait(timeout=30)
                    break
                time.sleep(poll_seconds)
    except (OSError, subprocess.SubprocessError) as exc:
        returncode = -1
        failure_reason = f"process_start_or_wait_error: {exc}"
    else:
        failure_reason = "timeout" if timed_out else (f"engine_returncode_{returncode}" if returncode != 0 else None)
    finished_utc = utc_now()
    outer_wsl_wall_seconds = time.perf_counter() - started
    outer_wsl_rss_peak = rss_peak
    # This coordinator-host process-tree wall value is available for both
    # native (including its WSL shim) and Rust cells.  It is the only common
    # runtime domain; native inner-Linux wall time remains diagnostic.
    common_runtime_wall_seconds = outer_wsl_wall_seconds
    native_monitor: dict[str, Any] | None = None
    native_monitor_error: str | None = None
    rust_wsl_monitor: dict[str, Any] | None = None
    rust_wsl_monitor_error: str | None = None
    wall_seconds = outer_wsl_wall_seconds
    effective_rss_peak = outer_wsl_rss_peak
    native_monitor_path = engine_output / "native_monitor.json"
    rust_wsl_monitor_path = engine_output / "rust_wsl_monitor.json"
    if native_wrapper_mode:
        try:
            expected_provenance = cell.request.get("native_provenance")
            native_monitor = _validate_native_monitor(
                native_monitor_path,
                engine_output=engine_output,
                expected_provenance=expected_provenance if isinstance(expected_provenance, Mapping) else None,
                expected_max_frames=max_frames,
                expected_poll_seconds=poll_seconds,
            )
            wall_seconds = float(native_monitor["wall_seconds"])
            effective_rss_peak = int(native_monitor["peak_process_tree_rss_bytes"])
        except (CoordinatorError, OSError, ValueError, json.JSONDecodeError) as exc:
            native_monitor_error = f"native_monitor_error: {exc}"
    if rust_wsl_wrapper_mode:
        try:
            rust_wsl_monitor = _validate_rust_wsl_monitor(
                rust_wsl_monitor_path,
                engine_output=engine_output,
                expected_binding=cell.request.get("rust_wsl_binding"),
                expected_max_frames=max_frames,
                expected_poll_seconds=poll_seconds,
            )
            wall_seconds = float(rust_wsl_monitor["wall_seconds"])
            effective_rss_peak = int(rust_wsl_monitor["peak_process_tree_rss_bytes"])
        except (CoordinatorError, OSError, ValueError, json.JSONDecodeError) as exc:
            rust_wsl_monitor_error = f"rust_wsl_monitor_error: {exc}"
    if native_monitor_error is not None:
        failure_reason = native_monitor_error
    elif rust_wsl_monitor_error is not None:
        failure_reason = rust_wsl_monitor_error
    linux_monitor = native_monitor if native_monitor is not None else rust_wsl_monitor
    rss_authoritative = linux_monitor is not None and linux_wrapper_mode
    trajectories = _trajectory_candidates_for_cell(
        engine_output,
        attempt_dir,
        trajectory_candidates,
        native=cell.method == METHOD_NATIVE,
    )
    trajectory = trajectories[0] if trajectories else None
    success = (
        returncode == 0
        and not timed_out
        and bool(trajectories)
        and native_monitor_error is None
        and rust_wsl_monitor_error is None
    )
    # Record all engine output before evaluation/pruning. A lean cell that
    # emitted diagnostics must fail closed even if prune would remove them.
    pre_prune_inventory = _artifact_inventory(attempt_dir)
    pre_prune_diagnostic_artifacts = _diagnostic_artifact_paths(pre_prune_inventory)
    pre_prune_absence_evidence = _pre_prune_absence_evidence(
        output_policy,
        pre_prune_inventory,
        pre_prune_diagnostic_artifacts,
    )
    pre_prune_absence_proof = bool(pre_prune_absence_evidence["proved_absent"])
    if output_policy == OUTPUT_POLICY_LEAN and pre_prune_diagnostic_artifacts:
        success = False
        failure_reason = (
            "output_policy_violation: lean cell emitted diagnostic artifacts before prune: "
            + ", ".join(pre_prune_diagnostic_artifacts)
        )
    evaluation: dict[str, Any] | None = None
    if success:
        try:
            record = _manifest_records(dataset_manifest).get(cell.sequence, {})
            ground_truth = _ground_truth_path(
                cell.source_root,
                record,
                sensor_only_input_view=sensor_only_input_view,
            )
            evaluation_error: Exception | None = None
            for candidate in trajectories:
                try:
                    evaluation = _evaluate_after_exit(
                        trajectory=candidate,
                        ground_truth=ground_truth,
                        evaluation_path=evaluation_path,
                        max_diff_ns=max_diff_ns,
                        tum_time_unit=_trajectory_time_unit(candidate, tum_time_unit),
                    )
                    trajectory = candidate
                    break
                except (CoordinatorError, OSError, ValueError, json.JSONDecodeError) as exc:
                    evaluation_error = exc
            if evaluation is None:
                raise CoordinatorError(str(evaluation_error or "all trajectory candidates failed evaluation"))
            evaluation["output_policy"] = output_policy
            evaluation["pre_prune_inventory_complete"] = True
            evaluation["pre_prune_diagnostic_artifacts"] = pre_prune_diagnostic_artifacts
            evaluation["pre_prune_absence_proof"] = pre_prune_absence_proof
            evaluation["pre_prune_absence_evidence"] = pre_prune_absence_evidence
            evaluation["executable_binding"] = cell.request.get("executable_binding")
            evaluation["timing_feature_binding"] = cell.request.get("timing_feature_binding")
            atomic_write_json(evaluation_path, evaluation)
        except (CoordinatorError, OSError, ValueError, json.JSONDecodeError) as exc:
            success = False
            failure_reason = f"post_exit_evaluation_error: {exc}"
    if success and trajectory is not None:
        summary = _summary_path(engine_output)
        if summary is None:
            summary = engine_output / "summary.json"
            atomic_write_json(
                summary,
                {
                    "method": cell.method,
                    "sequence": cell.sequence,
                    "repetition": cell.repetition,
                    "frames_requested": max_frames,
                    "expected_input_frames": cell.expected_frames,
                    "wall_seconds": wall_seconds,
                    "peak_process_tree_rss_bytes": effective_rss_peak,
                },
            )
        _prune_success_output(
            attempt_dir,
            representative=representative,
            trajectories=(trajectory,),
            summary=summary,
            preserve_logs=preserve_logs,
            extra_keep=(
                (native_monitor_path if native_wrapper_mode else rust_wsl_monitor_path,)
                if linux_wrapper_mode
                else ()
            ),
        )
    # A process that exits cleanly but cannot produce an evaluable trajectory
    # is still a denominator-preserving DNF (for example max-5 can end before
    # the first three GT timestamps), rather than an infrastructure failure.
    status = (
        "success"
        if success
        else "dnf"
        if timed_out
        or returncode not in {0, None}
        or str(failure_reason or "").startswith("post_exit_evaluation_error")
        or str(failure_reason or "").startswith("output_policy_violation")
        or native_monitor_error is not None
        or rust_wsl_monitor_error is not None
        else "failure"
    )
    artifacts = {
        "attempt_root": _safe_relative(attempt_dir, run_dir),
        "engine_output_root": _safe_relative(engine_output, run_dir),
        "pre_prune_inventory_complete": True,
        "pre_prune_inventory": pre_prune_inventory,
        "pre_prune_diagnostic_artifacts": pre_prune_diagnostic_artifacts,
        "pre_prune_absence_proof": pre_prune_absence_proof,
        "pre_prune_absence_evidence": pre_prune_absence_evidence,
        "retained_files": {},
    }
    for relative, metadata in _artifact_inventory(attempt_dir).items():
        if success and not preserve_logs and relative.casefold().endswith(("stdout.log", "stderr.log")):
            continue
        artifacts["retained_files"][f"{_safe_relative(attempt_dir, run_dir)}/{relative}"] = metadata
    if evaluation_path.is_file():
        artifacts["retained_files"][_safe_relative(evaluation_path, run_dir)] = {
            "bytes": evaluation_path.stat().st_size,
            "sha256": sha256_file(evaluation_path),
        }
    manifest = {
        **base_manifest,
        "status": status,
        "failure_reason": None if success else failure_reason or "trajectory_missing_or_engine_failure",
        "created_utc": started_utc,
        "finished_utc": finished_utc,
        "execution": {
            "argv": list(command),
            "cwd": str(attempt_dir),
            "returncode": returncode,
            "wall_seconds": wall_seconds,
            "peak_process_tree_rss_bytes": effective_rss_peak,
            "resource_poll_seconds": poll_seconds,
            "measurement_scope": "linux-process-tree" if linux_wrapper_mode else "windows-process-tree",
            "measurement_profile": cell.request.get(
                "rust_runtime_profile", RUST_RUNTIME_PROFILE_MSVC
            ) if cell.method == METHOD_RUST else (
                "native_wsl_linux" if native_wrapper_mode else "native_windows"
            ),
            "rss_measurement_domain": (
                RSS_AUTHORITATIVE_DOMAIN if linux_monitor is not None else "windows-psutil-process-tree"
            ),
            "rss_measurement_source": (
                "native_wsl_runner:/proc" if native_monitor is not None
                else "rust_wsl_runner:/proc" if rust_wsl_monitor is not None
                else "phase6_coordinator:psutil"
            ),
            "rss_authoritative": rss_authoritative,
            "rss_authoritative_domain": RSS_AUTHORITATIVE_DOMAIN if rss_authoritative else None,
            "wall_measurement_domain": (
                "linux-inner-command" if linux_monitor is not None else "windows-process-tree-command"
            ),
            "wall_measurement_source": (
                "native_wsl_runner:monotonic" if native_monitor is not None
                else "rust_wsl_runner:monotonic" if rust_wsl_monitor is not None
                else "phase6_coordinator:perf_counter"
            ),
            "runtime_common_wall_seconds": common_runtime_wall_seconds,
            "runtime_common_measurement_domain": RUNTIME_COMMON_MEASUREMENT_DOMAIN,
            "runtime_common_measurement_source": "phase6_coordinator:perf_counter",
            "outer_wsl_wall_seconds": outer_wsl_wall_seconds if linux_wrapper_mode else None,
            "outer_wsl_peak_process_tree_rss_bytes": outer_wsl_rss_peak if linux_wrapper_mode else None,
            "native_monitor": _safe_relative(native_monitor_path, run_dir) if native_monitor is not None else None,
            "native_monitor_sha256": sha256_file(native_monitor_path) if native_monitor_path.is_file() else None,
            "rust_wsl_monitor": _safe_relative(rust_wsl_monitor_path, run_dir) if rust_wsl_monitor is not None else None,
            "rust_wsl_monitor_sha256": sha256_file(rust_wsl_monitor_path) if rust_wsl_monitor_path.is_file() else None,
            "native_inner_wall_seconds": wall_seconds if native_monitor is not None else None,
            "native_inner_peak_process_tree_rss_bytes": effective_rss_peak if native_monitor is not None else None,
            "started_utc": started_utc,
            "finished_utc": finished_utc,
            "platform": platform.platform(),
            "python": sys.version,
            "stdout_log": _safe_relative(stdout_path, run_dir),
            "stderr_log": _safe_relative(stderr_path, run_dir),
            "timed_out": timed_out,
        },
        "artifacts": artifacts,
        "evaluation": _safe_relative(evaluation_path, run_dir) if evaluation is not None else None,
    }
    if success:
        if not preserve_logs:
            stdout_path.unlink(missing_ok=True)
            stderr_path.unlink(missing_ok=True)
        failure_marker.unlink(missing_ok=True)
    else:
        atomic_write_json(failure_marker, manifest)
    atomic_write_json(manifest_path, manifest)
    running_marker.unlink(missing_ok=True)
    return {
        "method": cell.method,
        "sequence": cell.sequence,
        "repetition": cell.repetition,
        "status": status,
        "failure_reason": manifest.get("failure_reason"),
        "run_dir": str(run_dir),
        "request_sha256": cell.request_sha256,
        "runtime_context": cell.request.get("runtime_context"),
        "resume_identity": cell.request.get("resume_identity", _resume_identity(cell.request)),
        "rust_runtime_profile": cell.request.get("rust_runtime_profile", RUST_RUNTIME_PROFILE_MSVC),
        "rust_wsl_binding": cell.request.get("rust_wsl_binding"),
        "rss_gate_candidate_eligible": cell.request.get("rss_gate_candidate_eligible", False),
        "timing_feature_binding": cell.request.get("timing_feature_binding"),
        "resume_context": {
            "session_reused": bool(session_reused),
            "cell_reused": False,
        },
        "output_policy": output_policy,
        "no_marg_data": no_marg_data,
        "no_trace": no_trace,
        "sensor_input_view_mode": sensor_input_view_mode,
        "sensor_bytes_copied": 0,
        "staged_sensor_only": sensor_input_view_mode == SENSOR_VIEW_MODE_HARDLINK,
        "wall_seconds": wall_seconds,
        # Native wrapper cells expose the inner Linux process-tree metric as
        # the canonical result value; the outer WSL shim sample remains in the
        # run manifest for diagnostics.
        "peak_process_tree_rss_bytes": effective_rss_peak,
        "rss_measurement_domain": RSS_AUTHORITATIVE_DOMAIN if linux_monitor is not None else "windows-psutil-process-tree",
        "rss_measurement_source": (
            "native_wsl_runner:/proc" if native_monitor is not None
            else "rust_wsl_runner:/proc" if rust_wsl_monitor is not None
            else "phase6_coordinator:psutil"
        ),
        "rss_authoritative": rss_authoritative,
        "rss_authoritative_domain": RSS_AUTHORITATIVE_DOMAIN if rss_authoritative else None,
        "wall_measurement_domain": (
            "linux-inner-command" if linux_monitor is not None else "windows-process-tree-command"
        ),
        "runtime_common_wall_seconds": common_runtime_wall_seconds,
        "runtime_common_measurement_domain": RUNTIME_COMMON_MEASUREMENT_DOMAIN,
        "evaluation": str(evaluation_path) if evaluation is not None else None,
    }


def _scheduler_failure(cell: Cell, exc: Exception) -> dict[str, Any]:
    """Turn an unexpected per-cell coordinator exception into a terminal cell.

    A worker exception must not abort the rest of a long matrix.  The normal
    engine/evaluation failure path already writes a complete manifest; this
    fallback writes a compact manifest plus ``failure.json`` when an exception
    escapes that path (for example an unexpected filesystem or serialization
    error).  Existing attempt directories are intentionally preserved.
    """

    run_dir = cell.run_dir
    run_dir.mkdir(parents=True, exist_ok=True)
    reason = f"scheduler_exception: {type(exc).__name__}: {exc}"
    manifest_path = run_dir / "run_manifest.json"
    failure_path = run_dir / "failure.json"
    document = {
        "schema_id": RUN_SCHEMA_ID,
        "schema_version": 1,
        "protocol_id": PROTOCOL_ID,
        "run_id": f"{cell.method}:{cell.sequence}:r{cell.repetition}",
        "method": cell.method,
        "sequence": cell.sequence,
        "family": cell.family,
        "repetition": cell.repetition,
        "request": cell.request,
        "request_sha256": cell.request_sha256,
        "executable_binding": cell.request.get("executable_binding"),
        "timing_feature_binding": cell.request.get("timing_feature_binding"),
        "runtime_context": cell.request.get("runtime_context"),
        "resume_identity": cell.request.get("resume_identity", _resume_identity(cell.request)),
        "input_namespace": cell.request.get("input_namespace", _input_namespace(cell.request)),
        "output_policy": cell.request.get("output_policy", OUTPUT_POLICY_LEAN),
        "no_marg_data": cell.request.get("no_marg_data", True),
        "no_trace": cell.request.get("no_trace", True),
        "status": "failure",
        "failure_reason": reason,
        "scheduler_exception": {"type": type(exc).__name__, "message": str(exc)},
        "finished_utc": utc_now(),
    }
    atomic_write_json(failure_path, document)
    atomic_write_json(manifest_path, document)
    # Leave a running marker in place if one exists.  It is a useful signal to
    # a later resumable invocation that an interrupted worker may need audit;
    # ``_recover_stale_running_marker`` will clear it after PID validation.
    return {
        "method": cell.method,
        "sequence": cell.sequence,
        "repetition": cell.repetition,
        "status": "failure",
        "failure_reason": reason,
        "run_dir": str(run_dir),
        "request_sha256": cell.request_sha256,
        "executable_binding": cell.request.get("executable_binding"),
        "timing_feature_binding": cell.request.get("timing_feature_binding"),
        "output_policy": cell.request.get("output_policy", OUTPUT_POLICY_LEAN),
        "no_marg_data": cell.request.get("no_marg_data", True),
        "no_trace": cell.request.get("no_trace", True),
        "representative_diagnostic": cell.request.get("representative_diagnostic", False),
    }


def _run_cell_guarded(cell: Cell, kwargs: Mapping[str, Any]) -> dict[str, Any]:
    try:
        return run_cell(cell, **dict(kwargs))
    except Exception as exc:  # noqa: BLE001 - preserve matrix denominator
        return _scheduler_failure(cell, exc)


def _number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if number == number and abs(number) != float("inf") else None


def _safe_ratio(numerator: Any, denominator: Any) -> float | None:
    """Return a finite ratio, treating missing/zero metrics as unevaluable."""

    numerator_value = _number(numerator)
    denominator_value = _number(denominator)
    if numerator_value is None or denominator_value is None or denominator_value <= 0:
        return None
    value = numerator_value / denominator_value
    return value if value == value and abs(value) != float("inf") else None


def _cell_evaluation_metrics(cell: Cell) -> dict[str, Any] | None:
    """Read one completed cell's evaluator/manifest into protocol metrics."""

    run_dir = cell.run_dir
    manifest_path = run_dir / "run_manifest.json"
    evaluation_path = run_dir / "evaluation_result.json"
    if not manifest_path.is_file() or not evaluation_path.is_file():
        return None
    try:
        manifest = _load_json(manifest_path)
        evaluation = _load_json(evaluation_path)
    except (CoordinatorError, OSError, ValueError, json.JSONDecodeError):
        return None
    if manifest.get("status") != "success":
        return None
    runs = evaluation.get("runs")
    if not isinstance(runs, list) or not runs or not isinstance(runs[0], Mapping):
        return None
    run = runs[0]
    ate = run.get("ate_translation_se3_m")
    rpe_translation = run.get("rpe_translation_consecutive_m")
    rpe_rotation = run.get("rpe_rotation_consecutive_deg")
    execution = manifest.get("execution")
    input_document = manifest.get("input")
    expected = _number(input_document.get("expected_cam0_frames")) if isinstance(input_document, Mapping) else None
    estimate = _number(run.get("estimate_poses"))
    coverage = _safe_ratio(estimate, expected)
    return {
        "method": cell.method,
        "sequence": cell.sequence,
        "family": cell.family,
        "repetition": cell.repetition,
        "run_dir": str(run_dir),
        "associated_poses": _number(run.get("associated_poses")),
        "estimate_poses": estimate,
        "expected_input_frames": expected,
        "coverage_tracked_fraction": coverage,
        "ate_translation_se3_rmse_m": _number(ate.get("rmse")) if isinstance(ate, Mapping) else None,
        "rpe_translation_consecutive_rmse_m": _number(rpe_translation.get("rmse")) if isinstance(rpe_translation, Mapping) else None,
        "rpe_rotation_consecutive_rmse_deg": _number(rpe_rotation.get("rmse")) if isinstance(rpe_rotation, Mapping) else None,
        "sim3_scale_diagnostic": _number(run.get("sim3_scale")),
        "runtime_wall_seconds": _number(execution.get("wall_seconds")) if isinstance(execution, Mapping) else None,
        "measurement_profile": (
            str(execution.get("measurement_profile"))
            if isinstance(execution, Mapping) and execution.get("measurement_profile")
            else None
        ),
        "peak_process_tree_rss_bytes": _number(execution.get("peak_process_tree_rss_bytes")) if isinstance(execution, Mapping) else None,
        "rss_measurement_domain": (
            str(execution.get("rss_measurement_domain"))
            if isinstance(execution, Mapping) and execution.get("rss_measurement_domain")
            else None
        ),
        "rss_measurement_source": (
            str(execution.get("rss_measurement_source"))
            if isinstance(execution, Mapping) and execution.get("rss_measurement_source")
            else None
        ),
        "rss_authoritative": (
            execution.get("rss_authoritative") is True
            if isinstance(execution, Mapping)
            else False
        ),
        "rss_authoritative_domain": (
            str(execution.get("rss_authoritative_domain"))
            if isinstance(execution, Mapping) and execution.get("rss_authoritative_domain")
            else None
        ),
        "wall_measurement_domain": (
            str(execution.get("wall_measurement_domain"))
            if isinstance(execution, Mapping) and execution.get("wall_measurement_domain")
            else None
        ),
        "wall_measurement_source": (
            str(execution.get("wall_measurement_source"))
            if isinstance(execution, Mapping) and execution.get("wall_measurement_source")
            else None
        ),
        "runtime_common_wall_seconds": (
            _number(execution.get("runtime_common_wall_seconds"))
            if isinstance(execution, Mapping)
            else None
        ),
        "runtime_common_measurement_domain": (
            str(execution.get("runtime_common_measurement_domain"))
            if isinstance(execution, Mapping) and execution.get("runtime_common_measurement_domain")
            else None
        ),
        "runtime_common_measurement_source": (
            str(execution.get("runtime_common_measurement_source"))
            if isinstance(execution, Mapping) and execution.get("runtime_common_measurement_source")
            else None
        ),
        "outer_wsl_wall_seconds": (
            _number(execution.get("outer_wsl_wall_seconds")) if isinstance(execution, Mapping) else None
        ),
        "outer_wsl_peak_process_tree_rss_bytes": (
            _number(execution.get("outer_wsl_peak_process_tree_rss_bytes"))
            if isinstance(execution, Mapping)
            else None
        ),
        "rss_gate_candidate_eligible": (
            manifest.get("rss_gate_candidate_eligible") is True
            or cell.request.get("rss_gate_candidate_eligible") is True
        ),
    }


def _aggregate_values(metrics: Sequence[Mapping[str, Any]], key: str) -> dict[str, float | None]:
    values = [value for item in metrics if (value := _number(item.get(key))) is not None]
    if not values:
        return {"mean": None, "median": None, "worst": None}
    return {
        "mean": sum(values) / len(values),
        "median": float(median(values)),
        "worst": min(values) if key == "coverage_tracked_fraction" else max(values),
    }


def _measurement_values(metrics: Sequence[Mapping[str, Any]], key: str) -> list[str]:
    return sorted(
        {
            str(item[key])
            for item in metrics
            if isinstance(item.get(key), str) and item.get(key)
        }
    )


def _measurement_flags(metrics: Sequence[Mapping[str, Any]], key: str) -> list[bool]:
    return sorted({bool(item.get(key)) for item in metrics})


def _plan_runtime_gate_canonical(plan: Mapping[str, Any]) -> bool:
    """Return the effective canonical flag, including resume recovery policy."""

    override = plan.get("runtime_gate_canonical_override")
    if isinstance(override, bool):
        return override
    request = plan.get("request")
    if not isinstance(request, Mapping):
        return False
    # Missing metadata is unsafe for a formal claim; older plans must not
    # silently become canonical by relying on a permissive default.
    return request.get("runtime_gate_canonical") is True


def aggregate_gate_report(
    plan: Mapping[str, Any],
    results: Sequence[Mapping[str, Any]],
    *,
    protocol_path: Path,
    output_root: Path,
) -> dict[str, Any]:
    """Build a denominator-preserving native-vs-Rust parity gate report."""

    protocol = _load_json(protocol_path)
    gates = protocol.get("parity_gates") if isinstance(protocol.get("parity_gates"), Mapping) else {}
    cells = [cell for cell in plan.get("cells", []) if isinstance(cell, Cell)]
    formal_cells = [
        cell for cell in cells if cell.request.get("output_policy", OUTPUT_POLICY_LEAN) == OUTPUT_POLICY_LEAN
    ]
    diagnostic_cells = [cell for cell in cells if cell not in formal_cells]
    metrics = [_cell_evaluation_metrics(cell) for cell in formal_cells]
    metric_rows = [item for item in metrics if item is not None]
    diagnostic_metrics = [_cell_evaluation_metrics(cell) for cell in diagnostic_cells]
    diagnostic_metric_rows = [item for item in diagnostic_metrics if item is not None]
    metric_names = (
        "ate_translation_se3_rmse_m",
        "rpe_translation_consecutive_rmse_m",
        "rpe_rotation_consecutive_rmse_deg",
        "coverage_tracked_fraction",
        "sim3_scale_diagnostic",
        "runtime_wall_seconds",
        "runtime_common_wall_seconds",
        "peak_process_tree_rss_bytes",
    )
    def make_aggregate(rows: Sequence[Mapping[str, Any]]) -> dict[str, dict[str, dict[str, Any]]]:
        by_method_sequence: dict[tuple[str, str], list[dict[str, Any]]] = {}
        for item in rows:
            by_method_sequence.setdefault((str(item["method"]), str(item["sequence"])), []).append(
                dict(item)
            )
        result: dict[str, dict[str, dict[str, Any]]] = {}
        for method in plan.get("request", {}).get("methods", []):
            for sequence in plan.get("request", {}).get("sequences", []):
                values = by_method_sequence.get((str(method), str(sequence)), [])
                result.setdefault(str(method), {})[str(sequence)] = {
                    "successful_repetitions": len(values),
                    "metrics": {name: _aggregate_values(values, name) for name in metric_names},
                    "measurement": {
                        "rss_domains": _measurement_values(values, "rss_measurement_domain"),
                        "rss_sources": _measurement_values(values, "rss_measurement_source"),
                        "rss_authoritative": _measurement_flags(values, "rss_authoritative"),
                        "rss_authoritative_domains": _measurement_values(
                            values, "rss_authoritative_domain"
                        ),
                        "wall_domains": _measurement_values(values, "wall_measurement_domain"),
                        "wall_sources": _measurement_values(values, "wall_measurement_source"),
                        "runtime_common_domains": _measurement_values(
                            values, "runtime_common_measurement_domain"
                        ),
                        "runtime_common_sources": _measurement_values(
                            values, "runtime_common_measurement_source"
                        ),
                        "outer_wsl_wall_seconds": _aggregate_values(values, "outer_wsl_wall_seconds"),
                        "outer_wsl_peak_process_tree_rss_bytes": _aggregate_values(
                            values, "outer_wsl_peak_process_tree_rss_bytes"
                        ),
                    },
                }
        return result

    aggregate = make_aggregate(metric_rows)
    diagnostic_aggregate = make_aggregate(diagnostic_metric_rows)

    request = plan.get("request") if isinstance(plan.get("request"), Mapping) else {}
    diagnostic_only = bool(plan.get("diagnostic_only") or request.get("diagnostic_only"))
    rust_runtime_profile = request.get("rust_runtime_profile", RUST_RUNTIME_PROFILE_MSVC)
    rss_gate_candidate_eligible = request.get("rss_gate_candidate_eligible") is True
    resource_gate_canonical = plan.get("resource_gate_canonical") is True or request.get("resource_gate_canonical") is True
    matrix = request.get("formal_matrix") if isinstance(request.get("formal_matrix"), Mapping) else {}
    expected_formal_cell_count = request.get("formal_expected_cell_count")
    if diagnostic_only:
        # A separate diagnostic plan intentionally has no formal denominator.
        # Do not infer one from its single diagnostic method/sequence cell.
        expected_formal_cell_count = 0
    elif not isinstance(expected_formal_cell_count, int) or expected_formal_cell_count < 1:
        expected_formal_cell_count = len(request.get("methods", [])) * len(
            request.get("sequences", [])
        ) * int(request.get("repetitions", 0) or 0)
    formal_count_ok = len(formal_cells) == expected_formal_cell_count
    # ``full_all11x3`` is accepted only when reading historical plans.  New
    # plans use the repetition-neutral key and carry their required count.
    full_formal_matrix = bool(matrix.get("full_all11") or matrix.get("full_all11x3"))
    required_full_count = matrix.get("required_full_count")
    if not isinstance(required_full_count, int) or required_full_count < 1:
        required_full_count = FORMAL_MATRIX_CELL_COUNT
    if full_formal_matrix:
        formal_count_ok = formal_count_ok and len(formal_cells) == required_full_count
    formal_count_contract = {
        "status": "pass" if formal_count_ok else "not_evaluable",
        "actual": len(formal_cells),
        "expected": expected_formal_cell_count,
        "required_full_count": required_full_count if full_formal_matrix else None,
        "full_all11": full_formal_matrix,
        "formal_repetitions": matrix.get("formal_repetitions"),
        "legacy_full_all11x3": bool(matrix.get("full_all11x3")),
        "diagnostic_cells_excluded": len(diagnostic_cells),
    }
    namespace_values = [
        cell.request.get("input_namespace", _input_namespace(cell.request))
        for cell in formal_cells
    ]
    namespace_ids = sorted({canonical_hash(value) for value in namespace_values})
    input_namespace_contract = {
        "status": "pass" if len(namespace_ids) == 1 else "not_evaluable",
        "unique_namespace_count": len(namespace_ids),
        "namespace_ids": namespace_ids,
        "observed": namespace_values,
        "detail": None if len(namespace_ids) <= 1 else "direct and staged/sensor-only namespaces must not be mixed",
    }

    def gate(value: bool | None, observed: Any = None, expected: Any = None, detail: str | None = None) -> dict[str, Any]:
        return {"status": "not_evaluable" if value is None else "pass" if value else "fail", "observed": observed, "expected": expected, "detail": detail}

    comparisons: list[dict[str, Any]] = []
    for sequence in plan.get("request", {}).get("sequences", []):
        native = aggregate.get(METHOD_NATIVE, {}).get(str(sequence), {})
        rust = aggregate.get(METHOD_RUST, {}).get(str(sequence), {})
        nm = native.get("metrics", {})
        rm = rust.get("metrics", {})
        native_median = {name: (values.get("median") if isinstance(values, Mapping) else None) for name, values in nm.items()}
        rust_median = {name: (values.get("median") if isinstance(values, Mapping) else None) for name, values in rm.items()}
        native_measurement = native.get("measurement", {}) if isinstance(native, Mapping) else {}
        rust_measurement = rust.get("measurement", {}) if isinstance(rust, Mapping) else {}

        def ratio_gate(name: str, ratio_key: str, floor_key: str) -> dict[str, Any]:
            n = _number(native_median.get(name))
            r = _number(rust_median.get(name))
            ratio = _number(gates.get(ratio_key))
            floor = _number(gates.get(floor_key))
            if n is None or r is None or ratio is None or floor is None:
                return gate(None, {"native": n, "rust": r})
            expected = n * ratio + floor
            return gate(r <= expected, {"native": n, "rust": r}, {"max": expected, "ratio": ratio, "additive_floor": floor})

        coverage_native = _number(native_median.get("coverage_tracked_fraction"))
        coverage_rust = _number(rust_median.get("coverage_tracked_fraction"))
        coverage_drop = _number(gates.get("coverage_max_drop"))
        coverage_observed = coverage_native - coverage_rust if coverage_native is not None and coverage_rust is not None else None
        coverage_gate = gate(
            None if coverage_observed is None or coverage_drop is None else coverage_observed <= coverage_drop,
            {"native": coverage_native, "rust": coverage_rust, "drop": coverage_observed},
            {"max_drop": coverage_drop},
        )
        native_scale = _number(native_median.get("sim3_scale_diagnostic"))
        rust_scale = _number(rust_median.get("sim3_scale_diagnostic"))
        abs_floor = _number(gates.get("scale_abs_diff_max"))
        rel_floor = _number(gates.get("scale_relative_diff_max"))
        scale_diff = abs(rust_scale - native_scale) if native_scale is not None and rust_scale is not None else None
        scale_limit = max(abs_floor or 0.0, abs(native_scale or 0.0) * (rel_floor or 0.0)) if native_scale is not None else None
        scale_gate = gate(
            None if scale_diff is None or scale_limit is None else scale_diff <= scale_limit,
            {"native": native_scale, "rust": rust_scale, "absolute_difference": scale_diff},
            {"max_difference": scale_limit, "absolute_gate": abs_floor, "relative_gate": rel_floor},
        )
        runtime_ratio = _number(gates.get("runtime_ratio_max"))
        rss_ratio = _number(gates.get("rss_ratio_max"))
        native_runtime = _number(native_median.get("runtime_wall_seconds"))
        rust_runtime = _number(rust_median.get("runtime_wall_seconds"))
        native_common_runtime = _number(native_median.get("runtime_common_wall_seconds"))
        rust_common_runtime = _number(rust_median.get("runtime_common_wall_seconds"))
        native_common_runtime_domains = native_measurement.get("runtime_common_domains", [])
        rust_common_runtime_domains = rust_measurement.get("runtime_common_domains", [])
        same_common_runtime_domain = (
            isinstance(native_common_runtime_domains, list)
            and isinstance(rust_common_runtime_domains, list)
            and len(native_common_runtime_domains) == 1
            and len(rust_common_runtime_domains) == 1
            and native_common_runtime_domains[0] == rust_common_runtime_domains[0]
            and native_common_runtime_domains[0] == RUNTIME_COMMON_MEASUREMENT_DOMAIN
        )
        native_rss = _number(native_median.get("peak_process_tree_rss_bytes"))
        rust_rss = _number(rust_median.get("peak_process_tree_rss_bytes"))
        runtime_gate = gate(
            None
            if (
                not same_common_runtime_domain
                or native_common_runtime is None
                or rust_common_runtime is None
                or runtime_ratio is None
            )
            else rust_common_runtime <= native_common_runtime * runtime_ratio,
            {
                "native": native_common_runtime,
                "rust": rust_common_runtime,
                "ratio": _safe_ratio(rust_common_runtime, native_common_runtime),
                "native_domain": native_common_runtime_domains,
                "rust_domain": rust_common_runtime_domains,
                "native_inner_wall_diagnostic": native_runtime,
                "rust_process_wall_diagnostic": rust_runtime,
            },
            {
                "max_ratio": runtime_ratio,
                "required_same_domain": True,
                "required_domain": RUNTIME_COMMON_MEASUREMENT_DOMAIN,
            },
            None
            if same_common_runtime_domain
            else "runtime wall domains differ or common coordinator wall metric is missing",
        )
        native_rss_domains = native_measurement.get("rss_domains", [])
        rust_rss_domains = rust_measurement.get("rss_domains", [])
        same_rss_domain = (
            isinstance(native_rss_domains, list)
            and isinstance(rust_rss_domains, list)
            and len(native_rss_domains) == 1
            and len(rust_rss_domains) == 1
            and native_rss_domains[0] == rust_rss_domains[0]
        )
        native_rss_authoritative = (
            native_measurement.get("rss_authoritative") == [True]
            and native_measurement.get("rss_authoritative_domains") == [RSS_AUTHORITATIVE_DOMAIN]
            and all(
                "proc" in source.casefold()
                for source in native_measurement.get("rss_sources", [])
            )
        )
        rust_rss_authoritative = (
            rust_measurement.get("rss_authoritative") == [True]
            and rust_measurement.get("rss_authoritative_domains") == [RSS_AUTHORITATIVE_DOMAIN]
            and all(
                "proc" in source.casefold()
                for source in rust_measurement.get("rss_sources", [])
            )
        )
        common_authoritative_rss = (
            rss_gate_candidate_eligible
            and rust_runtime_profile == RUST_RUNTIME_PROFILE_WSL
            and resource_gate_canonical
            and
            same_rss_domain
            and native_rss_domains == [RSS_AUTHORITATIVE_DOMAIN]
            and rust_rss_domains == [RSS_AUTHORITATIVE_DOMAIN]
            and native_rss_authoritative
            and rust_rss_authoritative
        )
        rss_evaluable = (
            common_authoritative_rss
            and native_rss is not None
            and rust_rss is not None
            and rss_ratio is not None
        )
        rss_gate = gate(
            None if not rss_evaluable else rust_rss <= native_rss * rss_ratio,
            {
                "native": native_rss,
                "rust": rust_rss,
                "ratio": _safe_ratio(rust_rss, native_rss),
                "native_domain": native_rss_domains,
                "rust_domain": rust_rss_domains,
                "native_authoritative": native_rss_authoritative,
                "rust_authoritative": rust_rss_authoritative,
            },
            {
                "max_ratio": rss_ratio,
                "required_same_domain": True,
                "required_authoritative_domain": RSS_AUTHORITATIVE_DOMAIN,
                "required_source": "Linux /proc process tree",
            },
            None
            if common_authoritative_rss
            else (
                "RSS domains differ; cross-domain RSS ratio is not evaluable"
                if not same_rss_domain
                else (
                    "Rust WSL RSS certificate/canonical resource binding is unavailable"
                    if not rss_gate_candidate_eligible
                    or rust_runtime_profile != RUST_RUNTIME_PROFILE_WSL
                    or not resource_gate_canonical
                    else "RSS requires one shared authoritative Linux /proc process-tree domain"
                )
            ),
        )
        input_namespace_gate = gate(
            True if input_namespace_contract["status"] == "pass" else None,
            input_namespace_contract,
            {"uniform": True},
            input_namespace_contract.get("detail"),
        )
        gate_map = {
            "coverage": coverage_gate,
            "ate_translation_se3": ratio_gate("ate_translation_se3_rmse_m", "ate_se3_ratio_max", "ate_se3_additive_floor_m"),
            "rpe_translation": ratio_gate("rpe_translation_consecutive_rmse_m", "rpe_translation_ratio_max", "rpe_translation_additive_floor_m"),
            "rpe_rotation": ratio_gate("rpe_rotation_consecutive_rmse_deg", "rpe_rotation_ratio_max", "rpe_rotation_additive_floor_deg"),
            "sim3_scale_diagnostic": scale_gate,
            "runtime": runtime_gate,
            "rss": rss_gate,
            "input_namespace": input_namespace_gate,
        }
        statuses = [item["status"] for item in gate_map.values()]
        comparison_status = "fail" if "fail" in statuses else "not_evaluable" if "not_evaluable" in statuses else "pass"
        comparisons.append(
            {
                "sequence": str(sequence),
                "family": family_for_sequence(str(sequence)),
                "native": native,
                "rust": rust,
                "gates": gate_map,
                "status": comparison_status,
            }
        )
    result_counts: dict[str, int] = {}
    for item in results:
        status = str(item.get("status", "unknown"))
        result_counts[status] = result_counts.get(status, 0) + 1
    formal_result_counts: dict[str, int] = {}
    diagnostic_result_counts: dict[str, int] = {}
    for cell, item in zip(cells, results):
        destination = (
            formal_result_counts
            if cell in formal_cells
            else diagnostic_result_counts
        )
        status = str(item.get("status", "unknown"))
        destination[status] = destination.get(status, 0) + 1
    statuses = [str(item["status"]) for item in comparisons]
    # A resumable invocation reports matching cells as ``skipped`` even though
    # their terminal manifests/evaluations are successful.  Gate the matrix
    # against the persisted metric rows, not only this invocation's statuses.
    canonical_runtime = _plan_runtime_gate_canonical(plan)
    overall = (
        "fail"
        if "fail" in statuses
        else "not_evaluable"
        if len(metric_rows) != len(formal_cells)
        or not formal_count_ok
        or not statuses
        or "not_evaluable" in statuses
        or input_namespace_contract["status"] != "pass"
        or not canonical_runtime
        or (
            rust_runtime_profile == RUST_RUNTIME_PROFILE_WSL
            and not resource_gate_canonical
        )
        else "pass"
    )
    return {
        "schema_id": "basalt.phase6.gate_report.v1",
        "schema_version": 1,
        "created_utc": utc_now(),
        "protocol_id": protocol.get("protocol_id"),
        "protocol_path": str(protocol_path),
        "request_sha256": plan.get("request_sha256"),
        "run_count": len(cells),
        "formal_run_count": len(formal_cells),
        "diagnostic_run_count": len(diagnostic_cells),
        "formal_cell_count": len(formal_cells),
        "formal_expected_cell_count": expected_formal_cell_count,
        "formal_count_contract": formal_count_contract,
        "result_counts": result_counts,
        "formal_result_counts": formal_result_counts,
        "diagnostic_result_counts": diagnostic_result_counts,
        "successful_metric_rows": len(metric_rows),
        "missing_metric_rows": len(formal_cells) - len(metric_rows),
        "excluded_diagnostic_cells": [
            f"{cell.method}:{cell.sequence}:r{cell.repetition}" for cell in diagnostic_cells
        ],
        "formal_output_policy": OUTPUT_POLICY_LEAN,
        "diagnostic_output_policy": OUTPUT_POLICY_DIAGNOSTIC,
        "diagnostic_only": diagnostic_only,
        "diagnostic_aggregate": diagnostic_aggregate,
        "diagnostic_metric_rows": len(diagnostic_metric_rows),
        "diagnostic_separate_from_formal": True,
        "cell_order_policy": plan.get("request", {}).get("cell_order_policy"),
        "runtime_gate_canonical": canonical_runtime,
        "runtime_gate_canonical_request": plan.get("request", {}).get("runtime_gate_canonical"),
        "runtime_gate_canonical_override": plan.get("runtime_gate_canonical_override"),
        "rust_runtime_profile": rust_runtime_profile,
        "rss_gate_candidate_eligible": rss_gate_candidate_eligible,
        "resource_gate_canonical": resource_gate_canonical,
        "workers_policy": plan.get("request", {}).get("workers_policy"),
        "fresh_output_root": plan.get("request", {}).get("fresh_output_root"),
        "runtime_gate_requirements": plan.get("request", {}).get("runtime_gate_requirements"),
        "input_namespace": input_namespace_contract,
        "runtime_session": {
            "session_id": plan.get("request", {}).get("run_session_id"),
            "cache_context_id": (
                plan.get("request", {}).get("runtime_context", {}).get("cache_context_id")
                if isinstance(plan.get("request", {}).get("runtime_context"), Mapping)
                else None
            ),
            "pair_context_recorded": all(
                isinstance(cell.request.get("runtime_context"), Mapping) for cell in cells
            ),
        },
        "resource_measurement_contract": {
            "rss_gate_requires_same_domain": True,
            "rss_authoritative_domain": RSS_AUTHORITATIVE_DOMAIN,
            "rss_authoritative_source": "Linux /proc process tree for both methods",
            "native_inner_rss_domain": RSS_AUTHORITATIVE_DOMAIN,
            "rust_rss_domain": "must_be_linux-proc-process-tree; Windows psutil is auxiliary only",
            "outer_wsl_measurements_are_auxiliary": True,
            "native_outer_wsl_wall_recorded_separately": True,
            "native_outer_wsl_rss_recorded_separately": True,
            "runtime_gate_metric": "runtime_common_wall_seconds",
            "runtime_common_measurement_domain": RUNTIME_COMMON_MEASUREMENT_DOMAIN,
            "runtime_common_measurement_source": "phase6_coordinator:perf_counter",
            "native_inner_wall_is_diagnostic": True,
            "mixed_runtime_wall_domains_are_not_evaluable": True,
        },
        "aggregate": aggregate,
        "comparisons": comparisons,
        "overall_status": overall,
        "output_root": str(output_root),
    }


def run_coordinator(
    *,
    dataset_root: Path,
    output_root: Path,
    protocol_path: Path = DEFAULT_PROTOCOL,
    dataset_manifest_path: Path | None = DEFAULT_DATASET_MANIFEST,
    methods: Iterable[str] = METHODS,
    sequences: Iterable[str] | None = None,
    families: Iterable[str] | None = None,
    repetitions: int = DEFAULT_REPETITIONS,
    max_frames: int | None = None,
    config_path: Path = DEFAULT_CONFIG,
    calibration_path: Path = DEFAULT_CALIBRATION,
    rust_executable: Path | str | None = None,
    rust_runtime_profile: str = RUST_RUNTIME_PROFILE_MSVC,
    rust_wsl_executable: str | None = None,
    rust_wsl_config: str | None = None,
    rust_wsl_calibration: str | None = None,
    rust_wsl_wrapper: Path | str | None = None,
    rust_wsl_provenance_path: Path | str | None = None,
    rust_wsl_exactness_certificate_path: Path | str | None = None,
    native_executable: Path | str | None = None,
    rust_command: str | Sequence[str] | None = None,
    native_command: str | Sequence[str] | None = None,
    threads: int = 1,
    workers: int = 1,
    seed: int = DEFAULT_SEED,
    timeout_seconds: float = DEFAULT_TIMEOUT_SECONDS,
    poll_seconds: float = DEFAULT_POLL_SECONDS,
    trajectory_candidates: Sequence[str] = ("trajectory.tum", "trajectory.csv", "trajectory.txt"),
    max_diff_ns: int = 10_000_000,
    tum_time_unit: str = "ns",
    representative: str | None = None,
    diagnostic_output_root: Path | None = None,
    preserve_logs: bool = True,
    resume: bool = True,
    out_path: Path | None = None,
    sensor_only_input_view: bool = False,
    sensor_input_view_mode: str | None = None,
    run_session_id: str | None = None,
    timing_feature: str | bool | None = None,
) -> Path:
    # Normalize once before constructing common worker kwargs.  Workers run
    # with ``cwd=attempt_dir``; passing a caller-relative config/protocol path
    # here would otherwise turn a valid plan into a runtime DNF.
    dataset_root = _lexical_absolute(Path(dataset_root))
    output_root = _lexical_absolute(Path(output_root))
    rust_runtime_profile = _normalize_rust_runtime_profile(rust_runtime_profile)
    protocol_path = _lexical_absolute(Path(protocol_path))
    config_path = _lexical_absolute(Path(config_path))
    calibration_path = _lexical_absolute(Path(calibration_path))
    if dataset_manifest_path is not None:
        dataset_manifest_path = _lexical_absolute(Path(dataset_manifest_path))
    # Capture whether the output root was pristine before this invocation.  A
    # resumed or pre-populated root is valid for recovery, but cannot satisfy
    # the formal fresh-root runtime gate.
    fresh_output_root = not output_root.exists()
    if rust_executable is not None and rust_runtime_profile == RUST_RUNTIME_PROFILE_MSVC:
        rust_executable = _canonical_executable(rust_executable)
    if native_executable is not None:
        native_executable = _canonical_executable(native_executable, allow_wsl_path=True)
    if diagnostic_output_root is not None:
        diagnostic_output_root = _lexical_absolute(Path(diagnostic_output_root))
    sensor_input_view_mode, sensor_only_input_view = _sensor_mode_for_root(
        dataset_root,
        sensor_input_view_mode,
        sensor_only_input_view=bool(sensor_only_input_view),
    )
    representative = _normalize_representative(representative)
    if representative is not None and diagnostic_output_root is None:
        raise CoordinatorError(
            "representative diagnostics require a separate run/output root; "
            "pass diagnostic_output_root"
        )
    if representative is None and diagnostic_output_root is not None:
        raise CoordinatorError(
            "diagnostic_output_root requires an explicit representative"
        )
    existing_session = _load_session_record(output_root)
    if existing_session is not None:
        # Preserve the original session's fresh-root contract when rebuilding
        # its plan for resume.  The physical directory necessarily exists now;
        # using the current existence bit would change request/cache identity.
        fresh_output_root = existing_session.get("fresh_output_root") is True
    if existing_session is not None and not resume:
        raise CoordinatorError(
            "cannot use --no-resume with an existing phase6 session; choose a fresh output root"
        )
    if existing_session is not None:
        session_id = _runtime_session_id(existing_session["session_id"])
        if run_session_id is not None and _runtime_session_id(run_session_id) != session_id:
            raise CoordinatorError("phase6 resume session id does not match the persisted session")
    elif run_session_id is not None:
        session_id = _runtime_session_id(run_session_id)
    else:
        session_id = uuid.uuid4().hex
    plan = build_plan(
        dataset_root=dataset_root,
        output_root=output_root,
        protocol_path=protocol_path,
        dataset_manifest_path=dataset_manifest_path,
        methods=methods,
        sequences=sequences,
        families=families,
        repetitions=repetitions,
        max_frames=max_frames,
        config_path=config_path,
        calibration_path=calibration_path,
        rust_executable=rust_executable,
        rust_runtime_profile=rust_runtime_profile,
        rust_wsl_executable=rust_wsl_executable,
        rust_wsl_config=rust_wsl_config,
        rust_wsl_calibration=rust_wsl_calibration,
        rust_wsl_wrapper=rust_wsl_wrapper,
        rust_wsl_provenance_path=rust_wsl_provenance_path,
        rust_wsl_exactness_certificate_path=rust_wsl_exactness_certificate_path,
        native_executable=native_executable,
        rust_command=rust_command,
        native_command=native_command,
        threads=threads,
        workers=workers,
        seed=seed,
        timeout_seconds=timeout_seconds,
        poll_seconds=poll_seconds,
        require_sources=True,
        sensor_only_input_view=sensor_only_input_view,
        sensor_input_view_mode=sensor_input_view_mode,
        # The formal plan is always lean-only.  A representative is materialized
        # below as a separate diagnostic plan/root after this plan is built.
        representative=None,
        run_session_id=session_id,
        timing_feature=timing_feature,
        fresh_root=fresh_output_root,
    )
    plan_runtime_context = plan["request"].get("runtime_context", {})
    plan_cache_context_id = plan_runtime_context.get("cache_context_id") if isinstance(plan_runtime_context, Mapping) else None
    if not _valid_sha256(plan_cache_context_id):
        raise CoordinatorError("phase6 plan has no valid runtime cache-context binding")
    if existing_session is not None:
        if existing_session.get("request_sha256") != plan.get("request_sha256"):
            raise CoordinatorError(
                "phase6 resume request identity mismatch (including executable/session/cache context); "
                "use a fresh output root"
            )
        if existing_session.get("cache_context_id") != plan_cache_context_id:
            raise CoordinatorError("phase6 resume cache-context binding mismatch; use a fresh output root")
        expected_rust_sha = (
            plan["request"].get("timing_feature_binding", {}).get("rust_executable_sha256")
            if isinstance(plan["request"].get("timing_feature_binding"), Mapping)
            else None
        )
        if existing_session.get("rust_executable_sha256") != expected_rust_sha:
            raise CoordinatorError(
                "phase6 resume Rust executable content SHA binding mismatch; use a fresh output root"
            )
        # A persisted session proves request identity, not the historical
        # launch order of cells.  Until pair-adjacent execution is persisted
        # and reconstructed, every resumed invocation is recovery-only and
        # cannot claim the fresh canonical timing gate.
        plan["runtime_gate_canonical_override"] = False
        plan["resource_gate_canonical"] = False
        plan["runtime_gate_canonical_override_reason"] = (
            "resumed_session_pair_adjacency_not_reconstructible"
        )
    else:
        atomic_write_json(
            output_root / SESSION_FILE_NAME,
            {
                "schema_id": SESSION_SCHEMA_ID,
                "schema_version": 1,
                "session_id": session_id,
                "request_sha256": plan["request_sha256"],
                "cache_context_id": plan_cache_context_id,
                "rust_executable_sha256": (
                    plan["request"].get("timing_feature_binding", {}).get("rust_executable_sha256")
                    if isinstance(plan["request"].get("timing_feature_binding"), Mapping)
                    else None
                ),
                "resume_identity_schema": RESUME_IDENTITY_SCHEMA_ID,
                "cell_order_policy": plan["request"].get("cell_order_policy"),
                "runtime_gate_canonical": plan["request"].get("runtime_gate_canonical"),
                "runtime_gate_canonical_effective": _plan_runtime_gate_canonical(plan),
                "resource_gate_canonical": plan["request"].get("resource_gate_canonical"),
                "runtime_gate_canonical_override": plan.get("runtime_gate_canonical_override"),
        "runtime_gate_canonical_override_reason": plan.get("runtime_gate_canonical_override_reason"),
                "workers_policy": plan["request"].get("workers_policy"),
                "fresh_output_root": fresh_output_root,
                "timing_feature_binding": plan["request"].get("timing_feature_binding"),
                "created_utc": utc_now(),
            },
        )
    manifest = _load_json(dataset_manifest_path) if dataset_manifest_path and Path(dataset_manifest_path).is_file() else None
    specs = _default_method_specs(
        rust_executable=rust_executable,
        rust_runtime_profile=rust_runtime_profile,
        rust_wsl_executable=rust_wsl_executable,
        rust_wsl_config=rust_wsl_config,
        rust_wsl_calibration=rust_wsl_calibration,
        rust_wsl_wrapper=rust_wsl_wrapper,
        rust_wsl_provenance_path=rust_wsl_provenance_path,
        rust_wsl_exactness_certificate_path=rust_wsl_exactness_certificate_path,
        native_executable=native_executable,
        rust_command=rust_command,
        native_command=native_command,
    )
    diagnostic_plan: dict[str, Any] | None = None
    if representative is not None:
        assert diagnostic_output_root is not None
        diagnostic_plan = build_diagnostic_plan(
            plan,
            representative=representative,
            diagnostic_output_root=diagnostic_output_root,
            specs=specs,
            config_path=Path(config_path),
            calibration_path=Path(calibration_path),
            max_frames=max_frames,
            threads=threads,
            seed=seed,
            poll_seconds=poll_seconds,
        )
    results: list[dict[str, Any]] = []
    native_probe = (
        native_available(specs[METHOD_NATIVE].executable)
        if METHOD_NATIVE in plan["request"].get("methods", [])
        else False
    )
    common_kwargs = {
        "protocol_path": Path(protocol_path),
        "dataset_manifest": manifest,
        "config_path": Path(config_path),
        "calibration_path": Path(calibration_path),
        "max_frames": max_frames,
        "threads": threads,
        "seed": seed,
        "timeout_seconds": timeout_seconds,
        "poll_seconds": poll_seconds,
        "trajectory_candidates": trajectory_candidates,
        "max_diff_ns": max_diff_ns,
        "tum_time_unit": tum_time_unit,
        "resume": resume,
        "preserve_logs": preserve_logs,
        "session_reused": existing_session is not None,
    }

    def submit_kwargs(cell: Cell) -> dict[str, Any]:
        values = dict(common_kwargs)
        values.update(
            {
                "spec": specs[cell.method],
                "representative": cell.request.get("output_policy") == OUTPUT_POLICY_DIAGNOSTIC,
                "native_is_available": native_probe if cell.method == METHOD_NATIVE else None,
            }
        )
        return values

    ordered_cells = [cell for cell in plan["cells"] if isinstance(cell, Cell)]
    progress_counts: dict[str, int] = {}

    def record_progress(item: Mapping[str, Any], completed: int) -> None:
        status = str(item.get("status", "unknown"))
        progress_counts[status] = progress_counts.get(status, 0) + 1
        print(
            f"[phase6] completed={completed}/{len(ordered_cells)} method={item.get('method')} "
            f"sequence={item.get('sequence')} repetition=r{item.get('repetition')} status={status} "
            f"counts={json.dumps(progress_counts, sort_keys=True)}",
            flush=True,
        )

    if workers == 1:
        for index, cell in enumerate(ordered_cells, start=1):
            item = _run_cell_guarded(cell, submit_kwargs(cell))
            results.append(item)
            record_progress(item, index)
    else:
        # Each cell owns an isolated attempt directory.  ThreadPoolExecutor
        # limits only cell concurrency; engine-level determinism remains bound
        # by ``threads=1`` and the fixed seed in every child environment/argv.
        with ThreadPoolExecutor(max_workers=workers, thread_name_prefix="phase6-cell") as executor:
            futures = {
                executor.submit(_run_cell_guarded, cell, submit_kwargs(cell)): index
                for index, cell in enumerate(ordered_cells)
            }
            completed_by_index: dict[int, dict[str, Any]] = {}
            for completed, future in enumerate(as_completed(futures), start=1):
                index = futures[future]
                item = future.result()
                completed_by_index[index] = item
                record_progress(item, completed)
            results = [completed_by_index[index] for index in range(len(ordered_cells))]
    diagnostic_result: dict[str, Any] | None = None
    diagnostic_result_path: Path | None = None
    diagnostic_gate_path: Path | None = None
    if diagnostic_plan is not None:
        diagnostic_cell = diagnostic_plan["cells"][0]
        diagnostic_result = _run_cell_guarded(
            diagnostic_cell,
            {
                **common_kwargs,
                "spec": specs[diagnostic_cell.method],
                "representative": True,
                "native_is_available": (
                    native_probe if diagnostic_cell.method == METHOD_NATIVE else None
                ),
                "session_reused": False,
            },
        )
        assert diagnostic_output_root is not None
        diagnostic_output_root.mkdir(parents=True, exist_ok=True)
        diagnostic_gate = aggregate_gate_report(
            diagnostic_plan,
            [diagnostic_result],
            protocol_path=Path(protocol_path),
            output_root=diagnostic_output_root,
        )
        diagnostic_gate_path = diagnostic_output_root / "phase6_diagnostic_gate_report.json"
        atomic_write_json(diagnostic_gate_path, diagnostic_gate)
        diagnostic_result_path = diagnostic_output_root / "phase6_diagnostic_result.json"
        atomic_write_json(
            diagnostic_result_path,
            {
                "schema_id": COORDINATOR_SCHEMA_ID,
                "schema_version": COORDINATOR_VERSION,
                "created_utc": utc_now(),
                "diagnostic_only": True,
                "diagnostic_separate_from_formal": True,
                "formal_cell_count": 0,
                "diagnostic_cell_count": 1,
                "representative": representative,
                "request": diagnostic_plan["request"],
                "request_sha256": diagnostic_plan["request_sha256"],
                "runs": [diagnostic_result],
                "aggregate_gate_report": str(diagnostic_gate_path),
            },
        )
    counts: dict[str, int] = {}
    for result in results:
        counts[result["status"]] = counts.get(result["status"], 0) + 1
    output_root = _lexical_absolute(Path(output_root))
    result = {
        "schema_id": COORDINATOR_SCHEMA_ID,
        "schema_version": COORDINATOR_VERSION,
        "created_utc": utc_now(),
        "request": plan["request"],
        "request_sha256": plan["request_sha256"],
        "run_count": len(results),
        "counts": counts,
        "direct_dataset_no_sensor_copy": True,
        "retention": {"default": ["trajectory", "summary", "evaluation", "manifest", "stdout.log", "stderr.log"], "representative": representative},
        "workers": workers,
        "sensor_only_input_view": sensor_only_input_view,
        "sensor_input_view_mode": sensor_input_view_mode,
        "sensor_bytes_copied": 0,
        "staged_sensor_only": sensor_input_view_mode == SENSOR_VIEW_MODE_HARDLINK,
        "output_policy": plan["request"].get("output_policy", OUTPUT_POLICY_LEAN),
        "no_marg_data": plan["request"].get("no_marg_data", True),
        "no_trace": plan["request"].get("no_trace", True),
        "executable_bindings": plan["request"].get("executable_bindings"),
        "timing_feature_binding": plan["request"].get("timing_feature_binding"),
        "rust_runtime_profile": plan["request"].get(
            "rust_runtime_profile", RUST_RUNTIME_PROFILE_MSVC
        ),
        "rust_wsl_binding": plan["request"].get("rust_wsl_binding"),
        "rss_gate_candidate_eligible": plan["request"].get(
            "rss_gate_candidate_eligible", False
        ),
        "cell_order_policy": plan["request"].get("cell_order_policy"),
        "runtime_gate_canonical": _plan_runtime_gate_canonical(plan),
        "resource_gate_canonical": bool(
            plan.get("resource_gate_canonical", False)
        ),
        "runtime_gate_canonical_request": plan["request"].get("runtime_gate_canonical"),
        "runtime_gate_canonical_override": plan.get("runtime_gate_canonical_override"),
        "runtime_gate_canonical_override_reason": plan.get("runtime_gate_canonical_override_reason"),
        "runtime_gate_requirements": plan["request"].get("runtime_gate_requirements"),
        "workers_policy": plan["request"].get("workers_policy"),
        "fresh_output_root": plan["request"].get("fresh_output_root", fresh_output_root),
        "formal_cell_count": plan["request"].get("formal_cell_count"),
        "formal_expected_cell_count": plan["request"].get("formal_expected_cell_count"),
        "representative": representative,
        "diagnostic_separate_from_formal": diagnostic_plan is not None,
        "diagnostic_result": str(diagnostic_result_path) if diagnostic_result_path else None,
        "diagnostic_gate_report": str(diagnostic_gate_path) if diagnostic_gate_path else None,
        "runtime_session": {
            "session_id": session_id,
            "cache_context_id": plan["request"].get("runtime_context", {}).get("cache_context_id"),
            "session_file": str(output_root / SESSION_FILE_NAME),
            "resumed_existing_session": existing_session is not None,
        },
        "runs": results,
        "storage_estimate": {
            "per_cell_sensor_copy_bytes": 0,
            "additional_sensor_staging_bytes": 0,
            "note": "Existing shared EuRoC source/junction storage is reused; engine working-output size is workload-dependent and non-representative cells are pruned after evaluation.",
        },
    }
    result_path = _lexical_absolute(Path(out_path)) if out_path is not None else output_root / "phase6_result.json"
    gate_path = output_root / "phase6_gate_report.json"
    try:
        gate_report = aggregate_gate_report(
            plan,
            results,
            protocol_path=Path(protocol_path),
            output_root=output_root,
        )
    except Exception as exc:  # noqa: BLE001 - always leave an aggregate artifact
        gate_report = {
            "schema_id": "basalt.phase6.gate_report.v1",
            "schema_version": 1,
            "created_utc": utc_now(),
            "overall_status": "not_evaluable",
            "error": f"aggregate_generation_error: {type(exc).__name__}: {exc}",
            "run_count": len(results),
        }
    atomic_write_json(gate_path, gate_report)
    result["aggregate_gate_report"] = str(gate_path)
    atomic_write_json(result_path, result)
    return result_path


def _parse_command_argument(raw: str | None) -> str | None:
    if raw is None:
        return None
    _parse_template(raw)
    return raw


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--dataset-manifest", type=Path, default=DEFAULT_DATASET_MANIFEST)
    parser.add_argument("--method", "--methods", action="append", help="native_core or rust_current; repeat/comma-separate")
    parser.add_argument("--sequence", "--sequences", action="append")
    parser.add_argument("--family", "--families", action="append", help="MH, V1, V2, or vicon_room")
    parser.add_argument("--repetitions", type=int, default=DEFAULT_REPETITIONS)
    parser.add_argument("--max-frames", type=int)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--rust-executable", type=Path)
    parser.add_argument(
        "--rust-runtime-profile",
        choices=RUST_RUNTIME_PROFILES,
        default=RUST_RUNTIME_PROFILE_MSVC,
        help="Rust execution profile; rust_wsl_linux enables the separate Linux measurement twin",
    )
    parser.add_argument("--rust-wsl-executable")
    parser.add_argument("--rust-wsl-config")
    parser.add_argument("--rust-wsl-calibration")
    parser.add_argument("--rust-wsl-wrapper", type=Path)
    parser.add_argument("--rust-wsl-provenance", type=Path)
    parser.add_argument("--rust-wsl-exactness-certificate", type=Path)
    parser.add_argument("--native-executable")
    parser.add_argument("--rust-command-json")
    parser.add_argument("--native-command-json")
    parser.add_argument(
        "--timing-feature",
        choices=("enabled", "disabled"),
        help=(
            "declare whether the Rust executable was built with Cargo feature "
            "basalt-timing-breakdown; VISLOC_BASALT_TIMING_BREAKDOWN=1 requires enabled"
        ),
    )
    parser.add_argument("--threads", type=int, default=1)
    parser.add_argument("--workers", type=int, default=1, help="maximum concurrent cells; engine threads remain controlled by --threads")
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    parser.add_argument("--timeout-seconds", type=float, default=DEFAULT_TIMEOUT_SECONDS)
    parser.add_argument("--poll-seconds", type=float, default=DEFAULT_POLL_SECONDS)
    parser.add_argument("--trajectory", action="append", default=[])
    parser.add_argument("--max-diff-ns", type=int, default=10_000_000)
    parser.add_argument("--tum-time-unit", choices=("ns", "s"), default="ns")
    parser.add_argument(
        "--representative",
        help="method:SEQUENCE:rN; run one diagnostic cell only in --diagnostic-output-root",
    )
    parser.add_argument(
        "--diagnostic-output-root",
        type=Path,
        help="separate output root for the explicit --representative diagnostic cell",
    )
    parser.add_argument(
        "--diagnostic-plan-out",
        type=Path,
        help="optional plan JSON path for the separate diagnostic cell during --dry-run",
    )
    parser.add_argument("--no-preserve-logs", action="store_true", help="prune per-cell stdout/stderr logs after successful evaluation")
    parser.add_argument("--out", type=Path)
    parser.add_argument("--dry-run", action="store_true", help="write/display a deterministic plan; do not create cell output")
    parser.add_argument("--plan-out", type=Path, help="optional atomic JSON plan path")
    parser.add_argument("--no-resume", action="store_true")
    parser.add_argument(
        "--sensor-only-input-view",
        action="store_true",
        help="require each sequence to expose only cam0/cam1/imu0 sensors and resolve GT post-exit from manifest physical_root",
    )
    parser.add_argument(
        "--sensor-input-view-mode",
        choices=(SENSOR_VIEW_MODE_JUNCTION, SENSOR_VIEW_MODE_HARDLINK),
        help="strict sensor-only view mode: existing junction tree or GT-hidden hardlink_staged tree",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        methods = _flatten(args.method) or list(METHODS)
        sequences = _flatten(args.sequence)
        families = _flatten(args.family)
        representative = _normalize_representative(args.representative)
        if representative is not None and args.diagnostic_output_root is None:
            raise CoordinatorError(
                "--representative requires --diagnostic-output-root; diagnostics are never inline formal cells"
            )
        if representative is None and args.diagnostic_output_root is not None:
            raise CoordinatorError(
                "--diagnostic-output-root requires --representative"
            )
        # A CLI invocation owns one session identity for both its plan and
        # execution. Dry runs intentionally remain unbound and deterministic;
        # resumable runs reuse the persisted session id or allocate one once.
        run_session_id: str | None = None
        if not args.dry_run:
            existing_session = _load_session_record(_lexical_absolute(args.output_root))
            run_session_id = (
                existing_session["session_id"] if existing_session is not None else uuid.uuid4().hex
            )
        plan = build_plan(
            dataset_root=args.dataset_root,
            output_root=args.output_root,
            protocol_path=args.protocol,
            dataset_manifest_path=args.dataset_manifest,
            methods=methods,
            sequences=sequences,
            families=families,
            repetitions=args.repetitions,
            max_frames=args.max_frames,
            config_path=args.config,
            calibration_path=args.calibration,
            rust_executable=args.rust_executable,
            rust_runtime_profile=args.rust_runtime_profile,
            rust_wsl_executable=args.rust_wsl_executable,
            rust_wsl_config=args.rust_wsl_config,
            rust_wsl_calibration=args.rust_wsl_calibration,
            rust_wsl_wrapper=args.rust_wsl_wrapper,
            rust_wsl_provenance_path=args.rust_wsl_provenance,
            rust_wsl_exactness_certificate_path=args.rust_wsl_exactness_certificate,
            native_executable=args.native_executable,
            rust_command=args.rust_command_json,
            native_command=args.native_command_json,
            threads=args.threads,
            workers=args.workers,
            seed=args.seed,
            timeout_seconds=args.timeout_seconds,
            poll_seconds=args.poll_seconds,
            require_sources=not args.dry_run,
            sensor_only_input_view=args.sensor_only_input_view,
            sensor_input_view_mode=args.sensor_input_view_mode,
            # The formal plan is always the complete lean denominator.  The
            # optional diagnostic plan is constructed separately below.
            representative=None,
            run_session_id=run_session_id,
            timing_feature=args.timing_feature,
        )
        document = plan_document(plan)
        diagnostic_plan: dict[str, Any] | None = None
        if representative is not None:
            diagnostic_specs = _default_method_specs(
                rust_executable=args.rust_executable,
                rust_runtime_profile=args.rust_runtime_profile,
                rust_wsl_executable=args.rust_wsl_executable,
                rust_wsl_config=args.rust_wsl_config,
                rust_wsl_calibration=args.rust_wsl_calibration,
                rust_wsl_wrapper=args.rust_wsl_wrapper,
                rust_wsl_provenance_path=args.rust_wsl_provenance,
                rust_wsl_exactness_certificate_path=args.rust_wsl_exactness_certificate,
                native_executable=args.native_executable,
                rust_command=args.rust_command_json,
                native_command=args.native_command_json,
            )
            diagnostic_plan = build_diagnostic_plan(
                plan,
                representative=representative,
                diagnostic_output_root=_lexical_absolute(args.diagnostic_output_root),
                specs=diagnostic_specs,
                config_path=_lexical_absolute(args.config),
                calibration_path=_lexical_absolute(args.calibration),
                max_frames=args.max_frames,
                threads=args.threads,
                seed=args.seed,
                poll_seconds=args.poll_seconds,
            )
            document["diagnostic_separate_from_formal"] = True
            document["diagnostic_plan"] = plan_document(diagnostic_plan)
            if args.diagnostic_plan_out is not None:
                atomic_write_json(
                    _lexical_absolute(args.diagnostic_plan_out),
                    plan_document(diagnostic_plan),
                )
        if args.plan_out is not None:
            atomic_write_json(_lexical_absolute(args.plan_out), document)
        if args.dry_run:
            print(json.dumps(document, indent=2, sort_keys=True))
            return 0
        result_path = run_coordinator(
            dataset_root=args.dataset_root,
            output_root=args.output_root,
            protocol_path=args.protocol,
            dataset_manifest_path=args.dataset_manifest,
            methods=methods,
            sequences=sequences,
            families=families,
            repetitions=args.repetitions,
            max_frames=args.max_frames,
            config_path=args.config,
            calibration_path=args.calibration,
            rust_executable=args.rust_executable,
            rust_runtime_profile=args.rust_runtime_profile,
            rust_wsl_executable=args.rust_wsl_executable,
            rust_wsl_config=args.rust_wsl_config,
            rust_wsl_calibration=args.rust_wsl_calibration,
            rust_wsl_wrapper=args.rust_wsl_wrapper,
            rust_wsl_provenance_path=args.rust_wsl_provenance,
            rust_wsl_exactness_certificate_path=args.rust_wsl_exactness_certificate,
            native_executable=args.native_executable,
            rust_command=args.rust_command_json,
            native_command=args.native_command_json,
            threads=args.threads,
            workers=args.workers,
            seed=args.seed,
            timeout_seconds=args.timeout_seconds,
            poll_seconds=args.poll_seconds,
            trajectory_candidates=tuple(args.trajectory) if args.trajectory else ("trajectory.tum", "trajectory.csv", "trajectory.txt"),
            max_diff_ns=args.max_diff_ns,
            tum_time_unit=args.tum_time_unit,
            representative=representative,
            diagnostic_output_root=args.diagnostic_output_root,
            preserve_logs=not args.no_preserve_logs,
            resume=not args.no_resume,
            out_path=args.out,
            sensor_only_input_view=args.sensor_only_input_view,
            sensor_input_view_mode=args.sensor_input_view_mode,
            run_session_id=run_session_id,
            timing_feature=args.timing_feature,
        )
        print(result_path)
        return 0
    except (CoordinatorError, OSError, ValueError, json.JSONDecodeError) as exc:
        parser.error(str(exc))
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
