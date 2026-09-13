"""Run an isolated no-MargData/no-trace Basalt performance cell.

This reusable runner is kept under ``benchmarks/basalt`` so release and
provenance inventories do not depend on the evidence-only ``work/`` tree. It
fails closed on the frozen protocol/input bindings, removes GT environment
inputs, records exact artifact hashes, and emits a dedicated result plus a
post-exit run manifest. Direct sensor-path execution is explicitly marked as a
performance-mode exception to the staged-copy parity protocol.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import platform
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
from benchmark_process_metrics import run_monitored  # noqa: E402


PROTOCOL_DEFAULT = ROOT / "benchmarks/basalt/protocols/basalt_euroc_parity_v1.json"
DATASET_MANIFEST_DEFAULT = ROOT / "benchmarks/basalt/euroc_dataset_manifest.json"
EQUIVALENCE_DEFAULT = ROOT / "work/m7im15_nomarg_real52_equivalence_20260829.json"
RESULT_SCHEMA = ROOT / "benchmarks/basalt/schemas/clonefree_perf_result_v1.schema.json"
NATIVE_WSL_ADAPTER = ROOT / "benchmarks/basalt/native_wsl_runner.py"
SCHEMA_ID = "basalt.clonefree_perf_result.v1"
PROTOCOL_ID = "basalt-euroc-parity-v1"
UPSTREAM_COMMIT = "0f3b2b52c807f70ff4e2973ce253c73329eea7bc"
UPSTREAM_TREE_SHA1 = "b7afb830d82b45b8209cf784ad9744025d838411"
NATIVE_CHECKOUT_WSL_DEFAULT = "/root/visloc-basalt-clean-m7cr-20260823"
NATIVE_EXECUTABLE_WSL_DEFAULT = (
    f"{NATIVE_CHECKOUT_WSL_DEFAULT}/build/core-relwithdebinfo/basalt_vio"
)
NATIVE_CONFIG_WSL_DEFAULT = f"{NATIVE_CHECKOUT_WSL_DEFAULT}/data/euroc_config.json"
NATIVE_CALIBRATION_WSL_DEFAULT = f"{NATIVE_CHECKOUT_WSL_DEFAULT}/data/euroc_ds_calib.json"
# These are the checked native artifacts used by the pinned WSL checkout on
# the benchmark host.  A caller can override the expected values explicitly,
# but an unbound replacement is never silently accepted.
NATIVE_EXECUTABLE_SHA256_DEFAULT = "89e0324ccd04c3b615bf7e945aa32480a2f86524987aa00eaf152dcd6b1d242c"
NATIVE_CONFIG_SHA256_DEFAULT = "82937bd6493e592ef89572d31260c10f7437b4fb3ff1fda179375713966e34fa"
NATIVE_CALIBRATION_SHA256_DEFAULT = "ad8c5a18c48c55dacf61d18ebbc18cd7d4f3acccd5840bd5a5cd8646adb6271c"
FRAME_GLOB = "mav0/cam0/data/*"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def sha256_json(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
    ).hexdigest()


def utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def require_file(path: Path, label: str) -> Path:
    path = path.resolve()
    if not path.is_file():
        raise FileNotFoundError(f"{label} is missing: {path}")
    return path


def load_json(path: Path, label: str) -> dict[str, Any]:
    require_file(path, label)
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{label} must contain a JSON object: {path}")
    return value


def file_binding(path: Path, *, role: str | None = None, expected_sha256: str | None = None) -> dict[str, Any]:
    path = require_file(path, "artifact")
    result: dict[str, Any] = {"path": str(path), "bytes": path.stat().st_size, "sha256": sha256(path)}
    if role is not None:
        result["role"] = role
    if expected_sha256 is not None:
        result["expected_sha256"] = expected_sha256
        result["expected_sha256_matches"] = result["sha256"].casefold() == expected_sha256.casefold()
    return result


def resolve_reference(value: str) -> Path:
    path = Path(value)
    return path.resolve() if path.is_absolute() else (ROOT / path).resolve()


def windows_to_wsl_path(path: Path) -> str:
    """Convert a local Windows path to the stable WSL mount spelling."""

    resolved = str(path.resolve())
    if len(resolved) < 3 or resolved[1] != ":":
        raise ValueError(f"native-wsl paths must be Windows drive paths: {path}")
    drive = resolved[0].lower()
    rest = resolved[2:].replace("\\", "/")
    return f"/mnt/{drive}{rest}"


def wsl_command(*args: str, check: bool = True) -> subprocess.CompletedProcess[str]:
    """Run a WSL utility with argv-only arguments (never through a shell)."""

    if os.name != "nt":
        raise RuntimeError("native-wsl mode requires Windows WSL2")
    return subprocess.run(
        ["wsl.exe", "-e", *args],
        check=check,
        capture_output=True,
        text=True,
    )


def wsl_file_binding(
    path: str, *, role: str, expected_sha256: str | None = None
) -> dict[str, Any]:
    """Bind one external WSL file by bytes, size, and SHA-256."""

    digest = wsl_command("sha256sum", path).stdout.strip().split()
    if not digest:
        raise ValueError(f"WSL sha256sum returned no digest for {path}")
    size = wsl_command("stat", "-c", "%s", path).stdout.strip()
    try:
        bytes_count = int(size)
    except ValueError as exc:
        raise ValueError(f"WSL stat returned invalid size for {path}: {size!r}") from exc
    result: dict[str, Any] = {
        "path": path,
        "bytes": bytes_count,
        "sha256": digest[0],
        "role": role,
    }
    if expected_sha256 is not None:
        result["expected_sha256"] = expected_sha256
        result["expected_sha256_matches"] = (
            result["sha256"].casefold() == expected_sha256.casefold()
        )
        if not result["expected_sha256_matches"]:
            raise ValueError(f"WSL {role} hash mismatch: {path}")
    return result


def wsl_git_value(checkout: str, *args: str) -> str:
    return wsl_command("git", "-C", checkout, *args).stdout.strip()


def load_native_binding(args: argparse.Namespace) -> dict[str, Any]:
    """Verify and return the pinned native checkout/file bindings."""

    checkout = str(args.native_checkout_wsl)
    commit = wsl_git_value(checkout, "rev-parse", "HEAD")
    tree_sha1 = wsl_git_value(checkout, "rev-parse", "HEAD^{tree}")
    if commit != UPSTREAM_COMMIT:
        raise ValueError(f"native checkout commit is not pinned: {commit}")
    if tree_sha1 != UPSTREAM_TREE_SHA1:
        raise ValueError(f"native checkout tree is not pinned: {tree_sha1}")
    dirty = wsl_git_value(checkout, "status", "--short").splitlines()
    if dirty:
        raise ValueError(
            "native checkout is dirty; use the clean pinned checkout or pass an explicitly verified override: "
            f"{checkout}: {dirty[:8]!r}"
        )
    binary = wsl_file_binding(
        str(args.native_exe_wsl),
        role="native_executable",
        expected_sha256=args.native_binary_sha256,
    )
    config = wsl_file_binding(
        str(args.native_config_wsl),
        role="native_config",
        expected_sha256=args.native_config_sha256,
    )
    calibration = wsl_file_binding(
        str(args.native_calibration_wsl),
        role="native_calibration",
        expected_sha256=args.native_calibration_sha256,
    )
    executable_check = wsl_command("test", "-x", str(args.native_exe_wsl), check=False)
    if executable_check.returncode != 0:
        raise ValueError(f"native executable is not executable: {args.native_exe_wsl}")
    return {
        "checkout_path": checkout,
        "commit": commit,
        "tree_sha1": tree_sha1,
        "expected_commit": UPSTREAM_COMMIT,
        "expected_tree_sha1": UPSTREAM_TREE_SHA1,
        "checkout_dirty": bool(dirty),
        "dirty_paths": dirty,
        "binary": binary,
        "config": config,
        "calibration": calibration,
        "adapter": file_binding(NATIVE_WSL_ADAPTER, role="native_wsl_adapter"),
    }


def native_engine_argv(
    *, executable: str, dataset: str, calibration: str, config: str, max_frames: int
) -> list[str]:
    """Return the complete, auditable native no-output argv."""

    return [
        executable,
        "--dataset-path",
        dataset,
        "--cam-calib",
        calibration,
        "--dataset-type",
        "euroc",
        "--show-gui",
        "0",
        "--config-path",
        config,
        "--save-trajectory",
        "euroc",
        "--num-threads",
        "1",
        "--max-frames",
        str(max_frames),
    ]


def native_csv_to_tum(raw_path: Path, tum_path: Path) -> int:
    """Normalize native ``--save-trajectory euroc`` output to TUM text."""

    with raw_path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.reader(stream))
    if len(rows) < 2 or len(rows[0]) < 8:
        raise ValueError(f"native trajectory is empty or malformed: {raw_path}")
    header = [item.strip() for item in rows[0]]
    aliases = {
        "timestamp_ns": ("timestamp_ns", "#timestamp [ns]"),
        "tx": ("tx", "p_RS_R_x [m]"),
        "ty": ("ty", "p_RS_R_y [m]"),
        "tz": ("tz", "p_RS_R_z [m]"),
        "qw": ("qw", "q_RS_w []"),
        "qx": ("qx", "q_RS_x []"),
        "qy": ("qy", "q_RS_y []"),
        "qz": ("qz", "q_RS_z []"),
    }
    required = list(aliases)
    positions = {
        name: next((header.index(alias) for alias in names if alias in header), -1)
        for name, names in aliases.items()
    }
    positions = {name: index for name, index in positions.items() if index >= 0}
    if len(positions) != len(required):
        raise ValueError(f"native trajectory header lacks pose fields: {raw_path}")
    tum_path.parent.mkdir(parents=True, exist_ok=True)
    count = 0
    with tum_path.open("w", encoding="utf-8", newline="") as stream:
        for row in rows[1:]:
            if len(row) <= max(positions.values()) or not row[positions["timestamp_ns"]].strip():
                continue
            timestamp_ns = int(row[positions["timestamp_ns"]])
            seconds, remainder = divmod(timestamp_ns, 1_000_000_000)
            timestamp = f"{seconds}.{remainder:09d}"
            stream.write(
                " ".join(
                    [
                        timestamp,
                        row[positions["tx"]],
                        row[positions["ty"]],
                        row[positions["tz"]],
                        row[positions["qx"]],
                        row[positions["qy"]],
                        row[positions["qz"]],
                        row[positions["qw"]],
                    ]
                )
                + "\n"
            )
            count += 1
    if count == 0:
        raise ValueError(f"native trajectory contains no pose rows: {raw_path}")
    return count


def write_native_summary(path: Path, *, frames: int, monitor: dict[str, Any]) -> None:
    path.write_text(
        "\n".join(
            [
                "engine_kind=native-wsl",
                "output_policy=no-marg-data,no-trace",
                f"trajectory_rows={frames}",
                f"wall_seconds={monitor.get('wall_seconds', 0)}",
                f"peak_process_tree_rss_bytes={monitor.get('peak_process_tree_rss_bytes', 0)}",
            ]
        )
        + "\n",
        encoding="utf-8",
    )


def collect_output_files(output_root: Path, log_path: Path) -> list[dict[str, Any]]:
    """Bind every produced regular file, including native stats sidecars."""

    roles = {
        "summary.txt": "engine_summary",
        "trajectory.csv": "trajectory_csv",
        "trajectory.tum": "trajectory_tum",
        "native_monitor.json": "native_process_tree_monitor",
    }
    candidates = {path.resolve() for path in output_root.rglob("*") if path.is_file()}
    if log_path.is_file():
        candidates.add(log_path.resolve())
    result: list[dict[str, Any]] = []
    for path in sorted(candidates, key=lambda item: str(item).casefold()):
        role = "stdout_log" if path == log_path.resolve() else roles.get(path.name, "engine_output")
        result.append(file_binding(path, role=role))
    return result


def load_protocol(path: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    protocol = load_json(path, "protocol")
    if protocol.get("protocol_id") != PROTOCOL_ID:
        raise ValueError(f"unexpected protocol id in {path}")
    if protocol.get("upstream", {}).get("commit") != UPSTREAM_COMMIT:
        raise ValueError(f"unexpected pinned upstream commit in {path}")
    policy = protocol.get("execution_policy", {})
    if policy.get("ground_truth_path_is_never_passed_to_engine") is not True:
        raise ValueError("frozen protocol must forbid a GT path in the engine command")
    if policy.get("ground_truth_environment_keys_are_removed") is not True:
        raise ValueError("frozen protocol must remove GT environment inputs")
    return protocol, file_binding(path, role="parity_protocol")


def verify_sensor_inputs(dataset: Path, sequence: dict[str, Any]) -> tuple[list[dict[str, Any]], dict[str, int], int]:
    bindings: list[dict[str, Any]] = []
    hashes = sequence.get("file_hashes", {})
    for group in ("csv_sha256", "sensor_yaml_sha256"):
        for relative, expected in sorted(hashes.get(group, {}).items()):
            if "state_groundtruth" in relative.lower():
                continue
            binding = file_binding(dataset / relative, role="sensor_input", expected_sha256=str(expected))
            if not binding["expected_sha256_matches"]:
                raise ValueError(f"sensor input hash mismatch for {relative}")
            bindings.append(binding)
    image_counts: dict[str, int] = {}
    for camera in ("cam0", "cam1"):
        expected = int(sequence["camera_counts"][camera]["png"])
        image_dir = dataset / "mav0" / camera / "data"
        if not image_dir.is_dir():
            raise FileNotFoundError(f"sensor image directory is missing: {image_dir}")
        actual = sum(1 for path in image_dir.iterdir() if path.is_file() and path.suffix.lower() == ".png")
        if actual != expected:
            raise ValueError(f"{camera} image count mismatch: expected {expected}, got {actual}")
        image_counts[camera] = actual
    return bindings, image_counts, sum(image_counts.values()) + len(bindings)


def load_input_binding(
    dataset: Path, manifest_path: Path, sequence_name: str
) -> tuple[dict[str, Any], dict[str, Any]]:
    manifest = load_json(manifest_path, "dataset manifest")
    if manifest.get("schema_id") != "basalt.euroc_dataset_manifest.v1":
        raise ValueError("unexpected EuRoC dataset manifest schema")
    sequence = next((item for item in manifest.get("sequences", []) if item.get("id") == sequence_name), None)
    if not isinstance(sequence, dict):
        raise ValueError(f"sequence {sequence_name!r} is not present in the dataset manifest")
    sensor_files, image_counts, sensor_file_count = verify_sensor_inputs(dataset, sequence)
    archive = sequence.get("archive", {})
    return {
        "dataset_path": str(dataset),
        "dataset_manifest_path": str(manifest_path.resolve()),
        "dataset_manifest_sha256": sha256(manifest_path),
        "sequence_manifest_sha256": sha256_json(sequence),
        "sequence_archive_path": str(archive.get("path", "")),
        "sequence_archive_sha256": str(archive.get("sha256", "")),
        "sequence_archive_binding": "dataset_manifest_freeze",
        "frame_count_glob": FRAME_GLOB,
        "expected_frame_count": int(sequence["camera_counts"]["cam0"]["csv_rows"]),
        "expected_sensor_file_count": sensor_file_count,
        "image_counts": image_counts,
        "sensor_file_hashes": sensor_files,
    }, sequence


def load_equivalence(path: Path) -> dict[str, Any]:
    evidence = load_json(path, "52-frame equivalence evidence")
    if evidence.get("schema") != "basalt.nomarg_real_equivalence.v1" or int(evidence.get("frames", -1)) != 52:
        raise ValueError("unexpected 52-frame equivalence evidence")
    for key in ("trajectory_tum", "trajectory_csv"):
        if evidence.get(key, {}).get("exact") is not True:
            raise ValueError(f"52-frame {key} equivalence is not exact")
    result: dict[str, Any] = {
        "evidence_path": str(path.resolve()),
        "evidence_bytes": path.stat().st_size,
        "evidence_sha256": sha256(path),
        "schema": evidence["schema"],
        "frames": 52,
        "counts": evidence.get("counts", {}),
        "trajectory_exact": True,
        "timing_usable_for_performance_gate": bool(evidence.get("timing_usable_for_performance_gate", False)),
    }
    for key, filename, role in (
        ("trajectory_tum", "trajectory.tum", "equivalence_trajectory_tum"),
        ("trajectory_csv", "trajectory.csv", "equivalence_trajectory_csv"),
    ):
        expected = evidence[key]
        full = file_binding(
            resolve_reference(str(evidence["full_output"])) / filename,
            role=role,
            expected_sha256=str(expected["sha256"]),
        )
        no_output = file_binding(
            resolve_reference(str(evidence["no_marg_no_trace"])) / filename,
            role=role,
            expected_sha256=str(expected["sha256"]),
        )
        if not full["expected_sha256_matches"] or not no_output["expected_sha256_matches"]:
            raise ValueError(f"52-frame equivalence hash mismatch: {key}")
        if full["sha256"] != no_output["sha256"]:
            raise ValueError(f"52-frame output modes differ: {key}")
        result[key] = {"expected_sha256": str(expected["sha256"]), "full_output": full, "no_marg_no_trace": no_output}
    return result


def is_gt_environment_key(key: str) -> bool:
    upper = key.upper()
    return "GROUND_TRUTH" in upper or upper in {"GT", "GT_PATH", "GT_FILE", "GT_ROOT", "EUROC_GT", "EUROC_GT_PATH"} or upper.endswith(("_GT_PATH", "_GT_FILE", "_GT_ROOT"))


def filtered_environment() -> tuple[dict[str, str], list[str]]:
    environment = os.environ.copy()
    removed = sorted(key for key in environment if is_gt_environment_key(key))
    for key in removed:
        environment.pop(key, None)
    return environment, removed


def present_payload(path: Path) -> bool:
    return path.is_file() or (path.is_dir() and any(path.iterdir()))


def validate_document(document: dict[str, Any], schema_path: Path) -> None:
    required = {"schema_version", "schema_id", "document_kind", "result_schema", "protocol", "run", "input", "build", "execution", "output_policy", "artifacts", "validation", "equivalence_52_frame", "self_binding", "constraints"}
    missing = sorted(required.difference(document))
    if missing:
        raise ValueError("result/run manifest is missing fields: " + ", ".join(missing))
    if document["schema_version"] != 1 or document["schema_id"] != SCHEMA_ID:
        raise ValueError("result/run manifest schema identity is invalid")
    if document["result_schema"]["sha256"] != sha256(schema_path):
        raise ValueError("result schema hash binding is stale")
    for artifact in document["artifacts"]["files"]:
        path = Path(artifact["path"])
        if not path.is_file() or int(artifact["bytes"]) != path.stat().st_size or artifact["sha256"].casefold() != sha256(path).casefold():
            raise ValueError(f"artifact hash/byte binding is invalid: {path}")
    engine_kind = document.get("engine_kind", document.get("run", {}).get("engine_kind", "rust"))
    if engine_kind not in {"rust", "native-wsl"}:
        raise ValueError(f"unsupported engine kind: {engine_kind}")
    if document["run"].get("engine_kind", "rust") != engine_kind:
        raise ValueError("run.engine_kind does not match document engine_kind")
    if engine_kind == "native-wsl":
        build = document.get("build", {})
        native = build.get("native_source")
        if not isinstance(native, dict):
            raise ValueError("native result is missing build.native_source")
        if native.get("commit") != UPSTREAM_COMMIT or native.get("tree_sha1") != UPSTREAM_TREE_SHA1:
            raise ValueError("native source binding is not pinned")
        execution = document.get("execution", {})
        native_argv = execution.get("native_argv")
        if not isinstance(native_argv, list) or "--marg-data" in native_argv:
            raise ValueError("native argv must omit --marg-data")
        if "--show-gui" not in native_argv or "0" not in native_argv:
            raise ValueError("native argv must be headless")
        if "--save-trajectory" not in native_argv or "euroc" not in native_argv:
            raise ValueError("native argv must save an EuRoC trajectory")
        thread_index = native_argv.index("--num-threads") if "--num-threads" in native_argv else -1
        if thread_index < 0 or thread_index + 1 >= len(native_argv) or native_argv[thread_index + 1] != "1":
            raise ValueError("native argv must use one thread")
        monitor = execution.get("native_process_tree")
        if not isinstance(monitor, dict):
            raise ValueError("native execution is missing native process-tree metrics")
        if execution.get("wall_seconds") != monitor.get("wall_seconds"):
            raise ValueError("top-level wall metric is not native process-tree wall")
        if execution.get("peak_process_tree_rss_bytes") != monitor.get("peak_process_tree_rss_bytes"):
            raise ValueError("top-level RSS metric is not native process-tree RSS")


def base_document(**kwargs: Any) -> dict[str, Any]:
    protocol = kwargs["protocol"]
    input_binding = kwargs["input_binding"]
    calibration = kwargs["calibration"]
    config = kwargs["config"]
    executable = kwargs["executable"]
    calibration_binding = kwargs.get("calibration_binding") or file_binding(
        calibration, role="calibration"
    )
    config_binding = kwargs.get("config_binding") or file_binding(config, role="config")
    build_binding = kwargs.get("build_binding") or {
        "executable_path": str(executable),
        "executable_sha256": sha256(executable),
    }
    engine_kind = kwargs.get("engine_kind", "rust")
    return {
        "schema_version": 1,
        "schema_id": SCHEMA_ID,
        "document_kind": kwargs["document_kind"],
        "result_schema": kwargs["result_schema_binding"],
        "protocol": {
            "id": PROTOCOL_ID,
            "path": kwargs["protocol_binding"]["path"],
            "sha256": kwargs["protocol_binding"]["sha256"],
            "upstream_commit": protocol["upstream"]["commit"],
            "execution_policy_audit": {
                "staged_copy_required_by_parity_protocol": True,
                "direct_path_used_for_performance_mode": True,
                "deviation_is_explicit": True,
                "note": "This isolated performance cell uses the verified direct sensor path; it is not a staged-copy parity cell.",
            },
        },
        "engine_kind": engine_kind,
        "run": {"run_id": kwargs["run_id"], "sequence": kwargs["sequence"], "max_frames": kwargs["max_frames"], "mode": "no-marg-data,no-trace", "engine_kind": engine_kind},
        "input": {
            **input_binding,
            "calibration_path": calibration_binding["path"],
            "calibration_sha256": calibration_binding["sha256"],
            "config_path": config_binding["path"],
            "config_sha256": config_binding["sha256"],
            "engine_dataset_path": kwargs.get("engine_dataset_path", str(input_binding["dataset_path"])),
            "ground_truth_firewall": {
                "ground_truth_path_passed_to_engine": False,
                "ground_truth_environment_keys_passed": False,
                "ground_truth_bytes_read_by_engine": False,
                "dataset_root_contains_post_exit_gt_metadata": True,
                "blocked_environment_keys_removed": kwargs["removed_gt_env_keys"],
            },
        },
        "build": build_binding,
        "execution": {**kwargs["execution"], "argv": kwargs["command"], "cwd": str(ROOT), "platform": platform.platform(), "python": sys.version},
        "output_policy": {"no_marg_data": True, "no_trace": True, "flags": kwargs.get("output_flags", ["--no-marg-data", "--no-trace"]), "direct_path_output_root": str(kwargs["output_root"]), "native_marg_data_argument": kwargs.get("native_marg_data_argument", "not-applicable")},
        "artifacts": {"files": kwargs["output_files"], "expected_absent": kwargs["expected_absent"]},
        "validation": kwargs["validation"],
        "equivalence_52_frame": kwargs["equivalence"],
        "status": kwargs["status"],
        "self_binding": {"path": str(kwargs["self_path"].resolve()), "mode": "external_sha256", "note": "The result/run manifest is excluded from its own artifact list; record its final SHA-256 externally."},
        "constraints": {"timing_started": True, "heavy_build_started": False, "commit": False, "push": False, "destructive_actions": False},
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--engine-kind", choices=("rust", "native-wsl"), default="rust")
    parser.add_argument("--exe", type=Path, default=None)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, default=None)
    parser.add_argument("--config", type=Path, default=None)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--result", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, default=None)
    parser.add_argument("--protocol", type=Path, default=PROTOCOL_DEFAULT)
    parser.add_argument("--dataset-manifest", type=Path, default=DATASET_MANIFEST_DEFAULT)
    parser.add_argument("--equivalence-evidence", type=Path, default=EQUIVALENCE_DEFAULT)
    parser.add_argument("--max-frames", type=int, required=True)
    parser.add_argument("--native-checkout-wsl", default=NATIVE_CHECKOUT_WSL_DEFAULT)
    parser.add_argument("--native-exe-wsl", default=NATIVE_EXECUTABLE_WSL_DEFAULT)
    parser.add_argument("--native-config-wsl", default=NATIVE_CONFIG_WSL_DEFAULT)
    parser.add_argument("--native-calibration-wsl", default=NATIVE_CALIBRATION_WSL_DEFAULT)
    parser.add_argument("--native-binary-sha256", default=NATIVE_EXECUTABLE_SHA256_DEFAULT)
    parser.add_argument("--native-config-sha256", default=NATIVE_CONFIG_SHA256_DEFAULT)
    parser.add_argument("--native-calibration-sha256", default=NATIVE_CALIBRATION_SHA256_DEFAULT)
    args = parser.parse_args()

    dataset = args.dataset.resolve()
    if not dataset.is_dir():
        raise FileNotFoundError(f"dataset is missing: {dataset}")
    if args.engine_kind == "rust":
        if args.exe is None or args.calibration is None or args.config is None:
            parser.error("rust mode requires --exe, --calibration, and --config")
        executable = require_file(args.exe, "executable")
        calibration = require_file(args.calibration, "calibration")
        config = require_file(args.config, "config")
    else:
        # WSL paths are not local Windows paths.  They are bound below by
        # sha256sum/stat/git before the engine is launched.
        executable = Path(str(args.native_exe_wsl))
        calibration = Path(str(args.native_calibration_wsl))
        config = Path(str(args.native_config_wsl))
    protocol_path = require_file(args.protocol, "protocol")
    dataset_manifest_path = require_file(args.dataset_manifest, "dataset manifest")
    equivalence_path = require_file(args.equivalence_evidence, "52-frame equivalence evidence")
    schema_path = require_file(RESULT_SCHEMA, "result schema")
    if args.max_frames < 1:
        raise ValueError("--max-frames must be positive")
    output_root = args.out_dir.resolve()
    log_path = args.log.resolve()
    result_path = args.result.resolve()
    manifest_path = args.manifest.resolve() if args.manifest is not None else output_root / "run_manifest.json"
    for path, label in ((log_path, "log"), (result_path, "result"), (manifest_path, "manifest")):
        if path.exists():
            raise FileExistsError(f"refusing to overwrite existing {label}: {path}")
    if output_root.exists() and any(output_root.iterdir()):
        raise FileExistsError(f"output directory is not empty: {output_root}")
    output_root.mkdir(parents=True, exist_ok=True)
    log_path.parent.mkdir(parents=True, exist_ok=True)

    sequence = dataset.name
    protocol, protocol_binding = load_protocol(protocol_path)
    if sequence not in protocol.get("sequences", []):
        raise ValueError(f"sequence {sequence!r} is not in the frozen protocol")
    input_binding, sequence_record = load_input_binding(dataset, dataset_manifest_path, sequence)
    equivalence = load_equivalence(equivalence_path)
    result_schema_binding = file_binding(schema_path, role="result_schema")
    run_id = f"clonefree-no-marg-no-trace/{args.engine_kind}/{sequence}/{output_root.name}"
    native_binding: dict[str, Any] | None = None
    native_argv: list[str] | None = None
    output_flags = ["--no-marg-data", "--no-trace"]
    native_marg_data_argument = "not-applicable"
    engine_dataset_path = str(dataset)
    if args.engine_kind == "rust":
        command = [
            str(executable),
            "--euroc-dir",
            str(dataset),
            "--calibration",
            str(calibration),
            "--config",
            str(config),
            "--out-dir",
            str(output_root),
            "--max-frames",
            str(args.max_frames),
            "--no-marg-data",
            "--no-trace",
        ]
        calibration_binding = file_binding(calibration, role="calibration")
        config_binding = file_binding(config, role="config")
        build_binding = {
            "engine_kind": "rust",
            "executable_path": str(executable),
            "executable_sha256": sha256(executable),
        }
    else:
        native_binding = load_native_binding(args)
        native_argv = native_engine_argv(
            executable=str(args.native_exe_wsl),
            dataset=windows_to_wsl_path(dataset),
            calibration=str(args.native_calibration_wsl),
            config=str(args.native_config_wsl),
            max_frames=args.max_frames,
        )
        adapter_wsl = windows_to_wsl_path(NATIVE_WSL_ADAPTER)
        monitor_wsl = windows_to_wsl_path(output_root / "native_monitor.json")
        output_wsl = windows_to_wsl_path(output_root)
        command = [
            "wsl.exe",
            "-e",
            "python3",
            adapter_wsl,
            "--binary",
            str(args.native_exe_wsl),
            "--config",
            str(args.native_config_wsl),
            "--calibration",
            str(args.native_calibration_wsl),
            "--input-root",
            windows_to_wsl_path(dataset),
            "--output-root",
            output_wsl,
            "--monitor-json",
            monitor_wsl,
            "--checkout",
            str(args.native_checkout_wsl),
            "--max-frames",
            str(args.max_frames),
            "--num-threads",
            "1",
        ]
        calibration_binding = native_binding["calibration"]
        config_binding = native_binding["config"]
        build_binding = {
            "engine_kind": "native-wsl",
            "executable_path": str(args.native_exe_wsl),
            "executable_sha256": native_binding["binary"]["sha256"],
            "native_adapter": native_binding["adapter"],
            "native_source": {
                "checkout_path": native_binding["checkout_path"],
                "commit": native_binding["commit"],
                "tree_sha1": native_binding["tree_sha1"],
                "expected_commit": native_binding["expected_commit"],
                "expected_tree_sha1": native_binding["expected_tree_sha1"],
                "checkout_dirty": native_binding["checkout_dirty"],
                "dirty_paths": native_binding["dirty_paths"],
                "binary": native_binding["binary"],
                "config": native_binding["config"],
                "calibration": native_binding["calibration"],
            },
        }
        output_flags = ["--marg-data omitted", "--show-gui 0", "--save-trajectory euroc", "--num-threads 1"]
        native_marg_data_argument = "omitted"
        engine_dataset_path = windows_to_wsl_path(dataset)
    run_environment, removed_gt_env_keys = filtered_environment()
    started_utc = utc_now()
    monitor = run_monitored(command, log_path, cwd=ROOT, poll_seconds=0.1, env=run_environment)
    native_monitor: dict[str, Any] | None = None
    if args.engine_kind == "native-wsl":
        monitor_path = output_root / "native_monitor.json"
        if monitor_path.is_file():
            native_monitor = load_json(monitor_path, "native WSL monitor")
        else:
            native_monitor = {
                "schema": "basalt.native_wsl_monitor.v1",
                "command": native_argv or [],
                "returncode": monitor["returncode"],
                "wall_seconds": monitor["wall_seconds"],
                "peak_process_tree_rss_bytes": 0,
                "resource_poll_seconds": 0.1,
                "platform": "wsl-linux",
                "adapter_started": False,
                "native_binding": native_binding,
            }
            monitor_path.write_text(json.dumps(native_monitor, indent=2) + "\n", encoding="utf-8")
        execution = {
            **monitor,
            "wall_seconds": native_monitor["wall_seconds"],
            "peak_process_tree_rss_bytes": native_monitor["peak_process_tree_rss_bytes"],
            "started_utc": started_utc,
            "finished_utc": utc_now(),
            "measurement_scope": "native-linux-process-tree",
            "native_process_tree": native_monitor,
            "native_argv": native_monitor.get("command", native_argv or []),
            "outer_wsl_wall_seconds": monitor["wall_seconds"],
            "outer_wsl_peak_process_tree_rss_bytes": monitor["peak_process_tree_rss_bytes"],
        }
    else:
        execution = {**monitor, "started_utc": started_utc, "finished_utc": utc_now()}

    raw_trajectory = output_root / "trajectory.csv"
    tum_trajectory = output_root / "trajectory.tum"
    if args.engine_kind == "native-wsl" and int(monitor["returncode"]) == 0 and raw_trajectory.is_file():
        native_csv_to_tum(raw_trajectory, tum_trajectory)
        with raw_trajectory.open(encoding="utf-8") as stream:
            frame_count = max(sum(1 for _ in stream) - 1, 0)
        write_native_summary(output_root / "summary.txt", frames=frame_count, monitor=native_monitor or {})

    expected_absent = []
    absent_reasons = [
        (output_root / "trace.jsonl", "trace output is disabled"),
        (output_root / "trace.csv", "trace output is disabled"),
        (output_root / "trace", "trace output is disabled"),
        (output_root / "marg_data", "MargData output is disabled/omitted"),
    ]
    for path, reason in absent_reasons:
        expected_absent.append({"path": str(path), "reason": reason, "present": present_payload(path), "filesystem_exists": path.exists()})
    output_files = collect_output_files(output_root, log_path)
    output_policy_ok = not any(item["present"] for item in expected_absent)
    trajectory_ok = all(path.is_file() for path in (raw_trajectory, tum_trajectory))
    artifact_hashes_ok = all(int(item["bytes"]) == Path(item["path"]).stat().st_size and item["sha256"].casefold() == sha256(Path(item["path"])).casefold() for item in output_files)
    process_ok = int(monitor["returncode"]) == 0
    success = process_ok and output_policy_ok and trajectory_ok and artifact_hashes_ok
    native_validation = {}
    if args.engine_kind == "native-wsl":
        monitor_binding = (native_monitor or {}).get("native_binding", {})
        native_binding_stable = bool(
            native_binding
            and isinstance(monitor_binding, dict)
            and monitor_binding.get("commit") == native_binding["commit"]
            and monitor_binding.get("tree_sha1") == native_binding["tree_sha1"]
            and monitor_binding.get("binary", {}).get("sha256") == native_binding["binary"]["sha256"]
            and monitor_binding.get("config", {}).get("sha256") == native_binding["config"]["sha256"]
            and monitor_binding.get("calibration", {}).get("sha256") == native_binding["calibration"]["sha256"]
        )
        native_validation = {
            "native_binding_verified": native_binding is not None,
            "native_commit_verified": bool(native_binding and native_binding["commit"] == UPSTREAM_COMMIT),
            "native_tree_verified": bool(native_binding and native_binding["tree_sha1"] == UPSTREAM_TREE_SHA1),
            "native_headless_verified": bool(native_argv and "--show-gui" in native_argv and native_argv[native_argv.index("--show-gui") + 1] == "0"),
            "native_trajectory_save_verified": bool(native_argv and "--save-trajectory" in native_argv and native_argv[native_argv.index("--save-trajectory") + 1] == "euroc"),
            "native_num_threads_verified": bool(native_argv and "--num-threads" in native_argv and native_argv[native_argv.index("--num-threads") + 1] == "1"),
            "native_marg_data_omitted": bool(native_argv and "--marg-data" not in native_argv),
            "native_process_tree_metrics_verified": bool(native_monitor and native_monitor.get("platform") == "wsl-linux"),
            "native_binding_stable": native_binding_stable,
        }
        success = success and all(native_validation.values())
    validation = {"status": "success" if success else "failure", "protocol_binding_verified": True, "input_binding_verified": True, "output_policy_verified": output_policy_ok, "artifact_hashes_verified": artifact_hashes_ok, "trajectory_artifacts_present": trajectory_ok, "process_returncode_zero": process_ok, "dataset_sequence_record": sequence_record.get("id"), **native_validation}
    status = "success" if success else "failure"
    common = {"result_schema_binding": result_schema_binding, "protocol_binding": protocol_binding, "protocol": protocol, "input_binding": input_binding, "calibration": calibration, "config": config, "executable": executable, "calibration_binding": calibration_binding, "config_binding": config_binding, "build_binding": build_binding, "engine_kind": args.engine_kind, "engine_dataset_path": engine_dataset_path, "output_flags": output_flags, "native_marg_data_argument": native_marg_data_argument, "run_id": run_id, "sequence": sequence, "max_frames": args.max_frames, "command": command, "execution": execution, "output_root": output_root, "output_files": output_files, "expected_absent": expected_absent, "equivalence": equivalence, "removed_gt_env_keys": removed_gt_env_keys, "status": status, "validation": validation}
    manifest = base_document(document_kind="run_manifest", self_path=manifest_path, **common)
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    manifest_binding = file_binding(manifest_path, role="run_manifest")
    result = base_document(document_kind="result", self_path=result_path, output_files=[*output_files, manifest_binding], **{k: v for k, v in common.items() if k != "output_files"})
    result["artifacts"]["run_manifest"] = manifest_binding
    result["run_manifest_path"] = str(manifest_path)
    result_path.parent.mkdir(parents=True, exist_ok=True)
    result_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    validate_document(result, schema_path)
    print(json.dumps({"result": str(result_path), "run_manifest": str(manifest_path), "status": status, "returncode": monitor["returncode"], "wall_seconds": execution["wall_seconds"], "peak_process_tree_rss_bytes": execution["peak_process_tree_rss_bytes"], "executable_sha256": build_binding["executable_sha256"], "config_sha256": config_binding["sha256"], "calibration_sha256": calibration_binding["sha256"], "protocol_sha256": protocol_binding["sha256"], "dataset_manifest_sha256": input_binding["dataset_manifest_sha256"]}, indent=2))
    if not success:
        return int(monitor["returncode"]) if int(monitor["returncode"]) != 0 else 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
