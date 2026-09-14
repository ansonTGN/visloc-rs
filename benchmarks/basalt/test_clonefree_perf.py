"""Focused contract tests for the Rust/native clone-free performance runner."""

from __future__ import annotations

import json
from pathlib import Path

from benchmarks.basalt import run_clonefree_perf as runner


def test_native_argv_is_headless_single_thread_and_margdata_free() -> None:
    argv = runner.native_engine_argv(
        executable="/root/pinned/basalt_vio",
        dataset="/mnt/e/datasets/euroc_mav/MH_01_easy",
        calibration="/root/pinned/data/euroc_ds_calib.json",
        config="/root/pinned/data/euroc_config.json",
        max_frames=80,
    )
    assert "--marg-data" not in argv
    assert argv[argv.index("--show-gui") + 1] == "0"
    assert argv[argv.index("--save-trajectory") + 1] == "euroc"
    assert argv[argv.index("--num-threads") + 1] == "1"
    assert argv[argv.index("--max-frames") + 1] == "80"


def test_native_csv_to_tum_preserves_pose_order(tmp_path: Path) -> None:
    raw = tmp_path / "trajectory.csv"
    tum = tmp_path / "trajectory.tum"
    raw.write_text(
        "frame_id,timestamp_ns,tx,ty,tz,qw,qx,qy,qz,cam0_observations\n"
        "0,1403636579763555584,1,2,3,0.9,0.1,0.2,0.3,5\n",
        encoding="utf-8",
    )
    assert runner.native_csv_to_tum(raw, tum) == 1
    assert tum.read_text(encoding="utf-8") == "1403636579.763555584 1 2 3 0.1 0.2 0.3 0.9\n"


def test_schema_declares_native_engine_contract() -> None:
    schema = json.loads(runner.RESULT_SCHEMA.read_text(encoding="utf-8"))
    # The field is emitted for every new result, while remaining optional in
    # the v1 schema so old Rust-only documents retain their compatibility.
    assert "engine_kind" not in schema["required"]
    assert schema["properties"]["engine_kind"]["enum"] == ["rust", "native-wsl"]
    native_then = schema["allOf"][0]["then"]["properties"]
    assert native_then["build"]["required"] == ["native_adapter", "native_source"]
    assert native_then["execution"]["required"] == [
        "measurement_scope",
        "native_argv",
        "native_process_tree",
    ]
    assert native_then["output_policy"]["properties"]["native_marg_data_argument"] == {
        "const": "omitted"
    }


def test_native_binding_defaults_match_pinned_checkout() -> None:
    assert runner.NATIVE_CHECKOUT_WSL_DEFAULT.endswith("visloc-basalt-clean-m7cr-20260823")
    assert runner.NATIVE_EXECUTABLE_WSL_DEFAULT.endswith("build/core-relwithdebinfo/basalt_vio")
    assert runner.UPSTREAM_COMMIT == "0f3b2b52c807f70ff4e2973ce253c73329eea7bc"
    assert runner.UPSTREAM_TREE_SHA1 == "b7afb830d82b45b8209cf784ad9744025d838411"
    for digest in (
        runner.NATIVE_EXECUTABLE_SHA256_DEFAULT,
        runner.NATIVE_CONFIG_SHA256_DEFAULT,
        runner.NATIVE_CALIBRATION_SHA256_DEFAULT,
    ):
        assert len(digest) == 64
        int(digest, 16)


def test_native_default_binary_is_the_clean_pinned_artifact() -> None:
    assert runner.NATIVE_EXECUTABLE_SHA256_DEFAULT == (
        "89e0324ccd04c3b615bf7e945aa32480a2f86524987aa00eaf152dcd6b1d242c"
    )
    assert runner.NATIVE_CONFIG_WSL_DEFAULT.startswith(runner.NATIVE_CHECKOUT_WSL_DEFAULT)
    assert runner.NATIVE_CALIBRATION_WSL_DEFAULT.startswith(runner.NATIVE_CHECKOUT_WSL_DEFAULT)
