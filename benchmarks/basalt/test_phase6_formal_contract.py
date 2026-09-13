"""Regression tests for the current Phase 6 formal matrix contract."""

from __future__ import annotations

import hashlib

from benchmarks.basalt import phase6_coordinator as coordinator


def test_formal_matrix_is_all11_once_for_both_engines():
    assert coordinator.DEFAULT_REPETITIONS == 1
    assert len(coordinator.PROTOCOL_SEQUENCE_IDS) == 11
    assert len(set(coordinator.PROTOCOL_SEQUENCE_IDS)) == 11
    assert coordinator.FORMAL_MATRIX_CELL_COUNT == 22


def test_default_calibration_is_checked_in_and_matches_native_oracle():
    path = coordinator.DEFAULT_CALIBRATION
    assert path == (
        coordinator.ROOT
        / "benchmarks"
        / "basalt"
        / "release_inputs"
        / "euroc_ds_calib.json"
    )
    assert path.is_file()
    assert hashlib.sha256(path.read_bytes()).hexdigest() == (
        "ad8c5a18c48c55dacf61d18ebbc18cd7d4f3acccd5840bd5a5cd8646adb6271c"
    )


def test_rust_config_bytes_match_pinned_native_oracle():
    assert hashlib.sha256(coordinator.DEFAULT_CONFIG.read_bytes()).hexdigest() == (
        "82937bd6493e592ef89572d31260c10f7437b4fb3ff1fda179375713966e34fa"
    )
