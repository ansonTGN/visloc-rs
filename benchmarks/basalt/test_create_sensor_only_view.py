"""Small local-fixture tests for the sensor-only hardlink view builder."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from benchmarks.basalt.create_sensor_only_view import SensorViewError, create_sensor_view


def _make_fixture(root: Path) -> tuple[Path, Path]:
    source = root / "physical" / "SYNTH"
    for camera in ("cam0", "cam1"):
        data = source / "mav0" / camera / "data"
        data.mkdir(parents=True)
        (data.parent / "sensor.yaml").write_text("sensor: synthetic\n", encoding="utf-8")
        rows = [("1", "000001.png"), ("2", "000002.png")]
        (data.parent / "data.csv").write_text(
            "#timestamp [ns],filename\n" + "".join(f"{timestamp},{name}\n" for timestamp, name in rows),
            encoding="utf-8",
        )
        for name, payload in (("000001.png", b"png-one"), ("000002.png", b"png-two")):
            (data / name).write_bytes(payload)
    imu = source / "mav0" / "imu0"
    imu.mkdir(parents=True)
    (imu / "sensor.yaml").write_text("sensor: synthetic imu\n", encoding="utf-8")
    (imu / "data.csv").write_text("1,0,0,0,0,0,0\n", encoding="utf-8")
    manifest = root / "manifest.json"
    manifest.write_text(
        json.dumps(
            {
                "sequences": [
                    {"id": "SYNTH", "physical_root": {"path": str(source)}}
                ],
                "protocol": {"sequence_order": ["SYNTH"]},
            }
        ),
        encoding="utf-8",
    )
    return manifest, source


def test_create_and_idempotently_validate_hardlinks(tmp_path: Path) -> None:
    manifest, source = _make_fixture(tmp_path)
    destination = tmp_path / "view"
    first = create_sensor_view(
        manifest,
        destination,
        enforce_allowed_paths=False,
        strict_all11=False,
    )
    assert first["status"] == "PASS"
    assert first["mode"] == "created"
    assert first["identity"]["data_bytes_copied"] == 0
    assert first["tree"]["hardlink_identity_exact_files"] == first["tree"]["destination_files"]
    source_png = source / "mav0" / "cam0" / "data" / "000001.png"
    destination_png = destination / "SYNTH" / "mav0" / "cam0" / "data" / "000001.png"
    assert source_png.stat().st_dev == destination_png.stat().st_dev
    assert source_png.stat().st_ino == destination_png.stat().st_ino
    assert source_png.stat().st_size == destination_png.stat().st_size

    second = create_sensor_view(
        manifest,
        destination,
        enforce_allowed_paths=False,
        strict_all11=False,
    )
    assert second["mode"] == "idempotent_existing"
    assert second["identity"]["os_link_calls"] == 0
    assert second["tree"]["hardlink_identity_exact_files"] == second["tree"]["destination_files"]


def test_existing_mismatch_fails_without_repair(tmp_path: Path) -> None:
    manifest, source = _make_fixture(tmp_path)
    destination = tmp_path / "view"
    create_sensor_view(manifest, destination, enforce_allowed_paths=False, strict_all11=False)
    source_png = source / "mav0" / "cam0" / "data" / "000001.png"
    destination_png = destination / "SYNTH" / "mav0" / "cam0" / "data" / "000001.png"
    destination_png.unlink()
    destination_png.write_bytes(b"tampered")
    before = destination_png.read_bytes()
    with pytest.raises(SensorViewError, match="destination tree is not exact"):
        create_sensor_view(manifest, destination, enforce_allowed_paths=False, strict_all11=False)
    assert destination_png.read_bytes() == before
    assert source_png.read_bytes() == b"png-one"
