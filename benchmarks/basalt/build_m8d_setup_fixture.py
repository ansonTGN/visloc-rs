#!/usr/bin/env python3
"""Pack the first-20 M8d tracks/features and mapper poses for the C++ oracle.

The checked-in M8c/M8d JSON artifacts intentionally do not carry mapper pose
state.  The pinned upstream executable emits those poses from its MargData
packets, after which this helper combines them with the exact feature/track
artifacts into a tiny endian-stable input stream consumed by
``upstream_m8d_setup_opt_oracle.cpp``.

Pose input is a JSON object with a ``poses`` array.  Each item contains
``frame_id``, ``translation`` (xyz), and ``quaternion_xyzw``.  No ground-truth
path is accepted or needed; these are the mapper poses emitted by upstream.
"""

from __future__ import annotations

import argparse
import json
import struct
from pathlib import Path


MAGIC = b"M8DOPT1\n"
UPSTREAM_COMMIT = "0f3b2b52c807f70ff4e2973ce253c73329eea7bc"


def tcid(value: dict) -> tuple[int, int]:
    return int(value["frame_id"]), int(value["cam_id"])


def read_json(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def build(source: dict, tracks_source: dict, poses_source: dict) -> bytes:
    feature_images = {}
    for record in source["features"]:
        image = tcid(record["time_cam_id"])
        corners = record.get("corners_xy")
        if corners is None:
            raise ValueError(
                "M8c feature record has only summary digests; pass a full-corner "
                "export (for example target/m8c_feature_raw20_final.json)"
            )
        feature_images[image] = [(float(point[0]), float(point[1])) for point in corners]

    poses = {}
    for record in poses_source["poses"]:
        frame_id = int(record["frame_id"])
        translation = tuple(float(value) for value in record["translation"])
        quaternion = tuple(float(value) for value in record["quaternion_xyzw"])
        if len(translation) != 3 or len(quaternion) != 4:
            raise ValueError(f"invalid pose record for frame {frame_id}")
        poses[frame_id] = (translation, quaternion)

    tracks = tracks_source["oracle"]["canonical_observations"]
    for track in tracks:
        for observation in track["observations"]:
            image = tcid(observation["image"])
            feature_id = int(observation["feature_id"])
            if image not in feature_images:
                raise ValueError(f"track references image absent from M8c: {image}")
            if not 0 <= feature_id < len(feature_images[image]):
                raise ValueError(f"track feature {feature_id} absent from image {image}")
        host = tcid(track["observations"][0]["image"])
        if host[0] not in poses:
            raise ValueError(f"host pose missing for frame {host[0]}")

    output = bytearray(MAGIC)
    output.extend(struct.pack("<Q", len(feature_images)))
    for (frame_id, cam_id), corners in sorted(feature_images.items()):
        output.extend(struct.pack("<qQQ", frame_id, cam_id, len(corners)))
        for x, y in corners:
            output.extend(struct.pack("<dd", x, y))

    output.extend(struct.pack("<Q", len(poses)))
    for frame_id, (translation, quaternion) in sorted(poses.items()):
        output.extend(struct.pack("<q", frame_id))
        output.extend(struct.pack("<ddd", *translation))
        # C++ Sophus / Rust SE3 use the conventional xyzw JSON spelling at
        # the file boundary; the oracle reconstructs Eigen's wxyz ctor order.
        output.extend(struct.pack("<dddd", *quaternion))

    output.extend(struct.pack("<Q", len(tracks)))
    for track in sorted(tracks, key=lambda value: int(value["id"])):
        observations = track["observations"]
        output.extend(struct.pack("<QQ", int(track["id"]), len(observations)))
        for observation in observations:
            frame_id, cam_id = tcid(observation["image"])
            output.extend(
                struct.pack("<qQQ", frame_id, cam_id, int(observation["feature_id"]))
            )
    return bytes(output)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--features",
        type=Path,
        default=Path(__file__).with_name("m8c_feature_oracle20.json"),
    )
    parser.add_argument(
        "--tracks",
        type=Path,
        default=Path(__file__).with_name("m8d_track_oracle20.json"),
    )
    parser.add_argument("--poses", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    payload = build(read_json(args.features), read_json(args.tracks), read_json(args.poses))
    args.output.write_bytes(payload)
    print(
        json.dumps(
            {
                "schema": "basalt-m8d-setup-opt-input-v1",
                "upstream_commit": UPSTREAM_COMMIT,
                "features": args.features.name,
                "tracks": args.tracks.name,
                "poses": str(args.poses),
                "output": str(args.output),
                "bytes": len(payload),
            },
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
