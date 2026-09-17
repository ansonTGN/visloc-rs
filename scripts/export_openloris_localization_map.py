#!/usr/bin/env python3
"""Export a landmark-descriptor store and query features for map-based
relocalization against a map built by the native COLMAP port.

Inputs:
  * a COLMAP text model directory (``cameras.txt`` / ``images.txt`` /
    ``points3D.txt``) written by ``examples/colmap_incremental_mapper``;
  * the COLMAP database that produced it (SIFT descriptors live there as
    128-byte uint8 blobs; the mapper export keeps keypoints only);
  * the flat-name -> COLMAP-name ``image_aliases.tsv``;
  * the rig manifest (``F frame_id flat_name sensor``) and the per-image
    ``*_features.txt`` keypoints (``x y`` per line, in DB keypoint order).

Outputs (in ``--out-dir``):
  * ``landmark_descriptors.txt``: ``<landmark_id> <128 floats>``, one
    representative map observation per landmark;
  * ``query_features/<flat>.txt``: ``x y <128 floats>`` for the held-out query
    images;
  * ``map/``: a filtered COLMAP text model (only map frames) when
    ``--held-out-parity`` is set, otherwise a copy of the input model.

With ``--held-out-parity even`` the map keeps even frames and the queries are
the odd frames (and vice versa): descriptors and the map model are built from
map observations only, so the held-out images never leak into the map. The
landmark *positions* still come from the full model, so this is an approximate
held-out split (see docs/openloris_map_relocalization_plan.md).
"""

from __future__ import annotations

import argparse
import json
import shutil
import sqlite3
from pathlib import Path


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--model-dir", type=Path, required=True)
    p.add_argument("--database", type=Path, required=True)
    p.add_argument("--image-aliases", type=Path, required=True)
    p.add_argument("--rig-manifest", type=Path, required=True)
    p.add_argument("--features-dir", type=Path, required=True)
    p.add_argument("--out-dir", type=Path, required=True)
    p.add_argument("--held-out-parity", choices=["none", "even", "odd"], default="none")
    return p.parse_args()


def parse_model_images(path: Path) -> dict[int, tuple[str, int]]:
    """model image_id -> (name, camera_id)."""
    out: dict[int, tuple[str, int]] = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        if not raw or raw.startswith("#"):
            continue
        f = raw.split()
        if len(f) == 10:
            out[int(f[0])] = (f[9], int(f[8]))
    return out


def parse_points3d(path: Path) -> dict[int, tuple[list[str], list[tuple[int, int]]]]:
    """landmark_id -> (header tokens [id xyz rgb error], track pairs)."""
    out: dict[int, tuple[list[str], list[tuple[int, int]]]] = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        if not raw or raw.startswith("#"):
            continue
        f = raw.split()
        landmark_id = int(f[0])
        rest = f[8:]
        track = [(int(rest[k]), int(rest[k + 1])) for k in range(0, len(rest) - 1, 2)]
        out[landmark_id] = (f[:8], track)
    return out


def load_descriptors(db: sqlite3.Connection) -> dict[int, tuple[int, bytes]]:
    out: dict[int, tuple[int, bytes]] = {}
    for image_id, rows, cols, data in db.execute(
        "SELECT image_id, rows, cols, data FROM descriptors"
    ):
        out[image_id] = (cols, bytes(data))
    return out


def main() -> int:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    flat_to_colmap: dict[str, str] = {}
    for line in args.image_aliases.read_text(encoding="utf-8").splitlines():
        if not line or line.startswith("flat_name") or line.startswith("original_name"):
            continue
        flat, colmap = line.split("\t")
        flat_to_colmap[flat] = colmap

    flat_to_frame: dict[str, int] = {}
    for line in args.rig_manifest.read_text(encoding="utf-8").splitlines():
        f = line.split()
        if f and f[0] == "F":
            flat_to_frame[f[2]] = int(f[1])

    db = sqlite3.connect(args.database)
    name_to_dbid = {name: i for i, name in db.execute("SELECT image_id, name FROM images")}
    descriptors = load_descriptors(db)

    model_images = parse_model_images(args.model_dir / "images.txt")
    points = parse_points3d(args.model_dir / "points3D.txt")

    def db_id_of_model_image(model_image_id: int) -> int | None:
        entry = model_images.get(model_image_id)
        if entry is None:
            return None
        colmap = flat_to_colmap.get(entry[0])
        return name_to_dbid.get(colmap) if colmap else None

    def frame_of_model_image(model_image_id: int) -> int | None:
        entry = model_images.get(model_image_id)
        return flat_to_frame.get(entry[0]) if entry else None

    def is_map(model_image_id: int) -> bool:
        if args.held_out_parity == "none":
            return True
        frame = frame_of_model_image(model_image_id)
        if frame is None:
            return False
        return (frame % 2 == 0) if args.held_out_parity == "even" else (frame % 2 == 1)

    # ---- landmark descriptors (map observations only) ----
    landmark_lines: list[str] = []
    skipped = 0
    for landmark_id, (_header, track) in points.items():
        chosen = None
        for model_image_id, idx in track:
            if not is_map(model_image_id):
                continue
            dbid = db_id_of_model_image(model_image_id)
            if dbid is None or dbid not in descriptors:
                continue
            cols, blob = descriptors[dbid]
            if idx * cols + cols > len(blob):
                continue
            chosen = blob[idx * cols : idx * cols + cols]
            break
        if chosen is None:
            skipped += 1
            continue
        landmark_lines.append(str(landmark_id) + " " + " ".join(str(float(b)) for b in chosen))
    (args.out_dir / "landmark_descriptors.txt").write_text(
        "\n".join(landmark_lines) + "\n", encoding="utf-8"
    )

    # ---- query features for held-out images ----
    query_dir = args.out_dir / "query_features"
    query_dir.mkdir(exist_ok=True)
    query_count = 0
    for model_image_id, (flat, _camera_id) in model_images.items():
        if is_map(model_image_id):
            continue
        dbid = db_id_of_model_image(model_image_id)
        if dbid is None or dbid not in descriptors:
            continue
        cols, blob = descriptors[dbid]
        feature_path = args.features_dir / f"{Path(flat).stem}_features.txt"
        if not feature_path.exists():
            continue
        lines = []
        for idx, raw in enumerate(feature_path.read_text(encoding="utf-8").splitlines()):
            f = raw.split()
            if len(f) < 2 or idx * cols + cols > len(blob):
                continue
            desc = blob[idx * cols : idx * cols + cols]
            lines.append(f[0] + " " + f[1] + " " + " ".join(str(float(b)) for b in desc))
        (query_dir / f"{Path(flat).stem}.txt").write_text("\n".join(lines) + "\n")
        query_count += 1

    # ---- filtered map model ----
    map_dir = args.out_dir / "map"
    if map_dir.exists():
        shutil.rmtree(map_dir)
    map_dir.mkdir()
    shutil.copy(args.model_dir / "cameras.txt", map_dir / "cameras.txt")

    kept_images = {i: v for i, v in model_images.items() if is_map(i)}
    src = (args.model_dir / "images.txt").read_text(encoding="utf-8").splitlines()
    out_lines: list[str] = []
    i = 0
    while i < len(src):
        raw = src[i]
        if not raw or raw.startswith("#"):
            i += 1
            continue
        f = raw.split()
        if len(f) == 10:
            if int(f[0]) in kept_images:
                out_lines.append(raw)
                if i + 1 < len(src):
                    out_lines.append(src[i + 1])
            i += 2
        else:
            i += 1
    (map_dir / "images.txt").write_text("\n".join(out_lines) + "\n")

    lines = []
    for landmark_id, (header, track) in points.items():
        kept = [(i, idx) for i, idx in track if i in kept_images]
        if not kept:
            continue
        lines.append(" ".join(header) + " " + " ".join(f"{i} {idx}" for i, idx in kept))
    (map_dir / "points3D.txt").write_text("\n".join(lines) + "\n")

    summary = {
        "schema": "visloc_openloris_localization_export_v1",
        "model_dir": str(args.model_dir),
        "held_out_parity": args.held_out_parity,
        "landmarks": len(landmark_lines),
        "landmarks_skipped": skipped,
        "query_images": query_count,
        "map_images": len(kept_images),
        "out_dir": str(args.out_dir),
    }
    (args.out_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
