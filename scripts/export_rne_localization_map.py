#!/usr/bin/env python3
"""Export a landmark-descriptor store and query features for map-based
relocalization against a COLMAP text model (external COLMAP).

This is the RNE/external-COLMAP counterpart of
``scripts/export_openloris_localization_map.py``. It reads the COLMAP SQLite
databases directly, so no rig manifest / feature-dir bridge is needed.

Inputs:
  * ``--model-dir``: COLMAP text model (cameras.txt / images.txt / points3D.txt).
  * ``--database``: the COLMAP database used to build the model (SIFT uint8).
  * ``--query-db``: a COLMAP database from ``colmap feature_extractor`` run on
    the query images with the same camera prior.
Outputs (``--out-dir``):
  * ``landmark_descriptors.txt``: ``<landmark_id> <128 floats>``.
  * ``query_features/<name>.txt``: ``x y <128 floats>`` per query image.

Representative descriptor: the first track observation of each landmark.
"""

from __future__ import annotations

import argparse
import sqlite3
from pathlib import Path

import numpy as np


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--model-dir", type=Path, required=True)
    p.add_argument("--database", type=Path, required=True)
    p.add_argument("--query-db", type=Path, required=True)
    p.add_argument("--out-dir", type=Path, required=True)
    return p.parse_args()


def read_blob(conn, table, image_id):
    row = conn.execute(
        f"SELECT rows, cols, data FROM {table} WHERE image_id=?", (image_id,)
    ).fetchone()
    if row is None:
        return None
    rows, cols, data = row
    return np.frombuffer(data, dtype=np.uint8 if table == "descriptors" else np.float32).reshape(
        rows, cols
    )


def parse_points3d(path):
    tracks = {}
    with open(path, encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            point_id = int(parts[0])
            # POINT3D_ID, X,Y,Z, R,G,B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)+
            track = []
            for i in range(8, len(parts), 2):
                track.append((int(parts[i]), int(parts[i + 1])))
            tracks[point_id] = track
    return tracks


def db_image_ids(conn):
    return {name: image_id for image_id, name in conn.execute("SELECT image_id, name FROM images")}


def main() -> int:
    args = parse_args()
    out = args.out_dir
    out.mkdir(parents=True, exist_ok=True)
    (out / "query_features").mkdir(exist_ok=True)

    tracks = parse_points3d(args.model_dir / "points3D.txt")
    map_conn = sqlite3.connect(str(args.database))
    map_name_to_id = db_image_ids(map_conn)

    desc_lines = []
    missing = 0
    for point_id in sorted(tracks):
        track = tracks[point_id]
        if not track:
            continue
        image_id, point_idx = track[0]
        blob = read_blob(map_conn, "descriptors", image_id)
        if blob is None or point_idx >= blob.shape[0]:
            missing += 1
            continue
        desc = " ".join(str(int(v)) for v in blob[point_idx])
        desc_lines.append(f"{point_id} {desc}\n")
    (out / "landmark_descriptors.txt").write_text("".join(desc_lines), encoding="utf-8")
    print(f"landmarks={len(tracks)} exported={len(desc_lines)} missing={missing}")

    query_conn = sqlite3.connect(str(args.query_db))
    count = 0
    for name, image_id in sorted(query_conn.execute("SELECT name, image_id FROM images")):
        kp = read_blob(query_conn, "keypoints", image_id)
        ds = read_blob(query_conn, "descriptors", image_id)
        if kp is None or ds is None or len(kp) != len(ds):
            continue
        lines = []
        for (x, y, *_), d in zip(kp, ds):
            lines.append(f"{x:.4f} {y:.4f} " + " ".join(str(int(v)) for v in d) + "\n")
        (out / "query_features" / (Path(name).stem + ".txt")).write_text(
            "".join(lines), encoding="utf-8"
        )
        count += 1
    print(f"query images exported={count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
