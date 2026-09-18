#!/usr/bin/env python3
"""Export a COLMAP text model + descriptors from a Basalt NFR map (Option B).

Converts NFR tracks (map.json) to metric 3D landmarks using the keyframe poses
(poses.json, T_w_i) composed with T_imu_cam, then attaches a SIFT descriptor by
projecting into the keyframe images and taking the nearest SIFT keypoint from
the COLMAP database. The output feeds the same localizer harness as Option A.

NFR convention (visloc Basalt port):
  bearing = stereographic(direction) = (2u, 2v, 1-r2)/(1+r2), unit vector
  p_host_cam = bearing / inverse_distance
  p_world = T_w_cam_host * p_host_cam,  T_w_cam = T_w_i * T_imu_cam
"""

from __future__ import annotations

import argparse
import json
import sqlite3
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


FX, FY, CX, CY = 350.8070272987445, 350.8070272987445, 320.0, 240.0
W, H = 640, 480
ASSOC_RADIUS_PX = 3.0


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--map-json", type=Path, required=True)
    p.add_argument("--poses-json", type=Path, required=True)
    p.add_argument("--calib-json", type=Path, required=True)
    p.add_argument("--database", type=Path, required=True)
    p.add_argument("--out-dir", type=Path, required=True)
    return p.parse_args()


def stereo_bearing(u, v):
    r2 = u * u + v * v
    return np.array([2.0 * u, 2.0 * v, 1.0 - r2]) / (1.0 + r2)


def read_blob(conn, table, image_id, dtype):
    row = conn.execute(
        f"SELECT rows, cols, data FROM {table} WHERE image_id=?", (image_id,)
    ).fetchone()
    if row is None:
        return None
    rows, cols, data = row
    return np.frombuffer(data, dtype=dtype).reshape(rows, cols)


def main() -> int:
    args = parse_args()
    out = args.out_dir
    out.mkdir(parents=True, exist_ok=True)

    tracks = json.load(open(args.map_json))
    poses = {p["frame_id"]: p for p in json.load(open(args.poses_json))}
    calib = json.load(open(args.calib_json))["value0"]["T_imu_cam"][0]
    r_ic = Rotation.from_quat([calib["qx"], calib["qy"], calib["qz"], calib["qw"]])
    t_ic = np.array([calib["px"], calib["py"], calib["pz"]])

    # Keyframe T_w_cam for cam0.
    keyframes = {}
    for frame_id, p in poses.items():
        r_wi = Rotation.from_quat(
            [p["quaternion_wxyz"][1], p["quaternion_wxyz"][2],
             p["quaternion_wxyz"][3], p["quaternion_wxyz"][0]]
        )
        t_wi = np.array(p["translation"])
        r_wc = r_wi * r_ic
        t_wc = r_wi.apply(t_ic) + t_wi
        keyframes[frame_id] = {
            "timestamp_ns": p["timestamp_ns"],
            "R_wc": r_wc,
            "t_wc": t_wc,
        }

    conn = sqlite3.connect(str(args.database))
    name_to_id = {n: i for i, n in conn.execute("SELECT image_id, name FROM images")}
    sift = {}
    for name, image_id in name_to_id.items():
        if not name.endswith(".png"):
            continue
        kp = read_blob(conn, "keypoints", image_id, np.float32)
        ds = read_blob(conn, "descriptors", image_id, np.uint8)
        if kp is None or ds is None or len(kp) != len(ds):
            continue
        try:
            ts = int(Path(name).stem)
        except ValueError:
            continue
        sift[ts] = {"image_id": image_id, "xy": kp[:, :2], "desc": ds}

    # NFR -> 3D world.
    landmarks = []
    for t in tracks:
        rho = t["inverse_distance"]
        if not np.isfinite(rho) or rho <= 1e-9:
            continue
        u, v = t["direction"]
        if not (np.isfinite(u) and np.isfinite(v)):
            continue
        host = t["host"]["frame_id"]
        if host not in keyframes:
            continue
        kf = keyframes[host]
        p_cam = stereo_bearing(u, v) / rho
        if p_cam[2] <= 0:
            continue
        p_world = kf["R_wc"].apply(p_cam) + kf["t_wc"]
        if not np.all(np.isfinite(p_world)):
            continue
        landmarks.append({"track_id": t["track_id"], "p_world": p_world})

    # Attach SIFT by projection into keyframe images present in the DB.
    usable_keyframes = []
    for frame_id, kf in keyframes.items():
        ts = kf["timestamp_ns"]
        if ts in sift:
            usable_keyframes.append((frame_id, kf, sift[ts]))
    print(f"landmarks={len(landmarks)} usable_keyframes={len(usable_keyframes)}")

    attached = []
    for lm in landmarks:
        best = None
        for _, kf, s in usable_keyframes:
            p_cam = kf["R_wc"].inv().apply(lm["p_world"] - kf["t_wc"])
            if p_cam[2] <= 0.05:
                continue
            px = FX * p_cam[0] / p_cam[2] + CX
            py = FY * p_cam[1] / p_cam[2] + CY
            if not (0 <= px < W and 0 <= py < H):
                continue
            d2 = ((s["xy"] - [px, py]) ** 2).sum(axis=1)
            j = int(np.argmin(d2))
            dist = float(np.sqrt(d2[j]))
            if dist <= ASSOC_RADIUS_PX and (best is None or dist < best[0]):
                best = (dist, s, j)
        if best is None:
            continue
        _, s, j = best
        attached.append({**lm, "desc": s["desc"][j], "db_image_id": s["image_id"],
                         "sift_idx": j})
    print(f"attached={len(attached)}")

    # Write COLMAP text model. Image ids: reuse DB image ids for keyframes that
    # have an associated landmark; list all SIFT as 2D points to keep indices.
    used_db_ids = sorted({a["db_image_id"] for a in attached})
    db_to_model = {db_id: k + 1 for k, db_id in enumerate(used_db_ids)}
    model_to_db = {v: k for k, v in db_to_model.items()}
    # Map DB image id -> keyframe pose (find via timestamp).
    ts_to_kf = {}
    for frame_id, kf in keyframes.items():
        if kf["timestamp_ns"] in sift:
            ts_to_kf[kf["timestamp_ns"]] = kf
    db_to_kf = {}
    for name, image_id in name_to_id.items():
        if not name.endswith(".png"):
            continue
        try:
            ts = int(Path(name).stem)
        except ValueError:
            continue
        if ts in ts_to_kf and image_id in db_to_model:
            db_to_kf[image_id] = ts_to_kf[ts]

    with open(out / "cameras.txt", "w", encoding="utf-8") as handle:
        handle.write("# Camera list\n")
        handle.write(f"1 PINHOLE {W} {H} {FX} {FY} {CX} {CY}\n")

    # Per-image SIFT for 2D listing.
    img_sift = {}
    for db_id in used_db_ids:
        kp = read_blob(conn, "keypoints", db_id, np.float32)
        img_sift[db_id] = kp[:, :2] if kp is not None else np.zeros((0, 2))
    # Representative observation per landmark: (model_image_id, sift_idx).
    rep = {}
    for a in attached:
        rep[a["track_id"]] = (db_to_model[a["db_image_id"]], a["sift_idx"])
    # Invert: model image -> list of (sift_idx, track_id).
    img_obs = {m: [] for m in db_to_model.values()}
    for track_id, (m, sidx) in rep.items():
        img_obs[m].append((sidx, track_id))

    # Reindex: list only associated 2D points per image (no -1 entries).
    rep2 = {}
    img_lines = {}
    for m in db_to_model.values():
        pairs = sorted(img_obs[m])  # (sift_idx, track_id)
        lines = []
        for new_idx, (sidx, track_id) in enumerate(pairs):
            db_id = next(d for d, mm in db_to_model.items() if mm == m)
            x, y = img_sift[db_id][sidx]
            lines.append(f"{x:.6f} {y:.6f} {track_id}\n")
            rep2[track_id] = (m, new_idx)
        img_lines[m] = lines

    with open(out / "images.txt", "w", encoding="utf-8") as handle:
        handle.write("# Image list\n")
        for db_id, m in sorted(db_to_model.items(), key=lambda kv: kv[1]):
            kf = db_to_kf[db_id]
            r_cw = kf["R_wc"].inv()
            t_cw = -(r_cw.apply(kf["t_wc"]))
            q = r_cw.as_quat()  # x,y,z,w
            # Recover image name.
            name = next(n for n, i in name_to_id.items() if i == db_id)
            handle.write(
                f"{m} {q[3]:.12f} {q[0]:.12f} {q[1]:.12f} {q[2]:.12f} "
                f"{t_cw[0]:.12f} {t_cw[1]:.12f} {t_cw[2]:.12f} 1 {name}\n"
            )
            # COLMAP text: all POINTS2D triples on a single line.
            handle.write(" ".join(line.strip() for line in img_lines[m]) + "\n")

    with open(out / "points3D.txt", "w", encoding="utf-8") as handle:
        handle.write("# 3D point list\n")
        for a in sorted(attached, key=lambda a: a["track_id"]):
            p = a["p_world"]
            m, sidx = rep2[a["track_id"]]
            handle.write(
                f"{a['track_id']} {p[0]:.9f} {p[1]:.9f} {p[2]:.9f} "
                f"128 128 128 1.0 {m} {sidx}\n"
            )

    with open(out / "landmark_descriptors.txt", "w", encoding="utf-8") as handle:
        for a in sorted(attached, key=lambda a: a["track_id"]):
            handle.write(
                f"{a['track_id']} " + " ".join(str(int(v)) for v in a["desc"]) + "\n"
            )
    print(f"wrote B map: images={len(db_to_model)} points={len(attached)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
