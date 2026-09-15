#!/usr/bin/env python3
"""Propagate NfrMapper keyframe corrections to the full per-frame VIO trajectory.

This is the same rigid spanning-tree propagation used by the Stage-A
loop-closure postprocess (`examples/basalt_loop_closure_postprocess.rs`,
`propagate_corrections`, on branch `exp/basalt-loop-closure-ceiling`), ported
to Python so it can be applied to the offline NFR mapper's keyframe-only
output without touching the mapper's Rust API.

Convention (matches `relative_world_to_camera` / `SE3::compose` in
`pipelines/slam/src/pose_graph.rs` and `crates/core/src/geometry/se3.rs`):

  V_f  = raw VIO body-to-world pose at frame f (rotation R_f, translation t_f)
  M_k  = mapper-corrected body-to-world pose at the nearest preceding
         keyframe k (frame_id <= f)
  V_k  = raw VIO body-to-world pose at that same keyframe k

  corrected(f) = Delta_k * V_f,  where Delta_k = M_k * V_k^{-1}

i.e. each frame keeps its VIO-derived relative motion to the nearest
preceding keyframe; only the keyframe's mapper correction moves it. This
degenerates to an exact match at keyframes themselves (f == k).

Inputs:
  --vio-trajectory-csv   full per-frame VIO trajectory.csv (frame_id,
                          timestamp_ns, tx,ty,tz, qw,qx,qy,qz, ...)
  --mapper-poses-json    mapper poses.json (list of {frame_id, timestamp_ns,
                          translation:[x,y,z], quaternion_wxyz:[w,x,y,z]})
  --out-tum              output path for the propagated full-trajectory TUM
                          file (timestamp_s tx ty tz qx qy qz qw)
"""

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def quat_wxyz_to_matrix(w, x, y, z):
    q = np.asarray([w, x, y, z], dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm <= 0.0:
        raise ValueError("invalid zero/non-finite quaternion")
    w, x, y, z = q / norm
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def matrix_to_quat_xyzw(rotation_matrix):
    # Shepperd's method via a battle-tested branch-based conversion.
    m = rotation_matrix
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.asarray([x, y, z, w], dtype=float)
    return q / np.linalg.norm(q)


class SE3:
    __slots__ = ("R", "t")

    def __init__(self, rotation_matrix, translation):
        self.R = rotation_matrix
        self.t = translation

    def compose(self, other):
        return SE3(self.R @ other.R, self.R @ other.t + self.t)

    def inverse(self):
        r_inv = self.R.T
        return SE3(r_inv, -(r_inv @ self.t))


def load_vio_trajectory_csv(path):
    """Return {frame_id: (timestamp_ns, SE3 body_to_world)} sorted by frame_id."""
    rows = {}
    with open(path, "r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            frame_id = int(row["frame_id"])
            timestamp_ns = int(row["timestamp_ns"])
            t = np.asarray([float(row["tx"]), float(row["ty"]), float(row["tz"])])
            r = quat_wxyz_to_matrix(
                float(row["qw"]), float(row["qx"]), float(row["qy"]), float(row["qz"])
            )
            rows[frame_id] = (timestamp_ns, SE3(r, t))
    return rows


def load_mapper_poses_json(path):
    """Return {frame_id: (timestamp_ns, SE3 body_to_world)}."""
    with open(path, "r", encoding="utf-8") as stream:
        entries = json.load(stream)
    poses = {}
    for entry in entries:
        frame_id = int(entry["frame_id"])
        timestamp_ns = int(entry["timestamp_ns"])
        t = np.asarray(entry["translation"], dtype=float)
        w, x, y, z = entry["quaternion_wxyz"]
        r = quat_wxyz_to_matrix(w, x, y, z)
        poses[frame_id] = (timestamp_ns, SE3(r, t))
    return poses


def propagate(vio_trajectory, mapper_poses):
    """Rigid spanning-tree propagation of keyframe corrections to every frame.

    `vio_trajectory` and `mapper_poses` are {frame_id: (timestamp_ns, SE3)}
    maps, keyed by the same frame_id numbering (mapper keyframe frame_ids are
    a subset of the full VIO frame_ids). Returns a list of
    (timestamp_ns, SE3 corrected_body_to_world) sorted by frame_id.
    """
    keyframe_ids = sorted(mapper_poses.keys())
    if not keyframe_ids:
        raise ValueError("mapper produced no keyframe poses")
    missing = [k for k in keyframe_ids if k not in vio_trajectory]
    if missing:
        raise ValueError(
            f"{len(missing)} mapper keyframe frame_ids are absent from the VIO "
            f"trajectory (first few: {missing[:5]})"
        )

    # Delta_k = M_k * V_k^{-1}: rigid transform from the VIO-raw keyframe pose
    # to the mapper-corrected keyframe pose, cached per keyframe.
    delta_by_keyframe = {}
    for k in keyframe_ids:
        _, v_k = vio_trajectory[k]
        _, m_k = mapper_poses[k]
        delta_by_keyframe[k] = m_k.compose(v_k.inverse())

    frame_ids = sorted(vio_trajectory.keys())
    output = []
    cursor = 0
    for frame_id in frame_ids:
        while cursor + 1 < len(keyframe_ids) and keyframe_ids[cursor + 1] <= frame_id:
            cursor += 1
        reference_id = keyframe_ids[cursor]
        delta = delta_by_keyframe[reference_id]
        timestamp_ns, v_f = vio_trajectory[frame_id]
        corrected = delta.compose(v_f)
        output.append((timestamp_ns, corrected))
    return output


def write_tum(path, poses):
    with open(path, "w", encoding="utf-8") as stream:
        for timestamp_ns, pose in poses:
            qx, qy, qz, qw = matrix_to_quat_xyzw(pose.R)
            stream.write(
                f"{timestamp_ns * 1.0e-9:.9f} "
                f"{pose.t[0]:.9f} {pose.t[1]:.9f} {pose.t[2]:.9f} "
                f"{qx:.12f} {qy:.12f} {qz:.12f} {qw:.12f}\n"
            )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vio-trajectory-csv", type=Path, required=True)
    parser.add_argument("--mapper-poses-json", type=Path, required=True)
    parser.add_argument("--out-tum", type=Path, required=True)
    args = parser.parse_args()

    vio_trajectory = load_vio_trajectory_csv(args.vio_trajectory_csv)
    mapper_poses = load_mapper_poses_json(args.mapper_poses_json)
    propagated = propagate(vio_trajectory, mapper_poses)
    args.out_tum.parent.mkdir(parents=True, exist_ok=True)
    write_tum(args.out_tum, propagated)
    print(
        f"propagated {len(propagated)} frames from {len(mapper_poses)} keyframe "
        f"corrections -> {args.out_tum}"
    )


if __name__ == "__main__":
    main()
