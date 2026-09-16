#!/usr/bin/env python3
"""Convert a LaMAria "pinhole_calibrations" JSON into a Basalt calibration
JSON consumable by our Rust port (pipelines/basalt/src/calibration.rs).

LaMAria's ASL export (`--type asl`) ships pinhole-undistorted "SLAM camera"
images alongside a per-sequence pinhole calibration JSON
(`pinhole_calibrations/training/<seq>.json`, one file per sequence, fetched
separately from the asl_folder zip -- see
https://cvg-data.inf.ethz.ch/lamaria/pinhole_calibrations/training/). Its
schema (confirmed 2026-09-15 against R_01_easy.json):

    {
      "cam0": {"model": "PINHOLE", "params": [fx, fy, cx, cy],
               "resolution": {"width": W, "height": H},
               "T_b_s": {"qvec": [qx, qy, qz, qw], "tvec": [px, py, pz]}},
      "cam1": {... mirrors cam0, independent resolution ...},
      "imu0": {"T_b_s": {"qvec": [0,0,0,1], "tvec": [0,0,0]},
               "gravity_magnitude": 9.806,
               "acc_noise_density": ..., "gyro_noise_density": ...,
               "acc_bias_random_walk_sigma": ...,
               "gyro_bias_random_walk_sigma": ...,
               "imu_rate": 1000}
    }

Key facts established by reading the `cvg/lamaria` package source
(`lamaria/utils/aria.py`), not assumed:

* `imu0.T_b_s` is the identity transform in every training sequence we
  inspected -- "body" IS the (right) IMU frame. This script asserts that
  rather than silently trusting it, so a future sequence with a non-identity
  IMU extrinsic fails loudly instead of producing a silently-wrong
  T_imu_cam.
* Because body == imu, `cam.T_b_s` (body<-sensor) IS ALREADY
  T_imu_cam (imu<-cam) -- no inversion or composition needed. This mirrors
  scripts/euroc_official_to_ds_calib.py's finding for EuRoC's T_BS.
* `qvec` in these JSON files is stored **[qx, qy, qz, qw]** (scalar-last),
  matching Basalt's own qx/qy/qz/qw fields directly with no reordering.
  This is NOT the COLMAP images.txt convention (scalar-first, wxyz); it is
  confirmed by `lamaria/utils/aria.py`'s `rigid3d_from_transform`, which
  rolls a wxyz quaternion from `projectaria_tools` to xyzw specifically "for
  pycolmap format" before it would ever be serialized this way, and by
  `get_t_cam_a_cam_b_from_calibration_file`, which feeds a JSON `qvec` field
  straight into `pycolmap.Rotation3d(...)` -- whose constructor the same
  module documents as expecting xyzw -- with no reordering step at all.

Basalt's camera model is Double Sphere (DS): with xi=0, alpha=0 the DS
projection collapses algebraically to plain pinhole (see
`ds_project`/`unit-test` below), so LaMAria's already-undistorted pinhole
images can be fed to the DS estimator unchanged by writing fx,fy,cx,cy
straight through with xi=alpha=0 -- no nonlinear fit needed (unlike the
EuRoC radtan->DS conversion, which does need one).

IMU noise fields map directly: Basalt's `accel_noise_std`/`gyro_noise_std`
are continuous-time noise *densities* (the estimator itself multiplies by
sqrt(imu_update_rate) -- see pipelines/basalt/src/adapter.rs's
`sample_density_f32` comment), i.e. exactly LaMAria's Kalibr-form
`acc_noise_density`/`gyro_noise_density`. Basalt's `accel_bias_std`/
`gyro_bias_std` are the continuous bias random-walk density, i.e. LaMAria's
`acc_bias_random_walk_sigma`/`gyro_bias_random_walk_sigma`.

`calib_accel_bias`/`calib_gyro_bias` (Basalt's static intrinsic
scale/misalignment + bias correction, 9- and 12-vectors respectively) have
no LaMAria equivalent -- the pinhole calibration carries no such factory
correction. All-zero is Basalt's own no-op default
(pipelines/basalt/src/vio/estimator.rs:524-525: `vec![0.0; 9]` /
`vec![0.0; 12]`; `calibrate_accel`/`calibrate_gyro` reduce to the identity
when every entry is zero), so this script writes that default explicitly
rather than omitting the fields.

Not carried over (LaMAria has no equivalent / Basalt tolerates their
absence): vignette (not even a field in `RawCalibration`), mocap fields
(T_mocap_world, T_imu_marker, mocap_time_offset_ns,
mocap_to_imu_offset_ns -- LaMAria has no mocap; written as identity/zero,
Basalt's estimator path never reads them for VIO). `cam_time_offset_ns` is
written as 0 -- LaMAria's ASL export does not document a nonzero cam/imu
time offset; if VIO diagnostics later show a temporal misalignment this is
the field to revisit.

Gravity: LaMAria's `imu0.gravity_magnitude` is 9.806 m/s^2 vs. the 9.81
hardcoded throughout pipelines/basalt/src/{initialization,vio/estimator,
vio/window}.rs (not config- or calibration-driven). This script prints the
delta but does not change it: 0.004 m/s^2 (0.04%) is negligible next to
other error sources over sequences of a few minutes, and wiring it through
correctly would touch estimator internals rather than config, which is out
of scope for a calibration converter (see the Stage 0 summary for the
same note flagged as a follow-up code-level item, not an estimator hack).

Usage:
    python scripts/lamaria_to_basalt_calib.py \
        --pinhole-calib E:/datasets/lamaria/training/R_01_easy/pinhole_calibrations/R_01_easy.json \
        --out configs/basalt/variants/lamaria/R_01_easy_calib.json
"""

import argparse
import json
import math
from pathlib import Path


def ds_project(fx, fy, cx, cy, xi, alpha, ray):
    """Basalt/Usenko et al. Double Sphere projection (matches
    scripts/euroc_official_to_ds_calib.py::ds_project, restated here for a
    standalone unit test with no numpy/scipy dependency)."""
    x, y, z = ray
    d1 = math.sqrt(x * x + y * y + z * z)
    zp = xi * d1 + z
    d2 = math.sqrt(x * x + y * y + zp * zp)
    denom = alpha * d2 + (1.0 - alpha) * zp
    u = fx * x / denom + cx
    v = fy * y / denom + cy
    return u, v


def pinhole_project(fx, fy, cx, cy, ray):
    x, y, z = ray
    return fx * x / z + cx, fy * y / z + cy


def check_ds_pinhole_identity(fx, fy, cx, cy):
    """Reproject a handful of off-axis rays through both models and assert
    bit-for-bit-close agreement (xi=0, alpha=0 must collapse DS to
    pinhole)."""
    rays = [
        (0.0, 0.0, 1.0),
        (0.15, 0.05, 1.0),
        (-0.2, 0.1, 1.0),
        (0.3, -0.25, 1.0),
        (-0.35, -0.3, 1.0),
    ]
    max_err = 0.0
    for ray in rays:
        u_ds, v_ds = ds_project(fx, fy, cx, cy, 0.0, 0.0, ray)
        u_ph, v_ph = pinhole_project(fx, fy, cx, cy, ray)
        max_err = max(max_err, abs(u_ds - u_ph), abs(v_ds - v_ph))
    if max_err > 1e-9:
        raise SystemExit(
            f"DS(xi=0,alpha=0) vs pinhole reprojection mismatch: "
            f"max_err={max_err:.3e}px (expected ~0)"
        )
    print(f"[selftest] DS(xi=0,alpha=0) == pinhole reprojection: max_err={max_err:.3e}px OK")


IDENTITY_TRANSFORM = {
    "px": 0.0, "py": 0.0, "pz": 0.0,
    "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0,
}


def transform_from_qvec_tvec(qvec, tvec):
    qx, qy, qz, qw = qvec
    px, py, pz = tvec
    return {
        "px": float(px), "py": float(py), "pz": float(pz),
        "qx": float(qx), "qy": float(qy), "qz": float(qz), "qw": float(qw),
    }


def is_identity_transform(t_b_s, atol=1e-9):
    qvec = t_b_s["qvec"]
    tvec = t_b_s["tvec"]
    identity_q = (0.0, 0.0, 0.0, 1.0)
    return (
        all(abs(a - b) < atol for a, b in zip(qvec, identity_q))
        and all(abs(v) < atol for v in tvec)
    )


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--pinhole-calib", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()

    calib = json.loads(args.pinhole_calib.read_text(encoding="utf-8"))

    imu0 = calib["imu0"]
    if not is_identity_transform(imu0["T_b_s"]):
        raise SystemExit(
            "imu0.T_b_s is not identity in "
            f"{args.pinhole_calib} -- the body==imu assumption this script "
            "relies on (T_imu_cam = cam.T_b_s directly) does not hold for "
            "this sequence; T_imu_cam must be recomputed as "
            "inv(T_body_imu) * T_body_cam instead. Aborting rather than "
            "writing a silently-wrong extrinsic."
        )

    cams = [calib["cam0"], calib["cam1"]]
    intrinsics = []
    transforms = []
    resolutions = []
    for index, cam in enumerate(cams):
        if cam["model"] != "PINHOLE":
            raise SystemExit(
                f"cam{index} model is {cam['model']!r}, expected PINHOLE "
                "(the DS xi=0/alpha=0 shortcut only applies to an already-"
                "undistorted pinhole camera)"
            )
        fx, fy, cx, cy = cam["params"]
        check_ds_pinhole_identity(fx, fy, cx, cy)
        intrinsics.append(
            {
                "camera_type": "ds",
                "intrinsics": {
                    "fx": fx, "fy": fy, "cx": cx, "cy": cy,
                    "xi": 0.0, "alpha": 0.0,
                },
            }
        )
        transforms.append(
            transform_from_qvec_tvec(cam["T_b_s"]["qvec"], cam["T_b_s"]["tvec"])
        )
        resolutions.append([cam["resolution"]["width"], cam["resolution"]["height"]])

    out = {
        "value0": {
            "T_imu_cam": transforms,
            "intrinsics": intrinsics,
            "resolution": resolutions,
            "calib_accel_bias": [0.0] * 9,
            "calib_gyro_bias": [0.0] * 12,
            "imu_update_rate": float(imu0["imu_rate"]),
            "accel_noise_std": [imu0["acc_noise_density"]] * 3,
            "gyro_noise_std": [imu0["gyro_noise_density"]] * 3,
            "accel_bias_std": [imu0["acc_bias_random_walk_sigma"]] * 3,
            "gyro_bias_std": [imu0["gyro_bias_random_walk_sigma"]] * 3,
            "T_mocap_world": dict(IDENTITY_TRANSFORM),
            "T_imu_marker": dict(IDENTITY_TRANSFORM),
            "mocap_time_offset_ns": 0,
            "mocap_to_imu_offset_ns": 0,
            "cam_time_offset_ns": 0,
        }
    }

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(out, indent=4), encoding="utf-8")
    print(f"wrote {args.out}")

    baseline = math.dist(
        (transforms[0]["px"], transforms[0]["py"], transforms[0]["pz"]),
        (transforms[1]["px"], transforms[1]["py"], transforms[1]["pz"]),
    )
    print(f"implied stereo baseline: {baseline:.6f} m")
    print(f"resolutions: cam0={resolutions[0]} cam1={resolutions[1]}")
    print(f"imu_update_rate: {imu0['imu_rate']} Hz")
    gravity = imu0.get("gravity_magnitude")
    if gravity is not None:
        print(
            f"[note] LaMAria gravity_magnitude={gravity} m/s^2 vs Basalt's "
            f"hardcoded 9.81 m/s^2 (delta={gravity - 9.81:+.4f}, "
            f"{(gravity - 9.81) / 9.81 * 100:+.3f}%) -- not config-driven in "
            "pipelines/basalt/src, left unchanged; see script docstring."
        )


if __name__ == "__main__":
    main()
