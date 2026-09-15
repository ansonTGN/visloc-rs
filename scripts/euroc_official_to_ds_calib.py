#!/usr/bin/env python3
"""Build a Basalt EuRoC Double Sphere calibration from the dataset's own
official factory (pinhole-radtan) camera calibration and body-frame
extrinsics, as an a-priori (GT-free) alternative to Basalt's own DS
recalibration.

Basalt's port only supports the "ds" (Double Sphere) camera model
(pipelines/basalt/src/calibration.rs:129), so the official EuRoC
pinhole-radtan intrinsics (mav0/cam{0,1}/sensor.yaml) cannot be fed to the
estimator directly. This script:

1. For each camera, unprojects a dense grid of pixels through the official
   radtan model (iterative OpenCV-style undistort) to unit bearing rays,
   then fits Double Sphere parameters (fx, fy, cx, cy, xi, alpha) by
   nonlinear least squares so that DS-projecting each ray reproduces the
   original pixel. Reports max/RMS fit residual in pixels.
2. Takes T_imu_cam from the official T_BS (body<-sensor) matrices directly:
   T_BS and Basalt's T_imu_cam are the same direction (IMU/body <- camera),
   confirmed by comparing the T_BS-cam0-derived quaternion/translation
   against the existing (Basalt-recalibrated) euroc_ds_calib.json cam0
   entry, which is close but not identical (as expected: two independent
   calibration procedures on the same physical rig).
3. Copies every non-visual field (IMU noise/bias, calib_accel_bias,
   calib_gyro_bias, imu_update_rate, vignette, T_mocap_world, T_imu_marker,
   mocap_time_offset_ns, mocap_to_imu_offset_ns, cam_time_offset_ns)
   unchanged from a base calibration file (the release euroc_ds_calib.json).
4. Prints the implied stereo baseline and the DS-effective focal length
   vs Basalt's own DS recalibration, for a sanity check.

This never touches ground-truth trajectories or ATE numbers -- only the
dataset's own published sensor.yaml files and Basalt's own release
calibration (for the non-visual fields and as a sanity-check reference).

Usage:
    python scripts/euroc_official_to_ds_calib.py \
        --cam0-sensor-yaml E:/datasets/euroc_mav/all11/MH_01_easy/mav0/cam0/sensor.yaml \
        --cam1-sensor-yaml E:/datasets/euroc_mav/all11/MH_01_easy/mav0/cam1/sensor.yaml \
        --base-calib benchmarks/basalt/release_inputs/euroc_ds_calib.json \
        --out configs/basalt/variants/official_euroc_ds/euroc_ds_calib.json
"""

import argparse
import json
import re
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares


def load_pinhole_radtan_yaml(path: Path):
    """Minimal EuRoC camera sensor.yaml parser (avoids a PyYAML dependency)."""
    text = path.read_text(encoding="utf-8")

    def find_floats(key):
        match = re.search(rf"{key}:\s*\[([^\]]*)\]", text)
        if not match:
            raise ValueError(f"missing key {key!r} in {path}")
        return [float(token) for token in match.group(1).split(",")]

    def find_ints(key):
        return [int(round(value)) for value in find_floats(key)]

    intrinsics = find_floats("intrinsics")
    distortion = find_floats("distortion_coefficients")
    resolution = find_ints("resolution")

    matrix_match = re.search(r"T_BS:.*?data:\s*\[([^\]]*)\]", text, re.DOTALL)
    if not matrix_match:
        raise ValueError(f"missing T_BS.data in {path}")
    t_bs = np.asarray(
        [float(token) for token in matrix_match.group(1).split(",")]
    ).reshape(4, 4)

    fu, fv, cu, cv = intrinsics
    k1, k2, p1, p2 = distortion
    width, height = resolution
    return {
        "fu": fu,
        "fv": fv,
        "cu": cu,
        "cv": cv,
        "k1": k1,
        "k2": k2,
        "p1": p1,
        "p2": p2,
        "width": width,
        "height": height,
        "T_BS": t_bs,
    }


def radtan_unproject_grid(cam, step: int):
    """Dense pixel grid -> unit bearing rays via iterative radtan undistort.

    Also returns the undistorted-normalized-plane radius ``r_und`` per
    point, used to report how the DS fit residual concentrates at the
    literal image corners (the extrapolated tail of the radtan polynomial,
    typically outside the region real checkerboard corners were ever
    observed during the original calibration).
    """
    us = np.arange(0, cam["width"], step, dtype=float)
    vs = np.arange(0, cam["height"], step, dtype=float)
    grid_u, grid_v = np.meshgrid(us, vs)
    pixels = np.stack([grid_u.ravel(), grid_v.ravel()], axis=1)

    x0 = (pixels[:, 0] - cam["cu"]) / cam["fu"]
    y0 = (pixels[:, 1] - cam["cv"]) / cam["fv"]
    x, y = x0.copy(), y0.copy()
    k1, k2, p1, p2 = cam["k1"], cam["k2"], cam["p1"], cam["p2"]
    for _ in range(30):
        r2 = x * x + y * y
        radial = 1.0 + k1 * r2 + k2 * r2 * r2
        x = (x0 - 2 * p1 * x * y - p2 * (r2 + 2 * x * x)) / radial
        y = (y0 - 2 * p2 * x * y - p1 * (r2 + 2 * y * y)) / radial

    r_und = np.sqrt(x * x + y * y)
    rays = np.stack([x, y, np.ones_like(x)], axis=1)
    rays /= np.linalg.norm(rays, axis=1, keepdims=True)
    return pixels, rays, r_und


def ds_project(rays, params):
    fx, fy, cx, cy, xi, alpha = params
    x, y, z = rays[:, 0], rays[:, 1], rays[:, 2]
    d1 = np.linalg.norm(rays, axis=1)
    zp = xi * d1 + z
    d2 = np.sqrt(x * x + y * y + zp * zp)
    denom = alpha * d2 + (1.0 - alpha) * zp
    u = fx * x / denom + cx
    v = fy * y / denom + cy
    return np.stack([u, v], axis=1)


def fit_double_sphere(pixels, rays, initial):
    def residuals(params):
        projected = ds_project(rays, params)
        return (projected - pixels).ravel()

    result = least_squares(residuals, initial, method="lm", max_nfev=20000)
    projected = ds_project(rays, result.x)
    error = np.linalg.norm(projected - pixels, axis=1)
    return result.x, float(error.max()), float(np.sqrt(np.mean(error**2)))


def print_radius_quantile_diagnostics(pixels, rays, r_und, initial, label):
    """Refit at several undistorted-radius quantile caps and report RMS/max,
    so a residual concentrated at the extreme image corners (outside the
    radtan model's validly-calibrated range) is visible even when the
    full-grid fit fails the acceptance gate."""
    print(f"{label}: radius-quantile diagnostics (n_full={len(pixels)}):")
    for quantile in (1.0, 0.95, 0.9, 0.85, 0.8, 0.7, 0.6, 0.5):
        threshold = np.quantile(r_und, quantile) if quantile < 1.0 else r_und.max() + 1.0
        mask = r_und <= threshold
        params, max_err, rms_err = fit_double_sphere(pixels[mask], rays[mask], initial)
        print(
            f"  radius<=p{quantile * 100:.0f} (n={mask.sum()}): "
            f"RMS={rms_err:.5f}px max={max_err:.5f}px "
            f"fx={params[0]:.3f} xi={params[4]:.5f} alpha={params[5]:.5f}"
        )


def se3_from_matrix(matrix):
    r = matrix[:3, :3]
    t = matrix[:3, 3]
    tr = np.trace(r)
    w = np.sqrt(max(1.0 + tr, 0.0)) / 2.0
    if w > 1e-8:
        x = (r[2, 1] - r[1, 2]) / (4 * w)
        y = (r[0, 2] - r[2, 0]) / (4 * w)
        z = (r[1, 0] - r[0, 1]) / (4 * w)
    else:  # pragma: no cover - EuRoC extrinsics never hit this branch
        raise ValueError("near-180-degree rotation not handled")
    return t, np.array([x, y, z, w])


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--cam0-sensor-yaml", type=Path, required=True)
    parser.add_argument("--cam1-sensor-yaml", type=Path, required=True)
    parser.add_argument("--base-calib", type=Path, required=True, help="release euroc_ds_calib.json; non-visual fields are copied from here")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--grid-step", type=int, default=8)
    parser.add_argument("--max-residual-px", type=float, default=0.1, help="abort if the RMS fit residual exceeds this")
    parser.add_argument(
        "--radius-quantile",
        type=float,
        default=1.0,
        help=(
            "only fit pixels whose undistorted-normalized-plane bearing "
            "radius is within this quantile of the full grid (1.0 = every "
            "pixel, including the literal image corners). Use <1.0 to "
            "exclude the extrapolated tail of the radtan polynomial."
        ),
    )
    args = parser.parse_args()

    base = json.loads(args.base_calib.read_text(encoding="utf-8"))
    base_value0 = base["value0"] if "value0" in base else base

    cams = [
        load_pinhole_radtan_yaml(args.cam0_sensor_yaml),
        load_pinhole_radtan_yaml(args.cam1_sensor_yaml),
    ]

    fitted_intrinsics = []
    fitted_transforms = []
    for index, cam in enumerate(cams):
        pixels_full, rays_full, r_und = radtan_unproject_grid(cam, args.grid_step)
        base_intrinsic = base_value0["intrinsics"][index]["intrinsics"]
        initial = np.array(
            [
                base_intrinsic["fx"],
                base_intrinsic["fy"],
                cam["cu"],
                cam["cv"],
                base_intrinsic["xi"],
                base_intrinsic["alpha"],
            ]
        )
        print_radius_quantile_diagnostics(pixels_full, rays_full, r_und, initial, f"cam{index}")

        if args.radius_quantile < 1.0:
            threshold = np.quantile(r_und, args.radius_quantile)
            mask = r_und <= threshold
        else:
            mask = np.ones(len(pixels_full), dtype=bool)
        pixels, rays = pixels_full[mask], rays_full[mask]

        params, max_err, rms_err = fit_double_sphere(pixels, rays, initial)
        fx, fy, cx, cy, xi, alpha = params
        print(
            f"cam{index}: SELECTED FIT (radius-quantile={args.radius_quantile}) "
            f"RMS={rms_err:.6f}px max={max_err:.6f}px "
            f"n={len(pixels)}/{len(pixels_full)} fx={fx:.6f} fy={fy:.6f} "
            f"cx={cx:.6f} cy={cy:.6f} xi={xi:.6f} alpha={alpha:.6f}"
        )
        if rms_err > args.max_residual_px:
            raise SystemExit(
                f"cam{index} DS fit RMS residual {rms_err:.6f}px exceeds "
                f"--max-residual-px={args.max_residual_px}; aborting, not "
                f"writing a calibration that does not represent the official "
                f"model faithfully"
            )
        fitted_intrinsics.append(
            {
                "camera_type": "ds",
                "intrinsics": {
                    "fx": fx,
                    "fy": fy,
                    "cx": cx,
                    "cy": cy,
                    "xi": xi,
                    "alpha": alpha,
                },
            }
        )
        t, quat = se3_from_matrix(cam["T_BS"])
        fitted_transforms.append(
            {
                "px": float(t[0]),
                "py": float(t[1]),
                "pz": float(t[2]),
                "qx": float(quat[0]),
                "qy": float(quat[1]),
                "qz": float(quat[2]),
                "qw": float(quat[3]),
            }
        )

    resolutions = [[cams[0]["width"], cams[0]["height"]], [cams[1]["width"], cams[1]["height"]]]

    out_value0 = dict(base_value0)  # start from base; overwrite visual fields
    out_value0["T_imu_cam"] = fitted_transforms
    out_value0["intrinsics"] = fitted_intrinsics
    out_value0["resolution"] = resolutions
    # vignette, calib_accel_bias, calib_gyro_bias, imu_update_rate,
    # accel_noise_std, gyro_noise_std, accel_bias_std, gyro_bias_std,
    # T_mocap_world, T_imu_marker, mocap_time_offset_ns,
    # mocap_to_imu_offset_ns, cam_time_offset_ns all remain from `base`.

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps({"value0": out_value0}, indent=4), encoding="utf-8")
    print(f"wrote {args.out}")

    # Sanity checks: stereo baseline and DS-effective focal length.
    p0 = np.array([fitted_transforms[0]["px"], fitted_transforms[0]["py"], fitted_transforms[0]["pz"]])
    p1 = np.array([fitted_transforms[1]["px"], fitted_transforms[1]["py"], fitted_transforms[1]["pz"]])
    baseline = float(np.linalg.norm(p1 - p0))
    print(f"implied stereo baseline: {baseline:.6f} m (expect ~0.11008)")

    for index, (cam, intrinsic) in enumerate(zip(cams, fitted_intrinsics)):
        fx = intrinsic["intrinsics"]["fx"]
        fy = intrinsic["intrinsics"]["fy"]
        xi = intrinsic["intrinsics"]["xi"]
        eff_fx = fx / (1.0 + xi)
        eff_fy = fy / (1.0 + xi)
        base_intrinsic = base_value0["intrinsics"][index]["intrinsics"]
        base_eff_fx = base_intrinsic["fx"] / (1.0 + base_intrinsic["xi"])
        base_eff_fy = base_intrinsic["fy"] / (1.0 + base_intrinsic["xi"])
        print(
            f"cam{index} DS-effective focal length: fx={eff_fx:.4f} "
            f"(Basalt DS: {base_eff_fx:.4f}, diff {(eff_fx - base_eff_fx) / base_eff_fx * 100:+.3f}%), "
            f"fy={eff_fy:.4f} (Basalt DS: {base_eff_fy:.4f}, diff "
            f"{(eff_fy - base_eff_fy) / base_eff_fy * 100:+.3f}%)"
        )


if __name__ == "__main__":
    main()
