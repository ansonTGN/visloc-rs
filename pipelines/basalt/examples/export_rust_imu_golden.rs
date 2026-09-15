//! Export the Rust frame-1 -> frame-2 IMU contract for field-wise comparison
//! with the pinned Basalt oracle.  This is deliberately sensor/trace only and
//! never reads EuRoC ground truth.

use std::{env, fs, path::PathBuf};

use nalgebra::{UnitQuaternion, Vector3};
use serde_json::{json, Value};
use visloc_basalt::{
    euroc::EurocSensorDataset,
    imu::{
        preintegration_residual, whitened_preintegration_factor, ImuNoiseModel, ImuPreintegrator,
    },
    BasaltNavState,
};
use visloc_core::geometry::SE3;

fn vec3(v: &Vector3<f64>) -> Vec<f64> {
    v.iter().copied().collect()
}

fn calibrated_gyro(params: &[f64], raw: Vector3<f64>) -> Vector3<f64> {
    let scale = nalgebra::Matrix3::from_columns(&[
        Vector3::new(params[3], params[4], params[5]),
        Vector3::new(params[6], params[7], params[8]),
        Vector3::new(params[9], params[10], params[11]),
    ]);
    raw + scale * raw - Vector3::from_row_slice(&params[..3])
}

fn calibrated_accel(params: &[f64], raw: Vector3<f64>) -> Vector3<f64> {
    let scale = nalgebra::Matrix3::new(
        params[3], 0.0, 0.0, params[4], params[6], 0.0, params[5], params[7], params[8],
    );
    raw + scale * raw - Vector3::from_row_slice(&params[..3])
}

fn matrix9(m: &nalgebra::SMatrix<f64, 9, 9>) -> Vec<Vec<f64>> {
    (0..9)
        .map(|r| (0..9).map(|c| m[(r, c)]).collect())
        .collect()
}

fn state_from_trace(value: &Value) -> BasaltNavState {
    let state = &value["state"];
    let q = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        state["qw"].as_f64().unwrap(),
        state["qx"].as_f64().unwrap(),
        state["qy"].as_f64().unwrap(),
        state["qz"].as_f64().unwrap(),
    ));
    let v = |key: &str| -> Vector3<f64> {
        Vector3::new(
            state[key][0].as_f64().unwrap(),
            state[key][1].as_f64().unwrap(),
            state[key][2].as_f64().unwrap(),
        )
    };
    BasaltNavState {
        imu_to_world: SE3::new(
            q,
            Vector3::new(
                state["tx"].as_f64().unwrap(),
                state["ty"].as_f64().unwrap(),
                state["tz"].as_f64().unwrap(),
            ),
        ),
        velocity_world_m_s: v("velocity"),
        gyro_bias_rad_s: v("gyro_bias"),
        accel_bias_m_s2: v("accel_bias"),
    }
}

fn trace_state(path: &PathBuf, frame: u64) -> Value {
    fs::read_to_string(path)
        .unwrap()
        .lines()
        .map(|line| serde_json::from_str::<Value>(line).unwrap())
        .find(|value| value["frame_id"].as_u64() == Some(frame))
        .unwrap_or_else(|| panic!("frame {frame} missing from {}", path.display()))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = env::args_os().collect();
    if args.len() != 6 {
        eprintln!(
            "usage: export_rust_imu_golden <MH_01_easy> <calib.json> <config.json> <trace.jsonl> <output.json>"
        );
        std::process::exit(2);
    }
    let dataset = EurocSensorDataset::open(&args[1], &args[2], &args[3])?;
    let f1 = dataset.frame(1)?;
    let f2 = dataset.frame(2)?;
    let start_ns = f1.timestamp_ns;
    let end_ns = f2.timestamp_ns;
    let selected: Vec<_> = dataset
        .imu_samples()
        .iter()
        .filter(|sample| sample.timestamp_ns > start_ns && sample.timestamp_ns <= end_ns)
        .copied()
        .collect();

    let calibration = dataset.calibration();
    let calibrated_samples: Vec<_> = dataset
        .imu_samples()
        .iter()
        .map(|sample| {
            visloc_basalt::ImuSample::new(
                sample.timestamp_ns,
                calibrated_gyro(&calibration.calib_gyro_bias, sample.gyro_rad_s),
                calibrated_accel(&calibration.calib_accel_bias, sample.accel_m_s2),
            )
        })
        .collect();
    let bg = Vector3::zeros();
    let ba = Vector3::zeros();
    let rms = |v: Vector3<f64>| (v.norm_squared() / 3.0).sqrt();
    let noise = ImuNoiseModel {
        gyro_density: rms(calibration.gyro_noise_std) * calibration.imu_update_rate_hz.sqrt(),
        accel_density: rms(calibration.accel_noise_std) * calibration.imu_update_rate_hz.sqrt(),
    };
    // Upstream IntegratedImuMeasurement consumes each packet directly with
    // dt=(packet.t-previous.t); it does not trapezoid-average adjacent
    // packets. This is also the estimator's fallback path for camera slices.
    let mut integrator = ImuPreintegrator::new(bg, ba)
        .with_noise(noise)
        .ok_or("invalid IMU noise")?;
    let mut previous = start_ns;
    for sample in calibrated_samples
        .iter()
        .filter(|sample| sample.timestamp_ns > start_ns && sample.timestamp_ns <= end_ns)
    {
        let dt = (sample.timestamp_ns - previous) as f64 * 1e-9;
        integrator.integrate_sample(sample.gyro_rad_s, sample.accel_m_s2, dt);
        previous = sample.timestamp_ns;
    }
    let delta = integrator.delta().clone();

    let trace = PathBuf::from(&args[4]);
    let from = state_from_trace(&trace_state(&trace, 1));
    let to = state_from_trace(&trace_state(&trace, 2));
    let gravity = Vector3::new(0.0, 0.0, -9.81);
    let raw_residual = preintegration_residual(&from, &to, &delta, gravity);
    let factor = whitened_preintegration_factor(&from, &to, &delta, gravity)
        .map_err(|error| format!("failed to build whitened IMU factor: {error:?}"))?;
    let samples: Vec<_> = selected
        .iter()
        .map(|s| {
            json!({
                "timestamp_ns": s.timestamp_ns,
                "gyro_rad_s": vec3(&s.gyro_rad_s),
                "accel_m_s2": vec3(&s.accel_m_s2),
                "calibrated_gyro_rad_s": vec3(&calibrated_gyro(&calibration.calib_gyro_bias, s.gyro_rad_s)),
                "calibrated_accel_m_s2": vec3(&calibrated_accel(&calibration.calib_accel_bias, s.accel_m_s2)),
            })
        })
        .collect();
    let out = json!({
        "schema_version": 1,
        "provenance": "rust-visloc-basalt-m7o",
        "frame_from": {"id": 1, "timestamp_ns": start_ns},
        "frame_to": {"id": 2, "timestamp_ns": end_ns},
        "interval_semantics": "(camera_timestamp_1, camera_timestamp_2]",
        "selected_sample_count": samples.len(),
        "selected_samples": samples,
        "bias_gyro": vec3(&bg),
        "bias_accel": vec3(&ba),
        "noise_density": {"gyro": noise.gyro_density, "accel": noise.accel_density},
        "delta_time_s": delta.delta_time,
        "delta_rotation_quaternion_wxyz": [delta.delta_rotation.w, delta.delta_rotation.i, delta.delta_rotation.j, delta.delta_rotation.k],
        "delta_velocity": vec3(&delta.delta_velocity),
        "delta_position": vec3(&delta.delta_position),
        "jacobians": {
            "rotation_gyro_bias": matrix9(&nalgebra::SMatrix::<f64,9,9>::from_fn(|r,c| if r < 3 && c < 3 { delta.jacobian_rotation_gyro_bias[(r,c)] } else { 0.0 })),
            "velocity_gyro_bias": matrix9(&nalgebra::SMatrix::<f64,9,9>::from_fn(|r,c| if r < 3 && c < 3 { delta.jacobian_velocity_gyro_bias[(r,c)] } else { 0.0 })),
            "velocity_accel_bias": matrix9(&nalgebra::SMatrix::<f64,9,9>::from_fn(|r,c| if r < 3 && c < 3 { delta.jacobian_velocity_accel_bias[(r,c)] } else { 0.0 })),
            "position_gyro_bias": matrix9(&nalgebra::SMatrix::<f64,9,9>::from_fn(|r,c| if r < 3 && c < 3 { delta.jacobian_position_gyro_bias[(r,c)] } else { 0.0 })),
            "position_accel_bias": matrix9(&nalgebra::SMatrix::<f64,9,9>::from_fn(|r,c| if r < 3 && c < 3 { delta.jacobian_position_accel_bias[(r,c)] } else { 0.0 })),
        },
        "covariance_row_order": ["position", "rotation", "velocity"],
        "covariance_9x9": matrix9(&delta.covariance),
        "sqrt_information_9x9": matrix9(&factor.sqrt_information),
        "state_from": trace_state(&trace, 1),
        "state_to": trace_state(&trace, 2),
        "raw_residual_position_rotation_velocity": raw_residual.iter().copied().collect::<Vec<_>>(),
        "whitened_residual_position_rotation_velocity": factor.residual.iter().copied().collect::<Vec<_>>(),
        "state_jacobian_shape": [factor.state_jacobian.nrows(), factor.state_jacobian.ncols()],
    });
    fs::write(&args[5], serde_json::to_string_pretty(&out)?)?;
    Ok(())
}
