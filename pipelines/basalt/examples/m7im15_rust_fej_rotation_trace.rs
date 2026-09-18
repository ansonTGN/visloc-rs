//! Dump the Rust f32 FEJ SO(3) producer stages from an existing IMU input
//! diagnostic.  This is intentionally a detached audit executable: it does
//! not run the estimator or change any production state.

use std::{env, fs};

use nalgebra::{Matrix3, Quaternion, UnitQuaternion, Vector3};
use serde_json::{json, Value};
use visloc_basalt::{
    imu::{
        preintegration_f32_rotation_producer_trace, preintegration_jacobian_f32,
        whitened_preintegration_factor_upstream_f32_fej, ImuPreintegratedDelta,
    },
    BasaltNavState,
};
use visloc_core::geometry::SE3;

fn f32_bits(value: &Value) -> f32 {
    let text = value.as_str().expect("bit string");
    f32::from_bits(u32::from_str_radix(text.trim_start_matches("0x"), 16).expect("bits"))
}

fn vec3(value: &Value) -> Vector3<f64> {
    Vector3::new(
        f32_bits(&value[0]) as f64,
        f32_bits(&value[1]) as f64,
        f32_bits(&value[2]) as f64,
    )
}

fn mat3(value: &Value) -> Matrix3<f64> {
    Matrix3::from_fn(|row, col| f32_bits(&value[col * 3 + row]) as f64)
}

fn mat9(value: &Value) -> nalgebra::SMatrix<f64, 9, 9> {
    nalgebra::SMatrix::from_fn(|row, col| f32_bits(&value[col * 9 + row]) as f64)
}

fn state(value: &Value) -> BasaltNavState {
    let q = &value["quaternion_xyzw_f32_bits"];
    let rotation = UnitQuaternion::new_unchecked(Quaternion::new(
        f32_bits(&q[3]) as f64,
        f32_bits(&q[0]) as f64,
        f32_bits(&q[1]) as f64,
        f32_bits(&q[2]) as f64,
    ));
    BasaltNavState {
        imu_to_world: SE3::new(rotation, vec3(&value["translation_f32_bits"])),
        velocity_world_m_s: vec3(&value["velocity_f32_bits"]),
        gyro_bias_rad_s: vec3(&value["bias_gyro_f32_bits"]),
        accel_bias_m_s2: vec3(&value["bias_accel_f32_bits"]),
    }
}

fn delta(root: &Value) -> ImuPreintegratedDelta {
    let p = &root["preintegrated"];
    let bg = vec3(&p["bias_gyro_f32_bits"]);
    let ba = vec3(&p["bias_accel_f32_bits"]);
    let mut result = ImuPreintegratedDelta::identity(bg, ba);
    result.delta_time = f32_bits(&root["delta_time_bits"]) as f64;
    let q = &p["delta_rotation_f32_bits"]; // diagnostic spelling is wxyz
    result.delta_rotation = UnitQuaternion::new_unchecked(Quaternion::new(
        f32_bits(&q[0]) as f64,
        f32_bits(&q[1]) as f64,
        f32_bits(&q[2]) as f64,
        f32_bits(&q[3]) as f64,
    ));
    result.delta_position = vec3(&p["delta_position_f32_bits"]);
    result.delta_velocity = vec3(&p["delta_velocity_f32_bits"]);
    result.jacobian_position_accel_bias = mat3(&p["jacobian_position_accel_bias_f32_bits"]);
    result.jacobian_position_gyro_bias = mat3(&p["jacobian_position_gyro_bias_f32_bits"]);
    result.jacobian_rotation_gyro_bias = mat3(&p["jacobian_rotation_gyro_bias_f32_bits"]);
    result.jacobian_velocity_accel_bias = mat3(&p["jacobian_velocity_accel_bias_f32_bits"]);
    result.jacobian_velocity_gyro_bias = mat3(&p["jacobian_velocity_gyro_bias_f32_bits"]);
    result.covariance = mat9(&p["covariance_f32_bits"]);
    result
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = env::args().collect();
    let input = args.get(1).map_or(
        "target/m7im15_so3_bias_fresh7_frame6_full_link0_iter3_inputs_20260828.json",
        String::as_str,
    );
    let output = args.get(2).map_or(
        "target/m7im15_rust_fej_rotation_producer_trace_20260828.json",
        String::as_str,
    );
    let root: Value = serde_json::from_str(&fs::read_to_string(input)?)?;
    let current_from = state(&root["current"]["from"]);
    let current_to = state(&root["current"]["to"]);
    let fej_from = state(&root["fej"]["from"]);
    let fej_to = state(&root["fej"]["to"]);
    let d = delta(&root);
    let gravity = Vector3::new(0.0, 0.0, -9.8100004196166992_f64);
    let fej_jacobian = preintegration_jacobian_f32(&fej_from, &fej_to, &d, gravity);
    let jacobian_bits = (0..30)
        .flat_map(|column| {
            (0..9).map(move |row| format!("{:08x}", fej_jacobian[(row, column)].to_bits()))
        })
        .collect::<Vec<_>>();
    let d_bg_bits = (0..3)
        .flat_map(|column| {
            (3..6).map(move |row| format!("{:08x}", fej_jacobian[(row, 9 + column)].to_bits()))
        })
        .collect::<Vec<_>>();
    let factor = whitened_preintegration_factor_upstream_f32_fej(
        &current_from,
        &current_to,
        &fej_from,
        &fej_to,
        &d,
        gravity,
    )
    .map_err(|error| format!("f32 factor failed: {error:?}"))?;
    let whitened_matrix = &factor.state_jacobian;
    let whitened_d_bg_bits = (0..3)
        .flat_map(|column| {
            (0..9).map(move |row| (whitened_matrix[(row, 9 + column)] as f32).to_bits())
        })
        .collect::<Vec<_>>();
    let output_value = json!({
        "schema": "basalt.m7im15.rust_fej_rotation_producer_trace.v1",
        "source": input,
        "active_frame_id": root["active_frame_id"],
        "from_index": root["from_index"],
        "iteration": root["iteration"],
        "start_t_ns": root["start_t_ns"],
        "fej_d_res_d_bg_rotation_bits": d_bg_bits,
        "fej_whitened_d_res_d_bg_bits": whitened_d_bg_bits
            .iter()
            .map(|value| format!("{:08x}", value))
            .collect::<Vec<_>>(),
        "fej_jacobian_bits": jacobian_bits,
        "fej": preintegration_f32_rotation_producer_trace(&fej_from, &fej_to, &d),
        "current": preintegration_f32_rotation_producer_trace(&current_from, &current_to, &d),
    });
    fs::write(output, serde_json::to_string_pretty(&output_value)?)?;
    Ok(())
}
