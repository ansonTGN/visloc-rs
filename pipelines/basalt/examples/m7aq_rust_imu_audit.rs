//! Rust side of the M7aq per-link IMU boundary audit.
//!
//! The input oracle JSON supplies the exact pinned-float deltas/covariances;
//! this keeps the comparison focused on Rust residual/Jacobian/LDLT semantics
//! before a later audit swaps in the estimator's live integration path.

use std::{env, fs};

use nalgebra::{DMatrix, DVector, Quaternion, SMatrix, UnitQuaternion, Vector3};
use serde_json::{json, Value};
use visloc_basalt::{
    imu::{
        imu_row_products_f32, preintegration_jacobian_f32, preintegration_residual_and_jacobian,
        preintegration_residual_f32, whitened_preintegration_factor_upstream_f32,
        ImuPreintegratedDelta,
    },
    BasaltNavState,
};
use visloc_core::geometry::SE3;

type M9 = SMatrix<f64, 9, 9>;
type M3 = SMatrix<f64, 3, 3>;
type M30F = SMatrix<f32, 9, 30>;
type H75F = SMatrix<f32, 75, 75>;
type B75F = SMatrix<f32, 75, 1>;

fn v3(value: &Value) -> Vector3<f64> {
    Vector3::new(
        value[0].as_f64().unwrap(),
        value[1].as_f64().unwrap(),
        value[2].as_f64().unwrap(),
    )
}

fn m9(value: &Value) -> M9 {
    M9::from_fn(|row, col| value[row][col].as_f64().unwrap())
}

fn state(value: &Value) -> BasaltNavState {
    let q = &value["rotation_xyzw"];
    BasaltNavState {
        imu_to_world: SE3::new(
            UnitQuaternion::from_quaternion(Quaternion::new(
                q[3].as_f64().unwrap(),
                q[0].as_f64().unwrap(),
                q[1].as_f64().unwrap(),
                q[2].as_f64().unwrap(),
            )),
            v3(&value["translation"]),
        ),
        velocity_world_m_s: v3(&value["velocity"]),
        gyro_bias_rad_s: v3(&value["bias_gyro"]),
        accel_bias_m_s2: v3(&value["bias_accel"]),
    }
}

fn delta(value: &Value) -> ImuPreintegratedDelta {
    let mut result = ImuPreintegratedDelta::identity(
        v3(&value["from_state"]["bias_gyro"]),
        v3(&value["from_state"]["bias_accel"]),
    );
    result.delta_time = (value["delta"]["dt_ns"].as_i64().unwrap() as f32 * 1.0e-9_f32) as f64;
    let q = &value["delta"]["rotation_xyzw"];
    result.delta_rotation = UnitQuaternion::from_quaternion(Quaternion::new(
        q[3].as_f64().unwrap(),
        q[0].as_f64().unwrap(),
        q[1].as_f64().unwrap(),
        q[2].as_f64().unwrap(),
    ));
    result.delta_position = v3(&value["delta"]["position"]);
    result.delta_velocity = v3(&value["delta"]["velocity"]);
    // d_state_d_* is 9x3 in the oracle. Parse the three named blocks below.
    let ba = &value["delta"]["d_state_d_ba"];
    let bg = &value["delta"]["d_state_d_bg"];
    result.jacobian_position_accel_bias = M3::from_fn(|r, c| ba[r][c].as_f64().unwrap());
    result.jacobian_velocity_accel_bias = M3::from_fn(|r, c| ba[r + 6][c].as_f64().unwrap());
    result.jacobian_position_gyro_bias = M3::from_fn(|r, c| bg[r][c].as_f64().unwrap());
    result.jacobian_rotation_gyro_bias = M3::from_fn(|r, c| bg[r + 3][c].as_f64().unwrap());
    result.jacobian_velocity_gyro_bias = M3::from_fn(|r, c| bg[r + 6][c].as_f64().unwrap());
    result.covariance = m9(&value["covariance"]);
    result
}

fn vec_json(values: impl IntoIterator<Item = f64>) -> Value {
    Value::Array(values.into_iter().map(|value| json!(value)).collect())
}

fn mat_json<const R: usize, const C: usize>(matrix: &SMatrix<f64, R, C>) -> Value {
    Value::Array(
        (0..R)
            .map(|row| vec_json((0..C).map(|col| matrix[(row, col)])))
            .collect(),
    )
}

fn dmat_json(matrix: &DMatrix<f64>) -> Value {
    Value::Array(
        (0..matrix.nrows())
            .map(|row| vec_json((0..matrix.ncols()).map(|col| matrix[(row, col)])))
            .collect(),
    )
}

fn dvec_json(vector: &DVector<f64>) -> Value {
    vec_json(vector.iter().copied())
}

fn add_global(
    h: &mut H75F,
    b: &mut B75F,
    j: &M30F,
    r: &SMatrix<f32, 9, 1>,
    from: usize,
    dt_ns: i64,
) {
    let h_local = j.transpose() * j;
    let b_local = j.transpose() * r;
    let to = from + 1;
    for row in 0..15 {
        for col in 0..15 {
            h[(from * 15 + row, from * 15 + col)] += h_local[(row, col)];
            h[(from * 15 + row, to * 15 + col)] += h_local[(row, col + 15)];
            h[(to * 15 + row, from * 15 + col)] += h_local[(row + 15, col)];
            h[(to * 15 + row, to * 15 + col)] += h_local[(row + 15, col + 15)];
        }
        b[from * 15 + row] += b_local[row];
        b[to * 15 + row] += b_local[row + 15];
    }
    let dt = ((dt_ns as f32) * 1.0e-9_f32).max(1.0e-9);
    let wg = 1.0e4_f32 / dt.sqrt();
    let wa = 1.0e3_f32 / dt.sqrt();
    for axis in 0..3 {
        let gi = from * 15 + 9 + axis;
        let gj = to * 15 + 9 + axis;
        let ai = from * 15 + 12 + axis;
        let aj = to * 15 + 12 + axis;
        h[(gi, gi)] += wg * wg;
        h[(gi, gj)] -= wg * wg;
        h[(gj, gi)] -= wg * wg;
        h[(gj, gj)] += wg * wg;
        h[(ai, ai)] += wa * wa;
        h[(ai, aj)] -= wa * wa;
        h[(aj, ai)] -= wa * wa;
        h[(aj, aj)] += wa * wa;
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = env::args().collect();
    if args.len() != 3 {
        eprintln!("usage: m7aq_rust_imu_audit <upstream-audit.json> <output.json>");
        std::process::exit(2);
    }
    let oracle: Value = serde_json::from_str(&fs::read_to_string(&args[1])?)?;
    let gravity = Vector3::new(0.0, 0.0, -9.8100004196166992);
    let mut links = Vec::new();
    let mut h = H75F::zeros();
    let mut b = B75F::zeros();
    for (index, link) in oracle["links"].as_array().unwrap().iter().enumerate() {
        let from = state(&link["from_state"]);
        let to = state(&link["to_state"]);
        let d = delta(link);
        let (raw64, jac64) = preintegration_residual_and_jacobian(&from, &to, &d, gravity);
        let raw32 = SMatrix::<f32, 9, 1>::from_fn(|row, _| raw64[row] as f32);
        let raw32_exact = preintegration_residual_f32(&from, &to, &d, gravity);
        let jacobian32_exact = preintegration_jacobian_f32(&from, &to, &d, gravity);
        let jacobian32 = M30F::from_fn(|row, col| jac64[(row, col)] as f32);
        let factor = whitened_preintegration_factor_upstream_f32(&from, &to, &d, gravity)
            .map_err(|error| format!("factor build failed: {error:?}"))?;
        let white_jac = M30F::from_fn(|row, col| factor.state_jacobian[(row, col)] as f32);
        let white_raw = SMatrix::<f32, 9, 1>::from_fn(|row, _| factor.residual[row] as f32);
        let (row_h, row_b) = imu_row_products_f32(&white_jac, &white_raw);
        add_global(
            &mut h,
            &mut b,
            &white_jac,
            &white_raw,
            index,
            link["delta"]["dt_ns"].as_i64().unwrap(),
        );
        let (dr, dv, dp) = d.corrected(from.gyro_bias_rad_s, from.accel_bias_m_s2);
        links.push(json!({
            "from_frame": link["from_frame"],
            "to_frame": link["to_frame"],
            "delta": link["delta"],
            "corrected_delta": {
                "dt_ns": link["delta"]["dt_ns"],
                "position": vec_json(dp.iter().copied()),
                "rotation_xyzw": vec_json([dr.i, dr.j, dr.k, dr.w]),
                "velocity": vec_json(dv.iter().copied()),
            },
            "raw_residual": dvec_json(&raw64),
            "raw_residual_f32": vec_json(raw32.iter().map(|value| *value as f64)),
            "raw_residual_f32_exact": vec_json(raw32_exact.iter().map(|value| *value as f64)),
            "jacobian": dmat_json(&jac64),
            "jacobian_f32": mat_json(&jacobian32.map(|value| value as f64)),
            "jacobian_f32_exact": mat_json(&jacobian32_exact.map(|value| value as f64)),
            "covariance": mat_json(&d.covariance),
            "oracle_covariance": link["covariance"],
            "sqrt_information": mat_json(&factor.sqrt_information),
            "whitened_residual": dvec_json(&factor.residual),
            "whitened_jacobian": dmat_json(&factor.state_jacobian),
            "row_h": mat_json(&row_h.map(|value| value as f64)),
            "row_b": vec_json(row_b.iter().map(|value| *value as f64)),
            "objective": 0.5_f32 * white_raw.norm_squared(),
        }));
    }
    let h64 = h.map(|value| value as f64);
    let b64 = b.map(|value| value as f64);
    fs::write(
        &args[2],
        serde_json::to_string_pretty(&json!({
            "schema": "basalt.imu_link_audit.v1",
            "scalar": "rust-f32-boundary",
            "links": links,
            "accumulated_h": mat_json(&h64),
            "accumulated_b": vec_json(b64.iter().copied()),
        }))?,
    )?;
    Ok(())
}
