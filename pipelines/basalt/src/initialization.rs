//! Basalt-style deterministic IMU-only initial state construction.
use nalgebra::{Unit, UnitQuaternion, Vector3};
use thiserror::Error;

use crate::imu::{integrate_between, ImuPreintegratedDelta, SamplingError};
use crate::{BasaltNavState, ImuSample};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InitializationConfig {
    pub gravity_m_s2: f64,
    pub minimum_static_samples: usize,
}

impl Default for InitializationConfig {
    fn default() -> Self {
        Self {
            gravity_m_s2: 9.81,
            minimum_static_samples: 2,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct InitialState {
    pub nav: BasaltNavState,
    pub gravity_world_m_s2: Vector3<f64>,
    pub first_camera_timestamp_ns: i64,
    pub preintegrated_to_camera: ImuPreintegratedDelta,
}

#[derive(Debug, Error, PartialEq)]
pub enum InitializationError {
    #[error("not enough static IMU samples")]
    InsufficientStaticSamples,
    #[error("static IMU average acceleration is degenerate")]
    DegenerateAcceleration,
    #[error("camera timestamp must be after the first IMU timestamp")]
    InvalidCameraTimestamp,
    #[error("IMU sampling failed: {0}")]
    Sampling(#[from] SamplingError),
}

/// Estimate gyro bias and gravity direction from the initial stationary window.
/// The measured specific force points opposite world gravity; the returned
/// rotation maps the body-frame force direction to world up.
pub fn estimate_initial_state(
    samples: &[ImuSample],
    camera_timestamp_ns: i64,
    config: InitializationConfig,
) -> Result<InitialState, InitializationError> {
    if samples.len() < config.minimum_static_samples.max(2) {
        return Err(InitializationError::InsufficientStaticSamples);
    }
    if camera_timestamp_ns <= samples[0].timestamp_ns {
        return Err(InitializationError::InvalidCameraTimestamp);
    }
    let mean_gyro = samples
        .iter()
        .fold(Vector3::zeros(), |sum, s| sum + s.gyro_rad_s)
        / samples.len() as f64;
    let mean_accel = samples
        .iter()
        .fold(Vector3::zeros(), |sum, s| sum + s.accel_m_s2)
        / samples.len() as f64;
    let norm = mean_accel.norm();
    if !norm.is_finite() || norm < 1e-9 {
        return Err(InitializationError::DegenerateAcceleration);
    }
    let up_body = Unit::new_normalize(mean_accel);
    let up_world = Vector3::z_axis();
    let rotation = UnitQuaternion::rotation_between(&up_body, &up_world)
        .unwrap_or_else(UnitQuaternion::identity);
    let gravity = -config.gravity_m_s2.abs() * up_world.into_inner();
    let delta = integrate_between(
        samples,
        samples[0].timestamp_ns,
        camera_timestamp_ns,
        mean_gyro,
        Vector3::zeros(),
        None,
    )?;
    let delta_velocity_world = rotation.transform_vector(&delta.delta_velocity);
    let nav = BasaltNavState {
        imu_to_world: visloc_core::geometry::SE3::new(
            rotation * delta.delta_rotation,
            rotation.transform_vector(&delta.delta_position)
                + gravity * (0.5 * delta.delta_time * delta.delta_time),
        ),
        velocity_world_m_s: delta_velocity_world + gravity * delta.delta_time,
        gyro_bias_rad_s: mean_gyro,
        accel_bias_m_s2: Vector3::zeros(),
    };
    Ok(InitialState {
        nav,
        gravity_world_m_s2: gravity,
        first_camera_timestamp_ns: camera_timestamp_ns,
        preintegrated_to_camera: delta,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    fn s(t: i64, g: Vector3<f64>, a: Vector3<f64>) -> ImuSample {
        ImuSample::new(t, g, a)
    }

    #[test]
    fn stationary_fixture_initializes_gravity_and_zero_motion() {
        let samples = [
            s(0, Vector3::zeros(), Vector3::new(0.0, 0.0, 9.81)),
            s(
                1_000_000_000,
                Vector3::zeros(),
                Vector3::new(0.0, 0.0, 9.81),
            ),
            s(
                2_000_000_000,
                Vector3::zeros(),
                Vector3::new(0.0, 0.0, 9.81),
            ),
        ];
        let x =
            estimate_initial_state(&samples, 500_000_000, InitializationConfig::default()).unwrap();
        assert!((x.gravity_world_m_s2 - Vector3::new(0.0, 0.0, -9.81)).norm() < 1e-12);
        assert!(x.nav.velocity_world_m_s.norm() < 1e-12);
        assert!(x.nav.imu_to_world.rotation.angle() < 1e-12);
    }

    #[test]
    fn tilted_gravity_fixture_aligns_body_force_to_world_up() {
        let tilt = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.4);
        let a = tilt
            .inverse()
            .transform_vector(&Vector3::new(0.0, 0.0, 9.81));
        let samples = [
            s(0, Vector3::zeros(), a),
            s(1_000_000_000, Vector3::zeros(), a),
        ];
        let x =
            estimate_initial_state(&samples, 500_000_000, InitializationConfig::default()).unwrap();
        assert!(
            (x.nav.imu_to_world.rotation.transform_vector(&a.normalize())
                - Vector3::new(0.0, 0.0, 1.0))
            .norm()
                < 1e-10
        );
    }

    #[test]
    fn constant_gyro_bias_is_recorded_and_removed() {
        let bias = Vector3::new(0.01, -0.02, 0.03);
        let samples = [
            s(0, bias, Vector3::new(0.0, 0.0, 9.81)),
            s(1_000_000_000, bias, Vector3::new(0.0, 0.0, 9.81)),
        ];
        let x =
            estimate_initial_state(&samples, 500_000_000, InitializationConfig::default()).unwrap();
        assert!((x.nav.gyro_bias_rad_s - bias).norm() < 1e-12);
        assert!(x.preintegrated_to_camera.delta_rotation.angle() < 1e-12);
    }

    #[test]
    fn first_camera_endpoint_is_integrated_by_interpolation() {
        let samples = [
            s(0, Vector3::zeros(), Vector3::new(2.0, 0.0, 0.0)),
            s(1_000_000_000, Vector3::zeros(), Vector3::new(2.0, 0.0, 0.0)),
        ];
        let x =
            estimate_initial_state(&samples, 500_000_000, InitializationConfig::default()).unwrap();
        assert!((x.preintegrated_to_camera.delta_time - 0.5).abs() < 1e-12);
        assert!((x.preintegrated_to_camera.delta_velocity.x - 1.0).abs() < 1e-12);
    }
}
