use nalgebra::{UnitQuaternion, Vector3};
use visloc_basalt::imu::{integrate_between, interpolate_at, SamplingError, TimestampedImu};

const fn sample(t: i64, gyro: Vector3<f64>, accel: Vector3<f64>) -> TimestampedImu {
    TimestampedImu::new(t, gyro, accel)
}

#[test]
fn static_acceleration_matches_position_and_velocity_golden() {
    let a = Vector3::new(0.0, 0.0, 9.81);
    let d = integrate_between(
        &[
            sample(0, Vector3::zeros(), a),
            sample(1_000_000_000, Vector3::zeros(), a),
        ],
        0,
        1_000_000_000,
        Vector3::zeros(),
        Vector3::zeros(),
        None,
    )
    .unwrap();
    assert!((d.delta_velocity - a).norm() < 1e-12);
    assert!((d.delta_position - a * 0.5).norm() < 1e-12);
    assert!(d.delta_rotation.angle() < 1e-12);
}

#[test]
fn constant_angular_rate_matches_rotation_golden() {
    let w = std::f64::consts::FRAC_PI_2;
    let d = integrate_between(
        &[
            sample(0, Vector3::new(0.0, 0.0, w), Vector3::zeros()),
            sample(1_000_000_000, Vector3::new(0.0, 0.0, w), Vector3::zeros()),
        ],
        0,
        1_000_000_000,
        Vector3::zeros(),
        Vector3::zeros(),
        None,
    )
    .unwrap();
    let expected = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), w);
    assert!((d.delta_rotation.inverse() * expected).angle() < 1e-12);
    assert!(d.delta_velocity.norm() < 1e-12);
}

#[test]
fn constant_acceleration_matches_closed_form_golden() {
    let a = Vector3::new(2.0, 0.0, 0.0);
    let d = integrate_between(
        &[
            sample(0, Vector3::zeros(), a),
            sample(1_000_000_000, Vector3::zeros(), a),
        ],
        0,
        1_000_000_000,
        Vector3::zeros(),
        Vector3::zeros(),
        None,
    )
    .unwrap();
    assert!((d.delta_velocity - a).norm() < 1e-12);
    assert!((d.delta_position - Vector3::new(1.0, 0.0, 0.0)).norm() < 1e-12);
}

#[test]
fn jacobian_and_covariance_contract_is_finite_and_symmetric() {
    let a = Vector3::new(0.2, -0.1, 9.81);
    let d = integrate_between(
        &[
            sample(0, Vector3::new(0.1, 0.0, -0.1), a),
            sample(1_000_000_000, Vector3::new(0.1, 0.0, -0.1), a),
        ],
        0,
        1_000_000_000,
        Vector3::zeros(),
        Vector3::zeros(),
        Some(visloc_basalt::imu::ImuNoiseModel {
            gyro_density: 1e-3,
            accel_density: 2e-3,
        }),
    )
    .unwrap();
    assert!((d.covariance - d.covariance.transpose()).norm() < 1e-15);
    assert!(d.jacobian_rotation_gyro_bias.norm().is_finite());
    assert!(d.jacobian_velocity_accel_bias.norm().is_finite());
}

#[test]
fn endpoints_are_interpolated_and_exact_boundaries_work() {
    let samples = [
        sample(0, Vector3::zeros(), Vector3::new(2.0, 0.0, 0.0)),
        sample(1_000_000_000, Vector3::zeros(), Vector3::new(2.0, 0.0, 0.0)),
    ];
    let mid = interpolate_at(&samples, 250_000_000).unwrap();
    assert_eq!(mid.timestamp_ns, 250_000_000);
    let d = integrate_between(
        &samples,
        250_000_000,
        750_000_000,
        Vector3::zeros(),
        Vector3::zeros(),
        None,
    )
    .unwrap();
    assert!((d.delta_time - 0.5).abs() < 1e-12);
    assert!((d.delta_velocity.x - 1.0).abs() < 1e-12);
    assert!((d.delta_position.x - 0.25).abs() < 1e-12);
}

#[test]
fn invalid_and_out_of_range_windows_are_rejected() {
    let s = [
        sample(10, Vector3::zeros(), Vector3::zeros()),
        sample(20, Vector3::zeros(), Vector3::zeros()),
    ];
    assert_eq!(
        integrate_between(&s, 20, 20, Vector3::zeros(), Vector3::zeros(), None),
        Err(SamplingError::InvalidInterval)
    );
    assert_eq!(
        integrate_between(&s, 0, 20, Vector3::zeros(), Vector3::zeros(), None),
        Err(SamplingError::EndpointOutside)
    );
    let duplicate = [s[0], s[0]];
    assert_eq!(
        interpolate_at(&duplicate, 10),
        Err(SamplingError::NonIncreasing)
    );
}
