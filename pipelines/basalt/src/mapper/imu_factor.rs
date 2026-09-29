//! Pose-only mapper factors from raw IMU preintegration.

use std::collections::{BTreeMap, BTreeSet};

use nalgebra::{DMatrix, DVector, UnitQuaternion, Vector3};
use visloc_core::geometry::SE3;

use crate::{imu::ImuPreintegrator, ImuSample};

use super::{row_major_values, RelativePoseFactor};

/// Preintegrate `(from_timestamp_ns, to_timestamp_ns]` into a relative pose.
/// Velocity and biases are frozen at the source VIO keyframe estimate; global
/// BA still optimizes only poses. Identity information deliberately avoids a
/// covariance model: all confidence is controlled by `weight` in this prototype.
/// Samples must be strictly ordered; the last selected measurement is held
/// through any trailing partial interval, matching VIO `fallback_integrate`.
#[allow(clippy::too_many_arguments)]
pub fn imu_preintegration_relative_pose_factor(
    from_id: u64,
    to_id: u64,
    from_pose: SE3,
    from_velocity_world: Vector3<f64>,
    from_gyro_bias: Vector3<f64>,
    from_accel_bias: Vector3<f64>,
    from_timestamp_ns: i64,
    to_timestamp_ns: i64,
    imu_samples_in_interval: &[ImuSample],
    weight: f64,
) -> Option<RelativePoseFactor> {
    if !weight.is_finite() {
        return None;
    }
    let delta = preintegrate_corrected_delta(
        from_gyro_bias,
        from_accel_bias,
        from_timestamp_ns,
        to_timestamp_ns,
        imu_samples_in_interval,
    )?;
    relative_pose_factor_from_delta(
        from_id,
        to_id,
        from_pose,
        from_velocity_world,
        &delta,
        weight,
    )
}

/// Retain rotation as well: the original measurement uses the IMU rotation.
pub(crate) struct CorrectedDelta {
    pub dt: f64,
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub rotation: UnitQuaternion<f64>,
}

pub(crate) fn preintegrate_corrected_delta(
    from_gyro_bias: Vector3<f64>,
    from_accel_bias: Vector3<f64>,
    from_timestamp_ns: i64,
    to_timestamp_ns: i64,
    imu_samples_in_interval: &[ImuSample],
) -> Option<CorrectedDelta> {
    let delta = preintegrate_raw_delta(
        from_gyro_bias,
        from_accel_bias,
        from_timestamp_ns,
        to_timestamp_ns,
        imu_samples_in_interval,
        None,
    )?;
    let dt = delta.delta_time;
    // Biases match the linearization point, but keep the IMU factor convention.
    let (rotation, velocity, position) = delta.corrected(from_gyro_bias, from_accel_bias);
    if !dt.is_finite()
        || dt <= 0.0
        || !position
            .iter()
            .chain(velocity.iter())
            .chain(rotation.coords.iter())
            .all(|x| x.is_finite())
    {
        return None;
    }
    Some(CorrectedDelta {
        dt,
        position,
        velocity,
        rotation,
    })
}

/// Preintegrate `(from_timestamp_ns, to_timestamp_ns]` into a raw
/// [`crate::imu::ImuPreintegratedDelta`], retaining the first-order
/// bias-correction Jacobians and (when `noise` is supplied) the
/// preintegration covariance. Shares the exact sample-selection/edge-case
/// contract with [`preintegrate_corrected_delta`], which now calls this
/// helper (with `noise: None`) and immediately collapses the result to a
/// single bias-corrected translation for the frozen-velocity relative-pose
/// factor. The joint VI-BA module (`mapper::imu_ba`) uses the raw delta
/// directly instead, so it can relinearize around the current bias estimate
/// at every LM iteration without re-touching raw IMU samples.
pub(crate) fn preintegrate_raw_delta(
    from_gyro_bias: Vector3<f64>,
    from_accel_bias: Vector3<f64>,
    from_timestamp_ns: i64,
    to_timestamp_ns: i64,
    imu_samples_in_interval: &[ImuSample],
    noise: Option<crate::imu::ImuNoiseModel>,
) -> Option<crate::imu::ImuPreintegratedDelta> {
    if imu_samples_in_interval.is_empty()
        || to_timestamp_ns <= from_timestamp_ns
        || imu_samples_in_interval
            .windows(2)
            .any(|s| s[0].timestamp_ns >= s[1].timestamp_ns)
    {
        return None;
    }
    let mut integrator = ImuPreintegrator::new(from_gyro_bias, from_accel_bias);
    if let Some(noise) = noise {
        integrator = integrator.with_noise(noise)?;
    }
    let selected = imu_samples_in_interval
        .iter()
        .filter(|sample| {
            sample.timestamp_ns > from_timestamp_ns && sample.timestamp_ns <= to_timestamp_ns
        })
        .collect::<Vec<_>>();
    let last = *selected.last()?;
    let mut previous_timestamp = from_timestamp_ns;
    for sample in selected {
        let dt = sample.timestamp_ns.checked_sub(previous_timestamp)? as f64 * 1e-9;
        if !dt.is_finite() || dt <= 0.0 {
            return None;
        }
        integrator.integrate_sample(sample.gyro_rad_s, sample.accel_m_s2, dt);
        previous_timestamp = sample.timestamp_ns;
    }
    if previous_timestamp < to_timestamp_ns {
        let dt = to_timestamp_ns.checked_sub(previous_timestamp)? as f64 * 1e-9;
        if !dt.is_finite() || dt <= 0.0 {
            return None;
        }
        integrator.integrate_sample(last.gyro_rad_s, last.accel_m_s2, dt);
    }
    let delta = integrator.delta().clone();
    if !delta.delta_time.is_finite() || delta.delta_time <= 0.0 {
        return None;
    }
    Some(delta)
}

pub(crate) fn gravity_world() -> Vector3<f64> {
    Vector3::new(0.0, 0.0, -9.81)
}

pub(crate) fn relative_pose_factor_from_delta(
    from_id: u64,
    to_id: u64,
    from_pose: SE3,
    from_velocity_world: Vector3<f64>,
    delta: &CorrectedDelta,
    weight: f64,
) -> Option<RelativePoseFactor> {
    let dt = delta.dt;
    let translation = from_pose.rotation.inverse()
        * (from_velocity_world * dt + gravity_world() * (0.5 * dt * dt))
        + delta.position;
    if !weight.is_finite() || !translation.iter().all(|x| x.is_finite()) {
        return None;
    }
    let q = delta.rotation.quaternion();
    Some(RelativePoseFactor {
        from: from_id,
        to: to_id,
        translation: translation.into(),
        rotation: [q.w, q.i, q.j, q.k],
        information: row_major_values(&DMatrix::identity(6, 6)),
        weight,
    })
}

pub type VelocityPair = (u64, u64, SE3, SE3, f64, Vector3<f64>, Vector3<f64>);

/// Solve position and velocity continuity residuals with poses held fixed.
pub fn refine_pair_velocities(
    pairs: &[VelocityPair],
    initial_velocities: &BTreeMap<u64, Vector3<f64>>,
) -> BTreeMap<u64, Vector3<f64>> {
    let mut result = initial_velocities.clone();
    let ids = pairs
        .iter()
        .flat_map(|p| [p.0, p.1])
        .collect::<BTreeSet<_>>();
    if ids.is_empty() {
        return result;
    }
    let indices = ids
        .into_iter()
        .enumerate()
        .map(|(i, id)| (id, i))
        .collect::<BTreeMap<_, _>>();
    let mut h = DMatrix::<f64>::zeros(3 * indices.len(), 3 * indices.len());
    let mut b = DVector::<f64>::zeros(3 * indices.len());
    // Tiny prior relative to the unit velocity-continuity terms. Untouched
    // IDs never enter the solve and retain their prior values exactly.
    const EPSILON: f64 = 1e-6;
    for (&id, &i) in &indices {
        let prior = initial_velocities.get(&id).copied().unwrap_or_default();
        for axis in 0..3 {
            h[(3 * i + axis, 3 * i + axis)] = EPSILON;
            b[3 * i + axis] = EPSILON * prior[axis];
        }
    }
    for (from, to, from_pose, to_pose, dt, dp, dv) in pairs {
        let i = 3 * indices[from];
        let j = 3 * indices[to];
        // Multiplying residuals by R preserves squared norms: J_p = dt I,
        // J_v = [-I, I], with these world-frame targets (solve H v = J^T target).
        let position_target = to_pose.translation
            - from_pose.translation
            - gravity_world() * (0.5 * dt * dt)
            - from_pose.rotation * dp;
        let velocity_target = gravity_world() * *dt + from_pose.rotation * dv;
        for axis in 0..3 {
            h[(i + axis, i + axis)] += dt * dt + 1.0;
            h[(j + axis, j + axis)] += 1.0;
            h[(i + axis, j + axis)] -= 1.0;
            h[(j + axis, i + axis)] -= 1.0;
            b[i + axis] += dt * position_target[axis] - velocity_target[axis];
            b[j + axis] += velocity_target[axis];
        }
    }
    if let Some(cholesky) = h.cholesky() {
        let solved = cholesky.solve(&b);
        if solved.iter().all(|x| x.is_finite()) {
            for (id, i) in indices {
                result.insert(
                    id,
                    Vector3::new(solved[3 * i], solved[3 * i + 1], solved[3 * i + 2]),
                );
            }
        }
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::UnitQuaternion;

    fn synthetic_pair(
        from: u64,
        to: u64,
        pose: SE3,
        next: SE3,
        v: Vector3<f64>,
        next_v: Vector3<f64>,
        dt: f64,
    ) -> VelocityPair {
        let dp = pose.rotation.inverse()
            * (next.translation - pose.translation - v * dt - gravity_world() * (0.5 * dt * dt));
        let dv = pose.rotation.inverse() * (next_v - v - gravity_world() * dt);
        (from, to, pose, next, dt, dp, dv)
    }

    #[test]
    fn refine_two_keyframe_velocities() {
        let v = Vector3::new(1.0, 2.0, 3.0);
        let next_v = Vector3::new(2.0, 3.0, 4.0);
        let pair = synthetic_pair(
            10,
            20,
            SE3::identity(),
            SE3::new(UnitQuaternion::identity(), v),
            v,
            next_v,
            1.0,
        );
        let prior = BTreeMap::from([(10, Vector3::zeros()), (20, Vector3::zeros()), (99, v)]);
        let solved = refine_pair_velocities(&[pair], &prior);
        assert!((solved[&10] - v).norm() < 2e-5);
        assert!((solved[&20] - next_v).norm() < 2e-5);
        assert_eq!(solved[&99], v);
    }

    #[test]
    fn refine_chain_shared_middle_velocity() {
        let velocities = [
            Vector3::new(1.0, 2.0, 3.0),
            Vector3::new(2.0, -1.0, 0.5),
            Vector3::new(3.0, 0.5, -2.0),
        ];
        let poses = [
            SE3::identity(),
            SE3::new(
                UnitQuaternion::from_euler_angles(0.2, -0.4, 0.8),
                Vector3::new(1.0, 2.0, 3.0),
            ),
            SE3::new(UnitQuaternion::identity(), Vector3::new(4.0, 3.0, 2.0)),
        ];
        let pairs = (0..2)
            .map(|i| {
                synthetic_pair(
                    i as u64,
                    i as u64 + 1,
                    poses[i].clone(),
                    poses[i + 1].clone(),
                    velocities[i],
                    velocities[i + 1],
                    0.5,
                )
            })
            .collect::<Vec<_>>();
        let solved = refine_pair_velocities(&pairs, &BTreeMap::new());
        for (i, v) in velocities.iter().enumerate() {
            assert!((solved[&(i as u64)] - v).norm() < 1e-4);
        }
    }

    fn factor(
        pose: SE3,
        velocity: Vector3<f64>,
        end: i64,
        samples: &[ImuSample],
    ) -> Option<RelativePoseFactor> {
        imu_preintegration_relative_pose_factor(
            0,
            1,
            pose,
            velocity,
            Vector3::zeros(),
            Vector3::zeros(),
            0,
            end,
            samples,
            2.0,
        )
    }

    #[test]
    fn stationary_including_trailing_interval() {
        let samples = [250_000_000, 750_000_000]
            .map(|t| ImuSample::new(t, Vector3::zeros(), Vector3::new(0.0, 0.0, 9.81)));
        let f = factor(SE3::identity(), Vector3::zeros(), 1_000_000_000, &samples).unwrap();
        assert!(Vector3::from(f.translation).norm() < 1e-6);
        assert_eq!(f.rotation, [1.0, 0.0, 0.0, 0.0]);
        assert_eq!(f.information, row_major_values(&DMatrix::identity(6, 6)));
        assert_eq!(f.weight, 2.0);
    }

    #[test]
    fn invalid_or_empty_intervals() {
        let sample = ImuSample::new(10, Vector3::zeros(), Vector3::zeros());
        for (end, samples) in [
            (20, &[][..]),
            (0, &[sample][..]),
            (-1, &[sample][..]),
            (5, &[sample][..]),
            (20, &[sample, sample][..]),
        ] {
            assert!(factor(SE3::identity(), Vector3::zeros(), end, samples).is_none());
        }
        let bad = ImuSample::new(10, Vector3::zeros(), Vector3::repeat(f64::NAN));
        assert!(factor(SE3::identity(), Vector3::zeros(), 20, &[bad]).is_none());
    }

    #[test]
    fn rotated_source_with_world_velocity() {
        let pose = SE3::new(
            UnitQuaternion::from_euler_angles(0.3, -0.5, 0.8),
            Vector3::zeros(),
        );
        let velocity = Vector3::new(1.0, 2.0, 3.0);
        let sample = ImuSample::new(1_000_000_000, Vector3::zeros(), Vector3::zeros());
        let f = factor(pose.clone(), velocity, 1_000_000_000, &[sample]).unwrap();
        let expected = pose.rotation.inverse() * (velocity + Vector3::new(0.0, 0.0, -4.905));
        assert!((Vector3::from(f.translation) - expected).norm() < 1e-9);
        assert_eq!(f.rotation, [1.0, 0.0, 0.0, 0.0]);
    }
}
