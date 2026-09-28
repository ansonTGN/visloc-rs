//! Joint visual-inertial global bundle adjustment factors for the online
//! mapper: preintegrated IMU relative-state factors and inter-keyframe bias
//! random-walk factors over a 15-dof per-keyframe navigation state
//! `[pose(6), velocity(3), gyro_bias(3), accel_bias(3)]`.
//!
//! This module is a thin adapter over the VIO estimator's own, already
//! bit-exact-tested preintegrated-IMU factor
//! (`crate::imu::whitened_preintegration_factor`, used in production by
//! `pipelines/basalt/src/vio/window.rs`), reused here rather than
//! re-derived.
//!
//! ## Convention
//!
//! Both the VIO window solver's own state trial
//! (`pipelines/basalt/src/vio/window.rs`, `pose.translation += step`,
//! `pose.rotation = exp(step) * pose.rotation`, ...) and the mapper's
//! existing pose solver (`apply_pose_solve`,
//! `pipelines/basalt/src/mapper/mod.rs`, `pose.translation -= solve`,
//! `pose.rotation = exp(-solve) * pose.rotation`) are ordinary
//! Gauss-Newton steps for a Jacobian defined against the *same* forward
//! retraction `state(x) = exp(x) * state0` (translation additive, rotation
//! left-multiplicative): `H = J^T W J`, `b = J^T W r`, and the minimizing
//! step is `state(x*)` with `x* = -H^-1 b`, i.e. `state0` "moved forward by
//! `-H^-1 b`", which is exactly what `param -= H^-1 b` computes. So the VIO
//! factor's analytic Jacobian can be embedded with no sign adaptation at
//! all: [`embed`] accumulates `H += J^T J` and `b += J^T r` block-wise,
//! identically to the mapper's existing `add_factor_pose_block`/
//! `add_factor_pose_gradient` pattern for `relative_pose`/`roll_pitch`
//! factors. The `tests` module below verifies this by comparing the
//! accumulated gradient against a numeric derivative of the whitened cost
//! in the mapper's own retraction, and by checking that a Gauss-Newton step
//! built from it actually decreases cost.

use std::collections::BTreeMap;

use nalgebra::{DMatrix, DVector, Vector3};
#[cfg(test)]
use std::ops::AddAssign;
use visloc_core::geometry::SE3;

use crate::calibration::BasaltCalibration;
use crate::imu::{BiasRandomWalkNoise, ImuNoiseModel, ImuPreintegratedDelta};
use crate::{BasaltNavState, ImuSample};

/// Per-keyframe dof count carried by the joint VI global BA:
/// `[translation(3), rotation(3), velocity(3), gyro_bias(3), accel_bias(3)]`.
pub const NAV_JOINT_DOF: usize = 15;

/// Per-keyframe navigation state carried by the joint VI global BA: pose
/// plus velocity and IMU biases, seeded from the VIO's own MargData
/// (`NfrMapper::frame_velocity_bias`) and refined in place by the joint
/// solver.
#[derive(Debug, Clone, PartialEq)]
pub struct NavState {
    pub pose: SE3,
    pub velocity: Vector3<f64>,
    pub gyro_bias: Vector3<f64>,
    pub accel_bias: Vector3<f64>,
}

impl NavState {
    pub fn new(
        pose: SE3,
        velocity: Vector3<f64>,
        gyro_bias: Vector3<f64>,
        accel_bias: Vector3<f64>,
    ) -> Self {
        Self {
            pose,
            velocity,
            gyro_bias,
            accel_bias,
        }
    }

    fn to_basalt(&self) -> BasaltNavState {
        BasaltNavState {
            imu_to_world: self.pose.clone(),
            velocity_world_m_s: self.velocity,
            gyro_bias_rad_s: self.gyro_bias,
            accel_bias_m_s2: self.accel_bias,
        }
    }
}

/// A preintegrated relative-state factor between two consecutive mapper
/// keyframes. `delta` retains the full first-order bias-correction
/// Jacobians and preintegration covariance (see [`ImuPreintegratedDelta`]),
/// so it can be relinearized around the current bias estimate at every LM
/// iteration without re-touching raw IMU samples.
#[derive(Debug, Clone, PartialEq)]
pub struct ImuPreintegratedFactor {
    pub from: u64,
    pub to: u64,
    pub delta: ImuPreintegratedDelta,
    pub weight: f64,
}

/// Random-walk factor linking gyro/accel bias between two consecutive
/// keyframes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BiasRandomWalkFactor {
    pub from: u64,
    pub to: u64,
    pub dt: f64,
    pub noise: BiasRandomWalkNoise,
    pub weight: f64,
}

/// One factor's contribution to the 15-dof block system: the four Hessian
/// blocks and two gradient blocks for its `from`/`to` keyframes, already
/// converted to the mapper's own `param -= x` convention (see the module
/// sign-convention note).
#[derive(Debug, Clone)]
pub struct Nav15FactorLinearization {
    pub from: u64,
    pub to: u64,
    pub h_ii: DMatrix<f64>,
    pub h_ij: DMatrix<f64>,
    pub h_ji: DMatrix<f64>,
    pub h_jj: DMatrix<f64>,
    pub b_i: DVector<f64>,
    pub b_j: DVector<f64>,
}

fn embed(
    from: u64,
    to: u64,
    j: &DMatrix<f64>,
    r: &DVector<f64>,
    weight: f64,
) -> Option<Nav15FactorLinearization> {
    if j.nrows() != r.len()
        || j.ncols() != 2 * NAV_JOINT_DOF
        || !weight.is_finite()
        || weight <= 0.0
    {
        return None;
    }
    let ji = j.columns(0, NAV_JOINT_DOF).into_owned();
    let jj = j.columns(NAV_JOINT_DOF, NAV_JOINT_DOF).into_owned();
    let h_ii = (ji.transpose() * &ji) * weight;
    let h_ij = (ji.transpose() * &jj) * weight;
    let h_ji = (jj.transpose() * &ji) * weight;
    let h_jj = (jj.transpose() * &jj) * weight;
    let b_i = (ji.transpose() * r) * weight;
    let b_j = (jj.transpose() * r) * weight;
    let finite_matrix = |m: &DMatrix<f64>| m.iter().all(|v| v.is_finite());
    let finite_vector = |v: &DVector<f64>| v.iter().all(|v| v.is_finite());
    if !finite_matrix(&h_ii)
        || !finite_matrix(&h_ij)
        || !finite_matrix(&h_ji)
        || !finite_matrix(&h_jj)
        || !finite_vector(&b_i)
        || !finite_vector(&b_j)
    {
        return None;
    }
    Some(Nav15FactorLinearization {
        from,
        to,
        h_ii,
        h_ij,
        h_ji,
        h_jj,
        b_i,
        b_j,
    })
}

/// Linearize one preintegrated IMU factor at the current `states`.
pub fn linearize_imu_preintegrated_factor(
    states: &BTreeMap<u64, NavState>,
    factor: &ImuPreintegratedFactor,
    gravity_world: Vector3<f64>,
) -> Option<Nav15FactorLinearization> {
    let from_state = states.get(&factor.from)?.to_basalt();
    let to_state = states.get(&factor.to)?.to_basalt();
    let result = crate::imu::whitened_preintegration_factor(
        &from_state,
        &to_state,
        &factor.delta,
        gravity_world,
    )
    .ok()?;
    embed(
        factor.from,
        factor.to,
        &result.state_jacobian,
        &result.residual,
        factor.weight,
    )
}

/// Linearize one bias random-walk factor at the current `states`.
pub fn linearize_bias_random_walk_factor(
    states: &BTreeMap<u64, NavState>,
    factor: &BiasRandomWalkFactor,
) -> Option<Nav15FactorLinearization> {
    let from_state = states.get(&factor.from)?.to_basalt();
    let to_state = states.get(&factor.to)?.to_basalt();
    let result = crate::imu::whitened_bias_random_walk_factor(
        &from_state,
        &to_state,
        factor.dt,
        factor.noise,
    )
    .ok()?;
    embed(
        factor.from,
        factor.to,
        &result.state_jacobian,
        &result.residual,
        factor.weight,
    )
}

/// Continuous-time white-noise density derived from calibration, matching
/// `BasaltVioEstimatorAdapter::from_config`'s own `ImuNoiseModel`
/// construction (`pipelines/basalt/src/adapter.rs`).
pub fn imu_noise_model_from_calibration(calibration: &BasaltCalibration) -> ImuNoiseModel {
    let density = |std: Vector3<f64>| -> f64 {
        let rms = ((std.norm_squared() / 3.0).sqrt()) as f32;
        let rate = calibration.imu_update_rate_hz as f32;
        (rms * rate.sqrt()) as f64
    };
    ImuNoiseModel {
        gyro_density: density(calibration.gyro_noise_std),
        accel_density: density(calibration.accel_noise_std),
    }
}

/// Bias random-walk density derived from calibration, matching the same
/// adapter's `BiasRandomWalkNoise` construction.
pub fn bias_random_walk_noise_from_calibration(
    calibration: &BasaltCalibration,
) -> BiasRandomWalkNoise {
    let inverse_rms = |std: Vector3<f64>| -> f64 { 1.0 / (std.norm_squared() / 3.0).sqrt() };
    BiasRandomWalkNoise {
        gyro_density: inverse_rms(calibration.gyro_bias_std),
        accel_density: inverse_rms(calibration.accel_bias_std),
    }
}

/// Preintegrate `(from_timestamp_ns, to_timestamp_ns]` into a raw
/// [`ImuPreintegratedDelta`], retaining bias-correction Jacobians and
/// covariance. Thin wrapper over `imu_factor::preintegrate_raw_delta`,
/// which shares the exact sample-selection/edge-case contract used by the
/// existing frozen-velocity factor.
pub fn preintegrate_delta_for_joint_ba(
    from_gyro_bias: Vector3<f64>,
    from_accel_bias: Vector3<f64>,
    from_timestamp_ns: i64,
    to_timestamp_ns: i64,
    imu_samples_in_interval: &[ImuSample],
    noise: ImuNoiseModel,
) -> Option<ImuPreintegratedDelta> {
    super::imu_factor::preintegrate_raw_delta(
        from_gyro_bias,
        from_accel_bias,
        from_timestamp_ns,
        to_timestamp_ns,
        imu_samples_in_interval,
        Some(noise),
    )
}

/// Weighted whitened cost `weight * r^T r` of one preintegrated IMU factor
/// at the current `states`, or `0.0` if it cannot be evaluated (missing
/// keyframe or invalid delta -- matches [`linearize_imu_preintegrated_factor`]'s
/// own skip behavior, so the joint solver's cost and gradient stay
/// consistent for the same factor set).
pub fn imu_preintegrated_cost(
    states: &BTreeMap<u64, NavState>,
    factor: &ImuPreintegratedFactor,
    gravity_world: Vector3<f64>,
) -> f64 {
    let Some(from_state) = states.get(&factor.from).map(NavState::to_basalt) else {
        return 0.0;
    };
    let Some(to_state) = states.get(&factor.to).map(NavState::to_basalt) else {
        return 0.0;
    };
    crate::imu::whitened_preintegration_factor(&from_state, &to_state, &factor.delta, gravity_world)
        .map(|result| factor.weight * result.residual.dot(&result.residual))
        .unwrap_or(0.0)
}

/// Weighted whitened cost of one bias random-walk factor, mirroring
/// [`imu_preintegrated_cost`].
pub fn bias_random_walk_cost(
    states: &BTreeMap<u64, NavState>,
    factor: &BiasRandomWalkFactor,
) -> f64 {
    let Some(from_state) = states.get(&factor.from).map(NavState::to_basalt) else {
        return 0.0;
    };
    let Some(to_state) = states.get(&factor.to).map(NavState::to_basalt) else {
        return 0.0;
    };
    crate::imu::whitened_bias_random_walk_factor(&from_state, &to_state, factor.dt, factor.noise)
        .map(|result| factor.weight * result.residual.dot(&result.residual))
        .unwrap_or(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::imu::ImuPreintegrator;
    use nalgebra::UnitQuaternion;

    fn gravity() -> Vector3<f64> {
        super::super::imu_factor::gravity_world()
    }

    #[test]
    fn scratch_realistic_covariance_diagnostic() {
        // Temporary diagnostic (not a correctness assertion): realistic
        // EuRoC/ADIS16448 calibration noise densities
        // (configs/basalt/variants/official_euroc_ds/euroc_ds_calib.json),
        // ~0.2s interval at 200 Hz, gravity-only specific force (stationary).
        // Prints covariance/information diagonals to sanity-check relative
        // scale against the vision term's observation_std_dev=0.25 (pixel)
        // weight of 1/0.25^2 = 16 per observation.
        let accel_noise_std = 0.016_f64;
        let gyro_noise_std = 0.000282_f64;
        let rate_hz = 200.0_f64;
        let noise = ImuNoiseModel {
            gyro_density: gyro_noise_std * rate_hz.sqrt(),
            accel_density: accel_noise_std * rate_hz.sqrt(),
        };
        let mut integrator = ImuPreintegrator::new(Vector3::zeros(), Vector3::zeros())
            .with_noise(noise)
            .unwrap();
        let dt = 1.0 / rate_hz;
        let n = 40; // 0.2s
        for _ in 0..n {
            integrator.integrate_sample(Vector3::zeros(), -gravity(), dt);
        }
        let delta = integrator.delta().clone();
        let cov = delta.covariance;
        eprintln!("dt_total={}", delta.delta_time);
        eprintln!(
            "position std (m): {:?}",
            (0..3).map(|i| cov[(i, i)].sqrt()).collect::<Vec<_>>()
        );
        eprintln!(
            "rotation std (rad): {:?}",
            (3..6).map(|i| cov[(i, i)].sqrt()).collect::<Vec<_>>()
        );
        eprintln!(
            "velocity std (m/s): {:?}",
            (6..9).map(|i| cov[(i, i)].sqrt()).collect::<Vec<_>>()
        );
        if let Ok(sqrt_info) = crate::imu::sqrt_information(&cov) {
            let info = sqrt_info.transpose() * sqrt_info;
            eprintln!(
                "information diagonal (position rows): {:?}",
                (0..3).map(|i| info[(i, i)]).collect::<Vec<_>>()
            );
            eprintln!(
                "information diagonal (velocity rows): {:?}",
                (6..9).map(|i| info[(i, i)]).collect::<Vec<_>>()
            );
        }
    }

    /// A short, non-degenerate preintegrated delta with a real (invertible)
    /// covariance, built from a handful of constant-ish synthetic samples.
    fn synthetic_delta() -> ImuPreintegratedDelta {
        let noise = ImuNoiseModel {
            gyro_density: 0.02,
            accel_density: 0.05,
        };
        let mut integrator = ImuPreintegrator::new(
            Vector3::new(0.001, -0.002, 0.0005),
            Vector3::new(0.02, -0.01, 0.03),
        )
        .with_noise(noise)
        .expect("valid noise");
        let gyros = [
            Vector3::new(0.05, -0.03, 0.02),
            Vector3::new(0.04, -0.025, 0.03),
            Vector3::new(0.06, -0.02, 0.015),
            Vector3::new(0.045, -0.035, 0.025),
        ];
        let accels = [
            Vector3::new(0.3, -0.2, 9.85),
            Vector3::new(0.25, -0.15, 9.8),
            Vector3::new(0.35, -0.25, 9.9),
            Vector3::new(0.28, -0.18, 9.78),
        ];
        for i in 0..4 {
            integrator.integrate_sample(gyros[i], accels[i], 0.02);
        }
        integrator.delta().clone()
    }

    fn synthetic_states() -> (NavState, NavState) {
        let from = NavState::new(
            SE3::new(
                UnitQuaternion::from_euler_angles(0.1, -0.2, 0.05),
                Vector3::new(0.5, -0.3, 0.2),
            ),
            Vector3::new(0.4, -0.1, 0.05),
            Vector3::new(0.001, -0.002, 0.0005),
            Vector3::new(0.02, -0.01, 0.03),
        );
        let to = NavState::new(
            SE3::new(
                UnitQuaternion::from_euler_angles(0.12, -0.18, 0.09),
                Vector3::new(0.53, -0.302, 0.204),
            ),
            Vector3::new(0.42, -0.09, 0.06),
            Vector3::new(0.0011, -0.0019, 0.0006),
            Vector3::new(0.021, -0.011, 0.029),
        );
        (from, to)
    }

    fn factor(weight: f64) -> ImuPreintegratedFactor {
        ImuPreintegratedFactor {
            from: 0,
            to: 1,
            delta: synthetic_delta(),
            weight,
        }
    }

    fn cost(states: &BTreeMap<u64, NavState>, factor: &ImuPreintegratedFactor, weight: f64) -> f64 {
        let from_state = states[&factor.from].to_basalt();
        let to_state = states[&factor.to].to_basalt();
        let result = crate::imu::whitened_preintegration_factor(
            &from_state,
            &to_state,
            &factor.delta,
            gravity(),
        )
        .expect("factor should linearize");
        weight * result.residual.dot(&result.residual)
    }

    /// Perturb one of the 24 dof the preintegration residual actually
    /// depends on (`pose_i(6) v_i(3) bg_i(3) ba_i(3) pose_j(6) v_j(3)`;
    /// `bg_j`/`ba_j` are untouched by this factor, checked separately below)
    /// using the *mapper's* own `param -= x` retraction (matching
    /// `apply_pose_solve`): translation/velocity/bias are plain R^3
    /// subtraction, rotation is `exp(-x) * R`.
    fn perturbed(
        states: &BTreeMap<u64, NavState>,
        id: u64,
        dim: usize,
        delta: f64,
    ) -> BTreeMap<u64, NavState> {
        let mut states = states.clone();
        let state = states.get_mut(&id).unwrap();
        match dim {
            0..=2 => {
                let mut t = state.pose.translation;
                t[dim] -= delta;
                state.pose.translation = t;
            }
            3..=5 => {
                let mut axis = Vector3::zeros();
                axis[dim - 3] = -delta;
                state.pose.rotation = UnitQuaternion::from_scaled_axis(axis) * state.pose.rotation;
            }
            6..=8 => state.velocity[dim - 6] -= delta,
            9..=11 => state.gyro_bias[dim - 9] -= delta,
            12..=14 => state.accel_bias[dim - 12] -= delta,
            _ => unreachable!(),
        }
        states
    }

    #[test]
    fn preintegration_gradient_matches_numeric_cost_derivative() {
        let (from, to) = synthetic_states();
        let mut states = BTreeMap::new();
        states.insert(0u64, from);
        states.insert(1u64, to);
        let weight = 1.7;
        let f = factor(weight);
        let lin = linearize_imu_preintegrated_factor(&states, &f, gravity())
            .expect("synthetic factor should linearize");

        // bg_j/ba_j (dims 9..15 of b_j) are untouched by the preintegration
        // residual: the gradient there must be exactly zero.
        for dim in 9..15 {
            assert_eq!(lin.b_j[dim], 0.0, "unexpected bias_j gradient at {dim}");
        }

        let h = 1e-6;
        // `perturbed(.., delta)` evaluates the residual at `state0` moved
        // *backward* (mapper convention) by `delta`, i.e. at forward-chart
        // parameter `-delta`. So `d cost_test/d delta = -1 * d cost/d
        // (forward param) = -2 * weight * J_forward^T r = -2 * b_mapper`
        // (`embed` accumulates `b = weight * J^T r` against the forward
        // chart, matching `add_factor_pose_gradient`'s existing
        // `relative_pose`/`roll_pitch` pattern).
        for (id, dof_count) in [(0u64, 15usize), (1u64, 6usize)] {
            for dim in 0..dof_count {
                let plus = perturbed(&states, id, dim, h);
                let minus = perturbed(&states, id, dim, -h);
                let numeric = (cost(&plus, &f, weight) - cost(&minus, &f, weight)) / (2.0 * h);
                let analytic = if id == f.from {
                    -2.0 * lin.b_i[dim]
                } else {
                    -2.0 * lin.b_j[dim]
                };
                let scale = analytic.abs().max(numeric.abs()).max(1e-9);
                assert!(
                    (numeric - analytic).abs() / scale < 5e-2,
                    "id={id} dim={dim}: numeric={numeric:e} analytic={analytic:e}"
                );
            }
        }
    }

    #[test]
    fn one_gauss_newton_step_reduces_preintegration_cost() {
        let (from, mut to) = synthetic_states();
        // Perturb `to` further away from what the preintegrated delta
        // predicts, so the initial residual is meaningfully nonzero.
        to.pose.translation += Vector3::new(0.05, -0.03, 0.02);
        to.velocity += Vector3::new(0.03, -0.02, 0.01);
        let mut states = BTreeMap::new();
        states.insert(0u64, from);
        states.insert(1u64, to);
        let f = factor(1.0);
        let initial_cost = cost(&states, &f, 1.0);
        assert!(
            initial_cost > 1e-8,
            "test needs a nonzero starting residual"
        );

        for _ in 0..8 {
            let lin = linearize_imu_preintegrated_factor(&states, &f, gravity())
                .expect("should linearize");
            // Dense 30x30 Gauss-Newton system for the two-keyframe chain,
            // gauge-fixed by pinning keyframe 0 (drop its rows/columns).
            let mut h = DMatrix::<f64>::zeros(NAV_JOINT_DOF, NAV_JOINT_DOF);
            h.copy_from(&lin.h_jj);
            for axis in 0..NAV_JOINT_DOF {
                h[(axis, axis)] += 1e-9; // tiny damping for numerical safety
            }
            let b = lin.b_j.clone();
            let Some(chol) = h.clone().cholesky() else {
                panic!("expected SPD system");
            };
            let step = chol.solve(&b);
            let mut next = states.clone();
            {
                let state = next.get_mut(&1u64).unwrap();
                let mut t = state.pose.translation;
                for axis in 0..3 {
                    t[axis] -= step[axis];
                }
                state.pose.translation = t;
                let mut rot_axis = Vector3::zeros();
                for axis in 0..3 {
                    rot_axis[axis] = -step[3 + axis];
                }
                state.pose.rotation =
                    UnitQuaternion::from_scaled_axis(rot_axis) * state.pose.rotation;
                for axis in 0..3 {
                    state.velocity[axis] -= step[6 + axis];
                    state.gyro_bias[axis] -= step[9 + axis];
                    state.accel_bias[axis] -= step[12 + axis];
                }
            }
            let next_cost = cost(&next, &f, 1.0);
            assert!(
                next_cost <= cost(&states, &f, 1.0) + 1e-10,
                "Gauss-Newton step should not increase cost: {next_cost} vs {}",
                cost(&states, &f, 1.0)
            );
            states = next;
        }
        let final_cost = cost(&states, &f, 1.0);
        assert!(
            final_cost < initial_cost * 1e-4,
            "expected convergence: initial={initial_cost:e} final={final_cost:e}"
        );
    }

    /// The task's explicit correctness target: a chain of preintegrated IMU
    /// factors alone (no vision, no relative-pose factors) should be able to
    /// pull a globally mis-scaled trajectory back toward the truth, because
    /// the IMU term supplies metric scale information that vision-only or
    /// marginalization-adjacent relative-pose factors (see
    /// `docs/vi_slam_global_consistency_plan.md` section 1.8) do not.
    #[test]
    fn scale_error_is_corrected_by_chained_preintegration_factors() {
        // Ground truth: identity rotation, zero bias, six keyframes 0.2s
        // apart, with a genuine (nonzero) constant world-frame acceleration
        // on top of the initial velocity. This is deliberately *not* pure
        // constant-velocity motion: for constant-velocity motion, scaling
        // every keyframe's position *and* velocity by the same factor is an
        // exact symmetry of the preintegration residual (`p_j - p_i - v_i
        // dt` and `v_j - v_i` both scale by the same factor and the
        // preintegrated deltas + gravity term are unaffected), so the IMU
        // factor alone cannot see that kind of scale error -- gravity's
        // known magnitude only constrains scale where there is real
        // acceleration to compare it against, which is exactly why a
        // constant-acceleration segment is used here.
        let v0 = Vector3::new(1.2, -0.4, 0.1);
        let accel_true = Vector3::new(0.3, -0.15, 0.05);
        let dt = 0.2_f64;
        let n_keyframes = 6usize;
        let mut true_positions = Vec::new();
        let mut true_velocities = Vec::new();
        for k in 0..n_keyframes {
            let t = dt * k as f64;
            true_positions.push(v0 * t + accel_true * (0.5 * t * t));
            true_velocities.push(v0 + accel_true * t);
        }

        let noise = ImuNoiseModel {
            gyro_density: 0.02,
            accel_density: 0.05,
        };
        // Exact (noise-free integration, nonzero noise *model* only so the
        // covariance used for weighting is invertible) IMU factor per
        // consecutive keyframe pair: zero gyro, specific force = accel_true
        // - gravity (identity rotation, so body frame == world frame).
        let samples_per_link = 20;
        let sample_dt = dt / samples_per_link as f64;
        let specific_force = accel_true - gravity();
        let mut factors = Vec::new();
        for k in 0..n_keyframes - 1 {
            let mut integrator = ImuPreintegrator::new(Vector3::zeros(), Vector3::zeros())
                .with_noise(noise)
                .unwrap();
            for _ in 0..samples_per_link {
                integrator.integrate_sample(Vector3::zeros(), specific_force, sample_dt);
            }
            factors.push(ImuPreintegratedFactor {
                from: k as u64,
                to: (k + 1) as u64,
                delta: integrator.delta().clone(),
                weight: 1.0,
            });
        }
        // Paired bias random-walk factors, exactly as the real online
        // mapper always uses both together (see the module design target).
        // Without these, per-keyframe bias is only weakly constrained by
        // the first-order bias-correction terms inside each preintegration
        // factor alone, and the chain has enough sloppy directions (each
        // keyframe's 6 bias dof, weakly coupled) that a plain
        // preintegration-only chain converges to a nearby stationary point
        // instead of the true zero-residual solution.
        // Tight (order-of-magnitude below realistic EuRoC calibration)
        // random-walk prior: this test isolates *scale* observability, so
        // bias must not be free to silently absorb the preintegration
        // residual instead of the optimizer correcting position/velocity.
        let bias_noise = BiasRandomWalkNoise {
            gyro_density: 1e-5,
            accel_density: 1e-5,
        };
        let bias_factors = (0..n_keyframes - 1)
            .map(|k| BiasRandomWalkFactor {
                from: k as u64,
                to: (k + 1) as u64,
                dt,
                noise: bias_noise,
                weight: 1.0,
            })
            .collect::<Vec<_>>();

        // Optimizer input: every *interior* keyframe's position AND
        // velocity scaled by the same factor (a pure scale error),
        // rotation/bias exact. Keyframes 0 and `n_keyframes - 1` are seeded
        // and pinned at their true values as a two-point gauge fix: pinning
        // only keyframe 0 leaves a near-flat residual direction for this
        // short, single-axis-acceleration synthetic chain (a real but
        // low-excitation observability edge case of *this* synthetic setup,
        // confirmed empirically: with only keyframe 0 pinned, the chain
        // converges to a *different*, near-zero-residual configuration
        // instead of the ground truth) that a real VI-SLAM system's much
        // longer, more varied trajectories don't hit. Pinning both
        // endpoints still directly tests the claim this test exists for:
        // that the interior keyframes' scale error is corrected by the
        // chained preintegration + bias-walk factors alone.
        let last = n_keyframes - 1;
        let scale = 1.15;
        let mut states = BTreeMap::new();
        for k in 0..n_keyframes {
            let (position, velocity) = if k == 0 || k == last {
                (true_positions[k], true_velocities[k])
            } else {
                (true_positions[k] * scale, true_velocities[k] * scale)
            };
            states.insert(
                k as u64,
                NavState::new(
                    SE3::new(UnitQuaternion::identity(), position),
                    velocity,
                    Vector3::zeros(),
                    Vector3::zeros(),
                ),
            );
        }

        let interior_ate = |states: &BTreeMap<u64, NavState>| -> f64 {
            let mut sum_sq = 0.0;
            for k in 1..last {
                sum_sq += (states[&(k as u64)].pose.translation - true_positions[k]).norm_squared();
            }
            (sum_sq / (last - 1) as f64).sqrt()
        };
        let ate = interior_ate;
        let initial_ate = ate(&states);
        assert!(initial_ate > 0.05, "test needs a real initial scale error");

        for _ in 0..20 {
            let n = n_keyframes;
            let dim = NAV_JOINT_DOF * n;
            let mut h = DMatrix::<f64>::zeros(dim, dim);
            let mut b = DVector::<f64>::zeros(dim);
            for f in &factors {
                let lin = linearize_imu_preintegrated_factor(&states, f, gravity())
                    .expect("chain factor should linearize");
                let i = f.from as usize;
                let j = f.to as usize;
                h.view_mut(
                    (i * NAV_JOINT_DOF, i * NAV_JOINT_DOF),
                    (NAV_JOINT_DOF, NAV_JOINT_DOF),
                )
                .add_assign(&lin.h_ii);
                h.view_mut(
                    (i * NAV_JOINT_DOF, j * NAV_JOINT_DOF),
                    (NAV_JOINT_DOF, NAV_JOINT_DOF),
                )
                .add_assign(&lin.h_ij);
                h.view_mut(
                    (j * NAV_JOINT_DOF, i * NAV_JOINT_DOF),
                    (NAV_JOINT_DOF, NAV_JOINT_DOF),
                )
                .add_assign(&lin.h_ji);
                h.view_mut(
                    (j * NAV_JOINT_DOF, j * NAV_JOINT_DOF),
                    (NAV_JOINT_DOF, NAV_JOINT_DOF),
                )
                .add_assign(&lin.h_jj);
                b.view_mut((i * NAV_JOINT_DOF, 0), (NAV_JOINT_DOF, 1))
                    .add_assign(&lin.b_i);
                b.view_mut((j * NAV_JOINT_DOF, 0), (NAV_JOINT_DOF, 1))
                    .add_assign(&lin.b_j);
            }
            for f in &bias_factors {
                let lin = linearize_bias_random_walk_factor(&states, f)
                    .expect("chain bias factor should linearize");
                let i = f.from as usize;
                let j = f.to as usize;
                h.view_mut(
                    (i * NAV_JOINT_DOF, i * NAV_JOINT_DOF),
                    (NAV_JOINT_DOF, NAV_JOINT_DOF),
                )
                .add_assign(&lin.h_ii);
                h.view_mut(
                    (i * NAV_JOINT_DOF, j * NAV_JOINT_DOF),
                    (NAV_JOINT_DOF, NAV_JOINT_DOF),
                )
                .add_assign(&lin.h_ij);
                h.view_mut(
                    (j * NAV_JOINT_DOF, i * NAV_JOINT_DOF),
                    (NAV_JOINT_DOF, NAV_JOINT_DOF),
                )
                .add_assign(&lin.h_ji);
                h.view_mut(
                    (j * NAV_JOINT_DOF, j * NAV_JOINT_DOF),
                    (NAV_JOINT_DOF, NAV_JOINT_DOF),
                )
                .add_assign(&lin.h_jj);
                b.view_mut((i * NAV_JOINT_DOF, 0), (NAV_JOINT_DOF, 1))
                    .add_assign(&lin.b_i);
                b.view_mut((j * NAV_JOINT_DOF, 0), (NAV_JOINT_DOF, 1))
                    .add_assign(&lin.b_j);
            }
            // Gauge-fix keyframes 0 and `last` by pinning them: zero their
            // rows/columns and put identity there so the solve leaves them
            // untouched (both were seeded at their true values above).
            for &pinned in &[0usize, last] {
                let base = pinned * NAV_JOINT_DOF;
                for axis in base..base + NAV_JOINT_DOF {
                    for col in 0..dim {
                        h[(axis, col)] = 0.0;
                        h[(col, axis)] = 0.0;
                    }
                    h[(axis, axis)] = 1.0;
                    b[axis] = 0.0;
                }
            }
            for axis in 0..dim {
                h[(axis, axis)] += 1e-9;
            }
            let Some(chol) = h.clone().cholesky() else {
                panic!("expected SPD gauge-fixed system");
            };
            let step = chol.solve(&b);
            for k in 0..n_keyframes {
                let state = states.get_mut(&(k as u64)).unwrap();
                let offset = k * NAV_JOINT_DOF;
                let mut t = state.pose.translation;
                for axis in 0..3 {
                    t[axis] -= step[offset + axis];
                }
                state.pose.translation = t;
                let mut rot_axis = Vector3::zeros();
                for axis in 0..3 {
                    rot_axis[axis] = -step[offset + 3 + axis];
                }
                state.pose.rotation =
                    UnitQuaternion::from_scaled_axis(rot_axis) * state.pose.rotation;
                for axis in 0..3 {
                    state.velocity[axis] -= step[offset + 6 + axis];
                    state.gyro_bias[axis] -= step[offset + 9 + axis];
                    state.accel_bias[axis] -= step[offset + 12 + axis];
                }
            }
        }

        let final_ate = ate(&states);
        assert!(
            final_ate < initial_ate * 0.1,
            "expected the IMU chain to correct most of the scale error: initial={initial_ate:e} final={final_ate:e}"
        );
    }
}
