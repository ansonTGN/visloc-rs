//! Faithful (C2-scoped) port of `estimators/bundle_adjustment.{h,cc}`'s
//! `BundleAdjustmentConfig` and the driver COLMAP wires around a Ceres
//! problem in `bundle_adjustment_ceres.cc`.
//!
//! Ported from (commit `64805cb870b574a569dccc34918d95a2db2b2fee`, pinned by
//! `docs/colmap_rig_mapper_port_plan.md` §0):
//! - `src/colmap/estimators/bundle_adjustment.h:77-150` — config surface
//!   ([`BundleAdjustmentConfig`]: `add_image`, `set_constant_rig_from_world_pose`,
//!   `add_variable_point`/`add_constant_point`, `fix_gauge`).
//! - `src/colmap/estimators/cost_functions/reprojection_error.h:389-417` —
//!   `RigReprojErrorConstantRigCostFunctor`, the control-configuration
//!   residual (fixed `sensor_from_rig` baked in as data, variable
//!   `rig_from_world` + `point3D`, fixed camera intrinsics). This is
//!   **reused, not re-derived**: `pipelines/slam/src/bundle.rs`'s
//!   `BaRigObservation`/`add_rig_observation` already implements exactly
//!   this formulation (verified against `bundle.rs`'s `rig_residual_jacobians`
//!   by a research pass during this port — `point_rig = pose.transform(X)`,
//!   `point_sensor = sensor_from_rig.transform(point_rig)`,
//!   `residual = project(point_sensor) - xy`, `sensor_from_rig`/intrinsics
//!   fixed, `pose`/`X` variable), per §3.3's explicit reuse instruction.
//! - `bundle_adjustment_ceres.cc:300-409` (`FixGaugeWithTwoCamsFromWorld`)
//!   and `:262-293` (`FixGaugeWithThreePoints`) — gauge fixing, mapped onto
//!   `bundle.rs`'s `fix_pose`/`fix_landmark` (whole-pose / whole-point
//!   fixes; see module doc below for why this port fixes *whole* poses/
//!   points rather than COLMAP's finer per-DoF gauge fix).
//!
//! ## Deviations from COLMAP, documented per the C2 task brief
//!
//! 1. **Per-image, not per-pose-DoF, config granularity.** COLMAP's
//!    `BundleAdjustmentConfig` is keyed by `image_t`/`rig_t`/`camera_t`
//!    independently. This port's callers (`mapper.rs`) always add or
//!    constant-fix a frame's images as a whole unit (never a lone camera of
//!    a 2-camera rig), so [`BundleAdjustmentConfig`] tracks `image_ids`
//!    (residual inclusion, exactly matching COLMAP) plus `constant_frame_ids`
//!    (pose gauge, one level coarser than COLMAP's per-sensor
//!    `SetConstantSensorFromRigPose` — irrelevant here since
//!    `ba_refine_sensor_from_rig=0` means `sensor_from_rig` is *never* a
//!    Ceres parameter block in this port at all, matching COLMAP's own
//!    control-configuration behavior exactly, see the reprojection_error.h
//!    citation above).
//! 2. **No intrinsics refinement code path at all** — the control config
//!    (`ba_refine_focal_length/principal_point/extra_params = 0`, plan
//!    §0.1/§1.2) never varies camera intrinsics, and
//!    `BaRigObservation`/`rig_residual_jacobians` in `bundle.rs` never
//!    exposes them as parameters, so there is nothing to wire up.
//! 3. **Point variable/constant default policy is a documented
//!    simplification of `ParameterizePoints`
//!    (`bundle_adjustment_ceres.cc:538-555`).** COLMAP's actual rule: a
//!    point is held constant iff `track.Length() > num_observations_added`
//!    (i.e. **not every** observation of that point was added as a
//!    residual to *this* problem) or it is in `ConstantPoints()`;
//!    `VariablePoints()` works by additionally pulling in the point's
//!    *remaining* observations from images outside `config.Images()` as
//!    extra fixed-pose residuals (`AddPointToProblem`,
//!    `bundle_adjustment_ceres.cc:819+`) so `track.Length() ==
//!    num_observations` holds and it becomes free. This port does **not**
//!    implement that extra-residual pull-in (it would require adding
//!    residuals against frames outside the config with their pose
//!    hard-coded as data, which `bundle.rs`'s `BaRigObservation` doesn't
//!    support without a variable pose parameter block per observation).
//!    Instead: a point is **constant** unless (a) explicitly in
//!    `constant_point3d_ids` (always constant), or (b) in
//!    `variable_point3d_ids` (always variable, trusting the caller — this
//!    is what `AdjustLocalBundle` passes the recently-modified point set
//!    as, per `incremental_mapper.cc:1072-1080`), or (c) **every** track
//!    element's image is inside `config.image_ids` (COLMAP's actual
//!    `track.Length() == num_observations` condition, reproduced exactly
//!    for the common case of a point fully inside the current window).
//!    This slightly under-optimizes points that are new/short-track *and*
//!    not fully contained in the window (COLMAP would still refine them
//!    via the extra pull-in; this port leaves them constant unless the
//!    caller explicitly requests variable) — flagged as a known,
//!    conservative gap, not a correctness bug (constant points still
//!    constrain poses; they just don't themselves improve).
//! 4. **Gauge fixing is whole-pose / whole-point**, not COLMAP's per-DoF
//!    `SetParameterization` trick (fixing e.g. only the X-translation
//!    component of one frame while leaving its other 5 DoF free).
//!    `bundle.rs` has no partial-DoF pose fix (confirmed by a research pass
//!    during this port); `Gauge::TwoFramesFromWorld` fixes two whole frame
//!    poses (12 numbers, redundantly over-determining the 7-DoF similarity
//!    gauge — generically forces it to identity, same practical effect as
//!    COLMAP's `TWO_CAMS_FROM_WORLD`) and `Gauge::ThreePoints` fixes three
//!    whole 3D points (9 numbers, same over-determination argument as
//!    COLMAP's `THREE_POINTS`). Numerically these are *stricter* than
//!    COLMAP's fix (more parameters pinned to exact values instead of one
//!    scalar per anchor), which only affects convergence rate/conditioning,
//!    not the solution's metric correctness (§1.6 of the port plan: gauge
//!    fixing is orthogonal to the metric-scale guarantee, which comes from
//!    the fixed `sensor_from_rig` baked into every residual, unaffected by
//!    this port's choice of *which* extra DoF to pin numerically).
//! 5. **Convergence criteria differ from Ceres.** COLMAP's
//!    `BundleAdjustmentOptions` sets Ceres `function_tolerance=0`,
//!    `gradient_tolerance` (10.0 local / 1.0 global, i.e. effectively
//!    "run to `max_num_iterations`"), `parameter_tolerance=0`
//!    (`incremental_pipeline.cc:201-210,244-249`) — i.e. COLMAP's control
//!    essentially disables early Ceres convergence checks and always runs
//!    the full iteration budget (local 25, global 50, doubled + halved
//!    tolerances for the first <10 frames). `bundle.rs`'s LM loop instead
//!    uses `step_tolerance`/`cost_tolerance`/`relative_cost_tolerance`
//!    (different quantities: parameter-step norm and absolute/relative
//!    cost decrease, not a gradient-norm test) — this port sets
//!    `max_iterations` from COLMAP's local/global defaults and otherwise
//!    keeps `BaConfig::default()`'s already-tight `step_tolerance=1e-7`/
//!    `cost_tolerance=1e-9`, which in practice also drives most solves to
//!    (or near) the iteration cap on non-trivial problems; exact
//!    iteration-by-iteration parity with Ceres is not claimed (§6 risk in
//!    the port plan).

use std::collections::BTreeSet;

use nalgebra::Point3;

use crate::bundle::{BaConfig, BaRigObservation, BundleAdjustment};
use visloc_core::geometry::{Pose, SE3};

use super::reconstruction::Reconstruction;
use super::types::{FrameT, ImageT, Point3DT, SensorT};

/// Port of `BundleAdjustmentGauge` (`bundle_adjustment.h:47-48`). See module
/// doc deviation 4.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Gauge {
    #[default]
    Unspecified,
    TwoFramesFromWorld,
    ThreePoints,
}

/// Port of `BundleAdjustmentConfig` (`bundle_adjustment.h:77-150`). See
/// module doc deviation 1 for the per-image (not per-sensor) granularity.
#[derive(Debug, Clone, Default)]
pub struct BundleAdjustmentConfig {
    image_ids: BTreeSet<ImageT>,
    constant_frame_ids: BTreeSet<FrameT>,
    constant_point3d_ids: BTreeSet<Point3DT>,
    variable_point3d_ids: BTreeSet<Point3DT>,
    gauge: Gauge,
}

impl BundleAdjustmentConfig {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_image(&mut self, image_id: ImageT) {
        self.image_ids.insert(image_id);
    }

    /// Convenience matching mapper.rs's usual call pattern: add every image
    /// of `frame_id`.
    pub fn add_frame(&mut self, recon: &Reconstruction, frame_id: FrameT) {
        for image_id in recon.frame(frame_id).image_ids() {
            self.add_image(image_id);
        }
    }

    pub fn images(&self) -> &BTreeSet<ImageT> {
        &self.image_ids
    }

    pub fn num_images(&self) -> usize {
        self.image_ids.len()
    }

    /// Port of `SetConstantRigFromWorldPose` (`bundle_adjustment.h`).
    pub fn set_constant_rig_from_world_pose(&mut self, frame_id: FrameT) {
        self.constant_frame_ids.insert(frame_id);
    }

    pub fn add_variable_point(&mut self, point3d_id: Point3DT) {
        self.variable_point3d_ids.insert(point3d_id);
    }

    pub fn add_constant_point(&mut self, point3d_id: Point3DT) {
        self.constant_point3d_ids.insert(point3d_id);
    }

    pub fn fix_gauge(&mut self, gauge: Gauge) {
        self.gauge = gauge;
    }
}

/// Port of `BundleAdjustmentOptions`'s control-relevant subset
/// (`bundle_adjustment_ceres.h`, `incremental_pipeline.cc:192-282`'s
/// `LocalBundleAdjustment`/`GlobalBundleAdjustment` factories). See module
/// doc deviation 5 for why only `max_iterations` is threaded through to
/// `bundle.rs`'s `BaConfig`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BundleAdjustmentOptions {
    pub max_num_iterations: usize,
}

impl BundleAdjustmentOptions {
    /// `kDefaultCeresLocalMaxNumIterations` (`incremental_pipeline.cc:47`).
    pub fn local() -> Self {
        Self {
            max_num_iterations: 25,
        }
    }
    /// `kDefaultCeresGlobalMaxNumIterations` (`incremental_pipeline.cc:48`).
    pub fn global() -> Self {
        Self {
            max_num_iterations: 50,
        }
    }
}

fn sensor_from_rig_for_image(recon: &Reconstruction, image_id: ImageT) -> SE3 {
    let image = recon.image(image_id);
    let frame = recon.frame(image.frame_id);
    let rig = recon.rig(frame.rig_id());
    let sensor_id = SensorT::camera(image.camera_id);
    if rig.is_ref_sensor(sensor_id) {
        SE3::identity()
    } else {
        rig.sensor_from_rig(sensor_id)
    }
}

/// Port of `CreateDefaultBundleAdjuster(...)->Solve()` for this control's
/// exact case (fixed `sensor_from_rig`, fixed intrinsics, trivial loss —
/// see module doc). Builds one `bundle.rs::BundleAdjustment` problem from
/// every image in `config`, solves it, and writes optimized frame poses /
/// point positions back into `recon`. Returns `true` iff the solve
/// succeeded and produced a usable solution (mirrors
/// `summary->IsSolutionUsable()`).
pub fn solve(
    options: &BundleAdjustmentOptions,
    config: &BundleAdjustmentConfig,
    recon: &mut Reconstruction,
) -> bool {
    if config.image_ids.is_empty() {
        return false;
    }

    let default_camera = recon
        .camera(
            recon
                .image(*config.image_ids.iter().next().unwrap())
                .camera_id,
        )
        .clone();
    let mut ba = BundleAdjustment::new(default_camera);

    let mut frame_ids: BTreeSet<FrameT> = BTreeSet::new();
    for &image_id in &config.image_ids {
        frame_ids.insert(recon.image(image_id).frame_id);
    }
    for &frame_id in &frame_ids {
        let rig_from_world = recon.frame(frame_id).rig_from_world().clone();
        ba.add_pose(
            frame_id,
            Pose {
                world_to_camera: rig_from_world,
            },
        );
        if config.constant_frame_ids.contains(&frame_id) {
            ba.fix_pose(frame_id);
        }
    }

    let mut added_points: BTreeSet<Point3DT> = BTreeSet::new();
    for &image_id in &config.image_ids {
        let image = recon.image(image_id);
        let frame_id = image.frame_id;
        let camera = recon.camera(image.camera_id).clone();
        let sensor_from_rig = sensor_from_rig_for_image(recon, image_id);
        for point2d in &image.points2d {
            let Some(point3d_id) = point2d.point3d_id else {
                continue;
            };
            added_points.insert(point3d_id);
            ba.add_rig_observation(BaRigObservation {
                keyframe_id: frame_id,
                landmark_id: point3d_id,
                xy: point2d.xy,
                camera: camera.clone(),
                sensor_from_rig: sensor_from_rig.clone(),
            });
        }
    }
    for &point3d_id in &added_points {
        let xyz = recon.point3d(point3d_id).xyz;
        ba.add_landmark(point3d_id, xyz);
        // Deviation 3: see module doc for the exact policy this
        // approximates from `ParameterizePoints`.
        let track_fully_in_window = recon
            .point3d(point3d_id)
            .track
            .iter()
            .all(|el| config.image_ids.contains(&el.image_id));
        let variable = !config.constant_point3d_ids.contains(&point3d_id)
            && (config.variable_point3d_ids.contains(&point3d_id) || track_fully_in_window);
        if !variable {
            ba.fix_landmark(point3d_id);
        }
    }

    if ba.rig_observations.is_empty() || ba.landmarks.is_empty() {
        return false;
    }

    match config.gauge {
        Gauge::Unspecified => {}
        Gauge::TwoFramesFromWorld => {
            let candidates: Vec<FrameT> = frame_ids
                .iter()
                .copied()
                .filter(|id| !ba.fixed_poses.contains(id))
                .take(2)
                .collect();
            for id in candidates {
                ba.fix_pose(id);
            }
        }
        Gauge::ThreePoints => {
            let candidates: Vec<Point3DT> = added_points
                .iter()
                .copied()
                .filter(|id| !ba.fixed_landmarks.contains(id))
                .take(3)
                .collect();
            for id in candidates {
                ba.fix_landmark(id);
            }
        }
    }

    let ba_config = BaConfig {
        max_iterations: options.max_num_iterations,
        // Deviation 6: COLMAP stops Ceres on gradient_tolerance=1e-4
        // (function_tolerance=0). bundle.rs has no gradient criterion, so a
        // relative cost tolerance stands in for it; without it every solve
        // runs to max_iterations while the cost changes by <1e-6.
        relative_cost_tolerance: Some(1.0e-6),
        ..BaConfig::default()
    };

    let started = std::time::Instant::now();
    let n_obs = ba.rig_observations.len();
    let n_lm = ba.landmarks.len();
    let n_fixed_lm = ba.fixed_landmarks.len();
    let Ok(result) = ba.optimize(&ba_config) else {
        return false;
    };
    eprintln!(
        "BA_SOLVE frames={} obs={n_obs} landmarks={n_lm} fixed_landmarks={n_fixed_lm} iterations={} elapsed_ms={}",
        frame_ids.len(),
        result.iterations.len(),
        started.elapsed().as_millis()
    );

    for &frame_id in &frame_ids {
        let pose = &ba.poses[&frame_id];
        recon
            .frame_mut(frame_id)
            .set_rig_from_world(pose.world_to_camera.clone());
    }
    for &point3d_id in &added_points {
        let xyz: Point3<f64> = ba.landmarks[&point3d_id];
        recon.point3d_mut(point3d_id).xyz = xyz;
    }

    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;

    use crate::colmap_incremental::pipeline::reconstruction_from_cache;
    use crate::colmap_incremental::reconstruction::TrackElement;
    use crate::colmap_incremental::test_support::build_synthetic_rig_scene;

    /// C2 task item 7: "BA on a synthetic rig scene converges to ground
    /// truth". Seeds exact ground-truth poses/points, perturbs every frame
    /// except two anchors and every point, then checks `solve` converges
    /// back within tight tolerances — including translation (hence scale),
    /// which the fixed `sensor_from_rig` baseline should pin exactly per
    /// `docs/colmap_rig_mapper_port_plan.md` §1.6.
    #[test]
    fn solve_converges_synthetic_rig_scene() {
        let scene = build_synthetic_rig_scene(4, 3);
        let mut recon = reconstruction_from_cache(&scene.db);

        for (&frame_id, gt) in &scene.ground_truth_rig_from_world {
            recon.frame_mut(frame_id).set_rig_from_world(gt.clone());
            recon.register_frame(frame_id);
        }

        let mut point3d_ids = Vec::new();
        for (j, gt_xyz) in scene.ground_truth_points.iter().enumerate() {
            let mut track = Vec::new();
            for &(i1, i2) in &scene.images_per_frame {
                track.push(TrackElement {
                    image_id: i1,
                    point2d_idx: j,
                });
                track.push(TrackElement {
                    image_id: i2,
                    point2d_idx: j,
                });
            }
            point3d_ids.push(recon.add_point3d(*gt_xyz, track));
        }

        let anchors = [0u64, (scene.images_per_frame.len() - 1) as u64];
        let noise = Vector3::new(0.03, -0.02, 0.015);
        for &frame_id in scene.ground_truth_rig_from_world.keys() {
            if anchors.contains(&frame_id) {
                continue;
            }
            let mut pose = recon.frame(frame_id).rig_from_world().clone();
            pose.translation += noise;
            recon.frame_mut(frame_id).set_rig_from_world(pose);
        }
        for &pid in &point3d_ids {
            let xyz = recon.point3d(pid).xyz;
            recon.point3d_mut(pid).xyz = xyz + Vector3::new(0.02, -0.015, 0.01);
        }

        let mut config = BundleAdjustmentConfig::new();
        for &(i1, i2) in &scene.images_per_frame {
            config.add_image(i1);
            config.add_image(i2);
        }
        for &pid in &point3d_ids {
            config.add_variable_point(pid);
        }
        for &frame_id in &anchors {
            config.set_constant_rig_from_world_pose(frame_id);
        }

        let options = BundleAdjustmentOptions {
            max_num_iterations: 50,
        };
        assert!(solve(&options, &config, &mut recon), "BA solve failed");

        for (j, gt_xyz) in scene.ground_truth_points.iter().enumerate() {
            let err = (recon.point3d(point3d_ids[j]).xyz - gt_xyz).norm();
            assert!(err < 0.01, "point {j} error {err} too large after BA");
        }
        for (&frame_id, gt) in &scene.ground_truth_rig_from_world {
            let got = recon.frame(frame_id).rig_from_world();
            let dt = (got.translation - gt.translation).norm();
            assert!(
                dt < 0.01,
                "frame {frame_id} translation error {dt} too large after BA"
            );
        }
    }
}
