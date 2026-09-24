//! Host side of the GPU bundle adjuster: problem upload, the LM loop and
//! dispatch of the linearization / PCG kernels.

use std::collections::BTreeMap;

use nalgebra::{Point3, Vector6};
use thiserror::Error;
use visloc_core::geometry::{Pose, SE3};
use visloc_core::types::CameraModel;
use visloc_gsplat_render::GpuContext;
use visloc_slam::{
    BaAccelerator, BaConfig, BaError, BaIterationStats, BaResult, BaScope, BundleAdjustment,
    RobustKernel,
};

/// Why a problem cannot run on the GPU path.
#[derive(Debug, Error)]
pub enum GpuBaError {
    #[error("unsupported bundle-adjustment problem: {0}")]
    Unsupported(&'static str),
}

/// PCG controls for each LM linear solve.
#[derive(Debug, Clone, Copy)]
pub struct GpuBaSettings {
    pub max_pcg_iterations: u32,
    /// Stop when ||r|| <= tol * ||b|| (f32 arithmetic: keep >= ~1e-6).
    pub pcg_relative_tolerance: f32,
    /// Also take the SfM's local BA windows (`BaScope::Local`). Off by
    /// default: those systems are small (~1e4 observations) and on a
    /// GTX 1660 Ti the GPU path is bound by submit/sync latency (~5 ms per
    /// round trip), so it only ties the CPU solver there.
    pub local_windows: bool,
}

impl Default for GpuBaSettings {
    fn default() -> Self {
        fn env_u<T: std::str::FromStr>(k: &str) -> Option<T> {
            std::env::var(k).ok().and_then(|v| v.parse().ok())
        }
        Self {
            max_pcg_iterations: env_u("VISLOC_BA_GPU_PCG_ITERS").unwrap_or(100),
            pcg_relative_tolerance: std::env::var("VISLOC_BA_GPU_PCG_TOL")
                .ok()
                .and_then(|v| v.parse().ok())
                .unwrap_or(1e-4),
            local_windows: std::env::var_os("VISLOC_BA_GPU_LOCAL").is_some(),
        }
    }
}

const ENTRIES: [&str; 10] = [
    "linearize",
    "pose_reduce",
    "lm_reduce",
    "lm_prep",
    "pose_prep",
    "pcg_init",
    "apply_lm",
    "apply_pose",
    "pcg_step",
    "backsub",
];
const LINEARIZE: usize = 0;
const POSE_REDUCE: usize = 1;
const LM_REDUCE: usize = 2;
const LM_PREP: usize = 3;
const POSE_PREP: usize = 4;
const PCG_INIT: usize = 5;
const APPLY_LM: usize = 6;
const APPLY_POSE: usize = 7;
const PCG_STEP: usize = 8;
const BACKSUB: usize = 9;
/// Bindings 1-4 and 8-11 are read-only storage; 0 is the uniform.
const READ_ONLY: [u32; 8] = [1, 2, 3, 4, 8, 9, 10, 11];
const NUM_BINDINGS: u32 = 24;
const NONE: u32 = u32::MAX;

/// GPU bundle adjuster (owns its [`GpuContext`]).
pub struct GpuBundleAdjuster {
    ctx: GpuContext,
    layout: wgpu::BindGroupLayout,
    pipes: Vec<wgpu::ComputePipeline>,
    pub settings: GpuBaSettings,
}

fn read_bytes(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    buffer: &wgpu::Buffer,
    len: usize,
) -> Vec<u8> {
    let staging = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("ba-staging"),
        size: len as u64,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("ba-staging-copy"),
    });
    encoder.copy_buffer_to_buffer(buffer, 0, &staging, 0, len as u64);
    queue.submit(Some(encoder.finish()));
    let slice = staging.slice(..);
    let (tx, rx) = std::sync::mpsc::channel();
    slice.map_async(wgpu::MapMode::Read, move |r| {
        let _ = tx.send(r);
    });
    device.poll(wgpu::PollType::wait_indefinitely()).ok();
    let _ = rx.recv();
    let data = slice.get_mapped_range().expect("map range").to_vec();
    staging.unmap();
    data
}

fn storage(dev: &wgpu::Device, label: &str, bytes: usize) -> wgpu::Buffer {
    dev.create_buffer(&wgpu::BufferDescriptor {
        label: Some(label),
        size: (bytes.max(16) as u64).div_ceil(4) * 4,
        usage: wgpu::BufferUsages::STORAGE
            | wgpu::BufferUsages::COPY_DST
            | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    })
}

/// 1D thread count -> (x, y) workgroups of 256 (y > 1 past 65535 groups).
fn groups_256(n: usize) -> (u32, u32) {
    let g = n.div_ceil(256).max(1) as u32;
    if g <= 65535 {
        (g, 1)
    } else {
        (65535, g.div_ceil(65535))
    }
}

/// Everything uploaded once per `optimize` call.
struct Problem {
    pose_ids: Vec<u64>,
    lm_ids: Vec<u64>,
    /// Variable pose slot -> index into `pose_ids`.
    var_poses: Vec<usize>,
    var_lms: Vec<usize>,
    n_obs: usize,
    bufs: Vec<wgpu::Buffer>,
    bind: wgpu::BindGroup,
    params: wgpu::Buffer,
}

impl GpuBundleAdjuster {
    pub fn new(ctx: GpuContext) -> Self {
        let dev = &ctx.device;
        let module = dev.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("ba"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders/ba.wgsl").into()),
        });
        let entries: Vec<wgpu::BindGroupLayoutEntry> = (0..NUM_BINDINGS)
            .map(|b| wgpu::BindGroupLayoutEntry {
                binding: b,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: if b == 0 {
                        wgpu::BufferBindingType::Uniform
                    } else {
                        wgpu::BufferBindingType::Storage {
                            read_only: READ_ONLY.contains(&b),
                        }
                    },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            })
            .collect();
        let layout = dev.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("ba"),
            entries: &entries,
        });
        let pl = dev.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("ba"),
            bind_group_layouts: &[Some(&layout)],
            immediate_size: 0,
        });
        let pipes = ENTRIES
            .iter()
            .map(|e| {
                dev.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                    label: Some(e),
                    layout: Some(&pl),
                    module: &module,
                    entry_point: Some(e),
                    compilation_options: Default::default(),
                    cache: None,
                })
            })
            .collect();
        Self {
            ctx,
            layout,
            pipes,
            settings: GpuBaSettings::default(),
        }
    }

    pub fn context(&self) -> &GpuContext {
        &self.ctx
    }

    /// Whether `ba`/`config` is the monocular pinhole problem this solver
    /// implements (no stereo/rig/inertial/prior terms, no intrinsics or
    /// rotation-only constraints, Huber or plain least squares).
    pub fn check(ba: &BundleAdjustment, config: &BaConfig) -> Result<(), GpuBaError> {
        let unsupported = |why| Err(GpuBaError::Unsupported(why));
        if config.refine_intrinsics || config.refine_distortion {
            return unsupported("intrinsics refinement");
        }
        if !matches!(
            config.robust_kernel,
            RobustKernel::None | RobustKernel::Huber { .. }
        ) {
            return unsupported("robust kernel");
        }
        if ba.camera.model != CameraModel::Pinhole || ba.camera.params.len() != 4 {
            return unsupported("camera model");
        }
        if !ba.stereo_observations.is_empty()
            || !ba.general_stereo_observations.is_empty()
            || !ba.rig_observations.is_empty()
        {
            return unsupported("stereo/rig observations");
        }
        if ba.gravity_prior.is_some()
            || ba.per_pose_gravity_prior.is_some()
            || ba.position_prior.is_some()
            || !ba.pairwise_pose_factors.is_empty()
            || !ba.velocities.is_empty()
            || !ba.imu_factors.is_empty()
            || !ba.biases.is_empty()
            || !ba.bias_random_walk_factors.is_empty()
            || ba.navigation_state_prior.is_some()
        {
            return unsupported("prior/inertial terms");
        }
        if !ba.fixed_pose_rotations.is_empty() {
            return unsupported("rotation-fixed poses");
        }
        if ba.fixed_poses.is_empty() {
            return unsupported("no fixed pose (gauge)");
        }
        Ok(())
    }

    fn upload(&self, ba: &BundleAdjustment) -> Problem {
        let dev = &self.ctx.device;
        let queue = &self.ctx.queue;
        let pose_ids: Vec<u64> = ba.poses.keys().copied().collect();
        let lm_ids: Vec<u64> = ba.landmarks.keys().copied().collect();
        let pose_at: BTreeMap<u64, usize> = pose_ids
            .iter()
            .enumerate()
            .map(|(i, &id)| (id, i))
            .collect();
        let lm_at: BTreeMap<u64, usize> =
            lm_ids.iter().enumerate().map(|(i, &id)| (id, i)).collect();
        let mut pose_slot = vec![NONE; pose_ids.len()];
        let mut var_poses = Vec::new();
        for (i, id) in pose_ids.iter().enumerate() {
            if !ba.fixed_poses.contains(id) {
                pose_slot[i] = var_poses.len() as u32;
                var_poses.push(i);
            }
        }
        let mut lm_slot = vec![NONE; lm_ids.len()];
        let mut var_lms = Vec::new();
        for (i, id) in lm_ids.iter().enumerate() {
            if !ba.fixed_landmarks.contains(id) {
                lm_slot[i] = var_lms.len() as u32;
                var_lms.push(i);
            }
        }
        let mut obs: Vec<[u32; 4]> = Vec::with_capacity(ba.observations.len());
        let mut xy: Vec<[f32; 2]> = Vec::with_capacity(ba.observations.len());
        for o in &ba.observations {
            let (Some(&pi), Some(&li)) = (pose_at.get(&o.keyframe_id), lm_at.get(&o.landmark_id))
            else {
                continue;
            };
            obs.push([pi as u32, li as u32, pose_slot[pi], lm_slot[li]]);
            xy.push([o.xy.x as f32, o.xy.y as f32]);
        }
        let csr = |slot_of: &dyn Fn(&[u32; 4]) -> u32, n: usize| {
            let mut counts = vec![0u32; n + 1];
            for o in &obs {
                let s = slot_of(o);
                if s != NONE {
                    counts[s as usize + 1] += 1;
                }
            }
            for i in 0..n {
                counts[i + 1] += counts[i];
            }
            let mut fill = counts.clone();
            let mut list = vec![0u32; counts[n] as usize];
            for (oi, o) in obs.iter().enumerate() {
                let s = slot_of(o);
                if s != NONE {
                    list[fill[s as usize] as usize] = oi as u32;
                    fill[s as usize] += 1;
                }
            }
            (counts, list)
        };
        let (pose_off, pose_obs) = csr(&|o| o[2], var_poses.len());
        let (lm_off, lm_obs) = csr(&|o| o[3], var_lms.len());

        let (np, nl, no) = (var_poses.len(), var_lms.len(), obs.len());
        let f = 4usize;
        let sizes: [(&str, usize); NUM_BINDINGS as usize] = [
            ("params", 0),
            ("poses", pose_ids.len() * 12 * f),
            ("points", lm_ids.len() * 16),
            ("obs", no * 16),
            ("obs_xy", no * 8),
            ("obs_w", no * 18 * f),
            ("obs_hpp", no * 27 * f),
            ("obs_hll", no * 9 * f),
            ("pose_off", (np + 1) * f),
            ("pose_obs", pose_obs.len() * f),
            ("lm_off", (nl + 1) * f),
            ("lm_obs", lm_obs.len() * f),
            ("pose_h", np * 42 * f),
            ("lm_h", nl * 9 * f),
            ("lm_inv", nl * 12 * f),
            ("minv", np * 36 * f),
            ("vx", np * 6 * f),
            ("vr", np * 6 * f),
            ("vd", np * 6 * f),
            ("vq", np * 6 * f),
            ("vz", np * 6 * f),
            ("lm_u", nl * 3 * f),
            ("scal", 8 * f),
            ("dl", nl * 3 * f),
        ];
        let params = dev.create_buffer(&wgpu::BufferDescriptor {
            label: Some("ba-params"),
            size: 48,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let bufs: Vec<wgpu::Buffer> = sizes
            .iter()
            .map(|&(label, bytes)| storage(dev, label, bytes))
            .collect();
        queue.write_buffer(&bufs[3], 0, bytemuck::cast_slice(&obs));
        queue.write_buffer(&bufs[4], 0, bytemuck::cast_slice(&xy));
        queue.write_buffer(&bufs[8], 0, bytemuck::cast_slice(&pose_off));
        queue.write_buffer(&bufs[9], 0, bytemuck::cast_slice(&pose_obs));
        queue.write_buffer(&bufs[10], 0, bytemuck::cast_slice(&lm_off));
        queue.write_buffer(&bufs[11], 0, bytemuck::cast_slice(&lm_obs));
        let entries: Vec<wgpu::BindGroupEntry> = (0..NUM_BINDINGS as usize)
            .map(|b| wgpu::BindGroupEntry {
                binding: b as u32,
                resource: if b == 0 {
                    params.as_entire_binding()
                } else {
                    bufs[b].as_entire_binding()
                },
            })
            .collect();
        let bind = dev.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("ba"),
            layout: &self.layout,
            entries: &entries,
        });
        Problem {
            pose_ids,
            lm_ids,
            var_poses,
            var_lms,
            n_obs: no,
            bufs,
            bind,
            params,
        }
    }

    fn write_state(&self, prob: &Problem, ba: &BundleAdjustment) {
        let mut poses: Vec<f32> = Vec::with_capacity(prob.pose_ids.len() * 12);
        for id in &prob.pose_ids {
            let t = &ba.poses[id].world_to_camera;
            let r = t.rotation.to_rotation_matrix();
            let m = r.matrix();
            for i in 0..3 {
                for j in 0..3 {
                    poses.push(m[(i, j)] as f32);
                }
            }
            poses.extend([
                t.translation.x as f32,
                t.translation.y as f32,
                t.translation.z as f32,
            ]);
        }
        let points: Vec<[f32; 4]> = prob
            .lm_ids
            .iter()
            .map(|id| {
                let p = ba.landmarks[id];
                [p.x as f32, p.y as f32, p.z as f32, 0.0]
            })
            .collect();
        self.ctx
            .queue
            .write_buffer(&prob.bufs[1], 0, bytemuck::cast_slice(&poses));
        self.ctx
            .queue
            .write_buffer(&prob.bufs[2], 0, bytemuck::cast_slice(&points));
    }

    fn write_params(&self, prob: &Problem, ba: &BundleAdjustment, lambda: f64, huber: f64) {
        let c = &ba.camera.params;
        let words: [u32; 12] = [
            prob.n_obs as u32,
            prob.var_poses.len() as u32,
            prob.var_lms.len() as u32,
            self.settings.max_pcg_iterations,
            (lambda as f32).to_bits(),
            (c[0] as f32).to_bits(),
            (c[1] as f32).to_bits(),
            (c[2] as f32).to_bits(),
            (c[3] as f32).to_bits(),
            (huber as f32).to_bits(),
            self.settings.pcg_relative_tolerance.to_bits(),
            0,
        ];
        self.ctx
            .queue
            .write_buffer(&prob.params, 0, bytemuck::cast_slice(&words));
    }

    fn run(&self, prob: &Problem, stages: &[usize], pcg_loops: u32) {
        let dev = &self.ctx.device;
        let mut encoder =
            dev.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("ba") });
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("ba"),
                timestamp_writes: None,
            });
            pass.set_bind_group(0, &prob.bind, &[]);
            let np = prob.var_poses.len() as u32;
            let dispatch = |pass: &mut wgpu::ComputePass<'_>, k: usize| {
                pass.set_pipeline(&self.pipes[k]);
                match k {
                    LINEARIZE => {
                        let (x, y) = groups_256(prob.n_obs);
                        pass.dispatch_workgroups(x, y, 1);
                    }
                    LM_REDUCE | LM_PREP | APPLY_LM | BACKSUB => {
                        let (x, y) = groups_256(prob.var_lms.len());
                        pass.dispatch_workgroups(x, y, 1);
                    }
                    POSE_REDUCE | POSE_PREP | APPLY_POSE => pass.dispatch_workgroups(np, 1, 1),
                    _ => pass.dispatch_workgroups(1, 1, 1),
                }
            };
            for &k in stages {
                if k == PCG_STEP {
                    for _ in 0..pcg_loops {
                        dispatch(&mut pass, APPLY_LM);
                        dispatch(&mut pass, APPLY_POSE);
                        dispatch(&mut pass, PCG_STEP);
                    }
                } else {
                    dispatch(&mut pass, k);
                }
            }
        }
        self.ctx.queue.submit(Some(encoder.finish()));
    }

    /// Solve the damped system at the current linearization.
    /// Returns (pose deltas 6 per variable pose, landmark deltas 3 per
    /// variable landmark, PCG iterations).
    fn solve(&self, prob: &Problem) -> Option<(Vec<f32>, Vec<f32>, u32)> {
        let np = prob.var_poses.len();
        let nl = prob.var_lms.len();
        let dev = &self.ctx.device;
        let queue = &self.ctx.queue;
        let read_scal = || -> Vec<f32> {
            bytemuck::cast_slice(&read_bytes(dev, queue, &prob.bufs[22], 32)).to_vec()
        };
        if np > 0 {
            // PCG in chunks: most solves converge in a handful of
            // iterations, so check the device-side done flag between chunks
            // instead of always dispatching the full iteration budget.
            let chunk: u32 = std::env::var("VISLOC_BA_GPU_PCG_CHUNK")
                .ok()
                .and_then(|v| v.parse().ok())
                .unwrap_or(10);
            let max = self.settings.max_pcg_iterations;
            let mut issued = chunk.min(max);
            self.run(prob, &[LM_PREP, POSE_PREP, PCG_INIT, PCG_STEP], issued);
            while issued < max && read_scal()[2] == 0.0 {
                let n = chunk.min(max - issued);
                self.run(prob, &[PCG_STEP], n);
                issued += n;
            }
            self.run(prob, &[BACKSUB], 0);
        } else {
            self.run(prob, &[LM_PREP, BACKSUB], 0);
        }
        // One staging copy + map for all three outputs.
        let parts = [(16usize, np * 6), (23, nl * 3), (22, 8)];
        let total: usize = parts.iter().map(|p| p.1 * 4).sum();
        let staging = dev.create_buffer(&wgpu::BufferDescriptor {
            label: Some("ba-out-staging"),
            size: total as u64,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let mut encoder = dev.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("ba-out-copy"),
        });
        let mut off = 0u64;
        for &(b, n) in &parts {
            if n > 0 {
                encoder.copy_buffer_to_buffer(&prob.bufs[b], 0, &staging, off, (n * 4) as u64);
            }
            off += (n * 4) as u64;
        }
        queue.submit(Some(encoder.finish()));
        let slice = staging.slice(..);
        let (tx, rx) = std::sync::mpsc::channel();
        slice.map_async(wgpu::MapMode::Read, move |r| {
            let _ = tx.send(r);
        });
        dev.poll(wgpu::PollType::wait_indefinitely()).ok();
        let _ = rx.recv();
        let all: Vec<f32> =
            bytemuck::cast_slice(&slice.get_mapped_range().expect("map range")).to_vec();
        staging.unmap();
        let x = all[..np * 6].to_vec();
        let dl = all[np * 6..np * 6 + nl * 3].to_vec();
        let scal = all[np * 6 + nl * 3..].to_vec();
        let iters = scal.get(3).copied().unwrap_or(0.0) as u32;
        let finite = x.iter().chain(&dl).all(|v| v.is_finite());
        finite.then_some((x, dl, iters))
    }

    /// Levenberg-Marquardt with the CPU optimizer's schedule and gates.
    pub fn optimize(
        &self,
        ba: &mut BundleAdjustment,
        config: &BaConfig,
    ) -> Result<BaResult, GpuBaError> {
        Self::check(ba, config)?;
        let huber = match config.robust_kernel {
            RobustKernel::Huber { delta } => delta,
            _ => 0.0,
        };
        let kernel = config.robust_kernel;
        let prob = self.upload(ba);
        let (initial_cost, initial_nonprojectable) = cost_and_nonprojectable(ba, &kernel);
        let mut current_cost = initial_cost;
        let mut current_nonprojectable = initial_nonprojectable;
        let mut lambda = config.initial_lambda.unwrap_or(0.0);
        let mut iterations = Vec::new();
        let mut converged = false;
        let mut linearized = false;
        let profile = std::env::var_os("VISLOC_BA_GPU_PROFILE").is_some();
        for iteration in 0..config.max_iterations {
            let t0 = std::time::Instant::now();
            let mut t_lin = 0.0;
            if !linearized {
                self.write_state(&prob, ba);
                self.write_params(&prob, ba, lambda, huber);
                self.run(&prob, &[LINEARIZE, POSE_REDUCE, LM_REDUCE], 0);
                if profile {
                    self.ctx
                        .device
                        .poll(wgpu::PollType::wait_indefinitely())
                        .ok();
                    t_lin = t0.elapsed().as_secs_f64();
                }
                linearized = true;
            }
            self.write_params(&prob, ba, lambda, huber);
            let solved = self.solve(&prob);
            let t_solve = t0.elapsed().as_secs_f64();
            let cost_before = current_cost;
            let saved_poses = ba.poses.clone();
            let saved_landmarks = ba.landmarks.clone();
            let (mut max_pose_step, mut max_landmark_step) = (0.0f64, 0.0f64);
            let mut pcg_iters = 0;
            let (cost_after, nonprojectable_after) = match &solved {
                Some((x, dl, it)) => {
                    pcg_iters = *it;
                    for (slot, &pi) in prob.var_poses.iter().enumerate() {
                        let xi = Vector6::from_iterator((0..6).map(|k| x[slot * 6 + k] as f64));
                        max_pose_step = max_pose_step.max(xi.norm());
                        let pose = ba.poses.get_mut(&prob.pose_ids[pi]).expect("pose");
                        *pose = Pose {
                            world_to_camera: pose.world_to_camera.compose(&SE3::exp(&xi)),
                        };
                    }
                    for (slot, &li) in prob.var_lms.iter().enumerate() {
                        let d = nalgebra::Vector3::new(
                            dl[slot * 3] as f64,
                            dl[slot * 3 + 1] as f64,
                            dl[slot * 3 + 2] as f64,
                        );
                        max_landmark_step = max_landmark_step.max(d.norm());
                        let p: &mut Point3<f64> =
                            ba.landmarks.get_mut(&prob.lm_ids[li]).expect("landmark");
                        *p += d;
                    }
                    cost_and_nonprojectable(ba, &kernel)
                }
                None => (f64::INFINITY, usize::MAX),
            };
            if profile {
                eprintln!(
                    "ba-gpu: iteration={iteration} lambda={lambda:.3e} linearize={t_lin:.4}s solve={:.4}s pcg={pcg_iters} total={:.4}s cost {cost_before:.6e} -> {cost_after:.6e}",
                    t_solve,
                    t0.elapsed().as_secs_f64()
                );
            }
            let cost_accepted = match config.initial_lambda {
                None => solved.is_some(),
                Some(_) => cost_after < cost_before,
            };
            let step_accepted = cost_accepted && nonprojectable_after <= current_nonprojectable;
            if !step_accepted {
                ba.poses = saved_poses;
                ba.landmarks = saved_landmarks;
                lambda = (lambda * config.lambda_increase_factor).min(config.max_lambda);
                iterations.push(BaIterationStats {
                    iteration,
                    cost_before,
                    cost_after,
                    max_pose_step,
                    max_landmark_step,
                    lambda,
                    step_accepted: false,
                });
                if config.initial_lambda.is_none() || lambda >= config.max_lambda {
                    break;
                }
                continue;
            }
            iterations.push(BaIterationStats {
                iteration,
                cost_before,
                cost_after,
                max_pose_step,
                max_landmark_step,
                lambda,
                step_accepted: true,
            });
            current_cost = cost_after;
            current_nonprojectable = nonprojectable_after;
            linearized = false;
            if config.initial_lambda.is_some() {
                lambda = (lambda * config.lambda_decrease_factor).max(config.min_lambda);
            }
            if max_pose_step < config.step_tolerance && max_landmark_step < config.step_tolerance {
                converged = true;
                break;
            }
            if (cost_before - cost_after).abs() < config.cost_tolerance {
                converged = true;
                break;
            }
            if config.relative_cost_tolerance.is_some_and(|tolerance| {
                tolerance.is_finite()
                    && tolerance >= 0.0
                    && (cost_before - cost_after) / cost_before.abs().max(f64::EPSILON) < tolerance
            }) {
                converged = true;
                break;
            }
        }
        Ok(BaResult {
            initial_cost,
            final_cost: current_cost,
            iterations,
            converged,
        })
    }
}

/// Robust cost (as `BundleAdjustment::robust_cost`) and the number of
/// observations whose point is behind the camera or does not project (the
/// CPU optimizer's feasibility count), in one parallel f64 pass.
fn cost_and_nonprojectable(ba: &BundleAdjustment, kernel: &RobustKernel) -> (f64, usize) {
    use rayon::prelude::*;
    ba.observations
        .par_chunks(4096)
        .map(|chunk| {
            let mut cost = 0.0;
            let mut bad = 0usize;
            for o in chunk {
                let (Some(pose), Some(point)) = (
                    ba.poses.get(&o.keyframe_id),
                    ba.landmarks.get(&o.landmark_id),
                ) else {
                    bad += 1;
                    continue;
                };
                let xc = pose.transform_world_point(point);
                if xc.z <= 0.0 {
                    bad += 1;
                    continue;
                }
                match ba.camera.project(&xc) {
                    Some(pred) => {
                        let r = pred - o.xy;
                        cost += kernel.cost(r.x * r.x + r.y * r.y);
                    }
                    None => bad += 1,
                }
            }
            (cost, bad)
        })
        .reduce(|| (0.0, 0), |a, b| (a.0 + b.0, a.1 + b.1))
}

impl BaAccelerator for GpuBundleAdjuster {
    fn optimize(
        &self,
        ba: &mut BundleAdjustment,
        config: &BaConfig,
        scope: BaScope,
    ) -> Option<Result<BaResult, BaError>> {
        if scope == BaScope::Local && !self.settings.local_windows {
            return None;
        }
        Self::check(ba, config).ok()?;
        GpuBundleAdjuster::optimize(self, ba, config).ok().map(Ok)
    }
}
