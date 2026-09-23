//! GPU trainer: render -> loss gradient -> backward -> Adam, all on the
//! device, with no per-step image transfer, plus Inria-style adaptive density
//! control every `densify.interval` steps (on the host, see [`crate::densify`]).
//!
//! Loss is `(1 - ssim_weight) * L1 + ssim_weight * (1 - SSIM)` (the Inria
//! loss); per-group Adam learning rates follow the Inria defaults (the mean's
//! rate is scaled by the scene extent and decays exponentially).

use visloc_gsplat_core::camera::CameraView;
use visloc_gsplat_core::cpu_render::Image;
use visloc_gsplat_core::gaussian::{Gaussian, Scene};
use visloc_gsplat_render::{GpuContext, GpuError, Renderer};

use crate::dataset::{load_view_rgb, Dataset, DatasetError, View};
use crate::densify::{densify, reset_opacity, DensifyConfig, DensifyReport, Group, Population};
use crate::loss::{SsimBinds, SsimKernels};

/// Training hyper-parameters.
#[derive(Debug, Clone)]
pub struct TrainConfig {
    pub steps: usize,
    /// Mean learning rate at the start / end, times the scene extent.
    pub lr_mean: f32,
    pub lr_mean_final: f32,
    pub lr_quat: f32,
    pub lr_scale: f32,
    pub lr_opacity: f32,
    pub lr_sh_dc: f32,
    pub lr_sh_rest: f32,
    pub background: [f32; 3],
    pub seed: u64,
    /// Weight of the D-SSIM term (0 = pure L1).
    pub ssim_weight: f32,
    /// `None` keeps the initial gaussians fixed in number.
    pub densify: Option<DensifyConfig>,
}

impl Default for TrainConfig {
    fn default() -> Self {
        Self {
            steps: 7000,
            lr_mean: 1.6e-4,
            lr_mean_final: 1.6e-6,
            lr_quat: 1e-3,
            lr_scale: 5e-3,
            lr_opacity: 5e-2,
            lr_sh_dc: 2.5e-3,
            lr_sh_rest: 2.5e-3 / 20.0,
            background: [0.0; 3],
            seed: 42,
            ssim_weight: 0.2,
            densify: Some(DensifyConfig::default()),
        }
    }
}

/// Errors from the trainer.
#[derive(Debug, thiserror::Error)]
pub enum TrainError {
    #[error(transparent)]
    Gpu(#[from] GpuError),
    #[error(transparent)]
    Dataset(#[from] DatasetError),
    #[error("dataset has no training views")]
    NoViews,
    #[error("all views must share one resolution ({0}x{1} vs {2}x{3})")]
    MixedResolution(u32, u32, u32, u32),
}

struct AdamGroup {
    uniforms: wgpu::Buffer,
    bind: wgpu::BindGroup,
    m1: wgpu::Buffer,
    m2: wgpu::Buffer,
    n: u32,
    stride: u32,
    split_a: u32,
    split_b: u32,
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct AdamUniforms {
    n: u32,
    stride: u32,
    split_a: u32,
    split_b: u32,
    lr_a: f32,
    lr_b: f32,
    lr_c: f32,
    beta1: f32,
    beta2: f32,
    eps: f32,
    bc1: f32,
    bc2: f32,
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct StatsUniforms {
    num_visible: u32,
    half_w: f32,
    half_h: f32,
    pad0: u32,
}

/// Everything sized by the gaussian count; rebuilt after densification.
struct SceneState {
    renderer: Renderer,
    loss_binds: Vec<wgpu::BindGroup>,
    ssim_binds: Vec<SsimBinds>,
    adam: [AdamGroup; 3],
    grad_accum: wgpu::Buffer,
    grad_count: wgpu::Buffer,
    stats_bind: wgpu::BindGroup,
}

/// The trainer. Owns the renderer (and so the scene on the GPU).
pub struct Trainer {
    state: Option<SceneState>,
    cfg: TrainConfig,
    views: Vec<View>,
    width: u32,
    height: u32,
    extent: f32,
    sh_degree: u32,
    gt: Vec<wgpu::Buffer>,
    loss_pipeline: wgpu::ComputePipeline,
    loss_uniforms: wgpu::Buffer,
    loss_acc: wgpu::Buffer,
    loss_steps: usize,
    ssim: SsimKernels,
    adam_pipeline: wgpu::ComputePipeline,
    stats_pipeline: wgpu::ComputePipeline,
    stats_uniforms: wgpu::Buffer,
    step: usize,
    order: Vec<usize>,
    rng: u64,
    last_densify: Option<DensifyReport>,
}

fn storage(dev: &wgpu::Device, label: &str, bytes: u64) -> wgpu::Buffer {
    dev.create_buffer(&wgpu::BufferDescriptor {
        label: Some(label),
        size: bytes.max(4),
        usage: wgpu::BufferUsages::STORAGE
            | wgpu::BufferUsages::COPY_DST
            | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    })
}

fn uniform(dev: &wgpu::Device, label: &str, bytes: u64) -> wgpu::Buffer {
    dev.create_buffer(&wgpu::BufferDescriptor {
        label: Some(label),
        size: bytes,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    })
}

/// A compute pipeline with an auto-derived layout for `entry` in `source`.
fn pipeline(dev: &wgpu::Device, label: &str, source: &str, entry: &str) -> wgpu::ComputePipeline {
    let module = dev.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some(label),
        source: wgpu::ShaderSource::Wgsl(source.into()),
    });
    dev.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some(label),
        layout: None,
        module: &module,
        entry_point: Some(entry),
        compilation_options: Default::default(),
        cache: None,
    })
}

fn bind(
    dev: &wgpu::Device,
    pipeline: &wgpu::ComputePipeline,
    label: &str,
    buffers: &[&wgpu::Buffer],
) -> wgpu::BindGroup {
    let entries: Vec<wgpu::BindGroupEntry> = buffers
        .iter()
        .enumerate()
        .map(|(i, b)| wgpu::BindGroupEntry {
            binding: i as u32,
            resource: b.as_entire_binding(),
        })
        .collect();
    dev.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some(label),
        layout: &pipeline.get_bind_group_layout(0),
        entries: &entries,
    })
}

/// 2D workgroup count for `threads` 256-wide invocations (see the kernels).
fn groups_2d(threads: u32) -> (u32, u32) {
    let g = threads.div_ceil(256).max(1);
    let x = g.min(65535);
    (x, g.div_ceil(x))
}

fn pack_rgba8(rgb: &[[f32; 3]]) -> Vec<u32> {
    rgb.iter()
        .map(|p| {
            let c = |v: f32| (v.clamp(0.0, 1.0) * 255.0).round() as u32;
            c(p[0]) | (c(p[1]) << 8) | (c(p[2]) << 16) | (255 << 24)
        })
        .collect()
}

/// Blocking read of `len` f32 values from a device buffer.
fn read_f32(dev: &wgpu::Device, queue: &wgpu::Queue, buf: &wgpu::Buffer, len: usize) -> Vec<f32> {
    if len == 0 {
        return Vec::new();
    }
    let bytes = (len * 4) as u64;
    let staging = dev.create_buffer(&wgpu::BufferDescriptor {
        label: Some("read_f32"),
        size: bytes,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let mut enc = dev.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
    enc.copy_buffer_to_buffer(buf, 0, &staging, 0, bytes);
    queue.submit(Some(enc.finish()));
    let slice = staging.slice(..);
    let (tx, rx) = std::sync::mpsc::channel();
    slice.map_async(wgpu::MapMode::Read, move |r| {
        let _ = tx.send(r);
    });
    dev.poll(wgpu::PollType::wait_indefinitely()).ok();
    let _ = rx.recv();
    let out = bytemuck::cast_slice(&slice.get_mapped_range().expect("map")).to_vec();
    staging.unmap();
    out
}

/// Gaussians from a population (the device parameter layouts).
fn population_to_scene(pop: &Population, degree: u32) -> Scene {
    let cpc2 = ((degree + 1) * (degree + 1)) as usize;
    let (t, o, sh) = (&pop.transforms.values, &pop.opacity.values, &pop.sh.values);
    let gaussians = (0..o.len())
        .map(|i| {
            let r = &t[i * 10..i * 10 + 10];
            let s = &sh[i * 3 * cpc2..(i + 1) * 3 * cpc2];
            Gaussian {
                mean: nalgebra::Vector3::new(r[0], r[1], r[2]),
                rotation: nalgebra::Quaternion::new(r[3], r[4], r[5], r[6]),
                scale_log: nalgebra::Vector3::new(r[7], r[8], r[9]),
                opacity_logit: o[i],
                sh_dc: [s[0], s[1], s[2]],
                sh_rest: s[3..].to_vec(),
                sh_degree: degree,
            }
        })
        .collect();
    Scene::new(gaussians, degree)
}

impl Trainer {
    /// Upload `init` and the dataset's training images and build the kernels.
    pub fn new(
        ctx: GpuContext,
        dataset: &Dataset,
        init: &Scene,
        cfg: TrainConfig,
    ) -> Result<Self, TrainError> {
        let views = dataset.train.clone();
        let first = views.first().ok_or(TrainError::NoViews)?;
        let (width, height) = (first.camera.camera.width, first.camera.camera.height);
        for v in dataset.train.iter().chain(&dataset.eval) {
            let (w, h) = (v.camera.camera.width, v.camera.camera.height);
            if (w, h) != (width, height) {
                return Err(TrainError::MixedResolution(width, height, w, h));
            }
        }
        // Scene extent: 1.1x the largest camera distance from their centroid.
        let centers: Vec<_> = views.iter().map(|v| v.camera.camera_center()).collect();
        let centroid = centers
            .iter()
            .fold(nalgebra::Vector3::zeros(), |a, c| a + c)
            / centers.len() as f32;
        let extent = 1.1
            * centers
                .iter()
                .map(|c| (c - centroid).norm())
                .fold(0.0f32, f32::max)
                .max(1e-3);

        let dev = ctx.device.clone();
        let queue = ctx.queue.clone();
        let npix = (width * height) as u64;
        let mut gt = Vec::with_capacity(views.len());
        for v in &views {
            let b = storage(&dev, "gt", npix * 4);
            queue.write_buffer(&b, 0, bytemuck::cast_slice(&pack_rgba8(&load_view_rgb(v)?)));
            gt.push(b);
        }
        let loss_pipeline = pipeline(
            &dev,
            "loss_l1",
            include_str!("shaders/loss_l1.wgsl"),
            "loss_l1",
        );
        let loss_uniforms = uniform(&dev, "loss_uniforms", 16);
        let l1_weight = 1.0 - cfg.ssim_weight;
        queue.write_buffer(
            &loss_uniforms,
            0,
            bytemuck::cast_slice(&[npix as u32, l1_weight.to_bits(), 0, 0]),
        );
        let loss_acc = storage(&dev, "loss_acc", 4);
        let ssim = SsimKernels::new(&dev, width, height);
        ssim.set_weight(&queue, cfg.ssim_weight);
        let adam_pipeline = pipeline(&dev, "adam", include_str!("shaders/adam.wgsl"), "adam");
        let stats_pipeline = pipeline(
            &dev,
            "densify_stats",
            include_str!("shaders/densify_stats.wgsl"),
            "densify_stats",
        );
        let stats_uniforms = uniform(&dev, "stats_uniforms", 16);

        let mut trainer = Self {
            state: None,
            views,
            width,
            height,
            extent,
            sh_degree: init.sh_degree,
            gt,
            loss_pipeline,
            loss_uniforms,
            loss_acc,
            loss_steps: 0,
            ssim,
            adam_pipeline,
            stats_pipeline,
            stats_uniforms,
            step: 0,
            order: Vec::new(),
            rng: cfg.seed.max(1),
            cfg,
            last_densify: None,
        };
        trainer.order = (0..trainer.views.len()).collect();
        trainer.build_state(ctx, init, None)?;
        Ok(trainer)
    }

    /// (Re)build everything sized by the gaussian count. `moments` carries
    /// Adam state across densification (zeros when `None`).
    fn build_state(
        &mut self,
        ctx: GpuContext,
        scene: &Scene,
        moments: Option<&Population>,
    ) -> Result<(), TrainError> {
        let mut renderer = Renderer::new(ctx, scene, self.width, self.height)?;
        renderer.set_skip_readback(true);
        let dev = renderer.ctx.device.clone();
        let queue = renderer.ctx.queue.clone();

        let out_img = renderer.output_buffer().clone();
        let d_image = renderer.d_image_buffer()?.clone();
        let loss_binds = self
            .gt
            .iter()
            .map(|g| {
                bind(
                    &dev,
                    &self.loss_pipeline,
                    "loss",
                    &[&self.loss_uniforms, &out_img, g, &d_image, &self.loss_acc],
                )
            })
            .collect();
        let ssim_binds = self
            .gt
            .iter()
            .map(|g| self.ssim.bind(&dev, &out_img, g, &d_image))
            .collect();

        let params = renderer.param_buffers();
        let (pt, po, ps) = (
            params.transforms.clone(),
            params.opacity.clone(),
            params.sh.clone(),
        );
        let grads = renderer.grad_buffers()?;
        let (gt, go, gs) = (
            grads.transforms.clone(),
            grads.opacity.clone(),
            grads.sh.clone(),
        );
        let n = scene.len() as u32;
        let cpc2 = (scene.sh_degree + 1) * (scene.sh_degree + 1);
        let mk_group = |label: &str,
                        p: &wgpu::Buffer,
                        g: &wgpu::Buffer,
                        stride: u32,
                        a: u32,
                        b: u32,
                        init: Option<&Group>| {
            let len = n * stride;
            let m1 = storage(&dev, label, len as u64 * 4);
            let m2 = storage(&dev, label, len as u64 * 4);
            if let Some(src) = init {
                queue.write_buffer(&m1, 0, bytemuck::cast_slice(&src.m1));
                queue.write_buffer(&m2, 0, bytemuck::cast_slice(&src.m2));
            }
            let u = uniform(&dev, label, std::mem::size_of::<AdamUniforms>() as u64);
            let bg = bind(&dev, &self.adam_pipeline, label, &[&u, p, g, &m1, &m2]);
            AdamGroup {
                uniforms: u,
                bind: bg,
                m1,
                m2,
                n: len,
                stride,
                split_a: a,
                split_b: b,
            }
        };
        let adam = [
            // mean | quat | log-scale
            mk_group("adam_t", &pt, &gt, 10, 3, 7, moments.map(|m| &m.transforms)),
            mk_group("adam_o", &po, &go, 1, 1, 1, moments.map(|m| &m.opacity)),
            // DC | rest
            mk_group(
                "adam_sh",
                &ps,
                &gs,
                3 * cpc2,
                3,
                3 * cpc2,
                moments.map(|m| &m.sh),
            ),
        ];

        let grad_accum = storage(&dev, "grad_accum", n as u64 * 4);
        let grad_count = storage(&dev, "grad_count", n as u64 * 4);
        let (gfc, screen, _) = renderer.screen_grad_buffers()?;
        let stats_bind = bind(
            &dev,
            &self.stats_pipeline,
            "stats",
            &[&self.stats_uniforms, gfc, screen, &grad_accum, &grad_count],
        );
        self.state = Some(SceneState {
            renderer,
            loss_binds,
            ssim_binds,
            adam,
            grad_accum,
            grad_count,
            stats_bind,
        });
        Ok(())
    }

    fn st(&self) -> &SceneState {
        self.state.as_ref().expect("trainer state")
    }

    /// Steps taken so far.
    pub fn steps_done(&self) -> usize {
        self.step
    }

    /// Current number of gaussians.
    pub fn num_gaussians(&self) -> usize {
        self.st().renderer.num_gaussians()
    }

    /// Scene extent used to scale the mean learning rate.
    pub fn extent(&self) -> f32 {
        self.extent
    }

    /// The report of the densification run by the last [`Trainer::step`], if any.
    pub fn take_densify_report(&mut self) -> Option<DensifyReport> {
        self.last_densify.take()
    }

    fn next_view(&mut self) -> usize {
        // Reshuffle every epoch (xorshift, deterministic from the seed).
        let k = self.step % self.order.len();
        if k == 0 {
            for i in (1..self.order.len()).rev() {
                self.rng ^= self.rng << 13;
                self.rng ^= self.rng >> 7;
                self.rng ^= self.rng << 17;
                let j = (self.rng % (i as u64 + 1)) as usize;
                self.order.swap(i, j);
            }
        }
        self.order[k]
    }

    /// One optimisation step on the next training view.
    pub fn step(&mut self) -> Result<(), TrainError> {
        let vi = self.next_view();
        let view = self.views[vi].camera;
        let bg = self.cfg.background;
        let densifying = self
            .cfg
            .densify
            .as_ref()
            .is_some_and(|d| self.step < d.stop);
        let npix = self.width * self.height;
        let (half_w, half_h) = (self.width as f32 * 0.5, self.height as f32 * 0.5);

        let st = self.state.as_mut().expect("trainer state");
        let _ = st.renderer.render(&view, bg);
        let dev = st.renderer.ctx.device.clone();
        let queue = st.renderer.ctx.queue.clone();
        {
            let mut enc = dev.create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("loss"),
            });
            {
                let mut pass = enc.begin_compute_pass(&wgpu::ComputePassDescriptor::default());
                pass.set_pipeline(&self.loss_pipeline);
                pass.set_bind_group(0, &st.loss_binds[vi], &[]);
                let (x, y) = groups_2d(npix);
                pass.dispatch_workgroups(x, y, 1);
                if self.cfg.ssim_weight > 0.0 {
                    self.ssim.encode(&mut pass, &st.ssim_binds[vi]);
                }
            }
            queue.submit(Some(enc.finish()));
        }
        self.loss_steps += 1;

        st.renderer.backward_on_device()?;

        // Densification statistics, then Adam (bias-corrected, mean-LR decay).
        let t = (self.step + 1) as f32;
        let (beta1, beta2) = (0.9f32, 0.999f32);
        let frac = (self.step as f32 / self.cfg.steps.max(1) as f32).min(1.0);
        let lr_mean = (self.cfg.lr_mean.ln() * (1.0 - frac) + self.cfg.lr_mean_final.ln() * frac)
            .exp()
            * self.extent;
        let lrs = [
            (lr_mean, self.cfg.lr_quat, self.cfg.lr_scale),
            (
                self.cfg.lr_opacity,
                self.cfg.lr_opacity,
                self.cfg.lr_opacity,
            ),
            (self.cfg.lr_sh_dc, self.cfg.lr_sh_rest, self.cfg.lr_sh_rest),
        ];
        let nv = st.renderer.screen_grad_buffers()?.2 as u32;
        let mut enc = dev.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("adam"),
        });
        {
            let mut pass = enc.begin_compute_pass(&wgpu::ComputePassDescriptor::default());
            if densifying && nv > 0 {
                let su = StatsUniforms {
                    num_visible: nv,
                    half_w,
                    half_h,
                    pad0: 0,
                };
                queue.write_buffer(&self.stats_uniforms, 0, bytemuck::bytes_of(&su));
                pass.set_pipeline(&self.stats_pipeline);
                pass.set_bind_group(0, &st.stats_bind, &[]);
                pass.dispatch_workgroups(nv.div_ceil(256), 1, 1);
            }
            pass.set_pipeline(&self.adam_pipeline);
            for (g, (a, b, c)) in st.adam.iter().zip(lrs) {
                let u = AdamUniforms {
                    n: g.n,
                    stride: g.stride,
                    split_a: g.split_a,
                    split_b: g.split_b,
                    lr_a: a,
                    lr_b: b,
                    lr_c: c,
                    beta1,
                    beta2,
                    eps: 1e-15,
                    bc1: 1.0 - beta1.powf(t),
                    bc2: 1.0 - beta2.powf(t),
                };
                queue.write_buffer(&g.uniforms, 0, bytemuck::bytes_of(&u));
                pass.set_bind_group(0, &g.bind, &[]);
                let (x, y) = groups_2d(g.n);
                pass.dispatch_workgroups(x, y, 1);
            }
        }
        queue.submit(Some(enc.finish()));
        self.step += 1;

        if let Some(d) = self.cfg.densify.clone() {
            let s = self.step;
            let at_densify = s >= d.start && s <= d.stop && s % d.interval == 0;
            let at_reset = s < d.stop && s % d.opacity_reset_interval == 0;
            if at_densify || at_reset {
                self.densify_now(&d, at_densify, at_reset)?;
            }
        }
        Ok(())
    }

    /// Download the population with its Adam moments.
    fn download(&self) -> Population {
        let st = self.st();
        let r = &st.renderer;
        let (dev, queue) = (&r.ctx.device, &r.ctx.queue);
        let (t, o, sh) = r.read_params();
        let group = |values: Vec<f32>, g: &AdamGroup| Group {
            stride: g.stride as usize,
            m1: read_f32(dev, queue, &g.m1, values.len()),
            m2: read_f32(dev, queue, &g.m2, values.len()),
            values,
        };
        Population {
            transforms: group(t, &st.adam[0]),
            opacity: group(o, &st.adam[1]),
            sh: group(sh, &st.adam[2]),
        }
    }

    fn densify_now(
        &mut self,
        d: &DensifyConfig,
        grow: bool,
        reset: bool,
    ) -> Result<(), TrainError> {
        let mut pop = self.download();
        let n = pop.len();
        if grow {
            let st = self.st();
            let (dev, queue) = (&st.renderer.ctx.device, &st.renderer.ctx.queue);
            let accum = read_f32(dev, queue, &st.grad_accum, n);
            let count = read_f32(dev, queue, &st.grad_count, n);
            let prune_large = self.step > d.opacity_reset_interval;
            let seed = self.cfg.seed ^ (self.step as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15);
            let (next, report) = densify(&pop, &accum, &count, self.extent, d, prune_large, seed);
            pop = next;
            self.last_densify = Some(report);
        }
        if reset {
            reset_opacity(&mut pop, 0.01);
        }
        let scene = population_to_scene(&pop, self.sh_degree);
        let ctx = self
            .state
            .take()
            .expect("trainer state")
            .renderer
            .into_context();
        self.build_state(ctx, &scene, Some(&pop))
    }

    /// Mean L1 loss over the steps since the last call (reads the device).
    pub fn take_mean_loss(&mut self) -> f32 {
        let st = self.st();
        let (dev, queue) = (&st.renderer.ctx.device, &st.renderer.ctx.queue);
        let v = read_f32(dev, queue, &self.loss_acc, 1);
        let bits = v.first().map(|x| x.to_bits()).unwrap_or(0);
        let mut enc = dev.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        enc.clear_buffer(&self.loss_acc, 0, None);
        queue.submit(Some(enc.finish()));
        let steps = self.loss_steps.max(1);
        self.loss_steps = 0;
        bits as f32 * 1e-6 / steps as f32
    }

    /// Render a view (with image readback), e.g. for evaluation.
    pub fn render(&mut self, view: &CameraView) -> Image {
        let bg = self.cfg.background;
        let r = &mut self.state.as_mut().expect("trainer state").renderer;
        r.set_skip_readback(false);
        let img = r.render(view, bg);
        r.set_skip_readback(true);
        img
    }

    /// Download the current gaussians as a [`Scene`].
    pub fn scene(&self) -> Scene {
        let (t, o, sh) = self.st().renderer.read_params();
        let group = |stride: usize, values: Vec<f32>| Group {
            stride,
            m1: Vec::new(),
            m2: Vec::new(),
            values,
        };
        let cpc2 = ((self.sh_degree + 1) * (self.sh_degree + 1)) as usize;
        population_to_scene(
            &Population {
                transforms: group(10, t),
                opacity: group(1, o),
                sh: group(3 * cpc2, sh),
            },
            self.sh_degree,
        )
    }
}
