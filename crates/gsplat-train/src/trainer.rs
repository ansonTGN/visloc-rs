//! GPU trainer (M2): render -> loss gradient -> backward -> Adam, all on the
//! device, with no per-step image transfer.
//!
//! This first version optimises a fixed set of gaussians (no densification)
//! with an L1 loss and per-group Adam learning rates (Inria defaults; the
//! mean's rate is scaled by the scene extent and decays exponentially).

use visloc_gsplat_core::camera::CameraView;
use visloc_gsplat_core::cpu_render::Image;
use visloc_gsplat_core::gaussian::{Gaussian, Scene};
use visloc_gsplat_render::{GpuContext, GpuError, Renderer};

use crate::dataset::{load_view_rgb, Dataset, DatasetError, View};

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

/// The trainer. Owns the renderer (and so the scene on the GPU).
pub struct Trainer {
    renderer: Renderer,
    cfg: TrainConfig,
    views: Vec<View>,
    width: u32,
    height: u32,
    extent: f32,
    loss_pipeline: wgpu::ComputePipeline,
    loss_binds: Vec<wgpu::BindGroup>,
    loss_acc: wgpu::Buffer,
    loss_steps: usize,
    adam_pipeline: wgpu::ComputePipeline,
    adam: [AdamGroup; 3],
    step: usize,
    order: Vec<usize>,
    rng: u64,
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

        let mut renderer = Renderer::new(ctx, init, width, height)?;
        renderer.set_skip_readback(true);
        let npix = (width * height) as u64;

        let dev = renderer.ctx.device.clone();
        let queue = renderer.ctx.queue.clone();

        // Loss: one bind group per training view (its ground-truth buffer).
        let loss_pipeline = pipeline(
            &dev,
            "loss_l1",
            include_str!("shaders/loss_l1.wgsl"),
            "loss_l1",
        );
        let loss_uniforms = uniform(&dev, "loss_uniforms", 16);
        queue.write_buffer(
            &loss_uniforms,
            0,
            bytemuck::cast_slice(&[npix as u32, 0u32, 0, 0]),
        );
        let loss_acc = storage(&dev, "loss_acc", 4);
        let out_img = renderer.output_buffer().clone();
        let d_image = renderer.d_image_buffer()?.clone();
        let mut loss_binds = Vec::with_capacity(views.len());
        for v in &views {
            let gt = storage(&dev, "gt", npix * 4);
            queue.write_buffer(
                &gt,
                0,
                bytemuck::cast_slice(&pack_rgba8(&load_view_rgb(v)?)),
            );
            loss_binds.push(bind(
                &dev,
                &loss_pipeline,
                "loss",
                &[&loss_uniforms, &out_img, &gt, &d_image, &loss_acc],
            ));
        }

        // Adam: moments per parameter buffer.
        let adam_pipeline = pipeline(&dev, "adam", include_str!("shaders/adam.wgsl"), "adam");
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
        let n = init.len() as u32;
        let cpc2 = (init.sh_degree + 1) * (init.sh_degree + 1);
        let mk_group = |label: &str, p: &wgpu::Buffer, g: &wgpu::Buffer, len: u32, stride, a, b| {
            let m1 = storage(&dev, label, len as u64 * 4);
            let m2 = storage(&dev, label, len as u64 * 4);
            let u = uniform(&dev, label, std::mem::size_of::<AdamUniforms>() as u64);
            let bg = bind(&dev, &adam_pipeline, label, &[&u, p, g, &m1, &m2]);
            AdamGroup {
                uniforms: u,
                bind: bg,
                n: len,
                stride,
                split_a: a,
                split_b: b,
            }
        };
        let adam = [
            // mean | quat | log-scale
            mk_group("adam_transforms", &pt, &gt, n * 10, 10, 3, 7),
            mk_group("adam_opacity", &po, &go, n, 1, 1, 1),
            // DC | rest
            mk_group("adam_sh", &ps, &gs, n * 3 * cpc2, 3 * cpc2, 3, 3 * cpc2),
        ];

        let order: Vec<usize> = (0..views.len()).collect();
        Ok(Self {
            renderer,
            views,
            width,
            height,
            extent,
            loss_pipeline,
            loss_binds,
            loss_acc,
            loss_steps: 0,
            adam_pipeline,
            adam,
            step: 0,
            order,
            rng: cfg.seed.max(1),
            cfg,
        })
    }

    /// Steps taken so far.
    pub fn steps_done(&self) -> usize {
        self.step
    }

    /// Scene extent used to scale the mean learning rate.
    pub fn extent(&self) -> f32 {
        self.extent
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
        let _ = self.renderer.render(&view, self.cfg.background);

        let dev = self.renderer.ctx.device.clone();
        let queue = self.renderer.ctx.queue.clone();
        let npix = self.width * self.height;
        {
            let mut enc = dev.create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("loss"),
            });
            {
                let mut pass = enc.begin_compute_pass(&wgpu::ComputePassDescriptor::default());
                pass.set_pipeline(&self.loss_pipeline);
                pass.set_bind_group(0, &self.loss_binds[vi], &[]);
                let (x, y) = groups_2d(npix);
                pass.dispatch_workgroups(x, y, 1);
            }
            queue.submit(Some(enc.finish()));
        }
        self.loss_steps += 1;

        self.renderer.backward_on_device()?;

        // Adam with bias correction; exponential mean-LR decay.
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
        let mut enc = dev.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("adam"),
        });
        {
            let mut pass = enc.begin_compute_pass(&wgpu::ComputePassDescriptor::default());
            pass.set_pipeline(&self.adam_pipeline);
            for (g, (a, b, c)) in self.adam.iter().zip(lrs) {
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
        Ok(())
    }

    /// Mean L1 loss over the steps since the last call (reads the device).
    pub fn take_mean_loss(&mut self) -> f32 {
        let dev = &self.renderer.ctx.device;
        let queue = &self.renderer.ctx.queue;
        let staging = dev.create_buffer(&wgpu::BufferDescriptor {
            label: Some("loss_read"),
            size: 4,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let mut enc = dev.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        enc.copy_buffer_to_buffer(&self.loss_acc, 0, &staging, 0, 4);
        enc.clear_buffer(&self.loss_acc, 0, None);
        queue.submit(Some(enc.finish()));
        let slice = staging.slice(..);
        let (tx, rx) = std::sync::mpsc::channel();
        slice.map_async(wgpu::MapMode::Read, move |r| {
            let _ = tx.send(r);
        });
        dev.poll(wgpu::PollType::wait_indefinitely()).ok();
        let _ = rx.recv();
        let v = {
            let data = slice.get_mapped_range().expect("map");
            u32::from_le_bytes([data[0], data[1], data[2], data[3]])
        };
        staging.unmap();
        let steps = self.loss_steps.max(1);
        self.loss_steps = 0;
        v as f32 * 1e-6 / steps as f32
    }

    /// Render a view (with image readback), e.g. for evaluation.
    pub fn render(&mut self, view: &CameraView) -> Image {
        self.renderer.set_skip_readback(false);
        let img = self.renderer.render(view, self.cfg.background);
        self.renderer.set_skip_readback(true);
        img
    }

    /// Download the current gaussians as a [`Scene`].
    pub fn scene(&self) -> Scene {
        let (t, o, sh) = self.renderer.read_params();
        let degree = self.renderer.scene.packed.sh_degree;
        let cpc2 = ((degree + 1) * (degree + 1)) as usize;
        let n = o.len();
        let gaussians = (0..n)
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
}
