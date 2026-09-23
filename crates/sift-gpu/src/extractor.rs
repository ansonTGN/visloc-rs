//! Host side of the GPU SIFT extractor.

use std::num::NonZeroU64;

use thiserror::Error;
use visloc_gsplat_render::GpuContext;
use visloc_vision::features::sift::{
    GrayImage, SiftConfig, SiftDetector, SiftKeypoint, SiftNormalization,
};

/// Errors from the GPU extractor.
#[derive(Debug, Error)]
pub enum SiftGpuError {
    #[error("SIFT configuration is not supported on the GPU path (use the CPU extractor)")]
    Unsupported,
    #[error("image too small for SIFT: {width}x{height}")]
    ImageTooSmall { width: usize, height: usize },
}

const MAX_OCTAVES: usize = 16;
const KP_COUNTER: usize = 16;
const COUNTER_WORDS: usize = 20;
/// Uniform slot stride (>= `min_uniform_buffer_offset_alignment` everywhere).
const SLOT: u64 = 256;
const SLOT_BIND: u64 = 64;
/// Words per oriented keypoint written by `orientation`.
const KP_WORDS: usize = 8;

struct Pipelines {
    upsample2x: wgpu::ComputePipeline,
    blur_h: wgpu::ComputePipeline,
    blur_v: wgpu::ComputePipeline,
    halve: wgpu::ComputePipeline,
    dog: wgpu::ComputePipeline,
    extrema: wgpu::ComputePipeline,
    orientation_args: wgpu::ComputePipeline,
    orientation: wgpu::ComputePipeline,
    describe: wgpu::ComputePipeline,
}

/// GPU SIFT extractor (owns its [`GpuContext`]).
pub struct SiftGpu {
    ctx: GpuContext,
    pipes: Pipelines,
    /// Candidate capacity per octave as a fraction of its pixel count
    /// (grown automatically when an image overflows it).
    cand_div: usize,
}

/// One detected, oriented keypoint before the host-side ordering/truncation.
#[derive(Clone, Copy)]
struct RawKp {
    octave: u32,
    level: u32,
    x: u32,
    y: u32,
    bin: u32,
    orientation: f32,
    contrast: f32,
}

fn gaussian_kernel(sigma: f64) -> Vec<f32> {
    // Same taps as visloc-vision's legacy `gaussian_kernel` (f64, then f32).
    let radius = (sigma * 3.0).ceil().max(1.0) as i64;
    let denom = 2.0 * sigma * sigma;
    let raw: Vec<f64> = (-radius..=radius)
        .map(|k| (-(k * k) as f64 / denom).exp())
        .collect();
    let sum: f64 = raw.iter().sum();
    raw.iter().map(|v| (v / sum) as f32).collect()
}

fn read_bytes(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    buffer: &wgpu::Buffer,
    len: usize,
) -> Vec<u8> {
    if len == 0 {
        return Vec::new();
    }
    let staging = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("sift-staging"),
        size: len as u64,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("sift-staging-copy"),
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

fn storage(dev: &wgpu::Device, label: &str, bytes: u64, extra: wgpu::BufferUsages) -> wgpu::Buffer {
    dev.create_buffer(&wgpu::BufferDescriptor {
        label: Some(label),
        size: bytes.max(16).div_ceil(4) * 4,
        usage: wgpu::BufferUsages::STORAGE
            | wgpu::BufferUsages::COPY_DST
            | wgpu::BufferUsages::COPY_SRC
            | extra,
        mapped_at_creation: false,
    })
}

/// Uniform parameter slots written once per extraction.
struct Params {
    words: Vec<u32>,
}

impl Params {
    fn push(&mut self, fields: &[u32]) -> u64 {
        let slot = (self.words.len() * 4) as u64;
        let mut s = [0u32; (SLOT / 4) as usize];
        s[..fields.len()].copy_from_slice(fields);
        self.words.extend_from_slice(&s);
        slot
    }
}

fn dispatch_2d(n: u32) -> (u32, u32) {
    if n <= 65535 {
        (n, 1)
    } else {
        (65535, n.div_ceil(65535))
    }
}

impl SiftGpu {
    pub fn new(ctx: GpuContext) -> Self {
        let dev = &ctx.device;
        let module = |label: &str, src: &str| {
            dev.create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some(label),
                source: wgpu::ShaderSource::Wgsl(src.into()),
            })
        };
        let image = module("sift-image", include_str!("shaders/image.wgsl"));
        let detect = module("sift-detect", include_str!("shaders/detect.wgsl"));
        let describe = module("sift-describe", include_str!("shaders/describe.wgsl"));
        let mk = |m: &wgpu::ShaderModule, entry: &str| {
            dev.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some(entry),
                layout: None,
                module: m,
                entry_point: Some(entry),
                compilation_options: Default::default(),
                cache: None,
            })
        };
        let pipes = Pipelines {
            upsample2x: mk(&image, "upsample2x"),
            blur_h: mk(&image, "blur_h"),
            blur_v: mk(&image, "blur_v"),
            halve: mk(&image, "halve"),
            dog: mk(&image, "dog"),
            extrema: mk(&detect, "extrema"),
            orientation_args: mk(&detect, "orientation_args"),
            orientation: mk(&detect, "orientation"),
            describe: mk(&describe, "describe"),
        };
        Self {
            ctx,
            pipes,
            cand_div: 32,
        }
    }

    pub fn context(&self) -> &GpuContext {
        &self.ctx
    }

    pub fn into_context(self) -> GpuContext {
        self.ctx
    }

    /// Whether `config` selects the path this extractor implements.
    pub fn supports(config: &SiftConfig) -> bool {
        config.detector == SiftDetector::Dog
            && !config.affine
            && !config.multi_anisotropy
            && !config.domain_size_pooling
            && !config.scale_adaptive_gradients
            && !config.vlfeat_compatible_descriptor
            && !config.vlfeat_compatible_detector
            && !config.standard_orientation_peaks
    }

    /// Extract keypoints and descriptors; same contract (ordering, early
    /// octave break, `max_keypoints` truncation) as the CPU `extract_sift`.
    pub fn extract(
        &mut self,
        image: &GrayImage<'_>,
        config: &SiftConfig,
    ) -> Result<(Vec<SiftKeypoint>, Vec<Vec<f32>>), SiftGpuError> {
        if !Self::supports(config) {
            return Err(SiftGpuError::Unsupported);
        }
        let raw = loop {
            match self.detect(image, config)? {
                Some(raw) => break raw,
                None => self.cand_div = (self.cand_div / 2).max(1),
            }
        };
        let keypoints = select_keypoints(raw, config);
        let descriptors = self.describe(image, &keypoints, config);
        Ok((keypoints, descriptors))
    }

    /// Scale space + extrema + orientations. `None` when a capacity
    /// overflowed (caller grows and retries).
    fn detect(
        &self,
        image: &GrayImage<'_>,
        config: &SiftConfig,
    ) -> Result<Option<Vec<RawKp>>, SiftGpuError> {
        let dev = &self.ctx.device;
        let queue = &self.ctx.queue;
        let (w0, h0) = (image.width, image.height);
        if w0 < 16 || h0 < 16 || image.pixels.len() < w0 * h0 {
            return Err(SiftGpuError::ImageTooSmall {
                width: w0,
                height: h0,
            });
        }
        let intervals = config.intervals.max(1);
        let auto_octaves = ((w0.min(h0) as f64).log2() - 4.0).max(1.0) as usize;
        let num_octaves = if config.octaves == 0 {
            auto_octaves
        } else {
            config.octaves.min(auto_octaves)
        }
        .min(MAX_OCTAVES);
        let levels = intervals + 3;
        let k = 2.0f64.powf(1.0 / intervals as f64);

        // Octave sizes: doubled input, then integer halving.
        let mut dims = Vec::with_capacity(num_octaves);
        let (mut w, mut h) = (w0 * 2, h0 * 2);
        for _ in 0..num_octaves {
            dims.push((w, h));
            w /= 2;
            h /= 2;
        }

        // Blur kernels: [initial, step 1..=intervals+2].
        let mut weights: Vec<f32> = Vec::new();
        let mut kernels: Vec<(u32, u32)> = Vec::new(); // (offset, radius)
        let mut add_kernel = |sigma: f64, weights: &mut Vec<f32>| {
            let kern = gaussian_kernel(sigma);
            kernels.push((weights.len() as u32, (kern.len() / 2) as u32));
            weights.extend_from_slice(&kern);
        };
        add_kernel(
            (config.sigma_base * config.sigma_base - (2.0 * config.sigma_input).powi(2))
                .max(1e-4)
                .sqrt(),
            &mut weights,
        );
        for j in 1..levels {
            let target = config.sigma_base * k.powi(j as i32);
            let prev = config.sigma_base * k.powi(j as i32 - 1);
            add_kernel(
                ((target * target - prev * prev).max(1e-6)).sqrt(),
                &mut weights,
            );
        }

        let use_ = wgpu::BufferUsages::empty();
        let img_buf = storage(dev, "sift-img", (w0 * h0 * 4) as u64, use_);
        queue.write_buffer(&img_buf, 0, bytemuck::cast_slice(&image.pixels[..w0 * h0]));
        let wt_buf = storage(dev, "sift-weights", (weights.len() * 4) as u64, use_);
        queue.write_buffer(&wt_buf, 0, bytemuck::cast_slice(&weights));
        let (tw, th) = dims[0];
        let tmp = storage(dev, "sift-tmp", (tw * th * 4) as u64, use_);
        let gauss: Vec<wgpu::Buffer> = dims
            .iter()
            .map(|&(w, h)| storage(dev, "sift-gauss", (w * h * levels * 4) as u64, use_))
            .collect();
        let dogs: Vec<wgpu::Buffer> = dims
            .iter()
            .map(|&(w, h)| storage(dev, "sift-dog", (w * h * (levels - 1) * 4) as u64, use_))
            .collect();
        let caps: Vec<usize> = dims
            .iter()
            .map(|&(w, h)| (w * h / self.cand_div).max(4096))
            .collect();
        let cands: Vec<wgpu::Buffer> = caps
            .iter()
            .map(|&c| storage(dev, "sift-cands", (c * 16) as u64, use_))
            .collect();
        let kp_cap: usize = caps.iter().sum::<usize>() * 2;
        let kp_buf = storage(dev, "sift-kps", (kp_cap * KP_WORDS * 4) as u64, use_);
        let counters = storage(dev, "sift-counters", (COUNTER_WORDS * 4) as u64, use_);
        queue.write_buffer(&counters, 0, &[0u8; COUNTER_WORDS * 4]);
        let args = storage(
            dev,
            "sift-args",
            (MAX_OCTAVES * 3 * 4) as u64,
            wgpu::BufferUsages::INDIRECT,
        );

        // Record the per-dispatch parameters, then build bind groups.
        let mut params = Params { words: Vec::new() };
        enum Op {
            Image {
                pipe: usize,
                slot: u64,
                src: (usize, usize),
                dst: (usize, usize),
                groups: (u32, u32, u32),
            },
            Detect {
                pipe: usize,
                slot: u64,
                octave: usize,
            },
        }
        // Buffer ids: 0 = img, 1 = tmp, 2 + o = gauss[o], 2 + n + o = dogs[o].
        let n = num_octaves;
        let gid = |o: usize| 2 + o;
        let did = |o: usize| 2 + n + o;
        let groups = |w: usize, h: usize, z: usize| {
            ((w as u32).div_ceil(16), (h as u32).div_ceil(16), z as u32)
        };
        let mut ops: Vec<Op> = Vec::new();
        let f = |v: f64| (v as f32).to_bits();
        for (o, &(w, h)) in dims.iter().enumerate() {
            let lvl = w * h;
            let (wu, hu) = (w as u32, h as u32);
            if o == 0 {
                // Upsample into gauss level 1 (scratch), blur into level 0.
                let slot = params.push(&[wu, hu, 0, lvl as u32, 0, 0, w0 as u32, h0 as u32]);
                ops.push(Op::Image {
                    pipe: 0,
                    slot,
                    src: (0, 0),
                    dst: (gid(0), 0),
                    groups: groups(w, h, 1),
                });
                let (wo, r) = kernels[0];
                let slot = params.push(&[wu, hu, lvl as u32, 0, r, wo, wu, hu]);
                ops.push(Op::Image {
                    pipe: 1,
                    slot,
                    src: (gid(0), 0),
                    dst: (1, 0),
                    groups: groups(w, h, 1),
                });
                let slot = params.push(&[wu, hu, 0, 0, r, wo, wu, hu]);
                ops.push(Op::Image {
                    pipe: 2,
                    slot,
                    src: (1, 0),
                    dst: (gid(0), 0),
                    groups: groups(w, h, 1),
                });
            } else {
                let (pw, ph) = dims[o - 1];
                let slot = params.push(&[
                    wu,
                    hu,
                    (intervals * pw * ph) as u32,
                    0,
                    0,
                    0,
                    pw as u32,
                    ph as u32,
                ]);
                ops.push(Op::Image {
                    pipe: 3,
                    slot,
                    src: (gid(o - 1), 0),
                    dst: (gid(o), 0),
                    groups: groups(w, h, 1),
                });
            }
            for (j, &(wo, r)) in kernels.iter().enumerate().skip(1) {
                let slot = params.push(&[wu, hu, ((j - 1) * lvl) as u32, 0, r, wo, wu, hu]);
                ops.push(Op::Image {
                    pipe: 1,
                    slot,
                    src: (gid(o), 0),
                    dst: (1, 0),
                    groups: groups(w, h, 1),
                });
                let slot = params.push(&[wu, hu, 0, (j * lvl) as u32, r, wo, wu, hu]);
                ops.push(Op::Image {
                    pipe: 2,
                    slot,
                    src: (1, 0),
                    dst: (gid(o), 0),
                    groups: groups(w, h, 1),
                });
            }
            let slot = params.push(&[wu, hu, 0, 0, 0, 0, wu, hu]);
            ops.push(Op::Image {
                pipe: 4,
                slot,
                src: (gid(o), 0),
                dst: (did(o), 0),
                groups: groups(w, h, levels - 1),
            });
            let slot = params.push(&[
                wu,
                hu,
                o as u32,
                caps[o] as u32,
                f(config.contrast_threshold),
                f(config.edge_threshold),
                f(1.5 * config.sigma_base),
                f(k),
                kp_cap as u32,
                config.max_orientations as u32,
            ]);
            ops.push(Op::Detect {
                pipe: 0,
                slot,
                octave: o,
            });
            ops.push(Op::Detect {
                pipe: 1,
                slot,
                octave: o,
            });
            ops.push(Op::Detect {
                pipe: 2,
                slot,
                octave: o,
            });
        }
        let param_buf = dev.create_buffer(&wgpu::BufferDescriptor {
            label: Some("sift-params"),
            size: (params.words.len() * 4) as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        queue.write_buffer(&param_buf, 0, bytemuck::cast_slice(&params.words));
        let uniform = |slot: u64| {
            wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                buffer: &param_buf,
                offset: slot,
                size: NonZeroU64::new(SLOT_BIND),
            })
        };
        let buf = |id: usize| -> &wgpu::Buffer {
            match id {
                0 => &img_buf,
                1 => &tmp,
                id if id < 2 + n => &gauss[id - 2],
                id => &dogs[id - 2 - n],
            }
        };
        let image_pipes = [
            &self.pipes.upsample2x,
            &self.pipes.blur_h,
            &self.pipes.blur_v,
            &self.pipes.halve,
            &self.pipes.dog,
        ];
        let detect_pipes = [
            &self.pipes.extrema,
            &self.pipes.orientation_args,
            &self.pipes.orientation,
        ];

        let mut encoder = dev.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("sift-detect"),
        });
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("sift-detect"),
                timestamp_writes: None,
            });
            for op in &ops {
                match *op {
                    Op::Image {
                        pipe,
                        slot,
                        src,
                        dst,
                        groups,
                    } => {
                        let pl = image_pipes[pipe];
                        let mut entries = vec![
                            wgpu::BindGroupEntry {
                                binding: 0,
                                resource: uniform(slot),
                            },
                            wgpu::BindGroupEntry {
                                binding: 1,
                                resource: buf(src.0).as_entire_binding(),
                            },
                            wgpu::BindGroupEntry {
                                binding: 2,
                                resource: buf(dst.0).as_entire_binding(),
                            },
                        ];
                        if pipe == 1 || pipe == 2 {
                            entries.push(wgpu::BindGroupEntry {
                                binding: 3,
                                resource: wt_buf.as_entire_binding(),
                            });
                        }
                        let bg = dev.create_bind_group(&wgpu::BindGroupDescriptor {
                            label: Some("sift-image"),
                            layout: &pl.get_bind_group_layout(0),
                            entries: &entries,
                        });
                        pass.set_pipeline(pl);
                        pass.set_bind_group(0, &bg, &[]);
                        pass.dispatch_workgroups(groups.0, groups.1, groups.2);
                    }
                    Op::Detect { pipe, slot, octave } => {
                        let pl = detect_pipes[pipe];
                        let mut entries = vec![wgpu::BindGroupEntry {
                            binding: 0,
                            resource: uniform(slot),
                        }];
                        let bufs: Vec<(u32, &wgpu::Buffer)> = match pipe {
                            0 => vec![(1, &dogs[octave]), (2, &cands[octave]), (3, &counters)],
                            1 => vec![(3, &counters), (5, &args)],
                            _ => vec![
                                (1, &dogs[octave]),
                                (2, &cands[octave]),
                                (3, &counters),
                                (4, &kp_buf),
                            ],
                        };
                        for (binding, b) in bufs {
                            entries.push(wgpu::BindGroupEntry {
                                binding,
                                resource: b.as_entire_binding(),
                            });
                        }
                        let bg = dev.create_bind_group(&wgpu::BindGroupDescriptor {
                            label: Some("sift-detect"),
                            layout: &pl.get_bind_group_layout(0),
                            entries: &entries,
                        });
                        pass.set_pipeline(pl);
                        pass.set_bind_group(0, &bg, &[]);
                        match pipe {
                            0 => {
                                let (w, h) = dims[octave];
                                let g = groups(w, h, intervals);
                                pass.dispatch_workgroups(g.0, g.1, g.2);
                            }
                            1 => pass.dispatch_workgroups(1, 1, 1),
                            _ => pass.dispatch_workgroups_indirect(&args, (octave * 12) as u64),
                        }
                    }
                }
            }
        }
        queue.submit(Some(encoder.finish()));

        let counts: Vec<u32> =
            bytemuck::cast_slice(&read_bytes(dev, queue, &counters, COUNTER_WORDS * 4)).to_vec();
        if (0..n).any(|o| counts[o] as usize > caps[o]) || counts[KP_COUNTER] as usize > kp_cap {
            return Ok(None);
        }
        let nk = counts[KP_COUNTER] as usize;
        let words: Vec<u32> =
            bytemuck::cast_slice(&read_bytes(dev, queue, &kp_buf, nk * KP_WORDS * 4)).to_vec();
        Ok(Some(
            words
                .chunks_exact(KP_WORDS)
                .map(|c| RawKp {
                    x: c[0],
                    y: c[1],
                    octave: c[2] >> 8,
                    level: c[2] & 0xFF,
                    bin: c[3],
                    orientation: f32::from_bits(c[4]),
                    contrast: f32::from_bits(c[5]),
                })
                .collect(),
        ))
    }

    fn describe(
        &self,
        image: &GrayImage<'_>,
        keypoints: &[SiftKeypoint],
        config: &SiftConfig,
    ) -> Vec<Vec<f32>> {
        let nk = keypoints.len();
        if nk == 0 {
            return Vec::new();
        }
        let dev = &self.ctx.device;
        let queue = &self.ctx.queue;
        let (w0, h0) = (image.width, image.height);
        let use_ = wgpu::BufferUsages::empty();
        let img_buf = storage(dev, "sift-img", (w0 * h0 * 4) as u64, use_);
        queue.write_buffer(&img_buf, 0, bytemuck::cast_slice(&image.pixels[..w0 * h0]));
        let kp_data: Vec<f32> = keypoints
            .iter()
            .flat_map(|k| [k.x as f32, k.y as f32, k.sigma as f32, k.orientation as f32])
            .collect();
        let kp_buf = storage(dev, "sift-desc-kps", (kp_data.len() * 4) as u64, use_);
        queue.write_buffer(&kp_buf, 0, bytemuck::cast_slice(&kp_data));
        let out = storage(dev, "sift-desc", (nk * 128 * 4) as u64, use_);
        let norm = match config.normalization {
            SiftNormalization::L2 => 0u32,
            SiftNormalization::L1Root => 1u32,
        };
        let pw: [u32; 8] = [
            nk as u32,
            w0 as u32,
            h0 as u32,
            norm,
            (config.descriptor_magnification as f32).to_bits(),
            0,
            0,
            0,
        ];
        let param_buf = dev.create_buffer(&wgpu::BufferDescriptor {
            label: Some("sift-desc-params"),
            size: 32,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        queue.write_buffer(&param_buf, 0, bytemuck::cast_slice(&pw));
        let pl = &self.pipes.describe;
        let bg = dev.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("sift-describe"),
            layout: &pl.get_bind_group_layout(0),
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: param_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: img_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: kp_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: out.as_entire_binding(),
                },
            ],
        });
        let mut encoder = dev.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("sift-describe"),
        });
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("sift-describe"),
                timestamp_writes: None,
            });
            pass.set_pipeline(pl);
            pass.set_bind_group(0, &bg, &[]);
            let (gx, gy) = dispatch_2d(nk as u32);
            pass.dispatch_workgroups(gx, gy, 1);
        }
        queue.submit(Some(encoder.finish()));
        let floats: Vec<f32> =
            bytemuck::cast_slice(&read_bytes(dev, queue, &out, nk * 128 * 4)).to_vec();
        floats.chunks_exact(128).map(|c| c.to_vec()).collect()
    }
}

/// Map raw detections to original-frame keypoints in the CPU's order, then
/// apply its early octave break and `max_keypoints` truncation.
fn select_keypoints(mut raw: Vec<RawKp>, config: &SiftConfig) -> Vec<SiftKeypoint> {
    raw.sort_by_key(|r| (r.octave, r.level, r.y, r.x, r.bin));
    let max = config.max_keypoints;
    if !config.prefer_larger_scale && !config.full_pyramid && raw.len() > max {
        // The CPU stops after the first octave that fills the budget.
        let mut end = raw.len();
        let mut i = 0;
        while i < raw.len() {
            let oct = raw[i].octave;
            while i < raw.len() && raw[i].octave == oct {
                i += 1;
            }
            if i >= max {
                end = i;
                break;
            }
        }
        raw.truncate(end);
    }
    let intervals = config.intervals.max(1);
    let k = 2.0f64.powf(1.0 / intervals as f64);
    let mut kps: Vec<(SiftKeypoint, f32)> = raw
        .iter()
        .map(|r| {
            let upsample = (1usize << r.octave) as f64 / 2.0;
            let sigma = config.sigma_base * k.powi(r.level as i32) * upsample;
            (
                SiftKeypoint::from_location_scale_orientation(
                    r.x as f64 * upsample,
                    r.y as f64 * upsample,
                    sigma,
                    r.orientation as f64,
                ),
                r.contrast,
            )
        })
        .collect();
    if kps.len() > max {
        if config.prefer_larger_scale {
            kps.sort_by(|a, b| {
                b.0.sigma
                    .total_cmp(&a.0.sigma)
                    .then_with(|| b.1.total_cmp(&a.1))
            });
        } else {
            kps.sort_by(|a, b| b.1.total_cmp(&a.1));
        }
        kps.truncate(max);
    }
    kps.into_iter().map(|(k, _)| k).collect()
}
