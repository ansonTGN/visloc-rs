//! Batched GPU descriptor matching over a device-resident feature bank.
//!
//! [`GpuMatcher::match_pairs`] reproduces `BruteForceMatcher { ratio }`
//! (optionally wrapped in `CrossCheckMatcher`) for many image pairs in one
//! dispatch: the GPU computes per-row top-2 scores, the host applies the
//! ratio test and the cross-check.

use visloc_gsplat_render::GpuContext;
use visloc_vision::matching::DescriptorMatch;

use crate::extractor::{read_bytes, storage};
use crate::SiftGpuError;

/// All descriptors of a set of images, uploaded once.
pub struct FeatureBank {
    desc: wgpu::Buffer,
    norms: wgpu::Buffer,
    offsets: Vec<usize>,
    counts: Vec<usize>,
    dim: usize,
    norm_sq: Vec<f32>,
}

impl FeatureBank {
    /// Upload one descriptor list per image. Every descriptor must have the
    /// same dimension, a multiple of 32 (SIFT 128, SuperPoint 256).
    pub fn upload(ctx: &GpuContext, images: &[&[Vec<f32>]]) -> Result<Self, SiftGpuError> {
        let dim = images
            .iter()
            .flat_map(|d| d.first())
            .map(Vec::len)
            .next()
            .unwrap_or(128);
        if dim == 0 || dim % 32 != 0 {
            return Err(SiftGpuError::Unsupported);
        }
        let mut offsets = Vec::with_capacity(images.len());
        let mut counts = Vec::with_capacity(images.len());
        let mut flat: Vec<f32> = Vec::new();
        let mut norm_sq: Vec<f32> = Vec::new();
        for d in images {
            offsets.push(norm_sq.len());
            counts.push(d.len());
            for row in d.iter() {
                if row.len() != dim {
                    return Err(SiftGpuError::Unsupported);
                }
                flat.extend_from_slice(row);
                norm_sq.push(row.iter().map(|x| x * x).sum());
            }
        }
        let dev = &ctx.device;
        let use_ = wgpu::BufferUsages::empty();
        let desc = storage(dev, "bank-desc", (flat.len() * 4) as u64, use_);
        ctx.queue
            .write_buffer(&desc, 0, bytemuck::cast_slice(&flat));
        let norms = storage(dev, "bank-norms", (norm_sq.len() * 4) as u64, use_);
        ctx.queue
            .write_buffer(&norms, 0, bytemuck::cast_slice(&norm_sq));
        Ok(Self {
            desc,
            norms,
            offsets,
            counts,
            dim,
            norm_sq,
        })
    }

    pub fn num_images(&self) -> usize {
        self.counts.len()
    }
}

/// Top-2 result for one query row.
#[derive(Clone, Copy)]
struct Top2 {
    best: u32,
    s1: f32,
    s2: f32,
}

pub struct GpuMatcher {
    pipeline: wgpu::ComputePipeline,
}

/// Rows of output per dispatch batch (16 B each).
const MAX_BATCH_ROWS: usize = 1 << 20;

impl GpuMatcher {
    pub fn new(ctx: &GpuContext) -> Self {
        let module = ctx
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("match"),
                source: wgpu::ShaderSource::Wgsl(include_str!("shaders/match.wgsl").into()),
            });
        let pipeline = ctx
            .device
            .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some("top2"),
                layout: None,
                module: &module,
                entry_point: Some("top2"),
                compilation_options: Default::default(),
                cache: None,
            });
        Self { pipeline }
    }

    /// Match image `i` (query) against image `j` (train) for every `(i, j)`
    /// in `pairs`; same result as `BruteForceMatcher { ratio }` and, with
    /// `cross_check`, `CrossCheckMatcher::new(BruteForceMatcher { ratio })`
    /// (up to f32 summation order in the dot products).
    pub fn match_pairs(
        &self,
        ctx: &GpuContext,
        bank: &FeatureBank,
        pairs: &[(usize, usize)],
        ratio: Option<f32>,
        cross_check: bool,
    ) -> Vec<Vec<DescriptorMatch>> {
        // Directed entries: forward (i -> j), then reverse (j -> i).
        let mut directed: Vec<(usize, usize)> = pairs.to_vec();
        if cross_check {
            directed.extend(pairs.iter().map(|&(i, j)| (j, i)));
        }
        let tops = self.top2(ctx, bank, &directed);
        let np = pairs.len();
        (0..np)
            .map(|p| {
                let (i, j) = pairs[p];
                let forward = self.finish(bank, i, j, &tops[p], ratio);
                if !cross_check || forward.is_empty() {
                    return forward;
                }
                let reverse = self.finish(bank, j, i, &tops[np + p], ratio);
                let mut back = vec![usize::MAX; bank.counts[j]];
                for m in &reverse {
                    back[m.query_index] = m.train_index;
                }
                forward
                    .into_iter()
                    .filter(|m| back[m.train_index] == m.query_index)
                    .collect()
            })
            .collect()
    }

    fn finish(
        &self,
        bank: &FeatureBank,
        qi: usize,
        ti: usize,
        tops: &[Top2],
        ratio: Option<f32>,
    ) -> Vec<DescriptorMatch> {
        let nt = bank.counts[ti];
        if nt == 0 {
            return Vec::new();
        }
        let qn = &bank.norm_sq[bank.offsets[qi]..bank.offsets[qi] + bank.counts[qi]];
        let mut out = Vec::new();
        for (query_index, t) in tops.iter().enumerate() {
            let distance = (qn[query_index] + t.s1).max(0.0).sqrt();
            let second = (nt >= 2).then(|| (qn[query_index] + t.s2).max(0.0).sqrt());
            if let (Some(r), Some(s)) = (ratio, second) {
                if distance >= r * s {
                    continue;
                }
            }
            out.push(DescriptorMatch {
                query_index,
                train_index: t.best as usize,
                distance,
                second_best_distance: second,
                ratio: second.map(|s| distance / s),
                confidence: None,
            });
        }
        out
    }

    /// Per directed entry, the top-2 of every query row.
    fn top2(
        &self,
        ctx: &GpuContext,
        bank: &FeatureBank,
        directed: &[(usize, usize)],
    ) -> Vec<Vec<Top2>> {
        let mut result: Vec<Vec<Top2>> = Vec::with_capacity(directed.len());
        let mut start = 0;
        while start < directed.len() {
            // Batch so the output stays bounded and entries fit one dispatch.
            let mut end = start;
            let mut rows = 0usize;
            while end < directed.len()
                && end - start < 65535
                && (rows == 0 || rows + bank.counts[directed[end].0] <= MAX_BATCH_ROWS)
            {
                rows += bank.counts[directed[end].0];
                end += 1;
            }
            result.extend(self.top2_batch(ctx, bank, &directed[start..end], rows));
            start = end;
        }
        result
    }

    fn top2_batch(
        &self,
        ctx: &GpuContext,
        bank: &FeatureBank,
        directed: &[(usize, usize)],
        rows: usize,
    ) -> Vec<Vec<Top2>> {
        let dev = &ctx.device;
        let queue = &ctx.queue;
        let mut words: Vec<u32> = Vec::with_capacity(directed.len() * 8);
        let mut out_offs = Vec::with_capacity(directed.len());
        let mut out_off = 0usize;
        let mut max_q = 0usize;
        for &(qi, ti) in directed {
            let nq = bank.counts[qi];
            words.extend_from_slice(&[
                bank.offsets[qi] as u32,
                nq as u32,
                bank.offsets[ti] as u32,
                bank.counts[ti] as u32,
                out_off as u32,
                0,
                0,
                0,
            ]);
            out_offs.push(out_off);
            out_off += nq;
            max_q = max_q.max(nq);
        }
        if rows == 0 {
            return directed.iter().map(|_| Vec::new()).collect();
        }
        let use_ = wgpu::BufferUsages::empty();
        let entries = storage(dev, "match-entries", (words.len() * 4) as u64, use_);
        queue.write_buffer(&entries, 0, bytemuck::cast_slice(&words));
        let out = storage(dev, "match-out", (rows * 16) as u64, use_);
        let params = dev.create_buffer(&wgpu::BufferDescriptor {
            label: Some("match-params"),
            size: 16,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let pw: [u32; 4] = [(bank.dim / 4) as u32, directed.len() as u32, 0, 0];
        queue.write_buffer(&params, 0, bytemuck::cast_slice(&pw));
        let bg = dev.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("match"),
            layout: &self.pipeline.get_bind_group_layout(0),
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: params.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: bank.desc.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: bank.norms.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: entries.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 4,
                    resource: out.as_entire_binding(),
                },
            ],
        });
        let mut encoder = dev.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("match"),
        });
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("match"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(0, &bg, &[]);
            pass.dispatch_workgroups(max_q.div_ceil(64) as u32, directed.len() as u32, 1);
        }
        queue.submit(Some(encoder.finish()));
        let raw: Vec<u32> = bytemuck::cast_slice(&read_bytes(dev, queue, &out, rows * 16)).to_vec();
        directed
            .iter()
            .zip(&out_offs)
            .map(|(&(qi, _), &off)| {
                (0..bank.counts[qi])
                    .map(|r| {
                        let w = &raw[(off + r) * 4..(off + r) * 4 + 4];
                        Top2 {
                            best: w[0],
                            s1: f32::from_bits(w[1]),
                            s2: f32::from_bits(w[2]),
                        }
                    })
                    .collect()
            })
            .collect()
    }
}
