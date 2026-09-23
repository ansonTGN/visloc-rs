//! Device-side D-SSIM loss term (tiled, see `shaders/ssim.wgsl`), shared by the
//! trainer and the GPU-vs-CPU gradient test.

/// The two tiled SSIM kernels plus their scratch buffers for one image size.
pub struct SsimKernels {
    width: u32,
    height: u32,
    fwd: wgpu::ComputePipeline,
    bwd: wgpu::ComputePipeline,
    uniforms: wgpu::Buffer,
    abc: wgpu::Buffer,
    /// Sum of SSIM over pixels and channels (fixed point, 1e-3 units).
    pub acc: wgpu::Buffer,
}

/// Bind groups for one (render, ground truth, d_image) triple.
pub struct SsimBinds {
    fwd: wgpu::BindGroup,
    bwd: wgpu::BindGroup,
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

fn bind_at(
    dev: &wgpu::Device,
    pipeline: &wgpu::ComputePipeline,
    entries: &[(u32, &wgpu::Buffer)],
) -> wgpu::BindGroup {
    let entries: Vec<wgpu::BindGroupEntry> = entries
        .iter()
        .map(|(binding, b)| wgpu::BindGroupEntry {
            binding: *binding,
            resource: b.as_entire_binding(),
        })
        .collect();
    dev.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("ssim"),
        layout: &pipeline.get_bind_group_layout(0),
        entries: &entries,
    })
}

impl SsimKernels {
    pub fn new(dev: &wgpu::Device, width: u32, height: u32) -> Self {
        let module = dev.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("ssim"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders/ssim.wgsl").into()),
        });
        let mk = |entry: &str| {
            dev.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some(entry),
                layout: None,
                module: &module,
                entry_point: Some(entry),
                compilation_options: Default::default(),
                cache: None,
            })
        };
        let npix = width as u64 * height as u64;
        Self {
            width,
            height,
            fwd: mk("ssim_fwd"),
            bwd: mk("ssim_bwd"),
            uniforms: dev.create_buffer(&wgpu::BufferDescriptor {
                label: Some("ssim_uniforms"),
                size: 16,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }),
            abc: storage(dev, "ssim_abc", npix * 9 * 4),
            acc: storage(dev, "ssim_acc", 4),
        }
    }

    /// Set the loss weight `lambda` of the `lambda * (1 - mean SSIM)` term.
    pub fn set_weight(&self, queue: &wgpu::Queue, lambda: f32) {
        let scale = lambda / (3.0 * self.width as f32 * self.height as f32);
        let words: [u32; 4] = [self.width, self.height, scale.to_bits(), 0];
        queue.write_buffer(&self.uniforms, 0, bytemuck::cast_slice(&words));
    }

    /// Bind a render (RGB f32), a ground truth (packed RGBA8) and the
    /// `dL/dC` buffer the gradient is added to.
    pub fn bind(
        &self,
        dev: &wgpu::Device,
        render: &wgpu::Buffer,
        gt: &wgpu::Buffer,
        d_image: &wgpu::Buffer,
    ) -> SsimBinds {
        let u = &self.uniforms;
        SsimBinds {
            fwd: bind_at(
                dev,
                &self.fwd,
                &[(0, u), (1, render), (2, gt), (4, &self.abc), (7, &self.acc)],
            ),
            bwd: bind_at(
                dev,
                &self.bwd,
                &[(0, u), (1, render), (2, gt), (4, &self.abc), (6, d_image)],
            ),
        }
    }

    /// Record the two passes (wgpu orders dispatches within a pass).
    pub fn encode(&self, pass: &mut wgpu::ComputePass<'_>, binds: &SsimBinds) {
        let tiles = self.width.div_ceil(16) * self.height.div_ceil(16);
        for (p, b) in [(&self.fwd, &binds.fwd), (&self.bwd, &binds.bwd)] {
            pass.set_pipeline(p);
            pass.set_bind_group(0, b, &[]);
            pass.dispatch_workgroups(tiles, 1, 1);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::metrics::ssim_with_grad;

    #[test]
    fn gpu_ssim_gradient_matches_cpu() {
        let Some(ctx) = visloc_gsplat_render::try_context() else {
            eprintln!("skipping: no GPU adapter");
            return;
        };
        let (w, h) = (37u32, 29u32);
        let n = (w * h) as usize;
        let mut s = 0x1234_5678_9abc_def0u64;
        let mut rnd = move || {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            (s >> 40) as f32 / (1u64 << 24) as f32
        };
        let render: Vec<[f32; 3]> = (0..n).map(|_| [rnd(), rnd(), rnd()]).collect();
        // Quantise the ground truth to 8 bits, as the device stores it.
        let gt8: Vec<[u8; 3]> = (0..n)
            .map(|_| {
                [
                    (rnd() * 255.0) as u8,
                    (rnd() * 255.0) as u8,
                    (rnd() * 255.0) as u8,
                ]
            })
            .collect();
        let packed: Vec<u32> = gt8
            .iter()
            .map(|p| p[0] as u32 | (p[1] as u32) << 8 | (p[2] as u32) << 16 | 255 << 24)
            .collect();
        let dev = &ctx.device;
        let queue = &ctx.queue;
        let rb = storage(dev, "render", n as u64 * 12);
        queue.write_buffer(&rb, 0, bytemuck::cast_slice(&render));
        let gb = storage(dev, "gt", n as u64 * 4);
        queue.write_buffer(&gb, 0, bytemuck::cast_slice(&packed));
        let db = storage(dev, "d_image", n as u64 * 12);
        let k = SsimKernels::new(dev, w, h);
        let lambda = 0.2;
        k.set_weight(queue, lambda);
        let binds = k.bind(dev, &rb, &gb, &db);
        let mut enc = dev.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        {
            let mut pass = enc.begin_compute_pass(&wgpu::ComputePassDescriptor::default());
            k.encode(&mut pass, &binds);
        }
        queue.submit(Some(enc.finish()));
        let staging = dev.create_buffer(&wgpu::BufferDescriptor {
            label: None,
            size: n as u64 * 12,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let mut enc = dev.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        enc.copy_buffer_to_buffer(&db, 0, &staging, 0, n as u64 * 12);
        queue.submit(Some(enc.finish()));
        let slice = staging.slice(..);
        slice.map_async(wgpu::MapMode::Read, |_| {});
        dev.poll(wgpu::PollType::wait_indefinitely()).ok();
        let gpu: Vec<f32> = bytemuck::cast_slice(&slice.get_mapped_range().unwrap()).to_vec();

        let r64: Vec<[f64; 3]> = render.iter().map(|p| p.map(|v| v as f64)).collect();
        let g64: Vec<[f64; 3]> = gt8.iter().map(|p| p.map(|v| v as f64 / 255.0)).collect();
        let (_, grad) = ssim_with_grad(&r64, &g64, w as usize, h as usize, true);
        let grad = grad.unwrap();
        // d_image started at zero, so the kernel wrote -lambda * dSSIM/dx.
        let scale = grad.iter().flatten().map(|v| v.abs()).fold(0.0, f64::max) * lambda as f64;
        let mut worst = 0.0f64;
        for (p, gp) in grad.iter().enumerate() {
            for c in 0..3 {
                let want = -(lambda as f64) * gp[c];
                let got = gpu[p * 3 + c] as f64;
                worst = worst.max((want - got).abs() / scale);
            }
        }
        eprintln!("ssim grad worst error {worst:.2e} of max");
        assert!(worst < 1e-3, "worst relative error {worst}");
    }
}
