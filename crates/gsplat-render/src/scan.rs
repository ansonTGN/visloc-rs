//! Device-side inclusive prefix sum over `u32` values.

use bytemuck::{Pod, Zeroable};

use crate::shaders;

const WG: u32 = 256;
const ELEMENTS_PER_THREAD: u32 = 8;
const BLOCK: u32 = WG * ELEMENTS_PER_THREAD;

/// Uniform block for the scan kernels.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Pod, Zeroable)]
pub struct ScanParams {
    pub num_elements: u32,
    pub num_blocks: u32,
    pub _pad: [u32; 2],
}

/// Number of workgroups needed for `n` elements.
pub fn block_count(n: usize) -> u32 {
    (n as u32).div_ceil(BLOCK).max(1)
}

fn entry(binding: u32, ty: wgpu::BufferBindingType) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Buffer {
            ty,
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    }
}

/// A reusable inclusive-scan pipeline and its block-sum scratch.
pub struct PrefixScanner {
    scan_blocks: wgpu::ComputePipeline,
    scan_apply: wgpu::ComputePipeline,
    layout: wgpu::BindGroupLayout,
    params: wgpu::Buffer,
    block_sums: wgpu::Buffer,
}

impl PrefixScanner {
    pub fn new(device: &wgpu::Device, max_elements: usize) -> Self {
        let kernel = shaders::scan();
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("scan"),
            source: wgpu::ShaderSource::Wgsl(kernel.source.into()),
        });
        let ro = wgpu::BufferBindingType::Storage { read_only: true };
        let rw = wgpu::BufferBindingType::Storage { read_only: false };
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("scan"),
            entries: &[
                entry(0, wgpu::BufferBindingType::Uniform),
                entry(1, ro),
                entry(2, rw),
                entry(3, rw),
            ],
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("scan"),
            bind_group_layouts: &[Some(&layout)],
            immediate_size: 0,
        });
        let mk = |entry_point: &str| {
            device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some(entry_point),
                layout: Some(&pipeline_layout),
                module: &module,
                entry_point: Some(entry_point),
                compilation_options: Default::default(),
                cache: None,
            })
        };
        let max_blocks = block_count(max_elements) as usize;
        let params = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("scan_params"),
            size: std::mem::size_of::<ScanParams>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let block_sums = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("scan_block_sums"),
            size: (max_blocks * 4).max(4) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        Self {
            scan_blocks: mk("scan_blocks"),
            scan_apply: mk("scan_apply"),
            layout,
            params,
            block_sums,
        }
    }

    /// Inclusively scan `n` elements from `input` into `output` (device-side).
    pub fn scan(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        input: &wgpu::Buffer,
        output: &wgpu::Buffer,
        n: usize,
    ) {
        if n == 0 {
            return;
        }
        let nblocks = block_count(n);
        let params = ScanParams {
            num_elements: n as u32,
            num_blocks: nblocks,
            _pad: [0; 2],
        };
        queue.write_buffer(&self.params, 0, bytemuck::bytes_of(&params));
        let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("scan"),
            layout: &self.layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: self.params.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: input.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: output.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: self.block_sums.as_entire_binding(),
                },
            ],
        });
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("scan"),
        });
        {
            let mut cp = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor::default());
            cp.set_bind_group(0, &bind, &[]);
            cp.set_pipeline(&self.scan_blocks);
            cp.dispatch_workgroups(nblocks, 1, 1);
            cp.set_pipeline(&self.scan_apply);
            cp.dispatch_workgroups(nblocks, 1, 1);
        }
        queue.submit(Some(encoder.finish()));
    }
}
