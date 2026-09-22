//! Device-side LSD radix sort over `(key, value)` `u32` pairs.
//!
//! Eight 4-bit passes (32-bit keys), each pass: histogram per workgroup, a
//! device-side exclusive scan of the histograms, then a stable scatter. Buffers
//! ping-pong between passes; with eight (even) passes the sorted result ends up
//! back in the input buffers.
//!
//! This removes the host round-trips that the stage-1 renderer needed for its
//! sorts.

use bytemuck::{Pod, Zeroable};

use crate::shaders;

const WORKGROUP: u32 = 256;
const BINS: u32 = 16;
/// Elements (key, value pairs) processed per workgroup per pass.
const ELEMENTS_PER_THREAD: u32 = 4;
const BLOCK: u32 = WORKGROUP * ELEMENTS_PER_THREAD;

/// Uniform block for the three radix kernels.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Pod, Zeroable)]
pub struct RadixParams {
    pub shift: u32,
    pub num_elements: u32,
    pub num_blocks: u32,
    pub _pad: u32,
}

/// Number of workgroups needed to cover `n` elements.
pub fn block_count(n: usize) -> u32 {
    (n as u32).div_ceil(BLOCK).max(1)
}

/// A key/value buffer pair used as radix input or output.
pub struct SortBuffers {
    pub keys: wgpu::Buffer,
    pub values: wgpu::Buffer,
}

fn storage(
    device: &wgpu::Device,
    label: &str,
    bytes: u64,
    extra: wgpu::BufferUsages,
) -> wgpu::Buffer {
    device.create_buffer(&wgpu::BufferDescriptor {
        label: Some(label),
        size: bytes.max(4),
        usage: wgpu::BufferUsages::STORAGE
            | wgpu::BufferUsages::COPY_DST
            | wgpu::BufferUsages::COPY_SRC
            | extra,
        mapped_at_creation: false,
    })
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

/// A reusable radix sort: pipelines plus histogram/base scratch.
pub struct RadixSorter {
    histogram: wgpu::ComputePipeline,
    scan: wgpu::ComputePipeline,
    scatter: wgpu::ComputePipeline,
    layout: wgpu::BindGroupLayout,
    params: wgpu::Buffer,
    hist: wgpu::Buffer,
    base: wgpu::Buffer,
    max_elements: usize,
}

impl RadixSorter {
    pub fn new(device: &wgpu::Device, max_elements: usize) -> Self {
        let kernel = shaders::radix();
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("radix"),
            source: wgpu::ShaderSource::Wgsl(kernel.source.into()),
        });

        let ro = wgpu::BufferBindingType::Storage { read_only: true };
        let rw = wgpu::BufferBindingType::Storage { read_only: false };
        let uni = wgpu::BufferBindingType::Uniform;
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("radix"),
            entries: &[
                entry(0, uni),
                entry(1, ro),
                entry(2, ro),
                entry(3, rw),
                entry(4, rw),
                entry(5, rw),
                entry(6, rw),
            ],
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("radix"),
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
            label: Some("radix_params"),
            size: std::mem::size_of::<RadixParams>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let hist = storage(
            device,
            "radix_hist",
            (max_blocks * BINS as usize * 4) as u64,
            wgpu::BufferUsages::empty(),
        );
        let base = storage(
            device,
            "radix_base",
            (BINS as usize * max_blocks * 4) as u64,
            wgpu::BufferUsages::empty(),
        );

        Self {
            histogram: mk("radix_histogram"),
            scan: mk("radix_scan"),
            scatter: mk("radix_scatter"),
            layout,
            params,
            hist,
            base,
            max_elements,
        }
    }

    /// Allocate a pair of ping-pong buffers for up to `max_elements` pairs.
    pub fn allocate(&self, device: &wgpu::Device, label: &str) -> [SortBuffers; 2] {
        let bytes = (self.max_elements * 4) as u64;
        let mk = |suffix: &str| SortBuffers {
            keys: storage(
                device,
                &format!("{label}_keys{suffix}"),
                bytes,
                wgpu::BufferUsages::empty(),
            ),
            values: storage(
                device,
                &format!("{label}_values{suffix}"),
                bytes,
                wgpu::BufferUsages::empty(),
            ),
        };
        [mk("_a"), mk("_b")]
    }

    fn bind(
        &self,
        device: &wgpu::Device,
        input: &SortBuffers,
        output: &SortBuffers,
    ) -> wgpu::BindGroup {
        device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("radix"),
            layout: &self.layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: self.params.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: input.keys.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: input.values.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: output.keys.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 4,
                    resource: output.values.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 5,
                    resource: self.hist.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 6,
                    resource: self.base.as_entire_binding(),
                },
            ],
        })
    }

    /// Sort `n` pairs from `pairs[0]`, leaving the result in the buffer pair
    /// returned. `pairs` must come from [`RadixSorter::allocate`].
    pub fn sort(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        pairs: &[SortBuffers; 2],
        n: usize,
    ) -> usize {
        if n == 0 {
            return 0;
        }
        let nblocks = block_count(n);
        // 8 passes of 4 bits; even number, so the result lands in pair 0.
        for pass in 0..8u32 {
            let shift = pass * 4;
            let params = RadixParams {
                shift,
                num_elements: n as u32,
                num_blocks: nblocks,
                _pad: 0,
            };
            queue.write_buffer(&self.params, 0, bytemuck::bytes_of(&params));
            // Zero the histogram for this pass.
            queue.write_buffer(
                &self.hist,
                0,
                &vec![0u8; nblocks as usize * BINS as usize * 4],
            );
            let src = if pass % 2 == 0 { 0 } else { 1 };
            let dst = 1 - src;
            let bind = self.bind(device, &pairs[src], &pairs[dst]);
            let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("radix_pass"),
            });
            {
                let mut cp = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor::default());
                cp.set_bind_group(0, &bind, &[]);
                cp.set_pipeline(&self.histogram);
                cp.dispatch_workgroups(nblocks, 1, 1);
                cp.set_pipeline(&self.scan);
                cp.dispatch_workgroups(1, 1, 1);
                cp.set_pipeline(&self.scatter);
                cp.dispatch_workgroups(nblocks, 1, 1);
            }
            queue.submit(Some(encoder.finish()));
        }
        0
    }
}
