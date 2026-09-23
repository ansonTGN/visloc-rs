//! Device-side LSD radix sort over `(key, value)` `u32` pairs.
//!
//! One 4-bit pass per nibble of the key (eight for full 32-bit keys, fewer when
//! the caller bounds the key width), each pass: device-side histogram clear,
//! histogram per workgroup, a device-side exclusive scan of the histograms,
//! then a stable scatter. Buffers ping-pong between passes, so an odd pass
//! count leaves the result in the second buffer pair.
//!
//! This removes the host round-trips that the stage-1 renderer needed for its
//! sorts.

use bytemuck::{Pod, Zeroable};

use crate::shaders;

const WORKGROUP: u32 = 256;
const BINS: u32 = 16;
/// Elements (key, value pairs) processed per workgroup per pass.
const ELEMENTS_PER_THREAD: u32 = 16;
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

/// One `RadixParams` slot per pass (up to 8 for 32-bit keys), addressed by a
/// dynamic uniform offset.
const NUM_PASSES: usize = 8;
/// Uniform dynamic offsets must be aligned to this; each pass gets one slot.
const PARAMS_STRIDE: u64 = 256;

/// A reusable radix sort: pipelines plus histogram/base scratch.
pub struct RadixSorter {
    clear: wgpu::ComputePipeline,
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
        let mut params_entry = entry(0, uni);
        params_entry.ty = wgpu::BindingType::Buffer {
            ty: uni,
            has_dynamic_offset: true,
            min_binding_size: wgpu::BufferSize::new(std::mem::size_of::<RadixParams>() as u64),
        };
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("radix"),
            entries: &[
                params_entry,
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
            size: PARAMS_STRIDE * NUM_PASSES as u64,
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
            clear: mk("radix_clear"),
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

    /// Capacity in (key, value) pairs of buffers from [`RadixSorter::allocate`].
    pub fn max_elements(&self) -> usize {
        self.max_elements
    }

    /// Grow the histogram/base scratch to sort up to `max_elements` pairs
    /// (no-op if already large enough). Pipelines are kept; ping-pong pairs
    /// from [`RadixSorter::allocate`] must be re-allocated by the caller.
    pub fn reserve(&mut self, device: &wgpu::Device, max_elements: usize) {
        if max_elements <= self.max_elements {
            return;
        }
        let max_blocks = block_count(max_elements) as usize;
        self.hist = storage(
            device,
            "radix_hist",
            (max_blocks * BINS as usize * 4) as u64,
            wgpu::BufferUsages::empty(),
        );
        self.base = storage(
            device,
            "radix_base",
            (BINS as usize * max_blocks * 4) as u64,
            wgpu::BufferUsages::empty(),
        );
        self.max_elements = max_elements;
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
                    // Bind one slot; the pass is selected by the dynamic offset.
                    resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                        buffer: &self.params,
                        offset: 0,
                        size: wgpu::BufferSize::new(std::mem::size_of::<RadixParams>() as u64),
                    }),
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

    /// Write the per-pass uniforms for sorting `n` pairs whose keys fit in the
    /// low `key_bits` bits. Must be called before the command buffer recorded by
    /// [`RadixSorter::encode`] is submitted.
    pub fn prepare(&self, queue: &wgpu::Queue, n: usize, key_bits: u32) {
        if n == 0 {
            return;
        }
        let nblocks = block_count(n);
        for pass in 0..pass_count(key_bits) {
            let params = RadixParams {
                shift: pass as u32 * 4,
                num_elements: n as u32,
                num_blocks: nblocks,
                _pad: 0,
            };
            queue.write_buffer(
                &self.params,
                PARAMS_STRIDE * pass as u64,
                bytemuck::bytes_of(&params),
            );
        }
    }

    /// Record the sort passes (4 bits each, enough to cover `key_bits`) into
    /// `encoder`, one compute pass each. The histogram is zeroed on-device by
    /// the `radix_clear` kernel, so no host `write_buffer` is needed per pass.
    /// [`RadixSorter::prepare`] must have been called with the same `n` and
    /// `key_bits`. `pairs` must come from [`RadixSorter::allocate`], with the
    /// input in `pairs[0]`; keys must be zero above `key_bits`. Returns the
    /// index of the pair holding the sorted result (1 for an odd pass count).
    pub fn encode(
        &self,
        device: &wgpu::Device,
        encoder: &mut wgpu::CommandEncoder,
        pairs: &[SortBuffers; 2],
        n: usize,
        key_bits: u32,
    ) -> usize {
        if n == 0 {
            return 0;
        }
        let nblocks = block_count(n);
        let passes = pass_count(key_bits);
        for pass in 0..passes {
            let src = pass % 2;
            let dst = 1 - src;
            let bind = self.bind(device, &pairs[src], &pairs[dst]);
            let offset = (PARAMS_STRIDE * pass as u64) as u32;
            let mut cp = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor::default());
            cp.set_bind_group(0, &bind, &[offset]);
            cp.set_pipeline(&self.clear);
            cp.dispatch_workgroups((nblocks * BINS).div_ceil(WORKGROUP), 1, 1);
            cp.set_pipeline(&self.histogram);
            cp.dispatch_workgroups(nblocks, 1, 1);
            cp.set_pipeline(&self.scan);
            cp.dispatch_workgroups(1, 1, 1);
            cp.set_pipeline(&self.scatter);
            cp.dispatch_workgroups(nblocks, 1, 1);
        }
        passes % 2
    }
}

/// Number of 4-bit passes needed to sort keys of `key_bits` significant bits.
fn pass_count(key_bits: u32) -> usize {
    key_bits.clamp(1, 32).div_ceil(4) as usize
}

/// Significant bits of the largest key `max_key` (at least 1).
pub fn key_bits_for(max_key: u32) -> u32 {
    (32 - max_key.leading_zeros()).max(1)
}
