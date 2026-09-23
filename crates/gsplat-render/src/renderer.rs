//! The wgpu forward rasterizer.
//!
//! Pipeline (per rendered frame):
//!
//! 1. `project_forward` — project every gaussian; compact the visible ones and
//!    count the tiles each covers.
//! 2. host readback — `num_visible` / `num_intersections`.
//! 3. host — depth-sort the visible ids, prefix-sum the tile counts.
//! 4. `project_visible` — re-project and evaluate SH colour for each visible
//!    gaussian into `projected_splats`.
//! 5. `map_gaussians` — expand each visible gaussian into `(tile_id, compact)`
//!    entries; then host-sort them by tile id.
//! 6. `tile_offsets` — build per-tile `[start, end)` ranges.
//! 7. `rasterize` — one workgroup per tile composites front-to-back.
//!
//! Stages 3 and 5's sorts run on the host at this stage; a device-side radix
//! sort is a planned follow-up (see `docs/rust_3dgs_plan.md`).

use visloc_gsplat_core::camera::CameraView;
use visloc_gsplat_core::cpu_render::Image;
use visloc_gsplat_core::gaussian::Scene;

use crate::gpu::{GpuContext, GpuError};
use crate::packing::PackedScene;
use crate::shaders;
use crate::uniforms::{tile_bounds, ProjectUniforms, RasterUniforms};

/// A scene uploaded to the GPU once.
pub struct GpuScene {
    pub packed: PackedScene,
    transforms: wgpu::Buffer,
    opacity: wgpu::Buffer,
    sh: wgpu::Buffer,
}

fn new_storage(device: &wgpu::Device, label: &str, bytes: u64) -> wgpu::Buffer {
    device.create_buffer(&wgpu::BufferDescriptor {
        label: Some(label),
        size: bytes.max(4),
        usage: wgpu::BufferUsages::STORAGE
            | wgpu::BufferUsages::COPY_DST
            | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    })
}

impl GpuScene {
    pub fn upload(ctx: &GpuContext, scene: &Scene) -> Self {
        let packed = PackedScene::from_scene(scene);
        let transforms = new_storage(
            &ctx.device,
            "transforms",
            (packed.transforms.len() * 4) as u64,
        );
        ctx.queue
            .write_buffer(&transforms, 0, bytemuck::cast_slice(&packed.transforms));
        let opacity = new_storage(&ctx.device, "opacity", (packed.opacity.len() * 4) as u64);
        ctx.queue
            .write_buffer(&opacity, 0, bytemuck::cast_slice(&packed.opacity));
        let sh = new_storage(&ctx.device, "sh", (packed.sh.len() * 4) as u64);
        ctx.queue
            .write_buffer(&sh, 0, bytemuck::cast_slice(&packed.sh));
        Self {
            packed,
            transforms,
            opacity,
            sh,
        }
    }

    /// Re-upload the packed arrays (used when the host mutates the scene).
    pub fn reupload(&self, ctx: &GpuContext, packed: &PackedScene) {
        ctx.queue.write_buffer(
            &self.transforms,
            0,
            bytemuck::cast_slice(&packed.transforms),
        );
        ctx.queue
            .write_buffer(&self.opacity, 0, bytemuck::cast_slice(&packed.opacity));
        ctx.queue
            .write_buffer(&self.sh, 0, bytemuck::cast_slice(&packed.sh));
    }
}

/// A named binding used to build layouts and bind groups uniformly.
struct Binding {
    binding: u32,
    ty: wgpu::BufferBindingType,
    buffer: wgpu::Buffer,
}

fn build_layout(device: &wgpu::Device, label: &str, bindings: &[Binding]) -> wgpu::BindGroupLayout {
    let entries: Vec<wgpu::BindGroupLayoutEntry> = bindings
        .iter()
        .map(|b| wgpu::BindGroupLayoutEntry {
            binding: b.binding,
            visibility: wgpu::ShaderStages::COMPUTE,
            ty: wgpu::BindingType::Buffer {
                ty: b.ty,
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            count: None,
        })
        .collect();
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some(label),
        entries: &entries,
    })
}

fn build_bind_group(
    device: &wgpu::Device,
    label: &str,
    layout: &wgpu::BindGroupLayout,
    bindings: &[Binding],
) -> wgpu::BindGroup {
    let entries: Vec<wgpu::BindGroupEntry> = bindings
        .iter()
        .map(|b| wgpu::BindGroupEntry {
            binding: b.binding,
            resource: b.buffer.as_entire_binding(),
        })
        .collect();
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some(label),
        layout,
        entries: &entries,
    })
}

/// A compute pipeline plus the bind group for its single bind group (group 0).
struct Stage {
    pipeline: wgpu::ComputePipeline,
    bind_group: wgpu::BindGroup,
}

fn build_stage(
    device: &wgpu::Device,
    label: &str,
    kernel_name: &str,
    source: String,
    bindings: &[Binding],
) -> Stage {
    let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some(label),
        source: wgpu::ShaderSource::Wgsl(source.into()),
    });
    let layout = build_layout(device, label, bindings);
    let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some(label),
        bind_group_layouts: &[Some(&layout)],
        immediate_size: 0,
    });
    let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some(label),
        layout: Some(&pipeline_layout),
        module: &module,
        entry_point: Some(kernel_name),
        compilation_options: Default::default(),
        cache: None,
    });
    let bind_group = build_bind_group(device, label, &layout, bindings);
    Stage {
        pipeline,
        bind_group,
    }
}

fn dispatch(pass: &mut wgpu::ComputePass<'_>, stage: &Stage, x: u32) {
    pass.set_pipeline(&stage.pipeline);
    pass.set_bind_group(0, &stage.bind_group, &[]);
    pass.dispatch_workgroups(x.max(1), 1, 1);
}

/// Per-frame scratch buffers sized from the gaussian count.
struct Scratch {
    global_from_compact: wgpu::Buffer,
    compact_from_global: wgpu::Buffer,
    depths: wgpu::Buffer,
    intersect_counts: wgpu::Buffer,
    cum_tiles_hit: wgpu::Buffer,
    projected_splats: wgpu::Buffer,
    tile_id_from_isect: wgpu::Buffer,
    compact_gid_from_isect: wgpu::Buffer,
    num_visible: wgpu::Buffer,
    num_intersections: wgpu::Buffer,
    readback: wgpu::Buffer,
    max_isects: usize,
}

impl Scratch {
    fn new(device: &wgpu::Device, n: usize, max_isects: usize) -> Self {
        Self {
            global_from_compact: new_storage(device, "global_from_compact", (n * 4) as u64),
            compact_from_global: new_storage(device, "compact_from_global", (n * 4) as u64),
            depths: new_storage(device, "depths", (n * 4) as u64),
            intersect_counts: new_storage(device, "intersect_counts", (n * 4) as u64),
            cum_tiles_hit: new_storage(device, "cum_tiles_hit", (n * 4) as u64),
            projected_splats: new_storage(device, "projected_splats", (max_isects * 9 * 4) as u64),
            tile_id_from_isect: new_storage(device, "tile_id_from_isect", (max_isects * 4) as u64),
            compact_gid_from_isect: new_storage(
                device,
                "compact_gid_from_isect",
                (max_isects * 4) as u64,
            ),
            num_visible: new_storage(device, "num_visible", 4),
            num_intersections: new_storage(device, "num_intersections", 4),
            readback: device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("readback"),
                size: 8,
                usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }),
            max_isects,
        }
    }
}

/// Read `num_visible` and `num_intersections` from their counter buffers.
fn read_counters(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    num_visible: &wgpu::Buffer,
    num_intersections: &wgpu::Buffer,
    readback: &wgpu::Buffer,
) -> (u32, u32) {
    let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("readback"),
    });
    encoder.copy_buffer_to_buffer(num_visible, 0, readback, 0, 4);
    encoder.copy_buffer_to_buffer(num_intersections, 0, readback, 4, 4);
    queue.submit(Some(encoder.finish()));
    let slice = readback.slice(..);
    let (tx, rx) = std::sync::mpsc::channel();
    slice.map_async(wgpu::MapMode::Read, move |r| {
        let _ = tx.send(r);
    });
    device.poll(wgpu::PollType::wait_indefinitely()).ok();
    let _ = rx.recv();
    let data = slice.get_mapped_range().expect("map range");
    let values: [u32; 2] = bytemuck::pod_read_unaligned(&data[..8]);
    drop(data);
    readback.unmap();
    (values[0], values[1])
}

/// The forward renderer.
pub struct Renderer {
    pub ctx: GpuContext,
    pub scene: GpuScene,
    scratch: Scratch,
    forward: Stage,
    visible: Stage,
    map: Stage,
    offsets: Stage,
    raster: Stage,
    proj_uniforms: wgpu::Buffer,
    raster_uniforms: wgpu::Buffer,
    tile_offsets: wgpu::Buffer,
    out_img: wgpu::Buffer,
    image_w: u32,
    image_h: u32,
    sh_degree: u32,
    // Device-side sorts and prefix scan.
    sorter: crate::sort::RadixSorter,
    scanner: crate::scan::PrefixScanner,
    depth_pairs: [crate::sort::SortBuffers; 2],
    tile_pairs: [crate::sort::SortBuffers; 2],
    counts_sorted: wgpu::Buffer,
    /// Skip the output-image readback (for GPU-only timing / viewer use).
    skip_readback: bool,
}

impl Renderer {
    pub fn new(
        ctx: GpuContext,
        scene: &Scene,
        image_w: u32,
        image_h: u32,
    ) -> Result<Self, GpuError> {
        let gpu_scene = GpuScene::upload(&ctx, scene);
        let n = scene.len();
        let (tbw, tbh) = tile_bounds(image_w, image_h);
        let num_tiles = (tbw * tbh) as usize;
        // The projected-splat buffer is 9 f32 per intersection, so an
        // intersection count is bounded by whichever storage limit is smaller.
        let bytes_per_isect = 9 * 4u64;
        let limit = ctx
            .limits
            .max_storage_buffer_binding_size
            .min(ctx.limits.max_buffer_size);
        let max_isects_by_limit = (limit / bytes_per_isect) as usize;
        let max_isects = (n * 64).clamp(1 << 20, 1 << 26).min(max_isects_by_limit);
        let scratch = Scratch::new(&ctx.device, n, max_isects);
        let dev = &ctx.device;

        let proj_uniforms = dev.create_buffer(&wgpu::BufferDescriptor {
            label: Some("proj_uniforms"),
            size: std::mem::size_of::<ProjectUniforms>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let raster_uniforms = dev.create_buffer(&wgpu::BufferDescriptor {
            label: Some("raster_uniforms"),
            size: std::mem::size_of::<RasterUniforms>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let tile_offsets = new_storage(dev, "tile_offsets", (num_tiles * 2 * 4) as u64);
        let out_img = new_storage(dev, "out_img", (image_w as u64) * (image_h as u64) * 3 * 4);

        let ro = wgpu::BufferBindingType::Storage { read_only: true };
        let rw = wgpu::BufferBindingType::Storage { read_only: false };
        // Compact-order tile counts, gathered by `project_visible` for the scan.
        let counts_sorted = new_storage(dev, "counts_sorted", (n * 4) as u64);

        let forward = build_stage(
            dev,
            "project_forward",
            "project_forward",
            shaders::project_forward().source,
            &[
                Binding {
                    binding: 0,
                    ty: wgpu::BufferBindingType::Uniform,
                    buffer: proj_uniforms.clone(),
                },
                Binding {
                    binding: 1,
                    ty: ro,
                    buffer: gpu_scene.transforms.clone(),
                },
                Binding {
                    binding: 2,
                    ty: ro,
                    buffer: gpu_scene.opacity.clone(),
                },
                Binding {
                    binding: 3,
                    ty: rw,
                    buffer: scratch.global_from_compact.clone(),
                },
                Binding {
                    binding: 4,
                    ty: rw,
                    buffer: scratch.depths.clone(),
                },
                Binding {
                    binding: 5,
                    ty: rw,
                    buffer: scratch.intersect_counts.clone(),
                },
                Binding {
                    binding: 6,
                    ty: rw,
                    buffer: scratch.num_visible.clone(),
                },
                Binding {
                    binding: 7,
                    ty: rw,
                    buffer: scratch.num_intersections.clone(),
                },
            ],
        );

        let visible = build_stage(
            dev,
            "project_visible",
            "project_visible",
            shaders::project_visible().source,
            &[
                Binding {
                    binding: 0,
                    ty: wgpu::BufferBindingType::Uniform,
                    buffer: proj_uniforms.clone(),
                },
                Binding {
                    binding: 1,
                    ty: ro,
                    buffer: gpu_scene.transforms.clone(),
                },
                Binding {
                    binding: 2,
                    ty: ro,
                    buffer: gpu_scene.opacity.clone(),
                },
                Binding {
                    binding: 3,
                    ty: ro,
                    buffer: gpu_scene.sh.clone(),
                },
                Binding {
                    binding: 4,
                    ty: ro,
                    buffer: scratch.global_from_compact.clone(),
                },
                Binding {
                    binding: 5,
                    ty: rw,
                    buffer: scratch.projected_splats.clone(),
                },
                Binding {
                    binding: 6,
                    ty: rw,
                    buffer: scratch.compact_from_global.clone(),
                },
                Binding {
                    binding: 7,
                    ty: ro,
                    buffer: scratch.intersect_counts.clone(),
                },
                Binding {
                    binding: 8,
                    ty: rw,
                    buffer: counts_sorted.clone(),
                },
            ],
        );

        let map = build_stage(
            dev,
            "map_gaussians",
            "map_gaussians",
            shaders::map_gaussians().source,
            &[
                Binding {
                    binding: 0,
                    ty: wgpu::BufferBindingType::Uniform,
                    buffer: proj_uniforms.clone(),
                },
                Binding {
                    binding: 1,
                    ty: ro,
                    buffer: gpu_scene.transforms.clone(),
                },
                Binding {
                    binding: 2,
                    ty: ro,
                    buffer: gpu_scene.opacity.clone(),
                },
                Binding {
                    binding: 3,
                    ty: ro,
                    buffer: scratch.cum_tiles_hit.clone(),
                },
                Binding {
                    binding: 4,
                    ty: ro,
                    buffer: scratch.global_from_compact.clone(),
                },
                Binding {
                    binding: 5,
                    ty: rw,
                    buffer: scratch.tile_id_from_isect.clone(),
                },
                Binding {
                    binding: 6,
                    ty: rw,
                    buffer: scratch.compact_gid_from_isect.clone(),
                },
            ],
        );

        let offsets = build_stage(
            dev,
            "get_tile_offsets",
            "get_tile_offsets",
            shaders::tile_offsets().source,
            &[
                Binding {
                    binding: 0,
                    ty: wgpu::BufferBindingType::Uniform,
                    buffer: proj_uniforms.clone(),
                },
                Binding {
                    binding: 1,
                    ty: ro,
                    buffer: scratch.tile_id_from_isect.clone(),
                },
                Binding {
                    binding: 2,
                    ty: rw,
                    buffer: tile_offsets.clone(),
                },
            ],
        );

        let raster = build_stage(
            dev,
            "rasterize",
            "rasterize",
            shaders::rasterize().source,
            &[
                Binding {
                    binding: 0,
                    ty: wgpu::BufferBindingType::Uniform,
                    buffer: raster_uniforms.clone(),
                },
                Binding {
                    binding: 1,
                    ty: ro,
                    buffer: scratch.projected_splats.clone(),
                },
                Binding {
                    binding: 2,
                    ty: ro,
                    buffer: scratch.compact_gid_from_isect.clone(),
                },
                Binding {
                    binding: 3,
                    ty: ro,
                    buffer: tile_offsets.clone(),
                },
                Binding {
                    binding: 4,
                    ty: rw,
                    buffer: out_img.clone(),
                },
                Binding {
                    binding: 5,
                    ty: ro,
                    buffer: scratch.global_from_compact.clone(),
                },
            ],
        );

        // Device-side sort/scan scratch. The depth sort and tile sort each need
        // ping-pong key/value pairs; `counts_sorted` feeds the prefix scan.
        let sort_capacity = n.max(max_isects).max(1);
        let sorter = crate::sort::RadixSorter::new(dev, sort_capacity);
        let scanner = crate::scan::PrefixScanner::new(dev, n.max(1));
        let depth_pairs = sorter.allocate(dev, "depth");
        let tile_pairs = sorter.allocate(dev, "tile");

        Ok(Self {
            ctx,
            scene: gpu_scene,
            scratch,
            forward,
            visible,
            map,
            offsets,
            raster,
            proj_uniforms,
            raster_uniforms,
            tile_offsets,
            out_img,
            image_w,
            image_h,
            sh_degree: scene.sh_degree,
            sorter,
            scanner,
            depth_pairs,
            tile_pairs,
            counts_sorted,
            skip_readback: false,
        })
    }

    /// Render without reading the output image back to the CPU.
    ///
    /// A native viewer presents the output buffer directly, so this is the
    /// per-frame cost that matters; use it to measure the true GPU frame time.
    pub fn set_skip_readback(&mut self, skip: bool) {
        self.skip_readback = skip;
    }

    pub fn num_gaussians(&self) -> usize {
        self.scene.packed.num_gaussians
    }

    /// Render `view` and return the linear-RGB image.
    pub fn render(&mut self, view: &CameraView, bg: [f32; 3]) -> Image {
        assert_eq!(
            view.camera.width, self.image_w,
            "view width matches renderer"
        );
        assert_eq!(
            view.camera.height, self.image_h,
            "view height matches renderer"
        );
        let mut prof = StageTimer::from_env();
        let u = ProjectUniforms::from_view(view, self.sh_degree, self.num_gaussians() as u32);
        let num_tiles = u.num_tiles();
        let raster_u = RasterUniforms::new(&u, bg);
        let n = self.num_gaussians() as u32;

        // ----- Upload uniforms + reset dynamic buffers. -----
        self.ctx
            .queue
            .write_buffer(&self.proj_uniforms, 0, bytemuck::bytes_of(&u));
        self.ctx
            .queue
            .write_buffer(&self.raster_uniforms, 0, bytemuck::bytes_of(&raster_u));
        self.ctx
            .queue
            .write_buffer(&self.scratch.num_visible, 0, &[0u8; 4]);
        self.ctx
            .queue
            .write_buffer(&self.scratch.num_intersections, 0, &[0u8; 4]);
        let mut init = vec![0u32; num_tiles as usize * 2];
        for t in 0..num_tiles as usize {
            init[t * 2] = u32::MAX;
        }
        self.ctx
            .queue
            .write_buffer(&self.tile_offsets, 0, bytemuck::cast_slice(&init));

        // ----- Pass 1: project_forward. -----
        {
            let mut encoder = self
                .ctx
                .device
                .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("pf") });
            {
                let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor::default());
                dispatch(&mut pass, &self.forward, n.div_ceil(256));
            }
            self.ctx.queue.submit(Some(encoder.finish()));
        }
        prof.mark(&self.ctx.device, "project_fwd");

        // ----- Host readback of compaction counts. -----
        let (num_visible, num_intersections) = read_counters(
            &self.ctx.device,
            &self.ctx.queue,
            &self.scratch.num_visible,
            &self.scratch.num_intersections,
            &self.scratch.readback,
        );
        prof.mark(&self.ctx.device, "counters");
        if prof.enabled() {
            eprintln!(
                "[profile] counters raw: visible={num_visible} isects={num_intersections} (cap {})",
                self.scratch.max_isects
            );
        }
        let nv = (num_visible as usize).min(self.num_gaussians());
        let ni = (num_intersections as usize).min(self.scratch.max_isects);

        // Re-upload the uniforms with the now-known compaction counts.
        let u = ProjectUniforms {
            num_visible: nv as u32,
            num_intersections: ni as u32,
            ..u
        };
        self.ctx
            .queue
            .write_buffer(&self.proj_uniforms, 0, bytemuck::bytes_of(&u));

        // ----- Device-side depth sort. -----
        //
        // Keys are the raw f32 depth bits (monotonic for positive z), values are
        // the global gaussian ids, so ascending key order is nearest-first
        // (front-to-back). Full 32-bit keys: eight ping-pong passes.
        if nv > 0 {
            self.sort_in_place(
                &self.scratch.depths,
                &self.scratch.global_from_compact,
                &self.depth_pairs,
                nv,
                32,
            );
        }
        prof.mark(&self.ctx.device, "depth_sort");

        // ----- Pass 2a: project_visible (also gathers tile counts). -----
        if nv > 0 {
            let mut encoder = self
                .ctx
                .device
                .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("pv") });
            {
                let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor::default());
                dispatch(&mut pass, &self.visible, (nv as u32).div_ceil(256));
            }
            self.ctx.queue.submit(Some(encoder.finish()));
        }
        prof.mark(&self.ctx.device, "project_vis");

        // ----- Device-side prefix scan of the compact-order tile counts. -----
        if nv > 0 {
            self.scanner.scan(
                &self.ctx.device,
                &self.ctx.queue,
                &self.counts_sorted,
                &self.scratch.cum_tiles_hit,
                nv,
            );
        }
        prof.mark(&self.ctx.device, "scan");

        // ----- Pass 2b: map_gaussians. -----
        if nv > 0 {
            let mut encoder = self
                .ctx
                .device
                .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("map") });
            {
                let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor::default());
                dispatch(&mut pass, &self.map, (nv as u32).div_ceil(256));
            }
            self.ctx.queue.submit(Some(encoder.finish()));
        }
        prof.mark(&self.ctx.device, "map");

        // ----- Device-side tile-id sort over the isect list. -----
        if ni > 0 {
            self.sort_in_place(
                &self.scratch.tile_id_from_isect,
                &self.scratch.compact_gid_from_isect,
                &self.tile_pairs,
                ni,
                // Keys are tile ids < num_tiles; the LSD sort is stable, so the
                // depth order from the first sort survives within each tile.
                crate::sort::key_bits_for(num_tiles.saturating_sub(1)),
            );
        }
        prof.mark(&self.ctx.device, "tile_sort");

        // ----- Pass 3: tile_offsets + rasterize. -----
        {
            let mut encoder =
                self.ctx
                    .device
                    .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                        label: Some("raster"),
                    });
            {
                let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor::default());
                dispatch(&mut pass, &self.offsets, (ni as u32).div_ceil(256));
            }
            self.ctx.queue.submit(Some(encoder.finish()));
        }
        prof.mark(&self.ctx.device, "tile_offsets");
        {
            let mut encoder =
                self.ctx
                    .device
                    .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                        label: Some("raster"),
                    });
            {
                let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor::default());
                dispatch(&mut pass, &self.raster, num_tiles);
            }
            self.ctx.queue.submit(Some(encoder.finish()));
        }
        prof.mark(&self.ctx.device, "rasterize");
        if prof.enabled() {
            prof.report(nv, ni, num_tiles);
            eprintln!("[profile] {}", self.tile_list_stats(num_tiles));
        }

        // A viewer renders to a surface and never reads the image back; this
        // path exists so the true GPU frame cost can be measured.
        if self.skip_readback {
            return Image {
                width: self.image_w,
                height: self.image_h,
                rgb: Vec::new(),
            };
        }

        let t_read = std::time::Instant::now();
        let rgb = read_f32s(
            &self.ctx.device,
            &self.ctx.queue,
            &self.out_img,
            (self.image_w as usize) * (self.image_h as usize) * 3,
        );
        if prof.enabled() {
            eprintln!("[profile] image readback: {:?}", t_read.elapsed());
        }
        let rgb = rgb.chunks_exact(3).map(|c| [c[0], c[1], c[2]]).collect();
        Image {
            width: self.image_w,
            height: self.image_h,
            rgb,
        }
    }

    /// Per-tile isect list length statistics (profiling only): a long tail
    /// means `rasterize` (one workgroup per tile) is load-imbalanced.
    fn tile_list_stats(&self, num_tiles: u32) -> String {
        let raw = read_bytes(
            &self.ctx.device,
            &self.ctx.queue,
            &self.tile_offsets,
            num_tiles as usize * 2 * 4,
        );
        let ranges: &[u32] = bytemuck::cast_slice(&raw);
        let mut lens: Vec<u32> = ranges
            .chunks_exact(2)
            .map(|r| if r[0] == u32::MAX { 0 } else { r[1] - r[0] })
            .collect();
        lens.sort_unstable();
        let total: u64 = lens.iter().map(|&l| l as u64).sum();
        let pct = |q: f64| lens[((lens.len() - 1) as f64 * q) as usize];
        format!(
            "tile lists: tiles={} mean={:.0} p50={} p90={} p99={} max={} (max/mean={:.1})",
            lens.len(),
            total as f64 / lens.len() as f64,
            pct(0.5),
            pct(0.9),
            pct(0.99),
            pct(1.0),
            pct(1.0) as f64 / (total as f64 / lens.len() as f64).max(1.0)
        )
    }

    /// Sort `n` (key, value) pairs held in `keys`/`values` in place, using
    /// `pairs` as ping-pong scratch; keys must fit in `key_bits` bits. Copy-in,
    /// the radix passes and copy-out are recorded into a single command buffer
    /// (one submit).
    fn sort_in_place(
        &self,
        keys: &wgpu::Buffer,
        values: &wgpu::Buffer,
        pairs: &[crate::sort::SortBuffers; 2],
        n: usize,
        key_bits: u32,
    ) {
        let len = (n * 4) as u64;
        self.sorter.prepare(&self.ctx.queue, n, key_bits);
        let mut encoder = self
            .ctx
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("sort"),
            });
        encoder.copy_buffer_to_buffer(keys, 0, &pairs[0].keys, 0, len);
        encoder.copy_buffer_to_buffer(values, 0, &pairs[0].values, 0, len);
        let out = self
            .sorter
            .encode(&self.ctx.device, &mut encoder, pairs, n, key_bits);
        encoder.copy_buffer_to_buffer(&pairs[out].keys, 0, keys, 0, len);
        encoder.copy_buffer_to_buffer(&pairs[out].values, 0, values, 0, len);
        self.ctx.queue.submit(Some(encoder.finish()));
    }
}

fn read_f32s(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    buffer: &wgpu::Buffer,
    count: usize,
) -> Vec<f32> {
    if count == 0 {
        return Vec::new();
    }
    let raw = read_bytes(device, queue, buffer, count * 4);
    bytemuck::cast_slice(&raw).to_vec()
}

fn read_bytes(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    buffer: &wgpu::Buffer,
    len: usize,
) -> Vec<u8> {
    let staging = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("staging"),
        size: len as u64,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("staging-copy"),
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

/// Opt-in (`GSPLAT_PROFILE`) per-stage wall-clock timer. Each `mark` blocks
/// until the GPU is idle, so stages are serialised and their costs separable;
/// when disabled it is a no-op and the frame stays pipelined.
struct StageTimer {
    last: Option<std::time::Instant>,
    stages: Vec<(&'static str, std::time::Duration)>,
}

impl StageTimer {
    fn from_env() -> Self {
        Self {
            last: std::env::var("GSPLAT_PROFILE")
                .is_ok()
                .then(std::time::Instant::now),
            stages: Vec::new(),
        }
    }

    fn enabled(&self) -> bool {
        self.last.is_some()
    }

    fn mark(&mut self, device: &wgpu::Device, stage: &'static str) {
        let Some(last) = self.last else {
            return;
        };
        device.poll(wgpu::PollType::wait_indefinitely()).ok();
        let now = std::time::Instant::now();
        self.stages.push((stage, now - last));
        self.last = Some(now);
    }

    fn report(&self, nv: usize, ni: usize, num_tiles: u32) {
        let total: std::time::Duration = self.stages.iter().map(|s| s.1).sum();
        let parts: Vec<String> = self
            .stages
            .iter()
            .map(|(name, d)| format!("{name}={:.2}", d.as_secs_f64() * 1e3))
            .collect();
        eprintln!(
            "[profile] ms: {} total={:.2} | visible={nv} isects={ni} tiles={num_tiles}",
            parts.join(" "),
            total.as_secs_f64() * 1e3
        );
    }
}
