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
    // Reusable host scratch.
    host_order: Vec<u32>,
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
            host_order: Vec::new(),
        })
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

        // ----- Host readback of compaction counts. -----
        let (num_visible, num_intersections) = read_counters(
            &self.ctx.device,
            &self.ctx.queue,
            &self.scratch.num_visible,
            &self.scratch.num_intersections,
            &self.scratch.readback,
        );
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

        // ----- Host depth sort + tile-count prefix sum. -----
        self.host_order = read_u32s(
            &self.ctx.device,
            &self.ctx.queue,
            &self.scratch.global_from_compact,
            nv,
        );
        let depths = read_f32s(&self.ctx.device, &self.ctx.queue, &self.scratch.depths, nv);
        // Sort compact indices by descending depth (back-to-front), matching the
        // CPU reference's implicit far-to-near accumulation being order
        // independent only for commutative alpha; we keep the Inria front-to-back
        // order, so sort ascending depth and composite near-to-far.
        let mut order: Vec<u32> = (0..nv as u32).collect();
        order.sort_by(|&a, &b| {
            depths[a as usize]
                .partial_cmp(&depths[b as usize])
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        // `global_from_compact[i]` must become the globally-ordered id at slot i,
        // and `compact_from_global` is written by project_visible.
        let sorted_globals: Vec<u32> = order.iter().map(|&i| self.host_order[i as usize]).collect();
        self.ctx.queue.write_buffer(
            &self.scratch.global_from_compact,
            0,
            bytemuck::cast_slice(&sorted_globals),
        );

        let intersect_counts = read_u32s(
            &self.ctx.device,
            &self.ctx.queue,
            &self.scratch.intersect_counts,
            self.num_gaussians(),
        );
        // cum_tiles_hit is indexed by compact (sorted) position, so reorder the
        // per-gaussian counts into sorted order first.
        let mut counts_sorted = vec![0u32; nv];
        for (slot, &i) in order.iter().enumerate() {
            counts_sorted[slot] = intersect_counts[i as usize];
        }
        let mut cum = vec![0u32; nv];
        let mut running = 0u32;
        for i in 0..nv {
            running += counts_sorted[i];
            cum[i] = running;
        }
        self.ctx
            .queue
            .write_buffer(&self.scratch.cum_tiles_hit, 0, bytemuck::cast_slice(&cum));

        // ----- Pass 2: project_visible + map_gaussians. -----
        {
            let mut encoder = self
                .ctx
                .device
                .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("pv") });
            {
                let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor::default());
                dispatch(&mut pass, &self.visible, (nv as u32).div_ceil(256));
                dispatch(&mut pass, &self.map, (nv as u32).div_ceil(256));
            }
            self.ctx.queue.submit(Some(encoder.finish()));
        }

        // ----- Host tile-id sort over the isect list. -----
        let tile_ids = read_u32s(
            &self.ctx.device,
            &self.ctx.queue,
            &self.scratch.tile_id_from_isect,
            ni,
        );
        let isect_gids = read_u32s(
            &self.ctx.device,
            &self.ctx.queue,
            &self.scratch.compact_gid_from_isect,
            ni,
        );
        let mut isect_order: Vec<u32> = (0..ni as u32).collect();
        isect_order.sort_by_key(|&i| tile_ids[i as usize]);
        let sorted_tile_ids: Vec<u32> = isect_order.iter().map(|&i| tile_ids[i as usize]).collect();
        let sorted_isect_gids: Vec<u32> = isect_order
            .iter()
            .map(|&i| isect_gids[i as usize])
            .collect();
        self.ctx.queue.write_buffer(
            &self.scratch.tile_id_from_isect,
            0,
            bytemuck::cast_slice(&sorted_tile_ids),
        );
        self.ctx.queue.write_buffer(
            &self.scratch.compact_gid_from_isect,
            0,
            bytemuck::cast_slice(&sorted_isect_gids),
        );

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
                dispatch(&mut pass, &self.raster, num_tiles);
            }
            self.ctx.queue.submit(Some(encoder.finish()));
        }

        let rgb = read_f32s(
            &self.ctx.device,
            &self.ctx.queue,
            &self.out_img,
            (self.image_w as usize) * (self.image_h as usize) * 3,
        );
        let rgb = rgb.chunks_exact(3).map(|c| [c[0], c[1], c[2]]).collect();
        Image {
            width: self.image_w,
            height: self.image_h,
            rgb,
        }
    }
}

/// Read `count` `u32`s from `buffer` (blocking).
fn read_u32s(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    buffer: &wgpu::Buffer,
    count: usize,
) -> Vec<u32> {
    if count == 0 {
        return Vec::new();
    }
    let raw = read_bytes(device, queue, buffer, count * 4);
    bytemuck::cast_slice(&raw).to_vec()
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
