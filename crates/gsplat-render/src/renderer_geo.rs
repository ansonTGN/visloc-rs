//! Geometry channels: per-pixel alpha-weighted camera-space normal and depth.
//!
//! With [`Renderer::set_geometry`] on, each frame also runs `project_geo`
//! (per visible gaussian: the axis of its smallest scale, flipped to face the
//! camera, and its depth) and the geometry variants of `rasterize` /
//! `rasterize_backward` / `project_backward`, which carry those 4 values as
//! extra blended channels: `out_geo[px] = sum_i T_i a_i (n_i, z_i)` (no
//! background). A loss writes `dL/d out_geo` into [`Renderer::geo_buffers`]
//! and the backward pass chains it to the means and rotations, the way
//! 2DGS / PGSR-style depth-normal regularisers need. The plain variants are
//! compiled without any of this, so training without it costs nothing.

use super::{
    build_stage, dispatch, new_storage, raster_bindings, Binding, Renderer, Stage, RO, RW,
};
use crate::shaders;

pub(super) struct GeoState {
    project_geo: Stage,
    raster: Stage,
    /// Per compact id: normal xyz, depth.
    geo_splats: wgpu::Buffer,
    /// Per pixel: alpha-weighted normal xyz, depth.
    out_geo: wgpu::Buffer,
    /// Per pixel: dL/d out_geo (written by the caller's loss).
    d_geo: wgpu::Buffer,
    backward: Option<GeoBackward>,
}

struct GeoBackward {
    raster_bwd: Stage,
    project_bwd: Stage,
    /// Per compact id: dL/d(normal xyz, depth).
    geo_grads: wgpu::Buffer,
}

fn b(binding: u32, ty: wgpu::BufferBindingType, buffer: &wgpu::Buffer) -> Binding {
    Binding {
        binding,
        ty,
        buffer: buffer.clone(),
    }
}

impl Renderer {
    /// Render (and differentiate) the geometry channels too; see the module
    /// docs. Buffers are built on first use.
    pub fn set_geometry(&mut self, on: bool) {
        self.geo_enabled = on;
        if on && self.geo.is_none() {
            let dev = &self.ctx.device;
            let n = self.num_gaussians().max(1) as u64;
            let pixels = self.image_w as u64 * self.image_h as u64;
            let geo_splats = new_storage(dev, "geo_splats", n * 16);
            let out_geo = new_storage(dev, "out_geo", pixels * 16);
            let d_geo = new_storage(dev, "d_geo", pixels * 16);
            let project_geo = build_stage(
                dev,
                "project_geo",
                "project_geo",
                shaders::project_geo().source,
                &[
                    b(0, wgpu::BufferBindingType::Uniform, &self.proj_uniforms),
                    b(1, RO, &self.scene.transforms),
                    b(2, RO, &self.scratch.global_from_compact),
                    b(3, RW, &geo_splats),
                ],
            );
            let raster = build_stage(
                dev,
                "rasterize_geo",
                "rasterize",
                shaders::rasterize_variant(true).source,
                &self.geo_raster_bindings(&geo_splats, &out_geo),
            );
            self.geo = Some(GeoState {
                project_geo,
                raster,
                geo_splats,
                out_geo,
                d_geo,
                backward: None,
            });
        }
    }

    /// Whether the geometry channels are rendered.
    pub fn geometry(&self) -> bool {
        self.geo_enabled && self.geo.is_some()
    }

    /// `(out_geo, d_geo, final_t)`: the last frame's geometry channels (4
    /// floats per pixel), their gradient input, and the per-pixel final
    /// transmittance (opacity = 1 - T). `None` unless `set_geometry(true)`.
    pub fn geo_buffers(&self) -> Option<(&wgpu::Buffer, &wgpu::Buffer, &wgpu::Buffer)> {
        let g = self.geo.as_ref()?;
        Some((&g.out_geo, &g.d_geo, &self.residuals.final_t))
    }

    fn geo_raster_bindings(
        &self,
        geo_splats: &wgpu::Buffer,
        out_geo: &wgpu::Buffer,
    ) -> Vec<Binding> {
        let mut v = raster_bindings(
            &self.raster_uniforms,
            &self.scratch,
            &self.tile_offsets,
            &self.out_img,
            &self.residuals,
        );
        v.push(b(8, RO, geo_splats));
        v.push(b(9, RW, out_geo));
        v
    }

    /// Forward: `project_geo` after `project_visible`.
    pub(super) fn encode_project_geo(&self, pass: &mut wgpu::ComputePass<'_>, nv: u32) {
        if let Some(g) = self.geo.as_ref().filter(|_| self.geo_enabled) {
            dispatch(pass, &g.project_geo, nv.div_ceil(256));
        }
    }

    /// The rasterize stage for this frame (re-pointed at the current isect
    /// buffers, which may have grown).
    pub(super) fn raster_stage(&mut self) -> &Stage {
        if self.geo_enabled {
            if let Some(mut g) = self.geo.take() {
                let bindings = self.geo_raster_bindings(&g.geo_splats, &g.out_geo);
                g.raster
                    .rebind(&self.ctx.device, "rasterize_geo", &bindings);
                self.geo = Some(g);
                return &self.geo.as_ref().expect("set above").raster;
            }
        }
        &self.raster
    }

    /// Backward: build (first use) or re-point the geometry variants of
    /// both kernels. No-op when geometry is off.
    pub(super) fn prepare_geo_backward(
        &mut self,
        d_image: &wgpu::Buffer,
        screen_grads: &wgpu::Buffer,
        grads: [&wgpu::Buffer; 3],
    ) {
        if !self.geo_enabled {
            return;
        }
        let Some(mut g) = self.geo.take() else {
            return;
        };
        let dev = self.ctx.device.clone();
        let geo_grads = match &g.backward {
            Some(gb) => gb.geo_grads.clone(),
            None => new_storage(&dev, "geo_grads", self.num_gaussians().max(1) as u64 * 16),
        };
        let mut rb = super::backward::raster_bwd_bindings(self, d_image, screen_grads);
        rb.push(b(9, RO, &g.geo_splats));
        rb.push(b(10, RO, &g.d_geo));
        rb.push(b(11, RW, &geo_grads));
        let mut pb =
            super::backward::project_bwd_bindings(self, screen_grads, grads[0], grads[1], grads[2]);
        pb.push(b(9, RO, &geo_grads));
        match g.backward.as_mut() {
            Some(gb) => {
                gb.raster_bwd.rebind(&dev, "rasterize_backward_geo", &rb);
                gb.project_bwd.rebind(&dev, "project_backward_geo", &pb);
            }
            None => {
                g.backward = Some(GeoBackward {
                    raster_bwd: build_stage(
                        &dev,
                        "rasterize_backward_geo",
                        "rasterize_backward",
                        shaders::rasterize_backward_variant(true).source,
                        &rb,
                    ),
                    project_bwd: build_stage(
                        &dev,
                        "project_backward_geo",
                        "project_backward",
                        shaders::project_backward_variant(true).source,
                        &pb,
                    ),
                    geo_grads,
                });
            }
        }
        self.geo = Some(g);
    }

    /// `(rasterize_backward, project_backward, geo_grads)` of the geometry
    /// variant, when geometry is on and prepared.
    pub(super) fn geo_backward(&self) -> Option<(&Stage, &Stage, &wgpu::Buffer)> {
        if !self.geo_enabled {
            return None;
        }
        let gb = self.geo.as_ref()?.backward.as_ref()?;
        Some((&gb.raster_bwd, &gb.project_bwd, &gb.geo_grads))
    }
}

impl Renderer {
    /// The last frame's geometry channels, read back (normal xyz, depth per
    /// pixel, alpha-weighted). `None` unless `set_geometry(true)`.
    pub fn read_geo(&self) -> Option<Vec<[f32; 4]>> {
        let g = self.geo.as_ref()?;
        let n = self.image_w as usize * self.image_h as usize;
        let raw = super::read_f32s(&self.ctx.device, &self.ctx.queue, &g.out_geo, n * 4);
        Some(
            raw.chunks_exact(4)
                .map(|c| [c[0], c[1], c[2], c[3]])
                .collect(),
        )
    }

    /// [`Renderer::backward`] with a gradient on the geometry channels too
    /// (needs `set_geometry(true)` before the frame was rendered).
    pub fn backward_geo(
        &mut self,
        d_image: &[[f32; 3]],
        d_geo: &[[f32; 4]],
    ) -> Result<super::ParamGrads, crate::gpu::GpuError> {
        let g = self
            .geo
            .as_ref()
            .ok_or(crate::gpu::GpuError::MissingFeature("set_geometry(true)"))?;
        self.ctx
            .queue
            .write_buffer(&g.d_geo, 0, bytemuck::cast_slice(d_geo));
        self.backward(d_image)
    }
}
