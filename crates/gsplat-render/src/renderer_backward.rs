//! Backward pass of the forward rasterizer (M1 of the trainer plan).
//!
//! Differentiates the **last rendered frame**: after [`Renderer::render`], the
//! screen-space gradients of `L = sum_px dot(d_image[px], C[px])` are computed
//! per gaussian by `rasterize_backward` (per-isect, no global atomics) and
//! `grad_reduce` (deterministic per-gaussian sum). Needs the `SUBGROUP` device
//! feature.

use super::{build_stage, new_storage, read_bytes, Binding, Renderer, Stage, RO, RW};
use crate::gpu::GpuError;
use crate::shaders;

/// Floats per screen-space gradient record: du, dv, dA, dB, dC, dopacity,
/// dr, dg, db (conic `[A, B, C]` as in `projected_splats`).
pub const SCREEN_GRAD_FLOATS: usize = 9;

/// Parameter gradients of one backward pass, in the forward input layouts
/// (see [`crate::packing::PackedScene`]), for every gaussian in scene order.
#[derive(Debug, Clone)]
pub struct ParamGrads {
    /// `10` per gaussian: mean xyz, quaternion wxyz (un-normalised), log-scale xyz.
    pub transforms: Vec<f32>,
    /// Opacity logit, one per gaussian.
    pub opacity: Vec<f32>,
    /// SH block per gaussian: 3 DC values then channel-major rest.
    pub sh: Vec<f32>,
}

pub(super) struct BackwardState {
    raster_bwd: Stage,
    reduce: Stage,
    project_bwd: Stage,
    d_image: wgpu::Buffer,
    isect_grads: wgpu::Buffer,
    screen_grads: wgpu::Buffer,
    grad_transforms: wgpu::Buffer,
    grad_opacity: wgpu::Buffer,
    grad_sh: wgpu::Buffer,
    /// Isect capacity `isect_grads` (and the bind groups) were built for.
    isect_capacity: usize,
}

fn raster_bwd_bindings(
    r: &Renderer,
    d_image: &wgpu::Buffer,
    isect_grads: &wgpu::Buffer,
) -> Vec<Binding> {
    let b = |binding, ty, buffer: &wgpu::Buffer| Binding {
        binding,
        ty,
        buffer: buffer.clone(),
    };
    vec![
        b(0, wgpu::BufferBindingType::Uniform, &r.raster_uniforms),
        b(1, RO, &r.scratch.projected_splats),
        b(2, RO, &r.scratch.compact_sorted),
        b(3, RO, &r.tile_offsets),
        b(4, RO, &r.scratch.isect_id),
        b(5, RO, &r.residuals.final_t),
        b(6, RO, &r.residuals.last_idx),
        b(7, RO, d_image),
        b(8, RW, isect_grads),
    ]
}

fn reduce_bindings(
    r: &Renderer,
    isect_grads: &wgpu::Buffer,
    screen_grads: &wgpu::Buffer,
) -> Vec<Binding> {
    let b = |binding, ty, buffer: &wgpu::Buffer| Binding {
        binding,
        ty,
        buffer: buffer.clone(),
    };
    vec![
        b(0, wgpu::BufferBindingType::Uniform, &r.proj_uniforms),
        b(1, RO, &r.scratch.cum_tiles_hit),
        b(2, RO, isect_grads),
        b(3, RW, screen_grads),
    ]
}

fn project_bwd_bindings(
    r: &Renderer,
    screen_grads: &wgpu::Buffer,
    grad_transforms: &wgpu::Buffer,
    grad_opacity: &wgpu::Buffer,
    grad_sh: &wgpu::Buffer,
) -> Vec<Binding> {
    let b = |binding, ty, buffer: &wgpu::Buffer| Binding {
        binding,
        ty,
        buffer: buffer.clone(),
    };
    vec![
        b(0, wgpu::BufferBindingType::Uniform, &r.proj_uniforms),
        b(1, RO, &r.scene.transforms),
        b(2, RO, &r.scene.opacity),
        b(3, RO, &r.scene.sh),
        b(4, RO, &r.scratch.global_from_compact),
        b(5, RO, screen_grads),
        b(6, RW, grad_transforms),
        b(7, RW, grad_opacity),
        b(8, RW, grad_sh),
    ]
}

impl Renderer {
    /// Build (or, after isect-buffer growth, re-point) the backward state.
    fn ensure_backward(&mut self) -> Result<(), GpuError> {
        if !self.ctx.features.contains(wgpu::Features::SUBGROUP) {
            return Err(GpuError::MissingFeature("SUBGROUP"));
        }
        let cap = self.scratch.max_isects;
        let isect_bytes = (cap * SCREEN_GRAD_FLOATS * 4) as u64;
        match self.backward.take() {
            Some(mut st) => {
                if st.isect_capacity != cap {
                    let dev = &self.ctx.device;
                    st.isect_grads = new_storage(dev, "isect_grads", isect_bytes);
                    st.isect_capacity = cap;
                    let rb = raster_bwd_bindings(self, &st.d_image, &st.isect_grads);
                    st.raster_bwd.rebind(dev, "rasterize_backward", &rb);
                    let red = reduce_bindings(self, &st.isect_grads, &st.screen_grads);
                    st.reduce.rebind(dev, "grad_reduce", &red);
                } else {
                    // The forward may have re-pointed its own buffers; rebind
                    // cheaply every frame so the backward never reads stale ones.
                    let dev = &self.ctx.device;
                    let rb = raster_bwd_bindings(self, &st.d_image, &st.isect_grads);
                    st.raster_bwd.rebind(dev, "rasterize_backward", &rb);
                }
                self.backward = Some(st);
            }
            None => {
                let dev = &self.ctx.device;
                let pixels = self.image_w as u64 * self.image_h as u64;
                let d_image = new_storage(dev, "d_image", pixels * 3 * 4);
                let isect_grads = new_storage(dev, "isect_grads", isect_bytes);
                let screen_grads = new_storage(
                    dev,
                    "screen_grads",
                    (self.num_gaussians().max(1) * SCREEN_GRAD_FLOATS * 4) as u64,
                );
                let raster_bwd = build_stage(
                    dev,
                    "rasterize_backward",
                    "rasterize_backward",
                    shaders::rasterize_backward().source,
                    &raster_bwd_bindings(self, &d_image, &isect_grads),
                );
                let reduce = build_stage(
                    dev,
                    "grad_reduce",
                    "grad_reduce",
                    shaders::grad_reduce().source,
                    &reduce_bindings(self, &isect_grads, &screen_grads),
                );
                let n = self.num_gaussians().max(1) as u64;
                let grad_transforms = new_storage(dev, "grad_transforms", n * 10 * 4);
                let grad_opacity = new_storage(dev, "grad_opacity", n * 4);
                let grad_sh = new_storage(
                    dev,
                    "grad_sh",
                    (self.scene.packed.sh.len().max(1) * 4) as u64,
                );
                let project_bwd = build_stage(
                    dev,
                    "project_backward",
                    "project_backward",
                    shaders::project_backward().source,
                    &project_bwd_bindings(
                        self,
                        &screen_grads,
                        &grad_transforms,
                        &grad_opacity,
                        &grad_sh,
                    ),
                );
                self.backward = Some(BackwardState {
                    raster_bwd,
                    reduce,
                    project_bwd,
                    d_image,
                    isect_grads,
                    screen_grads,
                    grad_transforms,
                    grad_opacity,
                    grad_sh,
                    isect_capacity: cap,
                });
            }
        }
        Ok(())
    }

    /// Run the backward kernels for the last rendered frame, uploading
    /// `d_image` first unless it is `None` (already written on the device, e.g.
    /// by a loss kernel into [`Renderer::d_image_buffer`]).
    fn run_backward(&mut self, d_image: Option<&[[f32; 3]]>) -> Result<(), GpuError> {
        self.ensure_backward()?;
        let frame = self.last_frame;
        let st = self.backward.as_ref().expect("ensured above");
        let dev = &self.ctx.device;
        let queue = &self.ctx.queue;
        if let Some(d_image) = d_image {
            assert_eq!(
                d_image.len(),
                (self.image_w * self.image_h) as usize,
                "d_image must match the render size"
            );
            queue.write_buffer(&st.d_image, 0, bytemuck::cast_slice(d_image));
        }
        let mut encoder = dev.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("backward"),
        });
        // Gaussians not visible this frame keep zero parameter gradients.
        encoder.clear_buffer(&st.grad_transforms, 0, None);
        encoder.clear_buffer(&st.grad_opacity, 0, None);
        encoder.clear_buffer(&st.grad_sh, 0, None);
        if frame.ni > 0 {
            // Slots of isects past a tile's last blended entry are never
            // written; clear so grad_reduce sums zeros for them.
            encoder.clear_buffer(
                &st.isect_grads,
                0,
                Some((frame.ni * SCREEN_GRAD_FLOATS * 4) as u64),
            );
        }
        if frame.nv > 0 {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor::default());
            if frame.ni > 0 {
                super::dispatch(&mut pass, &st.raster_bwd, frame.num_tiles);
            }
            let groups = (frame.nv as u32).div_ceil(256);
            super::dispatch(&mut pass, &st.reduce, groups);
            super::dispatch(&mut pass, &st.project_bwd, groups);
        }
        queue.submit(Some(encoder.finish()));
        Ok(())
    }

    /// Screen-space gradients of the last rendered frame.
    ///
    /// `d_image` is `dL/dC` per pixel (row-major RGB, the rendered size).
    /// Returns one [`SCREEN_GRAD_FLOATS`]-float record per gaussian (scene
    /// order; zeros for gaussians not visible in that frame): gradients of
    /// the projected mean `(u, v)`, conic `[A, B, C]`, opacity (the value, not
    /// its logit) and colour (after the `max(0)` clamp).
    pub fn backward_screen(
        &mut self,
        d_image: &[[f32; 3]],
    ) -> Result<Vec<[f32; SCREEN_GRAD_FLOATS]>, GpuError> {
        self.run_backward(Some(d_image))?;
        let frame = self.last_frame;
        let mut out = vec![[0.0f32; SCREEN_GRAD_FLOATS]; self.num_gaussians()];
        if frame.nv == 0 {
            return Ok(out);
        }
        let st = self.backward.as_ref().expect("ensured above");
        let dev = &self.ctx.device;
        let queue = &self.ctx.queue;
        let grads = read_bytes(
            dev,
            queue,
            &st.screen_grads,
            frame.nv * SCREEN_GRAD_FLOATS * 4,
        );
        let grads: &[f32] = bytemuck::cast_slice(&grads);
        let gfc = read_bytes(dev, queue, &self.scratch.global_from_compact, frame.nv * 4);
        let gfc: &[u32] = bytemuck::cast_slice(&gfc);
        for (c, &g) in gfc.iter().enumerate() {
            if let Some(slot) = out.get_mut(g as usize) {
                slot.copy_from_slice(&grads[c * SCREEN_GRAD_FLOATS..(c + 1) * SCREEN_GRAD_FLOATS]);
            }
        }
        Ok(out)
    }

    /// Parameter gradients of `L = sum_px dot(d_image[px], C[px])` for the last
    /// rendered frame (see [`ParamGrads`]), read back to the host.
    pub fn backward(&mut self, d_image: &[[f32; 3]]) -> Result<ParamGrads, GpuError> {
        self.run_backward(Some(d_image))?;
        let st = self.backward.as_ref().expect("ensured above");
        let dev = &self.ctx.device;
        let queue = &self.ctx.queue;
        let n = self.num_gaussians();
        let read = |buf: &wgpu::Buffer, len: usize| -> Vec<f32> {
            bytemuck::cast_slice(&read_bytes(dev, queue, buf, len * 4)).to_vec()
        };
        Ok(ParamGrads {
            transforms: read(&st.grad_transforms, n * 10),
            opacity: read(&st.grad_opacity, n),
            sh: read(&st.grad_sh, self.scene.packed.sh.len()),
        })
    }
    /// Run the backward pass with `dL/dC` already on the device in
    /// [`Renderer::d_image_buffer`]; gradients stay on the device in
    /// [`Renderer::grad_buffers`] (for an on-device optimizer).
    pub fn backward_on_device(&mut self) -> Result<(), GpuError> {
        self.run_backward(None)
    }

    /// The scene's parameter buffers (forward inputs; an optimizer updates
    /// them in place).
    pub fn param_buffers(&self) -> DeviceParams<'_> {
        DeviceParams {
            transforms: &self.scene.transforms,
            opacity: &self.scene.opacity,
            sh: &self.scene.sh,
        }
    }

    /// Gradient buffers written by the backward pass (same layouts as
    /// [`Renderer::param_buffers`]).
    pub fn grad_buffers(&mut self) -> Result<DeviceParams<'_>, GpuError> {
        self.ensure_backward()?;
        let st = self.backward.as_ref().expect("ensured above");
        Ok(DeviceParams {
            transforms: &st.grad_transforms,
            opacity: &st.grad_opacity,
            sh: &st.grad_sh,
        })
    }

    /// `dL/dC` input of the backward pass: row-major RGB `f32`, render size.
    pub fn d_image_buffer(&mut self) -> Result<&wgpu::Buffer, GpuError> {
        self.ensure_backward()?;
        Ok(&self.backward.as_ref().expect("ensured above").d_image)
    }

    /// The rendered image of the last frame: row-major RGB `f32` (valid after
    /// [`Renderer::render`] even with `set_skip_readback(true)`).
    pub fn output_buffer(&self) -> &wgpu::Buffer {
        &self.out_img
    }

    /// Per visible gaussian of the last frame (compact order): its scene index
    /// and the screen-space gradient record, as device buffers, plus the
    /// visible count. For densification statistics.
    pub fn screen_grad_buffers(
        &mut self,
    ) -> Result<(&wgpu::Buffer, &wgpu::Buffer, usize), GpuError> {
        self.ensure_backward()?;
        let st = self.backward.as_ref().expect("ensured above");
        Ok((
            &self.scratch.global_from_compact,
            &st.screen_grads,
            self.last_frame.nv,
        ))
    }

    /// Read the current parameters back into a [`crate::packing::PackedScene`]
    /// layout: `(transforms, opacity, sh)`.
    pub fn read_params(&self) -> (Vec<f32>, Vec<f32>, Vec<f32>) {
        let dev = &self.ctx.device;
        let queue = &self.ctx.queue;
        let p = &self.scene.packed;
        let read = |buf: &wgpu::Buffer, len: usize| -> Vec<f32> {
            bytemuck::cast_slice(&read_bytes(dev, queue, buf, len * 4)).to_vec()
        };
        (
            read(&self.scene.transforms, p.transforms.len()),
            read(&self.scene.opacity, p.opacity.len()),
            read(&self.scene.sh, p.sh.len()),
        )
    }
}

/// Borrowed device buffers of one parameter set (see [`ParamGrads`] layouts).
pub struct DeviceParams<'a> {
    pub transforms: &'a wgpu::Buffer,
    pub opacity: &'a wgpu::Buffer,
    pub sh: &'a wgpu::Buffer,
}
