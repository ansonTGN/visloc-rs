//! GPU (wgpu) forward renderer for 3D Gaussian Splatting — stage 1 of the
//! visloc-rs 3DGS effort.
//!
//! This crate is the GPU counterpart of `visloc-gsplat-core`'s CPU reference
//! rasterizer. It consumes the same [`visloc_gsplat_core::gaussian::Scene`] and
//! [`visloc_gsplat_core::camera::CameraView`] and produces the same linear-RGB
//! [`visloc_gsplat_core::cpu_render::Image`], so the two can be compared
//! directly (see the `gpu_matches_cpu_reference` test).
//!
//! Scope (stage 1): the whole forward pass runs on the GPU
//! (`project_forward` → `project_visible` → `map_gaussians` → `tile_offsets` →
//! `rasterize`). The two per-frame sorts currently run on the host; a
//! subgroup-free device-side radix sort is a planned follow-up. All shaders use
//! only baseline WebGPU features (no subgroups, only `u32` atomics).
//!
//! # Example
//!
//! ```no_run
//! use visloc_gsplat_core::splat;
//! use visloc_gsplat_render::{GpuContext, Renderer};
//!
//! let scene = splat::load_splat("scene.splat")?;
//! let ctx = GpuContext::new()?;
//! let mut renderer = Renderer::new(ctx, &scene, 640, 480)?;
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```

pub mod gpu;
pub mod packing;
pub mod renderer;
pub mod shaders;
pub mod uniforms;

pub use gpu::{try_context, GpuContext, GpuError};
pub use packing::{PackedScene, TRANSFORM_FLOATS};
pub use renderer::{GpuScene, Renderer};
pub use uniforms::{tile_bounds, ProjectUniforms, RasterUniforms, TILE_SIZE, TILE_WIDTH};

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Matrix3, Quaternion, Vector3};
    use visloc_gsplat_core::camera::PinholeCamera;
    use visloc_gsplat_core::cpu_render;
    use visloc_gsplat_core::gaussian::{Gaussian, Scene};

    fn front_camera() -> visloc_gsplat_core::camera::CameraView {
        visloc_gsplat_core::camera::CameraView::new(
            Matrix3::identity(),
            Vector3::zeros(),
            PinholeCamera::new(64, 64, 50.0, 50.0, 32.0, 32.0),
        )
    }

    fn solid_gaussian(mean: Vector3<f32>, scale: f32, color: [f32; 3]) -> Gaussian {
        Gaussian {
            mean,
            scale_log: Vector3::new(scale.ln(), scale.ln(), scale.ln()),
            rotation: Quaternion::new(1.0, 0.0, 0.0, 0.0),
            opacity_logit: 10.0,
            sh_dc: [
                (color[0] - 0.5) / visloc_gsplat_core::sh::SH_C0,
                (color[1] - 0.5) / visloc_gsplat_core::sh::SH_C0,
                (color[2] - 0.5) / visloc_gsplat_core::sh::SH_C0,
            ],
            sh_rest: Vec::new(),
            sh_degree: 0,
        }
    }

    #[test]
    fn packing_matches_cpu_conventions() {
        let scene = Scene::new(
            vec![solid_gaussian(
                Vector3::new(0.0, 0.0, 5.0),
                0.2,
                [1.0, 0.0, 0.0],
            )],
            0,
        );
        let p = PackedScene::from_scene(&scene);
        assert_eq!(p.num_gaussians, 1);
        assert_eq!(p.sh_coeffs_per_channel, 1);
    }

    #[test]
    fn context_only_creates() {
        let Some(ctx) = try_context() else {
            return;
        };
        assert!(!ctx.adapter_info.name.is_empty());
    }

    #[test]
    fn gpu_single_gaussian_matches_cpu() {
        let Some(ctx) = try_context() else {
            eprintln!("skipping gpu_single_gaussian_matches_cpu: no GPU adapter");
            return;
        };
        let scene = Scene::new(
            vec![solid_gaussian(
                Vector3::new(0.0, 0.0, 5.0),
                0.3,
                [0.9, 0.1, 0.1],
            )],
            0,
        );
        let view = front_camera();
        let bg = [0.0, 0.0, 0.0];
        let cpu = cpu_render::render(&scene, &view, bg);
        let mut renderer = Renderer::new(ctx, &scene, 64, 64).expect("renderer");
        let gpu = renderer.render(&view, bg);
        let mut max_err = 0.0f32;
        for (a, b) in cpu.rgb.iter().zip(gpu.rgb.iter()) {
            for c in 0..3 {
                max_err = max_err.max((a[c] - b[c]).abs());
            }
        }
        assert!(max_err < 0.02, "single gaussian max abs error {max_err}");
    }

    #[test]
    fn gpu_matches_cpu_reference() {
        // Skip gracefully on machines with no adapter.
        let Some(ctx) = try_context() else {
            eprintln!("skipping gpu_matches_cpu_reference: no GPU adapter");
            return;
        };
        let gaussians = vec![
            solid_gaussian(Vector3::new(0.0, 0.0, 5.0), 0.3, [0.9, 0.1, 0.1]),
            solid_gaussian(Vector3::new(0.5, -0.3, 5.5), 0.4, [0.1, 0.8, 0.2]),
            solid_gaussian(Vector3::new(-0.4, 0.4, 6.0), 0.5, [0.2, 0.3, 0.9]),
        ];
        let scene = Scene::new(gaussians, 0);
        let view = front_camera();
        let bg = [0.05, 0.06, 0.07];

        let cpu = cpu_render::render(&scene, &view, bg);
        let mut renderer = Renderer::new(ctx, &scene, 64, 64).expect("renderer");
        let gpu = renderer.render(&view, bg);

        let mut max_err = 0.0f32;
        let mut sum_err = 0.0f64;
        for (a, b) in cpu.rgb.iter().zip(gpu.rgb.iter()) {
            for c in 0..3 {
                let e = (a[c] - b[c]).abs();
                max_err = max_err.max(e);
                sum_err += e as f64;
            }
        }
        let mean_err = sum_err / (cpu.rgb.len() * 3) as f64;
        assert!(mean_err < 0.002, "mean abs error {mean_err} too high");
        assert!(max_err < 0.05, "max abs error {max_err} too high");
    }
}
