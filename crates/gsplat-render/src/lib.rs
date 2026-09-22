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
//! # Feature flags
//!
//! The GPU path is behind the **`gpu`** feature (off by default). wgpu 30 pulls
//! `naga`, whose `indexmap` dependency needs edition2024 (Rust >= 1.85), so
//! enabling `gpu` requires a current toolchain. The pure host-side packing and
//! camera math always compile, so the default build keeps the workspace MSRV.
//!
//! # Example
//!
//! ```no_run
//! # #[cfg(feature = "gpu")] {
//! use visloc_gsplat_core::splat;
//! use visloc_gsplat_render::{GpuContext, Renderer};
//!
//! let scene = splat::load_splat("scene.splat")?;
//! let ctx = GpuContext::new()?;
//! let mut renderer = Renderer::new(ctx, &scene, 640, 480)?;
//! # }
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```

pub mod packing;

#[cfg(feature = "gpu")]
pub mod gpu;
#[cfg(feature = "gpu")]
pub mod renderer;
#[cfg(feature = "gpu")]
pub mod scan;
#[cfg(feature = "gpu")]
pub mod shaders;
#[cfg(feature = "gpu")]
pub mod sort;
#[cfg(feature = "gpu")]
pub mod uniforms;

pub use packing::{PackedScene, TRANSFORM_FLOATS};

#[cfg(feature = "gpu")]
pub use gpu::{try_context, GpuContext, GpuError};
#[cfg(feature = "gpu")]
pub use renderer::{GpuScene, Renderer};
#[cfg(feature = "gpu")]
pub use sort::{RadixParams, RadixSorter, SortBuffers};
#[cfg(feature = "gpu")]
pub use uniforms::{tile_bounds, ProjectUniforms, RasterUniforms, TILE_SIZE, TILE_WIDTH};

#[cfg(all(test, not(feature = "gpu")))]
mod tests {
    // The host-side packing is pure and testable without a GPU or wgpu.
    use super::*;
    use nalgebra::{Quaternion, Vector3};
    use visloc_gsplat_core::gaussian::{Gaussian, Scene};

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
}

#[cfg(all(test, feature = "gpu"))]
mod gpu_tests {
    use nalgebra::{Matrix3, Quaternion, Vector3};
    use visloc_gsplat_core::camera::{CameraView, PinholeCamera};
    use visloc_gsplat_core::cpu_render;
    use visloc_gsplat_core::gaussian::{Gaussian, Scene};

    use crate::{try_context, RadixSorter, Renderer};

    /// Blocking read of `count` `u32`s from `buffer`.
    fn read_back_u32(ctx: &crate::GpuContext, buffer: &wgpu::Buffer, count: usize) -> Vec<u32> {
        let staging = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("test_staging"),
            size: (count * 4) as u64,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let mut encoder = ctx
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        encoder.copy_buffer_to_buffer(buffer, 0, &staging, 0, (count * 4) as u64);
        ctx.queue.submit(Some(encoder.finish()));
        let slice = staging.slice(..);
        let (tx, rx) = std::sync::mpsc::channel();
        slice.map_async(wgpu::MapMode::Read, move |r| {
            let _ = tx.send(r);
        });
        ctx.device.poll(wgpu::PollType::wait_indefinitely()).ok();
        let _ = rx.recv();
        let data = slice.get_mapped_range().expect("map");
        let out = bytemuck::cast_slice(&data).to_vec();
        drop(data);
        staging.unmap();
        out
    }

    fn front_camera() -> CameraView {
        CameraView::new(
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
    fn gpu_radix_sort_is_correct_and_stable() {
        let Some(ctx) = try_context() else {
            eprintln!("skipping gpu_radix_sort_is_correct_and_stable: no GPU adapter");
            return;
        };
        // Keys with many duplicates so stability is observable: values are the
        // original indices (mod small) and must be non-decreasing within a key.
        let n = 5000usize;
        let mut state = 0x1234_5678u32;
        let mut keys = Vec::with_capacity(n);
        for _ in 0..n {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            keys.push(state % 17);
        }
        let values: Vec<u32> = (0..n as u32).collect();

        let sorter = RadixSorter::new(&ctx.device, n);
        let pairs = sorter.allocate(&ctx.device, "test");
        ctx.queue
            .write_buffer(&pairs[0].keys, 0, bytemuck::cast_slice(&keys));
        ctx.queue
            .write_buffer(&pairs[0].values, 0, bytemuck::cast_slice(&values));
        sorter.sort(&ctx.device, &ctx.queue, &pairs, n);

        let gpu_keys: Vec<u32> = read_back_u32(&ctx, &pairs[0].keys, n);
        let gpu_values: Vec<u32> = read_back_u32(&ctx, &pairs[0].values, n);

        // Reference: stable host sort.
        let mut order: Vec<usize> = (0..n).collect();
        order.sort_by_key(|&i| keys[i]);
        let expect_keys: Vec<u32> = order.iter().map(|&i| keys[i]).collect();
        let expect_values: Vec<u32> = order.iter().map(|&i| values[i]).collect();

        assert_eq!(gpu_keys, expect_keys, "sorted keys differ from host sort");
        assert_eq!(gpu_values, expect_values, "sort is not stable");
    }

    #[test]
    fn gpu_matches_cpu_reference() {
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
