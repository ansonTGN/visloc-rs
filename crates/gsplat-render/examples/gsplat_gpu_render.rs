//! Render a 3DGS scene with the GPU forward rasterizer and write a PNG.
//!
//! ```text
//! cargo run -p visloc-gsplat-render --example gsplat_gpu_render -- \
//!     --splat docs/euroc_splat/euroc_v101.splat --out frame.png --width 640 --height 480
//! ```
//!
//! By default the camera auto-frames the whole scene from outside, which packs
//! most gaussians into a few tiles. `--pose-cw qw,qx,qy,qz,tx,ty,tz` instead
//! renders from a real camera pose (world-to-camera, COLMAP `images.txt`
//! convention) with EuRoC cam0-like intrinsics (fx = fy = 0.61 * width).

use std::path::PathBuf;

use nalgebra::{Matrix3, Quaternion, UnitQuaternion, Vector3};
use visloc_gsplat_core::camera::{CameraView, PinholeCamera};
use visloc_gsplat_core::gaussian::{Gaussian, Scene};
use visloc_gsplat_core::splat;
use visloc_gsplat_render::{GpuContext, Renderer};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let mut splat_path: Option<PathBuf> = None;
    let mut out: Option<PathBuf> = None;
    let mut width = 640u32;
    let mut height = 480u32;
    let mut pose_cw: Option<Vec<f32>> = None;
    while let Some(a) = args.next() {
        match a.as_str() {
            "--splat" => splat_path = args.next().map(PathBuf::from),
            "--out" => out = args.next().map(PathBuf::from),
            "--width" => width = args.next().and_then(|v| v.parse().ok()).unwrap_or(width),
            "--height" => height = args.next().and_then(|v| v.parse().ok()).unwrap_or(height),
            "--pose-cw" => {
                let v: Vec<f32> = args
                    .next()
                    .ok_or("--pose-cw needs qw,qx,qy,qz,tx,ty,tz")?
                    .split(',')
                    .map(|x| x.trim().parse::<f32>())
                    .collect::<Result<_, _>>()?;
                if v.len() != 7 {
                    return Err("--pose-cw needs 7 comma-separated values".into());
                }
                pose_cw = Some(v);
            }
            other => return Err(format!("unknown argument {other}").into()),
        }
    }
    let splat_path = splat_path.ok_or("--splat <path> is required")?;
    let out = out.unwrap_or_else(|| PathBuf::from("gsplat_gpu_render.png"));

    let scene: Scene = splat::load_splat(&splat_path)?;
    println!(
        "loaded {} gaussians (degree {})",
        scene.len(),
        scene.sh_degree
    );

    // Trained `.splat` files often carry a long "floater" tail (positions up to
    // ~1e5 and linear scales up to ~1e8) that is invisible from any real camera
    // but covers the frame from an arbitrary viewpoint. Prune to the dense core
    // so an auto-framed view is representative.
    let scene = prune_outliers(&scene, 0.995);
    println!("kept {} gaussians after outlier prune", scene.len());

    // Frame the scene from a robustly estimated viewpoint (median centre,
    // 75th-percentile extent) so floaters do not dominate the view.
    let view = match &pose_cw {
        Some(p) => pose_view(p, width, height),
        None => frame_view(&scene, width, height),
    };

    if std::env::var("GSPLAT_CPU_COMPARE").is_ok() {
        // The CPU reference is O(gaussians * footprint); validate it against the
        // GPU on a spatial subsample so the comparison finishes quickly.
        let step = (scene.len() / 4000).max(1);
        let sub: Vec<Gaussian> = scene.gaussians.iter().step_by(step).cloned().collect();
        let sub_scene = Scene::new(sub, scene.sh_degree);
        eprintln!(
            "CPU reference on {} gaussians (1/{step} subsample)…",
            sub_scene.len()
        );
        let cpu = visloc_gsplat_core::cpu_render::render(&sub_scene, &view, [0.0, 0.0, 0.0]);
        let mut gpu_renderer =
            Renderer::new(GpuContext::new().unwrap(), &sub_scene, width, height)?;
        let gpu = gpu_renderer.render(&view, [0.0, 0.0, 0.0]);
        image::save_buffer(
            "gsplat_cpu_reference.png",
            &cpu.to_rgb8(),
            width,
            height,
            image::ColorType::Rgb8,
        )?;
        image::save_buffer(
            "gsplat_gpu_subsample.png",
            &gpu.to_rgb8(),
            width,
            height,
            image::ColorType::Rgb8,
        )?;
        let mut max_err = 0.0f32;
        let mut sum_err = 0.0f64;
        for (a, b) in cpu.rgb.iter().zip(gpu.rgb.iter()) {
            for c in 0..3 {
                let e = (a[c] - b[c]).abs();
                max_err = max_err.max(e);
                sum_err += e as f64;
            }
        }
        let mut big = 0usize;
        for (a, b) in cpu.rgb.iter().zip(gpu.rgb.iter()) {
            if (0..3).any(|c| (a[c] - b[c]).abs() > 0.1) {
                big += 1;
            }
        }
        eprintln!(
            "real-data subsample parity: max={max_err:.5} mean={:.6} pixels>0.1={big}/{}",
            sum_err / (cpu.rgb.len() * 3) as f64,
            cpu.rgb.len()
        );
    }

    let ctx = GpuContext::new()?;
    println!("using adapter: {}", ctx.adapter_info.name);
    let mut renderer = Renderer::new(ctx, &scene, width, height)?;
    let gaussians = scene.len();

    // Time the full path (with the output-image readback), which is what an
    // offline PNG render costs.
    let _ = renderer.render(&view, [0.0, 0.0, 0.0]);
    let iters = 20;
    let start = std::time::Instant::now();
    let mut image = renderer.render(&view, [0.0, 0.0, 0.0]);
    for _ in 1..iters {
        image = renderer.render(&view, [0.0, 0.0, 0.0]);
    }
    let per_frame = start.elapsed().as_secs_f64() / iters as f64;
    println!(
        "render+readback: {:.1} ms/frame at {width}x{height} ({} gaussians, {:.1} M splats/s)",
        per_frame * 1e3,
        gaussians,
        gaussians as f64 / per_frame / 1e6
    );

    // Time the GPU-only path (no image readback): this is the per-frame cost a
    // native viewer would pay.
    renderer.set_skip_readback(true);
    let _ = renderer.render(&view, [0.0, 0.0, 0.0]);
    let start = std::time::Instant::now();
    for _ in 0..iters {
        let _ = renderer.render(&view, [0.0, 0.0, 0.0]);
    }
    let gpu_frame = start.elapsed().as_secs_f64() / iters as f64;
    renderer.set_skip_readback(false);
    println!(
        "gpu-only:       {:.1} ms/frame at {width}x{height} ({} gaussians, {:.1} M splats/s)",
        gpu_frame * 1e3,
        gaussians,
        gaussians as f64 / gpu_frame / 1e6
    );

    let bytes = image.to_rgb8();
    image::save_buffer(&out, &bytes, width, height, image::ColorType::Rgb8)?;
    println!("wrote {}", out.display());
    Ok(())
}

/// Camera at a world-to-camera pose `[qw, qx, qy, qz, tx, ty, tz]`.
fn pose_view(p: &[f32], width: u32, height: u32) -> CameraView {
    let q = UnitQuaternion::from_quaternion(Quaternion::new(p[0], p[1], p[2], p[3]));
    let f = 0.61 * width as f32;
    CameraView::new(
        *q.to_rotation_matrix().matrix(),
        Vector3::new(p[4], p[5], p[6]),
        PinholeCamera::new(width, height, f, f, width as f32 * 0.5, height as f32 * 0.5),
    )
}

fn frame_view(scene: &Scene, width: u32, height: u32) -> CameraView {
    let center = robust_center(scene);
    let extent = robust_extent(scene, center).max(1e-3);
    eprintln!("scene center {center:?}, robust extent {extent:.3}");
    let fx = 0.5 * width as f32;
    let fy = 0.5 * height as f32;
    let half_fov = (0.5 * fx / (0.5 * width as f32)).atan();
    let distance = extent / half_fov.tan();
    // The camera looks down its local +Z, so place it on the -Z side of the
    // scene centre to see the scene in front of it.
    let eye = center - Vector3::new(0.0, 0.0, distance);
    CameraView::new(
        Matrix3::identity(),
        -eye,
        PinholeCamera::new(
            width,
            height,
            fx,
            fy,
            width as f32 * 0.5,
            height as f32 * 0.5,
        ),
    )
}

/// Keep gaussians whose mean is within the `keep`-quantile radius of the median
/// centre and whose linear scale is within the `keep`-quantile scale.
fn prune_outliers(scene: &Scene, keep: f32) -> Scene {
    if scene.is_empty() {
        return scene.clone();
    }
    let center = robust_center(scene);
    let mut dists: Vec<f32> = scene
        .gaussians
        .iter()
        .map(|g| (g.mean - center).norm())
        .collect();
    dists.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let radius = dists[((dists.len() as f32 * keep) as usize).min(dists.len() - 1)];

    let mut scales: Vec<f32> = scene.gaussians.iter().map(|g| g.scale().max()).collect();
    scales.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let max_scale = scales[((scales.len() as f32 * keep) as usize).min(scales.len() - 1)];

    let kept: Vec<Gaussian> = scene
        .gaussians
        .iter()
        .filter(|g| (g.mean - center).norm() <= radius && g.scale().max() <= max_scale)
        .cloned()
        .collect();
    Scene::new(kept, scene.sh_degree)
}

/// Per-axis median of the gaussian means (robust to outliers).
fn robust_center(scene: &Scene) -> Vector3<f32> {
    let n = scene.len();
    if n == 0 {
        return Vector3::zeros();
    }
    let mut xs: Vec<f32> = scene.gaussians.iter().map(|g| g.mean.x).collect();
    let mut ys: Vec<f32> = scene.gaussians.iter().map(|g| g.mean.y).collect();
    let mut zs: Vec<f32> = scene.gaussians.iter().map(|g| g.mean.z).collect();
    xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    ys.sort_by(|a, b| a.partial_cmp(b).unwrap());
    zs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    Vector3::new(xs[n / 2], ys[n / 2], zs[n / 2])
}

/// 75th-percentile distance of gaussian means from `center`.
fn robust_extent(scene: &Scene, center: Vector3<f32>) -> f32 {
    if scene.is_empty() {
        return 1.0;
    }
    let mut d: Vec<f32> = scene
        .gaussians
        .iter()
        .map(|g| (g.mean - center).norm())
        .collect();
    d.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let idx = (d.len() * 3 / 4).min(d.len() - 1);
    d[idx]
}
