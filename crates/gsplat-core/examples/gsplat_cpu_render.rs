//! Render a 3DGS scene with the CPU reference rasterizer.
//!
//! Two input modes:
//!
//! - `--splat <file.splat>`: load an antimatter15 `.splat` and render it from a
//!   camera that frames the scene bounds, or from the camera given by
//!   `--camera`.
//! - `--colmap <dir>`: load a COLMAP model, seed a degree-0 scene from its
//!   sparse landmarks, and render from one of its registered cameras.
//!
//! Usage:
//! ```text
//! cargo run -p visloc-gsplat-core --features colmap-io \
//!   --example gsplat_cpu_render -- --splat scene.splat --out frame.png \
//!   [--width 752 --height 480 --colmap-camera 0]
//! ```

use std::path::PathBuf;

use nalgebra::{Matrix3, Vector3};
use visloc_gsplat_core::camera::{CameraView, PinholeCamera};
use visloc_gsplat_core::gaussian::Scene;
use visloc_gsplat_core::splat;

#[cfg(feature = "colmap-io")]
use visloc_gsplat_core::colmap_scene::load_colmap_scene;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    let mut splat_path: Option<PathBuf> = None;
    let mut colmap_dir: Option<PathBuf> = None;
    let mut out = PathBuf::from("gsplat_cpu_frame.png");
    let mut width = 752u32;
    let mut height = 480u32;
    let mut camera_index = 0usize;

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--splat" => {
                splat_path = Some(PathBuf::from(&args[i + 1]));
                i += 2;
            }
            "--colmap" => {
                colmap_dir = Some(PathBuf::from(&args[i + 1]));
                i += 2;
            }
            "--out" => {
                out = PathBuf::from(&args[i + 1]);
                i += 2;
            }
            "--width" => {
                width = args[i + 1].parse()?;
                i += 2;
            }
            "--height" => {
                height = args[i + 1].parse()?;
                i += 2;
            }
            "--colmap-camera" => {
                camera_index = args[i + 1].parse()?;
                i += 2;
            }
            other => {
                eprintln!("unknown argument: {other}");
                std::process::exit(2);
            }
        }
    }

    let (scene, view, label) = if let Some(dir) = colmap_dir {
        #[cfg(feature = "colmap-io")]
        {
            let loaded = load_colmap_scene(&dir, -3.0)?;
            let idx = camera_index.min(loaded.views.len().saturating_sub(1));
            let name = loaded
                .image_names
                .get(idx)
                .cloned()
                .unwrap_or_else(|| format!("camera {idx}"));
            (
                loaded.scene,
                loaded.views[idx],
                format!("{} (colmap camera {idx}: {name})", dir.display()),
            )
        }
        #[cfg(not(feature = "colmap-io"))]
        {
            let _ = dir;
            return Err("built without the `colmap-io` feature".into());
        }
    } else if let Some(path) = splat_path {
        let scene = splat::load_splat(&path)?;
        let view = frame_scene(&scene, width, height);
        (scene, view, path.display().to_string())
    } else {
        return Err("pass either --splat <file> or --colmap <dir>".into());
    };

    eprintln!(
        "loaded {} gaussians from {label}; rendering {}x{}",
        scene.len(),
        width,
        height
    );
    let started = std::time::Instant::now();
    let image = visloc_gsplat_core::render(&scene, &view, [0.0, 0.0, 0.0]);
    let elapsed = started.elapsed();
    eprintln!(
        "rendered in {:.2} s ({:.1} Mpixel/s)",
        elapsed.as_secs_f64(),
        (width as f64 * height as f64) / elapsed.as_secs_f64() / 1e6
    );

    let bytes = image.to_rgb8();
    let img = image::RgbImage::from_raw(width, height, bytes).expect("buffer size");
    img.save(&out)?;
    eprintln!("wrote {}", out.display());
    Ok(())
}

/// Build a camera that frames the scene bounds from a corner viewpoint.
///
/// Uses a robust extent from the dense core of the point cloud rather than
/// `max - min`: trained `.splat` files routinely contain floater outliers
/// thousands of metres out, and framing on those collapses the real scene to a
/// dot. The extent is the 75th-percentile distance from the median point, which
/// on the EuRoC splats matches the visible room (~8-40 m) instead of the
/// floater tail (~600 m).
fn frame_scene(scene: &Scene, width: u32, height: u32) -> CameraView {
    let center = robust_center(scene);
    let extent = robust_extent(scene, center).max(1e-3);
    eprintln!("scene center {center:?}, robust extent {extent:.3}");
    let fx = 0.5 * width as f32;
    let fy = 0.5 * height as f32;
    // Distance so the object of radius `extent` fills the frame.
    let half_fov = (0.5 * fx / (0.5 * width as f32)).atan();
    let distance = extent / half_fov.tan();
    let eye = center + Vector3::new(0.0, 0.0, distance);
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
        .filter(|v| v.is_finite())
        .collect();
    if d.is_empty() {
        return 1.0;
    }
    d.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let idx = ((d.len() as f32) * 0.75) as usize;
    d[idx.min(d.len() - 1)].max(1e-3)
}
