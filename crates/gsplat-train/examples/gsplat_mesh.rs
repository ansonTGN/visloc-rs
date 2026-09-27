//! Extract a coloured triangle mesh from a trained splat.
//!
//! ```text
//! cargo run --release -p visloc-gsplat-train --features gpu --example gsplat_mesh -- \
//!     --ply ours.ply --data <colmap_root> --out mesh.ply \
//!     [--voxel V] [--trunc-voxels 4] [--max-depth D] [--min-component 500]
//! ```
//!
//! Renders the median depth of the splat from every dataset view, fuses the
//! depths into a sparse TSDF and extracts the surface with surface nets (see
//! `visloc_gsplat_train::mesh`). Defaults are relative to the camera rig:
//! `scale` = median distance of the camera centres from their centroid,
//! voxel = scale / 256, max depth = 2 * scale.

use std::path::PathBuf;
use std::time::Instant;

use nalgebra::Vector3;
use visloc_gsplat_core::ply::load_ply;
use visloc_gsplat_render::{GpuContext, Renderer};
use visloc_gsplat_train::dataset::load_colmap_dataset;
use visloc_gsplat_train::mesh::{DepthFrame, Tsdf};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let mut ply: Option<PathBuf> = None;
    let mut data: Option<PathBuf> = None;
    let mut out = PathBuf::from("mesh.ply");
    let mut voxel: Option<f32> = None;
    let mut trunc_voxels = 4.0f32;
    let mut max_depth: Option<f32> = None;
    let mut min_component = 500usize;
    let num = |v: Option<String>, flag: &str| -> Result<f32, String> {
        v.and_then(|s| s.parse().ok())
            .ok_or_else(|| format!("{flag} needs a number"))
    };
    while let Some(a) = args.next() {
        match a.as_str() {
            "--ply" => ply = args.next().map(PathBuf::from),
            "--data" => data = args.next().map(PathBuf::from),
            "--out" => out = args.next().map(PathBuf::from).ok_or("--out needs a path")?,
            "--voxel" => voxel = Some(num(args.next(), "--voxel")?),
            "--trunc-voxels" => trunc_voxels = num(args.next(), "--trunc-voxels")?,
            "--max-depth" => max_depth = Some(num(args.next(), "--max-depth")?),
            "--min-component" => min_component = num(args.next(), "--min-component")? as usize,
            other => return Err(format!("unknown argument {other}").into()),
        }
    }
    let ply = ply.ok_or("--ply <path> is required")?;
    let data = data.ok_or("--data <colmap root> is required")?;

    let scene = load_ply(&ply)?;
    let dataset = load_colmap_dataset(&data, None)?;
    let views: Vec<_> = dataset.train.iter().chain(&dataset.eval).collect();
    let centers: Vec<Vector3<f32>> = views.iter().map(|v| v.camera.camera_center()).collect();
    let centroid = centers.iter().sum::<Vector3<f32>>() / centers.len().max(1) as f32;
    let mut dists: Vec<f32> = centers.iter().map(|c| (c - centroid).norm()).collect();
    dists.sort_by(f32::total_cmp);
    let scale = dists.get(dists.len() / 2).copied().unwrap_or(1.0).max(1e-6);
    let voxel = voxel.unwrap_or(scale / 256.0);
    let max_depth = max_depth.unwrap_or(2.0 * scale);
    println!(
        "{} gaussians, {} views; scale {scale:.3}, voxel {voxel:.4}, trunc {:.4}, max depth {max_depth:.3}",
        scene.len(),
        views.len(),
        voxel * trunc_voxels
    );

    let mut tsdf = Tsdf::new(voxel, voxel * trunc_voxels, max_depth);
    let mut renderer: Option<(u32, u32, Renderer)> = None;
    let (mut t_render, mut t_fuse) = (0.0f64, 0.0f64);
    for view in &views {
        let (w, h) = (view.camera.camera.width, view.camera.camera.height);
        if renderer.as_ref().map(|r| (r.0, r.1)) != Some((w, h)) {
            let ctx = match renderer.take() {
                Some((_, _, r)) => r.ctx,
                None => GpuContext::new()?,
            };
            renderer = Some((w, h, Renderer::new(ctx, &scene, w, h)?));
        }
        let (_, _, r) = renderer.as_mut().expect("renderer set above");
        let t0 = Instant::now();
        let (image, depth, _opacity) = r.render_depth(&view.camera, [0.0, 0.0, 0.0]);
        let t1 = Instant::now();
        tsdf.integrate(&DepthFrame {
            view: &view.camera,
            depth: &depth,
            rgb: &image.rgb,
        });
        t_render += (t1 - t0).as_secs_f64();
        t_fuse += t1.elapsed().as_secs_f64();
    }
    println!(
        "fused {} views: render {t_render:.1} s, fuse {t_fuse:.1} s, {} blocks",
        views.len(),
        tsdf.num_blocks()
    );
    let t0 = Instant::now();
    let mut mesh = tsdf.extract();
    let raw = mesh.triangles.len();
    mesh.remove_small_components(min_component);
    println!(
        "extracted {raw} triangles, {} after dropping components < {min_component} in {:.1} s",
        mesh.triangles.len(),
        t0.elapsed().as_secs_f64()
    );
    mesh.write_ply(&out)?;
    println!("wrote {}", out.display());
    Ok(())
}
