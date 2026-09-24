//! Compare GPU SIFT with the CPU `extract_sift` on a directory of images:
//! wall time, keypoint agreement and descriptor similarity.
//!
//! cargo run --release -p visloc-sift-gpu --features gpu --example sift_gpu_bench -- \
//!     --images <dir> [--max-images 5] [--max-keypoints N] [--no-cpu]

use std::collections::HashMap;
use std::path::PathBuf;
use std::time::Instant;

use visloc_sift_gpu::{GpuContext, SiftGpu};
use visloc_vision::features::sift::{extract_sift, GrayImage, SiftConfig, SiftKeypoint};

fn arg(args: &[String], name: &str) -> Option<String> {
    args.iter()
        .position(|a| a == name)
        .and_then(|i| args.get(i + 1).cloned())
}

fn key(k: &SiftKeypoint) -> (i64, i64, i64) {
    (
        (k.x * 8.0).round() as i64,
        (k.y * 8.0).round() as i64,
        (k.sigma * 1000.0).round() as i64,
    )
}

fn angle_diff(a: f64, b: f64) -> f64 {
    let d = (a - b).rem_euclid(std::f64::consts::TAU);
    d.min(std::f64::consts::TAU - d)
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let dir = PathBuf::from(arg(&args, "--images").expect("--images <dir>"));
    let max_images: usize = arg(&args, "--max-images").map_or(5, |v| v.parse().unwrap());
    let run_cpu = !args.iter().any(|a| a == "--no-cpu");
    let mut config = SiftConfig::default();
    if let Some(v) = arg(&args, "--max-keypoints") {
        config.max_keypoints = v.parse().unwrap();
    }
    let mut paths: Vec<PathBuf> = std::fs::read_dir(&dir)
        .unwrap()
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| {
            p.extension().is_some_and(|e| {
                let e = e.to_string_lossy().to_ascii_lowercase();
                e == "png" || e == "jpg" || e == "jpeg"
            })
        })
        .collect();
    paths.sort();
    paths.truncate(max_images);

    let t = Instant::now();
    let mut gpu = SiftGpu::new(GpuContext::new().expect("gpu"));
    eprintln!(
        "gpu: {} (init {:.2}s)",
        gpu.context().adapter_info.name,
        t.elapsed().as_secs_f64()
    );
    let (mut cpu_total, mut gpu_total) = (0.0f64, 0.0f64);
    let mut all_dots: Vec<f32> = Vec::new();
    for path in &paths {
        let gray = image::open(path).unwrap().to_luma8();
        let (w, h) = (gray.width() as usize, gray.height() as usize);
        let pixels: Vec<f32> = gray.as_raw().iter().map(|&b| b as f32).collect();
        let img = GrayImage::new(w, h, &pixels).unwrap();

        // Warm-up once so pipeline/driver setup is not timed.
        if gpu_total == 0.0 {
            let _ = gpu.extract(&img, &config).unwrap();
        }
        let t = Instant::now();
        let (gk, gd) = gpu.extract(&img, &config).unwrap();
        let tg = t.elapsed().as_secs_f64();
        gpu_total += tg;
        let name = path.file_name().unwrap().to_string_lossy();
        if !run_cpu {
            println!("{name} {w}x{h}: gpu {tg:.3}s {} kps", gk.len());
            continue;
        }
        let t = Instant::now();
        let (ck, cd) = extract_sift(&img, &config).unwrap();
        let tc = t.elapsed().as_secs_f64();
        cpu_total += tc;

        let mut index: HashMap<(i64, i64, i64), Vec<usize>> = HashMap::new();
        for (i, k) in gk.iter().enumerate() {
            index.entry(key(k)).or_default().push(i);
        }
        let mut matched = 0usize;
        let mut dots: Vec<f32> = Vec::new();
        for (ci, k) in ck.iter().enumerate() {
            let Some(cands) = index.get(&key(k)) else {
                continue;
            };
            let best = cands
                .iter()
                .copied()
                .min_by(|&a, &b| {
                    angle_diff(gk[a].orientation, k.orientation)
                        .total_cmp(&angle_diff(gk[b].orientation, k.orientation))
                })
                .unwrap();
            if angle_diff(gk[best].orientation, k.orientation) < 0.02 {
                matched += 1;
                let dot: f32 = cd[ci].iter().zip(&gd[best]).map(|(a, b)| a * b).sum();
                dots.push(dot);
            }
        }
        dots.sort_by(|a, b| a.total_cmp(b));
        let pct = |q: f64| dots.get(((dots.len() as f64 - 1.0) * q) as usize).copied();
        println!(
            "{name} {w}x{h}: cpu {tc:.2}s {} kps | gpu {tg:.3}s {} kps ({:.0}x) | cpu kps matched {:.2}% | desc dot p1 {:?} p50 {:?}",
            ck.len(),
            gk.len(),
            tc / tg,
            100.0 * matched as f64 / ck.len().max(1) as f64,
            pct(0.01),
            pct(0.5),
        );
        all_dots.extend(dots);
    }
    if run_cpu {
        all_dots.sort_by(|a, b| a.total_cmp(b));
        let pct = |q: f64| {
            all_dots
                .get(((all_dots.len() as f64 - 1.0) * q) as usize)
                .copied()
        };
        println!(
            "total: cpu {cpu_total:.2}s gpu {gpu_total:.3}s ({:.0}x) | desc dot p0.1 {:?} p1 {:?} p50 {:?}",
            cpu_total / gpu_total,
            pct(0.001),
            pct(0.01),
            pct(0.5)
        );
    } else {
        println!("total: gpu {gpu_total:.3}s");
    }
}
