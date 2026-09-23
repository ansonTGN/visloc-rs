//! Replay a dumped SfM bundle-adjustment problem (`VISLOC_SFM_BA_DUMP`) on
//! the GPU solver and, unless `--no-cpu`, on the CPU dense solver: time,
//! LM iterations, robust cost and RMS reprojection error.
//!
//! cargo run --release -p visloc-ba-gpu --features gpu --example ba_gpu_replay -- \
//!     <problem.txt> [--iterations 20] [--no-cpu]

use std::path::PathBuf;
use std::time::Instant;

use visloc_ba_gpu::{GpuBundleAdjuster, GpuContext};
use visloc_slam::ba_problem_io::read_ba_problem;
use visloc_slam::{BaResult, BundleAdjustment, IncrementalSfmConfig, RobustKernel};

fn report(name: &str, secs: f64, r: &BaResult, ba: &BundleAdjustment) {
    let n = ba.observations.len().max(1) as f64;
    println!(
        "{name}: {secs:.2}s, {} iterations ({} accepted), robust cost {:.6e} -> {:.6e}, rms reproj {:.4} px",
        r.iterations.len(),
        r.iterations.iter().filter(|i| i.step_accepted).count(),
        r.initial_cost,
        r.final_cost,
        (ba.robust_cost(&RobustKernel::None) / n).sqrt()
    );
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let path = PathBuf::from(args.get(1).expect("problem path"));
    let opt = |name: &str| {
        args.iter()
            .position(|a| a == name)
            .and_then(|i| args.get(i + 1).cloned())
    };
    let mut config = IncrementalSfmConfig::default().ba_config;
    if let Some(it) = opt("--iterations") {
        config.max_iterations = it.parse().unwrap();
    }
    let (ba0, _) = read_ba_problem(&path).expect("read problem");
    println!(
        "problem: {} poses ({} fixed), {} landmarks, {} observations",
        ba0.poses.len(),
        ba0.fixed_poses.len(),
        ba0.landmarks.len(),
        ba0.observations.len()
    );
    let t = Instant::now();
    let gpu = GpuBundleAdjuster::new(GpuContext::new().expect("gpu"));
    println!("gpu init {:.2}s", t.elapsed().as_secs_f64());
    // Warm-up (driver/pipeline caches) on a copy.
    {
        let mut warm = read_ba_problem(&path).unwrap().0;
        let mut c = config;
        c.max_iterations = 1;
        let _ = gpu.optimize(&mut warm, &c);
    }
    let mut ba = read_ba_problem(&path).unwrap().0;
    let t = Instant::now();
    let r = gpu.optimize(&mut ba, &config).expect("gpu optimize");
    report("gpu", t.elapsed().as_secs_f64(), &r, &ba);
    if !args.iter().any(|a| a == "--no-cpu") {
        let mut ba = ba0;
        let t = Instant::now();
        let r = ba.optimize(&config).expect("cpu optimize");
        report("cpu dense", t.elapsed().as_secs_f64(), &r, &ba);
    }
}
