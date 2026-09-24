//! Replay a dumped SfM bundle-adjustment problem (`VISLOC_SFM_BA_DUMP`) on
//! the GPU solver and, unless `--no-cpu`, on the CPU dense solver: time,
//! LM iterations, robust cost and RMS reprojection error.
//!
//! cargo run --release -p visloc-ba-gpu --features gpu --example ba_gpu_replay -- \
//!     <problem.txt> [--iterations 20] [--no-cpu] [--local N]
//!
//! `--local N` keeps only the last N poses variable (fixing the rest), the
//! shape of the SfM's local BA window.

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
    let local: Option<usize> = opt("--local").map(|v| v.parse().unwrap());
    let load = || {
        let mut ba = read_ba_problem(&path).expect("read problem").0;
        if let Some(n) = local {
            // Shape of the SfM's local BA: the last n poses variable, only the
            // landmarks they observe, and every pose observing those.
            let ids: Vec<u64> = ba.poses.keys().copied().collect();
            let variable: std::collections::BTreeSet<u64> =
                ids[ids.len().saturating_sub(n)..].iter().copied().collect();
            let keep_lm: std::collections::BTreeSet<u64> = ba
                .observations
                .iter()
                .filter(|o| variable.contains(&o.keyframe_id))
                .map(|o| o.landmark_id)
                .collect();
            ba.observations.retain(|o| keep_lm.contains(&o.landmark_id));
            ba.landmarks.retain(|id, _| keep_lm.contains(id));
            let used: std::collections::BTreeSet<u64> =
                ba.observations.iter().map(|o| o.keyframe_id).collect();
            ba.poses.retain(|id, _| used.contains(id));
            ba.fixed_poses.clear();
            for id in ba.poses.keys().copied().collect::<Vec<_>>() {
                if !variable.contains(&id) {
                    ba.fix_pose(id);
                }
            }
        }
        ba
    };
    let ba0 = load();
    println!(
        "problem: {} poses ({} fixed), {} landmarks, {} observations (local window: {local:?})",
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
        let mut warm = load();
        let mut c = config;
        c.max_iterations = 1;
        let _ = gpu.optimize(&mut warm, &c);
    }
    let mut ba = load();
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
