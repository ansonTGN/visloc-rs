//! Replay a bundle-adjustment problem dumped by the incremental SfM
//! (`VISLOC_SFM_BA_DUMP=<path>`) with a chosen solver, for timing/accuracy
//! comparisons on the exact same problem.
//!
//! cargo run --release -p visloc-slam --example ba_replay -- <problem.txt> \
//!     [--solver dense|sparse|matrix-free] [--iterations 20] [--pcg-iters 128] [--pcg-tol 1e-12]
//!
//! Uses the SfM's BA settings (`IncrementalSfmConfig::default().ba_config`).
//! Set `VISLOC_BA_TRACE_PHASE_TIMING=1` for the per-iteration phase split.

use std::path::PathBuf;
use std::time::Instant;

use visloc_slam::ba_problem_io::read_ba_problem;
use visloc_slam::{IncrementalSfmConfig, MatrixFreeBaOptions};
use visloc_slam::{LinearSolver, RobustKernel};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let path = PathBuf::from(args.get(1).expect("problem path"));
    let opt = |name: &str| {
        args.iter()
            .position(|a| a == name)
            .and_then(|i| args.get(i + 1).cloned())
    };
    let solver = opt("--solver").unwrap_or_else(|| "dense".into());
    let mut config = IncrementalSfmConfig::default().ba_config;
    if let Some(it) = opt("--iterations") {
        config.max_iterations = it.parse().unwrap();
    }
    match solver.as_str() {
        "dense" => config.linear_solver = LinearSolver::Dense,
        "sparse" => config.linear_solver = LinearSolver::Sparse,
        "matrix-free" => config.matrix_free_ba = true,
        other => panic!("unknown solver {other}"),
    }
    let t = Instant::now();
    let (mut ba, weights) = read_ba_problem(&path).expect("read problem");
    println!(
        "problem: {} poses ({} fixed), {} landmarks, {} observations, weights {} (load {:.2}s)",
        ba.poses.len(),
        ba.fixed_poses.len(),
        ba.landmarks.len(),
        ba.observations.len(),
        weights.is_some(),
        t.elapsed().as_secs_f64()
    );
    let initial_l2 = ba.robust_cost(&RobustKernel::None);
    let t = Instant::now();
    let result = if config.matrix_free_ba {
        let mut options = MatrixFreeBaOptions::default();
        if let Some(v) = opt("--pcg-iters") {
            options.max_pcg_iterations = v.parse().unwrap();
        }
        if let Some(v) = opt("--pcg-tol") {
            options.pcg_relative_tolerance = v.parse().unwrap();
            options.pcg_absolute_tolerance = 1e-12;
        }
        ba.optimize_matrix_free(&config, options)
            .map(|r| visloc_slam::BaResult {
                initial_cost: r.initial_cost,
                final_cost: r.final_cost,
                iterations: r.iterations,
                converged: r.converged,
            })
            .map_err(|e| format!("{e}"))
    } else if let Some(w) = weights.as_deref() {
        ba.optimize_with_observation_weights(&config, w)
            .map_err(|e| format!("{e}"))
    } else {
        ba.optimize(&config).map_err(|e| format!("{e}"))
    }
    .expect("optimize");
    let secs = t.elapsed().as_secs_f64();
    let final_l2 = ba.robust_cost(&RobustKernel::None);
    let n = ba.observations.len().max(1) as f64;
    println!(
        "{solver}: {secs:.2}s, {} iterations ({} accepted), robust cost {:.6e} -> {:.6e}, rms reproj {:.4} -> {:.4} px",
        result.iterations.len(),
        result.iterations.iter().filter(|i| i.step_accepted).count(),
        result.initial_cost,
        result.final_cost,
        (initial_l2 / n).sqrt(),
        (final_l2 / n).sqrt(),
    );
}
