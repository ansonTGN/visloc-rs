//! Phase-A diagnostic: per-stage wall-clock cost of the offline NFR mapper
//! (`pipelines/basalt/src/mapper/session.rs`) as a function of the number of
//! ingested MargData packets (keyframes).
//!
//! This is a throwaway measurement tool for
//! `docs/basalt_online_mapper_design.md`, not a parity artifact: it calls the
//! same stateful `NfrMapper` methods the offline demo and `run_headless` use,
//! in the same order, but times each one individually and reports at
//! configurable prefix lengths instead of only the full sequence. It does not
//! change any mapper behavior.
//!
//! usage: basalt_mapper_stage_profile --marg-dir DIR --calibration FILE
//!   --config FILE [--limits 50,100,200,454]

use std::{env, fs, path::PathBuf, process, time::Instant};

use serde_json::json;
use visloc_basalt::{config::BasaltConfig, mapper::NfrMapper, vio::MargData, BasaltCalibration};

fn main() {
    if let Err(error) = run() {
        eprintln!("basalt_mapper_stage_profile: {error}");
        process::exit(1);
    }
}

struct Args {
    marg_dir: PathBuf,
    calibration: PathBuf,
    config: PathBuf,
    limits: Vec<usize>,
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let args = parse_args(env::args_os().skip(1))?;

    let mut paths = fs::read_dir(&args.marg_dir)?
        .filter_map(|entry| entry.ok())
        .map(|entry| entry.path())
        .filter(|path| {
            path.extension()
                .is_some_and(|extension| extension == "json")
        })
        .collect::<Vec<_>>();
    paths.sort();
    if paths.is_empty() {
        return Err("no marg_data packets found".into());
    }

    let config_text = fs::read_to_string(&args.config)?;
    let config = BasaltConfig::from_json(&config_text)?;
    let mapper_config = config.mapper_config()?;
    let feature_config = config.offline_mapper_config()?;
    let optimize_config = config.mapper_global_ba_config()?;
    let calibration_text = fs::read_to_string(&args.calibration)?;
    let calibration = BasaltCalibration::from_json_str(&calibration_text)?;

    println!("[");
    let mut first_entry = true;
    for &limit in &args.limits {
        let n = limit.min(paths.len());
        let selected = &paths[..n];

        let load_start = Instant::now();
        let mut packets = selected
            .iter()
            .map(|path| {
                let text = fs::read_to_string(path).map_err(|error| error.to_string())?;
                MargData::from_json_checked(&text)
                    .map_err(|error| format!("{}: {error}", path.display()))
            })
            .collect::<Result<Vec<_>, String>>()?;
        let load_seconds = load_start.elapsed().as_secs_f64();

        let mut mapper = NfrMapper::with_calibration(mapper_config, calibration.clone());
        mapper.set_feature_config(feature_config);
        mapper.set_optimize_config(optimize_config);

        let ingest_start = Instant::now();
        for packet in &mut packets {
            mapper
                .add_marg_data(packet)
                .map_err(|error| format!("add_marg_data: {error:?}"))?;
        }
        let ingest_seconds = ingest_start.elapsed().as_secs_f64();

        let t = Instant::now();
        let detection = mapper
            .detect_keypoints()
            .map_err(|error| format!("detect_keypoints: {error:?}"))?;
        let detect_seconds = t.elapsed().as_secs_f64();

        let t = Instant::now();
        let stereo = mapper
            .match_stereo()
            .map_err(|error| format!("match_stereo: {error:?}"))?;
        let stereo_seconds = t.elapsed().as_secs_f64();

        let t = Instant::now();
        let match_all = mapper.match_all();
        let match_all_seconds = t.elapsed().as_secs_f64();

        let t = Instant::now();
        let tracks = mapper.build_tracks();
        let build_tracks_seconds = t.elapsed().as_secs_f64();

        let t = Instant::now();
        let setup = mapper.setup_opt()?;
        let setup_opt_seconds = t.elapsed().as_secs_f64();

        let t = Instant::now();
        let optimize1 = mapper.optimize(10)?;
        let optimize1_seconds = t.elapsed().as_secs_f64();

        let t = Instant::now();
        let filter = mapper.filter_outliers(3.0, 4)?;
        let filter_seconds = t.elapsed().as_secs_f64();

        let t = Instant::now();
        let optimize2 = mapper.optimize(10)?;
        let optimize2_seconds = t.elapsed().as_secs_f64();

        let peak_rss_bytes = peak_working_set_bytes();

        let entry = json!({
            "n_requested": limit,
            "n_packets": n,
            "load_seconds": load_seconds,
            "ingest_seconds": ingest_seconds,
            "detect_seconds": detect_seconds,
            "detect_processed_images": detection.processed_image_count,
            "detect_feature_count": detection.feature_count,
            "stereo_seconds": stereo_seconds,
            "stereo_accepted_pairs": stereo.accepted_pair_count,
            "match_all_seconds": match_all_seconds,
            "match_all_query_count": match_all.query_count,
            "match_all_candidate_pair_count": match_all.candidate_pair_count,
            "match_all_raw_match_count": match_all.raw_match_count,
            "match_all_ransac_attempt_count": match_all.ransac_attempt_count,
            "build_tracks_seconds": build_tracks_seconds,
            "exported_track_count": tracks.exported_track_count,
            "setup_opt_seconds": setup_opt_seconds,
            "accepted_track_count": setup.accepted_track_count,
            "optimize1_seconds": optimize1_seconds,
            "optimize1_iterations": optimize1.iterations,
            "optimize1_pose_count": optimize1.pose_count,
            "optimize1_landmark_count": optimize1.landmark_count,
            "filter_seconds": filter_seconds,
            "removed_landmark_count": filter.removed_landmark_count,
            "optimize2_seconds": optimize2_seconds,
            "optimize2_iterations": optimize2.iterations,
            "peak_working_set_bytes": peak_rss_bytes,
        });
        if !first_entry {
            println!(",");
        }
        first_entry = false;
        print!("{}", serde_json::to_string_pretty(&entry)?);
        // Flush per-N so a detached/backgrounded run can be tailed live.
        use std::io::Write;
        std::io::stdout().flush()?;
        eprintln!(
            "n={n} detect={detect_seconds:.2}s stereo={stereo_seconds:.2}s match_all={match_all_seconds:.2}s \
             build_tracks={build_tracks_seconds:.2}s setup_opt={setup_opt_seconds:.2}s \
             optimize1={optimize1_seconds:.2}s filter={filter_seconds:.2}s optimize2={optimize2_seconds:.2}s \
             peak_rss={:.0}MB",
            peak_rss_bytes as f64 / 1e6
        );
        drop(mapper);
        drop(packets);
    }
    println!("\n]");

    Ok(())
}

#[cfg(windows)]
fn peak_working_set_bytes() -> u64 {
    // Minimal FFI to GetProcessMemoryInfo via PROCESS_MEMORY_COUNTERS, avoiding
    // a new Cargo dependency for a throwaway diagnostic tool.
    #[repr(C)]
    #[allow(non_snake_case)]
    struct ProcessMemoryCounters {
        cb: u32,
        PageFaultCount: u32,
        PeakWorkingSetSize: usize,
        WorkingSetSize: usize,
        QuotaPeakPagedPoolUsage: usize,
        QuotaPagedPoolUsage: usize,
        QuotaPeakNonPagedPoolUsage: usize,
        QuotaNonPagedPoolUsage: usize,
        PagefileUsage: usize,
        PeakPagefileUsage: usize,
    }
    #[link(name = "psapi")]
    extern "system" {
        fn GetProcessMemoryInfo(
            process: isize,
            counters: *mut ProcessMemoryCounters,
            size: u32,
        ) -> i32;
    }
    #[link(name = "kernel32")]
    extern "system" {
        fn GetCurrentProcess() -> isize;
    }
    unsafe {
        let mut counters: ProcessMemoryCounters = std::mem::zeroed();
        counters.cb = std::mem::size_of::<ProcessMemoryCounters>() as u32;
        let handle = GetCurrentProcess();
        if GetProcessMemoryInfo(handle, &mut counters, counters.cb) != 0 {
            counters.PeakWorkingSetSize as u64
        } else {
            0
        }
    }
}

#[cfg(not(windows))]
const fn peak_working_set_bytes() -> u64 {
    0
}

fn parse_args<I>(arguments: I) -> Result<Args, String>
where
    I: IntoIterator<Item = std::ffi::OsString>,
{
    let mut marg_dir = None;
    let mut calibration = None;
    let mut config = None;
    let mut limits = vec![50usize, 100, 200, 454];
    let mut arguments = arguments.into_iter();
    while let Some(argument) = arguments.next() {
        let option = argument.to_string_lossy().into_owned();
        match option.as_str() {
            "--marg-dir" => marg_dir = Some(PathBuf::from(next(&mut arguments, &option)?)),
            "--calibration" => calibration = Some(PathBuf::from(next(&mut arguments, &option)?)),
            "--config" => config = Some(PathBuf::from(next(&mut arguments, &option)?)),
            "--limits" => {
                let value = next(&mut arguments, &option)?;
                limits = value
                    .to_string_lossy()
                    .split(',')
                    .map(|s| s.trim().parse::<usize>())
                    .collect::<Result<Vec<_>, _>>()
                    .map_err(|error| format!("invalid --limits: {error}"))?;
            }
            unknown => return Err(format!("unknown option `{unknown}`")),
        }
    }
    Ok(Args {
        marg_dir: marg_dir.ok_or("--marg-dir is required")?,
        calibration: calibration.ok_or("--calibration is required")?,
        config: config.ok_or("--config is required")?,
        limits,
    })
}

fn next<I>(arguments: &mut I, option: &str) -> Result<std::ffi::OsString, String>
where
    I: Iterator<Item = std::ffi::OsString>,
{
    arguments
        .next()
        .ok_or_else(|| format!("{option} requires a value"))
}
