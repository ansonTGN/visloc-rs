//! Online EuRoC VI-SLAM demo: VIO thread + concurrent mapper thread.
//!
//! Single process, one run over the sensor stream. The VIO thread (this
//! process's main thread) consumes images/IMU at either dataset rate
//! (`--realtime`) or as fast as possible (default), and for every
//! `MargData` mapper packet moves it (no JSON/base64 round trip --
//! `EstimatorOutput::marg_data` is already an owned in-memory value) into an
//! *unbounded* channel to a dedicated mapper thread running
//! `visloc_basalt::mapper_online::OnlineNfrMapper`. The VIO thread must
//! never block on the mapper: a bounded channel's backpressure does exactly
//! that the moment the mapper falls behind, which is observable and reported
//! here instead as mapper queue depth/lag (`max_mapper_queue_depth`,
//! `mapper_queue_lag_seconds`) -- a queued packet is cheap to hold since
//! `ingest_packet` extracts features and drops raw pixels in the same call
//! that receives it, so queue growth costs keypoints/descriptors, not
//! images.
//!
//! See `docs/basalt_online_mapper_design.md` for the full design and
//! `docs/vi_slam_global_consistency_plan.md` Sec1.4/3/4 for why this exists:
//! turning the offline NFR mapper that already reaches 8/11 wins vs
//! ORB-SLAM3 into an online system that runs concurrently with the VIO.
//!
//! ```text
//! cargo run --release --example basalt_euroc_online_slam_demo -- \
//!   --euroc-dir /data/MH_01_easy \
//!   --calibration configs/basalt/variants/official_euroc_ds/euroc_ds_calib.json \
//!   --config configs/basalt/variants/official_euroc_ds/euroc_config.json \
//!   --out-dir target/basalt_mh01_online --optimize-every-k 100 --periodic-iterations 4
//! ```

use std::{
    collections::VecDeque,
    env, fs,
    path::PathBuf,
    process,
    sync::{mpsc, Arc, Mutex},
    thread,
    time::{Duration, Instant},
};

use serde_json::json;
use visloc_basalt::{
    mapper_online::{run_mapper_thread, OnlineIngestReport, OnlineMapperConfig, OnlineNfrMapper},
    vio::MargData,
    BasaltVioEstimatorAdapter, EurocSensorDataset,
};

#[derive(Debug)]
struct Args {
    euroc_dir: PathBuf,
    calibration: PathBuf,
    config: PathBuf,
    out_dir: PathBuf,
    max_frames: Option<usize>,
    optimize_every_k: usize,
    periodic_iterations: usize,
    realtime: bool,
}

fn main() {
    if let Err(error) = run() {
        eprintln!("basalt_euroc_online_slam_demo: {error}");
        process::exit(1);
    }
}

/// Running sums of [`OnlineIngestReport`] fields across the whole sequence,
/// accumulated on the mapper thread and read back after it joins. Kept as
/// aggregate counters (not one record per packet) so the output stays small
/// even on EuRoC's largest sequences.
#[derive(Debug, Default)]
struct MapperAggregate {
    packet_count: usize,
    new_key_count: usize,
    accepted_temporal_pair_count: usize,
    accepted_loop_pair_count: usize,
    optimize_pass_count: usize,
    detect_seconds: f64,
    stereo_seconds: f64,
    match_seconds: f64,
    optimize_seconds: f64,
}

impl MapperAggregate {
    fn add(&mut self, report: &OnlineIngestReport) {
        self.packet_count += 1;
        self.new_key_count += report.new_key_count;
        self.accepted_temporal_pair_count += report.accepted_temporal_pair_count;
        self.accepted_loop_pair_count += report.accepted_loop_pair_count;
        self.detect_seconds += report.detect_seconds;
        self.stereo_seconds += report.stereo_seconds;
        self.match_seconds += report.match_seconds;
        self.optimize_seconds += report.optimize_seconds;
        if report.optimize_triggered {
            self.optimize_pass_count += 1;
        }
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse(env::args_os().skip(1))
        .map_err(|error| std::io::Error::new(std::io::ErrorKind::InvalidInput, error))?;
    fs::create_dir_all(&args.out_dir)?;

    let dataset = EurocSensorDataset::open(&args.euroc_dir, &args.calibration, &args.config)?;
    let mut adapter =
        BasaltVioEstimatorAdapter::from_config(dataset.calibration(), dataset.config())?;
    let frame_limit = args
        .max_frames
        .unwrap_or(dataset.frame_count())
        .min(dataset.frame_count());
    if frame_limit == 0 {
        return Err("EuRoC cam0 manifest has no frames".into());
    }

    let mapper_config = dataset.config().mapper_config()?;
    let feature_config = dataset.config().offline_mapper_config()?;
    let optimize_config = dataset.config().mapper_global_ba_config()?;
    let online_mapper = OnlineNfrMapper::new(
        mapper_config,
        dataset.calibration().clone(),
        feature_config,
        optimize_config,
        OnlineMapperConfig {
            optimize_every_k: args.optimize_every_k,
            periodic_iterations: args.periodic_iterations,
            ..OnlineMapperConfig::default()
        },
    );

    // Unbounded: the VIO thread must never block on the mapper (rule 1 in
    // mapper_online's module doc). A queued MargData packet is cheap to
    // hold -- ingest_packet extracts features and drops its raw pixels in
    // the same call that receives it -- so queue growth costs
    // keypoints/descriptors, not images. Backpressure is measured and
    // reported (queue depth, lag), not silently applied.
    let (sender, receiver) = mpsc::channel::<MargData>();
    let send_times: Arc<Mutex<VecDeque<Instant>>> = Arc::new(Mutex::new(VecDeque::new()));
    let lag_seconds: Arc<Mutex<Vec<f64>>> = Arc::new(Mutex::new(Vec::new()));
    let aggregate: Arc<Mutex<MapperAggregate>> = Arc::new(Mutex::new(MapperAggregate::default()));

    let send_times_for_mapper = Arc::clone(&send_times);
    let lag_seconds_for_mapper = Arc::clone(&lag_seconds);
    let aggregate_for_mapper = Arc::clone(&aggregate);
    let mapper_handle = thread::spawn(move || {
        run_mapper_thread(online_mapper, receiver, None, move |report| {
            if let Some(sent_at) = send_times_for_mapper.lock().expect("lock").pop_front() {
                lag_seconds_for_mapper
                    .lock()
                    .expect("lock")
                    .push(sent_at.elapsed().as_secs_f64());
            }
            aggregate_for_mapper.lock().expect("lock").add(report);
        })
    });

    let mut first_timestamp_ns: Option<i64> = None;
    let mut last_timestamp_ns: i64 = 0;
    let mut total_imu = 0usize;
    let mut total_observations = 0usize;
    let mut mapper_packet_count = 0u64;
    let mut max_queue_depth = 0usize;
    let vio_start = Instant::now();
    for index in 0..frame_limit {
        let sensor_frame = dataset.frame(index)?;
        if args.realtime {
            if let Some(first) = first_timestamp_ns {
                let target_offset =
                    Duration::from_nanos((sensor_frame.timestamp_ns - first).max(0) as u64);
                let target = vio_start + target_offset;
                let now = Instant::now();
                if target > now {
                    thread::sleep(target - now);
                }
            }
        }
        let output = adapter.process(sensor_frame)?;
        let timestamp_ns = output.tracks.frame.timestamp_ns;
        first_timestamp_ns.get_or_insert(timestamp_ns);
        last_timestamp_ns = timestamp_ns;
        total_imu += output.imu_count;
        total_observations += output.tracks.observations.len();

        if output.estimator.marg_data.is_mapper_packet() {
            let depth_after_send = {
                let mut guard = send_times.lock().expect("lock");
                guard.push_back(Instant::now());
                guard.len()
            };
            max_queue_depth = max_queue_depth.max(depth_after_send);
            sender
                .send(output.estimator.marg_data)
                .map_err(|_| "mapper thread ended before the VIO stream finished")?;
            mapper_packet_count += 1;
        }

        if index == 0 || (index + 1) % 50 == 0 || index + 1 == frame_limit {
            eprintln!(
                "frame={} timestamp_ns={} mapper_packets={} imu={}",
                output.tracks.frame.frame_id, timestamp_ns, mapper_packet_count, total_imu,
            );
        }
    }
    let vio_wall_seconds = vio_start.elapsed().as_secs_f64();

    // Close the channel: the mapper thread's receive loop exits and this
    // demo runs OnlineNfrMapper::finalize (rule 3 -- the exact
    // `run_headless` tail) after joining, below.
    drop(sender);
    let (mut online_mapper, _stop_reason, mapper_errors) =
        mapper_handle.join().map_err(|_| "mapper thread panicked")?;
    if let Some(error) = mapper_errors.first() {
        return Err(format!("mapper thread reported an error: {error}").into());
    }
    let mapper_join_seconds = vio_start.elapsed().as_secs_f64() - vio_wall_seconds;

    let final_report = online_mapper.finalize()?;
    let total_wall_seconds = vio_start.elapsed().as_secs_f64();

    let dataset_duration_seconds =
        (last_timestamp_ns - first_timestamp_ns.unwrap_or(last_timestamp_ns)) as f64 * 1e-9;
    let real_time_factor = if vio_wall_seconds > 0.0 {
        dataset_duration_seconds / vio_wall_seconds
    } else {
        0.0
    };

    let lag = lag_seconds.lock().expect("lock");
    let lag_max = lag.iter().cloned().fold(0.0_f64, f64::max);
    let lag_sample_count = lag.len();
    let lag_mean = if lag.is_empty() {
        0.0
    } else {
        lag.iter().sum::<f64>() / lag.len() as f64
    };
    drop(lag);
    let aggregate = aggregate.lock().expect("lock");
    let peak_rss_bytes = peak_working_set_bytes();

    let summary = json!({
        "schema": "basalt.online_mapper.run.v1",
        "pacing": if args.realtime { "dataset_rate" } else { "as_fast_as_possible" },
        "frames_processed": frame_limit,
        "dataset_duration_seconds": dataset_duration_seconds,
        "vio_wall_seconds": vio_wall_seconds,
        "mapper_join_seconds": mapper_join_seconds,
        "total_wall_seconds": total_wall_seconds,
        "real_time_factor": real_time_factor,
        "imu_samples": total_imu,
        "observations": total_observations,
        "mapper_packets_sent": mapper_packet_count,
        "max_mapper_queue_depth": max_queue_depth,
        "mapper_queue_lag_seconds": {"max": lag_max, "mean": lag_mean, "samples": lag_sample_count},
        "mapper": {
            "packet_count": aggregate.packet_count,
            "new_key_count": aggregate.new_key_count,
            "accepted_temporal_pair_count": aggregate.accepted_temporal_pair_count,
            "accepted_loop_pair_count": aggregate.accepted_loop_pair_count,
            "optimize_pass_count": aggregate.optimize_pass_count,
            "detect_seconds": aggregate.detect_seconds,
            "stereo_seconds": aggregate.stereo_seconds,
            "match_seconds": aggregate.match_seconds,
            "optimize_seconds": aggregate.optimize_seconds,
        },
        "final_optimize": {
            "first_optimize_final_cost": final_report.first_optimize.final_cost,
            "second_optimize_final_cost": final_report.second_optimize.final_cost,
            "pose_count": final_report.result.poses.len(),
            "landmark_count": final_report.result.landmarks.len(),
        },
        "peak_working_set_bytes": peak_rss_bytes,
    });

    fs::write(
        args.out_dir.join("trajectory_online.tum"),
        &final_report.trajectory_tum,
    )?;
    fs::write(
        args.out_dir.join("timing_breakdown_online.json"),
        serde_json::to_string_pretty(&summary)?,
    )?;
    println!("{}", serde_json::to_string_pretty(&summary)?);
    eprintln!(
        "pacing={} rtf={:.3} vio_wall={:.1}s total_wall={:.1}s max_queue_depth={} \
         lag_max={:.3}s lag_mean={:.3}s loops={} optimizes={} peak_rss={:.0}MB out={}",
        if args.realtime {
            "dataset_rate"
        } else {
            "as_fast_as_possible"
        },
        real_time_factor,
        vio_wall_seconds,
        total_wall_seconds,
        max_queue_depth,
        lag_max,
        lag_mean,
        aggregate.accepted_loop_pair_count,
        aggregate.optimize_pass_count,
        peak_rss_bytes as f64 / 1e6,
        args.out_dir.display(),
    );
    Ok(())
}

#[cfg(windows)]
fn peak_working_set_bytes() -> u64 {
    // Same minimal FFI as examples/basalt_mapper_stage_profile.rs; examples
    // are independent crate roots so this small helper is duplicated rather
    // than shared.
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
fn peak_working_set_bytes() -> u64 {
    0
}

impl Args {
    fn parse<I>(arguments: I) -> Result<Self, String>
    where
        I: IntoIterator<Item = std::ffi::OsString>,
    {
        let mut euroc_dir = None;
        let mut calibration = None;
        let mut config = PathBuf::from("configs/basalt/euroc_config.json");
        let mut out_dir = PathBuf::from("target/basalt_euroc_online_slam_demo");
        let mut max_frames = None;
        let mut optimize_every_k = OnlineMapperConfig::default().optimize_every_k;
        let mut periodic_iterations = OnlineMapperConfig::default().periodic_iterations;
        let mut realtime = false;
        let mut arguments = arguments.into_iter();
        while let Some(argument) = arguments.next() {
            let option = argument.to_string_lossy().into_owned();
            match option.as_str() {
                "--help" | "-h" => return Err(Self::usage()),
                "--euroc-dir" => euroc_dir = Some(PathBuf::from(next(&mut arguments, &option)?)),
                "--calibration" => {
                    calibration = Some(PathBuf::from(next(&mut arguments, &option)?))
                }
                "--config" => config = PathBuf::from(next(&mut arguments, &option)?),
                "--out-dir" => out_dir = PathBuf::from(next(&mut arguments, &option)?),
                "--max-frames" => {
                    max_frames = Some(
                        next(&mut arguments, &option)?
                            .to_string_lossy()
                            .parse::<usize>()
                            .map_err(|error| format!("invalid --max-frames: {error}"))?,
                    );
                }
                "--optimize-every-k" => {
                    optimize_every_k = next(&mut arguments, &option)?
                        .to_string_lossy()
                        .parse::<usize>()
                        .map_err(|error| format!("invalid --optimize-every-k: {error}"))?;
                }
                "--periodic-iterations" => {
                    periodic_iterations = next(&mut arguments, &option)?
                        .to_string_lossy()
                        .parse::<usize>()
                        .map_err(|error| format!("invalid --periodic-iterations: {error}"))?;
                }
                "--realtime" => realtime = true,
                "--as-fast-as-possible" => realtime = false,
                unknown => return Err(format!("unknown option `{unknown}`\n\n{}", Self::usage())),
            }
        }
        Ok(Self {
            euroc_dir: euroc_dir
                .ok_or_else(|| format!("--euroc-dir is required\n\n{}", Self::usage()))?,
            calibration: calibration
                .ok_or_else(|| format!("--calibration is required\n\n{}", Self::usage()))?,
            config,
            out_dir,
            max_frames,
            optimize_every_k,
            periodic_iterations,
            realtime,
        })
    }

    fn usage() -> String {
        "usage: basalt_euroc_online_slam_demo --euroc-dir DIR --calibration FILE \
         [--config FILE] [--out-dir DIR] [--max-frames N] [--optimize-every-k K] \
         [--periodic-iterations N] [--realtime | --as-fast-as-possible]"
            .into()
    }
}

fn next<I>(arguments: &mut I, option: &str) -> Result<std::ffi::OsString, String>
where
    I: Iterator<Item = std::ffi::OsString>,
{
    arguments
        .next()
        .ok_or_else(|| format!("{option} requires a value"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parser_requires_euroc_dir_and_calibration() {
        assert!(Args::parse(["--calibration", "c.json"].map(Into::into)).is_err());
        assert!(Args::parse(["--euroc-dir", "d"].map(Into::into)).is_err());
    }

    #[test]
    fn parser_defaults_to_as_fast_as_possible() {
        let args = Args::parse(["--euroc-dir", "d", "--calibration", "c.json"].map(Into::into))
            .expect("parses");
        assert!(!args.realtime);
        assert_eq!(
            args.optimize_every_k,
            OnlineMapperConfig::default().optimize_every_k
        );
        assert_eq!(
            args.periodic_iterations,
            OnlineMapperConfig::default().periodic_iterations
        );
    }

    #[test]
    fn parser_accepts_realtime_and_k() {
        let args = Args::parse(
            [
                "--euroc-dir",
                "d",
                "--calibration",
                "c.json",
                "--realtime",
                "--optimize-every-k",
                "5",
                "--periodic-iterations",
                "2",
            ]
            .map(Into::into),
        )
        .expect("parses");
        assert!(args.realtime);
        assert_eq!(args.optimize_every_k, 5);
        assert_eq!(args.periodic_iterations, 2);
    }
}
