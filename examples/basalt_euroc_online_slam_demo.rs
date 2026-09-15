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
    collections::{BTreeMap, VecDeque},
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
use visloc_core::geometry::SE3;

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
    /// A background optimize job was *started* (rule 1's rate-limited
    /// trigger; see `mapper_online`'s module doc).
    optimize_trigger_count: usize,
    /// A background optimize job *finished and was merged* -- may lag
    /// `optimize_trigger_count` by up to one in-flight job at any time.
    optimize_merge_count: usize,
    detect_seconds: f64,
    stereo_seconds: f64,
    match_seconds: f64,
    /// Sum/max of merged background jobs' wall time and per-stage
    /// breakdown, for the "where does the time go" report.
    optimize_total_seconds: f64,
    optimize_max_seconds: f64,
    optimize_build_tracks_seconds: f64,
    optimize_setup_opt_seconds: f64,
    optimize_lm_seconds: f64,
    optimize_filter_seconds: f64,
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
        if report.optimize_triggered {
            self.optimize_trigger_count += 1;
        }
        if let Some(breakdown) = report.optimize_merge {
            self.optimize_merge_count += 1;
            self.optimize_total_seconds += breakdown.total_seconds;
            self.optimize_max_seconds = self.optimize_max_seconds.max(breakdown.total_seconds);
            self.optimize_build_tracks_seconds += breakdown.build_tracks_seconds;
            self.optimize_setup_opt_seconds += breakdown.setup_opt_seconds;
            self.optimize_lm_seconds += breakdown.optimize1_seconds + breakdown.optimize2_seconds;
            self.optimize_filter_seconds += breakdown.filter_seconds;
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
        let mapper_processed = std::sync::atomic::AtomicU64::new(0);
        run_mapper_thread(online_mapper, receiver, None, move |report| {
            if let Some(sent_at) = send_times_for_mapper.lock().expect("lock").pop_front() {
                lag_seconds_for_mapper
                    .lock()
                    .expect("lock")
                    .push(sent_at.elapsed().as_secs_f64());
            }
            aggregate_for_mapper.lock().expect("lock").add(report);
            let processed = mapper_processed.fetch_add(1, std::sync::atomic::Ordering::Relaxed) + 1;
            // Live progress: this is the only visibility into the mapper
            // thread once the VIO thread's own frame-count log has finished
            // (mapper packets can queue up behind a slow keyframe, and a
            // periodic optimize's own cost only shows up here). Print every
            // packet during the (typically short) tail so a queue drain or
            // an unexpectedly slow optimize pass is visible live rather than
            // only in the final aggregate JSON.
            let merge_note = match report.optimize_merge {
                Some(breakdown) => format!(
                    " MERGED(total={:.2}s build_tracks={:.2}s setup_opt={:.2}s lm={:.2}s filter={:.2}s)",
                    breakdown.total_seconds,
                    breakdown.build_tracks_seconds,
                    breakdown.setup_opt_seconds,
                    breakdown.optimize1_seconds + breakdown.optimize2_seconds,
                    breakdown.filter_seconds,
                ),
                None => String::new(),
            };
            eprintln!(
                "mapper packet={processed} new_keys={} detect={:.2}s stereo={:.2}s match={:.2}s \
                 optimize_triggered={} accepted_loops={}{merge_note}",
                report.new_key_count,
                report.detect_seconds,
                report.stereo_seconds,
                report.match_seconds,
                report.optimize_triggered,
                report.accepted_loop_pair_count,
            );
        })
    });

    let mut first_timestamp_ns: Option<i64> = None;
    let mut last_timestamp_ns: i64 = 0;
    let mut total_imu = 0usize;
    let mut total_observations = 0usize;
    let mut mapper_packet_count = 0u64;
    let mut max_queue_depth = 0usize;
    // Every processed frame's raw VIO body-to-world pose, keyed by frame_id.
    // Used after the run to propagate the mapper's keyframe corrections to
    // every frame (see `propagate_to_all_frames` below) -- the same rigid
    // spanning-tree convention as scripts/propagate_basalt_mapper_corrections.py.
    let mut vio_trajectory: BTreeMap<u64, (i64, SE3)> = BTreeMap::new();
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
        vio_trajectory.insert(
            output.tracks.frame.frame_id,
            (timestamp_ns, output.estimator.state.imu_to_world.clone()),
        );

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

    // Propagate the mapper's keyframe corrections to every VIO frame (rigid
    // spanning-tree: scripts/propagate_basalt_mapper_corrections.py's exact
    // convention, ported to Rust). `final_report.trajectory_tum` remains the
    // keyframe-only trajectory (written separately below); this is the
    // full-frame trajectory the sweep evaluates, matching the offline
    // path's `trajectory_full_propagated.tum` protocol.
    let mapper_poses: BTreeMap<u64, SE3> = final_report
        .result
        .poses
        .iter()
        .map(|record| {
            let [qw, qx, qy, qz] = record.quaternion_wxyz;
            let pose = SE3::new(
                nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                    qw, qx, qy, qz,
                )),
                nalgebra::Vector3::new(
                    record.translation[0],
                    record.translation[1],
                    record.translation[2],
                ),
            );
            (record.frame_id, pose)
        })
        .collect();
    let propagated = propagate_to_all_frames(&vio_trajectory, &mapper_poses)?;
    let full_trajectory_tum = {
        let mut buffer = String::from("# timestamp tx ty tz qx qy qz qw\n");
        for (timestamp_ns, pose) in &propagated {
            let q = pose.rotation.quaternion();
            buffer.push_str(&format!(
                "{:.18e} {:.18e} {:.18e} {:.18e} {:.18e} {:.18e} {:.18e} {:.18e}\n",
                *timestamp_ns as f64 * 1e-9,
                pose.translation.x,
                pose.translation.y,
                pose.translation.z,
                q.i,
                q.j,
                q.k,
                q.w,
            ));
        }
        buffer
    };
    let propagated_frame_count = propagated.len();

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
            "optimize_trigger_count": aggregate.optimize_trigger_count,
            "optimize_merge_count": aggregate.optimize_merge_count,
            "detect_seconds": aggregate.detect_seconds,
            "stereo_seconds": aggregate.stereo_seconds,
            "match_seconds": aggregate.match_seconds,
            "optimize_total_seconds": aggregate.optimize_total_seconds,
            "optimize_max_seconds": aggregate.optimize_max_seconds,
            "optimize_build_tracks_seconds": aggregate.optimize_build_tracks_seconds,
            "optimize_setup_opt_seconds": aggregate.optimize_setup_opt_seconds,
            "optimize_lm_seconds": aggregate.optimize_lm_seconds,
            "optimize_filter_seconds": aggregate.optimize_filter_seconds,
        },
        "final_optimize": {
            "first_optimize_final_cost": final_report.first_optimize.final_cost,
            "second_optimize_final_cost": final_report.second_optimize.final_cost,
            "pose_count": final_report.result.poses.len(),
            "landmark_count": final_report.result.landmarks.len(),
        },
        "propagated_frame_count": propagated_frame_count,
        "peak_working_set_bytes": peak_rss_bytes,
    });

    // `trajectory_online.tum` is the full-frame propagated trajectory (every
    // VIO frame, keyframe corrections applied) -- the file the evaluation
    // sweep scores, matching the offline path's protocol.
    // `trajectory_online_kf.tum` is the mapper's own keyframe-only output,
    // kept for debugging/comparison.
    fs::write(
        args.out_dir.join("trajectory_online.tum"),
        &full_trajectory_tum,
    )?;
    fs::write(
        args.out_dir.join("trajectory_online_kf.tum"),
        &final_report.trajectory_tum,
    )?;
    fs::write(
        args.out_dir.join("timing_breakdown_online.json"),
        serde_json::to_string_pretty(&summary)?,
    )?;
    println!("{}", serde_json::to_string_pretty(&summary)?);
    eprintln!(
        "pacing={} rtf={:.3} vio_wall={:.1}s total_wall={:.1}s max_queue_depth={} \
         lag_max={:.3}s lag_mean={:.3}s loops={} triggers={} merges={} \
         optimize_max={:.1}s optimize_total={:.1}s peak_rss={:.0}MB out={}",
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
        aggregate.optimize_trigger_count,
        aggregate.optimize_merge_count,
        aggregate.optimize_max_seconds,
        aggregate.optimize_total_seconds,
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

/// Rigid spanning-tree propagation of the mapper's keyframe corrections to
/// every VIO frame. A direct Rust port of
/// `scripts/propagate_basalt_mapper_corrections.py::propagate` (kept
/// numerically equivalent, not merely similar -- see that script's
/// module docstring for the exact convention):
///
/// ```text
/// V_f  = raw VIO body-to-world pose at frame f (rotation R_f, translation t_f)
/// M_k  = mapper-corrected body-to-world pose at the nearest preceding
///        keyframe k (frame_id <= f)
/// V_k  = raw VIO body-to-world pose at that same keyframe k
///
/// corrected(f) = Delta_k * V_f,  where Delta_k = M_k * V_k^{-1}
/// ```
///
/// i.e. each frame keeps its VIO-derived relative motion to the nearest
/// preceding keyframe; only the keyframe's mapper correction moves it. This
/// is exact at keyframes themselves (f == k, `Delta_k * V_k == M_k`) and,
/// like the Python original, uses the *first* keyframe's delta for any
/// frame before it (there is no earlier keyframe to interpolate from).
fn propagate_to_all_frames(
    vio_trajectory: &BTreeMap<u64, (i64, SE3)>,
    mapper_poses: &BTreeMap<u64, SE3>,
) -> Result<Vec<(i64, SE3)>, Box<dyn std::error::Error>> {
    let keyframe_ids: Vec<u64> = mapper_poses.keys().copied().collect();
    if keyframe_ids.is_empty() {
        return Err("mapper produced no keyframe poses".into());
    }
    let missing: Vec<u64> = keyframe_ids
        .iter()
        .copied()
        .filter(|k| !vio_trajectory.contains_key(k))
        .collect();
    if !missing.is_empty() {
        return Err(format!(
            "{} mapper keyframe frame_ids are absent from the VIO trajectory (first few: {:?})",
            missing.len(),
            &missing[..missing.len().min(5)]
        )
        .into());
    }

    let delta_by_keyframe: BTreeMap<u64, SE3> = keyframe_ids
        .iter()
        .map(|&k| {
            let (_, v_k) = &vio_trajectory[&k];
            let m_k = &mapper_poses[&k];
            (k, m_k.compose(&v_k.inverse()))
        })
        .collect();

    let mut output = Vec::with_capacity(vio_trajectory.len());
    let mut cursor = 0usize;
    for (&frame_id, (timestamp_ns, v_f)) in vio_trajectory {
        while cursor + 1 < keyframe_ids.len() && keyframe_ids[cursor + 1] <= frame_id {
            cursor += 1;
        }
        let delta = &delta_by_keyframe[&keyframe_ids[cursor]];
        output.push((*timestamp_ns, delta.compose(v_f)));
    }
    Ok(output)
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

    fn se3(tx: f64, ty: f64, tz: f64, yaw_deg: f64) -> SE3 {
        SE3::new(
            nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, yaw_deg.to_radians()),
            nalgebra::Vector3::new(tx, ty, tz),
        )
    }

    #[test]
    fn propagation_is_exact_at_keyframes() {
        let mut vio = BTreeMap::new();
        vio.insert(0, (0, se3(0.0, 0.0, 0.0, 0.0)));
        vio.insert(10, (10, se3(1.0, 0.0, 0.0, 0.0)));
        let mut mapper = BTreeMap::new();
        // A keyframe correction that moves frame 10 sideways and rotates it.
        mapper.insert(0, se3(0.0, 0.0, 0.0, 0.0));
        mapper.insert(10, se3(1.5, 0.2, 0.0, 5.0));

        let propagated = propagate_to_all_frames(&vio, &mapper).expect("propagates");
        let by_frame: BTreeMap<i64, &SE3> = propagated
            .iter()
            .map(|(timestamp_ns, pose)| (*timestamp_ns, pose))
            .collect();
        let corrected_10 = by_frame[&10];
        assert!((corrected_10.translation - mapper[&10].translation).norm() < 1e-9);
        assert!(corrected_10.rotation.angle_to(&mapper[&10].rotation).abs() < 1e-9);
    }

    #[test]
    fn propagation_keeps_relative_motion_to_nearest_preceding_keyframe() {
        let mut vio = BTreeMap::new();
        vio.insert(0, (0, se3(0.0, 0.0, 0.0, 0.0)));
        vio.insert(5, (5, se3(0.5, 0.0, 0.0, 0.0))); // non-keyframe, between 0 and 10
        vio.insert(10, (10, se3(1.0, 0.0, 0.0, 0.0)));
        let mut mapper = BTreeMap::new();
        mapper.insert(0, se3(2.0, 0.0, 0.0, 0.0)); // shift everything by +2.0 in x
        mapper.insert(10, se3(3.0, 0.0, 0.0, 0.0));

        let propagated = propagate_to_all_frames(&vio, &mapper).expect("propagates");
        let by_frame: BTreeMap<i64, &SE3> = propagated
            .iter()
            .map(|(timestamp_ns, pose)| (*timestamp_ns, pose))
            .collect();
        // Frame 5's raw VIO relative motion from keyframe 0 is +0.5 in x;
        // keyframe 0's correction shifts everything by +2.0, so frame 5
        // should land at 2.5, not be independently corrected.
        assert!((by_frame[&5].translation.x - 2.5).abs() < 1e-9);
    }

    #[test]
    fn propagation_rejects_a_keyframe_missing_from_the_vio_trajectory() {
        let mut vio = BTreeMap::new();
        vio.insert(0, (0, se3(0.0, 0.0, 0.0, 0.0)));
        let mut mapper = BTreeMap::new();
        mapper.insert(0, se3(0.0, 0.0, 0.0, 0.0));
        mapper.insert(99, se3(1.0, 0.0, 0.0, 0.0)); // not in vio_trajectory
        assert!(propagate_to_all_frames(&vio, &mapper).is_err());
    }
}
