//! Headless offline mapper for Basalt schema-4 MargData packets.
//!
//! The command accepts only estimator-produced mapper packets, calibration,
//! and mapper configuration. It has no ground-truth input path.

use std::{
    env, fs,
    io::{BufWriter, Write},
    path::PathBuf,
    process,
    time::Instant,
};

use serde_json::{json, Value};
use visloc_basalt::{
    config::BasaltConfig,
    mapper::{NfrMapper, NfrMapperHeadlessConfig, NfrMapperOptimizeReport},
    vio::MargData,
    BasaltCalibration,
};

const PINNED_UPSTREAM_COMMIT: &str = "0f3b2b52c807f70ff4e2973ce253c73329eea7bc";

#[derive(Debug)]
struct Args {
    packet: Option<PathBuf>,
    marg_dir: Option<PathBuf>,
    calibration: PathBuf,
    config: PathBuf,
    out_dir: PathBuf,
    temporal_seed: Option<u32>,
}

fn main() {
    if let Err(error) = run() {
        eprintln!("basalt_mapper_offline_demo: {error}");
        process::exit(1);
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse(env::args_os().skip(1))
        .map_err(|error| std::io::Error::new(std::io::ErrorKind::InvalidInput, error))?;
    let packet_paths = collect_packet_paths(&args)?;

    // Parse and validate every immutable input before creating output state.
    let config_text = fs::read_to_string(&args.config)?;
    let config = BasaltConfig::from_json(&config_text)?;
    let mapper_config = config.mapper_config()?;
    let feature_config = config.offline_mapper_config()?;
    let optimize_config = config.mapper_global_ba_config()?;
    let calibration_text = fs::read_to_string(&args.calibration)?;
    let calibration = BasaltCalibration::from_json_str(&calibration_text)?;

    let started = Instant::now();
    let mut mapper = NfrMapper::with_calibration(mapper_config, calibration);
    mapper.set_feature_config(feature_config);
    mapper.set_optimize_config(optimize_config);
    // Stream packets one at a time instead of parsing the whole
    // `--marg-dir` into a `Vec<MargData>` up front: on LaMAria-scale inputs
    // (1300+ packets) holding every parsed packet's raw image buffers alive
    // for the whole ingest loop, on top of `NfrMapper::img_data`'s own copy
    // of the same pixels (`retain_images`, session.rs), roughly doubles peak
    // RSS during ingest for no benefit -- each packet is only ever read once,
    // by `add_marg_data`. Read, ingest, and let `packet` drop before the next
    // iteration.
    let mut ingest = Vec::with_capacity(packet_paths.len());
    for path in &packet_paths {
        let text = fs::read_to_string(path)?;
        let mut packet = MargData::from_json_checked(&text).map_err(|error| {
            std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!("{}: {error}", path.display()),
            )
        })?;
        let report = mapper.add_marg_data(&mut packet).map_err(|error| {
            std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!("{}: {error:?}", path.display()),
            )
        })?;
        ingest.push(json!({
            "path": path,
            "byte_length": fs::metadata(path)?.len(),
            "input_size": report.input_size,
            "output_size": report.output_size,
            "accepted": report.accepted,
            "frame_pose_count": report.frame_pose_count,
            "relative_pose_factor_count": report.relative_pose_factor_count,
            "roll_pitch_factor_count": report.roll_pitch_factor_count,
            "image_timestamp_count": report.image_timestamp_count,
        }));
        // `packet` (and its raw image buffers) drops here, before the next
        // file is even read.
    }
    if ingest.iter().all(|entry| entry["accepted"] == false) {
        return Err("all MargData packets were rejected by the mapper rank gate".into());
    }

    let headless_config = NfrMapperHeadlessConfig {
        temporal_seed: args.temporal_seed,
        ..NfrMapperHeadlessConfig::default()
    };
    // Always run the streaming/memory-conscious replica of `run_headless`
    // (frees `img_data`'s raw image buffers right after the last stage that
    // reads them -- see `run_headless_streaming` below); per-stage wall-clock
    // timers are an optional diagnostic on top, gated behind
    // BASALT_MAPPER_STAGE_TIMERS.
    let print_stage_timers = env::var_os("BASALT_MAPPER_STAGE_TIMERS").is_some();
    let report = run_headless_streaming(&mut mapper, headless_config, print_stage_timers)?;
    let elapsed_seconds = started.elapsed().as_secs_f64();
    let trajectory_csv = mapper.trajectory_euroc();
    let trajectory_tum = mapper.trajectory_tum();
    let matches_json = Value::Array(
        mapper
            .feature_match_data
            .iter()
            .map(|(&(left, right), data)| {
                let transform = data.t_i_j.matrix();
                json!({
                    "left": [left.frame_id, u64::from(left.cam_id)],
                    "right": [right.frame_id, u64::from(right.cam_id)],
                    "transform": [
                        [transform[(0, 0)], transform[(0, 1)], transform[(0, 2)], transform[(0, 3)]],
                        [transform[(1, 0)], transform[(1, 1)], transform[(1, 2)], transform[(1, 3)]],
                        [transform[(2, 0)], transform[(2, 1)], transform[(2, 2)], transform[(2, 3)]],
                        [transform[(3, 0)], transform[(3, 1)], transform[(3, 2)], transform[(3, 3)]],
                    ],
                    "matches": data.matches,
                    "inliers": data.inliers,
                })
            })
            .collect(),
    );

    let report_json = json!({
        "schema": "basalt.offline_mapper.run.v1",
        "pinned_upstream_commit": PINNED_UPSTREAM_COMMIT,
        "ground_truth_input": false,
        "inputs": {
            "packets": ingest,
            "calibration": {"path": args.calibration, "byte_length": calibration_text.len()},
            "config": {"path": args.config, "byte_length": config_text.len()},
        },
        "headless_config": report.config,
        "detection": {
            "input_timestamp_count": report.detection.input_timestamp_count,
            "eligible_timestamp_count": report.detection.eligible_timestamp_count,
            "processed_image_count": report.detection.processed_image_count,
            "feature_count": report.detection.feature_count,
        },
        "stereo": {
            "input_timestamp_count": report.stereo.input_timestamp_count,
            "attempted_timestamp_count": report.stereo.attempted_timestamp_count,
            "missing_feature_timestamp_count": report.stereo.missing_feature_timestamp_count,
            "raw_match_count": report.stereo.raw_match_count,
            "essential_inlier_count": report.stereo.essential_inlier_count,
            "accepted_pair_count": report.stereo.accepted_pair_count,
            "total_pair_count": report.stereo.total_pair_count,
        },
        "match_all": {
            "input_feature_count": report.match_all.input_feature_count,
            "query_count": report.match_all.query_count,
            "candidate_pair_count": report.match_all.candidate_pair_count,
            "raw_match_count": report.match_all.raw_match_count,
            "cumulative_raw_match_count": report.match_all.cumulative_raw_match_count,
            "cumulative_inlier_match_count": report.match_all.cumulative_inlier_match_count,
            "stereo_match_pair_count": report.match_all.stereo_match_pair_count,
            "temporal_match_pair_count": report.match_all.temporal_match_pair_count,
            "ransac_attempt_count": report.match_all.ransac_attempt_count,
            "accepted_pair_count": report.match_all.accepted_pair_count,
            "total_pair_count": report.match_all.total_pair_count,
        },
        "tracks": {
            "input_pair_count": report.tracks.input_pair_count,
            "inlier_match_count": report.tracks.inlier_match_count,
            "node_count": report.tracks.node_count,
            "component_count_before": report.tracks.component_count_before,
            "component_count_after": report.tracks.component_count_after,
            "total_track_obs_count": report.tracks.total_track_obs_count,
            "average_track_length": report.tracks.average_track_length,
            "exported_track_count": report.tracks.exported_track_count,
            "rejected_conflict_count": report.tracks.rejected_conflict_ids.len(),
            "rejected_short_count": report.tracks.rejected_short_ids.len(),
            "rejected_track_count": report.tracks.rejected_track_ids.len(),
        },
        "setup": {
            "input_track_count": report.setup.input_track_count,
            "attempted_track_count": report.setup.attempted_track_count,
            "accepted_track_count": report.setup.accepted_track_count,
            "skipped_short_track_count": report.setup.skipped_short_track_count,
            "observation_count": report.setup.observation_count,
            "candidate_attempt_count": report.setup.candidate_attempt_count,
            "canonical_hash": report.setup.canonical_hash,
        },
        "first_optimize": optimize_json(&report.first_optimize),
        "filter": report.filter,
        "second_optimize": optimize_json(&report.second_optimize),
        "stages": report.stages,
        "result": {
            "pose_count": report.result.poses.len(),
            "landmark_count": report.result.landmarks.len(),
            "final_point_count": report.final_points.points.len(),
        },
        "elapsed_seconds": elapsed_seconds,
        "outputs": ["map.json", "matches.json", "points.json", "poses.json", "trajectory.csv", "trajectory.tum", "mapper_report.json"],
    });

    fs::create_dir_all(&args.out_dir)?;
    write_json(
        args.out_dir.join("map.json"),
        &serde_json::to_value(&report.result.landmarks)?,
    )?;
    write_json(args.out_dir.join("matches.json"), &matches_json)?;
    write_json(
        args.out_dir.join("poses.json"),
        &serde_json::to_value(&report.result.poses)?,
    )?;
    write_json(
        args.out_dir.join("points.json"),
        &json!({"points": report.final_points.points, "ids": report.final_points.ids}),
    )?;
    fs::write(args.out_dir.join("trajectory.csv"), trajectory_csv)?;
    fs::write(args.out_dir.join("trajectory.tum"), trajectory_tum)?;
    write_json(args.out_dir.join("mapper_report.json"), &report_json)?;
    eprintln!(
        "packets={} poses={} landmarks={} elapsed_seconds={elapsed_seconds:.3} out={}",
        packet_paths.len(),
        report.result.poses.len(),
        report.result.landmarks.len(),
        args.out_dir.display(),
    );
    Ok(())
}

/// Memory-conscious, optionally-timed re-implementation of
/// `NfrMapper::run_headless`.
///
/// This calls only the mapper's existing public stage methods, in the same
/// order `run_headless` uses internally, so it does not change mapper
/// output -- it is an example-local restructuring, not a parity-affecting
/// change. Two things it does differently from a direct `run_headless` call:
///
/// 1. Frees `mapper.img_data`'s raw image buffers right after `match_stereo`
///    returns. `img_data` is read only by `detect_keypoints` and
///    `match_stereo` (session.rs) -- no later stage (`match_all`,
///    `build_tracks`, `setup_opt`, `optimize`, `filter_outliers`,
///    `get_current_points`, `result`) touches it, so on LaMAria-scale inputs
///    (1300+ packets' worth of raw pixels) holding it alive for the rest of
///    the pipeline is pure waste.
/// 2. Optionally prints a wall-clock duration for each stage to stderr (the
///    Stage 0 slowness-diagnosis instrumentation), gated by `print_timers`
///    (set from `BASALT_MAPPER_STAGE_TIMERS`).
fn run_headless_streaming(
    mapper: &mut visloc_basalt::mapper::NfrMapper,
    config: NfrMapperHeadlessConfig,
    print_timers: bool,
) -> Result<visloc_basalt::mapper::NfrMapperHeadlessReport, Box<dyn std::error::Error>> {
    use visloc_basalt::mapper::NfrMapperHeadlessStage;

    macro_rules! timed {
        ($label:expr, $body:expr) => {{
            let t0 = Instant::now();
            let value = $body;
            if print_timers {
                eprintln!(
                    "[stage_timer] {:<24} {:>10.3}s",
                    $label,
                    t0.elapsed().as_secs_f64()
                );
            }
            value
        }};
    }

    let stage_record = |mapper: &visloc_basalt::mapper::NfrMapper,
                        name: &str,
                        point_count: usize,
                        reprojection_error: Option<f64>| {
        NfrMapperHeadlessStage {
            name: name.into(),
            pose_count: mapper.frame_poses.len(),
            landmark_count: mapper.lmdb.num_landmarks(),
            observation_count: mapper.lmdb.num_observations(),
            point_count,
            reprojection_error,
        }
    };

    mapper.feature_corners.clear();
    mapper.hash_index.clear();
    mapper.feature_matches.clear();
    mapper.feature_match_data.clear();
    let detection = timed!("detect_keypoints", mapper.detect_keypoints())?;

    mapper.feature_matches.clear();
    mapper.feature_match_data.clear();
    let stereo = timed!("match_stereo", mapper.match_stereo())?;
    // `img_data` is not read again after this point in the headless
    // lifecycle -- see the function doc above.
    let freed_image_timestamps = mapper.img_data.len();
    // `BTreeMap` is node-based (no spare-capacity buffer to shrink, unlike
    // `Vec`/`HashMap`), so `clear()` alone drops every node -- and with it
    // every `OfImageData::data` pixel buffer -- immediately.
    mapper.img_data.clear();
    if print_timers {
        eprintln!("[stage_timer] freed img_data ({freed_image_timestamps} timestamps)");
    }
    let match_all = timed!(
        "match_all",
        match config.temporal_seed {
            Some(seed) => mapper.match_all_seeded(seed),
            None => mapper.match_all(),
        }
    );
    let tracks = timed!("build_tracks", mapper.build_tracks());
    let setup = timed!("setup_opt", mapper.setup_opt())?;
    let initial_points = timed!("get_current_points_1", mapper.get_current_points());
    let mut stages = vec![stage_record(
        mapper,
        "setup_opt_get_points",
        initial_points.points.len(),
        Some(timed!(
            "compute_reprojection_error_1",
            mapper.compute_reprojection_error()
        )),
    )];

    let first_optimize = timed!("optimize_1", mapper.optimize(config.num_opt_iter))?;
    let optimized_points = timed!("get_current_points_2", mapper.get_current_points());
    stages.push(stage_record(
        mapper,
        "optimize",
        optimized_points.points.len(),
        Some(first_optimize.final_cost),
    ));

    let filter = timed!(
        "filter_outliers",
        mapper.filter_outliers(config.outlier_threshold, config.min_num_obs)
    )?;
    let filtered_points = timed!("get_current_points_3", mapper.get_current_points());
    stages.push(stage_record(
        mapper,
        "filter_get_points",
        filtered_points.points.len(),
        Some(filter.reprojection_error),
    ));

    let second_optimize = timed!("optimize_2", mapper.optimize(config.num_opt_iter))?;
    let optimized_filtered_points = timed!("get_current_points_4", mapper.get_current_points());
    stages.push(stage_record(
        mapper,
        "optimize_filtered",
        optimized_filtered_points.points.len(),
        Some(second_optimize.final_cost),
    ));
    let final_points = timed!("get_current_points_5", mapper.get_current_points());
    let result = timed!("result", mapper.result());
    stages.push(stage_record(
        mapper,
        "final_get_points_result",
        final_points.points.len(),
        Some(second_optimize.final_cost),
    ));

    Ok(visloc_basalt::mapper::NfrMapperHeadlessReport {
        config,
        detection,
        stereo,
        match_all,
        tracks,
        setup,
        first_optimize,
        filter,
        second_optimize,
        initial_points,
        filtered_points,
        final_points,
        result,
        stages,
    })
}

fn optimize_json(report: &NfrMapperOptimizeReport) -> Value {
    json!({
        "requested_iterations": report.requested_iterations,
        "iterations": report.iterations,
        "pose_count": report.pose_count,
        "landmark_count": report.landmark_count,
        "initial_cost": report.initial_cost,
        "final_cost": report.final_cost,
        "accepted_step_count": report.accepted_step_count,
        "rejected_trial_count": report.rejected_trial_count,
        "initial_lambda": report.initial_lambda,
        "final_lambda": report.final_lambda,
        "final_lambda_vee": report.final_lambda_vee,
        "final_state_hash": report.final_state_hash,
        "trace_hash": report.trace_hash,
    })
}

fn collect_packet_paths(args: &Args) -> Result<Vec<PathBuf>, Box<dyn std::error::Error>> {
    let mut paths = if let Some(path) = &args.packet {
        vec![path.clone()]
    } else {
        let mut paths = Vec::new();
        for entry in fs::read_dir(
            args.marg_dir
                .as_ref()
                .expect("argument parser requires a source"),
        )? {
            let path = entry?.path();
            if path
                .extension()
                .is_some_and(|extension| extension == "json")
            {
                paths.push(path);
            }
        }
        paths
    };
    paths.sort();
    if paths.is_empty() {
        return Err("MargData input contains no JSON packets".into());
    }
    Ok(paths)
}

fn write_json(path: PathBuf, value: &Value) -> Result<(), Box<dyn std::error::Error>> {
    let mut writer = BufWriter::new(fs::File::create(path)?);
    serde_json::to_writer_pretty(&mut writer, value)?;
    writer.write_all(b"\n")?;
    writer.flush()?;
    Ok(())
}

impl Args {
    fn parse<I>(arguments: I) -> Result<Self, String>
    where
        I: IntoIterator<Item = std::ffi::OsString>,
    {
        let mut packet = None;
        let mut marg_dir = None;
        let mut calibration = None;
        let mut config = PathBuf::from("configs/basalt/euroc_config.json");
        let mut out_dir = PathBuf::from("target/basalt_mapper_offline_demo");
        let mut temporal_seed = None;
        let mut arguments = arguments.into_iter();
        while let Some(argument) = arguments.next() {
            let option = argument.to_string_lossy();
            match option.as_ref() {
                "--help" | "-h" => return Err(Self::usage()),
                "--packet" => packet = Some(next_path(&mut arguments, &option)?),
                "--marg-dir" => marg_dir = Some(next_path(&mut arguments, &option)?),
                "--calibration" => calibration = Some(next_path(&mut arguments, &option)?),
                "--config" => config = next_path(&mut arguments, &option)?,
                "--out-dir" => out_dir = next_path(&mut arguments, &option)?,
                "--temporal-seed" => {
                    temporal_seed = Some(
                        arguments
                            .next()
                            .ok_or_else(|| format!("{option} requires a value"))?
                            .to_string_lossy()
                            .parse::<u32>()
                            .map_err(|error| format!("invalid --temporal-seed: {error}"))?,
                    );
                }
                unknown => return Err(format!("unknown option `{unknown}`\n\n{}", Self::usage())),
            }
        }
        if packet.is_some() == marg_dir.is_some() {
            return Err(format!(
                "exactly one of --packet or --marg-dir is required\n\n{}",
                Self::usage()
            ));
        }
        let calibration =
            calibration.ok_or_else(|| format!("--calibration is required\n\n{}", Self::usage()))?;
        Ok(Self {
            packet,
            marg_dir,
            calibration,
            config,
            out_dir,
            temporal_seed,
        })
    }

    fn usage() -> String {
        "usage: basalt_mapper_offline_demo (--packet FILE | --marg-dir DIR) --calibration FILE [--config FILE] [--out-dir DIR] [--temporal-seed U32]".into()
    }
}

fn next_path<I>(arguments: &mut I, option: &str) -> Result<PathBuf, String>
where
    I: Iterator<Item = std::ffi::OsString>,
{
    arguments
        .next()
        .map(PathBuf::from)
        .ok_or_else(|| format!("{option} requires a path"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parser_requires_exactly_one_packet_source() {
        let common = ["--calibration", "calib.json"];
        assert!(Args::parse(common.map(Into::into)).is_err());
        let both = [
            "--packet",
            "packet.json",
            "--marg-dir",
            "packets",
            "--calibration",
            "calib.json",
        ];
        assert!(Args::parse(both.map(Into::into)).is_err());
    }

    #[test]
    fn parser_rejects_unknown_ground_truth_option() {
        let args = [
            "--packet",
            "packet.json",
            "--calibration",
            "calib.json",
            "--ground-truth",
            "gt.csv",
        ];
        assert!(Args::parse(args.map(Into::into)).is_err());
    }
}
