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
    let mut packets = packet_paths
        .iter()
        .map(|path| {
            let text = fs::read_to_string(path)?;
            MargData::from_json_checked(&text).map_err(|error| {
                std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    format!("{}: {error}", path.display()),
                )
            })
        })
        .collect::<Result<Vec<_>, std::io::Error>>()?;

    let started = Instant::now();
    let mut mapper = NfrMapper::with_calibration(mapper_config, calibration);
    mapper.set_feature_config(feature_config);
    mapper.set_optimize_config(optimize_config);
    let mut ingest = Vec::with_capacity(packets.len());
    for (path, packet) in packet_paths.iter().zip(&mut packets) {
        let report = mapper.add_marg_data(packet).map_err(|error| {
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
    }
    if ingest.iter().all(|entry| entry["accepted"] == false) {
        return Err("all MargData packets were rejected by the mapper rank gate".into());
    }

    let headless_config = NfrMapperHeadlessConfig {
        temporal_seed: args.temporal_seed,
        ..NfrMapperHeadlessConfig::default()
    };
    let report = mapper.run_headless(headless_config)?;
    let elapsed_seconds = started.elapsed().as_secs_f64();
    let trajectory_csv = mapper.trajectory_euroc();
    let trajectory_tum = mapper.trajectory_tum();

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
        "outputs": ["map.json", "poses.json", "trajectory.csv", "trajectory.tum", "mapper_report.json"],
    });

    fs::create_dir_all(&args.out_dir)?;
    write_json(
        args.out_dir.join("map.json"),
        &serde_json::to_value(&report.result.landmarks)?,
    )?;
    write_json(
        args.out_dir.join("poses.json"),
        &serde_json::to_value(&report.result.poses)?,
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
