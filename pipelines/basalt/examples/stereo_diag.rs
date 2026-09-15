use std::{
    env,
    fs::File,
    io::{BufWriter, Write},
    path::PathBuf,
};

use serde::Serialize;

use visloc_basalt::{direct_klt_config, DirectKltStream, EurocSensorDataset};

#[derive(Serialize)]
struct EndpointObservation {
    track_id: u64,
    x: f64,
    y: f64,
}

#[derive(Serialize)]
struct EndpointRecord {
    trace_schema: &'static str,
    frame_index: usize,
    timestamp_ns: i64,
    cam0: Vec<EndpointObservation>,
    cam1: Vec<EndpointObservation>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = env::args_os().skip(1);
    let root = PathBuf::from(args.next().ok_or("dataset root")?);
    let calib = PathBuf::from(args.next().ok_or("calibration")?);
    let config_path = PathBuf::from(args.next().ok_or("config")?);
    let max_frames: usize = args
        .next()
        .map(|v| v.to_string_lossy().parse())
        .transpose()?
        .unwrap_or(80);
    let endpoint_path = args.next().map(PathBuf::from);
    let mut endpoint = endpoint_path
        .as_ref()
        .map(File::create)
        .transpose()
        .map(|file| file.map(BufWriter::new))?;
    let dataset = EurocSensorDataset::open(root, calib, config_path)?;
    let config = direct_klt_config(dataset.config())?;
    let mut stream = DirectKltStream::new(dataset.calibration().clone(), config)?;
    for i in 0..max_frames.min(dataset.frame_count()) {
        let frame = dataset.frame(i)?;
        let output = stream.process_frame(visloc_basalt::StereoFrame::new(
            frame.frame_id,
            frame.timestamp_ns,
            frame.cam0,
            frame.cam1,
        ))?;
        if let Some(endpoint) = endpoint.as_mut() {
            let mut cam0 = Vec::new();
            let mut cam1 = Vec::new();
            for observation in &output.observations {
                let destination = if observation.camera_id == 0 {
                    &mut cam0
                } else {
                    &mut cam1
                };
                destination.push(EndpointObservation {
                    track_id: observation.track_id,
                    x: observation.pixel.x,
                    y: observation.pixel.y,
                });
            }
            serde_json::to_writer(
                &mut *endpoint,
                &EndpointRecord {
                    trace_schema: "basalt.stereo_endpoint.v1",
                    frame_index: i,
                    timestamp_ns: frame.timestamp_ns,
                    cam0,
                    cam1,
                },
            )?;
            endpoint.write_all(b"\n")?;
        }
        let cam0 = output
            .observations
            .iter()
            .filter(|o| o.camera_id == 0)
            .count();
        let cam1 = output
            .observations
            .iter()
            .filter(|o| o.camera_id == 1)
            .count();
        let counters = output
            .reject_counters
            .iter()
            .map(|(r, c)| format!("{r:?}={c}"))
            .collect::<Vec<_>>()
            .join(";");
        println!(
            "frame={} cam0={} cam1={} created={} retained={} rejected={} rejects={}",
            i,
            cam0,
            cam1,
            output.created_track_ids.len(),
            output.retained_track_ids.len(),
            output.rejected_track_ids.len(),
            counters
        );
        if i < 3 {
            let pts = output
                .observations
                .iter()
                .filter(|o| o.camera_id == 1)
                .take(20)
                .map(|o| format!("{}:{:.6},{:.6}", o.track_id, o.pixel.x, o.pixel.y))
                .collect::<Vec<_>>()
                .join(" ");
            println!("cam1pts {pts}");
        }
    }
    Ok(())
}
