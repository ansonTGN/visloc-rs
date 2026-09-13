//! Focused exact-bit regression for the pinned MH_01 cam1 observation.
//!
//! The test is ignored by default because the EuRoC image archive and Basalt
//! JSON inputs are external benchmark artifacts.  Set the three path
//! variables below and run it explicitly when auditing the optical-flow
//! endpoint.

use std::{env, path::PathBuf};

use visloc_basalt::{direct_klt_config, DirectKltStream, EurocSensorDataset, StereoFrame};

#[test]
#[ignore = "requires external EuRoC MH_01 images and pinned Basalt JSON inputs"]
fn mh01_frame1_cam1_track1_pixel_matches_pinned_bits() {
    let root = PathBuf::from(
        env::var_os("VISLOC_BASALT_MH01_ROOT")
            .expect("set VISLOC_BASALT_MH01_ROOT to the MH_01_easy sensor root"),
    );
    let calibration = PathBuf::from(
        env::var_os("VISLOC_BASALT_CALIBRATION")
            .expect("set VISLOC_BASALT_CALIBRATION to euroc_ds_calib.json"),
    );
    let config = PathBuf::from(
        env::var_os("VISLOC_BASALT_CONFIG").expect("set VISLOC_BASALT_CONFIG to euroc_config.json"),
    );

    let dataset = EurocSensorDataset::open(root, calibration, config).unwrap();
    let mut stream = DirectKltStream::new(
        dataset.calibration().clone(),
        direct_klt_config(dataset.config()).unwrap(),
    )
    .unwrap();

    let mut final_output = None;
    for frame_index in 0..=1 {
        let frame = dataset.frame(frame_index).unwrap();
        let output = stream
            .process_frame(StereoFrame::new(
                frame.frame_id,
                frame.timestamp_ns,
                frame.cam0,
                frame.cam1,
            ))
            .unwrap();
        if frame_index == 1 {
            final_output = Some(output);
        }
    }

    let frame1_output = final_output.expect("frame 1 output must be captured");
    let observation = frame1_output
        .observations
        .iter()
        .find(|observation| observation.camera_id == 1 && observation.track_id == 1)
        .expect("frame 1 cam1 track 1 must be present");
    assert_eq!((observation.pixel.x as f32).to_bits(), 0x41da8172);
    assert_eq!((observation.pixel.y as f32).to_bits(), 0x42d4c7e1);
}

/// Capture the first current cross-platform trajectory mismatch's raw KLT
/// endpoint.  This is deliberately diagnostic-only: the pinned native
/// endpoint is recorded for comparison, but this test does not turn the
/// current MSVC/Linux difference into a pass/fail oracle.
#[test]
#[ignore = "diagnostic-only frame-14 raw frontend endpoint; requires external EuRoC inputs"]
fn mh01_frame14_cam0_track203_raw_klt_endpoint_diagnostic() {
    let root = PathBuf::from(
        env::var_os("VISLOC_BASALT_MH01_ROOT")
            .expect("set VISLOC_BASALT_MH01_ROOT to the MH_01_easy sensor root"),
    );
    let calibration = PathBuf::from(
        env::var_os("VISLOC_BASALT_CALIBRATION")
            .expect("set VISLOC_BASALT_CALIBRATION to euroc_ds_calib.json"),
    );
    let config = PathBuf::from(
        env::var_os("VISLOC_BASALT_CONFIG").expect("set VISLOC_BASALT_CONFIG to euroc_config.json"),
    );
    let output_path = PathBuf::from(
        env::var_os("VISLOC_BASALT_ENDPOINT_DIAGNOSTIC")
            .expect("set VISLOC_BASALT_ENDPOINT_DIAGNOSTIC to an E: output path"),
    );
    let dataset = EurocSensorDataset::open(root, calibration.clone(), config.clone()).unwrap();
    let mut stream = DirectKltStream::new(
        dataset.calibration().clone(),
        direct_klt_config(dataset.config()).unwrap(),
    )
    .unwrap();

    const SOURCE_FRAME: usize = 13;
    const TARGET_FRAME: usize = 14;
    const TRACK_ID: u64 = 203;
    const SOURCE_TIMESTAMP_NS: i64 = 1_403_636_580_413_555_456;
    const TARGET_TIMESTAMP_NS: i64 = 1_403_636_580_463_555_584;
    const NATIVE_SOURCE_X: f64 = 170.6181640625;
    const NATIVE_SOURCE_Y: f64 = 90.51874542236328;
    const NATIVE_TARGET_X: f64 = 170.787841796875;
    const NATIVE_TARGET_Y: f64 = 81.55065155029297;

    let mut source_path = None;
    let mut target_path = None;
    let mut source_observation = None;
    let mut target_observation = None;
    for frame_index in 0..=TARGET_FRAME {
        let frame = dataset.frame(frame_index).unwrap();
        if frame_index == SOURCE_FRAME {
            assert_eq!(frame.timestamp_ns, SOURCE_TIMESTAMP_NS);
            source_path = Some(frame.cam0_path.clone());
        }
        let output = stream
            .process_frame(StereoFrame::new(
                frame.frame_id,
                frame.timestamp_ns,
                frame.cam0,
                frame.cam1,
            ))
            .unwrap();
        if frame_index == SOURCE_FRAME {
            source_observation = output
                .observations
                .iter()
                .find(|observation| observation.camera_id == 0 && observation.track_id == TRACK_ID)
                .map(|observation| (observation.pixel.x, observation.pixel.y));
        } else if frame_index == TARGET_FRAME {
            assert_eq!(frame.timestamp_ns, TARGET_TIMESTAMP_NS);
            target_path = Some(frame.cam0_path.clone());
            target_observation = output
                .observations
                .iter()
                .find(|observation| observation.camera_id == 0 && observation.track_id == TRACK_ID)
                .map(|observation| (observation.pixel.x, observation.pixel.y));
        }
    }
    let (source_x, source_y) = source_observation.expect("frame 13 cam0 track 203 must exist");
    let (target_x, target_y) = target_observation.expect("frame 14 cam0 track 203 must exist");
    let record = serde_json::json!({
        "schema": "visloc.basalt.m11.frame14_raw_frontend_endpoint.v1",
        "source": "rust_direct_klt_stream",
        "camera_id": 0,
        "track_id": TRACK_ID,
        "source_frame_id": SOURCE_FRAME,
        "target_frame_id": TARGET_FRAME,
        "source_timestamp_ns": SOURCE_TIMESTAMP_NS,
        "target_timestamp_ns": TARGET_TIMESTAMP_NS,
        "source_path": source_path,
        "target_path": target_path,
        "decoder": "EurocSensorDataset::frame -> euroc::read_raw_u16_png -> dynamic_to_raw_u16",
        "input": {
            "source_pixel_f64": [source_x, source_y],
            "source_pixel_f32_bits": [(source_x as f32).to_bits(), (source_y as f32).to_bits()],
            "pinned_native_source_pixel_f64": [NATIVE_SOURCE_X, NATIVE_SOURCE_Y],
            "target_pixel_f64": [target_x, target_y],
            "target_pixel_f32_bits": [(target_x as f32).to_bits(), (target_y as f32).to_bits()],
            "pinned_native_target_pixel_f64": [NATIVE_TARGET_X, NATIVE_TARGET_Y]
        },
        "endpoint_delta_from_pinned_native": [
            target_x - NATIVE_TARGET_X,
            target_y - NATIVE_TARGET_Y
        ],
        "source_delta_from_pinned_native": [
            source_x - NATIVE_SOURCE_X,
            source_y - NATIVE_SOURCE_Y
        ],
        "config": {
            "pyramid_levels": direct_klt_config(dataset.config()).unwrap().pyramid_levels,
            "max_iterations": direct_klt_config(dataset.config()).unwrap().max_iterations,
            "calibration": calibration,
            "config": config
        }
    });
    if let Some(parent) = output_path.parent() {
        std::fs::create_dir_all(parent).unwrap();
    }
    std::fs::write(&output_path, serde_json::to_vec_pretty(&record).unwrap()).unwrap();
    println!("{}", serde_json::to_string(&record).unwrap());
}
