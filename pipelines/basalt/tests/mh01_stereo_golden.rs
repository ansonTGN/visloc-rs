//! Sensor-only MH_01 optical-flow parity gate.
//!
//! This test is intentionally ignored in ordinary CI because the EuRoC image
//! archive and the upstream trace are external benchmark artifacts.  When the
//! four paths below are supplied it asserts the complete first-80 stereo
//! track-count and endpoint sequence; it does not merely print a diagnostic.

use std::{
    collections::{BTreeMap, BTreeSet},
    env, fs,
    path::PathBuf,
};

use serde::Deserialize;
use visloc_basalt::vio::{MargData, OfImageData};
use visloc_basalt::{
    direct_klt_config, BasaltVioEstimatorAdapter, DirectKltStream, EurocSensorDataset,
};

const FIRST_80_COUNTS: [(usize, usize); 80] = [
    (135, 61),
    (140, 63),
    (146, 75),
    (161, 81),
    (167, 85),
    (178, 90),
    (179, 89),
    (175, 91),
    (178, 90),
    (182, 84),
    (182, 81),
    (180, 70),
    (174, 64),
    (177, 59),
    (174, 53),
    (171, 49),
    (167, 48),
    (160, 49),
    (156, 50),
    (153, 57),
    (153, 58),
    (152, 61),
    (155, 70),
    (167, 89),
    (168, 95),
    (173, 101),
    (181, 109),
    (185, 116),
    (192, 123),
    (196, 131),
    (198, 134),
    (208, 146),
    (211, 154),
    (210, 155),
    (216, 158),
    (219, 163),
    (226, 164),
    (228, 164),
    (225, 163),
    (209, 148),
    (216, 144),
    (209, 134),
    (208, 131),
    (207, 128),
    (199, 119),
    (194, 115),
    (186, 104),
    (179, 94),
    (173, 83),
    (173, 79),
    (171, 74),
    (169, 74),
    (170, 73),
    (168, 73),
    (165, 74),
    (172, 80),
    (175, 87),
    (184, 93),
    (196, 99),
    (198, 108),
    (199, 116),
    (206, 124),
    (212, 142),
    (210, 141),
    (216, 144),
    (222, 156),
    (229, 164),
    (230, 168),
    (237, 174),
    (238, 172),
    (239, 171),
    (240, 170),
    (229, 163),
    (230, 165),
    (229, 163),
    (225, 157),
    (223, 151),
    (212, 145),
    (201, 137),
    (201, 129),
];

#[derive(Debug, Deserialize)]
struct TraceRecord {
    frame_index: usize,
    timestamp_ns: i64,
    cam0_tracks: usize,
    cam1_tracks: usize,
}

#[derive(Debug, Deserialize)]
struct EndpointObservation {
    track_id: u64,
    x: f64,
    y: f64,
}

#[derive(Debug, Deserialize)]
struct EndpointRecord {
    trace_schema: String,
    #[serde(default)]
    frame_index: Option<usize>,
    timestamp_ns: i64,
    cam0: Vec<EndpointObservation>,
    cam1: Vec<EndpointObservation>,
}

const PINNED_ENDPOINT_TRACE: &str =
    include_str!("../../../benchmarks/basalt/mh01_stereo_endpoint_v1_first80.jsonl");

const PINNED_ENDPOINT_ID_SWAPS: &str =
    include_str!("../../../benchmarks/basalt/mh01_stereo_endpoint_v1_first80_id_swaps.jsonl");

#[derive(Debug, Clone, Deserialize)]
struct EndpointIdSwap {
    frame_index: usize,
    camera: usize,
    upstream_id: u64,
    rust_id: u64,
    upstream_x: f64,
    upstream_y: f64,
    rust_x: f64,
    rust_y: f64,
    distance_px: f64,
}

fn trace_records(path: &PathBuf) -> Vec<TraceRecord> {
    fs::read_to_string(path)
        .expect("upstream MH01 trace must be readable")
        .lines()
        .take(FIRST_80_COUNTS.len())
        .map(|line| serde_json::from_str(line).expect("trace record must be valid JSON"))
        .collect()
}

fn endpoint_records() -> Vec<EndpointRecord> {
    PINNED_ENDPOINT_TRACE
        .lines()
        .take(FIRST_80_COUNTS.len())
        .map(|line| serde_json::from_str(line).expect("endpoint record must be valid JSON"))
        .collect()
}

fn endpoint_id_swaps() -> Vec<EndpointIdSwap> {
    PINNED_ENDPOINT_ID_SWAPS
        .lines()
        .map(|line| serde_json::from_str(line).expect("endpoint ID swap must be valid JSON"))
        .collect()
}

fn endpoint_map(observations: &[EndpointObservation]) -> BTreeMap<u64, (f64, f64)> {
    observations
        .iter()
        .map(|observation| (observation.track_id, (observation.x, observation.y)))
        .collect()
}

fn percentile(sorted: &[f64], fraction: f64) -> f64 {
    assert!(!sorted.is_empty());
    let index = (fraction * sorted.len() as f64).ceil() as usize - 1;
    sorted[index]
}

#[test]
#[ignore = "requires external EuRoC MH_01 images and the frozen upstream trace"]
fn mh01_first80_stereo_endpoints_match_upstream_trace() {
    let root = PathBuf::from(
        env::var_os("VISLOC_BASALT_MH01_ROOT")
            .expect("set VISLOC_BASALT_MH01_ROOT to the MH_01_easy sensor root"),
    );
    let calibration = PathBuf::from(
        env::var_os("VISLOC_BASALT_CALIBRATION")
            .expect("set VISLOC_BASALT_CALIBRATION to euroc_ds_calib.json"),
    );
    let config = PathBuf::from(
        env::var_os("VISLOC_BASALT_CONFIG")
            .expect("set VISLOC_BASALT_CONFIG to the pinned euroc_config.json"),
    );
    let trace_path = PathBuf::from(
        env::var_os("VISLOC_BASALT_TRACE")
            .expect("set VISLOC_BASALT_TRACE to the frozen trace.jsonl"),
    );

    let trace = trace_records(&trace_path);
    assert_eq!(trace.len(), FIRST_80_COUNTS.len());
    for (index, (record, expected)) in trace.iter().zip(FIRST_80_COUNTS).enumerate() {
        assert_eq!(record.frame_index, index);
        assert_eq!((record.cam0_tracks, record.cam1_tracks), expected);
    }
    let endpoint = endpoint_records();
    assert_eq!(endpoint.len(), FIRST_80_COUNTS.len());
    for (index, record) in endpoint.iter().enumerate() {
        assert_eq!(record.trace_schema, "basalt.stereo_endpoint.v1");
        assert_eq!(record.frame_index.unwrap_or(index), index);
        assert_eq!(record.timestamp_ns, trace[index].timestamp_ns);
    }

    let mut swaps = BTreeMap::new();
    for swap in endpoint_id_swaps() {
        assert!(swap.frame_index < FIRST_80_COUNTS.len());
        assert!(swap.camera < 2);
        assert!(
            swaps
                .insert((swap.frame_index, swap.camera, swap.upstream_id), swap)
                .is_none(),
            "duplicate endpoint ID swap fixture entry"
        );
    }

    let dataset = EurocSensorDataset::open(root, calibration, config).unwrap();
    let mut stream = DirectKltStream::new(
        dataset.calibration().clone(),
        direct_klt_config(dataset.config()).unwrap(),
    )
    .unwrap();

    let mut errors = [Vec::new(), Vec::new()];
    for (index, expected) in FIRST_80_COUNTS.into_iter().enumerate() {
        let frame = dataset.frame(index).unwrap();
        let output = stream
            .process_frame(visloc_basalt::StereoFrame::new(
                frame.frame_id,
                frame.timestamp_ns,
                frame.cam0,
                frame.cam1,
            ))
            .unwrap();
        let observed = (
            output
                .observations
                .iter()
                .filter(|observation| observation.camera_id == 0)
                .count(),
            output
                .observations
                .iter()
                .filter(|observation| observation.camera_id == 1)
                .count(),
        );

        // The endpoint error for each camera's track-count sequence is an
        // exact integer zero, asserted rather than emitted as a print-only
        // diagnostic.
        assert_eq!(
            observed, expected,
            "MH01 frame {index} track-count endpoint"
        );
        assert_eq!((observed.0 as i64 - expected.0 as i64).abs(), 0);
        assert_eq!((observed.1 as i64 - expected.1 as i64).abs(), 0);

        for camera in 0..2 {
            let upstream = endpoint_map(if camera == 0 {
                &endpoint[index].cam0
            } else {
                &endpoint[index].cam1
            });
            let rust: BTreeMap<_, _> = output
                .observations
                .iter()
                .filter(|observation| observation.camera_id == camera as u16)
                .map(|observation| {
                    (
                        observation.track_id,
                        (observation.pixel.x, observation.pixel.y),
                    )
                })
                .collect();
            let mut used_rust_ids = BTreeSet::new();
            for (&upstream_id, &(upstream_x, upstream_y)) in &upstream {
                let swap = swaps.get(&(index, camera, upstream_id));
                let rust_id = swap.map_or(upstream_id, |swap| {
                    assert!((swap.upstream_x - upstream_x).abs() <= 1e-12);
                    assert!((swap.upstream_y - upstream_y).abs() <= 1e-12);
                    swap.rust_id
                });
                let (rust_x, rust_y) = rust.get(&rust_id).copied().unwrap_or_else(|| {
                    panic!(
                        "MH01 frame {index} cam{camera} unknown endpoint ID mapping: upstream {upstream_id} -> Rust {rust_id}"
                    )
                });
                assert!(
                    used_rust_ids.insert(rust_id),
                    "endpoint ID mapping is not one-to-one"
                );
                if let Some(swap) = swap {
                    assert!((swap.rust_x - rust_x).abs() <= 1e-3);
                    assert!((swap.rust_y - rust_y).abs() <= 1e-3);
                    let expected_distance = ((swap.rust_x - swap.upstream_x).powi(2)
                        + (swap.rust_y - swap.upstream_y).powi(2))
                    .sqrt();
                    assert!((swap.distance_px - expected_distance).abs() <= 1e-3);
                } else {
                    let error =
                        ((rust_x - upstream_x).powi(2) + (rust_y - upstream_y).powi(2)).sqrt();
                    assert!(
                        error <= 1.0,
                        "MH01 frame {index} cam{camera} unknown endpoint coordinate difference for ID {upstream_id}: {error} px"
                    );
                }
                errors[camera]
                    .push(((rust_x - upstream_x).powi(2) + (rust_y - upstream_y).powi(2)).sqrt());
            }
            let rust_ids: BTreeSet<_> = rust.keys().copied().collect();
            assert_eq!(used_rust_ids, rust_ids);
            for swap in swaps
                .values()
                .filter(|swap| swap.frame_index == index && swap.camera == camera)
            {
                assert!(upstream.contains_key(&swap.upstream_id));
                assert!(rust.contains_key(&swap.rust_id));
            }
        }
    }

    for (camera, camera_errors) in errors.iter_mut().enumerate() {
        camera_errors.sort_by(f64::total_cmp);
        let median = percentile(camera_errors, 0.5);
        let p95 = percentile(camera_errors, 0.95);
        let max = camera_errors.last().copied().unwrap();
        eprintln!(
            "MH01 cam{camera} common endpoint errors: n={} median={median:.17e} p95={p95:.17e} max={max:.17e}",
            camera_errors.len()
        );
        assert!(
            median <= 0.25,
            "MH01 cam{camera} endpoint median {median} > 0.25 px"
        );
        assert!(p95 <= 1.0, "MH01 cam{camera} endpoint p95 {p95} > 1.0 px");
    }
}

#[test]
#[ignore = "requires external EuRoC MH_01 images and pinned calibration/config"]
fn mh01_first_frame_raw_camera_payload_matches_source_and_roundtrips() {
    let root = PathBuf::from(
        env::var_os("VISLOC_BASALT_MH01_ROOT")
            .expect("set VISLOC_BASALT_MH01_ROOT to the MH_01_easy sensor root"),
    );
    let calibration = PathBuf::from(
        env::var_os("VISLOC_BASALT_CALIBRATION")
            .expect("set VISLOC_BASALT_CALIBRATION to euroc_ds_calib.json"),
    );
    let config = PathBuf::from(
        env::var_os("VISLOC_BASALT_CONFIG")
            .expect("set VISLOC_BASALT_CONFIG to the pinned euroc_config.json"),
    );
    let dataset = EurocSensorDataset::open(root, calibration, config).unwrap();
    let frame = dataset.frame(0).unwrap();
    let expected = [
        OfImageData::new(
            frame.frame_id,
            frame.timestamp_ns,
            0,
            frame.cam0.width(),
            frame.cam0.height(),
            frame.cam0.pixels().to_vec(),
        )
        .unwrap(),
        OfImageData::new(
            frame.frame_id,
            frame.timestamp_ns,
            1,
            frame.cam1.as_ref().unwrap().width(),
            frame.cam1.as_ref().unwrap().height(),
            frame.cam1.as_ref().unwrap().pixels().to_vec(),
        )
        .unwrap(),
    ];
    let mut adapter =
        BasaltVioEstimatorAdapter::from_config(dataset.calibration(), dataset.config()).unwrap();
    let output = adapter.process(frame).unwrap();
    assert_eq!(output.estimator.marg_data.of_images, expected);
    assert_eq!(
        output.estimator.marg_data.of_images[0].sample_hash(),
        expected[0].sample_hash()
    );
    assert_eq!(
        output.estimator.marg_data.of_images[1].sample_hash(),
        expected[1].sample_hash()
    );
    assert_eq!(output.estimator.marg_data.of_images[0].width, 752);
    assert_eq!(output.estimator.marg_data.of_images[0].height, 480);
    assert_eq!(output.estimator.marg_data.of_images[1].width, 752);
    assert_eq!(output.estimator.marg_data.of_images[1].height, 480);

    let roundtrip = MargData::from_json(&output.estimator.marg_data.to_json().unwrap()).unwrap();
    assert_eq!(roundtrip, output.estimator.marg_data);
    assert_eq!(
        roundtrip.stable_hash(),
        output.estimator.marg_data.stable_hash()
    );
}
