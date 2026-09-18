//! Prior-based sequential map-matching localization for RNE episodes.
//!
//! Loads a COLMAP text model plus a `landmark_descriptors.txt` store, then
//! localizes each query feature file in order. The previous successful pose is
//! fed forward as a [`RadiusLandmarkSelector`] prior; frames without a prior
//! (first frame, or after repeated failures) fall back to global matching.
//!
//! Usage:
//! ```text
//! cargo run --release --example localize_rne_map_sequence -- \
//!   --out-dir <dir> [--radius-m 5.0] [--min-inliers 6] \
//!   <colmap_text_dir> <landmark_descriptors.txt> <camera_id> \
//!   <query_features.txt> [query_features_2.txt ...]
//! ```
//!
//! Query filenames are expected to be `<timestamp_ns>.txt`; the timestamp is
//! carried into `localization.tum` (seconds).

use std::fs;
use std::path::{Path, PathBuf};
use std::time::Instant;

use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use visloc_rs::core::geometry::Pose;
use visloc_rs::core::types::{LandmarkDescriptorStore, LocalizationResult, QueryImage, VisualMap};
use visloc_rs::io::colmap::ColmapMapProvider;
use visloc_rs::io::query_features::read_query_features_txt;
use visloc_rs::{DescriptorProvider, LocalizationPipeline, MapProvider, RadiusLandmarkSelector};

fn parse_flag(args: &mut Vec<String>, name: &str) -> Option<String> {
    let index = args.iter().position(|arg| arg == name)?;
    args.remove(index);
    if index < args.len() {
        Some(args.remove(index))
    } else {
        None
    }
}

fn localize_with_prior(
    pipeline: &LocalizationPipeline,
    query: &QueryImage,
    map: &VisualMap,
    store: &LandmarkDescriptorStore,
    pose: &Pose,
    radius_m: f64,
) -> LocalizationResult {
    let selector = RadiusLandmarkSelector::new(pose.camera_center_world(), radius_m);
    pipeline.localize_with_candidate_selector_and_descriptor_store_and_pose_prior(
        query,
        map,
        store,
        selector,
        Some(pose),
    )
}

fn query_timestamp_seconds(path: &Path, fallback_index: usize) -> f64 {
    path.file_stem()
        .and_then(|stem| stem.to_str())
        .and_then(|stem| stem.parse::<f64>().ok())
        .map_or(fallback_index as f64 * 0.05, |nanoseconds| {
            nanoseconds * 1.0e-9
        })
}

/// Load a TUM trajectory as (timestamp_ns, Pose) priors. The TUM quaternion is
/// interpreted as camera-to-world; the Pose stores world-to-camera.
fn load_prior_tum(path: &Path) -> Result<Vec<(i64, Pose)>, Box<dyn std::error::Error>> {
    let mut priors = Vec::new();
    let text = fs::read_to_string(path)?;
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.len() < 8 {
            continue;
        }
        let timestamp_ns = (parts[0].parse::<f64>()? * 1.0e9).round() as i64;
        let center = Vector3::new(parts[1].parse()?, parts[2].parse()?, parts[3].parse()?);
        let rotation_cw = UnitQuaternion::from_quaternion(Quaternion::new(
            parts[7].parse()?,
            parts[4].parse()?,
            parts[5].parse()?,
            parts[6].parse()?,
        ));
        let rotation_wc = rotation_cw.inverse();
        let translation_wc = -(rotation_wc * center);
        priors.push((
            timestamp_ns,
            Pose::from_world_to_camera(rotation_wc, translation_wc),
        ));
    }
    priors.sort_by_key(|(timestamp_ns, _)| *timestamp_ns);
    Ok(priors)
}

fn nearest_prior(priors: &[(i64, Pose)], timestamp_ns: i64, tolerance_ns: i64) -> Option<Pose> {
    let mut best: Option<(i64, Pose)> = None;
    for (candidate_ns, pose) in priors {
        let gap = (candidate_ns - timestamp_ns).abs();
        if gap <= tolerance_ns
            && best
                .as_ref()
                .is_none_or(|(best_ns, _)| gap < (*best_ns - timestamp_ns).abs())
        {
            best = Some((*candidate_ns, pose.clone()));
        }
    }
    best.map(|(_, pose)| pose)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1).collect::<Vec<_>>();
    let out_dir = parse_flag(&mut args, "--out-dir").map(PathBuf::from);
    let radius_m: f64 = parse_flag(&mut args, "--radius-m")
        .map(|value| value.parse())
        .transpose()?
        .unwrap_or(5.0);
    let min_inliers: usize = parse_flag(&mut args, "--min-inliers")
        .map(|value| value.parse())
        .transpose()?
        .unwrap_or(6);
    let prior_tum: Option<Vec<(i64, Pose)>> = parse_flag(&mut args, "--prior-tum")
        .map(|path| load_prior_tum(Path::new(&path)))
        .transpose()?;
    if let Some(priors) = &prior_tum {
        println!("loaded {} trajectory priors", priors.len());
    }
    let (map_dir, descriptor_path, camera_id, query_paths) = match args.as_slice() {
        [map_dir, descriptor_path, camera_id, queries @ ..] if !queries.is_empty() => {
            // Process in timestamp order (filenames are <ns>.txt); lexicographic
            // order is NOT temporal and breaks the feed-forward prior.
            let mut query_paths: Vec<PathBuf> = queries.iter().map(PathBuf::from).collect();
            query_paths.sort_by_cached_key(|p| {
                p.file_stem()
                    .and_then(|stem| stem.to_str())
                    .and_then(|stem| stem.parse::<i64>().ok())
                    .unwrap_or(i64::MAX)
            });
            (
                PathBuf::from(map_dir),
                PathBuf::from(descriptor_path),
                camera_id.parse::<u64>()?,
                query_paths,
            )
        }
        _ => {
            eprintln!(
                "usage: localize_rne_map_sequence --out-dir <dir> [--radius-m F] [--min-inliers N] \
                 <colmap_text_dir> <landmark_descriptors.txt> <camera_id> <query_features.txt> [...]"
            );
            std::process::exit(2);
        }
    };

    let provider = ColmapMapProvider::from_text_model_dir_with_descriptors_validated(
        &map_dir,
        &descriptor_path,
    )?;
    let map = provider.visual_map();
    let store = provider
        .landmark_descriptor_store()
        .ok_or("map provider has no descriptor store")?;
    let camera = map
        .cameras
        .get(&camera_id)
        .cloned()
        .ok_or_else(|| format!("camera id {camera_id} not found in map"))?;
    println!(
        "loaded map: cameras={} keyframes={} landmarks={} descriptors={} queries={}",
        map.cameras.len(),
        map.keyframes.len(),
        map.landmarks.len(),
        store.len(),
        query_paths.len(),
    );

    let pipeline = LocalizationPipeline::default();
    // Seed the prior from the earliest map keyframe (GT-free; the query
    // starts near the map start in the held-out split).
    let mut prior: Option<Pose> = map
        .keyframes
        .iter()
        .min_by_key(|(id, _)| *id)
        .and_then(|(_, keyframe)| keyframe.frame.pose.clone());
    println!("seed prior: {}", prior.is_some());
    let mut consecutive_failures = 0_usize;
    let mut tum = String::new();
    let mut stats =
        String::from("frame,query,success,inliers,inlier_ratio,latency_ms,used_prior,fallback\n");
    let mut localized = 0_usize;

    for (index, query_path) in query_paths.iter().enumerate() {
        let features = read_query_features_txt(query_path)?;
        let query = QueryImage {
            camera: camera.clone(),
            keypoints: features.keypoints,
            descriptors: features.descriptors,
        };
        let timestamp = query_timestamp_seconds(query_path, index);
        let timestamp_ns = (timestamp * 1.0e9).round() as i64;
        // Prefer an external trajectory prior (e.g. VIO) when provided;
        // otherwise fall back to the feed-forward prior.
        let external = prior_tum
            .as_ref()
            .and_then(|priors| nearest_prior(priors, timestamp_ns, 100_000_000));
        let active: Option<Pose> = external.or_else(|| prior.clone());
        let used_prior = active.is_some();
        let start = Instant::now();
        let prior_result = active
            .as_ref()
            .map(|pose| localize_with_prior(&pipeline, &query, map, store, pose, radius_m));
        let prior_accepted = prior_result.as_ref().is_some_and(|result| {
            result.success && result.inlier_count >= min_inliers && result.pose.is_some()
        });
        // Two-stage: prior first; on failure fall back to global matching so a
        // stale prior cannot poison the track and loss is recovered immediately.
        let (result, used_fallback) = if prior_accepted {
            (prior_result.expect("prior accepted"), false)
        } else {
            (
                pipeline.localize_with_descriptor_store(&query, map, store),
                used_prior,
            )
        };
        let latency_ms = start.elapsed().as_secs_f64() * 1000.0;

        let accepted =
            result.success && result.inlier_count >= min_inliers && result.pose.is_some();
        if accepted {
            let pose = result.pose.clone().expect("accepted pose exists");
            let center = pose.camera_center_world();
            let quaternion = *pose.camera_to_world().rotation.quaternion();
            tum.push_str(&format!(
                "{timestamp:.9} {:.9} {:.9} {:.9} {:.9} {:.9} {:.9} {:.9}\n",
                center.x,
                center.y,
                center.z,
                quaternion.i,
                quaternion.j,
                quaternion.k,
                quaternion.w,
            ));
            prior = Some(pose);
            consecutive_failures = 0;
            localized += 1;
        } else {
            consecutive_failures += 1;
            if consecutive_failures >= 10 {
                prior = None;
                consecutive_failures = 0;
            }
        }
        stats.push_str(&format!(
            "{index},{},{},{},{:.4},{:.2},{used_prior},{used_fallback}\n",
            query_path.display(),
            result.success,
            result.inlier_count,
            result.inlier_ratio,
            latency_ms,
        ));
    }

    println!("localized {localized}/{} queries", query_paths.len());
    if let Some(out_dir) = out_dir {
        fs::create_dir_all(&out_dir)?;
        fs::write(out_dir.join("localization.tum"), &tum)?;
        fs::write(out_dir.join("per_frame.csv"), &stats)?;
        println!("wrote {}", out_dir.display());
    } else {
        println!("localization.tum:\n{tum}");
    }
    Ok(())
}
