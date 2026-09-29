//! Post-hoc Sim(3) merge of independently-registered `generalized_rig_sfm`
//! COLMAP-text components, using cross-component verified pairs that the
//! frontend already produced (no new matching/verification required).
//!
//! `generalized_rig_sfm --max-models N` reconstructs disjoint components via
//! independent seed-and-grow passes; each component gets its own gauge. When
//! two components are temporally adjacent (the camera never actually lost
//! track), the frontend's temporal-pyramid candidates frequently already
//! verify cross-component pairs that the incremental mapper never uses,
//! because by the time growth reaches the boundary the neighboring frames
//! were already claimed by an earlier or later independent seed. This tool
//! recovers that: it resolves matched keypoints in cross-component verified
//! pairs against each component's own triangulated tracks (points3D.txt), so
//! a verified 2D-2D correspondence with a 3D point on each side yields one
//! 3D-3D correspondence between the two components' gauges. RANSAC + Umeyama
//! over those correspondences gives the Sim(3) registering one gauge into
//! the other; accepted edges are unioned into clusters and every non-root
//! component in a cluster is rewritten (poses + landmarks) into its root's
//! gauge and merged into one COLMAP text model per cluster.
//!
//! Components that do not verify-merge with anything (a real gap: no cross
//! pairs exist because the camera actually lost view continuity) pass
//! through unchanged as singleton clusters, so the output always covers
//! every input image and never loses data even if zero merges succeed.
//!
//! With `--refine`, each cluster with more than one merged component also
//! gets a rig-aware joint bundle adjustment pass (`BaRigObservation`, one
//! pose per image with an identity `sensor_from_rig`, since each component's
//! poses are already per-image/per-camera absolute rather than per-rig-frame)
//! to clean up the Sim(3)-stitched seam before the final poses are written.

use std::collections::{HashMap, HashSet};
use std::env;
use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use nalgebra::{Point3, Quaternion, UnitQuaternion, Vector3};

use visloc_rs::io::colmap::parse_cameras_txt;
use visloc_rs::{
    verified_pair_snapshot, BaConfig, BaRigObservation, BundleAdjustment, Camera, LinearSolver,
    Pose, RobustKernel, Sim3, SE3,
};

#[derive(Debug, Clone)]
struct ImageRow {
    local_id: u64,
    name: String,
    q: UnitQuaternion<f64>,
    t: Vector3<f64>,
    camera_id: u64,
    /// `(x, y, point3d_id)`; index in this vec IS the keypoint/feature index
    /// (matches the verified-pairs snapshot's per-image match indices, since
    /// both were written from the same original per-image `FeatureSet`).
    points: Vec<(f64, f64, i64)>,
}

#[derive(Debug, Clone)]
struct Point3DRow {
    id: u64,
    xyz: Point3<f64>,
    rgb: (u8, u8, u8),
    error: f64,
    /// `(local_image_id, keypoint_index)`.
    track: Vec<(u64, u64)>,
}

/// An unordered pair of component indices, `(min, max)`.
type ComponentPairKey = (usize, usize);
/// 3D-3D correspondences between two components' gauges: `(point in the
/// smaller-index component's frame, point in the larger-index component's
/// frame)`.
type CorrespondenceMap = HashMap<ComponentPairKey, Vec<(Point3<f64>, Point3<f64>)>>;

#[derive(Debug, Clone)]
struct ComponentModel {
    /// Directory name, e.g. "component-000".
    name: String,
    cameras: Vec<Camera>,
    images: Vec<ImageRow>,
    points3d: Vec<Point3DRow>,
}

fn parse_images_txt_with_names(contents: &str) -> Result<Vec<ImageRow>, Box<dyn Error>> {
    let mut rows = Vec::new();
    let mut lines = contents
        .lines()
        .map(str::trim)
        .filter(|line| !line.starts_with('#'));
    while let Some(header) = lines.next() {
        if header.is_empty() {
            continue;
        }
        let tokens: Vec<&str> = header.split_whitespace().collect();
        if tokens.len() < 10 {
            return Err(format!("malformed images.txt header: {header}").into());
        }
        let local_id: u64 = tokens[0].parse()?;
        let qw: f64 = tokens[1].parse()?;
        let qx: f64 = tokens[2].parse()?;
        let qy: f64 = tokens[3].parse()?;
        let qz: f64 = tokens[4].parse()?;
        let tx: f64 = tokens[5].parse()?;
        let ty: f64 = tokens[6].parse()?;
        let tz: f64 = tokens[7].parse()?;
        let camera_id: u64 = tokens[8].parse()?;
        let name = tokens[9].to_owned();
        let q = UnitQuaternion::from_quaternion(Quaternion::new(qw, qx, qy, qz));
        let t = Vector3::new(tx, ty, tz);

        let points_line = lines.next().unwrap_or("");
        let point_tokens: Vec<&str> = points_line.split_whitespace().collect();
        if point_tokens.len() % 3 != 0 {
            return Err(format!("malformed images.txt points row for {name}").into());
        }
        let mut points = Vec::with_capacity(point_tokens.len() / 3);
        for chunk in point_tokens.chunks(3) {
            let x: f64 = chunk[0].parse()?;
            let y: f64 = chunk[1].parse()?;
            let point3d_id: i64 = chunk[2].parse()?;
            points.push((x, y, point3d_id));
        }
        rows.push(ImageRow {
            local_id,
            name,
            q,
            t,
            camera_id,
            points,
        });
    }
    Ok(rows)
}

fn parse_points3d_txt_with_tracks(contents: &str) -> Result<Vec<Point3DRow>, Box<dyn Error>> {
    let mut rows = Vec::new();
    for line in contents.lines().map(str::trim) {
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let tokens: Vec<&str> = line.split_whitespace().collect();
        if tokens.len() < 8 || (tokens.len() - 8) % 2 != 0 {
            return Err(format!("malformed points3D.txt row: {line}").into());
        }
        let id: u64 = tokens[0].parse()?;
        let xyz = Point3::new(tokens[1].parse()?, tokens[2].parse()?, tokens[3].parse()?);
        let rgb = (tokens[4].parse()?, tokens[5].parse()?, tokens[6].parse()?);
        let error: f64 = tokens[7].parse()?;
        let mut track = Vec::new();
        let mut rest = tokens[8..].iter();
        while let (Some(image_id), Some(point2d_idx)) = (rest.next(), rest.next()) {
            track.push((image_id.parse()?, point2d_idx.parse()?));
        }
        rows.push(Point3DRow {
            id,
            xyz,
            rgb,
            error,
            track,
        });
    }
    Ok(rows)
}

fn parse_component(dir: &Path) -> Result<ComponentModel, Box<dyn Error>> {
    let name = dir
        .file_name()
        .ok_or("component directory has no name")?
        .to_string_lossy()
        .into_owned();
    let cameras_txt = fs::read_to_string(dir.join("cameras.txt"))?;
    let cameras = parse_cameras_txt(&cameras_txt)?;
    let images_txt = fs::read_to_string(dir.join("images.txt"))?;
    let images = parse_images_txt_with_names(&images_txt)?;
    let points3d_txt = fs::read_to_string(dir.join("points3D.txt"))?;
    let points3d = parse_points3d_txt_with_tracks(&points3d_txt)?;
    Ok(ComponentModel {
        name,
        cameras,
        images,
        points3d,
    })
}

fn format_f(value: f64) -> String {
    format!("{value:.9}")
}

fn write_component(dir: &Path, model: &ComponentModel) -> Result<(), Box<dyn Error>> {
    fs::create_dir_all(dir)?;
    let mut cameras_text = String::from("# CAMERA_ID MODEL WIDTH HEIGHT PARAMS[]\n");
    for camera in &model.cameras {
        let model_name = camera
            .model
            .colmap_name()
            .ok_or_else(|| format!("camera {} has no COLMAP-exportable model", camera.id))?;
        cameras_text.push_str(&format!(
            "{} {} {} {}",
            camera.id, model_name, camera.width, camera.height,
        ));
        for param in &camera.params {
            cameras_text.push_str(&format!(" {}", format_f(*param)));
        }
        cameras_text.push('\n');
    }
    fs::write(dir.join("cameras.txt"), cameras_text)?;

    let mut images_text = String::from(
        "# IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME\n# POINTS2D[] as X Y POINT3D_ID\n",
    );
    let mut sorted_images = model.images.clone();
    sorted_images.sort_by_key(|row| row.local_id);
    for row in &sorted_images {
        images_text.push_str(&format!(
            "{} {} {} {} {} {} {} {} {} {}\n",
            row.local_id,
            format_f(row.q.w),
            format_f(row.q.i),
            format_f(row.q.j),
            format_f(row.q.k),
            format_f(row.t.x),
            format_f(row.t.y),
            format_f(row.t.z),
            row.camera_id,
            row.name,
        ));
        let point_tokens: Vec<String> = row
            .points
            .iter()
            .map(|(x, y, id)| format!("{} {} {}", format_f(*x), format_f(*y), id))
            .collect();
        images_text.push_str(&point_tokens.join(" "));
        images_text.push('\n');
    }
    fs::write(dir.join("images.txt"), images_text)?;

    let mut points3d_text =
        String::from("# POINT3D_ID X Y Z R G B ERROR TRACK[] as IMAGE_ID POINT2D_IDX\n");
    for point in &model.points3d {
        points3d_text.push_str(&format!(
            "{} {} {} {} {} {} {} {}",
            point.id,
            format_f(point.xyz.x),
            format_f(point.xyz.y),
            format_f(point.xyz.z),
            point.rgb.0,
            point.rgb.1,
            point.rgb.2,
            format_f(point.error),
        ));
        for (image_id, keypoint) in &point.track {
            points3d_text.push_str(&format!(" {image_id} {keypoint}"));
        }
        points3d_text.push('\n');
    }
    fs::write(dir.join("points3D.txt"), points3d_text)?;
    Ok(())
}

/// Apply a Sim(3) to a world-to-camera pose: `X_cam_new = s * X_cam_old` when
/// re-expressed against a world rescaled/rotated/translated by `sim`
/// (`X_world_new = s * R * X_world_old + t`). Derivation (also cross-checked
/// via the camera-center form `C_new = sim.transform_point(C_old)`):
///   R_wc_new = R_wc_old * R_sim^{-1}
///   t_wc_new = s * t_wc_old - R_wc_new * t_sim
fn transform_pose(
    sim: &Sim3,
    q_old: &UnitQuaternion<f64>,
    t_old: &Vector3<f64>,
) -> (UnitQuaternion<f64>, Vector3<f64>) {
    let q_new = q_old * sim.rotation.inverse();
    let t_new = sim.scale * t_old - q_new.transform_vector(&sim.translation);
    (q_new, t_new)
}

fn transform_component(model: &ComponentModel, sim: &Sim3) -> ComponentModel {
    let images = model
        .images
        .iter()
        .map(|row| {
            let (q, t) = transform_pose(sim, &row.q, &row.t);
            ImageRow {
                q,
                t,
                ..row.clone()
            }
        })
        .collect();
    let points3d = model
        .points3d
        .iter()
        .map(|point| Point3DRow {
            xyz: sim.transform_point(&point.xyz),
            ..point.clone()
        })
        .collect();
    ComponentModel {
        name: model.name.clone(),
        cameras: model.cameras.clone(),
        images,
        points3d,
    }
}

/// Small dependency-free xorshift64* RNG, deterministic given a seed. Used
/// only for RANSAC minimal-sample selection; not cryptographic.
struct XorShift64(u64);

impl XorShift64 {
    fn new(seed: u64) -> Self {
        Self(seed.wrapping_mul(0x9E3779B97F4A7C15) | 1)
    }
    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }
    fn next_below(&mut self, bound: usize) -> usize {
        (self.next_u64() % bound as u64) as usize
    }
    /// Three distinct indices in `0..n` via rejection sampling.
    fn sample_three(&mut self, n: usize) -> Option<[usize; 3]> {
        if n < 3 {
            return None;
        }
        let a = self.next_below(n);
        let mut b = self.next_below(n);
        while b == a {
            b = self.next_below(n);
        }
        let mut c = self.next_below(n);
        while c == a || c == b {
            c = self.next_below(n);
        }
        Some([a, b, c])
    }
}

fn ransac_sim3(
    correspondences: &[(Point3<f64>, Point3<f64>)],
    iters: usize,
    inlier_threshold_m: f64,
    seed: u64,
) -> Option<(Sim3, Vec<usize>)> {
    if correspondences.len() < 3 {
        return None;
    }
    let mut rng = XorShift64::new(seed);
    let mut best: Option<(Sim3, Vec<usize>)> = None;
    for _ in 0..iters {
        let Some(sample) = rng.sample_three(correspondences.len()) else {
            break;
        };
        let source: Vec<Point3<f64>> = sample.iter().map(|&i| correspondences[i].0).collect();
        let target: Vec<Point3<f64>> = sample.iter().map(|&i| correspondences[i].1).collect();
        let Some(transform) = visloc_rs::umeyama_similarity_transform(&source, &target, true)
        else {
            continue;
        };
        let rotation = UnitQuaternion::from_rotation_matrix(&transform.rotation);
        let sim = Sim3::new(rotation, transform.translation, transform.scale);
        let inliers: Vec<usize> = (0..correspondences.len())
            .filter(|&i| {
                let (source, target) = correspondences[i];
                (sim.transform_point(&source) - target).norm() <= inlier_threshold_m
            })
            .collect();
        let better = match &best {
            None => true,
            Some((_, best_inliers)) => inliers.len() > best_inliers.len(),
        };
        if better {
            best = Some((sim, inliers));
        }
    }
    // Refit on all inliers of the best sample for a lower-variance estimate.
    let (_, inliers) = best?;
    if inliers.len() < 3 {
        return None;
    }
    let source: Vec<Point3<f64>> = inliers.iter().map(|&i| correspondences[i].0).collect();
    let target: Vec<Point3<f64>> = inliers.iter().map(|&i| correspondences[i].1).collect();
    let refit = visloc_rs::umeyama_similarity_transform(&source, &target, true)?;
    let rotation = UnitQuaternion::from_rotation_matrix(&refit.rotation);
    let sim = Sim3::new(rotation, refit.translation, refit.scale);
    // Recompute inliers against the refit transform.
    let final_inliers: Vec<usize> = (0..correspondences.len())
        .filter(|&i| {
            let (source, target) = correspondences[i];
            (sim.transform_point(&source) - target).norm() <= inlier_threshold_m
        })
        .collect();
    if final_inliers.len() < 3 {
        return None;
    }
    Some((sim, final_inliers))
}

struct Args {
    components_dir: PathBuf,
    snapshot: PathBuf,
    output_dir: PathBuf,
    min_cross_pairs: usize,
    min_correspondences: usize,
    min_inliers: usize,
    min_inlier_ratio: f64,
    ransac_iters: usize,
    inlier_threshold_m: f64,
    max_scale_deviation: f64,
    seed: u64,
    refine: bool,
    rig_manifest: Option<PathBuf>,
    ba_max_iterations: usize,
    ba_max_reprojection_error_px: f64,
    report_json: Option<PathBuf>,
}

fn parse_args() -> Result<Args, Box<dyn Error>> {
    let mut components_dir = None;
    let mut snapshot = None;
    let mut output_dir = None;
    let mut min_cross_pairs = 20usize;
    let mut min_correspondences = 12usize;
    let mut min_inliers = 6usize;
    let mut min_inlier_ratio = 0.5f64;
    let mut ransac_iters = 4000usize;
    let mut inlier_threshold_m = 0.3f64;
    let mut max_scale_deviation = 0.3f64;
    let mut seed = 1u64;
    let mut refine = false;
    let mut rig_manifest = None;
    let mut ba_max_iterations = 20usize;
    let mut ba_max_reprojection_error_px = 8.0f64;
    let mut report_json = None;

    let mut args = env::args().skip(1);
    while let Some(flag) = args.next() {
        match flag.as_str() {
            "--components-dir" => {
                components_dir = Some(PathBuf::from(
                    args.next().ok_or("--components-dir requires PATH")?,
                ))
            }
            "--snapshot" => {
                snapshot = Some(PathBuf::from(
                    args.next().ok_or("--snapshot requires PATH")?,
                ))
            }
            "--output-dir" => {
                output_dir = Some(PathBuf::from(
                    args.next().ok_or("--output-dir requires PATH")?,
                ))
            }
            "--min-cross-pairs" => {
                min_cross_pairs = args.next().ok_or("--min-cross-pairs requires N")?.parse()?
            }
            "--min-correspondences" => {
                min_correspondences = args
                    .next()
                    .ok_or("--min-correspondences requires N")?
                    .parse()?
            }
            "--min-inliers" => {
                min_inliers = args.next().ok_or("--min-inliers requires N")?.parse()?
            }
            "--min-inlier-ratio" => {
                min_inlier_ratio = args
                    .next()
                    .ok_or("--min-inlier-ratio requires F")?
                    .parse()?
            }
            "--ransac-iters" => {
                ransac_iters = args.next().ok_or("--ransac-iters requires N")?.parse()?
            }
            "--inlier-threshold-m" => {
                inlier_threshold_m = args
                    .next()
                    .ok_or("--inlier-threshold-m requires F")?
                    .parse()?
            }
            "--max-scale-deviation" => {
                max_scale_deviation = args
                    .next()
                    .ok_or("--max-scale-deviation requires F")?
                    .parse()?
            }
            "--seed" => seed = args.next().ok_or("--seed requires U64")?.parse()?,
            "--refine" => refine = true,
            "--rig-manifest" => {
                rig_manifest = Some(PathBuf::from(
                    args.next().ok_or("--rig-manifest requires PATH")?,
                ))
            }
            "--ba-max-iterations" => {
                ba_max_iterations = args
                    .next()
                    .ok_or("--ba-max-iterations requires N")?
                    .parse()?
            }
            "--ba-max-reprojection-error-px" => {
                ba_max_reprojection_error_px = args
                    .next()
                    .ok_or("--ba-max-reprojection-error-px requires F")?
                    .parse()?
            }
            "--report-json" => {
                report_json = Some(PathBuf::from(
                    args.next().ok_or("--report-json requires PATH")?,
                ))
            }
            other => return Err(format!("unknown flag: {other}").into()),
        }
    }
    Ok(Args {
        components_dir: components_dir.ok_or("--components-dir is required")?,
        snapshot: snapshot.ok_or("--snapshot is required")?,
        output_dir: output_dir.ok_or("--output-dir is required")?,
        min_cross_pairs,
        min_correspondences,
        min_inliers,
        min_inlier_ratio,
        ransac_iters,
        inlier_threshold_m,
        max_scale_deviation,
        seed,
        refine,
        rig_manifest,
        ba_max_iterations,
        ba_max_reprojection_error_px,
        report_json,
    })
}

/// Parsed `generalized-rig-manifest-v1` calibration: per-sensor intrinsics/
/// extrinsics plus the image-name -> (rig frame id, sensor index) table.
/// This is the metric ground truth the original mapper used (the T265
/// stereo baseline is a hard-constrained ~6.4 cm translation on sensor 1),
/// so re-deriving per-image poses from a shared rig-frame pose plus this
/// extrinsic is what lets a refinement BA actually recover/correct scale,
/// unlike treating every image as an independently-posed monocular camera.
struct RigCalibration {
    /// sensor_index -> (camera_id, sensor_from_rig).
    sensors: Vec<(u64, SE3)>,
    /// image name -> (frame_id, sensor_index).
    frame_of: HashMap<String, (u64, usize)>,
}

fn parse_rig_manifest(path: &Path) -> Result<RigCalibration, Box<dyn Error>> {
    let contents = fs::read_to_string(path)?;
    let mut sensors: Vec<(u64, SE3)> = Vec::new();
    let mut frame_of = HashMap::new();
    for line in contents.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let tokens: Vec<&str> = line.split_whitespace().collect();
        match tokens.first().copied() {
            Some("S") => {
                // S index camera_id width height fx fy cx cy qw qx qy qz tx ty tz
                if tokens.len() < 16 {
                    return Err(format!("malformed rig manifest S row: {line}").into());
                }
                let index: usize = tokens[1].parse()?;
                let camera_id: u64 = tokens[2].parse()?;
                let qw: f64 = tokens[9].parse()?;
                let qx: f64 = tokens[10].parse()?;
                let qy: f64 = tokens[11].parse()?;
                let qz: f64 = tokens[12].parse()?;
                let tx: f64 = tokens[13].parse()?;
                let ty: f64 = tokens[14].parse()?;
                let tz: f64 = tokens[15].parse()?;
                let sensor_from_rig = SE3::new(
                    UnitQuaternion::from_quaternion(Quaternion::new(qw, qx, qy, qz)),
                    Vector3::new(tx, ty, tz),
                );
                while sensors.len() <= index {
                    sensors.push((0, SE3::identity()));
                }
                sensors[index] = (camera_id, sensor_from_rig);
            }
            Some("F") => {
                // F frame_id image_name sensor_index
                if tokens.len() < 4 {
                    return Err(format!("malformed rig manifest F row: {line}").into());
                }
                let frame_id: u64 = tokens[1].parse()?;
                let name = tokens[2].to_owned();
                let sensor_index: usize = tokens[3].parse()?;
                frame_of.insert(name, (frame_id, sensor_index));
            }
            _ => {}
        }
    }
    Ok(RigCalibration { sensors, frame_of })
}

struct EdgeResult {
    a: usize,
    b: usize,
    cross_pairs: usize,
    correspondences: usize,
    inliers: usize,
    scale: f64,
    // Sim(3) that maps component `b`'s gauge into component `a`'s gauge.
    sim_b_to_a: Sim3,
    // (point3d_id in `a`, point3d_id in `b`) pairs that were RANSAC inliers:
    // the same physical landmark, to be fused into one output point.
    fused_landmark_pairs: Vec<(u64, u64)>,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = parse_args()?;
    let rig_calibration = match &args.rig_manifest {
        Some(path) => Some(parse_rig_manifest(path)?),
        None => None,
    };
    if args.refine && rig_calibration.is_none() {
        eprintln!("warning: --refine without --rig-manifest falls back to independent per-image BA (cannot recover metric scale)");
    }

    let mut component_dirs: Vec<PathBuf> = fs::read_dir(&args.components_dir)?
        .filter_map(|entry| entry.ok())
        .map(|entry| entry.path())
        .filter(|path| path.is_dir())
        .collect();
    component_dirs.sort();
    if component_dirs.is_empty() {
        return Err(format!(
            "no component directories under {}",
            args.components_dir.display()
        )
        .into());
    }
    eprintln!("loading {} components ...", component_dirs.len());
    let components: Vec<ComponentModel> = component_dirs
        .iter()
        .map(|dir| parse_component(dir))
        .collect::<Result<_, _>>()?;
    for c in &components {
        eprintln!(
            "  {} images={} points3d={}",
            c.name,
            c.images.len(),
            c.points3d.len()
        );
    }

    eprintln!("loading verified-pairs snapshot (mapper-compact) ...");
    let snapshot = verified_pair_snapshot::read_mapper_compact(&args.snapshot)?;
    eprintln!(
        "snapshot: {} images, {} pairs",
        snapshot.image_names.len(),
        snapshot.pairs.len()
    );

    // name -> (component index, local image id)
    let mut name_to_component: HashMap<&str, (usize, u64)> = HashMap::new();
    for (comp_idx, comp) in components.iter().enumerate() {
        for row in &comp.images {
            if name_to_component
                .insert(&row.name, (comp_idx, row.local_id))
                .is_some()
            {
                return Err(
                    format!("image name {} appears in more than one component", row.name).into(),
                );
            }
        }
    }

    // (local_image_id, keypoint_idx) -> point3d_id, and point3d_id -> position, per component.
    let mut keypoint_to_point3d: Vec<HashMap<(u64, u64), u64>> =
        Vec::with_capacity(components.len());
    let mut point3d_position: Vec<HashMap<u64, Point3<f64>>> = Vec::with_capacity(components.len());
    for comp in &components {
        let mut kp_map = HashMap::new();
        for row in &comp.images {
            for (keypoint_idx, &(_, _, point3d_id)) in row.points.iter().enumerate() {
                if point3d_id >= 0 {
                    kp_map.insert((row.local_id, keypoint_idx as u64), point3d_id as u64);
                }
            }
        }
        keypoint_to_point3d.push(kp_map);
        let pos_map = comp.points3d.iter().map(|p| (p.id, p.xyz)).collect();
        point3d_position.push(pos_map);
    }

    // Single pass over the snapshot: bucket cross-component correspondences.
    let mut cross_pair_count: HashMap<ComponentPairKey, usize> = HashMap::new();
    let mut correspondence_set: HashMap<ComponentPairKey, HashSet<(u64, u64)>> = HashMap::new();
    let mut correspondences: CorrespondenceMap = HashMap::new();
    // Parallel to `correspondences`: the underlying (point3d_id_in_a,
    // point3d_id_in_b) pair each correspondence came from, so accepted
    // RANSAC inliers can be turned into landmark fusions later.
    let mut correspondence_points: HashMap<ComponentPairKey, Vec<(u64, u64)>> = HashMap::new();
    for pair in &snapshot.pairs {
        let name_i = snapshot
            .image_names
            .get(pair.image_i as usize)
            .map(String::as_str);
        let name_j = snapshot
            .image_names
            .get(pair.image_j as usize)
            .map(String::as_str);
        let (Some(name_i), Some(name_j)) = (name_i, name_j) else {
            continue;
        };
        let Some(&(comp_i, local_i)) = name_to_component.get(name_i) else {
            continue;
        };
        let Some(&(comp_j, local_j)) = name_to_component.get(name_j) else {
            continue;
        };
        if comp_i == comp_j {
            continue;
        }
        let (comp_a, local_a, comp_b, local_b) = if comp_i < comp_j {
            (comp_i, local_i, comp_j, local_j)
        } else {
            (comp_j, local_j, comp_i, local_i)
        };
        let key = (comp_a, comp_b);
        *cross_pair_count.entry(key).or_insert(0) += 1;

        for &(feat_i, feat_j) in &pair.matches {
            let (feat_a, feat_b) = if comp_i < comp_j {
                (feat_i, feat_j)
            } else {
                (feat_j, feat_i)
            };
            let Some(&point_a) = keypoint_to_point3d[comp_a].get(&(local_a, feat_a)) else {
                continue;
            };
            let Some(&point_b) = keypoint_to_point3d[comp_b].get(&(local_b, feat_b)) else {
                continue;
            };
            let seen = correspondence_set.entry(key).or_default();
            if !seen.insert((point_a, point_b)) {
                continue;
            }
            let pos_a = point3d_position[comp_a][&point_a];
            let pos_b = point3d_position[comp_b][&point_b];
            // source = b's gauge, target = a's gauge (sim maps b -> a).
            correspondences.entry(key).or_default().push((pos_b, pos_a));
            correspondence_points
                .entry(key)
                .or_default()
                .push((point_a, point_b));
        }
    }

    eprintln!(
        "\ncomponent-pair edges (cross_pairs >= {}):",
        args.min_cross_pairs
    );
    let mut edges: Vec<EdgeResult> = Vec::new();
    let mut edge_keys: Vec<&(usize, usize)> = cross_pair_count.keys().collect();
    edge_keys.sort();
    for &&(a, b) in &edge_keys {
        let cp = cross_pair_count[&(a, b)];
        if cp < args.min_cross_pairs {
            continue;
        }
        let corr = correspondences.get(&(a, b)).cloned().unwrap_or_default();
        if corr.len() < args.min_correspondences {
            eprintln!(
                "  {} <-> {}: cross_pairs={cp} correspondences={} (< min {}) SKIP",
                components[a].name,
                components[b].name,
                corr.len(),
                args.min_correspondences
            );
            continue;
        }
        let Some((sim_b_to_a, inliers)) =
            ransac_sim3(&corr, args.ransac_iters, args.inlier_threshold_m, args.seed)
        else {
            eprintln!(
                "  {} <-> {}: cross_pairs={cp} correspondences={} RANSAC FAILED",
                components[a].name,
                components[b].name,
                corr.len()
            );
            continue;
        };
        let inlier_ratio = inliers.len() as f64 / corr.len() as f64;
        let scale_ok = (sim_b_to_a.scale - 1.0).abs() <= args.max_scale_deviation;
        let count_ok = inliers.len() >= args.min_inliers;
        let ratio_ok = inlier_ratio >= args.min_inlier_ratio;
        let accept = scale_ok && count_ok && ratio_ok;
        eprintln!(
            "  {} <-> {}: cross_pairs={cp} correspondences={} inliers={} ({:.0}%) scale={:.4} {}",
            components[a].name,
            components[b].name,
            corr.len(),
            inliers.len(),
            100.0 * inlier_ratio,
            sim_b_to_a.scale,
            if accept {
                "ACCEPT".to_owned()
            } else {
                let mut reasons = Vec::new();
                if !scale_ok {
                    reasons.push("scale out of range");
                }
                if !count_ok {
                    reasons.push("too few inliers");
                }
                if !ratio_ok {
                    reasons.push("inlier ratio too low");
                }
                format!("REJECT ({})", reasons.join(", "))
            },
        );
        if !accept {
            continue;
        }
        let point_ids = correspondence_points
            .get(&(a, b))
            .cloned()
            .unwrap_or_default();
        let fused_landmark_pairs = inliers.iter().map(|&i| point_ids[i]).collect();
        edges.push(EdgeResult {
            a,
            b,
            cross_pairs: cp,
            correspondences: corr.len(),
            inliers: inliers.len(),
            scale: sim_b_to_a.scale,
            sim_b_to_a,
            fused_landmark_pairs,
        });
    }

    // Union-find over accepted edges.
    let n = components.len();
    let mut parent: Vec<usize> = (0..n).collect();
    fn find(parent: &mut [usize], x: usize) -> usize {
        if parent[x] != x {
            parent[x] = find(parent, parent[x]);
        }
        parent[x]
    }
    for edge in &edges {
        let ra = find(&mut parent, edge.a);
        let rb = find(&mut parent, edge.b);
        if ra != rb {
            parent[rb] = ra;
        }
    }
    let mut clusters: HashMap<usize, Vec<usize>> = HashMap::new();
    for i in 0..n {
        let root = find(&mut parent, i);
        clusters.entry(root).or_default().push(i);
    }

    // Adjacency (both directions) for BFS composition within a cluster.
    let mut adjacency: HashMap<usize, Vec<(usize, &EdgeResult)>> = HashMap::new();
    for edge in &edges {
        adjacency.entry(edge.a).or_default().push((edge.b, edge));
        adjacency.entry(edge.b).or_default().push((edge.a, edge));
    }

    // Global landmark union-find: every RANSAC-inlier (point_a, point_b) pair
    // from every accepted edge names the same physical 3D point, so those
    // points should collapse into one output landmark (with a track that is
    // the union of both sides' observations). This is what actually creates
    // cross-component reprojection coupling for the optional joint BA pass;
    // without it the merged clusters would be Sim(3)-stitched but otherwise
    // disjoint landmark sets.
    let mut landmark_uf_index: HashMap<(usize, u64), usize> = HashMap::new();
    let mut landmark_uf_parent: Vec<usize> = Vec::new();
    fn uf_id_for(
        key: (usize, u64),
        index: &mut HashMap<(usize, u64), usize>,
        parent: &mut Vec<usize>,
    ) -> usize {
        *index.entry(key).or_insert_with(|| {
            let id = parent.len();
            parent.push(id);
            id
        })
    }
    for edge in &edges {
        for &(point_a, point_b) in &edge.fused_landmark_pairs {
            let ia = uf_id_for(
                (edge.a, point_a),
                &mut landmark_uf_index,
                &mut landmark_uf_parent,
            );
            let ib = uf_id_for(
                (edge.b, point_b),
                &mut landmark_uf_index,
                &mut landmark_uf_parent,
            );
            let ra = find(&mut landmark_uf_parent, ia);
            let rb = find(&mut landmark_uf_parent, ib);
            if ra != rb {
                landmark_uf_parent[rb] = ra;
            }
        }
    }
    let total_fused_landmarks = edges
        .iter()
        .map(|e| e.fused_landmark_pairs.len())
        .sum::<usize>();
    eprintln!(
        "landmark fusion: {total_fused_landmarks} inlier correspondences feed {} union-find nodes",
        landmark_uf_parent.len()
    );

    fs::create_dir_all(&args.output_dir)?;
    let mut cluster_indices: Vec<&usize> = clusters.keys().collect();
    cluster_indices.sort();
    let mut report_lines = Vec::new();
    let mut cluster_out_idx = 0usize;
    let mut total_images_out = 0usize;
    for &&_root in &cluster_indices {
        let members = &clusters[&_root];
        // Pick the member with the most registered images as the BFS root.
        let bfs_root = *members
            .iter()
            .max_by_key(|&&idx| components[idx].images.len())
            .unwrap();

        // BFS composing sim_member_to_root for every member.
        let mut sim_to_root: HashMap<usize, Sim3> = HashMap::new();
        sim_to_root.insert(bfs_root, Sim3::identity());
        let mut queue = std::collections::VecDeque::new();
        queue.push_back(bfs_root);
        while let Some(current) = queue.pop_front() {
            let current_to_root = sim_to_root[&current].clone();
            if let Some(neighbors) = adjacency.get(&current) {
                for &(neighbor, edge) in neighbors {
                    if sim_to_root.contains_key(&neighbor) {
                        continue;
                    }
                    // edge.sim_b_to_a maps b's gauge into a's gauge.
                    let neighbor_to_current = if edge.a == current && edge.b == neighbor {
                        edge.sim_b_to_a.clone()
                    } else if edge.b == current && edge.a == neighbor {
                        edge.sim_b_to_a.inverse()
                    } else {
                        continue;
                    };
                    let neighbor_to_root = current_to_root.compose(&neighbor_to_current);
                    sim_to_root.insert(neighbor, neighbor_to_root);
                    queue.push_back(neighbor);
                }
            }
        }

        // Merge: renumber images and points sequentially, fusing landmarks
        // that a cross-component RANSAC inlier identified as the same
        // physical point.
        let mut merged_cameras: Vec<Camera> = Vec::new();
        let mut merged_images: Vec<ImageRow> = Vec::new();
        let mut merged_points: Vec<Point3DRow> = Vec::new();
        let mut contributor_count: Vec<u32> = Vec::new();
        let mut next_image_id = 0u64;
        let mut next_point_id = 1u64;
        let mut ordered_members = members.clone();
        ordered_members.sort_by_key(|&idx| if idx == bfs_root { 0 } else { 1 });

        // Pass 1: transform each member and renumber its images.
        let mut member_transformed: Vec<ComponentModel> = Vec::new();
        let mut member_local_to_new: Vec<HashMap<u64, u64>> = Vec::new();
        for &member_idx in &ordered_members {
            let sim = sim_to_root
                .get(&member_idx)
                .cloned()
                .unwrap_or_else(Sim3::identity);
            let transformed =
                if sim.scale == 1.0 && sim.translation.norm() == 0.0 && sim.rotation.angle() == 0.0
                {
                    components[member_idx].clone()
                } else {
                    transform_component(&components[member_idx], &sim)
                };
            for camera in &transformed.cameras {
                if !merged_cameras
                    .iter()
                    .any(|existing| existing.id == camera.id)
                {
                    merged_cameras.push(camera.clone());
                }
            }
            let mut local_to_new: HashMap<u64, u64> = HashMap::new();
            for row in &transformed.images {
                let new_id = next_image_id;
                next_image_id += 1;
                local_to_new.insert(row.local_id, new_id);
                merged_images.push(ImageRow {
                    local_id: new_id,
                    ..row.clone()
                });
            }
            member_local_to_new.push(local_to_new);
            member_transformed.push(transformed);
        }

        // Pass 2: emit points, fusing across the landmark union-find.
        let mut fusion_root_to_index: HashMap<usize, usize> = HashMap::new();
        let mut fused_point_count = 0usize;
        for (member_pos, &member_idx) in ordered_members.iter().enumerate() {
            let transformed = &member_transformed[member_pos];
            let local_to_new = &member_local_to_new[member_pos];
            for point in &transformed.points3d {
                let remapped_track: Vec<(u64, u64)> = point
                    .track
                    .iter()
                    .filter_map(|&(image_id, keypoint)| {
                        local_to_new
                            .get(&image_id)
                            .map(|&new_image_id| (new_image_id, keypoint))
                    })
                    .collect();
                let fusion_root = landmark_uf_index
                    .get(&(member_idx, point.id))
                    .map(|&idx| find(&mut landmark_uf_parent, idx));
                if let Some(root) = fusion_root {
                    if let Some(&existing_index) = fusion_root_to_index.get(&root) {
                        // Fuse into the already-emitted point: extend its
                        // track and running-average its position.
                        let count = contributor_count[existing_index] as f64;
                        let existing = &mut merged_points[existing_index];
                        existing.xyz = Point3::from(
                            (existing.xyz.coords * count + point.xyz.coords) / (count + 1.0),
                        );
                        existing.track.extend(remapped_track);
                        contributor_count[existing_index] += 1;
                        fused_point_count += 1;
                        continue;
                    }
                    let new_id = next_point_id;
                    next_point_id += 1;
                    merged_points.push(Point3DRow {
                        id: new_id,
                        track: remapped_track,
                        ..point.clone()
                    });
                    contributor_count.push(1);
                    fusion_root_to_index.insert(root, merged_points.len() - 1);
                    continue;
                }
                let new_id = next_point_id;
                next_point_id += 1;
                merged_points.push(Point3DRow {
                    id: new_id,
                    track: remapped_track,
                    ..point.clone()
                });
                contributor_count.push(1);
            }
        }
        if fused_point_count > 0 {
            eprintln!("  fused {fused_point_count} landmark(s) across component boundaries");
        }
        // Rebuild each image's POINT3D_ID column from the renumbered tracks
        // (component-local ids in `points` are now stale after renumbering).
        let mut image_point_map: HashMap<(u64, u64), u64> = HashMap::new();
        for point in &merged_points {
            for &(image_id, keypoint) in &point.track {
                image_point_map.insert((image_id, keypoint), point.id);
            }
        }
        for row in &mut merged_images {
            for (keypoint_idx, entry) in row.points.iter_mut().enumerate() {
                entry.2 = image_point_map
                    .get(&(row.local_id, keypoint_idx as u64))
                    .map(|&id| id as i64)
                    .unwrap_or(-1);
            }
        }

        let cluster_name = format!("cluster-{cluster_out_idx:03}");
        let member_names: Vec<&str> = ordered_members
            .iter()
            .map(|&idx| components[idx].name.as_str())
            .collect();
        eprintln!(
            "{cluster_name}: {} component(s) -> {} images, {} points  members=[{}]",
            ordered_members.len(),
            merged_images.len(),
            merged_points.len(),
            member_names.join(", "),
        );
        report_lines.push(format!(
            "{{\"cluster\":\"{cluster_name}\",\"members\":[{}],\"images\":{},\"points3d\":{}}}",
            member_names
                .iter()
                .map(|n| format!("\"{n}\""))
                .collect::<Vec<_>>()
                .join(","),
            merged_images.len(),
            merged_points.len(),
        ));

        let mut merged = ComponentModel {
            name: cluster_name.clone(),
            cameras: merged_cameras,
            images: merged_images,
            points3d: merged_points,
        };

        if args.refine {
            eprintln!("  refining {cluster_name} with rig-aware joint BA ...");
            if let Some(rig) = &rig_calibration {
                refine_cluster_rig_coupled(&mut merged, rig, &args)?;
            } else {
                refine_cluster(&mut merged, &args)?;
            }
        }

        total_images_out += merged.images.len();
        write_component(&args.output_dir.join(&cluster_name), &merged)?;
        cluster_out_idx += 1;
    }

    eprintln!(
        "\n{} input components -> {} output clusters, {} total images (input had {} images across all components)",
        n,
        cluster_out_idx,
        total_images_out,
        components.iter().map(|c| c.images.len()).sum::<usize>(),
    );

    if let Some(report_path) = &args.report_json {
        let edge_lines: Vec<String> = edges
            .iter()
            .map(|edge| {
                format!(
                    "{{\"a\":\"{}\",\"b\":\"{}\",\"cross_pairs\":{},\"correspondences\":{},\"inliers\":{},\"scale\":{}}}",
                    components[edge.a].name,
                    components[edge.b].name,
                    edge.cross_pairs,
                    edge.correspondences,
                    edge.inliers,
                    edge.scale,
                )
            })
            .collect();
        let json = format!(
            "{{\"clusters\":[{}],\"accepted_edges\":[{}]}}\n",
            report_lines.join(","),
            edge_lines.join(","),
        );
        fs::write(report_path, json)?;
    }

    Ok(())
}

/// Rig-coupled joint BA: one pose per rig frame (not per image), with each
/// image's observation carrying the real `sensor_from_rig` extrinsic from
/// the calibration (cam2's ~6.4 cm stereo baseline off cam1). This is the
/// metric constraint the original incremental mapper had and the
/// independent-per-image `refine_cluster` above does not: a hard-known
/// baseline between two simultaneous views is what makes absolute scale
/// observable to a bundle adjustment at all. Applied to every cluster,
/// including untouched singleton components, so it can also correct
/// internal per-component drift, not just merge seams.
fn refine_cluster_rig_coupled(
    model: &mut ComponentModel,
    rig: &RigCalibration,
    args: &Args,
) -> Result<(), Box<dyn Error>> {
    let camera_by_id: HashMap<u64, Camera> =
        model.cameras.iter().map(|c| (c.id, c.clone())).collect();
    let image_by_local: HashMap<u64, &ImageRow> =
        model.images.iter().map(|r| (r.local_id, r)).collect();

    // Group this model's images by rig frame id, and remember each image's
    // sensor index for its observation's extrinsic.
    let mut frame_images: HashMap<u64, Vec<u64>> = HashMap::new();
    let mut image_sensor_index: HashMap<u64, usize> = HashMap::new();
    let mut unresolved = 0usize;
    for row in &model.images {
        match rig.frame_of.get(&row.name) {
            Some(&(frame_id, sensor_index)) => {
                frame_images.entry(frame_id).or_default().push(row.local_id);
                image_sensor_index.insert(row.local_id, sensor_index);
            }
            None => unresolved += 1,
        }
    }
    if unresolved > 0 {
        eprintln!(
            "    {unresolved} image(s) not found in the rig manifest, excluded from rig-coupled BA"
        );
    }
    if frame_images.is_empty() {
        eprintln!("    no images resolve against the rig manifest, skipping rig-coupled BA");
        return Ok(());
    }

    // Seed one Pose (T_rig<-world) per frame, from whichever sensor's image
    // is present, preferring the lowest sensor index (index 0 is the
    // reference sensor with an identity extrinsic in this calibration).
    let mut problem = BundleAdjustment::new(model.cameras[0].clone());
    let mut frame_observation_count: HashMap<u64, usize> = HashMap::new();
    for (&frame_id, members) in &frame_images {
        let Some(&seed_local_id) = members.iter().min_by_key(|&&local_id| {
            image_sensor_index
                .get(&local_id)
                .copied()
                .unwrap_or(usize::MAX)
        }) else {
            continue;
        };
        let Some(&sensor_index) = image_sensor_index.get(&seed_local_id) else {
            continue;
        };
        let Some((_, sensor_from_rig)) = rig.sensors.get(sensor_index) else {
            continue;
        };
        let Some(row) = image_by_local.get(&seed_local_id) else {
            continue;
        };
        let pose_cam = SE3::new(row.q, row.t);
        // pose_cam = sensor_from_rig ∘ pose_rig  =>  pose_rig = sensor_from_rig^-1 ∘ pose_cam
        let pose_rig = sensor_from_rig.inverse().compose(&pose_cam);
        problem.add_pose(
            frame_id,
            Pose {
                world_to_camera: pose_rig,
            },
        );
    }

    // Anchor the rig-frame with the most resolved images/observations.
    let anchor = frame_images
        .iter()
        .max_by_key(|(_, members)| members.len())
        .map(|(&frame_id, _)| frame_id);
    if let Some(anchor) = anchor {
        problem.fix_pose(anchor);
    }

    let mut used_points = 0usize;
    let mut used_observations = 0usize;
    for point in &model.points3d {
        if point.track.len() < 2 {
            continue;
        }
        let mut observations = Vec::new();
        for &(image_id, keypoint) in &point.track {
            let Some(row) = image_by_local.get(&image_id) else {
                continue;
            };
            let Some(&(frame_id, sensor_index)) = rig.frame_of.get(&row.name) else {
                continue;
            };
            if !problem.poses.contains_key(&frame_id) {
                continue;
            }
            let Some(&(x, y, _)) = row.points.get(keypoint as usize) else {
                continue;
            };
            let Some(camera) = camera_by_id.get(&row.camera_id) else {
                continue;
            };
            let Some((_, sensor_from_rig)) = rig.sensors.get(sensor_index) else {
                continue;
            };
            observations.push(BaRigObservation {
                keyframe_id: frame_id,
                landmark_id: point.id,
                xy: nalgebra::Point2::new(x, y),
                camera: camera.clone(),
                sensor_from_rig: sensor_from_rig.clone(),
            });
            *frame_observation_count.entry(frame_id).or_insert(0) += 1;
        }
        if observations.len() < 2 {
            continue;
        }
        problem.add_landmark(point.id, point.xyz);
        used_points += 1;
        used_observations += observations.len();
        for observation in observations {
            problem.add_rig_observation(observation);
        }
    }
    eprintln!(
        "    rig-coupled BA problem: {} rig-frame poses, {} landmarks, {} observations",
        problem.poses.len(),
        used_points,
        used_observations,
    );
    if used_points == 0 {
        eprintln!("    no usable landmarks with >=2 observations, skipping BA");
        return Ok(());
    }

    let config = BaConfig {
        max_iterations: args.ba_max_iterations,
        linear_solver: LinearSolver::Sparse,
        robust_kernel: RobustKernel::Huber {
            delta: args.ba_max_reprojection_error_px,
        },
        ..BaConfig::default()
    };
    let result = problem.optimize(&config)?;
    eprintln!(
        "    BA done: cost {:.6} -> {:.6} in {} iterations",
        result.initial_cost,
        result.final_cost,
        result.iterations.len(),
    );

    // Re-derive each image's own absolute pose from its (optimized) rig
    // frame pose composed with its sensor's fixed extrinsic.
    for row in &mut model.images {
        let Some(&(frame_id, sensor_index)) = rig.frame_of.get(&row.name) else {
            continue;
        };
        let Some(pose_rig) = problem.poses.get(&frame_id) else {
            continue;
        };
        let Some((_, sensor_from_rig)) = rig.sensors.get(sensor_index) else {
            continue;
        };
        let pose_cam = sensor_from_rig.compose(&pose_rig.world_to_camera);
        row.q = pose_cam.rotation;
        row.t = pose_cam.translation;
    }
    for point in &mut model.points3d {
        if let Some(&xyz) = problem.landmarks.get(&point.id) {
            point.xyz = xyz;
        }
    }
    Ok(())
}

fn refine_cluster(model: &mut ComponentModel, args: &Args) -> Result<(), Box<dyn Error>> {
    let camera_by_id: HashMap<u64, Camera> =
        model.cameras.iter().map(|c| (c.id, c.clone())).collect();
    let mut problem = BundleAdjustment::new(model.cameras[0].clone());
    for row in &model.images {
        let pose = Pose::from_world_to_camera(row.q, row.t);
        problem.add_pose(row.local_id, pose);
    }
    // Anchor the pose with the most observations to fix the residual gauge
    // freedom (Sim(3) merge already fixed inter-component scale/rotation;
    // BA only needs one fixed pose to pin translation/rotation/gauge null
    // space left over from the joint optimization).
    let anchor = model
        .images
        .iter()
        .max_by_key(|row| row.points.iter().filter(|p| p.2 >= 0).count())
        .map(|row| row.local_id);
    if let Some(anchor) = anchor {
        problem.fix_pose(anchor);
    }

    let image_by_local: HashMap<u64, &ImageRow> =
        model.images.iter().map(|r| (r.local_id, r)).collect();
    let mut used_points = 0usize;
    let mut used_observations = 0usize;
    for point in &model.points3d {
        if point.track.len() < 2 {
            continue;
        }
        let mut observations = Vec::new();
        for &(image_id, keypoint) in &point.track {
            let Some(row) = image_by_local.get(&image_id) else {
                continue;
            };
            let Some(&(x, y, _)) = row.points.get(keypoint as usize) else {
                continue;
            };
            let Some(camera) = camera_by_id.get(&row.camera_id) else {
                continue;
            };
            observations.push(BaRigObservation {
                keyframe_id: image_id,
                landmark_id: point.id,
                xy: nalgebra::Point2::new(x, y),
                camera: camera.clone(),
                sensor_from_rig: SE3::identity(),
            });
        }
        if observations.len() < 2 {
            continue;
        }
        problem.add_landmark(point.id, point.xyz);
        used_points += 1;
        used_observations += observations.len();
        for observation in observations {
            problem.add_rig_observation(observation);
        }
    }
    eprintln!(
        "    BA problem: {} poses, {} landmarks, {} observations",
        problem.poses.len(),
        used_points,
        used_observations,
    );
    if used_points == 0 {
        eprintln!("    no usable landmarks with >=2 observations, skipping BA");
        return Ok(());
    }

    let config = BaConfig {
        max_iterations: args.ba_max_iterations,
        linear_solver: LinearSolver::Sparse,
        robust_kernel: RobustKernel::Huber {
            delta: args.ba_max_reprojection_error_px,
        },
        ..BaConfig::default()
    };
    let result = problem.optimize(&config)?;
    eprintln!(
        "    BA done: cost {:.6} -> {:.6} in {} iterations",
        result.initial_cost,
        result.final_cost,
        result.iterations.len(),
    );

    for row in &mut model.images {
        if let Some(pose) = problem.poses.get(&row.local_id) {
            row.q = pose.world_to_camera.rotation;
            row.t = pose.world_to_camera.translation;
        }
    }
    for point in &mut model.points3d {
        if let Some(&xyz) = problem.landmarks.get(&point.id) {
            point.xyz = xyz;
        }
    }
    Ok(())
}
