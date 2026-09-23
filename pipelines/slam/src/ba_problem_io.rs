//! Plain-text dump/load of a monocular pinhole [`BundleAdjustment`] problem.
//!
//! Used to replay the exact problems the incremental SfM solves (set
//! `VISLOC_SFM_BA_DUMP=<path>`; every global BA overwrites the file, so it
//! ends up holding the last, largest one) in solver benchmarks. Floats are
//! written with Rust's shortest round-trip formatting, so a reload is exact.

use std::fmt::Write as _;
use std::io;
use std::path::Path;

use nalgebra::{Point2, Point3, Quaternion, UnitQuaternion, Vector3};
use visloc_core::geometry::{Pose, SE3};
use visloc_core::types::Camera;

use crate::bundle::{BaObservation, BundleAdjustment};

/// Write the pinhole monocular part of `ba` (camera, poses with fixed flags,
/// landmarks, observations and optional per-observation weights).
pub fn write_ba_problem(
    ba: &BundleAdjustment,
    weights: Option<&[f64]>,
    path: &Path,
) -> io::Result<()> {
    let c = &ba.camera;
    let mut s = String::new();
    let _ = writeln!(
        s,
        "camera {} {} {} {} {} {}",
        c.width, c.height, c.params[0], c.params[1], c.params[2], c.params[3]
    );
    let _ = writeln!(s, "poses {}", ba.poses.len());
    for (id, pose) in &ba.poses {
        let q = pose.world_to_camera.rotation.quaternion();
        let t = &pose.world_to_camera.translation;
        let _ = writeln!(
            s,
            "{id} {} {} {} {} {} {} {} {}",
            q.w,
            q.i,
            q.j,
            q.k,
            t.x,
            t.y,
            t.z,
            u8::from(ba.fixed_poses.contains(id))
        );
    }
    let _ = writeln!(s, "landmarks {}", ba.landmarks.len());
    for (id, p) in &ba.landmarks {
        let _ = writeln!(s, "{id} {} {} {}", p.x, p.y, p.z);
    }
    let _ = writeln!(
        s,
        "observations {} {}",
        ba.observations.len(),
        u8::from(weights.is_some())
    );
    for (i, o) in ba.observations.iter().enumerate() {
        let _ = write!(
            s,
            "{} {} {} {}",
            o.keyframe_id, o.landmark_id, o.xy.x, o.xy.y
        );
        if let Some(w) = weights {
            let _ = write!(s, " {}", w[i]);
        }
        s.push('\n');
    }
    std::fs::write(path, s)
}

fn bad(msg: &str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, msg.to_string())
}

/// Load a problem written by [`write_ba_problem`].
pub fn read_ba_problem(path: &Path) -> io::Result<(BundleAdjustment, Option<Vec<f64>>)> {
    let text = std::fs::read_to_string(path)?;
    let mut lines = text.lines();
    let mut next = |tag: &str| -> io::Result<Vec<String>> {
        let line = lines.next().ok_or_else(|| bad(tag))?;
        Ok(line.split_whitespace().map(str::to_string).collect())
    };
    let f = |s: &str| s.parse::<f64>().map_err(|_| bad("float"));
    let u = |s: &str| s.parse::<u64>().map_err(|_| bad("int"));

    let cam = next("camera")?;
    if cam.len() != 7 || cam[0] != "camera" {
        return Err(bad("camera line"));
    }
    let camera = Camera::pinhole(
        1,
        u(&cam[1])? as u32,
        u(&cam[2])? as u32,
        f(&cam[3])?,
        f(&cam[4])?,
        f(&cam[5])?,
        f(&cam[6])?,
    );
    let mut ba = BundleAdjustment::new(camera);
    let head = next("poses")?;
    let np = u(&head[1])? as usize;
    for _ in 0..np {
        let v = next("pose")?;
        let id = u(&v[0])?;
        let q = UnitQuaternion::from_quaternion(Quaternion::new(
            f(&v[1])?,
            f(&v[2])?,
            f(&v[3])?,
            f(&v[4])?,
        ));
        let t = Vector3::new(f(&v[5])?, f(&v[6])?, f(&v[7])?);
        ba.add_pose(
            id,
            Pose {
                world_to_camera: SE3 {
                    rotation: q,
                    translation: t,
                },
            },
        );
        if v[8] == "1" {
            ba.fix_pose(id);
        }
    }
    let head = next("landmarks")?;
    let nl = u(&head[1])? as usize;
    for _ in 0..nl {
        let v = next("landmark")?;
        ba.add_landmark(u(&v[0])?, Point3::new(f(&v[1])?, f(&v[2])?, f(&v[3])?));
    }
    let head = next("observations")?;
    let no = u(&head[1])? as usize;
    let weighted = head.get(2).is_some_and(|w| w == "1");
    let mut weights = weighted.then(|| Vec::with_capacity(no));
    for _ in 0..no {
        let v = next("observation")?;
        ba.add_observation(BaObservation {
            keyframe_id: u(&v[0])?,
            landmark_id: u(&v[1])?,
            xy: Point2::new(f(&v[2])?, f(&v[3])?),
        });
        if let Some(w) = weights.as_mut() {
            w.push(f(&v[4])?);
        }
    }
    Ok((ba, weights))
}
