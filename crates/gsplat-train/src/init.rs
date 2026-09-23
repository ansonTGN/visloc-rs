//! Initial gaussians from SfM points.
//!
//! One gaussian per point, as in the Inria recipe: colour from the point's RGB
//! (as the SH DC term), isotropic scale from the mean distance to its three
//! nearest neighbours, identity rotation, opacity 0.1, higher SH zero.

use std::collections::HashMap;
use std::path::Path;

use nalgebra::{Quaternion, Vector3};
use visloc_gsplat_core::gaussian::{sh_rest_coeffs_per_channel, Gaussian, Scene};
use visloc_gsplat_core::sh::SH_C0;

/// A coloured SfM point.
#[derive(Debug, Clone, Copy)]
pub struct ColoredPoint {
    pub position: Vector3<f32>,
    pub rgb: [u8; 3],
}

/// Errors reading `points3D.txt`.
#[derive(Debug, thiserror::Error)]
pub enum PointsError {
    #[error("reading {path}: {source}")]
    Io {
        path: std::path::PathBuf,
        source: std::io::Error,
    },
    #[error("{path}:{line}: malformed point line")]
    Malformed {
        path: std::path::PathBuf,
        line: usize,
    },
}

/// Read a COLMAP text `points3D.txt` (`ID X Y Z R G B ERROR TRACK[]`).
pub fn read_points3d_txt(path: impl AsRef<Path>) -> Result<Vec<ColoredPoint>, PointsError> {
    let path = path.as_ref();
    let text = std::fs::read_to_string(path).map_err(|source| PointsError::Io {
        path: path.to_path_buf(),
        source,
    })?;
    let mut out = Vec::new();
    for (i, line) in text.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let f: Vec<&str> = line.split_whitespace().take(7).collect();
        let bad = || PointsError::Malformed {
            path: path.to_path_buf(),
            line: i + 1,
        };
        if f.len() < 7 {
            return Err(bad());
        }
        let num = |k: usize| f[k].parse::<f32>().map_err(|_| bad());
        let byte = |k: usize| f[k].parse::<u8>().map_err(|_| bad());
        out.push(ColoredPoint {
            position: Vector3::new(num(1)?, num(2)?, num(3)?),
            rgb: [byte(4)?, byte(5)?, byte(6)?],
        });
    }
    Ok(out)
}

/// Mean distance from each point to its `k` nearest other points, via a
/// uniform hash grid (cell ~ the average spacing). Points with fewer than `k`
/// neighbours in the searched rings get the mean over what was found; an
/// isolated point gets the global average.
pub fn knn_mean_distance(points: &[Vector3<f32>], k: usize) -> Vec<f32> {
    let n = points.len();
    if n < 2 {
        return vec![0.01; n];
    }
    let (mut lo, mut hi) = (points[0], points[0]);
    for p in points {
        lo = lo.inf(p);
        hi = hi.sup(p);
    }
    let extent = (hi - lo).max().max(1e-6);
    let cell = (extent / (n as f32).cbrt()).max(1e-6);
    let key = |p: &Vector3<f32>| {
        (
            ((p.x - lo.x) / cell).floor() as i64,
            ((p.y - lo.y) / cell).floor() as i64,
            ((p.z - lo.z) / cell).floor() as i64,
        )
    };
    let mut grid: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::new();
    for (i, p) in points.iter().enumerate() {
        grid.entry(key(p)).or_default().push(i as u32);
    }
    let mut out = vec![f32::NAN; n];
    for (i, p) in points.iter().enumerate() {
        let (cx, cy, cz) = key(p);
        let mut best: Vec<f32> = Vec::with_capacity(k + 1);
        // Grow the searched cube ring by ring until k neighbours are found
        // and the next ring cannot hold anything closer.
        for r in 1..=4i64 {
            best.clear();
            for dx in -r..=r {
                for dy in -r..=r {
                    for dz in -r..=r {
                        if let Some(ids) = grid.get(&(cx + dx, cy + dy, cz + dz)) {
                            for &j in ids {
                                if j as usize != i {
                                    best.push((points[j as usize] - p).norm());
                                }
                            }
                        }
                    }
                }
            }
            best.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            if best.len() >= k && best[k - 1] <= r as f32 * cell {
                break;
            }
        }
        if !best.is_empty() {
            let m = best.len().min(k);
            out[i] = best[..m].iter().sum::<f32>() / m as f32;
        }
    }
    let (sum, cnt) = out
        .iter()
        .filter(|d| d.is_finite() && **d > 0.0)
        .fold((0.0f64, 0usize), |(s, c), d| (s + *d as f64, c + 1));
    let fallback = if cnt > 0 {
        (sum / cnt as f64) as f32
    } else {
        cell
    };
    for d in &mut out {
        if !d.is_finite() || *d <= 0.0 {
            *d = fallback;
        }
    }
    out
}

/// Seed a scene of SH degree `sh_degree` from coloured points.
pub fn seed_scene(points: &[ColoredPoint], sh_degree: u32) -> Scene {
    let positions: Vec<Vector3<f32>> = points.iter().map(|p| p.position).collect();
    let dist = knn_mean_distance(&positions, 3);
    let rest = 3 * sh_rest_coeffs_per_channel(sh_degree);
    // logit(0.1)
    let opacity_logit = (0.1f32 / 0.9).ln();
    let gaussians = points
        .iter()
        .zip(dist)
        .map(|(p, d)| {
            let ls = d.max(1e-7).ln();
            Gaussian {
                mean: p.position,
                scale_log: Vector3::new(ls, ls, ls),
                rotation: Quaternion::new(1.0, 0.0, 0.0, 0.0),
                opacity_logit,
                sh_dc: p.rgb.map(|c| (c as f32 / 255.0 - 0.5) / SH_C0),
                sh_rest: vec![0.0; rest],
                sh_degree,
            }
        })
        .collect();
    Scene::new(gaussians, sh_degree)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn knn_on_a_lattice() {
        // Unit lattice: the 3 nearest neighbours of an interior point are at 1.
        let mut pts = Vec::new();
        for x in 0..6 {
            for y in 0..6 {
                for z in 0..6 {
                    pts.push(Vector3::new(x as f32, y as f32, z as f32));
                }
            }
        }
        let d = knn_mean_distance(&pts, 3);
        let interior = pts
            .iter()
            .position(|p| *p == Vector3::new(2.0, 3.0, 2.0))
            .unwrap();
        assert!((d[interior] - 1.0).abs() < 1e-5, "got {}", d[interior]);
        assert!(d.iter().all(|x| x.is_finite() && *x > 0.0));
    }

    #[test]
    fn seed_colours_round_trip_through_dc() {
        let scene = seed_scene(
            &[
                ColoredPoint {
                    position: Vector3::new(0.0, 0.0, 0.0),
                    rgb: [255, 128, 0],
                },
                ColoredPoint {
                    position: Vector3::new(1.0, 0.0, 0.0),
                    rgb: [0, 0, 0],
                },
            ],
            3,
        );
        let g = &scene.gaussians[0];
        let c = g.sh_dc.map(|d| d * SH_C0 + 0.5);
        assert!((c[0] - 1.0).abs() < 1e-5 && (c[2] - 0.0).abs() < 1e-5);
        assert_eq!(g.sh_rest.len(), 45);
        assert!(g.sh_is_consistent());
    }
}
