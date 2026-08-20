//! Frame-to-frame KLT with Basalt Pattern51 + LSSD.

use super::config::BasaltOpticalFlowConfig;
use super::lssd::LssdPatch;
use super::pyramid::{build_pyramid, GrayImage};

/// Basalt Pattern52 raw offsets; Pattern51 = 0.5 × Pattern52.
const PATTERN52: [[f32; 2]; 52] = [
    [-3.0, 7.0],
    [-1.0, 7.0],
    [1.0, 7.0],
    [3.0, 7.0],
    [-5.0, 5.0],
    [-3.0, 5.0],
    [-1.0, 5.0],
    [1.0, 5.0],
    [3.0, 5.0],
    [5.0, 5.0],
    [-7.0, 3.0],
    [-5.0, 3.0],
    [-3.0, 3.0],
    [-1.0, 3.0],
    [1.0, 3.0],
    [3.0, 3.0],
    [5.0, 3.0],
    [7.0, 3.0],
    [-7.0, 1.0],
    [-5.0, 1.0],
    [-3.0, 1.0],
    [-1.0, 1.0],
    [1.0, 1.0],
    [3.0, 1.0],
    [5.0, 1.0],
    [7.0, 1.0],
    [-7.0, -1.0],
    [-5.0, -1.0],
    [-3.0, -1.0],
    [-1.0, -1.0],
    [1.0, -1.0],
    [3.0, -1.0],
    [5.0, -1.0],
    [7.0, -1.0],
    [-7.0, -3.0],
    [-5.0, -3.0],
    [-3.0, -3.0],
    [-1.0, -3.0],
    [1.0, -3.0],
    [3.0, -3.0],
    [5.0, -3.0],
    [7.0, -3.0],
    [-5.0, -5.0],
    [-3.0, -5.0],
    [-1.0, -5.0],
    [1.0, -5.0],
    [3.0, -5.0],
    [5.0, -5.0],
    [-3.0, -7.0],
    [-1.0, -7.0],
    [1.0, -7.0],
    [3.0, -7.0],
];

/// Pattern51 sample count (Basalt `PATTERN_SIZE`).
pub const PATTERN51_SIZE: usize = 52;

pub(crate) fn pattern51_offsets() -> [(f32, f32); PATTERN51_SIZE] {
    let mut out = [(0.0f32, 0.0f32); PATTERN51_SIZE];
    for (i, p) in PATTERN52.iter().enumerate() {
        out[i] = (0.5 * p[0], 0.5 * p[1]);
    }
    out
}

#[cfg(test)]
pub(crate) fn pattern51_offsets_for_test() -> [(f32, f32); PATTERN51_SIZE] {
    pattern51_offsets()
}

/// Track a list of points from `from` into `to` (one-shot, no track ids).
pub fn track_points_between(
    from: &GrayImage,
    to: &GrayImage,
    points: &[(f32, f32)],
    config: &BasaltOpticalFlowConfig,
) -> Vec<Option<(f32, f32)>> {
    let levels = config.optical_flow_levels.max(1) as usize;
    let max_iters = config.optical_flow_max_iterations.max(1) as usize;
    let max_fb2 = config.optical_flow_max_recovered_dist2;
    let pattern = pattern51_offsets();
    let from_pyr = build_pyramid(from, levels);
    let to_pyr = build_pyramid(to, levels);
    points
        .iter()
        .map(|&(x, y)| {
            let (xf, yf) = track_point(&from_pyr, &to_pyr, x, y, max_iters, &pattern)?;
            let (xb, yb) = track_point(&to_pyr, &from_pyr, xf, yf, max_iters, &pattern)?;
            let dx = xb - x;
            let dy = yb - y;
            if dx * dx + dy * dy > max_fb2 {
                return None;
            }
            Some((xf, yf))
        })
        .collect()
}

pub type KeypointId = u64;

#[derive(Debug, Clone, Copy)]
pub struct TrackedKeypoint {
    pub id: KeypointId,
    pub x: f32,
    pub y: f32,
}

#[derive(Debug, Clone)]
pub struct OpticalFlowObservation {
    pub id: KeypointId,
    pub x: f32,
    pub y: f32,
}

pub struct OpticalFlowTracker {
    config: BasaltOpticalFlowConfig,
    next_id: KeypointId,
    prev_pyramid: Option<Vec<GrayImage>>,
    tracks: Vec<TrackedKeypoint>,
    pattern: [(f32, f32); PATTERN51_SIZE],
}

impl OpticalFlowTracker {
    pub fn new(config: BasaltOpticalFlowConfig) -> Self {
        Self {
            config,
            next_id: 1,
            prev_pyramid: None,
            tracks: Vec::new(),
            pattern: pattern51_offsets(),
        }
    }

    pub fn with_euroc_defaults() -> Self {
        Self::new(BasaltOpticalFlowConfig::default())
    }

    pub fn track_count(&self) -> usize {
        self.tracks.len()
    }

    pub fn process(&mut self, image: &GrayImage) -> Vec<OpticalFlowObservation> {
        let levels = self.config.optical_flow_levels.max(1) as usize;
        let pyramid = build_pyramid(image, levels);

        if let Some(prev) = self.prev_pyramid.take() {
            self.tracks = self.track_all(&prev, &pyramid);
        } else {
            self.tracks.clear();
        }

        self.detect_new(&pyramid[0]);
        self.prev_pyramid = Some(pyramid);

        self.tracks
            .iter()
            .map(|t| OpticalFlowObservation {
                id: t.id,
                x: t.x,
                y: t.y,
            })
            .collect()
    }

    fn track_all(
        &self,
        prev_pyr: &[GrayImage],
        curr_pyr: &[GrayImage],
    ) -> Vec<TrackedKeypoint> {
        let max_iters = self.config.optical_flow_max_iterations.max(1) as usize;
        let max_fb2 = self.config.optical_flow_max_recovered_dist2;
        let mut kept = Vec::with_capacity(self.tracks.len());

        for track in &self.tracks {
            let Some((xf, yf)) =
                track_point(prev_pyr, curr_pyr, track.x, track.y, max_iters, &self.pattern)
            else {
                continue;
            };
            let Some((xb, yb)) =
                track_point(curr_pyr, prev_pyr, xf, yf, max_iters, &self.pattern)
            else {
                continue;
            };
            let dx = xb - track.x;
            let dy = yb - track.y;
            if dx * dx + dy * dy > max_fb2 {
                continue;
            }
            kept.push(TrackedKeypoint {
                id: track.id,
                x: xf,
                y: yf,
            });
        }
        kept
    }

    fn detect_new(&mut self, image: &GrayImage) {
        let cell = self.config.optical_flow_detection_grid_size.max(8) as usize;
        let cols = image.width.div_ceil(cell);
        let rows = image.height.div_ceil(cell);
        let mut occupied = vec![false; cols * rows];
        for t in &self.tracks {
            let cx = (t.x as usize / cell).min(cols.saturating_sub(1));
            let cy = (t.y as usize / cell).min(rows.saturating_sub(1));
            occupied[cy * cols + cx] = true;
        }

        for cy in 0..rows {
            for cx in 0..cols {
                if occupied[cy * cols + cx] {
                    continue;
                }
                let x0 = cx * cell;
                let y0 = cy * cell;
                let x1 = (x0 + cell).min(image.width);
                let y1 = (y0 + cell).min(image.height);
                if let Some((bx, by)) = strongest_corner(image, x0, y0, x1, y1) {
                    let id = self.next_id;
                    self.next_id += 1;
                    self.tracks.push(TrackedKeypoint {
                        id,
                        x: bx,
                        y: by,
                    });
                }
            }
        }
    }
}

fn strongest_corner(
    image: &GrayImage,
    x0: usize,
    y0: usize,
    x1: usize,
    y1: usize,
) -> Option<(f32, f32)> {
    let mut best_score = 0.0f32;
    let mut best: Option<(f32, f32)> = None;
    let margin = 4usize;
    let xs = (x0 + margin)..x1.saturating_sub(margin);
    let ys = (y0 + margin)..y1.saturating_sub(margin);
    for y in ys {
        for x in xs.clone() {
            let c = image.data[y * image.width + x] as f32;
            let mut sum = 0.0f32;
            let mut sum2 = 0.0f32;
            let mut n = 0.0f32;
            for dy in -1i32..=1 {
                for dx in -1i32..=1 {
                    let v = image.data[(y as i32 + dy) as usize * image.width
                        + (x as i32 + dx) as usize] as f32;
                    sum += v;
                    sum2 += v * v;
                    n += 1.0;
                }
            }
            let mean = sum / n;
            let var = (sum2 / n) - mean * mean;
            let score = var + 0.01 * (c - 128.0).abs();
            if score > best_score {
                best_score = score;
                best = Some((x as f32, y as f32));
            }
        }
    }
    if best_score < 20.0 {
        return None;
    }
    best
}

/// Coarse-to-fine LSSD translation tracking (Basalt `trackPoint` layout).
///
/// Pyramid layers are `0..=optical_flow_levels`. Any invalid patch / residual
/// fails the whole track (Basalt `patch_valid &= ...`).
fn track_point(
    from_pyr: &[GrayImage],
    to_pyr: &[GrayImage],
    x0: f32,
    y0: f32,
    max_iters: usize,
    pattern: &[(f32, f32); PATTERN51_SIZE],
) -> Option<(f32, f32)> {
    let num_layers = from_pyr.len().min(to_pyr.len());
    if num_layers == 0 {
        return None;
    }
    let max_level = num_layers - 1;
    let mut x = x0;
    let mut y = y0;
    for level in (0..=max_level).rev() {
        let scale = (1 << level) as f32;
        x /= scale;
        y /= scale;

        let patch = LssdPatch::from_image(&from_pyr[level], (x0 / scale, y0 / scale), pattern);
        if !patch.valid {
            return None;
        }
        for _ in 0..max_iters {
            let Some((dx, dy)) = patch.track_step(&to_pyr[level], (x, y), pattern) else {
                return None;
            };
            // Basalt: transform *= SE2::exp(inc); translation-only → +=.
            x += dx;
            y += dy;
            if !to_pyr[level].in_bounds_margin(x, y, 2.0) {
                return None;
            }
            if dx * dx + dy * dy < 1e-4 {
                break;
            }
        }
        x *= scale;
        y *= scale;
    }
    Some((x, y))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::optical_flow::BasaltOpticalFlowConfig;

    fn shift_image(src: &GrayImage, dx: i32, dy: i32) -> GrayImage {
        let mut data = vec![0u8; src.data.len()];
        for y in 0..src.height {
            for x in 0..src.width {
                let sx = x as i32 - dx;
                let sy = y as i32 - dy;
                if sx >= 0 && sy >= 0 && (sx as usize) < src.width && (sy as usize) < src.height {
                    data[y * src.width + x] = src.data[sy as usize * src.width + sx as usize];
                }
            }
        }
        GrayImage {
            width: src.width,
            height: src.height,
            data,
        }
    }

    #[test]
    fn tracks_integer_shift_with_lssd() {
        let mut data = vec![40u8; 128 * 96];
        let (cx, cy) = (58.0f32, 48.0f32);
        for y in 0..96 {
            for x in 0..128 {
                let dx = x as f32 - cx;
                let dy = y as f32 - cy;
                let g = (-0.08 * (dx * dx + dy * dy)).exp();
                data[y * 128 + x] = (40.0 + 180.0 * g) as u8;
            }
        }
        let img0 = GrayImage::from_luma8(128, 96, data).unwrap();
        let img1 = shift_image(&img0, 3, -2);

        let mut cfg = BasaltOpticalFlowConfig::default();
        cfg.optical_flow_levels = 3;
        cfg.optical_flow_max_iterations = 5;
        cfg.optical_flow_max_recovered_dist2 = 0.04;
        let mut tracker = OpticalFlowTracker::new(cfg.clone());
        tracker.tracks.push(TrackedKeypoint {
            id: 1,
            x: cx,
            y: cy,
        });
        tracker.next_id = 2;
        tracker.prev_pyramid = Some(build_pyramid(&img0, cfg.optical_flow_levels as usize));

        let obs1 = tracker.process(&img1);
        let hit = obs1.iter().find(|o| o.id == 1).expect("track 1 survived FB=0.04");
        assert!(
            (hit.x - (cx + 3.0)).abs() < 0.5 && (hit.y - (cy - 2.0)).abs() < 0.5,
            "expected ~(+3,-2) flow, got ({}, {})",
            hit.x,
            hit.y
        );
    }

    #[test]
    fn tracks_under_global_gain() {
        let mut data = vec![40u8; 128 * 96];
        let (cx, cy) = (58.0f32, 48.0f32);
        for y in 0..96 {
            for x in 0..128 {
                let dx = x as f32 - cx;
                let dy = y as f32 - cy;
                let g = (-0.08 * (dx * dx + dy * dy)).exp();
                data[y * 128 + x] = (40.0 + 120.0 * g) as u8;
            }
        }
        let img0 = GrayImage::from_luma8(128, 96, data).unwrap();
        let mut shifted = shift_image(&img0, 2, 1);
        for v in &mut shifted.data {
            *v = ((*v as f32) * 1.4).min(255.0) as u8;
        }

        let mut cfg = BasaltOpticalFlowConfig::default();
        cfg.optical_flow_levels = 2;
        cfg.optical_flow_max_iterations = 5;
        cfg.optical_flow_max_recovered_dist2 = 0.25;
        let mut tracker = OpticalFlowTracker::new(cfg.clone());
        tracker.tracks.push(TrackedKeypoint {
            id: 1,
            x: cx,
            y: cy,
        });
        tracker.next_id = 2;
        tracker.prev_pyramid = Some(build_pyramid(&img0, cfg.optical_flow_levels as usize));
        let obs1 = tracker.process(&shifted);
        let hit = obs1.iter().find(|o| o.id == 1).expect("gain-robust track");
        assert!((hit.x - (cx + 2.0)).abs() < 1.0 && (hit.y - (cy + 1.0)).abs() < 1.0);
    }
}
