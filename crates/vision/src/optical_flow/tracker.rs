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
    let max_fb2 = config.stereo_max_recovered_dist2();
    let pattern = pattern51_offsets();
    let from_pyr = build_pyramid(from, levels);
    let to_pyr = build_pyramid(to, levels);
    points
        .iter()
        .map(|&(x, y)| {
            let (xf, yf) = track_point(&from_pyr, &to_pyr, x, y, x, y, max_iters, &pattern)?;
            let (xb, yb) = track_point(&to_pyr, &from_pyr, xf, yf, x, y, max_iters, &pattern)?;
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
    /// Last accepted displacement (curr − prev). Seeds the next search.
    pub vx: f32,
    pub vy: f32,
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

    fn track_all(&self, prev_pyr: &[GrayImage], curr_pyr: &[GrayImage]) -> Vec<TrackedKeypoint> {
        let max_iters = self.config.optical_flow_max_iterations.max(1) as usize;
        let max_fb2 = self.config.temporal_max_recovered_dist2();
        let mut kept = Vec::with_capacity(self.tracks.len());

        for track in &self.tracks {
            // Constant-velocity seed: start LK near the predicted location so
            // coarse levels do not have to recover large inter-frame motion
            // from a zero-flow init (mid-MH_01 rotation bursts).
            let x_pred = track.x + track.vx;
            let y_pred = track.y + track.vy;
            let Some((xf, yf)) = track_point(
                prev_pyr,
                curr_pyr,
                track.x,
                track.y,
                x_pred,
                y_pred,
                max_iters,
                &self.pattern,
            ) else {
                continue;
            };
            let Some((xb, yb)) = track_point(
                curr_pyr,
                prev_pyr,
                xf,
                yf,
                track.x,
                track.y,
                max_iters,
                &self.pattern,
            ) else {
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
                vx: xf - track.x,
                vy: yf - track.y,
            });
        }
        kept
    }

    fn detect_new(&mut self, image: &GrayImage) {
        // Basalt `detectKeypoints`: centered grid of PATCH_SIZE cells, skip
        // cells that already contain a live track, FAST per empty cell.
        let patch = self.config.optical_flow_detection_grid_size.max(8) as usize;
        if image.width < patch * 2 || image.height < patch * 2 {
            return;
        }
        let x_start = (image.width % patch) / 2;
        let y_start = (image.height % patch) / 2;
        let x_stop = x_start + patch * (image.width / patch - 1);
        let y_stop = y_start + patch * (image.height / patch - 1);
        let cols = (image.width / patch) + 1;
        let rows = (image.height / patch) + 1;
        let mut occupied = vec![false; cols * rows];
        for t in &self.tracks {
            if t.x >= x_start as f32
                && t.y >= y_start as f32
                && t.x < (x_stop + patch) as f32
                && t.y < (y_stop + patch) as f32
            {
                let cx = ((t.x as usize - x_start) / patch).min(cols.saturating_sub(1));
                let cy = ((t.y as usize - y_start) / patch).min(rows.saturating_sub(1));
                occupied[cy * cols + cx] = true;
            }
        }

        let mut x = x_start;
        while x <= x_stop {
            let mut y = y_start;
            while y <= y_stop {
                let cx = (x - x_start) / patch;
                let cy = (y - y_start) / patch;
                if !occupied[cy * cols + cx] {
                    if let Some((bx, by)) =
                        super::fast::best_fast_in_cell(image, x, y, x + patch, y + patch)
                    {
                        let id = self.next_id;
                        self.next_id += 1;
                        self.tracks.push(TrackedKeypoint {
                            id,
                            x: bx,
                            y: by,
                            vx: 0.0,
                            vy: 0.0,
                        });
                    }
                }
                y += patch;
            }
            x += patch;
        }
    }
}

/// Coarse-to-fine LSSD translation tracking (Basalt `trackPoint` layout).
///
/// Pyramid layers are `0..=optical_flow_levels`. Any invalid patch / residual
/// fails the whole track (Basalt `patch_valid &= ...`).
///
/// `x_ref,y_ref` locate the template in `from`; `x_init,y_init` seed the
/// search in `to` (full-resolution). Same-point init recovers identity /
/// stereo; temporal tracking seeds with the last displacement.
#[allow(clippy::too_many_arguments)]
fn track_point(
    from_pyr: &[GrayImage],
    to_pyr: &[GrayImage],
    x_ref: f32,
    y_ref: f32,
    x_init: f32,
    y_init: f32,
    max_iters: usize,
    pattern: &[(f32, f32); PATTERN51_SIZE],
) -> Option<(f32, f32)> {
    let num_layers = from_pyr.len().min(to_pyr.len());
    if num_layers == 0 {
        return None;
    }
    let max_level = num_layers - 1;
    let mut x = x_init;
    let mut y = y_init;
    for level in (0..=max_level).rev() {
        let scale = (1 << level) as f32;
        x /= scale;
        y /= scale;

        let patch =
            LssdPatch::from_image(&from_pyr[level], (x_ref / scale, y_ref / scale), pattern);
        if !patch.valid {
            return None;
        }
        for _ in 0..max_iters {
            let (dx, dy) = patch.track_step(&to_pyr[level], (x, y), pattern)?;
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

        let mut cfg = BasaltOpticalFlowConfig {
            optical_flow_levels: 3,
            optical_flow_max_iterations: 5,
            ..Default::default()
        };
        cfg.optical_flow_max_recovered_dist2 = 0.04;
        let mut tracker = OpticalFlowTracker::new(cfg.clone());
        tracker.tracks.push(TrackedKeypoint {
            id: 1,
            x: cx,
            y: cy,
            vx: 0.0,
            vy: 0.0,
        });
        tracker.next_id = 2;
        tracker.prev_pyramid = Some(build_pyramid(&img0, cfg.optical_flow_levels as usize));

        let obs1 = tracker.process(&img1);
        let hit = obs1
            .iter()
            .find(|o| o.id == 1)
            .expect("track 1 survived FB=0.04");
        assert!(
            (hit.x - (cx + 3.0)).abs() < 0.5 && (hit.y - (cy - 2.0)).abs() < 0.5,
            "expected ~(+3,-2) flow, got ({}, {})",
            hit.x,
            hit.y
        );
    }

    #[test]
    fn velocity_seed_tracks_larger_shift() {
        let mut data = vec![40u8; 128 * 96];
        let (cx, cy) = (50.0f32, 48.0f32);
        for y in 0..96 {
            for x in 0..128 {
                let dx = x as f32 - cx;
                let dy = y as f32 - cy;
                let g = (-0.08 * (dx * dx + dy * dy)).exp();
                data[y * 128 + x] = (40.0 + 180.0 * g) as u8;
            }
        }
        let img0 = GrayImage::from_luma8(128, 96, data).unwrap();
        // 8px shift is hard from a zero seed at FB=0.04; prior velocity helps.
        let img1 = shift_image(&img0, 8, -4);

        let mut cfg = BasaltOpticalFlowConfig {
            optical_flow_levels: 3,
            optical_flow_max_iterations: 5,
            ..Default::default()
        };
        cfg.optical_flow_max_recovered_dist2 = 0.04;
        let mut tracker = OpticalFlowTracker::new(cfg.clone());
        tracker.tracks.push(TrackedKeypoint {
            id: 1,
            x: cx,
            y: cy,
            vx: 8.0,
            vy: -4.0,
        });
        tracker.next_id = 2;
        tracker.prev_pyramid = Some(build_pyramid(&img0, cfg.optical_flow_levels as usize));

        let obs1 = tracker.process(&img1);
        let hit = obs1
            .iter()
            .find(|o| o.id == 1)
            .expect("velocity-seeded track should survive");
        assert!(
            (hit.x - (cx + 8.0)).abs() < 0.75 && (hit.y - (cy - 4.0)).abs() < 0.75,
            "expected ~(+8,-4) flow, got ({}, {})",
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

        let mut cfg = BasaltOpticalFlowConfig {
            optical_flow_levels: 2,
            optical_flow_max_iterations: 5,
            ..Default::default()
        };
        cfg.optical_flow_max_recovered_dist2 = 0.25;
        let mut tracker = OpticalFlowTracker::new(cfg.clone());
        tracker.tracks.push(TrackedKeypoint {
            id: 1,
            x: cx,
            y: cy,
            vx: 0.0,
            vy: 0.0,
        });
        tracker.next_id = 2;
        tracker.prev_pyramid = Some(build_pyramid(&img0, cfg.optical_flow_levels as usize));
        let obs1 = tracker.process(&shifted);
        let hit = obs1.iter().find(|o| o.id == 1).expect("gain-robust track");
        assert!((hit.x - (cx + 2.0)).abs() < 1.0 && (hit.y - (cy + 1.0)).abs() < 1.0);
    }
}
