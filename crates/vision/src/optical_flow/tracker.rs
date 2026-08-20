//! Frame-to-frame KLT with Basalt Pattern51 patch offsets.
//!
//! Uses inverse-compositional LK + SSD on Pattern51 samples. Illumination-
//! invariant LSSD and exact Basalt recovery checks land in follow-up commits;
//! grid detection / pyramid schedule already match `euroc_config.json`.

use super::config::BasaltOpticalFlowConfig;
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

fn pattern51_offsets() -> [(f32, f32); 52] {
    let mut out = [(0.0f32, 0.0f32); 52];
    for (i, p) in PATTERN52.iter().enumerate() {
        out[i] = (0.5 * p[0], 0.5 * p[1]);
    }
    out
}

/// Stable track id (Basalt `KeypointId`).
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

/// Stateful frame-to-frame optical-flow tracker.
pub struct OpticalFlowTracker {
    config: BasaltOpticalFlowConfig,
    next_id: KeypointId,
    prev_pyramid: Option<Vec<GrayImage>>,
    tracks: Vec<TrackedKeypoint>,
    pattern: [(f32, f32); 52],
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

    /// Process one grayscale frame; returns surviving (+ newly detected) observations.
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
            // Forward-backward consistency (Basalt recovered-dist gate).
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
    // Simple Harris-like score via intensity variance in a 3×3 window; enough
    // to seed tracks. Basalt uses FAST; swap in when wiring stereo OF.
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

fn track_point(
    from_pyr: &[GrayImage],
    to_pyr: &[GrayImage],
    x0: f32,
    y0: f32,
    max_iters: usize,
    pattern: &[(f32, f32); 52],
) -> Option<(f32, f32)> {
    let levels = from_pyr.len().min(to_pyr.len());
    let scale = 1.0f32 / (1 << (levels - 1)) as f32;
    let mut x = x0 * scale;
    let mut y = y0 * scale;

    for level in (0..levels).rev() {
        let level_scale = 1.0f32 / (1 << level) as f32;
        let target_x = x0 * level_scale;
        let target_y = y0 * level_scale;
        // Coarse init already in (x,y); refine on this level.
        let _ = (target_x, target_y);
        let from = &from_pyr[level];
        let to = &to_pyr[level];
        for _ in 0..max_iters {
            let Some((dx, dy, ok)) = lk_step(from, to, target_x, target_y, x, y, pattern) else {
                return None;
            };
            x += dx;
            y += dy;
            if !ok {
                break;
            }
            if dx * dx + dy * dy < 1e-4 {
                break;
            }
        }
        if level > 0 {
            x *= 2.0;
            y *= 2.0;
        }
    }
    Some((x, y))
}

fn lk_step(
    template: &GrayImage,
    image: &GrayImage,
    tx: f32,
    ty: f32,
    ix: f32,
    iy: f32,
    pattern: &[(f32, f32); 52],
) -> Option<(f32, f32, bool)> {
    // Inverse compositional: ∇T from the template; residual I(x+Δ) − T(x).
    let mut a11 = 0.0f32;
    let mut a12 = 0.0f32;
    let mut a22 = 0.0f32;
    let mut b1 = 0.0f32;
    let mut b2 = 0.0f32;
    let mut valid = 0usize;

    for &(ox, oy) in pattern {
        let px = tx + ox;
        let py = ty + oy;
        let Some(t) = template.sample_bilinear(px, py) else {
            continue;
        };
        let Some(t_xm) = template.sample_bilinear(px - 0.5, py) else {
            continue;
        };
        let Some(t_xp) = template.sample_bilinear(px + 0.5, py) else {
            continue;
        };
        let Some(t_ym) = template.sample_bilinear(px, py - 0.5) else {
            continue;
        };
        let Some(t_yp) = template.sample_bilinear(px, py + 0.5) else {
            continue;
        };
        let gx = t_xp - t_xm;
        let gy = t_yp - t_ym;
        let Some(i_val) = image.sample_bilinear(ix + ox, iy + oy) else {
            continue;
        };
        let r = i_val - t;
        a11 += gx * gx;
        a12 += gx * gy;
        a22 += gy * gy;
        b1 += gx * r;
        b2 += gy * r;
        valid += 1;
    }

    if valid < 16 {
        return None;
    }
    let det = a11 * a22 - a12 * a12;
    if det.abs() < 1e-6 {
        return Some((0.0, 0.0, false));
    }
    // Δ = H⁻¹ Σ ∇T · r  moves the *template* warp; apply −Δ to the image point.
    let dx = (a22 * b1 - a12 * b2) / det;
    let dy = (-a12 * b1 + a11 * b2) / det;
    Some((-dx, -dy, true))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::optical_flow::pyramid::build_pyramid;
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
    fn tracks_integer_shift() {
        let mut data = vec![40u8; 128 * 96];
        // Soft Gaussian blob → non-zero Pattern51 gradients at the center.
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
        cfg.optical_flow_levels = 1;
        cfg.optical_flow_max_iterations = 12;
        cfg.optical_flow_max_recovered_dist2 = 2.0;
        let mut tracker = OpticalFlowTracker::new(cfg);
        tracker.tracks.push(TrackedKeypoint {
            id: 1,
            x: cx,
            y: cy,
        });
        tracker.next_id = 2;
        tracker.prev_pyramid = Some(build_pyramid(&img0, 1));

        let obs1 = tracker.process(&img1);
        let hit = obs1.iter().find(|o| o.id == 1).expect("track 1 survived");
        assert!(
            (hit.x - (cx + 3.0)).abs() < 1.0 && (hit.y - (cy - 2.0)).abs() < 1.0,
            "expected ~(+3,-2) flow, got ({}, {})",
            hit.x,
            hit.y
        );
    }
}
