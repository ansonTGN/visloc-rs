//! Basalt-style LSSD residual (mean-normalized SSD) for translation IC.
//!
//! Clean-room port of the illumination model in Basalt `OpticalFlowPatch`
//! (`data /= mean`, residual = `I/mean_I - T/mean_T`). SE(2) Jacobian is
//! staged separately — full SE2 currently thins EuRoC tracks under FB=0.04.

use super::pyramid::GrayImage;
use super::tracker::PATTERN51_SIZE;

const INVALID: f32 = -1.0;

#[derive(Clone)]
pub struct LssdPatch {
    data: [f32; PATTERN51_SIZE],
    h_inv_jt: [[f32; PATTERN51_SIZE]; 2],
    pub valid: bool,
}

impl LssdPatch {
    // The fixed-size `PATTERN51_SIZE` pattern is intentionally walked by
    // index (not `.iter().enumerate()`) because several bodies below index
    // more than one array (e.g. `out.data`, `j`) in lockstep over the same
    // `PATTERN51_SIZE` range.
    #[allow(clippy::needless_range_loop)]
    pub fn from_image(
        image: &GrayImage,
        pos: (f32, f32),
        pattern: &[(f32, f32); PATTERN51_SIZE],
    ) -> Self {
        let mut data = [0.0f32; PATTERN51_SIZE];
        let mut j = [[0.0f32; 2]; PATTERN51_SIZE];
        let mut sum = 0.0f32;
        let mut grad_sum = [0.0f32; 2];
        let mut n_valid = 0usize;

        for i in 0..PATTERN51_SIZE {
            let (ox, oy) = pattern[i];
            let px = pos.0 + ox;
            let py = pos.1 + oy;
            let Some((val, gx, gy)) = sample_intensity_grad(image, px, py) else {
                data[i] = INVALID;
                continue;
            };
            data[i] = val;
            j[i][0] = gx;
            j[i][1] = gy;
            sum += val;
            grad_sum[0] += gx;
            grad_sum[1] += gy;
            n_valid += 1;
        }

        let mut out = Self {
            data,
            h_inv_jt: [[0.0; PATTERN51_SIZE]; 2],
            valid: false,
        };
        if n_valid < PATTERN51_SIZE / 2 || sum <= f32::EPSILON {
            return out;
        }

        let mean_inv = n_valid as f32 / sum;
        for i in 0..PATTERN51_SIZE {
            if out.data[i] < 0.0 {
                j[i] = [0.0, 0.0];
                continue;
            }
            j[i][0] -= grad_sum[0] * out.data[i] / sum;
            j[i][1] -= grad_sum[1] * out.data[i] / sum;
            out.data[i] *= mean_inv;
            j[i][0] *= mean_inv;
            j[i][1] *= mean_inv;
        }

        let mut h11 = 0.0f32;
        let mut h12 = 0.0f32;
        let mut h22 = 0.0f32;
        for i in 0..PATTERN51_SIZE {
            if out.data[i] < 0.0 {
                continue;
            }
            h11 += j[i][0] * j[i][0];
            h12 += j[i][0] * j[i][1];
            h22 += j[i][1] * j[i][1];
        }
        let det = h11 * h22 - h12 * h12;
        if !det.is_finite() || det.abs() < 1e-12 {
            return out;
        }
        let inv11 = h22 / det;
        let inv12 = -h12 / det;
        let inv22 = h11 / det;
        for i in 0..PATTERN51_SIZE {
            if out.data[i] < 0.0 {
                continue;
            }
            out.h_inv_jt[0][i] = inv11 * j[i][0] + inv12 * j[i][1];
            out.h_inv_jt[1][i] = inv12 * j[i][0] + inv22 * j[i][1];
            if !out.h_inv_jt[0][i].is_finite() || !out.h_inv_jt[1][i].is_finite() {
                return Self {
                    data: [0.0; PATTERN51_SIZE],
                    h_inv_jt: [[0.0; PATTERN51_SIZE]; 2],
                    valid: false,
                };
            }
        }
        out.valid = out.data.iter().all(|v| v.is_finite());
        out
    }

    /// Returns translation increment `(dx, dy)` to add to the image point.
    #[allow(clippy::needless_range_loop)]
    pub fn track_step(
        &self,
        image: &GrayImage,
        pos: (f32, f32),
        pattern: &[(f32, f32); PATTERN51_SIZE],
    ) -> Option<(f32, f32)> {
        if !self.valid {
            return None;
        }
        let mut raw = [0.0f32; PATTERN51_SIZE];
        let mut sum = 0.0f32;
        let mut n_valid = 0usize;
        for i in 0..PATTERN51_SIZE {
            let (ox, oy) = pattern[i];
            let px = pos.0 + ox;
            let py = pos.1 + oy;
            if !image.in_bounds_margin(px, py, 2.0) {
                raw[i] = INVALID;
                continue;
            }
            match image.sample_bilinear(px, py) {
                Some(v) => {
                    raw[i] = v;
                    sum += v;
                    n_valid += 1;
                }
                None => raw[i] = INVALID,
            }
        }
        if sum <= f32::EPSILON || n_valid < PATTERN51_SIZE / 2 {
            return None;
        }
        let mut residual = [0.0f32; PATTERN51_SIZE];
        let mut n_res = 0usize;
        for i in 0..PATTERN51_SIZE {
            if raw[i] >= 0.0 && self.data[i] >= 0.0 {
                residual[i] = (n_valid as f32) * raw[i] / sum - self.data[i];
                n_res += 1;
            } else {
                residual[i] = 0.0;
            }
        }
        if n_res <= PATTERN51_SIZE / 2 {
            return None;
        }
        let mut ix = 0.0f32;
        let mut iy = 0.0f32;
        for i in 0..PATTERN51_SIZE {
            ix -= self.h_inv_jt[0][i] * residual[i];
            iy -= self.h_inv_jt[1][i] * residual[i];
        }
        if !ix.is_finite() || !iy.is_finite() || ix.abs() > 1e6 || iy.abs() > 1e6 {
            return None;
        }
        Some((ix, iy))
    }
}

fn sample_intensity_grad(image: &GrayImage, x: f32, y: f32) -> Option<(f32, f32, f32)> {
    const MARGIN: f32 = 2.0;
    if x < MARGIN
        || y < MARGIN
        || x > (image.width as f32 - 1.0 - MARGIN)
        || y > (image.height as f32 - 1.0 - MARGIN)
    {
        return None;
    }
    let v = image.sample_bilinear(x, y)?;
    let xm = image.sample_bilinear(x - 0.5, y)?;
    let xp = image.sample_bilinear(x + 0.5, y)?;
    let ym = image.sample_bilinear(x, y - 0.5)?;
    let yp = image.sample_bilinear(x, y + 0.5)?;
    Some((v, xp - xm, yp - ym))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::optical_flow::tracker::pattern51_offsets_for_test;

    #[test]
    fn lssd_invariant_to_gain() {
        let pattern = pattern51_offsets_for_test();
        let mut data = vec![40u8; 64 * 64];
        for y in 20..44 {
            for x in 20..44 {
                let dx = x as f32 - 32.0;
                let dy = y as f32 - 32.0;
                let g = (-0.05 * (dx * dx + dy * dy)).exp();
                data[y * 64 + x] = (40.0 + 180.0 * g) as u8;
            }
        }
        let img = GrayImage::from_luma8(64, 64, data).unwrap();
        let patch = LssdPatch::from_image(&img, (32.0, 32.0), &pattern);
        assert!(patch.valid);
        let bright: Vec<u8> = img
            .data
            .iter()
            .map(|&v| ((v as f32) * 1.5).min(255.0) as u8)
            .collect();
        let img_b = GrayImage::from_luma8(64, 64, bright).unwrap();
        let (dx, dy) = patch
            .track_step(&img_b, (32.0, 32.0), &pattern)
            .expect("LSSD step");
        assert!(dx.abs() < 0.5 && dy.abs() < 0.5, "got ({dx}, {dy})");
    }
}
