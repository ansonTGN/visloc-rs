//! FAST-9 corner detection for Basalt-style OF grid seeding.
//!
//! Clean-room port of the detection pattern in Basalt `detectKeypoints`
//! (OpenCV `cv::FAST` per grid cell, threshold 40 → 20 → 10 → 5, best
//! response wins). Operates on u8 images (Basalt feeds `uint16 >> 8`).

use super::pyramid::GrayImage;

/// Bresenham circle of radius 3 (16 samples) used by FAST.
const CIRCLE: [(i32, i32); 16] = [
    (0, -3),
    (1, -3),
    (2, -2),
    (3, -1),
    (3, 0),
    (3, 1),
    (2, 2),
    (1, 3),
    (0, 3),
    (-1, 3),
    (-2, 2),
    (-3, 1),
    (-3, 0),
    (-3, -1),
    (-2, -2),
    (-1, -3),
];

/// Basalt `EDGE_THRESHOLD` — reject detections this close to the image border.
pub const EDGE_THRESHOLD: usize = 19;

/// Best FAST-9 corner in `[x0,x1) × [y0,y1)`, or `None` if none pass.
///
/// Tries thresholds `40, 20, 10, 5` (Basalt) and returns the highest-scoring
/// detection that also clears [`EDGE_THRESHOLD`] in the full image.
pub fn best_fast_in_cell(
    image: &GrayImage,
    x0: usize,
    y0: usize,
    x1: usize,
    y1: usize,
) -> Option<(f32, f32)> {
    let mut threshold = 40i32;
    while threshold >= 5 {
        if let Some(hit) = best_fast_at_threshold(image, x0, y0, x1, y1, threshold) {
            return Some(hit);
        }
        threshold /= 2;
    }
    None
}

fn best_fast_at_threshold(
    image: &GrayImage,
    x0: usize,
    y0: usize,
    x1: usize,
    y1: usize,
    threshold: i32,
) -> Option<(f32, f32)> {
    let margin = 3usize;
    let xs = (x0 + margin)..x1.saturating_sub(margin);
    let ys = (y0 + margin)..y1.saturating_sub(margin);
    let mut best_score = 0i32;
    let mut best: Option<(f32, f32)> = None;

    for y in ys {
        for x in xs.clone() {
            if x < EDGE_THRESHOLD
                || y < EDGE_THRESHOLD
                || x + EDGE_THRESHOLD >= image.width
                || y + EDGE_THRESHOLD >= image.height
            {
                continue;
            }
            if let Some(score) = fast9_score(image, x, y, threshold) {
                if score > best_score {
                    best_score = score;
                    best = Some((x as f32, y as f32));
                }
            }
        }
    }
    best
}

/// Returns FAST response (sum of |d| over the contiguous arc) if the pixel
/// is a FAST-9 corner at `threshold`, else `None`.
fn fast9_score(image: &GrayImage, x: usize, y: usize, threshold: i32) -> Option<i32> {
    let c = image.data[y * image.width + x] as i32;
    let mut brighter = [false; 16];
    let mut darker = [false; 16];
    let mut diffs = [0i32; 16];
    for (i, &(dx, dy)) in CIRCLE.iter().enumerate() {
        let px = (x as i32 + dx) as usize;
        let py = (y as i32 + dy) as usize;
        let v = image.data[py * image.width + px] as i32;
        let d = v - c;
        diffs[i] = d;
        if d > threshold {
            brighter[i] = true;
        } else if d < -threshold {
            darker[i] = true;
        }
    }

    let mut best_run_score = 0i32;
    for start in 0..16 {
        // Brighter run
        let mut run = 0;
        let mut score = 0i32;
        for k in 0..16 {
            let i = (start + k) % 16;
            if brighter[i] {
                run += 1;
                score += diffs[i];
            } else {
                break;
            }
        }
        if run >= 9 {
            best_run_score = best_run_score.max(score);
        }
        // Darker run
        run = 0;
        score = 0;
        for k in 0..16 {
            let i = (start + k) % 16;
            if darker[i] {
                run += 1;
                score += -diffs[i];
            } else {
                break;
            }
        }
        if run >= 9 {
            best_run_score = best_run_score.max(score);
        }
    }
    if best_run_score > 0 {
        Some(best_run_score)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detects_bright_corner() {
        let mut data = vec![40u8; 64 * 64];
        // Bright L-corner around (32, 32).
        for y in 29..36 {
            for x in 29..36 {
                if x >= 32 || y >= 32 {
                    data[y * 64 + x] = 220;
                }
            }
        }
        let img = GrayImage::from_luma8(64, 64, data).unwrap();
        let hit = best_fast_in_cell(&img, 20, 20, 44, 44).expect("FAST corner");
        assert!(
            (hit.0 - 32.0).abs() < 4.0 && (hit.1 - 32.0).abs() < 4.0,
            "got {hit:?}"
        );
    }

    #[test]
    fn flat_cell_returns_none() {
        let img = GrayImage::from_luma8(64, 64, vec![128u8; 64 * 64]).unwrap();
        assert!(best_fast_in_cell(&img, 20, 20, 44, 44).is_none());
    }
}
