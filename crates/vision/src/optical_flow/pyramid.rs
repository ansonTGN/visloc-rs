//! Grayscale image + pyramid helpers for Basalt-style OF.

/// Contiguous u8 grayscale image (row-major).
#[derive(Debug, Clone)]
pub struct GrayImage {
    pub width: usize,
    pub height: usize,
    pub data: Vec<u8>,
}

impl GrayImage {
    pub fn from_luma8(width: usize, height: usize, data: Vec<u8>) -> Option<Self> {
        if width == 0 || height == 0 || data.len() != width * height {
            return None;
        }
        Some(Self {
            width,
            height,
            data,
        })
    }

    #[inline]
    pub fn get(&self, x: i32, y: i32) -> Option<u8> {
        if x < 0 || y < 0 || x as usize >= self.width || y as usize >= self.height {
            return None;
        }
        Some(self.data[y as usize * self.width + x as usize])
    }

    /// Basalt `Image::InBounds(p, border)` — true when a `border`-pixel
    /// margin remains around `(x, y)`.
    #[inline]
    pub fn in_bounds_margin(&self, x: f32, y: f32, border: f32) -> bool {
        x >= border
            && y >= border
            && x < (self.width as f32 - border)
            && y < (self.height as f32 - border)
    }

    /// Bilinear sample; returns `None` outside the valid interior.
    pub fn sample_bilinear(&self, x: f32, y: f32) -> Option<f32> {
        if x < 0.0 || y < 0.0 || x > (self.width as f32 - 1.0) || y > (self.height as f32 - 1.0)
        {
            return None;
        }
        let x0 = x.floor() as i32;
        let y0 = y.floor() as i32;
        let x1 = x0 + 1;
        let y1 = y0 + 1;
        let dx = x - x0 as f32;
        let dy = y - y0 as f32;
        let i00 = self.get(x0, y0)? as f32;
        let i10 = self.get(x1, y0)? as f32;
        let i01 = self.get(x0, y1)? as f32;
        let i11 = self.get(x1, y1)? as f32;
        Some(
            i00 * (1.0 - dx) * (1.0 - dy)
                + i10 * dx * (1.0 - dy)
                + i01 * (1.0 - dx) * dy
                + i11 * dx * dy,
        )
    }
}

/// Number of pyramid layers for Basalt `optical_flow_levels`.
///
/// Basalt `ManagedImagePyr::setFromImage(img, num_levels)` builds levels
/// `0..=num_levels` (inclusive), so `optical_flow_levels = 3` → 4 layers.
#[inline]
pub fn pyramid_layer_count(optical_flow_levels: usize) -> usize {
    optical_flow_levels + 1
}

/// Basalt `ManagedImagePyr::border101` (used on the far side of a tap).
#[inline]
fn border101(x: i32, size: i32) -> i32 {
    size - 1 - (size - 1 - x).abs()
}

/// Basalt 5-tap binomial `[1,4,6,4,1]` subsample-by-2 (separable, `/256`).
///
/// Clean-room port of `ManagedImagePyr::subsample` in basalt-headers.
fn subsample_binomial(img: &GrayImage) -> GrayImage {
    const KERNEL: [i32; 5] = [1, 4, 6, 4, 1];
    let out_w = (img.width / 2).max(1);
    let out_h = (img.height / 2).max(1);
    let w = img.width as i32;
    let h = img.height as i32;

    // Vertical pass → (out_h × width) int accumulator (Basalt `tmp`).
    let mut tmp = vec![0i32; out_h * img.width];
    for r in 0..out_h {
        let r_i = r as i32;
        // Basalt: abs on the near (top) side, border101 on the far side.
        let ys = [
            (2 * r_i - 2).abs(),
            (2 * r_i - 1).abs(),
            2 * r_i,
            border101(2 * r_i + 1, h),
            border101(2 * r_i + 2, h),
        ];
        for c in 0..img.width {
            let mut acc = 0i32;
            for k in 0..5 {
                let y = ys[k] as usize;
                acc += KERNEL[k] * img.data[y * img.width + c] as i32;
            }
            tmp[r * img.width + c] = acc;
        }
    }

    // Horizontal pass → (out_h × out_w) u8, round-div 256.
    let mut data = vec![0u8; out_w * out_h];
    for r in 0..out_h {
        for c in 0..out_w {
            let c_i = c as i32;
            let xs = [
                (2 * c_i - 2).abs(),
                (2 * c_i - 1).abs(),
                2 * c_i,
                border101(2 * c_i + 1, w),
                border101(2 * c_i + 2, w),
            ];
            let mut acc = 0i32;
            for k in 0..5 {
                let x = xs[k] as usize;
                acc += KERNEL[k] * tmp[r * img.width + x];
            }
            data[r * out_w + c] = ((acc + (1 << 7)) >> 8).clamp(0, 255) as u8;
        }
    }

    GrayImage {
        width: out_w,
        height: out_h,
        data,
    }
}

/// Build pyramid levels `0..=optical_flow_levels` (Basalt convention).
///
/// Level 0 is full resolution; coarser levels use Basalt's 5×5 binomial
/// Gaussian subsample.
pub fn build_pyramid(image: &GrayImage, optical_flow_levels: usize) -> Vec<GrayImage> {
    let num_layers = pyramid_layer_count(optical_flow_levels);
    let mut out = Vec::with_capacity(num_layers);
    out.push(image.clone());
    for _ in 1..num_layers {
        let next = subsample_binomial(out.last().unwrap());
        out.push(next);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pyramid_matches_basalt_level_count() {
        let img = GrayImage::from_luma8(64, 48, vec![128u8; 64 * 48]).unwrap();
        // optical_flow_levels = 3 → layers 0..=3
        let pyr = build_pyramid(&img, 3);
        assert_eq!(pyr.len(), 4);
        assert_eq!((pyr[0].width, pyr[0].height), (64, 48));
        assert_eq!((pyr[1].width, pyr[1].height), (32, 24));
        assert_eq!((pyr[2].width, pyr[2].height), (16, 12));
        assert_eq!((pyr[3].width, pyr[3].height), (8, 6));
    }

    #[test]
    fn binomial_preserves_constant_image() {
        let img = GrayImage::from_luma8(32, 24, vec![100u8; 32 * 24]).unwrap();
        let half = subsample_binomial(&img);
        assert_eq!((half.width, half.height), (16, 12));
        assert!(
            half.data.iter().all(|&v| v == 100),
            "constant field must survive /256 binomial, got {:?}",
            half.data.iter().copied().max()
        );
    }

    #[test]
    fn binomial_smooths_impulse() {
        let mut data = vec![0u8; 32 * 32];
        data[16 * 32 + 16] = 255;
        let img = GrayImage::from_luma8(32, 32, data).unwrap();
        let half = subsample_binomial(&img);
        // Impulse energy spreads; center neighbourhood must be nonzero and
        // strictly less than 255.
        let cx = 8usize;
        let cy = 8usize;
        let center = half.data[cy * half.width + cx];
        assert!(center > 0 && center < 255, "center={center}");
        let sum: u32 = half.data.iter().map(|&v| v as u32).sum();
        assert!(sum > 0);
    }
}
