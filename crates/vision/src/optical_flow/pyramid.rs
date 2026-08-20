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

/// Build pyramid levels `0..=optical_flow_levels` (Basalt convention).
///
/// Level 0 is full resolution. Downsample is currently 2×2 box (stable);
/// Basalt's 5×5 binomial Gaussian returns once temporal FB is green.
pub fn build_pyramid(image: &GrayImage, optical_flow_levels: usize) -> Vec<GrayImage> {
    let num_layers = pyramid_layer_count(optical_flow_levels);
    let mut out = Vec::with_capacity(num_layers);
    out.push(image.clone());
    for _ in 1..num_layers {
        let prev = out.last().unwrap();
        let w = (prev.width / 2).max(1);
        let h = (prev.height / 2).max(1);
        let mut data = vec![0u8; w * h];
        for y in 0..h {
            for x in 0..w {
                let x0 = x * 2;
                let y0 = y * 2;
                let x1 = (x0 + 1).min(prev.width - 1);
                let y1 = (y0 + 1).min(prev.height - 1);
                let s = prev.data[y0 * prev.width + x0] as u32
                    + prev.data[y0 * prev.width + x1] as u32
                    + prev.data[y1 * prev.width + x0] as u32
                    + prev.data[y1 * prev.width + x1] as u32;
                data[y * w + x] = (s / 4) as u8;
            }
        }
        out.push(GrayImage {
            width: w,
            height: h,
            data,
        });
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
}
