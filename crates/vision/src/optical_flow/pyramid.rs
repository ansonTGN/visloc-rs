//! Grayscale image + Gaussian pyramid helpers for Basalt-style OF.

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

    /// Bilinear sample; returns `None` outside the valid interior.
    pub fn sample_bilinear(&self, x: f32, y: f32) -> Option<f32> {
        if x < 0.0 || y < 0.0 || x > (self.width as f32 - 1.0) || y > (self.height as f32 - 1.0)
        {
            return None;
        }
        let x0 = x.floor() as i32;
        let y0 = y.floor() as i32;
        let x1 = (x0 + 1).min(self.width as i32 - 1);
        let y1 = (y0 + 1).min(self.height as i32 - 1);
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

/// Build a pyramid with `levels` layers (level 0 = full resolution).
/// Downsampling is 2×2 box average (Basalt uses a Gaussian pyramid; this is
/// the first faithful scaffolding step and matches resolution schedule).
pub fn build_pyramid(image: &GrayImage, levels: usize) -> Vec<GrayImage> {
    let levels = levels.max(1);
    let mut out = Vec::with_capacity(levels);
    out.push(image.clone());
    for _ in 1..levels {
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
    fn pyramid_halves_each_level() {
        let img = GrayImage::from_luma8(64, 48, vec![128u8; 64 * 48]).unwrap();
        let pyr = build_pyramid(&img, 3);
        assert_eq!(pyr.len(), 3);
        assert_eq!((pyr[0].width, pyr[0].height), (64, 48));
        assert_eq!((pyr[1].width, pyr[1].height), (32, 24));
        assert_eq!((pyr[2].width, pyr[2].height), (16, 12));
    }
}
