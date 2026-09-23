//! Image-quality metrics on display-space RGB in `[0, 1]`.

/// Peak signal-to-noise ratio (dB) between a render and its ground truth, both
/// row-major RGB in `[0, 1]`. The render is clamped to `[0, 1]` first, as it
/// would be when written to an 8-bit image. Returns `f64::INFINITY` for an
/// exact match.
///
/// # Panics
/// If the two images have different lengths.
pub fn psnr(render: &[[f32; 3]], ground_truth: &[[f32; 3]]) -> f64 {
    assert_eq!(render.len(), ground_truth.len(), "image sizes differ");
    if render.is_empty() {
        return f64::INFINITY;
    }
    let mut sum = 0.0f64;
    for (r, g) in render.iter().zip(ground_truth) {
        for c in 0..3 {
            let d = (r[c].clamp(0.0, 1.0) - g[c]) as f64;
            sum += d * d;
        }
    }
    let mse = sum / (render.len() * 3) as f64;
    if mse == 0.0 {
        f64::INFINITY
    } else {
        -10.0 * mse.log10()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn psnr_of_uniform_error() {
        // Every channel off by 0.1 -> MSE 0.01 -> 20 dB.
        let gt = vec![[0.5f32; 3]; 16];
        let r = vec![[0.6f32; 3]; 16];
        assert!((psnr(&r, &gt) - 20.0).abs() < 1e-4);
    }

    #[test]
    fn psnr_clamps_render() {
        // A render overshooting to 2.0 scores as 1.0.
        let gt = vec![[1.0f32; 3]; 4];
        let r = vec![[2.0f32; 3]; 4];
        assert!(psnr(&r, &gt).is_infinite());
    }
}
