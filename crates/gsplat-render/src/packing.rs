//! Host-side packing of a [`visloc_gsplat_core::Scene`] into flat GPU buffers.
//!
//! The GPU kernels want the gaussian parameters as densely packed `f32` arrays
//! rather than an array of structs:
//!
//! - `transforms`: `[mean.x, mean.y, mean.z, qw, qx, qy, qz, lsx, lsy, lsz]`
//!   per gaussian (10 floats, the Inria / brush convention).
//! - `sh`: the degree-0 DC term followed by the channel-major higher-order
//!   coefficients, `[dc_r, dc_g, dc_b, rest...]` per gaussian, where `rest`
//!   has `3 * sh_rest_coeffs_per_channel(degree)` entries.
//! - `opacity`: the raw opacity logit per gaussian.

use visloc_gsplat_core::gaussian::{sh_rest_coeffs_per_channel, Scene};
use visloc_gsplat_core::sh::SH_C0;

/// Number of `f32` values per gaussian in the `transforms` buffer.
pub const TRANSFORM_FLOATS: usize = 10;

/// A scene packed for upload.
#[derive(Debug, Clone)]
pub struct PackedScene {
    pub transforms: Vec<f32>,
    pub sh: Vec<f32>,
    /// `(1 + degree)^2` SH coefficients per channel stored in `sh`.
    pub sh_coeffs_per_channel: usize,
    pub opacity: Vec<f32>,
    pub num_gaussians: usize,
    pub sh_degree: u32,
}

impl PackedScene {
    /// Pack `scene`, promoting every gaussian's SH to the scene degree by
    /// zero-padding missing higher-order coefficients.
    pub fn from_scene(scene: &Scene) -> Self {
        let degree = scene.sh_degree;
        let coeffs_per_channel = 1 + sh_rest_coeffs_per_channel(degree);
        let rest_per_channel = coeffs_per_channel - 1;
        let n = scene.len();

        let mut transforms = Vec::with_capacity(n * TRANSFORM_FLOATS);
        let mut sh = Vec::with_capacity(n * 3 * coeffs_per_channel);
        let mut opacity = Vec::with_capacity(n);

        for g in &scene.gaussians {
            transforms.extend_from_slice(&[
                g.mean.x,
                g.mean.y,
                g.mean.z,
                g.rotation.w,
                g.rotation.i,
                g.rotation.j,
                g.rotation.k,
                g.scale_log.x,
                g.scale_log.y,
                g.scale_log.z,
            ]);
            // DC term, then the per-channel high-order terms. The gaussian may
            // store fewer terms than the scene degree; missing ones are zeros.
            sh.extend_from_slice(&g.sh_dc);
            let own = sh_rest_coeffs_per_channel(g.sh_degree);
            for c in 0..3 {
                for k in 0..rest_per_channel {
                    let v = if k < own { g.sh_rest[c * own + k] } else { 0.0 };
                    sh.push(v);
                }
            }
            opacity.push(g.opacity_logit);
        }

        Self {
            transforms,
            sh,
            sh_coeffs_per_channel: coeffs_per_channel,
            opacity,
            num_gaussians: n,
            sh_degree: degree,
        }
    }

    /// The raw SH colour of gaussian `i` for a view direction, using the same
    /// `+0.5` DC offset as the renderer. Only used by tests / host-side
    /// reference comparisons.
    pub fn raw_dc_color(&self, i: usize) -> [f32; 3] {
        let base = i * 3 * self.sh_coeffs_per_channel;
        [
            SH_C0 * self.sh[base] + 0.5,
            SH_C0 * self.sh[base + 1] + 0.5,
            SH_C0 * self.sh[base + 2] + 0.5,
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Quaternion, Vector3};
    use visloc_gsplat_core::gaussian::Gaussian;

    fn gaussian(degree: u32) -> Gaussian {
        let n = sh_rest_coeffs_per_channel(degree);
        Gaussian {
            mean: Vector3::new(1.0, 2.0, 3.0),
            scale_log: Vector3::new(-1.0, 0.0, 1.0),
            rotation: Quaternion::new(1.0, 0.0, 0.0, 0.0),
            opacity_logit: 2.0,
            sh_dc: [0.1, 0.2, 0.3],
            sh_rest: (0..3 * n).map(|i| i as f32).collect(),
            sh_degree: degree,
        }
    }

    #[test]
    fn packs_transforms_in_expected_order() {
        let scene = Scene::new(vec![gaussian(1)], 1);
        let p = PackedScene::from_scene(&scene);
        assert_eq!(p.num_gaussians, 1);
        assert_eq!(
            &p.transforms,
            &[1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0]
        );
        assert_eq!(p.opacity, vec![2.0]);
        // Degree 1: 1 + 9 = 10 coefficients per channel.
        assert_eq!(p.sh_coeffs_per_channel, 10);
        // 3 dc + 3 channels * 9 rest = 30.
        assert_eq!(p.sh.len(), 30);
    }

    #[test]
    fn promotes_lower_degree_to_scene_degree() {
        let scene = Scene::new(vec![gaussian(0)], 2);
        let p = PackedScene::from_scene(&scene);
        // Degree 2: 1 + 24 = 25 coefficients per channel.
        assert_eq!(p.sh_coeffs_per_channel, 25);
        // 3 dc + 3 * 24 rest, all rest zeros.
        assert_eq!(p.sh.len(), 3 + 3 * 24);
        assert!(p.sh[3..].iter().all(|&v| v == 0.0));
    }

    #[test]
    fn dc_color_applies_offset_and_c0() {
        let scene = Scene::new(vec![gaussian(0)], 0);
        let p = PackedScene::from_scene(&scene);
        let c = p.raw_dc_color(0);
        assert!((c[0] - (SH_C0 * 0.1 + 0.5)).abs() < 1e-6);
    }
}
