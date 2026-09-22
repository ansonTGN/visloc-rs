//! Core 3D Gaussian Splatting types shared by the reference renderer and the
//! (future) GPU renderer/trainer.
//!
//! A 3DGS scene is a set of anisotropic 3D Gaussians. Each primitive stores a
//! world-space mean, a log-scale in each local axis, a rotation as a
//! unit quaternion, an opacity logit, and spherical-harmonics colour
//! coefficients. The stored representation mirrors the Inria / gsplat
//! convention: activation functions (`exp` on scale, `sigmoid` on opacity)
//! are applied at render time, and the raw values are what an optimizer
//! updates.

use nalgebra::{Quaternion, Vector3};

/// Number of spherical-harmonics coefficients per colour channel for a given
/// SH degree: `(degree + 1)^2` total coefficients, minus the degree-0 term
/// which is stored separately as the DC component.
pub const fn sh_rest_coeffs_per_channel(degree: u32) -> usize {
    match degree {
        0 => 0,
        1 => 9,
        2 => 24,
        3 => 45,
        other => ((other + 1) * (other + 1)) as usize - 1,
    }
}

/// One anisotropic 3D Gaussian primitive.
#[derive(Debug, Clone, PartialEq)]
pub struct Gaussian {
    /// World-space mean of the Gaussian.
    pub mean: Vector3<f32>,
    /// Per-axis log-scale. The linear scale is `scale_log.exp()`, matching the
    /// Inria `scale_*` PLY fields.
    pub scale_log: Vector3<f32>,
    /// Rotation as an unnormalized quaternion `(w, x, y, z)`, matching the
    /// Inria `rot_*` PLY fields. It is normalized before use.
    pub rotation: Quaternion<f32>,
    /// Opacity logit. The linear opacity is `sigmoid(opacity_logit)`, matching
    /// the Inria `opacity` PLY field.
    pub opacity_logit: f32,
    /// Degree-0 spherical-harmonics coefficient per RGB channel (the DC term).
    pub sh_dc: [f32; 3],
    /// Higher-order spherical-harmonics coefficients, channel-major: all
    /// coefficients for R, then G, then B. Empty for degree 0.
    ///
    /// Layout: `sh_rest[channel * coeffs_per_channel + k]`.
    pub sh_rest: Vec<f32>,
    /// Spherical-harmonics degree this primitive stores (`sh_rest` must have
    /// `3 * sh_rest_coeffs_per_channel(degree)` elements).
    pub sh_degree: u32,
}

impl Gaussian {
    /// Linear per-axis scale (`exp` of the stored log-scale).
    pub fn scale(&self) -> Vector3<f32> {
        self.scale_log.map(f32::exp)
    }

    /// Linear opacity in `(0, 1)` (`sigmoid` of the stored logit).
    pub fn opacity(&self) -> f32 {
        sigmoid(self.opacity_logit)
    }

    /// Unit rotation quaternion (normalizes the stored value).
    pub fn unit_rotation(&self) -> Quaternion<f32> {
        let n = self.rotation.norm();
        if n.is_finite() && n > f32::EPSILON {
            self.rotation / n
        } else {
            Quaternion::new(1.0, 0.0, 0.0, 0.0)
        }
    }

    /// Validate that `sh_rest` length is consistent with `sh_degree`.
    pub fn sh_is_consistent(&self) -> bool {
        self.sh_rest.len() == 3 * sh_rest_coeffs_per_channel(self.sh_degree)
    }
}

/// A scene is just a flat list of primitives plus the SH degree they share.
#[derive(Debug, Clone, PartialEq)]
pub struct Scene {
    pub gaussians: Vec<Gaussian>,
    pub sh_degree: u32,
}

impl Scene {
    pub fn new(gaussians: Vec<Gaussian>, sh_degree: u32) -> Self {
        Self {
            gaussians,
            sh_degree,
        }
    }

    pub fn len(&self) -> usize {
        self.gaussians.len()
    }

    pub fn is_empty(&self) -> bool {
        self.gaussians.is_empty()
    }

    /// World-space axis-aligned bounds of the means, or `None` if empty.
    pub fn bounds(&self) -> Option<(Vector3<f32>, Vector3<f32>)> {
        let mut it = self.gaussians.iter();
        let first = it.next()?.mean;
        let mut min = first;
        let mut max = first;
        for g in it {
            min = min.inf(&g.mean);
            max = max.sup(&g.mean);
        }
        Some((min, max))
    }
}

#[inline]
pub fn sigmoid(x: f32) -> f32 {
    1.0 / (1.0 + (-x).exp())
}

#[inline]
pub fn srgb_to_linear(c: f32) -> f32 {
    if c <= 0.04045 {
        c / 12.92
    } else {
        ((c + 0.055) / 1.055).powf(2.4)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sigmoid_matches_reference() {
        assert!((sigmoid(0.0) - 0.5).abs() < 1e-6);
        assert!(sigmoid(10.0) > 0.999);
        assert!(sigmoid(-10.0) < 1e-3);
    }

    #[test]
    fn scale_and_opacity_use_activation() {
        let g = Gaussian {
            mean: Vector3::zeros(),
            scale_log: Vector3::new(0.0, std::f32::consts::LN_2, -std::f32::consts::LN_2),
            rotation: Quaternion::new(1.0, 0.0, 0.0, 0.0),
            opacity_logit: 0.0,
            sh_dc: [0.0; 3],
            sh_rest: Vec::new(),
            sh_degree: 0,
        };
        let s = g.scale();
        assert!((s.x - 1.0).abs() < 1e-5);
        assert!((s.y - 2.0).abs() < 1e-5);
        assert!((s.z - 0.5).abs() < 1e-5);
        assert!((g.opacity() - 0.5).abs() < 1e-6);
    }

    #[test]
    fn rotation_normalizes() {
        let g = Gaussian {
            mean: Vector3::zeros(),
            scale_log: Vector3::zeros(),
            rotation: Quaternion::new(2.0, 0.0, 0.0, 0.0),
            opacity_logit: 0.0,
            sh_dc: [0.0; 3],
            sh_rest: Vec::new(),
            sh_degree: 0,
        };
        let q = g.unit_rotation();
        assert!((q.norm() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn sh_coeff_counts() {
        assert_eq!(sh_rest_coeffs_per_channel(0), 0);
        assert_eq!(sh_rest_coeffs_per_channel(1), 9);
        assert_eq!(sh_rest_coeffs_per_channel(2), 24);
        assert_eq!(sh_rest_coeffs_per_channel(3), 45);
    }

    #[test]
    fn bounds_of_scene() {
        let mk = |x: f32| Gaussian {
            mean: Vector3::new(x, 0.0, 0.0),
            scale_log: Vector3::zeros(),
            rotation: Quaternion::new(1.0, 0.0, 0.0, 0.0),
            opacity_logit: 0.0,
            sh_dc: [0.0; 3],
            sh_rest: Vec::new(),
            sh_degree: 0,
        };
        let scene = Scene::new(vec![mk(1.0), mk(-2.0), mk(3.0)], 0);
        let (min, max) = scene.bounds().unwrap();
        assert_eq!(min.x, -2.0);
        assert_eq!(max.x, 3.0);
        assert!(Scene::new(vec![], 0).bounds().is_none());
    }
}
