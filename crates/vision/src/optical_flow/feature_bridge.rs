//! Bridge Basalt-style optical flow into [`crate::features::FeatureSet`].
//!
//! Track IDs are encoded as unique descriptors so the existing OnlineSlam
//! descriptor matcher re-associates the same landmark across frames without
//! a separate temporal index (faithful enough for the first wiring step).

use std::cell::RefCell;

use nalgebra::Point2;

use crate::features::{FeatureExtractor, FeatureSet, GrayscaleImage};

use super::config::BasaltOpticalFlowConfig;
use super::pyramid::GrayImage;
use super::tracker::{
    track_points_between, OpticalFlowTracker, PATTERN51_SIZE,
};

/// Descriptor length for track-id encodings (compact + L2-friendly).
pub const TRACK_ID_DESCRIPTOR_DIM: usize = 16;

/// Encode a stable optical-flow track id as a unique unit-ish descriptor.
pub fn encode_track_id_descriptor(id: u64) -> Vec<f32> {
    let mut desc = vec![0.0f32; TRACK_ID_DESCRIPTOR_DIM];
    for i in 0..8 {
        let byte = ((id >> (8 * i)) & 0xff) as f32;
        desc[i] = byte;
        desc[i + 8] = 255.0 - byte;
    }
    let norm = desc.iter().map(|v| v * v).sum::<f32>().sqrt();
    if norm > 0.0 {
        for v in &mut desc {
            *v /= norm;
        }
    }
    desc
}

/// Convert a normalized [`GrayscaleImage`] (float 0..1) into OF [`GrayImage`].
pub fn gray_image_from_features_image(image: &GrayscaleImage) -> GrayImage {
    let data: Vec<u8> = image
        .pixels()
        .iter()
        .map(|p| (p.clamp(0.0, 1.0) * 255.0).round() as u8)
        .collect();
    GrayImage {
        width: image.width(),
        height: image.height(),
        data,
    }
}

/// Stateful extractor: one [`OpticalFlowTracker`] owned across frames.
pub struct OpticalFlowFeatureExtractor {
    tracker: RefCell<OpticalFlowTracker>,
    config: BasaltOpticalFlowConfig,
}

impl OpticalFlowFeatureExtractor {
    pub fn new(config: BasaltOpticalFlowConfig) -> Self {
        Self {
            tracker: RefCell::new(OpticalFlowTracker::new(config.clone())),
            config,
        }
    }

    pub fn with_euroc_defaults() -> Self {
        Self::new(BasaltOpticalFlowConfig::default())
    }

    pub fn config(&self) -> &BasaltOpticalFlowConfig {
        &self.config
    }

    /// Track `points` from `from` into `to` (e.g. cam0 → cam1 stereo).
    pub fn track_points_to(
        &self,
        from: &GrayscaleImage,
        to: &GrayscaleImage,
        points: &[(f32, f32)],
    ) -> Vec<Option<(f32, f32)>> {
        let from_g = gray_image_from_features_image(from);
        let to_g = gray_image_from_features_image(to);
        track_points_between(&from_g, &to_g, points, &self.config)
    }
}

impl FeatureExtractor for OpticalFlowFeatureExtractor {
    type Image = GrayscaleImage;
    type Error = String;

    fn extract(&self, image: &Self::Image) -> Result<FeatureSet, Self::Error> {
        let gray = gray_image_from_features_image(image);
        let obs = self.tracker.borrow_mut().process(&gray);
        let mut keypoints = Vec::with_capacity(obs.len());
        let mut descriptors = Vec::with_capacity(obs.len());
        for o in obs {
            keypoints.push(Point2::new(o.x as f64, o.y as f64));
            descriptors.push(encode_track_id_descriptor(o.id));
        }
        FeatureSet::new(keypoints, descriptors).map_err(|e| e.to_string())
    }
}

impl std::fmt::Debug for OpticalFlowFeatureExtractor {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("OpticalFlowFeatureExtractor")
            .field("config", &self.config)
            .field("pattern_size", &PATTERN51_SIZE)
            .finish()
    }
}

// RefCell interior is not Sync for Clone via tracker state — demo holds one
// extractor. Provide Clone that resets the tracker (fresh stream).
impl Clone for OpticalFlowFeatureExtractor {
    fn clone(&self) -> Self {
        Self::new(self.config.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn track_id_descriptors_are_unique_and_stable() {
        let a = encode_track_id_descriptor(42);
        let b = encode_track_id_descriptor(42);
        let c = encode_track_id_descriptor(43);
        assert_eq!(a, b);
        assert_ne!(a, c);
        assert_eq!(a.len(), TRACK_ID_DESCRIPTOR_DIM);
    }
}
