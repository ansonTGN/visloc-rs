//! Basalt `euroc_config.json` bindings (faithful defaults).

use serde::Deserialize;

/// Optical-flow knobs from Basalt `config.optical_flow_*`.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct BasaltOpticalFlowConfig {
    /// Basalt: `config.optical_flow_type` (`frame_to_frame`).
    #[serde(default = "default_flow_type", rename = "config.optical_flow_type")]
    pub optical_flow_type: String,
    /// Basalt: `config.optical_flow_detection_grid_size` (default 50).
    #[serde(
        default = "default_grid_size",
        rename = "config.optical_flow_detection_grid_size"
    )]
    pub optical_flow_detection_grid_size: i32,
    /// Basalt: `config.optical_flow_max_recovered_dist2` (default 0.04).
    /// Used for **stereo** same-timestamp LK. Temporal frame-to-frame may
    /// override via [`Self::optical_flow_temporal_max_recovered_dist2`].
    #[serde(
        default = "default_max_recovered_dist2",
        rename = "config.optical_flow_max_recovered_dist2"
    )]
    pub optical_flow_max_recovered_dist2: f32,
    /// Optional looser FB² gate for **temporal** tracking only. `None` means
    /// use [`Self::optical_flow_max_recovered_dist2`]. Not present in Basalt
    /// JSON — set by the demo profile when EuRoC needs longer track life
    /// without poisoning stereo triangulation.
    #[serde(default, skip)]
    pub optical_flow_temporal_max_recovered_dist2: Option<f32>,
    /// Basalt: `config.optical_flow_pattern` (default 51).
    #[serde(default = "default_pattern", rename = "config.optical_flow_pattern")]
    pub optical_flow_pattern: i32,
    /// Basalt: `config.optical_flow_max_iterations` (default 5).
    #[serde(
        default = "default_max_iterations",
        rename = "config.optical_flow_max_iterations"
    )]
    pub optical_flow_max_iterations: i32,
    /// Basalt: `config.optical_flow_epipolar_error` (default 0.005).
    #[serde(
        default = "default_epipolar_error",
        rename = "config.optical_flow_epipolar_error"
    )]
    pub optical_flow_epipolar_error: f32,
    /// Basalt: `config.optical_flow_levels` (default 3).
    #[serde(default = "default_levels", rename = "config.optical_flow_levels")]
    pub optical_flow_levels: i32,
    /// Basalt: `config.optical_flow_skip_frames` (default 1).
    #[serde(
        default = "default_skip_frames",
        rename = "config.optical_flow_skip_frames"
    )]
    pub optical_flow_skip_frames: i32,
}

impl BasaltOpticalFlowConfig {
    /// FB² threshold for frame-to-frame tracking.
    #[inline]
    pub fn temporal_max_recovered_dist2(&self) -> f32 {
        self.optical_flow_temporal_max_recovered_dist2
            .unwrap_or(self.optical_flow_max_recovered_dist2)
    }

    /// FB² threshold for same-timestamp stereo LK (always the Basalt value).
    #[inline]
    pub const fn stereo_max_recovered_dist2(&self) -> f32 {
        self.optical_flow_max_recovered_dist2
    }
}

impl Default for BasaltOpticalFlowConfig {
    fn default() -> Self {
        Self {
            optical_flow_type: default_flow_type(),
            optical_flow_detection_grid_size: default_grid_size(),
            optical_flow_max_recovered_dist2: default_max_recovered_dist2(),
            optical_flow_temporal_max_recovered_dist2: None,
            optical_flow_pattern: default_pattern(),
            optical_flow_max_iterations: default_max_iterations(),
            optical_flow_epipolar_error: default_epipolar_error(),
            optical_flow_levels: default_levels(),
            optical_flow_skip_frames: default_skip_frames(),
        }
    }
}

/// VIO knobs from Basalt `config.vio_*` used by the sliding window.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct BasaltVioConfig {
    #[serde(flatten)]
    pub optical_flow: BasaltOpticalFlowConfig,
    /// Basalt: `config.vio_linearization_type` (`ABS_QR`).
    #[serde(
        default = "default_linearization",
        rename = "config.vio_linearization_type"
    )]
    pub vio_linearization_type: String,
    /// Basalt: `config.vio_sqrt_marg` (default true).
    #[serde(default = "default_true", rename = "config.vio_sqrt_marg")]
    pub vio_sqrt_marg: bool,
    /// Basalt: `config.vio_max_states` (default 3).
    #[serde(default = "default_max_states", rename = "config.vio_max_states")]
    pub vio_max_states: i32,
    /// Basalt: `config.vio_max_kfs` (default 7).
    #[serde(default = "default_max_kfs", rename = "config.vio_max_kfs")]
    pub vio_max_kfs: i32,
    /// Basalt: `config.vio_min_frames_after_kf` (default 5).
    #[serde(
        default = "default_min_frames_after_kf",
        rename = "config.vio_min_frames_after_kf"
    )]
    pub vio_min_frames_after_kf: i32,
    /// Basalt: `config.vio_new_kf_keypoints_thresh` (default 0.7).
    #[serde(
        default = "default_new_kf_thresh",
        rename = "config.vio_new_kf_keypoints_thresh"
    )]
    pub vio_new_kf_keypoints_thresh: f32,
    /// Basalt: `config.vio_obs_std_dev` (default 0.5).
    #[serde(default = "default_obs_std", rename = "config.vio_obs_std_dev")]
    pub vio_obs_std_dev: f64,
    /// Basalt: `config.vio_obs_huber_thresh` (default 1.0).
    #[serde(default = "default_huber", rename = "config.vio_obs_huber_thresh")]
    pub vio_obs_huber_thresh: f64,
    /// Basalt: `config.vio_min_triangulation_dist` (default 0.05).
    #[serde(
        default = "default_min_tri_dist",
        rename = "config.vio_min_triangulation_dist"
    )]
    pub vio_min_triangulation_dist: f64,
    /// Basalt: `config.vio_outlier_threshold` (default 3.0).
    #[serde(
        default = "default_outlier_threshold",
        rename = "config.vio_outlier_threshold"
    )]
    pub vio_outlier_threshold: f64,
    /// Basalt: `config.vio_max_iterations` (default 7).
    #[serde(default = "default_vio_iters", rename = "config.vio_max_iterations")]
    pub vio_max_iterations: i32,
    /// Basalt: `config.vio_use_lm` (default true).
    #[serde(default = "default_true", rename = "config.vio_use_lm")]
    pub vio_use_lm: bool,
    /// Basalt: `config.vio_lm_lambda_initial` (default 1e-4).
    #[serde(
        default = "default_lm_lambda_initial",
        rename = "config.vio_lm_lambda_initial"
    )]
    pub vio_lm_lambda_initial: f64,
    /// Basalt: `config.vio_lm_lambda_min` (default 1e-6).
    #[serde(default = "default_lm_lambda_min", rename = "config.vio_lm_lambda_min")]
    pub vio_lm_lambda_min: f64,
    /// Basalt: `config.vio_lm_lambda_max` (default 1e2).
    #[serde(default = "default_lm_lambda_max", rename = "config.vio_lm_lambda_max")]
    pub vio_lm_lambda_max: f64,
}

impl Default for BasaltVioConfig {
    fn default() -> Self {
        Self {
            optical_flow: BasaltOpticalFlowConfig::default(),
            vio_linearization_type: default_linearization(),
            vio_sqrt_marg: true,
            vio_max_states: default_max_states(),
            vio_max_kfs: default_max_kfs(),
            vio_min_frames_after_kf: default_min_frames_after_kf(),
            vio_new_kf_keypoints_thresh: default_new_kf_thresh(),
            vio_obs_std_dev: default_obs_std(),
            vio_obs_huber_thresh: default_huber(),
            vio_min_triangulation_dist: default_min_tri_dist(),
            vio_outlier_threshold: default_outlier_threshold(),
            vio_max_iterations: default_vio_iters(),
            vio_use_lm: true,
            vio_lm_lambda_initial: default_lm_lambda_initial(),
            vio_lm_lambda_min: default_lm_lambda_min(),
            vio_lm_lambda_max: default_lm_lambda_max(),
        }
    }
}

/// Wrapper matching Basalt's cereal `{"value0": { ... }}` JSON layout.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct BasaltVioConfigFile {
    pub value0: BasaltVioConfig,
}

impl BasaltVioConfigFile {
    pub fn from_json_str(json: &str) -> Result<Self, serde_json::Error> {
        serde_json::from_str(json)
    }

    pub fn from_path(
        path: impl AsRef<std::path::Path>,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let text = std::fs::read_to_string(path)?;
        Ok(Self::from_json_str(&text)?)
    }
}

fn default_flow_type() -> String {
    "frame_to_frame".into()
}
const fn default_grid_size() -> i32 {
    50
}
const fn default_max_recovered_dist2() -> f32 {
    0.04
}
const fn default_pattern() -> i32 {
    51
}
const fn default_max_iterations() -> i32 {
    5
}
const fn default_epipolar_error() -> f32 {
    0.005
}
const fn default_levels() -> i32 {
    3
}
const fn default_skip_frames() -> i32 {
    1
}
fn default_linearization() -> String {
    "ABS_QR".into()
}
const fn default_true() -> bool {
    true
}
const fn default_max_states() -> i32 {
    3
}
const fn default_max_kfs() -> i32 {
    7
}
const fn default_min_frames_after_kf() -> i32 {
    5
}
const fn default_new_kf_thresh() -> f32 {
    0.7
}
const fn default_obs_std() -> f64 {
    0.5
}
const fn default_huber() -> f64 {
    1.0
}
const fn default_min_tri_dist() -> f64 {
    0.05
}
const fn default_outlier_threshold() -> f64 {
    3.0
}
const fn default_vio_iters() -> i32 {
    7
}
const fn default_lm_lambda_initial() -> f64 {
    1e-4
}
const fn default_lm_lambda_min() -> f64 {
    1e-6
}
const fn default_lm_lambda_max() -> f64 {
    1e2
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn loads_checked_in_euroc_config() {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../configs/basalt/euroc_config.json"
        );
        let cfg = BasaltVioConfigFile::from_path(path).expect("load euroc_config.json");
        assert_eq!(cfg.value0.optical_flow.optical_flow_levels, 3);
        assert_eq!(cfg.value0.optical_flow.optical_flow_pattern, 51);
        assert_eq!(cfg.value0.vio_max_kfs, 7);
        assert!(cfg.value0.vio_sqrt_marg);
        assert_eq!(cfg.value0.vio_linearization_type, "ABS_QR");
        assert!((cfg.value0.vio_obs_std_dev - 0.5).abs() < 1e-12);
    }
}
