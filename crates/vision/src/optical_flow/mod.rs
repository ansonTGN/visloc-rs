//! Frame-to-frame optical flow (Basalt-style KLT) for the faithful VI port.
//!
//! Parameter names and EuRoC defaults mirror Basalt's `euroc_config.json`
//! (`configs/basalt/euroc_config.json`). This is a clean-room reimplementation
//! of the *interface* and defaults, not a line copy of Basalt C++.

mod config;
mod feature_bridge;
mod pyramid;
mod tracker;

pub use config::{BasaltOpticalFlowConfig, BasaltVioConfig, BasaltVioConfigFile};
pub use feature_bridge::{
    encode_track_id_descriptor, gray_image_from_features_image, OpticalFlowFeatureExtractor,
    TRACK_ID_DESCRIPTOR_DIM,
};
pub use pyramid::{build_pyramid, GrayImage};
pub use tracker::{
    track_points_between, OpticalFlowObservation, OpticalFlowTracker, TrackedKeypoint,
    PATTERN51_SIZE,
};
