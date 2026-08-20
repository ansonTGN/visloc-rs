//! Frame-to-frame optical flow (Basalt-style KLT) for the faithful VI port.
//!
//! Parameter names and EuRoC defaults mirror Basalt's `euroc_config.json`
//! (`configs/basalt/euroc_config.json`). This is a clean-room reimplementation
//! of the *interface* and defaults, not a line copy of Basalt C++.

mod config;
mod pyramid;
mod tracker;

pub use config::{BasaltOpticalFlowConfig, BasaltVioConfig, BasaltVioConfigFile};
pub use pyramid::{build_pyramid, GrayImage};
pub use tracker::{OpticalFlowObservation, OpticalFlowTracker, TrackedKeypoint};
