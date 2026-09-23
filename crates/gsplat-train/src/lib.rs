//! 3D Gaussian Splatting training and evaluation for visloc-rs.
//!
//! Stage 3 of `docs/rust_3dgs_plan.md`. This crate holds what the trainer and
//! the benchmark harness share:
//!
//! - [`dataset`]: a COLMAP model plus its images, split into train / eval views
//!   the same way brush and the Inria code do (sort by image name, every Nth
//!   view is held out).
//! - [`metrics`]: image-quality metrics (PSNR) computed on display-space RGB in
//!   `[0, 1]`, the space 3DGS trainers optimise in.
//! - [`init`]: initial gaussians from coloured SfM points.
//! - [`densify`]: Inria-style clone / split / prune on the host.
//! - `trainer` (feature `gpu`): the on-device training loop.
//!
//! Scoring every method's splat with the same renderer, split and metric keeps
//! comparisons (e.g. against brush) apples to apples.

pub mod dataset;
pub mod densify;
pub mod init;
#[cfg(feature = "gpu")]
pub mod loss;
pub mod metrics;
#[cfg(feature = "gpu")]
pub mod trainer;
