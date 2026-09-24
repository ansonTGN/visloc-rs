//! wgpu bundle adjustment for visloc-rs SfM.
//!
//! `GpuBundleAdjuster` runs the same Levenberg-Marquardt loop as
//! visloc-slam's CPU `BundleAdjustment::optimize` for monocular pinhole
//! problems (same damping schedule, acceptance test, non-projectable gate
//! and convergence tests; costs evaluated on the CPU in f64), but builds and
//! solves each linear system on the GPU in f32: per-observation Jacobians
//! with the Huber IRLS weight, per-pose / per-landmark reductions, and an
//! implicit Schur complement solved by block-Jacobi PCG, then landmark
//! back-substitution. The f32 step is only a proposal; the f64 cost test
//! decides, so the result is validated like any LM step.
//!
//! Register it with `visloc_slam::set_ba_accelerator` to make the
//! incremental SfM use it for its global bundle adjustments (local windows
//! are opt-in, see `GpuBaSettings::local_windows`).

#[cfg(feature = "gpu")]
mod solver;

#[cfg(feature = "gpu")]
pub use solver::{GpuBaError, GpuBaSettings, GpuBundleAdjuster};
#[cfg(feature = "gpu")]
pub use visloc_gsplat_render::{try_context, GpuContext, GpuError};
