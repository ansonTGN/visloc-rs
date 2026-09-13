//! Numeric ownership for the VIO compatibility core.
//!
//! Basalt's public feed is double precision (`ImuData<double>`, calibration
//! and trajectory output), but the default estimator instantiated by
//! `VioEstimatorFactory` is `SqrtKeypointVioEstimator<float>`.  Keeping the
//! choice explicit prevents a final `H/b` cast from being mistaken for the
//! upstream scalar boundary: the selected mode owns state prediction,
//! landmark/factor arithmetic, QR, solve, and LM cost evaluation.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScalarMode {
    /// The pinned Basalt default: `SqrtKeypointVioEstimator<float>`.
    UpstreamF32,
    /// Extended Rust/API mode retained for numerical experiments and tests.
    ExtendedF64,
}

impl Default for ScalarMode {
    fn default() -> Self {
        Self::UpstreamF32
    }
}
