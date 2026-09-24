//! Optional accelerator for the incremental SfM's bundle adjustments.
//!
//! visloc-slam stays GPU-free: a backend crate (e.g. `visloc-ba-gpu`)
//! implements [`BaAccelerator`] and registers it once with
//! [`set_ba_accelerator`]. The SfM then offers every unweighted,
//! free-rotation global BA and every local BA to it; returning `None` falls
//! back to the CPU solver for that call.

use std::sync::OnceLock;

use crate::bundle::{BaConfig, BaError, BaResult, BundleAdjustment};

/// Which SfM solve is being offered to the accelerator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BaScope {
    /// Whole-model refinement (periodic, growth and final global BA).
    Global,
    /// The per-registration window around a newly added image.
    Local,
}

/// A replacement solver for SfM bundle adjustment.
pub trait BaAccelerator: Send + Sync {
    /// Optimize `ba` in place with `config`'s LM settings, or return `None`
    /// when the problem is outside what the accelerator supports.
    fn optimize(
        &self,
        ba: &mut BundleAdjustment,
        config: &BaConfig,
        scope: BaScope,
    ) -> Option<Result<BaResult, BaError>>;
}

static ACCELERATOR: OnceLock<Box<dyn BaAccelerator>> = OnceLock::new();

/// Register the process-wide accelerator. Returns `false` if one was
/// already registered (the first registration wins).
pub fn set_ba_accelerator(accelerator: Box<dyn BaAccelerator>) -> bool {
    ACCELERATOR.set(accelerator).is_ok()
}

pub(crate) fn ba_accelerator() -> Option<&'static dyn BaAccelerator> {
    ACCELERATOR.get().map(|a| a.as_ref())
}
