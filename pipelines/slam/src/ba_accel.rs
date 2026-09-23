//! Optional accelerator for the incremental SfM's global bundle adjustment.
//!
//! visloc-slam stays GPU-free: a backend crate (e.g. `visloc-ba-gpu`)
//! implements [`GlobalBaAccelerator`] and registers it once with
//! [`set_global_ba_accelerator`]. The SfM then offers every unweighted,
//! free-rotation global BA to it; returning `None` falls back to the CPU
//! solver for that call.

use std::sync::OnceLock;

use crate::bundle::{BaConfig, BaError, BaResult, BundleAdjustment};

/// A replacement solver for global bundle adjustment.
pub trait GlobalBaAccelerator: Send + Sync {
    /// Optimize `ba` in place with `config`'s LM settings, or return `None`
    /// when the problem is outside what the accelerator supports.
    fn optimize(
        &self,
        ba: &mut BundleAdjustment,
        config: &BaConfig,
    ) -> Option<Result<BaResult, BaError>>;
}

static ACCELERATOR: OnceLock<Box<dyn GlobalBaAccelerator>> = OnceLock::new();

/// Register the process-wide accelerator. Returns `false` if one was
/// already registered (the first registration wins).
pub fn set_global_ba_accelerator(accelerator: Box<dyn GlobalBaAccelerator>) -> bool {
    ACCELERATOR.set(accelerator).is_ok()
}

pub(crate) fn global_ba_accelerator() -> Option<&'static dyn GlobalBaAccelerator> {
    ACCELERATOR.get().map(|a| a.as_ref())
}
