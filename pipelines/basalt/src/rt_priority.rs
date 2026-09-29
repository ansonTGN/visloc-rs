//! Best-effort OS thread priority hints for the online real-time VI-SLAM
//! demo (`examples/basalt_euroc_online_slam_demo.rs`).
//!
//! On a shared, busy machine (other processes competing for CPU: builds,
//! other sessions' work, etc.) the online demo's real-time budget is thin
//! (see `docs/vi_slam_global_consistency_plan.md`'s real-time sections): the
//! frontend/decode and estimator threads must win scheduling contention
//! against everything else on the box, while the mapper thread's periodic
//! background optimize pass -- which does not sit on the real-time critical
//! path and is already documented as timing-dependent -- must not compete
//! with them for the same cores. This module gives the demo two knobs for
//! that: [`set_current_thread_priority_above_normal_best_effort`] for the
//! former, [`set_current_thread_priority_below_normal_best_effort`] for the
//! latter.
//!
//! Both are OS scheduler *hints*, not algorithm inputs: every call discards
//! its `Result`, so a platform that does not support the requested priority,
//! or a process without the OS privilege to raise it (e.g. an unprivileged
//! Linux process asking to go above normal niceness), silently no-ops rather
//! than erroring or panicking. Neither function can change which bytes a VIO
//! or mapper pass computes -- only how promptly the OS schedules the thread
//! that computes them -- so using them must not (and, per the bit-identity
//! check recorded alongside this module's introduction, does not) change the
//! recovered VIO trajectory or `MargData`.
//!
//! Gated by the `basalt-rt-priority` Cargo feature (see `docs/feature_matrix.md`
//! and `tests/test_feature_matrix.py`): with the feature off, both functions
//! are compiled to no-ops so call sites never need their own `#[cfg(...)]`.
//! When compiling this feature in, remember it needs a Rust toolchain new
//! enough for whatever `thread-priority` version is pinned in
//! `pipelines/basalt/Cargo.toml` (pinned to `1.2.0` specifically to keep this
//! crate's MSRV 1.83, since newer `thread-priority` majors require 1.85+).

/// Cross-platform priority value handed to the `thread-priority` crate's
/// `ThreadPriority::Crossplatform`. On Windows this maps into
/// `THREAD_PRIORITY_ABOVE_NORMAL` (its `61..=79` bucket); on Unix it nudges
/// the calling thread's niceness down (higher priority) via `setpriority`,
/// which requires `CAP_SYS_NICE`/elevated privilege on most distros and
/// simply fails (ignored) without it.
#[cfg(feature = "basalt-rt-priority")]
const ABOVE_NORMAL_CROSSPLATFORM_VALUE: u8 = 75;

/// Cross-platform priority value mapping to Windows `THREAD_PRIORITY_BELOW_NORMAL`
/// (its `21..=39` bucket); on Unix it raises niceness (lower priority), which
/// unprivileged processes can normally do.
#[cfg(feature = "basalt-rt-priority")]
const BELOW_NORMAL_CROSSPLATFORM_VALUE: u8 = 30;

/// Ask the OS scheduler to favor the calling thread: intended for the online
/// demo's frontend/decode and estimator threads, which sit on the real-time
/// critical path. Best-effort -- see the module docs.
#[cfg(feature = "basalt-rt-priority")]
pub fn set_current_thread_priority_above_normal_best_effort() {
    use thread_priority::{set_current_thread_priority, ThreadPriority, ThreadPriorityValue};
    if let Ok(value) = ThreadPriorityValue::try_from(ABOVE_NORMAL_CROSSPLATFORM_VALUE) {
        let _ = set_current_thread_priority(ThreadPriority::Crossplatform(value));
    }
}

/// No-op build of [`set_current_thread_priority_above_normal_best_effort`]
/// when `basalt-rt-priority` is not compiled in.
#[cfg(not(feature = "basalt-rt-priority"))]
pub fn set_current_thread_priority_above_normal_best_effort() {}

/// Ask the OS scheduler to deprioritize the calling thread: intended for the
/// mapper thread (`mapper_online::run_mapper_thread`), whose periodic
/// background optimize pass must not steal cycles from the real-time
/// threads above. Best-effort -- see the module docs.
#[cfg(feature = "basalt-rt-priority")]
pub fn set_current_thread_priority_below_normal_best_effort() {
    use thread_priority::{set_current_thread_priority, ThreadPriority, ThreadPriorityValue};
    if let Ok(value) = ThreadPriorityValue::try_from(BELOW_NORMAL_CROSSPLATFORM_VALUE) {
        let _ = set_current_thread_priority(ThreadPriority::Crossplatform(value));
    }
}

/// No-op build of [`set_current_thread_priority_below_normal_best_effort`]
/// when `basalt-rt-priority` is not compiled in.
#[cfg(not(feature = "basalt-rt-priority"))]
pub fn set_current_thread_priority_below_normal_best_effort() {}

#[cfg(test)]
mod tests {
    use super::*;

    /// Both functions must be callable from any thread without panicking,
    /// with or without the feature compiled in -- the whole point is a safe
    /// fallback under contention or missing OS privilege.
    #[test]
    fn priority_hints_never_panic() {
        set_current_thread_priority_above_normal_best_effort();
        set_current_thread_priority_below_normal_best_effort();
        std::thread::spawn(|| {
            set_current_thread_priority_above_normal_best_effort();
            set_current_thread_priority_below_normal_best_effort();
        })
        .join()
        .expect("priority hints must not panic on a background thread");
    }
}
