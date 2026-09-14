//! Faithful ("ベタ移植") port of COLMAP's rig-aware incremental-mapper data
//! model, milestone C1 of `docs/colmap_rig_mapper_port_plan.md`.
//!
//! This module mirrors COLMAP's own naming and file layout so future diffs
//! against upstream COLMAP stay structural (`docs/colmap_rig_mapper_port_plan.md`
//! §3.1). C1 covers only the *data model and inputs* — [`types`] (`Rig`,
//! `Frame`, `sensor_t`/`data_t`, `Frame::SetCamFromWorld`), [`reconstruction`]
//! (`Reconstruction`, `Image`, `Point3D`, COLMAP text export), and
//! [`database_cache`] (`DatabaseCache`, reusing the M2
//! [`visloc_vision::two_view::CorrespondenceGraph`] port verbatim). The
//! incremental registration loop itself (`IncrementalMapper`, Path A/B
//! registration, generalized pose solvers, bundle adjustment) is out of
//! scope here — see the Lead review's C2/C3 milestones in the port plan.
//!
//! Deliberately **separate from [`crate::rig_sfm`]**, not a patch to it: this
//! is the from-source COLMAP port, validated against COLMAP's own object
//! model in isolation before any decision is made about the relationship
//! between the two mappers (see the port plan's §3.1).

pub mod database_cache;
pub mod reconstruction;
pub mod types;

pub use database_cache::{DatabaseCache, DatabaseCacheError};
pub use reconstruction::{Camera, Image, Point2D, Point3D, Reconstruction, TrackElement};
pub use types::{
    CameraT, DataT, Frame, FrameT, ImageT, Point2DT, Point3DT, Rig, RigT, SensorT, SensorType,
};
