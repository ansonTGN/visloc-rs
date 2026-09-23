//! WGSL shader assembly.
//!
//! WGSL has no `#include`, so each compute pipeline gets its own module built
//! by concatenating the shared `common.wgsl` prelude with the kernel body.

/// Source of a compute kernel: the shared prelude plus one kernel file.
pub struct Kernel {
    pub name: &'static str,
    pub source: String,
}

const COMMON: &str = include_str!("shaders/common.wgsl");

fn build(name: &'static str, body: &str) -> Kernel {
    Kernel {
        name,
        source: format!("{COMMON}\n{body}"),
    }
}

/// Uses subgroup ops: naga (wgpu 30) accepts them without an `enable`
/// directive when the device has `Features::SUBGROUP`.
pub fn rasterize_backward() -> Kernel {
    build(
        "rasterize_backward",
        include_str!("shaders/rasterize_backward.wgsl"),
    )
}

pub fn project_backward() -> Kernel {
    build(
        "project_backward",
        include_str!("shaders/project_backward.wgsl"),
    )
}

pub fn grad_reduce() -> Kernel {
    build("grad_reduce", include_str!("shaders/grad_reduce.wgsl"))
}

pub fn project_forward() -> Kernel {
    build(
        "project_forward",
        include_str!("shaders/project_forward.wgsl"),
    )
}

pub fn project_visible() -> Kernel {
    build(
        "project_visible",
        include_str!("shaders/project_visible.wgsl"),
    )
}

pub fn map_gaussians() -> Kernel {
    build("map_gaussians", include_str!("shaders/map_gaussians.wgsl"))
}

pub fn gather_compact() -> Kernel {
    build(
        "gather_compact",
        include_str!("shaders/gather_compact.wgsl"),
    )
}

pub fn tile_offsets() -> Kernel {
    build("tile_offsets", include_str!("shaders/tile_offsets.wgsl"))
}

pub fn rasterize() -> Kernel {
    build("rasterize", include_str!("shaders/rasterize.wgsl"))
}

pub fn radix() -> Kernel {
    build("radix", include_str!("shaders/radix.wgsl"))
}

pub fn scan() -> Kernel {
    build("scan", include_str!("shaders/scan.wgsl"))
}
