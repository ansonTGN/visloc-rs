//! wgpu device/queue creation for the headless renderer.

use thiserror::Error;

/// Errors from GPU setup.
#[derive(Debug, Error)]
pub enum GpuError {
    #[error("no compatible GPU adapter was found")]
    NoAdapter,
    #[error("failed to create the GPU device: {0}")]
    Device(#[from] wgpu::RequestDeviceError),
}

/// A ready-to-use wgpu device and queue.
pub struct GpuContext {
    // Keep the instance alive for the lifetime of the device: some backends
    // release resources the device depends on when the instance is dropped.
    _instance: wgpu::Instance,
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
    pub adapter_info: wgpu::AdapterInfo,
    /// Adapter limits actually requested for the device (used to size buffers).
    pub limits: wgpu::Limits,
}

impl GpuContext {
    /// Create a high-performance adapter (a discrete GPU if present).
    ///
    /// On Windows the default backend set is restricted to DX12 there, because
    /// third-party Vulkan layers (Steam / Epic / Rockstar overlays) and hybrid
    /// Intel+NVIDIA setups are known to crash Vulkan/GL instance creation. Set
    /// `WGPU_BACKEND` or use [`GpuContext::with_backends`] to override.
    pub fn new() -> Result<Self, GpuError> {
        if let Ok(env) = std::env::var("WGPU_BACKEND") {
            let backends = match env.to_ascii_lowercase().as_str() {
                "dx12" => wgpu::Backends::DX12,
                "vulkan" => wgpu::Backends::VULKAN,
                "gl" => wgpu::Backends::GL,
                "metal" => wgpu::Backends::METAL,
                _ => wgpu::Backends::all(),
            };
            return Self::with_backends(backends);
        }
        #[cfg(target_os = "windows")]
        {
            Self::with_backends(wgpu::Backends::DX12)
        }
        #[cfg(not(target_os = "windows"))]
        {
            Self::with_backends(wgpu::Backends::PRIMARY)
        }
    }

    /// Create a context restricted to specific backends (useful in tests).
    pub fn with_backends(backends: wgpu::Backends) -> Result<Self, GpuError> {
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
            backends,
            ..wgpu::InstanceDescriptor::new_without_display_handle()
        });
        let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            force_fallback_adapter: false,
            compatible_surface: None,
            apply_limit_buckets: false,
        }))
        .map_err(|_| GpuError::NoAdapter)?;
        let info = adapter.get_info();
        // Request the adapter's real limits so large scenes are not capped by
        // the conservative WebGPU defaults (e.g. 256 MiB buffers).
        let limits = adapter.limits();
        let (device, queue) =
            pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
                label: Some("visloc-gsplat-render"),
                required_limits: limits.clone(),
                ..Default::default()
            }))?;
        Ok(Self {
            _instance: instance,
            device,
            queue,
            adapter_info: info,
            limits,
        })
    }
}

/// Try to create a GPU context, returning `None` when no adapter is available.
///
/// Tests use this so they skip gracefully on headless machines instead of
/// failing.
pub fn try_context() -> Option<GpuContext> {
    GpuContext::new().ok()
}
