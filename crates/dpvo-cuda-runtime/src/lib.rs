//! Runtime-loaded native CUDA DPVO correlation backend.
//!
//! The DLL is built explicitly with `scripts/build_dpvo_cuda_kernels.ps1`;
//! it is not compiled by Cargo, so ordinary builds and docs.rs do not require
//! a CUDA toolkit. The SHA-256-bound V4 runner owns the resulting artifact.

#![deny(unsafe_op_in_unsafe_fn)]

use std::ffi::{c_char, c_float, c_int, c_void, CStr};
use std::fmt;
use std::path::{Path, PathBuf};
use std::ptr::NonNull;

use libloading::Library;
use ndarray::{Array2, Array3, ArrayView4};

const FNET_DIM: usize = 128;
const PATCH: usize = 3;
const CORR_DIM: usize = 882;

type AbiVersionFn = unsafe extern "C" fn() -> u32;
type CreateFn = unsafe extern "C" fn() -> *mut c_void;
type DestroyFn = unsafe extern "C" fn(*mut c_void);
type LastErrorFn = unsafe extern "C" fn(*mut c_void) -> *const c_char;
type RunFn = unsafe extern "C" fn(
    *mut c_void,
    *const c_float,
    *const *const c_float,
    *const *const c_float,
    *const c_float,
    *const i32,
    *const u64,
    *mut c_float,
    c_int,
    c_int,
    c_int,
    c_int,
    c_int,
    c_int,
    c_int,
    c_int,
    c_int,
    c_int,
    *mut c_float,
) -> c_int;

#[derive(Debug)]
pub enum NativeCudaCorrelationError {
    Load { path: PathBuf, message: String },
    AbiVersion(u32),
    NullContext,
    Shape(String),
    Runtime { code: i32, message: String },
}

impl fmt::Display for NativeCudaCorrelationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Load { path, message } => {
                write!(
                    f,
                    "load native CUDA correlation {}: {message}",
                    path.display()
                )
            }
            Self::AbiVersion(version) => {
                write!(f, "native CUDA correlation ABI {version}, expected 3")
            }
            Self::NullContext => write!(f, "native CUDA correlation returned a null context"),
            Self::Shape(message) => write!(f, "native CUDA correlation shape: {message}"),
            Self::Runtime { code, message } => {
                write!(f, "native CUDA correlation failed ({code}): {message}")
            }
        }
    }
}

impl std::error::Error for NativeCudaCorrelationError {}

pub struct NativeCudaCorrelation {
    _library: Library,
    context: NonNull<c_void>,
    destroy: DestroyFn,
    last_error: LastErrorFn,
    run: RunFn,
    resident_map_version: Option<u64>,
}

impl fmt::Debug for NativeCudaCorrelation {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("NativeCudaCorrelation")
            .finish_non_exhaustive()
    }
}

impl NativeCudaCorrelation {
    pub const ABI_VERSION: u32 = 3;

    pub fn load(path: impl AsRef<Path>) -> Result<Self, NativeCudaCorrelationError> {
        let path = path.as_ref();
        let load_error = |message: String| NativeCudaCorrelationError::Load {
            path: path.to_path_buf(),
            message,
        };
        // SAFETY: every symbol is checked immediately against the versioned
        // C ABI declared by native/dpvo_cuda/dpvo_corr.cu. The Library is
        // retained for at least as long as every copied function pointer.
        let library = unsafe { Library::new(path) }.map_err(|e| load_error(e.to_string()))?;
        // SAFETY: every symbol is requested under the exact versioned name
        // declared by `native/dpvo_cuda/dpvo_corr.cu`, and the copied function
        // pointers are only ever called while `library` is still owned by the
        // struct (`_library`), so the resolved code stays mapped.
        let (abi_version, create, destroy, last_error, run): (
            AbiVersionFn,
            CreateFn,
            DestroyFn,
            LastErrorFn,
            RunFn,
        ) = unsafe {
            (
                *library
                    .get(b"visloc_dpvo_corr_abi_version\0")
                    .map_err(|e| load_error(e.to_string()))?,
                *library
                    .get(b"visloc_dpvo_corr_create\0")
                    .map_err(|e| load_error(e.to_string()))?,
                *library
                    .get(b"visloc_dpvo_corr_destroy\0")
                    .map_err(|e| load_error(e.to_string()))?,
                *library
                    .get(b"visloc_dpvo_corr_last_error\0")
                    .map_err(|e| load_error(e.to_string()))?,
                *library
                    .get(b"visloc_dpvo_corr_run\0")
                    .map_err(|e| load_error(e.to_string()))?,
            )
        };
        // SAFETY: `abi_version` was resolved from the ABI-checked library and
        // takes no arguments, so the call cannot observe invalid state.
        let version = unsafe { abi_version() };
        if version != Self::ABI_VERSION {
            return Err(NativeCudaCorrelationError::AbiVersion(version));
        }
        // SAFETY: `create` was resolved from the ABI-checked library. A null
        // return is rejected immediately, and the non-null context is owned by
        // `self` and released exactly once through the matching `destroy`.
        let context =
            NonNull::new(unsafe { create() }).ok_or(NativeCudaCorrelationError::NullContext)?;
        Ok(Self {
            _library: library,
            context,
            destroy,
            last_error,
            run,
            resident_map_version: None,
        })
    }

    pub const fn abi_version(&self) -> u32 {
        Self::ABI_VERSION
    }

    pub fn run(
        &mut self,
        anchors: ArrayView4<'_, f32>,
        level0_frames: &[&Array3<f32>],
        level1_frames: &[&Array3<f32>],
        coords: ArrayView4<'_, f32>,
        targets: &[i32],
    ) -> Result<(Array2<f32>, f32), NativeCudaCorrelationError> {
        self.run_impl(
            anchors,
            level0_frames,
            level1_frames,
            coords,
            targets,
            None,
            None,
        )
    }

    /// Run while retaining feature maps on the device across calls carrying
    /// the same immutable map-set version. Callers must advance `map_version`
    /// whenever a frame is added, removed, reordered, or modified.
    pub fn run_cached(
        &mut self,
        anchors: ArrayView4<'_, f32>,
        level0_frames: &[&Array3<f32>],
        level1_frames: &[&Array3<f32>],
        coords: ArrayView4<'_, f32>,
        targets: &[i32],
        map_version: u64,
    ) -> Result<(Array2<f32>, f32), NativeCudaCorrelationError> {
        self.run_impl(
            anchors,
            level0_frames,
            level1_frames,
            coords,
            targets,
            Some(map_version),
            None,
        )
    }

    /// Retain immutable frame pyramids in stable device slots identified by
    /// caller-owned IDs. Reordering or removing IDs does not re-upload the
    /// retained maps; a previously unseen ID uploads exactly that frame.
    pub fn run_stable(
        &mut self,
        anchors: ArrayView4<'_, f32>,
        level0_frames: &[&Array3<f32>],
        level1_frames: &[&Array3<f32>],
        coords: ArrayView4<'_, f32>,
        targets: &[i32],
        frame_ids: &[u64],
    ) -> Result<(Array2<f32>, f32), NativeCudaCorrelationError> {
        self.run_impl(
            anchors,
            level0_frames,
            level1_frames,
            coords,
            targets,
            None,
            Some(frame_ids),
        )
    }

    // This private dispatcher mirrors the versioned native C ABI's flat
    // argument list. Keep the ABI-facing call site explicit rather than
    // changing the public Rust API or native symbol contract merely to satisfy
    // a style threshold.
    #[allow(clippy::too_many_arguments)]
    fn run_impl(
        &mut self,
        anchors: ArrayView4<'_, f32>,
        level0_frames: &[&Array3<f32>],
        level1_frames: &[&Array3<f32>],
        coords: ArrayView4<'_, f32>,
        targets: &[i32],
        map_version: Option<u64>,
        frame_ids: Option<&[u64]>,
    ) -> Result<(Array2<f32>, f32), NativeCudaCorrelationError> {
        let (edges, channels, patch_y, patch_x) = anchors.dim();
        if channels != FNET_DIM || patch_y != PATCH || patch_x != PATCH {
            return Err(NativeCudaCorrelationError::Shape(format!(
                "anchors {:?}, expected (E,{FNET_DIM},{PATCH},{PATCH})",
                anchors.dim()
            )));
        }
        if coords.dim() != (edges, PATCH, PATCH, 2) || targets.len() != edges {
            return Err(NativeCudaCorrelationError::Shape(format!(
                "coords {:?}, targets {}, edges {edges}",
                coords.dim(),
                targets.len()
            )));
        }
        if level0_frames.is_empty() || level0_frames.len() != level1_frames.len() {
            return Err(NativeCudaCorrelationError::Shape(
                "pyramid frame lists are empty or differ in length".into(),
            ));
        }
        if frame_ids.is_some_and(|ids| ids.len() != level0_frames.len()) {
            return Err(NativeCudaCorrelationError::Shape(format!(
                "frame IDs {}, pyramid frames {}",
                frame_ids.map_or(0, <[u64]>::len),
                level0_frames.len()
            )));
        }
        if let Some((edge, target)) = targets
            .iter()
            .copied()
            .enumerate()
            .find(|(_, target)| *target < 0 || (*target as usize) >= level0_frames.len())
        {
            return Err(NativeCudaCorrelationError::Shape(format!(
                "target {target} at edge {edge} is outside 0..{}",
                level0_frames.len()
            )));
        }
        let (_, height0, width0) = level0_frames[0].dim();
        let (_, height1, width1) = level1_frames[0].dim();
        for (index, (level0, level1)) in level0_frames.iter().zip(level1_frames.iter()).enumerate()
        {
            if level0.dim() != (FNET_DIM, height0, width0)
                || level1.dim() != (FNET_DIM, height1, width1)
            {
                return Err(NativeCudaCorrelationError::Shape(format!(
                    "pyramid frame {index} has inconsistent dimensions"
                )));
            }
        }
        let anchors = anchors.as_slice().ok_or_else(|| {
            NativeCudaCorrelationError::Shape("anchors are not contiguous".into())
        })?;
        let coords = coords
            .as_slice()
            .ok_or_else(|| NativeCudaCorrelationError::Shape("coords are not contiguous".into()))?;
        let level0_pointers: Result<Vec<_>, _> = level0_frames
            .iter()
            .map(|frame| {
                frame.as_slice().map(|slice| slice.as_ptr()).ok_or_else(|| {
                    NativeCudaCorrelationError::Shape("level0 is not contiguous".into())
                })
            })
            .collect();
        let level1_pointers: Result<Vec<_>, _> = level1_frames
            .iter()
            .map(|frame| {
                frame.as_slice().map(|slice| slice.as_ptr()).ok_or_else(|| {
                    NativeCudaCorrelationError::Shape("level1 is not contiguous".into())
                })
            })
            .collect();
        let level0_pointers = level0_pointers?;
        let level1_pointers = level1_pointers?;
        let checked_c_int = |name: &str, value: usize| {
            c_int::try_from(value).map_err(|_| {
                NativeCudaCorrelationError::Shape(format!(
                    "{name}={value} exceeds the native ABI integer range"
                ))
            })
        };
        let edges_c = checked_c_int("edges", edges)?;
        let frames_c = checked_c_int("frames", level0_frames.len())?;
        let height0_c = checked_c_int("height0", height0)?;
        let width0_c = checked_c_int("width0", width0)?;
        let height1_c = checked_c_int("height1", height1)?;
        let width1_c = checked_c_int("width1", width1)?;
        let mut output = Array2::<f32>::zeros((edges, CORR_DIM));
        let mut device_elapsed_ms = 0.0_f32;
        let cache_mode = if frame_ids.is_some() {
            2
        } else if map_version.is_some_and(|version| self.resident_map_version == Some(version)) {
            1
        } else {
            0
        };
        let frame_ids_pointer = frame_ids.map_or(std::ptr::null(), <[u64]>::as_ptr);
        // SAFETY: every pointer argument is derived from a live, contiguous
        // buffer whose shape and length were validated above; the output is an
        // `edges * CORR_DIM` contiguous `f32` buffer, the target and frame-ID
        // arrays match `edges` and `frames`, and each dimension was checked to
        // fit `c_int`. The native side only reads these inputs and writes
        // `output` and `device_elapsed_ms`.
        let code = unsafe {
            (self.run)(
                self.context.as_ptr(),
                anchors.as_ptr(),
                level0_pointers.as_ptr(),
                level1_pointers.as_ptr(),
                coords.as_ptr(),
                targets.as_ptr(),
                frame_ids_pointer,
                output
                    .as_slice_mut()
                    .expect("owned Array2 is contiguous")
                    .as_mut_ptr(),
                edges_c,
                frames_c,
                FNET_DIM as c_int,
                PATCH as c_int,
                height0_c,
                width0_c,
                height1_c,
                width1_c,
                3,
                cache_mode,
                &mut device_elapsed_ms,
            )
        };
        if code != 0 {
            // SAFETY: `last_error` was resolved from the ABI-checked library
            // and is passed the same live context as the failed call.
            let pointer = unsafe { (self.last_error)(self.context.as_ptr()) };
            let message = if pointer.is_null() {
                "no native error string".to_string()
            } else {
                // SAFETY: the native contract returns either null or a
                // NUL-terminated string that is valid until the next native
                // call. It is copied into an owned string immediately and never
                // retained past this scope.
                unsafe { CStr::from_ptr(pointer) }
                    .to_string_lossy()
                    .into_owned()
            };
            return Err(NativeCudaCorrelationError::Runtime { code, message });
        }
        self.resident_map_version = if frame_ids.is_some() {
            None
        } else {
            map_version
        };
        Ok((output, device_elapsed_ms))
    }
}

impl Drop for NativeCudaCorrelation {
    fn drop(&mut self) {
        // SAFETY: `context` is the non-null pointer returned by `create` and
        // has not been destroyed yet; it is released exactly once here, after
        // which no other method can observe it.
        unsafe { (self.destroy)(self.context.as_ptr()) };
    }
}

// ---------------------------------------------------------------------------
// Batched descriptor top-2 search (`native/descriptor_gemm/descriptor_gemm.cu`)
// ---------------------------------------------------------------------------

type DescriptorGemmAbiFn = unsafe extern "C" fn() -> u32;
type DescriptorGemmCreateFn = unsafe extern "C" fn() -> *mut c_void;
type DescriptorGemmDestroyFn = unsafe extern "C" fn(*mut c_void);
type DescriptorGemmLastErrorFn = unsafe extern "C" fn(*mut c_void) -> *const c_char;
type DescriptorGemmRunFn = unsafe extern "C" fn(
    *mut c_void,
    *const c_float,
    c_int,
    *const c_float,
    c_int,
    c_int,
    *mut c_float,
) -> c_int;

#[derive(Debug)]
pub enum NativeDescriptorGemmError {
    Load { path: PathBuf, message: String },
    AbiVersion(u32),
    NullContext,
    Shape(String),
    Runtime { code: i32, message: String },
}

impl fmt::Display for NativeDescriptorGemmError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Load { path, message } => {
                write!(
                    f,
                    "load native descriptor GEMM {}: {message}",
                    path.display()
                )
            }
            Self::AbiVersion(version) => {
                write!(f, "native descriptor GEMM ABI {version}, expected 1")
            }
            Self::NullContext => write!(f, "native descriptor GEMM returned a null context"),
            Self::Shape(message) => write!(f, "native descriptor GEMM shape: {message}"),
            Self::Runtime { code, message } => {
                write!(f, "native descriptor GEMM failed ({code}): {message}")
            }
        }
    }
}

impl std::error::Error for NativeDescriptorGemmError {}

/// One query's nearest and second-nearest train rows, as returned by
/// [`NativeDescriptorGemm::run`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DescriptorTop2 {
    pub best_distance: f32,
    pub best_index: usize,
    /// `None` when the train set has a single row.
    pub second_distance: Option<f32>,
    pub second_index: Option<usize>,
}

/// Runtime-loaded native descriptor top-2 search.
///
/// Mirrors [`NativeCudaCorrelation`]: the shared library is loaded lazily from
/// an explicit path, every symbol is checked against the versioned C ABI, and
/// the context is released exactly once on drop.
pub struct NativeDescriptorGemm {
    _library: Library,
    context: NonNull<c_void>,
    destroy: DescriptorGemmDestroyFn,
    last_error: DescriptorGemmLastErrorFn,
    run: DescriptorGemmRunFn,
}

impl fmt::Debug for NativeDescriptorGemm {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("NativeDescriptorGemm")
            .field("context", &self.context)
            .finish()
    }
}

impl NativeDescriptorGemm {
    pub const ABI_VERSION: u32 = 1;

    pub fn load(path: impl AsRef<Path>) -> Result<Self, NativeDescriptorGemmError> {
        let path = path.as_ref();
        let load_error = |message: String| NativeDescriptorGemmError::Load {
            path: path.to_path_buf(),
            message,
        };
        // SAFETY: every symbol is checked immediately against the versioned C
        // ABI declared by native/descriptor_gemm/descriptor_gemm.cu. The
        // Library is retained for at least as long as every copied pointer.
        let library = unsafe { Library::new(path) }.map_err(|e| load_error(e.to_string()))?;
        // SAFETY: symbols are requested under the exact versioned names
        // declared by the native source, and the copied function pointers are
        // only called while `library` is still owned by the struct.
        let (abi_version, create, destroy, last_error, run): (
            DescriptorGemmAbiFn,
            DescriptorGemmCreateFn,
            DescriptorGemmDestroyFn,
            DescriptorGemmLastErrorFn,
            DescriptorGemmRunFn,
        ) = unsafe {
            (
                *library
                    .get(b"visloc_descriptor_gemm_abi_version\0")
                    .map_err(|e| load_error(e.to_string()))?,
                *library
                    .get(b"visloc_descriptor_gemm_create\0")
                    .map_err(|e| load_error(e.to_string()))?,
                *library
                    .get(b"visloc_descriptor_gemm_destroy\0")
                    .map_err(|e| load_error(e.to_string()))?,
                *library
                    .get(b"visloc_descriptor_gemm_last_error\0")
                    .map_err(|e| load_error(e.to_string()))?,
                *library
                    .get(b"visloc_descriptor_gemm_run\0")
                    .map_err(|e| load_error(e.to_string()))?,
            )
        };
        // SAFETY: `abi_version` takes no arguments and cannot observe state.
        let version = unsafe { abi_version() };
        if version != Self::ABI_VERSION {
            return Err(NativeDescriptorGemmError::AbiVersion(version));
        }
        // SAFETY: `create` is from the ABI-checked library; a null return is
        // rejected and the non-null context is owned by `self`.
        let context =
            NonNull::new(unsafe { create() }).ok_or(NativeDescriptorGemmError::NullContext)?;
        Ok(Self {
            _library: library,
            context,
            destroy,
            last_error,
            run,
        })
    }

    pub const fn abi_version(&self) -> u32 {
        Self::ABI_VERSION
    }

    /// Exact nearest + second-nearest train row per query row.
    ///
    /// `query` is `n_query x dim` row-major and `train` is `n_train x dim`
    /// row-major. Distances are Euclidean; `second_*` is `None` when the train
    /// set has fewer than two rows. Ties break toward the lower train index.
    pub fn run(
        &mut self,
        query: &[f32],
        n_query: usize,
        train: &[f32],
        n_train: usize,
        dim: usize,
    ) -> Result<Vec<DescriptorTop2>, NativeDescriptorGemmError> {
        if dim == 0 {
            return Err(NativeDescriptorGemmError::Shape("dim is zero".to_string()));
        }
        if query.len() != n_query * dim || train.len() != n_train * dim {
            return Err(NativeDescriptorGemmError::Shape(format!(
                "query len {} != {n_query}x{dim} or train len {} != {n_train}x{dim}",
                query.len(),
                train.len()
            )));
        }
        if n_query == 0 || n_train == 0 {
            return Ok(Vec::new());
        }
        let Ok(n_query_i32) = c_int::try_from(n_query) else {
            return Err(NativeDescriptorGemmError::Shape(
                "n_query exceeds i32".to_string(),
            ));
        };
        let Ok(n_train_i32) = c_int::try_from(n_train) else {
            return Err(NativeDescriptorGemmError::Shape(
                "n_train exceeds i32".to_string(),
            ));
        };
        let Ok(dim_i32) = c_int::try_from(dim) else {
            return Err(NativeDescriptorGemmError::Shape(
                "dim exceeds i32".to_string(),
            ));
        };

        let mut output = vec![0.0_f32; n_query * 4];
        // SAFETY: `self.context` is a valid non-null context owned by `self`;
        // the buffers outlive the call and their lengths were validated above
        // against the declared shape, which is the native function's contract.
        let code = unsafe {
            (self.run)(
                self.context.as_ptr(),
                query.as_ptr(),
                n_query_i32,
                train.as_ptr(),
                n_train_i32,
                dim_i32,
                output.as_mut_ptr(),
            )
        };
        if code != 0 {
            // SAFETY: `last_error` returns a pointer to a NUL-terminated buffer
            // owned by the context, which is alive for the duration of the call.
            let message = unsafe {
                let pointer = (self.last_error)(self.context.as_ptr());
                if pointer.is_null() {
                    "unknown error".to_string()
                } else {
                    std::ffi::CStr::from_ptr(pointer)
                        .to_string_lossy()
                        .into_owned()
                }
            };
            return Err(NativeDescriptorGemmError::Runtime { code, message });
        }

        let mut results = Vec::with_capacity(n_query);
        for chunk in output.chunks_exact(4) {
            let best_index = chunk[1];
            let second_index = chunk[3];
            results.push(DescriptorTop2 {
                best_distance: chunk[0],
                best_index: best_index as usize,
                second_distance: (second_index >= 0.0).then_some(chunk[2]),
                second_index: (second_index >= 0.0).then_some(second_index as usize),
            });
        }
        Ok(results)
    }
}

impl Drop for NativeDescriptorGemm {
    fn drop(&mut self) {
        // SAFETY: `context` is the non-null pointer returned by `create` and is
        // released exactly once here, after which no method can observe it.
        unsafe { (self.destroy)(self.context.as_ptr()) };
    }
}
