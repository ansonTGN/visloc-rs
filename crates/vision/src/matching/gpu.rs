//! GPU descriptor matcher backed by the native top-2 kernel.
//!
//! The online SLAM tracker matches every query descriptor against the whole
//! map. The CPU kernel is a `nalgebra` GEMM (`O(N_query x N_train x dim)`), and
//! the front-end breakdown shows it is the binding frame cost. This matcher
//! offloads the same exact nearest + second-nearest search to a CUDA kernel
//! (`native/descriptor_gemm/descriptor_gemm.cu`) and copies back only the
//! per-query top-2, so the `N_query x N_train` distance matrix never crosses
//! PCIe.
//!
//! The Rust wrapper is loaded at runtime from an explicit DLL path (the same
//! boundary style as [`crate::dpvo::native_cuda_correlation`]); callers decide
//! whether a GPU is available and pass the library path. With no path (or a
//! failed load) the matcher transparently falls back to
//! [`BruteForceMatcher`], so it is always safe to construct.

use std::path::Path;

use visloc_dpvo_cuda_runtime::{NativeDescriptorGemm, NativeDescriptorGemmError};

use super::{BruteForceMatcher, DescriptorMatch, Matcher};

/// Executes the exact top-2 descriptor search on a device.
///
/// This is the seam the tests substitute; the production implementation is
/// [`NativeDescriptorGemm`]. Public so callers can supply their own backend
/// through [`GpuDescriptorMatcher::with_backend`].
pub trait Top2Backend {
    fn top2(
        &mut self,
        query: &[f32],
        n_query: usize,
        train: &[f32],
        n_train: usize,
        dim: usize,
    ) -> Result<Vec<visloc_dpvo_cuda_runtime::DescriptorTop2>, NativeDescriptorGemmError>;
}

impl Top2Backend for NativeDescriptorGemm {
    fn top2(
        &mut self,
        query: &[f32],
        n_query: usize,
        train: &[f32],
        n_train: usize,
        dim: usize,
    ) -> Result<Vec<visloc_dpvo_cuda_runtime::DescriptorTop2>, NativeDescriptorGemmError> {
        self.run(query, n_query, train, n_train, dim)
    }
}

/// Exact descriptor matcher with an optional GPU backend.
///
/// Semantics are identical to [`BruteForceMatcher`] — nearest neighbour, Lowe
/// ratio on the second-best, optional cross-check — because the kernel returns
/// exactly the two quantities the ratio gate needs. When the GPU backend is
/// absent or a call fails, the matcher falls back to the CPU path for that
/// call.
pub struct GpuDescriptorMatcher<B = NativeDescriptorGemm> {
    inner: BruteForceMatcher,
    backend: Option<B>,
    /// Set once a GPU call has failed, so subsequent calls skip the GPU rather
    /// than paying a failed launch per frame.
    disabled: bool,
}

impl GpuDescriptorMatcher<NativeDescriptorGemm> {
    /// Load the native kernel from `library_path`.
    ///
    /// Returns `None` (a CPU-only matcher) when the library cannot be loaded,
    /// so a caller can attempt the GPU and degrade without a hard error.
    pub fn load(inner: BruteForceMatcher, library_path: impl AsRef<Path>) -> Self {
        let backend = NativeDescriptorGemm::load(library_path).ok();
        let disabled = backend.is_none();
        Self {
            inner,
            backend,
            disabled,
        }
    }
}

impl<B> GpuDescriptorMatcher<B>
where
    B: Top2Backend,
{
    /// Construct with an explicit backend (or `None` for CPU-only). Useful for
    /// tests that inject a fake backend.
    pub const fn with_backend(inner: BruteForceMatcher, backend: Option<B>) -> Self {
        let disabled = backend.is_none();
        Self {
            inner,
            backend,
            disabled,
        }
    }

    /// Whether a working GPU backend is currently active.
    pub fn is_gpu_active(&self) -> bool {
        self.backend.is_some() && !self.disabled
    }

    /// Borrow the inner brute-force policy.
    pub const fn inner(&self) -> &BruteForceMatcher {
        &self.inner
    }

    fn match_impl(&mut self, query: &[Vec<f32>], train: &[Vec<f32>]) -> Vec<DescriptorMatch> {
        if self.disabled {
            return self.inner.match_descriptors(query, train);
        }
        let Some(backend) = self.backend.as_mut() else {
            return self.inner.match_descriptors(query, train);
        };
        // The GPU kernel assumes a single uniform dimension, as does the CPU
        // GEMM path; anything else goes to the exact element-wise reference.
        let Some(dim) =
            uniform_dimension(query).filter(|&dim| dim == uniform_dimension(train).unwrap_or(0))
        else {
            return self.inner.match_descriptors(query, train);
        };
        if query.is_empty() || train.is_empty() {
            return Vec::new();
        }

        let flat_query = flatten(query, dim);
        let flat_train = flatten(train, dim);
        let top2 = match backend.top2(&flat_query, query.len(), &flat_train, train.len(), dim) {
            Ok(top2) => top2,
            Err(_) => {
                // One failure disables the GPU for the rest of the run and this
                // call is served from the CPU so tracking never stalls.
                self.disabled = true;
                return self.inner.match_descriptors(query, train);
            }
        };

        let mut matches = Vec::new();
        for (query_index, top) in top2.iter().enumerate() {
            if top.best_index >= train.len() {
                continue;
            }
            let distance = top.best_distance;
            let second_distance = top.second_distance;
            if let Some(ratio) = self.inner.ratio {
                if let Some(second_distance) = second_distance {
                    if distance >= ratio * second_distance {
                        continue;
                    }
                }
            }
            matches.push(DescriptorMatch {
                query_index,
                train_index: top.best_index,
                distance,
                second_best_distance: second_distance,
                ratio: second_distance.map(|second| distance / second),
                confidence: None,
            });
        }
        matches
    }
}

impl<B> Matcher for GpuDescriptorMatcher<B>
where
    B: Top2Backend,
{
    fn match_descriptors(&self, query: &[Vec<f32>], train: &[Vec<f32>]) -> Vec<DescriptorMatch> {
        // `match_impl` needs `&mut` for the persistent device context; the
        // `Matcher` trait is `&self`. Callers that need the GPU path hold a
        // mutable matcher and call `match_descriptors_mut`; this immutable
        // entry point stays exact on the CPU.
        self.inner.match_descriptors(query, train)
    }
}

impl<B> GpuDescriptorMatcher<B>
where
    B: Top2Backend,
{
    /// GPU-accelerated matching entry point (needs `&mut` for the device
    /// context). Falls back to the CPU path internally when the GPU is absent
    /// or has failed.
    pub fn match_descriptors_mut(
        &mut self,
        query: &[Vec<f32>],
        train: &[Vec<f32>],
    ) -> Vec<DescriptorMatch> {
        self.match_impl(query, train)
    }
}

fn uniform_dimension(descriptors: &[Vec<f32>]) -> Option<usize> {
    let dim = descriptors.first().map(Vec::len)?;
    if dim == 0 || descriptors.iter().any(|descriptor| descriptor.len() != dim) {
        return None;
    }
    Some(dim)
}

fn flatten(descriptors: &[Vec<f32>], dim: usize) -> Vec<f32> {
    let mut flat = Vec::with_capacity(descriptors.len() * dim);
    for descriptor in descriptors {
        flat.extend_from_slice(descriptor);
    }
    flat
}

#[cfg(test)]
mod tests {
    use super::*;
    use visloc_dpvo_cuda_runtime::{DescriptorTop2, NativeDescriptorGemmError};

    /// A deterministic CPU "device" that computes the exact top-2, so the
    /// matcher's ratio/cross-check wiring is testable without a GPU.
    struct FakeBackend {
        calls: usize,
        fail: bool,
    }

    impl Top2Backend for FakeBackend {
        fn top2(
            &mut self,
            query: &[f32],
            n_query: usize,
            train: &[f32],
            n_train: usize,
            dim: usize,
        ) -> Result<Vec<DescriptorTop2>, NativeDescriptorGemmError> {
            self.calls += 1;
            if self.fail {
                return Err(NativeDescriptorGemmError::Runtime {
                    code: 7,
                    message: "injected failure".to_string(),
                });
            }
            let mut out = Vec::with_capacity(n_query);
            for q in 0..n_query {
                let mut best = (0usize, f32::INFINITY);
                let mut second = (0usize, f32::INFINITY);
                for t in 0..n_train {
                    let mut sum = 0.0_f32;
                    for k in 0..dim {
                        let diff = query[q * dim + k] - train[t * dim + k];
                        sum += diff * diff;
                    }
                    if sum < best.1 {
                        second = best;
                        best = (t, sum);
                    } else if sum < second.1 {
                        second = (t, sum);
                    }
                }
                out.push(DescriptorTop2 {
                    best_distance: best.1.sqrt(),
                    best_index: best.0,
                    second_distance: (n_train > 1).then(|| second.1.sqrt()),
                    second_index: (n_train > 1).then_some(second.0),
                });
            }
            Ok(out)
        }
    }

    fn descriptors() -> (Vec<Vec<f32>>, Vec<Vec<f32>>) {
        let query = vec![
            vec![1.0, 0.0, 0.0],
            vec![0.0, 1.0, 0.0],
            vec![0.0, 0.0, 1.0],
        ];
        let train = vec![
            vec![1.0, 0.0, 0.0],
            vec![0.0, 1.0, 0.0],
            vec![0.9, 0.4, 0.0], // a decoy near the second query
            vec![0.0, 0.0, 1.0],
        ];
        (query, train)
    }

    #[test]
    fn gpu_path_matches_brute_force() {
        let (query, train) = descriptors();
        let inner = BruteForceMatcher { ratio: Some(0.8) };
        let expected = inner.match_descriptors(&query, &train);

        let mut matcher = GpuDescriptorMatcher::with_backend(
            inner,
            Some(FakeBackend {
                calls: 0,
                fail: false,
            }),
        );
        assert!(matcher.is_gpu_active());
        let actual = matcher.match_descriptors_mut(&query, &train);
        assert_eq!(expected, actual);
    }

    #[test]
    fn absence_of_backend_uses_cpu() {
        let (query, train) = descriptors();
        let inner = BruteForceMatcher { ratio: Some(0.8) };
        let mut matcher = GpuDescriptorMatcher::<FakeBackend>::with_backend(inner, None);
        assert!(!matcher.is_gpu_active());
        assert_eq!(
            matcher.match_descriptors_mut(&query, &train),
            inner.match_descriptors(&query, &train)
        );
    }

    #[test]
    fn gpu_failure_falls_back_and_persists() {
        let (query, train) = descriptors();
        let inner = BruteForceMatcher { ratio: Some(0.8) };
        let mut matcher = GpuDescriptorMatcher::with_backend(
            inner,
            Some(FakeBackend {
                calls: 0,
                fail: true,
            }),
        );
        // First call: GPU fails, CPU result returned, GPU disabled.
        assert_eq!(
            matcher.match_descriptors_mut(&query, &train),
            inner.match_descriptors(&query, &train)
        );
        assert!(!matcher.is_gpu_active());
        // Second call: no further GPU attempts.
        assert_eq!(
            matcher.match_descriptors_mut(&query, &train),
            inner.match_descriptors(&query, &train)
        );
    }

    /// Real-kernel A/B against the CPU path. Runs only when
    /// `VISLOC_DESCRIPTOR_GEMM_DLL` points at the built library, so CI without a
    /// GPU stays green. `cargo test -p visloc-vision --features gpu-matcher --
    /// --ignored gpu_kernel_matches_cpu_and_times`.
    #[test]
    #[ignore = "requires VISLOC_DESCRIPTOR_GEMM_DLL and a CUDA device"]
    fn gpu_kernel_matches_cpu_and_times() {
        let Some(path) = std::env::var_os("VISLOC_DESCRIPTOR_GEMM_DLL") else {
            return;
        };
        let mut matcher = GpuDescriptorMatcher::load(BruteForceMatcher { ratio: Some(0.8) }, &path);
        assert!(matcher.is_gpu_active(), "GPU backend failed to load");

        // Deterministic descriptors; matches exist because the train set
        // contains near-duplicates of the query set.
        let dim = 256;
        let mut state = 0x1234_5678_9abc_def0_u64;
        let mut next = || {
            state = state
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((state >> 33) as f32 / (1u64 << 31) as f32) - 1.0
        };
        for &(nq, nc) in &[(300usize, 2048usize), (1173, 929), (1173, 6500)] {
            let train: Vec<Vec<f32>> = (0..nc)
                .map(|_| (0..dim).map(|_| next()).collect())
                .collect();
            // Queries are perturbed train rows, so the GPU and CPU agree
            // exactly (the kernel is exact, not approximate).
            let query: Vec<Vec<f32>> = (0..nq)
                .map(|i| train[i % nc].iter().map(|v| v + 0.01 * next()).collect())
                .collect();

            let cpu = matcher.inner().match_descriptors(&query, &train);
            let gpu = matcher.match_descriptors_mut(&query, &train);
            // Same match pairs and distances to f32 tolerance.
            let cpu_pairs: std::collections::HashSet<(usize, usize)> =
                cpu.iter().map(|m| (m.query_index, m.train_index)).collect();
            let gpu_pairs: std::collections::HashSet<(usize, usize)> =
                gpu.iter().map(|m| (m.query_index, m.train_index)).collect();
            assert_eq!(cpu_pairs, gpu_pairs, "GPU and CPU disagree on pairs");

            // Rough timing A/B over repeated runs (includes the H2D/D2H copies,
            // so it is the honest per-call cost, not a kernel-only number).
            let iters = 10;
            let start = std::time::Instant::now();
            for _ in 0..iters {
                let _ = matcher.inner().match_descriptors(&query, &train);
            }
            let cpu_ms = start.elapsed().as_secs_f64() * 1000.0 / iters as f64;
            let start = std::time::Instant::now();
            for _ in 0..iters {
                let _ = matcher.match_descriptors_mut(&query, &train);
            }
            let gpu_ms = start.elapsed().as_secs_f64() * 1000.0 / iters as f64;
            eprintln!(
                "Nq={nq} Nc={nc}: CPU={cpu_ms:.2} ms  GPU={gpu_ms:.2} ms  speedup={:.2}x",
                cpu_ms / gpu_ms
            );
        }
    }
}
