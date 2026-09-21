//! Inverted-file (IVF) descriptor matcher.
//!
//! Brute-force matching is exact but `O(N_query x N_train x dim)`. On the
//! online-SLAM tracker the train side is the whole map, so the cost grows with
//! the map and the matcher dominates the frame budget. This matcher fits a
//! coarse k-means codebook over the train descriptors and delegates an exact
//! search over only the probed cells, using
//! [`Matcher::match_descriptors_indexed`] so the candidate rows are consumed
//! **without** a `Vec<Vec<f32>>` copy — the copy is what otherwise cancels the
//! saving.
//!
//! # Semantics and recall
//!
//! Cell assignment is a pre-filter, and the Lowe ratio is evaluated inside the
//! probed pool. Relative to [`BruteForceMatcher`] the ANN can therefore both
//! drop matches (the partner was in an unprobed cell) and admit a partner brute
//! force rejected (the second-best inside the pool is farther away). Recall
//! over brute force's pair set is monotone in [`IvfConfig::n_probe`], and
//! `n_probe == n_cells` reproduces brute force exactly (asserted in the tests).
//!
//! # Determinism
//!
//! k-means is seeded by farthest-point sampling over the input order (no RNG)
//! and runs a fixed iteration count, so the codebook and match set are
//! reproducible for a given train set.
//!
//! # Measured limitation (why this is not wired into the tracker yet)
//!
//! The clone-free indexed kernel is a real win on a **subset** —
//! `match_descriptors_indexed` over every 8th row runs 5.6x (929 train rows) to
//! 10x (6,500 rows) faster than the full set. But turning that into an
//! end-to-end matcher win needs the per-query candidate sets to stay small,
//! and the two available strategies both fail at realistic query counts:
//!
//! - **One union pool per call** (what [`IvfMatcher::match_descriptors_indexed`]
//!   does): with ~1,173 queries and 32-256 cells the union of probed cells
//!   covers essentially the whole map (measured/estimated 99-100%), so there is
//!   nothing to save.
//! - **Grouping queries by probe pattern**: up to `n_cells` buckets, each
//!   paying a query-subset clone and GEMM setup — measured slower than brute
//!   force.
//!
//! Dropping to `probe = 1` with many cells (512-4096) still measured 0.5-0.8x
//! (i.e. slower) because the per-query candidate bookkeeping dominates, and
//! codebook training cost explodes (608 s at 512 cells, 2.3 ks at 1,024 on a
//! 6,500-row set). A future version needs either a much larger cell count with a
//! cheap inverted-list intersection, or an actual GPU/batched kernel; the
//! indexed API added here is the prerequisite for either.

use nalgebra::{DMatrix, DVector};

use super::{BruteForceMatcher, DescriptorMatch, Matcher};

/// Configuration for [`IvfMatcher`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IvfConfig {
    /// Number of coarse cells (k-means clusters). Clamped to `>= 1`.
    pub n_cells: usize,
    /// Cells probed per query. Clamped to `1..=n_cells` at match time.
    pub n_probe: usize,
    /// Lloyd iterations used to fit the codebook.
    pub kmeans_iterations: usize,
    /// Train sets smaller than this skip the index and run plain brute force.
    pub min_train_for_index: usize,
}

impl Default for IvfConfig {
    fn default() -> Self {
        Self {
            n_cells: 32,
            n_probe: 4,
            kmeans_iterations: 12,
            min_train_for_index: 256,
        }
    }
}

/// An inverted-file (coarse-quantised) descriptor matcher over
/// [`BruteForceMatcher`]'s exact kernel.
#[derive(Debug, Clone)]
pub struct IvfMatcher {
    inner: BruteForceMatcher,
    config: IvfConfig,
    /// Cell centroids (`n_cells x dim`), or `None` when the last training set
    /// was too small to index.
    centroids: Option<DMatrix<f32>>,
    /// Train indices assigned to each cell.
    cells: Vec<Vec<usize>>,
}

impl IvfMatcher {
    /// Build an IVF matcher with the given inner policy and index config.
    pub const fn new(inner: BruteForceMatcher, config: IvfConfig) -> Self {
        Self {
            inner,
            config,
            centroids: None,
            cells: Vec::new(),
        }
    }

    /// Borrow the inner brute-force policy (ratio / cross-check knobs).
    pub const fn inner(&self) -> &BruteForceMatcher {
        &self.inner
    }

    /// The number of cells built by the most recent training call.
    pub fn cell_count(&self) -> usize {
        self.cells.len()
    }

    /// Whether a codebook is currently active.
    pub const fn is_trained(&self) -> bool {
        self.centroids.is_some()
    }

    /// Fit the coarse codebook on `train`.
    ///
    /// Leaves the matcher in exact brute-force mode (a no-op) when the train
    /// set is smaller than [`IvfConfig::min_train_for_index`], when
    /// `kmeans_iterations` is zero, or when the descriptors have mixed
    /// dimensions.
    pub fn train(&mut self, train: &[Vec<f32>]) {
        self.centroids = None;
        self.cells.clear();
        let n_cells = self.config.n_cells.max(1);
        let Some(dim) = uniform_dimension(train) else {
            return;
        };
        if train.len() < self.config.min_train_for_index.max(n_cells)
            || self.config.kmeans_iterations == 0
        {
            return;
        }
        let train_matrix = descriptor_matrix(train, dim);
        let centroids = kmeans(&train_matrix, n_cells, self.config.kmeans_iterations);
        let mut cells = vec![Vec::new(); n_cells];
        for (index, descriptor) in train.iter().enumerate() {
            cells[nearest_centroid(&centroids, descriptor)].push(index);
        }
        self.centroids = Some(centroids);
        self.cells = cells;
    }

    /// Union of the probed cells' train indices, ascending and deduplicated.
    fn probe_candidates(&self, query: &[f32]) -> Vec<usize> {
        let Some(centroids) = self.centroids.as_ref() else {
            return Vec::new();
        };
        let n_probe = self.config.n_probe.clamp(1, self.cells.len().max(1));
        let mut order: Vec<(usize, f32)> = (0..self.cells.len())
            .map(|cell| (cell, squared_centroid_distance(centroids, cell, query)))
            .collect();
        order.sort_by(|left, right| {
            left.1
                .total_cmp(&right.1)
                .then_with(|| left.0.cmp(&right.0))
        });
        let mut candidates: Vec<usize> = order
            .into_iter()
            .take(n_probe)
            .flat_map(|(cell, _)| self.cells[cell].iter().copied())
            .collect();
        candidates.sort_unstable();
        candidates.dedup();
        candidates
    }
}

impl Matcher for IvfMatcher {
    /// Exact brute-force over the whole train set; kept for callers that do not
    /// go through `match_descriptors_indexed`.
    fn match_descriptors(&self, query: &[Vec<f32>], train: &[Vec<f32>]) -> Vec<DescriptorMatch> {
        self.inner.match_descriptors(query, train)
    }

    fn match_descriptors_indexed(
        &self,
        query: &[Vec<f32>],
        train: &[Vec<f32>],
        _train_indices: &[usize],
    ) -> Vec<DescriptorMatch> {
        if !self.is_trained() || self.cells.len() != self.config.n_cells.max(1) {
            return self.inner.match_descriptors(query, train);
        }
        if query.is_empty() {
            return Vec::new();
        }

        // One global candidate pool per call: the union of every query's probed
        // cells. Grouping queries by their individual probe pattern instead
        // would create up to `n_cells` buckets, each paying a full query-subset
        // clone and GEMM setup — measured to be slower than brute force. A
        // single union keeps exactly one clone-free indexed GEMM, then filters
        // each match to those queries whose own probe set contains its partner.
        let mut allowed: std::collections::HashSet<usize> = std::collections::HashSet::new();
        let mut per_query: Vec<std::collections::HashSet<usize>> = Vec::with_capacity(query.len());
        for descriptor in query {
            let candidates = self.probe_candidates(descriptor);
            allowed.extend(candidates.iter().copied());
            per_query.push(candidates.into_iter().collect());
        }
        if allowed.is_empty() {
            return Vec::new();
        }
        let mut pool: Vec<usize> = allowed.into_iter().collect();
        pool.sort_unstable();

        let mut matches = self.inner.match_descriptors_indexed(query, train, &pool);
        matches.retain(|descriptor_match| {
            per_query
                .get(descriptor_match.query_index)
                .is_some_and(|candidates| candidates.contains(&descriptor_match.train_index))
        });
        matches.sort_by(|left, right| {
            left.query_index
                .cmp(&right.query_index)
                .then_with(|| left.train_index.cmp(&right.train_index))
        });
        matches
    }
}

/// The common descriptor dimension, or `None` when empty or mixed.
fn uniform_dimension(descriptors: &[Vec<f32>]) -> Option<usize> {
    let dim = descriptors.first().map(Vec::len)?;
    if dim == 0 || descriptors.iter().any(|descriptor| descriptor.len() != dim) {
        return None;
    }
    Some(dim)
}

fn descriptor_matrix(descriptors: &[Vec<f32>], dim: usize) -> DMatrix<f32> {
    DMatrix::from_fn(descriptors.len(), dim, |row, column| {
        descriptors[row][column]
    })
}

fn squared_centroid_distance(centroids: &DMatrix<f32>, cell: usize, descriptor: &[f32]) -> f32 {
    let mut sum = 0.0_f32;
    for column in 0..centroids.ncols() {
        let diff = centroids[(cell, column)] - descriptor[column];
        sum += diff * diff;
    }
    sum
}

fn nearest_centroid(centroids: &DMatrix<f32>, descriptor: &[f32]) -> usize {
    let mut best = 0usize;
    let mut best_distance = f32::INFINITY;
    for cell in 0..centroids.nrows() {
        let distance = squared_centroid_distance(centroids, cell, descriptor);
        if distance < best_distance {
            best_distance = distance;
            best = cell;
        }
    }
    best
}

/// Deterministic Lloyd k-means with farthest-point seeding (no RNG).
fn kmeans(data: &DMatrix<f32>, k: usize, iterations: usize) -> DMatrix<f32> {
    let rows = data.nrows();
    let dim = data.ncols();
    let mut centroids = DMatrix::zeros(k, dim);

    let mut chosen: Vec<usize> = Vec::with_capacity(k);
    chosen.push(0);
    while chosen.len() < k {
        let mut best_row = 0usize;
        let mut best_distance = -1.0_f32;
        for row in 0..rows {
            let mut nearest = f32::INFINITY;
            for &center in &chosen {
                let mut sum = 0.0_f32;
                for column in 0..dim {
                    let diff = data[(row, column)] - data[(center, column)];
                    sum += diff * diff;
                }
                nearest = nearest.min(sum);
            }
            if nearest > best_distance {
                best_distance = nearest;
                best_row = row;
            }
        }
        chosen.push(best_row);
    }
    for (cell, &row) in chosen.iter().enumerate() {
        for column in 0..dim {
            centroids[(cell, column)] = data[(row, column)];
        }
    }

    let mut assignments = vec![usize::MAX; rows];
    for _ in 0..iterations {
        let mut changed = false;
        for row in 0..rows {
            let mut best = 0usize;
            let mut best_distance = f32::INFINITY;
            for cell in 0..k {
                let mut sum = 0.0_f32;
                for column in 0..dim {
                    let diff = data[(row, column)] - centroids[(cell, column)];
                    sum += diff * diff;
                }
                if sum < best_distance {
                    best_distance = sum;
                    best = cell;
                }
            }
            if assignments[row] != best {
                assignments[row] = best;
                changed = true;
            }
        }
        let mut sums = DMatrix::<f32>::zeros(k, dim);
        let mut counts = DVector::<f32>::zeros(k);
        for row in 0..rows {
            let cell = assignments[row];
            for column in 0..dim {
                sums[(cell, column)] += data[(row, column)];
            }
            counts[cell] += 1.0;
        }
        for cell in 0..k {
            if counts[cell] > 0.0 {
                for column in 0..dim {
                    centroids[(cell, column)] = sums[(cell, column)] / counts[cell];
                }
            }
        }
        if !changed {
            break;
        }
    }
    centroids
}

#[cfg(test)]
mod tests {
    use super::*;

    fn synthetic(n: usize, dim: usize, seed: u64) -> Vec<Vec<f32>> {
        let mut state = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
        (0..n)
            .map(|_| {
                (0..dim)
                    .map(|_| {
                        state = state
                            .wrapping_mul(6364136223846793005)
                            .wrapping_add(1442695040888963407);
                        ((state >> 33) as f32 / (1u64 << 31) as f32) - 1.0
                    })
                    .collect()
            })
            .collect()
    }

    fn recall(ann: &[DescriptorMatch], exact: &[DescriptorMatch]) -> f64 {
        if exact.is_empty() {
            return 1.0;
        }
        let pairs: std::collections::HashSet<(usize, usize)> = exact
            .iter()
            .map(|m| (m.query_index, m.train_index))
            .collect();
        ann.iter()
            .filter(|m| pairs.contains(&(m.query_index, m.train_index)))
            .count() as f64
            / exact.len() as f64
    }

    #[test]
    fn indexed_bruteforce_matches_the_slice_path() {
        // The clone-free indexed kernel must equal the materialised-subset path.
        let query = synthetic(40, 24, 5);
        let train = synthetic(200, 24, 6);
        let indices: Vec<usize> = (0..200).step_by(3).collect();
        let matcher = BruteForceMatcher { ratio: Some(0.8) };
        let subset: Vec<Vec<f32>> = indices.iter().map(|&i| train[i].clone()).collect();
        let mut expected = matcher.match_descriptors(&query, &subset);
        for m in &mut expected {
            m.train_index = indices[m.train_index];
        }
        let mut actual = matcher.match_descriptors_indexed(&query, &train, &indices);
        expected.sort_by_key(|m| (m.query_index, m.train_index));
        actual.sort_by_key(|m| (m.query_index, m.train_index));
        assert_eq!(expected, actual);
    }

    #[test]
    fn untrained_matcher_is_exact_brute_force() {
        let query = synthetic(32, 16, 1);
        let train = synthetic(64, 16, 2);
        let inner = BruteForceMatcher { ratio: Some(0.8) };
        let ivf = IvfMatcher::new(inner, IvfConfig::default());
        assert!(!ivf.is_trained());
        let mut small = ivf.clone();
        small.train(&train);
        assert!(!small.is_trained(), "below min_train_for_index stays exact");
        assert_eq!(
            small.match_descriptors(&query, &train),
            inner.match_descriptors(&query, &train)
        );
        let indices: Vec<usize> = (0..train.len()).collect();
        assert_eq!(
            small.match_descriptors_indexed(&query, &train, &indices),
            inner.match_descriptors(&query, &train)
        );
    }

    #[test]
    fn probing_every_cell_reproduces_brute_force() {
        let query = synthetic(64, 32, 11);
        let train = synthetic(1024, 32, 12);
        let inner = BruteForceMatcher { ratio: Some(0.8) };
        let exact = inner.match_descriptors(&query, &train);
        let mut full = IvfMatcher::new(
            inner,
            IvfConfig {
                n_cells: 32,
                n_probe: 32,
                ..IvfConfig::default()
            },
        );
        full.train(&train);
        let indices: Vec<usize> = (0..train.len()).collect();
        let mut ann = full.match_descriptors_indexed(&query, &train, &indices);
        ann.sort_by_key(|m| (m.query_index, m.train_index));
        let mut exact_sorted = exact;
        exact_sorted.sort_by_key(|m| (m.query_index, m.train_index));
        assert_eq!(ann, exact_sorted);
    }

    #[test]
    fn ann_recall_is_monotone_in_n_probe() {
        // The candidate pool is a pre-filter, so the ANN can drop matches AND
        // (because the Lowe ratio is evaluated inside the probed pool) admit a
        // partner brute force rejected. What is guaranteed is that recall over
        // brute-force's pair set is monotone in `n_probe`, and exact at
        // `n_probe == n_cells` (covered by the test above).
        let query = synthetic(48, 32, 21);
        let train = synthetic(2048, 32, 22);
        let inner = BruteForceMatcher { ratio: Some(0.8) };
        let exact = inner.match_descriptors(&query, &train);
        let all: Vec<usize> = (0..train.len()).collect();

        let mut narrow = IvfMatcher::new(
            inner,
            IvfConfig {
                n_cells: 64,
                n_probe: 1,
                ..IvfConfig::default()
            },
        );
        narrow.train(&train);
        let mut wide = IvfMatcher::new(
            inner,
            IvfConfig {
                n_cells: 64,
                n_probe: 16,
                ..IvfConfig::default()
            },
        );
        wide.train(&train);

        let narrow_recall = recall(
            &narrow.match_descriptors_indexed(&query, &train, &all),
            &exact,
        );
        let wide_recall = recall(
            &wide.match_descriptors_indexed(&query, &train, &all),
            &exact,
        );
        assert!(
            wide_recall + 1.0e-9 >= narrow_recall,
            "more probing must not lose recall: {wide_recall} < {narrow_recall}"
        );
    }

    #[test]
    fn training_is_deterministic() {
        let train = synthetic(1024, 32, 31);
        let inner = BruteForceMatcher { ratio: Some(0.8) };
        let mut first = IvfMatcher::new(inner, IvfConfig::default());
        let mut second = IvfMatcher::new(inner, IvfConfig::default());
        first.train(&train);
        second.train(&train);
        assert_eq!(first.cell_count(), second.cell_count());
        assert_eq!(first.centroids, second.centroids);
        let query = synthetic(32, 32, 32);
        let indices: Vec<usize> = (0..train.len()).collect();
        assert_eq!(
            first.match_descriptors_indexed(&query, &train, &indices),
            second.match_descriptors_indexed(&query, &train, &indices)
        );
    }
}
