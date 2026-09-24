# Mapper memory: block-sparse global-BA pose Hessian (2026-09-24)

## Why

The urgent keyframe policy (`vi_slam_urgent_keyframe_20260924.md`) improves
accuracy on hard sequences. Its main cost is peak RSS, which rises by 44 to 78%.
The question here is where the extra memory per keyframe goes.

## Diagnosis

`NfrMapper::approx_heap_breakdown()` (in `mapper/session.rs`) sums the element
payload of every persistent mapper container. The demo prints the result as a
`mapper_heap_estimate` line before `finalize()`.

V2_03 runs, KF5 reference against urgent 0.5/2, dense binary, with output under
`/mnt/win/linux_data/visloc_mapper_heap_probe_20260924`:

| | reference | urgent |
|---|---|---|
| mapper keyframe images | 548 | 940 |
| feature_corners (descriptors, rays, BoW) | 15.6 MB | 26.3 MB |
| feature_match_data | 1.6 MB | 3.8 MB |
| matches + tracks + lmdb | 1.9 MB | 5.0 MB |
| **persistent mapper total** | **≈19 MB** | **≈35 MB** |
| peak RSS | 262 MB | 464 MB |

The peak RSS difference is about 200 MB. Persistent mapper state explains only
about 16 MB of it. For the same reason, the full `NfrMapper` clone taken for
each background optimization cannot explain the gap either, because it copies
at most 35 MB. The RSS timeline in `rss.log` shows the jumps at optimization
passes. The urgent run also had `optimize_max` of 26.2 s against 6.6 s.

Cause: global BA stored the Schur-reduced pose system as a **dense `(6K)^2`
`DMatrix`**, where K is the number of mapper poses:

* K = 274 gives 1644² × 8 B ≈ 22 MB;
* K = 470 gives 2820² × 8 B ≈ 64 MB.

`solve_damped_system` then builds a second dense copy with
`(h + h^T) * 0.5`. Memory therefore grows quadratically with keyframes.
Upstream Basalt uses `SparseHashAccumulator` with diagonal-preconditioned CG.
The Rust port keeps the same CG, but its matrix-vector product runs over a
dense matrix.

## Change (default)

`PoseBlockHessian` in `pipelines/basalt/src/mapper/mod.rs` stores 6x6 blocks
in a `BTreeMap<(row, col), Matrix6>`. It is used by `linearize_vision`,
`linearize_factors` and the new `solve_damped_block_system`.

Arithmetic is preserved as follows:

* Blocks accumulate element-wise in the same order as the former
  `view_mut(..).add_assign(..)` calls.
* Symmetrization uses `(h[r,c] + h[c,r]) * 0.5` for each entry, followed by
  the same LM diagonal term.
* `mul_vector` accumulates each row in ascending column order as
  `y = A[r,c] * x[c] + y`. This is the same operation sequence as nalgebra's
  column-major gemv. Absent blocks are exact zeros, so the results differ only
  in the sign of zero.
* The CG body is shared: `preconditioned_cg` takes a diagonal accessor and a
  multiply closure, and the dense path is unchanged.
* Non-finite values, or a CG failure, densify and call the original
  `solve_damped_system`, so the Cholesky, LU and QR fallbacks are unchanged.

Tests in `mapper::pose_block_hessian_tests`:

* the block product matches dense gemv exactly (8 seeds, 23 poses);
* the damped block solve matches the dense solve exactly (8 seeds × 3 lambdas,
  31 poses, asymmetric roundoff residue);
* the non-finite fallback matches bit for bit.

All 433 `visloc-basalt` tests pass.

## Measurement

### V2_03 probe: dense vs sparse binary

Each cell is one run, taken sequentially rather than interleaved. Output is in
`..._heap_probe_20260924` for the dense binary and
`..._heap_probe_sparse_20260924` for the sparse binary.

| | dense ref | dense urgent | sparse ref | sparse urgent |
|---|---|---|---|---|
| peak RSS | 262 MB | 464 MB | 259 MB | **303 MB** |
| optimize_total | 7.9 s | 49.3 s | 6.6 s | **21.5 s** |
| optimize_max | 6.6 s | 26.2 s | 5.9 s | **11.2 s** |
| total_wall | 322.3 s | 290.8 s | 189.7 s | 207.8 s |
| SE3 ATE | 0.07578 | 0.04526 | 0.07419 | 0.04510 |

With urgent keyframes, the sparse binary lowers peak RSS by 35% and total
optimizer time by 56%. The reference, with about 274 poses, is barely affected
because its peak is not dominated by the pose matrix.

After the change, the urgent policy costs +17% RSS over the reference, down
from +77%. ATE differences are within asynchronous run-to-run variation. Wall
times are not comparable across the two probes because host load differed.

A 4-sequence paired screen with the sparse binary (kf5 vs urgent) is running.
Output: `/mnt/win/linux_data/visloc_urgent_kf_screen3_sparse_20260924/experiment`.

### 4-sequence screen with the sparse binary (screen 3)

All 8 runs finished with 0 timeouts. The protocol is identical to screen 2 of
the urgent-keyframe doc, which used the dense binary. Absolute values are
compared across screens 2 and 3, which ran under similar host load.

| Sequence | urgent peak RSS dense → sparse | kf5 peak RSS dense → sparse | urgent vs kf5 ATE (screen 3) |
|---|---|---|---|
| MH_03 | 701564 → 615700 KiB (−12%) | 435076 → 366720 KiB (−16%) | −8.8% |
| MH_04 | 476320 → 367300 KiB (−23%) | 331428 → 269696 KiB (−19%) | −15.4% |
| V2_03 | 453836 → 284732 KiB (−37%) | 254452 → 252068 KiB (−1%) | −40.8% |
| V2_02 | 617736 → 431208 KiB (−30%) | 391548 → 300444 KiB (−23%) | +0.1% |

Findings:

* **Memory:** the sparse Hessian lowers peak RSS in all 8 cells, by 1 to 37%.
  The saving is larger with more poses.
* **Urgent-policy memory overhead:** within screen 3 it is now +13 to +68%.
  In the dense screen it was +44 to +78%.
* **CPU:** the urgent-arm CPU is equal or lower with the sparse binary (for
  example, V2_03 597 → 538 s).
* **Accuracy:** the urgent-policy gains reproduce: −8.8%, −15.4% and −40.8%
  on MH_03, MH_04 and V2_03, and neutral on V2_02.

MH_03 urgent still peaks at 616 MB. Its remaining peak is not the pose matrix
and needs separate attribution.

Decision: adopt the block-sparse Hessian by default. It is numerically
equivalent and gives lower memory and optimizer time.
