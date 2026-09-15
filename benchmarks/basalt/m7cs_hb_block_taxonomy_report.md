# M7cs H/b block taxonomy

Date: 2026-08-23 JST  
Scope: read-only taxonomy of the frame-4 uninstrumented m7cp H/b capture
against the m7cm fresh five-frame detail snapshot. No production source,
binary, test, commit, or push was changed.

## Decision boundary

This is a bounded, provisional diagnostic comparison. The native input is the
same uninstrumented executable capture used by m7cp, but that executable
retains the pre-existing BA_REL_DIAG/RUNTIME_REL diagnostic text. Treat every
native-versus-Rust conclusion below as provisional until m7cr supplies a clean
oracle. The m7ck result remains relevant context: all 61 frame-0 landmark
direction/rho triples were exact at binary32 storage precision, so this report
starts after an exact landmark-database boundary. The m7cm LDLT change and
endpoint checks do not make this H/b capture a clean-oracle result.

Machine-readable output: [target/m7cs_hb_block_taxonomy.json](../../target/m7cs_hb_block_taxonomy.json).

## Inputs and comparison

| item | value |
|---|---|
| native capture | target/m7cp_uninstrumented_frame4_hb.json |
| native capture SHA-256 | 7d5d471bf3c115342feaf5c1fe678f44ddd7a212649b141c5f6ec4f109f4d495 |
| Rust detail | target/m7cm_fresh5_detail_20260823.jsonl |
| Rust detail SHA-256 | 978c79b646232080d37697dc66fb25bfeb216578b0fe9f4c89403e93de1c1d1c |
| Rust record | snapshot, phase iteration_start, iteration 0, trial 0, frame 4 |
| AOM | five ordered 15-DoF states, frames 0..4, total 75 DoF |
| semantic order | pose6 [0:6], vel3 [6:9], bg3 [9:12], ba3 [12:15] |
| cast/layout | Rust JSON values cast to binary32; logical H cells compared after accounting for native Eigen column-major storage |

ULP is an ordered binary32 integer distance with -0/+0 adjacent. Values that
cross sign are retained in the requested maximum but counted separately as
sign flips; same-sign maxima are shown to make those large sign-crossing
distances interpretable.

## Overall result

| buffer | total | exact | mismatch | maximum absolute delta | maximum ULP | max same-sign ULP | sign-flip mismatches |
|---|---:|---:|---:|---:|---:|---:|---:|
| H | 5,625 | 3,174 | 2,451 | 960 | 2,143,085,834 | 40,174,088 | 108 |
| b | 75 | 6 | 69 | 97.6171875 | 2,124,010,077 | 41,952,068 | 8 |

The H summary agrees with the m7cp comparator: 2,451 mismatches out of
5,625; b has 69 mismatches out of 75.

## 15x15 state block pairs

Each cell is formatted as mismatch/exact, then maximum absolute delta, then
maximum ULP. State rows and columns are AOM frames 0 through 4.

| row/col | f0 | f1 | f2 | f3 | f4 |
|---|---|---|---|---|---|
| f0 | 205/20<br>352<br>2,143,085,834 | 126/99<br>384<br>2,143,085,834 | 36/189<br>270<br>5,740 | 36/189<br>116<br>4,685 | 36/189<br>233<br>8,122 |
| f1 | 126/99<br>384<br>2,143,085,834 | 219/6<br>704<br>2,050,955,197 | 141/84<br>384<br>2,116,726,895 | 36/189<br>66<br>2,948 | 36/189<br>255.5<br>21,404 |
| f2 | 36/189<br>270<br>5,740 | 141/84<br>384<br>2,116,726,895 | 221/4<br>960<br>2,118,539,172 | 132/93<br>384<br>2,102,265,605 | 36/189<br>307<br>1,216 |
| f3 | 36/189<br>116<br>4,685 | 36/189<br>66<br>2,948 | 132/93<br>384<br>2,102,265,605 | 217/8<br>512<br>2,138,365,119 | 138/87<br>384<br>2,086,997,510 |
| f4 | 36/189<br>233<br>8,122 | 36/189<br>255.5<br>21,404 | 36/189<br>307<br>1,216 | 138/87<br>384<br>2,086,997,510 | 83/142<br>896<br>2,024,428,532 |

The mismatch count matrix is exactly symmetric. The largest state-pair
mismatch count is frame 2 x frame 2 (221/225), followed by frame 1 x frame 1
(219/225), frame 3 x frame 3 (217/225), and frame 0 x frame 0 (205/225).
Cross-state pairs involving frames 0 and 2--4 have 36 mismatches in several
pose-dominated blocks, while frame 1 x frame 2 has 141 and frame 3 x frame 4
has 138.

## Semantic subblocks

The following table aggregates each semantic subblock over all 25 state pairs;
the cell format is mismatch/exact, maximum absolute delta, maximum ULP.

| row/col | pose6 | vel3 | bg3 | ba3 |
|---|---|---|---|---|
| pose6 | 891/9<br>960<br>219,572 | 226/224<br>11<br>2,143,085,834 | 138/312<br>52.625<br>2,727 | 140/310<br>0.089425087<br>375,076 |
| vel3 | 226/224<br>11<br>2,143,085,834 | 110/115<br>0.4375<br>2,054,621,352 | 72/153<br>0.0142822266<br>194,446 | 65/160<br>0.00317382812<br>13 |
| bg3 | 138/312<br>52.625<br>2,727 | 72/153<br>0.0142822266<br>194,446 | 48/177<br>512<br>481,820 | 36/189<br>0.000202178955<br>157,114 |
| ba3 | 140/310<br>0.089425087<br>375,076 | 65/160<br>0.00317382812<br>13 | 36/189<br>0.000202178955<br>157,114 | 48/177<br>4<br>102,707 |

Every semantic row/column class has mismatches. In particular, the pure
IMU/bias region vel3/bg3/ba3 by vel3/bg3/ba3 has 552 mismatches and 1,473
exact cells; the mismatch stream is therefore not confined to pose-only visual
blocks. The aggregate pose6 x pose6 region has 891 mismatches, while cells
with at least one pose6 axis have 1,899 mismatches. The full state-pair x
semantic cross-tab, including exact counts, max absolute deltas,
max ULP, and sign-flip counts for every pair, is retained in
semantic_by_state_pair in the JSON artifact.

## b vector groups

| group | total | exact | mismatch | maximum absolute delta | maximum ULP | max same-sign ULP | sign-flip mismatches |
|---|---:|---:|---:|---:|---:|---:|---:|
| pose6 | 30 | 0 | 30 | 97.6171875 | 39,098 | 39,098 | 0 |
| vel3 | 15 | 0 | 15 | 0.0176478047 | 2,014,310,326 | 32,298,852 | 2 |
| bg3 | 15 | 3 | 12 | 2.53084373 | 2,124,010,077 | 41,952,068 | 5 |
| ba3 | 15 | 3 | 12 | 0.000287306168 | 1,888,425,448 | 33,511,223 | 1 |

All four b groups participate: pose6 is 30/30 mismatched, vel3 is 15/15,
and bg3 and ba3 are each 12/15 mismatched. The three exact entries in each
bias group are retained in the artifact as exact cells; zero/nonzero structure
is unchanged.

## First and maximum cells

- H first mismatch (row-major logical order): H[0,0] native 4de48a64 (479,284,352) vs Rust 4de48a6f (479,284,704); abs 352, ULP 11.
- H maximum absolute delta: H[34,34] native 4e0184d8 (543,241,728) vs Rust 4e0184e7 (543,242,688); abs 960, ULP 15.
- H maximum ordered ULP: H[2,6] native c02fe5c2 (-2.7483983) vs Rust 3f8cff48 (1.10154057); abs 3.84993887, ULP 2,143,085,834. This is a sign flip;
  the largest same-sign H distance is 40,174,088 ULP.
- b first mismatch: b[0] native 45de4e51 (7113.78955) vs Rust 45de4e23 (7113.76709); abs 0.0224609375, ULP 46.
- b maximum absolute delta: b[49] native 47bd2f14 (96862.1562) vs Rust 47bd5fe3 (96959.7734); abs 97.6171875, ULP 12,495.
- b maximum ordered ULP: b[25] native 3f5a1a8e (0.851967692) vs Rust bf3fb7cf (-0.748898447); abs 1.60086614, ULP 2,124,010,077. This is a sign flip;
  the largest same-sign b distance is 41,952,068 ULP.

The H maximum absolute cell is also the largest absolute diagonal error:
H[34,34], native 543,241,728, Rust
543,242,688, absolute delta 960.
The largest ordered diagonal ULP is at H[65,65], with
28 ULP. Per-group diagonal counts are:

| diagonal group | exact | mismatch | maximum absolute delta | maximum ULP |
|---|---:|---:|---:|---:|
| pose6 | 5 | 25 | 960 | 28 |
| vel3 | 3 | 12 | 0.4375 | 9 |
| bg3 | 3 | 12 | 512 | 3 |
| ba3 | 3 | 12 | 4 | 2 |

## Symmetry, zeros, and nonzeros

Native H is bitwise symmetric (0
asymmetric cells), and the Rust H is also bitwise symmetric
(0). The mismatch mask is
bitwise symmetric: 2,451
cells mismatch in both transposed directions and
0 cells are one-sided.
The state-block mismatch matrix is symmetric as well. Thus the taxonomy does
not show a transpose/layout asymmetry in the mismatches.

| buffer | native zero | Rust zero | native nonzero | Rust nonzero | native-zero/Rust-nonzero | native-nonzero/Rust-zero | zero pattern equal |
|---|---:|---:|---:|---:|---:|---:|---:|
| H | 3,078 | 3,078 | 2,547 | 2,547 | 0 | 0 | True |
| b | 6 | 6 | 69 | 69 | 0 | 0 | True |

All 3,078 H zeros and all six b zeros occur at the same locations in both
buffers. The broad numerical mismatch is therefore value/order arithmetic,
not a changed sparsity pattern or missing block.

## Rank and diagonal

Rank here means numerical rank of the binary32 matrices promoted to float64
for SVD; it is not a claim about an exact symbolic rank.

| H matrix | default rank | rank @1e-6 | rank @1e-8 | rank @1e-10 | smallest singular value | largest singular value | condition number |
|---|---:|---:|---:|---:|---:|---:|---:|
| native | 75 | 67 | 72 | 75 | 20.0032246 | 7.2367596e+09 | 361,779,651 |
| Rust | 75 | 67 | 72 | 75 | 20.4244412 | 7.23675878e+09 | 354,318,569 |

Both matrices have full numerical rank 75 at the default SVD threshold and the
same relative-rank profile (67 at 1e-6, 72 at 1e-8, and 75 at 1e-10). The
smallest singular values are about 20.003 (native) and 20.424 (Rust); this is
a value-spectrum change, not a rank change. The complete singular tail and
all diagonal cell records are in the JSON artifact.

## Interpretation and next oracle boundary

The mismatch pattern includes pose, velocity, gyro-bias, and accel-bias
semantic lanes in both H and b. It is symmetric, preserves zero/nonzero
structure, and leaves numerical rank unchanged. This points to arithmetic or
accumulation-order differences within the linearization/assembly path, but the
capture cannot identify a production fix, especially while the native binary
is diagnostic. Keep this report as a taxonomy only and re-run the same
comparison against the clean m7cr H/b oracle before making any source change.
