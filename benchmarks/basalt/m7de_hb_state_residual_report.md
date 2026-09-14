# M7de clean H/b state-residual frontier

Date: 2026-08-23 JST  
Status: **Read-only analysis; production source unchanged**

This report joins the clean-native frame-4 H/b oracle, the clean-native five-
state capture, and the Rust `iteration=0/phase=iteration_start` detail
snapshot. It maps the four state-lane differences to the visual/IMU factor
rows and separates residuals on supports where one factor family is absent.

Machine-readable output: [`target/m7de_hb_state_residual.json`](../../target/m7de_hb_state_residual.json).  
Reproducible analysis script: [`m7de_state_residual.py`](m7de_state_residual.py).

## Inputs and boundary

| input | role |
|---|---|
| [`m7ct_clean_frame4_hb.json`](../../target/m7ct_clean_frame4_hb.json) | authoritative clean native binary32 Eigen H (75×75) and b (75×1) |
| [`m7db_clean_frame4_states.json`](../../target/m7db_clean_frame4_states.json) | authoritative clean native state lanes, frames 0..4 |
| [`m7cm_fresh5_detail_20260823.jsonl`](../../target/m7cm_fresh5_detail_20260823.jsonl) | Rust detail; snapshot `iteration=0`, `trial=0`, `phase=iteration_start` |

The H comparison accounts for native Eigen column-major storage. Rust values
are cast to binary32 before bit comparison. The Rust detail has per-factor
reduced rows/Jacobians; no clean-native per-factor rows were captured, so
mixed-support attribution below is explicitly conditional rather than a
claim of native per-factor equality.

## Result at a glance

The authoritative clean-vs-current-Rust counts are unchanged from M7ct:

| buffer | total | bit mismatches | maximum absolute delta | L2 residual |
|---|---:|---:|---:|---:|
| H | 5,625 | **2,451** | 960.0 | 3,658.1794 |
| b | 75 | **69** | 97.6171875 | 153.4376 |

The H mismatch mask partitions into three disjoint supports:

| support | meaning | cells | mismatches | max abs | L2 | relative L2 |
|---|---|---:|---:|---:|---:|---:|
| visual-only non-adjacent pose | pose6×pose6, `|frame_row-frame_col| > 1`; no IMU/damping support | 432 | **432** | 307.0 | 1,079.7139 | 1.99e-5 |
| IMU-only non-pose | at least one velocity/gyro-bias/accel-bias lane; visual rows are zero there | 4,725 | **1,560** | 512.0 | 2,151.4612 | 1.32e-7 |
| mixed pose | remaining diagonal/adjacent pose6×pose6 cells | 468 | **459** | 960.0 | 2,754.5796 | 7.21e-7 |

The first two rows are the useful kernel boundaries: the first is a direct
visual residual, while the second is a direct IMU residual (the damping rows
are diagonal and were reconstructed separately). Their mismatch counts plus
the mixed count sum exactly to 2,451.

The clean-vs-Rust 15×15 state-block mismatch matrix is:

```text
205 126  36  36  36
126 219 141  36  36
 36 141 221 132  36
 36  36 132 217 138
 36  36  36 138  83
```

## State lane gate: exactly four differences

The clean state comparison is 4/75 lanes, all same-sign two-ULP differences
where reported below. The clean and Rust bits are the comparison boundary:

| frame | field/lane | clean bits | Rust bits | delta | factor exposure |
|---:|---|---|---|---:|---|
| 2 | quaternion `xyzw[3]` | `3f17e7a6` | `3f17e7a4` | −2 ULP | visual target-frame rows 236; IMU factors 1 (f1→f2), 2 (f2→f3), 30 rows |
| 4 | quaternion `xyzw[3]` | `3f159719` | `3f159717` | −2 ULP | visual target-frame rows 220; IMU factor 3 (f3→f4), 15 rows |
| 4 | velocity `xyz[1]` | `bc8bed06` | `bc8bed08` | +2 ULP | no direct visual dependency; IMU factor 3, 15 rows |
| 4 | velocity `xyz[2]` | `be67f1c6` | `be67f1c8` | +2 ULP | no direct visual dependency; IMU factor 3, 15 rows |

The visual row counts are observation-row counts (two residual rows per
observation). There are 584 visual observations / 1,168 rows in total; the
quaternion lanes expose 456 rows (39.04%). All 61 landmark factors are in the
union because every factor has a frame-2 observation; frame 4 has 58 factors
and 220 rows. The changed IMU factor union is factors 1, 2, and 3: 45/60
rows (75%). Exacting these four state lanes is therefore the smallest next
implementation frontier, but row exposure is a sensitivity bound, not a
promise that the same number of H/b bits will disappear.

## Factor-row partition and reconstruction

The Rust snapshot's global reduced-row array has an exact alignment check:
the concatenated 61 per-landmark `reduced_rows` and `reduced_rhs` match global
rows 15:1183 with maximum absolute delta **0.0**. The remaining tail groups
cleanly into four contiguous 15-row IMU factors.

| family | global rows | factors | rows | H support | b support |
|---|---:|---:|---:|---:|---:|
| LM damping | 0:15 | — | 15 | 10 cells | 0 lanes |
| visual landmark | 15:1183 | 61 | 1,168 (584 observations) | 900 cells (pose6×pose6) | 30 lanes (pose6) |
| IMU | 1183:1243 | 4 frame-pair factors | 60 | 2,115 cells | 69 lanes |

For each family, the report forms `H_family = Σ rowᵀrow` and
`b_family = Σ rowᵀrhs`. Family contribution norms are:

| family | `||H_family||F` | `||b_family||2` |
|---|---:|---:|
| damping | 2.0000000e8 | 0 |
| visual | 1.8154760e8 | 305,291.3676 |
| IMU | 1.6665484e10 | 157.7212 |

The four IMU factors are frame pairs 0→1, 1→2, 2→3, and 3→4. Their H/b
norms and global spans are retained in the JSON artifact.

## b residual by semantic lane group

The b result uses each semantic group in every 15-DoF state (not contiguous
file slices):

| group | lanes | mismatches | max abs | L2 |
|---|---:|---:|---:|---:|
| pose6 | 30 | **30** | 97.6171875 | 153.3853 |
| velocity3 | 15 | **15** | 0.0176478 | 0.02765 |
| gyro bias3 | 15 | **12** | 2.5308437 | 4.0047 |
| accel bias3 | 15 | **12** | 0.0002873 | 0.0004433 |

Thus the pure non-pose IMU b support is 39/45 mismatched; pose b remains a
mixed visual/IMU support.

## Minimal frontier and remaining kernel boundary

1. Exact the four state lanes at the Rust/native state boundary, then rerun
   the same clean H/b comparator. This closes the only observed state-input
   gate (4→0 state mismatches) and directly exercises 456 visual rows plus 45
   IMU rows.
2. Use the rerun to distinguish improvement due to state exactification from
   residual kernel arithmetic. If only the state lanes are changed, the
   current disjoint-support residual is the quantitative baseline: 432/432
   visual-only H cells, 1,560/4,725 IMU-only H cells, 459/468 mixed pose H
   cells, 30/30 pose b lanes, and 39/45 non-pose b lanes.
3. Do not infer that state exactification alone will make H/b bit-exact. The
   clean capture has aggregate H/b only; without native per-factor rows, the
   mixed pose cells cannot be assigned uniquely to the visual or IMU kernel.
   The pure-support residuals above are the safe quantitative frontier for the
   next implementation decision.

No production source, native binary, Rust source, commit, or push was changed.
