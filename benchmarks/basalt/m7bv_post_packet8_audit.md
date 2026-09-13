# m7bv post-packet8 endpoint audit

**Date:** 2026-08-23 JST  
**Scope:** read-only comparison of the actual m7bq packet-8 80-frame endpoint with the prior m7bm GEMV endpoint and the authoritative native fixture. No source edits, benchmark runs, commit, or push were performed.

## Result

Packet8 improves exact point-pair count by 56 versus the prior GEMV endpoint (6,306 → 6,362), but loses 60 exact scalar coordinate fields (19,752 → 19,692) and increases cumulative endpoint ULP distance by 3,048. The first native divergence remains unchanged at frame 0, cam1, track 3, x: `0x42395ac3` versus native `0x42395ac4` (1 ULP). The isolated packet-8 L3/I0 and L3/I1 fixtures are exact, but the downstream endpoint is still not native-bit exact.

The machine-readable companion is [`target/m7bv_post_packet8_audit_20260823.json`](../../target/m7bv_post_packet8_audit_20260823.json).

## Artifact selection and provenance

The actual 80-frame endpoint is [`target/m7bq_packet8_first80.jsonl`](../../target/m7bq_packet8_first80.jsonl), SHA-256 `C03D1588…C8F1C3D`, 1,415,012 bytes. The similarly named [`target/m7bq_packet8_endpoint.jsonl`](../../target/m7bq_packet8_endpoint.jsonl) is only one frame (8,651 bytes), so it was not used for 80-frame totals. The m7bq provenance report records the 80-frame runtime as **7.122 s** and reports the same 6,362/24,256 pair and 19,692/48,512 field counts.

The prior endpoint is `target/m7_gemv_endpoint80_20260823.jsonl` (`55B8926B…F915C9F4`); the native fixture is `benchmarks/basalt/mh01_stereo_endpoint_v1_first80.jsonl` (`CEE7E291…D2205B2C`).

## Structural parity

Packet8, prior GEMV, and native contain 80 aligned records, with 15,347 cam0 points and 8,909 cam1 points (24,256 total; 48,512 scalar coordinate fields). After line-index alignment for the native fixture, packet8 has zero schema, timestamp, point-count, or track-ID set/order mismatches against either comparison artifact.

## Packet8 endpoint versus native

| Camera | Points | Exact pairs | Exact fields | Max ULP (x/y) | Max absolute delta (x/y px) |
|---|---:|---:|---:|---:|---:|
| cam0 | 15,347 | 5,459 (35.5705%) | 14,948/30,694 (48.7001%) | 56 / 203 | 0.0008240 / 0.0015488 |
| cam1 | 8,909 | 903 (10.1358%) | 4,744/17,818 (26.6248%) | 29 / 110 | 0.0007935 / 0.0006104 |
| **total** | **24,256** | **6,362 (26.2286%)** | **19,692/48,512 (40.5920%)** | **203** | **0.0015488** |

Packet8 ULP bins, counting mismatched scalar fields only:

| Camera/axis | 1 | 2 | 3–4 | 5–8 | 9–16 | 17–32 | 33–64 | 65+ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| cam0 x | 3,689 | 1,534 | 1,288 | 677 | 153 | 36 | 9 | 0 |
| cam0 y | 3,184 | 1,596 | 1,530 | 1,226 | 579 | 189 | 48 | 8 |
| cam1 x | 2,669 | 1,475 | 1,186 | 522 | 177 | 56 | 0 | 0 |
| cam1 y | 2,303 | 1,255 | 1,366 | 1,149 | 637 | 237 | 36 | 6 |

The largest packet8 error is cam0 frame 79, track 1755, y: 67.63995361328125 versus native 67.6415023803711 (203 ULP, 0.0015487671 px). The largest x absolute error is cam0 frame 75, track 1108 (0.0008239746 px). Cam1 reaches 110 ULP at frame 45, track 1805, y.

## Change versus prior m7bm GEMV

Packet8 and prior are structurally identical. Direct bitwise equality is 35,718/48,512 fields and 15,568/24,256 pairs; the remaining 12,794 fields changed.

| Camera | Exact pairs prior → packet8 | Pair gain/loss transitions | Exact fields prior → packet8 | Field gain/loss transitions |
|---|---:|---:|---:|---:|
| cam0 | 5,381 → 5,459 (**+78**) | 272 / 194 | 14,952 → 14,948 (**−4**) | 955 / 959 |
| cam1 | 925 → 903 (**−22**) | 141 / 163 | 4,800 → 4,744 (**−56**) | 832 / 888 |
| **total** | **6,306 → 6,362 (+56)** | **413 / 357** | **19,752 → 19,692 (−60)** | **1,787 / 1,847** |

The cumulative ULP sum changes from 94,915 to 97,963 (+3,048): cam0 x +858, cam0 y +879, cam1 x −38, and cam1 y +1,349. Thus the pair-count improvement is not a scalar-field or aggregate-ULP improvement.

At field level, packet8 improves / preserves / regresses relative to prior by cam0 x 1,165 / 12,769 / 1,413, cam0 y 1,770 / 11,789 / 1,788, cam1 x 1,426 / 6,156 / 1,327, and cam1 y 1,582 / 5,501 / 1,826. Exact-field gains/losses are cam0 x 466/497, cam0 y 489/462, cam1 x 479/470, and cam1 y 353/418.

### Tracks that move

Aggregating ULP distance across all frames for each track, packet8 has:

- cam0: 189 tracks improved, 3,334 unchanged, 208 regressed;
- cam1: 123 improved, 197 unchanged, 121 regressed.

The largest aggregate improvements are cam0 tracks 2 (−367 ULP), 109 (−264), 119 (−240), 1451 (−221), and 1976 (−218); and cam1 tracks 11 (−397), 30 (−258), 242 (−239), 1432 (−237), and 1536 (−206).

The largest aggregate regressions are cam0 tracks 384 (+966), 178 (+397), 77 (+309), 1309 (+282), and 1108 (+259); and cam1 tracks 1101 (+522), 19 (+383), 73 (+354), 109 (+335), and 315 (+281).

Representative field-level improvements include frame 55 cam0 track 34 x (73 → 18 ULP), frame 32 cam0 track 1451 x (32 → exact), frame 79 cam0 track 2904 y (49 → 19), frame 40 cam1 track 1751 y (30 → exact), and frame 79 cam1 track 1125 y (31 → 3). Representative regressions include frame 50 cam1 track 1101 y (10 → 72 ULP), frame 79 cam0 track 1755 y (145 → 203), and frames 47–50 cam1 track 1101 y (9–10 → 60–72 ULP).

The first direct packet8 change is frame 0 cam1 track 11 x: prior `0x429bc555`, packet8/native `0x429bc556`. This field reaches native exactness, but the same point's y moves one ULP away, so the point pair remains inexact.

The first native mismatch is earlier and unchanged: frame 0 cam1 track 3 x remains packet8/prior `0x42395ac3` versus native `0x42395ac4`.

## Latest available fresh5 check

No packet8-tagged fresh5 detail artifact was present. The newest available fresh5 output is [`target/m7br_packet4_fresh5`](../../target/m7br_packet4_fresh5), timestamped after the packet8 endpoint but named packet4 and trace-only. Its associated m7br report says production was unchanged, so it is not attributed here as a packet8 run.

Compared with the prior m7 GEMV fresh5 trace, all five frame records have identical observation topology, created/retained/rejected track IDs, reject-stage multisets, stage traces, VIO phase traces, and window structure. Frame 4 remains 252 observations, 33 rejected IDs, 68 reject-stage events, state DoF 75, 70 factors, 1,243 rows (15 prior, 1,168 visual, 36 IMU, 24 bias), 58 window landmarks, and successful optimization.

Both traces make eight accepted LM iterations with no rejections (`AAAAAAAA`). The latest trace starts at 4215.8623046875 and ends at 248.47486877441406; prior GEMV starts at 4215.86279296875 and ends at 248.47360229492188. Relative to the authoritative fresh5 final cost 248.54075622558594, the latest trace is −0.065887451171875. No m7br H/b/landmark detail JSONL exists, so no new packet8 fresh5 H/b/landmark claim is made and no fresh5 benchmark was run.

## Interpretation

The m7bq report records exact isolated packet-8 L3/I0 and L3/I1 increment fixtures. That proof is narrower than endpoint parity: packet8 still leaves the same first native divergence, loses scalar-field exactness overall, increases total ULP distance, and introduces a 203-ULP worst case. The packet8 endpoint therefore improves selected tracks but does not close the downstream native mismatch.

