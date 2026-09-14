# M7dj post-M7dg relative-pose / H-b measurement

Date: 2026-08-23 JST  
Status: **Pass — read-only replay and comparison; no production source change**

## Scope and provenance

The current release source was rebuilt and replayed on `MH_01_easy` with
`--max-frames 5`.  Detail tracing was enabled only for frame 4 and emitted
the requested JSONL iteration trace:

* detail: `target/m7dj_fresh5_detail.jsonl`
* run output: `target/m7dj_fresh5_run/`
* machine comparison: `target/m7dj_hb_comparison.json`
* authoritative clean H/b: `target/m7ct_clean_frame4_hb.json`
* authoritative clean track-1 point/Jacobian: `target/m7dh_clean_point_jac.json`

The detail SHA-256 is `c6b560d64e785b40294e3d5dff4136fb75835142755b567a98eb199fcc4ce7c3`.  The release executable SHA-256 is
`cf77945c9757911e9b756f8ff855eddb1290f2efee9dd0f23a73c5f677c8eb17`.
No source, commit, or push was changed by this measurement.

## Runtime and structure

| metric | m7dj |
|---|---:|
| frames processed | 5 / 5 |
| IMU samples delivered | 42 |
| observations emitted | 1114 |
| frame-4 endpoint observations | 252 |
| frame-4 created / retained / rejected | 39 / 128 / 33 |
| frame-4 active states / poses | 2 / 1 |
| iteration-start state DoF | 75 |
| AOM items / DoF | 5 / 75 |
| factors / rows | 70 / 1243 |
| landmark records / visual factors | 61 / 61 |
| runtime window landmarks / IMU links | 58 / 4 |
| runtime factor rows prior / visual / IMU / bias | 15 / 1168 / 36 / 24 |
| visual observations / rows | 584 / 1168 |
| detail records | 25 (header + 24 snapshots) |
| LM accepted decisions | `accepted accepted accepted accepted accepted accepted accepted accepted` |

The m7dj and m7cm details have identical AOM, factor/row, landmark, and
visual-observation topology.  The frame-4 iteration-start cost is
`4215.86865234375` and lambda is `9.999999747378752e-05`.

## Clean native H75x75 / b75 comparison

Rust values are cast to binary32; H is transposed from JSON row-major into the
native Eigen column-major sequence before bit comparison.

| buffer | m7dj exact | m7dj mismatch | m7cm mismatch | change |
|---|---:|---:|---:|---:|
| H (5625 lanes) | 3174/5625 | 2451 | 2451 | +0 |
| b (75 lanes) | 7/75 | 68 | 69 | -1 |

H remains exactly at the m7cm baseline mismatch of 2451.  b improves by one
lane: 68 mismatches (7/75 exact) versus the m7cm 69 mismatches (6/75 exact).
The H maximum absolute delta remains `960.0` and
the b maximum absolute delta is `97.0859375`.

## Track 1 clean point / projection / raw / Jacobian

The requested factor is track 1, host frame 0 cam0, target frame 1 cam1,
iteration 0.  m7dj directly serializes the target pixel, projection, raw
residual, and weighted Jacobian.  The current source's M7dg fixed-chain tests
assert the relative-pose packet; the JSONL schema does not serialize that
packet or the homogeneous target point.

| quantity | exact lanes | result |
|---|---:|---|
| relative pose q(xyzw)+t(xyz) | 7/7 | M7dg fixed-chain evidence; not a direct detail field |
| homogeneous target point x,y,z,rho | 4/4 | M7dg downstream packet + detail rho; point not a direct detail field |
| projected u,v | 2/2 | direct m7dj vs clean m7dh |
| raw residual u,v | 2/2 | direct m7dj vs clean m7dh |
| raw d_res_d_p scaled/transposed | 6/6 | diagnostic weight alignment; direct m7dj `jl` vs m7dh raw Jacobian |

Current track-1 bits are projection `41da6d17, 42d70d32`
and raw `bc22d800, 3f915440`, exactly equal to the
m7dh clean values.  Native d_res_d_xi is a relative-pose block, whereas detail
`jp_target` is a weighted absolute target-pose block; those are not direct lane
comparisons.

## All visual factors (detail-available counts)

The detail contains 61 visual factors and 584 observations (1,168 scalar
pixel/projection/raw lanes).  The following isolates M7dg against the m7cm
detail at the same topology; it is not a claim that m7ct supplies a full
per-factor native oracle.

| field | exact lanes / total | exact observation pairs / total |
|---|---:|---:|
| pixel | 1168/1168 | 584/584 |
| projection | 814/1168 | 307/584 |
| raw residual | 814/1168 | 307/584 |
| weighted residual | 686/1168 | 307/584 |
| weighted landmark J | 1707/3504 | 207/584 |
| weighted target pose J | 3748/7008 | 246/584 |
| weighted anchor pose J | 3501/7008 | 230/584 |
| landmark direction | 122/122 | — |
| landmark rho | 61/61 | — |

Projection and raw residual both improve from the m7cm path at the same
counts: 814/1,168 scalar lanes and 307/584 complete observation pairs remain
bit exact.  Relative pose and homogeneous point are explicitly unavailable as
per-observation JSONL fields, so no unsupported all-factor native count is
claimed.

## Verification

```text
cargo test --release -p visloc-basalt --lib 'vio::aom::tests::m7_' -- --nocapture
10 passed, 0 failed
```

The complete machine-readable comparison, including hashes, first/max H/b
mismatches, clean track-1 bits, Jacobian lane accounting, and runtime/state/
landmark counts is `target/m7dj_hb_comparison.json`.
