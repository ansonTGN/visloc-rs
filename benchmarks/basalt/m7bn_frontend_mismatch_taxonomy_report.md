# M7bn frontend mismatch taxonomy

Date: 2026-08-23 JST  
Pinned Basalt revision: `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`  
Scope: read-only first-80 direct KLT endpoint comparison; no production source
edit, commit, or push.

## Result at a glance

The endpoint was refreshed with the current release `stereo_diag` binary after
the current patch. The fresh run preserves complete structural parity with the
authoritative endpoint: 80/80 records, timestamps, camera counts, track-ID
sets, and track-ID order all match. It is not bit-exact numerically:

| metric | cam0 | cam1 | total |
|---|---:|---:|---:|
| observations | 15,347 | 8,909 | 24,256 |
| exact point pairs | 5,447 (35.49%) | 878 (9.86%) | 6,325 (26.08%) |
| exact coordinate fields | 15,128 / 30,694 (49.29%) | 4,782 / 17,818 (26.84%) | 19,910 / 48,512 (41.04%) |
| mismatched fields | 15,566 | 13,036 | 28,602 |
| maximum ULP distance | 91 | 122 | 122 |
| maximum absolute delta | 0.0010070801 px | 0.0008544922 px | 0.0010070801 px |

The machine-readable artifact is
[`target/m7bn_frontend_mismatch_taxonomy.json`](../../target/m7bn_frontend_mismatch_taxonomy.json).
The refreshed endpoint and stdout are retained as
`target/m7bn_current_endpoint_20260823.jsonl` and
`target/m7bn_current_endpoint_20260823.stdout.log`.

Inputs and hashes are frozen in the JSON artifact. The authoritative fixture is
`benchmarks/basalt/mh01_stereo_endpoint_v1_first80.jsonl` (SHA-256
`cee7e2918a8e9383949508f74b6ec956f6c8a2a8e053648d15d05544d2205b2c`). The
fresh Rust endpoint is 1,415,036 bytes (SHA-256
`8d126d260ea6e3470f0106976f20c349014cfdb5e4ed823653195cc751136cd0`).

## Change relative to M7bi baseline

The requested baseline was
`target/m7bi_rust_endpoint_first80_post_fma_20260823.jsonl`, summarized by
`benchmarks/basalt/m7bi_frontend_first80_post_fma_report.md`. The current
patch's endpoint effect is mixed:

| exactness delta, current minus M7bi | cam0 | cam1 | total |
|---|---:|---:|---:|
| point pairs | +11 | -36 | -25 |
| coordinate fields | +130 | -93 | +37 |
| pair fraction (percentage points) | +0.0717 | -0.4041 | -0.1031 |
| field fraction (percentage points) | +0.4235 | -0.5219 | +0.0763 |

The current run therefore improves aggregate field exactness slightly but
regresses whole-pair exactness, driven by cam1. This rules out treating the
current f32 LDLT/patch result as a monotonic endpoint fix. The aggregate reject
counter total remains 7,132 and all structural output remains unchanged, but
some stage labels move by a few counts (for example `StereoFbSquared` +6,
`StereoBackward(TargetOutOfBounds)` -6, and `FrameFbSquared` +1).

## Mismatch clusters and first divergence

The endpoint has no per-track trace inside the KLT kernel. The clusters below
are consequently lifecycle boundaries, not claims about an unobserved scalar
instruction. Forward/backward/FB substeps, pyramid levels, and iterations are
explicitly marked untraceable unless the endpoint itself proves the boundary.

| cluster | first-divergent tracks | first-divergence evidence | nominal path |
|---|---:|---|---|
| cam0 temporal KLT | 1,031 / 3,731 | all 1,031 diverge after birth; earliest is frame 1 | `FrameForwardSe2Ic → FrameBackwardSe2Ic → FrameFbSquared` |
| cam1 new stereo KLT | 235 / 441 | 235 diverge at birth; earliest frame 0 is track 3 | `StereoForwardSe2Ic → StereoBackwardSe2Ic → StereoFbSquared → DsBearingEssentialResidual` |
| cam1 existing stereo KLT | 180 / 441 | exact at birth, then diverge after birth; earliest first-divergence frame is 1 | `ExistingStereoForwardSe2Ic → ExistingStereoBackwardSe2Ic → ExistingStereoFbSquared` |

The remaining 26 cam1 tracks are bit-exact for their complete observed
lifecycle. No cam0 track first diverges at birth: every frame-0 cam0 point is
an exact integer FAST result.

### Three representative earliest tracks

These are deliberately from three different stream boundaries rather than
three copies of the first endpoint mismatch.

1. **Track 3, cam1, frame 0 — new stereo KLT.** The pinned endpoint is
   `(46.33863830566406, 220.4403533935547)` with bits `(0x42395ac4,
   0x435c70bb)`. Rust is `(46.3386344909668, 220.4403533935547)` with bits
   `(0x42395ac3, 0x435c70bb)`: signed ULP `(-1, 0)`. Its cam0 birth source is
   integer and exact; the first non-exact value is therefore at the new
   cam0-to-cam1 KLT boundary. The endpoint cannot choose between stereo
   forward, stereo backward, FB, and essential-residual arithmetic.

2. **Track 0, cam0, frame 1 — temporal KLT.** Frame-0 birth is `(44, 44)`
   exactly. At frame 1, pinned is `(41.650657653808594,
   44.14083480834961)` with y bits `0x42309037`; Rust is
   `(41.650657653808594, 44.140830993652344)` with y bits `0x42309036`:
   signed ULP `(0, -1)`. This is the earliest later temporal boundary, but
   the endpoint does not record whether forward, backward, or FB arithmetic
   first differs.

3. **Track 1, cam1, frame 5 — existing stereo KLT.** Its cam1 birth and
   frames 1--4 are exact. At frame 5, pinned is
   `(22.85756492614746, 83.79851531982422)` with bits
   `(0x41b6dc4b, 0x42a798d7)`; Rust is
   `(22.857566833496094, 83.79852294921875)` with bits
   `(0x41b6dc4c, 0x42a798d8)`: signed ULP `(+1, +1)`. Exact early life makes
   this a clean existing-cam1-map witness, separate from new stereo birth.

## ULP sign and magnitude

Signed ULP is `Rust - pinned`; positive and negative counts are both large, so
there is no global one-sided correction to apply.

| camera/field | positive | negative | 1 ULP | 2 ULP | 3–4 | 5–8 | 9–16 | 17–32 | 33–64 | 65+ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| cam0 x | 3,674 | 3,621 | 3,778 | 1,583 | 1,281 | 513 | 100 | 33 | 7 | 0 |
| cam0 y | 4,192 | 4,079 | 3,215 | 1,598 | 1,449 | 1,234 | 535 | 180 | 56 | 4 |
| cam1 x | 3,292 | 2,728 | 2,802 | 1,381 | 1,082 | 500 | 235 | 20 | 0 | 0 |
| cam1 y | 3,607 | 3,409 | 2,214 | 1,321 | 1,424 | 1,195 | 576 | 233 | 50 | 3 |

The long tails are concentrated in y. The largest cam1 y miss is track 1805
at frame 45 (122 ULP, +0.0002326965 px); the largest cam0 y miss is track
1755 at frame 56 (91 ULP, +0.0003471375 px). These are ordinary retained
tracks, not ID or gate changes.

## Integer versus subpixel source

The source class is the first emitted point of a track, not the later endpoint
being compared:

| source class | tracks | observation behavior |
|---|---:|---|
| cam0 integer FAST source | 3,731 | all frame-0 cam0 births exact |
| cam1 subpixel stereo source | 441 | 235 tracks differ at stereo birth; all cam1 sources are subpixel |

For later endpoints, cam0 has 3,596 integer-valued observations and all are
exact, while its 11,616 subpixel observations contain 15,566 mismatched
fields and 9,900 mismatched point pairs. All 8,909 cam1 observations are
subpixel; 13,036 fields and 8,031 point pairs mismatch. Every first divergent
coordinate in either camera is subpixel. This is evidence for auditing the
interpolation/normalized-patch arithmetic, not for changing integer FAST
detection.

## Exactness by track lifecycle

The lifecycle table uses `birth` (first observed), `terminal` (last observed
within the 80-frame window), `singleton`, and the interior positions. Exact
fractions are after decoding coordinates to f32.

| camera / position | observations | field exact | pair exact |
|---|---:|---:|---:|
| cam0 birth | 1,272 | 100.00% | 100.00% |
| cam0 second | 892 | 69.39% | 49.66% |
| cam0 middle | 8,720 | 28.23% | 10.14% |
| cam0 penultimate | 732 | 31.01% | 13.80% |
| cam0 terminal | 1,272 | 41.27% | 22.64% |
| cam0 singleton | 2,459 | 100.00% | 100.00% |
| cam1 birth | 403 | 68.49% | 46.65% |
| cam1 second | 380 | 56.84% | 32.89% |
| cam1 middle | 7,324 | 23.11% | 6.65% |
| cam1 penultimate | 361 | 23.82% | 8.31% |
| cam1 terminal | 403 | 23.95% | 7.44% |
| cam1 singleton | 38 | 63.16% | 47.37% |

Long-lived tracks amplify tiny per-step differences. For cam0, tracks with
40+ observations have 6.50% exact pairs and 22.03% exact fields; for cam1 the
corresponding values are 4.39% and 18.03%. Short singleton tracks can remain
exact even though they do not exercise temporal propagation.

## Level/iteration exit traceability

The current endpoint and stdout expose only final observations and aggregate
`RejectReason` counters. The source path is statically known to use pyramid
levels `[3, 2, 1, 0]` and five iterations at every level (20 IC updates per
successful direction), with no small-increment early break. A failed direction
returns immediately, but the current artifacts do not record its level or
iteration.

Current aggregate counters total 7,132:

| reject reason | count |
|---|---:|
| `FrameFbSquared` | 2,316 |
| `StereoFbSquared` | 1,784 |
| `StereoForward(TargetOutOfBounds)` | 690 |
| `FrameForward(TargetInsufficientOverlap)` | 561 |
| `StereoForward(TargetInsufficientOverlap)` | 448 |
| `FrameForward(TargetOutOfBounds)` | 410 |
| `StereoBackward(TargetOutOfBounds)` | 231 |
| `ExistingStereoFbSquared` | 223 |
| all remaining reasons | 469 |

The M7bi-to-current counter changes are small and sum to zero, while ID sets
remain exact. They should not be interpreted as a per-track causal map until
the stream emits `(track_id, camera/map, direction, level, iteration, failure)`
records.

## Kernel ownership and performance-sensitive locations

The first common arithmetic boundary is the direct KLT packet, not endpoint
serialization. Relevant hot locations are:

- `pipelines/basalt/src/patch.rs:34-111`: 52-sample interpolation/gradient,
  mean-normalized Jacobian, Hessian, and inverse packet construction.
- `pipelines/basalt/src/patch.rs:120-169`: per-iteration residual
  normalization and overlap gate.
- `pipelines/basalt/src/patch.rs:181-319`: explicit f32 FMA Hessian and 3x3
  pivoted LDLT solve, the current patch boundary under audit.
- `pipelines/basalt/src/stream.rs:305-389`: temporal cam0 and existing-stereo
  cam1 forward/backward/FB lifecycle.
- `pipelines/basalt/src/stream.rs:432-472`: new cam0-to-cam1 stereo
  forward/backward/FB lifecycle.
- `pipelines/basalt/src/stream.rs:561-610`: four-level/five-iteration KLT
  loop, currently without per-level instrumentation.
- `pipelines/basalt/src/pyramid.rs:94-137,140-225`: bilinear/gradient
  interpolation and four-level raw-u16 pyramid construction.
- `pipelines/basalt/src/fast.rs:86-186`: integer FAST grid replenishment;
  this boundary is structurally exact and should remain frozen during the
  arithmetic audit.

## Prioritized general fixes

1. Add an optional per-track KLT trace before changing arithmetic. Emit the
   track ID, camera/map, direction, level, iteration, valid-overlap count,
   increment, transform, and FB-squared value immediately before each return.
   This converts the three lifecycle clusters above into a real first-scalar
   boundary without perturbing the release path.

2. Audit the f32 patch packet as one native scalar contract: interpolation,
   normalized-Jacobian correction, Hessian FMA order, LDLT pivot/RHS order, and
   SE(2) composition. Require exact fixtures for one new-stereo track, one
   temporal track, and one existing-stereo track before accepting a rewrite.
   The mixed M7bi delta means a local track-specific correction is not justified.

3. Keep FAST/grid detection and lifecycle thresholds unchanged while the packet
   is audited. Their integer births, all ID sets/order, and camera counts are
   already exact; changing them would attack a closed structural boundary.

4. Benchmark any parity-preserving kernel rewrite at the hot locations. Patch
   construction is repeated for every retained/new point, direction, pyramid
   level, and iteration, so exactness work must report release wall time as well
   as endpoint bits.

## Conclusion

The fresh current endpoint is structurally exact but numerically non-exact.
Its divergences arise at three observable KLT lifecycle boundaries—new stereo
birth, temporal cam0 propagation, and existing stereo cam1 propagation—and
every first divergent coordinate is subpixel. The evidence supports optional
level/iteration instrumentation and a general native-order patch-kernel audit;
it does not support track-specific corrections, threshold changes, or FAST
edits.
