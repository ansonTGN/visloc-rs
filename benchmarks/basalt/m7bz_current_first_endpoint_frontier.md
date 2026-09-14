# m7bz current first endpoint frontier

Date: 2026-08-23 JST  
Scope: bounded two-frame diagnostic; no production change.

## Deterministic first mismatch

The authoritative native endpoint
`benchmarks/basalt/mh01_stereo_endpoint_v1_first80.jsonl` and the current
post-composition endpoint `target/m7bt_first80_final3.jsonl` were compared in
the required order: frame line, `cam0` then `cam1`, stored point order, and
`x` then `y`.  The first difference is:

| field | native | Rust |
|---|---:|---:|
| frame line / timestamp | 1 / `1403636579813555456` | same |
| camera / stored point / track | cam1 / 2 / 3 | same |
| x bits | `0x422f68f8` | `0x422f68f7` |
| x value | 43.852508544921875 | 43.85250473022461 |
| signed ULP (Rust − native) | — | −1 |

The corresponding `y` is exact at `0x435c045a`.  Therefore the m7bx
track3/x classification is the true current frontier; the previously
reported frame-1 cam1 track-117/y residual is later in stored point order.
For reference, track 117/y is native `0x4249983c` versus Rust
`0x42499838` (four ULP), while its x is exact (`0x44302fd9`).

## Track-3 operation trace

Track 3 is an existing cam1 observation at frame 1.  Its frame-0 cam1 birth
is exact (`0x42395ac4,0x435c70bb`), so the nominal next path is

```text
ExistingStereoForwardSe2Ic
  -> ExistingStereoBackwardSe2Ic
  -> ExistingStereoFbSquared
```

The focused native trace is `target/m7bz_native_existing_trace.txt`; the
focused Rust trace is `target/m7bz_rust_existing_trace.txt`.  Both trace the
frame-0 birth only to seed the frame-1 cam1 source, then trace frame-1
existing-stereo forward/backward.  The forward endpoint from that trace is
native `0x422f68f8,0x435c045a` and Rust `0x422f68f7,0x435c045a`, matching the
authoritative endpoint comparison.

The first divergent scalar after the already-closed composition boundary is
not residual evaluation.  At `ExistingStereoForward`, level 3, iteration 0:

```text
pre linear = [0x3f800000, 0x00000000, 0x00000000, 0x3f800000]
pre t      = [0x40b95ac4, 0x41dc70bb]
residual[0..51] = exact (residual[0] = 0x3d782160)
```

The next operation is the pinned `3x52 * 52x1` inverse-Jacobian GEMV in
`patch.ic_increment`:

```text
native increment = [0xbeb5dbcd, 0xbd7fc4c0, 0x3b582340]
Rust increment   = [0xbeb5dbcd, 0xbd7fc4c0, 0x3b582338]
```

The x and y lanes are exact; the theta lane is eight ULP low in Rust
(native about `0.0032979995`, Rust about `0.0032979976`).  The first post-
composition affine values, written in canonical
`[a00,a01,a10,a11,tx,ty]` order, are consequently:

```text
native = [0x3f7fffa5, 0xbb582326, 0x3b582326, 0x3f7fffa5,
          0x40adfde0, 0x41dbefa6]
Rust   = [0x3f7fffa5, 0xbb58231e, 0x3b58231e, 0x3f7fffa5,
          0x40adfde0, 0x41dbefa6]
```

This isolates the next fix boundary to the general packet-8 GEMV reduction
for an existing-stereo frame-1 residual.  The existing packet-8 fixtures
prove the new-stereo frame-0 L3/I0 and L3/I1 cases, but do not cover this
different residual vector.  Recommended follow-up: add an isolated
existing-stereo L3/I0 fixture containing the pre-warp, all 52 residual bits,
and all three native increment bits, then adjust only the general Eigen
5.0.1 packet-8 accumulator/reduction order.  Do not add a track-specific
endpoint correction or change SE2 composition until that fixture is exact.

## Focused verification and cleanup

After removing the opt-in trace code and rebuilding the release example, a
two-frame run completed with `rc=0` in 0.762 s.  The clean endpoint
`target/m7bz_rust_endpoint2_clean.jsonl` retained the expected frame-1
track-3 bits `x=0x422f68f7,y=0x435c045a`; no full first-80 replay was run in
this bounded diagnostic.  `rg` confirms no `M7BZ`/`VISLOC_BASALT_M7BZ_TRACE`
instrumentation remains in `pipelines/basalt/src`.

No production source was changed in this turn; no commit or push was made.
