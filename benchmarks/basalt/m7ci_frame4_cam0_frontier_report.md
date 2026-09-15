# M7ci frame-4 cam0 temporal frontier

Date: 2026-08-23 JST

## Decision

No production arithmetic change was justified at this frontier.  The
current first endpoint mismatch is a normal forward temporal cam0 KLT path:
the frame-3 observation of track 210 is carried from
`1403636579913555456` at `(0x43830000,0x42140000)` (262, 37) into frame 4,
`1403636579963555584`.  It is not a new-stereo or stereo-correction path.

The exact boundary fixture is
`pipelines/basalt/tests/fixtures/m7ci_frame4_cam0_ldlt_boundary.json`, and
the regression test is
`patch::tests::m7ci_frame4_cam0_ldlt_boundary_is_recorded`.  The test records
the current Rust result and the native result; it does not hide the
unclosed difference or alter the production path.

The prior general m7ce Jacobian order and packet-8 GEMV implementation remain
unchanged.  `update.rs`, AOM, and all tracking branches remain unchanged.

## First divergent scalar

The native trace is the pinned Eigen 5.0.1 O2 temporal replay, compared with
the current Rust trace one scalar at a time.

| stage | lanes | first result | native | Rust |
|---|---:|---|---|---|
| source normalized data | 52 | exact | — | — |
| normalized Jacobian | 156 | exact | — | — |
| Hessian `JᵀJ` | 9 | exact | — | — |
| pivoted LDLT inverse | 9 | row-major index 2 (`row 0,col 2`) | `0xbd3213f2` | `0xbd3213f1` |
| pivoted LDLT inverse | 9 | second mismatch, index 8 | `0x3cd0be6f` | `0x3cd0be6e` |
| `H⁻¹Jᵀ` | 156 | first coefficient, row 0/sample 1 | `0xbe9fbd4e` | `0xbe9fbd4d` |

The inverse has 7/9 exact row-major entries; its only mismatches are indices
2 and 8.  The final coefficient matrix has 48/156 mismatches (maximum 32
ULP).  Thus residual sampling, the fixed 3x52 packet-8 GEMV, and SE(2)
composition are downstream effects, not the first divergent operation.

The next operation to isolate is the pinned Eigen `H.ldlt().solve(I)` upper
triangular solve/reconstruction order for this general 3x3 Hessian.  The
existing hand-written `ldlt_inverse` order is not proven equivalent for this
input, and changing it without a general native fixture would risk regressing
the already exact Hessian/LDLT cases.  No track-, camera-, row-, threshold-,
unsafe-, or debug-specific workaround is appropriate.

## Temporal iteration trace

The first L3 residual is still exact; its increment diverges only because the
stored inverse/coefficient matrix already diverged:

| level / iteration | residual comparison | native increment | Rust increment |
|---|---|---|---|
| L3/I0 | 52/52 exact | `bd66e4de,bf75005a,3d6bf2d0` | `bd66e4e3,bf75005a,3d6bf2ce` |
| L3/I1 | first residual mismatch at sample 17: `3e57a3d0` vs `3e57a3bc` | `3e616f4f,3d07d706,bd7922b1` | `3e616f4d,3d07d708,bd7922ae` |
| L2/I0 | residual exact; one-ULP increment difference | `be0a0740,3e071882,bc54752c` | `be0a0740,3e071883,bc54752c` |
| L1/I0..I4 | residuals and increments exact | — | — |
| L0/I0 | residual exact; x differs by 3 ULP | `3c2e349c,bda8a7f3,3c1a3153` | `3c2e349f,bda8a7f3,3c1a3153` |

The differing L3 updates are absorbed by later affine translation rounding at
the displayed precision, but the later residual and L0 increments expose the
coefficient difference.  The final cam0 track-210 endpoint is:

```text
native:  x=0x4382f3cb  y=0x41ee177e
Rust:    x=0x4382f3cc  y=0x41ee177d
```

## Verification

Focused release checks after removing all temporary probes and wrappers:

```text
cargo test -p visloc-basalt patch --release --lib       7 passed
cargo test -p visloc-basalt update --release --lib      7 passed
m7bd_obs_pixel_exact --ignored                            1 passed
m7bw_cam0_temporal_exact                                 1 passed
```

The m7ce first-80 artifact remains the current production artifact because
this task made no production arithmetic change.  It parses as 80 records with
exact structure and retains the m7ce aggregate:

```text
exact coordinate fields: 47,372 / 48,512
exact point pairs:       23,444 / 24,256
maximum ULP difference:  22
maximum absolute delta:  0.00048828125 px
first mismatch: frame 4, cam0, track 210, x
native 0x4382f3cb, Rust 0x4382f3cc
SHA-256: 1bb1c9d44de3c67babd91e112110f041250bc38b3b286769f1ed2ae4ee48efee
```

The temporary O2/O3 native binaries, trace source, Rust dump test, compare
script, and their outputs were removed.  No commit or push was made.
