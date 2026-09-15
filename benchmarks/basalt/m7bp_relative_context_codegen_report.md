# M7bp relative-pose call-site code-generation audit

Date: 2026-08-23 JST  
Status: call-site arithmetic isolated; no speculative production change.

## Decision

The authoritative tuple in `m7bo_relative_oracle_provenance_report.md` is
emitted by the second, no-Jacobian `computeRelPose<float>` path in
`LinearizationAbsQR::linearizeProblem`.  It is reached after the
`isLinearized()` branch and is inlined into the caller.  The standalone
`computeRelPose` symbol, and a direct probe calling that symbol, remain the
local tuple (`bde1e254,baf1ecc0,ba884364`;
`bc0e2956,bb507eff,ba002940,3f7ffd31`).

No fixture/track branch or unverified arithmetic was added to Rust.  The
existing audited `F32Pose` path and M7 assertions are unchanged.

## Assembly context

The diagnostic upstream build is the GCC 11.4.0, `-O3 -g -march=native`
RelWithDebInfo build recorded in the m7bo report.  In the disassembly of
`LinearizationAbsQR<float,6>::linearizeProblem`, the Jacobian-bearing call is
at `0x2e6003`.  When either pose is linearized, control reaches `0x2e69d8`
and jumps back to the inlined no-Jacobian body at `0x2e6056`.

The inlined body uses Eigen `Packet4f` operations.  The relative quaternion
product and packet normalization occupy approximately `0x2e6208`–`0x2e62c4`:
`vpermilps`, packed multiply, contracted `vfmsub/vfmadd/vfnmadd`, lane blend,
then pairwise packet norm reduction (`vmovhlps`, `vmovshdup`, `vaddps`,
`vsqrtss`, `vdivps`).  The same source-level quaternion product compiled as
a separate direct function has a different register lifetime and FMA
reassociation.  Thus the difference is call-site code generation/inlining
context, not a different input snapshot or calibration.

## Reproduction boundary

The Eigen packet source reduction was decoded and evaluated independently.
Its straightforward scalar spelling produced
`bc0e2970,bb507ef4,ba002c00,3f7ffd32` for the final quaternion on the pinned
operands, rather than the captured inlined tuple
`bc0e2957,bb507f02,ba002d40,3f7ffd31`.  Because that candidate does not
reproduce the authoritative transform, it was removed rather than promoted
to a general Rust implementation.

The authoritative downstream comparison remains the m7bo table: the local
target point/projection/raw residual are
`bf2fc73d,be94aaaa,3f2ec6b2` /
`41da6d22,42d70d2e` / `bc228000,3f915340`, while the captured call-site
transform replays to `bf2fc73e,be94aaa8,3f2ec6b2` /
`41da6d17,42d70d32` / `bc22d800,3f915440`.

## Verification

The Windows checkout has no `cargo` executable, and the WSL environment has
no `cargo` installation, so focused/full Rust tests could not be run in this
environment.  No source or fixture changes were left from the rejected
candidate.
