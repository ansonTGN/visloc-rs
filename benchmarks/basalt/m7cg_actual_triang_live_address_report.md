# M7cg actual-call triangulation live-address report

Date: 2026-08-23 (JST)  
Scope: pinned Basalt 0f3b2b52c807f70ff4e2973ce253c73329eea7bc, existing
build/core-relwithdebinfo/basalt_vio/libbasalt.so, frame 0, track 1.

## Result

One bounded, uninstrumented GDB run was completed with --max-frames 5.
The run resolved the ASLR load base inside GDB and hit the requested
triangulation entry for the target ray pair:

~~~text
M7CG live libbasalt base = 0x7ffff6e00000
M7CG MATCH ENTRY #23
~~~

This is the correct call: the entry arguments contain the exact frame-0
track-1 rays and the exact known T_0_1 packet, and the three internal
normalization breakpoints subsequently hit for that invocation.  The
pre-SVD A packet was not captured at the allowed stops.  The captured
sequence has no observed bit divergence through the final normalized world
point; the first exact divergence against the previously recorded
authoritative endpoint therefore remains outside this capture (or in an
unobserved A/SVD boundary), rather than being evidence for a production
camera/landmark patch.

No production Rust source, upstream source, or native binary was changed,
rebuilt, or instrumented.  No fixture/test was added.  The capture is
retained at [target/m7cg_actual_gdb.out](../../target/m7cg_actual_gdb.out).
The temporary GDB command file was removed after the run; no commit or push
was made.

## Live breakpoint addresses

The base and addresses printed by the run were:

| requested offset | live address | purpose |
|---:|---:|---|
| +0x4d7400 | 0x7ffff72d7400 | triangulate entry |
| +0x50b613 | 0x7ffff730b613 | call argument setup |
| +0x50b668 | 0x7ffff730b668 | call instruction |
| +0x4d7f3c | 0x7ffff72d7f3c | normalization start |
| +0x4d7f78 | 0x7ffff72d7f78 | raw-vector divide |
| +0x4d7fbb | 0x7ffff72d7fbb | post-sign/return path |

At the matching entry the SysV arguments were:

~~~text
rdi = 0x7ffff53542b0   f0 object
rsi = 0x7ffff5354320   f1 object
rdx = 0x7ffff5354400   T_0_1 object
~~~

The filtered data words were exactly:

~~~text
f0 = bf2a7307 be9af9d0 3f2e94d6 00000000
f1 = bf2d0c0d be93697d 3f2da937 00000000
~~~

These correspond to the known pixels (21,93) and
(29.425615310668945,107.3565444946289) with bits
41a80000 42ba0000 and 41eb67a9 42d6b68d.

## Actual packet comparison

### T_0_1

The actual entry dump begins:

~~~text
qxyzw = 3befaa96 39eadb91 3a8bfffa 3f7ffe35
t      = 3de1c150 b87a2400 39ac1888
~~~

All seven compared T_0_1 values are bit-exact with the m7by known
fixture.  The remaining words in the 16-word diagnostic dump include object
storage/padding and were not treated as additional pose fields.

### P2

At the first normalization stop, the stack packet at rsp+0xa0 through
rsp+0xcc is column-major.  Reordered to the fixture's row-major form, the
actual values are:

~~~text
3f7fffd3 3b0c6cef ba66c162 bde1c0f0
bb0b910f 3f7ff8d7 3c6facec 3997d306
3a6ef276 bc6fa4e5 3f7ff8f6 b9e136c3
~~~

This is bit-exact with the known P2 packet.

### A

No complete actual 4x4 A packet is available from this run.  The allowed
normalization stops occur after the SVD work has reused the matrix storage;
the full rsp dump at +0x4d7f3c contains no complete copy of the known
fixture's A values.  In particular, the known standalone rows

~~~text
bf2e94d6 80000000 bf2a7307 80000000
80000000 bf2e94d6 be9af9d0 80000000
bf2dd17a 3c0a2d1b bf2ce029 3d99bcd8
3a9af4a4 bf2c905f be987a21 b898995a
~~~

were not asserted as actual-call bytes.  A pre-SVD stop (for example at the
matrix handoff before the Jacobi SVD call) would be required to close this
boundary; it was not run in this task.

### V.col(3), normalization, and sign

The raw SVD vector at rsp+0x150 was:

~~~text
bf28a750 be994a0a 3f2cbdf9 3e147902
~~~

It is bit-exact with the known V.col(3).  The final stop at +0x4d7fbb
was the fall-through positive-ray path (the negative-sign branch was not
taken).  The final world packet at rsp+0x60 was:

~~~text
bf2a746e be9aed26 3f2e9645 3e160ef3
~~~

This is bit-exact with the known normalized world vector.  The intermediate
+0x4d7f78 stop is before its vdivps instruction, so its printed xmm1
is still the raw V packet; the final stack packet at +0x4d7fbb is the
authoritative normalized result from this capture.

## Call-site filter status and next filter

The setup and call breakpoints were installed at the requested live
addresses, but the output contains neither M7CG MATCH CALL-SETUP nor
M7CG MATCH CALL before PLT.  Their conditional output depended on the
pre-move $r14, [$rbp-0x560], $rbx packet and a $site_match latch.
The entry match proves that the target invocation did execute, so this is a
filter/argument-timing miss, not evidence that triangulation was skipped.

If another probe is authorized, the next filter should inspect the direct
post-move registers at +0x50b668 (or immediately after the moves at
+0x50b620): $rdi, $rsi, and $rdx, dereferencing the two ray block
objects there, without requiring the setup latch.  A separate pre-SVD
breakpoint is needed for A.  No such retry was performed here.

## First-divergence statement

For the captured invocation, the exact chain is:

~~~text
target rays -> T_0_1 -> P2 -> V.col(3) -> normalized world/sign
     exact       exact   exact     exact          exact
~~~

Thus this run does not identify an internal first differing scalar.  The
previous authoritative endpoint comparison remains the first observable
endpoint difference (becaaef6 vs standalone becaaef7 for direction x,
then be38380d vs be383810 for direction y, and 3e160f08 vs 3e160ef3 for
rho), but that downstream endpoint was not produced by a newly added
breakpoint in this run.  Since A was not captured and the post-return
stereographic code was not stopped, no general compiler/Eigen order is
proven and no landmarks.rs/camera.rs patch is justified.

The GDB inferior exited normally, and a final process check found no running
gdb or basalt_vio process.
