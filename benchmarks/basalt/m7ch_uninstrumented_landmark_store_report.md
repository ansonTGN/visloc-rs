# M7ch uninstrumented landmark-store provenance report

Date: 2026-08-23 (JST)  
Scope: pinned Basalt 0f3b2b52c807f70ff4e2973ce253c73329eea7bc, existing
build/core-relwithdebinfo/basalt_vio/libbasalt.so, frame 0, track 1.

## Result

One bounded, uninstrumented GDB run with --max-frames 5 resolved the live
ASLR base, matched the target call at the direct call instruction and at
triangulate entry, and stopped after the actual DB write for landmark id 1.
The stored Keypoint packet was:

~~~text
direction.x = becaaef7
direction.y = be383810
inv_dist    = 3e160ef3
~~~

These bits are exactly the current Rust/standalone fixture.  They differ
from the separately retained instrumented-detail endpoint:

~~~text
instrumented detail: becaaef6 be38380d 3e160f08
~~~

Therefore, for this pinned existing native binary and this actual frame-0
track-1 call, the current Rust/standalone provenance is authoritative.  The
instrumented-detail triplet is not the actual stored landmark packet for
this binary/run; it belongs to a different artifact/context and must not
drive a production landmarks.rs/camera.rs patch.

No production Rust source, upstream source, native binary, fixture, or test
was changed.  No rebuild or instrumentation was performed.  The complete
capture is retained at
[target/m7ch_actual_gdb.out](../../target/m7ch_actual_gdb.out).  No commit
or push was made.

## Live addresses and target confirmation

The run printed:

~~~text
M7CH live libbasalt base = 0x7ffff6e00000
M7CH DIRECT CALL rdi=0x7ffff53542b0 rsi=0x7ffff5354320 rdx=0x7ffff5354400
M7CH MATCH ENTRY #23 rdi=0x7ffff53542b0 rsi=0x7ffff5354320 rdx=0x7ffff5354400
~~~

The requested live addresses were:

| offset | live address | role |
|---:|---:|---|
| +0x4d7400 | 0x7ffff72d7400 | triangulate entry |
| +0x50b613 | 0x7ffff730b613 | call argument setup |
| +0x50b668 | 0x7ffff730b668 | direct triangulate call |
| +0x4d7f3c | 0x7ffff72d7f3c | normalization start |
| +0x4d7f78 | 0x7ffff72d7f78 | normalization divide |
| +0x4d7fbb | 0x7ffff72d7fbb | post-sign/return path |
| +0x3e1802 | 0x7ffff71e1802 | addLandmark write-complete |

The filtered rays were:

~~~text
f0 = bf2a7307 be9af9d0 3f2e94d6
f1 = bf2d0c0d be93697d 3f2da937
~~~

They correspond to the known pixels (21,93) and
(29.425615310668945,107.3565444946289).

## Actual call and intermediate packets

The direct call and entry dumps agree on the pose packet:

~~~text
T_0_1 qxyzw = 3befaa96 39eadb91 3a8bfffa 3f7ffe35
T_0_1 t     = 3de1c150 b87a2400 39ac1888
~~~

The first normalization stop captured the same raw SVD vector as the
standalone fixture:

~~~text
V.col(3) = bf28a750 be994a0a 3f2cbdf9 3e147902
world    = bf2a746e be9aed26 3f2e9645 3e160ef3
~~~

The best-effort stack packet also contains the known P2 matrix (column-major
in memory, shown here row-major):

~~~text
3f7fffd3 3b0c6cef ba66c162 bde1c0f0
bb0b910f 3f7ff8d7 3c6facec 3997d306
3a6ef276 bc6fa4e5 3f7ff8f6 b9e136c3
~~~

The allowed stops did not retain a complete pre-SVD A matrix.  Its full
capture was not pursued as a separate run, per scope; the actual direct DB
store is the decisive endpoint for this provenance check.

## Direct landmark DB store

The breakpoint at +0x3e1802 is immediately after
LandmarkDatabase<float>::addLandmark has written the direction, inverse
distance, and host fields.  At that stop:

~~~text
id       = 1
db       = 0x5555558d3eb0
keypoint = 0x7fffe8005e30
input    = 0x7ffff5354500

stored keypoint words:
becaaef7 be383810 3e160ef3 00000000

input keypoint words:
becaaef7 be383810 3e160ef3 3f365a1e
~~~

The first three words are the actual DB fields
Keypoint.direction.x, Keypoint.direction.y, and Keypoint.inv_dist.  The
stored object and the input object passed by measure agree exactly in those
fields, proving that the packet was not changed by the map insertion.

## Provenance conclusion and first differing scalar

For this uninstrumented pinned binary the complete observed chain is:

~~~text
target rays -> direct T_0_1 call -> V -> normalized world
     exact          exact            exact       exact
                                      |
                                      v
                         DB direction/rho = standalone
~~~

Against the current Rust/standalone fixture, there is no divergence through
the stored landmark.  Against the instrumented-detail triplet, the first
different scalar is the stored direction.x:

~~~text
pinned uninstrumented DB: becaaef7
instrumented detail:      becaaef6
~~~

The following y and inverse-distance differences are likewise already
present at the stored landmark.  This resolves the provenance question for
the pinned binary: the standalone/current-Rust packet, not the instrumented
detail packet, is authoritative for this actual call.  No general compiler,
Eigen, landmarks.rs, or camera.rs change is justified.

The inferior exited normally and the final process check found no running
gdb or basalt_vio process.
