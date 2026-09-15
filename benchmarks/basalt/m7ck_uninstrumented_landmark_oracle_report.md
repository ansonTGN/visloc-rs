# M7ck uninstrumented landmark oracle report

Date: 2026-08-23 (JST)  
Scope: pinned Basalt 0f3b2b52c807f70ff4e2973ce253c73329eea7bc, existing
core-relwithdebinfo native binary, MH_01_easy, max5.

## Result

The complete uninstrumented LandmarkDatabase<float>::addLandmark write
stream contains 61 unique records.  Every record has the frame-0 host
timestamp 1403636579763555584 and host camera 0; there were no additional
landmark writes through frame 4.  This exactly defines the 61 active
frame-0-created stereo landmarks expected by the frame-4 reference.

The machine-readable oracle is
[target/m7ck_uninstrumented_landmarks.json](../../target/m7ck_uninstrumented_landmarks.json).
Its SHA-256 is:

~~~text
793b9095e34124c0b036d3e3caa4761a92a22957c31e3c37cb16c15e3f475a9d
~~~

Coverage and comparison against the retained current-Rust detail are:

| quantity | result |
|---|---:|
| native write records | 61 |
| unique native track IDs | 61 |
| m7bt reference landmarks | 61 |
| ID intersection | 61 |
| missing native IDs | 0 |
| missing reference IDs | 0 |
| exact direction.x bits | 61/61 |
| exact direction.y bits | 61/61 |
| exact inverse-distance bits | 61/61 |
| exact complete direction/rho triples | 61/61 |

The comparison casts the m7bt f64 landmark fields to IEEE-754 binary32 and
compares their u32 bits with the native DB fields.  There is no first
difference in this 61-landmark set.

## Native capture provenance

The capture used the existing pinned binary and library without source
instrumentation or rebuilding:

~~~text
binary md5: a71e7212670956057066bbcc8dc203bc
lib md5:    eaa1b52affd00b418a5077fc408e2e0f
live base:  0x7ffff6e00000
breakpoint: libbasalt.so +0x3e1802
            LandmarkDatabase<float>::addLandmark write-complete
max frames: 5
~~~

The raw GDB capture is
[target/m7ck_actual_gdb.out](../../target/m7ck_actual_gdb.out), SHA-256:

~~~text
59a73ac55fcbcab187c6e2bbf6bae552ceb2eac768707368681c48630659fbf4
~~~

At +0x3e1802, the native rbx points to the stored Keypoint<float> in
the unordered-map node after direction, inverse distance, and host fields
have been written.  The captured records store:

~~~text
track_id
host_frame_index (mapped from the reference's frame index)
native TimeCamId frame_id / timestamp_ns
host_cam
direction.x u32 bits
direction.y u32 bits
inv_dist u32 bits
~~~

The native TimeCamId.frame_id field is the timestamp
1403636579763555584; the corresponding dataset/reference frame index is
0.  IDs are unique and the creation ordinal is retained in the JSON.

## Reference and exactness boundary

The comparison reference is:

~~~text
target/m7bt_composition_fresh5_detail_20260823.jsonl
record: snapshot
phase: iteration_start
frame_id: 4
reference sha256:
7119fa451b9b9d5c26766a46e19bab6a6198257c4b5dd4c8f5e2689658cd32af
~~~

The target track 1 record is one representative direct check:

~~~text
native stored: becaaef7 be383810 3e160ef3
m7bt f32:      becaaef7 be383810 3e160ef3
~~~

The prior m7ch direct-store probe established the same actual DB boundary
for track 1.  This m7ck run extends that evidence to every frame-0
landmark creation, not just one selected track.

## Superseded claims

This corrected uninstrumented oracle supersedes the earlier instrumented
detail exactness claims:

- m7bs reported complete triples exact for 39/61.
- m7bx reported complete triples exact for 1/61 under a different
  authoritative-detail comparison.

Those numbers describe codegen/context-perturbed or artifact-mismatched
detail comparisons; they are not the stored values from the pinned
uninstrumented native binary.  The direct native DB store proves 61/61
against the retained m7bt current-Rust detail at binary32 precision.

Consequently, the authoritative provenance for the pinned binary's
frame-0-created landmarks is the uninstrumented oracle in
m7ck_uninstrumented_landmarks.json, matching the current Rust/standalone
packet.  No production source change is justified.

No production source, upstream source, binary, manifest, or test was
changed.  No rebuild, commit, or push was performed.  The temporary GDB
script was removed, and the native/GDB processes were stopped after the
inferior exited normally.
