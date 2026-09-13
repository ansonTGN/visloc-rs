# M7EB clean same-timestamp stereo `computeRelPose` q/t capture

Date: 2026-08-23 JST  
Status: **Pass — one bounded GDB run captured the requested post-store.**

## Result

The successful M7DD command was copied to
[`target/m7eb_stereo_qt.cmd`](../../target/m7eb_stereo_qt.cmd) and its filter
was narrowed to the same-timestamp frame-0 stereo factor:

```text
host:   frame 1403636579763555584, cam 0
target: frame 1403636579763555584, cam 1
```

The run used the existing clean pinned Basalt binary (no rebuild).  The
dynamic ASLR base was read from the live `libbasalt.so` mapping.  The entry
breakpoint was ELF offset `0x2bc930`; the post-store breakpoint was
`0x2bce0b`, after all seven q/t stores and before any Jacobian work.

The exact raw post-store packet is q `xyzw` followed by t `xyz`, seven
contiguous binary32 words:

```text
bbefaaa9 b9eadbb0 ba8bff3a 3f7ffe35 bde1c0f0 3997d300 b9e136c8
```

Decoded values:

```text
q_xyzw = [-0.007314045447856188, -0.0004479563795030117,
          -0.0010680921841412783, 0.9999726414680481]
t_xyz  = [-0.11023128032684326, 0.00028958171606063843,
          -0.00042956159450113773]
```

This is the camera-to-camera transform returned by `computeRelPose`; it is
landmark-independent.  The runtime track id (`2`) is only the witness that
selected the call and is not an input to the transform.  No landmark,
`linearizePoint`, or Jacobian buffer was inspected.

## Provenance and artifacts

* Existing clean checkout: `/root/visloc-basalt-clean-m7cr-20260823`
* Existing clean commit: `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`
* `basalt_vio` SHA-256: `89e0324ccd04c3b615bf7e945aa32480a2f86524987aa00eaf152dcd6b1d242c`
* `libbasalt.so` SHA-256: `55f842642b370edf4025afd1d66c7b9d9011e5b4ce08efeb9d76be401b45b2cc`
* Command SHA-256: `a4aaa7a3b6b38485a256f48a90cfd8a5885bf80c774782ff0d4ec734345941fc`
* Raw GDB output: [`target/m7eb_stereo_qt.gdb.out`](../../target/m7eb_stereo_qt.gdb.out), 4015 bytes, SHA-256 `60f1c9da2918fcb779e82d76fd65292786f6600bde63409e72504c41f7da7a92`
* Machine-readable record: [`target/m7eb_stereo_qt.json`](../../target/m7eb_stereo_qt.json)

The run matched the same clean-binary same-timestamp capture retained from
the preceding M7DY probe lane-for-lane (7/7).  No production or clean source,
binary, or build state was changed; no commit or push was performed.
