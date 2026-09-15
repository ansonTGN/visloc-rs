# M7dd clean `computeRelPose` output capture

Date: 2026-08-23 JST  
Status: **Pass — one bounded clean GDB run captured the requested post-store.**

## Result

The clean pinned Basalt binary was run with the already validated M7db
dataset/config/marg-data arguments.  A dynamic ASLR base was derived from the
live `/proc/<inferior>/maps` `libbasalt.so` mapping.  The GDB breakpoint was
set at `computeRelPose<float>` ELF offset `0x2bc930`, filtered by the exact
iteration-0 host/target quaternion words and the four TimeCam key words, then
stopped at post-store ELF offset `0x2bce0b`.

The capture was for frame 4, iteration 0, track 1, host `(frame 0, cam 0)` to
target `(frame 1, cam 1)`.  The only output inspected at the stop was raw
`x/7wx $rdi` (sret), in `q xyzw` followed by `t xyz` order:

```text
bc0e2957 bb507f02 ba002d40 3f7ffd31 bde1e255 baf1ec60 ba884368
```

This is an exact 7/7 binary32 match to the authoritative m7bo diagnostic
packet.  Against the current Rust m7cm reference, only `q.w` is equal; the
other six lanes differ (`q.x`, `q.y`, `q.z`, `t.x`, `t.y`, `t.z`).

## Provenance

* Clean commit: `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.
* `basalt_vio` SHA-256:
  `89e0324ccd04c3b615bf7e945aa32480a2f86524987aa00eaf152dcd6b1d242c`.
* `libbasalt.so` SHA-256:
  `55f842642b370edf4025afd1d66c7b9d9011e5b4ce08efeb9d76be401b45b2cc`.
* Raw GDB output: `target/m7dd_clean_relpose.gdb.out` (3888 bytes;
  SHA-256 `9e96fb169a26d90ee9ea0e3b9eb9efbc3df6e58452c2072acc1929d2d1644d9f`).
* Machine-readable record:
  [`target/m7dd_clean_relpose.json`](../../target/m7dd_clean_relpose.json).

The run recorded three computeRelPose entries before the matching entry and
quit immediately after the matching post-store.  No source, binary, build,
commit, or push was changed.  `linearizePoint` and Jacobians were deliberately
not traced.
