# M7ds clean `computeRelPose` raw Jacobian capture

Date: 2026-08-23 JST  
Status: **Pass — the existing bounded m7dp capture reached both Jacobian writes.**

## Result

The existing output in
[`target/m7dp_clean_relpose_jacs.gdb.out`](../../target/m7dp_clean_relpose_jacs.gdb.out)
is a successful post-write capture. It matched the requested frame-4 /
iteration-0 / track-1 factor (host frame 0 cam 0 to target frame 1 cam 1),
with:

* `computeRelPose` entry at live `0x7e2e926bc930` (ELF offset `0x2bc930`);
* final `d_rel_d_t` store at ELF offset `0x2bd671`;
* completion stop at live `0x7e2e926bd679` (ELF offset `0x2bd679`), immediately
  after that store and before the epilogue/return.

The saved ABI pointers were valid throughout the capture:

```text
sret      = 0x7e2e908f78c0
d_rel_d_h = 0x7e2e8401de00
d_rel_d_t = 0x7e2e8401de90
```

Both matrices are therefore captured after their writes. Each has 36/36
finite binary32 words and zero NaN words. The machine-readable record is
[`target/m7ds_clean_relpose_jacs.json`](../../target/m7ds_clean_relpose_jacs.json).

## q/t result

The raw q/t words are:

```text
bc0e2957 bb507f02 ba002d40 3f7ffd31
bde1e255 baf1ec60 ba884368
```

They are an exact 7/7 binary32 match to the authoritative m7bo runtime tuple.
Against the retained Rust m7cm tuple, only q.w is bit-equal; q.x, q.y, q.z,
and all three translation lanes differ.

## Raw Jacobian buffers

The required `x/36wx` dumps use Eigen column-major order:
`[r0c0,r1c0,...,r5c0,r0c1,...,r5c5]`.

`d_rel_d_h` (`0x7e2e8401de00`):

```text
3dda79b6 3e8b3905 bf74d5e7 00000000 00000000 00000000
3f7e5132 bd8df611 3dba92eb 00000000 00000000 00000000
bd2a12f8 bf75b6c9 be8e17f1 00000000 00000000 00000000
3c93c295 bd226d6d bc17c309 3dda79b6 3e8b3905 bf74d5e7
baf80708 b9828894 3ca77d77 3f7e5132 bd8df611 3dba92eb
3a8bd263 bc37c50e 3d1e3cc5 bd2a12f8 bf75b6c9 be8e17f1
```

`d_rel_d_t` (`0x7e2e8401de90`):

```text
bdda79af be8b3907 3f74d5e9 00000000 00000000 00000000
bf7e5131 3d8df601 bdba92eb 00000000 00000000 80000000
3d2a12d7 3f75b6ca 3e8e17f1 00000000 00000000 00000000
bc833104 3d223cf7 3c1b3e27 bdda79af be8b3907 3f74d5e9
3addb39c 388626ba bc96b372 bf7e5131 3d8df601 bdba92eb
ba312e45 3c37c62b bd1e7b0f 3d2a12d7 3f75b6ca 3e8e17f1
```

`0x80000000` at column 1 / row 5 is a signed negative zero and is retained
as captured; it is not normalized away.

## Rust comparison

The retained Rust m7cm detail serializes weighted absolute-pose/state blocks
(`jp_anchor`, `jp_target`, and aggregate `state_jacobian`), not the raw
`computeRelPose` `d_rel_d_h` / `d_rel_d_t` buffers. Consequently there is no
direct Rust 6x6 lane comparison to claim. The q/t comparison is recorded in
the JSON artifact, including the 1/7 Rust bit-equality result.

For context only, the retained pinned native m7aq pose probe has 21/36 equal
`d_rel_d_h` lanes and 36/36 equal `d_rel_d_t` lanes. That probe uses the
m7cm/native relative-pose packet and is not a Rust comparison or a substitute
for the m7dp live clean capture.

No source, clean checkout, binary, or build was changed. No new run was
started for this record; the artifacts were parsed from the existing m7dp
output. No commit or push was performed.
