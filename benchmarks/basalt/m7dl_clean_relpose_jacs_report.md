# M7dl clean `computeRelPose` raw Jacobian-buffer capture

Date: 2026-08-23 JST  
Status: **Pass — one bounded clean GDB run reached the exact requested post-store.**

## Scope and method

The run used the unchanged pinned clean artifact from m7cw/m7dd:

* commit `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`;
* `basalt_vio` SHA-256
  `89e0324ccd04c3b615bf7e945aa32480a2f86524987aa00eaf152dcd6b1d242c`;
* `libbasalt.so` SHA-256
  `55f842642b370edf4025afd1d66c7b9d9011e5b4ce08efeb9d76be401b45b2cc`.

The command is a copy of the validated m7dd GDB command:
[`target/m7dl_clean_relpose_jacs.cmd`](../../target/m7dl_clean_relpose_jacs.cmd).
At the exact filtered `computeRelPose<float>` entry, it saves the ABI output
pointers in GDB convenience variables before the callee clobbers its argument
registers:

* `$r9` -> `$m7dl_d_h` (`d_rel_d_h`);
* `*(uint64_t*)($rsp+8)` -> `$m7dl_d_t` (`d_rel_d_t`);
* `$rdi` -> `$m7dl_sret` (q/t output).

The single bounded run reached the exact frame-4 / iteration-0 / track-1
factor (host frame 0 cam 0 to target frame 1 cam 1), then stopped at dynamic
address `0x74d493ebce0b`, which is ELF offset `0x2bce0b`. The raw output is
[`target/m7dl_clean_relpose_jacs.gdb.out`](../../target/m7dl_clean_relpose_jacs.gdb.out)
(6,177 bytes; SHA-256
`e7548d92aeea2f2206e9e19a81a95a94e6c6d932eae80f6b26cf5ba96ac3d290`).

## q/t result

The raw `x/7wx` q/t words are:

```text
bc0e2957 bb507f02 ba002d40 3f7ffd31 bde1e255 baf1ec60 ba884368
```

They are an exact 7/7 binary32 match to the m7bo clean relative-pose packet.
Against the current Rust m7cm reference, only q.w is bit-equal (1/7); the
other six lanes differ as recorded in the machine-readable artifact
[`target/m7dl_clean_relpose_jacs.json`](../../target/m7dl_clean_relpose_jacs.json).

## Raw 6x6 buffers at the requested stop

The required raw commands were issued exactly as `x/36wx` for both buffers.
Both pointers were retained correctly:

```text
d_rel_d_h = 0x74d48401de00
d_rel_d_t = 0x74d48401de90
```

Each 36-word Eigen column-major sequence is:

```text
7fc00000 (repeated 36 times)
```

`0x7fc00000` is canonical binary32 quiet NaN. This is expected at this
boundary: ELF `0x2bce0b` is the `test %r9,%r9` immediately after the complete
q/t stores and before the d_rel_d_h Jacobian branch. The clean function writes
the two output matrices later in that branch. Therefore the exact requested
post-store capture proves the ABI pointers and records the buffers' actual
state, but it is not a post-Jacobian matrix-value capture.

## Rust comparison

`target/m7cm_fresh5_detail_20260823.jsonl` contains per-observation
`jp_target`, `jp_anchor`, and aggregate `state_jacobian` fields. Those are
weighted absolute-pose/state blocks and do not serialize the raw relative-pose
`d_rel_d_h` / `d_rel_d_t` 6x6 buffers. No direct matrix lane comparison is
claimed. The q/t comparison and all raw words are recorded in the JSON
artifact.

No source, clean checkout, binary, build, commit, or push was changed. The
inferior and all Basalt processes were gone after the capture.
