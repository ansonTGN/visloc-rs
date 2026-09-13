# M7cu clean uninstrumented landmark oracle

Date: 2026-08-23 (JST)  
Scope: the separate clean M7cr Basalt core RelWithDebInfo checkout at
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc`, its `basalt_vio` and
`libbasalt.so` only, MH_01_easy, and one bounded `--max-frames 5` GDB run.

## Result

**Pass: 61/61 clean frame-0 landmark stores are captured and agree at
binary32 precision with every retained reference.** The write-complete
stream has 61 unique track IDs. Every captured record has the frame-0 host
timestamp `1403636579763555584` and host camera 0. The command list disabled
the breakpoint at record 61, and the clean five-frame smoke retained 61 active
frame-0 landmarks before the inferior exited normally.

The machine-readable capture is
[target/m7cu_clean_landmarks.json](../../target/m7cu_clean_landmarks.json),
SHA-256 `1ab6c55d2b57766cafef4384b2655438f89939f9abefa956d7cf655f04cd893d`.
The raw bounded capture is
[target/m7cu_clean_gdb.out](../../target/m7cu_clean_gdb.out), SHA-256:

~~~text
701ef095def935b7d81d7d1a163c93a2b4c5154dddfe4abe813551c785303dae
~~~

All comparisons use the native stored `direction.x`, `direction.y`, and
`inv_dist` words. Rust detail values are f64 in the JSONL files and are cast
to IEEE-754 binary32 before comparing u32 bits.

| reference | IDs | direction.x | direction.y | rho | complete triples |
|---|---:|---:|---:|---:|---:|
| diagnostic m7ck | 61/61 | 61/61 | 61/61 | 61/61 | 61/61 |
| current Rust m7bt | 61/61 | 61/61 | 61/61 | 61/61 | 61/61 |
| current Rust m7cm | 61/61 | 61/61 | 61/61 | 61/61 | 61/61 |

There is no first difference against any of the three references. The
diagnostic oracle source hash is
`793b9095e34124c0b036d3e3caa4761a92a22957c31e3c37cb16c15e3f475a9d`.
The m7bt and m7cm detail hashes are respectively
`7119fa451b9b9d5c26766a46e19bab6a6198257c4b5dd4c8f5e2689658cd32af` and
`978c79b646232080d37697dc66fb25bfeb216578b0fe9f4c89403e93de1c1d1c`.

## Clean artifact and marker verification

The clean M7cr artifact identity is:

~~~text
basalt_vio SHA-256: 89e0324ccd04c3b615bf7e945aa32480a2f86524987aa00eaf152dcd6b1d242c
basalt_vio MD5:    f97e14f0654e47ed0e8a7fb77ef102af
basalt_vio size:   18024832 bytes
basalt_vio Build ID: 5d10252dc9e249bc228a4c25cdc11cc68d8944d8

libbasalt.so SHA-256: 55f842642b370edf4025afd1d66c7b9d9011e5b4ce08efeb9d76be401b45b2cc
libbasalt.so MD5:    a35a93ef059efe6bdc7a953d20a098fd
libbasalt.so size:   421516504 bytes
libbasalt.so Build ID: cd04ec0e0ef75097c5a2e8c3bbc4cd9e719be362
~~~

The clean source checkout and M7cr manifest report no
`BA_REL_DIAG`, `RUNTIME_REL`, `BASALT_TRACE_JSONL`, or
`BASALT_IMU_TRACE_JSONL` source matches. A fresh byte scan of both clean
binary artifacts also found no occurrence of any of those four markers.

## Dynamic GDB resolution and capture boundary

The GDB script resolved the exported symbol at runtime:

~~~text
symbol: basalt::LandmarkDatabase<float>::addLandmark(unsigned long, basalt::Keypoint<float> const&)
file symbol address: 0x3bf950
live symbol address:  0x7ffff71bf950
live lib base:        0x7ffff6e00000
write-complete stop:  symbol +0x192 = 0x7ffff71bfae2
~~~

In the clean disassembly, the final `inv_dist` store is at file address
`0x3bfadd`; the breakpoint is the following instruction, after direction,
rho, and host fields have been written. At that stop, `$rbx` is the stored
Keypoint and `$rbp` is the track ID. The capture command list emitted
`M7CU_REC` for each store, disabled itself at record 61, and then let the
normal max-5 run finish.

This is an uninstrumented native DB boundary check. No source marker,
production Rust change, native rebuild, commit, or push was made. The GDB
and inferior processes were stopped; the inferior exited normally.
