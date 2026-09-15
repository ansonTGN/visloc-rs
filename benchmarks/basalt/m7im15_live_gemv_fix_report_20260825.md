# M7IM15 live GEMV-fix native local-product capture (2026-08-25)

This is a fresh, bounded native capture for Rust `frame_id=4`,
`iteration=0`. The native executable was built from detached worktree
`target/m7im15-native-logger-20260825` at pinned commit
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc`. The only native source edit is an
environment-gated diagnostic in
`include/basalt/linearization/imu_block.hpp`, immediately after production
`const H = Jp.transpose() * Jp` and `const b = Jp.transpose() * r`. It records
the production `Jp`, `r`, `H`, and `b` as little-endian binary32 bits. No
production arithmetic was changed, no commit was made, and the historical
`m7im15_native_local_capture_frame6_iter6_20260825.json` and reconstructed
native oracle were not used.

## Build and bounded run

Build command:

```text
wsl.exe bash -lc 'cmake --build /mnt/c/Users/rsasa/Workspace/visloc-rs/target/m7im15-native-logger-20260825/build/core-relwithdebinfo --target basalt_vio -j2'
```

Run command (one process, `num_threads=1`, `max_frames=5`):

```text
wsl.exe bash -lc 'M7IM15_NATIVE_INPUT_LOG=/mnt/c/Users/rsasa/Workspace/visloc-rs/target/m7im15_native_live_gemv_fix_input_20260825.bin M7IM15_NATIVE_INPUT_LOCAL_BIN=/mnt/c/Users/rsasa/Workspace/visloc-rs/target/m7im15_native_live_gemv_fix_input_local_20260825.bin M7IM15_NATIVE_INPUT_CALL_ORDINALS=0 M7IM15_NATIVE_LOCAL_PRODUCT_BIN=/mnt/c/Users/rsasa/Workspace/visloc-rs/target/m7im15_native_local_product_frame4_iter0_20260825.bin M7IM15_NATIVE_LOCAL_PRODUCT_CALL_ORDINALS=0,1,2,3 /mnt/c/Users/rsasa/Workspace/visloc-rs/target/m7im15-native-logger-20260825/build/core-relwithdebinfo/basalt_vio --dataset-path /mnt/e/datasets/euroc_mav/machine_hall/MH_01_easy --cam-calib /mnt/c/Users/rsasa/Workspace/visloc-rs/target/m7im15-native-logger-20260825/data/euroc_ds_calib.json --dataset-type euroc --config-path /mnt/c/Users/rsasa/Workspace/visloc-rs/target/m7im15-native-logger-20260825/data/euroc_config.json --marg-data /marg_data_m7he_local_hb --show-gui 0 --save-trajectory euroc --num-threads 1 --max-frames 5'
```

The local-product binary contains exactly four records (ordinals 0, 1, 2,
3), each with `Jp 15x30`, `r 15`, `H 30x30`, and `b 30`, all in Eigen
column-major order. The fresh `M7IM15INP2` local-input binary contains exactly
one record (ordinal 0), and the fresh `M7IM15INP1` input binary contains one
record for the same start timestamp.

## Fresh-boundary gates

The current fresh native local-input record passes all requested ordinal-0
boundary comparisons against Rust: raw `r` 9/9, raw `J` 270/270, `W` 81/81,
wide `W*raw r` 9/9, and wide `W*raw J` 270/270. The direct production local
product record 0 also has `r[0:9]` equal to the current fresh native
whitened-residual `r[0:9]` 9/9, and equal to Rust's residual `r[0:9]` 9/9.

## Direct production local-product comparison

Counts are native bit-exact elements over Rust's four `local_blocks`; the
first mismatch is a zero-based column-major flat index.

| block | start timestamp | `Jp` | `r` | `H` | `b` |
|---:|---:|---:|---:|---:|---:|
| 0 | 1403636579763555584 | 431/450; index 123: `41f442f6` vs `41f442f7` | 15/15 | 752/900; index 8: `bf6ba7c6` vs `bfcbb184` | 27/30; index 8: `3b781dc5` vs `3b781dc7` |
| 1 | 1403636579813555456 | 401/450; index 6: `c22a68c6` vs `c22a68c5` | 10/15; index 3: `b645a9cf` vs `b5f34710` | 577/900; index 1: `4116b12b` vs `4116b138` | 7/30; index 0: `3ed7df97` vs `3ed6dcd1` |
| 2 | 1403636579863555584 | 419/450; index 47: `41465719` vs `4146571a` | 6/15; index 0: `b60bc0d4` vs `b62eb109` | 685/900; index 3: `3faa69b2` vs `3fa8e9b2` | 6/30; index 0: `3db32671` vs `3e618883` |
| 3 | 1403636579913555456 | 381/450; index 8: `c00ef804` vs `c00ef803` | 6/15; index 0: `b58bc09b` vs `b50bc09b` | 520/900; index 1: `c082747c` vs `c004e887` | 6/30; index 0: `bf29ab6c` vs `befc4e4c` |

Aggregate counts are `Jp 1632/1800`, `r 37/60`, `H 2534/3600`, and
`b 46/120`. The first direct-product mismatch in scan order is block 0,
`Jp`, flat index 123 (`41f442f6` native versus `41f442f7` Rust).

The wide ordinal-0 diagnostic `W*rawJ` is exact 270/270, while the native
production `Jp` block 0 differs at index 123. This is consistent with (but
does not prove) a difference between Eigen's blockwise fixed-size
`W*d_res` assignments used by production `Jp` and the wide diagnostic's
`W*rawJ` schedule. Blocks 1–3 have not had fresh native input ordinals 1–3
captured; their residual/J differences therefore must not yet be attributed
to a whitening schedule without that input-side capture.

## Artifacts and SHA-256

The updated comparison is
`target/m7im15_live_gemv_fix_compare_20260825.json`; its provenance records
the direct product JSON and explicitly marks the reconstructed oracle as not
used. The product parser is
`work/m7im15_parse_native_local_product_20260825.py`.

| artifact | SHA-256 |
|---|---|
| detached `basalt_vio` | `438B9AE448F5F977D7305B28106F8C3A0A8870CAB6A92A29E3A31D6E0CFA3E8F` |
| detached `libbasalt.so` | `4EE2BAF3CBF0539376A89FE4FFBB87A0F0A43E4D42132F5D2A74BD319572EA0A` |
| `target/m7im15_native_local_product_frame4_iter0_20260825.bin` | `DAFBB85689165DBB1AF519118B51F0B19537C0E5A91DC352128C408DE3F80223` |
| `target/m7im15_native_local_product_frame4_iter0_20260825.json` | `6179FFE5D23A7E48840422706A17586E72CB77A8ADB4A9EF3B7F98E4D160DA54` |
| `target/m7im15_native_live_gemv_fix_input_local_20260825.bin` | `90FB1CD9DD889110F72B6F30C9F235106B961FD8F179191493035DCC25140A51` |
| `target/m7im15_native_live_gemv_fix_input_20260825.bin` | `41FFE300003316BFE4285029AEDE065EAE4E555D337BB07C781F22D908200A5E` |
| `target/m7im15_live_gemv_fix_compare_20260825.json` | `DAE201968FE965F8C657A0E1515D9590F21B7F50E16CED67BD49751BE58E8C38` |
| `target/m7im15_rust_live_gemv_fix_v3_20260825.jsonl` | `A2DF8B73A4FC05168D9442FB1B6CEF518C75928C517734CE80340F8C2568A022` |
