# M7 IMU factor input frontier (Luna, 2026-08-25)

The Rust-only live boundary diagnostic is implemented and exercised for the
frame-4 solve, `from_index=0` (the active window link is frame 0→1). It records
raw residual (9), raw Jacobian (9×30), f32 square-root information (9×9), and
the two whitened products, all as exact binary32 bits, plus frame/state/delta
bits. The active-frame filter uses the window's latest frame id, so it does
not accidentally select the post-marginalized frame 3→4 link.

Artifacts:

- Rust boundary: `target/m7im15_rust_imu_factor_input_frame4_link0_linux_luna20260825_r2.json`
- Stage comparison: `target/m7im15_imu_factor_input_compare_linux_luna20260825_r2.json`
- Comparison helper: `work/m7im15_compare_input.ps1`

The comparison helper was pointed at the existing offline `m7aq` pinned-header
oracle only. It is not the live native `dense_call1` record: that record has
already been shown to differ from m7aq (121/270 J entries and 5/9 residual
entries), so the stage counts are provenance diagnostics, not a native-live
parity claim. Against m7aq, the first observed input difference is the
preintegration delta (`delta_position[0]`: Rust `3c1df76d`, oracle
`3c1df76c`); therefore no source-faithful arithmetic fix is justified.

Native GDB raw residual/J/W capture was not achieved in this bounded turn; the
existing native script captures only post-linearize `Jp/r` and local `H/b`.
No production arithmetic edit was made.

The exploratory probe is [m7im15_native_raw_probe_luna.cmd](../../target/m7im15_native_raw_probe_luna.cmd).
It reached `imu_block.hpp:49` three times, but the optimized RelWithDebInfo
DWARF reported `Cannot evaluate function -- may be inlined` for the accessor
needed to identify `start_t`; the earlier line-50 probe reported the locals
(`start_t`, `res`, and derivative temporaries) as optimized out. Consequently
there is no valid live-native raw artifact and no stagewise native-live claim.

Verification: WSL `cargo fmt`; `RUSTFLAGS='-A dead_code' CARGO_INCREMENTAL=0`
Linux release example rebuild succeeded. The focused release test was started
but remained in compilation when the bounded turn ended; no result is claimed.
