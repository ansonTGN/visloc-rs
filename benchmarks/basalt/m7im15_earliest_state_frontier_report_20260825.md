# M7IM15 earliest state frontier (2026-08-25)

The first non-bit-exact state input is identified before the previously investigated frame-6/iteration-6 boundary.

- Pinned Basalt commit: `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.
- Native capture: `target/m7im15_native_earliest_input_0_100_20260825.bin` (69 records, ordinals 0..68).
- Rust capture: `target/m7im15_earliest_rust_imu_20260825.jsonl` (64 flattened local blocks).
- Aligned by explicit frame/iteration/link mapping: 64 blocks; native-only calls: `[32, 33, 34, 35, 68]`.
- Start-time/dt key mismatches: `0`.

## Frontier

- Native call ordinal: `4`.
- Rust frame/iteration/block: `4/1/0`.
- Link: `0 -> 1`.
- Lane/component/index: `from_current` / `translation[0]` / `0`.
- Native bits: `366aec00`; Rust bits: `366ae3a9`.
- All four state lanes at frame 4 iteration 0 are exact; FEJ lanes remain exact at the frontier while current lanes first differ.

## Delta input check

The delta input is not bit-exact at the initial aligned block:
- Native call ordinal / Rust frame-iteration-block: `0` / `4/0/0`.
- First delta field/index: `delta.rotation_matrix` / `4`.
- Native bits: `3f7ffefc`; Rust bits: `3f7ffefd`.
Thus the state-update frontier and the independent preintegration-delta frontier are both recorded; the first state-lane mismatch still occurs at native ordinal 4 after the accepted iteration-0 update.

## Update/branch interpretation

The native and Rust LM traces both accept iteration 0. The next IMU call is frame 4 iteration 1, where the FEJ state still agrees but the current state does not. This places the first state-input divergence in the accepted iteration-0 apply/update path, before the next linearization; it is not caused by a reject/restore branch.

The complete machine-readable alignment, per-lane first differences, delta-stage comparisons, counts, and source hashes are in:
`target/m7im15_earliest_state_frontier_20260825.json`.
