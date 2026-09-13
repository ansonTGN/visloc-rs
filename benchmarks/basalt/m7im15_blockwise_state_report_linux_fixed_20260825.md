# M7IM15 native/Rust frame-state comparison (2026-08-25)

Pinned detached Basalt commit: `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.
This compares direct native M7IM15STEP1 state maps against the fresh Rust detail replay; no reconstructed state oracle was used.

- Pre-apply current state: `80/80` exact across `5` frames.
- Post-apply current state: `0/80` exact across `5` frames.
- Pre-apply first difference: `None`.
- Post-apply first difference: `{'frame_timestamp_ns': '1403636579763555584', 'index': 0, 'native': '366aec00', 'rust': '366b1c89'}`.
- State payload order is native/Rust compatible: translation xyz, quaternion xyzw, velocity xyz, gyro bias xyz, accel bias xyz.
- The paired solver report records the fresh global H/b, damping diagonal, and applied increment; these are emitted at their direct boundaries and are not reconstructed.

## Provenance

- Native step SHA-256: `368e2782ca8fbedec2440e9bbdde9a0dbde2e68239b5354a016dd517082de48a`.
- Rust compact step SHA-256: `86ddef44f4bb85b2c70ed7e17cc2d3af03e3ab2e02d5a845e0dfbc5608d8efc3`.
- Native source: direct detached M7IM15STEP1 state_map capture.
- Rust source: fresh detail trace after the blockwise whitening patch.
