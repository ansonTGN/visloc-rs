# M7IM15 native/Rust frame-state comparison (2026-08-25)

Pinned detached Basalt commit: `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.
This compares direct native M7IM15STEP1 state maps against the fresh Rust detail replay; no reconstructed state oracle was used.

- Pre-apply current state: `80/80` exact across `5` frames.
- Post-apply current state: `1/80` exact across `5` frames.
- Pre-apply first difference: `None`.
- Post-apply first difference: `{'frame_timestamp_ns': '1403636579763555584', 'index': 0, 'native': '366aec00', 'rust': '366b1e02'}`.
- State payload order is native/Rust compatible: translation xyz, quaternion xyzw, velocity xyz, gyro bias xyz, accel bias xyz.
- The paired solver report records the fresh global H/b, damping diagonal, and applied increment; these are emitted at their direct boundaries and are not reconstructed.

## Provenance

- Native step SHA-256: `368e2782ca8fbedec2440e9bbdde9a0dbde2e68239b5354a016dd517082de48a`.
- Rust compact step SHA-256: `a14e79712c78ce20c6f6890c0294cff1250b277e4230c09ed63c8ade34e79255`.
- Native source: direct detached M7IM15STEP1 state_map capture.
- Rust source: fresh detail trace after the blockwise whitening patch.
