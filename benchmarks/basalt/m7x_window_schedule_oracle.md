# M7x pinned-Basalt window/keyframe schedule oracle

This audit targets upstream Basalt commit
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc` and the sensor-only MH_01
trace
`target/basalt_upstream_mh01_400f_core_trace_20260821T000012Z/trace.jsonl`.
The source checkout used for line evidence is
`/root/visloc-basalt-oracle-0f3b2b52` in WSL. No upstream file was modified.

## Exact upstream rules

| concern | pinned source evidence | exact rule | Rust status after M7x |
| --- | --- | --- | --- |
| initial KF | `sqrt_keypoint_vio.cpp:60-64,380-385` | `take_kf=true`; frame 0 is unconditionally a KF, then the flag is cleared and `frames_after_kf=0` | implemented with persistent `take_kf` and `frames_after_kf` |
| subsequent KF | `:336-373,380-468` | camera-0 `connected0 / (connected0 + unique unconnected0) < 0.7` and **strictly** `frames_after_kf > 5`; only the non-KF branch increments the counter | implemented; connectivity is sampled before current-frame triangulation |
| solve start | `:1012-1019` | solve only when `opt_started || frame_states.size() > 4`; once true it stays true | implemented before M7x and covered by the frame-4 gate test |
| marg trigger | `:552-566` | after solve, enter when `frame_poses.size() > 7 || frame_states.size() >= 3`; set boundary index to `size - 3 + 1` | state target selector now uses `>=` and the exact boundary |
| old non-KF state | `:585-608,784-813,910-914` | full 15-DoF marginalization and erase | target selection implemented |
| old KF state | `:590-595,802-806,916-923` | marginalize velocity/bias only, retain its 6-DoF pose in `frame_poses` | **missing in the live solver**: Rust currently marginalizes the entire 15-DoF block because it has no pose-only window variable |
| stale pose | `:570-583,925-930` | every pose not present in `kf_ids` is marginalized/erased | missing together with pose-only variables |
| max seven KFs | `:610-680` | only after a KF state becomes pose-only: while `kf_ids.size()>7`, skip newest two, first prefer an old KF with missing connection count or connected/created ratio `<0.1`; otherwise use the DSO distance score | missing; `WindowPolicy.max_kfs` alone is not an equivalent implementation |
| lost landmarks | `:470-482,729-731,930-934` | when enabled, every landmark absent from **all cameras of the current frame** is passed into marginalization and then removed immediately | being handled separately by the landmark-port task; it must not be approximated as a state-drop criterion |

The EuRoC config values are authoritative in
`configs/basalt/euroc_config.json`: `max_states=3`, `max_kfs=7`, minimum
frames `5`, new-KF ratio `0.7`, old-KF feature ratio `0.1`, and
`vio_marg_lost_landmarks=true`.

## First 100 MH_01 frames

The upstream keyframes in frames 0--99 are exactly:

```text
0, 7, 14, 21, 28, 35, 42, 49, 56, 63, 70, 77, 84, 91, 98
```

Thus the strict counter produces a seven-frame index gap, not five. The
candidate-frame connected ratios are all below 0.7 in this prefix. Frame 0
starts at `(states, poses)=(1,0)`. Frames 1--3 grow to `(4,0)` without a
solve. Frame 4 is the first solve and converts KF 0 to a pose while fully
marginalizing states 1 and 2, producing `(2,1)`. Thereafter the normal active
window remains at two 15-DoF states. KF 7 remains a state through frame 8 and
is converted at frame 9, producing `(2,2)`.

The KF set temporarily reaches eight at frames 49, 56, 63, ... because a new
KF is still a full state. Two frames later, when that state is converted to a
pose, the old-KF selection runs: marginalization events with one KF occur at
frames 51, 58, 65, 72, 79, 86, and 93. The post-event window is always two
full states and seven poses.

| frame | KF | post states | post poses | state marg | KF marg |
| ---: | :---: | ---: | ---: | ---: | ---: |
| 0 | yes | 1 | 0 | 0 | 0 |
| 1 | no | 2 | 0 | 0 | 0 |
| 2 | no | 3 | 0 | 0 | 0 |
| 3 | no | 4 | 0 | 0 | 0 |
| 4 | no | 2 | 1 | 3 | 0 |
| 5 | no | 2 | 1 | 1 | 0 |
| 6 | no | 2 | 1 | 1 | 0 |
| 7 | yes | 2 | 1 | 1 | 0 |
| 8 | no | 2 | 1 | 1 | 0 |
| 9 | no | 2 | 2 | 1 | 0 |
| 10 | no | 2 | 2 | 1 | 0 |
| 49 | yes | 2 | 7 | 1 | 0 |
| 51 | no | 2 | 7 | 1 | 1 |

## Consequence for the Rust port

The previous Rust selector preferred keeping KF navigation states, triggered
only above three states, and used connection/loss as navigation-state drop
criteria. Those rules contradict the pinned source and can destroy the
contiguous two-state IMU link. M7x corrects the KF counter/config plumbing and
the full-state boundary selection. It deliberately does not claim complete
window parity: a faithful next step must add 6-DoF `frame_poses` to the AOM,
visual factors, FEJ prior, and MargData, then implement the seven-KF selection
above. Until that is done, a Rust keyframe's pose information is Schur-reduced
with its full state rather than remaining explicitly optimizable.

## M7x live Rust replay

The final sensor-only release replay is
`target/basalt_m7x_window_run100_kf_landmarks/trace.jsonl` (MH_01 frames
0--99). It includes the concurrent landmark-port correction that restricts
new-landmark triangulation/insertion to selected keyframes. Its
post-marginalization active full-state count matches the upstream trace on all
100 frames: 1, 2, 3, 4, then two from frame 4 onward. Its KF list is also
exact:

```text
upstream: 0,7,14,21,28,35,42,49,56,63,70,77,84,91,98
Rust:     0,7,14,21,28,35,42,49,56,63,70,77,84,91,98
```

The raw connected/unconnected counts are not numerically identical (for
example frame 7 is upstream `43/132`, Rust `48/147`), so equality of this
prefix's decisions does not prove landmark/track parity. It does prove the
counter, threshold, classification ordering, KF-only insertion, solve gate,
and full-state count schedule together reproduce the pinned first-100
decisions. Pose-only optimization and old-KF selection remain the explicit
structural gaps described above.
