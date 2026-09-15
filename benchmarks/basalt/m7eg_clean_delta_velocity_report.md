# M7eg clean IMU delta-velocity boundary

Date: 2026-08-23 JST  
Status: **clean endpoint captured; no production edit**

## Result

One bounded GDB run of the pinned clean `basalt_vio` stopped in the actual
`IntegratedImuMeasurement<float>::predictState` call for frame 3 → 4.  The
measurement object was the live estimator object, not a reconstructed
standalone probe.  Raw float32 words at that boundary are:

| operand/result | x | y | z |
|---|---:|---:|---:|
| `delta_state.vel_w_i` | `3ede21c0` | `bcb31cdc` | `be20c273` |
| `state0.vel_w_i` | `3d850448` | `bc998d12` | `be4a560c` |
| gravity | `00000000` | `00000000` | `c11cf5c3` |

The endpoint duration is `delta_t_ns=50000128`, with `dt` bits `3d4cccef`
(`0.050000127404928207` as f32).  The clean direct prediction therefore
produces `3da7a8b8,bc8bed06,be67f1c6`, matching the M7db clean frame-4 state.

The machine-readable capture is
[`target/m7eg_clean_delta_velocity.json`](../../target/m7eg_clean_delta_velocity.json).
The binary raw record is
[`target/m7eg_clean_delta_velocity.raw`](../../target/m7eg_clean_delta_velocity.raw);
the GDB transcript is
[`target/m7eg_clean_delta_velocity.out`](../../target/m7eg_clean_delta_velocity.out),
and the reproducible bounded command file is
[`target/m7eg_clean_delta_velocity.cmd`](../../target/m7eg_clean_delta_velocity.cmd).

The run also recorded 40 float propagation packets, with the final frame-3 →
4 group at ordinals 30–39.  The selected instruction has authoritative raw
`old_velocity`, `dt`, corrected input accel/gyro, and packet `accel_world`
`x/y` words.  Its `z` stack slot is before the compiler's z-lane spill and is
therefore explicitly marked non-authoritative in the JSON/raw record; no z
packet value is used for the conclusion.  The final live measurement
`delta_velocity` above is authoritative.

## Structure and symbols

The clean library is from Basalt commit
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc`:

```text
libbasalt.so
  IntegratedImuMeasurement<float>::propagateState  0x4bd760
  IntegratedImuMeasurement<float>::predictState    0x4b0b40
  IntegratedImuMeasurement<float>::integrate        0x4c23b0
```

`sizeof(IntegratedImuMeasurement<float>)` is 976 (`0x3d0`) bytes.  The
observed layout is:

```text
PoseVelState<float>:              t +0x00, q(xyzw) +0x10, p +0x20, v +0x30
Integrated measurement delta:    t +0x10, q +0x20, p +0x30, dv +0x40
```

The clean source expressions are:

```cpp
Vec3 accel_world = RR_w_i_new_2 * data.accel;
next_state.vel_w_i = curr_state.vel_w_i + accel_world * dt;
```

## First divergence versus Rust/current probes

The existing O2/O3 packet logs are the controlled comparison for the current
Rust intermediate.  The clean endpoint exactly matches the O3 packet path;
the retained Rust path matches O2:

| boundary | clean/O3 | Rust/current O2 |
|---|---:|---:|
| packet 6 velocity-x | `3e85be13` | `3e85be12` |
| final `delta_velocity.x` | `3ede21c0` | `3ede21be` |
| final `delta_velocity.y` | `bcb31cdc` | `bcb31cdc` |
| final `delta_velocity.z` | `be20c273` | `be20c273` |

Thus the first divergence is packet 6's velocity-x evaluation inside the
Eigen packet expression `curr_state.vel_w_i + accel_world * dt`; it is before
`predictState` and not a gravity association or Sophus point-action issue.
The M7du audit records that the O2 evaluator uses packet multiply-plus-add,
whereas the clean O3 compiler schedule separates/fuses scalar lanes.  A
per-lane FMA production change was already rejected because it regressed the
frame-1/2/3 exact gates.  No source-level Eigen-faithful rewrite can be
accepted from this fixture without changing that compiler-dependent contract.

## Provenance and hashes

```text
checkout: /root/visloc-basalt-clean-m7cr-20260823
binary:   /root/visloc-basalt-clean-m7cr-20260823/build/core-relwithdebinfo/basalt_vio
library:  /root/visloc-basalt-clean-m7cr-20260823/build/core-relwithdebinfo/libbasalt.so
run:      MH_01_easy, four native threads, --max-frames 5, /marg_data_m7eg

cmd sha256:  D0ECC18EC90CCBFA57F2D4E11DF79A611489D405FC87AD79E67F534F099B6DD7
raw sha256:  17BF7A9515DFBF032233140A62C416460D7D01BE60B8401A7AE0406FA3848891
out sha256:  103E7FE9616BA0C5B6F24C3E8DF37C4028FFE9425D367BA1AFAFD6D8A6C277EA
json sha256: D28FBC5E801F4A2E050957C7DFD69C2ABFE1DEE75488CEC93C6995580475E49E
```

No clean checkout, production Rust source, build outputs, commit, or push was
changed.
