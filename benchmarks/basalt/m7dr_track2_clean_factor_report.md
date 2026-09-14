# M7dr clean track-2 same-timestamp stereo factor

Date: 2026-08-23 JST  
Status: **Pass — retained bounded clean GDB capture reached the exact
pixel-filtered factor and final Jacobian write.**

## Factor and provenance

The run used the unchanged pinned clean artifact from m7cr/m7dd:

- commit `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`;
- `basalt_vio` SHA-256
  `89e0324ccd04c3b615bf7e945aa32480a2f86524987aa00eaf152dcd6b1d242c`;
- `libbasalt.so` SHA-256
  `55f842642b370edf4025afd1d66c7b9d9011e5b4ce08efeb9d76be401b45b2cc`.

The retained command and raw output are
[`target/m7do_track2_clean_factor.cmd`](../../target/m7do_track2_clean_factor.cmd)
and
[`target/m7do_track2_clean_factor.gdb.out`](../../target/m7do_track2_clean_factor.gdb.out).
The output is 6,630 bytes with SHA-256
`79710923b69753d1eab7c5e65fbc405d8cd998a0c4aaa1e94ddd10e1a868143db`.

The exact factor is frame 4, iteration 0, trial 0, track 2, host
(frame 0, cam 0) to target (frame 0, cam 1), with observation pixel bits
`42517d4a,43046ac8`. The filter saw 194 `linearizePoint` entries and one
matching candidate, then stopped at the final Jacobian store. The dynamic
ASLR base was `0x744f9b800000`.

Machine-readable values and comparisons are in
[`target/m7dr_track2_clean_factor.json`](../../target/m7dr_track2_clean_factor.json).

## Native capture

The target homogeneous point (`x,y,z,rho`, xmm6 lane order) is:

```text
bf26f362 be7d2383 3f3c2d9d 3e42a1b2
(-0.6521512269973755, -0.24720577895641327, 0.7350710034370422, 0.19006994366645813)
```

The native projection and raw residual are:

```text
projection: 42518143 43046250 = (52.3762321472168, 132.384033203125)
raw:        3b7e4000 bd078000 = (0.003879547119140625, -0.0330810546875)
```

The final fixed-size Eigen buffers are recorded in both column-major and
row-major form in the JSON artifact:

- `d_res_d_xi` (`2x6`, relative-pose block):
  `428635d1,c12a9240,c12b15bb,42b5f916,425fc1d6,41a92060,c1fb36f6,c3bdafeb,43e1ba28,41fa75e8,42f7ee4a,c3a3066d`;
- `d_res_d_p` (`2x3`, landmark block):
  `44486188,c1c7d418,c1ccfe70,44561706,c21c3dda,40c8c0c7`.

## Comparison with current Rust m7dk detail

The matching `target/m7dk_fresh5_detail.jsonl` record is
`iteration_start`, iteration 0, track 2, observation target frame 0/cam 1.
Its projection/raw bits are `4251812c,4304624e` and
`3b788000,bd07a000`, respectively:

| quantity | clean native | Rust m7dk | equal |
|---|---|---|---:|
| target point `x,y,z,rho` | `bf26f362,be7d2383,3f3c2d9d,3e42a1b2` | point not serialized; input rho exact | n/a |
| projected `u,v` | `42518143,43046250` | `4251812c,4304624e` | 0/2 |
| raw residual `u,v` | `3b7e4000,bd078000` | `3b788000,bd07a000` | 0/2 |
| landmark J, raw scaled by Rust `sqrt_weight=2` | `44c86188,c24cfe70,c29c3dda,c247d418,44d61706,4148c0c7` | `44c8618a,c24cfe6c,c29c3dda,c247d420,44d6170b,4148c0cf` | 1/6 |

The native `d_res_d_xi` is a raw relative-pose Jacobian. The Rust detail
does not serialize that intermediate; its absolute target/anchor pose blocks
are both zero here because host and target timestamps are equal. Therefore no
direct Jxi-versus-`jp_target` lane parity is claimed. Likewise, m7dk does not
serialize `T_t_h` or the transformed target point; it does serialize the
matching direction and rho input (2/2 and 1/1 exact).

## Integrity

No production source, clean source, binary, build, commit, or push was
changed. No GDB rerun was needed after the retained output was verified.
