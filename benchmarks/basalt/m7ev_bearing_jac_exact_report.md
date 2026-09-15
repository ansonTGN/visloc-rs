# M7ev `bearing_jacobian_f32` exactness

Date: 2026-08-23 JST  
Status: **Pass — pinned Basalt operation grouping restored; focused/full tests and fresh-five replay are green.**

## Change

The pinned header
`/root/visloc-basalt-oracle-0f3b2b52/thirdparty/vcpkg/buildtrees/basalt-headers/src/2e4db7e43d-fa35c15fa5.clean/include/basalt/camera/stereographic_param.hpp`
implements `StereographicParam<float>::unproject` Jacobian arithmetic by
materializing `x2`, `y2`, `r2`, `norm_inv`, `norm_inv2`, and `xy` in that order.
Rust previously spelled the denominator and cross product inline, allowing a
different binary32 association.

`StereographicDirection::bearing_jacobian_f32` now follows the pinned scalar
stages: `x2 = u*u`, `y2 = v*v`, `r2 = x2+y2`,
`norm_inv = 2/(1+r2)`, `norm_inv2 = norm_inv*norm_inv`, and `xy = u*v`,
then uses those temporaries for all six active matrix lanes. No value-specific
branch, unsafe code, debug output, or ignored test was added.

The existing ordinal-0 packet fixture now also asserts the direct 3x2 Jup
boundary. Its identity transform means the first six active fixture lanes are
the expected `bearing_jacobian_f32` result; the downstream homogeneous packet
and fixed-size landmark product remain covered by the same test.

## Verification

Focused release M7 suite:

```text
cargo test --release -p visloc-basalt --lib vio::aom::tests::m7 -- --nocapture
13 passed, 0 failed, 0 ignored
```

Full release Basalt library:

```text
cargo test --release -p visloc-basalt --lib
169 passed, 0 failed, 1 ignored
```

The one ignored test is the pre-existing M8c diagnostic test. The release
example also rebuilt successfully:

```text
cargo build --release -p visloc-rs --example basalt_euroc_vio_demo
```

## Fresh-five replay

The established sensor-only MH01 replay was run with `--max-frames 5`, detail
frame 4, and one iteration. It processed 5/5 frames, delivered 42 IMU samples,
emitted 1,114 observations, and retained 70 factors / 1,243 rows (61 visual
factors / 584 visual observations at frame 4). Iteration-start cost was
`4215.93310546875`; all eight LM decisions were accepted (`AAAAAAAA`).

Artifacts:

- [fresh detail](../../target/m7ev_bearing_jac_exact_fresh5_detail.jsonl)
- [fresh run summary](../../target/m7ev_bearing_jac_exact_fresh5_run/summary.txt)
- [rebuilt executable](../../target/release/examples/basalt_euroc_vio_demo.exe)

SHA-256:

```text
c27ec8f5516dc322f369099590f282d9b4a8b1eb9c5e911492457891fb216c01  target/release/examples/basalt_euroc_vio_demo.exe
26e23fee9f269606abc1a63f58a8dbe1729e0a8b74026441bb47bc60f9f78654  target/m7ev_bearing_jac_exact_fresh5_detail.jsonl
51107dde483da65dd894515c4bc07d548baa6c1b04d68daa546b3cfabcfc7a2b  target/m7ev_bearing_jac_exact_fresh5_run/summary.txt
```

## Native visual comparison

The fresh detail was matched to all 584 clean native `m7ef` records by exact
binary32 direction/rho and pixel keys. Weighted Jp compares native raw
`d_res_d_p` multiplied in binary32 by the Rust observation weight against the
serialized Rust weighted landmark Jacobian; raw Jp removes that same weight in
binary32. H is compared after transposing Rust's JSON matrix to native
column-major order.

| replay | projection pairs / lanes | raw residual pairs / lanes | weighted Jp pairs / lanes | raw Jp pairs / lanes | first Jp mismatch |
|---|---:|---:|---:|---:|---|
| M7ee baseline | 564/584; 1,143/1,168 | 564/584; 1,143/1,168 | 364/584; 2,860/3,504 | 256/584; 2,656/3,504 | ordinal 0 |
| M7et baseline | 564/584; 1,143/1,168 | 564/584; 1,143/1,168 | 356/584; 2,838/3,504 | 250/584; 2,636/3,504 | ordinal 0 |
| **M7ev** | **564/584; 1,143/1,168** | **564/584; 1,143/1,168** | **488/584; 3,285/3,504** | **350/584; 3,057/3,504** | **ordinal 3** |

Ordinal 0 (track 66, same-TimeCam identity) is now exact:

```text
weighted Jp native = 44e5f922,3fe52460,3fe5d1e0,44e4cfbf,00000000,00000000
weighted Jp Rust   = 44e5f922,3fe52460,3fe5d1e0,44e4cfbf,00000000,00000000
raw Jp native      = 4465f922,3f652460,3f65d1e0,4464cfbf,00000000,00000000
raw Jp Rust        = 4465f922,3f652460,3f65d1e0,4464cfbf,00000000,00000000
```

The new first weighted/raw Jp mismatch is ordinal 3, track 11 observation 0
(same-TimeCam). Its projection and residual remain exact; the three differing
active lanes are the expected later boundary, not the former ordinal-0
stereographic Jacobian boundary.

Aggregate frame-4 H/b against clean native `m7ct`:

| replay | H exact / total | b exact / total | H first mismatch | b first mismatch |
|---|---:|---:|---|---|
| M7ee | 3,247/5,625 | 6/75 | `4de48a64` vs `4de48a6f` | `45de4e52` vs `45de4ea7` |
| M7et | 3,241/5,625 | 6/75 | `4de48a64` vs `4de48a6f` | `45de4e52` vs `45de4ea9` |
| **M7ev** | **3,247/5,625** | **6/75** | `4de48a64` vs `4de48a6f` | `45de4e52` vs `45de4ea9` |

The bearing repair therefore closes the ordinal-0 Jup/Jpp/Jp mismatch without
claiming full visual or aggregate H parity. Projection/raw and H remain at the
established M7ee boundary; no unrelated worktree changes were modified.

No commit or push was performed.
