# M7bw cam0 temporal exactness report

Date: 2026-08-23 (JST)

Scope: the earliest requested temporal sample, MH_01 frame 0 to frame 1,
cam0, track 2, source `(46, 118)`. The pinned native endpoint is the second
record in `benchmarks/basalt/mh01_stereo_endpoint_v1_first80.jsonl`.

## Result

The current artifact `target/m7bt_first80_final3.jsonl` is already exact for
the requested endpoint:

| value | native pinned | current Rust |
| --- | --- | --- |
| x | `0x422e3d26` | `0x422e3d26` |
| y | `0x42ea1cb7` | `0x42ea1cb7` |

No production source was changed by M7bw. In particular, there is no
track-specific correction, threshold change, unsafe code, or production trace
output.

## Trace boundary

`benchmarks/basalt/m7bw_temporal_probe.cpp` replays the pinned public Basalt
patch loop, while `pipelines/basalt/examples/m7bw_temporal_probe.rs` replays
the Rust public patch/update primitives. For the four pyramid levels and five
iterations per level, the following common f32 fields matched bit-for-bit:

- all 624 `H_se2_inv_J_se2_T` scalars (12 rows of 52 samples);
- all 1,040 residual scalars (20 residual vectors of 52 samples);
- all 60 increment scalars;
- all 40 per-iteration composed translation scalars;
- all 8 per-level world translations;
- patch means/data prefixes, source, and final translation.

Thus the first divergent scalar after the last proven boundary is in the
first `Sophus::SE2f::exp` update, before affine composition. The first tangent
is:

```
tx = 0xbeb28e40, ty = 0xbe1adabe, theta = 0xbb9ce858
```

The native and Rust scalar probe results are:

| field | native Sophus | current Rust `Se2::exp` |
| --- | --- | --- |
| rotation `(c,-s,s,c)` | `3f7fff40,3b9ce831,bb9ce831,3f7fff40` | identical |
| translation x | `0xbeb2bd73` | `0xbeb2bd72` |
| translation y | `0xbe1a001a` | `0xbe1a001a` |

The direct product/subtraction spelling produces `0xbeb2bd72` on both
compilers. The pinned GCC/Eigen expression contracts the x numerator as:

```text
second = one_minus_cos_over_theta * ty
x      = sin_over_theta.mul_add(tx, -second)
```

This produces the native `0xbeb2bd73`. The y numerator is correspondingly the
fixed-shape order `one_minus_cos_over_theta.mul_add(tx,
sin_over_theta * ty)`. The assembly for the pinned Sophus instantiation was
checked directly; the dedicated external probe is
`benchmarks/basalt/m7bw_se2_exp_probe.cpp`.

The one-ULP SE2 translation difference is masked by the subsequent fixed-size
affine composition for this track: the first level-3 composed translation is
already native at `0x40acd429,0x41699800`, and the endpoint remains exact.

## Closure status and next probe

The requested endpoint is closed by the current artifact, so no additional
production patch is justified here. Literal intermediate exactness is not
fully closed: `Se2::exp` still has the general native-vs-Rust x-order
divergence above. Before any future production change, the next probe must run
all 20 recorded increments through the uninstrumented pinned Sophus path and
compare both SE2 translation components, including the small-angle branch; only
then could an explicit fixed-order Rust implementation be considered for
production and revalidated against the full endpoint trace.

## Fixtures and tests

- `pipelines/basalt/tests/fixtures/m7bw_cam0_temporal_track2.json` records the
  first tangent, native/Rust SE2 bits, the first composed translation, and the
  exact endpoint bits.
- `pipelines/basalt/tests/m7bw_cam0_temporal_exact.rs` verifies the fixture,
  the proven native-order scalar, and the exact first composed translation.
- `pipelines/basalt/tests/m7bw_cam0_temporal_exact.rs` passed in release mode.
- Existing unit test
  `update::tests::right_composition_matches_native_l3_i1_boundary` also passed
  in release mode.

Trace hashes used for this report:

```text
native temporal trace  F0149D2F0FCDECC28D088DF1B775D32A87D9E3040E1FB96768CBE67C1CB19FB0
Rust temporal trace     CED0166EDA12F4E90562B3B97ECCDE7590DC1BF79964A7EF316DFA04012038A8
native SE2 probe        66E7999D4B7837447F70432346A44985D7F6A626A2757BC2378A5A0039C9A6E8
Rust SE2 probe          39A8562144704ECDD4CAA78D3C63F4D478AD23581CBAE4AF8677CF7A13B09BDD
```
