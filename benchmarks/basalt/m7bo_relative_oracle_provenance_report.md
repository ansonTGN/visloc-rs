# M7bo relative-pose oracle provenance

Date: 2026-08-23 JST  
Status: resolved; this audit made no production-source change.

## Decision

The owner of track 1, observation 2 in the upstream frame-4 dump is the
relative transform emitted by the real upstream `LinearizationAbsQR` call:

```text
t     = bde1e255,baf1ec60,ba884368
q_xyzw= bc0e2957,bb507f02,ba002d40,3f7ffd31
```

The iteration record is therefore correct.  The Rust/local standalone probe
value (`bde1e254,baf1ecc0,ba884364` and
`bc0e2956,bb507eff,ba002940,3f7ffd31`) is not the authoritative value for
this factor.  No production relative-pose implementation was changed here.

## Provenance and locator

The upstream source is commit
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc`, tree
`b7afb830d82b45b8209cf784ad9744025d838411`, checked out at
`/root/visloc-basalt-oracle-0f3b2b52`.  The dump and record are:

```text
target/m7al_upstream_f4_20260821T000200Z/iteration.jsonl
size       32541464
sha256     18e87cff493f758729f91e8270c9968e55bc8fb33cdcf1c3307fce9119711f7a
record     line 57, iteration=0, trial=0, phase=linearization
track      1, host=(1403636579763555584, cam 0)
observation target=(1403636579813555456, cam 1)
pixel      (27.31320571899414, 106.39038848876953)
pixel bits 41da8172,42d4c7e1
record raw residual  (-0.009939193725585938, 1.1353836059570312)
record residual bits  bc22d800,3f915440
```

The exact calibration/config inputs are the pinned checkout's
`data/euroc_ds_calib.json` (SHA-256
`ad8c5a18c48c55dacf61d18ebbc18cd7d4f3acccd5840bd5a5cd8646adb6271c`) and
`data/euroc_config.json` (SHA-256
`82937bd6493e592ef89572d31260c10f7437b4fb3ff1fda179375713966e34fa`).

The upstream `core-relwithdebinfo` build is Ninja/RelWithDebInfo with GCC
11.4.0 (`/usr/bin/c++`), C++17, `-O3 -g -march=native`,
`-DEIGEN_DONT_PARALLELIZE`, and
`-DEIGEN_INITIALIZE_MATRICES_BY_NAN`.  The complete compile command is the
command recorded by `ninja -C build/core-relwithdebinfo -t commands` for
`src/linearization/linearization_abs_qr.cpp`; in particular it has no
`-DNDEBUG`.  The rebuilt diagnostic-linked objects were:

The command's relevant exact prefix/suffix (the intervening `-D` and
`-I` entries are retained verbatim in the machine artifact) is:

```text
/usr/bin/c++ -DBASALT_INSTANTIATIONS_DOUBLE -DBASALT_INSTANTIATIONS_FLOAT ... -DEIGEN_DONT_PARALLELIZE -march=native -O3 -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN -std=c++17 -fPIC -MD -MT CMakeFiles/basalt.dir/src/linearization/linearization_abs_qr.cpp.o -MF CMakeFiles/basalt.dir/src/linearization/linearization_abs_qr.cpp.o.d -o CMakeFiles/basalt.dir/src/linearization/linearization_abs_qr.cpp.o -c /root/visloc-basalt-oracle-0f3b2b52/src/linearization/linearization_abs_qr.cpp
```

```text
basalt_vio sha256 edb6586f7ec03fa055b262840db9ae8216fb4ec18d9291aa6e1dc4714d06188f
libbasalt.so sha256 0cea5175689d28605a96cf1aad934a43f3c6487076eb490870280ebb78c83032
```

The standalone generator is
[`m7bo_relative_oracle_probe.cpp`](m7bo_relative_oracle_probe.cpp), SHA-256
`814b61a7dd9da1ea297b638c5caebae3aafcbfc6536e637ddef2355e1de07085`.
It was rebuilt with the same include paths, defines, warning flags, C++17,
`-O3 -g -march=native`, and Eigen flags.  Its binary was
`/tmp/m7bo_relative_oracle_inline_like`, SHA-256
`7e92fc9bc3556bdfd24c1be102261d1ce69fc5ccf8f0e28d4bcf449022dfe04d`.

The checkout was already dirty for diagnostic instrumentation in
`landmark_block.hpp`, `landmark_block_abs_dynamic.hpp`, `ba_utils.h`,
`linearization_abs_qr.cpp`, and `sqrt_keypoint_vio.cpp`; it was not reset.
The combined binary diff hash for the latter three diagnostic files is
`2ea1a502e0183393a126702fbe97383ce42caea0cb98dda1f2ae3946b7d1ce4e`.

## Runtime proof

`linearization_abs_qr.cpp` was instrumented only in the external upstream
checkout to print `RUNTIME_REL` when the exact host/target/camera tuple above
was encountered.  The first matching line in the five-frame run
(`stderr.log` SHA-256
`1302b6c2747e9f915ecfaf8fbffc2b413292ae3fb002214e909898693557783f`)
contains these f32 operands.  The run command was:

```text
/root/visloc-basalt-oracle-0f3b2b52/build/core-relwithdebinfo/basalt_vio --dataset-path /mnt/e/datasets/euroc_mav/machine_hall/MH_01_easy --cam-calib /root/visloc-basalt-oracle-0f3b2b52/data/euroc_ds_calib.json --dataset-type euroc --config-path /root/visloc-basalt-oracle-0f3b2b52/data/euroc_config.json --marg-data /marg_data --show-gui 0 --save-trajectory euroc --num-threads 4 --max-frames 5
```

The first matching line contains these f32 operands:

```text
host t = 00000000,00000000,00000000
host q = bd582e43,bf4d686f,00000000,3f182ffd
target t = 39c8bb60,b8cebae8,bb0527fc
target q = bd5cdea6,bf4d341f,bb2aa78b,3f186f67
cam0 t = bc896b48,bd8d2fdc,3ba86617
cam0 q = bbed3c0f,3bf71cd4,3f33a827,3f365a1e
cam1 t = bc76fa76,3d290319,3b4f4832
cam1 q = bb19188b,3c55012e,3f33d4ed,3f362af6
```

The same line emits the decision transform shown at the top of this report.
The operands exactly match the iteration-start snapshot at line 124 (and the
track's host/target state values).  The calibration f32 cast and the pose
snapshot therefore do not explain the local-vs-upstream difference.

The double-sphere projection operands were also held constant by the cast:

```text
cam0 intrinsics (fx,fy,cx,cy,xi,alpha)
  = 43aee0c5,43ae5cbe,43b6f27c,43795478,be76bd89,3f1126b5
cam1 intrinsics (fx,fy,cx,cy,xi,alpha)
  = 43b4d5f0,43b44af7,43bdb43f,437ffa30,be5a1edd,3f13a2ab
```

Thus both the target-camera intrinsics and both camera extrinsics are
bitwise accounted for before comparing the projected point.

For completeness, the canonical f32-bit rows for the relevant host frame 0
and target frame 1 across all 24 snapshots (8 iteration starts plus their
trial/accepted states) have SHA-256
`4a2cbcacfe56dea32213a48e2091233f8cd9c25fb1be536492f2158f1110bc43`.
Each snapshot has five state blocks; every trial was accepted, so each
accepted state is the following iteration start.  The eight iteration-start
rows are:

| iteration | host frame 0: `t` / `q_xyzw` | target frame 1: `t` / `q_xyzw` |
|---:|---|---|
| 0 | `00000000,00000000,00000000` / `bd582e43,bf4d686f,00000000,3f182ffd` | `39c8bb60,b8cebae8,bb0527fc` / `bd5cdea6,bf4d341f,bb2aa78b,3f186f67` |
| 1 | `366aea00,368cce48,388e781c` / `bd587fec,bf4d699e,38d8db06,3f182df0` | `b939cb10,ba512a18,bc577245` / `bd5cff33,bf4d37d1,bb25d838,3f186a45` |
| 2 | `366f1a00,b4112a20,36b72520` / `bd589afc,bf4d86a2,38cfb3e7,3f180698` | `baf8ecc5,ba3eecb7,bc81de68` / `bd5d03be,bf4d508e,bb27434f,3f1848e6` |
| 3 | `3482d800,b2fbf858,35105550` / `bd578395,bf4db51c,b9a9a3f9,3f17c93b` | `bb116914,ba2c68f1,bc88458c` / `bd5bd8b5,bf4d7bf3,bb441940,3f180fe3` |
| 4 | `b1a38000,b2cefd41,3381c630` / `bd55e156,bf4e10c8,ba814073,3f174ef6` | `bb09f8f2,ba09f33c,bc8a2482` / `bd5a2798,bf4dd659,bb6fc56f,3f1797a3` |
| 5 | `b29f2000,b168acd0,3158fe00` / `bd553465,bf4eac03,bab9fde6,3f167b8f` | `bae002de,b9f9e198,bc89fc4d` / `bd596b3d,bf4e7186,bb8615ca,3f16c4ec` |
| 6 | `b237c000,2edbb9a0,ad0a0000` / `bd5580b6,bf4f1424,babf9af5,3f15ebb5` | `babc6ed6,b9fea14e,bc899c67` / `bd59aed2,bf4ed9ce,bb8786b0,3f163553` |
| 7 | `b1dcc000,2fdb8861,2e6a8000` / `bd55dc3a,bf4f536e,babb6717,3f1593a1` | `baa7b22b,ba02bd03,bc896848` / `bd5a06d2,bf4f192f,bb8689e8,3f15dd68` |

## Projection comparison

The direct in-process probe calls (`compute_rel`, with and without Jacobian,
and the state-like call) all produce the old/local value:

```text
direct q = bc0e2956,bb507eff,ba002940,3f7ffd31
direct t = bde1e254,baf1ecc0,ba884364
```

That path is not the factor's authoritative call site.  The actual
`LinearizationAbsQR` call above produces the requested transform.  Replaying
that transform as a literal in the same probe produces the iteration record's
projection exactly.  Sophus normalizes the literal constructor's quaternion,
so its stored q bits become `bc0e2958,bb507f03,ba002d41,3f7ffd32`; those
constructor bits are not substituted for the captured runtime `RUNTIME_REL`
bits.

| path | target point `(x,y,z,rho)` f32 bits | projected `(u,v)` bits | residual bits |
|---|---|---|---|
| local/direct transform | `bf2fc73d,be94aaaa,3f2ec6b2,3e160ef3` | `41da6d22,42d70d2e` | `bc228000,3f915340` |
| captured upstream transform / literal replay | `bf2fc73e,be94aaa8,3f2ec6b2,3e160ef3` | `41da6d17,42d70d32` | `bc22d800,3f915440` |
| iteration record, track 1 obs 2 | — | — | `bc22d800,3f915440` |

The first differing projection operand is `target_point.x`:
`bf2fc73d` (local/direct) versus `bf2fc73e` (authoritative), followed by
`target_point.y` (`be94aaaa` versus `be94aaa8`).  `z` and `rho` are bitwise
equal.  Thus the first projection difference is downstream of the relative
transform arithmetic; it is not a calibration-extrinsic or snapshot-pose
input difference.  The exact iteration residual proves that the iteration
record's transform inference was right.

The machine-readable copy of this audit is
[`m7bo_relative_oracle_provenance.json`](m7bo_relative_oracle_provenance.json).
