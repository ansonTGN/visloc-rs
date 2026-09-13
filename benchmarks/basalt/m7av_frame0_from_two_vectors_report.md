# M7av frame-0 `FromTwoVectors` scalar-boundary report

Date: 2026-08-22 (JST)  
Scope: `pipelines/basalt/src/vio/estimator.rs` frame-0 initialization only.
No production source was changed by this audit.

## Finding

The pinned native Eigen source call reproduces the current Rust f32 result
exactly, including every intermediate and every final quaternion lane.  The
authoritative pinned upstream frame-0 trace also contains those same f32 bits.
There is no frame-0 initialization mismatch to fix.

The upstream source declares `T_w_i_init` as `Sophus::SE3<Scalar>` and calls
`Eigen::Quaternion<Scalar>::FromTwoVectors(data->accel, Vec3::UnitZ())`.
`popFromImuDataQueue()` casts the double IMU queue to `Scalar`.  Consequently,
the same source call has two valid frame-0 boundaries:

| native call | frame-0 `wxyz` | exact source result |
| --- | --- | --- |
| `Quaternionf::FromTwoVectors` | `(0.5944822430610657, -0.052778493613004684, -0.8023747801780701, 0)` | Rust `ScalarMode::UpstreamF32` and authoritative upstream trace |
| `Quaterniond::FromTwoVectors` | `(0.5944822292098873584, -0.0527784890241279891, -0.802374794126767821, 0)` | diagnostic contrast only; not the authoritative upstream frame-0 result |

No `estimator.rs` helper change is justified.  In particular, do not tune or
replace the f32 result with the double diagnostic: that would cross the
declared `ScalarMode::UpstreamF32` state boundary and change downstream state
bits.

## Native source-call probe

Probe source: `m7av_frame0_from_two_vectors_probe.cpp`  
Probe log: `target/m7av_frame0_from_two_vectors_probe.log`

Build flags and pinned include roots:

```text
g++ -std=c++17 -O3 -march=native -DEIGEN_DONT_PARALLELIZE \
  -DEIGEN_INITIALIZE_MATRICES_BY_NAN -fPIC
```

Inputs are the first MH_01 IMU packet at the frame-0 camera timestamp,
`t_ns=1403636579763555584`, raw acceleration
`(8.0332807916666666, -0.40861041666666664, -2.40262925)`, cast to the tested
scalar.  The pinned accelerometer calibration is
`(-0.003025405479279035, 0.1200005286487319, 0.06708820471592454, 0, ..., 0)`.

For `float`, the native source-call intermediates are:

```text
calibrated accel = (8.0363054275512695, -0.5286109447479248, -2.469717264175415)
v0 = accel.normalized() = (0.9539951086044312, -0.06275175511837006, -0.29318177700042725)
v1 = UnitZ.normalized() = (0, 0, 1)
c = v1.dot(v0) = -0.29318177700042725
cross = v0.cross(v1) = (-0.06275175511837006, -0.9539951086044312, 0)
s = sqrt((1+c)*2) = 1.1889644861221313
invs = 1/s = 0.8410680294036865
vec = cross*invs = (-0.052778493613004684, -0.8023747801780701, 0)
q = (w,x,y,z) = (0.5944822430610657, -0.052778493613004684,
                 -0.8023747801780701, 0)
bits(q) = (3f182ffd, bd582e43, bf4d686f, 00000000)
```

For completeness, `double` produces the following distinct diagnostic values
when the same raw packet is intentionally evaluated as `Quaterniond`; these
are not used by the authoritative float estimator trace:

```text
q = (w,x,y,z) = (0.5944822292098873584, -0.0527784890241279891,
                 -0.802374794126767821, 0)
bits(q) = (3fe305ff98904efd, bfab05c83894f414,
           bfe9ad0de77d182c, 0000000000000000)
```

The float final quaternion has squared norm exactly `1.0f` in the native
probe, so nalgebra's current `UnitQuaternion::from_quaternion` does not alter
these frame-0 bits.  The probe also calls the native source factory directly;
all printed float intermediates and final lanes match the Rust helper's
current f32 path.

## Source provenance

The native probe uses the pinned checkout
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc` and its Eigen/Basalt headers.  The
Eigen implementation is the direct `QuaternionBase::setFromTwoVectors` path:

```text
v0 = a.normalized(); v1 = b.normalized(); c = v1.dot(v0);
axis = v0.cross(v1);
s = sqrt((1+c)*2); invs = 1/s;
vec() = axis * invs; w() = s * 0.5;
```

The source-call probe is diagnostic only; its generated executable/log are
local artifacts and must not be committed as production dependencies.

The authoritative upstream state-generation record is
`target/basalt_upstream_mh01_400f_core_trace_20260821T000012Z/trace.jsonl` and
the source audit is `benchmarks/basalt/m7q_state_generation_oracle.md`; both
record the f32 tuple above.  The older `target/basalt_m7ac_rawdlt8` tuple that
motivated the double contrast is not an upstream oracle and must not drive a
production change.

## Decision

Keep `from_two_vectors_eigen_f32` unchanged.  The native source-call probe,
authoritative upstream trace, and current Rust f32 path agree exactly.  No
tuning, ground truth, landmark, AOM, or LM changes are part of this finding.
