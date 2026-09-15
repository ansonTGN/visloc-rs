# M8a/M8b upstream NFR oracle

This is one real `MH_01_easy` MargData record processed by the pinned
upstream Basalt implementation. The fixture contains the complete input and
post-`processMargData` matrices/vectors, AOM maps, states/poses, image IDs,
and every recovered factor:

[`m8a_m8b_mh01_1403636579763555584.json`](m8a_m8b_mh01_1403636579763555584.json)

The record is the first MargData event (`kf_to_marg =
1403636579763555584`) from the core-only MH01 smoke output. The binary input
is outside this repository at
`/root/basalt-oracle-results-runner-smoke/MH_01_easy/20260821T000003Z/marg_data/1403636579763555584.cereal`
in Ubuntu-22.04 WSL2.

## Provenance and reproduction

- upstream commit: `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`
- upstream source tree: `/root/visloc-basalt-oracle-0f3b2b52`
- `src/vi_estimator/nfr_mapper.cpp` SHA-256:
  `ba1f179e78b857c7978d9489039a9588003b784b43caf1d8af7302c41fe0047a`
- `include/basalt/vi_estimator/nfr_mapper.h` SHA-256:
  `b274ed7cbdb90fb763f016f315b0a78eeba936475a25970a42b86a58a48be4f5`
- `src/vi_estimator/marg_helper.cpp` SHA-256:
  `5ea4164cbabbb8e3891eea25c366dcac4d16de9c6863f4ae164bf7eefcdb6d77`
- MargData cereal SHA-256:
  `24959dd11e60389f2770272eefcc4f38ce3f47598bd784ca837b05e8f8d612b7`
- `data/euroc_config.json` SHA-256:
  `82937bd6493e592ef89572d31260c10f7437b4fb3ff1fda179375713966e34fa`
- `data/euroc_ds_calib.json` SHA-256:
  `ad8c5a18c48c55dacf61d18ebbc18cd7d4f3acccd5840bd5a5cd8646adb6271c`

The diagnostic source is
[`upstream_nfr_oracle.cpp`](upstream_nfr_oracle.cpp), SHA-256
`ca4f1b575eacf4ef4bc6f5c5c8d8a37aaf815cefcc9cabd3dd215fecbcdf4781`.
It links the pinned `libbasalt.so`, uses `MargDataLoader` on the existing
MH01 artifact directory (therefore attaching the eight raw image records),
and calls `NfrMapper::processMargData` followed by
`NfrMapper::extractNonlinearFactors`. The exact WSL build/run shape is:

```bash
c++ -std=c++17 -O2 -march=native -DEIGEN_DONT_PARALLELIZE \
  -DBASALT_INSTANTIATIONS_DOUBLE -DBASALT_INSTANTIATIONS_FLOAT \
  -DCLI11_COMPILE -DHAVE_EIGEN -DHAVE_EPOXY \
  -DPANGO_DEFAULT_WIN_URI=\"x11\" -D_LINUX_ \
  -I/root/visloc-basalt-oracle-0f3b2b52/include \
  -I/root/visloc-basalt-oracle-0f3b2b52/thirdparty/ros/include \
  -I/root/visloc-basalt-oracle-0f3b2b52/thirdparty/apriltag/include \
  -isystem /root/visloc-basalt-oracle-0f3b2b52/thirdparty/vcpkg/buildtrees/eigen3/src/5.0.1-d487a628b0.clean \
  -isystem /root/visloc-basalt-oracle-0f3b2b52/build/relwithdebinfo/vcpkg_installed/x64-linux/include/opencv4 \
  -isystem /root/visloc-basalt-oracle-0f3b2b52/build/relwithdebinfo/vcpkg_installed/x64-linux/include \
  -L/root/visloc-basalt-oracle-0f3b2b52/build/core-relwithdebinfo \
  -Wl,-rpath,/root/visloc-basalt-oracle-0f3b2b52/build/core-relwithdebinfo \
  -o /tmp/upstream_nfr_oracle benchmarks/basalt/upstream_nfr_oracle.cpp \
  -lbasalt -lpthread

/tmp/upstream_nfr_oracle \
  /root/visloc-basalt-oracle-0f3b2b52/data/euroc_ds_calib.json \
  /root/visloc-basalt-oracle-0f3b2b52/data/euroc_config.json \
  /root/basalt-oracle-results-runner-smoke/MH_01_easy/20260821T000003Z/marg_data \
  benchmarks/basalt/m8a_m8b_mh01_1403636579763555584.json
```

Fixture SHA-256:
`385aa938289efe2de7d273ff035f780792c0eb0e0d44d4a4945a26688b100b60`.

## Exact M8a result

Input AOM has `total_size=72`, `items=9`, and these ordered blocks
`(id, offset, size)`:

```text
(1403636579763555584, 0, 6)
(1403636580113555456, 6, 6)
(1403636580813555456, 12, 6)
(1403636583263555584, 18, 6)
(1403636583613555456, 24, 6)
(1403636583963555584, 30, 6)
(1403636584313555456, 36, 6)
(1403636584663555584, 42, 15)
(1403636584713555456, 57, 15)
```

`kfs_all` contains the first eight IDs above; `kfs_to_marg` is the first ID.
The literal process keeps columns `0..47` and marginalizes columns `48..71`.
The kept state block is ID `1403636584663555584`: its pose columns are
retained and its velocity/bias columns are marginalized. The non-KF state
block ID `1403636584713555456` is fully marginalized. The unrelated state
ID `1403636584763555584` is not present in the AOM and remains in the output
state map.

Output AOM has `total_size=48`, `items=0` (the upstream `aom_new` leaves this
field at its default), and eight 6-DoF pose blocks at offsets `0,6,...,42`.
Output rank is 48; input rank is 72; `extractNonlinearFactors` returned
`true`. Input/output `abs_H` and `abs_b` are recorded in full in the JSON as
the `gram_and_gradient` pair. As compact anchors, input `H[0][0:8]` is
`[103486016, 1825271.875, -1015166.9375, 717501.4375, -2536983.25,
-1962043.125, -2823453.75, -1661072.5]`, while output
`H[0][0:8]` is
`[103475455.29287204, 1823402.6718330656, -1016532.7540364418,
716748.3170558064, -2535986.997957071, -1961137.5254066733,
-2812767.916846095, -1659704.7880923636]`.

The converted state pose is identical in timestamp and pose to the input
state's `T_w_i`; it appears in the output pose map under ID
`1403636584663555584`. Output pose IDs are the seven original pose IDs plus
that converted ID. Output state map contains only
`1403636584763555584`.

## Exact M8b result

There is one roll/pitch factor (`t_ns=1403636579763555584`). Its measured
rotation and 2x2 information matrix are:

```text
R = [[-0.2969657216667472,  0.08528223091320769, -0.9510721850866076],
     [ 0.07960632547150237, 0.9947476721698187,   0.06434206755810773],
     [ 0.9515640772432896,-0.05660397338697336,  -0.30219496537323987]]
info = [[14378.351063510157, -1072.8930504021657],
        [-1072.8930506613833, 10678.326033266372]]
```

Seven relative-pose factors connect the kf-to-marg ID to each other retained
KF. The first is `i=1403636579763555584`,
`j=1403636580113555456`; its measurement translation is
`[-0.15506096689839458, 0.0012404600842321181, 0.051255521629034545]` and
quaternion `(x,y,z,w)` is
`[-0.005875862140385705, -0.023078930784649287, 0.002403954551583674,
0.9997134880556822]`. Its complete 6x6 information matrix, as well as all
six remaining factors' measurements and matrices, is in
`output.factors.relative_pose` in the fixture. The first information row is
`[2805434.8972435985, -126637.92458052564, 755577.7069345374,
-105350.89194830004, 5628325.834159151, 258506.89870771026]`.

## Gaps and guardrails

- This fixture is a single real MH01 MargData event, not synthetic data and
  not a ground-truth seed. No GT path or evaluator data enters the diagnostic.
- The checked-out upstream source was not modified; the only new source is
  the benchmark diagnostic above. No Rust production file was edited here.
- The fixture records the upstream's full-rank acceptance (`72 -> 48` after
  M8a, rank 48 for factor extraction). A rank-deficient rejection case was
  not run; add one separately if the Rust test requires that negative gate.
- Factors are recovered with the pinned default `mapper_no_factor_weights =
  false`; the complete covariance-propagated information matrices are
  authoritative in the JSON. The diagnostic does not tune thresholds.
