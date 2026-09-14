# M7bh landmark frontier ownership

Date: 2026-08-23 JST  
Scope: frame-4 landmark linearization against the pinned Basalt checkout
`0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.

## Result

The fresh combined Rust artifact
`target/m7_optflow_norm_fresh5_detail_20260823.jsonl` matches
`target/m7al_upstream_f4_20260821T000200Z/iteration.jsonl` for 39/61
landmarks.  The 22 mismatches are tracks
`3, 11, 12, 23, 27, 29, 30, 38, 40, 47, 49, 58, 63, 75, 92, 101, 102,
108, 111, 119, 120, 131`.

The local frame-to-upstream timestamp alignment was:

```text
local 0 -> 1403636579763555584
local 1 -> 1403636579813555456
local 2 -> 1403636579863555584
local 3 -> 1403636579913555456
local 4 -> 1403636579963555584
```

For every mismatched track, host identity, observation keys, and observation
counts agree.  The earliest differing operand is the frame-0, camera-1 source
pixel; therefore all 22 are owned by the frontend observation path.  The
camera bearing, relative pose, DLT `A`, raw JacobiSVD `V`, normalization, and
stereographic/rho stages cannot be classified as independent landmark
divergences because they do not receive identical operands.

## Mismatch ownership table

Values are the decoded f32 pixel coordinates from the two artifacts.

| track | pinned upstream `(u,v)` | Rust `(u,v)` | earliest boundary | owner |
|---:|---|---|---|---|
| 3 | `(46.3386383056641, 220.440353393555)` | `(46.3386344909668, 220.440353393555)` | source/target pixel | frontend (explicit skip) |
| 11 | `(77.8854217529297, 145.35514831543)` | `(77.8854217529297, 145.355163574219)` | source/target pixel | frontend |
| 12 | `(102.103363037109, 222.634292602539)` | `(102.103370666504, 222.63427734375)` | source/target pixel | frontend |
| 23 | `(134.694595336914, 309.95947265625)` | `(134.694595336914, 309.959442138672)` | source/target pixel | frontend |
| 27 | `(190.11701965332, 66.7915725708008)` | `(190.117004394531, 66.7915725708008)` | source/target pixel | frontend |
| 29 | `(171.093353271484, 138.289779663086)` | `(171.093338012695, 138.289779663086)` | source/target pixel | frontend |
| 30 | `(192.716949462891, 191.971405029297)` | `(192.716949462891, 191.971389770508)` | source/target pixel | frontend |
| 38 | `(246.172058105469, 133.184432983398)` | `(246.172073364258, 133.184417724609)` | source/target pixel | frontend |
| 40 | `(207.957183837891, 259.507385253906)` | `(207.957214355469, 259.507415771484)` | source/target pixel | frontend |
| 47 | `(297.050994873047, 162.916000366211)` | `(297.051025390625, 162.916000366211)` | source/target pixel | frontend |
| 49 | `(273.171752929688, 261.601287841797)` | `(273.171722412109, 261.601287841797)` | source/target pixel | frontend |
| 58 | `(326.982543945312, 252.694900512695)` | `(326.982543945312, 252.694915771484)` | source/target pixel | frontend |
| 63 | `(379.971466064453, 61.7161674499512)` | `(379.971435546875, 61.7161636352539)` | source/target pixel | frontend |
| 75 | `(440.763946533203, 206.258178710938)` | `(440.763916015625, 206.258178710938)` | source/target pixel | frontend |
| 92 | `(529.356567382812, 161.764862060547)` | `(529.356506347656, 161.764862060547)` | source/target pixel | frontend |
| 101 | `(584.677490234375, 164.249557495117)` | `(584.677551269531, 164.249572753906)` | source/target pixel | frontend |
| 102 | `(593.489379882812, 197.381118774414)` | `(593.489379882812, 197.381088256836)` | source/target pixel | frontend |
| 108 | `(656.236328125, 37.9774742126465)` | `(656.236389160156, 37.9774513244629)` | source/target pixel | frontend |
| 111 | `(629.698303222656, 208.216888427734)` | `(629.698303222656, 208.216873168945)` | source/target pixel | frontend |
| 119 | `(694.225158691406, 132.557495117188)` | `(694.22509765625, 132.557510375977)` | source/target pixel | frontend |
| 120 | `(663.622314453125, 198.792861938477)` | `(663.622253417969, 198.792861938477)` | source/target pixel | frontend |
| 131 | `(708.016540527344, 307.462768554688)` | `(708.016540527344, 307.462738037109)` | source/target pixel | frontend |

## Action

No identical observation/pose operand mismatch was found.  Consequently no
general literal Eigen/Sophus landmark change is justified, and `landmarks.rs`
was not modified for this frontier.  Track 3 remains explicitly skipped here
because its source-pixel discrepancy is already assigned to the frontend
agent.
