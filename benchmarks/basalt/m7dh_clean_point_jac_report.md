# M7dh clean linearizePoint point/Jacobian capture

Date: 2026-08-23 JST  
Status: **Pass — retained bounded clean GDB capture reached the exact
pixel-filtered factor and final Jacobian write.**

## Factor and artifact

The retained run uses the pinned, unmodified clean artifact:

- commit 0f3b2b52c807f70ff4e2973ce253c73329eea7bc;
- basalt_vio SHA-256
  89e0324ccd04c3b615bf7e945aa32480a2f86524987aa00eaf152dcd6b1d242c;
- libbasalt.so SHA-256
  55f842642b370edf4025afd1d66c7b9d9011e5b4ce08efeb9d76be401b45b2cc.

The exact factor is frame 4, iteration 0, trial 0, track 1, host
(frame 0, cam 0) to target (frame 1, cam 1), with observation pixel bits
41da8172,42d4c7e1. The dynamic ASLR base was 0x769d0fa00000; the run saw
497 entry hits and one matching pixel candidate, then quit at the final
Jacobian stop.

The retained command and raw output are
[target/m7dh_clean_point_jac.cmd](../../target/m7dh_clean_point_jac.cmd) and
[target/m7dh_clean_point_jac.gdb.out](../../target/m7dh_clean_point_jac.gdb.out).
The output is 4,821 bytes, SHA-256
38aff8d902b1911d1eab6714c56aeec18a9f8217980fb80999054aeb1f641e21.

## Captured native values

The target homogeneous point at ELF offset 0x275158, in xmm6 lane order
x,y,z,rho, is:

~~~
bits: bf2fc73e,be94aaa8,3f2ec6b2,3e160ef3
~~~

The caller passes proj=null. Therefore the projection stop is the fallback
store boundary 0x275250, immediately after the projected y store and before
residual subtraction; the residual buffer contains raw projected u/v there:

~~~
projected uv: 41da6d17,42d70d32
raw residual: bc22d800,3f915440
~~~

At 0x275774, the raw fixed-size Eigen blocks are:

~~~
d_res_d_xi (2x6, column-major, 12 words):
4247c89b,c129fef5,c12a8200,428cdaf2,4236ccb5,419a2412,
c2239d47,c3b7255f,43df6969,42231f8d,4314e605,c3af8621

d_res_d_p (2x3, column-major, 6 words):
44446eae,c1e493b0,c20f4734,445399f4,c21720bd,40df22eb
~~~

The machine-readable record, including decoded f32 values and row/column
layouts, is
[target/m7dh_clean_point_jac.json](../../target/m7dh_clean_point_jac.json).

## Bit comparison

The clean point/projection/residual chain is exact against the authoritative
m7bo values:

| quantity | clean live | m7bo | equal |
| --- | --- | --- | --- |
| target homogeneous point | bf2fc73e,be94aaa8,3f2ec6b2,3e160ef3 | same | 4/4 |
| projected uv | 41da6d17,42d70d32 | same | 2/2 |
| raw residual | bc22d800,3f915440 | same | 2/2 |

Against the Rust m7cm iteration-start per-factor detail for track 1 /
target frame 1 cam 1:

| quantity | clean live | m7cm | equal |
| --- | --- | --- | --- |
| target homogeneous point | bf2fc73e,be94aaa8,3f2ec6b2,3e160ef3 | bf2fc73d,be94aaaa,3f2ec6b2,3e160ef3 | 2/4 |
| projected uv | 41da6d17,42d70d32 | 41da6d22,42d70d2e | 0/2 |
| raw residual | bc22d800,3f915440 | bc228000,3f915340 | 0/2 |

The Rust detail's jl is weighted and row-major, while native d_res_d_p is
raw/unweighted and column-major. For a diagnostic layout comparison only,
multiplying native d_res_d_p by the m7cm sqrt_weight (3ff04066) and
transposing gives 3/6 equal lanes against m7cm jl; this is not a same-weight
parity claim because the raw residual and Huber weight differ. Native
d_res_d_xi is a relative-pose block; m7cm jp_target is a weighted absolute
target-pose block and is not a direct lane-equivalent comparison.

No source, clean binary, build, commit, or push was changed. No
pretty-printer or ptype command was used, and the inferior was gone after
capture.
