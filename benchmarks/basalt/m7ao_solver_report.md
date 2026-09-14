# M7ao frame-4 f32 QR/solve audit

Date: 2026-08-21 (JST)  
Pinned upstream: Basalt `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`  
Dataset: EuRoC `MH_01_easy`, sensor-only, first five frames

## Result

No solver change is retained in `aom.rs`.  The temporary f32 solver was
replaced with an Eigen-compatible diagonal-pivot LDLT for a controlled replay,
then restored to the prior nalgebra f32 Cholesky path because it did not
improve the LM decision sequence.  The exact solver matches the standalone
upstream H/b oracle closely, but applying it to the current Rust H/b still
produces `AAARAAAA` rather than upstream `AAAAAAAA`.

The measured next blocker is therefore the full reduced system outside the
visual QR rows: the visual Q2/H/b differences are small, while a nonvisual/IMU
cross-block differs by millions.  Further QR or solve tuning in `aom.rs` is not
justified until that H/b input divergence is resolved.

## Q2/reduced-row oracle

These are frame-4 iteration-0 records.  Rows are compared after the per-row QR
sign convention is aligned; all three selected tracks had the same active row
sign.  `Q2` denotes the stored reduced rows after eliminating the landmark
columns (the trace field is `reduced_rows`).

| Track | Rows x columns | max `abs(Q2_up - Q2_rs)` | max `abs(rhs_up - rhs_rs)` |
| ---: | ---: | ---: | ---: |
| 1 | 12 x 75 | `1.8371582e-2` | `8.197874e-5` |
| 2 | 20 x 75 | `6.469727e-3` | `6.556511e-5` |
| 117 | 20 x 75 | `5.931854e-4` | `1.986923e-4` |

Accumulating all 61 Q2 factors in f32 gives:

| Quantity | Upstream | Rust | Difference (Rust - upstream) |
| --- | ---: | ---: | ---: |
| max visual `Q2ᵀQ2` difference (at [20,20]) | `65368512` | `65366540` | `-1972` |
| max visual `Q2ᵀr` difference (at [4]) | `-242568.65625` | `-242577.609375` | `-8.953125` |

The visual Gram difference has RMS `62.2673`; the visual gradient difference
has RMS `1.58914`.  These are not large enough to explain the full H mismatch.

## Full H/b and damping

The iteration-start full reduced system is 75 x 75 before adding the recorded
diagonal damping.  Selected exact f32 values are:

| Entry | Upstream | Rust | Difference |
| --- | ---: | ---: | ---: |
| `H[0,0]` | `479284352` | `479284192` | `-160` |
| `H[19,19]` | `557600000` | `557604224` | `+4224` |
| `H[24,24]` | `4000629248` | `4000628736` | `-512` |
| `H[45,49]` | `-1666753.125` | `-14007340` | `-12340587` |
| `H[49,49]` | `533197056` | `533453568` | `+256512` |
| `b[0]` | `7113.7900390625` | `7114.30615234375` | `+0.51611328125` |
| `b[19]` | `-32461.0546875` | `-32402.4765625` | `+58.578125` |
| `b[49]` | `96862.15625` | `96841.5703125` | `-20.5859375` |

Full H max difference is `12340587` (RMS `614426.06`), at `[45,49]`; full b
max difference is `58.578125` (RMS `10.2989`), at `[19]`.  The f32 damping
diagonal max difference is `33.3984375`, at index 64 (`27397.78515625` upstream
versus `27431.18359375` Rust).  Thus the dominant discrepancy is already in
the nonvisual/IMU part of the assembled system, not in the AOM Q2 rows.

## Delta and LM comparison

The pinned upstream frame-4 replay starts at `4215.9326171875`; the current
Rust f32 replay starts at `4216.1083984375` (`+0.17578125`).  Trial values are:

| Iteration | Upstream model / actual | Rust Cholesky model / actual | Eigen-LDLT trial model / actual |
| ---: | ---: | ---: | ---: |
| 0 | `1866.805176 / 412.955017` | `1866.861572 / 412.910431` | `1866.885742 / 412.995636` |
| 1 | `305.371582 / 277.509705` | `305.330902 / 277.683136` | `305.355988 / 277.689270` |
| 2 | `264.806244 / 259.794006` | `264.725739 / 261.591766` | `264.722321 / 261.592072` |
| 3 | `254.890900 / 253.377335` | `254.765686 / 262.969208` | `254.763596 / 263.101959` |

The upstream recorded delta is reproduced by the standalone Eigen-compatible
solver to max absolute errors `1.1995e-5`, `2.6785e-6`, `4.5132e-6`, and
`5.4952e-5` for iterations 0--3 respectively.  On the Rust H/b, however, the
same solver produced `412.995636`, `277.689270`, and `261.592072` for the first
three actual costs and still rejected iteration 3.  The sequence remained
`AAARAAAA`; the current Cholesky path was marginally closer on the first
trial, so the LDLT experiment was reverted.

## Verification and artifacts

- Temporary exact-Eigen f32 LDLT: `cargo test --release -p visloc-basalt vio::aom::tests --lib` — **18 passed**.
- Current Cholesky replay: `target/m7ao_current_f32.jsonl`.
- Temporary Eigen-LDLT replay: `target/m7ao_eig_ldlt_f32.jsonl`.
- Prior non-pivot LDLT replay: `target/m7ao_ldlt_f32.jsonl`.
- Pinned upstream detail trace: `target/m7al_upstream_f4_20260821T000200Z/iteration.jsonl`.

After restoring the solver, the same focused cargo command was blocked by the
shared dirty workspace's unrelated missing
`pipelines/basalt/src/mapper/triangulation.rs` (`E0583`); no mapper file was
created or changed for this audit.

Next investigation should compare the frame-4 nonvisual/IMU block assembly and
state/linearization inputs that generate `H[45,49]` and `b[19]`.  Keep the
current `aom.rs` solver path unchanged pending that boundary check.
