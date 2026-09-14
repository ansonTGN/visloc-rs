# M7ay Double Sphere f32 unprojection FMA audit

Date: 2026-08-22 JST

## Scope

This audit closes the first Track 2 mismatch at the Double Sphere raw f32
unprojection boundary. The f64 `DoubleSphereCamera::unproject_raw` path was
left unchanged; only `unproject_raw_f32` uses the native upstream scalar
expression order.

The native probe is `target/m7_track2_camera_probe.cpp`, compiled against the
pinned Basalt/Eigen tree at `/root/visloc-basalt-oracle-0f3b2b52` with
`-O3 -march=native -DEIGEN_DONT_PARALLELIZE`. The authoritative header is
`thirdparty/vcpkg/buildtrees/basalt-headers/.../include/basalt/camera/double_sphere_camera.hpp`.

## Native contraction audit

The `-S -fverbose-asm` output for `DoubleSphereCamera<float>::unproject`
shows these scalar contractions:

| source expression | native instruction | Rust expression |
| --- | --- | --- |
| `mx * mx + my * my` | `vfmadd231ss` | `mx.mul_add(mx, my * my)` |
| `1 - (2 * alpha - 1) * r2` | `vfnmadd132ss` | `(-(2 * alpha - 1)).mul_add(r2, 1)` |
| `alpha * sqrt2 + 1 - alpha` | `vfmadd132ss` then subtract | `alpha.mul_add(sqrt2, 1) - alpha` |
| `1 - alpha² * r2` | `vfnmadd132ss` | `(-alpha²).mul_add(r2, 1)` |
| `mz² + r2` | `vfmadd132ss` | `mz.mul_add(mz, r2)` |
| `mz² + (1 - xi²) * r2` | `vfmadd231ss` | `mz.mul_add(mz, (1 - xi²) * r2)` |
| `mz * xi + sqrt1` | `vfmadd231ss` | `mz.mul_add(xi, sqrt1)` |
| `k * mz - xi` | `vfmsub132ss` | `k.mul_add(mz, -xi)` |

The first pre-fix mismatch was `norm1` for Track 2 camera 0:
upstream `3fbe4853` versus Rust `3fbe4852`. For camera 1, the first mismatch
was `norm2`: upstream `3f75007f` versus Rust `3f750081`.

## Exact endpoint bits

The pinned native probe reports:

```text
track2 cam0 raw = bf214708,be84d05b,3f3b6451
track2 cam1 raw = bf24c3d6,be79c13a,3f39b6ef
pixel (97,258) raw = bf0cf651,3c91df79,3f55a58a
```

The direct comparison for the pre-existing pixel is:

| source | x | y | z |
| --- | --- | --- | --- |
| old Rust expected | `bf0cf651` | `3c91df7a` | `3f55a58b` |
| pinned native probe | `bf0cf651` | `3c91df79` | `3f55a58a` |
| current Rust | `bf0cf651` | `3c91df79` | `3f55a58a` |

The Rust f32 path now reproduces all three native triples exactly. The stale
old expected bits were updated to the authoritative native values, and the
camera test module now passes 5/5.

## Verification

```text
C:\Users\rsasa\.cargo\bin\cargo.exe test --release -p visloc-basalt --lib camera::tests -- --nocapture
5 passed, 0 failed

C:\Users\rsasa\.cargo\bin\cargo.exe test --release -p visloc-basalt --lib vio::landmarks::tests -- --nocapture
11 passed, 0 failed
```

The exact Track 2 camera fixture and both Track 1/Track 2 landmark fixtures
remain active. The temporary `VISLOC_BASALT_TRIANG_TRACE` estimator dump was
removed from `estimator.rs`; no `M7_TRI`/`M7_TRI_BITS` production dump remains.
