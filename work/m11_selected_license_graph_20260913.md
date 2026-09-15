# Selected Cargo license graph

Status: **pass_selected_graph_reconciled**  
Captured: `2026-09-13T17:56:52+09:00`  
Schema: `visloc.basalt.selected_license_graph.v1`

This is a locked/offline Cargo graph reconciliation for the canonical `visloc-rs` / `basalt_euroc_vio_demo` profile. It is a license payload checklist, not legal clearance or a final binary/native SBOM.

## Graph and inventory counts

- Cargo reachable packages: **66** (10 local, 56 registry)
- v2 overlay rows selected: **0**; v1-base rows selected: **66** (56 registry + 10 workspace)
- Inventory rows unused by this profile: **53**
- Selected registry rows lacking authoritative v2 per-file license evidence (cache texts listed below): **56**
- Selected registry rows with no named license text in the known local cache: **0**

## Supplemental non-Cargo source coverage

- Manual source dependencies: **1**; status: `pass_supplemental_source_dependency`; release-ready: **true**.
- Known upstream source hashes verified: **4/4**; affected production-file hashes verified: **1/1**.
- AOR license hash verified: **true**; persisted payload: **true**.

Metadata command:

```text
C:\Users\rsasa\.cargo\bin\cargo.exe metadata --format-version 1 --locked --offline --filter-platform x86_64-pc-windows-msvc --manifest-path E:\visloc_archive\ssfm_runset_core_pr_20260819\Cargo.toml --no-default-features
```

## Selected 66-package payload checklist

`v2_evidence_present` means an evidence path from the v2 overlay exists. `cache_only_not_in_v2_inventory` means a local cache license file was found but is not an authoritative v2 payload. `workspace_license_files_present` refers to the root MIT/Apache texts used by local path packages.

| Package | Kind | License | Inventory | Payload status | License text evidence |
|---|---|---|---|---|---|
| `adler2 2.0.1` | registry | `0BSD OR MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/adler2-2.0.1/LICENSE-0BSD, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/adler2-2.0.1/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/adler2-2.0.1/LICENSE-MIT` |
| `approx 0.5.1` | registry | `Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/approx-0.5.1/LICENSE` |
| `autocfg 1.5.0` | registry | `Apache-2.0 OR MIT` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/autocfg-1.5.0/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/autocfg-1.5.0/LICENSE-MIT` |
| `base64 0.22.1` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/base64-0.22.1/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/base64-0.22.1/LICENSE-MIT` |
| `bitflags 1.3.2` | registry | `MIT/Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/bitflags-1.3.2/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/bitflags-1.3.2/LICENSE-MIT` |
| `bytemuck 1.25.0` | registry | `Zlib OR Apache-2.0 OR MIT` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/bytemuck-1.25.0/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/bytemuck-1.25.0/LICENSE-MIT, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/bytemuck-1.25.0/LICENSE-ZLIB` |
| `byteorder-lite 0.1.0` | registry | `Unlicense OR MIT` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/byteorder-lite-0.1.0/LICENSE-MIT` |
| `cfg-if 1.0.4` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/cfg-if-1.0.4/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/cfg-if-1.0.4/LICENSE-MIT` |
| `crc32fast 1.5.0` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/crc32fast-1.5.0/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/crc32fast-1.5.0/LICENSE-MIT` |
| `crossbeam-deque 0.8.6` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/crossbeam-deque-0.8.6/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/crossbeam-deque-0.8.6/LICENSE-MIT` |
| `crossbeam-epoch 0.9.18` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/crossbeam-epoch-0.9.18/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/crossbeam-epoch-0.9.18/LICENSE-MIT` |
| `crossbeam-utils 0.8.21` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/crossbeam-utils-0.8.21/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/crossbeam-utils-0.8.21/LICENSE-MIT` |
| `either 1.16.0` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/either-1.16.0/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/either-1.16.0/LICENSE-MIT` |
| `fdeflate 0.3.7` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/fdeflate-0.3.7/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/fdeflate-0.3.7/LICENSE-MIT` |
| `flate2 1.1.9` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/flate2-1.1.9/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/flate2-1.1.9/LICENSE-MIT` |
| `getrandom 0.2.17` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/getrandom-0.2.17/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/getrandom-0.2.17/LICENSE-MIT` |
| `image 0.25.5` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/image-0.25.5/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/image-0.25.5/LICENSE-MIT` |
| `itoa 1.0.18` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/itoa-1.0.18/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/itoa-1.0.18/LICENSE-MIT` |
| `matrixmultiply 0.3.10` | registry | `MIT/Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/matrixmultiply-0.3.10/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/matrixmultiply-0.3.10/LICENSE-MIT` |
| `memchr 2.8.3` | registry | `Unlicense OR MIT` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/memchr-2.8.3/COPYING, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/memchr-2.8.3/LICENSE-MIT` |
| `miniz_oxide 0.8.9` | registry | `MIT OR Zlib OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/miniz_oxide-0.8.9/LICENSE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/miniz_oxide-0.8.9/LICENSE-APACHE.md, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/miniz_oxide-0.8.9/LICENSE-MIT.md, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/miniz_oxide-0.8.9/LICENSE-ZLIB.md` |
| `nalgebra 0.33.3` | registry | `Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/nalgebra-0.33.3/LICENSE` |
| `nalgebra-macros 0.2.2` | registry | `Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/nalgebra-macros-0.2.2/LICENSE` |
| `nalgebra-sparse 0.10.0` | registry | `Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/nalgebra-sparse-0.10.0/LICENSE` |
| `num-bigint 0.4.6` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/num-bigint-0.4.6/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/num-bigint-0.4.6/LICENSE-MIT` |
| `num-complex 0.4.6` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/num-complex-0.4.6/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/num-complex-0.4.6/LICENSE-MIT` |
| `num-integer 0.1.46` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/num-integer-0.1.46/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/num-integer-0.1.46/LICENSE-MIT` |
| `num-rational 0.4.2` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/num-rational-0.4.2/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/num-rational-0.4.2/LICENSE-MIT` |
| `num-traits 0.2.19` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/num-traits-0.2.19/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/num-traits-0.2.19/LICENSE-MIT` |
| `paste 1.0.15` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/paste-1.0.15/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/paste-1.0.15/LICENSE-MIT` |
| `png 0.17.16` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/png-0.17.16/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/png-0.17.16/LICENSE-MIT` |
| `ppv-lite86 0.2.21` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/ppv-lite86-0.2.21/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/ppv-lite86-0.2.21/LICENSE-MIT` |
| `proc-macro2 1.0.106` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/proc-macro2-1.0.106/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/proc-macro2-1.0.106/LICENSE-MIT` |
| `quote 1.0.45` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/quote-1.0.45/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/quote-1.0.45/LICENSE-MIT` |
| `rand 0.8.6` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/rand-0.8.6/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/rand-0.8.6/LICENSE-MIT` |
| `rand_chacha 0.3.1` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/rand_chacha-0.3.1/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/rand_chacha-0.3.1/LICENSE-MIT` |
| `rand_core 0.6.4` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/rand_core-0.6.4/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/rand_core-0.6.4/LICENSE-MIT` |
| `rawpointer 0.2.1` | registry | `MIT/Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/rawpointer-0.2.1/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/rawpointer-0.2.1/LICENSE-MIT` |
| `rayon 1.12.0` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/rayon-1.12.0/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/rayon-1.12.0/LICENSE-MIT` |
| `rayon-core 1.13.0` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/rayon-core-1.13.0/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/rayon-core-1.13.0/LICENSE-MIT` |
| `safe_arch 0.7.4` | registry | `Zlib OR Apache-2.0 OR MIT` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/safe_arch-0.7.4/LICENSE-APACHE.md, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/safe_arch-0.7.4/LICENSE-MIT.md, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/safe_arch-0.7.4/LICENSE-ZLIB.md` |
| `serde 1.0.229` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/serde-1.0.229/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/serde-1.0.229/LICENSE-MIT` |
| `serde_core 1.0.229` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/serde_core-1.0.229/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/serde_core-1.0.229/LICENSE-MIT` |
| `serde_derive 1.0.229` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/serde_derive-1.0.229/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/serde_derive-1.0.229/LICENSE-MIT` |
| `serde_json 1.0.151` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/serde_json-1.0.151/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/serde_json-1.0.151/LICENSE-MIT` |
| `simba 0.9.1` | registry | `Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/simba-0.9.1/LICENSE` |
| `simd-adler32 0.3.9` | registry | `MIT` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/simd-adler32-0.3.9/LICENSE.md` |
| `syn 2.0.117` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/syn-2.0.117/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/syn-2.0.117/LICENSE-MIT` |
| `syn 3.0.3` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/syn-3.0.3/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/syn-3.0.3/LICENSE-MIT` |
| `thiserror 2.0.18` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/thiserror-2.0.18/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/thiserror-2.0.18/LICENSE-MIT` |
| `thiserror-impl 2.0.18` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/thiserror-impl-2.0.18/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/thiserror-impl-2.0.18/LICENSE-MIT` |
| `typenum 1.20.0` | registry | `MIT OR Apache-2.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/typenum-1.20.0/LICENSE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/typenum-1.20.0/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/typenum-1.20.0/LICENSE-MIT` |
| `unicode-ident 1.0.24` | registry | `(MIT OR Apache-2.0) AND Unicode-3.0` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/unicode-ident-1.0.24/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/unicode-ident-1.0.24/LICENSE-MIT, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/unicode-ident-1.0.24/LICENSE-UNICODE` |
| `visloc-basalt 0.1.0` | local_path | `MIT OR Apache-2.0 (workspace/local)` | v1_base / v1_base_resolved | `workspace_license_files_present` | `LICENSE-APACHE, LICENSE-MIT` |
| `visloc-core 0.1.0` | local_path | `MIT OR Apache-2.0 (workspace/local)` | v1_base / v1_base_resolved | `workspace_license_files_present` | `LICENSE-APACHE, LICENSE-MIT` |
| `visloc-fusion 0.1.0` | local_path | `MIT OR Apache-2.0 (workspace/local)` | v1_base / v1_base_resolved | `workspace_license_files_present` | `LICENSE-APACHE, LICENSE-MIT` |
| `visloc-io 0.1.0` | local_path | `MIT OR Apache-2.0 (workspace/local)` | v1_base / v1_base_resolved | `workspace_license_files_present` | `LICENSE-APACHE, LICENSE-MIT` |
| `visloc-localization 0.1.0` | local_path | `MIT OR Apache-2.0 (workspace/local)` | v1_base / v1_base_resolved | `workspace_license_files_present` | `LICENSE-APACHE, LICENSE-MIT` |
| `visloc-mapping 0.1.0` | local_path | `MIT OR Apache-2.0 (workspace/local)` | v1_base / v1_base_resolved | `workspace_license_files_present` | `LICENSE-APACHE, LICENSE-MIT` |
| `visloc-rs 0.1.0` | local_path | `MIT OR Apache-2.0 (workspace/local)` | v1_base / v1_base_resolved | `workspace_license_files_present` | `LICENSE-APACHE, LICENSE-MIT` |
| `visloc-slam 0.1.0` | local_path | `MIT OR Apache-2.0 (workspace/local)` | v1_base / v1_base_resolved | `workspace_license_files_present` | `LICENSE-APACHE, LICENSE-MIT` |
| `visloc-tracking 0.1.0` | local_path | `MIT OR Apache-2.0 (workspace/local)` | v1_base / v1_base_resolved | `workspace_license_files_present` | `LICENSE-APACHE, LICENSE-MIT` |
| `visloc-vision 0.1.0` | local_path | `MIT OR Apache-2.0 (workspace/local)` | v1_base / v1_base_resolved | `workspace_license_files_present` | `LICENSE-APACHE, LICENSE-MIT` |
| `wide 0.7.33` | registry | `Zlib OR Apache-2.0 OR MIT` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/wide-0.7.33/LICENSE-ZLIB.md` |
| `zerocopy 0.8.48` | registry | `BSD-2-Clause OR Apache-2.0 OR MIT` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/zerocopy-0.8.48/LICENSE-APACHE, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/zerocopy-0.8.48/LICENSE-BSD, C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/zerocopy-0.8.48/LICENSE-MIT` |
| `zmij 1.0.23` | registry | `MIT` | v1_base / v1_base_resolved | `cache_only_not_in_v2_inventory` | `C:/Users/rsasa/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/zmij-1.0.23/LICENSE-MIT` |

## Unused inventory rows

These are present in the 119-row workspace inventory but are not reachable from the selected target-filtered root graph:

`byteorder 1.5.0`, `bytes 1.11.1`, `cc 1.2.61`, `find-msvc-tools 0.1.9`, `http 1.4.0`, `httparse 1.10.1`, `libc 0.2.186`, `libloading 0.8.9`, `libloading 0.9.0`, `log 0.4.29`, `ndarray 0.17.2`, `once_cell 1.21.4`, `ort 2.0.0-rc.12`, `ort-sys 2.0.0-rc.12`, `percent-encoding 2.3.2`, `pin-project-lite 0.2.17`, `portable-atomic 1.13.1`, `portable-atomic-util 0.2.7`, `ring 0.17.14`, `rustls 0.23.40`, `rustls-pki-types 1.14.1`, `rustls-webpki 0.103.13`, `shlex 1.3.0`, `smallvec 1.15.1`, `socks 0.3.4`, `subtle 2.6.1`, `tracing 0.1.44`, `tracing-core 0.1.36`, `untrusted 0.9.0`, `ureq 3.3.0`, `ureq-proto 0.6.0`, `utf8-zero 0.8.1`, `visloc-dpvo-cuda-runtime 0.1.0`, `wasi 0.11.1+wasi-snapshot-preview1`, `webpki-roots 1.0.7`, `winapi 0.3.9`, `winapi-i686-pc-windows-gnu 0.4.0`, `winapi-x86_64-pc-windows-gnu 0.4.0`, `windows-link 0.2.1`, `windows-sys 0.52.0`, `windows-targets 0.52.6`, `windows_aarch64_gnullvm 0.52.6`, `windows_aarch64_msvc 0.52.6`, `windows_i686_gnu 0.52.6`, `windows_i686_gnullvm 0.52.6`, `windows_i686_msvc 0.52.6`, `windows_x86_64_gnu 0.52.6`, `windows_x86_64_gnullvm 0.52.6`, `windows_x86_64_msvc 0.52.6`, `zerocopy-derive 0.8.48`, `zeroize 1.8.2`, `zune-core 0.4.12`, `zune-jpeg 0.4.21`

## Special rows requested for review

| Package | Selected | Inventory status | Exact graph result |
|---|---:|---|---|
| `ort 2.0.0-rc.12` | false | `resolved_with_provenance_caveat` | not reachable under x86_64-pc-windows-msvc and --no-default-features |
| `ort-sys 2.0.0-rc.12` | false | `resolved_with_provenance_caveat` | not reachable under x86_64-pc-windows-msvc and --no-default-features |
| `shlex 1.3.0` | false | `resolved_local_registry_evidence` | not reachable under x86_64-pc-windows-msvc and --no-default-features |
| `socks 0.3.4` | false | `resolved_local_registry_evidence` | not reachable under x86_64-pc-windows-msvc and --no-default-features |
| `winapi 0.3.9` | false | `v1_base_resolved` | not reachable under x86_64-pc-windows-msvc and --no-default-features |
| `winapi-i686-pc-windows-gnu 0.4.0` | false | `resolved_upstream_license_not_crate_included` | not reachable under x86_64-pc-windows-msvc and --no-default-features |
| `winapi-x86_64-pc-windows-gnu 0.4.0` | false | `resolved_upstream_license_not_crate_included` | not reachable under x86_64-pc-windows-msvc and --no-default-features |

## Native dependency boundary

Cargo `links` packages in the selected graph: **1**.

| Package | `links` value | Kind |
|---|---|---|
| `rayon-core 1.13.0` | `rayon-core` | registry |

Cargo `links` packages: **1**; literal native link directives observed: **0**. Cargo metadata does not represent linker/SDK/system-runtime inputs; those must be bound from the final build. vcpkg is not a selected Cargo package, so this report does not infer that it applies to this example.

## Protocol and mapper boundary

The frozen protocol [`basalt_euroc_parity_v1.json`](../benchmarks/basalt/protocols/basalt_euroc_parity_v1.json) specifies 11 sequences, 2 methods, and 3 repetitions (66 VIO cells) plus trajectory/coverage/runtime/RSS metrics. It does **not** specify 52/80/400 as mapper gates. Those records are VIO/control evidence.

Mapper obligations are recorded in [`MAPPER_PROVENANCE.md`](../pipelines/basalt/MAPPER_PROVENANCE.md) and [`m11_mapper_completion_audit_20260903.json`](m11_mapper_completion_audit_20260903.json): real native packet/image/companion → Rust mapper E2E, FEJ/schema4 closure, dense optimization field equivalence where claimed, complete keyframe/landmark lifecycle, and native final map/trajectory comparison.

## Actionable missing payload

- Add authoritative per-license-text records for the selected v1-base registry rows listed in the JSON; local-cache files are not yet release payloads.
- Persist the complete applicable AOR license text in the reviewed release payload, then rerun `verify_supplemental_source_dependencies.py`; the current supplemental status is pending/non-release until that payload is present.
- Bind the selected Cargo `links` packages (if any) and the final linker/SDK/system-runtime/native inputs; do not substitute the unrelated pinned vcpkg oracle.
- Keep 52/80/400 as VIO twin/control evidence and produce mapper-specific evidence under the mapper obligations above.
- Preserve v1/v2 inventories unchanged; this report only reconciles the selected profile.
