# Candidate selected-profile license payload

Status: **pass_candidate_selected_license_payload_staged**  
Captured: `2026-09-13T17:57:30+09:00`  
Schema: `visloc.basalt.selected_license_payload.v1`

This is a candidate selected-profile license payload, not legal clearance, a final release archive, or a native SDK/CRT SBOM.

## Scope and verification

- Selected profile: `x86_64-pc-windows-msvc`, `basalt_euroc_vio_demo`, `--no-default-features`.
- Packages: **66** (56 registry + 10 local).
- Unique payload files: **115**; source/destination hash mismatches: **0**.
- Cargo.lock identities verified: **66**; .cargo-checksum verified: **0**; .crate verified: **0**.
- Optional archive evidence unavailable: **56**; optional .cargo-checksum evidence unavailable: **56**.

## Supplemental non-Cargo source coverage

- Manual source dependencies: **1**; status: `pass_supplemental_source_dependency`; release-ready: **true**.
- Known upstream source hashes verified: **4/4**; affected production-file hashes verified: **1/1**.
- AOR license hash verified: **true**; persisted payload: **true**.
- AOR license payload referenced/verified: **true**; included in this candidate's `files`: **false**. This candidate is not a self-contained combined release.

## Payload files

| Destination | Source package/version | SPDX | Bytes | SHA-256 |
|---|---|---|---:|---|
| `licenses/basalt/benchmarks/basalt/NOTICE` | `basalt-upstream 0f3b2b52c807f70ff4e2973ce253c73329eea7bc` | `BSD-3-Clause` | 5814 | `303760eced8131574a1d757be6f656b02672161d45ad5b466a20066d1ecfc3f9` |
| `licenses/basalt/pipelines/basalt/NOTICE` | `basalt-upstream 0f3b2b52c807f70ff4e2973ce253c73329eea7bc` | `BSD-3-Clause` | 10949 | `8a5fc2f9884fb4b39eaed719da407b772993a8e6ecc5ce77463b2200ed0cdbef` |
| `licenses/registry/adler2-2.0.1/LICENSE-0BSD` | `adler2 2.0.1` | `0BSD OR MIT OR Apache-2.0` | 665 | `861399f8c21c042b110517e76dc6b63a2b334276c8cf17412fc3c8908ca8dc17` |
| `licenses/registry/adler2-2.0.1/LICENSE-APACHE` | `adler2 2.0.1` | `0BSD OR MIT OR Apache-2.0` | 10860 | `8ada45cd9f843acf64e4722ae262c622a2b3b3007c7310ef36ac1061a30f6adb` |
| `licenses/registry/adler2-2.0.1/LICENSE-MIT` | `adler2 2.0.1` | `0BSD OR MIT OR Apache-2.0` | 1023 | `23f18e03dc49df91622fe2a76176497404e46ced8a715d9d2b67a7446571cca3` |
| `licenses/registry/approx-0.5.1/LICENSE` | `approx 0.5.1` | `Apache-2.0` | 11560 | `3ddf9be5c28fe27dad143a5dc76eea25222ad1dd68934a047064e56ed2fa40c5` |
| `licenses/registry/autocfg-1.5.0/LICENSE-APACHE` | `autocfg 1.5.0` | `Apache-2.0 OR MIT` | 10847 | `a60eea817514531668d7e00765731449fe14d059d3249e0bc93b36de45f759f2` |
| `licenses/registry/autocfg-1.5.0/LICENSE-MIT` | `autocfg 1.5.0` | `Apache-2.0 OR MIT` | 1054 | `27995d58ad5c1145c1a8cd86244ce844886958a35eb2b78c6b772748669999ac` |
| `licenses/registry/base64-0.22.1/LICENSE-APACHE` | `base64 0.22.1` | `MIT OR Apache-2.0` | 10847 | `a60eea817514531668d7e00765731449fe14d059d3249e0bc93b36de45f759f2` |
| `licenses/registry/base64-0.22.1/LICENSE-MIT` | `base64 0.22.1` | `MIT OR Apache-2.0` | 1076 | `0dd882e53de11566d50f8e8e2d5a651bcf3fabee4987d70f306233cf39094ba7` |
| `licenses/registry/bitflags-1.3.2/LICENSE-APACHE` | `bitflags 1.3.2` | `MIT/Apache-2.0` | 10847 | `a60eea817514531668d7e00765731449fe14d059d3249e0bc93b36de45f759f2` |
| `licenses/registry/bitflags-1.3.2/LICENSE-MIT` | `bitflags 1.3.2` | `MIT/Apache-2.0` | 1071 | `6485b8ed310d3f0340bf1ad1f47645069ce4069dcc6bb46c7d5c6faf41de1fdb` |
| `licenses/registry/bytemuck-1.25.0/LICENSE-APACHE` | `bytemuck 1.25.0` | `Zlib OR Apache-2.0 OR MIT` | 10398 | `e3ba223bb1423f0aad8c3dfce0fe3148db48926d41e6fbc3afbbf5ff9e1c89cb` |
| `licenses/registry/bytemuck-1.25.0/LICENSE-MIT` | `bytemuck 1.25.0` | `Zlib OR Apache-2.0 OR MIT` | 1110 | `9df9ba60a11af705f2e451b53762686e615d86f76b169cf075c3237730dbd7e2` |
| `licenses/registry/bytemuck-1.25.0/LICENSE-ZLIB` | `bytemuck 1.25.0` | `Zlib OR Apache-2.0 OR MIT` | 851 | `84b34dd7608f7fb9b17bd588a6bf392bf7de504e2716f024a77d89f1b145a151` |
| `licenses/registry/byteorder-lite-0.1.0/LICENSE-MIT` | `byteorder-lite 0.1.0` | `Unlicense OR MIT` | 1081 | `0f96a83840e146e43c0ec96a22ec1f392e0680e6c1226e6f3ba87e0740af850f` |
| `licenses/registry/cfg-if-1.0.4/LICENSE-APACHE` | `cfg-if 1.0.4` | `MIT OR Apache-2.0` | 10847 | `a60eea817514531668d7e00765731449fe14d059d3249e0bc93b36de45f759f2` |
| `licenses/registry/cfg-if-1.0.4/LICENSE-MIT` | `cfg-if 1.0.4` | `MIT OR Apache-2.0` | 1057 | `378f5840b258e2779c39418f3f2d7b2ba96f1c7917dd6be0713f88305dbda397` |
| `licenses/registry/crc32fast-1.5.0/LICENSE-APACHE` | `crc32fast 1.5.0` | `MIT OR Apache-2.0` | 11358 | `c6596eb7be8581c18be736c846fb9173b69eccf6ef94c5135893ec56bd92ba08` |
| `licenses/registry/crc32fast-1.5.0/LICENSE-MIT` | `crc32fast 1.5.0` | `MIT OR Apache-2.0` | 1097 | `61d383b05b87d78f94d2937e2580cce47226d17823c0430fbcad09596537efcf` |
| `licenses/registry/crossbeam-deque-0.8.6/LICENSE-APACHE` | `crossbeam-deque 0.8.6` | `MIT OR Apache-2.0` | 10847 | `a60eea817514531668d7e00765731449fe14d059d3249e0bc93b36de45f759f2` |
| `licenses/registry/crossbeam-deque-0.8.6/LICENSE-MIT` | `crossbeam-deque 0.8.6` | `MIT OR Apache-2.0` | 1099 | `5734ed989dfca1f625b40281ee9f4530f91b2411ec01cb748223e7eb87e201ab` |
| `licenses/registry/crossbeam-epoch-0.9.18/LICENSE-APACHE` | `crossbeam-epoch 0.9.18` | `MIT OR Apache-2.0` | 10847 | `a60eea817514531668d7e00765731449fe14d059d3249e0bc93b36de45f759f2` |
| `licenses/registry/crossbeam-epoch-0.9.18/LICENSE-MIT` | `crossbeam-epoch 0.9.18` | `MIT OR Apache-2.0` | 1099 | `5734ed989dfca1f625b40281ee9f4530f91b2411ec01cb748223e7eb87e201ab` |
| `licenses/registry/crossbeam-utils-0.8.21/LICENSE-APACHE` | `crossbeam-utils 0.8.21` | `MIT OR Apache-2.0` | 10847 | `a60eea817514531668d7e00765731449fe14d059d3249e0bc93b36de45f759f2` |
| `licenses/registry/crossbeam-utils-0.8.21/LICENSE-MIT` | `crossbeam-utils 0.8.21` | `MIT OR Apache-2.0` | 1099 | `5734ed989dfca1f625b40281ee9f4530f91b2411ec01cb748223e7eb87e201ab` |
| `licenses/registry/either-1.16.0/LICENSE-APACHE` | `either 1.16.0` | `MIT OR Apache-2.0` | 10847 | `a60eea817514531668d7e00765731449fe14d059d3249e0bc93b36de45f759f2` |
| `licenses/registry/either-1.16.0/LICENSE-MIT` | `either 1.16.0` | `MIT OR Apache-2.0` | 1043 | `7576269ea71f767b99297934c0b2367532690f8c4badc695edf8e04ab6a1e545` |
| `licenses/registry/fdeflate-0.3.7/LICENSE-APACHE` | `fdeflate 0.3.7` | `MIT OR Apache-2.0` | 10174 | `0d542e0c8804e39aa7f37eb00da5a762149dc682d7829451287e11b938e94594` |
| `licenses/registry/fdeflate-0.3.7/LICENSE-MIT` | `fdeflate 0.3.7` | `MIT OR Apache-2.0` | 1036 | `c77a4cf9da729987d0fe7ccd811e3bd27393914ddf3d23467c18cc22954513b3` |
| `licenses/registry/flate2-1.1.9/LICENSE-APACHE` | `flate2 1.1.9` | `MIT OR Apache-2.0` | 10847 | `a60eea817514531668d7e00765731449fe14d059d3249e0bc93b36de45f759f2` |
| `licenses/registry/flate2-1.1.9/LICENSE-MIT` | `flate2 1.1.9` | `MIT OR Apache-2.0` | 1062 | `025436edff4cfcdde17a5811fdea78892d8482efd1abdec5a17872d07a4f2112` |
| `licenses/registry/getrandom-0.2.17/LICENSE-APACHE` | `getrandom 0.2.17` | `MIT OR Apache-2.0` | 10849 | `aaff376532ea30a0cd5330b9502ad4a4c8bf769c539c87ffe78819d188a18ebf` |
| `licenses/registry/getrandom-0.2.17/LICENSE-MIT` | `getrandom 0.2.17` | `MIT OR Apache-2.0` | 1130 | `42fa16951ce7f24b5a467a40e5b449a1d41e662f97ca779864f053f39e097737` |
| `licenses/registry/image-0.25.5/LICENSE-APACHE` | `image 0.25.5` | `MIT OR Apache-2.0` | 10174 | `0d542e0c8804e39aa7f37eb00da5a762149dc682d7829451287e11b938e94594` |
| `licenses/registry/image-0.25.5/LICENSE-MIT` | `image 0.25.5` | `MIT OR Apache-2.0` | 1036 | `c77a4cf9da729987d0fe7ccd811e3bd27393914ddf3d23467c18cc22954513b3` |
| `licenses/registry/itoa-1.0.18/LICENSE-APACHE` | `itoa 1.0.18` | `MIT OR Apache-2.0` | 9723 | `62c7a1e35f56406896d7aa7ca52d0cc0d272ac022b5d2796e7d6905db8a3636a` |
| `licenses/registry/itoa-1.0.18/LICENSE-MIT` | `itoa 1.0.18` | `MIT OR Apache-2.0` | 1023 | `23f18e03dc49df91622fe2a76176497404e46ced8a715d9d2b67a7446571cca3` |
| `licenses/registry/matrixmultiply-0.3.10/LICENSE-APACHE` | `matrixmultiply 0.3.10` | `MIT/Apache-2.0` | 10847 | `a60eea817514531668d7e00765731449fe14d059d3249e0bc93b36de45f759f2` |
| `licenses/registry/matrixmultiply-0.3.10/LICENSE-MIT` | `matrixmultiply 0.3.10` | `MIT/Apache-2.0` | 1159 | `792d075c7bad6dac258a44e799eb64cbf465e24d9932d27669be08c5ec957e27` |
| `licenses/registry/memchr-2.8.3/COPYING` | `memchr 2.8.3` | `Unlicense OR MIT` | 126 | `01c266bced4a434da0051174d6bee16a4c82cf634e2679b6155d40d75012390f` |
| `licenses/registry/memchr-2.8.3/LICENSE-MIT` | `memchr 2.8.3` | `Unlicense OR MIT` | 1081 | `0f96a83840e146e43c0ec96a22ec1f392e0680e6c1226e6f3ba87e0740af850f` |
| `licenses/registry/miniz_oxide-0.8.9/LICENSE` | `miniz_oxide 0.8.9` | `MIT OR Zlib OR Apache-2.0` | 1213 | `4108245a1f2df9d4e94df8abed5b4ba0759bb2f9b40a6b939f1be141077ae50b` |
| `licenses/registry/miniz_oxide-0.8.9/LICENSE-APACHE.md` | `miniz_oxide 0.8.9` | `MIT OR Zlib OR Apache-2.0` | 10174 | `0d542e0c8804e39aa7f37eb00da5a762149dc682d7829451287e11b938e94594` |
| `licenses/registry/miniz_oxide-0.8.9/LICENSE-MIT.md` | `miniz_oxide 0.8.9` | `MIT OR Zlib OR Apache-2.0` | 1212 | `799e9ca9d179295ef372f25d3769cdda7d25bb2668add6a6a1e22d1e4c678b8d` |
| `licenses/registry/miniz_oxide-0.8.9/LICENSE-ZLIB.md` | `miniz_oxide 0.8.9` | `MIT OR Zlib OR Apache-2.0` | 984 | `0a54e647fe54104658b5e563c04c6f9edf251710e47bce692e0bd990a4ddaa39` |
| `licenses/registry/nalgebra-0.33.3/LICENSE` | `nalgebra 0.33.3` | `Apache-2.0` | 11548 | `86ae51c591e742d80ce56823d35e2e176b53e38a99bd8499c85dae31ef4fe1f7` |
| `licenses/registry/nalgebra-macros-0.2.2/LICENSE` | `nalgebra-macros 0.2.2` | `Apache-2.0` | 10 | `1e881ecf2862d01e6e5bc2b861e46886d2a6fb01499c0c508a209a7271b13cf2` |
| `licenses/registry/nalgebra-sparse-0.10.0/LICENSE` | `nalgebra-sparse 0.10.0` | `Apache-2.0` | 10 | `1e881ecf2862d01e6e5bc2b861e46886d2a6fb01499c0c508a209a7271b13cf2` |
| `licenses/registry/num-bigint-0.4.6/LICENSE-APACHE` | `num-bigint 0.4.6` | `MIT OR Apache-2.0` | 10847 | `a60eea817514531668d7e00765731449fe14d059d3249e0bc93b36de45f759f2` |
| `licenses/registry/num-bigint-0.4.6/LICENSE-MIT` | `num-bigint 0.4.6` | `MIT OR Apache-2.0` | 1071 | `6485b8ed310d3f0340bf1ad1f47645069ce4069dcc6bb46c7d5c6faf41de1fdb` |
| `licenses/registry/num-complex-0.4.6/LICENSE-APACHE` | `num-complex 0.4.6` | `MIT OR Apache-2.0` | 10847 | `a60eea817514531668d7e00765731449fe14d059d3249e0bc93b36de45f759f2` |
| `licenses/registry/num-complex-0.4.6/LICENSE-MIT` | `num-complex 0.4.6` | `MIT OR Apache-2.0` | 1071 | `6485b8ed310d3f0340bf1ad1f47645069ce4069dcc6bb46c7d5c6faf41de1fdb` |
| `licenses/registry/num-integer-0.1.46/LICENSE-APACHE` | `num-integer 0.1.46` | `MIT OR Apache-2.0` | 10847 | `a60eea817514531668d7e00765731449fe14d059d3249e0bc93b36de45f759f2` |
| `licenses/registry/num-integer-0.1.46/LICENSE-MIT` | `num-integer 0.1.46` | `MIT OR Apache-2.0` | 1071 | `6485b8ed310d3f0340bf1ad1f47645069ce4069dcc6bb46c7d5c6faf41de1fdb` |
| `licenses/registry/num-rational-0.4.2/LICENSE-APACHE` | `num-rational 0.4.2` | `MIT OR Apache-2.0` | 10847 | `a60eea817514531668d7e00765731449fe14d059d3249e0bc93b36de45f759f2` |
| `licenses/registry/num-rational-0.4.2/LICENSE-MIT` | `num-rational 0.4.2` | `MIT OR Apache-2.0` | 1071 | `6485b8ed310d3f0340bf1ad1f47645069ce4069dcc6bb46c7d5c6faf41de1fdb` |
| `licenses/registry/num-traits-0.2.19/LICENSE-APACHE` | `num-traits 0.2.19` | `MIT OR Apache-2.0` | 10847 | `a60eea817514531668d7e00765731449fe14d059d3249e0bc93b36de45f759f2` |
| `licenses/registry/num-traits-0.2.19/LICENSE-MIT` | `num-traits 0.2.19` | `MIT OR Apache-2.0` | 1071 | `6485b8ed310d3f0340bf1ad1f47645069ce4069dcc6bb46c7d5c6faf41de1fdb` |
| `licenses/registry/paste-1.0.15/LICENSE-APACHE` | `paste 1.0.15` | `MIT OR Apache-2.0` | 9723 | `62c7a1e35f56406896d7aa7ca52d0cc0d272ac022b5d2796e7d6905db8a3636a` |
| `licenses/registry/paste-1.0.15/LICENSE-MIT` | `paste 1.0.15` | `MIT OR Apache-2.0` | 1023 | `23f18e03dc49df91622fe2a76176497404e46ced8a715d9d2b67a7446571cca3` |
| `licenses/registry/png-0.17.16/LICENSE-APACHE` | `png 0.17.16` | `MIT OR Apache-2.0` | 10847 | `a60eea817514531668d7e00765731449fe14d059d3249e0bc93b36de45f759f2` |
| `licenses/registry/png-0.17.16/LICENSE-MIT` | `png 0.17.16` | `MIT OR Apache-2.0` | 1048 | `eaf40297c75da471f7cda1f3458e8d91b4b2ec866e609527a13acfa93b638652` |
| `licenses/registry/ppv-lite86-0.2.21/LICENSE-APACHE` | `ppv-lite86 0.2.21` | `MIT OR Apache-2.0` | 10854 | `0218327e7a480793ffdd4eb792379a9709e5c135c7ba267f709d6f6d4d70af0a` |
| `licenses/registry/ppv-lite86-0.2.21/LICENSE-MIT` | `ppv-lite86 0.2.21` | `MIT OR Apache-2.0` | 1076 | `4cada0bd02ea3692eee6f16400d86c6508bbd3bafb2b65fed0419f36d4f83e8f` |
| `licenses/registry/proc-macro2-1.0.106/LICENSE-APACHE` | `proc-macro2 1.0.106` | `MIT OR Apache-2.0` | 9723 | `62c7a1e35f56406896d7aa7ca52d0cc0d272ac022b5d2796e7d6905db8a3636a` |
| `licenses/registry/proc-macro2-1.0.106/LICENSE-MIT` | `proc-macro2 1.0.106` | `MIT OR Apache-2.0` | 1023 | `23f18e03dc49df91622fe2a76176497404e46ced8a715d9d2b67a7446571cca3` |
| `licenses/registry/quote-1.0.45/LICENSE-APACHE` | `quote 1.0.45` | `MIT OR Apache-2.0` | 9723 | `62c7a1e35f56406896d7aa7ca52d0cc0d272ac022b5d2796e7d6905db8a3636a` |
| `licenses/registry/quote-1.0.45/LICENSE-MIT` | `quote 1.0.45` | `MIT OR Apache-2.0` | 1023 | `23f18e03dc49df91622fe2a76176497404e46ced8a715d9d2b67a7446571cca3` |
| `licenses/registry/rand-0.8.6/LICENSE-APACHE` | `rand 0.8.6` | `MIT OR Apache-2.0` | 9724 | `35242e7a83f69875e6edeff02291e688c97caafe2f8902e4e19b49d3e78b4cab` |
| `licenses/registry/rand-0.8.6/LICENSE-MIT` | `rand 0.8.6` | `MIT OR Apache-2.0` | 1117 | `209fbbe0ad52d9235e37badf9cadfe4dbdc87203179c0899e738b39ade42177b` |
| `licenses/registry/rand_chacha-0.3.1/LICENSE-APACHE` | `rand_chacha 0.3.1` | `MIT OR Apache-2.0` | 10849 | `aaff376532ea30a0cd5330b9502ad4a4c8bf769c539c87ffe78819d188a18ebf` |
| `licenses/registry/rand_chacha-0.3.1/LICENSE-MIT` | `rand_chacha 0.3.1` | `MIT OR Apache-2.0` | 1117 | `209fbbe0ad52d9235e37badf9cadfe4dbdc87203179c0899e738b39ade42177b` |
| `licenses/registry/rand_core-0.6.4/LICENSE-APACHE` | `rand_core 0.6.4` | `MIT OR Apache-2.0` | 10282 | `6df43f6f4b5d4587f3d8d71e45532c688fd168afa5fe89d571cb32fa09c4ef51` |
| `licenses/registry/rand_core-0.6.4/LICENSE-MIT` | `rand_core 0.6.4` | `MIT OR Apache-2.0` | 1117 | `209fbbe0ad52d9235e37badf9cadfe4dbdc87203179c0899e738b39ade42177b` |
| `licenses/registry/rawpointer-0.2.1/LICENSE-APACHE` | `rawpointer 0.2.1` | `MIT/Apache-2.0` | 10847 | `a60eea817514531668d7e00765731449fe14d059d3249e0bc93b36de45f759f2` |
| `licenses/registry/rawpointer-0.2.1/LICENSE-MIT` | `rawpointer 0.2.1` | `MIT/Apache-2.0` | 1043 | `7576269ea71f767b99297934c0b2367532690f8c4badc695edf8e04ab6a1e545` |
| `licenses/registry/rayon-1.12.0/LICENSE-APACHE` | `rayon 1.12.0` | `MIT OR Apache-2.0` | 10847 | `a60eea817514531668d7e00765731449fe14d059d3249e0bc93b36de45f759f2` |
| `licenses/registry/rayon-1.12.0/LICENSE-MIT` | `rayon 1.12.0` | `MIT OR Apache-2.0` | 1071 | `0621878e61f0d0fda054bcbe02df75192c28bde1ecc8289cbd86aeba2dd72720` |
| `licenses/registry/rayon-core-1.13.0/LICENSE-APACHE` | `rayon-core 1.13.0` | `MIT OR Apache-2.0` | 10847 | `a60eea817514531668d7e00765731449fe14d059d3249e0bc93b36de45f759f2` |
| `licenses/registry/rayon-core-1.13.0/LICENSE-MIT` | `rayon-core 1.13.0` | `MIT OR Apache-2.0` | 1071 | `0621878e61f0d0fda054bcbe02df75192c28bde1ecc8289cbd86aeba2dd72720` |
| `licenses/registry/safe_arch-0.7.4/LICENSE-APACHE.md` | `safe_arch 0.7.4` | `Zlib OR Apache-2.0 OR MIT` | 10398 | `e3ba223bb1423f0aad8c3dfce0fe3148db48926d41e6fbc3afbbf5ff9e1c89cb` |
| `licenses/registry/safe_arch-0.7.4/LICENSE-MIT.md` | `safe_arch 0.7.4` | `Zlib OR Apache-2.0 OR MIT` | 1110 | `e57011537d230b14e790f6666dc00816f7b371ebbd7da8a12491e51086fec278` |
| `licenses/registry/safe_arch-0.7.4/LICENSE-ZLIB.md` | `safe_arch 0.7.4` | `Zlib OR Apache-2.0 OR MIT` | 851 | `c43b9a9b1387ed53d2c49263838261129a010e280e3a174a792242c3e2c98db9` |
| `licenses/registry/serde-1.0.229/LICENSE-APACHE` | `serde 1.0.229` | `MIT OR Apache-2.0` | 9723 | `62c7a1e35f56406896d7aa7ca52d0cc0d272ac022b5d2796e7d6905db8a3636a` |
| `licenses/registry/serde-1.0.229/LICENSE-MIT` | `serde 1.0.229` | `MIT OR Apache-2.0` | 1023 | `23f18e03dc49df91622fe2a76176497404e46ced8a715d9d2b67a7446571cca3` |
| `licenses/registry/serde_core-1.0.229/LICENSE-APACHE` | `serde_core 1.0.229` | `MIT OR Apache-2.0` | 9723 | `62c7a1e35f56406896d7aa7ca52d0cc0d272ac022b5d2796e7d6905db8a3636a` |
| `licenses/registry/serde_core-1.0.229/LICENSE-MIT` | `serde_core 1.0.229` | `MIT OR Apache-2.0` | 1023 | `23f18e03dc49df91622fe2a76176497404e46ced8a715d9d2b67a7446571cca3` |
| `licenses/registry/serde_derive-1.0.229/LICENSE-APACHE` | `serde_derive 1.0.229` | `MIT OR Apache-2.0` | 9723 | `62c7a1e35f56406896d7aa7ca52d0cc0d272ac022b5d2796e7d6905db8a3636a` |
| `licenses/registry/serde_derive-1.0.229/LICENSE-MIT` | `serde_derive 1.0.229` | `MIT OR Apache-2.0` | 1023 | `23f18e03dc49df91622fe2a76176497404e46ced8a715d9d2b67a7446571cca3` |
| `licenses/registry/serde_json-1.0.151/LICENSE-APACHE` | `serde_json 1.0.151` | `MIT OR Apache-2.0` | 9723 | `62c7a1e35f56406896d7aa7ca52d0cc0d272ac022b5d2796e7d6905db8a3636a` |
| `licenses/registry/serde_json-1.0.151/LICENSE-MIT` | `serde_json 1.0.151` | `MIT OR Apache-2.0` | 1023 | `23f18e03dc49df91622fe2a76176497404e46ced8a715d9d2b67a7446571cca3` |
| `licenses/registry/simba-0.9.1/LICENSE` | `simba 0.9.1` | `Apache-2.0` | 11347 | `ceacfa4d7fa67df64ab09a56fe248c50a3bfc9bc00374d69bd74ad763b14b89c` |
| `licenses/registry/simd-adler32-0.3.9/LICENSE.md` | `simd-adler32 0.3.9` | `MIT` | 1078 | `42a35170233e83e18856792e748de4c1ce4a63b2afce9a370c89ef3fe23f9f2d` |
| `licenses/registry/syn-2.0.117/LICENSE-APACHE` | `syn 2.0.117` | `MIT OR Apache-2.0` | 9723 | `62c7a1e35f56406896d7aa7ca52d0cc0d272ac022b5d2796e7d6905db8a3636a` |
| `licenses/registry/syn-2.0.117/LICENSE-MIT` | `syn 2.0.117` | `MIT OR Apache-2.0` | 1023 | `23f18e03dc49df91622fe2a76176497404e46ced8a715d9d2b67a7446571cca3` |
| `licenses/registry/syn-3.0.3/LICENSE-APACHE` | `syn 3.0.3` | `MIT OR Apache-2.0` | 9723 | `62c7a1e35f56406896d7aa7ca52d0cc0d272ac022b5d2796e7d6905db8a3636a` |
| `licenses/registry/syn-3.0.3/LICENSE-MIT` | `syn 3.0.3` | `MIT OR Apache-2.0` | 1023 | `23f18e03dc49df91622fe2a76176497404e46ced8a715d9d2b67a7446571cca3` |
| `licenses/registry/thiserror-2.0.18/LICENSE-APACHE` | `thiserror 2.0.18` | `MIT OR Apache-2.0` | 9723 | `62c7a1e35f56406896d7aa7ca52d0cc0d272ac022b5d2796e7d6905db8a3636a` |
| `licenses/registry/thiserror-2.0.18/LICENSE-MIT` | `thiserror 2.0.18` | `MIT OR Apache-2.0` | 1023 | `23f18e03dc49df91622fe2a76176497404e46ced8a715d9d2b67a7446571cca3` |
| `licenses/registry/thiserror-impl-2.0.18/LICENSE-APACHE` | `thiserror-impl 2.0.18` | `MIT OR Apache-2.0` | 9723 | `62c7a1e35f56406896d7aa7ca52d0cc0d272ac022b5d2796e7d6905db8a3636a` |
| `licenses/registry/thiserror-impl-2.0.18/LICENSE-MIT` | `thiserror-impl 2.0.18` | `MIT OR Apache-2.0` | 1023 | `23f18e03dc49df91622fe2a76176497404e46ced8a715d9d2b67a7446571cca3` |
| `licenses/registry/typenum-1.20.0/LICENSE` | `typenum 1.20.0` | `MIT OR Apache-2.0` | 17 | `db11fec9946737df39ca3898d9cd8c10ec6f6c3a884a6802b0ad0b81b4e8f23a` |
| `licenses/registry/typenum-1.20.0/LICENSE-APACHE` | `typenum 1.20.0` | `MIT OR Apache-2.0` | 10835 | `516b24e051bf5630880ebbd55c40a25ce9552ebaf8970a53e8976eb70e522406` |
| `licenses/registry/typenum-1.20.0/LICENSE-MIT` | `typenum 1.20.0` | `MIT OR Apache-2.0` | 1083 | `a825bd853ab71619a4923d7b4311221427848070ff44d990da39b0b274c1683f` |
| `licenses/registry/unicode-ident-1.0.24/LICENSE-APACHE` | `unicode-ident 1.0.24` | `(MIT OR Apache-2.0) AND Unicode-3.0` | 9723 | `62c7a1e35f56406896d7aa7ca52d0cc0d272ac022b5d2796e7d6905db8a3636a` |
| `licenses/registry/unicode-ident-1.0.24/LICENSE-MIT` | `unicode-ident 1.0.24` | `(MIT OR Apache-2.0) AND Unicode-3.0` | 1023 | `23f18e03dc49df91622fe2a76176497404e46ced8a715d9d2b67a7446571cca3` |
| `licenses/registry/unicode-ident-1.0.24/LICENSE-UNICODE` | `unicode-ident 1.0.24` | `(MIT OR Apache-2.0) AND Unicode-3.0` | 1995 | `f7db81051789b729fea528a63ec4c938fdcb93d9d61d97dc8cc2e9df6d47f2a1` |
| `licenses/registry/wide-0.7.33/LICENSE-ZLIB.md` | `wide 0.7.33` | `Zlib OR Apache-2.0 OR MIT` | 851 | `c43b9a9b1387ed53d2c49263838261129a010e280e3a174a792242c3e2c98db9` |
| `licenses/registry/zerocopy-0.8.48/LICENSE-APACHE` | `zerocopy 0.8.48` | `BSD-2-Clause OR Apache-2.0 OR MIT` | 11350 | `9d185ac6703c4b0453974c0d85e9eee43e6941009296bb1f5eb0b54e2329e9f3` |
| `licenses/registry/zerocopy-0.8.48/LICENSE-BSD` | `zerocopy 0.8.48` | `BSD-2-Clause OR Apache-2.0 OR MIT` | 1275 | `83c1763356e822adde0a2cae748d938a73fdc263849ccff6b27776dff213bd32` |
| `licenses/registry/zerocopy-0.8.48/LICENSE-MIT` | `zerocopy 0.8.48` | `BSD-2-Clause OR Apache-2.0 OR MIT` | 1060 | `1a2f5c12ddc934d58956aa5dbdd3255fe55fd957633ab7d0d39e4f0daa73f7df` |
| `licenses/registry/zmij-1.0.23/LICENSE-MIT` | `zmij 1.0.23` | `MIT` | 1023 | `23f18e03dc49df91622fe2a76176497404e46ced8a715d9d2b67a7446571cca3` |
| `licenses/workspace/LICENSE-APACHE` | `workspace-root 0.1.0` | `MIT OR Apache-2.0 (workspace/local)` | 9231 | `590ad8e27b461fe02e71044906ba9405ec4e40a3d7d36bf9caf9eee33bbf6cab` |
| `licenses/workspace/LICENSE-MIT` | `workspace-root 0.1.0` | `MIT OR Apache-2.0 (workspace/local)` | 1100 | `6b47d20e4ea07503d832383a5fd53180180405c32ed52ff6f5d490e36a481a14` |

## Provenance caveats

- Registry license texts are copied from the local Cargo source cache and are hash-verified against the selected graph report; they are not silently promoted to authoritative v2 inventory evidence.
- Cargo.lock identity is required for every selected package. Optional `.cargo-checksum.json` and `.crate` files are verified when present; `.crate` license members are matched read-only by safe `<name-version>/<path>` tar member and bytes/hash, while unavailable archive evidence remains explicitly recorded.
- The two checked-in Basalt NOTICE files are included for the pinned BSD-3-Clause attribution and commit `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.
- Excluded `ort`, `ort-sys`, `shlex`, `socks`, winapi rows, vcpkg, and non-Cargo linker/SDK/system-runtime inputs are not copied by this selected-profile payload.
- No legal clearance, final release promotion, or native dependency closure is implied.
