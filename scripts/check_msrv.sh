#!/usr/bin/env sh
set -eu

if ! command -v cargo >/dev/null 2>&1 && [ -d "$HOME/.cargo/bin" ]; then
    PATH="$HOME/.cargo/bin:$PATH"
    export PATH
fi

# The 1.83 MSRV guarantee covers the core library and the `image-io` demo path.
#
# It deliberately does NOT cover the opt-in `onnx-inference` feature: that pulls
# the `ort` ONNX runtime, whose build dependency `ureq`/`ureq-proto` requires
# edition2024 (Rust >= 1.85) and cannot even be parsed by Cargo 1.83. The ONNX
# runtime tracks its own toolchain floor; keeping it out of this check confines
# that requirement to the feature boundary instead of leaking it into the core
# MSRV. Build the ONNX path on a current stable toolchain instead.
#
# The same applies to the opt-in `gpu` feature of `visloc-gsplat-render`: wgpu
# 30 depends on naga, which needs `indexmap >= 2.11.4` (edition2024, Rust >=
# 1.85). The feature is off by default, so this build never pulls it; build the
# GPU renderer with `--features gpu` on a current stable toolchain.
# `visloc-gsplat-train/gpu`, `visloc-sift-gpu/gpu` and `visloc-ba-gpu/gpu` are
# the same opt-in wgpu boundary.
cargo +1.83.0 check --workspace --all-targets --features image-io
