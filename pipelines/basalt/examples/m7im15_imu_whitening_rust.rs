//! Re-run the production f32 IMU whitening expression on native raw buffers.
//!
//! This is an audit-only companion to
//! `work/m7im15_imu_whitening_harness_20260828.cpp`.  It deliberately does
//! not reconstruct a state or preintegration; the raw matrices are the exact
//! buffers captured at `ImuBlock::linearizeImu`, and the public diagnostic seam
//! delegates to the production fixed-size whitening helpers.

use nalgebra::{SMatrix, SVector};
use serde_json::json;
use std::{env, fs, path::Path};
use visloc_basalt::imu::diagnostic_whiten_imu_raw_f32;

type Vec9 = SVector<f32, 9>;
type Matrix9F32 = SMatrix<f32, 9, 9>;
type Matrix9x30F32 = SMatrix<f32, 9, 30>;

fn read_values(path: &Path, count: usize) -> Vec<f32> {
    let bytes = fs::read(path).unwrap_or_else(|error| panic!("{}: {error}", path.display()));
    assert_eq!(bytes.len(), count * 4, "{} has wrong size", path.display());
    bytes
        .chunks_exact(4)
        .map(|chunk| f32::from_bits(u32::from_le_bytes(chunk.try_into().unwrap())))
        .collect()
}

fn read_vec9(path: &Path) -> Vec9 {
    Vec9::from_column_slice(&read_values(path, 9))
}

fn read_matrix<const R: usize, const C: usize>(path: &Path) -> SMatrix<f32, R, C> {
    SMatrix::from_column_slice(&read_values(path, R * C))
}

fn bits(value: f32) -> String {
    format!("{:08x}", value.to_bits())
}

fn vec_bits(values: &[f32]) -> Vec<String> {
    values.iter().copied().map(bits).collect()
}

fn matrix_bits<const R: usize, const C: usize>(matrix: &SMatrix<f32, R, C>) -> Vec<String> {
    vec_bits(matrix.as_slice())
}

fn read_whitener(base: &Path, sequence: usize) -> (Matrix9F32, usize) {
    // The first raw probe runs before Eigen's lazy LDLT cache is materialized.
    // All LM iterations share this immutable measurement, so use the valid
    // record-3 cache for record 0 and reject an all-zero input rather than
    // producing a misleading all-zero whitening result.
    let whitener_sequence = if sequence == 0 { 3 } else { sequence };
    let path = base.join(format!("{whitener_sequence:03}.raw.sqrt_cov_inv.bin"));
    let whitener = read_matrix::<9, 9>(&path);
    assert!(whitener.as_slice().iter().any(|value| *value != 0.0));
    (whitener, whitener_sequence)
}

fn record(base: &Path, sequence: usize) -> serde_json::Value {
    let stem = base.join(format!("{sequence:03}.raw."));
    let residual = read_vec9(&stem.with_extension("res.bin"));
    let start = read_matrix::<9, 9>(&stem.with_extension("d_res_d_start.bin"));
    let end = read_matrix::<9, 9>(&stem.with_extension("d_res_d_end.bin"));
    let gyro_bias = read_matrix::<9, 3>(&stem.with_extension("d_res_d_bg.bin"));
    let accel_bias = read_matrix::<9, 3>(&stem.with_extension("d_res_d_ba.bin"));
    let (whitener, whitener_sequence) = read_whitener(base, sequence);
    let (whitened_residual, whitened_jacobian): (Vec9, Matrix9x30F32) =
        diagnostic_whiten_imu_raw_f32(&whitener, &residual, &start, &gyro_bias, &accel_bias, &end);
    json!({
        "sequence": sequence,
        "whitener_sequence": whitener_sequence,
        "sqrt_information_bits": matrix_bits(&whitener),
        "whitened_residual_bits": vec_bits(whitened_residual.as_slice()),
        "whitened_jacobian_bits": matrix_bits(&whitened_jacobian),
    })
}

fn main() {
    let args: Vec<_> = env::args_os().collect();
    assert_eq!(
        args.len(),
        3,
        "usage: m7im15_imu_whitening_rust RAW_DIR OUTPUT_JSON"
    );
    let result = json!({
        "schema": "basalt.m7im15.imu_whitening_rust_production.v1",
        "records": [record(Path::new(&args[1]), 0), record(Path::new(&args[1]), 3)],
    });
    fs::write(&args[2], serde_json::to_vec_pretty(&result).unwrap()).unwrap();
}
