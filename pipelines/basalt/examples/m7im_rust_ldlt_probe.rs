//! Bit-level Rust LDLT/whitener check against the pinned M7IM oracle.
//!
//! This is a diagnostic harness only.  It reads the generated oracle JSON,
//! narrows the ten packet covariance matrices to f32, and calls the same
//! public whitener used by the production IMU factor path.

use std::{env, fs, path::Path};

use nalgebra::SMatrix;
use serde_json::Value;
use visloc_basalt::imu::sqrt_information_f32;

type Matrix9F32 = SMatrix<f32, 9, 9>;

fn matrix_bits(value: &Value) -> Vec<u32> {
    value["bits_row_major"]
        .as_array()
        .expect("matrix bits")
        .iter()
        .map(|item| u32::from_str_radix(item.as_str().expect("hex bits"), 16).unwrap())
        .collect()
}

fn matrix_from_bits(value: &Value) -> Matrix9F32 {
    let bits = matrix_bits(value);
    Matrix9F32::from_fn(|row, column| f32::from_bits(bits[row * 9 + column]))
}

fn compare(expected: &[u32], actual: &Matrix9F32) -> (usize, Option<(usize, u32, u32)>) {
    let mut mismatches = 0;
    let mut first = None;
    for row in 0..9 {
        for column in 0..9 {
            let index = row * 9 + column;
            let got = actual[(row, column)].to_bits();
            if got != expected[index] {
                mismatches += 1;
                if first.is_none() {
                    first = Some((index, expected[index], got));
                }
            }
        }
    }
    (mismatches, first)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = env::args()
        .nth(1)
        .unwrap_or_else(|| "target/m7im_cov_ldlt_oracle_20260824.json".to_owned());
    if !Path::new(&path).exists() {
        return Err(format!("oracle does not exist: {path}").into());
    }
    let oracle: Value = serde_json::from_str(&fs::read_to_string(&path)?)?;
    let packets = oracle["first_link_packets"]
        .as_array()
        .ok_or("first_link_packets is not an array")?;
    let mut total = 0;
    for packet in packets.iter().take(10) {
        let record = &packet["record"];
        let covariance = matrix_from_bits(&record["covariance_after"]);
        let actual = sqrt_information_f32(&covariance).map_err(|error| format!("{error:?}"))?;
        let expected = matrix_bits(&record["ldlt"]["sqrt_reconstructed"]);
        let (mismatches, first) = compare(&expected, &actual);
        total += mismatches;
        println!(
            "packet={} sqrt_mismatches={}/81 first={}",
            packet["packet"],
            mismatches,
            first.map_or_else(
                || "none".to_owned(),
                |(index, expected, actual)| {
                    format!("lane={index} expected={expected:08x} actual={actual:08x}")
                },
            )
        );
    }
    let frame4 = &oracle["frame4_factor"];
    let frame4_covariance = matrix_from_bits(&frame4["covariance"]);
    let frame4_actual =
        sqrt_information_f32(&frame4_covariance).map_err(|error| format!("{error:?}"))?;
    let (frame4_mismatches, frame4_first) =
        compare(&matrix_bits(&frame4["sqrt_information"]), &frame4_actual);
    println!(
        "frame4 sqrt_mismatches={}/81 first={}",
        frame4_mismatches,
        frame4_first.map_or_else(
            || "none".to_owned(),
            |(index, expected, actual)| {
                format!("lane={index} expected={expected:08x} actual={actual:08x}")
            },
        )
    );
    println!(
        "summary packet_sqrt_mismatches={}/810 frame4_sqrt_mismatches={}/81",
        total, frame4_mismatches
    );
    if total != 0 || frame4_mismatches != 0 {
        return Err("M7IM LDLT/whitener mismatch".into());
    }
    Ok(())
}
