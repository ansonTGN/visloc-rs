//! Disposable Rust/nalgebra companion to `examples/m8c_exact_bits_probe.cpp`.
//!
//! Run explicitly with:
//!
//! ```text
//! cargo test -p visloc-basalt --test m8c_exact_bits_probe -- --ignored --nocapture
//! ```
//!
//! The test is ignored so it cannot become part of the ordinary mapper gate.
//! It reads the same raw/oracle JSON as the C++ probe and prints f64 bit
//! patterns for the seed-7 cam0->cam1 model and correspondence zero.  An
//! explicit run must set `M8C_FEATURE_RAW_JSON` and
//! `M8C_FEATURE_ORACLE_JSON`; neither path has a target-directory fallback.

use std::{fs, path::PathBuf};

use nalgebra::{Matrix2, Matrix3, SMatrix, Vector2, Vector3, Vector4};
use serde_json::Value;
use visloc_basalt::mapper::{
    match_temporal_ransac_seeded, MapperImageFeatures, OfflineMapperConfig,
};

const LEFT_FRAME: u64 = 1_403_636_580_113_555_456;
const RIGHT_FRAME: u64 = 1_403_636_579_763_555_584;

const fn bits(value: f64) -> u64 {
    value.to_bits()
}

fn scalar(name: &str, value: f64) {
    println!("{name}=0x{:016x}", bits(value));
}

fn dot3_fma_left(a: &Vector3<f64>, b: &Vector3<f64>) -> f64 {
    a[0].mul_add(b[0], a[1].mul_add(b[1], a[2] * b[2]))
}

fn dot3_fma_right(a: &Vector3<f64>, b: &Vector3<f64>) -> f64 {
    a[2].mul_add(b[2], a[1].mul_add(b[1], a[0] * b[0]))
}

fn dot3_pair_fma(a: &Vector3<f64>, b: &Vector3<f64>) -> f64 {
    a[0].mul_add(b[0], a[2].mul_add(b[2], a[1] * b[1]))
}

fn dot3_fma_one(a: &Vector3<f64>, b: &Vector3<f64>, fused: usize) -> f64 {
    let p0 = a[0] * b[0];
    let p1 = a[1] * b[1];
    let p2 = a[2] * b[2];
    match fused {
        0 => a[0].mul_add(b[0], p1 + p2),
        1 => a[1].mul_add(b[1], p0 + p2),
        _ => a[2].mul_add(b[2], p0 + p1),
    }
}

fn vec3(name: &str, value: &Vector3<f64>) {
    for index in 0..3 {
        scalar(&format!("{name}[{index}]"), value[index]);
    }
}

fn eigen_dot3(left: &Vector3<f64>, right: &Vector3<f64>) -> f64 {
    let first_two = left[0] * right[0] + left[1] * right[1];
    left[2].mul_add(right[2], first_two)
}

fn eigen_norm3(value: &Vector3<f64>) -> f64 {
    let first_two = value[0] * value[0] + value[1] * value[1];
    value[2].mul_add(value[2], first_two).sqrt()
}

fn eigen_matvec3(matrix: Matrix3<f64>, value: Vector3<f64>) -> Vector3<f64> {
    Vector3::new(
        matrix[(0, 2)].mul_add(
            value[2],
            matrix[(0, 1)].mul_add(value[1], matrix[(0, 0)] * value[0]),
        ),
        matrix[(1, 2)].mul_add(
            value[2],
            matrix[(1, 1)].mul_add(value[1], matrix[(1, 0)] * value[0]),
        ),
        matrix[(2, 2)].mul_add(
            value[2],
            matrix[(2, 1)].mul_add(value[1], matrix[(2, 0)] * value[0]),
        ),
    )
}

fn eigen_mat34_vec4(matrix: SMatrix<f64, 3, 4>, value: Vector4<f64>) -> Vector3<f64> {
    Vector3::new(
        matrix[(0, 3)].mul_add(
            value[3],
            matrix[(0, 2)].mul_add(
                value[2],
                matrix[(0, 1)].mul_add(value[1], matrix[(0, 0)] * value[0]),
            ),
        ),
        matrix[(1, 3)].mul_add(
            value[3],
            matrix[(1, 2)].mul_add(
                value[2],
                matrix[(1, 1)].mul_add(value[1], matrix[(1, 0)] * value[0]),
            ),
        ),
        matrix[(2, 3)].mul_add(value[3], matrix[(2, 2)] * value[2])
            + matrix[(2, 1)].mul_add(value[1], matrix[(2, 0)] * value[0]),
    )
}

fn mat3(name: &str, value: &Matrix3<f64>) {
    for row in 0..3 {
        for column in 0..3 {
            scalar(&format!("{name}[{row}][{column}]"), value[(row, column)]);
        }
    }
}

fn feature<'a>(raw: &'a Value, frame: u64, cam: u64) -> &'a Value {
    raw["features"]
        .as_array()
        .expect("features array")
        .iter()
        .find(|item| {
            item["time_cam_id"]["frame_id"].as_u64() == Some(frame)
                && item["time_cam_id"]["cam_id"].as_u64() == Some(cam)
        })
        .expect("feature pair")
}

fn ray(feature: &Value, index: usize) -> Vector3<f64> {
    let value = feature["corners_3d"][index].as_array().expect("ray array");
    Vector3::new(
        value[0].as_f64().expect("ray x"),
        value[1].as_f64().expect("ray y"),
        value[2].as_f64().expect("ray z"),
    )
}

fn inverse3(matrix: Matrix3<f64>) -> Matrix3<f64> {
    let cofactor = |row: usize, column: usize| {
        let i1 = (row + 1) % 3;
        let i2 = (row + 2) % 3;
        let j1 = (column + 1) % 3;
        let j2 = (column + 2) % 3;
        matrix[(i1, j1)] * matrix[(i2, j2)] - matrix[(i1, j2)] * matrix[(i2, j1)]
    };
    let cofactors_col0 = Vector3::new(cofactor(0, 0), cofactor(1, 0), cofactor(2, 0));
    let determinant = cofactors_col0[0] * matrix[(0, 0)]
        + cofactors_col0[1] * matrix[(1, 0)]
        + cofactors_col0[2] * matrix[(2, 0)];
    let inverse_determinant = 1.0 / determinant;
    Matrix3::new(
        cofactors_col0[0] * inverse_determinant,
        cofactor(1, 0) * inverse_determinant,
        cofactor(2, 0) * inverse_determinant,
        cofactor(0, 1) * inverse_determinant,
        cofactor(1, 1) * inverse_determinant,
        cofactor(2, 1) * inverse_determinant,
        cofactor(0, 2) * inverse_determinant,
        cofactor(1, 2) * inverse_determinant,
        cofactor(2, 2) * inverse_determinant,
    )
}

fn rot2cayley(rotation: Matrix3<f64>) -> Vector3<f64> {
    let identity = Matrix3::identity();
    let cayley = (rotation - identity) * inverse3(rotation + identity);
    Vector3::new(-cayley[(1, 2)], cayley[(0, 2)], -cayley[(0, 1)])
}

fn cayley2rot(cayley: Vector3<f64>) -> Matrix3<f64> {
    let c0sq = cayley[0].powi(2);
    let c1sq = cayley[1].powi(2);
    let c2sq = cayley[2].powi(2);
    let scale = 1.0 + c0sq + c1sq + c2sq;
    let inverse_scale = 1.0 / scale;
    Matrix3::new(
        inverse_scale * (1.0 + c0sq - c1sq - c2sq),
        inverse_scale * (2.0 * (cayley[0] * cayley[1] - cayley[2])),
        inverse_scale * (2.0 * (cayley[0] * cayley[2] + cayley[1])),
        inverse_scale * (2.0 * (cayley[0] * cayley[1] + cayley[2])),
        inverse_scale * (1.0 - c0sq + c1sq - c2sq),
        inverse_scale * (2.0 * (cayley[1] * cayley[2] - cayley[0])),
        inverse_scale * (2.0 * (cayley[0] * cayley[2] - cayley[1])),
        inverse_scale * (2.0 * (cayley[1] * cayley[2] + cayley[0])),
        inverse_scale * (1.0 - c0sq - c1sq + c2sq),
    )
}

fn model_rotation(entry: &Value) -> Matrix3<f64> {
    Matrix3::from_fn(|row, column| {
        entry["ransac_model_rotation"][row][column]
            .as_f64()
            .expect("R coefficient")
    })
}

fn model_translation(entry: &Value) -> Vector3<f64> {
    Vector3::new(
        entry["ransac_model_translation"][0].as_f64().expect("t0"),
        entry["ransac_model_translation"][1].as_f64().expect("t1"),
        entry["ransac_model_translation"][2].as_f64().expect("t2"),
    )
}

fn production_feature(value: &Value) -> MapperImageFeatures {
    let corners = value["corners_xy"]
        .as_array()
        .expect("corners")
        .iter()
        .map(|item| nalgebra::Point2::new(item[0].as_f64().unwrap(), item[1].as_f64().unwrap()))
        .collect();
    let corner_angles = value["corner_angles"]
        .as_array()
        .expect("angles")
        .iter()
        .map(|item| item.as_f64().unwrap())
        .collect();
    let descriptors = value["descriptor_bytes"]
        .as_array()
        .expect("descriptors")
        .iter()
        .map(|item| {
            let mut result = [0_u8; 32];
            for (index, byte) in item.as_array().unwrap().iter().enumerate() {
                result[index] = byte.as_u64().unwrap() as u8;
            }
            result
        })
        .collect();
    let rays = value["corners_3d"]
        .as_array()
        .expect("rays")
        .iter()
        .map(|item| {
            [
                item[0].as_f64().unwrap(),
                item[1].as_f64().unwrap(),
                item[2].as_f64().unwrap(),
                item[3].as_f64().unwrap_or(0.0),
            ]
        })
        .collect();
    let hashes = value["hashes"]
        .as_array()
        .expect("hashes")
        .iter()
        .map(|item| item.as_u64().unwrap() as u32)
        .collect();
    MapperImageFeatures {
        corners,
        corner_angles,
        descriptors,
        rays,
        hashes,
        bow_vector: Vec::new(),
    }
}

fn emit_residual(
    prefix: &str,
    left: Vector3<f64>,
    right: Vector3<f64>,
    translation: Vector3<f64>,
    rotation: Matrix3<f64>,
) {
    let right_unrotated = eigen_matvec3(rotation, right);
    let a00 = eigen_dot3(&left, &left);
    let a10 = eigen_dot3(&left, &right_unrotated);
    let a01 = -a10;
    let a11 = -eigen_dot3(&right_unrotated, &right_unrotated);
    let b0 = eigen_dot3(&translation, &left);
    let b1 = eigen_dot3(&translation, &right_unrotated);
    scalar(
        &format!("{prefix}.b0_fma_left"),
        dot3_fma_left(&translation, &left),
    );
    scalar(
        &format!("{prefix}.b0_fma_right"),
        dot3_fma_right(&translation, &left),
    );
    scalar(
        &format!("{prefix}.b0_pair_fma"),
        dot3_pair_fma(&translation, &left),
    );
    for fused in 0..3 {
        scalar(
            &format!("{prefix}.b0_fma_one{fused}"),
            dot3_fma_one(&translation, &left, fused),
        );
    }
    scalar(
        &format!("{prefix}.b1_fma_left"),
        dot3_fma_left(&translation, &right_unrotated),
    );
    scalar(
        &format!("{prefix}.b1_fma_right"),
        dot3_fma_right(&translation, &right_unrotated),
    );
    scalar(
        &format!("{prefix}.b1_pair_fma"),
        dot3_pair_fma(&translation, &right_unrotated),
    );
    for fused in 0..3 {
        scalar(
            &format!("{prefix}.b1_fma_one{fused}"),
            dot3_fma_one(&translation, &right_unrotated, fused),
        );
    }
    let determinant = a00 * a11 - a10 * a01;
    let inverse_determinant = 1.0 / determinant;
    let inverse00 = a11 * inverse_determinant;
    let inverse01 = -a01 * inverse_determinant;
    let inverse10 = -a10 * inverse_determinant;
    let inverse11 = a00 * inverse_determinant;
    let lambda0 = inverse01.mul_add(b1, inverse00 * b0);
    let lambda1 = inverse11.mul_add(b1, inverse10 * b0);
    let matrix_a = Matrix2::new(a00, a01, a10, a11);
    let matrix_inverse = matrix_a.try_inverse().expect("A inverse");
    let lambda_matrix = matrix_inverse * Vector2::new(b0, b1);
    let xm = lambda0 * left;
    let xn = Vector3::new(
        right_unrotated[0].mul_add(lambda1, translation[0]),
        right_unrotated[1].mul_add(lambda1, translation[1]),
        right_unrotated[2].mul_add(lambda1, translation[2]),
    );
    let point_manual = (xm + xn) / 2.0;
    if prefix == "x_row12" {
        let recovered_lambda0 = f64::from_bits(0x40270f8201670329);
        let recovered_lambda1 = f64::from_bits(0x40274d9c5a75ac89);
        let recovered_xm = recovered_lambda0 * left;
        let recovered_xn = Vector3::new(
            translation[0] + recovered_lambda1 * right_unrotated[0],
            translation[1] + recovered_lambda1 * right_unrotated[1],
            translation[2] + recovered_lambda1 * right_unrotated[2],
        );
        let recovered_point = (recovered_xm + recovered_xn) / 2.0;
        println!(
            "{prefix}.recovered point=0x{:016x},0x{:016x},0x{:016x}",
            recovered_point[0].to_bits(),
            recovered_point[1].to_bits(),
            recovered_point[2].to_bits(),
        );
        let determinant_candidates = [
            ("manual", a00 * a11 - a10 * a01),
            ("fma_left", a00.mul_add(a11, -(a10 * a01))),
            ("fma_right", (-a10).mul_add(a01, a00 * a11)),
            ("square_left", a10.mul_add(a10, a00 * a11)),
            ("square_right", a00.mul_add(a11, a10 * a10)),
            ("square_pair", a10.mul_add(a10, a00.mul_add(a11, 0.0))),
        ];
        for (label, candidate) in determinant_candidates {
            let inverse_determinant = 1.0 / candidate;
            let inv00 = a11 * inverse_determinant;
            let inv01 = -a01 * inverse_determinant;
            let inv10 = -a10 * inverse_determinant;
            let inv11 = a00 * inverse_determinant;
            let candidate_lambda0 = inv01.mul_add(b1, inv00 * b0);
            let candidate_lambda1 = inv11.mul_add(b1, inv10 * b0);
            let candidate_xm = candidate_lambda0 * left;
            let candidate_xn = Vector3::new(
                right_unrotated[0].mul_add(candidate_lambda1, translation[0]),
                right_unrotated[1].mul_add(candidate_lambda1, translation[1]),
                right_unrotated[2].mul_add(candidate_lambda1, translation[2]),
            );
            let candidate_point = (candidate_xm + candidate_xn) / 2.0;
            println!(
                "{prefix}.{label}_det=0x{:016x} invdet=0x{:016x} lam=0x{:016x},0x{:016x} point=0x{:016x},0x{:016x},0x{:016x}",
                candidate.to_bits(),
                inverse_determinant.to_bits(),
                candidate_lambda0.to_bits(),
                candidate_lambda1.to_bits(),
                candidate_point[0].to_bits(),
                candidate_point[1].to_bits(),
                candidate_point[2].to_bits(),
            );
            if label == "manual" || label == "fma_right" || label == "square_left" {
                let separate_xn = Vector3::new(
                    translation[0] + candidate_lambda1 * right_unrotated[0],
                    translation[1] + candidate_lambda1 * right_unrotated[1],
                    translation[2] + candidate_lambda1 * right_unrotated[2],
                );
                let separate_point = (candidate_xm + separate_xn) / 2.0;
                println!(
                    "{prefix}.{label}_separate point=0x{:016x},0x{:016x},0x{:016x}",
                    separate_point[0].to_bits(),
                    separate_point[1].to_bits(),
                    separate_point[2].to_bits(),
                );
            }
            if label == "manual" {
                let sum_lambda0 = inv00 * b0 + inv01 * b1;
                let sum_lambda1 = inv10 * b0 + inv11 * b1;
                let sum_xm = sum_lambda0 * left;
                let sum_xn = Vector3::new(
                    right_unrotated[0].mul_add(sum_lambda1, translation[0]),
                    right_unrotated[1].mul_add(sum_lambda1, translation[1]),
                    right_unrotated[2].mul_add(sum_lambda1, translation[2]),
                );
                let sum_point = (sum_xm + sum_xn) / 2.0;
                println!(
                    "{prefix}.manual_sum lam=0x{:016x},0x{:016x} point=0x{:016x},0x{:016x},0x{:016x}",
                    sum_lambda0.to_bits(),
                    sum_lambda1.to_bits(),
                    sum_point[0].to_bits(),
                    sum_point[1].to_bits(),
                    sum_point[2].to_bits(),
                );
            }
        }
    }
    let inverse_rotation = rotation.transpose();
    let inverse_translation = -eigen_matvec3(inverse_rotation, translation);
    let second_split = eigen_matvec3(inverse_rotation, point_manual) + inverse_translation;
    let inverse_solution: SMatrix<f64, 3, 4> = SMatrix::from_columns(&[
        inverse_rotation.column(0).into_owned(),
        inverse_rotation.column(1).into_owned(),
        inverse_rotation.column(2).into_owned(),
        inverse_translation,
    ]);
    let point_hom = Vector4::new(point_manual[0], point_manual[1], point_manual[2], 1.0);
    let second_matrix = eigen_mat34_vec4(inverse_solution, point_hom);
    let first_norm = eigen_norm3(&point_manual);
    let second_norm_split = eigen_norm3(&second_split);
    let second_norm_matrix = eigen_norm3(&second_matrix);
    let first_unit = point_manual / first_norm;
    let second_unit_split = second_split / second_norm_split;
    let second_unit_matrix = second_matrix / second_norm_matrix;
    let left_dot_split = eigen_dot3(&left, &first_unit);
    let right_dot_split = eigen_dot3(&right, &second_unit_split);
    let left_dot_matrix = eigen_dot3(&left, &first_unit);
    let right_dot_matrix = eigen_dot3(&right, &second_unit_matrix);
    let residual_split = (1.0 - left_dot_split) + (1.0 - right_dot_split);
    let residual_matrix = (1.0 - left_dot_matrix) + (1.0 - right_dot_matrix);

    vec3(&format!("{prefix}.left_bearing"), &left);
    vec3(&format!("{prefix}.right_bearing"), &right);
    vec3(&format!("{prefix}.right_unrotated"), &right_unrotated);
    mat3(&format!("{prefix}.R"), &rotation);
    vec3(&format!("{prefix}.t"), &translation);
    scalar(&format!("{prefix}.A00"), a00);
    scalar(&format!("{prefix}.A10"), a10);
    scalar(&format!("{prefix}.A01"), a01);
    scalar(&format!("{prefix}.A11"), a11);
    scalar(&format!("{prefix}.b0"), b0);
    scalar(&format!("{prefix}.b1"), b1);
    scalar(&format!("{prefix}.det_manual"), determinant);
    scalar(
        &format!("{prefix}.det_fma_right"),
        (-a10).mul_add(a01, a00 * a11),
    );
    scalar(
        &format!("{prefix}.det_fma_left"),
        a00.mul_add(a11, -(a10 * a01)),
    );
    scalar(&format!("{prefix}.det_matrix"), matrix_a.determinant());
    scalar(&format!("{prefix}.invdet"), inverse_determinant);
    scalar(&format!("{prefix}.inv00"), inverse00);
    scalar(&format!("{prefix}.inv01"), inverse01);
    scalar(&format!("{prefix}.inv10"), inverse10);
    scalar(&format!("{prefix}.inv11"), inverse11);
    scalar(&format!("{prefix}.inv_matrix00"), matrix_inverse[(0, 0)]);
    scalar(&format!("{prefix}.inv_matrix01"), matrix_inverse[(0, 1)]);
    scalar(&format!("{prefix}.inv_matrix10"), matrix_inverse[(1, 0)]);
    scalar(&format!("{prefix}.inv_matrix11"), matrix_inverse[(1, 1)]);
    scalar(&format!("{prefix}.lambda0"), lambda0);
    scalar(&format!("{prefix}.lambda1"), lambda1);
    scalar(&format!("{prefix}.lambda_matrix0"), lambda_matrix[0]);
    scalar(&format!("{prefix}.lambda_matrix1"), lambda_matrix[1]);
    vec3(&format!("{prefix}.xm"), &xm);
    vec3(&format!("{prefix}.xn"), &xn);
    vec3(&format!("{prefix}.point_manual"), &point_manual);
    mat3(&format!("{prefix}.inverse_R"), &inverse_rotation);
    vec3(&format!("{prefix}.inverse_t"), &inverse_translation);
    vec3(&format!("{prefix}.second_matrix"), &second_matrix);
    vec3(&format!("{prefix}.second_split"), &second_split);
    scalar(&format!("{prefix}.first_norm"), first_norm);
    scalar(&format!("{prefix}.second_norm_matrix"), second_norm_matrix);
    scalar(&format!("{prefix}.second_norm_split"), second_norm_split);
    vec3(&format!("{prefix}.first_unit"), &first_unit);
    vec3(&format!("{prefix}.second_unit_matrix"), &second_unit_matrix);
    vec3(&format!("{prefix}.second_unit_split"), &second_unit_split);
    scalar(&format!("{prefix}.left_dot_matrix"), left_dot_matrix);
    scalar(&format!("{prefix}.right_dot_matrix"), right_dot_matrix);
    scalar(&format!("{prefix}.left_dot_split"), left_dot_split);
    scalar(&format!("{prefix}.right_dot_split"), right_dot_split);
    scalar(&format!("{prefix}.residual_matrix"), residual_matrix);
    scalar(&format!("{prefix}.residual_split"), residual_split);
}

#[test]
#[ignore = "requires explicit M8C_FEATURE_RAW_JSON and M8C_FEATURE_ORACLE_JSON paths"]
fn seed7_cam0_to_cam1_exact_bits() {
    let raw_path = std::env::var_os("M8C_FEATURE_RAW_JSON")
        .map(PathBuf::from)
        .expect("set M8C_FEATURE_RAW_JSON to the external raw feature JSON");
    let oracle_path = std::env::var_os("M8C_FEATURE_ORACLE_JSON")
        .map(PathBuf::from)
        .expect("set M8C_FEATURE_ORACLE_JSON to the pinned oracle JSON");
    let raw: Value = serde_json::from_str(&fs::read_to_string(raw_path).expect("raw JSON"))
        .expect("raw JSON parse");
    let oracle: Value =
        serde_json::from_str(&fs::read_to_string(oracle_path).expect("oracle JSON"))
            .expect("oracle JSON parse");
    let left_feature = feature(&raw, LEFT_FRAME, 0);
    let right_feature = feature(&raw, RIGHT_FRAME, 1);
    let entry = oracle["bow_query"]["seeded_temporal_oracle"]
        .as_array()
        .expect("seeded oracle")
        .iter()
        .find(|entry| {
            entry["seed"].as_u64() == Some(7)
                && entry["left"]["frame_id"].as_u64() == Some(LEFT_FRAME)
                && entry["left"]["cam_id"].as_u64() == Some(0)
                && entry["right"]["frame_id"].as_u64() == Some(RIGHT_FRAME)
                && entry["right"]["cam_id"].as_u64() == Some(1)
        })
        .expect("seed7 cam0->cam1 entry");
    let ids = entry["ransac_inlier_ids"].as_array().expect("inlier IDs");
    let left = ray(left_feature, ids[0][0].as_u64().expect("left ID") as usize);
    let right = ray(
        right_feature,
        ids[0][1].as_u64().expect("right ID") as usize,
    );
    let rotation = model_rotation(entry);
    let translation = model_translation(entry);
    let cayley = rot2cayley(rotation);
    let rotation_from_cayley = cayley2rot(cayley);
    println!("pair.seed=7");
    vec3("x", &translation);
    for index in 0..3 {
        scalar(&format!("x[{}]", index + 3), cayley[index]);
    }
    mat3("R_initial", &rotation);
    mat3("R_from_cayley", &rotation_from_cayley);
    vec3("t_initial", &translation);
    let mut x = [0.0_f64; 6];
    x[0..3].copy_from_slice(translation.as_slice());
    x[3..6].copy_from_slice(cayley.as_slice());
    for (index, value) in x.into_iter().enumerate() {
        let mut h = f64::EPSILON.sqrt() * value.abs();
        if h == 0.0 {
            h = f64::EPSILON.sqrt();
        }
        scalar(&format!("h[{index}]"), h);
        scalar(&format!("xh[{index}]"), value + h);
    }
    emit_residual("x", left, right, translation, rotation_from_cayley);
    if std::env::var_os("M8C_EXACT_BITS_ALL_ROWS").is_some() {
        let count = ids.len();
        for index in 1..count {
            let left_row = ray(
                left_feature,
                ids[index][0].as_u64().expect("left ID") as usize,
            );
            let right_row = ray(
                right_feature,
                ids[index][1].as_u64().expect("right ID") as usize,
            );
            emit_residual(
                &format!("x_row{index}"),
                left_row,
                right_row,
                translation,
                rotation_from_cayley,
            );
        }
    }
    let mut xh_translation = translation;
    let mut h0 = f64::EPSILON.sqrt() * translation[0].abs();
    if h0 == 0.0 {
        h0 = f64::EPSILON.sqrt();
    }
    xh_translation[0] += h0;
    emit_residual("xh_col0", left, right, xh_translation, rotation_from_cayley);
    if std::env::var_os("M8C_EXACT_BITS_ALL_ROWS").is_some() {
        let count = ids.len();
        for index in 1..count {
            let left_row = ray(
                left_feature,
                ids[index][0].as_u64().expect("left ID") as usize,
            );
            let right_row = ray(
                right_feature,
                ids[index][1].as_u64().expect("right ID") as usize,
            );
            emit_residual(
                &format!("xh_col0_row{index}"),
                left_row,
                right_row,
                xh_translation,
                rotation_from_cayley,
            );
        }
    }

    // Also invoke the production path on this exact pair.  The environment
    // trace is optional and keeps this probe independent of the ordinary gate.
    if std::env::var_os("M8C_EXACT_BITS_RUN_PRODUCTION").is_some() {
        let left_production = production_feature(left_feature);
        let right_production = production_feature(right_feature);
        let config = OfflineMapperConfig {
            max_points: 800,
            max_hamming: 70,
            second_best_ratio: 1.2,
            bow_bits: 16,
            match_window: 30,
            frames_to_match_threshold: 0.04,
            min_matches: 20,
            ransac_threshold: 5e-5,
            min_track_length: 5,
            min_triangulation_distance: 0.07,
        };
        let result = match_temporal_ransac_seeded(&left_production, &right_production, config, 7);
        eprintln!("M8C_PRODUCTION result={result:?}");
    }
}
