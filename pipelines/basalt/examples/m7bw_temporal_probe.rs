//! M7bw external temporal KLT trace for MH_01 frame 0 -> frame 1, cam0 track 2.
//!
//! This probe is intentionally outside the production stream. It exposes the
//! public patch/update primitives at every level/iteration so their f32 bits
//! can be compared with the pinned Eigen/Sophus probe.

use std::{env, path::PathBuf};

use nalgebra::{Matrix2, Vector2};
use visloc_basalt::{
    AffineCompact2f, DirectKltConfig, EurocSensorDataset, MeanNormalizedPatch51, RawU16Pyramid, Se2,
};

fn bits(value: f32) -> String {
    format!("0x{:08x}", value.to_bits())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = env::args_os().skip(1);
    let root = PathBuf::from(args.next().ok_or("dataset root")?);
    let calib = PathBuf::from(args.next().ok_or("calibration")?);
    let config_path = PathBuf::from(args.next().ok_or("config")?);
    let dataset = EurocSensorDataset::open(root, calib, config_path)?;
    let frame0 = dataset.frame(0)?;
    let frame1 = dataset.frame(1)?;
    let old_pyramid = RawU16Pyramid::from_image(frame0.cam0, 3)?;
    let current_pyramid = RawU16Pyramid::from_image(frame1.cam0, 3)?;
    let config = DirectKltConfig {
        pyramid_levels: 3,
        max_iterations: 5,
        ..DirectKltConfig::default()
    };
    let source = Vector2::new(46.0_f32, 118.0_f32);
    let mut transform = AffineCompact2f::new(Matrix2::identity(), source);

    println!("source={},{}", bits(source.x), bits(source.y));
    for level in (0..=config.pyramid_levels).rev() {
        let scale = (1_u32 << level) as f32;
        let old_position = source / scale;
        let old_image = old_pyramid.level(level).expect("old level");
        let current_image = current_pyramid.level(level).expect("current level");
        let patch = MeanNormalizedPatch51::from_image(old_image, old_position);
        println!(
            "L{level}.mean={} data0={} jac0={},{},{} valid={}",
            bits(patch.mean),
            bits(patch.data[0]),
            bits(patch.jacobian_se2[(0, 0)]),
            bits(patch.jacobian_se2[(0, 1)]),
            bits(patch.jacobian_se2[(0, 2)]),
            patch.valid
        );
        for row in 0..3 {
            print!("L{level}.inv{row}=");
            for sample in 0..52 {
                if sample != 0 {
                    print!(",");
                }
                print!("{}", bits(patch.h_se2_inv_j_se2_t[(row, sample)]));
            }
            println!();
        }
        let mut level_transform =
            AffineCompact2f::new(*transform.linear(), *transform.translation() / scale);
        for iteration in 0..config.max_iterations {
            let residual = patch
                .residual(current_image, &level_transform)
                .expect("temporal residual");
            print!("L{level}.I{iteration}.res=");
            for (index, value) in residual.iter().enumerate() {
                if index != 0 {
                    print!(",");
                }
                print!("{}", bits(*value));
            }
            println!();
            let increment = patch.ic_increment(&residual);
            println!(
                "L{level}.I{iteration}.inc={},{},{}",
                bits(increment.x),
                bits(increment.y),
                bits(increment.z)
            );
            level_transform.right_compose_se2(increment);
            println!(
                "L{level}.I{iteration}.t={},{}",
                bits(level_transform.translation().x),
                bits(level_transform.translation().y)
            );
            println!(
                "L{level}.I{iteration}.m={},{},{},{},{},{}",
                bits(level_transform.linear()[(0, 0)]),
                bits(level_transform.linear()[(0, 1)]),
                bits(level_transform.linear()[(1, 0)]),
                bits(level_transform.linear()[(1, 1)]),
                bits(level_transform.translation().x),
                bits(level_transform.translation().y)
            );
            let update = Se2::exp(increment);
            println!(
                "L{level}.I{iteration}.upd={},{},{},{},{},{}",
                bits(update.rotation[(0, 0)]),
                bits(update.rotation[(0, 1)]),
                bits(update.rotation[(1, 0)]),
                bits(update.rotation[(1, 1)]),
                bits(update.translation.x),
                bits(update.translation.y)
            );
            if !current_image.in_bounds(*level_transform.translation(), 2.0) {
                break;
            }
        }
        transform = AffineCompact2f::new(
            *level_transform.linear(),
            *level_transform.translation() * scale,
        );
        println!(
            "L{level}.world_t={},{}",
            bits(transform.translation().x),
            bits(transform.translation().y)
        );
        println!(
            "L{level}.world_m={},{},{},{},{},{}",
            bits(transform.linear()[(0, 0)]),
            bits(transform.linear()[(0, 1)]),
            bits(transform.linear()[(1, 0)]),
            bits(transform.linear()[(1, 1)]),
            bits(transform.translation().x),
            bits(transform.translation().y)
        );
    }
    println!(
        "final={},{}",
        bits(transform.translation().x),
        bits(transform.translation().y)
    );
    Ok(())
}
