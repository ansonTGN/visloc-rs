//! Adapter from the direct optical-flow stream into the Basalt VIO estimator.
//!
//! The adapter is intentionally thin: frame ordering and image processing
//! stay in [`crate::DirectKltStream`], while the estimator receives only the
//! emitted observations plus the IMU interval selected by the EuRoC reader.
//! This leaves the input boundary stable when the estimator grows from the
//! current scaffold into the full ABS_QR implementation.

use thiserror::Error;

use crate::{
    config::{BasaltConfig, ConfigError},
    imu::{BiasRandomWalkNoise, ImuNoiseModel},
    stream::{DirectKltConfig, DirectKltStream, StreamError},
    timing::{TimingBreakdown, TimingBucket},
    vio::{BasaltVioEstimator, EstimatorOutput, OfImageData},
    EurocSensorFrame, TrackFrameOutput,
};

/// Result of one sensor event after frontend and estimator ingestion.
#[derive(Debug)]
pub struct BasaltAdapterOutput {
    pub tracks: TrackFrameOutput,
    pub estimator: EstimatorOutput,
    pub imu_count: usize,
}

/// A direct-KLT-to-Basalt-VIO boundary with no descriptor/PnP path.
#[derive(Debug)]
pub struct BasaltVioEstimatorAdapter {
    pub frontend: DirectKltStream,
    pub estimator: BasaltVioEstimator,
    timing: TimingBreakdown,
}

impl BasaltVioEstimatorAdapter {
    /// Builds both sides from the same pinned calibration/config contracts.
    pub fn from_config(
        calibration: &crate::BasaltCalibration,
        config: &BasaltConfig,
    ) -> Result<Self, BasaltAdapterError> {
        let direct_config = direct_klt_config(config)?;
        let estimator_config = config.estimator_config()?;
        let frontend = DirectKltStream::new(calibration.clone(), direct_config)?;
        let camera = *calibration
            .camera(0)
            .ok_or(BasaltAdapterError::MissingCamera0Calibration)?;
        let estimator = BasaltVioEstimator::new(camera, estimator_config)
            .with_camera_rig(calibration.cameras.clone(), calibration.t_imu_cam.clone())
            .map_err(BasaltAdapterError::Estimator)?
            .with_imu_noise(
                ImuNoiseModel {
                    // ImuPreintegrator injects per-sample covariance through
                    // the continuous-density API.  Basalt's EuRoC contract
                    // uses discrete sample covariance density² * rate, hence
                    // the equivalent sqrt(rate) scaling here.
                    // The pinned Basalt path keeps the continuous density in
                    // Scalar throughout `std * sqrt(rate)`.  Preserve that
                    // f32 boundary here before the value is stored in the
                    // public f64 noise model; otherwise an f64 product cast
                    // down by preintegration is one ulp below Eigen's
                    // discrete covariance (e.g. 3d51b717 vs 3d51b718).
                    gyro_density: sample_density_f32(
                        calibration.gyro_noise_std,
                        calibration.imu_update_rate_hz,
                    ),
                    accel_density: sample_density_f32(
                        calibration.accel_noise_std,
                        calibration.imu_update_rate_hz,
                    ),
                },
                BiasRandomWalkNoise {
                    // Basalt stores the *inverse* calibration standard
                    // deviations as bias square-root weights, then divides
                    // them by sqrt(dt) in the discrete random-walk rows.
                    gyro_density: inverse_rms_weight(calibration.gyro_bias_std),
                    accel_density: inverse_rms_weight(calibration.accel_bias_std),
                },
            )
            .map_err(BasaltAdapterError::Estimator)?
            .with_imu_calibration(
                calibration.calib_accel_bias.clone(),
                calibration.calib_gyro_bias.clone(),
            )
            .map_err(BasaltAdapterError::Estimator)?;
        Ok(Self {
            frontend,
            estimator,
            timing: TimingBreakdown::from_env(),
        })
    }

    /// Returns the optional cumulative timing breakdown for this adapter.
    pub fn timing_breakdown(&self) -> &TimingBreakdown {
        &self.timing
    }

    /// Returns adapter buckets plus the estimator's internal phase buckets.
    /// The returned value is a small fixed-size snapshot, so merging happens
    /// once at sidecar emission rather than taking a lock or allocating per
    /// frame.
    pub fn timing_breakdown_with_estimator(&self) -> TimingBreakdown {
        let mut timing = self.timing.clone();
        timing.merge_from(self.estimator.timing_breakdown());
        timing
    }

    pub fn process(
        &mut self,
        frame: EurocSensorFrame,
    ) -> Result<BasaltAdapterOutput, BasaltAdapterError> {
        self.process_impl(frame, true, true)
    }

    /// Processes a frame without retaining raw images or building the
    /// diagnostic MargData snapshot.  Frontend, solver, prior, and state
    /// updates remain identical to [`Self::process`].
    pub fn process_without_marg_data(
        &mut self,
        frame: EurocSensorFrame,
    ) -> Result<BasaltAdapterOutput, BasaltAdapterError> {
        self.process_impl(frame, false, true)
    }

    /// Processes a frame without raw-image/MargData retention or estimator
    /// trace payloads.  The returned [`EstimatorOutput`] is a lean view:
    /// `marg_data` is empty, `phases` is empty, optional state/IMU trace fields
    /// are absent, and window diagnostics contain only cheap metadata.  The
    /// solver, prior, writeback, and window lifecycle remain unchanged.
    ///
    /// This is a one-way mode for an adapter instance.  After this method has
    /// successfully started, [`Self::process`] rejects a return to
    /// MargData-retaining operation because earlier active keyframes may lack
    /// complete [`OfImageData`].  Continue with no-MargData methods or create
    /// a new adapter.  Basalt probe environment variables still make the
    /// estimator fall back to its retained diagnostic path.
    pub fn process_without_marg_data_no_trace(
        &mut self,
        frame: EurocSensorFrame,
    ) -> Result<BasaltAdapterOutput, BasaltAdapterError> {
        self.process_impl(frame, false, false)
    }

    fn process_impl(
        &mut self,
        frame: EurocSensorFrame,
        retain_marg_data: bool,
        retain_trace: bool,
    ) -> Result<BasaltAdapterOutput, BasaltAdapterError> {
        let started = self.timing.start();
        let result = self.process_impl_inner(frame, retain_marg_data, retain_trace);
        self.timing.finish(TimingBucket::AdapterTotal, started);
        result
    }

    fn process_impl_inner(
        &mut self,
        frame: EurocSensorFrame,
        retain_marg_data: bool,
        retain_trace: bool,
    ) -> Result<BasaltAdapterOutput, BasaltAdapterError> {
        if retain_marg_data && self.estimator.no_output_mode_active() {
            return Err(BasaltAdapterError::Estimator(
                "cannot retain MargData after no-output mode: active keyframes may lack complete OfImageData; recreate the adapter"
                    .into(),
            ));
        }
        let imu_count = frame.imu.len();
        // `StereoFrame::new` takes ownership of the decoded images.  Capture
        // the exact raw samples once here so the estimator can retain them
        // for keyframe MargData while the frontend consumes its own images;
        // no solver/pyramid copy is made afterward.
        let of_images = if retain_marg_data {
            let mut images = vec![raw_image_data(
                frame.frame_id,
                frame.timestamp_ns,
                0,
                &frame.cam0,
            )?];
            if let Some(cam1) = frame.cam1.as_ref() {
                images.push(raw_image_data(frame.frame_id, frame.timestamp_ns, 1, cam1)?);
            }
            Some(images)
        } else {
            None
        };
        let stereo =
            crate::StereoFrame::new(frame.frame_id, frame.timestamp_ns, frame.cam0, frame.cam1);
        let frontend_started = self.timing.start();
        let tracks_result = self.frontend.process_frame(stereo);
        self.timing
            .finish(TimingBucket::AdapterFrontend, frontend_started);
        let tracks = tracks_result?;
        let estimator_started = self.timing.start();
        let estimator_result = if let Some(of_images) = of_images {
            self.estimator.process_with_images(
                tracks.frame.frame_id,
                tracks.frame.timestamp_ns,
                &tracks.observations,
                &frame.imu,
                Some(of_images),
            )
        } else if retain_trace {
            self.estimator.process_without_marg_data(
                tracks.frame.frame_id,
                tracks.frame.timestamp_ns,
                &tracks.observations,
                &frame.imu,
            )
        } else {
            self.estimator.process_without_marg_data_no_trace(
                tracks.frame.frame_id,
                tracks.frame.timestamp_ns,
                &tracks.observations,
                &frame.imu,
            )
        };
        let estimator_result = estimator_result.map_err(BasaltAdapterError::Estimator);
        self.timing
            .finish(TimingBucket::AdapterEstimator, estimator_started);
        let estimator = estimator_result?;
        let output_started = self.timing.start();
        let output = BasaltAdapterOutput {
            tracks,
            estimator,
            imu_count,
        };
        self.timing
            .finish(TimingBucket::AdapterOutput, output_started);
        Ok(output)
    }
}

fn raw_image_data(
    frame_id: crate::FrameId,
    timestamp_ns: crate::TimestampNs,
    camera_id: u16,
    image: &crate::RawU16Image,
) -> Result<OfImageData, BasaltAdapterError> {
    OfImageData::new(
        frame_id,
        timestamp_ns,
        camera_id,
        image.width(),
        image.height(),
        image.pixels().to_vec(),
    )
    .ok_or_else(|| {
        BasaltAdapterError::Estimator(format!(
            "raw camera {camera_id} image dimensions/data are not representable"
        ))
    })
}

fn rms_density(vector: nalgebra::Vector3<f64>) -> f64 {
    (vector.norm_squared() / 3.0).sqrt()
}

fn sample_density_f32(vector: nalgebra::Vector3<f64>, rate_hz: f64) -> f64 {
    let rms = rms_density(vector) as f32;
    let rate = rate_hz as f32;
    (rms * rate.sqrt()) as f64
}

fn inverse_rms_weight(vector: nalgebra::Vector3<f64>) -> f64 {
    1.0 / rms_density(vector)
}

/// Extracts the direct-flow subset of the upstream config without importing
/// the repository's generic vision config.
pub fn direct_klt_config(config: &BasaltConfig) -> Result<DirectKltConfig, ConfigError> {
    let positive_usize = |key: &str| -> Result<usize, ConfigError> {
        let value: i64 = config.value(key)?;
        usize::try_from(value).map_err(|_| ConfigError::Value(format!("{key} must be positive")))
    };
    let positive_f32 = |key: &str| -> Result<f32, ConfigError> {
        let value: f64 = config.value(key)?;
        if value.is_finite() && value > 0.0 {
            Ok(value as f32)
        } else {
            Err(ConfigError::Value(format!("{key} must be positive")))
        }
    };
    Ok(DirectKltConfig {
        pyramid_levels: positive_usize("config.optical_flow_levels")?,
        max_iterations: positive_usize("config.optical_flow_max_iterations")?,
        fb_squared_threshold: positive_f32("config.optical_flow_max_recovered_dist2")?,
        essential_residual_threshold: f64::from(positive_f32(
            "config.optical_flow_epipolar_error",
        )?),
        fast: crate::GridFastConfig {
            cell_size: positive_usize("config.optical_flow_detection_grid_size")?,
            ..crate::GridFastConfig::default()
        },
    })
}

#[derive(Debug, Error)]
pub enum BasaltAdapterError {
    #[error("direct KLT stream error: {0}")]
    Frontend(#[from] StreamError),
    #[error("Basalt estimator error: {0}")]
    Estimator(String),
    #[error("camera 0 calibration is missing")]
    MissingCamera0Calibration,
    #[error("Basalt config error: {0}")]
    Config(#[from] ConfigError),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn direct_config_consumes_upstream_thresholds() {
        let json = include_str!("../../../configs/basalt/euroc_config.json");
        let config = BasaltConfig::from_json(json).unwrap();
        let direct = direct_klt_config(&config).unwrap();
        assert_eq!(direct.pyramid_levels, 3);
        assert_eq!(direct.max_iterations, 5);
        assert!((direct.fb_squared_threshold - 0.04).abs() < 1e-6);
        assert!((direct.essential_residual_threshold - 0.005).abs() < 1e-9);
        assert_eq!(direct.fast.cell_size, 50);
    }

    #[test]
    fn bias_random_walk_uses_inverse_calibration_std_as_sqrt_weight() {
        assert!((inverse_rms_weight(nalgebra::Vector3::repeat(1.0e-4)) - 1.0e4).abs() < 1.0e-9);
        assert!((inverse_rms_weight(nalgebra::Vector3::repeat(1.0e-3)) - 1.0e3).abs() < 1.0e-10);
    }
}
