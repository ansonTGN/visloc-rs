//! Adapter from the direct optical-flow stream into the Basalt VIO estimator.
//!
//! The adapter is intentionally thin: frame ordering and image processing
//! stay in [`crate::DirectKltStream`], while the estimator receives only the
//! emitted observations plus the IMU interval selected by the EuRoC reader.
//! This leaves the input boundary stable when the estimator grows from the
//! current scaffold into the full ABS_QR implementation.

use std::sync::mpsc;

use thiserror::Error;

use crate::{
    config::{BasaltConfig, ConfigError},
    euroc::{EurocReaderError, EurocSensorDataset},
    imu::{BiasRandomWalkNoise, ImuNoiseModel},
    stream::{DirectKltConfig, DirectKltStream, StreamError},
    timing::{TimingBreakdown, TimingBucket},
    types::ImuSample,
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
        let tracks_result = self
            .frontend
            .process_frame_with_timing(stereo, &mut self.timing);
        self.timing
            .finish(TimingBucket::AdapterFrontend, frontend_started);
        let tracks = tracks_result?;
        let estimator_started = self.timing.start();
        let estimator_result = self.estimator.process_adapter_frame(
            tracks.frame.frame_id,
            tracks.frame.timestamp_ns,
            &tracks.observations,
            &frame.imu,
            frame.initialization_imu,
            of_images,
            retain_marg_data,
            retain_trace,
        );
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

    /// Runs `frame_count` EuRoC frames through the frontend and estimator on
    /// two threads: one drives `dataset` acquisition plus [`DirectKltStream`]
    /// tracking (mirroring upstream Basalt's `OpticalFlow` thread), the other
    /// -- the calling thread -- drains a bounded channel of frontend packets
    /// into the estimator (mirroring upstream Basalt's estimator thread).
    ///
    /// This changes no numerics: both threads still process frame indices
    /// `0, 1, 2, ...` strictly in order, and each frame's frontend output is
    /// still fully computed before that same frame reaches the estimator.
    /// The only thing that overlaps in wall-clock time is frontend work for
    /// frame `N+1` running concurrently with estimator work for frame `N` --
    /// the arithmetic performed by each stage, and the order in which it is
    /// performed, is identical to calling [`Self::process`] /
    /// [`Self::process_without_marg_data`] /
    /// [`Self::process_without_marg_data_no_trace`] serially once per frame.
    /// The recovered trajectory is therefore bit-identical to the serial
    /// path given the same inputs.
    ///
    /// `on_output` is invoked once per frame, in frame order, on the
    /// estimator thread immediately after that frame's estimator step
    /// completes -- mirroring a serial per-frame replay loop's trajectory
    /// and MargData writes -- so memory stays bounded to roughly
    /// `channel_capacity` in-flight frames rather than the whole replay. It
    /// receives a thread-local [`TimingBreakdown`] to record its own timed
    /// regions (e.g. `DemoOutput`) into.
    ///
    /// Returns the producer-side (dataset + frontend) and consumer-side
    /// (estimator + `on_output`) cumulative timing breakdowns; the caller
    /// merges these into its own collector the same way
    /// [`Self::timing_breakdown_with_estimator`] merges the serial path's.
    pub fn process_euroc_stream_pipelined<F>(
        &mut self,
        dataset: &EurocSensorDataset,
        frame_count: usize,
        retain_marg_data: bool,
        retain_trace: bool,
        channel_capacity: usize,
        mut on_output: F,
    ) -> Result<(TimingBreakdown, TimingBreakdown), BasaltAdapterError>
    where
        F: FnMut(BasaltAdapterOutput, &mut TimingBreakdown) -> Result<(), BasaltAdapterError>,
    {
        let capacity = channel_capacity.max(1);
        let (sender, receiver) =
            mpsc::sync_channel::<Result<PipelineFrontendPacket, BasaltAdapterError>>(capacity);
        let frontend = &mut self.frontend;
        let estimator = &mut self.estimator;

        let (producer_timing, producer_result, mut consumer_timing, consumer_error) =
            std::thread::scope(|scope| {
                let producer_handle = scope.spawn(move || {
                    run_pipeline_frontend(frontend, dataset, frame_count, retain_marg_data, &sender)
                });

                let (consumer_timing, consumer_error) = run_pipeline_estimator(
                    estimator,
                    &receiver,
                    retain_marg_data,
                    retain_trace,
                    &mut on_output,
                );

                // Keep draining after an early consumer error (or after the
                // consumer loop above already exhausted the channel) so the
                // producer's next `send` cannot block forever on a channel
                // nobody is reading; this discards packets, it does not
                // process them.
                while receiver.recv().is_ok() {}

                let (producer_timing, producer_result) = producer_handle
                    .join()
                    .expect("basalt pipeline frontend thread panicked");
                (
                    producer_timing,
                    producer_result,
                    consumer_timing,
                    consumer_error,
                )
            });

        producer_result?;
        if let Some(error) = consumer_error {
            return Err(error);
        }
        consumer_timing.merge_from(self.estimator.timing_breakdown());
        Ok((producer_timing, consumer_timing))
    }
}

/// One frontend-produced, estimator-bound packet queued between the two
/// [`BasaltVioEstimatorAdapter::process_euroc_stream_pipelined`] threads.
struct PipelineFrontendPacket {
    tracks: TrackFrameOutput,
    imu: Vec<ImuSample>,
    initialization_imu: Option<ImuSample>,
    of_images: Option<Vec<OfImageData>>,
    imu_count: usize,
}

/// Producer-thread body: dataset acquisition plus frontend tracking, in
/// frame order, sent downstream as they complete. Returns its own timing
/// collector plus an error if dataset acquisition itself failed outside the
/// per-frame `Result` already carried over the channel (currently always
/// `Ok(())`; the `Result` return is kept so a future fatal-before-loop error
/// has somewhere to go without changing this function's signature).
fn run_pipeline_frontend(
    frontend: &mut DirectKltStream,
    dataset: &EurocSensorDataset,
    frame_count: usize,
    retain_marg_data: bool,
    sender: &mpsc::SyncSender<Result<PipelineFrontendPacket, BasaltAdapterError>>,
) -> (TimingBreakdown, Result<(), BasaltAdapterError>) {
    let mut timing = TimingBreakdown::from_env();
    for index in 0..frame_count {
        let frame_result = if timing.enabled() {
            timing
                .measure_with(TimingBucket::DatasetFrameAcquisition, |timing| {
                    dataset.frame_with_timing(index, timing)
                })
                .map_err(BasaltAdapterError::from)
        } else {
            dataset.frame(index).map_err(BasaltAdapterError::from)
        };
        let frame = match frame_result {
            Ok(frame) => frame,
            Err(error) => {
                let _ = sender.send(Err(error));
                return (timing, Ok(()));
            }
        };
        // Capture the exact raw samples before `StereoFrame::new` consumes
        // the decoded images, exactly as the serial path does.
        let of_images = if retain_marg_data {
            match pipeline_of_images(&frame) {
                Ok(images) => Some(images),
                Err(error) => {
                    let _ = sender.send(Err(error));
                    return (timing, Ok(()));
                }
            }
        } else {
            None
        };
        let imu_count = frame.imu.len();
        let stereo =
            crate::StereoFrame::new(frame.frame_id, frame.timestamp_ns, frame.cam0, frame.cam1);
        let frontend_started = timing.start();
        let tracks_result = frontend.process_frame_with_timing(stereo, &mut timing);
        timing.finish(TimingBucket::AdapterFrontend, frontend_started);
        let tracks = match tracks_result {
            Ok(tracks) => tracks,
            Err(error) => {
                let _ = sender.send(Err(BasaltAdapterError::from(error)));
                return (timing, Ok(()));
            }
        };
        let packet = PipelineFrontendPacket {
            tracks,
            imu: frame.imu,
            initialization_imu: frame.initialization_imu,
            of_images,
            imu_count,
        };
        if sender.send(Ok(packet)).is_err() {
            // The estimator thread stopped reading, almost always because it
            // already returned an error of its own.
            break;
        }
    }
    (timing, Ok(()))
}

fn pipeline_of_images(frame: &EurocSensorFrame) -> Result<Vec<OfImageData>, BasaltAdapterError> {
    let mut images = vec![raw_image_data(
        frame.frame_id,
        frame.timestamp_ns,
        0,
        &frame.cam0,
    )?];
    if let Some(cam1) = frame.cam1.as_ref() {
        images.push(raw_image_data(frame.frame_id, frame.timestamp_ns, 1, cam1)?);
    }
    Ok(images)
}

/// Consumer-thread (the caller's own thread) body: estimator ingestion plus
/// `on_output`, in the frame order the channel delivers -- which is frame
/// order, since the producer sends strictly in `0..frame_count` order into a
/// FIFO channel.
fn run_pipeline_estimator<F>(
    estimator: &mut BasaltVioEstimator,
    receiver: &mpsc::Receiver<Result<PipelineFrontendPacket, BasaltAdapterError>>,
    retain_marg_data: bool,
    retain_trace: bool,
    on_output: &mut F,
) -> (TimingBreakdown, Option<BasaltAdapterError>)
where
    F: FnMut(BasaltAdapterOutput, &mut TimingBreakdown) -> Result<(), BasaltAdapterError>,
{
    let mut timing = TimingBreakdown::from_env();
    while let Ok(message) = receiver.recv() {
        let packet = match message {
            Ok(packet) => packet,
            Err(error) => return (timing, Some(error)),
        };
        if retain_marg_data && estimator.no_output_mode_active() {
            return (
                timing,
                Some(BasaltAdapterError::Estimator(
                    "cannot retain MargData after no-output mode: active keyframes may lack complete OfImageData; recreate the adapter"
                        .into(),
                )),
            );
        }
        let estimator_started = timing.start();
        let estimator_result = estimator.process_adapter_frame(
            packet.tracks.frame.frame_id,
            packet.tracks.frame.timestamp_ns,
            &packet.tracks.observations,
            &packet.imu,
            packet.initialization_imu,
            packet.of_images,
            retain_marg_data,
            retain_trace,
        );
        timing.finish(TimingBucket::AdapterEstimator, estimator_started);
        let estimator_output = match estimator_result {
            Ok(output) => output,
            Err(error) => return (timing, Some(BasaltAdapterError::Estimator(error))),
        };
        let output_started = timing.start();
        let output = BasaltAdapterOutput {
            tracks: packet.tracks,
            estimator: estimator_output,
            imu_count: packet.imu_count,
        };
        timing.finish(TimingBucket::AdapterOutput, output_started);
        if let Err(error) = on_output(output, &mut timing) {
            return (timing, Some(error));
        }
    }
    (timing, None)
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
    #[error("EuRoC dataset error: {0}")]
    Dataset(#[from] EurocReaderError),
    /// A caller-provided `on_output` callback (e.g. trajectory/MargData
    /// serialization in [`Self::process_euroc_stream_pipelined`]) failed.
    /// This variant exists purely to carry that error type through the
    /// pipelined API's `Result<_, BasaltAdapterError>` boundary.
    #[error("pipeline output callback error: {0}")]
    Output(String),
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
