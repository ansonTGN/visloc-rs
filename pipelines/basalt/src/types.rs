//! Foundational Basalt measurement and state types.
//!
//! The aliases in this module are intentionally small and explicit. A
//! timestamp is a signed count of nanoseconds in the EuRoC dataset clock;
//! camera pixels are expressed in the calibrated camera image plane; and
//! [`BasaltNavState::imu_to_world`] maps a point from the IMU frame into the
//! world frame.

use nalgebra::{Point2, Vector3};
use visloc_core::geometry::SE3;

/// EuRoC/ Basalt timestamps measured in nanoseconds since the dataset epoch.
pub type TimestampNs = i64;

/// Stable identifier for a camera stream in a Basalt calibration.
pub type CameraId = u16;

/// Stable identifier for a visual frame.
pub type FrameId = u64;

/// Stable identifier for a tracked feature/landmark.
pub type TrackId = u64;

/// A camera frame header used by the Basalt pipeline.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BasaltFrame {
    pub frame_id: FrameId,
    pub timestamp_ns: TimestampNs,
    pub camera_id: CameraId,
}

impl BasaltFrame {
    pub const fn new(frame_id: FrameId, timestamp_ns: TimestampNs, camera_id: CameraId) -> Self {
        Self {
            frame_id,
            timestamp_ns,
            camera_id,
        }
    }
}

/// One raw IMU sample.
///
/// `gyro_rad_s` is angular velocity in radians per second and
/// `accel_m_s2` is specific force in metres per second squared. Both vectors
/// are expressed in the IMU body frame. Interval selection is defined in
/// [`crate::time`]; this type does not silently convert units or timestamps.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuSample {
    pub timestamp_ns: TimestampNs,
    pub gyro_rad_s: Vector3<f64>,
    pub accel_m_s2: Vector3<f64>,
}

impl ImuSample {
    pub fn new(
        timestamp_ns: TimestampNs,
        gyro_rad_s: Vector3<f64>,
        accel_m_s2: Vector3<f64>,
    ) -> Self {
        Self {
            timestamp_ns,
            gyro_rad_s,
            accel_m_s2,
        }
    }
}

/// A feature observation in a calibrated camera image.
///
/// `pixel` is in pixel coordinates (the origin and axis orientation are those
/// of the EuRoC image stream), not a normalized pinhole coordinate.
#[derive(Debug, Clone, PartialEq)]
pub struct TrackObservation {
    pub track_id: TrackId,
    pub frame_id: FrameId,
    pub timestamp_ns: TimestampNs,
    pub camera_id: CameraId,
    pub pixel: Point2<f64>,
}

/// The minimal navigation state required by the later Basalt VI optimizer.
///
/// `imu_to_world` is the active transform `T_w_i`: it maps IMU-frame points
/// into the world frame. This is deliberately distinct from the calibration
/// field `T_imu_cam`, which maps camera-frame points into the IMU frame.
#[derive(Debug, Clone, PartialEq)]
pub struct BasaltNavState {
    pub imu_to_world: SE3,
    pub velocity_world_m_s: Vector3<f64>,
    pub gyro_bias_rad_s: Vector3<f64>,
    pub accel_bias_m_s2: Vector3<f64>,
}

impl Default for BasaltNavState {
    fn default() -> Self {
        Self {
            imu_to_world: SE3::identity(),
            velocity_world_m_s: Vector3::zeros(),
            gyro_bias_rad_s: Vector3::zeros(),
            accel_bias_m_s2: Vector3::zeros(),
        }
    }
}
