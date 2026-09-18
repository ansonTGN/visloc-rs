//! Timestamp and raw-IMU interval contracts.
//!
//! A [`TimeInterval`] is half-open: `[start_ns, end_ns)`. A sample exactly at
//! `start_ns` belongs to the interval, while a sample exactly at `end_ns`
//! belongs to the following interval. This makes adjacent camera intervals
//! partition an IMU stream without duplicating a boundary sample. The first
//! contract only selects existing raw samples; it does not synthesize endpoint
//! samples or perform preintegration.

use thiserror::Error;

use crate::types::{ImuSample, TimestampNs};

const NANOS_PER_SECOND: f64 = 1_000_000_000.0;

/// A half-open interval in the EuRoC nanosecond clock.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimeInterval {
    start_ns: TimestampNs,
    end_ns: TimestampNs,
}

impl TimeInterval {
    /// Creates `[start_ns, end_ns)`, allowing a zero-duration interval.
    pub const fn new(
        start_ns: TimestampNs,
        end_ns: TimestampNs,
    ) -> Result<Self, TimeIntervalError> {
        if end_ns < start_ns {
            return Err(TimeIntervalError::EndBeforeStart { start_ns, end_ns });
        }
        Ok(Self { start_ns, end_ns })
    }

    pub const fn start_ns(self) -> TimestampNs {
        self.start_ns
    }

    pub const fn end_ns(self) -> TimestampNs {
        self.end_ns
    }

    pub const fn is_empty(self) -> bool {
        self.start_ns == self.end_ns
    }

    pub const fn duration_ns(self) -> TimestampNs {
        self.end_ns - self.start_ns
    }

    pub fn duration_seconds(self) -> f64 {
        self.duration_ns() as f64 / NANOS_PER_SECOND
    }

    pub const fn contains(self, timestamp_ns: TimestampNs) -> bool {
        self.start_ns <= timestamp_ns && timestamp_ns < self.end_ns
    }

    /// Shifts both boundaries while checking signed-nanosecond overflow.
    pub fn checked_shift(self, offset_ns: TimestampNs) -> Result<Self, TimeIntervalError> {
        let start_ns = self
            .start_ns
            .checked_add(offset_ns)
            .ok_or(TimeIntervalError::TimestampOverflow)?;
        let end_ns = self
            .end_ns
            .checked_add(offset_ns)
            .ok_or(TimeIntervalError::TimestampOverflow)?;
        Self::new(start_ns, end_ns)
    }
}

/// A contiguous view of raw IMU samples selected by a half-open interval.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuInterval<'a> {
    interval: TimeInterval,
    samples: &'a [ImuSample],
}

impl<'a> ImuInterval<'a> {
    pub const fn interval(self) -> TimeInterval {
        self.interval
    }

    pub const fn samples(self) -> &'a [ImuSample] {
        self.samples
    }

    pub const fn sample_count(self) -> usize {
        self.samples.len()
    }

    pub fn duration_seconds(self) -> f64 {
        self.interval.duration_seconds()
    }
}

/// Errors raised while constructing or selecting a timestamp interval.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum TimeIntervalError {
    #[error("IMU interval ends before it starts: [{start_ns}, {end_ns})")]
    EndBeforeStart {
        start_ns: TimestampNs,
        end_ns: TimestampNs,
    },
    #[error("timestamp arithmetic overflow")]
    TimestampOverflow,
    #[error("IMU timestamps are not strictly increasing at {previous_ns} then {current_ns}")]
    NonIncreasing {
        previous_ns: TimestampNs,
        current_ns: TimestampNs,
    },
    #[error("no IMU sample lies in [{start_ns}, {end_ns})")]
    EmptySelection {
        start_ns: TimestampNs,
        end_ns: TimestampNs,
    },
}

/// Selects raw samples whose timestamp lies in `[interval.start, interval.end)`.
///
/// The complete input is checked for strictly increasing timestamps before
/// selection. This catches duplicate or reordered packets before a future
/// preintegrator can turn them into a silent timing error.
pub fn select_imu_interval<'a>(
    samples: &'a [ImuSample],
    interval: TimeInterval,
) -> Result<ImuInterval<'a>, TimeIntervalError> {
    for pair in samples.windows(2) {
        if pair[1].timestamp_ns <= pair[0].timestamp_ns {
            return Err(TimeIntervalError::NonIncreasing {
                previous_ns: pair[0].timestamp_ns,
                current_ns: pair[1].timestamp_ns,
            });
        }
    }

    let start = samples.partition_point(|sample| sample.timestamp_ns < interval.start_ns());
    let end = samples.partition_point(|sample| sample.timestamp_ns < interval.end_ns());
    if start == end {
        return Err(TimeIntervalError::EmptySelection {
            start_ns: interval.start_ns(),
            end_ns: interval.end_ns(),
        });
    }

    Ok(ImuInterval {
        interval,
        samples: &samples[start..end],
    })
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::*;

    fn imu(timestamp_ns: TimestampNs) -> ImuSample {
        ImuSample::new(timestamp_ns, Vector3::zeros(), Vector3::zeros())
    }

    #[test]
    fn interval_is_half_open_at_the_end_boundary() {
        let interval = TimeInterval::new(10, 20).unwrap();
        assert!(interval.contains(10));
        assert!(interval.contains(19));
        assert!(!interval.contains(20));

        let samples = [imu(9), imu(10), imu(19), imu(20)];
        let selected = select_imu_interval(&samples, interval).unwrap();
        assert_eq!(
            selected
                .samples()
                .iter()
                .map(|s| s.timestamp_ns)
                .collect::<Vec<_>>(),
            [10, 19]
        );
    }

    #[test]
    fn reordered_or_duplicate_samples_are_rejected() {
        let samples = [imu(10), imu(10)];
        let interval = TimeInterval::new(0, 20).unwrap();
        assert!(matches!(
            select_imu_interval(&samples, interval),
            Err(TimeIntervalError::NonIncreasing { .. })
        ));
    }
}
