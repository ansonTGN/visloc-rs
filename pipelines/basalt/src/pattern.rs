//! Basalt optical-flow sampling patterns.

/// Basalt's `Pattern51`: the 52-point `Pattern52` layout scaled by one half.
///
/// The ordering is part of the numerical contract because patch vectors and
/// Jacobian rows use the same sample index.
pub struct Pattern51;

impl Pattern51 {
    pub const SAMPLE_COUNT: usize = 52;

    pub const OFFSETS: [[f32; 2]; Self::SAMPLE_COUNT] = [
        [-1.5, 3.5],
        [-0.5, 3.5],
        [0.5, 3.5],
        [1.5, 3.5],
        [-2.5, 2.5],
        [-1.5, 2.5],
        [-0.5, 2.5],
        [0.5, 2.5],
        [1.5, 2.5],
        [2.5, 2.5],
        [-3.5, 1.5],
        [-2.5, 1.5],
        [-1.5, 1.5],
        [-0.5, 1.5],
        [0.5, 1.5],
        [1.5, 1.5],
        [2.5, 1.5],
        [3.5, 1.5],
        [-3.5, 0.5],
        [-2.5, 0.5],
        [-1.5, 0.5],
        [-0.5, 0.5],
        [0.5, 0.5],
        [1.5, 0.5],
        [2.5, 0.5],
        [3.5, 0.5],
        [-3.5, -0.5],
        [-2.5, -0.5],
        [-1.5, -0.5],
        [-0.5, -0.5],
        [0.5, -0.5],
        [1.5, -0.5],
        [2.5, -0.5],
        [3.5, -0.5],
        [-3.5, -1.5],
        [-2.5, -1.5],
        [-1.5, -1.5],
        [-0.5, -1.5],
        [0.5, -1.5],
        [1.5, -1.5],
        [2.5, -1.5],
        [3.5, -1.5],
        [-2.5, -2.5],
        [-1.5, -2.5],
        [-0.5, -2.5],
        [0.5, -2.5],
        [1.5, -2.5],
        [2.5, -2.5],
        [-1.5, -3.5],
        [-0.5, -3.5],
        [0.5, -3.5],
        [1.5, -3.5],
    ];

    pub const fn offsets() -> &'static [[f32; 2]; Self::SAMPLE_COUNT] {
        &Self::OFFSETS
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pattern51_has_the_upstream_52_sample_order() {
        assert_eq!(Pattern51::SAMPLE_COUNT, 52);
        assert_eq!(Pattern51::OFFSETS[0], [-1.5, 3.5]);
        assert_eq!(Pattern51::OFFSETS[51], [1.5, -3.5]);
        assert_eq!(Pattern51::OFFSETS.iter().map(|p| p[0]).sum::<f32>(), 0.0);
        assert_eq!(Pattern51::OFFSETS.iter().map(|p| p[1]).sum::<f32>(), 0.0);
    }
}
