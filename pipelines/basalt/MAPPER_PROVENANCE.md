# Offline mapper provenance

The mapper factor contracts are a clean-room reconstruction target for Basalt
fixed SHA `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`. Relative-pose, roll-pitch,
and BA covisibility factors are recovered independently from versioned
`MargData`; no pose-only Chow--Liu implementation is used.
