//go:build race

package dynamics_test

// raceDetector reports a build under the race detector, which slows the
// longest scene runs past the package's test budget.
const raceDetector = true
