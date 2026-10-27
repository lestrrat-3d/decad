//go:build !race

package revolvemesh

// raceDetector reports a build under the race detector, which slows the
// float pre-test differential sweeps past the package's test budget.
const raceDetector = false
