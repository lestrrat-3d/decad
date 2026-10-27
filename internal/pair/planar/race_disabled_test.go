//go:build !race

package planar

// raceDetector reports a build under the race detector. Differential sweeps
// run a smaller case set under it; the plain build runs the full sweep.
const raceDetector = false
