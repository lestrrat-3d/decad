//go:build race

package twistsweep

// raceDetector reports a build under the race detector, which slows the
// stationed-cell differential sweep past the package's test budget.
const raceDetector = true
