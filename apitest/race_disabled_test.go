//go:build !race

package apitest_test

// raceDetector reports a build under the race detector. A table test may run a
// smaller case set under it; the plain build runs every case.
const raceDetector = false
