//go:build race

package smoothloft

// raceDetector reports a build under the race detector, which slows the
// exact-rational reference evaluations past the package's test budget.
const raceDetector = true
