//go:build !race

package decad

// raceDetector reports a build under the race detector, which slows the full
// z = 20 and z = 40 gear builds past the race shards' budget.
const raceDetector = false
