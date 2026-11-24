//go:build !race

package loftmesh

// boxes6Overlap is meshbool.BoxesOverlap over two six-number boxes. The race
// build's variant (box6_race.go) loads each box whole instead.
func boxes6Overlap(a, b []float64) bool {
	return a[0] <= b[3] && b[0] <= a[3] && a[1] <= b[4] && b[1] <= a[4] && a[2] <= b[5] && b[2] <= a[5]
}
