//go:build race

package loftmesh

// boxes6Overlap is meshbool.BoxesOverlap over two six-number boxes.
//
// Under -race, the build CI tests with, it loads each box whole: one load of
// six numbers is one instrumented access, where indexing the slice is one per
// number compared. The plain build indexes and stops at the first separating
// comparison, which is faster there; measure both builds before changing
// either.
func boxes6Overlap(a, b []float64) bool {
	x, y := [6]float64(a), [6]float64(b)
	return x[0] <= y[3] && y[0] <= x[3] && x[1] <= y[4] && y[1] <= x[4] && x[2] <= y[5] && y[2] <= x[5]
}
