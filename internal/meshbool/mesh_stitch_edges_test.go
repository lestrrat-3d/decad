package meshbool

import (
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/stretchr/testify/require"
)

// directedCensusClosed is the original closure rule, evaluated over separate
// directed keys and integer counts rather than the paired-edge bit table.
func directedCensusClosed(tris [][3]int) bool {
	directed := make(map[[2]int]int)
	for _, tri := range tris {
		for k, a := range tri {
			directed[[2]int{a, tri[(k+1)%3]}]++
		}
	}
	for edge, count := range directed {
		if count != 1 || directed[[2]int{edge[1], edge[0]}] != 1 {
			return false
		}
	}
	return true
}

func TestStitchEdgeDirectionsMatchDirectedCensus(t *testing.T) {
	t.Parallel()
	closed := [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}}
	check := func(tris [][3]int) {
		t.Helper()
		want := directedCensusClosed(tris)
		got := stitchEdgesPaired(tris, make(map[[2]int]uint8))
		require.Equal(t, want, got, "triangle count %d", len(tris))
	}
	check(nil)
	check(closed)
	check([][3]int{{0, 0, 1}}) // A self-edge is its own reverse in the original rule.
	for mask := range 1 << len(closed) {
		tris := slices.Clone(closed)
		for i := range tris {
			if mask&(1<<i) != 0 {
				tris[i][1], tris[i][2] = tris[i][2], tris[i][1]
			}
		}
		check(tris)
		check(tris[:3])
		check(append(slices.Clone(tris), tris[0]))
		check(append(slices.Clone(tris), tris...))
	}
	rng := rand.New(rand.NewPCG(419, 431))
	for range 1000 {
		tris := make([][3]int, rng.IntN(12))
		for i := range tris {
			for k := range tris[i] {
				tris[i][k] = rng.IntN(5)
			}
		}
		check(tris)
	}
}
