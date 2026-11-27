package meshbool

import (
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/edgetable"
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
		got := stitchEdgesPaired(tris, edgetable.New(0))
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

// stitchEdgesPairedMap is the map-based stitchEdgesPaired that the edge table
// version replaced, kept as the differential reference.
func stitchEdgesPairedMap(tris [][3]int, edges map[[2]int]uint8) bool {
	for _, tri := range tris {
		for k, a := range tri {
			b := tri[(k+1)%3]
			var direction uint8 = 1
			if a > b {
				a, b = b, a
				direction = 2
			} else if a == b {
				direction = 3
			}
			key := [2]int{a, b}
			seen := edges[key]
			if seen&direction != 0 {
				return false
			}
			edges[key] = seen | direction
		}
	}
	for _, seen := range edges {
		if seen != 3 {
			return false
		}
	}
	return true
}

// closedStitchFixture returns a random closed triangulated surface: a
// tetrahedron whose faces are repeatedly split, so every directed edge is
// paired. Vertex ids start at base.
func closedStitchFixture(rng *rand.Rand, base, splits int) [][3]int {
	tris := [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}}
	next := 4
	for range splits {
		i := rng.IntN(len(tris))
		t := tris[i]
		c := next
		next++
		tris[i] = [3]int{t[0], t[1], c}
		tris = append(tris, [3]int{t[1], t[2], c}, [3]int{t[2], t[0], c})
	}
	for i := range tris {
		for k := range tris[i] {
			tris[i][k] += base
		}
	}
	return tris
}
