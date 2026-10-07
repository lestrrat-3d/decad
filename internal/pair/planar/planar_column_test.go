package planar_test

import (
	"errors"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/pair/planar"
	"github.com/stretchr/testify/require"
)

// PlanarColumnClear's fixtures: the plane z = 0 with normal +Z, so the
// projection drops z, and the box [0, 1]³. Each triangle is a loose facet;
// the column test reads triangles one at a time and never the solid's
// closure.
//
// Legs shown to fail (each deleted in turn, fixture red, then restored):
//   - the front filter (a vertex strictly in front of the plane):
//     TestPlanarColumnIgnoresTrianglesBehind reads the triangles on and
//     behind the plane and reports the column blocked;
//   - the edge-normal axes: the diagonal triangle's gap falls to zero on the
//     box axes alone, and the column reads blocked;
//   - the division by the axis length: the diagonal gap reads 8, not √2.

func columnBox() ([3]*big.Rat, [3]*big.Rat) {
	return [3]*big.Rat{new(big.Rat), new(big.Rat), new(big.Rat)},
		[3]*big.Rat{big.NewRat(1, 1), big.NewRat(1, 1), big.NewRat(1, 1)}
}

func columnClear(t *testing.T, tris ...[3][3]float64) (*big.Rat, bool) {
	t.Helper()
	var s planar.PlanarSolid
	for _, tri := range tris {
		base := len(s.Verts)
		for _, v := range tri {
			s.Verts = append(s.Verts, vec(v[0], v[1], v[2]))
		}
		s.Tris = append(s.Tris, [3]int{base, base + 1, base + 2})
	}
	lo, hi := columnBox()
	clearance, open, err := planar.PlanarColumnClear(&s, vec(0, 0, 1), vec(0, 0, 0), lo, hi, noPoll)
	require.NoError(t, err)
	return clearance, open
}

func TestPlanarColumnDiagonalGap(t *testing.T) {
	// The triangle's edge from (4, 0) to (0, 4) lies on x + y = 4, which the
	// box's corner (1, 1) misses by (4 − 2)/√2 = √2; neither box axis
	// separates them. The axis (−4, −4) has length √32, bounded above by a
	// float within one ULP, so the clearance lies at or below √2 and within a
	// relative 2⁻⁵⁰ of it.
	diagonal := [3][3]float64{{4, 0, 1}, {0, 4, 1}, {4, 4, 5}}
	clearance, open := columnClear(t, diagonal)
	require.True(t, open)
	require.NotNil(t, clearance)
	squared := new(big.Rat).Mul(clearance, clearance)
	require.LessOrEqual(t, squared.Cmp(big.NewRat(2, 1)), 0)
	floor := new(big.Rat).Sub(big.NewRat(2, 1), new(big.Rat).SetFrac64(1, 1<<49))
	require.GreaterOrEqual(t, squared.Cmp(floor), 0)

	// A vertical wall projects to the segment x = 2: one unit from the box,
	// read on the box's own x axis with no rounding. The clearance is the
	// least over the triangles in front.
	wall := [3][3]float64{{2, -5, 0}, {2, 5, 0}, {2, 5, 3}}
	clearance, open = columnClear(t, wall)
	require.True(t, open)
	require.Zero(t, clearance.Cmp(big.NewRat(1, 1)))
	clearance, open = columnClear(t, diagonal, wall)
	require.True(t, open)
	require.Zero(t, clearance.Cmp(big.NewRat(1, 1)))

	// A triangle in front whose projection meets the box blocks the column.
	_, open = columnClear(t, wall, [3][3]float64{{0, 0, 2}, {3, 0, 2}, {0, 3, 2}})
	require.False(t, open)
	// Touching projections are not strictly apart.
	_, open = columnClear(t, [3][3]float64{{1, -5, 0}, {1, 5, 0}, {1, 5, 3}})
	require.False(t, open)
}

func TestPlanarColumnIgnoresTrianglesBehind(t *testing.T) {
	// Both triangles project over the box, one on the plane and one behind
	// it; neither has a vertex strictly in front, so the column is open
	// with nothing in front: an unbounded clearance.
	clearance, open := columnClear(t,
		[3][3]float64{{-1, -1, 0}, {3, -1, 0}, {-1, 3, 0}},
		[3][3]float64{{-1, -1, -1}, {3, -1, 0}, {-1, 3, -2}})
	require.True(t, open)
	require.Nil(t, clearance)
}

func TestPlanarColumnPollsAndStops(t *testing.T) {
	s := boxSolid([3]float64{2, 2, -1}, [3]float64{3, 3, 4})
	lo, hi := columnBox()
	stop := errors.New("stop")
	calls := 0
	_, _, err := planar.PlanarColumnClear(&s, vec(0, 0, 1), vec(0, 0, 0), lo, hi, func() error {
		calls++
		if calls > 3 {
			return stop
		}
		return nil
	})
	require.ErrorIs(t, err, stop)
}
