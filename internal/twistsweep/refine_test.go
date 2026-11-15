package twistsweep

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/stretchr/testify/require"
)

func refineRectangle(t *testing.T) TwistPolygon {
	t.Helper()
	poly, err := TwistPolygonOf(polygonRecord([2]float64{-5, -2}, [2]float64{5, -2},
		[2]float64{5, 2}, [2]float64{-5, 2}))
	require.NoError(t, err)
	return poly
}

// cellDeparture is the per-cell departure of a straight twist turning a radians
// per cell: R*a*a/8 + E*a/4, as the stationed build proves it with exact stations.
func cellDeparture(poly TwistPolygon, a float64) float64 {
	return poly.MaxRadiusL1*a*a/8 + poly.MaxEdgeL1*a/4
}

// A refinement uses the fewest cells whose per-cell departure is within
// refineTargetScale of the profile radius, and one cell fewer is not enough.
func TestRefinedCellCountReachesTarget(t *testing.T) {
	t.Parallel()
	poly := refineRectangle(t)
	target := refineTargetScale * poly.MaxRadiusL1
	for _, deg := range []float64{30, 60, 87} {
		upper := deg * math.Pi / 180
		cells, err := refinedCellCount(poly, upper)
		require.NoError(t, err, "%g degrees", deg)
		require.Less(t, cells, maxRefinedTwistCells, "%g degrees", deg)
		require.LessOrEqual(t, cellDeparture(poly, upper/float64(cells)), target, "%g degrees", deg)
		require.Greater(t, cellDeparture(poly, upper/float64(cells-1)), target, "%g degrees", deg)
	}
}

// The refinement target is the sleeve-class 0.15% of the profile radius: a
// 10 by 4 rectangle (radius L1 7) twisting 87 degrees departs by at most
// 0.0105 mm per cell.
func TestRefinedCellCountTargetScale(t *testing.T) {
	t.Parallel()
	poly := refineRectangle(t)
	upper := 87 * math.Pi / 180
	cells, err := refinedCellCount(poly, upper)
	require.NoError(t, err)
	require.LessOrEqual(t, cellDeparture(poly, upper/float64(cells)), 0.0015*poly.MaxRadiusL1)
}

// A twist that needs more cells than the ceiling stays at the ceiling, and a
// non-finite or empty request still refuses.
func TestRefinedCellCountCeilingAndRefusals(t *testing.T) {
	t.Parallel()
	poly := refineRectangle(t)
	cells, err := refinedCellCount(poly, 6)
	require.NoError(t, err)
	require.LessOrEqual(t, cells, maxRefinedTwistCells)
	require.Greater(t, cellDeparture(poly, 6/float64(cells)), refineTargetScale*poly.MaxRadiusL1)
	more, err := refinedCellCount(poly, 12)
	require.NoError(t, err)
	require.Equal(t, cells, more)

	for _, upper := range []float64{0, -1, math.Inf(1), math.NaN()} {
		_, err := refinedCellCount(poly, upper)
		require.ErrorIs(t, err, decaderr.ErrUnsupported, "angle %v", upper)
	}
	_, err = refinedCellCount(TwistPolygon{}, 0.5)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
}
