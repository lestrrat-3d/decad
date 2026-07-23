package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestRequireLoopClearanceOffersValidRetry(t *testing.T) {
	pts := []Point2{
		{U: 0, V: 0}, {U: 1, V: 0}, {U: 1, V: 1}, {U: 0, V: 1},
		{U: 1.25, V: 0}, {U: 2.25, V: 0}, {U: 2.25, V: 1}, {U: 1.25, V: 1},
	}
	loops := [][]int{{0, 1, 2, 3}, {4, 5, 6, 7}}
	sags := []float64{0.2, 0.2}
	floor := 1e-9*math.Hypot(2.25, 1) + 4*(math.Nextafter(2.25, math.Inf(1))-2.25)

	err := requireLoopClearance(t.Context(), pts, loops, sags)
	require.ErrorIs(t, err, ErrDegenerate)
	require.Contains(t, err.Error(), `cap boundary loops 0 and 1`)
	require.Contains(t, err.Error(), `measured distance `+units.Millimeters(0.25).String())
	require.Contains(t, err.Error(), `required clearance gate `+units.Millimeters(0.4+floor).String())
	require.Contains(t, err.Error(), `retry with a finer chord tolerance`)

	require.NoError(t, requireLoopClearance(t.Context(), pts, loops, []float64{0.1, 0.1}))
}

func TestRequireLoopClearanceOmitsInvalidRetry(t *testing.T) {
	pts := []Point2{
		{U: 0, V: 0}, {U: 1, V: 0}, {U: 1, V: 1}, {U: 0, V: 1},
		{U: 1, V: 0}, {U: 2, V: 0}, {U: 2, V: 1}, {U: 1, V: 1},
	}
	loops := [][]int{{0, 1, 2, 3}, {4, 5, 6, 7}}
	sags := []float64{0.2, 0.2}
	floor := 1e-9*math.Hypot(2, 1) + 4*(math.Nextafter(2, math.Inf(1))-2)

	err := requireLoopClearance(t.Context(), pts, loops, sags)
	require.ErrorIs(t, err, ErrDegenerate)
	require.Contains(t, err.Error(), `cap boundary loops 0 and 1`)
	require.Contains(t, err.Error(), `measured distance 0 mm`)
	require.Contains(t, err.Error(), `required clearance gate `+units.Millimeters(0.4+floor).String())
	require.NotContains(t, err.Error(), `retry`)

	err = requireLoopClearance(t.Context(), pts, loops, []float64{0, 0})
	require.ErrorIs(t, err, ErrDegenerate)
}
