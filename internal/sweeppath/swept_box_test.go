package sweeppath_test

import (
	"math"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sweeppath"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestCheckedBoundsCorners(t *testing.T) {
	lower := r3.Vec{X: 0, Y: 1, Z: 2}
	upper := r3.Vec{X: 4, Y: 5, Z: 6}

	t.Run("corners are pushed outward by the bound", func(t *testing.T) {
		corners, ok := sweeppath.CheckedBoundsCorners(lower, upper, units.Millimeters(0.5))
		require.True(t, ok)
		for index, corner := range corners {
			for axis, lo := range [3]float64{-0.5, 0.5, 1.5} {
				hi := [3]float64{4.5, 5.5, 6.5}[axis]
				want := lo
				if (index>>axis)&1 == 1 {
					want = hi
				}
				got := proofarith.DyFloatDown(corner[axis])
				require.Equal(t, want, got)
			}
		}
	})
	t.Run("refusals", func(t *testing.T) {
		for name, tc := range map[string]struct {
			lower, upper r3.Vec
			bound        units.Value
		}{
			"negative bound":      {lower, upper, units.Millimeters(-1)},
			"nan bound":           {lower, upper, units.Millimeters(math.NaN())},
			"infinite bound":      {lower, upper, units.Millimeters(math.Inf(1))},
			"non-length bound":    {lower, upper, units.Radians(0.1)},
			"nan coordinate":      {r3.Vec{X: math.NaN()}, upper, units.Millimeters(0)},
			"infinite coordinate": {lower, r3.Vec{Z: math.Inf(1)}, units.Millimeters(0)},
			"inverted box":        {upper, lower, units.Millimeters(0)},
		} {
			_, ok := sweeppath.CheckedBoundsCorners(tc.lower, tc.upper, tc.bound)
			require.False(t, ok, name)
		}
	})
}
