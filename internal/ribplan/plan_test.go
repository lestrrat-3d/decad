package ribplan_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/ribplan"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestFootprintHorizontalLine(t *testing.T) {
	t.Parallel()
	got, err := ribplan.Footprint(0, 0, 10, 0, 2, 1)
	require.NoError(t, err)
	want := [4][2]float64{{-1, -1}, {11, -1}, {11, 1}, {-1, 1}}
	require.Equal(t, want, got)
}

func TestFootprintDiagonalLine(t *testing.T) {
	t.Parallel()
	got, err := ribplan.Footprint(0, 0, 3, 3, math.Sqrt2, 0)
	require.NoError(t, err)
	for i, want := range [4][2]float64{{0.5, -0.5}, {3.5, 2.5}, {2.5, 3.5}, {-0.5, 0.5}} {
		require.InDelta(t, want[0], got[i][0], 1e-12, "corner %d u", i)
		require.InDelta(t, want[1], got[i][1], 1e-12, "corner %d v", i)
	}
}

func TestFootprintRefusals(t *testing.T) {
	t.Parallel()
	cases := map[string]struct{ u1, thick, over float64 }{
		"no length":      {0, 2, 0},
		"zero thickness": {5, 0, 0},
		"nan thickness":  {5, math.NaN(), 0},
		"negative over":  {5, 2, -1},
		"infinite over":  {5, 2, math.Inf(1)},
	}
	for name, tc := range cases {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			_, err := ribplan.Footprint(0, 0, tc.u1, 0, tc.thick, tc.over)
			require.Error(t, err)
		})
	}
}

func TestDisjoint(t *testing.T) {
	t.Parallel()
	lo, hi := r3.Vec{}, r3.Vec{X: 10, Y: 10, Z: 10}
	require.True(t, ribplan.Disjoint(lo, hi, r3.Vec{X: 11}, r3.Vec{X: 12, Y: 1, Z: 1}, 0))
	require.False(t, ribplan.Disjoint(lo, hi, r3.Vec{X: 11}, r3.Vec{X: 12, Y: 1, Z: 1}, 0.5), "slack closes the gap")
	require.False(t, ribplan.Disjoint(lo, hi, r3.Vec{X: 10}, r3.Vec{X: 12, Y: 1, Z: 1}, 0), "touching boxes are not disjoint")
}

func TestAddsNothing(t *testing.T) {
	t.Parallel()
	require.True(t, ribplan.AddsNothing(100, 0, 100, 0))
	require.False(t, ribplan.AddsNothing(100, 0, 100.5, 0))
	require.False(t, ribplan.AddsNothing(100, 1, 100, 0), "a bound on the reading before keeps the answer open")
}
