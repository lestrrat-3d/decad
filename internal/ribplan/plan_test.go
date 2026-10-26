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

func TestArcBandQuarterTurn(t *testing.T) {
	t.Parallel()
	// A quarter of the radius-10 circle about (1, 2), from angle 0 to 90 degrees.
	got, err := ribplan.ArcBand(1, 2, 11, 2, 1, 12, 2, 0)
	require.NoError(t, err)
	require.InDelta(t, 9, got.Inner, 1e-12)
	require.InDelta(t, 11, got.Outer, 1e-12)
	require.InDelta(t, 0, got.Start, 1e-12)
	require.InDelta(t, math.Pi/2, got.End, 1e-12)
	u, v := got.Point(got.Outer, got.End)
	require.InDelta(t, 1, u, 1e-12)
	require.InDelta(t, 13, v, 1e-12)
}

func TestArcBandOverrunExtendsBothEndsByArcLength(t *testing.T) {
	t.Parallel()
	got, err := ribplan.ArcBand(0, 0, 10, 0, 0, 10, 2, 5)
	require.NoError(t, err)
	// 5 mm along a radius-10 arc is half a radian at each end.
	require.InDelta(t, -0.5, got.Start, 1e-12)
	require.InDelta(t, math.Pi/2+0.5, got.End, 1e-12)
}

func TestArcBandSweepsCounterClockwiseAcrossTheCut(t *testing.T) {
	t.Parallel()
	// From 270 degrees round through 0 to 90 degrees: a half turn.
	got, err := ribplan.ArcBand(0, 0, 0, -10, 0, 10, 2, 0)
	require.NoError(t, err)
	require.InDelta(t, math.Pi, got.End-got.Start, 1e-12)
}

func TestArcBandRefusals(t *testing.T) {
	t.Parallel()
	cases := map[string]func() error{
		"zero thickness":         func() error { _, err := ribplan.ArcBand(0, 0, 10, 0, 0, 10, 0, 0); return err },
		"negative overrun":       func() error { _, err := ribplan.ArcBand(0, 0, 10, 0, 0, 10, 2, -1); return err },
		"ends on two circles":    func() error { _, err := ribplan.ArcBand(0, 0, 10, 0, 0, 11, 2, 0); return err },
		"end on the centre":      func() error { _, err := ribplan.ArcBand(0, 0, 10, 0, 0, 0, 2, 0); return err },
		"rib too thick":          func() error { _, err := ribplan.ArcBand(0, 0, 3, 0, 0, 3, 8, 0); return err },
		"overrun wraps the turn": func() error { _, err := ribplan.ArcBand(0, 0, 10, 0, 0, 10, 2, 40); return err },
	}
	for name, call := range cases {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			require.Error(t, call())
		})
	}
}
