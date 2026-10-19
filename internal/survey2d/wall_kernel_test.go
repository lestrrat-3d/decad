package survey2d_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/stretchr/testify/require"
)

func TestWallKernelFlagsOffJunctionSubTolerance(t *testing.T) {
	outer, ok := survey2d.ArcElem(0, 0, 10, 0, 2*math.Pi, true)
	require.True(t, ok)
	inner, ok := survey2d.ArcElem(0, 0, 10-2e-8, 2*math.Pi, 0, true)
	require.True(t, ok)
	out := survey2d.NewWallKernel([]survey2d.SurveyElem{outer, inner}, nil, math.Inf(1)).Run()
	require.True(t, out.SubTolFar)
}

func TestWallKernelCleanProfileDoesNotFlag(t *testing.T) {
	points := [][2]float64{{0, 0}, {100, 0}, {100, 60}, {0, 60}}
	elems := make([]survey2d.SurveyElem, 0, len(points))
	for i := range points {
		elem, ok := survey2d.LineElem(points[i][0], points[i][1], points[(i+1)%len(points)][0], points[(i+1)%len(points)][1])
		require.True(t, ok)
		elems = append(elems, elem)
	}
	out := survey2d.NewWallKernel(elems, points, math.Inf(1)).Run()
	require.True(t, out.Ok)
	require.False(t, out.SubTolFar)
	require.True(t, out.HasSpan)
	require.InDelta(t, 60, out.Span, 1e-9)
}

func TestDecidePullMatchesOpposesPullAtZeroAllowance(t *testing.T) {
	pairs := [][2]float64{{0, 0}, {-1, -1}, {-1, 0}, {-1e-17, -1e-17}, {-0.5, 0.5}, {-0.5, -0.5}, {0.5, 0.5}, {-1, 1}, {-2, 0.5}, {-0.1, 0.1}}
	for _, pair := range pairs {
		want := survey2d.PullClear
		if survey2d.OpposesPull(pair[0], pair[1]) {
			want = survey2d.PullOpposes
		}
		got := survey2d.DecidePull(pair[0], pair[1], 0)
		require.Equalf(t, want, got, "mn=%v mx=%v", pair[0], pair[1])
		require.NotEqual(t, survey2d.PullUndecided, got)
	}
}
