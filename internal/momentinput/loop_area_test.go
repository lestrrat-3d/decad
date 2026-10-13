package momentinput_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

// rectangleLoop walks the rectangle (0,0)-(2,1), counter-clockwise or clockwise.
func rectangleLoop(ccw bool) sectionrecord.LoopRecord {
	pts := [][2]float64{{0, 0}, {2, 0}, {2, 1}, {0, 1}}
	if !ccw {
		pts = [][2]float64{{0, 0}, {0, 1}, {2, 1}, {2, 0}}
	}
	segs := make([]sectionrecord.CurveSegment, len(pts))
	for i, p := range pts {
		q := pts[(i+1)%len(pts)]
		segs[i] = sectionrecord.LineSeg{
			Start:  sectionrecord.Point2{U: p[0], V: p[1]},
			End:    sectionrecord.Point2{U: q[0], V: q[1]},
			TStart: 0, TEnd: 1,
		}
	}
	return sectionrecord.LoopRecord{Segments: segs}
}

// requireWithin checks the reading encloses want: |value - want| <= bound.
func requireWithin(t *testing.T, got proofbound.BoundedScalar, want float64, what string) {
	t.Helper()
	require.LessOrEqual(t, math.Abs(got.Value-want), got.Bound, what)
}

func TestLoopEnclosedAreaAndMoments(t *testing.T) {
	for _, ccw := range []bool{true, false} {
		loop := rectangleLoop(ccw)
		area, err := momentinput.LoopEnclosedArea(t.Context(), loop)
		require.NoError(t, err)
		requireWithin(t, area, 2, "area")
		require.Greater(t, area.Value, 0.0)

		a, mu, mv, err := momentinput.LoopEnclosedMoments(t.Context(), loop)
		require.NoError(t, err)
		requireWithin(t, a, 2, "moments area")
		requireWithin(t, mu, 2, "mu")
		requireWithin(t, mv, 1, "mv")
		require.Greater(t, a.Value, 0.0)
		require.Greater(t, mu.Value, 0.0)
		require.Greater(t, mv.Value, 0.0)
	}
}

func TestLoopEnclosedAreaHonoursCancellation(t *testing.T) {
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err := momentinput.LoopEnclosedArea(ctx, rectangleLoop(true))
	require.ErrorIs(t, err, context.Canceled)
}
