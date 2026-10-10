package circularbounds

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestCircleBoxBoundsOnlyTrimmedRange(t *testing.T) {
	for _, tc := range []struct {
		name       string
		start, end float64
	}{
		{name: "short root interval", start: 0.01, end: 0.02},
		{name: "reversed short interval", start: 0.02, end: 0.01},
		{name: "crosses top extreme", start: 0.2, end: 0.3},
		{name: "whole circle", start: 0, end: 1},
	} {
		t.Run(tc.name, func(t *testing.T) {
			circle := CircleSeg{
				Center: Point2{U: 3, V: 4}, Radius: units.Millimeters(6.875),
				TStart: tc.start, TEnd: tc.end,
			}
			u, v, ok := CircleBox(circle)
			require.True(t, ok)
			for i := range 101 {
				angle := 2 * math.Pi * (tc.start + (tc.end-tc.start)*float64(i)/100)
				x, y := 3+6.875*math.Cos(angle), 4+6.875*math.Sin(angle)
				require.LessOrEqual(t, proofbound.RatFloatDown(u.Lo)-1e-12, x)
				require.GreaterOrEqual(t, proofbound.RatFloatUp(u.Hi)+1e-12, x)
				require.LessOrEqual(t, proofbound.RatFloatDown(v.Lo)-1e-12, y)
				require.GreaterOrEqual(t, proofbound.RatFloatUp(v.Hi)+1e-12, y)
			}
			if tc.name == "short root interval" {
				require.Greater(t, proofbound.RatFloatDown(u.Lo), 9.0)
			}
		})
	}
}

func TestArcBoxEnclosesReversedQuarter(t *testing.T) {
	arc := ArcSeg{
		Center: Point2{U: 3, V: 4},
		Start:  Point2{U: 5, V: 4},
		End:    Point2{U: 3, V: 6},
		TStart: 1,
		TEnd:   0,
	}
	u, v, ok := ArcBox(arc)
	require.True(t, ok)
	for i := range 101 {
		angle := (math.Pi / 2) * float64(i) / 100
		x, y := 3+2*math.Cos(angle), 4+2*math.Sin(angle)
		require.LessOrEqual(t, proofbound.RatFloatDown(u.Lo)-1e-14, x)
		require.GreaterOrEqual(t, proofbound.RatFloatUp(u.Hi)+1e-14, x)
		require.LessOrEqual(t, proofbound.RatFloatDown(v.Lo)-1e-14, y)
		require.GreaterOrEqual(t, proofbound.RatFloatUp(v.Hi)+1e-14, y)
	}
	require.Less(t, proofbound.RatFloatUp(u.Hi)-proofbound.RatFloatDown(u.Lo), 2.1)
	require.Less(t, proofbound.RatFloatUp(v.Hi)-proofbound.RatFloatDown(v.Lo), 2.1)
}
