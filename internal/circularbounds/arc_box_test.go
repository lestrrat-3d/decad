package circularbounds

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

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
