package momentinput

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/stretchr/testify/require"
)

// TestEndsOfWalkCoversArcNaturalEnd holds an arc's natural end to the point
// the arc denotes there. The record pins End at a radius about 500 ulps past
// Start's, so the walk holds End verbatim at zero bound while the arc denotes
// (0, 10) — Start's radius at End's angle — and the end's bound must reach
// it. The t == 0 end is Start itself and stays at zero.
//
// Shown-to-fail: dropping the radial residual from endsOfWalk leaves the
// t == 1 bound at zero, and the coverage leg goes red.
func TestEndsOfWalkCoversArcNaturalEnd(t *testing.T) {
	t.Parallel()
	top := 10.0
	for range 500 {
		top = math.Nextafter(top, 11)
	}
	arc := ArcSeg{Center: Point2{}, Start: Point2{U: 10}, End: Point2{U: 0, V: top}, TStart: 0, TEnd: 1}
	walk, err := boundarywalk.WalkOf(arc, nil)
	require.NoError(t, err)
	ends := endsOfWalk(arc, walk)

	require.Equal(t, Point2{U: 0, V: top}, ends.end, `the walk holds the recorded End`)
	require.GreaterOrEqual(t, ends.endBound.V, top-10, `the bound reaches the denoted (0, 10)`)
	require.Equal(t, Point2{U: 10}, ends.start)
	require.Zero(t, ends.startBound.U)
	require.Zero(t, ends.startBound.V)
}
