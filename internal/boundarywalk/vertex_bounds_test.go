package boundarywalk

import (
	"math"
	"testing"

	"github.com/stretchr/testify/require"
)

// TestJunctionVertexReachesArcNaturalEnd places a junction vertex where
// an arc's natural end meets a line's natural start. The record pins the arc's
// End at a radius about 500 ulps past Start's, so both walks hold End verbatim
// at zero bound, while the arc denotes (0, 10) there: Start's radius at End's
// angle. The line denotes End itself, so the vertex must reach both points.
// Where the line starts the loop and the arc's t == 0 end closes it, both
// neighbours sit on one recorded coordinate and the vertex stays exact.
//
// Shown-to-fail: dropping the radial residual from arcNaturalEndBound leaves
// the junction at zero, and the first leg goes red.
func TestJunctionVertexReachesArcNaturalEnd(t *testing.T) {
	t.Parallel()
	top := 10.0
	for range 500 {
		top = math.Nextafter(top, 11)
	}
	arc := ArcSeg{Center: Point2{}, Start: Point2{U: 10}, End: Point2{U: 0, V: top}, TStart: 0, TEnd: 1}
	line := LineSeg{Start: Point2{U: 0, V: top}, End: Point2{U: 10}, TStart: 0, TEnd: 1}
	arcWalk, err := WalkOf(arc, nil)
	require.NoError(t, err)
	lineWalk, err := WalkOf(line, nil)
	require.NoError(t, err)
	require.Zero(t, arcWalk.EndBound.V, `the walk itself holds End verbatim`)

	u, v, b := JunctionVertex(arc, arcWalk, line, lineWalk)
	require.Equal(t, [2]float64{0, top}, [2]float64{u, v}, `the vertex sits at the line's held start`)
	require.GreaterOrEqual(t, b.V, top-10, `the bound reaches the denoted (0, 10)`)
	require.GreaterOrEqual(t, b.U, 0.0)

	_, _, closing := JunctionVertex(line, lineWalk, arc, arcWalk)
	require.Zero(t, closing.U, `two natural ends at one recorded coordinate are exact`)
	require.Zero(t, closing.V)
}
