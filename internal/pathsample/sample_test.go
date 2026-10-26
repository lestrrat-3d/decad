package pathsample_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/pathsample"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestSampleStraightLine(t *testing.T) {
	t.Parallel()
	got, err := pathsample.Sample(r3.Vec{}, []pathsample.Segment{{End: r3.Vec{X: 12}}}, 4)
	require.NoError(t, err)
	require.Equal(t, []r3.Vec{{}, {X: 4}, {X: 8}, {X: 12}}, got)
}

func TestSampleTwoLinesSpaceByLength(t *testing.T) {
	t.Parallel()
	// An L: 6 along X, then 6 along Y. Five points sit every 3 mm of length.
	segs := []pathsample.Segment{{End: r3.Vec{X: 6}}, {End: r3.Vec{X: 6, Y: 6}}}
	got, err := pathsample.Sample(r3.Vec{}, segs, 5)
	require.NoError(t, err)
	want := []r3.Vec{{}, {X: 3}, {X: 6}, {X: 6, Y: 3}, {X: 6, Y: 6}}
	for i := range want {
		require.True(t, got[i].Equal(want[i], 1e-12), "point %d is %v, want %v", i, got[i], want[i])
	}
}

func TestSampleQuarterArcCounterClockwise(t *testing.T) {
	t.Parallel()
	// A quarter of the radius-10 circle about the origin from (10,0) to (0,10).
	mid := 10 / math.Sqrt2
	segs := []pathsample.Segment{{Arc: true, Through: r3.Vec{X: mid, Y: mid}, End: r3.Vec{Y: 10}}}
	got, err := pathsample.Sample(r3.Vec{X: 10}, segs, 3)
	require.NoError(t, err)
	require.True(t, got[1].Equal(r3.Vec{X: mid, Y: mid}, 1e-9), "the middle point is at 45 degrees: %v", got[1])
	for _, p := range got {
		require.InDelta(t, 10, p.Len(), 1e-9, "every point stays on the circle")
	}
}

func TestSampleQuarterArcClockwise(t *testing.T) {
	t.Parallel()
	// The same circle run the other way, from (10,0) to (0,-10) below the axis.
	mid := 10 / math.Sqrt2
	segs := []pathsample.Segment{{Arc: true, Through: r3.Vec{X: mid, Y: -mid}, End: r3.Vec{Y: -10}}}
	got, err := pathsample.Sample(r3.Vec{X: 10}, segs, 3)
	require.NoError(t, err)
	require.True(t, got[1].Equal(r3.Vec{X: mid, Y: -mid}, 1e-9), "the middle point is at -45 degrees: %v", got[1])
}

func TestSampleLongArcPassesThroughItsMiddle(t *testing.T) {
	t.Parallel()
	// Three quarters of the circle, counter-clockwise from (10,0) through (-10,0).
	segs := []pathsample.Segment{{Arc: true, Through: r3.Vec{X: -10}, End: r3.Vec{Y: -10}}}
	got, err := pathsample.Sample(r3.Vec{X: 10}, segs, 4)
	require.NoError(t, err)
	// Four points divide the 270 degrees into three 90 degree steps.
	require.True(t, got[1].Equal(r3.Vec{Y: 10}, 1e-9), "%v", got[1])
	require.True(t, got[2].Equal(r3.Vec{X: -10}, 1e-9), "%v", got[2])
}

func TestSampleRefusesDegenerateSegments(t *testing.T) {
	t.Parallel()
	_, err := pathsample.Sample(r3.Vec{}, []pathsample.Segment{{End: r3.Vec{}}}, 3)
	require.ErrorIs(t, err, pathsample.ErrDegenerate, "a zero-length line")
	collinear := pathsample.Segment{Arc: true, Through: r3.Vec{X: 1}, End: r3.Vec{X: 2}}
	_, err = pathsample.Sample(r3.Vec{}, []pathsample.Segment{collinear}, 3)
	require.ErrorIs(t, err, pathsample.ErrDegenerate, "three points on a line have no circle")
}
