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

func TestSampleStationsLineAndArcTangents(t *testing.T) {
	t.Parallel()
	// 10 along X, then a quarter circle of radius 10 about (10, 10) from (10, 0)
	// to (20, 10), turning counter-clockwise.
	mid := 10 / math.Sqrt2
	segs := []pathsample.Segment{
		{End: r3.Vec{X: 10}},
		{Arc: true, Through: r3.Vec{X: 10 + mid, Y: 10 - mid}, End: r3.Vec{X: 20, Y: 10}},
	}
	// Total length is 10 + 5π; stations sit at 0, 10 (the join) and the end.
	got, err := pathsample.SampleStations(r3.Vec{}, segs, 3)
	require.NoError(t, err)
	require.True(t, got[0].Tangent.Equal(r3.Vec{X: 1}, 1e-12), "start runs along +X: %v", got[0].Tangent)
	require.True(t, got[2].Tangent.Equal(r3.Vec{Y: 1}, 1e-9), "the arc ends running along +Y: %v", got[2].Tangent)
	for i, st := range got {
		require.InDelta(t, 1, st.Tangent.Len(), 1e-12, "tangent %d is a unit vector", i)
	}
}

func TestTurn(t *testing.T) {
	t.Parallel()
	axis, angle, err := pathsample.Turn(r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	require.True(t, axis.Equal(r3.Vec{Z: 1}, 1e-12))
	require.InDelta(t, math.Pi/2, angle, 1e-12)

	_, angle, err = pathsample.Turn(r3.Vec{X: 1}, r3.Vec{X: 1})
	require.NoError(t, err)
	require.Zero(t, angle)

	_, _, err = pathsample.Turn(r3.Vec{X: 1}, r3.Vec{X: -1})
	require.ErrorIs(t, err, pathsample.ErrDegenerate)
}

func TestSampleSpacedStraightLine(t *testing.T) {
	t.Parallel()
	got, err := pathsample.SampleSpaced(r3.Vec{}, []pathsample.Segment{{End: r3.Vec{X: 10}}}, 3)
	require.NoError(t, err)
	require.Len(t, got, 4, "points at 0, 3, 6 and 9; 12 lies past the end")
	for i, st := range got {
		require.True(t, st.Point.Equal(r3.Vec{X: 3 * float64(i)}, 1e-12), "point %d is %v", i, st.Point)
	}
}

func TestSampleSpacedPlacesAPointAtTheEndWhenItDivides(t *testing.T) {
	t.Parallel()
	segs := []pathsample.Segment{{End: r3.Vec{X: 0.3}}}
	got, err := pathsample.SampleSpaced(r3.Vec{}, segs, 0.1)
	require.NoError(t, err)
	require.Len(t, got, 4, "0.3 / 0.1 is 2.9999999999999996 in floating point, and the end still gets its point")
	require.Equal(t, r3.Vec{X: 0.3}, got[3].Point)
}

func TestSampleSpacedWalksAcrossSegments(t *testing.T) {
	t.Parallel()
	// An L of 6 + 6 with a spacing of 4: points at 0, 4 and 8 mm of length.
	segs := []pathsample.Segment{{End: r3.Vec{X: 6}}, {End: r3.Vec{X: 6, Y: 6}}}
	got, err := pathsample.SampleSpaced(r3.Vec{}, segs, 4)
	require.NoError(t, err)
	require.Len(t, got, 4)
	want := []r3.Vec{{}, {X: 4}, {X: 6, Y: 2}, {X: 6, Y: 6}}
	// 12 / 4 is exact, so the last point is the end.
	for i := range want {
		require.True(t, got[i].Point.Equal(want[i], 1e-12), "point %d is %v, want %v", i, got[i].Point, want[i])
	}
	require.True(t, got[2].Tangent.Equal(r3.Vec{Y: 1}, 1e-12))
}

func TestSampleSpacedRefusals(t *testing.T) {
	t.Parallel()
	segs := []pathsample.Segment{{End: r3.Vec{X: 10}}}
	for _, spacing := range []float64{0, -1, math.NaN(), math.Inf(1)} {
		_, err := pathsample.SampleSpaced(r3.Vec{}, segs, spacing)
		require.ErrorIs(t, err, pathsample.ErrDegenerate, "spacing %v", spacing)
	}
	_, err := pathsample.SampleSpaced(r3.Vec{}, segs, 1e-9)
	require.ErrorIs(t, err, pathsample.ErrTooMany)
}
