package rimoffset_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/rimoffset"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch/geom"
	"github.com/stretchr/testify/require"
)

// circleLoop records a circle of radius r as n fit splines of m+1 points each, counter-clockwise.
func circleLoop(r float64, n, m int) sectionrecord.LoopRecord {
	var loop sectionrecord.LoopRecord
	for s := range n {
		fit := make([]sectionrecord.Point2, m+1)
		for i := range fit {
			a := 2 * math.Pi * (float64(s) + float64(i)/float64(m)) / float64(n)
			fit[i] = sectionrecord.Point2{U: r * math.Cos(a), V: r * math.Sin(a)}
		}
		loop.Segments = append(loop.Segments, sectionrecord.FitSplineSeg{Fit: fit, TStart: 0, TEnd: 1})
	}
	return loop
}

// radii returns the distance from the origin of dense samples of every spline of segments.
func radii(t *testing.T, segments []sectionrecord.CurveSegment) (lo, hi float64) {
	t.Helper()
	lo, hi = math.Inf(1), 0
	for _, segment := range segments {
		seg := segment.(sectionrecord.FitSplineSeg)
		fit := make([][2]float64, len(seg.Fit))
		for i, q := range seg.Fit {
			fit[i] = [2]float64{q.U, q.V}
		}
		poly, err := geom.SampleFitSpline(fit, 400)
		require.NoError(t, err)
		for _, q := range poly {
			r := math.Hypot(q[0], q[1])
			lo, hi = math.Min(lo, r), math.Max(hi, r)
		}
	}
	return lo, hi
}

// A circle offset toward its material by d is the concentric circle of radius r-d, and away by d of radius r+d.
// The recorded splines sit up to a few microns off the ideal circle at their joints, so the offset is compared
// with the recorded splines' own radii.
func TestOffsetCircleLoop(t *testing.T) {
	t.Parallel()
	loop := circleLoop(10, 8, 8)
	lo0, hi0 := radii(t, loop.Segments)
	for _, dist := range []float64{0.5, 2, -0.25} {
		res, err := rimoffset.OffsetLoop(loop, dist, 1e-3*math.Abs(dist))
		require.NoError(t, err)
		lo, hi := radii(t, res.Segments)
		require.InDelta(t, lo0-dist, lo, 1e-3, "dist %v", dist)
		require.InDelta(t, hi0-dist, hi, 1e-3, "dist %v", dist)
		require.Less(t, res.Deviation, rimoffset.MaxDeviationRatio*math.Abs(dist))
	}
}

func TestOffsetRefusals(t *testing.T) {
	t.Parallel()
	loop := circleLoop(10, 8, 8)
	_, err := rimoffset.OffsetLoop(loop, 0, 1e-3)
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
	_, err = rimoffset.OffsetLoop(loop, 9.9, 1e-3)
	require.ErrorIs(t, err, decaderr.ErrDegenerate, "an offset that reaches the radius of curvature folds the curve")

	arc := sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.ArcSeg{TStart: 0, TEnd: 1}, sectionrecord.ArcSeg{TStart: 0, TEnd: 1},
	}}
	_, err = rimoffset.OffsetLoop(arc, 1, 1e-3)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)

	trimmed := circleLoop(10, 8, 8)
	seg := trimmed.Segments[0].(sectionrecord.FitSplineSeg)
	seg.TEnd = 0.5
	trimmed.Segments[0] = seg
	_, err = rimoffset.OffsetLoop(trimmed, 1, 1e-3)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)

	// A square with its corners turns 90° at each joint, which no smooth offset follows.
	var square sectionrecord.LoopRecord
	corners := []sectionrecord.Point2{{U: 0, V: 0}, {U: 10, V: 0}, {U: 10, V: 10}, {U: 0, V: 10}}
	for i, c := range corners {
		square.Segments = append(square.Segments, sectionrecord.LineSeg{
			Start: c, End: corners[(i+1)%4], TStart: 0, TEnd: 1})
	}
	_, err = rimoffset.OffsetLoop(square, 1, 1e-3)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "corner")
}
