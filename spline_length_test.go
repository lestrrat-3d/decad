package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/geom"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestFreeformArcLengthMatchesIndependentSplineSampling checks the public
// length result against sketch's independent B-spline evaluator. The control
// net has interior spans, whose Bézier conversion introduces non-dyadic
// denominators before the arc-length subdivision starts.
func TestFreeformArcLengthMatchesIndependentSplineSampling(t *testing.T) {
	t.Parallel()
	control := [][2]float64{
		{0, 0}, {1, 1.5}, {2, 2.4}, {3, 2.7}, {4, 2.4}, {5, 1.5}, {6, 0},
	}
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	points := make([]*sketch.Point, len(control))
	for i, p := range control {
		points[i] = s.CreatePoint(p[0], p[1])
	}
	_, err = s.CreateSpline(points...)
	require.NoError(t, err)
	s.CreateLine(points[len(points)-1], points[0])
	profiles := s.Profiles()
	require.Len(t, profiles, 1)

	d := decad.New()
	body, err := d.Extrude(s, profiles[0], decad.Distance{D: units.Millimeters(1), Dir: decad.Along})
	require.NoError(t, err)
	var splineEdges []*decad.Edge
	for _, edge := range body.Edges() {
		if _, ok := edge.Curve().(decad.NURBSCurve); ok {
			splineEdges = append(splineEdges, edge)
		}
	}
	require.Len(t, splineEdges, 2, "the free-form boundary has one rim on each cap")

	samples, err := geom.SampleCubicBSpline(control, 500000)
	require.NoError(t, err)
	dense := 0.0
	for i := 1; i < len(samples); i++ {
		dense += math.Hypot(samples[i][0]-samples[i-1][0], samples[i][1]-samples[i-1][1])
	}

	for _, edge := range splineEdges {
		length, err := edge.Length()
		require.NoError(t, err)
		require.Equal(t, decad.Approximate, length.Exactness)
		require.Greater(t, length.Bound.Mag(), 0.0)
		require.InDelta(t, dense, length.Value.Mag(), length.Bound.Mag()+2e-6,
			"the public interval agrees with independent dense evaluation of the same recorded curve")
	}
}
