package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/sketch/geom"
	"github.com/stretchr/testify/require"
)

// The conversion is only sound if the spans ARE the recorded curve. sketch's own
// evaluator is the falsifier: evaluate both at the same parameter and require
// agreement to machine precision. Any indexing, knot or basis mistake shows up
// here rather than as a quietly wrong area.

func evalSpans(t *testing.T, spans []bezierSpan, at float64) (float64, float64) {
	t.Helper()
	require.NotEmpty(t, spans)
	// Spans partition [0, 1] evenly in the converted parameter.
	scaled := at * float64(len(spans))
	index := int(math.Floor(scaled))
	if index >= len(spans) {
		index = len(spans) - 1
	}
	local := scaled - float64(index)
	span := spans[index]

	// de Casteljau over exact rationals.
	us := make([]*big.Rat, len(span))
	vs := make([]*big.Rat, len(span))
	for i, point := range span {
		us[i] = new(big.Rat).Set(point.u)
		vs[i] = new(big.Rat).Set(point.v)
	}
	tr := new(big.Rat).SetFloat64(local)
	oneMinus := new(big.Rat).Sub(big.NewRat(1, 1), tr)
	for round := len(span) - 1; round > 0; round-- {
		for i := range round {
			blend := func(values []*big.Rat) {
				lo := new(big.Rat).Mul(oneMinus, values[i])
				hi := new(big.Rat).Mul(tr, values[i+1])
				values[i] = lo.Add(lo, hi)
			}
			blend(us)
			blend(vs)
		}
	}
	u, _ := us[0].Float64()
	v, _ := vs[0].Float64()
	return u, v
}

func TestSplineBezierMatchesGeomEvaluator(t *testing.T) {
	control := []Point2{{U: 0, V: 0}, {U: 1, V: 2}, {U: 3, V: 2}, {U: 4, V: 0}, {U: 6, V: 1}, {U: 7, V: -2}}
	coords := make([][2]float64, len(control))
	for i, point := range control {
		coords[i] = [2]float64{point.U, point.V}
	}

	spans, err := splineBezierSpans(SplineSeg{Control: control, TStart: 0, TEnd: 1}, &freeformWork{})
	require.NoError(t, err)
	require.Len(t, spans, len(control)-3, "a clamped cubic over n controls has n-3 spans")

	for step := range 65 {
		at := float64(step) / 64
		wantU, wantV, err := geom.EvalCubicBSpline(coords, at)
		require.NoError(t, err)
		gotU, gotV := evalSpans(t, spans, at)
		require.InDelta(t, wantU, gotU, 1e-12, "u at t=%v", at)
		require.InDelta(t, wantV, gotV, 1e-12, "v at t=%v", at)
	}
}

func TestClosedSplineBezierMatchesGeomEvaluator(t *testing.T) {
	control := []Point2{{U: 0, V: 0}, {U: 4, V: 0}, {U: 5, V: 3}, {U: 2, V: 5}, {U: -1, V: 3}}
	coords := make([][2]float64, len(control))
	for i, point := range control {
		coords[i] = [2]float64{point.U, point.V}
	}

	spans, err := closedSplineBezierSpans(
		ClosedSplineSeg{Control: control, CCW: true, TStart: 0, TEnd: 1},
		&freeformWork{},
	)
	require.NoError(t, err)
	require.Len(t, spans, len(control), "a periodic cubic over n controls has n spans")

	for step := range 64 {
		at := float64(step) / 64
		wantU, wantV, err := geom.EvalPeriodicCubicBSpline(coords, at)
		require.NoError(t, err)
		gotU, gotV := evalSpans(t, spans, at)
		require.InDelta(t, wantU, gotU, 1e-12, "u at t=%v", at)
		require.InDelta(t, wantV, gotV, 1e-12, "v at t=%v", at)
	}
}

func TestNURBSBezierMatchesGeomEvaluator(t *testing.T) {
	control := []Point2{{U: 0, V: 0}, {U: 1, V: 3}, {U: 4, V: 3}, {U: 5, V: 0}, {U: 8, V: 2}}
	coords := make([]*geom.Point, len(control))
	for i, point := range control {
		coords[i] = geom.NewPoint(point.U, point.V)
	}
	// A clamped uniform degree-3 knot vector over 5 control points, and unit
	// weights: non-rational, so Tier A.
	knots := geom.ClampedUniformKnots(len(control), 3)
	weights := []float64{1, 1, 1, 1, 1}
	curve := geom.NewNURBS(3, coords, knots, weights)

	spans, err := nurbsBezierSpans(NURBSSeg{
		Degree:  3,
		Control: control,
		Knots:   knots,
		Weights: weights,
		TStart:  0,
		TEnd:    1,
	}, &freeformWork{})
	require.NoError(t, err)

	lo, hi := curve.Domain()
	for step := range 65 {
		at := float64(step) / 64
		wantU, wantV := curve.Eval(lo + (hi-lo)*at)
		gotU, gotV := evalSpans(t, spans, at)
		require.InDelta(t, wantU, gotU, 1e-12, "u at t=%v", at)
		require.InDelta(t, wantV, gotV, 1e-12, "v at t=%v", at)
	}
}

func TestFreeformBezierSpansRefusals(t *testing.T) {
	for _, tc := range []struct {
		name    string
		segment CurveSegment
		message string
	}{
		{
			name:    "fit spline",
			segment: FitSplineSeg{Fit: []Point2{{}, {U: 1}}, TStart: 0, TEnd: 1},
			message: "interpolation solve",
		},
		{
			name: "elliptical arc",
			segment: EllipticalArcSeg{
				Center: Point2{}, Start: Point2{U: 1}, End: Point2{V: 1},
				TStart: 0, TEnd: 1,
			},
			message: "pinned endpoints",
		},
		{
			name:    "rational NURBS",
			segment: rationalNURBSFixture(),
			message: "rational NURBS",
		},
		{
			name: "trimmed spline",
			segment: SplineSeg{
				Control: []Point2{{}, {U: 1, V: 1}, {U: 2, V: 1}, {U: 3}},
				TStart:  0.25, TEnd: 0.75,
			},
			message: "full domain",
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			_, _, err := freeformBezierSpans(tc.segment, &freeformWork{})
			require.Error(t, err)
			require.ErrorIs(t, err, ErrUnsupported)
			require.Contains(t, err.Error(), tc.message)
		})
	}
}

func rationalNURBSFixture() NURBSSeg {
	control := []Point2{{U: 0, V: 0}, {U: 1, V: 2}, {U: 3, V: 2}, {U: 4, V: 0}}
	return NURBSSeg{
		Degree:  3,
		Control: control,
		Knots:   []float64{0, 0, 0, 0, 1, 1, 1, 1},
		Weights: []float64{1, 2, 1, 1},
		TStart:  0,
		TEnd:    1,
	}
}

func TestFreeformWorkLimitRefuses(t *testing.T) {
	work := &freeformWork{spent: freeformWorkLimit - 1}
	err := work.step(4)
	require.Error(t, err)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "work budget")
}
