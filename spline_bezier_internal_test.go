package decad

import (
	"math"
	"math/big"
	"testing"
	"time"

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

// The charge a conversion levies must grow with the work it actually does. A
// clamped degree-3 B-spline inserts twice per interior knot and each insertion
// scans and copies both whole vectors, so the cost is QUADRATIC in the control
// count — a charge that only counted insertions would let a hundred-thousand
// control record run for hours inside the ceiling.
func TestClampedConversionCostIsQuadratic(t *testing.T) {
	cost := func(controls int) uint64 {
		return clampedConversionCost(controls, controls+4, uniformKnotDemand(controls, 3))
	}
	small, doubled := cost(100), cost(200)
	require.Less(t, small, freeformWorkLimit, "a 100-control cubic stays inside the ceiling")
	require.Greater(t, doubled, 3*small, "doubling the controls more than triples the charge")
	require.Equal(t, freeformCostCeiling, cost(100000), "the finding's shape saturates over budget")
	require.Equal(t, freeformCostCeiling, cost(2000), "a quadratic cost the old charge admitted now refuses")

	// A degree-1 record owes no insertion — every run is already at degree — so
	// the whole charge is the terminating probe each target still pays for.
	var probesOnly knotInsertionDemand
	for range 2998 {
		probesOnly.add(1, 1)
	}
	require.Zero(t, probesOnly.insertions, "a degree-1 target is already at its own degree")
	require.Equal(t, freeformCostCeiling, clampedConversionCost(3000, 3002, probesOnly),
		"probes that insert nothing are charged")
}

// The demand a conversion is charged from must be readable WITHOUT lifting a
// knot into a rational, because the charge has to clear before the lift
// allocates. The float scan and the restated uniform vector must therefore agree
// with the rational walk the insertion pass itself runs.
func TestKnotInsertionDemandMatchesRationalWalk(t *testing.T) {
	rationalDemand := func(degree, n int, knots []*big.Rat) knotInsertionDemand {
		_, runs, _ := interiorKnotRuns(degree, n, knots)
		var demand knotInsertionDemand
		for _, run := range runs {
			demand.add(degree, run)
		}
		return demand
	}

	for _, controls := range []int{4, 5, 9, 40} {
		knots := clampedUniformKnots(controls, 3)
		require.Equal(t,
			rationalDemand(3, controls, knots),
			uniformKnotDemand(controls, 3),
			"restated uniform demand at %d controls", controls)
	}

	// A degree-2 vector mixing a simple interior knot with a doubled one.
	floats := []float64{0, 0, 0, 0.25, 0.5, 0.5, 1, 1, 1}
	rats := make([]*big.Rat, len(floats))
	for i, value := range floats {
		rats[i] = new(big.Rat).SetFloat64(value)
	}
	require.Equal(t, rationalDemand(2, 6, rats), floatKnotDemand(2, 6, floats))
}

// The stride-degree slicing rests on consecutive spans SHARING their boundary
// control point, and the divisibility test that used to stand in for that
// precondition is satisfied by inputs that break it: a cubic whose four interior
// knots each sit at multiplicity 4 needs no insertion at all, holds 16 control
// points, and 15 is divisible by 3 — so the slicer would cut five spans across
// the four pieces the record states and quietly round a corner.
//
// The SENTINEL that refusal carries is decided by the curve, not by the slicer.
// At multiplicity degree+1 the two one-sided limits are two recorded control
// points; identical, and the curve is continuous and the body exists, so this
// evaluator's inability to slice it is ErrUnsupported; different, and the curve
// really does break apart, so no such body exists and it is ErrDegenerate.
func TestBezierSliceCountSplitsBrokenFromUnsliceable(t *testing.T) {
	third := 1.0 / 3
	squareControls := func(joint Point2) []Point2 {
		return []Point2{
			{U: 0, V: 0}, {U: third, V: 0}, {U: 2 * third, V: 0}, {U: 1, V: 0},
			joint, {U: 1, V: third}, {U: 1, V: 2 * third}, {U: 1, V: 1},
			{U: 1, V: 1}, {U: 2 * third, V: 1}, {U: third, V: 1}, {U: 0, V: 1},
			{U: 0, V: 1}, {U: 0, V: 2 * third}, {U: 0, V: third}, {U: 0, V: 0},
		}
	}
	quarterKnots := func() []*big.Rat {
		knots := make([]*big.Rat, 0, 20)
		for _, value := range []int64{0, 1, 2, 3, 4} {
			for range 4 {
				knots = append(knots, big.NewRat(value, 4))
			}
		}
		return knots
	}

	t.Run("continuous", func(t *testing.T) {
		// The two one-sided limits at every break are the same recorded point, so
		// the four cubic pieces meet: one connected curve this slicer cannot cut.
		ctrl, err := ratPointsOf(squareControls(Point2{U: 1, V: 0}))
		require.NoError(t, err)
		knots := quarterKnots()
		require.Len(t, knots, len(ctrl)+3+1)

		_, err = clampedBezierSpans(3, ctrl, knots)
		require.Error(t, err)
		require.ErrorIs(t, err, ErrUnsupported)
		require.Contains(t, err.Error(), "share no boundary control point")
	})

	t.Run("discontinuous", func(t *testing.T) {
		// Move the first break's right-hand limit away from its left-hand one and
		// the curve genuinely jumps there.
		ctrl, err := ratPointsOf(squareControls(Point2{U: 1.5, V: 0}))
		require.NoError(t, err)

		_, err = clampedBezierSpans(3, ctrl, quarterKnots())
		require.Error(t, err)
		require.ErrorIs(t, err, ErrDegenerate)
		require.Contains(t, err.Error(), "disjoint pieces")
	})

	t.Run("over-clamped end", func(t *testing.T) {
		// A degree-2 vector clamped one repeat too far at the start: a single
		// quadratic Bézier with one dead control point, continuous everywhere, but
		// 3 control points do not stride into whole degree-2 spans.
		ctrl, err := ratPointsOf([]Point2{{U: 0, V: 0}, {U: 0, V: 0}, {U: 1, V: 2}, {U: 2, V: 0}})
		require.NoError(t, err)
		knots := []*big.Rat{
			new(big.Rat), new(big.Rat), new(big.Rat), new(big.Rat),
			big.NewRat(1, 1), big.NewRat(1, 1), big.NewRat(1, 1),
		}
		require.Len(t, knots, len(ctrl)+2+1)

		_, err = clampedBezierSpans(2, ctrl, knots)
		require.Error(t, err)
		require.ErrorIs(t, err, ErrUnsupported)
		require.Contains(t, err.Error(), "whole number of degree-2 Bézier spans")
	})
}

// The conversion budget must charge every knotMultiplicity PROBE, not only the
// probes that go on to insert. A degree-1 record with thousands of distinct
// interior knots owes no insertion at all — each target is already at degree
// multiplicity — yet the loop still scans the whole knot vector once per target,
// which is quadratic work a charge counting insertions alone reads as nothing.
func TestUnchargedKnotProbesRefuse(t *testing.T) {
	const controls = 3000
	control := make([]Point2, controls)
	weights := make([]float64, controls)
	for i := range control {
		control[i] = Point2{U: float64(i), V: float64(i % 3)}
		weights[i] = 1
	}
	interior := controls - 2
	knots := []float64{0, 0}
	for j := 1; j <= interior; j++ {
		knots = append(knots, float64(j)/float64(interior+1))
	}
	knots = append(knots, 1, 1)
	seg := NURBSSeg{Degree: 1, Control: control, Knots: knots, Weights: weights, TStart: 0, TEnd: 1}
	require.NoError(t, validateNURBSSegment(seg), "the record itself is well formed")

	start := time.Now()
	_, _, err := freeformBezierSpans(seg, &freeformWork{})
	require.Error(t, err)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "work budget")
	require.Less(t, time.Since(start), 2*time.Second, "the refusal precedes the probe pass")
}

// The integration cost is CUBIC in span degree while the span is only degree+1
// control points long, so a charge read off the control count alone admits an
// arbitrarily wide span. The finding's degree-1024 span must be over budget.
func TestFreeformSpanCostIsCubic(t *testing.T) {
	require.Less(t, freeformSpanCost(4), freeformWorkLimit, "a cubic Bézier span is cheap")
	require.Less(t, freeformSpanCost(2), freeformSpanCost(4))
	require.Greater(t, freeformSpanCost(65), 20*freeformSpanCost(17),
		"quadrupling the degree raises the charge by more than its square")
	require.Equal(t, freeformCostCeiling, freeformSpanCost(1025), "the finding's degree-1024 span")
	require.Equal(t, freeformCostCeiling, freeformSpanCost(1<<20), "an absurd degree saturates, never wraps")
	require.Less(t, uint64(8*1025), freeformWorkLimit,
		"a charge proportional to the span length is what let that degree through")
}

// The measured defect: a validator-accepted degree-1024 single-span NURBS
// integrated for over seven minutes and returned success. The record-level
// preflight — which owns every free-form charge — must refuse it before a single
// Bernstein coefficient is expanded.
func TestWideSpanIntegrationRefusesBeforeExpanding(t *testing.T) {
	const degree = 1024
	control := make([]Point2, degree+1)
	knots := make([]float64, 0, 2*(degree+1))
	weights := make([]float64, degree+1)
	for i := range control {
		control[i] = Point2{U: float64(i), V: float64(i % 7)}
		weights[i] = 1
	}
	for range degree + 1 {
		knots = append(knots, 0)
	}
	for range degree + 1 {
		knots = append(knots, 1)
	}
	seg := NURBSSeg{Degree: degree, Control: control, Knots: knots, Weights: weights, TStart: 0, TEnd: 1}
	require.NoError(t, validateNURBSSegment(seg), "the record itself is well formed")

	spans, _, err := freeformBezierSpans(seg, &freeformWork{})
	require.NoError(t, err, "a single span needs no knot insertion")
	require.Len(t, spans, 1)

	start := time.Now()
	checked, anchor, plan, err := validateFreeformMomentSegment(seg, &freeformWork{})
	require.Error(t, err)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "work budget")
	require.Less(t, time.Since(start), 10*time.Second, "the refusal precedes the expansion")
	require.Nil(t, checked, "a refused segment is not admitted")
	require.Zero(t, anchor)
	require.Nil(t, plan.spans, "no chain is carried into the moments pass")
}
