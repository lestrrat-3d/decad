package decad

import (
	"math"
	"math/big"
	"runtime"
	"strconv"
	"testing"
	"time"

	"github.com/lestrrat-3d/sketch/geom"
	"github.com/lestrrat-3d/units"
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

type floatBezierSpan [][2]float64

type floatBezierSpans []floatBezierSpan

func floatBezierSpanOf(span bezierSpan) floatBezierSpan {
	got := make(floatBezierSpan, len(span))
	for i, point := range span {
		got[i][0], _ = point.u.Float64()
		got[i][1], _ = point.v.Float64()
	}
	return got
}

func floatBezierSpansOf(spans []bezierSpan) floatBezierSpans {
	got := make(floatBezierSpans, len(spans))
	for i, span := range spans {
		got[i] = floatBezierSpanOf(span)
	}
	return got
}

// evalFloatBezierSpan is the dense-sampling oracle for tests whose sample
// count is large. The exact-rational oracle above remains the conversion
// check; this one avoids creating a large denominator for every sample.
func evalFloatBezierSpan(span floatBezierSpan, at float64) (float64, float64) {
	u := make([]float64, len(span))
	v := make([]float64, len(span))
	for i, point := range span {
		u[i], v[i] = point[0], point[1]
	}
	oneMinus := 1 - at
	for round := len(span) - 1; round > 0; round-- {
		for i := range round {
			u[i] = oneMinus*u[i] + at*u[i+1]
			v[i] = oneMinus*v[i] + at*v[i+1]
		}
	}
	return u[0], v[0]
}

func evalFloatBezierSpans(spans floatBezierSpans, at float64) (float64, float64) {
	scaled := at * float64(len(spans))
	index := int(math.Floor(scaled))
	if index >= len(spans) {
		index = len(spans) - 1
	}
	return evalFloatBezierSpan(spans[index], scaled-float64(index))
}

func TestSplineBezierMatchesGeomEvaluator(t *testing.T) {
	t.Parallel()
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

// evalSpanExact is de Casteljau over exact rationals on ONE span, at a rational
// local parameter. It rounds nothing, so its result is comparable by Cmp.
func evalSpanExact(span bezierSpan, at *big.Rat) (*big.Rat, *big.Rat) {
	us := make([]*big.Rat, len(span))
	vs := make([]*big.Rat, len(span))
	for i, point := range span {
		us[i] = new(big.Rat).Set(point.u)
		vs[i] = new(big.Rat).Set(point.v)
	}
	oneMinus := new(big.Rat).Sub(big.NewRat(1, 1), at)
	for round := len(span) - 1; round > 0; round-- {
		for i := range round {
			blend := func(values []*big.Rat) {
				lo := new(big.Rat).Mul(oneMinus, values[i])
				hi := new(big.Rat).Mul(at, values[i+1])
				values[i] = lo.Add(lo, hi)
			}
			blend(us)
			blend(vs)
		}
	}
	return us[0], vs[0]
}

// splineBasisRat is Cox–de Boor N_{i,p}(t) over exact rationals with the
// 0/0 = 0 convention — the same recursion geom's own bsplineBasis runs, evaluated
// without rounding so it can serve as an exact reference.
func splineBasisRat(i, p int, at *big.Rat, knots []*big.Rat) *big.Rat {
	if p == 0 {
		if knots[i].Cmp(at) <= 0 && at.Cmp(knots[i+1]) < 0 {
			return big.NewRat(1, 1)
		}
		return new(big.Rat)
	}
	sum := new(big.Rat)
	if d := new(big.Rat).Sub(knots[i+p], knots[i]); d.Sign() > 0 {
		weight := new(big.Rat).Quo(new(big.Rat).Sub(at, knots[i]), d)
		sum.Add(sum, new(big.Rat).Mul(weight, splineBasisRat(i, p-1, at, knots)))
	}
	if d := new(big.Rat).Sub(knots[i+p+1], knots[i+1]); d.Sign() > 0 {
		weight := new(big.Rat).Quo(new(big.Rat).Sub(knots[i+p+1], at), d)
		sum.Add(sum, new(big.Rat).Mul(weight, splineBasisRat(i+1, p-1, at, knots)))
	}
	return sum
}

// coxDeBoorExact evaluates the clamped cubic B-spline over the given control
// points and knot vector exactly, as the literal Σ N_{i,3}(t)·P_i.
func coxDeBoorExact(control []Point2, knots []*big.Rat, at *big.Rat) (*big.Rat, *big.Rat) {
	u, v := new(big.Rat), new(big.Rat)
	for i, point := range control {
		basis := splineBasisRat(i, 3, at, knots)
		if basis.Sign() == 0 {
			continue
		}
		u.Add(u, new(big.Rat).Mul(basis, mustRatOf(point.U)))
		v.Add(v, new(big.Rat).Mul(basis, mustRatOf(point.V)))
	}
	return u, v
}

// The converted spans must be the curve SKETCH defines, and the knot vector is
// where the two can silently diverge: geom.ClampedKnots builds each interior knot
// as float64(j)/float64(n−3), so a converter that rebuilds them as the exact
// rationals j/(n−3) integrates a different curve whenever n−3 is not a power of
// two — and does so with no rounding anywhere, which is what let it publish a
// false zero bound.
//
// This test is exact and therefore a proof, not a tolerance: it compares each
// converted span against Cox–de Boor over sketch's own FLOAT knots at rational
// parameters, and requires bit-for-bit rational equality. A re-derivation of the
// knots fails it loudly. The control counts are both AFFECTED ones — 6 and 9,
// where n−3 is 3 and 6 — since the four- and five-control fixtures elsewhere have
// n−3 a power of two and cannot see the difference.
func TestSplineBezierSpansUseSketchFloatKnots(t *testing.T) {
	t.Parallel()
	// The offending values, stated once: 1/3 and the float geom actually holds.
	require.NotEqual(t, 0, clampedUniformKnots(6)[4].Cmp(big.NewRat(1, 3)),
		"geom's interior knot is the rounding of 1/3, not 1/3")
	require.Equal(t, 0, clampedUniformKnots(6)[4].Cmp(mustRatOf(geom.ClampedKnots(6)[4])),
		"the lifted vector is geom's own float, taken exactly")

	for _, controls := range []int{6, 9} {
		t.Run(strconv.Itoa(controls), func(t *testing.T) {
			control := make([]Point2, controls)
			for i := range control {
				control[i] = Point2{U: float64(i), V: float64((i * 7) % 5)}
			}
			// The reference reads geom DIRECTLY rather than through the converter's
			// own helper, so the comparison below is a statement about sketch's knot
			// values and not a self-consistency check.
			knots := make([]*big.Rat, controls+4)
			for i, knot := range geom.ClampedKnots(controls) {
				knots[i] = mustRatOf(knot)
			}

			spans, err := splineBezierSpans(SplineSeg{Control: control, TStart: 0, TEnd: 1}, &freeformWork{})
			require.NoError(t, err)
			require.Len(t, spans, controls-3)

			for index, span := range spans {
				lo, hi := knots[3+index], knots[4+index]
				width := new(big.Rat).Sub(hi, lo)
				// The half-open basis convention leaves t = hi to the next span, and
				// the chain's far endpoint is covered by the join at every other span.
				for _, numerator := range []int64{0, 1, 2, 3, 4} {
					local := big.NewRat(numerator, 5)
					global := new(big.Rat).Add(lo, new(big.Rat).Mul(local, width))
					wantU, wantV := coxDeBoorExact(control, knots, global)
					gotU, gotV := evalSpanExact(span, local)
					require.Equal(t, 0, gotU.Cmp(wantU),
						"span %d u at local %v", index, local)
					require.Equal(t, 0, gotV.Cmp(wantV),
						"span %d v at local %v", index, local)
				}
			}
		})
	}
}

func TestClosedSplineBezierMatchesGeomEvaluator(t *testing.T) {
	t.Parallel()
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
	t.Parallel()
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
	t.Parallel()
	for _, tc := range []struct {
		name    string
		segment CurveSegment
		message string
	}{
		{
			name: "trimmed fit spline",
			segment: FitSplineSeg{
				Fit:    []Point2{{}, {U: 1, V: 1}, {U: 2}},
				TStart: 0.25, TEnd: 0.75,
			},
			message: "full domain",
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
	t.Parallel()
	work := &freeformWork{spent: freeformWorkLimit - 1}
	err := work.step(4)
	require.Error(t, err)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "work budget")
}

// Whether a NURBS segment is Tier A at all is decided by its recorded weights, so
// a rational one owes its OWN Table R reason — and it gets that reason for every
// record whose SIZE fits the ceiling, which is every record that could ever yield
// a measurement. What it does NOT get is precedence over the size-derived lift
// charge: deciding the tier reads all n weights, so it is inherently linear, and a
// scan placed ahead of every charge is unbounded and uncancellable. So the two
// halves are asserted separately.
func TestRationalNURBSReasonPrecedesTheConversionCharge(t *testing.T) {
	t.Parallel()
	// The fixture's lift charge is 2 controls + knots + weights = 20 units, and its
	// conversion charge is a further 8. Leaving room for exactly the lift proves
	// the tier is read before the conversion charge and not merely when the
	// counter is empty.
	const liftCost = 2*4 + 8 + 4
	work := &freeformWork{spent: freeformWorkLimit - liftCost}
	_, _, err := freeformBezierSpans(rationalNURBSFixture(), work)
	require.Error(t, err)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "rational NURBS")
	require.NotContains(t, err.Error(), "work budget",
		"a record whose size fits the ceiling reads its own tier reason")

	// The stated cost of the bound: a record the lift charge alone cannot afford
	// reports R7 instead. Such a record is refused either way, so R7 is equally
	// true of it.
	exhausted := &freeformWork{spent: freeformWorkLimit}
	_, _, err = freeformBezierSpans(rationalNURBSFixture(), exhausted)
	require.Error(t, err)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "work budget")
}

// maxDegreeOneNURBSControls is the largest degree-1 NURBS control count the
// size-derived lift charge admits. Such a record holds n control points, n+2
// knots and n weights, so it charges 2n + (n+2) + n = 4n+2 units, and 4n+2 fits
// 2^20 exactly up to this count. It is exact integer arithmetic, so the boundary
// does not move from machine to machine.
const maxDegreeOneNURBSControls = 262143

// degreeOneNURBSWithTrailingNaN carries its non-finite element as far from the
// start as a record can: the LAST interior knot, so the whole control array and
// almost the whole knot vector are walked before the content scan can refuse it.
func degreeOneNURBSWithTrailingNaN(controls int) NURBSSeg {
	seg := wellFormedDegreeOneNURBS(controls)
	seg.Knots[len(seg.Knots)-3] = math.NaN()
	return seg
}

// wellFormedDegreeOneNURBS is the same record with every element valid and every
// weight equal, so it clears the content scan and the tier test and is refused
// only by the conversion charge — the path on which every preflight pass runs.
func wellFormedDegreeOneNURBS(controls int) NURBSSeg {
	control := make([]Point2, controls)
	weights := make([]float64, controls)
	for i := range control {
		control[i] = Point2{U: float64(i), V: float64(i % 3)}
		weights[i] = 1
	}
	interior := controls - 2
	knots := make([]float64, 0, controls+2)
	knots = append(knots, 0, 0)
	for j := 1; j <= interior; j++ {
		knots = append(knots, float64(j)/float64(interior+1))
	}
	knots = append(knots, 1, 1)
	return NURBSSeg{Degree: 1, Control: control, Knots: knots, Weights: weights, TStart: 0, TEnd: 1}
}

// The size-derived lift charge must be levied BEFORE the per-element content
// scan, because that scan is linear in a length the caller chose and nothing
// downstream can cancel it: a well-formed degree-1 record took 213.7 ms at
// 1,048,576 control points and 1.672 s at 8,000,000, each time running to
// completion and only then refusing.
//
// The proof is mechanical rather than timed. Two records one control point apart
// straddle the ceiling, and each carries a non-finite knot at the very end of
// its vector. The smaller one still reports that content error, so the scan runs
// when the charge admits it; the larger reports R7, which it can only do by
// refusing before reading the knot.
func TestNURBSLiftChargePrecedesTheContentScan(t *testing.T) {
	const admitted = maxDegreeOneNURBSControls
	require.LessOrEqual(t, rationalLiftCost(admitted, admitted+2, admitted), freeformWorkLimit,
		"%d controls are the most the lift charge admits", admitted)
	require.Greater(t, rationalLiftCost(admitted+1, admitted+3, admitted+1), freeformWorkLimit,
		"one more control point does not fit")

	start := time.Now()
	_, _, err := freeformBezierSpans(degreeOneNURBSWithTrailingNaN(admitted), &freeformWork{})
	require.Error(t, err)
	require.ErrorIs(t, err, ErrNotFinite)
	require.Contains(t, err.Error(), "must be finite",
		"the largest scan the charge admits still runs and still reports its own content error")
	require.Less(t, time.Since(start), time.Second,
		"and the charge is what bounds how long that scan can be")

	_, _, err = freeformBezierSpans(degreeOneNURBSWithTrailingNaN(admitted+1), &freeformWork{})
	require.Error(t, err)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "work budget",
		"one control point past the ceiling, nothing scans the knot vector at all")
}

// costOf reports the wall-clock, allocation count and allocated bytes of one
// call. The counts are deltas across the call, so the record the call reads must
// already be built when it starts.
func costOf(call func()) (time.Duration, uint64, uint64) {
	runtime.GC()
	var before, after runtime.MemStats
	runtime.ReadMemStats(&before)
	start := time.Now()
	call()
	elapsed := time.Since(start)
	runtime.ReadMemStats(&after)
	return elapsed, after.Mallocs - before.Mallocs, after.TotalAlloc - before.TotalAlloc
}

// This test stays SERIAL: it measures process-wide allocation, which any
// test running alongside it would inflate. Adding t.Parallel here makes its
// reading meaningless rather than making it fail loudly.
// What chargeRationalLift's invariant claims — that the preflight's element
// visits are a fixed multiple of the units it charges — is backed HERE, by
// MEASURED cost at the admission boundary, and deliberately not by a per-pass
// accounting identity. An identity has to be restated every time a validator is
// added, and it goes stale silently; a boundary on wall-clock and allocation
// fails whenever an added pass is unbounded or allocates per element, whatever
// the accounting says.
//
// The record measured is the WORST the charge admits and it is well formed, so
// its refusal is the conversion charge's and every pass the invariant covers has
// run first. It costs about 1.9 ms, 3.9 kB and 9 allocations here, against about
// 5 us for the first record past the boundary. The limits below sit far above
// those readings on purpose: what they exist to catch — an uncharged pass over an
// array the charge does not count, or one allocation per element — misses them by
// orders of magnitude, while ordinary machine-to-machine variation does not come
// near them.
//
// This test stays SERIAL: it measures process-wide allocation, which any
// test running alongside it would inflate. Adding t.Parallel here makes its
// reading meaningless rather than making it fail loudly.
func TestFreeformPreflightBoundaryCost(t *testing.T) {
	const admitted = maxDegreeOneNURBSControls
	worst := wellFormedDegreeOneNURBS(admitted)
	past := wellFormedDegreeOneNURBS(admitted + 1)

	var err error
	elapsed, mallocs, bytes := costOf(func() {
		_, _, err = freeformBezierSpans(worst, &freeformWork{})
	})
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "work budget",
		"the worst admitted record clears every content pass and is refused by the conversion charge")
	require.Less(t, elapsed, 500*time.Millisecond,
		"the worst record the ceiling admits costs milliseconds, so the ceiling bounds a real cost")
	require.Less(t, mallocs, uint64(1000),
		"the preflight allocates per refusal, never per element")
	require.Less(t, bytes, uint64(1<<20),
		"and it lifts no array of the record's own size")

	elapsed, mallocs, _ = costOf(func() {
		_, _, err = freeformBezierSpans(past, &freeformWork{})
	})
	require.ErrorIs(t, err, ErrUnsupported)
	require.Less(t, elapsed, 50*time.Millisecond,
		"one control point past the boundary, no pass over the record runs at all")
	require.Less(t, mallocs, uint64(100))
}

// The chord counts the reconstruction charge is built on are sketch's own
// (geom/arrange.go sampleParams), restated here because the charge has to be
// levied before that sampler runs. They are asserted per kind because each row
// is a separate fact: the FLOOR is what makes a record of tiny curves expensive,
// and the open spline is the one free-form kind sampled per span rather than per
// control point.
func TestReconstructionChordsRestateSketchSampling(t *testing.T) {
	t.Parallel()
	point := func(u, v float64) Point2 { return Point2{U: u, V: v} }
	controls := func(n int) []Point2 {
		out := make([]Point2, n)
		for i := range out {
			out[i] = point(float64(i), float64(i%3))
		}
		return out
	}

	for _, tc := range []struct {
		name    string
		segment CurveSegment
		want    uint64
	}{
		{name: "line", segment: LineSeg{Start: point(0, 0), End: point(1, 0)}, want: 1},
		{
			name:    "circle",
			segment: CircleSeg{Center: point(0, 0), Radius: units.Millimeters(1), CCW: true},
			want:    256,
		},
		{
			name:    "quarter arc",
			segment: ArcSeg{Center: point(0, 0), Start: point(1, 0), End: point(0, 1)},
			want:    64,
		},
		{
			name:    "half arc",
			segment: ArcSeg{Center: point(0, 0), Start: point(1, 0), End: point(-1, 0)},
			want:    128,
		},
		{
			name:    "three-control closed spline floors at 64",
			segment: ClosedSplineSeg{Control: controls(3), CCW: true},
			want:    64,
		},
		{
			name:    "large closed spline is 16 per control",
			segment: ClosedSplineSeg{Control: controls(100), CCW: true},
			want:    1600,
		},
		{
			name:    "open spline is 16 per span",
			segment: SplineSeg{Control: controls(100)},
			want:    16 * 97,
		},
		{
			name:    "four-control open spline floors at 64",
			segment: SplineSeg{Control: controls(4)},
			want:    64,
		},
		{
			name: "NURBS is 16 per control",
			segment: NURBSSeg{
				Degree: 1, Control: controls(100),
				Knots: make([]float64, 102), Weights: make([]float64, 100),
			},
			want: 1600,
		},
		{name: "fit spline is 16 per fit point", segment: FitSplineSeg{Fit: controls(100)}, want: 1600},
	} {
		t.Run(tc.name, func(t *testing.T) {
			require.Equal(t, tc.want, reconstructionChords(tc.segment))
		})
	}
}

// The arrangement is GLOBAL, so the charge squares the record-wide chord total
// rather than summing per-source squares. The difference is every cross-source
// pair, which is most of them: two sources of 64 chords each arrange 16384
// ordered pairs, where per-source squares see 8192.
func TestReconstructionChargeSquaresTheRecordTotal(t *testing.T) {
	t.Parallel()
	control := []Point2{{U: 0, V: 0}, {U: 4, V: 0}, {U: 2, V: 3}}
	one := ClosedSplineSeg{Control: control, CCW: true, TStart: 0, TEnd: 1}

	single := reconstructionOf(ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{one}}})
	require.Equal(t, uint64(64), single.chords)
	require.Equal(t, uint64(64*64), single.arrangement)

	pair := reconstructionOf(ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{one, one}}})
	require.Equal(t, uint64(128), pair.chords, "the chord total is the whole record's")
	require.Equal(t, uint64(128*128), pair.arrangement)
	require.Greater(t, pair.arrangement, 2*single.arrangement,
		"a sum of per-source squares drops every cross-source pair")

	// A hole's chords are in the same arrangement as the outer loop's.
	withHole := reconstructionOf(ProfileRecord{
		Outer: LoopRecord{Segments: []CurveSegment{one}},
		Holes: []LoopRecord{{Segments: []CurveSegment{one}}},
	})
	require.Equal(t, pair, withHole)

	// The record-level half of the charge pays for the two whole-scene
	// arrangements the validation always runs, and reports the per-arrangement
	// charge each candidate profile then levies for itself.
	work := &freeformWork{}
	arrangement, err := chargeReconstruction(
		ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{one}}},
		work,
	)
	require.NoError(t, err)
	require.Equal(t, single.arrangement, arrangement)
	require.Equal(t, 2*single.arrangement, work.reconstructionSpent)
	require.Zero(t, work.spent, "the reconstruction charge leaves exact-rational work available")
}

// A circle that a crossing split into two recorded fragments still produces one
// entity in the scene. Its fragment ranges differ, but each names the same
// center and radius, so the reconstruction charge must add its chords once.
func TestReconstructionChargeInternsSharedAnalyticEntity(t *testing.T) {
	t.Parallel()
	first := CircleSeg{
		Center: Point2{U: 4, V: 5}, Radius: units.Millimeters(2), CCW: true,
		TStart: 0, TEnd: 0.5,
	}
	second := first
	second.TStart, second.TEnd = 0.5, 1

	single := reconstructionOf(ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{first}}})
	shared := reconstructionOf(ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{first, second}}})
	require.Equal(t, single.chords, shared.chords, "two fragments naming one circle are charged once")

	distinct := second
	distinct.Center = Point2{U: 9, V: 5}
	twoEntities := reconstructionOf(ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{first, distinct}}})
	require.Equal(t, 2*single.chords, twoEntities.chords, "two distinct circles each contribute their chords")
}

// The charge a conversion levies must grow with the work it actually does. A
// clamped degree-3 B-spline inserts twice per interior knot and each insertion
// scans and copies both whole vectors, so the cost is QUADRATIC in the control
// count — a charge that only counted insertions would let a hundred-thousand
// control record run for hours inside the ceiling.
func TestClampedConversionCostIsQuadratic(t *testing.T) {
	t.Parallel()
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
	t.Parallel()
	rationalDemand := func(degree, n int, knots []*big.Rat) knotInsertionDemand {
		_, runs, _ := interiorKnotRuns(degree, n, knots)
		var demand knotInsertionDemand
		for _, run := range runs {
			demand.add(degree, run)
		}
		return demand
	}

	for _, controls := range []int{4, 5, 9, 40} {
		knots := clampedUniformKnots(controls)
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
	t.Parallel()
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
	t.Parallel()
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
	t.Parallel()
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
	t.Parallel()
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
