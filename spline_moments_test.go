package decad_test

import (
	"math"
	"math/big"
	"strconv"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/geom"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// closedSplineControls is the fixture both the record and the dense reference
// are built from, so the two answers cannot drift apart.
var closedSplineControls = [][2]float64{{0, 0}, {4, 0}, {5, 3}, {2, 5}, {-1, 3}}

// scaledClosedSplineControls is the same section at three times the size. Its
// exact area is 9·293/18 = 293/2, which IS representable in float64 — so it
// exercises the Exact side of the spline design §3 rounding rule that the base
// fixture (293/18, not representable) exercises the Approximate side of.
func scaledClosedSplineControls() [][2]float64 {
	out := make([][2]float64, len(closedSplineControls))
	for i, control := range closedSplineControls {
		out[i] = [2]float64{control[0] * 3, control[1] * 3}
	}
	return out
}

func recordClosedSplineFrom(t *testing.T, controls [][2]float64) decad.ProfileRecord {
	t.Helper()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)

	points := make([]*sketch.Point, len(controls))
	for i, control := range controls {
		points[i] = s.CreatePoint(control[0], control[1])
	}
	_, err = s.CreateClosedSpline(points...)
	require.NoError(t, err)

	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)

	record, _, err := decad.RecordProfile(s, profiles[0])
	require.NoError(t, err)
	require.Len(t, record.Outer.Segments, 1)
	require.IsType(t, decad.ClosedSplineSeg{}, record.Outer.Segments[0])
	return record
}

// densePolylineArea is the falsifier for the exact answer: a 400k-segment
// shoelace over sketch's own sampler. It never replaces the exact integral; it
// only disproves a wrong one.
func densePolylineArea(t *testing.T, controls [][2]float64) float64 {
	t.Helper()
	ring, err := geom.SamplePeriodicCubicBSpline(controls, 400000)
	require.NoError(t, err)
	var twice float64
	n := len(ring) - 1 // drop the repeated closing point
	for i := range n {
		j := (i + 1) % n
		twice += ring[i][0]*ring[j][1] - ring[j][0]*ring[i][1]
	}
	return math.Abs(twice / 2)
}

// The exact integral is 293/18, which float64 cannot represent, so spline
// design §3 requires Approximate with a bound of ONE rounding — not a
// quadrature-sized bound, and not a false Exact.
func TestClosedSplineProfileMomentsRoundOnce(t *testing.T) {
	record := recordClosedSplineFrom(t, closedSplineControls)

	area, err := record.Area()
	require.NoError(t, err)
	require.Equal(t, units.Area, area.Value.Kind())
	require.Equal(t, decad.Approximate, area.Exactness, "293/18 is not representable in float64")

	value, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	// The correctly-rounded 293/18 — and it is what sketch's own arrangement reports.
	require.Equal(t, 293.0/18.0, value)
	require.InDelta(t, densePolylineArea(t, closedSplineControls), value, 1e-6,
		"the exact area survives a dense-sample falsifier")

	bound, err := area.Bound.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.Positive(t, bound)
	require.LessOrEqual(t, bound, math.Nextafter(value, math.Inf(1))-value,
		"the bound is a single rounding of the exact rational, not an estimate")

	centroid, err := record.Centroid()
	require.NoError(t, err)
	require.InDelta(t, 2.0, centroid.Value.X, 1e-15, "∫u dA / A is 293/9 ÷ 293/18 = 2")
	require.InDelta(t, 2.1941383606912619, centroid.Value.Y, 1e-12)
	require.Zero(t, centroid.Value.Z, "a plane-local centroid has no third coordinate")

	moments, err := record.SecondMoments()
	require.NoError(t, err)
	for name, moment := range map[string]decad.Measurement{
		"UU": moments.UU,
		"UV": moments.UV,
		"VV": moments.VV,
	} {
		require.Equal(t, units.SecondMomentOfArea, moment.Value.Kind(), "%s kind", name)
	}
	uu, err := moments.UU.Value.In(units.QuarticMillimeter)
	require.NoError(t, err)
	require.InDelta(t, 89.798354410507187, uu, 1e-9)
}

// The Exact side of the same rule: 293/2 IS representable, so the single
// rounding is no rounding at all and the bound is zero.
func TestClosedSplineProfileAreaExactWhenRepresentable(t *testing.T) {
	controls := scaledClosedSplineControls()
	record := recordClosedSplineFrom(t, controls)

	area, err := record.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, area.Exactness, "293/2 is representable in float64")
	bound, err := area.Bound.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.Zero(t, bound, "an Exact measurement carries a zero bound")

	value, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.Equal(t, 293.0/2.0, value, "9 x 293/18 exactly")
	require.InDelta(t, densePolylineArea(t, controls), value, 1e-5)
}

// requireSingleRounding proves a published moment IS its region's exact
// rational rounded once: the held float is that rational's own nearest float64,
// and the bound is the distance between them, which correct rounding keeps
// within half an ulp. A held float summed from PER-SEGMENT roundings fails
// both — its value drifts off the correctly rounded one and its bound is the
// sum of the roundings that produced the drift.
func requireSingleRounding(t *testing.T, exact *big.Rat, value, bound float64) {
	t.Helper()
	want, _ := exact.Float64()
	require.Equal(t, want, value, "the held float is the region rational rounded once")
	halfUlp := (math.Nextafter(value, math.Inf(1)) - value) / 2
	require.LessOrEqual(t, bound, math.Nextafter(halfUlp, math.Inf(1)),
		"a single rounding never exceeds half an ulp")
	// The enclosure is checked over rationals: the bound is a small fraction of
	// an ulp here, so value±bound would round straight back to value.
	drift := new(big.Rat).Sub(new(big.Rat).SetFloat64(value), exact)
	require.LessOrEqual(t, drift.Abs(drift).Cmp(new(big.Rat).SetFloat64(bound)), 0,
		"the bound encloses the exact value")
}

// nurbsEdge is one straight polygon side recorded as its own degree-1 NURBS
// segment, so a polygon built from them is a MULTI-segment Tier A region.
func nurbsEdge(a, b decad.Point2) decad.NURBSSeg {
	return decad.NURBSSeg{
		Degree:  1,
		Control: []decad.Point2{a, b},
		Knots:   []float64{0, 0, 1, 1},
		Weights: []float64{1, 1},
		TStart:  0,
		TEnd:    1,
	}
}

// spline design §3 requires the held float to be the exact rational rounded
// ONCE. A region with several Tier A boundary segments must therefore round
// its own region-level sum, not each segment's contribution: five separate
// degree-1 NURBS edges whose per-curve roundings are summed land a full ulp
// off the correctly rounded area and report a bound wider than the rounding
// they actually committed.
func TestMultiSegmentFreeformRegionRoundsOnce(t *testing.T) {
	polygon := []decad.Point2{{}, {U: 0.1}, {U: 0.3, V: 0.2}, {U: 0.05, V: 0.1}, {V: 0.4}}
	segments := make([]decad.CurveSegment, len(polygon))
	for i := range polygon {
		segments[i] = nurbsEdge(polygon[i], polygon[(i+1)%len(polygon)])
	}
	record := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: segments}}

	// The falsifier: the polygon's own shoelace over exact rationals.
	exact := new(big.Rat)
	for i := range polygon {
		j := (i + 1) % len(polygon)
		ax, ay := new(big.Rat).SetFloat64(polygon[i].U), new(big.Rat).SetFloat64(polygon[i].V)
		bx, by := new(big.Rat).SetFloat64(polygon[j].U), new(big.Rat).SetFloat64(polygon[j].V)
		exact.Add(exact, new(big.Rat).Sub(new(big.Rat).Mul(ax, by), new(big.Rat).Mul(bx, ay)))
	}
	exact.Quo(exact, big.NewRat(2, 1))

	area, err := record.Area()
	require.NoError(t, err)
	value, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	bound, err := area.Bound.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, area.Exactness, "this polygon's area is not representable")
	requireSingleRounding(t, exact, value, bound)
}

// The measured defect behind the conversion budget: a validator-accepted
// degree-3 NURBS with thousands of control points charged eight units per
// insertion while each insertion scanned and copied the whole vectors, so the
// ceiling could not fire until billions of operations had run. It must refuse
// promptly instead.
func TestDenseNURBSRecordRefusesWithinBudget(t *testing.T) {
	const controls = 2000
	const degree = 3
	control := make([]decad.Point2, controls)
	weights := make([]float64, controls)
	for i := range control {
		control[i] = decad.Point2{U: float64(i), V: float64(i % 5)}
		weights[i] = 1
	}
	spans := float64(controls - degree)
	knots := make([]float64, 0, controls+degree+1)
	for range degree + 1 {
		knots = append(knots, 0)
	}
	for j := 1; j < int(spans); j++ {
		knots = append(knots, float64(j)/spans)
	}
	for range degree + 1 {
		knots = append(knots, 1)
	}
	record := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
		decad.NURBSSeg{Degree: degree, Control: control, Knots: knots, Weights: weights, TStart: 0, TEnd: 1},
	}}}

	start := time.Now()
	_, err := record.Area()
	require.Error(t, err)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "work budget")
	require.Less(t, time.Since(start), 5*time.Second, "the refusal precedes the insertion pass")
}

// A NURBS whose interior knots repeat degree+1 times is four disconnected cubic
// pieces, not one boundary curve: the four sides of a unit square, each its own
// Bézier, spliced into a single segment. It satisfies every count, ordering and
// clamping rule, so nothing else refuses it — and the exact conversion's
// stride-degree slicing then reads five spans across those four pieces, rounding
// the (1,1) corner and losing 1/180 of the area under a bound fourteen orders of
// magnitude too small. The record is malformed and must be REFUSED, never
// measured.
func TestBrokenNURBSKnotVectorRefuses(t *testing.T) {
	third := 1.0 / 3
	record := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
		decad.NURBSSeg{
			Degree: 3,
			Control: []decad.Point2{
				{U: 0, V: 0}, {U: third, V: 0}, {U: 2 * third, V: 0}, {U: 1, V: 0},
				{U: 1, V: 0}, {U: 1, V: third}, {U: 1, V: 2 * third}, {U: 1, V: 1},
				{U: 1, V: 1}, {U: 2 * third, V: 1}, {U: third, V: 1}, {U: 0, V: 1},
				{U: 0, V: 1}, {U: 0, V: 2 * third}, {U: 0, V: third}, {U: 0, V: 0},
			},
			Knots: []float64{
				0, 0, 0, 0,
				0.25, 0.25, 0.25, 0.25,
				0.5, 0.5, 0.5, 0.5,
				0.75, 0.75, 0.75, 0.75,
				1, 1, 1, 1,
			},
			Weights: []float64{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
			TStart:  0,
			TEnd:    1,
		},
	}}}

	for name, measure := range map[string]func() error{
		"Area":          func() error { _, err := record.Area(); return err },
		"Centroid":      func() error { _, err := record.Centroid(); return err },
		"SecondMoments": func() error { _, err := record.SecondMoments(); return err },
	} {
		t.Run(name, func(t *testing.T) {
			err := measure()
			require.Error(t, err, "a broken knot vector states no single curve")
			require.ErrorIs(t, err, decad.ErrDegenerate)
			require.Contains(t, err.Error(), "disjoint pieces")
		})
	}
}

// The walk anchor is subtracted from every coordinate before integration, and
// subtracting it in float64 first would round the geometry away: this rectangle
// sits at (0.1, 0.1), so fl(100.1−0.1) is exactly 100 and fl(1.1−0.1) exactly 1,
// turning it into a 100×1 rectangle whose area is the integer 100 — representable,
// hence published as Exact with a zero bound. The recorded rectangle's own exact
// shoelace is NOT representable, so the honest reading is Approximate with the
// single rounding as its bound.
func TestFreeformAnchorSubtractsExactly(t *testing.T) {
	corners := []decad.Point2{{U: 0.1, V: 0.1}, {U: 100.1, V: 0.1}, {U: 100.1, V: 1.1}, {U: 0.1, V: 1.1}}
	segments := make([]decad.CurveSegment, len(corners))
	for i := range corners {
		segments[i] = nurbsEdge(corners[i], corners[(i+1)%len(corners)])
	}
	record := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: segments}}

	// The falsifier: the shoelace over the RECORDED float coordinates, exactly.
	exact := new(big.Rat)
	for i := range corners {
		j := (i + 1) % len(corners)
		ax, ay := new(big.Rat).SetFloat64(corners[i].U), new(big.Rat).SetFloat64(corners[i].V)
		bx, by := new(big.Rat).SetFloat64(corners[j].U), new(big.Rat).SetFloat64(corners[j].V)
		exact.Add(exact, new(big.Rat).Sub(new(big.Rat).Mul(ax, by), new(big.Rat).Mul(bx, ay)))
	}
	exact.Quo(exact, big.NewRat(2, 1))
	_, representable := exact.Float64()
	require.False(t, representable, "the fixture's own area is not a float64")

	area, err := record.Area()
	require.NoError(t, err)
	value, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	bound, err := area.Bound.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, area.Exactness,
		"the recorded rectangle's area is not representable, so nothing may claim Exact")
	require.Positive(t, bound)
	requireSingleRounding(t, exact, value, bound)
}

// The work ceiling exists because the public ProfileRecord methods take no
// context and so cannot be cancelled. It must therefore fire BEFORE the sketch
// reconstruction that validation runs, which samples the curve a multiple of the
// control count and grows without any bound of its own: a degree-128 record
// spent 235ms there before the ceiling refused it, and a degree-599 one over
// thirteen seconds.
func TestOverBudgetFreeformRefusesBeforeSketchSampling(t *testing.T) {
	const controls = 600
	const degree = controls - 1
	control := make([]decad.Point2, controls)
	weights := make([]float64, controls)
	for i := range control {
		angle := 2 * math.Pi * float64(i) / float64(controls-1)
		control[i] = decad.Point2{U: 10 * math.Cos(angle), V: 10 * math.Sin(angle)}
		weights[i] = 1
	}
	control[controls-1] = control[0]
	knots := make([]float64, 0, 2*controls)
	for range controls {
		knots = append(knots, 0)
	}
	for range controls {
		knots = append(knots, 1)
	}
	record := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
		decad.NURBSSeg{Degree: degree, Control: control, Knots: knots, Weights: weights, TStart: 0, TEnd: 1},
	}}}

	start := time.Now()
	_, err := record.Area()
	require.Error(t, err)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "work budget")
	require.Less(t, time.Since(start), 2*time.Second, "the refusal precedes the sketch reconstruction")
}

// A spline chained with a straight chord exercises the mixed path: the line
// contributes through the existing exact-rational line formulas and the spline
// through the Bézier integrals, into one region.
func TestSplineAndChordProfileMoments(t *testing.T) {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	start := s.CreatePoint(0, 0)
	c1 := s.CreatePoint(1, 2)
	c2 := s.CreatePoint(3, 2)
	end := s.CreatePoint(4, 0)
	_, err = s.CreateSpline(start, c1, c2, end)
	require.NoError(t, err)
	s.CreateLine(end, start)

	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)

	record, _, err := decad.RecordProfile(s, profiles[0])
	require.NoError(t, err)

	area, err := record.Area()
	require.NoError(t, err)
	value, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.Greater(t, value, 0.0, "the region area is a positive magnitude")

	// Both kinds integrate to exact rationals, so the mixed region is summed
	// exactly and rounded once — its bound stays within a single rounding
	// rather than adding the line's rounding to the spline's.
	bound, err := area.Bound.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.LessOrEqual(
		t,
		bound,
		math.Nextafter((math.Nextafter(value, math.Inf(1))-value)/2, math.Inf(1)),
		"a single rounding never exceeds half an ulp",
	)

	// The dense reference: the hump's sampled area closed by its chord.
	ring, err := geom.SampleCubicBSpline([][2]float64{{0, 0}, {1, 2}, {3, 2}, {4, 0}}, 200000)
	require.NoError(t, err)
	var twice float64
	for i := 0; i+1 < len(ring); i++ {
		twice += ring[i][0]*ring[i+1][1] - ring[i+1][0]*ring[i][1]
	}
	twice += ring[len(ring)-1][0]*ring[0][1] - ring[0][0]*ring[len(ring)-1][1]
	require.InDelta(t, math.Abs(twice/2), value, 1e-6)
}

func TestFreeformProfileRefusals(t *testing.T) {
	for _, tc := range []struct {
		name    string
		segment decad.CurveSegment
		message string
	}{
		{
			name:    "fit spline",
			segment: decad.FitSplineSeg{Fit: []decad.Point2{{}, {U: 4}}, TStart: 0, TEnd: 1},
			message: "interpolation solve",
		},
		{
			name: "elliptical arc",
			segment: decad.EllipticalArcSeg{
				Center:   decad.Point2{},
				Start:    decad.Point2{U: 2},
				End:      decad.Point2{V: 1},
				Rx:       units.Millimeters(2),
				Ry:       units.Millimeters(1),
				Rotation: units.Radians(0),
				TStart:   0,
				TEnd:     1,
			},
			message: "pinned endpoints",
		},
		{
			name: "conic",
			segment: decad.ConicSeg{
				Start: decad.Point2{}, Apex: decad.Point2{U: 1, V: 1}, End: decad.Point2{U: 2},
				Rho: 0.4, TStart: 0, TEnd: 1,
			},
			message: "closed form",
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			record := decad.ProfileRecord{
				Outer: decad.LoopRecord{Segments: []decad.CurveSegment{tc.segment}},
			}
			_, err := record.Area()
			require.Error(t, err)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Contains(t, err.Error(), tc.message)
		})
	}
}

// closedSplineRecordOf is a caller-built ClosedSplineSeg record over a ring of
// controls — the shape the record-level work counter is calibrated on, since a
// closed spline converts to one Bézier span per control point.
func closedSplineRecordOf(controls int) decad.ProfileRecord {
	segments := []decad.CurveSegment{closedSplineSegmentOf(controls, 10)}
	return decad.ProfileRecord{Outer: decad.LoopRecord{Segments: segments}}
}

func closedSplineSegmentOf(controls int, radius float64) decad.ClosedSplineSeg {
	control := make([]decad.Point2, controls)
	for i := range control {
		angle := 2 * math.Pi * float64(i) / float64(controls)
		control[i] = decad.Point2{U: radius * math.Cos(angle), V: radius * math.Sin(angle)}
	}
	return decad.ClosedSplineSeg{Control: control, CCW: true, TStart: 0, TEnd: 1}
}

// A caller can hand a degree-1 NURBS segment millions of control points and no
// knots at all. Whether the knot count can match the control count is an O(1)
// question about SIZES, so it must be answered before the control array is
// read: the point scan formats a description per point, so a record that cannot
// be well formed at ANY content used to cost three allocations and about 90ns
// per control point before its refusal — 12 million allocations on a
// four-million-point record, and none of it charged against the work ceiling.
func TestMalformedNURBSRefusesBeforeScanningControls(t *testing.T) {
	const controls = 200000
	control := make([]decad.Point2, controls)
	for i := range control {
		control[i] = decad.Point2{U: float64(i), V: float64(i % 3)}
	}
	record := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
		decad.NURBSSeg{Degree: 1, Control: control, TStart: 0, TEnd: 1},
	}}}

	var err error
	allocs := testing.AllocsPerRun(1, func() { _, err = record.Area() })
	require.Error(t, err)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Contains(t, err.Error(), "knots")
	require.Less(t, allocs, 1000.0,
		"the refusal reads sizes only, never the %d recorded control points", controls)
}

// The work ceiling bounds a RECORD's total free-form work, never each segment's
// own. Two closed splines of 500 controls are individually affordable — 547,000
// charged units each against a ceiling of 1,048,576 — and unaffordable together,
// so a counter opened per segment reads both as cheap and lets the record run to
// a topology answer instead of refusing. The refusal names the SECOND segment,
// which is the proof that the first segment's charge carried into it.
func TestFreeformWorkBudgetBoundsTheWholeRecord(t *testing.T) {
	const controls = 500
	record := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
		closedSplineSegmentOf(controls, 10),
		closedSplineSegmentOf(controls, 3),
	}}}

	start := time.Now()
	_, err := record.Area()
	require.Error(t, err)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "work budget")
	require.Contains(t, err.Error(), "profile loop 0 segment 1 is invalid",
		"the first segment was affordable, so the record's own counter is what refused the second")
	require.Less(t, time.Since(start), 10*time.Second, "the refusal precedes the topology reconstruction")
}

// Every charge a free-form segment will ever owe is levied at the record-level
// preflight, the re-anchoring of its converted chain among them. A 960-control
// closed spline charges 1,050,240 units — over the 1,048,576 ceiling by less
// than the 7,680 its re-anchoring contributes — so a preflight that omits that
// term admits the record, spends about six uncancellable seconds reconstructing
// and sampling the curve in sketch, and only then refuses from the moments pass.
// The validation prefix is what tells the two apart: validateMomentFields adds
// it to everything the preflight refuses, and the moments pass adds nothing.
func TestFreeformReanchoringChargeRefusesAtValidation(t *testing.T) {
	record := closedSplineRecordOf(960)

	start := time.Now()
	_, err := record.Area()
	require.Error(t, err)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "work budget")
	require.Contains(t, err.Error(), "profile loop 0 segment 0 is invalid",
		"the refusal is the preflight's, ahead of the sketch reconstruction")
	require.Less(t, time.Since(start), 3*time.Second, "no reconstruction ran before the refusal")
}

// A non-finite recorded range is a non-finite INPUT, and core §12 gives it
// ErrNotFinite — which is what a line, a circle and an arc all report for
// exactly this field, and what the free-form path itself reports for a
// non-finite control coordinate. NaN fails both of the full-domain equality
// tests, so a range test consulted first reads it as a trimmed range and
// reports the staging sentinel ErrUnsupported instead.
func TestNonFiniteFreeformRangeIsNotFinite(t *testing.T) {
	control := []decad.Point2{{}, {U: 1, V: 1}, {U: 2, V: 1}, {U: 3}}
	for _, tc := range []struct {
		name    string
		segment decad.CurveSegment
	}{
		{
			name:    "spline NaN start",
			segment: decad.SplineSeg{Control: control, TStart: math.NaN(), TEnd: 1},
		},
		{
			name:    "spline Inf end",
			segment: decad.SplineSeg{Control: control, TStart: 0, TEnd: math.Inf(1)},
		},
		{
			name: "closed spline NaN end",
			segment: decad.ClosedSplineSeg{
				Control: []decad.Point2{{}, {U: 4}, {U: 2, V: 3}}, CCW: true,
				TStart: 0, TEnd: math.NaN(),
			},
		},
		{
			name: "NURBS NaN start",
			segment: decad.NURBSSeg{
				Degree: 1, Control: []decad.Point2{{}, {U: 1}},
				Knots: []float64{0, 0, 1, 1}, Weights: []float64{1, 1},
				TStart: math.NaN(), TEnd: 1,
			},
		},
		{
			name:    "line NaN start",
			segment: decad.LineSeg{Start: decad.Point2{}, End: decad.Point2{U: 1}, TStart: math.NaN(), TEnd: 1},
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			record := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{tc.segment}}}
			_, err := record.Area()
			require.Error(t, err)
			require.ErrorIs(t, err, decad.ErrNotFinite)
			require.NotErrorIs(t, err, decad.ErrUnsupported)
		})
	}
}

// A NURBS whose weights are ALL EQUAL is the same curve at every magnitude —
// equal weights cancel in the homogeneous quotient — so it is Tier A and owes
// an answer whatever magnitude the record states. The reconstruction sketch
// runs to decide topology squares the homogeneous denominator, which overflows
// past about sqrt(MaxFloat64), so the raw magnitudes have to be normalized for
// it: without that, the unit square at weight 1e300 is refused as a region that
// does not close while the identical curve at weight 1 measures exactly 1 mm².
func TestEqualWeightNURBSMeasuresAtEveryMagnitude(t *testing.T) {
	square := []decad.Point2{{}, {U: 1}, {U: 1, V: 1}, {V: 1}, {}}
	for _, weight := range []float64{1, 1e150, 1e300, math.MaxFloat64} {
		t.Run(strconv.FormatFloat(weight, 'g', -1, 64), func(t *testing.T) {
			weights := make([]float64, len(square))
			for i := range weights {
				weights[i] = weight
			}
			record := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
				decad.NURBSSeg{
					Degree:  1,
					Control: square,
					Knots:   []float64{0, 0, 0.25, 0.5, 0.75, 1, 1},
					Weights: weights,
					TStart:  0,
					TEnd:    1,
				},
			}}}

			area, err := record.Area()
			require.NoError(t, err)
			value, err := area.Value.In(units.SquareMillimeter)
			require.NoError(t, err)
			require.Equal(t, 1.0, value, "the unit square's area does not depend on its weights")
			require.Equal(t, decad.Exact, area.Exactness, "1 is representable")
		})
	}
}

// A region whose exact area is strictly positive owes a measurement even when
// that area underflows float64: the accumulator holds the positive rational, so
// the honest reading is the bounded zero it rounds to — value 0 with the
// rational rounded up as its bound, hence Approximate — not a refusal. The
// module already publishes a subnormal area one step above this scale, so a gate
// reading the float accumulator refuses the very next step down.
func TestUnderflowingSplineAreaPublishesBoundedZero(t *testing.T) {
	const scale = 1e-200
	controls := make([][2]float64, len(closedSplineControls))
	for i, control := range closedSplineControls {
		controls[i] = [2]float64{control[0] * scale, control[1] * scale}
	}
	control := make([]decad.Point2, len(controls))
	for i, c := range controls {
		control[i] = decad.Point2{U: c[0], V: c[1]}
	}
	record := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
		decad.ClosedSplineSeg{Control: control, CCW: true, TStart: 0, TEnd: 1},
	}}}

	area, err := record.Area()
	require.NoError(t, err, "the exact rational area is 293/18 x 1e-400, which is strictly positive")
	value, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.Zero(t, value, "no float64 holds this area")
	bound, err := area.Bound.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.Positive(t, bound, "the bound is the rounding that produced the zero")
	require.Equal(t, decad.Approximate, area.Exactness)
}

// A caller-built record whose spline control points all coincide states no
// boundary. It must refuse rather than integrate a zero-area region into a
// centroid division.
func TestDegenerateSplineRecordRefuses(t *testing.T) {
	same := decad.Point2{U: 3, V: 3}
	record := decad.ProfileRecord{
		Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
			decad.ClosedSplineSeg{Control: []decad.Point2{same, same, same}, CCW: true, TStart: 0, TEnd: 1},
		}},
	}
	_, err := record.Area()
	require.Error(t, err)
	require.ErrorIs(t, err, decad.ErrDegenerate)
}
