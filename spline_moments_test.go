package decad_test

import (
	"encoding/json"
	"math"
	"math/big"
	"runtime"
	"slices"
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
	t.Parallel()
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
	t.Parallel()
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
	t.Parallel()
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
	t.Parallel()
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

// A NURBS whose interior knots repeat degree+1 times is four cubic pieces
// spliced into one segment — the four sides of a unit square, each its own
// Bézier. It satisfies every count, ordering and clamping rule, so nothing else
// refuses it, and the exact conversion's stride-degree slicing then reads five
// spans across those four pieces, rounding the (1,1) corner and losing 1/180 of
// the area under a bound fourteen orders of magnitude too small. It must be
// REFUSED, never measured.
//
// WHICH refusal is decided by the recorded curve rather than by the slicer. At
// multiplicity degree+1 the two one-sided limits at a break are exactly two
// recorded control points; when they are the SAME point the four pieces meet,
// the curve is one connected curve and the body exists, so the refusal is this
// evaluator's own limitation. Move one of those two points and the curve really
// does break apart, so no such body exists.
func TestBrokenNURBSKnotVectorRefuses(t *testing.T) {
	t.Parallel()
	third := 1.0 / 3
	squareRecord := func(joint decad.Point2) decad.ProfileRecord {
		return decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
			decad.NURBSSeg{
				Degree: 3,
				Control: []decad.Point2{
					{U: 0, V: 0}, {U: third, V: 0}, {U: 2 * third, V: 0}, {U: 1, V: 0},
					joint, {U: 1, V: third}, {U: 1, V: 2 * third}, {U: 1, V: 1},
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
	}

	for _, tc := range []struct {
		name     string
		joint    decad.Point2
		sentinel error
		message  string
	}{
		{
			name:     "continuous",
			joint:    decad.Point2{U: 1, V: 0},
			sentinel: decad.ErrUnsupported,
			message:  "share no boundary control point",
		},
		{
			name:     "discontinuous",
			joint:    decad.Point2{U: 1.5, V: 0},
			sentinel: decad.ErrDegenerate,
			message:  "disjoint pieces",
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			record := squareRecord(tc.joint)
			for name, measure := range map[string]func() error{
				"Area":          func() error { _, err := record.Area(); return err },
				"Centroid":      func() error { _, err := record.Centroid(); return err },
				"SecondMoments": func() error { _, err := record.SecondMoments(); return err },
			} {
				t.Run(name, func(t *testing.T) {
					err := measure()
					require.Error(t, err, "the slicer's precondition does not hold on this record")
					require.ErrorIs(t, err, tc.sentinel)
					require.Contains(t, err.Error(), tc.message)
				})
			}
		})
	}
}

// record.go admits a knot vector clamped one repeat PAST degree+1 at an end: the
// extra repeat leaves a dead control point and no discontinuity anywhere, so the
// record states a perfectly ordinary single quadratic Bézier. The evaluator
// still cannot slice it — 4 control points are not a whole number of degree-2
// spans — and that is a limitation of the evaluator, not a claim that no such
// body exists.
func TestOverClampedNURBSRefusesAsUnsupported(t *testing.T) {
	t.Parallel()
	segment := decad.NURBSSeg{
		Degree:  2,
		Control: []decad.Point2{{U: 0, V: 0}, {U: 0, V: 0}, {U: 1, V: 2}, {U: 2, V: 0}},
		Knots:   []float64{0, 0, 0, 0, 1, 1, 1},
		Weights: []float64{1, 1, 1, 1},
		TStart:  0,
		TEnd:    1,
	}
	record := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
		segment,
		decad.LineSeg{Start: decad.Point2{U: 2}, End: decad.Point2{}, TStart: 0, TEnd: 1},
	}}}

	_, err := record.Area()
	require.Error(t, err)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.NotErrorIs(t, err, decad.ErrDegenerate)
	require.Contains(t, err.Error(), "whole number of degree-2 Bézier spans")
}

// The walk anchor is subtracted from every coordinate before integration, and
// subtracting it in float64 first would round the geometry away: this rectangle
// sits at (0.1, 0.1), so fl(100.1−0.1) is exactly 100 and fl(1.1−0.1) exactly 1,
// turning it into a 100×1 rectangle whose area is the integer 100 — representable,
// hence published as Exact with a zero bound. The recorded rectangle's own exact
// shoelace is NOT representable, so the honest reading is Approximate with the
// single rounding as its bound.
func TestFreeformAnchorSubtractsExactly(t *testing.T) {
	t.Parallel()
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

// recordSplineAndChord records the hump-and-chord region: an open cubic spline
// closed by a straight line, recorded through sketch so the loop carries the
// arrangement's own segment order and walk direction.
func recordSplineAndChord(t *testing.T) decad.ProfileRecord {
	t.Helper()
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
	return record
}

// recordFitSplineAndChord records a hump-and-chord region built from a
// FitSplineSeg instead of a SplineSeg: an open fit spline through three points
// closed by a straight line, recorded through sketch so the loop carries the
// arrangement's own segment order and walk direction.
func recordFitSplineAndChord(t *testing.T) decad.ProfileRecord {
	t.Helper()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	start := s.CreatePoint(0, 0)
	mid := s.CreatePoint(1, 1)
	end := s.CreatePoint(2, 0)
	_, err = s.CreateFitSpline(start, mid, end)
	require.NoError(t, err)
	s.CreateLine(end, start)

	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)

	record, _, err := decad.RecordProfile(s, profiles[0])
	require.NoError(t, err)
	return record
}

// allocatedBy reports how many bytes a call allocates in total, which is the
// only way to tell a charge levied BEFORE an allocation from one levied after
// it: both refuse, and only the measurement distinguishes them.
func allocatedBy(call func()) uint64 {
	runtime.GC()
	var before, after runtime.MemStats
	runtime.ReadMemStats(&before)
	call()
	runtime.ReadMemStats(&after)
	return after.TotalAlloc - before.TotalAlloc
}

// The conversion's charge has to be RESERVED before the rational lift, not after
// it. An open spline's conversion is quadratic, so the largest record the
// ceiling can ever admit holds around 136 control points — and a record three
// orders of magnitude past that was still lifting two big.Rat per control point
// and a whole rational knot vector before its refusal: 118 MB and 179 ms at
// 200,000 controls, against 16 us and no allocation at all one control point
// past the point where the LIFT's own linear charge saturated. A refused record
// must allocate on the order of the record itself.
func TestOverBudgetConversionRefusesBeforeLifting(t *testing.T) {
	for _, tc := range []struct {
		name     string
		controls int
		segment  func([]decad.Point2) decad.CurveSegment
	}{
		{
			name:     "spline",
			controls: 200000,
			segment: func(control []decad.Point2) decad.CurveSegment {
				return decad.SplineSeg{Control: control, TStart: 0, TEnd: 1}
			},
		},
		{
			name:     "closed spline",
			controls: 300000,
			segment: func(control []decad.Point2) decad.CurveSegment {
				return decad.ClosedSplineSeg{Control: control, CCW: true, TStart: 0, TEnd: 1}
			},
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			control := make([]decad.Point2, tc.controls)
			for i := range control {
				control[i] = decad.Point2{U: float64(i), V: float64(i % 5)}
			}
			record := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
				tc.segment(control),
			}}}

			var err error
			start := time.Now()
			allocated := allocatedBy(func() { _, err = record.Area() })
			elapsed := time.Since(start)

			require.Error(t, err)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Contains(t, err.Error(), "work budget")
			// The record's own control points are 16 bytes each and are already
			// allocated when the measurement starts, so this budget is generous
			// against a refusal that lifts nothing and tiny against one that does.
			require.Less(t, allocated, uint64(tc.controls)*16,
				"a refused record allocates on the order of the record, not the rationals it would have lifted")
			require.Less(t, elapsed, 2*time.Second)
		})
	}
}

// The reconstruction counter has its own larger ceiling because sketch's chord
// arrangement is independent of exact-rational conversion and integration. It
// still charges before sketch is asked anything, because public ProfileRecord
// methods have no context and cannot cancel an arrangement that has started.
//
// Four quarter arcs are 256 chords and six are 384, so both records measure.
// One hundred quarter arcs are 6400 chords, above the 5792-chord reconstruction
// boundary, and refuse at the record-level preflight.
func TestReconstructionIsChargedBeforeItRuns(t *testing.T) {
	t.Parallel()
	for _, n := range []int{4, 6} {
		area, err := scallopedDiskRecord(n).Area()
		require.NoError(t, err, "%d arcs are inside the reconstruction ceiling", n)
		value, err := area.Value.In(units.SquareMillimeter)
		require.NoError(t, err)
		require.InDelta(t, scallopedDiskArea(n), value, 1e-9)
	}

	start := time.Now()
	_, err := scallopedDiskRecord(100).Area()
	require.Error(t, err, "100 arcs are past the reconstruction ceiling")
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "work budget")
	require.Contains(t, err.Error(), "profile record is invalid",
		"the refusal happens before sketch reconstructs the record")
	require.Less(t, time.Since(start), 3*time.Second,
		"the charge bounds the reconstruction rather than following it")
}

// The reconstruction charge is the RECORD's, and the arrangement it pays for is
// GLOBAL: sketch chords every source in the scene and then tests every pair of
// chords in one loop. A charge summing per-source squares therefore drops every
// cross-source pair, which is nearly all of them — and it undercounts each
// source too, because sketch floors free-form sampling at 64 chords however few
// control points a curve holds.
//
// One hundred three-control closed splines hold 6400 chords, past the
// reconstruction ceiling. A per-source charge misses the cross-source pairs;
// the record-wide charge refuses before sketch starts its arrangement.
//
// The fixture's own topology is never reached, which is the point: the charge is
// levied before sketch is asked anything.
func TestCrossSourceChordsAreChargedOnTheWholeRecord(t *testing.T) {
	t.Parallel()
	segments := make([]decad.CurveSegment, 100)
	for i := range segments {
		control := make([]decad.Point2, 3)
		for j := range control {
			angle := 2 * math.Pi * float64(j) / 3
			control[j] = decad.Point2{U: float64(i)*20 + 3*math.Cos(angle), V: 3 * math.Sin(angle)}
		}
		segments[i] = decad.ClosedSplineSeg{Control: control, CCW: true, TStart: 0, TEnd: 1}
	}
	record := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: segments}}

	start := time.Now()
	_, err := record.Area()
	require.Error(t, err)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "work budget")
	require.Contains(t, err.Error(), "profile record is invalid",
		"the record-level charge fires, not a per-segment one")
	require.Less(t, time.Since(start), time.Second, "no arrangement ran before the refusal")
}

// Analytic sources are chorded and arranged beside the free-form ones, so they
// have to be counted. A chord total that skips them bounds nothing about the
// pass they are arranged in: a hundred quarter arcs contribute 6400 chords to
// the same global pair loop one four-control spline contributes 64 to, and a
// charge reading only the spline admits the record and then spends seconds
// arranging all of it.
func TestAnalyticChordsAreCharged(t *testing.T) {
	t.Parallel()
	segments := make([]decad.CurveSegment, 0, 101)
	for i := range 100 {
		center := decad.Point2{U: float64(i) * 10}
		segments = append(segments, decad.ArcSeg{
			Center: center,
			Start:  decad.Point2{U: center.U + 4, V: center.V},
			End:    decad.Point2{U: center.U, V: center.V + 4},
			TStart: 0, TEnd: 1,
		})
	}
	segments = append(segments, decad.SplineSeg{
		Control: []decad.Point2{{}, {U: 1, V: 1}, {U: 2, V: 1}, {U: 3}},
		TStart:  0, TEnd: 1,
	})
	record := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: segments}}

	start := time.Now()
	_, err := record.Area()
	require.Error(t, err)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "work budget")
	require.Contains(t, err.Error(), "profile record is invalid")
	require.Less(t, time.Since(start), time.Second, "no arrangement ran before the refusal")
}

// Plan storage is the preflight's OWN state, so its size has to follow the
// segments that actually converted and never the segment count an untrusted
// record states. Storage sized by the recorded loop length is levied ahead of
// the first per-segment charge — the one thing that can refuse the record — and
// it is levied on every record, analytic ones included: a freeformPlan is 32
// bytes, so a 262,144-segment record paid 8.00 MiB for plan storage on top of
// the 4.00 MiB its own normalized segments cost, and a pure line record paid all
// of it for plans it can never read.
//
// Both readings are marginal costs per recorded segment, which is what tells
// storage that follows the loop length from storage that follows the
// conversions. The converted spline chains are a fixed cost here — the ceiling
// refuses this record at loop 0 segment 942, whatever its length past that — so
// a per-segment plan allocation is the only term either measurement can grow by.
//
// This test stays SERIAL: it measures process-wide allocation, which any
// test running alongside it would inflate. Adding t.Parallel here makes its
// reading meaningless rather than making it fail loudly.
func TestPlanStorageFollowsConvertedSegments(t *testing.T) {
	// One recorded segment costs 16 bytes of normalized CurveSegment, which is
	// the whole marginal cost the preflight owes. The slack covers the map
	// growth of the fixed 942 converted plans and the counters beside them.
	const perSegmentAllowance = 24

	for _, tc := range []struct {
		name    string
		segment func(index int) decad.CurveSegment
	}{
		{
			// A record of minimal cubic splines: every segment converts, and
			// the record's work ceiling refuses it partway through.
			name: "free-form",
			segment: func(index int) decad.CurveSegment {
				base := float64(index)
				return decad.SplineSeg{
					Control: []decad.Point2{
						{U: base},
						{U: base + 1, V: 1},
						{U: base + 2, V: 1},
						{U: base + 3},
					},
					TStart: 0, TEnd: 1,
				}
			},
		},
		{
			// A pure analytic record converts nothing at all, so it must pay
			// exactly what an evaluator with no free-form path would: its own
			// normalized segments and not one byte of plan storage. Its first
			// segment is degenerate, so the refusal lands before any walk is
			// integrated and the reading is the preflight's allocation alone.
			name: "analytic",
			segment: func(int) decad.CurveSegment {
				return decad.LineSeg{TStart: 0, TEnd: 1}
			},
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			// 262,144 is what recipe_decode.go admits per document, so it is
			// the largest loop the only untrusted channel can state.
			const small = 65536
			const large = 262144

			recordOf := func(segments int) decad.ProfileRecord {
				out := make([]decad.CurveSegment, segments)
				for i := range out {
					out[i] = tc.segment(i)
				}
				return decad.ProfileRecord{Outer: decad.LoopRecord{Segments: out}}
			}

			measure := func(segments int) uint64 {
				record := recordOf(segments)
				var err error
				allocated := allocatedBy(func() { _, err = record.Area() })
				require.Error(t, err, "neither fixture is a measurable region")
				return allocated
			}

			marginal := float64(measure(large)-measure(small)) / float64(large-small)
			require.Less(t, marginal, float64(perSegmentAllowance),
				"a recorded segment the preflight converts no plan for owes no plan storage")
		})
	}
}

// A spline chained with a straight chord exercises the mixed path: the line
// contributes through the existing exact-rational line formulas and the spline
// through the Bézier integrals, into one region.
func TestSplineAndChordProfileMoments(t *testing.T) {
	t.Parallel()
	record := recordSplineAndChord(t)

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

// An OPEN spline's knot vector is geom.ClampedKnots's own floats, and its interior
// knots are float64(j)/float64(n−3) — NOT the exact rationals j/(n−3). Where n−3
// is not a power of two the two differ, and the difference is the whole
// exactness claim: over the rational knots this six-control hump's area is
// −543/32, which IS representable, so the conversion published Exact with a zero
// bound; over the knots sketch really holds it is not representable at all and
// sits about 1.53e-17 mm² away.
//
// So the honest reading is Approximate with that single rounding as its bound.
// This fixture is a public one — recorded through RecordProfile — so it is the
// end-to-end guard: a converter that re-derives the knots republishes the Exact.
func TestOpenSplineAreaRoundsOverSketchKnots(t *testing.T) {
	t.Parallel()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	coords := [][2]float64{{0, 0}, {1, 3}, {2, 4}, {4, 4}, {5, 3}, {6, 0}}
	points := make([]*sketch.Point, len(coords))
	for i, coord := range coords {
		points[i] = s.CreatePoint(coord[0], coord[1])
	}
	_, err = s.CreateSpline(points...)
	require.NoError(t, err)
	s.CreateLine(points[len(points)-1], points[0])

	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)
	record, _, err := decad.RecordProfile(s, profiles[0])
	require.NoError(t, err)
	require.Len(t, record.Outer.Segments, 2)

	// Six control points put n−3 at 3, which is one of the affected counts: 4, 5,
	// 7 and 11 controls have a power-of-two span count and cannot see this.
	spline, ok := record.Outer.Segments[0].(decad.SplineSeg)
	if !ok {
		spline, ok = record.Outer.Segments[1].(decad.SplineSeg)
	}
	require.True(t, ok, "the loop carries the recorded spline")
	require.Len(t, spline.Control, 6)

	area, err := record.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, area.Exactness,
		"the area over sketch's own float knots is not representable")

	value, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	bound, err := area.Bound.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.Positive(t, bound, "a rational that is not representable rounds, and the bound is that rounding")
	require.LessOrEqual(t, bound, (math.Nextafter(value, math.Inf(1))-value)/2,
		"the bound is one rounding, never an estimate")

	// The dense-sample falsifier over sketch's own sampler: the hump closed by its
	// chord. It disproves a wrong magnitude; it never blesses this one.
	ring, err := geom.SampleCubicBSpline(coords, 200000)
	require.NoError(t, err)
	var twice float64
	for i := 0; i+1 < len(ring); i++ {
		twice += ring[i][0]*ring[i+1][1] - ring[i+1][0]*ring[i][1]
	}
	twice += ring[len(ring)-1][0]*ring[0][1] - ring[0][0]*ring[len(ring)-1][1]
	require.InDelta(t, math.Abs(twice/2), value, 1e-6)
}

func TestFreeformProfileRefusals(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name    string
		segment decad.CurveSegment
		message string
	}{
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

// This test stays SERIAL: it measures process-wide allocation, which any
// test running alongside it would inflate. Adding t.Parallel here makes its
// reading meaningless rather than making it fail loudly.
// A caller can hand a degree-1 NURBS segment millions of control points and no
// knots at all. Whether the knot count can match the control count is an O(1)
// question about SIZES, so it must be answered before the control array is
// read: the point scan formats a description per point, so a record that cannot
// be well formed at ANY content used to cost three allocations and about 90ns
// per control point before its refusal — 12 million allocations on a
// four-million-point record, and none of it charged against the work ceiling.
//
// This test stays SERIAL: it measures process-wide allocation, which any
// test running alongside it would inflate. Adding t.Parallel here makes its
// reading meaningless rather than making it fail loudly.
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
// own. Two closed splines of 700 controls are individually affordable — 765,800
// charged units each against a ceiling of 1,048,576 — and unaffordable together,
// so a counter opened per segment reads both as cheap and lets the record run to
// a topology answer instead of refusing. The refusal names the SECOND segment,
// which is the proof that the first segment's charge carried into it.
func TestFreeformWorkBudgetBoundsTheWholeRecord(t *testing.T) {
	t.Parallel()
	const controls = 700
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

// Every charge a free-form SEGMENT owes is levied at the record-level preflight,
// the re-anchoring of its converted chain among them. A 960-control closed
// spline charges 1,050,240 units — over the 1,048,576 ceiling by less than the
// 7,680 its re-anchoring contributes — so a preflight that omits that term
// admits the record and refuses from the moments pass instead. The validation
// prefix is what tells the two apart: validateMomentFields adds it to everything
// the preflight refuses, and the moments pass adds nothing.
func TestFreeformReanchoringChargeRefusesAtValidation(t *testing.T) {
	t.Parallel()
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
	t.Parallel()
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

// The recorded range's FINITENESS and its FULL-DOMAIN shape are two different
// refusals, and each has to reach the same verdict on every free-form kind.
//
// A non-finite range is a non-finite INPUT, so core §12 gives it ErrNotFinite on
// all seven kinds — reading it inside the Tier A arms alone reports it there and
// leaves the other four answering with their own kind reason instead. A trimmed
// range is the opposite: Table R states R2 and the Tier B rows unconditionally
// and carries no row for a trimmed range reaching the evaluator, so a kind
// refused for its own cause keeps reporting that cause whatever its range says.
// A FitSplineSeg carries no such unconditional refusal for the moments path —
// it is Tier A (Table F) — so it reaches this same range check instead of
// skipping it.
func TestFreeformRecordedRangeRefusals(t *testing.T) {
	t.Parallel()
	spline := func(tStart, tEnd float64) decad.ProfileRecord {
		record := recordSplineAndChord(t)
		segments := slices.Clone(record.Outer.Segments)
		for i, segment := range segments {
			seg, ok := segment.(decad.SplineSeg)
			if !ok {
				continue
			}
			// The arrangement decides which way this loop walks the spline, so the
			// requested pair is applied in the recorded walk's own sense.
			if seg.TStart > seg.TEnd {
				tStart, tEnd = tEnd, tStart
			}
			seg.TStart, seg.TEnd = tStart, tEnd
			segments[i] = seg
		}
		return decad.ProfileRecord{Outer: decad.LoopRecord{Segments: segments}}
	}
	closedSpline := func(tStart, tEnd float64) decad.ProfileRecord {
		control := make([]decad.Point2, len(closedSplineControls))
		for i, c := range closedSplineControls {
			control[i] = decad.Point2{U: c[0], V: c[1]}
		}
		return decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
			decad.ClosedSplineSeg{Control: control, CCW: true, TStart: tStart, TEnd: tEnd},
		}}}
	}
	nurbs := func(tStart, tEnd float64) decad.ProfileRecord {
		square := []decad.Point2{{}, {U: 1}, {U: 1, V: 1}, {V: 1}}
		segments := make([]decad.CurveSegment, len(square))
		for i := range square {
			edge := nurbsEdge(square[i], square[(i+1)%len(square)])
			edge.TStart, edge.TEnd = tStart, tEnd
			segments[i] = edge
		}
		return decad.ProfileRecord{Outer: decad.LoopRecord{Segments: segments}}
	}
	single := func(build func(tStart, tEnd float64) decad.CurveSegment) func(float64, float64) decad.ProfileRecord {
		return func(tStart, tEnd float64) decad.ProfileRecord {
			return decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
				build(tStart, tEnd),
			}}}
		}
	}
	// A lone open FitSplineSeg does not close on itself the way a closed spline
	// or the NURBS square do, so — like spline above — it needs the arrangement's
	// own record of a curve-plus-chord region, with only the range overwritten.
	fitSpline := func(tStart, tEnd float64) decad.ProfileRecord {
		record := recordFitSplineAndChord(t)
		segments := slices.Clone(record.Outer.Segments)
		for i, segment := range segments {
			seg, ok := segment.(decad.FitSplineSeg)
			if !ok {
				continue
			}
			if seg.TStart > seg.TEnd {
				tStart, tEnd = tEnd, tStart
			}
			seg.TStart, seg.TEnd = tStart, tEnd
			segments[i] = seg
		}
		return decad.ProfileRecord{Outer: decad.LoopRecord{Segments: segments}}
	}
	ellipticalArc := single(func(tStart, tEnd float64) decad.CurveSegment {
		return decad.EllipticalArcSeg{
			Center: decad.Point2{}, Start: decad.Point2{U: 2}, End: decad.Point2{V: 1},
			Rx: units.Millimeters(2), Ry: units.Millimeters(1), Rotation: units.Radians(0),
			TStart: tStart, TEnd: tEnd,
		}
	})
	conic := single(func(tStart, tEnd float64) decad.CurveSegment {
		return decad.ConicSeg{
			Start: decad.Point2{}, Apex: decad.Point2{U: 1, V: 1}, End: decad.Point2{U: 2},
			Rho: 0.4, TStart: tStart, TEnd: tEnd,
		}
	})
	ellipse := single(func(tStart, tEnd float64) decad.CurveSegment {
		return decad.EllipseSeg{
			Center: decad.Point2{}, Rx: units.Millimeters(2), Ry: units.Millimeters(1),
			Rotation: units.Radians(0), CCW: true, TStart: tStart, TEnd: tEnd,
		}
	})

	for _, tc := range []struct {
		name string
		of   func(tStart, tEnd float64) decad.ProfileRecord
		// fullSentinel is nil where the kind measures over its full domain.
		fullSentinel error
		fullMessage  string
		// trimmedMessage names the cause that wins over the trimmed range.
		trimmedMessage string
	}{
		{name: "spline", of: spline, trimmedMessage: "full domain"},
		{name: "closed spline", of: closedSpline, trimmedMessage: "full domain"},
		{name: "NURBS", of: nurbs, trimmedMessage: "full domain"},
		{name: "fit spline", of: fitSpline, trimmedMessage: "full domain"},
		{
			name: "elliptical arc", of: ellipticalArc,
			fullSentinel: decad.ErrUnsupported, fullMessage: "pinned endpoints",
			trimmedMessage: "pinned endpoints",
		},
		{
			name: "conic", of: conic,
			fullSentinel: decad.ErrUnsupported, fullMessage: "no closed form",
			trimmedMessage: "no closed form",
		},
		{
			name: "ellipse", of: ellipse,
			fullSentinel: decad.ErrUnsupported, fullMessage: "no closed form",
			trimmedMessage: "no closed form",
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			// Every cell is asserted twice: on the caller-built record, and on the
			// same record decoded from its own wire form, so the codec cannot smuggle
			// a different range in.
			measure := func(t *testing.T, record decad.ProfileRecord) error {
				t.Helper()
				_, err := record.Area()
				return err
			}
			decoded := func(t *testing.T, record decad.ProfileRecord) (decad.ProfileRecord, bool) {
				t.Helper()
				encoded, err := json.Marshal(record)
				if err != nil {
					return decad.ProfileRecord{}, false
				}
				var out decad.ProfileRecord
				require.NoError(t, json.Unmarshal(encoded, &out))
				return out, true
			}

			t.Run("full", func(t *testing.T) {
				err := measure(t, tc.of(0, 1))
				if tc.fullSentinel == nil {
					require.NoError(t, err, "a full recorded domain measures")
				} else {
					require.ErrorIs(t, err, tc.fullSentinel)
					require.Contains(t, err.Error(), tc.fullMessage)
				}

				record, ok := decoded(t, tc.of(0, 1))
				require.True(t, ok, "a full range encodes")
				err = measure(t, record)
				if tc.fullSentinel == nil {
					require.NoError(t, err)
				} else {
					require.ErrorIs(t, err, tc.fullSentinel)
				}

				// [1, 0] is the other full domain (spline design §2), so the range
				// gate admits it too. What a reversed OUTER loop then fails on is its
				// own negative area, never a range refusal.
				if err := measure(t, tc.of(1, 0)); err != nil {
					require.NotErrorIs(t, err, decad.ErrNotFinite)
					require.NotContains(t, err.Error(), "full domain")
				}
			})

			t.Run("trimmed", func(t *testing.T) {
				err := measure(t, tc.of(0.25, 0.75))
				require.ErrorIs(t, err, decad.ErrUnsupported)
				require.NotErrorIs(t, err, decad.ErrNotFinite)
				require.Contains(t, err.Error(), tc.trimmedMessage,
					"the kind's own cause wins over the trimmed range")

				record, ok := decoded(t, tc.of(0.25, 0.75))
				require.True(t, ok, "a trimmed range encodes")
				err = measure(t, record)
				require.ErrorIs(t, err, decad.ErrUnsupported)
				require.Contains(t, err.Error(), tc.trimmedMessage)
			})

			t.Run("non-finite", func(t *testing.T) {
				for _, record := range []decad.ProfileRecord{
					tc.of(math.NaN(), 1),
					tc.of(0, math.Inf(1)),
					tc.of(math.Inf(-1), math.NaN()),
				} {
					err := measure(t, record)
					require.ErrorIs(t, err, decad.ErrNotFinite,
						"a non-finite range is a non-finite input on every free-form kind")
					require.NotErrorIs(t, err, decad.ErrUnsupported)
					require.Contains(t, err.Error(), "not finite")

					// No decoded recipe can present this cell: JSON has no NaN and no
					// infinity, so the wire form cannot carry one either way.
					_, ok := decoded(t, record)
					require.False(t, ok, "a non-finite range has no wire form")
				}
			})
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
	t.Parallel()
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
	t.Parallel()
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

// The same underflowing section still has a CENTROID, and the accumulator that
// proved its area positive is what holds it: the exact first moments divided by
// the exact area are 2e-200 and 2.194...e-200, both perfectly representable.
// Dividing the PUBLISHED floats instead reads a zero area with a subnormal bound
// and refuses an answer already in hand.
func TestUnderflowingSplineCentroidDividesExactly(t *testing.T) {
	t.Parallel()
	const scale = 1e-200
	control := make([]decad.Point2, len(closedSplineControls))
	for i, c := range closedSplineControls {
		control[i] = decad.Point2{U: c[0] * scale, V: c[1] * scale}
	}
	record := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
		decad.ClosedSplineSeg{Control: control, CCW: true, TStart: 0, TEnd: 1},
	}}}

	area, err := record.Area()
	require.NoError(t, err)
	value, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.Zero(t, value, "the fixture's own area underflows, which is what makes the guard fire")

	centroid, err := record.Centroid()
	require.NoError(t, err, "the exact area is strictly positive, so the centroid exists")
	require.InDelta(t, 2*scale, centroid.Value.X, 1e-215, "∫u dA / A is 2, scaled")
	require.InDelta(t, 2.1941383606912614*scale, centroid.Value.Y, 1e-215)
	require.Zero(t, centroid.Value.Z)

	// The unit section's own centroid is the same coordinates at scale 1, so the
	// scaled reading is not a coincidence of the underflow.
	unit, err := recordClosedSplineFrom(t, closedSplineControls).Centroid()
	require.NoError(t, err)
	require.InDelta(t, unit.Value.X*scale, centroid.Value.X, 1e-215)
	require.InDelta(t, unit.Value.Y*scale, centroid.Value.Y, 1e-215)
}

// A centroid taken over the region's own rationals is rounded ONCE, so it obeys
// the same spline design §3 rule the moments do: Exact with a zero bound exactly
// when the quotient is representable, Approximate with a single rounding when it
// is not. A unit square's centroid is (1/2, 1/2) and representable; the
// closed-spline section's v is 293·.../… and is not.
func TestFreeformCentroidRoundsOnce(t *testing.T) {
	t.Parallel()
	square := []decad.Point2{{}, {U: 1}, {U: 1, V: 1}, {V: 1}}
	segments := make([]decad.CurveSegment, len(square))
	for i := range square {
		segments[i] = nurbsEdge(square[i], square[(i+1)%len(square)])
	}
	exactCentroid, err := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: segments}}.Centroid()
	require.NoError(t, err)
	require.Equal(t, 0.5, exactCentroid.Value.X)
	require.Equal(t, 0.5, exactCentroid.Value.Y)
	require.Equal(t, decad.Exact, exactCentroid.Exactness, "(1/2, 1/2) is representable")
	bound, err := exactCentroid.Bound.In(units.Millimeter)
	require.NoError(t, err)
	require.Zero(t, bound)

	approximate, err := recordClosedSplineFrom(t, closedSplineControls).Centroid()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, approximate.Exactness)
	bound, err = approximate.Bound.In(units.Millimeter)
	require.NoError(t, err)
	require.Positive(t, bound)

	// Each coordinate is one rounding — at most half an ulp — and the two are
	// combined into a plane distance by the sqrt(2) enclosure, so the whole bound
	// cannot exceed sqrt(2)/2 of an ulp of the larger coordinate. A bound
	// accumulated through a float division of already-bounded moments is wider
	// than that, because it carries the area's own rounding into the quotient.
	largest := math.Max(math.Abs(approximate.Value.X), math.Abs(approximate.Value.Y))
	halfUlp := (math.Nextafter(largest, math.Inf(1)) - largest) / 2
	const sqrt2Up = 1.4142135623730952
	require.LessOrEqual(t, bound, sqrt2Up*halfUlp,
		"the bound is a single rounding per coordinate, not an accumulated interval")
}

// A caller-built record whose spline control points all coincide states no
// boundary. It must refuse rather than integrate a zero-area region into a
// centroid division.
func TestDegenerateSplineRecordRefuses(t *testing.T) {
	t.Parallel()
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

// A closed-spline profile's moments now answer AND its side face now builds
// (docs/spline-design.md §10 P4b, Table R row R6 retired): Extrude reaches the
// walk-kind discriminant's free-form arm instead of refusing there, and the
// resulting solid's Volume is the Tier A rational times the sweep height —
// the same exact area TestClosedSplineExactArea proves, composed through the
// unchanged height arithmetic every straight-walled prism already uses.
func TestExtrudeClosedSplineProfileBuilds(t *testing.T) {
	t.Parallel()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	points := make([]*sketch.Point, len(closedSplineControls))
	for i, control := range closedSplineControls {
		points[i] = s.CreatePoint(control[0], control[1])
	}
	_, err = s.CreateClosedSpline(points...)
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)

	record, _, err := decad.RecordProfile(s, profiles[0])
	require.NoError(t, err)
	area, err := record.Area()
	require.NoError(t, err)

	d := decad.New()
	body, err := d.Extrude(s, profiles[0], &decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	require.NotNil(t, body)
	require.NotEmpty(t, d.Bodies(), "a built extrude registers a body")
	require.NotEmpty(t, d.Recipe().Steps, "a built extrude records a step")

	volume, err := body.Volume()
	require.NoError(t, err)
	require.InDelta(t, area.Value.Mag()*10, volume.Value.Mag(), 1e-9)
	// 293/18 is not representable in float64 (spline design §3), so the region
	// area itself is Approximate — the volume it feeds inherits that bound
	// even though the height (10 mm) is exact.
	require.Greater(t, volume.Bound.Mag(), 0.0, "the region area's own rounding is never zero here")
}

// This test stays SERIAL: it measures process-wide allocation, which any
// test running alongside it would inflate. Adding t.Parallel here makes its
// reading meaningless rather than making it fail loudly.
// One public operation over one record spends ONE R7 work ceiling
// (docs/spline-design.md §5.2). Extrude runs the record-wide moments preflight
// for its area falsifier, the same preflight again inside the prism build, and
// then reaches the free-form side face's own build machinery — the length
// bracket, the directional-extreme bracket and the convexity certificate all
// draw on the SAME counter the two preflights already spent from. This
// 45-control fixture (the widest single Tier A segment the record preflight
// still admits) now runs real subdivision work inside the length bracket
// before refusing on the R7 ceiling, rather than refusing at a pre-gate —
// that pre-gate is gone (§10 P4b retires Table R row R6). The two preflights
// are phases of one operation over one record, so they share the record's
// counter; the internal evaluator test keeps that counter propagation covered
// directly.
//
// The allocation assertion is what still proves "one ceiling, not two": a
// SECOND ceiling minted for the side-face build would let the length bracket
// spend a full budget of its own on top of what the two preflights already
// spent, which measured 1.51 GB on this same fixture. Wall-clock time is
// NOT asserted here any more — this test does real subdivision work before
// refusing, so its elapsed time is real and scales with the machine running
// it, never a signal a second ceiling was minted; the allocation bound is
// the measure that still discriminates the two cases.
//
// This test stays SERIAL: it measures process-wide allocation, which any
// test running alongside it would inflate. Adding t.Parallel here makes its
// reading meaningless rather than making it fail loudly.
func TestExtrudeSplineProfileSpendsOneWorkCeiling(t *testing.T) {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	// An open cubic arch of 45 control points, closed by its chord: the widest
	// single Tier A segment the record preflight still admits, which is what
	// leaves the second phase almost no budget of the record's own.
	const controls = 45
	points := make([]*sketch.Point, controls)
	for i := range points {
		a := math.Pi * float64(i) / float64(controls-1)
		points[i] = s.CreatePoint(10*math.Cos(a), 10*math.Sin(a))
	}
	_, err = s.CreateSpline(points...)
	require.NoError(t, err)
	s.CreateLine(points[len(points)-1], points[0])
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)

	d := decad.New()
	var body *decad.Body
	allocated := allocatedBy(func() {
		body, err = d.Extrude(s, profiles[0], &decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	})

	require.Error(t, err)
	require.Nil(t, body)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "free-form")
	require.Empty(t, d.Bodies(), "a refused extrude registers no body")
	require.Empty(t, d.Recipe().Steps, "a refused extrude records no step")

	// A per-phase ceiling ran the whole arc-length bracket over this chain after
	// two full preflights had already run: 1.51 GB measured. One ceiling
	// refuses the moment the record's own budget is gone; a second one would
	// let this same bracket allocate gigabytes more.
	require.Less(t, allocated, uint64(1)<<30,
		"a second ceiling would allocate gigabytes over a record already found unaffordable")
}

// scallopedDiskRecord is a closed analytic loop of n quarter-turn arcs: the
// vertices of a regular n-gon of radius 10, joined by arcs that bulge outward.
// Each arc sweeps exactly a quarter turn, so sketch chords it 64 times and the
// record's chord total is 64n whatever the arcs enclose. Nothing in it is
// free-form.
func scallopedDiskRecord(n int) decad.ProfileRecord {
	vertex := func(i int) decad.Point2 {
		angle := 2 * math.Pi * float64(i) / float64(n)
		return decad.Point2{U: 10 * math.Cos(angle), V: 10 * math.Sin(angle)}
	}
	segments := make([]decad.CurveSegment, n)
	for i := range segments {
		start, end := vertex(i), vertex((i+1)%n)
		middle := decad.Point2{U: (start.U + end.U) / 2, V: (start.V + end.V) / 2}
		// The centre sits inward of the chord by half the chord's length, which is
		// what makes the sweep from Start to End a quarter turn.
		reach := math.Hypot(end.U-start.U, end.V-start.V) / 2
		outward := math.Hypot(middle.U, middle.V)
		segments[i] = decad.ArcSeg{
			Center: decad.Point2{
				U: middle.U - reach*middle.U/outward,
				V: middle.V - reach*middle.V/outward,
			},
			Start:  start,
			End:    end,
			TStart: 0,
			TEnd:   1,
		}
	}
	return decad.ProfileRecord{Outer: decad.LoopRecord{Segments: segments}}
}

// scallopedDiskArea is the same shape's area in closed form: the regular n-gon
// through the vertices, plus one circular segment per outward bulge.
func scallopedDiskArea(n int) float64 {
	count := float64(n)
	radius := math.Sqrt2 * 10 * math.Sin(math.Pi/count)
	polygon := 0.5 * count * 100 * math.Sin(2*math.Pi/count)
	return polygon + count*radius*radius/2*(math.Pi/2-1)
}

// An entity several segments name is ONE entity in the scene sketch arranges, so
// the charge counts it once. A circle a single crossing cuts into two fragments
// is the ordinary shape of a recorded region, and counting the fragments
// separately would square a chord total the arrangement never holds.
func TestSharedAnalyticEntityIsChargedOnce(t *testing.T) {
	t.Parallel()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(95, 30), 15)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	fragmented := 0
	for _, profile := range s.Profiles() {
		if !profile.Valid {
			continue
		}
		record, _, err := decad.RecordProfile(s, profile)
		require.NoError(t, err)
		circles := 0
		for _, segment := range record.Outer.Segments {
			if _, ok := segment.(decad.CircleSeg); ok {
				circles++
			}
		}
		if circles < 2 {
			continue
		}
		fragmented++
		area, err := record.Area()
		require.NoError(t, err, "the two fragments name one circle, which is charged once")
		got, err := area.Value.In(units.SquareMillimeter)
		require.NoError(t, err)
		require.InDelta(t, profile.Area, got, 1e-9)
	}
	require.Positive(t, fragmented, "the fixture must record a region naming one circle twice")
}

// recordPlateWithCircularHoles records the one profile that contains every
// circular hole. It uses the same solved sketch as callers do, so the regression
// covers both RecordProfile and ProfileRecord.Area.
func recordPlateWithCircularHoles(t *testing.T, holes int) (decad.ProfileRecord, float64) {
	t.Helper()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 300, 100)
	s.Fix(rect.A)
	for i := range holes {
		x := 300 * float64(i+1) / float64(holes+1)
		s.CreateCircle(s.CreatePoint(x, 50), 10)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	for _, profile := range s.Profiles() {
		if !profile.Valid {
			continue
		}
		record, _, err := decad.RecordProfile(s, profile)
		require.NoError(t, err)
		if len(record.Holes) == holes {
			return record, profile.Area
		}
	}
	require.FailNowf(t, "profile", "no profile records all %d circular holes", holes)
	return decad.ProfileRecord{}, 0
}

// The reconstruction counter must admit ordinary analytic plates with several
// bolt holes. Each whole circle contributes 256 chords, so eight holes exercise
// the record-wide charge well beyond the former two-circle boundary.
func TestAnalyticPlateWithCircularHolesArea(t *testing.T) {
	t.Parallel()
	for holes := 1; holes <= 8; holes++ {
		t.Run(strconv.Itoa(holes), func(t *testing.T) {
			record, want := recordPlateWithCircularHoles(t, holes)
			area, err := record.Area()
			require.NoError(t, err)
			got, err := area.Value.In(units.SquareMillimeter)
			require.NoError(t, err)
			require.InDelta(t, want, got, 1e-9)
		})
	}
}
