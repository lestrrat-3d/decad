package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// A recorded segment's walk states the record's own endpoints at the two
// natural bounds. Each analytic kind's parameterization runs Start → End over
// [0, 1] (record.go), so its value at t = 0 is Start and at t = 1 is End,
// exactly — while the float routes to those points (a line's chord lerp, an
// arc's atan2 plus sweep) need not land back on them.
//
// Each arm states that premise before asserting the walk, and states it where
// it is decidable. The line's route is a chord formula over two exact float64
// constants, and its own multiply is by one — exact whether or not the build
// contracts the multiply-add — so whether that route misses follows from
// IEEE-754 arithmetic alone and one fixture can require it. The arc's route runs
// through atan2 and cos/sin, whose last bit does differ between platforms: a
// build that contracts a multiply-add rounds once where another rounds twice, so
// whether any single arc misses is a fact about the host rather than about this
// seam. The arc arm therefore measures the premise over the family the diagnosis
// covered, on the running platform, and requires that family to hold a miss;
// that is what keeps its walk assertions non-vacuous wherever the test runs.
//
// What depends on it: buildPrismScene (prism_boolean.go) creates one sketch
// point per walked endpoint, so a walk landing an ulp off the vertex two
// segments share would offer sketch two points where the record states one, and
// RecordProfile would then refuse the region the arrangement admits on its own
// proximity threshold.
func TestWholeSegmentWalkStatesTheRecordedEndpoints(t *testing.T) {
	t.Run("line", func(t *testing.T) {
		// 4/7 → 10/3 is a pair the chord formula does not round-trip: the
		// difference needs a finer grid than its own exponent carries, so it
		// rounds, and start + (end − start) then lands one ulp short of end.
		start := Point2{U: 4.0 / 7.0, V: 2}
		end := Point2{U: 10.0 / 3.0, V: 2}
		require.NotEqual(t, end.U, start.U+1.0*(end.U-start.U),
			`premise: evaluating the chord formula at t = 1 misses this endpoint`)

		u0, v0 := lerp2(start, end, 0)
		require.Equal(t, start, Point2{U: u0, V: v0})
		u1, v1 := lerp2(start, end, 1)
		require.Equal(t, end, Point2{U: u1, V: v1})

		// An interior parameter names no recorded coordinate, so the formula
		// remains what states it.
		uq, vq := lerp2(start, end, 0.25)
		require.Equal(t, start.U+0.25*(end.U-start.U), uq)
		require.Equal(t, start.V+0.25*(end.V-start.V), vq)

		w, err := walkOf(LineSeg{Start: start, End: end, TStart: 0, TEnd: 1}, nil)
		require.NoError(t, err)
		require.Equal(t,
			[4]float64{start.U, start.V, end.U, end.V},
			[4]float64{w.startU, w.startV, w.endU, w.endV})

		// A reversed whole edge records TStart = 1, TEnd = 0 (seam.go), so the
		// walk's own ends swap while each still states a recorded coordinate.
		rev, err := walkOf(LineSeg{Start: start, End: end, TStart: 1, TEnd: 0}, nil)
		require.NoError(t, err)
		require.Equal(t,
			[4]float64{end.U, end.V, start.U, start.V},
			[4]float64{rev.startU, rev.startV, rev.endU, rev.endV})
	})

	t.Run("arc", func(t *testing.T) {
		const cu, cv, r = 10.3, 9.7, 4.7
		th0, th1 := 0.05, 0.75
		seg := ArcSeg{
			Center: Point2{U: cu, V: cv},
			Start:  Point2{U: cu + r*math.Cos(th0), V: cv + r*math.Sin(th0)},
			End:    Point2{U: cu + r*math.Cos(th1), V: cv + r*math.Sin(th1)},
			TStart: 0, TEnd: 1,
		}

		w, err := walkOf(seg, nil)
		require.NoError(t, err)

		// Observation, not a requirement: whether THIS arc's own end angle
		// lands back on its recorded endpoint depends on the host's arithmetic.
		// The family below is what states the premise.
		t.Logf("this fixture's recorded arc end is %v; its walk's own end angle reaches %v",
			seg.End, walkEndFromModel(w))

		require.Equal(t,
			[4]float64{seg.Start.U, seg.Start.V, seg.End.U, seg.End.V},
			[4]float64{w.startU, w.startV, w.endU, w.endV})

		rev, err := walkOf(ArcSeg{
			Center: seg.Center, Start: seg.Start, End: seg.End,
			TStart: 1, TEnd: 0,
		}, nil)
		require.NoError(t, err)
		require.Equal(t,
			[4]float64{seg.End.U, seg.End.V, seg.Start.U, seg.Start.V},
			[4]float64{rev.startU, rev.startV, rev.endU, rev.endV})

		// A trimmed bound keeps the circular model's own value: the record
		// states no coordinate there, and this seam never invents one.
		part, err := walkOf(ArcSeg{
			Center: seg.Center, Start: seg.Start, End: seg.End,
			TStart: 0, TEnd: 0.5,
		}, nil)
		require.NoError(t, err)
		require.Equal(t, [2]float64{seg.Start.U, seg.Start.V}, [2]float64{part.startU, part.startV})
		require.Equal(t, walkEndFromModel(part), Point2{U: part.endU, V: part.endV})

		// The premise, stated over the family the diagnosis measured rather
		// than over one fixture: across these arcs at least one walk's own end
		// angle misses its recorded endpoint, so at least one of the endpoint
		// assertions this loop makes would fail if a whole segment's walk
		// answered its circular model at that bound instead of the record.
		var missed, total int
		for _, c := range []struct{ cu, cv, r float64 }{
			{10.3, 9.7, 4.7},
			{-3.35, 0.15, 1.9},
			{0.7, -12.25, 21.3},
			{1000 + 1.0/3.0, 2.0 / 7.0, 0.35},
		} {
			for i := range 5 {
				a0 := 0.05 + 0.71*float64(i)
				for j := range 4 {
					a1 := a0 + 0.13 + 0.79*float64(j)
					fam := ArcSeg{
						Center: Point2{U: c.cu, V: c.cv},
						Start:  Point2{U: c.cu + c.r*math.Cos(a0), V: c.cv + c.r*math.Sin(a0)},
						End:    Point2{U: c.cu + c.r*math.Cos(a1), V: c.cv + c.r*math.Sin(a1)},
						TStart: 0, TEnd: 1,
					}
					fw, err := walkOf(fam, nil)
					require.NoError(t, err)
					require.Equal(t,
						[4]float64{fam.Start.U, fam.Start.V, fam.End.U, fam.End.V},
						[4]float64{fw.startU, fw.startV, fw.endU, fw.endV},
						`the walk states this arc's own recorded endpoints`)
					total++
					if walkEndFromModel(fw) != fam.End {
						missed++
					}
				}
			}
		}
		t.Logf("%d of %d arcs in the family have a walk end angle that misses the recorded endpoint", missed, total)
		require.Positive(t, missed,
			`premise: this platform's arc family holds an end angle that misses its recorded endpoint`)
	})
}

// cosEighthPi and sinEighthPi are cos(π/8) and sin(π/8) to 60 significant
// digits — the values the trimmed fixtures below actually denote, computed
// outside this package from the closed forms √(2+√2)/2 and √(2−√2)/2. They are
// the TRUTH each assertion measures a published interval against, so they must
// come from arithmetic none of the code under test performs; a float64 constant
// could not state either one, since the whole defect lives in the last two bits
// of a float64.
var (
	cosEighthPi = mustRatDecimal("0.923879532511286756128183189396788286822416625863642486115097")
	sinEighthPi = mustRatDecimal("0.382683432365089771728459984030398866761344562485627041433800")
)

// requireEnclosesTruth proves a published reading covers the value the record
// denotes: the gap between the held float and the truth must not exceed the
// bound published beside it. A zero bound therefore demands an exactly held
// reading, which is the whole point of asserting it this way.
func requireEnclosesTruth(t *testing.T, held, bound float64, truth *big.Rat, what string) {
	t.Helper()
	gap := new(big.Rat).Sub(floatRat(held), truth)
	gap.Abs(gap)
	require.LessOrEqual(t, gap.Cmp(floatRat(bound)), 0,
		`%s: the held %.20f sits %s from the value the record denotes, past the published bound %g`,
		what, held, gap.FloatString(22), bound)
}

func oneSegmentProfile(seg CurveSegment) ProfileRecord {
	return ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{seg}}}
}

// A walk endpoint the record does not state verbatim is a coordinate this
// evaluator COMPUTED — a float lerp for a trimmed line, a math.Cos/math.Sin at
// a computed angle for a trimmed arc and for every circle — so the walk states
// what it is worth and the boundary-extreme scan publishes that width. Read
// along a direction the trimmed end itself attains, the scan's own interval must
// contain the value the record denotes; a zero bound there is a false claim,
// because the true endpoint sits outside the interval the scan reports.
func TestBoundaryExtremesChargeAComputedWalkEndpoint(t *testing.T) {
	// The trimmed quarter of a quarter-circle arc: the walk sweeps θ ∈
	// [π/8, 3π/8], so along (1, 0) both extremes ARE endpoints — the interior
	// apex at θ = 0 is not swept and contributes nothing.
	trimmedArc := ArcSeg{
		Center: Point2{U: 0, V: 0},
		Start:  Point2{U: 1, V: 0},
		End:    Point2{U: 0, V: 1},
		TStart: 0.25, TEnd: 0.75,
	}
	// The same two angles read off a circle instead, where no endpoint is ever
	// pinned to a recorded coordinate: 2π·0.0625 = π/8 and 2π·0.1875 = 3π/8.
	trimmedCircle := CircleSeg{
		Center: Point2{U: 0, V: 0},
		Radius: units.Millimeters(1),
		CCW:    true,
		TStart: 0.0625, TEnd: 0.1875,
	}

	for _, tc := range []struct {
		name string
		seg  CurveSegment
	}{
		{name: "arc", seg: trimmedArc},
		{name: "circle", seg: trimmedCircle},
	} {
		t.Run(tc.name, func(t *testing.T) {
			w, err := walkOf(tc.seg, nil)
			require.NoError(t, err)
			require.Positive(t, w.startBound.u, `a trimmed circular endpoint is not a recorded coordinate`)
			require.Positive(t, w.endBound.u, `a trimmed circular endpoint is not a recorded coordinate`)
			require.Less(t, w.startBound.u, 1e-15,
				`the bound is this endpoint's own displacement, not the circle's extent`)

			lo, hi, bound, err := boundaryExtremesBoundedContext(
				t.Context(), oneSegmentProfile(tc.seg), 1, 0, newFreeformWork())
			require.NoError(t, err)
			require.Positive(t, bound)
			requireEnclosesTruth(t, hi, bound, cosEighthPi, `the maximum along (1, 0)`)
			requireEnclosesTruth(t, lo, bound, sinEighthPi, `the minimum along (1, 0)`)
		})
	}

	// A trimmed LINE endpoint is the same class: lerp2 pins only t = 0 and
	// t = 1, and every other parameter is a float lerp whose exact value is the
	// rational one. 0.3·0.1 is not representable, so this endpoint is held short
	// of what the record denotes.
	t.Run("line", func(t *testing.T) {
		seg := LineSeg{
			Start:  Point2{U: 0, V: 0},
			End:    Point2{U: 0.1, V: 0.1},
			TStart: 0.3, TEnd: 1,
		}
		truth := ratLerp(seg.Start.U, seg.End.U, seg.TStart)
		require.NotEqual(t, 0, truth.Cmp(floatRat(0.3*0.1)),
			`premise: this trimmed line endpoint is not exactly representable`)

		w, err := walkOf(seg, nil)
		require.NoError(t, err)
		require.Positive(t, w.startBound.u)
		require.Equal(t, walkEndBound{}, w.endBound, `t = 1 names the recorded End`)

		lo, hi, bound, err := boundaryExtremesBoundedContext(
			t.Context(), oneSegmentProfile(seg), 1, 0, newFreeformWork())
		require.NoError(t, err)
		require.Positive(t, bound)
		requireEnclosesTruth(t, lo, bound, truth, `the minimum along (1, 0)`)
		requireEnclosesTruth(t, hi, bound, floatRat(seg.End.U), `the maximum along (1, 0)`)
	})

	// An endpoint whose displacement no arithmetic here can state refuses the
	// whole scan. Folding its +Inf into the accumulators instead would publish
	// an infinite box bound, or read as the empty region's own ErrDegenerate.
	t.Run("underivable", func(t *testing.T) {
		seg := LineSeg{
			Start:  Point2{U: math.Inf(1), V: 0},
			End:    Point2{U: 1, V: 1},
			TStart: 0.5, TEnd: 1,
		}
		w, err := walkOf(seg, nil)
		require.NoError(t, err)
		require.False(t, w.startBound.derivable())

		_, _, _, err = boundaryExtremesBoundedContext(
			t.Context(), oneSegmentProfile(seg), 1, 0, newFreeformWork())
		require.ErrorIs(t, err, ErrUnsupported)
	})
}

// The charge is levied on a COMPUTED endpoint only. Where the record states the
// endpoint verbatim — a line's own bounds, an arc's own bounds — and every other
// candidate is exactly representable, the scan keeps the zero bound and the
// section's box stays Exact.
func TestBoundaryExtremesKeepAProvenZero(t *testing.T) {
	square := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: Point2{U: 0, V: 0}, End: Point2{U: 2, V: 0}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: 2, V: 0}, End: Point2{U: 2, V: 2}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: 2, V: 2}, End: Point2{U: 0, V: 2}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: 0, V: 2}, End: Point2{U: 0, V: 0}, TStart: 0, TEnd: 1},
	}}}
	wholeArc := ArcSeg{
		Center: Point2{U: 0, V: 0},
		Start:  Point2{U: 1, V: 0},
		End:    Point2{U: 0, V: 1},
		TStart: 0, TEnd: 1,
	}
	wholeCircle := CircleSeg{
		Center: Point2{U: 0, V: 0},
		Radius: units.Millimeters(1),
		CCW:    true,
		TStart: 0, TEnd: 1,
	}

	for _, tc := range []struct {
		name    string
		profile ProfileRecord
		lo, hi  float64
	}{
		{name: "all straight", profile: square, lo: 0, hi: 2},
		{name: "whole arc", profile: oneSegmentProfile(wholeArc), lo: 0, hi: 1},
		{name: "whole circle", profile: oneSegmentProfile(wholeCircle), lo: -1, hi: 1},
	} {
		t.Run(tc.name, func(t *testing.T) {
			lo, hi, bound, err := boundaryExtremesBoundedContext(
				t.Context(), tc.profile, 1, 0, newFreeformWork())
			require.NoError(t, err)
			require.Equal(t, 0.0, bound, `every candidate here is a value the record states exactly`)
			require.Equal(t, tc.lo, lo)
			require.Equal(t, tc.hi, hi)
		})
	}

	// The whole circle keeps its zero along (1, 0) for a reason the endpoint's
	// own components state: at t = 0 the record's own quarter-turn readings are
	// exact in both, and at t = 1 the walk lands exactly on the circle's u
	// extreme while missing v — math.Cos returns 1 at that angle and math.Sin
	// does not return 0 — so a direction reading u alone charges nothing while
	// the v error is still stated rather than dropped.
	w, err := walkOf(wholeCircle, nil)
	require.NoError(t, err)
	require.Equal(t, walkEndBound{}, w.startBound)
	require.Equal(t, 0.0, w.endBound.u)
	require.Positive(t, w.endBound.v)
	require.Less(t, w.endBound.v, 1e-15)
	require.Equal(t, 0.0, pointPerturbationAllow(w.endBound, 1, 0))
	require.Positive(t, pointPerturbationAllow(w.endBound, 0, 1))
}

// walkEndFromModel re-derives a circular walk's far endpoint from the walk's OWN
// published model — its centre, radius and end angle — rather than from a second
// copy of walkOf's formula. It is the coordinate the walk would carry at that
// bound if it answered its model there instead of the record.
func walkEndFromModel(w segmentWalk) Point2 {
	sin, cos := math.Sincos(w.th1)
	return Point2{U: w.cU + w.radius*cos, V: w.cV + w.radius*sin}
}
