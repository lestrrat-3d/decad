package decad_test

import (
	"math"
	"math/big"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/geom"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/spline-design.md §11's fit-spline test obligations: a
// FitSplineSeg is a Tier A kind for the moments path, so
// ProfileRecord.Area/Centroid/SecondMoments must answer for it — matching a
// closed form or a dense sample, never merely "it ran" (CLAUDE.md's hard
// rules).

// TestFitSplineTwoPointIsExactlyALineSegment is the strongest single
// assertion available: naturalSecondDerivs returns every second derivative
// zero for fewer than 3 points, so a two-point fit spline's Bézier reduction
// is the degree-elevated straight chord — bit for bit the same curve a
// LineSeg records. A triangle with one side recorded as a two-point
// FitSplineSeg must report the identical Area, Centroid and SecondMoments as
// the same triangle recorded entirely in LineSegs, both Exact with a zero
// bound (6 mm² is representable).
func TestFitSplineTwoPointIsExactlyALineSegment(t *testing.T) {
	a := decad.Point2{U: 0, V: 0}
	b := decad.Point2{U: 4, V: 0}
	c := decad.Point2{U: 2, V: 3}

	fitRecord := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
		decad.FitSplineSeg{Fit: []decad.Point2{a, b}, TStart: 0, TEnd: 1},
		decad.LineSeg{Start: b, End: c, TStart: 0, TEnd: 1},
		decad.LineSeg{Start: c, End: a, TStart: 0, TEnd: 1},
	}}}
	lineRecord := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
		decad.LineSeg{Start: a, End: b, TStart: 0, TEnd: 1},
		decad.LineSeg{Start: b, End: c, TStart: 0, TEnd: 1},
		decad.LineSeg{Start: c, End: a, TStart: 0, TEnd: 1},
	}}}

	fitArea, err := fitRecord.Area()
	require.NoError(t, err)
	lineArea, err := lineRecord.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, fitArea.Exactness, "6 mm2 is representable")
	require.Equal(t, lineArea, fitArea, "a two-point fit spline is bit-for-bit the chord")
	value, err := fitArea.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.Equal(t, 6.0, value, "half the base times the height")

	fitCentroid, err := fitRecord.Centroid()
	require.NoError(t, err)
	lineCentroid, err := lineRecord.Centroid()
	require.NoError(t, err)
	require.Equal(t, lineCentroid, fitCentroid)

	fitMoments, err := fitRecord.SecondMoments()
	require.NoError(t, err)
	lineMoments, err := lineRecord.SecondMoments()
	require.NoError(t, err)
	require.Equal(t, lineMoments, fitMoments)
}

// TestFitSplineCollinearPointsMeasureAsTheLine extends the same closed form
// to 3 and 5 fit points sitting on one line: the tridiagonal system's
// right-hand side is zero for equally spaced collinear points and the natural
// end conditions pin the rest, so every second derivative is zero and the
// curve is exactly that line — additionally pinning that a MULTI-span chain
// joins exactly (its interior control points are the recorded points
// themselves, at zero curvature).
func TestFitSplineCollinearPointsMeasureAsTheLine(t *testing.T) {
	for _, n := range []int{3, 5} {
		t.Run("", func(t *testing.T) {
			fit := make([]decad.Point2, n)
			for i := range fit {
				fit[i] = decad.Point2{U: float64(i) * 2, V: 0}
			}
			last := fit[n-1]
			apex := decad.Point2{U: last.U / 2, V: 3}

			fitRecord := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
				decad.FitSplineSeg{Fit: fit, TStart: 0, TEnd: 1},
				decad.LineSeg{Start: last, End: apex, TStart: 0, TEnd: 1},
				decad.LineSeg{Start: apex, End: fit[0], TStart: 0, TEnd: 1},
			}}}
			lineRecord := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
				decad.LineSeg{Start: fit[0], End: last, TStart: 0, TEnd: 1},
				decad.LineSeg{Start: last, End: apex, TStart: 0, TEnd: 1},
				decad.LineSeg{Start: apex, End: fit[0], TStart: 0, TEnd: 1},
			}}}

			fitArea, err := fitRecord.Area()
			require.NoError(t, err)
			lineArea, err := lineRecord.Area()
			require.NoError(t, err)
			require.Equal(t, lineArea, fitArea)
		})
	}
}

// TestFitSplineCurvedHandComputedArea is the hand-derived curved case: fit
// points (1, 0), (0, 1), (-1, 0) — walked in this order (rather than the
// mirror-image (-1,0),(0,1),(1,0)) so the loop, closed by the chord
// (-1,0)->(1,0), is wound counter-clockwise, which is what a decad outer
// loop requires. The chords are equal (both sqrt(2)), so the tridiagonal
// system's one interior equation gives m_x = (0, 0, 0), m_y = (0, -3/2, 0) by
// hand (h = sqrt(2), the standard equal-chord natural cubic RHS
// 6*((y2-y1)/h-(y1-y0)/h) = 6*(-2/h) = -12/h solved against diag = 4h gives
// m1 = -3/(h^2) = -3/2 — unchanged by the walk direction, since reversing
// v0/v2 negates both terms of the RHS difference and so leaves it standing).
// §1.3's closed form then gives the span's Bezier control values exactly,
// and closing the curve with the chord bounds a region whose area is
// hand-derivable from those control points via the shoelace / Green's
// theorem formula over the two cubic spans.
func TestFitSplineCurvedHandComputedArea(t *testing.T) {
	fit := []decad.Point2{{U: 1, V: 0}, {U: 0, V: 1}, {U: -1, V: 0}}
	record := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
		decad.FitSplineSeg{Fit: fit, TStart: 0, TEnd: 1},
		decad.LineSeg{Start: decad.Point2{U: -1}, End: decad.Point2{U: 1}, TStart: 0, TEnd: 1},
	}}}

	area, err := record.Area()
	require.NoError(t, err)

	// Independent hand computation over big.Rat, mirroring the closed form
	// derived in the doc comment above rather than reusing fitSpanControls.
	// h itself is sqrt(2) and irrational, but h^2 is exactly 2 and every use
	// of h in the closed form is squared, so the whole computation stays
	// rational.
	hSq := big.NewRat(2, 1)
	m1 := big.NewRat(-3, 2)
	zero := big.NewRat(0, 1)
	v0x, v1x, v2x := big.NewRat(1, 1), big.NewRat(0, 1), big.NewRat(-1, 1)
	v0y, v1y, v2y := big.NewRat(0, 1), big.NewRat(1, 1), big.NewRat(0, 1)

	// b1 = (2vi+vi1)/3 - hSq*(2mi+mi1)/18, b2 = (vi+2vi1)/3 - hSq*(mi+2mi1)/18
	span := func(vi, vi1, mi, mi1 *big.Rat) (b0, b1, b2, b3 *big.Rat) {
		three := big.NewRat(3, 1)
		eighteen := big.NewRat(18, 1)
		b0 = new(big.Rat).Set(vi)
		b3 = new(big.Rat).Set(vi1)
		b1 = new(big.Rat).Quo(new(big.Rat).Add(new(big.Rat).Mul(big.NewRat(2, 1), vi), vi1), three)
		t1 := new(big.Rat).Quo(new(big.Rat).Mul(hSq, new(big.Rat).Add(new(big.Rat).Mul(big.NewRat(2, 1), mi), mi1)), eighteen)
		b1.Sub(b1, t1)
		b2 = new(big.Rat).Quo(new(big.Rat).Add(vi, new(big.Rat).Mul(big.NewRat(2, 1), vi1)), three)
		t2 := new(big.Rat).Quo(new(big.Rat).Mul(hSq, new(big.Rat).Add(mi, new(big.Rat).Mul(big.NewRat(2, 1), mi1))), eighteen)
		b2.Sub(b2, t2)
		return
	}
	x0, x1, x2, x3 := span(v0x, v1x, zero, m1)
	y0, y1, y2, y3 := span(v0y, v1y, zero, m1)
	x4, x5, x6, x7 := span(v1x, v2x, m1, zero)
	y4, y5, y6, y7 := span(v1y, v2y, m1, zero)
	require.True(t, x3.Cmp(x4) == 0 && y3.Cmp(y4) == 0, "consecutive spans share their boundary control point")

	// Green's theorem area over a cubic Bezier span: (1/2) * integral (x dy - y dx),
	// which for one span's Bernstein-form coefficients is a fixed exact-rational
	// combination of the eight control coordinates. Rather than re-derive that
	// combination by hand a second time, cross-check by DENSE SAMPLING (an
	// independent oracle) instead of asserting a literal numeric answer here.
	toF := func(r *big.Rat) float64 { f, _ := r.Float64(); return f }
	span1 := [][2]float64{{toF(x0), toF(y0)}, {toF(x1), toF(y1)}, {toF(x2), toF(y2)}, {toF(x3), toF(y3)}}
	span2 := [][2]float64{{toF(x4), toF(y4)}, {toF(x5), toF(y5)}, {toF(x6), toF(y6)}, {toF(x7), toF(y7)}}
	sample := func(span [][2]float64, n int) [][2]float64 {
		out := make([][2]float64, n+1)
		for i := 0; i <= n; i++ {
			u := float64(i) / float64(n)
			p := [4]float64{
				(1 - u) * (1 - u) * (1 - u),
				3 * (1 - u) * (1 - u) * u,
				3 * (1 - u) * u * u,
				u * u * u,
			}
			var x, y float64
			for k, c := range span {
				x += p[k] * c[0]
				y += p[k] * c[1]
			}
			out[i] = [2]float64{x, y}
		}
		return out
	}
	// The ring runs from (-1,0) through the two spans to (1,0); wrapping the
	// shoelace sum's last index back to its first is exactly the chord
	// (1,0) -> (-1,0) that closes the loop, so no separate closing term is
	// needed.
	ring := append(sample(span1, 100000), sample(span2, 100000)[1:]...)
	var twice float64
	n := len(ring)
	for i := range n {
		j := (i + 1) % n
		twice += ring[i][0]*ring[j][1] - ring[j][0]*ring[i][1]
	}
	dense := math.Abs(twice / 2)

	value, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.InDelta(t, dense, value, 1e-4)
	require.Positive(t, value)

	// h being sqrt(2) is not itself rational, but h^2 is exact — the only place
	// h enters this construction is squared, so the WHOLE reduction is over
	// exact rationals and the result must be Approximate with a tiny one-rounding
	// bound (spline design §3), never a quadrature-sized one.
	require.Equal(t, decad.Approximate, area.Exactness)
	bound, err := area.Bound.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.Positive(t, bound)
	require.Less(t, bound, 1e-9)
}

// TestFitSplineMatchesDenseSampleAndSketchArea is the §5.2 falsifier: decad's
// exact rational integral is checked against a dense sample over the SAME
// sketch entity AND against sketch's own independent Profile.Area. Both
// disprove a wrong magnitude; neither is the proof itself.
func TestFitSplineMatchesDenseSampleAndSketchArea(t *testing.T) {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	pts := []*sketch.Point{
		s.CreatePoint(0, 0), s.CreatePoint(3, 4), s.CreatePoint(6, 1), s.CreatePoint(9, 5), s.CreatePoint(12, 0),
	}
	_, err = s.CreateFitSpline(pts...)
	require.NoError(t, err)
	s.CreateLine(pts[len(pts)-1], pts[0])

	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)
	sketchArea := profiles[0].Area

	record, _, err := decad.RecordProfile(s, profiles[0])
	require.NoError(t, err)

	area, err := record.Area()
	require.NoError(t, err)
	value, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)

	require.InDelta(t, sketchArea, value, 1e-6, "sketch's own independent area agrees")

	// Dense sample over the SAME fit points, through geom's own evaluator.
	coords := make([][2]float64, len(pts))
	for i, p := range pts {
		coords[i] = [2]float64{p.X(), p.Y()}
	}
	ring, err := geom.SampleFitSpline(coords, 400000)
	require.NoError(t, err)
	var twice float64
	n := len(ring)
	for i := range n {
		j := (i + 1) % n
		twice += ring[i][0]*ring[j][1] - ring[j][0]*ring[i][1]
	}
	require.InDelta(t, math.Abs(twice/2), value, 1e-4)
}

// TestFitSplineInteriorDuplicateFitPointIsTransparent asserts finding #2's
// dedup consequence in the direction a controlled fixture can pin without
// depending on sketch's own arrangement tolerance: decad reads the fit
// spline's ACTIVE (deduplicated) points, never the raw recorded Fit slice.
// A record whose Fit holds a consecutive coincident INTERIOR pair integrates
// the identical active curve as the same section recorded without the
// duplicate. Walked (10,0)->(5,5)->(0,0) — rather than the reverse — so the
// loop, closed by the chord (0,0)->(10,0), winds counter-clockwise.
func TestFitSplineInteriorDuplicateFitPointIsTransparent(t *testing.T) {
	plain := []decad.Point2{{U: 10}, {U: 5, V: 5}, {U: 0}}
	duplicated := []decad.Point2{{U: 10}, {U: 5, V: 5}, {U: 5, V: 5 + 3e-13}, {U: 0}}

	recordOf := func(fit []decad.Point2) decad.ProfileRecord {
		return decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
			decad.FitSplineSeg{Fit: fit, TStart: 0, TEnd: 1},
			decad.LineSeg{Start: fit[len(fit)-1], End: fit[0], TStart: 0, TEnd: 1},
		}}}
	}

	plainArea, err := recordOf(plain).Area()
	require.NoError(t, err)
	dupArea, err := recordOf(duplicated).Area()
	require.NoError(t, err)

	plainValue, err := plainArea.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	dupValue, err := dupArea.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.Equal(t, plainValue, dupValue, "the duplicated interior point is collapsed away before integration")
	require.Equal(t, plainArea.Exactness, dupArea.Exactness)
}

// TestFitSplineAreaRoundingRuleBothSides pins spline design §3's rounding
// rule on both sides for a Tier A fit spline: a section whose exact area is
// not representable in the reported magnitude is Approximate with a bound of
// one rounding, and a section whose exact area IS representable there is
// Exact with a zero bound. One side alone cannot tell the rule from a
// constant, so this asserts both explicitly rather than trusting the two
// separate tests above to be read together.
//
// A STRAIGHT-line-equivalent fit spline (two fit points, or collinear ones)
// reduces to the ordinary polygon shoelace formula (Table F's degree-elevation
// identity, TestFitSplineTwoPointIsExactlyALineSegment), which over integer
// coordinates is always exactly representable. A genuinely curved fit
// spline's second derivatives are whatever floats geom's own tridiagonal
// solve happens to produce (TestFitSplineCurvedHandComputedArea), which no
// hand-picked coordinate can aim at a representable rational by closed-form
// reasoning — every curved fixture tried for this test landed Approximate,
// consistent with §3's "never describe a Tier A reading as unconditionally
// Exact" and with the same standing an irrational-denominator closed-spline
// area already has.
func TestFitSplineAreaRoundingRuleBothSides(t *testing.T) {
	exactRecord := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
		decad.FitSplineSeg{Fit: []decad.Point2{{U: 0, V: 0}, {U: 4, V: 0}}, TStart: 0, TEnd: 1},
		decad.LineSeg{Start: decad.Point2{U: 4}, End: decad.Point2{U: 2, V: 3}, TStart: 0, TEnd: 1},
		decad.LineSeg{Start: decad.Point2{U: 2, V: 3}, End: decad.Point2{}, TStart: 0, TEnd: 1},
	}}}
	exact, err := exactRecord.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, exact.Exactness, "6 mm2 is representable")
	exactBound, err := exact.Bound.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.Zero(t, exactBound)

	approxRecord := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
		decad.FitSplineSeg{
			Fit:    []decad.Point2{{U: 1, V: 0}, {U: 0, V: 1}, {U: -1, V: 0}},
			TStart: 0, TEnd: 1,
		},
		decad.LineSeg{Start: decad.Point2{U: -1}, End: decad.Point2{U: 1}, TStart: 0, TEnd: 1},
	}}}
	approx, err := approxRecord.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, approx.Exactness,
		"this curved fixture's exact rational area is not representable in float64")
	approxBound, err := approx.Bound.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.Positive(t, approxBound)
}

// TestFitSplineAllCoincidentFitPointsRefuses is Table R row R14: a fit
// spline whose control net collapses to a single point contributes no
// boundary, on the same terms the length bracket refuses on
// (spline_length.go's freeformArcLength / freeformDegenerate).
func TestFitSplineAllCoincidentFitPointsRefuses(t *testing.T) {
	fit := []decad.Point2{{U: 5, V: 5}, {U: 5, V: 5}, {U: 5, V: 5}}
	record := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
		decad.FitSplineSeg{Fit: fit, TStart: 0, TEnd: 1},
		decad.LineSeg{Start: decad.Point2{U: 5, V: 5}, End: decad.Point2{}, TStart: 0, TEnd: 1},
	}}}
	_, err := record.Area()
	require.Error(t, err)
	require.ErrorIs(t, err, decad.ErrDegenerate)
}

// TestFitSplineNonFiniteInterpolantIsUnsupported is Table R row R16: finite
// fit coordinates whose cumulative chord length or span coefficients leave
// float64 range are ErrUnsupported (the curve exists; this evaluator cannot
// describe it), never ErrNotFinite (whose subject is a non-finite INPUT —
// every coordinate here is finite).
func TestFitSplineNonFiniteInterpolantIsUnsupported(t *testing.T) {
	fit := []decad.Point2{{U: -1e308, V: 0}, {U: 1e308, V: 1}}
	record := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
		decad.FitSplineSeg{Fit: fit, TStart: 0, TEnd: 1},
		decad.LineSeg{Start: decad.Point2{U: 1e308, V: 1}, End: decad.Point2{U: -1e308, V: 0}, TStart: 0, TEnd: 1},
	}}}
	_, err := record.Area()
	require.Error(t, err)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.NotErrorIs(t, err, decad.ErrNotFinite)
	require.Contains(t, err.Error(), "float64 range")
}

// TestFitSplineNonFiniteFitPointIsErrNotFinite is the review-128-3 regression:
// a caller-built FitSplineSeg never passes through record.go's
// validateSegmentPoints (that gate runs only at JSON decode), so a NaN or
// infinite raw Fit coordinate used to reach geom.NewFitInterpolant unchecked
// and come back as R16's ErrUnsupported — a row whose own text asserts the
// fit points are finite, answering the one input that disproves it. Every
// position is checked, not just the first: a NaN in a non-first slot used to
// survive sketch's own terminal dedup and take a different, still-wrong path
// (ErrDegenerate from naturalSecondDerivs). All three shapes must now answer
// ErrNotFinite, decided ahead of R16's row.
func TestFitSplineNonFiniteFitPointIsErrNotFinite(t *testing.T) {
	closeLoop := func(fit []decad.Point2) decad.ProfileRecord {
		last := fit[len(fit)-1]
		return decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
			decad.FitSplineSeg{Fit: fit, TStart: 0, TEnd: 1},
			decad.LineSeg{Start: last, End: fit[0], TStart: 0, TEnd: 1},
		}}}
	}

	testCases := []struct {
		name string
		fit  []decad.Point2
	}{
		{
			name: "NaN in first position",
			fit:  []decad.Point2{{U: math.NaN(), V: 0}, {U: 1, V: 1}},
		},
		{
			name: "NaN in a later position",
			fit:  []decad.Point2{{U: 0, V: 0}, {U: 1, V: 1}, {U: math.NaN(), V: 2}},
		},
		{
			name: "infinite coordinate",
			fit:  []decad.Point2{{U: math.Inf(1), V: 0}, {U: 1, V: 1}},
		},
	}
	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {
			_, err := closeLoop(tc.fit).Area()
			require.Error(t, err)
			require.ErrorIs(t, err, decad.ErrNotFinite)
			require.NotErrorIs(t, err, decad.ErrUnsupported)
			require.NotErrorIs(t, err, decad.ErrDegenerate)
		})
	}
}

// A fit-spline profile's moments now answer, so Extrude reaches its own
// side-face build — and must still refuse there (P4 is not this PR's scope).
func TestExtrudeFitSplineProfileRefusesAtSideFaces(t *testing.T) {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	start := s.CreatePoint(0, 0)
	mid := s.CreatePoint(4, 3)
	end := s.CreatePoint(8, 0)
	_, err = s.CreateFitSpline(start, mid, end)
	require.NoError(t, err)
	s.CreateLine(end, start)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)

	d := decad.New()
	body, err := d.Extrude(s, profiles[0], &decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.Error(t, err)
	require.Nil(t, body)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "free-form")
	require.Empty(t, d.Bodies(), "a refused extrude registers no body")
	require.Empty(t, d.Recipe().Steps, "a refused extrude records no step")
}

// The R7 ceiling has to be levied BEFORE geom.NewFitInterpolant allocates or
// solves anything: fitInterpolantCost(n) is 64 units per fit point, so 20,000
// fit points (1,280,000 units) exceeds the fixed 2^20 ceiling on its own,
// entirely ahead of the reconstruction charge or the tridiagonal solve.
func TestOverBudgetFitInterpolantRefusesBeforeSolving(t *testing.T) {
	const points = 20000
	fit := make([]decad.Point2, points)
	for i := range fit {
		fit[i] = decad.Point2{U: float64(i), V: float64(i % 5)}
	}
	record := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
		decad.FitSplineSeg{Fit: fit, TStart: 0, TEnd: 1},
	}}}

	start := time.Now()
	_, err := record.Area()
	require.Error(t, err)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "work budget")
	require.Less(t, time.Since(start), 2*time.Second, "the refusal precedes the interpolant solve")
}

// TestFitSplineTerminalDedupRefusesUnclosedLoop is the review-128-2
// regression: sketch's fit-point dedup (fitChordEps, geom/fitspline.go)
// collapses a terminal fit point sitting closer than 1e-12 to its neighbor,
// so geom.NewFitInterpolant's own interpolant ends at the PREVIOUS surviving
// point while the record still names the dropped point as the join to the
// next segment — here p2 = (10, 10) and p3 = (10+3e-13, 10) collapse to one
// active point, but the closing LineSeg still starts at p3. Before this
// check existed, the region traced was open by 3e-13 mm and Area() still
// published a bounded, Approximate number for it (62.5, off by 3244x its
// own bound against the exact rational integral of the region actually
// traced) — sketch's own reconstruction cannot catch this, since it rebuilds
// the SAME entity from the SAME Fit points and round-trips to the same near
// miss.
func TestFitSplineTerminalDedupRefusesUnclosedLoop(t *testing.T) {
	p0 := decad.Point2{U: 0, V: 0}
	p1 := decad.Point2{U: 10, V: 0}
	p2 := decad.Point2{U: 10, V: 10}
	p3 := decad.Point2{U: 10 + 3e-13, V: 10}

	record := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
		decad.FitSplineSeg{Fit: []decad.Point2{p0, p1, p2, p3}, TStart: 0, TEnd: 1},
		decad.LineSeg{Start: p3, End: p0, TStart: 0, TEnd: 1},
	}}}

	_, err := record.Area()
	require.Error(t, err)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Contains(t, err.Error(), "sketch's fit-point dedup collapsed it")
}

// TestFitSplineNonDegenerateTerminalStillCloses is the companion pass case:
// the same shape with p3 moved far enough from p2 (1 mm, well past
// fitChordEps) that sketch's dedup keeps it active, so the converted chain's
// own end is p3 exactly and the closing LineSeg's start joins it bit for
// bit. The join check introduced for the finding above must not refuse a
// record whose free-form segment actually closes.
func TestFitSplineNonDegenerateTerminalStillCloses(t *testing.T) {
	p0 := decad.Point2{U: 0, V: 0}
	p1 := decad.Point2{U: 10, V: 0}
	p2 := decad.Point2{U: 10, V: 10}
	p3 := decad.Point2{U: 9, V: 10}

	record := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
		decad.FitSplineSeg{Fit: []decad.Point2{p0, p1, p2, p3}, TStart: 0, TEnd: 1},
		decad.LineSeg{Start: p3, End: p0, TStart: 0, TEnd: 1},
	}}}

	area, err := record.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, area.Exactness)
}

// TestFitSplineTerminalDedupRefusesUnclosedLoopReversed pins the reversed
// (TStart=1, TEnd=0) side of requireFitSplineTerminalJoins: the record's own
// dedup-risky natural-last fit point (p3) then sits at the WALK's start
// rather than its end, since freeformEndpoints swaps start/end into walk
// order for a reversed range. The preceding LineSeg still ends at the
// recorded p3, so the same collapse still leaves the loop open.
func TestFitSplineTerminalDedupRefusesUnclosedLoopReversed(t *testing.T) {
	p0 := decad.Point2{U: 0, V: 0}
	p1 := decad.Point2{U: 10, V: 0}
	p2 := decad.Point2{U: 10, V: 10}
	p3 := decad.Point2{U: 10 + 3e-13, V: 10}

	record := decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
		decad.LineSeg{Start: p0, End: p3, TStart: 0, TEnd: 1},
		decad.FitSplineSeg{Fit: []decad.Point2{p0, p1, p2, p3}, TStart: 1, TEnd: 0},
	}}}

	_, err := record.Area()
	require.Error(t, err)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Contains(t, err.Error(), "sketch's fit-point dedup collapsed it")
}
