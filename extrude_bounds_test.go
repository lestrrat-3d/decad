package decad_test

import (
	"fmt"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file tests the certified-bracket tightening of a straight prism's
// Area/Centroid/edge-length bounds (moments.go's circularAreaInterval sibling
// circularFirstMomentInterval, circularLengthInterval, and the sqrt-bracket
// fallback in lineWalkBounds): every bound must still PROVE what it claims
// (soundness — the true value lies within Value±Bound) while landing close
// enough to zero for a caller's default tolerance to read Sound.

// regularNGonSketch builds a regular n-gon of circumradius r, vertex 0 on the
// +u axis, wound CCW.
func regularNGonSketch(t *testing.T, n int, r float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	pts := make([][2]float64, n)
	for k := range n {
		theta := 2 * math.Pi * float64(k) / float64(n)
		pts[k] = [2]float64{r * math.Cos(theta), r * math.Sin(theta)}
	}
	return polygonSketch(t, pts)
}

// TestExtrudeNGonPrismBoundsTighten checks the sqrt-bracket line-length
// tightening (extrude.go's lineWalkBounds) over regular n-gon prisms of
// circumradius 10 mm and height 5 mm: every side length but a very small set
// of special angles is irrational (Hypot is not exactly representable), which
// is exactly the case lineWalkBounds' fallback bound covers. The exact area is
// closed form: two regular-polygon caps plus n rectangular sides.
func TestExtrudeNGonPrismBoundsTighten(t *testing.T) {
	t.Parallel()
	const r = 10.0
	const h = 5.0
	for _, n := range []int{3, 4, 5, 6, 8, 12, 32, 128} {
		side := 2 * r * math.Sin(math.Pi/float64(n))
		capArea := 0.5 * float64(n) * r * r * math.Sin(2*math.Pi/float64(n))
		exactArea := 2*capArea + float64(n)*side*h

		s, p := regularNGonSketch(t, n, r)
		doc := decad.New()
		body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
		require.NoErrorf(t, err, "n=%d", n)

		area, err := body.Area()
		require.NoErrorf(t, err, "n=%d", n)
		value, bound := area.Value.Base(), area.Bound.Base()

		// Soundness: the proven bound must actually enclose the true error.
		require.LessOrEqualf(t, math.Abs(value-exactArea), bound,
			"n=%d: the area bound %g mm² does not enclose the true error against the exact %g mm²", n, bound, exactArea)

		// Tightness: the certified bracket must land far inside default tolerance.
		require.LessOrEqualf(t, bound, 1e-9*value,
			"n=%d: the area bound %g mm² is not tight against the value %g mm²", n, bound, value)
	}
}

// TestExtrudeCirclePrismBoundsTighten is the diagnosed symptom directly: a
// circle of radius 10 mm extruded 5 mm previously reported an Area bound
// exactly twice its own value and a Centroid bound over 13 times the radius,
// because conservativeValueError's magnitude envelope carries no information
// about the actual error. circularAreaInterval and circularFirstMomentInterval
// (moments.go) now bracket a whole-turn CircleSeg exactly (pi times a rational,
// certified to 76 decimal digits), so the bound shrinks to a few ulps and the
// body reads Sound at the default tolerance.
func TestExtrudeCirclePrismBoundsTighten(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, 10)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)

	area, err := body.Area()
	require.NoError(t, err)
	wantArea := 300 * math.Pi // two pi*r^2 caps (200*pi) + one 2*pi*r*h side (100*pi)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-wantArea), area.Bound.Base(),
		"the area bound does not enclose the true error against 300*pi")
	require.LessOrEqual(t, area.Bound.Base(), 1e-9*area.Value.Base(),
		"the area bound is not tight against its value")

	c, err := body.Centroid()
	require.NoError(t, err)
	require.InDelta(t, 0.0, c.Value.X, 1e-9)
	require.InDelta(t, 0.0, c.Value.Y, 1e-9)
	require.InDelta(t, 2.5, c.Value.Z, 1e-9)
	require.LessOrEqual(t, c.Bound.Base(), 1e-6,
		"the centroid bound is not small")

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	require.True(t, report.Trustworthy())
	require.Empty(t, report.Diagnostics)
}

// TestExtrudeArcSegPrismBoundsTighten covers the ArcSeg arm specifically: a
// quarter-disk pie slice (two straight radii plus one 90° arc, TStart=0,
// TEnd=1 over the arc's own full recorded range) routes circularLengthInterval
// and circularFirstMomentInterval through atan2Interval's exact-rational
// bracket rather than the closed CircleSeg form circularAreaInterval already
// had.
func TestExtrudeArcSegPrismBoundsTighten(t *testing.T) {
	t.Parallel()
	const r = 20.0
	const h = 5.0
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 0)
	s.Fix(o)
	px := s.CreatePoint(r, 0)
	py := s.CreatePoint(0, r)
	s.CreateLine(o, px)
	s.CreateLine(py, o)
	s.CreateArc(o, px, py)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)

	area, err := body.Area()
	require.NoError(t, err)
	quarterDiskArea := math.Pi * r * r / 4
	wantArea := 2*quarterDiskArea + (r+r+2*math.Pi*r/4)*h
	require.LessOrEqual(t, math.Abs(area.Value.Base()-wantArea), area.Bound.Base(),
		"the area bound does not enclose the true error")
	require.LessOrEqual(t, area.Bound.Base(), 1e-9*area.Value.Base(),
		"the area bound is not tight against its value")

	// The arc's own edge length is r*pi/2, exactly the atan2Interval-bracketed
	// reading circularLengthInterval's ArcSeg arm proves.
	var arcEdge *decad.Edge
	for _, e := range body.Edges() {
		if _, ok := e.Curve().(decad.Arc3); ok {
			arcEdge = e
			break
		}
	}
	require.NotNil(t, arcEdge, "the quarter disk has one Arc3 edge")
	length, err := arcEdge.Length()
	require.NoError(t, err)
	wantLength := r * math.Pi / 2
	require.LessOrEqual(t, math.Abs(length.Value.Base()-wantLength), length.Bound.Base(),
		"the arc length bound does not enclose the true error")
	require.LessOrEqual(t, length.Bound.Base(), 1e-9*length.Value.Base(),
		"the arc length bound is not tight against its value")
}

// TestExtrudeArcSegPrismReadingsBitIdenticalAcrossAtanCarrier pins fu159's
// central claim: routing atanSmallInterval through the fixed-point grid
// instead of exact big.Rat must not change a single published bit. Every
// golden string below was captured from this same quarter-disk-prism
// scenario on the pre-port commit (exact-rational atanSmallInterval) via
// fmt.Sprintf("%b", ...) — the exact binary float64 representation, not a
// rounded decimal — so a mismatch here means the carrier swap moved a
// published value or bound, not merely that a tolerance loosened.
func TestExtrudeArcSegPrismReadingsBitIdenticalAcrossAtanCarrier(t *testing.T) {
	t.Parallel()
	const r = 20.0
	const h = 5.0
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 0)
	s.Fix(o)
	px := s.CreatePoint(r, 0)
	py := s.CreatePoint(0, r)
	s.CreateLine(o, px)
	s.CreateLine(py, o)
	s.CreateArc(o, px, py)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)

	area, err := body.Area()
	require.NoError(t, err)
	require.Equal(t, "8667653909156874p-43", fmt.Sprintf("%b", area.Value.Base()))
	require.Equal(t, "7222303692821024p-97", fmt.Sprintf("%b", area.Bound.Base()))

	vol, err := body.Volume()
	require.NoError(t, err)
	require.Equal(t, "6908435304715274p-42", fmt.Sprintf("%b", vol.Value.Base()))
	require.Equal(t, "5281773380312612p-96", fmt.Sprintf("%b", vol.Bound.Base()))

	c, err := body.Centroid()
	require.NoError(t, err)
	require.Equal(t, "4778467616018883p-49", fmt.Sprintf("%b", c.Value.X))
	require.Equal(t, "4778467616018881p-49", fmt.Sprintf("%b", c.Value.Y))
	require.Equal(t, "5629499534213120p-51", fmt.Sprintf("%b", c.Value.Z))
	require.Equal(t, "4698155632678034p-98", fmt.Sprintf("%b", c.Bound.Base()))

	var arcEdge *decad.Edge
	for _, e := range body.Edges() {
		if _, ok := e.Curve().(decad.Arc3); ok {
			arcEdge = e
			break
		}
	}
	require.NotNil(t, arcEdge, "the quarter disk has one Arc3 edge")
	length, err := arcEdge.Length()
	require.NoError(t, err)
	require.Equal(t, "8842797190035550p-48", fmt.Sprintf("%b", length.Value.Base()))
	require.Equal(t, "6209697000026892p-102", fmt.Sprintf("%b", length.Bound.Base()))
}

// TestExtrudeCircularReadingsStayApproximate is the negative guard: a
// pi-derived length, area or centroid is never exactly representable in
// float64, so tightening the bound must never make one report Exact — that
// would be a false claim, not merely an imprecise one.
func TestExtrudeCircularReadingsStayApproximate(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, 10)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)

	area, err := body.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, area.Exactness)
	require.Positive(t, area.Bound.Base())

	vol, err := body.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, vol.Exactness)
	require.Positive(t, vol.Bound.Base())

	c, err := body.Centroid()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, c.Exactness)
	require.Positive(t, c.Bound.Base())

	for _, e := range body.Edges() {
		if _, ok := e.Curve().(decad.Circle3); !ok {
			continue
		}
		length, err := e.Length()
		require.NoError(t, err)
		require.Equal(t, decad.Approximate, length.Exactness, `a circle's circumference is never exactly representable`)
		require.Positive(t, length.Bound.Base())
	}
}

// quarterDiscRecord builds the quarter disc of radius r as a ProfileRecord
// whose curved boundary is a trimmed CircleSeg (TStart 0, TEnd 0.25) closed
// by two straight radii — the exact fixture the defect diagnosis isolated:
// the same geometry a whole-circle CircleSeg or a two-radius-plus-ArcSeg
// record already bounds tightly, recorded instead the way a genuine
// circle-circle overlap arrangement records a partial circle (seam.go's
// recordEdge narrows CircleSeg's own TStart/TEnd rather than promoting it to
// an ArcSeg).
func quarterDiscRecord(r float64) decad.ProfileRecord {
	return decad.ProfileRecord{Outer: decad.LoopRecord{Segments: []decad.CurveSegment{
		decad.CircleSeg{
			Center: decad.Point2{U: 0, V: 0},
			Radius: units.Millimeters(r),
			CCW:    true, TStart: 0, TEnd: 0.25,
		},
		decad.LineSeg{Start: decad.Point2{U: 0, V: r}, End: decad.Point2{U: 0, V: 0}, TStart: 0, TEnd: 1},
		decad.LineSeg{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: r, V: 0}, TStart: 0, TEnd: 1},
	}}}
}

// TestExtrudeTrimmedCircleSegPrismBoundsTighten is the diagnosed defect's own
// fixture: moments.go's circularAreaInterval/circularFirstMomentInterval
// refused a CircleSeg whose recorded range was not a whole turn, falling back
// to conservativeValueError's body-scale envelope — a bound 4.9 orders of
// magnitude looser than the identical quarter disc recorded as an ArcSeg
// (design A7 §1.2: 385.619 against 4.91e-16 on an area of 78.5398). The
// fractional-turn arm (moments_trig.go's turnSinCosInterval) now brackets a
// trimmed CircleSeg the same way.
func TestExtrudeTrimmedCircleSegPrismBoundsTighten(t *testing.T) {
	t.Parallel()
	const r = 10.0
	rec := quarterDiscRecord(r)
	wantArea := math.Pi * r * r / 4

	area, err := rec.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-wantArea), area.Bound.Base(),
		"the area bound does not enclose the true error against pi*r^2/4")
	require.LessOrEqual(t, area.Bound.Base(), 1e-9*area.Value.Base(),
		"the area bound is not tight against its value")
	require.Equal(t, decad.Approximate, area.Exactness, "pi*r^2/4 is never exactly representable")

	// The quarter disc's centroid sits at (4r/(3*pi), 4r/(3*pi)) — the
	// standard quarter-circle centroid distance from each straight edge.
	wantCentroid := 4 * r / (3 * math.Pi)
	c, err := rec.Centroid()
	require.NoError(t, err)
	require.InDelta(t, wantCentroid, c.Value.X, 1e-9)
	require.InDelta(t, wantCentroid, c.Value.Y, 1e-9)
	require.LessOrEqual(t, c.Bound.Base(), 1e-6, "the centroid bound is not small")
	require.Equal(t, decad.Approximate, c.Exactness)
}

// TestExtrudeSquarePrismStaysExact is the regression guard: an axis-aligned
// square section's side lengths, area, volume and centroid are all exactly
// representable, and none of the tightening above may disturb that.
func TestExtrudeSquarePrismStaysExact(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)

	area, err := body.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, area.Exactness)
	require.Zero(t, area.Bound.Base())

	vol, err := body.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, vol.Exactness)
	require.Zero(t, vol.Bound.Base())

	c, err := body.Centroid()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, c.Exactness)
	require.Zero(t, c.Bound.Base())
}

// TestExtrudeArcSectionBoxEnclosesArcApex is the prism site of the shared
// boundary-extreme scan's own arc term. The section is the one whose apex
// radius √37 no float64 holds, and the prism box reads the SAME scan the
// revolve box does, so the term belongs to the scan and the prism publishes it
// like any other consumer: the box is Approximate and its interval contains the
// apex the section actually reaches.
func TestExtrudeArcSectionBoxEnclosesArcApex(t *testing.T) {
	t.Parallel()
	s, p := arcApexSketch(t, 1, 6)
	body, err := decad.New().Extrude(s, p, decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)

	box, err := body.Bounds()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, box.Exactness)
	require.Greater(t, box.Bound.Base(), 0.0)
	require.Equal(t, math.Hypot(1, 6), box.Max.Y)
	requireEnclosesApex(t, box, 1, 6)
}

// TestExtrudeBoundsSweepEnclosesArcApex is the prism half of the acceptance
// sweep: 144 ordinary circular-segment sections, each extruded and each asked
// whether the box's own published interval contains the apex radius the section
// truly reaches. A single fixture cannot show this — the sign and size of the
// hypot rounding change with the coordinates — so the grid is the test.
func TestExtrudeBoundsSweepEnclosesArcApex(t *testing.T) {
	t.Parallel()
	for i := 1; i <= 12; i++ {
		for j := 1; j <= 12; j++ {
			u, v := 0.7*float64(i), 1.3*float64(j)
			t.Run(fmt.Sprintf("u=%g/v=%g", u, v), func(t *testing.T) {
				s, p := arcApexSketch(t, u, v)
				body, err := decad.New().Extrude(s, p, decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
				require.NoError(t, err)
				box, err := body.Bounds()
				require.NoError(t, err)
				requireEnclosesApex(t, box, u, v)
			})
		}
	}
}

// exactApply computes the EXACT rational image of p under xform's OWN held
// basis and translation entries — the same isometry a placed body's box
// reads through xform.Apply as an exact leaf — rounded to float64 only once,
// at the end, rather than through xform.Apply's own float arithmetic. It is
// an independent check: it never calls decad's own exactIsometryDotRound, so
// a test built on it cannot pass merely because the production code agrees
// with itself.
func exactApply(t *testing.T, xform r3.Transform, p r3.Vec) r3.Vec {
	t.Helper()
	toRat := func(x float64) *big.Rat {
		r := new(big.Rat)
		require.NotNil(t, r.SetFloat64(x), "float64 %v must be finite to convert exactly", x)
		return r
	}
	basis := xform.Basis()
	tr := xform.Translation()
	px, py, pz := toRat(p.X), toRat(p.Y), toRat(p.Z)
	component := func(exC, eyC, ezC, trC float64) float64 {
		sum := new(big.Rat)
		sum.Add(sum, new(big.Rat).Mul(toRat(exC), px))
		sum.Add(sum, new(big.Rat).Mul(toRat(eyC), py))
		sum.Add(sum, new(big.Rat).Mul(toRat(ezC), pz))
		sum.Add(sum, toRat(trC))
		f, _ := sum.Float64()
		return f
	}
	return r3.NewVec(
		component(basis.EX.X, basis.EY.X, basis.EZ.X, tr.X),
		component(basis.EX.Y, basis.EY.Y, basis.EZ.Y, tr.Y),
		component(basis.EX.Z, basis.EY.Z, basis.EZ.Z, tr.Z),
	)
}

// TestExtrudeBoxBoundsEnclosesPlacedCorners pins fu203: a 10x8x5 box placed
// by Rotation(X, 37 degrees) published Exact with a zero bound while missing
// the exact rational image of its own corners by 7.7715611723760958e-16 in
// Z, because the box read the placement's own xform.Apply as an exact leaf.
// The frame and placement ARE isometries in exact arithmetic, but their FLOAT
// evaluation rounds for a non-axis-aligned rotation, and the box's own
// published interval must contain the true corner it misses.
func TestExtrudeBoxBoundsEnclosesPlacedCorners(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 10, 8)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)

	rot, err := r3.Rotation(r3.NewVec(1, 0, 0), units.Degrees(37))
	require.NoError(t, err)
	placed, err := body.Placed(rot)
	require.NoError(t, err)

	bounds, err := placed.Bounds()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, bounds.Exactness)
	boundMM, err := bounds.Bound.In(units.Millimeter)
	require.NoError(t, err)
	require.Greater(t, boundMM, 0.0)

	minC := r3.NewVec(math.Inf(1), math.Inf(1), math.Inf(1))
	maxC := r3.NewVec(math.Inf(-1), math.Inf(-1), math.Inf(-1))
	for _, x := range []float64{0, 10} {
		for _, y := range []float64{0, 8} {
			for _, z := range []float64{0, 5} {
				exact := exactApply(t, rot, r3.NewVec(x, y, z))
				minC = r3.NewVec(math.Min(minC.X, exact.X), math.Min(minC.Y, exact.Y), math.Min(minC.Z, exact.Z))
				maxC = r3.NewVec(math.Max(maxC.X, exact.X), math.Max(maxC.Y, exact.Y), math.Max(maxC.Z, exact.Z))
			}
		}
	}
	require.LessOrEqual(t, math.Abs(bounds.Min.X-minC.X), boundMM)
	require.LessOrEqual(t, math.Abs(bounds.Min.Y-minC.Y), boundMM)
	require.LessOrEqual(t, math.Abs(bounds.Min.Z-minC.Z), boundMM)
	require.LessOrEqual(t, math.Abs(bounds.Max.X-maxC.X), boundMM)
	require.LessOrEqual(t, math.Abs(bounds.Max.Y-maxC.Y), boundMM)
	require.LessOrEqual(t, math.Abs(bounds.Max.Z-maxC.Z), boundMM)
}

// requireEnclosesExactSum asserts that a published box coordinate's own proven
// interval contains the EXACT sum of the terms that coordinate is the float
// sum of. The comparison runs over the rationals from end to end: rounding the
// true sum to a float64 first would round it straight back onto the held
// coordinate and the assertion could never fail. It is an independent check —
// it never calls decad's own exactSumRound — so a test built on it cannot pass
// merely because the production code agrees with itself.
func requireEnclosesExactSum(t *testing.T, coord, bound float64, terms ...float64) {
	t.Helper()
	toRat := func(x float64) *big.Rat {
		r := new(big.Rat)
		require.NotNil(t, r.SetFloat64(x), "float64 %v must be finite to convert exactly", x)
		return r
	}
	truth := new(big.Rat)
	for _, term := range terms {
		truth.Add(truth, toRat(term))
	}
	gap := new(big.Rat).Abs(new(big.Rat).Sub(truth, toRat(coord)))
	require.LessOrEqualf(t, gap.Cmp(toRat(bound)), 0,
		"the published coordinate %v misses the exact sum of %v by %v, which its own bound %v mm does not cover",
		coord, terms, gap.FloatString(20), bound)
}

// TestExtrudeBoxBoundsEnclosesTranslatedCorner pins the recombination the box
// commits after every coefficient is proven exact: a 10x8x5 box moved by
// Translation(0.1, 0, 0) has base 0.1, gu 1, gv 0 and gz 0, so the placement
// rounds nothing and every multiply is exact — and the addition base + 10 that
// produces Max.X is still not representable, missing the true 0.1 + 10 by
// 3.6e-16. A box that charges only the coefficients publishes that miss as
// Exact with a zero bound.
func TestExtrudeBoxBoundsEnclosesTranslatedCorner(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 10, 8)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)

	shift, err := r3.Translation(r3.NewVec(0.1, 0, 0))
	require.NoError(t, err)
	placed, err := body.Placed(shift)
	require.NoError(t, err)

	bounds, err := placed.Bounds()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, bounds.Exactness)
	boundMM, err := bounds.Bound.In(units.Millimeter)
	require.NoError(t, err)
	require.Greater(t, boundMM, 0.0)

	requireEnclosesExactSum(t, bounds.Max.X, boundMM, 10, 0.1)
}
