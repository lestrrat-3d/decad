package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
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

// TestExtrudeCircularReadingsStayApproximate is the negative guard: a
// pi-derived length, area or centroid is never exactly representable in
// float64, so tightening the bound must never make one report Exact — that
// would be a false claim, not merely an imprecise one.
func TestExtrudeCircularReadingsStayApproximate(t *testing.T) {
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

// TestExtrudeSquarePrismStaysExact is the regression guard: an axis-aligned
// square section's side lengths, area, volume and centroid are all exactly
// representable, and none of the tightening above may disturb that.
func TestExtrudeSquarePrismStaysExact(t *testing.T) {
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
