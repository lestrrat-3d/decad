package apitest_test

import (
	"math"
	"math/big"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file holds the readings of bodies whose section walks an ArcSeg over
// a narrowed range — a cut fragment of a sketch arc, or a Split piece of a
// ball — to their closed forms, and holds each bound near the rounding level.
// Every fixture is built on the radius-5 semicircle about the origin, from
// (0, −5) through (5, 0) to (0, 5), revolved about the sketch's v axis
// (trimRevolveAxis) or extruded 2 mm. The cut line v = 5/2 meets the arc at
// θ = π/6, so the cap above it has height h = 5/2: πh²(3R − h)/3 = 625π/24
// mm³, a curved surface of 2πRh = 25π mm² and a cut disc of radius 5√3/2.
//
// Each bound is held under 1e-9 of its reading, and a centroid's under
// 5e-9 mm. Those ceilings are far looser than the 1e-16 to 5e-13 relative
// these bounds measure, because a bound moves by ulps between amd64 and
// arm64; the float envelope the readings fall back to without a circular
// bracket measures above 1.3 relative and 180 mm, so the ceilings still
// separate the two.
//
// Shown-to-fail: restoring the whole-range gate (ok == false unless the
// range is 0 → 1 or 1 → 0) in circularAreaInterval, anchoredArcWalk and
// circularAxisMomentInterval turns every ceiling leg in this file red: the
// volumes and areas read bounds of 1.4 to 3346 times their values and the
// centroids bounds of 180 to 238 mm. The enclosure legs stay green there,
// since the envelope still encloses.

// arcFragmentForm is rat + pi·π + sqrt3·√3 over rationals.
type arcFragmentForm struct{ rat, pi, sqrt3 *big.Rat }

func arcForm(rat, pi, sqrt3 *big.Rat) arcFragmentForm { return arcFragmentForm{rat, pi, sqrt3} }

func arcPi(n, d int64) arcFragmentForm { return arcForm(new(big.Rat), big.NewRat(n, d), new(big.Rat)) }

var (
	sqrt3RefLo = mustTestDecimal("1.732050807568877293527446341505872366942805253810380628055806979")
	sqrt3RefHi = mustTestDecimal("1.732050807568877293527446341505872366942805253810380628055806980")
)

func (f arcFragmentForm) bracket() (*big.Rat, *big.Rat) {
	term := func(c, lo, hi *big.Rat) (*big.Rat, *big.Rat) {
		a, b := new(big.Rat).Mul(c, lo), new(big.Rat).Mul(c, hi)
		if a.Cmp(b) > 0 {
			a, b = b, a
		}
		return a, b
	}
	l1, h1 := term(f.pi, piRefLo, piRefHi)
	l2, h2 := term(f.sqrt3, sqrt3RefLo, sqrt3RefHi)
	lo := new(big.Rat).Add(f.rat, new(big.Rat).Add(l1, l2))
	hi := new(big.Rat).Add(f.rat, new(big.Rat).Add(h1, h2))
	return lo, hi
}

// requireArcFragmentTight holds m's interval to the whole closed-form
// bracket, and its bound under 1e-9 of the value.
func requireArcFragmentTight(t *testing.T, what string, m decad.Measurement, want arcFragmentForm) {
	t.Helper()
	requireSplitEncloses(t, what, m, want)
	require.LessOrEqual(t, m.Bound.Base(), 1e-9*math.Abs(m.Value.Base()), "%s: %v ± %v", what, m.Value, m.Bound)
}

// requireArcFragmentCentroid holds a centroid on the axis at height y to
// within its bound, and its bound under 1e-9 of the semicircle's radius.
func requireArcFragmentCentroid(t *testing.T, what string, body *decad.Body, y float64) {
	t.Helper()
	c, err := body.Centroid()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Hypot(c.Value.X, c.Value.Y-y), c.Bound.Base(), "%s: %v ± %v", what, c.Value, c.Bound)
	require.LessOrEqual(t, math.Abs(c.Value.Z), c.Bound.Base(), "%s: %v ± %v", what, c.Value, c.Bound)
	require.LessOrEqual(t, c.Bound.Base(), 5e-9, "%s: %v ± %v", what, c.Value, c.Bound)
}

// arcFragmentCapSketch is the semicircle closed along the axis, boxed to
// u = 8 by three lines and cut by the line v = 5/2. Its four regions are the
// cap and the rest of the half disc inside the arc, which walk the arc
// forward, and the two pieces of the box outside it, which walk it reversed.
func arcFragmentCapSketch(t *testing.T) *sketch.Sketch {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	fixed := func(u, v float64) *sketch.Point {
		p := s.CreatePoint(u, v)
		s.Fix(p)
		return p
	}
	c, bottom, top := fixed(0, 0), fixed(0, -5), fixed(0, 5)
	s.CreateArc(c, bottom, top)
	s.CreateLine(top, bottom)
	topRight, bottomRight := fixed(8, 5), fixed(8, -5)
	s.CreateLine(top, topRight)
	s.CreateLine(topRight, bottomRight)
	s.CreateLine(bottomRight, bottom)
	s.CreateLine(fixed(0, 2.5), fixed(8, 2.5))
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 4)
	return s
}

// arcFragmentProfile picks the region whose sketch-reported area is nearest
// want: the selection only, never an assertion.
func arcFragmentProfile(t *testing.T, s *sketch.Sketch, want float64) *sketch.Profile {
	t.Helper()
	profiles := s.Profiles()
	return slices.MinFunc(profiles, func(a, b *sketch.Profile) int {
		da, db := math.Abs(a.Area-want), math.Abs(b.Area-want)
		switch {
		case da < db:
			return -1
		case da > db:
			return 1
		}
		return 0
	})
}

// The four regions' sketch areas: the cap is 25π/6 − 25√3/8.
var (
	arcFragmentCapArea      = 25*math.Pi/6 - 25*math.Sqrt(3)/8
	arcFragmentLowerArea    = 25*math.Pi/2 - arcFragmentCapArea
	arcFragmentOuterTopArea = 20 - arcFragmentCapArea
	arcFragmentOuterLowArea = 60 - arcFragmentLowerArea
)

func TestArcFragmentSplitHemispheresAreTight(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	ball := splitRevolveSolid(t, doc, splitRevolveBall, decad.FullRevolution{})
	disc := splitRevolveSheet(t, doc, 0, 0, 8, 0, decad.FullRevolution{})
	pieces, err := doc.Split(t.Context(), ball, disc)
	require.NoError(t, err)
	require.Len(t, pieces, 2)
	// Each hemisphere walks the meridian arc over half its range, [0, 1/2]
	// below the cut and [1/2, 1] above: 250π/3 mm³, 75π mm² with the cut
	// disc, centroid 3R/8 off the centre.
	ys := make([]float64, 0, 2)
	for _, piece := range pieces {
		c, err := piece.Centroid()
		require.NoError(t, err)
		ys = append(ys, c.Value.Y)
	}
	if ys[0] > ys[1] {
		pieces[0], pieces[1] = pieces[1], pieces[0]
	}
	for i, piece := range pieces {
		volume, err := piece.Volume()
		require.NoError(t, err)
		requireArcFragmentTight(t, "hemisphere volume", volume, arcPi(250, 3))
		area, err := piece.Area()
		require.NoError(t, err)
		requireArcFragmentTight(t, "hemisphere area", area, arcPi(75, 1))
		requireArcFragmentCentroid(t, "hemisphere centroid", piece, []float64{-15.0 / 8, 15.0 / 8}[i])
	}
}

func TestArcFragmentRevolveReadingsAreTight(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name      string
		sketchA   float64
		volume    arcFragmentForm
		area      arcFragmentForm
		centroidY float64
	}{
		// The cap, its arc fragment walked forward over [2/3, 1].
		{"cap", arcFragmentCapArea, arcPi(625, 24), arcPi(175, 4), 27.0 / 8},
		// The rest of the ball, forward over [0, 2/3]: 500π/3 − 625π/24 mm³,
		// the zone's 75π mm² and the cut disc's 75π/4.
		{"ball below the cap", arcFragmentLowerArea, arcPi(1125, 8), arcPi(375, 4), -5.0 / 8},
		// The radius-8 cylinder over v ∈ [5/2, 5] less the cap, its arc
		// fragment walked REVERSED over [1, 2/3]: the 40π side, the 64π top,
		// the 181π/4 annulus and the cap's own 25π.
		{"box above the cut", arcFragmentOuterTopArea, arcPi(3215, 24), arcPi(697, 4), 19665.0 / 5144},
		// The cylinder over v ∈ [−5, 5/2] less the lower ball, reversed over
		// [2/3, 0]: the 120π side, the 64π bottom, the annulus and the 75π zone.
		{"box below the cut", arcFragmentOuterLowArea, arcPi(2715, 8), arcPi(1217, 4), -2185.0 / 1448},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			s := arcFragmentCapSketch(t)
			doc := decad.New()
			body, err := doc.Revolve(s, arcFragmentProfile(t, s, tc.sketchA), trimRevolveAxis(), decad.FullRevolution{})
			require.NoError(t, err)
			volume, err := body.Volume()
			require.NoError(t, err)
			requireArcFragmentTight(t, "volume", volume, tc.volume)
			area, err := body.Area()
			require.NoError(t, err)
			requireArcFragmentTight(t, "area", area, tc.area)
			requireArcFragmentCentroid(t, "centroid", body, tc.centroidY)
		})
	}
}

func TestArcFragmentPrismReadingsAreTight(t *testing.T) {
	t.Parallel()
	third := func(n int64) *big.Rat { return big.NewRat(n, 3) }
	quarter := func(n int64) *big.Rat { return big.NewRat(n, 4) }
	for _, tc := range []struct {
		name    string
		sketchA float64
		volume  arcFragmentForm
		area    arcFragmentForm
	}{
		// Twice the cap's 25π/6 − 25√3/8, and twice that plus the 2 mm wall
		// over its perimeter 5π/3 + 5√3/2 + 5/2.
		{"cap", arcFragmentCapArea,
			arcForm(new(big.Rat), third(25), quarter(-25)),
			arcForm(big.NewRat(5, 1), third(35), quarter(-5))},
		// The reversed fragment: twice 20 − 25π/6 + 25√3/8, and the wall over
		// the perimeter 5π/3 + 37/2 − 5√3/2.
		{"box above the cut", arcFragmentOuterTopArea,
			arcForm(big.NewRat(40, 1), third(-25), quarter(25)),
			arcForm(big.NewRat(77, 1), big.NewRat(-5, 1), quarter(5))},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			s := arcFragmentCapSketch(t)
			doc := decad.New()
			body, err := doc.Extrude(s, arcFragmentProfile(t, s, tc.sketchA),
				decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
			require.NoError(t, err)
			volume, err := body.Volume()
			require.NoError(t, err)
			requireArcFragmentTight(t, "volume", volume, tc.volume)
			area, err := body.Area()
			require.NoError(t, err)
			requireArcFragmentTight(t, "area", area, tc.area)
		})
	}
}
