package apitest_test

import (
	"fmt"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file pins three readings docs/clearance-design.md §2 and §5 owe a
// published Clearance row: a cone carrier is measured as its float apex and
// window stand, a topology vertex widens the row by its own proven bound, and
// a coalesced wall stands only for exactly collinear segments. Each fixture's
// truth is computed exactly (or, where it needs a sine, to 300 bits), never
// read back from the kernel.

// fixedPolygonSketch draws a closed polygon on the XY plane with every vertex
// fixed at its authored coordinate.
func fixedPolygonSketch(t *testing.T, pts [][2]float64) *sketch.Sketch {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	sp := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		sp[i] = s.CreatePoint(p[0], p[1])
		s.Fix(sp[i])
	}
	for i := range sp {
		s.CreateLine(sp[i], sp[(i+1)%len(sp)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	return s
}

// ballAtCentre revolves a half disc a full turn into a ball of radius r
// centred exactly at (cx, cy, cz): the disc is drawn on the XY plane offset
// to z = cz, about the line through its centre along the plane's u.
func ballAtCentre(t *testing.T, doc *decad.Document, cx, cy, cz, r float64) {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), cz)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	a, b, c := s.CreatePoint(cx-r, cy), s.CreatePoint(cx+r, cy), s.CreatePoint(cx, cy)
	s.Fix(a)
	s.Fix(b)
	s.Fix(c)
	s.CreateLine(a, b)
	s.CreateArc(c, b, a)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	_, err = doc.Revolve(s, s.Profiles()[0],
		decad.SketchLine{Start: decad.Point2{U: cx, V: cy}, End: decad.Point2{U: cx + 1, V: cy}}, decad.FullRevolution{})
	require.NoError(t, err)
}

// requireGapEnclosesRoot requires the row's interval [v − b, v + b] to
// contain √truth2 − off, comparing squares exactly: the truth is a square
// root of an exact rational, less a dyadic offset.
func requireGapEnclosesRoot(t *testing.T, gap decad.Measurement, truth2, off *big.Rat) {
	t.Helper()
	v, b := ratOf(gap.Value.Mag()), ratOf(gap.Bound.Mag())
	hi := new(big.Rat).Add(new(big.Rat).Add(v, b), off)
	lo := new(big.Rat).Add(new(big.Rat).Sub(v, b), off)
	require.GreaterOrEqualf(t, new(big.Rat).Mul(hi, hi).Cmp(truth2), 0,
		"the gap %.17g (%v) with bound %g has its high end below the truth", gap.Value.Mag(), gap.Exactness, gap.Bound.Mag())
	if lo.Sign() > 0 {
		require.LessOrEqualf(t, new(big.Rat).Mul(lo, lo).Cmp(truth2), 0,
			"the gap %.17g (%v) with bound %g has its low end above the truth", gap.Value.Mag(), gap.Exactness, gap.Bound.Mag())
	}
}

// TestClearanceRevolveConeApexContainsTruth revolves the trapezoid
// (0,0), (1,0), (1,4), (0,1) by ±1 rad about the u axis, anchored at
// u = −2²⁰, so its slanted wall is the cone through (z, ρ) = (2²⁰, 1) and
// (2²⁰ + 1, 4). Every axis coordinate is exact, and so is the sweep, so the
// body's displacement is zero; but the cone carrier rebuilds its apex
// z = 2²⁰ − 1/3 in float, at the axis coordinate's own scale, where it rounds
// by about 4e-11. A ball of radius 1/4 centred at (−1/4, 11/4, 0), 1/4 along
// the wall's outward normal (−3, 1) from its midpoint, faces the wall
// interior, so the true gap is √10/4 − 1/4.
//
// Shown to fail: before the gap meter compared the cone carrier itself
// (revolveGapMeter.cone) and compared the walk's ends instead, the row read
// 3.68e-11 below the truth with a bound of 4.6e-15.
func TestClearanceRevolveConeApexContainsTruth(t *testing.T) {
	t.Parallel()
	const a = -1048576
	doc := decad.New()
	s := fixedPolygonSketch(t, [][2]float64{{0, 0}, {1, 0}, {1, 4}, {0, 1}})
	_, err := doc.Revolve(s, s.Profiles()[0],
		decad.SketchLine{Start: decad.Point2{U: a, V: 0}, End: decad.Point2{U: a + 1, V: 0}},
		decad.SymmetricAngle{A: units.Radians(1)})
	require.NoError(t, err)
	ballAtCentre(t, doc, -0.25, 2.75, 0, 0.25)
	requireGapEnclosesRoot(t, clearanceRow(t, doc), big.NewRat(10, 16), big.NewRat(1, 4))
}

// bigSinCos returns sin x and cos x to 300 bits by their Taylor series, whose
// terms at |x| ≤ 1 fall below 2⁻³⁰⁰ long before the 80th.
func bigSinCos(x float64) (*big.Float, *big.Float) {
	const prec = 300
	xf := new(big.Float).SetPrec(prec).SetFloat64(x)
	sin, cos := new(big.Float).SetPrec(prec), new(big.Float).SetPrec(prec)
	term := new(big.Float).SetPrec(prec).SetInt64(1)
	for n := range 80 {
		switch n % 4 {
		case 0:
			cos.Add(cos, term)
		case 1:
			sin.Add(sin, term)
		case 2:
			cos.Sub(cos, term)
		case 3:
			sin.Sub(sin, term)
		}
		term.Mul(term, xf)
		term.Quo(term, new(big.Float).SetPrec(prec).SetInt64(int64(n+1)))
	}
	return sin, cos
}

// TestClearanceRevolveCapVertexContainsTruth revolves the unit square
// u, v ∈ [0, 1] by ±φ about the line v = −2²⁰ (along u), so the body sits
// near the world origin at a radius of 2²⁰ from its axis. Every axis
// coordinate and both sweep ends are exact, so no carrier term charges
// anything; but the cap vertex at (z, ρ, φ) = (1, 2²⁰ + 1, φ) is lifted
// through the held cos φ and sin φ times that radius, and rounds by about
// 1e-10 at y ≈ 0.48. A ball of radius 1/8 sits in that corner's normal cone,
// 0.3 along each of the three faces' normals, so the corner itself is the
// nearest point and the true gap is |C − V| − 1/8 with V the exact corner.
//
// Shown to fail: before clearanceDeltaWiden read bodyGeom.widenDelta (the
// largest vertex bound beside delta), φ = 0.001 read the row 7.0e-11 off the
// truth with a bound of 3.2e-15, and φ = 0.003 1.9e-11 off with 7.1e-15.
func TestClearanceRevolveCapVertexContainsTruth(t *testing.T) {
	t.Parallel()
	const axisV = -1048576.0
	for _, phi := range []float64{0.001, 0.003} {
		t.Run(fmt.Sprintf("phi=%v", phi), func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			s := rectSketch(t, 0, 0, 1, 1)
			_, err := doc.Revolve(s, s.Profiles()[0],
				decad.SketchLine{Start: decad.Point2{U: 0, V: axisV}, End: decad.Point2{U: 1, V: axisV}},
				decad.SymmetricAngle{A: units.Radians(phi)})
			require.NoError(t, err)

			// The exact cap corner: (1, axisV + ρ·cos φ, ρ·sin φ), ρ = 2²⁰ + 1.
			sin, cos := bigSinCos(phi)
			rho := new(big.Float).SetPrec(300).SetFloat64(1 - axisV)
			vy := new(big.Float).Mul(rho, cos)
			vy.Add(vy, new(big.Float).SetFloat64(axisV))
			vz := new(big.Float).Mul(rho, sin)
			fy, _ := vy.Float64()
			fz, _ := vz.Float64()
			sn, cs := math.Sincos(phi)
			const r = 0.125
			grid := func(f float64) float64 { return math.Round(f*1024) / 1024 }
			cx, cy, cz := 1.3, grid(fy+0.3*cs-0.3*sn), grid(fz+0.3*sn+0.3*cs)
			ballAtCentre(t, doc, cx, cy, cz, r)

			gap := clearanceRow(t, doc)
			dx := new(big.Float).SetPrec(300).Sub(new(big.Float).SetFloat64(cx), new(big.Float).SetFloat64(1))
			dy := new(big.Float).SetPrec(300).Sub(new(big.Float).SetFloat64(cy), vy)
			dz := new(big.Float).SetPrec(300).Sub(new(big.Float).SetFloat64(cz), vz)
			d2 := new(big.Float).SetPrec(300).Mul(dx, dx)
			d2.Add(d2, new(big.Float).Mul(dy, dy))
			d2.Add(d2, new(big.Float).Mul(dz, dz))
			truth := new(big.Float).SetPrec(300).Sqrt(d2)
			truth.Sub(truth, new(big.Float).SetFloat64(r))
			miss := new(big.Float).SetPrec(300).Sub(new(big.Float).SetFloat64(gap.Value.Mag()), truth)
			miss.Abs(miss)
			off, _ := miss.Float64()
			require.LessOrEqualf(t, miss.Cmp(new(big.Float).SetFloat64(gap.Bound.Mag())), 0,
				"the gap %.17g (%v) sits %g from the true gap, outside its bound %g", gap.Value.Mag(), gap.Exactness, off, gap.Bound.Mag())
		})
	}
}

// TestClearanceKinkedWallContainsTruth draws a wall with a kink 2⁻³³ out of
// line at its midpoint, (0, 0) → (500, ∓2⁻³³) → (1000, 0): its two segments
// turn by about 5e-13 rad, inside the 1e-12 tolerance coalescing once merged
// them at. A ball (or a box) faces the kink, whose vertex is the nearest
// point, so the true gap is the distance to it, 2⁻³³ closer than the chord.
//
// Shown to fail: with the tolerance merge, the prism read its one wall as the
// plane through (0, 0) along the first segment and published 1 − 1.2·2⁻³³
// Exact against 1 − 2⁻³³, and the revolve read the merged wall as a cylinder
// of radius 2 and published 0.75 Exact against 0.75 − 2⁻³³.
func TestClearanceKinkedWallContainsTruth(t *testing.T) {
	t.Parallel()
	k := math.Ldexp(1, -33)
	t.Run("prism", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		body := polyPrismBody(t, doc, [][2]float64{{0, 0}, {500, -k}, {1000, 0}, {1000, 100}, {0, 100}}, 10)
		require.Len(t, body.Faces(), 7, "the kink keeps its own two walls")
		boxBodyAtZ(t, doc, 400, -3, 600, -1, 2, 6)
		requireGapEncloses(t, clearanceRow(t, doc), ratOf(1-k))
	})
	t.Run("revolve", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		s := fixedPolygonSketch(t, [][2]float64{{0, 0}, {1000, 0}, {1000, 2}, {500, 2 + k}, {0, 2}})
		_, err := doc.Revolve(s, s.Profiles()[0], uAxis, decad.FullRevolution{})
		require.NoError(t, err)
		ballAtCentre(t, doc, 500, 3, 0, 0.25)
		requireGapEncloses(t, clearanceRow(t, doc), new(big.Rat).Sub(big.NewRat(3, 4), ratOf(k)))
	})
}
