package apitest_test

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

// This file pins four readings docs/clearance-design.md §2 and §5 owe a
// published Clearance row: a cone carrier is measured as its float apex and
// window stand, a topology vertex widens the row by its own proven bound, a
// coalesced wall stands only for exactly collinear segments, and a coarse
// witness bounds the gap only with its own proven gap added. Each fixture's
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

// TestClearanceRevolveSteepConeContainsTruth revolves the triangle
// (2²⁰, 0), (2²⁰ + a·K, b·K), (2²⁰ + a·K, 0) a full turn about the u axis,
// K = 2¹⁶, so its slanted wall is a cone whose apex sits on the axis 2²⁰ from
// the origin and whose half angle atan(b/a), with a² + b² = c², is no
// rational multiple of π. Every coordinate is an integer, so the carriers
// match the record exactly. A ball of radius 1/4 is centred c along the
// wall's outward normal (−b, a)/c from the wall point at slant c·K/2 from the
// apex, so the true gap is c − 1/4 and the row must hold it.
//
// No failure reproduced through this public path. With the cone × sphere
// cell reading math.Sincos of the carrier's float half angle, the kernel's
// held interval was the single point 4.7500000000145519 (a = 3, b = 4) and
// 12.750000000029104 (a = 5, b = 12), marked exact and about 1.5e-11 off the
// truth, but both bodies' full-turn angular term widened the row to a bound
// of about 2e-9, which held the truth. TestConeCellsReadTheSlope
// (internal/clearance) pins the held interval itself and was seen red.
func TestClearanceRevolveSteepConeContainsTruth(t *testing.T) {
	t.Parallel()
	const z0, k = 1048576.0, 65536.0
	for _, tc := range []struct{ a, b, c float64 }{{3, 4, 5}, {5, 12, 13}} {
		t.Run(fmt.Sprintf("slope=%v/%v", tc.b, tc.a), func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			s := fixedPolygonSketch(t, [][2]float64{{z0, 0}, {z0 + tc.a*k, tc.b * k}, {z0 + tc.a*k, 0}})
			_, err := doc.Revolve(s, s.Profiles()[0], uAxis, decad.FullRevolution{})
			require.NoError(t, err)
			const m = k / 2
			ballAtCentre(t, doc, z0+tc.a*m-tc.b, tc.b*m+tc.a, 0, 0.25)
			requireGapEncloses(t, clearanceRow(t, doc), new(big.Rat).Sub(ratOf(tc.c), big.NewRat(1, 4)))
		})
	}
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

// witnessConeBody revolves the triangle (0, 0), (e, e), (e, 0),
// e = 1 + 2⁻³², by ±1 rad about the u axis, anchored at u = −2²⁰. Its slanted
// wall is the 45° cone ρ = x from the apex at the origin, read in the axis
// frame from z = 2²⁰ to z = 2²⁰ + e. Every axis coordinate is exact, and so
// is the carrier, so the body's displacement is zero. The cone's one witness
// sits at the mean of the two axial ends, 2²¹ + e, which rounds to 2²¹ + 1, so
// the witness lands 2⁻³³ behind the wall along the axis at ρ = e/2: 2⁻³³/√2,
// about 8.2e-11, outside the cone.
func witnessConeBody(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	e := 1 + math.Ldexp(1, -32)
	s := fixedPolygonSketch(t, [][2]float64{{0, 0}, {e, e}, {e, 0}})
	const a = -1048576
	body, err := doc.Revolve(s, s.Profiles()[0],
		decad.SketchLine{Start: decad.Point2{U: a, V: 0}, End: decad.Point2{U: a + 1, V: 0}},
		decad.SymmetricAngle{A: units.Radians(1)})
	require.NoError(t, err)
	return body
}

// witnessSliceBody extrudes the quarter disc of radius r about (cx, cy),
// bounded by the arc from −90° to 0°, over z ∈ [−1/2, 1/2]. Its cylinder
// wall's mid witness sits at −45° and z = 0.
func witnessSliceBody(t *testing.T, doc *decad.Document, cx, cy, r float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), -0.5)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	c, p1, p2 := s.CreatePoint(cx, cy), s.CreatePoint(cx, cy-r), s.CreatePoint(cx+r, cy)
	s.Fix(c)
	s.Fix(p1)
	s.Fix(p2)
	s.CreateArc(c, p1, p2)
	s.CreateLine(p2, c)
	s.CreateLine(c, p1)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(1), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// TestClearanceCoarseWitnessContainsTruth reads a cone × cylinder pair, which
// no closed-form cell solves, so its upper end comes from the coarse
// enclosure's closest witness pair (docs/clearance-design.md §5). The cone is
// witnessConeBody's, its witness about 8.2e-11 outside the wall. A quarter
// disc of radius 1/4 about (−1/2, 3/2) faces the wall's φ = 0 generator, the
// line y = x, from √2 away, and its arc's mid witness at −45° is the slice's
// nearest point. Every point of the cone has y ≤ x and every point of the
// slice lies at least √2 − 1/4 from that half-space, which the cone point
// (1/2, 1/2, 0) attains, so the true gap is √2 − 1/4. The two faces' boxes lie
// about 0.354 apart, so the row is decided, with that lower end.
//
// "anchored far" reads the pair as built. "placed far" translates both bodies
// by 2²⁰ along every axis, so both carry the placement's own displacement.
//
// Shown to fail: before the coarse enclosure added each witness's own proven
// gap (clearance.Witness), "anchored far" read an upper end of
// 1.1642135622908296, 8.2e-11 below the truth, because the envelope charge it
// carried, about 5e-14, is read off the witnesses' own coordinates near the
// origin while the witness rounded at the axis anchor's scale. "placed far"
// held: its displacement, about 1.1e-7, covered the witness. Dropping both
// gaps from clearance.CellSink.Coarse turns "anchored far" red again.
func TestClearanceCoarseWitnessContainsTruth(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		off  r3.Vec
	}{
		{name: "anchored far"},
		{name: "placed far", off: r3.NewVec(1<<20, 1<<20, 1<<20)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			cone := witnessConeBody(t, doc)
			slice := witnessSliceBody(t, doc, -0.5, 1.5, 0.25)
			if tc.off != (r3.Vec{}) {
				tr, err := r3.Translation(tc.off)
				require.NoError(t, err)
				_, err = cone.Placed(t.Context(), tr)
				require.NoError(t, err)
				_, err = slice.Placed(t.Context(), tr)
				require.NoError(t, err)
			}
			requireGapEnclosesSurd(t, clearanceRow(t, doc), big.NewRat(-1, 4), 1, big.NewRat(2, 1))
		})
	}
}

// appleBody revolves the circular segment cut from the circle of radius 5
// about (3, 0) by the v axis a full turn about that axis: a spindle torus
// (minor radius 5 above major radius 3), the apple whose top and bottom rings
// sit at ρ = 3, v = ±5.
func appleBody(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	c, a, b := s.CreatePoint(3, 0), s.CreatePoint(0, -4), s.CreatePoint(0, 4)
	s.Fix(c)
	s.Fix(a)
	s.Fix(b)
	s.CreateArc(c, a, b)
	s.CreateLine(b, a)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	body, err := doc.Revolve(s, s.Profiles()[0],
		decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 0, V: 1}}, decad.FullRevolution{})
	require.NoError(t, err)
	return body
}

// TestClearanceTiltedBoxContainsTruth reads two spindle tori, which only the
// coarse enclosure reads (docs/clearance-design.md §5), so the row's lower end
// is the two faces' box distance. The lower apple is turned 1e-8 rad about
// the x axis, so its axis is (0, 1, s) with s = sin(1e-8) and its spine
// circle of radius 3 rises 3s above y = 0 on one side; the upper apple is
// moved 11 up, its bottom ring at y = 6. The lower apple's rotated ring point
// (0, 5c + 3s, 5s − 3c), with c and s the placement's own matrix entries,
// lies within √((1 − 5c − 3s + 5)² + (5s − 3c + 3)²), about 1 − 3e-8, of the
// upper apple's point (0, 6, −3), so the row must hold a gap at most that.
//
// Shown to fail: before every face box was built over exact rationals and
// rounded outward (internal/clearance/face_box.go), the spine circle's box
// read its extent along y as 3·√(1 − a_y²) with a_y = fl(cos 1e-8) = 1, which
// is zero, and the row's lower end read 0.99999999999885281, about 3e-8 above
// the gap.
func TestClearanceTiltedBoxContainsTruth(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	lower := appleBody(t, doc)
	rot, err := r3.Rotation(r3.NewVec(1, 0, 0), units.Radians(1e-8))
	require.NoError(t, err)
	_, err = lower.Placed(t.Context(), rot)
	require.NoError(t, err)
	upper := appleBody(t, doc)
	up, err := r3.Translation(r3.NewVec(0, 11, 0))
	require.NoError(t, err)
	_, err = upper.Placed(t.Context(), up)
	require.NoError(t, err)

	// The rotation's own matrix: its z column is (0, −s, c), read exactly.
	col := rot.ApplyDir(r3.NewVec(0, 0, 1))
	s, c := ratOf(-col.Y), ratOf(col.Z)
	five, three := big.NewRat(5, 1), big.NewRat(3, 1)
	py := new(big.Rat).Add(new(big.Rat).Mul(five, c), new(big.Rat).Mul(three, s))
	pz := new(big.Rat).Sub(new(big.Rat).Mul(five, s), new(big.Rat).Mul(three, c))
	dy := new(big.Rat).Sub(big.NewRat(6, 1), py)
	dz := new(big.Rat).Sub(big.NewRat(-3, 1), pz)
	gap2 := new(big.Rat).Add(new(big.Rat).Mul(dy, dy), new(big.Rat).Mul(dz, dz))

	gap := clearanceRow(t, doc)
	lo := new(big.Rat).Sub(ratOf(gap.Value.Mag()), ratOf(gap.Bound.Mag()))
	require.Positive(t, lo.Sign())
	require.LessOrEqualf(t, new(big.Rat).Mul(lo, lo).Cmp(gap2), 0,
		"the gap %.17g (%v) with bound %g has its low end above the truth", gap.Value.Mag(), gap.Exactness, gap.Bound.Mag())
}
