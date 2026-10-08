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

// This file pins docs/evaluator-design.md §6's swept-vertex bound: every
// junction, seam and cap vertex a revolve publishes sits within its own bound
// of the point the record denotes — the RECORDED plane point, rotated about the
// axis the record names by the angle it states. The axis coordinates (z, ρ) a
// build reads are a float re-expression of that point, snapped onto the axis
// inside the contact tolerance, so a vertex placed from them can sit off the
// recorded point even at φ = 0, where no angle enters at all.

// rectSketch draws the rectangle [u0, u1] × [v0, v1] on the world XY plane.
func rectSketch(t *testing.T, u0, v0, u1, v1 float64) *sketch.Sketch {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(u0, v0, u1, v1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	return s
}

// rat3 is an exact rational point built from float64 leaves.
func rat3(x, y, z *big.Rat) [3]*big.Rat { return [3]*big.Rat{x, y, z} }

// ratF is ratOf for a coordinate list, so a candidate reads as one line.
func ratF(x, y, z float64) [3]*big.Rat { return rat3(ratOf(x), ratOf(y), ratOf(z)) }

// ratSub is a − b over the rationals.
func ratSub(a, b *big.Rat) *big.Rat { return new(big.Rat).Sub(a, b) }

// requireRevolveVertices matches every vertex of b to its nearest candidate
// and requires the exact residual to sit inside the vertex's published bound.
// It returns the largest residual among the vertices that matched a phi0
// candidate, so a caller can prove the fixture genuinely rounds where no angle
// is involved.
func requireRevolveVertices(t *testing.T, b *decad.Body, phi0, others [][3]*big.Rat) float64 {
	t.Helper()
	vs := b.Vertices()
	require.NotEmpty(t, vs)
	worst0 := 0.0
	for _, v := range vs {
		pos := v.Position()
		res0, res := math.Inf(1), math.Inf(1)
		for _, truth := range phi0 {
			res0 = math.Min(res0, vertexResidual(pos.Value, truth))
		}
		for _, truth := range others {
			res = math.Min(res, vertexResidual(pos.Value, truth))
		}
		best := math.Min(res0, res)
		require.LessOrEqualf(t, best, 1e-3, "vertex %v matches no candidate", pos.Value)
		require.LessOrEqualf(t, best, pos.Bound.Base(),
			"vertex %v (%v) sits %g from the point it denotes, outside its bound %g", pos.Value, pos.Exactness, best, pos.Bound.Base())
		if res0 <= res {
			worst0 = math.Max(worst0, res0)
		}
	}
	return worst0
}

// TestRevolveVertexBoundCoversAxisReexpression revolves a rectangle about an
// axis the profile does not touch, a quarter turn from φ = 0. A φ = 0 cap
// vertex denotes its own recorded point, but the build places it from the
// re-expressed radius ρ = −fl(u − aU): at u = 0.1 against aU = 1 that is
// 0.9000000000000000222 rather than the exact 0.8999999999999999944, and the
// lift back adds it to aU exactly (Sterbenz), so the vertex lands 2⁻⁵⁵ mm off
// the recorded corner. The far-anchor case is the same rounding at ulp(10⁶).
//
// Shown to fail: on the build before the swept-vertex comparison, both phi0
// subtests published Exact (bound 0) vertices 2⁻⁵⁵ mm (near) and 2.3e-11 mm
// (far) off their recorded corners.
func TestRevolveVertexBoundCoversAxisReexpression(t *testing.T) {
	t.Parallel()
	for _, a := range []float64{1, 1e6 + 0.3} {
		t.Run(fmt.Sprintf("anchor=%v", a), func(t *testing.T) {
			t.Parallel()
			s := rectSketch(t, 0.1, 0, 0.5, 1)
			b, err := decad.New().Revolve(s, s.Profiles()[0],
				decad.SketchLine{Start: decad.Point2{U: a, V: 0}, End: decad.Point2{U: a, V: 1}},
				decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
			require.NoError(t, err)
			require.Len(t, b.Vertices(), 8)
			var phi0, phi1 [][3]*big.Rat
			for _, u := range []float64{0.1, 0.5} {
				for _, v := range []float64{0, 1} {
					phi0 = append(phi0, ratF(u, v, 0))
					// A quarter turn about the axis line u = a carries the
					// point to radius a − u out of the plane, on either side.
					rho := ratSub(ratOf(a), ratOf(u))
					phi1 = append(phi1,
						rat3(ratOf(a), ratOf(v), rho),
						rat3(ratOf(a), ratOf(v), new(big.Rat).Neg(rho)))
				}
			}
			worst := requireRevolveVertices(t, b, phi0, phi1)
			require.Positive(t, worst, "the fixture must genuinely round at phi0")
		})
	}
}

// TestRevolveVertexBoundCoversAxisSnap revolves a profile whose first corner
// sits 5e-10·scale off the axis, inside the contact tolerance 1e-9·scale, so
// the build snaps it onto the axis and places one shared vertex there. That
// vertex denotes the recorded corner on BOTH caps, and its bound must cover
// the whole discarded radius at each.
//
// Shown to fail: on the build before the swept-vertex comparison, the snapped
// vertex published Exact at the origin, 5e-10 mm (scale 1) and 5e-7 mm (scale
// 1000) from the recorded corner.
func TestRevolveVertexBoundCoversAxisSnap(t *testing.T) {
	t.Parallel()
	for _, scale := range []float64{1, 1000} {
		t.Run(fmt.Sprintf("scale=%v", scale), func(t *testing.T) {
			t.Parallel()
			near := 5e-10 * scale
			pts := [][2]float64{{near, 0}, {scale, 0}, {scale, scale}, {0, scale}}
			s, p := polygonSketch(t, pts)
			b, err := decad.New().Revolve(s, p,
				decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 0, V: 1}},
				decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
			require.NoError(t, err)
			var phi0, phi1 [][3]*big.Rat
			for _, pt := range pts {
				phi0 = append(phi0, ratF(pt[0], pt[1], 0))
				phi1 = append(phi1, ratF(0, pt[1], pt[0]), ratF(0, pt[1], -pt[0]))
			}
			requireRevolveVertices(t, b, phi0, phi1)

			// The snapped vertex stands for the recorded corner at both ends.
			var snapped *decad.Vertex
			for _, v := range b.Vertices() {
				if v.Position().Value == (r3.Vec{}) {
					snapped = v
				}
			}
			require.NotNil(t, snapped, "the near-axis corner must snap onto the axis")
			pos := snapped.Position()
			for _, truth := range [][3]*big.Rat{ratF(near, 0, 0), ratF(0, 0, near)} {
				res := vertexResidual(pos.Value, truth)
				require.Positive(t, res)
				require.LessOrEqualf(t, res, pos.Bound.Base(), "the snapped vertex sits %g off %v, outside its bound %g", res, truth, pos.Bound.Base())
			}
		})
	}
}

// TestRevolveVertexBoundCoversTiltedAxis revolves a rectangle a half turn about
// the tilted axis through (0,0) and (1,1). The axis direction (√½, √½) rounds,
// so every vertex is lifted through a rounded direction; a half turn maps the
// plane onto itself, carrying (u, v) to (v, u) exactly, so the φ1 cap's
// denoted corners are rational and the comparison is exact at both caps.
//
// Shown to fail: on the build before the swept-vertex comparison, a φ = π cap
// vertex published a bound of 3.3e-16 against a 4.4e-16 residual from its
// mirrored corner, since nothing charged the re-expressed (z, ρ) rounding or
// the rounded axis direction. Dropping only the axis's own bounds from the
// comparison (a zero revolvemesh.AxisBound in sweptGap) turns it red again
// at 3.0e-16.
func TestRevolveVertexBoundCoversTiltedAxis(t *testing.T) {
	t.Parallel()
	s := rectSketch(t, 2, 0, 3, 1)
	b, err := decad.New().Revolve(s, s.Profiles()[0],
		decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 1}},
		decad.AngleExtent{A: units.Degrees(180), Dir: decad.Along})
	require.NoError(t, err)
	require.Len(t, b.Vertices(), 8)
	var phi0, phi1 [][3]*big.Rat
	for _, u := range []float64{2, 3} {
		for _, v := range []float64{0, 1} {
			phi0 = append(phi0, ratF(u, v, 0))
			phi1 = append(phi1, ratF(v, u, 0))
		}
	}
	requireRevolveVertices(t, b, phi0, phi1)
}

// TestRevolveChainVertexBoundCoversAxisReexpression is the RevolveChain build's
// reading of the same φ = 0 rounding: an open chain's junctions, its two free
// ends included, carry the bound a closed profile's junctions do.
//
// Shown to fail: on the build before the swept-vertex comparison, the chain's
// φ = 0 vertices at u = 0.1 published Exact while 2⁻⁵⁵ mm off.
func TestRevolveChainVertexBoundCoversAxisReexpression(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	pts := [][2]float64{{0.1, 0}, {0.5, 0}, {0.5, 1}, {0.1, 1}}
	sp := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		sp[i] = s.CreatePoint(p[0], p[1])
	}
	s.Fix(sp[0])
	for i := range len(sp) - 1 {
		s.CreateLine(sp[i], sp[i+1])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Chains(), 1)
	b, err := decad.New().RevolveChain(s, s.Chains()[0],
		decad.SketchLine{Start: decad.Point2{U: 1, V: 0}, End: decad.Point2{U: 1, V: 1}},
		decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	require.NoError(t, err)
	require.Len(t, b.Vertices(), 8)
	var phi0, phi1 [][3]*big.Rat
	for _, pt := range pts {
		phi0 = append(phi0, ratF(pt[0], pt[1], 0))
		rho := ratSub(ratOf(1), ratOf(pt[0]))
		phi1 = append(phi1, rat3(ratOf(1), ratOf(pt[1]), rho), rat3(ratOf(1), ratOf(pt[1]), new(big.Rat).Neg(rho)))
	}
	worst := requireRevolveVertices(t, b, phi0, phi1)
	require.Positive(t, worst, "the fixture must genuinely round at phi0")
}

// TestRevolveVertexIntegerStaysExact is the exact half: an integer rectangle
// about a coordinate axis re-expresses, lifts and rotates by 0 exactly, so a
// full turn's seam vertices and a partial turn's φ = 0 cap vertices keep Exact
// with a zero bound. Shown to fail: replacing sweptVertex's comparison with a
// magnitude charge (proofbound.RigidRoundAllow at the held point's magnitude)
// turns both subtests red.
func TestRevolveVertexIntegerStaysExact(t *testing.T) {
	t.Parallel()
	t.Run("full", func(t *testing.T) {
		t.Parallel()
		s := rectSketch(t, 0, 5, 10, 15)
		b, err := decad.New().Revolve(s, s.Profiles()[0], uAxis, decad.FullRevolution{})
		require.NoError(t, err)
		require.Len(t, b.Vertices(), 4)
		for _, v := range b.Vertices() {
			require.Equal(t, decad.Exact, v.Position().Exactness)
			require.Equal(t, units.Millimeters(0), v.Position().Bound)
		}
	})
	t.Run("partial", func(t *testing.T) {
		t.Parallel()
		s := rectSketch(t, 0, 5, 10, 15)
		b, err := decad.New().Revolve(s, s.Profiles()[0], uAxis, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
		require.NoError(t, err)
		start := 0
		for _, v := range b.Vertices() {
			if v.Position().Value.Z != 0 {
				continue
			}
			start++
			require.Equal(t, decad.Exact, v.Position().Exactness)
			require.Equal(t, units.Millimeters(0), v.Position().Bound)
		}
		require.Equal(t, 4, start)
	})
}

// fullRevolveTiltedCentroid is the exact centroid of a full revolution of the
// world-space rectangle [x0, x1] × [y0, y1] (z = 0) about the line through
// (ax, ay, 0) along (1, 1, 0). With X = x − ax and Y = y − ay the axial
// coordinate is (X + Y)/√2 and the radius |X − Y|/√2, so the centroid sits on
// the axis at (ax, ay) + (R/2)(1, 1) with R = ∫(X² − Y²) dA / ∫(X − Y) dA,
// rational for every rational rectangle.
func fullRevolveTiltedCentroid(ax, ay, x0, y0, x1, y1 *big.Rat) [3]*big.Rat {
	X0, X1 := ratSub(x0, ax), ratSub(x1, ax)
	Y0, Y1 := ratSub(y0, ay), ratSub(y1, ay)
	pow := func(r *big.Rat, n int) *big.Rat {
		out := big.NewRat(1, 1)
		for range n {
			out.Mul(out, r)
		}
		return out
	}
	diff := func(hi, lo *big.Rat, n int) *big.Rat { return ratSub(pow(hi, n), pow(lo, n)) }
	mul := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Mul(a, b) }
	third, half := big.NewRat(1, 3), big.NewRat(1, 2)
	xx := mul(mul(diff(X1, X0, 3), third), ratSub(Y1, Y0))
	yy := mul(mul(diff(Y1, Y0, 3), third), ratSub(X1, X0))
	x := mul(mul(diff(X1, X0, 2), half), ratSub(Y1, Y0))
	y := mul(mul(diff(Y1, Y0, 2), half), ratSub(X1, X0))
	r := new(big.Rat).Quo(ratSub(xx, yy), ratSub(x, y))
	r.Mul(r, half)
	return rat3(new(big.Rat).Add(ax, r), new(big.Rat).Add(ay, r), new(big.Rat))
}

// TestRevolveCentroidBoundCoversTiltedAxis checks the centroid's lift
// A3 + W·axial against the exact centroid of a full turn, for axes whose own
// direction or anchor rounds and frames whose anchor lift rounds.
//
// Shown to fail: on the build before revolveCentroidLift.charge,
// far-frame-anchor published a 5.7e-15 bound against a 2.3e-11 residual (the
// anchor's own aUBound, uncharged in the lift) and far-tilted-frame a
// 1.6e-322 bound against 4.4e-11 (A3's frame-lift rounding at the frame
// origin's magnitude). Dropping the charge's anchor term alone turns
// far-frame-anchor red, and its A3 rounding term alone far-tilted-frame. The
// sketch-line and construction-axis subtests passed before the charge: the
// direction terms are smaller than the axial coordinate's own bound, which
// axisMoments already widens by dUBound/dVBound, in every fixture here, so
// no subtest isolates them.
func TestRevolveCentroidBoundCoversTiltedAxis(t *testing.T) {
	t.Parallel()
	t.Run("sketch-line", func(t *testing.T) {
		t.Parallel()
		s := rectSketch(t, 1000, 0, 1001, 1)
		b, err := decad.New().Revolve(s, s.Profiles()[0],
			decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 1}}, decad.FullRevolution{})
		require.NoError(t, err)
		requireCentroidCovers(t, b, fullRevolveTiltedCentroid(new(big.Rat), new(big.Rat),
			ratOf(1000), ratOf(0), ratOf(1001), ratOf(1)))
	})
	t.Run("construction-axis", func(t *testing.T) {
		t.Parallel()
		o := r3.NewVec(0.1, 0.2, 0)
		f, err := r3.NewFrame(o, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
		require.NoError(t, err)
		w := sketch.NewWorld()
		plane, err := w.CreatePlaneFromFrame(f)
		require.NoError(t, err)
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		const u0, v0 = 1e6 + 5, 1e6
		rect := s.CreateRectangle(u0, v0, u0+1, v0+1)
		s.Fix(rect.A)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		axis := decad.ConstructionAxis{Origin: r3.NewVec(1e6+0.1, 1e6+0.3, 0), Dir: r3.NewVec(1, 1, 0)}
		b, err := decad.New().Revolve(s, s.Profiles()[0], axis, decad.FullRevolution{})
		require.NoError(t, err)
		lift := func(o, c float64) *big.Rat { return new(big.Rat).Add(ratOf(o), ratOf(c)) }
		requireCentroidCovers(t, b, fullRevolveTiltedCentroid(ratOf(axis.Origin.X), ratOf(axis.Origin.Y),
			lift(o.X, u0), lift(o.Y, v0), lift(o.X, u0+1), lift(o.Y, v0+1)))
	})
	t.Run("far-frame-anchor", func(t *testing.T) {
		t.Parallel()
		// The sketch plane sits 10⁶ mm out and the construction axis passes
		// through world x = 0.1 along +y, so the anchor's plane-local
		// coordinate fl(0.1 − 10⁶) rounds at ulp(10⁶) while the anchor itself
		// lifts back near the world origin. The profile is the world square
		// [0.5, 1.5] × [−0.5, 0.5], centred on the axis's own z = 0, so the
		// exact centroid is (0.1, 0, 0) and its axial coordinate carries none
		// of the anchor's rounding.
		o := r3.NewVec(1e6, 0, 0)
		f, err := r3.NewFrame(o, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
		require.NoError(t, err)
		w := sketch.NewWorld()
		plane, err := w.CreatePlaneFromFrame(f)
		require.NoError(t, err)
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		rect := s.CreateRectangle(0.5-1e6, -0.5, 1.5-1e6, 0.5)
		s.Fix(rect.A)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		axis := decad.ConstructionAxis{Origin: r3.NewVec(0.1, 0, 0), Dir: r3.NewVec(0, 1, 0)}
		b, err := decad.New().Revolve(s, s.Profiles()[0], axis, decad.FullRevolution{})
		require.NoError(t, err)
		requireCentroidCovers(t, b, ratF(0.1, 0, 0))
	})
	t.Run("far-tilted-frame", func(t *testing.T) {
		t.Parallel()
		// A tilted sketch plane whose origin sits 10⁶ mm out, with the axis
		// anchored 10⁶ mm back along U, so the anchor lifts to a point near the
		// world origin through products that round at ulp(10⁶). The profile is
		// centred on v = 0, so the exact centroid is the anchor's own exact
		// lift O + U·a.
		const a = 1e6
		f, err := r3.NewFrame(r3.NewVec(6e5, 8e5, 0), r3.NewVec(-0.6, -0.8, 0), r3.NewVec(0.8, -0.6, 0))
		require.NoError(t, err)
		w := sketch.NewWorld()
		plane, err := w.CreatePlaneFromFrame(f)
		require.NoError(t, err)
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		rect := s.CreateRectangle(a+0.5, -0.5, a+1.5, 0.5)
		s.Fix(rect.A)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		b, err := decad.New().Revolve(s, s.Profiles()[0],
			decad.SketchLine{Start: decad.Point2{U: a, V: 0}, End: decad.Point2{U: a, V: 1}}, decad.FullRevolution{})
		require.NoError(t, err)
		o, u := f.Origin(), f.U()
		lift := func(oc, uc float64) *big.Rat {
			return new(big.Rat).Add(ratOf(oc), new(big.Rat).Mul(ratOf(uc), ratOf(a)))
		}
		requireCentroidCovers(t, b, rat3(lift(o.X, u.X), lift(o.Y, u.Y), lift(o.Z, u.Z)))
	})
}

func requireCentroidCovers(t *testing.T, b *decad.Body, truth [3]*big.Rat) {
	t.Helper()
	c, err := b.Centroid()
	require.NoError(t, err)
	res := vertexResidual(c.Value, truth)
	require.LessOrEqualf(t, res, c.Bound.Base(), "centroid %v sits %g from the exact centroid, outside its bound %g", c.Value, res, c.Bound.Base())
	t.Logf("centroid residual %g, bound %g", res, c.Bound.Base())
}
