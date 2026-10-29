package apitest_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The class-B blend fixtures of agent-docs/brep-modify-design.md §4.4: a
// Fillet or Chamfer of the long edges where a class-B cut meets the shaft
// cylinder (revolve_shaft_class_b_test.go's D-flat and keyway) builds from the
// record's exact carriers. Each removed corner is a closed form in the
// section (R = 10, flat at h = 7), and the volume encloses
// shaftLess(segment + corners) within its bound.
//
// Shown-to-fail: without Receiver's displaced arm every build here refuses
// SB1 again. Without PD4 the one-edge fillet and the keyway floor fillet
// build. Without PD1 the cap-loop chamfer reaches the prism path and refuses
// with the prism's displaced-receiver text. Building the fillet result with
// the receiver's displacement makes the cap-loop chamfer of composition refuse.

// shaftEdges selects the long straight edges along x whose (y, z) is one of
// at, within 1e-6, and requires exactly one per point.
func shaftEdges(t *testing.T, b *decad.Body, at ...[2]float64) *decad.EdgeQuery {
	t.Helper()
	x := r3.NewVec(1, 0, 0)
	var q *decad.EdgeQuery
	n := 0
	for _, e := range b.Edges() {
		if _, ok := e.Curve().(decad.Line3); !ok {
			continue
		}
		a, c := e.Start().Position().Value, e.End().Position().Value
		if math.Abs(a.X-c.X) < shaftL/2 {
			continue
		}
		for _, p := range at {
			if math.Abs(a.Y-p[0]) > 1e-6 || math.Abs(a.Z-p[1]) > 1e-6 {
				continue
			}
			preds := []decad.EdgePredicate{decad.EndpointAt(a), decad.ParallelTo(x)}
			if q == nil {
				q = decad.Edges(preds...)
			} else {
				q.Or(preds...)
			}
			n++
		}
	}
	require.Equal(t, len(at), n, "one long edge at each section point")
	return q.Exactly(n)
}

// The section points of the D-flat's and the keyway's long edges.
var (
	flatCorners    = [][2]float64{{math.Sqrt(51), flatZ}, {-math.Sqrt(51), flatZ}}
	keyTopCorners  = [][2]float64{{3, math.Sqrt(91)}, {-3, math.Sqrt(91)}}
	keyFloorCorner = [][2]float64{{3, flatZ}, {-3, flatZ}}
)

// bigPoint is a section point (u, v) at the closed forms' precision.
type bigPoint [2]*big.Float

func bp(u, v *big.Float) bigPoint { return bigPoint{u, v} }

func bAdd(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(junctionPrec).Add(a, b) }
func bSub(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(junctionPrec).Sub(a, b) }
func bMul(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(junctionPrec).Mul(a, b) }
func bQuo(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(junctionPrec).Quo(a, b) }
func bSqrt(a *big.Float) *big.Float   { return new(big.Float).SetPrec(junctionPrec).Sqrt(a) }

// bAtan is atan(y) for y ≥ 0, through π/2 − atan(1/y) above 1.
func bAtan(y *big.Float) *big.Float {
	if y.Cmp(jf(1)) <= 0 {
		return junctionAtan(y)
	}
	return bSub(bQuo(junctionPi(), jf(2)), junctionAtan(bQuo(jf(1), y)))
}

// bSinCos is sin(x) and cos(x) by their Taylor series, for |x| ≤ 1.
func bSinCos(x *big.Float) (*big.Float, *big.Float) {
	sin, cos := jf(0), jf(0)
	term := jf(1) // x^n / n!
	tiny := new(big.Float).SetMantExp(jf(1), -junctionPrec-8)
	for n := 0; n < 4 || new(big.Float).Abs(term).Cmp(tiny) > 0; n++ {
		switch n % 4 {
		case 0:
			cos = bAdd(cos, term)
		case 1:
			sin = bAdd(sin, term)
		case 2:
			cos = bSub(cos, term)
		case 3:
			sin = bSub(sin, term)
		}
		term = bQuo(bMul(term, x), jf(float64(n+1)))
	}
	return sin, cos
}

// shoelace is the unsigned area of the polygon through pts.
func shoelace(pts ...bigPoint) *big.Float {
	sum := jf(0)
	for i, p := range pts {
		q := pts[(i+1)%len(pts)]
		sum = bAdd(sum, bSub(bMul(p[0], q[1]), bMul(q[0], p[1])))
	}
	return bQuo(new(big.Float).Abs(sum), jf(2))
}

// circleSegment is (R²/2)·(β − sin β) for the chord from a to b on the
// shaft circle, with sin β = |a × b| / R² and β = atan(|a × b| / (a · b)).
func circleSegment(a, b bigPoint) *big.Float {
	cross := new(big.Float).Abs(bSub(bMul(a[0], b[1]), bMul(a[1], b[0])))
	dot := bAdd(bMul(a[0], b[0]), bMul(a[1], b[1]))
	r2 := jf(shaftR * shaftR)
	beta := bAtan(bQuo(cross, dot))
	return bMul(bQuo(r2, jf(2)), bSub(beta, bQuo(cross, r2)))
}

// filletCorner is the area a ball of radius r centred at c removes at the
// line–circle corner k: the quadrilateral (k, ta, c, tb) less the ball's
// sector of angle alpha, plus the circle segment beyond the chord tb–k. tb is
// c scaled out to the shaft circle.
func filletCorner(k, ta, c bigPoint, r float64, alpha *big.Float) *big.Float {
	scale := jf(shaftR / (shaftR - r))
	tb := bp(bMul(c[0], scale), bMul(c[1], scale))
	area := bSub(shoelace(k, ta, c, tb), bMul(jf(r*r/2), alpha))
	return bAdd(area, circleSegment(tb, k))
}

// flatFilletCorner is the D-flat's corner at (√51, 7) under a ball of
// radius r: c = (√((R − r)² − (h − r)²), h − r), ta = (c_u, h), and the
// sector angle atan(c_u / (h − r)).
func flatFilletCorner(r float64) *big.Float {
	cu := bSqrt(jf((shaftR-r)*(shaftR-r) - (flatZ-r)*(flatZ-r)))
	c := bp(cu, jf(flatZ-r))
	return filletCorner(bp(shaftSqrt(51), jf(flatZ)), bp(cu, jf(flatZ)), c, r, bAtan(bQuo(cu, jf(flatZ-r))))
}

// keyFilletCorner is the keyway's top corner at (3, √91) under a ball of
// radius r: c = (3 + r, √((R − r)² − (3 + r)²)), ta = (3, c_v), and the
// obtuse sector angle π − atan(c_v / (3 + r)).
func keyFilletCorner(r float64) *big.Float {
	cv := bSqrt(jf((shaftR-r)*(shaftR-r) - (3+r)*(3+r)))
	c := bp(jf(3+r), cv)
	alpha := bSub(junctionPi(), bAtan(bQuo(cv, jf(3+r))))
	return filletCorner(bp(jf(3), shaftSqrt(91)), bp(jf(3), cv), c, r, alpha)
}

// flatChamferCorner is the D-flat's corner at k = (√51, 7) under an equal
// chamfer d: ta = (√51 − d, 7) and tb = R·dir(φ_k − d/R), cos φ_k = √51/10,
// sin φ_k = 7/10; the triangle (k, ta, tb) plus the segment beyond tb–k.
func flatChamferCorner(d float64) *big.Float {
	k := bp(shaftSqrt(51), jf(flatZ))
	s, c := bSinCos(jf(d / shaftR))
	cosK, sinK := bQuo(k[0], jf(shaftR)), jf(flatZ/shaftR)
	tb := bp(bMul(jf(shaftR), bAdd(bMul(cosK, c), bMul(sinK, s))),
		bMul(jf(shaftR), bSub(bMul(sinK, c), bMul(cosK, s))))
	return bAdd(shoelace(k, bp(bSub(k[0], jf(d)), jf(flatZ)), tb), circleSegment(tb, k))
}

// twice is 2·a.
func twice(a *big.Float) *big.Float { return bMul(jf(2), a) }

// dFlat is the class-B D-flat cut of a fresh shaft.
func dFlat(t *testing.T, shaft func(*testing.T, *retentionScene) *decad.Body) *decad.Body {
	t.Helper()
	sc := newRetentionScene()
	cut, err := decad.Cut(t.Context(), shaft(t, sc), shaftTool(t, sc, flatPlane(t, sc), -5, -20, shaftL+5, 20))
	require.NoError(t, err)
	return cut
}

// keyway is the class-B keyway cut of a fresh shaft.
func keyway(t *testing.T) *decad.Body {
	t.Helper()
	sc := newRetentionScene()
	cut, err := decad.Cut(t.Context(), revolvedShaft(t, sc), shaftTool(t, sc, flatPlane(t, sc), -5, -3, shaftL+5, 3))
	require.NoError(t, err)
	return cut
}

// requireBlendedShaft checks a blended class-B shaft: no Faceted face, the
// plane count, the cylinder count with blends of them at radius blend, a
// volume within its tiny bound of truth, and a mesh that verifies.
func requireBlendedShaft(t *testing.T, got *decad.Body, planes, cylinders, blends int, blend float64, truth *big.Float) (float64, float64) {
	t.Helper()
	nPlanes, nCylinders, nBlends := 0, 0, 0
	for _, f := range got.Faces() {
		switch s := f.Surface().(type) {
		case decad.Faceted:
			require.Fail(t, "a class-B blend has no Faceted face")
		case decad.Cylinder:
			nCylinders++
			if math.Abs(s.Radius.Base()-blend) < 1e-9 {
				nBlends++
			}
		case decad.Plane:
			nPlanes++
		}
	}
	require.Equal(t, planes, nPlanes)
	require.Equal(t, cylinders, nCylinders)
	require.Equal(t, blends, nBlends)
	volume, err := got.Volume()
	require.NoError(t, err)
	value, err := volume.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	bound, err := volume.Bound.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.Less(t, bound, 1e-6, `the exact arm, not the mesh path, measures the blend`)
	requireJunctionEncloses(t, `blended shaft volume`, value, bound, truth)
	mesh, err := got.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	return value, bound
}

// requireAgreesWithMeshTwin checks a value against the Transform-placed
// tool's mesh-path blend of the same flat (its retained blend), within both
// bounds.
func requireAgreesWithMeshTwin(t *testing.T, value, bound float64, fillet bool, size float64) {
	t.Helper()
	sc := newRetentionScene()
	cut, err := decad.Cut(t.Context(), revolvedShaft(t, sc), flatTool(t, sc))
	require.NoError(t, err)
	var twin *decad.Body
	if fillet {
		twin, err = decad.Fillet(t.Context(), cut, flatEdges(t, cut), units.Millimeters(size))
	} else {
		twin, err = decad.Chamfer(t.Context(), cut, flatEdges(t, cut), units.Millimeters(size))
	}
	require.NoError(t, err)
	v, err := twin.Volume()
	require.NoError(t, err)
	tv, err := v.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	tb, err := v.Bound.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(value-tv), bound+tb)
}

func TestRevolvedShaftClassBBlend(t *testing.T) {
	t.Parallel()

	flatFillet := shaftLess(bAdd(flatSegment(), twice(flatFilletCorner(1))))

	t.Run("D-flat fillet", func(t *testing.T) {
		t.Parallel()
		cut := dFlat(t, extrudedShaft)
		got, err := decad.Fillet(t.Context(), cut, flatEdges(t, cut), units.Millimeters(1))
		require.NoError(t, err)
		requireBlendedShaft(t, got, 3, 3, 2, 1, flatFillet)

		// The Revolve twin is the same class-B record, so it measures the
		// same value and bound bit for bit.
		rcut := dFlat(t, revolvedShaft)
		twin, err := decad.Fillet(t.Context(), rcut, flatEdges(t, rcut), units.Millimeters(1))
		require.NoError(t, err)
		v, err := twin.Volume()
		require.NoError(t, err)
		mine, err := got.Volume()
		require.NoError(t, err)
		require.Equal(t, mine.Value, v.Value)
		require.Equal(t, mine.Bound, v.Bound)
	})

	t.Run("D-flat fillet r=2", func(t *testing.T) {
		t.Parallel()
		cut := dFlat(t, revolvedShaft)
		got, err := decad.Fillet(t.Context(), cut, flatEdges(t, cut), units.Millimeters(2))
		require.NoError(t, err)
		value, bound := requireBlendedShaft(t, got, 3, 3, 2, 2, shaftLess(bAdd(flatSegment(), twice(flatFilletCorner(2)))))
		requireAgreesWithMeshTwin(t, value, bound, true, 2)
	})

	t.Run("D-flat chamfer", func(t *testing.T) {
		t.Parallel()
		cut := dFlat(t, revolvedShaft)
		got, err := decad.Chamfer(t.Context(), cut, flatEdges(t, cut), units.Millimeters(1))
		require.NoError(t, err)
		value, bound := requireBlendedShaft(t, got, 5, 1, 0, 1, shaftLess(bAdd(flatSegment(), twice(flatChamferCorner(1)))))
		requireAgreesWithMeshTwin(t, value, bound, false, 1)

		// The chamfer carries the crossings' displacement (PD7), so a further
		// lateral blend refuses as any displaced prism does.
		_, err = decad.Fillet(t.Context(), got, flatEdges(t, got), units.Millimeters(0.5))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, `proven displacement`)
	})

	t.Run("fillet then cap-loop chamfer", func(t *testing.T) {
		t.Parallel()
		cut := dFlat(t, revolvedShaft)
		filleted, err := decad.Fillet(t.Context(), cut, flatEdges(t, cut), units.Millimeters(1))
		require.NoError(t, err)
		// The fillet result is an exact prism, so the cap-loop chamfer of its
		// x = 40 end builds.
		end := decad.Faces(decad.Facing(r3.NewVec(1, 0, 0)))
		faces, err := end.SelectFaces(filleted)
		require.NoError(t, err)
		require.Len(t, faces, 1)
		loop := decad.Edges(decad.OuterLoopOf(faces[0].Origins()[0]))
		const d = 0.5
		got, err := decad.Chamfer(t.Context(), filleted, loop, units.Millimeters(d))
		require.NoError(t, err)
		// The filleted section is smooth and convex, so the band removes the
		// d × d right triangle swept along it: by Pappus, (d²/2)·(P − 2π·d/3)
		// for its perimeter P = 2·√45 + 2·α + R·(2π − 2α), α = atan(√45/6),
		// the turn of each r = 1 fillet arc.
		alpha := bAtan(bQuo(shaftSqrt(45), jf(6)))
		twoPi := bMul(jf(2), junctionPi())
		perimeter := bAdd(bAdd(twice(shaftSqrt(45)), twice(alpha)), bMul(jf(shaftR), bSub(twoPi, twice(alpha))))
		band := bMul(jf(d*d/2), bSub(perimeter, bMul(twoPi, jf(d/3))))
		volume, err := got.Volume()
		require.NoError(t, err)
		value, err := volume.Value.In(units.CubicMillimeter)
		require.NoError(t, err)
		bound, err := volume.Bound.In(units.CubicMillimeter)
		require.NoError(t, err)
		require.Less(t, bound, 1e-6, `the cap band measures exactly`)
		requireJunctionEncloses(t, `filleted and chamfered shaft volume`, value, bound, bSub(flatFillet, band))
	})

	t.Run("keyway", func(t *testing.T) {
		t.Parallel()
		const r = 0.5
		top := twice(keyFilletCorner(r))
		cut := keyway(t)
		got, err := decad.Fillet(t.Context(), cut, shaftEdges(t, cut, keyTopCorners...), units.Millimeters(r))
		require.NoError(t, err)
		requireBlendedShaft(t, got, 5, 3, 2, r, shaftLess(bAdd(keySeat(), top)))

		// The two floor corners are concave, so their fillets fill in
		// r²(1 − π/4) each.
		fill := bMul(jf(2*r*r), bSub(jf(1), bQuo(junctionPi(), jf(4))))
		cut = keyway(t)
		all := append(append([][2]float64{}, keyTopCorners...), keyFloorCorner...)
		got, err = decad.Fillet(t.Context(), cut, shaftEdges(t, cut, all...), units.Millimeters(r))
		require.NoError(t, err)
		requireBlendedShaft(t, got, 5, 5, 4, r, shaftLess(bSub(bAdd(keySeat(), top), fill)))
	})

	t.Run("placed receiver", func(t *testing.T) {
		t.Parallel()
		// A placement moves the record's transform only, so the placed D-flat
		// fillets to item 1's volume.
		move, err := r3.Translation(r3.Vec{X: 3})
		require.NoError(t, err)
		placed, err := decad.Transform(t.Context(), dFlat(t, extrudedShaft), move)
		require.NoError(t, err)
		got, err := decad.Fillet(t.Context(), placed, flatEdges(t, placed), units.Millimeters(1))
		require.NoError(t, err)
		requireBlendedShaft(t, got, 3, 3, 2, 1, flatFillet)
	})

	endLoop := func(t *testing.T, b *decad.Body) decad.EdgeSelector {
		t.Helper()
		faces, err := decad.Faces(decad.Facing(r3.NewVec(1, 0, 0))).SelectFaces(b)
		require.NoError(t, err)
		require.Len(t, faces, 1)
		return decad.Edges(decad.OuterLoopOf(faces[0].Origins()[0]))
	}
	flat := decad.Faces(decad.Facing(r3.NewVec(0, 0, 1)))
	for _, tc := range []struct {
		name     string
		key      bool
		op       func(*testing.T, *decad.Body) error
		sentinel error
		text     string
	}{
		{"one flat edge", false, func(t *testing.T, b *decad.Body) error {
			_, err := decad.Fillet(t.Context(), b, shaftEdges(t, b, flatCorners[0]), units.Millimeters(1))
			return err
		}, decad.ErrUnsupported, `brep-modify PD4`},
		{"keyway floor edges alone", true, func(t *testing.T, b *decad.Body) error {
			_, err := decad.Fillet(t.Context(), b, shaftEdges(t, b, keyFloorCorner...), units.Millimeters(0.5))
			return err
		}, decad.ErrUnsupported, `brep-modify PD4`},
		{"cap-loop chamfer", false, func(t *testing.T, b *decad.Body) error {
			_, err := decad.Chamfer(t.Context(), b, endLoop(t, b), units.Millimeters(0.5))
			return err
		}, decad.ErrUnsupported, `brep-modify PD1`},
		{"asymmetric chamfer", false, func(t *testing.T, b *decad.Body) error {
			_, err := decad.Chamfer(t.Context(), b, flatEdges(t, b), units.Millimeters(1),
				decad.WithAsymmetricChamfer(flat, units.Millimeters(0.5)))
			return err
		}, decad.ErrUnsupported, `brep-modify SB1`},
		{"shell", false, func(t *testing.T, b *decad.Body) error {
			_, err := decad.Shell(t.Context(), b, decad.Faces(decad.Facing(r3.NewVec(0, 0, 1))).Exactly(1), units.Millimeters(1))
			return err
		}, decad.ErrUnsupported, `brep-modify SB1`},
		{"fillet as large as the shaft", false, func(t *testing.T, b *decad.Body) error {
			_, err := decad.Fillet(t.Context(), b, flatEdges(t, b), units.Millimeters(shaftR))
			return err
		}, decad.ErrDegenerate, `no fillet of radius`},
	} {
		t.Run("refuses "+tc.name, func(t *testing.T) {
			t.Parallel()
			var cut *decad.Body
			if tc.key {
				cut = keyway(t)
			} else {
				cut = dFlat(t, revolvedShaft)
			}
			err := tc.op(t, cut)
			require.ErrorIs(t, err, tc.sentinel)
			require.ErrorContains(t, err, tc.text)

			// The refusal leaves the receiver live: a full selection still
			// builds from it.
			full := flatEdges(t, cut)
			if tc.key {
				full = shaftEdges(t, cut, keyTopCorners...)
			}
			_, err = decad.Fillet(t.Context(), cut, full, units.Millimeters(0.5))
			require.NoError(t, err)
		})
	}
}
