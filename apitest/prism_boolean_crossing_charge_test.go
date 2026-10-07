package apitest_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/general-boolean-design.md §3 A6's public-API suite: a
// pair whose re-expression or prior displacement meets a split boundary
// (prism-boolean §3.4) builds analytically, its crossings charged the
// displacement they can amplify, and a tangent crossing falls back to the
// mesh path. The charge's own mechanics are
// internal/prismcells/crossing_internal_test.go's.

// zRotation is RotationAround the world z axis through the origin.
func zRotation(t *testing.T, angle units.Value) r3.Transform {
	t.Helper()
	tr, err := r3.RotationAround(r3.Vec{}, r3.NewVec(0, 0, 1), angle)
	require.NoError(t, err)
	return tr
}

// exactPolygon is a polygon of exact rational vertices.
type exactPolygon [][2]*big.Rat

// placedRatPoly places plane points through tr's stored basis and
// translation, exactly over rationals.
func placedRatPoly(t *testing.T, tr r3.Transform, pts ...[2]float64) exactPolygon {
	t.Helper()
	b, o := tr.Basis(), tr.Translation()
	rat := func(f float64) *big.Rat {
		r := new(big.Rat)
		require.NotNil(t, r.SetFloat64(f))
		return r
	}
	out := make(exactPolygon, len(pts))
	for i, p := range pts {
		u, v := rat(p[0]), rat(p[1])
		x := new(big.Rat).Add(new(big.Rat).Mul(u, rat(b.EX.X)), new(big.Rat).Mul(v, rat(b.EY.X)))
		y := new(big.Rat).Add(new(big.Rat).Mul(u, rat(b.EX.Y)), new(big.Rat).Mul(v, rat(b.EY.Y)))
		out[i] = [2]*big.Rat{x.Add(x, rat(o.X)), y.Add(y, rat(o.Y))}
	}
	return out
}

// area is the polygon's signed area, exactly.
func (p exactPolygon) area() *big.Rat {
	total := new(big.Rat)
	for i := range p {
		a, b := p[i], p[(i+1)%len(p)]
		total.Add(total, new(big.Rat).Sub(new(big.Rat).Mul(a[0], b[1]), new(big.Rat).Mul(b[0], a[1])))
	}
	return total.Mul(total, big.NewRat(1, 2))
}

// clip is p clipped by the counter-clockwise convex polygon c
// (Sutherland–Hodgman), exactly: the test's own oracle for a convex overlap.
func (p exactPolygon) clip(c exactPolygon) exactPolygon {
	side := func(a, b, q [2]*big.Rat) *big.Rat {
		x := new(big.Rat).Mul(new(big.Rat).Sub(b[0], a[0]), new(big.Rat).Sub(q[1], a[1]))
		return x.Sub(x, new(big.Rat).Mul(new(big.Rat).Sub(b[1], a[1]), new(big.Rat).Sub(q[0], a[0])))
	}
	out := p
	for i := range c {
		a, b := c[i], c[(i+1)%len(c)]
		in := out
		out = nil
		for j := range in {
			q, r := in[j], in[(j+1)%len(in)]
			sq, sr := side(a, b, q), side(a, b, r)
			if sq.Sign() >= 0 {
				out = append(out, q)
			}
			if sq.Sign()*sr.Sign() < 0 {
				k := new(big.Rat).Quo(sq, new(big.Rat).Sub(sq, sr))
				out = append(out, [2]*big.Rat{
					new(big.Rat).Add(q[0], new(big.Rat).Mul(k, new(big.Rat).Sub(r[0], q[0]))),
					new(big.Rat).Add(q[1], new(big.Rat).Mul(k, new(big.Rat).Sub(r[1], q[1]))),
				})
			}
		}
	}
	return out
}

// exactResidualUp is |reported − truth| over rationals, rounded up.
func exactResidualUp(reported float64, truth *big.Rat) float64 {
	d := new(big.Rat).Sub(new(big.Rat).SetFloat64(reported), truth)
	d.Abs(d)
	f, exact := d.Float64()
	if !exact {
		f = math.Nextafter(f, math.Inf(1))
	}
	return f
}

// TestPrismIntersectRotatedBoxBuildsAnalyticOctagon is general-boolean §2's
// S9: a 20×20×10 box intersected with the same box rotated 45° about z. The
// rotated box's re-expression is nonidentity and every one of its walls
// crosses the other's. The intersection builds as an analytic octagonal
// prism, and its volume bound contains the exact residual against the
// intersection of the two recorded squares, the rotated one placed through
// its stored transform over math/big.Rat. Shown to fail with the §3.4
// split-boundary reroute restored in resolvePrismCrossingCells.
func TestPrismIntersectRotatedBoxBuildsAnalyticOctagon(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a := boxBody(t, doc, -10, -10, 10, 10, 10)
	src := boxBody(t, doc, -10, -10, 10, 10, 10)
	tr := zRotation(t, units.Degrees(45))
	b, err := src.Placed(t.Context(), tr)
	require.NoError(t, err)

	got, err := decad.Intersect(t.Context(), a, b)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(got))
	require.Len(t, got.Faces(), 10, "an octagonal prism: 8 walls and 2 caps")

	vol, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, vol.Exactness)
	require.Less(t, vol.Bound.Base(), 1e-9)

	square := placedRatPoly(t, r3.Identity(), [2]float64{-10, -10}, [2]float64{10, -10}, [2]float64{10, 10}, [2]float64{-10, 10})
	turned := placedRatPoly(t, tr, [2]float64{-10, -10}, [2]float64{10, -10}, [2]float64{10, 10}, [2]float64{-10, 10})
	truth := new(big.Rat).Mul(turned.clip(square).area(), big.NewRat(10, 1))
	residual := exactResidualUp(vol.Value.Base(), truth)
	require.LessOrEqualf(t, residual, vol.Bound.Base(),
		"the published bound %g must contain the true error %g", vol.Bound.Base(), residual)
	// 800(√2 − 1) mm² is the exactly rotated octagon's area.
	decadtest.Measures(t, "octagon volume", vol, units.CubicMillimeters(8000*(math.Sqrt2-1)))
}

// TestPrismUnionRotatedToothBuildsAnalytic is A6's rotated tooth: a Ø40
// hub unioned with a 7×3 tooth whose root sits inside the hub, placed by
// RotationAround through 60° (a step that keeps the normal exactly (0,0,1),
// so G3 admits it). Its walls cross the hub's circle nearly square, the
// crossing charge covers the re-expression rounding they can amplify, and
// the union builds analytically with the closed-form volume
// 10·(400π + 75 − (1.5·√397.75 + 400·asin 0.075)) within its bound. Shown
// to fail with the §3.4 split-boundary reroute restored in resolvePrismUnion.
func TestPrismUnionRotatedToothBuildsAnalytic(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	hub := hubBody(t, doc, 20, 10)
	src := boxBody(t, doc, 18, -1.5, 25, 1.5, 10)
	tooth, err := src.Placed(t.Context(), zRotation(t, units.Radians(math.Pi/3)))
	require.NoError(t, err)

	got, err := decad.Union(t.Context(), hub, tooth)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(got))
	vol, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, vol.Exactness)
	require.Less(t, vol.Bound.Base(), 1e-6)
	outside := 75 - (1.5*math.Sqrt(397.75) + 400*math.Asin(0.075))
	decadtest.Measures(t, "hub and rotated tooth", vol, units.CubicMillimeters(10*(400*math.Pi+outside)))
}

// TestPrismUnionToothOnHubCircleFallsBackAtTheTangentRoot is the P1 tooth
// of general-boolean §2: its root arc lies on the hub's own circle. Placed by
// RotationAround through 60°, the arc and the hub circle meet tangentially
// where the arc ends, so no positive sine bound exists there and A6 has no
// charge for that crossing. The pair falls back to the mesh path instead of
// refusing on the analytic one, and the error it reports is the mesh path's
// own proximity refusal. Shown to fail with CrossingCharge's declining
// crossing turned into an ErrUnsupported refusal.
func TestPrismUnionToothOnHubCircleFallsBackAtTheTangentRoot(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	hub := hubBody(t, doc, 20, 10)
	src := toothBody(t, doc, 20, 25, -0.1, 0.1, 10)
	tooth, err := src.Placed(t.Context(), zRotation(t, units.Radians(math.Pi/3)))
	require.NoError(t, err)

	_, err = decad.Union(t.Context(), hub, tooth)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "held facets come within the chord tolerance",
		"the mesh path's own refusal, reached by falling back")
}
