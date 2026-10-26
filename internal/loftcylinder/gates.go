package loftcylinder

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// SourceIdentitiesMatch reports whether teeth, read in order, are Lofts
// sharing the first tooth's non-nil source identity. teeth must not be empty.
// A continuation can only reuse the prior local certificate for the same
// authenticated Loft source; checking this before rebuilding the next tooth
// leaves an unrelated Loft to the ordinary Boolean's error and admission
// policy.
func SourceIdentitiesMatch(teeth []*topo.Body) bool {
	first, ok := teeth[0].Payload.(loftrecord.Payload)
	if !ok || first.SourceIdentity == nil {
		return false
	}
	for _, tooth := range teeth[1:] {
		next, ok := tooth.Payload.(loftrecord.Payload)
		if !ok || next.SourceIdentity != first.SourceIdentity {
			return false
		}
	}
	return true
}

// TriangleMinRadius is a rounded-down lower bound on the triangle's minimum
// projected radius. It is computed over its whole convex
// patch using exact dyadic inputs, not from its sampled corner radii.
func TriangleMinRadius(verts []r3.Vec, tri [3]int) float64 {
	return proofbound.RatSqrtDown(TriangleMinRadius2(verts, tri))
}

// TriangleMinRadius2 is the exact squared distance from the axis
// to the projected triangle: zero when the origin lies inside, otherwise the
// minimum over the three edges of the squared distance to the closed edge.
// Every coordinate is a float, so its denominator is a power of two and the
// largest one is common to all six; the algebra then runs in integers and one
// rational is formed at the end.
func TriangleMinRadius2(verts []r3.Vec, tri [3]int) *big.Rat {
	var coords [3][2]*big.Rat
	den := big.NewInt(1)
	for i, index := range tri {
		coords[i] = [2]*big.Rat{proofarith.FloatRat(verts[index].X), proofarith.FloatRat(verts[index].Y)}
		for _, c := range coords[i] {
			if c.Denom().Cmp(den) > 0 {
				den = new(big.Int).Set(c.Denom())
			}
		}
	}
	var p [3][2]*big.Int
	for i := range coords {
		for k, c := range coords[i] {
			p[i][k] = new(big.Int).Mul(c.Num(), new(big.Int).Quo(den, c.Denom()))
		}
	}
	// If the origin lies inside the projected triangle, its distance is zero.
	var signs [3]int
	for i := range 3 {
		a, b := p[i], p[(i+1)%3]
		cross := new(big.Int).Sub(new(big.Int).Mul(a[0], b[1]), new(big.Int).Mul(a[1], b[0]))
		signs[i] = cross.Sign()
	}
	if (signs[0] > 0 && signs[1] > 0 && signs[2] > 0) ||
		(signs[0] < 0 && signs[1] < 0 && signs[2] < 0) {
		return new(big.Rat)
	}
	den2 := new(big.Int).Mul(den, den)
	norm2 := func(x, y *big.Int) *big.Int {
		return new(big.Int).Add(new(big.Int).Mul(x, x), new(big.Int).Mul(y, y))
	}
	var bestNum, bestDen *big.Int
	for i := range 3 {
		a, b := p[i], p[(i+1)%3]
		dx, dy := new(big.Int).Sub(b[0], a[0]), new(big.Int).Sub(b[1], a[1])
		dd := norm2(dx, dy)
		num := new(big.Int).Add(new(big.Int).Mul(a[0], dx), new(big.Int).Mul(a[1], dy))
		num.Neg(num)
		var qNum, qDen *big.Int
		switch {
		case dd.Sign() == 0 || num.Sign() <= 0:
			qNum, qDen = norm2(a[0], a[1]), den2
		case num.Cmp(dd) >= 0:
			qNum, qDen = norm2(b[0], b[1]), den2
		default:
			// |a|^2 - (a.d)^2/|d|^2 at the foot of the perpendicular.
			qNum = new(big.Int).Sub(new(big.Int).Mul(norm2(a[0], a[1]), dd), new(big.Int).Mul(num, num))
			qDen = new(big.Int).Mul(dd, den2)
		}
		if i == 0 || new(big.Int).Mul(qNum, bestDen).Cmp(new(big.Int).Mul(bestNum, qDen)) < 0 {
			bestNum, bestDen = qNum, qDen
		}
	}
	return new(big.Rat).SetFrac(bestNum, bestDen)
}

// RadialGate reports whether whole fitted-flank facets stay outside the circle after the source mesh's
// certified displacement. Root and connector cells may enter only one narrow
// annulus, whose maximum inset is returned for the join proof. Here verts and
// tris are the held Loft mesh and segment its per-cell segment kinds.
func RadialGate(verts []r3.Vec, tris [][3]int, segment [][]int, root int, radius, bound, rootGap float64) (float64, bool) {
	n := len(verts) / 2
	if root < 2 || bound < 0 || proofbound.IsNonFinite(bound) {
		return 0, false
	}
	inset := 0.0
	for capIndex := range 2 {
		offset := capIndex * n
		for j := range n {
			p := verts[offset+j]
			if !proofbound.FiniteVec(p) {
				return 0, false
			}
			if j == 0 || j >= root {
				x, y := proofarith.FloatRat(p.X), proofarith.FloatRat(p.Y)
				q := proofarith.AddRat(new(big.Rat), proofarith.MulRat(new(big.Rat), x, x), proofarith.MulRat(new(big.Rat), y, y))
				radLo, radHi := proofbound.RatSqrtDown(q), proofbound.RatSqrtUp(q)
				allow := proofbound.AbsSumUpper(bound, rootGap)
				lower := proofarith.SubRat(new(big.Rat), proofarith.FloatRat(radius), proofarith.FloatRat(allow))
				upper := proofarith.AddRat(new(big.Rat), proofarith.FloatRat(radius), proofarith.FloatRat(allow))
				if proofarith.FloatRat(radLo).Cmp(lower) < 0 || proofarith.FloatRat(radHi).Cmp(upper) > 0 {
					return 0, false
				}
			}
		}
	}
	for j := range n {
		if segment[0][j] == 5 && j < root {
			return 0, false
		}
		for side := range 2 {
			lo := TriangleMinRadius(verts, tris[2*j+side])
			if j > 0 && j < root-1 {
				threshold := proofarith.AddRat(new(big.Rat), proofarith.FloatRat(radius), proofarith.FloatRat(bound))
				if proofarith.FloatRat(lo).Cmp(threshold) <= 0 {
					return 0, false
				}
			} else {
				radialGap := proofarith.SubRat(new(big.Rat), proofarith.FloatRat(radius), proofarith.FloatRat(lo))
				if radialGap.Sign() < 0 {
					radialGap.SetInt64(0)
				}
				inset = max(inset, proofbound.AbsSumUpper(
					proofbound.RatFloatUp(radialGap), bound, rootGap))
			}
		}
	}
	return inset, !proofbound.IsNonFinite(inset) && inset < 0.1
}

// WedgeGate bounds the lateral departure of a held Loft mesh from the moving
// sector. The root endpoint rulings delimit a moving angular sector. For one held
// triangle, its signed distance from either ruling is a quadratic in its
// barycentric coordinates. Every Bernstein coefficient bounds that quadratic
// over the triangle, so a negative coefficient gives a certified maximum
// lateral excursion into the cylinder's complementary gap.
func WedgeGate(verts []r3.Vec, tris [][3]int, root int, z0, z1 float64) (float64, bool) {
	n := len(verts) / 2
	start := [2]r3.Vec{verts[0], verts[n]}
	end := [2]r3.Vec{verts[root], verts[n+root]}
	minStart := TriangleMinRadius(start[:], [3]int{0, 1, 1})
	minEnd := TriangleMinRadius(end[:], [3]int{0, 1, 1})
	if minStart <= 0 || minEnd <= 0 {
		return 0, false
	}
	cross := func(a, b r3.Vec) *big.Rat {
		return proofarith.SubRat(new(big.Rat),
			proofarith.MulRat(new(big.Rat), proofarith.FloatRat(a.X), proofarith.FloatRat(b.Y)),
			proofarith.MulRat(new(big.Rat), proofarith.FloatRat(a.Y), proofarith.FloatRat(b.X)))
	}
	departure := 0.0
	for _, tri := range tris {
		var p, a, b [3]r3.Vec
		for i, index := range tri {
			p[i] = verts[index]
			capIndex := 0
			switch p[i].Z {
			case z0:
			case z1:
				capIndex = 1
			default:
				return 0, false
			}
			a[i], b[i] = start[capIndex], end[capIndex]
		}
		for i := range 3 {
			for j := i; j < 3; j++ {
				left, right := cross(a[i], p[j]), cross(p[i], b[j])
				if i != j {
					proofarith.AddRat(left, left, cross(a[j], p[i]))
					proofarith.QuoRat(left, left, big.NewRat(2, 1))
					proofarith.AddRat(right, right, cross(p[j], b[i]))
					proofarith.QuoRat(right, right, big.NewRat(2, 1))
				}
				if left.Sign() < 0 {
					departure = max(departure, proofbound.RatFloatUp(
						proofarith.QuoRat(new(big.Rat), new(big.Rat).Neg(left), proofarith.FloatRat(minStart))))
				}
				if right.Sign() < 0 {
					departure = max(departure, proofbound.RatFloatUp(
						proofarith.QuoRat(new(big.Rat), new(big.Rat).Neg(right), proofarith.FloatRat(minEnd))))
				}
			}
		}
	}
	return departure, !proofbound.IsNonFinite(departure) && departure < 0.02
}

// RingOrder reports whether the angle between neighboring ring rulings stays positive at every z:
// their cross product is a quadratic in height whose three Bernstein
// coefficients are positive. Continuity makes the ring cover each
// intermediate angular ray between its ordered endpoints.
func RingOrder(b0, b1, t0, t1 r3.Vec) bool {
	cross := func(a, b r3.Vec) *big.Rat {
		return proofarith.SubRat(new(big.Rat),
			proofarith.MulRat(new(big.Rat), proofarith.FloatRat(a.X), proofarith.FloatRat(b.Y)),
			proofarith.MulRat(new(big.Rat), proofarith.FloatRat(a.Y), proofarith.FloatRat(b.X)))
	}
	if cross(b0, b1).Sign() <= 0 || cross(t0, t1).Sign() <= 0 {
		return false
	}
	middle := proofarith.AddRat(new(big.Rat), cross(b0, t1), cross(t0, b1))
	return middle.Sign() > 0
}
