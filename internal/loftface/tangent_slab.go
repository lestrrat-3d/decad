package loftface

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// ConeDepth returns eps_K: the largest -psi over the held cone vertices, so
// the held cone lies in {psi >= -eps_K} (its convex hull is in that
// half-space). It reports false for a non-finite vertex or no vertex.
func ConeDepth(coneVerts []r3.Vec, plane TangentPlane) (*big.Rat, bool) {
	var depth *big.Rat
	for _, v := range coneVerts {
		if !proofbound.FiniteVec(v) {
			return nil, false
		}
		d := new(big.Rat).Neg(plane.Psi(Vec(v)))
		if depth == nil || d.Cmp(depth) > 0 {
			depth = d
		}
	}
	return depth, depth != nil
}

// ExtensionClearsCone is conditions 1 and 2 of Gate E5. With eps_K = ConeDepth
// it requires:
//
//  1. every extension-cap vertex of slab has psi < -eps_K, so the extension
//     cap lies strictly beyond every held cone vertex;
//  2. every held cone facet whose float box meets the slab box, clipped
//     exactly to that box, has psi > slab.Eps at every clipped vertex.
//
// Equality refuses. It returns an error wrapping ErrUnsupported on a refusal
// and polls budget once per facet.
func ExtensionClearsCone(slab Slab, coneVerts []r3.Vec, coneTris [][3]int, plane TangentPlane,
	budget *proofbound.WorkBudget) error {
	refuse := func(why string) error {
		return fmt.Errorf("%w: the held cone-tangent extension %s", decaderr.ErrUnsupported, why)
	}
	if slab.Eps == nil || len(slab.ExtCap) == 0 || plane.D == nil || plane.Side == 0 {
		return refuse("has no usable slab or plane")
	}
	if err := budget.Err(); err != nil {
		return err
	}
	depth, ok := ConeDepth(coneVerts, plane)
	if !ok {
		return refuse("meets a non-finite cone vertex")
	}
	bound := new(big.Rat).Neg(depth)
	for _, v := range slab.ExtCap {
		if plane.Psi(Vec(v)).Cmp(bound) >= 0 {
			return refuse("has an extension cap that is not strictly beyond the held cone")
		}
	}
	box := [2]RatVec{slab.Low, slab.High}
	for _, tri := range coneTris {
		if err := budget.Step(); err != nil {
			return err
		}
		if triMissesBox(coneVerts, tri, slab.LowF, slab.HighF) {
			continue
		}
		corners := [3]RatVec{Vec(coneVerts[tri[0]]), Vec(coneVerts[tri[1]]), Vec(coneVerts[tri[2]])}
		margin := ClippedPlaneMargin(corners, box, plane)
		if margin == nil {
			continue
		}
		if margin.Cmp(slab.Eps) <= 0 {
			return refuse("is not strictly clear of a held cone facet")
		}
	}
	return nil
}

// ResultBeyondSlab reports whether some vertex of a Boolean result has
// psi < -bound, which Gate E5 makes impossible. The caller passes
// max(eps_K, eps_S) and refuses on true.
func ResultBeyondSlab(verts []r3.Vec, plane TangentPlane, bound *big.Rat) bool {
	floor := new(big.Rat).Neg(bound)
	for _, v := range verts {
		if !proofbound.FiniteVec(v) || plane.Psi(Vec(v)).Cmp(floor) < 0 {
			return true
		}
	}
	return false
}

// ClippedPlaneMargin returns the exact minimum of Side*phi over the part of
// the triangle that lies in the closed axis box, or nil when that part is
// empty. box is {low corner, high corner}. The triangle is clipped against
// the six box planes exactly; phi is affine, so its minimum over the convex
// clipped polygon is at a vertex.
func ClippedPlaneMargin(tri [3]RatVec, box [2]RatVec, plane TangentPlane) *big.Rat {
	polygon := [][3]*big.Rat{tri[0], tri[1], tri[2]}
	for axis := range 3 {
		polygon = clipHalfSpace(polygon, axis, box[0][axis], true)
		polygon = clipHalfSpace(polygon, axis, box[1][axis], false)
		if len(polygon) == 0 {
			return nil
		}
	}
	var margin *big.Rat
	for _, vertex := range polygon {
		value := plane.Phi(vertex)
		if plane.Side < 0 {
			value = new(big.Rat).Neg(value)
		}
		if margin == nil || value.Cmp(margin) < 0 {
			margin = value
		}
	}
	return margin
}

// clipHalfSpace keeps the part of the convex polygon with coordinate axis at
// least bound (lower) or at most bound (upper), closed. It is one step of
// Sutherland-Hodgman on exact rationals.
func clipHalfSpace(polygon [][3]*big.Rat, axis int, bound *big.Rat, lower bool) [][3]*big.Rat {
	// excess is positive inside, zero on the boundary plane.
	excess := func(p [3]*big.Rat) *big.Rat {
		if lower {
			return ratSub(p[axis], bound)
		}
		return ratSub(bound, p[axis])
	}
	var out [][3]*big.Rat
	for i, current := range polygon {
		next := polygon[(i+1)%len(polygon)]
		ec, en := excess(current), excess(next)
		if ec.Sign() >= 0 {
			out = append(out, current)
		}
		if (ec.Sign() < 0 && en.Sign() > 0) || (ec.Sign() > 0 && en.Sign() < 0) {
			t := ratDiv(ec, ratSub(ec, en))
			out = append(out, VAdd(current, VScale(VSub(next, current), t)))
		}
	}
	return out
}

// triMissesBox reports whether the float bounds of the triangle lie wholly
// outside the float box. It is exact: float comparisons are exact.
func triMissesBox(verts []r3.Vec, tri [3]int, low, high [3]float64) bool {
	for axis := range 3 {
		coord := func(v r3.Vec) float64 { return [3]float64{v.X, v.Y, v.Z}[axis] }
		a, b, c := coord(verts[tri[0]]), coord(verts[tri[1]]), coord(verts[tri[2]])
		if max(a, b, c) < low[axis] || min(a, b, c) > high[axis] {
			return true
		}
	}
	return false
}
