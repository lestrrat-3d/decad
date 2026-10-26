package planar

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proof"
)

// This file is the §9.2 convexity certificate (agent-docs/multibody-dynamics-design.md)
// and its float pre-test, whose error bound, range guard and poll count
// agent-docs/planar-convex-filter-design.md derives. The pre-test only skips an
// exact sign it proves nonpositive, so every answer is the exact loop's.

// convexGuard is the range guard: the pre-test runs only on a solid whose
// every vertex coordinate rounds to a finite float of magnitude at most 2²⁵⁰,
// which keeps every float the pre-test forms finite.
const convexGuard = 0x1p250

// convexDotEps is 5ε with ε = 2⁻⁵³, the relative part of the pre-test's error
// bound on the dot product's rounding.
const convexDotEps = 5 * 0x1p-53

// convexDotFloor is 2⁻¹⁰⁷³, the bound's absolute part: at most half the least
// subnormal per product that underflows, with room for the sums' rounding.
const convexDotFloor = 0x1p-1073

// PlanarConvex is the §9.2 certificate: every vertex lies on or behind every
// triangle's plane. Combined with CheckPlanarSolid's audit this proves the
// solid is the convex polytope its facet half-spaces cut out; a hollow or
// multi-lump solid fails it, since some vertex lies in front of a facet. The
// result is invariant under any affine map with a positive determinant.
//
// poll is charged once per triangle and vertex pair, whether the float
// pre-test (convexFilter) or the exact sign decides the pair.
func PlanarConvex(s *PlanarSolid, poll func() error) (bool, error) {
	filter, filtered := newConvexFilter(s.Verts)
	for _, tri := range s.Tris {
		a := s.Verts[tri[0]]
		normal := proof.DvCross(proof.DvSub(s.Verts[tri[1]], a), proof.DvSub(s.Verts[tri[2]], a))
		var plane convexPlane
		if filtered {
			plane = filter.plane(normal, tri[0])
		}
		for v := range s.Verts {
			if err := poll(); err != nil {
				return false, err
			}
			if filtered && plane.behind(&filter.coords[v]) {
				continue
			}
			point := s.Verts[v]
			if proof.DvDot(normal, proof.DvSub(point, a)).Sign() > 0 {
				return false, nil
			}
		}
	}
	return true, nil
}

// convexFilter holds one solid's rounded vertex coordinates and the per-axis
// terms of the pre-test's error bound. extent is at or above the spread of
// the rounded coordinates on each axis, and round at or above every
// coordinate's rounding error on it.
type convexFilter struct {
	coords [][3]float64
	extent [3]float64
	round  [3]float64
}

// newConvexFilter rounds every vertex coordinate to the nearest float, and
// returns false when one falls outside the range guard (convexGuard); the
// certificate then takes every sign exactly.
func newConvexFilter(verts []proof.DyV3) (*convexFilter, bool) {
	if len(verts) == 0 {
		return nil, false
	}
	f := &convexFilter{coords: make([][3]float64, len(verts))}
	var lo, hi [3]float64
	for v, point := range verts {
		for axis := range 3 {
			c := point[axis]
			x, exact := c.Float64()
			if !(math.Abs(x) <= convexGuard) {
				return nil, false
			}
			if !exact {
				f.round[axis] = math.Max(f.round[axis], proof.DyadicFloatError(c, x))
			}
			f.coords[v][axis] = x
			if v == 0 || x < lo[axis] {
				lo[axis] = x
			}
			if v == 0 || x > hi[axis] {
				hi[axis] = x
			}
		}
	}
	for axis := range 3 {
		f.extent[axis] = subUp(hi[axis], lo[axis])
	}
	return f, true
}

// convexPlane is one triangle's float pre-test: the rounded normal, the
// rounded anchor vertex and a bound at or above the error of the float dot
// product behind against the exact one, for every vertex of the solid.
type convexPlane struct {
	normal, anchor [3]float64
	bound          float64
}

// plane returns the pre-test of the triangle with exact normal and anchor
// vertex index anchor. Every term of the bound is nonnegative and rounded
// upward; a NaN or infinite bound makes behind defer every vertex.
func (f *convexFilter) plane(normal proof.DyV3, anchor int) convexPlane {
	out := convexPlane{anchor: f.coords[anchor]}
	bound := 0.0
	for axis := range 3 {
		c := normal[axis]
		n, exact := c.Float64()
		out.normal[axis] = n
		normalErr := 0.0
		if !exact {
			normalErr = proof.DyadicFloatError(c, n)
		}
		twoRound := mulUp(2, f.round[axis])
		// |n̂|·(5ε·D + 2R) + ν·(D + 2R), the design doc's per-axis term.
		term := addUp(mulUp(math.Abs(n), addUp(mulUp(convexDotEps, f.extent[axis]), twoRound)),
			mulUp(normalErr, addUp(f.extent[axis], twoRound)))
		bound = addUp(bound, term)
	}
	out.bound = addUp(bound, convexDotFloor)
	return out
}

// behind reports whether the float dot product of the plane's normal with
// v − anchor proves the exact one at or below zero. Each product is converted
// to float64 before the sum, which stops the compiler from fusing it into an
// add, so every platform rounds the same operations the same way.
func (p *convexPlane) behind(v *[3]float64) bool {
	d0, d1, d2 := v[0]-p.anchor[0], v[1]-p.anchor[1], v[2]-p.anchor[2]
	s := float64(p.normal[0]*d0) + float64(p.normal[1]*d1)
	s += float64(p.normal[2] * d2)
	return s <= -p.bound
}
