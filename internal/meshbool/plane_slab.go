package meshbool

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// SetSlabBounds sets SlabBound for the facets whose true piece lies in an
// exact plane (agent-docs/faceted-source-retention-design.md §11). planeOf returns
// the frame of facet i's plane and true for such a facet. Every other facet
// keeps FacetBound. SetVertexBounds must have run.
func (bm *BoolMesh) SetSlabBounds(planeOf func(facet int) (r3.Frame, bool)) {
	for i := range bm.Tris {
		frame, ok := planeOf(i)
		if !ok {
			continue
		}
		if slab, ok := bm.planeSlab(i, frame); ok && slab < bm.SlabBound[i] {
			bm.SlabBound[i] = slab
		}
	}
}

// planeSlab bounds how far the true piece of facet i lies from the facet's own
// plane, given that the piece lies in the plane T of frame and within
// FacetBound[i] of the facet. Let h be the largest distance of a corner from
// T, so every point y of the facet lies within h of T (the distance to a plane
// is convex), and let φ be the angle between the facet's plane P and T. A
// point x of T within β = FacetBound[i] of y has |x − y_T| ≤ β for y_T the
// projection of y onto T, because the projection is 1-Lipschitz. The distance
// to P is affine on T with slope sin φ, and dist(y_T, P) ≤ |y_T − y| ≤ h, so
//
//	dist(x, P) ≤ h + β·sin φ.
//
// The bound is the slab half-width RimBound reads. Both h and sin φ are
// computed exactly from the held floats and rounded up.
func (bm *BoolMesh) planeSlab(i int, frame r3.Frame) (float64, bool) {
	o := ratPoint(frame.Origin())
	n := ratCross(ratPoint(frame.U()), ratPoint(frame.V()))
	nn := ratDot(n, n)
	if nn.Sign() == 0 {
		return 0, false
	}
	tri := bm.Tris[i]
	corners := [3][3]*big.Rat{ratPoint(bm.Verts[tri[0]]), ratPoint(bm.Verts[tri[1]]), ratPoint(bm.Verts[tri[2]])}
	farthest := new(big.Rat)
	for _, c := range corners {
		s := ratDot(n, ratSub(c, o))
		s.Mul(s, s)
		if s.Cmp(farthest) > 0 {
			farthest = s
		}
	}
	h := proofbound.RatSqrtUp(new(big.Rat).Quo(farthest, nn))
	nt := ratCross(ratSub(corners[1], corners[0]), ratSub(corners[2], corners[0]))
	ntnt := ratDot(nt, nt)
	if ntnt.Sign() == 0 {
		return 0, false
	}
	cross := ratCross(nt, n)
	sin2 := new(big.Rat).Quo(ratDot(cross, cross), new(big.Rat).Mul(ntnt, nn))
	slope := proofbound.RatSqrtUp(sin2)
	slab := proofbound.AbsSumUpper(h, proofbound.ProductUpper(bm.FacetBound[i], slope))
	if proofbound.IsNonFinite(slab) {
		return 0, false
	}
	return slab, true
}

func ratPoint(v r3.Vec) [3]*big.Rat {
	return [3]*big.Rat{proof.FloatRat(v.X), proof.FloatRat(v.Y), proof.FloatRat(v.Z)}
}

func ratSub(a, b [3]*big.Rat) [3]*big.Rat {
	return [3]*big.Rat{new(big.Rat).Sub(a[0], b[0]), new(big.Rat).Sub(a[1], b[1]), new(big.Rat).Sub(a[2], b[2])}
}

func ratDot(a, b [3]*big.Rat) *big.Rat {
	out := new(big.Rat)
	for k := range a {
		out.Add(out, new(big.Rat).Mul(a[k], b[k]))
	}
	return out
}

func ratCross(a, b [3]*big.Rat) [3]*big.Rat {
	c := func(x, y, z, w int) *big.Rat {
		return new(big.Rat).Sub(new(big.Rat).Mul(a[x], b[y]), new(big.Rat).Mul(a[z], b[w]))
	}
	return [3]*big.Rat{c(1, 2, 2, 1), c(2, 0, 0, 2), c(0, 1, 1, 0)}
}
