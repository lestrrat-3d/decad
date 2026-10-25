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
	// These finite held floats and their sums and products are all dyadic.
	// Keep them exact without general-fraction normalization until division.
	o := proof.DyVec(frame.Origin())
	n := proof.DvCross(proof.DyVec(frame.U()), proof.DyVec(frame.V()))
	nn := proof.DvDot(n, n)
	if nn.IsZero() {
		return 0, false
	}
	tri := bm.Tris[i]
	corners := [3]proof.DyV3{proof.DyVec(bm.Verts[tri[0]]), proof.DyVec(bm.Verts[tri[1]]), proof.DyVec(bm.Verts[tri[2]])}
	farthest := proof.DyZero()
	for _, c := range corners {
		s := proof.DvDot(n, proof.DvSub(c, o))
		s = proof.DyMul(s, s)
		if proof.DyCmp(s, farthest) > 0 {
			farthest = s
		}
	}
	h := proofbound.RatSqrtUp(proof.QuoRat(new(big.Rat), farthest.Rat(), nn.Rat()))
	nt := proof.DvCross(proof.DvSub(corners[1], corners[0]), proof.DvSub(corners[2], corners[0]))
	ntnt := proof.DvDot(nt, nt)
	if ntnt.IsZero() {
		return 0, false
	}
	cross := proof.DvCross(nt, n)
	sin2 := proof.QuoRat(new(big.Rat), proof.DvDot(cross, cross).Rat(), proof.DyMul(ntnt, nn).Rat())
	slope := proofbound.RatSqrtUp(sin2)
	slab := proofbound.AbsSumUpper(h, proofbound.ProductUpper(bm.FacetBound[i], slope))
	if proofbound.IsNonFinite(slab) {
		return 0, false
	}
	return slab, true
}
