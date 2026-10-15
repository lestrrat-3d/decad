package momentinput

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// LoopEnclosedArea is the absolute area a single loop encloses — the magnitude
// of its Green's-theorem signed area, so a hole (clockwise) and an outer loop
// (counter-clockwise) both report a positive area. A rim band's area is the
// difference of the two loops it spans, and both are strictly nested (the audit
// proved the cavity simple and inside the outer region), so the absolute
// difference is the band's analytic area, with both source bounds carried.
func LoopEnclosedArea(ctx context.Context, l LoopRecord) (proofbound.BoundedScalar, error) {
	ig, err := loopRegionIntegrals(ctx, l, freeform.MomentAreaOrder)
	if err != nil {
		return proofbound.BoundedScalar{}, err
	}
	return proofbound.MeasuredScalar(math.Abs(ig.Area), ig.AreaBound), nil
}

// loopRegionIntegrals runs the Integrals accumulator over one
// loop's own segments about the plane origin, through the order its caller reads.
func loopRegionIntegrals(ctx context.Context, l LoopRecord, order freeform.MomentIntegralOrder) (Integrals, error) {
	var ig Integrals
	for _, seg := range l.Segments {
		if err := ctx.Err(); err != nil {
			return Integrals{}, err
		}
		// Integrated about the plane origin itself; the band's area is a
		// difference of two loop areas, so no walk anchor is involved.
		if err := ig.AddFor(seg, Plan{}, Point2{}, order); err != nil {
			return Integrals{}, err
		}
	}
	return ig, nil
}

// LoopEnclosedMoments is a signed first-moment sibling of
// LoopEnclosedArea (docs/modify-reach-design.md §8.4's slab term): the
// SAME Integrals accumulator over the loop's own segments about the
// plane origin, returning the loop's own area together with its first
// moments (∫u dA, ∫v dA) — each canonicalized the SAME way
// LoopEnclosedArea's |area| already is, via orient = sign(the loop's
// own raw signed area). Reversing a walk's direction negates every one of
// Green's theorem's contour integrals identically — area and first moments
// alike, since they are all contour integrals of the same shape — so the
// SAME orient factor that turns a clockwise hole's negative raw area into
// LoopEnclosedArea's positive reading turns its first moments
// consistently too, and orient*ig.Area equals math.Abs(ig.Area) exactly, so
// this function's own area field always matches LoopEnclosedArea's.
// The caller (capblend.Build) applies its own per-loop sign
// (outer/hole, by loop index) on top of this canonicalized triple, exactly as
// it already does to LoopEnclosedArea's |area| for the slab volume.
func LoopEnclosedMoments(ctx context.Context, l LoopRecord) (area, mu, mv proofbound.BoundedScalar, err error) {
	ig, err := loopRegionIntegrals(ctx, l, freeform.MomentFirstOrder)
	if err != nil {
		return proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, proofbound.BoundedScalar{}, err
	}
	orient := 1.0
	if ig.Area < 0 {
		orient = -1
	}
	return proofbound.MeasuredScalar(orient*ig.Area, ig.AreaBound),
		proofbound.MeasuredScalar(orient*ig.Mu, ig.MuBound),
		proofbound.MeasuredScalar(orient*ig.Mv, ig.MvBound), nil
}
