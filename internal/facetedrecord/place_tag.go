package facetedrecord

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
)

// placedTagMaxDefect is the largest orthonormality defect of a placement for
// which the normal charge below is stated. Any real frame is far below it
// (r3 admits 1e-9); a placement past it keeps the trivial bound of 2.
const placedTagMaxDefect = 1e-3

// PlacedTagDeparture returns the normal departure a retained tag gains when
// the body is placed by delta, and whether the tag keeps a stated bound at all
// (docs/faceted-source-retention-design.md §3.4). tag is the tag before the
// placement, moved its float image (surfaceplace.TransformSurface), and verts
// and reach the held mesh before the placement with the distance of any true
// boundary point from a held vertex. A Cone declines: its bound needs a lower
// bound on the distance from the axis that the placement does not have.
//
// The true placed solid is A(S) for the linear map B and translation t of
// delta, read as exact rationals, and B departs from orthonormal by e.
//
//   - Plane. (B·U)×(B·V) = det(B)·B⁻ᵀ(U×V), so the exact image of the tag is
//     the true normal's line, and the departure is only the float rounding of
//     the moved frame vectors. With a the largest distance of a moved vector
//     from its exact image, the crossed normal moves by under 3a, and
//     normalising by a length of at least 0.99 gives under 7a. The charge is
//     8a. The origin's rounding gap and 4e are charged as well, though the
//     normal does not need them, so that a zero bound still means the tag is
//     the exact image plane under an exactly orthonormal map (MirrorFace reads
//     that).
//   - Cylinder, Sphere, Torus. The true normal is B⁻ᵀn and the tag's is B·n,
//     and (B⁻ᵀ−B)n = B⁻ᵀ(I−BᵀB)n has length at most e/σ with σ² ≥ 1−e the
//     smallest singular value squared, so the unit normals differ by at most
//     2e/σ² ≤ 4e. The surface itself sits away from the moved tag by the
//     origin's rounding gap P, plus (a + 2e)·D for the moved axis and the
//     stretch, D the distance from the tag's origin to any true point. That
//     displacement enters the held term of §3.3 at twice its size.
//
// For a translation or a signed permutation e and a are zero, so the charge is
// the origin's rounding alone.
func PlacedTagDeparture(tag, moved surfacegeom.Surface, delta r3.Transform, verts []r3.Vec,
	reach float64) (float64, bool) {
	linear, err := massmoment.PlacementRotation(delta)
	if err != nil {
		return 0, false
	}
	charge, err := massmoment.MapChargeOf(linear)
	if err != nil || !(charge.Stretch <= placedTagMaxDefect) {
		return 0, false
	}
	translation, ok := massmoment.ExactVec(delta.Translation())
	if !ok {
		return 0, false
	}
	e := charge.Stretch
	switch t := tag.(type) {
	case surfacegeom.Plane:
		m, isPlane := moved.(surfacegeom.Plane)
		if !isPlane {
			return 0, false
		}
		a := max(imageGap(linear, t.Frame.U(), m.Frame.U()), imageGap(linear, t.Frame.V(), m.Frame.V()))
		p := pointGap(linear, translation, t.Frame.Origin(), m.Frame.Origin())
		return proofbound.AbsSumUpper(proofbound.ProductUpper(8, a), proofbound.ProductUpper(4, e), p), true
	case surfacegeom.Cylinder:
		m, isCyl := moved.(surfacegeom.Cylinder)
		if !isCyl {
			return 0, false
		}
		return curvedDeparture(tag, e, imageGap(linear, t.Axis, m.Axis),
			pointGap(linear, translation, t.Origin, m.Origin), t.Origin, verts, reach)
	case surfacegeom.Sphere:
		m, isSphere := moved.(surfacegeom.Sphere)
		if !isSphere {
			return 0, false
		}
		return curvedDeparture(tag, e, 0, pointGap(linear, translation, t.Center, m.Center), t.Center, verts, reach)
	case surfacegeom.Torus:
		m, isTorus := moved.(surfacegeom.Torus)
		if !isTorus {
			return 0, false
		}
		return curvedDeparture(tag, e, imageGap(linear, t.Axis, m.Axis),
			pointGap(linear, translation, t.Center, m.Center), t.Center, verts, reach)
	}
	return 0, false
}

// curvedDeparture is the Cylinder, Sphere and Torus charge of
// PlacedTagDeparture: 4e for the normal map plus the held term of the
// displacement 2·(P + (a + 2e)·D).
func curvedDeparture(tag surfacegeom.Surface, e, a, p float64, origin r3.Vec, verts []r3.Vec,
	reach float64) (float64, bool) {
	far := reach
	for _, v := range verts {
		far = proofbound.AbsSumUpper(far, proofbound.DvLenUpper(proofbound.HeldDelta(v, origin)))
	}
	shift := proofbound.AbsSumUpper(p, proofbound.ProductUpper(proofbound.AbsSumUpper(a, 2*e), far))
	term, ok := HeldNormalTerm(tag, proofbound.ProductUpper(2, shift), 0)
	if !ok {
		return 0, false
	}
	return proofbound.AbsSumUpper(proofbound.ProductUpper(4, e), term), true
}

// imageGap bounds the distance of the float image moved from the exact image
// of the direction w under the linear map: B·w with B and w read as rationals.
func imageGap(linear [3][3]*big.Rat, w, moved r3.Vec) float64 {
	return pointGap(linear, [3]*big.Rat{new(big.Rat), new(big.Rat), new(big.Rat)}, w, moved)
}

// pointGap bounds the distance of the float image moved from the exact image
// B·p + t of the point p, with B, t and p read as rationals.
func pointGap(linear [3][3]*big.Rat, translation [3]*big.Rat, p, moved r3.Vec) float64 {
	coords := [3]float64{p.X, p.Y, p.Z}
	got := [3]float64{moved.X, moved.Y, moved.Z}
	gap := 0.0
	for i := range got {
		exact := new(big.Rat).Set(translation[i])
		for j := range coords {
			exact.Add(exact, pointPlaneMul(linear[i][j], ratOfPointPlane(coords[j])))
		}
		gap = max(gap, proofbound.RatAbsDiff(exact, got[i]))
	}
	return proofbound.Radius3D(gap)
}
