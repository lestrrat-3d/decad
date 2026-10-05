package pair

import "github.com/lestrrat-3d/decad/internal/proof"

// PlanarDeepVertex reports whether a vertex of either audited solid lies
// strictly inside the other solid, farther than margin from its boundary.
// Such a vertex proves the interiors overlap after moving each solid by any
// map that displaces its points by at most mA and mB with mA + mB <= margin:
// the ball of radius margin around the vertex lies in the other solid, so its
// moved image still holds the moved vertex in that solid's interior, and every
// neighbourhood of a boundary vertex meets its own solid's interior. The
// rotating sweep (docs/multibody-dynamics-design.md §10.1) uses it to carry a
// float-pose overlap to the ideal path. margin must be nonnegative; poll is
// charged once per exact predicate group.
func PlanarDeepVertex(a, b *PlanarSolid, margin proof.Dyadic, poll func() error) (bool, error) {
	marginSquared := proof.DyMul(margin, margin)
	pa, pb := preparePlanar(a), preparePlanar(b)
	k := &planarKernel{a: pa, b: pb, poll: poll}
	for _, side := range [][2]*planarPrep{{pa, pb}, {pb, pa}} {
		from, to := side[0], side[1]
		for _, point := range from.s.Verts {
			deep, err := k.deepInside(point, to, marginSquared)
			if err != nil || deep {
				return deep, err
			}
		}
	}
	return false, nil
}

// deepInside reports whether p lies strictly inside solid with every
// boundary triangle farther than sqrt(marginSquared) away.
func (k *planarKernel) deepInside(p proof.DyV3, solid *planarPrep, marginSquared proof.Dyadic) (bool, error) {
	for t := range solid.s.Tris {
		if err := k.poll(); err != nil {
			return false, err
		}
		if proof.DyCmp(boxGapSquared(p, p, solid.triLo[t], solid.triHi[t]), marginSquared) > 0 {
			continue
		}
		d := pointTriangle(solid, t, p)
		if proof.DyCmp(d.num, proof.DyMul(marginSquared, d.den)) <= 0 {
			return false, nil
		}
	}
	result, err := k.cast(p, solid)
	return result == castInside, err
}

// pointTriangle is the exact squared distance from x to the closed triangle
// t: the plane distance when x projects inside it, else the nearest edge.
func pointTriangle(solid *planarPrep, t int, x proof.DyV3) frac {
	tri := solid.s.Tris[t]
	normal := solid.normal[t]
	inside := true
	for i := range 3 {
		if edgeSide(normal, solid.s.Verts[tri[i]], solid.s.Verts[tri[(i+1)%3]], dyPoint(x)) < 0 {
			inside = false
			break
		}
	}
	if inside {
		height := proof.DvDot(normal, proof.DvSub(x, solid.s.Verts[tri[0]]))
		return frac{num: proof.DyMul(height, height), den: proof.DvDot(normal, normal)}
	}
	best := pointSegment(x, solid.s.Verts[tri[0]], solid.s.Verts[tri[1]])
	for i := 1; i < 3; i++ {
		if d := pointSegment(x, solid.s.Verts[tri[i]], solid.s.Verts[tri[(i+1)%3]]); fracCmp(d, best) < 0 {
			best = d
		}
	}
	return best
}
