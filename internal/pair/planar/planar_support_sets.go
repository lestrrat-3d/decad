package planar

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/pair"
	"github.com/lestrrat-3d/decad/internal/proof"
)

// PlanarSupportSets returns the lifted sets of the support planes
// (docs/multibody-dynamics-design.md §10.5), each plane's in turn, in the
// order of planes. One plane's lifted set is every guest vertex whose exact
// height h = n·(p − q) over the host face's plane is at most band (in
// millimetres, h² <= band²·n·n, compared exactly) and whose exact foot on the
// plane lies strictly inside the face, holes excluded. Without overlap every
// guest vertex must lie on or in front of it, and the vertices at zero
// height, the contact set, are left out; with overlap the guest must poke
// through it, and its deepest vertices, which the penetration patch
// publishes, are left out instead. Each point pairs the foot on the host with
// the vertex on the guest, carries the host face's outward normal oriented A
// to B, and the vertex's exact signed height enclosed once as its
// Separation. A plane that is not a support plane adds nothing. A nil result
// means no plane added a point. result is ClassifyPlanar's result for a and
// b, whose derived data each solid's adjacency is built over once for every
// plane; the zero PlanarResult builds that data afresh. poll is charged once
// per guest vertex per plane, twice for a plane that reaches its feet.
func PlanarSupportSets(a, b *PlanarSolid, result PlanarResult, planes []SupportPlane, band proof.Dyadic,
	overlap bool, poll func() error) ([]PatchPoint, error) {
	if len(a.Faces) != len(a.Tris) || len(b.Faces) != len(b.Tris) || band.Sign() <= 0 || len(planes) == 0 {
		return nil, nil
	}
	sa := newPreparedPatchSide(result.preparedFor(a), false, true)
	sb := newPreparedPatchSide(result.preparedFor(b), false, false)
	var out []PatchPoint
	for _, plane := range planes {
		host, guest := sb, sa
		if plane.HostIsA {
			host, guest = sa, sb
		}
		points, err := supportSet(host, guest, plane, band, overlap, poll)
		if err != nil {
			return nil, err
		}
		out = append(out, points...)
	}
	return out, nil
}

// supportSet is one plane's lifted set (PlanarSupportSets) over the host and
// guest sides. A nil result means the plane is not a support plane or its set
// is empty.
func supportSet(host, guest *patchSide, plane SupportPlane, band proof.Dyadic, overlap bool,
	poll func() error) ([]PatchPoint, error) {
	guestSolid := guest.prep.s
	if _, ok := host.faceTris[plane.Face]; !ok || !host.isFlat(plane.Face) {
		return nil, nil
	}
	n, o := host.faceNormal(plane.Face), host.faceOrigin(plane.Face)
	heights := make([]proof.Dyadic, len(guestSolid.Verts))
	lowest := proof.DyZero()
	for v, at := range guestSolid.Verts {
		if err := poll(); err != nil {
			return nil, err
		}
		heights[v] = proof.DvDot(n, proof.DvSub(at, o))
		if v == 0 || proof.DyCmp(heights[v], lowest) < 0 {
			lowest = heights[v]
		}
	}
	if (lowest.Sign() < 0) != overlap {
		return nil, nil
	}
	frame := NewPlaneFrame(n, o)
	outer, holes, ok := host.frameLoops(plane.Face, frame, Point3{})
	if !ok {
		return nil, nil
	}
	region := append([][]Point2{outer}, holes...)
	norm := proof.DvDot(n, n)
	limit := proof.DyMul(proof.DyMul(band, band), norm)
	hostFeature := PatchFeature{Kind: FeatureFacet, Faces: []int{plane.Face}}
	var points []PatchPoint
	for v, h := range heights {
		if err := poll(); err != nil {
			return nil, err
		}
		if lowest.Sign() <= 0 && proof.DyCmp(h, lowest) == 0 {
			continue
		}
		if h.Sign() > 0 && proof.DyCmp(proof.DyMul(h, h), limit) > 0 {
			continue
		}
		vertex := ratPoint3(guestSolid.Verts[v])
		lift := new(big.Rat).Quo(h.Rat(), norm.Rat())
		var foot Point3
		for axis := range 3 {
			foot[axis] = new(big.Rat).Sub(vertex[axis], new(big.Rat).Mul(lift, n[axis].Rat()))
		}
		if locate(frame.Project(foot), region) <= 0 {
			continue
		}
		var separation pair.ScalarReading
		if h.Sign() != 0 {
			reading, ok := canonicalSqrt(frac{num: proof.DyMul(h, h), den: norm})
			if !ok {
				return nil, nil
			}
			separation = reading
			if h.Sign() < 0 {
				separation.ValueMM = -separation.ValueMM
			}
		}
		feature := PatchFeature{Kind: FeatureVertex, Faces: guest.faceIDs(guest.vertTris[v])}
		point := PatchPoint{OnA: foot, OnB: vertex, A: hostFeature, B: feature, Normal: n, Separation: separation}
		if !plane.HostIsA {
			point.OnA, point.OnB, point.A, point.B, point.Normal = vertex, foot, feature, hostFeature, dvNeg(n)
		}
		points = append(points, point)
	}
	return points, nil
}
