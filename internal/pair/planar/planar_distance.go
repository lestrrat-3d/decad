package planar

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proof"
)

func (k *planarKernel) addSite(site contactSite) {
	if k.seen == nil {
		k.seen = make(map[PlanarContact]struct{})
	}
	if _, ok := k.seen[site.contact]; ok {
		return
	}
	k.seen[site.contact] = struct{}{}
	k.sites = append(k.sites, site)
}

// offer folds one candidate squared distance into the running minimum.
func (k *planarKernel) offer(d frac) {
	if !k.hasBest || fracCmp(d, k.best) < 0 {
		k.best, k.bestUp, k.hasBest = d, fracAbove(d), true
		k.nearest = k.cur
	}
}

// pruned reports whether a box pair is provably farther than the current
// minimum, or than the cap, so no candidate inside it can lower the minimum or
// touch. Every offered candidate has a nonnegative numerator, so boxes that
// are not apart (gap zero) are never pruned, and against a zero minimum over a
// positive denominator any boxes apart are. fa and fb are the boxes' outward
// float copies. A float gap already past a bound's float copy prunes without
// an exact test (planar_prune.go): that bound is at least zero, so a float gap
// past it also proves the boxes apart.
func (k *planarKernel) pruned(alo, ahi, blo, bhi [3]proof.Dyadic, fa, fb floatBox) bool {
	up, bounded := k.boundUp()
	if !bounded {
		return false
	}
	if gapSquaredBelow(fa, fb) > up {
		return true
	}
	if !boxesApart(alo, ahi, blo, bhi) {
		return false
	}
	var gap *frac
	beyond := func(bound frac) bool {
		if bound.num.Sign() == 0 && bound.den.Sign() > 0 {
			return true
		}
		if gap == nil {
			gap = &frac{num: boxGapSquared(alo, ahi, blo, bhi), den: proof.DyInt(1)}
		}
		return fracCmp(*gap, bound) > 0
	}
	return (k.hasBest && beyond(k.best)) || (k.hasCap && beyond(k.cap))
}

// boundUp returns the least float upper bound among the running minimum's and
// the cap's, and false when neither is set.
func (k *planarKernel) boundUp() (float64, bool) {
	switch {
	case k.hasBest && k.hasCap:
		return math.Min(k.bestUp, k.capUp), true
	case k.hasBest:
		return k.bestUp, true
	case k.hasCap:
		return k.capUp, true
	}
	return 0, false
}

// distances computes the exact minimum squared distance over vertex-facet
// and edge-edge candidates and records every zero-distance site. The scan
// visits each vertex against blocks of consecutive facets, and each edge of
// a against blocks of consecutive edges of b, in index order; a block whose
// float box is past the bound skips every pair in it (planar_prune.go).
func (k *planarKernel) distances() error {
	k.a.floatBoxes()
	k.b.floatBoxes()
	k.applyHint()
	return k.scan()
}

func (k *planarKernel) scan() error {
	for _, side := range [][2]*planarPrep{{k.a, k.b}, {k.b, k.a}} {
		verts, tris := side[0], side[1]
		for v, point := range verts.s.Verts {
			for block, box := range tris.triBlock {
				t0, t1 := block*scanBlock, min((block+1)*scanBlock, len(tris.s.Tris))
				skip, err := k.blockPruned(verts.vertBox[v], box, t1-t0)
				if err != nil {
					return err
				}
				if skip {
					continue
				}
				for t := t0; t < t1; t++ {
					if err := k.poll(); err != nil {
						return err
					}
					if k.pruned(point, point, tris.triLo[t], tris.triHi[t], verts.vertBox[v], tris.triBox[t]) {
						continue
					}
					k.cur = vertexFacetHint(verts == k.b, v, t)
					k.vertexFacet(verts, v, tris, t)
				}
			}
		}
	}
	for ea, edgeA := range k.a.edges {
		for block, box := range k.b.edgeBlock {
			e0, e1 := block*scanBlock, min((block+1)*scanBlock, len(k.b.edges))
			skip, err := k.blockPruned(k.a.edgeBox[ea], box, e1-e0)
			if err != nil {
				return err
			}
			if skip {
				continue
			}
			for eb := e0; eb < e1; eb++ {
				if err := k.poll(); err != nil {
					return err
				}
				if k.pruned(k.a.edgeLo[ea], k.a.edgeHi[ea], k.b.edgeLo[eb], k.b.edgeHi[eb], k.a.edgeBox[ea], k.b.edgeBox[eb]) {
					continue
				}
				k.cur = PlanarHint{kind: hintEdgeEdge, i: ea, j: eb}
				k.edgeEdge(edgeA, k.b.edges[eb])
			}
		}
	}
	return nil
}

// vertexFacet offers the distance from a vertex to a facet's plane when its
// projection lies in the closed facet; projections outside are covered by
// the edge-edge candidates. A zero distance records a site at the vertex.
// Each side sign is edgeSide's, read through the triangle's side planes and
// their float pre-test (planar_prune.go).
func (k *planarKernel) vertexFacet(verts *planarPrep, v int, tris *planarPrep, t int) {
	point := verts.s.Verts[v]
	tri := tris.s.Tris[t]
	normal := tris.normal[t]
	planes, boxes := tris.sidePlanes(t)
	var sides [3]int
	for i := range 3 {
		sign, ok := floatDotSign(verts.vertBox[v], tris.vertBox[tri[i]], boxes[i])
		if !ok {
			sign = proof.DvDot(proof.DvSub(point, tris.s.Verts[tri[i]]), planes[i]).Sign()
		}
		sides[i] = sign
		if sides[i] < 0 {
			return
		}
	}
	height := proof.DvDot(normal, proof.DvSub(point, tris.s.Verts[tri[0]]))
	k.offer(frac{num: proof.DyMul(height, height), den: proof.DvDot(normal, normal)})
	if height.Sign() != 0 {
		return
	}
	var onTri PlanarFeature
	zeros := 0
	for i := range 3 {
		if sides[i] == 0 {
			zeros++
		}
	}
	switch zeros {
	case 0:
		onTri = PlanarFeature{Kind: FeatureFacet, Facet: t}
	case 1:
		for i := range 3 {
			if sides[i] == 0 {
				u, w := tri[i], tri[(i+1)%3]
				onTri = PlanarFeature{Kind: FeatureEdge, Edge: [2]int{min(u, w), max(u, w)}}
			}
		}
	default:
		for i := range 3 {
			if sides[i] == 0 && sides[(i+2)%3] == 0 {
				onTri = PlanarFeature{Kind: FeatureVertex, Vertex: tri[i]}
			}
		}
	}
	fv := PlanarFeature{Kind: FeatureVertex, Vertex: v}
	contact := PlanarContact{A: fv, B: onTri}
	if verts == k.b {
		contact = PlanarContact{A: onTri, B: fv}
	}
	k.addSite(contactSite{contact: contact, at: dyPoint(point)})
}

// edgeEdge offers the exact squared distance between two segments: the four
// endpoint-to-segment distances, and the line distance when the closest pair
// is interior to both. An interior crossing records a site at the crossing; a
// collinear overlap records a site at its midpoint along its direction.
func (k *planarKernel) edgeEdge(edgeA, edgeB [2]int) {
	p0, p1 := k.a.s.Verts[edgeA[0]], k.a.s.Verts[edgeA[1]]
	q0, q1 := k.b.s.Verts[edgeB[0]], k.b.s.Verts[edgeB[1]]
	k.offer(pointSegment(p0, q0, q1))
	k.offer(pointSegment(p1, q0, q1))
	k.offer(pointSegment(q0, p0, p1))
	k.offer(pointSegment(q1, p0, p1))
	d1, d2 := proof.DvSub(p1, p0), proof.DvSub(q1, q0)
	r := proof.DvSub(p0, q0)
	cross := proof.DvCross(d1, d2)
	contact := PlanarContact{A: PlanarFeature{Kind: FeatureEdge, Edge: edgeA},
		B: PlanarFeature{Kind: FeatureEdge, Edge: edgeB}}
	if proof.DvIsZero(cross) {
		if !proof.DvIsZero(proof.DvCross(r, d1)) {
			return
		}
		a := proof.DvDot(d1, d1)
		t0 := proof.DvDot(proof.DvSub(q0, p0), d1)
		t1 := proof.DvDot(proof.DvSub(q1, p0), d1)
		lo, hi := dyMax(proof.DyZero(), dyMin(t0, t1)), dyMin(a, dyMax(t0, t1))
		if proof.DyCmp(lo, hi) >= 0 {
			return
		}
		w := proof.DyMul(proof.DyInt(2), a)
		mid := hpoint{x: proof.DvAdd(dvScale(p0, w), dvScale(d1, proof.DyAdd(lo, hi))), w: w}
		k.addSite(contactSite{contact: contact, at: mid, dir: d1, cell: true})
		return
	}
	a, b, c := proof.DvDot(d1, d1), proof.DvDot(d1, d2), proof.DvDot(d2, d2)
	d, e := proof.DvDot(d1, r), proof.DvDot(d2, r)
	den := proof.DvDot(cross, cross)
	s := proof.DySubScalar(proof.DyMul(b, e), proof.DyMul(c, d))
	t := proof.DySubScalar(proof.DyMul(a, e), proof.DyMul(b, d))
	if s.Sign() <= 0 || proof.DyCmp(s, den) >= 0 || t.Sign() <= 0 || proof.DyCmp(t, den) >= 0 {
		return
	}
	height := proof.DvDot(r, cross)
	k.offer(frac{num: proof.DyMul(height, height), den: den})
	if height.Sign() != 0 {
		return
	}
	at := hpoint{x: proof.DvAdd(dvScale(p0, den), dvScale(d1, s)), w: den}
	k.addSite(contactSite{contact: contact, at: at})
}

// pointSegment is the exact squared distance from x to the segment s0s1.
func pointSegment(x, s0, s1 proof.DyV3) frac {
	d := proof.DvSub(s1, s0)
	rel := proof.DvSub(x, s0)
	t := proof.DvDot(rel, d)
	den := proof.DvDot(d, d)
	one := proof.DyInt(1)
	switch {
	case t.Sign() <= 0:
		return frac{num: proof.DvDot(rel, rel), den: one}
	case proof.DyCmp(t, den) >= 0:
		far := proof.DvSub(x, s1)
		return frac{num: proof.DvDot(far, far), den: one}
	default:
		return frac{num: proof.DySubScalar(proof.DyMul(proof.DvDot(rel, rel), den), proof.DyMul(t, t)), den: den}
	}
}
