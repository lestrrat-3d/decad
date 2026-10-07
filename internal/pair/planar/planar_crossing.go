package planar

import (
	"github.com/lestrrat-3d/decad/internal/proof"
)

// crossings records every certified transversal crossing in both directions
// and every matching coplanar facet overlap in k.crossed, and every coplanar
// edge-in-facet chord and opposed coplanar facet pair as a contact site. Any
// recorded crossing proves an overlap. A float box pair apart skips a pair
// as its exact boxes would (planar_prune.go).
func (k *planarKernel) crossings() error {
	k.a.floatBoxes()
	k.b.floatBoxes()
	for _, side := range [][2]*planarPrep{{k.a, k.b}, {k.b, k.a}} {
		edges, tris := side[0], side[1]
		for e, edge := range edges.edges {
			p, q := edges.s.Verts[edge[0]], edges.s.Verts[edge[1]]
			for t := range tris.s.Tris {
				if err := k.poll(); err != nil {
					return err
				}
				if floatApart(edges.edgeBox[e], tris.triBox[t]) ||
					boxesApart(edges.edgeLo[e], edges.edgeHi[e], tris.triLo[t], tris.triHi[t]) {
					continue
				}
				sp, sq := orientSign(tris, t, p), orientSign(tris, t, q)
				if sp*sq < 0 && edgeThroughInterior(tris, t, p, q) {
					edgePart := CrossingPart{Edge: true, Ends: edge, Facets: edges.edgeFacets[e]}
					facetPart := CrossingPart{Facets: [2]int{t, t}}
					crossing := PlanarCrossing{A: edgePart, B: facetPart}
					if edges == k.b {
						crossing = PlanarCrossing{A: facetPart, B: edgePart}
					}
					k.crossed = append(k.crossed, crossing)
					continue
				}
				if sp == 0 && sq == 0 {
					k.edgeInFacet(edges, edge, tris, t)
				}
			}
		}
	}
	for ta := range k.a.s.Tris {
		for tb := range k.b.s.Tris {
			if err := k.poll(); err != nil {
				return err
			}
			if floatApart(k.a.triBox[ta], k.b.triBox[tb]) ||
				boxesApart(k.a.triLo[ta], k.a.triHi[ta], k.b.triLo[tb], k.b.triHi[tb]) {
				continue
			}
			coplanar := true
			for _, v := range k.a.s.Tris[ta] {
				if orientSign(k.b, tb, k.a.s.Verts[v]) != 0 {
					coplanar = false
					break
				}
			}
			if !coplanar || !coplanarAreaOverlap(k.a, ta, k.b, tb) {
				continue
			}
			if proof.DvDot(k.a.normal[ta], k.b.normal[tb]).Sign() > 0 {
				k.crossed = append(k.crossed, PlanarCrossing{
					A: CrossingPart{Facets: [2]int{ta, ta}},
					B: CrossingPart{Facets: [2]int{tb, tb}},
				})
				continue
			}
			// Opposed coplanar facets put the two materials on opposite
			// sides of one plane; their shared interior needs no local test.
			k.addSite(contactSite{contact: PlanarContact{
				A: PlanarFeature{Kind: FeatureFacet, Facet: ta},
				B: PlanarFeature{Kind: FeatureFacet, Facet: tb},
			}, skipLocal: true})
		}
	}
	return nil
}

// edgeThroughInterior reports whether the line pq passes strictly inside
// triangle t: the three edge orientations share one nonzero sign.
func edgeThroughInterior(tris *planarPrep, t int, p, q proof.DyV3) bool {
	tri := tris.s.Tris[t]
	d := proof.DvSub(q, p)
	sign := 0
	for i := range 3 {
		u, w := tris.s.Verts[tri[i]], tris.s.Verts[tri[(i+1)%3]]
		s := proof.DvDot(d, proof.DvCross(proof.DvSub(u, p), proof.DvSub(w, p))).Sign()
		if s == 0 || (sign != 0 && s != sign) {
			return false
		}
		sign = s
	}
	return true
}

// coplanarAreaOverlap reports whether two coplanar triangles share a
// positive-area region: no edge line of either weakly separates them.
func coplanarAreaOverlap(a *planarPrep, ta int, b *planarPrep, tb int) bool {
	separates := func(p *planarPrep, t int, other *planarPrep, ot int) bool {
		tri := p.s.Tris[t]
		for i := range 3 {
			u, w := p.s.Verts[tri[i]], p.s.Verts[tri[(i+1)%3]]
			outside := true
			for _, v := range other.s.Tris[ot] {
				if edgeSide(p.normal[t], u, w, dyPoint(other.s.Verts[v])) > 0 {
					outside = false
					break
				}
			}
			if outside {
				return true
			}
		}
		return false
	}
	return !separates(a, ta, b, tb) && !separates(b, tb, a, ta)
}

// edgeInFacet records the open chord of a coplanar edge through a facet's
// interior. A chord that only reaches the facet's boundary is covered by the
// vertex and edge-edge sites.
func (k *planarKernel) edgeInFacet(edges *planarPrep, edge [2]int, tris *planarPrep, t int) {
	p, q := edges.s.Verts[edge[0]], edges.s.Verts[edge[1]]
	d := proof.DvSub(q, p)
	tri := tris.s.Tris[t]
	normal := tris.normal[t]
	lo, hi := frac{num: proof.DyZero(), den: proof.DyInt(1)}, frac{num: proof.DyInt(1), den: proof.DyInt(1)}
	for i := range 3 {
		u, w := tris.s.Verts[tri[i]], tris.s.Verts[tri[(i+1)%3]]
		at := proof.DvDot(proof.DvCross(proof.DvSub(w, u), proof.DvSub(p, u)), normal)
		slope := proof.DvDot(proof.DvCross(proof.DvSub(w, u), d), normal)
		switch slope.Sign() {
		case 0:
			if at.Sign() < 0 {
				return
			}
		case 1:
			bound := frac{num: proof.DyNeg(at), den: slope}
			if fracCmp(bound, lo) > 0 {
				lo = bound
			}
		default:
			bound := frac{num: at, den: proof.DyNeg(slope)}
			if fracCmp(bound, hi) < 0 {
				hi = bound
			}
		}
	}
	if fracCmp(lo, hi) >= 0 {
		return
	}
	// The chord midpoint t = (lo + hi) / 2 in homogeneous form.
	tw := proof.DyMul(proof.DyInt(2), proof.DyMul(lo.den, hi.den))
	tn := proof.DyAdd(proof.DyMul(lo.num, hi.den), proof.DyMul(hi.num, lo.den))
	mid := hpoint{x: proof.DvAdd(dvScale(p, tw), dvScale(d, tn)), w: tw}
	for i := range 3 {
		if edgeSide(normal, tris.s.Verts[tri[i]], tris.s.Verts[tri[(i+1)%3]], mid) <= 0 {
			return
		}
	}
	fe := PlanarFeature{Kind: FeatureEdge, Edge: edge}
	ff := PlanarFeature{Kind: FeatureFacet, Facet: t}
	contact := PlanarContact{A: fe, B: ff}
	if edges == k.b {
		contact = PlanarContact{A: ff, B: fe}
	}
	k.addSite(contactSite{contact: contact, at: mid, dir: d, cell: true})
}
