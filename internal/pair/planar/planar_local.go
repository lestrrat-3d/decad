package planar

import "github.com/lestrrat-3d/decad/internal/proof"

// This file is the local half of the planar relation (planar.go): the proof
// that two solids' materials do not overlap near one zero-distance site.

// fan is the part of one solid's boundary that contains a contact point:
// every triangle holding it with its outward normal, the fan's vertices, the
// directions of the edges holding it, the normals of the triangles whose own
// plane bounds the whole fan from outside (supports), and whether two of its
// triangles are coplanar with opposed normals (folded), which no embedded
// solid has.
type fan struct {
	normals  []proof.DyV3
	verts    []proof.DyV3
	edgeDirs []proof.DyV3
	supports []proof.DyV3
	folded   bool
}

func (k *planarKernel) fanAt(p *planarPrep, x hpoint) (fan, error) {
	var f fan
	var tris []int
	seenVert := make(map[int]struct{})
	seenEdge := make(map[[2]int]struct{})
	p.floatBoxes()
	xb, boxed := x.floatBox()
	for t, tri := range p.s.Tris {
		if err := k.poll(); err != nil {
			return fan{}, err
		}
		if !pointInFacetBoxed(p, t, x, xb, boxed) {
			continue
		}
		tris = append(tris, t)
		f.normals = append(f.normals, p.normal[t])
		for i, v := range tri {
			if _, ok := seenVert[v]; !ok {
				seenVert[v] = struct{}{}
				f.verts = append(f.verts, p.s.Verts[v])
			}
			u, w := tri[i], tri[(i+1)%3]
			key := [2]int{min(u, w), max(u, w)}
			if _, ok := seenEdge[key]; ok {
				continue
			}
			seenEdge[key] = struct{}{}
			if onSegment(p.s.Verts[key[0]], p.s.Verts[key[1]], x) {
				f.edgeDirs = append(f.edgeDirs, proof.DvSub(p.s.Verts[key[1]], p.s.Verts[key[0]]))
			}
		}
	}
	for i, t := range tris {
		a := p.s.Verts[p.s.Tris[t][0]]
		support := true
		for _, v := range f.verts {
			if proof.DvDot(p.normal[t], proof.DvSub(v, a)).Sign() > 0 {
				support = false
				break
			}
		}
		if support {
			f.supports = append(f.supports, p.normal[t])
		}
		for _, other := range f.normals[:i] {
			if proof.DvIsZero(proof.DvCross(other, p.normal[t])) && proof.DvDot(other, p.normal[t]).Sign() < 0 {
				f.folded = true
			}
		}
	}
	return f, nil
}

// onSegment reports whether x lies on the closed segment uw.
func onSegment(u, w proof.DyV3, x hpoint) bool {
	d := proof.DvSub(w, u)
	rel := x.from(u)
	if !proof.DvIsZero(proof.DvCross(d, rel)) {
		return false
	}
	t := proof.DvDot(rel, d)
	return t.Sign() >= 0 && proof.DyCmp(t, proof.DyMul(x.w, proof.DvDot(d, d))) <= 0
}

// localSeparation proves that, near every point of the site (and of its open
// cell, when it carries one), the two materials have disjoint interiors. Two
// certificates are accepted; each rests on one fact: a fan's boundary lying in
// a closed region leaves the open rest of a small ball connected and free of
// that boundary, so it is all material or all empty, and one known point
// decides which.
//
// A separating plane with normal n: A's fan on or behind it, B's on or in
// front, and for each solid a support triangle (its own plane bounds its fan,
// so its material lies behind that plane) whose normal is not a positive
// multiple of the one direction that would let the material fill the far
// half-space. Candidates are the fans' facet normals and the cross products
// of the edges through the site.
//
// A notch: the convex cone Q in front of every fan plane of one solid (the
// host) never meets the host's interior, since a path from an interior point
// out of the host first leaves through some fan triangle, from its back to
// its front, so the point lies behind that triangle's plane. The other
// solid's fan lies in Q, and one of its supports is not a positive multiple
// of a host normal, so its material stays in Q. A box resting in a tray
// corner is this case.
func (k *planarKernel) localSeparation(site *contactSite) (bool, error) {
	fa, err := k.fanAt(k.a, site.at)
	if err != nil {
		return false, err
	}
	fb, err := k.fanAt(k.b, site.at)
	if err != nil {
		return false, err
	}
	if len(fa.normals) == 0 || len(fb.normals) == 0 {
		return false, nil
	}
	candidates := make([]proof.DyV3, 0, len(fa.normals)+len(fb.normals)+2*len(fa.edgeDirs)*len(fb.edgeDirs))
	candidates = append(candidates, fa.normals...)
	for _, n := range fb.normals {
		candidates = append(candidates, dvNeg(n))
	}
	for _, da := range fa.edgeDirs {
		for _, db := range fb.edgeDirs {
			c := proof.DvCross(da, db)
			if !proof.DvIsZero(c) {
				candidates = append(candidates, c, dvNeg(c))
			}
		}
	}
	for _, n := range candidates {
		if err := k.poll(); err != nil {
			return false, err
		}
		if separatesFans(n, site, &fa, &fb) {
			return true, nil
		}
	}
	return fitsNotch(site, &fb, &fa) || fitsNotch(site, &fa, &fb), nil
}

func separatesFans(n proof.DyV3, site *contactSite, fa, fb *fan) bool {
	if site.cell && proof.DvDot(n, site.dir).Sign() != 0 {
		return false
	}
	for _, v := range fa.verts {
		if proof.DvDot(n, site.at.relative(v)).Sign() > 0 {
			return false
		}
	}
	for _, v := range fb.verts {
		if proof.DvDot(n, site.at.relative(v)).Sign() < 0 {
			return false
		}
	}
	// A's material could fill {n·y > 0} only behind a support of normal -λn;
	// B's could fill {n·y < 0} only behind a support of normal λn.
	return supportNotAlong(fa.supports, dvNeg(n)) && supportNotAlong(fb.supports, n)
}

// fitsNotch reports whether the guest fan lies in the cone Q in front of
// every host fan plane, with a guest support that keeps the guest material
// in Q. A folded host would make Q a plane whose outside is two pieces, which
// the guest argument cannot cross. Every host plane holds the whole site,
// since each host triangle contains it, so the test is the same along an
// open cell.
func fitsNotch(site *contactSite, host, guest *fan) bool {
	if host.folded {
		return false
	}
	for _, n := range host.normals {
		for _, v := range guest.verts {
			if proof.DvDot(n, site.at.relative(v)).Sign() < 0 {
				return false
			}
		}
	}
	// The guest material could fill the host's material side only behind a
	// support of normal λ·n for every host normal n.
	for _, n := range host.normals {
		if supportNotAlong(guest.supports, n) {
			return true
		}
	}
	return false
}

// supportNotAlong reports whether some support normal is not a positive
// multiple of dir.
func supportNotAlong(supports []proof.DyV3, dir proof.DyV3) bool {
	for _, s := range supports {
		if !proof.DvIsZero(proof.DvCross(s, dir)) || proof.DvDot(s, dir).Sign() <= 0 {
			return true
		}
	}
	return false
}
