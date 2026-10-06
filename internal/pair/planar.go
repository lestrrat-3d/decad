package pair

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proof"
)

// This file is the exact relation of two closed planar solids
// (docs/multibody-dynamics-design.md §9.1) and the convexity certificate of
// one (§9.2). Every predicate is an exact sign over dyadic coordinates; the
// only rounding is the final conversion of a positive gap.
//
// The relation is decided in four steps, each sound on its own:
//
//  1. An edge of one solid crossing the interior of a triangle of the other
//     transversally proves Overlapping: just past the crossing, the edge's
//     material wedge lies inside the other solid's material half-space. Two
//     coplanar triangles with matching outward normals and a positive-area
//     overlap prove it the same way. Every such crossing is recorded, so the
//     face-local penetration patch (planar_face_penetration.go) can tell
//     which faces an overlap passes through.
//  2. One parity cast per shell, from a vertex that is not on the other
//     solid's boundary, proves that shell inside (Overlapping) or outside.
//     A shell with no such vertex meets the contact set, which step 4 covers.
//  3. The exact minimum squared distance over vertex-facet and edge-edge
//     candidates, which is the true minimum once step 1 found no crossing.
//     A positive minimum with every shell outside proves Separated.
//  4. At a zero minimum, every point of the contact set must have proven
//     local separation (localSeparation). With that, no boundary can enter
//     the other solid's interior without a contact point where the local
//     cones overlap, and every shell that misses the contact set was cast
//     outside in step 2, so the interiors are disjoint: Touching. Points
//     inside an opposed coplanar facet pair need no test: there the two
//     materials lie on opposite sides of one plane.

// PlanarSolid is the exact closed boundary of an admitted planar solid at one
// pose: every vertex an exact dyadic point and every triangle wound
// counterclockwise seen from outside, so (b-a)×(c-a) is its outward normal.
// Faces, when set, names per triangle the original face that owns it; the
// relation never reads it, and a manifold (planar_manifold.go) needs it.
// CheckPlanarPose may attach the solid's derived data (planar_topology.go),
// which serves it only while Verts and Tris are the slices it was read off.
type PlanarSolid struct {
	Verts []proof.DyV3
	Tris  [][3]int
	Faces []int

	pose *planarPose
}

// FeatureKind names the dimension of a boundary feature.
type FeatureKind uint8

const (
	FeatureVertex FeatureKind = iota + 1
	FeatureEdge
	FeatureFacet
)

// PlanarFeature is the smallest boundary feature that holds a contact point.
// Vertex names a vertex index; Edge names two vertex indices in ascending
// order; Facet names a triangle index. Only the field of Kind is set.
type PlanarFeature struct {
	Kind   FeatureKind
	Vertex int
	Edge   [2]int
	Facet  int
}

// PlanarContact is one zero-distance feature pair of a touching relation.
type PlanarContact struct {
	A, B PlanarFeature
}

// CrossingPart is one solid's part of a certified crossing: a facet, or an
// edge (Ends, its vertex indices in ascending order) with the two facets that
// hold it. Facets names the facet twice when Edge is false.
type CrossingPart struct {
	Edge   bool
	Ends   [2]int
	Facets [2]int
}

// PlanarCrossing is one certified crossing of the relation's first step: an
// edge of one solid through the interior of a facet of the other, or two
// coplanar facets with matching outward normals and a positive-area overlap.
type PlanarCrossing struct {
	A, B CrossingPart
}

// PlanarResult is the proven relation of two planar solids. Gap is set for
// Separated and Touching; Contacts lists every zero-distance feature pair of a
// Touching relation in scan order. Crossings lists, in scan order, every
// certified crossing behind an Overlapping relation; it is empty when the
// overlap was proven by nesting alone.
type PlanarResult struct {
	Relation  Relation
	Reason    Reason
	Gap       *ScalarReading
	Contacts  []PlanarContact
	Crossings []PlanarCrossing

	// preps is the derived data ClassifyPlanar built for its two solids,
	// which PlanarSupportSets reuses (preparedFor).
	preps [2]*planarPrep
}

// preparedFor returns the derived data of s that ClassifyPlanar built, or
// builds it when s is not one of the solids this result classified. The data
// is a pure function of the solid, so a solid unchanged since
// ClassifyPlanar read it gets the same data either way.
func (r PlanarResult) preparedFor(s *PlanarSolid) *planarPrep {
	for _, prep := range r.preps {
		if prep != nil && prep.s == s {
			return prep
		}
	}
	return preparePlanar(s)
}

// CheckPlanarSolid audits a snapshot before any relation reads it: indices in
// range, a nonzero exact normal on every triangle, every directed edge matched
// by exactly one reverse, and a positive exact signed volume. A false result is
// a refusal of the snapshot, never a relation. poll charges one unit of work
// and reports cancellation.
func CheckPlanarSolid(s *PlanarSolid, poll func() error) (bool, error) {
	if len(s.Verts) < 4 || len(s.Tris) < 4 {
		return false, nil
	}
	directed := make(map[[2]int]int, 3*len(s.Tris))
	volume := proof.DyZero()
	origin := s.Verts[0]
	for _, tri := range s.Tris {
		if err := poll(); err != nil {
			return false, err
		}
		for _, v := range tri {
			if v < 0 || v >= len(s.Verts) {
				return false, nil
			}
		}
		if tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0] {
			return false, nil
		}
		a, b, c := s.Verts[tri[0]], s.Verts[tri[1]], s.Verts[tri[2]]
		normal := proof.DvCross(proof.DvSub(b, a), proof.DvSub(c, a))
		if proof.DvIsZero(normal) {
			return false, nil
		}
		volume = proof.DyAdd(volume, proof.DvDot(normal, proof.DvSub(a, origin)))
		for i := range 3 {
			directed[[2]int{tri[i], tri[(i+1)%3]}]++
		}
	}
	for edge, count := range directed {
		if err := poll(); err != nil {
			return false, err
		}
		if count != 1 || directed[[2]int{edge[1], edge[0]}] != 1 {
			return false, nil
		}
	}
	return volume.Sign() > 0, nil
}

// PlanarConvex is the §9.2 certificate: every vertex lies on or behind every
// triangle's plane. Combined with CheckPlanarSolid's audit this proves the
// solid is the convex polytope its facet half-spaces cut out; a hollow or
// multi-lump solid fails it, since some vertex lies in front of a facet. The
// result is invariant under any affine map with a positive determinant.
func PlanarConvex(s *PlanarSolid, poll func() error) (bool, error) {
	for _, tri := range s.Tris {
		a := s.Verts[tri[0]]
		normal := proof.DvCross(proof.DvSub(s.Verts[tri[1]], a), proof.DvSub(s.Verts[tri[2]], a))
		for _, v := range s.Verts {
			if err := poll(); err != nil {
				return false, err
			}
			if proof.DvDot(normal, proof.DvSub(v, a)).Sign() > 0 {
				return false, nil
			}
		}
	}
	return true, nil
}

// ClassifyPlanar decides the relation of two audited planar solids. Both must
// have passed CheckPlanarSolid. poll is charged once per exact predicate
// group; its error is returned unchanged. The result carries the solids'
// derived data for PlanarSupportSets, so neither solid may change while the
// result is in use.
func ClassifyPlanar(a, b *PlanarSolid, poll func() error) (PlanarResult, error) {
	pa, pb := preparePlanar(a), preparePlanar(b)
	result, err := classifyPrepared(pa, pb, poll)
	if err != nil {
		return PlanarResult{}, err
	}
	result.preps = [2]*planarPrep{pa, pb}
	return result, nil
}

func classifyPrepared(pa, pb *planarPrep, poll func() error) (PlanarResult, error) {
	k := &planarKernel{a: pa, b: pb, poll: poll}
	if err := k.crossings(); err != nil {
		return PlanarResult{}, err
	}
	if len(k.crossed) > 0 {
		return PlanarResult{Relation: Overlapping, Crossings: k.crossed}, nil
	}
	for _, side := range [][2]*planarPrep{{pa, pb}, {pb, pa}} {
		inside, decided, err := k.shellsInside(side[0], side[1])
		if err != nil {
			return PlanarResult{}, err
		}
		if !decided {
			return PlanarResult{Reason: AmbiguousFeature}, nil
		}
		if inside {
			return PlanarResult{Relation: Overlapping}, nil
		}
	}
	if err := k.distances(); err != nil {
		return PlanarResult{}, err
	}
	if k.best.num.Sign() > 0 {
		gap, ok := fracSqrtReading(k.best)
		if !ok {
			return PlanarResult{Reason: NoGapProof}, nil
		}
		return PlanarResult{Relation: Separated, Gap: &gap}, nil
	}
	if len(k.sites) == 0 {
		// A zero minimum always comes with a recorded site; without one the
		// contact set is unknown.
		return PlanarResult{Reason: AmbiguousFeature}, nil
	}
	for i := range k.sites {
		if err := poll(); err != nil {
			return PlanarResult{}, err
		}
		site := &k.sites[i]
		if site.skipLocal {
			continue
		}
		separated, err := k.localSeparation(site)
		if err != nil {
			return PlanarResult{}, err
		}
		if !separated {
			return PlanarResult{Reason: AmbiguousFeature}, nil
		}
	}
	contacts := make([]PlanarContact, 0, len(k.sites))
	for _, site := range k.sites {
		contacts = append(contacts, site.contact)
	}
	return PlanarResult{Relation: Touching, Gap: &ScalarReading{}, Contacts: contacts}, nil
}

// planarPrep holds one solid's derived exact data: outward normals, per
// triangle and per edge boxes, the unique undirected edges with the two
// triangles holding each, and the shell of every vertex.
type planarPrep struct {
	s              *PlanarSolid
	normal         []proof.DyV3
	triLo, triHi   [][3]proof.Dyadic
	edges          [][2]int
	edgeFacets     [][2]int
	edgeLo, edgeHi [][3]proof.Dyadic
	shellOf        []int
	shells         int

	// The distance scan's shortcuts (planar_prune.go), filled on first use.
	vertBox, triBox, edgeBox []floatBox
	sides                    [][3]proof.DyV3
	sideBoxes                [][3]floatBox
	sidesSet                 []bool
}

// preparePlanar builds s's derived data, or reuses the pose data
// CheckPlanarPose attached to it.
func preparePlanar(s *PlanarSolid) *planarPrep {
	if d := s.pose; d != nil && d.serves(s) {
		return d.prep(s)
	}
	return newPlanarPose(s, planarTopologyOf(len(s.Verts), s.Tris), planarNormals(s)).prep(s)
}

func pointBox(verts []proof.DyV3, indices []int) ([3]proof.Dyadic, [3]proof.Dyadic) {
	var lo, hi [3]proof.Dyadic
	for i, v := range indices {
		for axis := range 3 {
			if i == 0 || proof.DyCmp(verts[v][axis], lo[axis]) < 0 {
				lo[axis] = verts[v][axis]
			}
			if i == 0 || proof.DyCmp(verts[v][axis], hi[axis]) > 0 {
				hi[axis] = verts[v][axis]
			}
		}
	}
	return lo, hi
}

// boxGapSquared is the exact squared distance between two closed boxes.
func boxGapSquared(alo, ahi, blo, bhi [3]proof.Dyadic) proof.Dyadic {
	out := proof.DyZero()
	for axis := range 3 {
		var gap proof.Dyadic
		switch {
		case proof.DyCmp(ahi[axis], blo[axis]) < 0:
			gap = proof.DySubScalar(blo[axis], ahi[axis])
		case proof.DyCmp(bhi[axis], alo[axis]) < 0:
			gap = proof.DySubScalar(alo[axis], bhi[axis])
		default:
			continue
		}
		out = proof.DyAdd(out, proof.DyMul(gap, gap))
	}
	return out
}

// boxesApart reports whether two closed boxes are strictly apart along some
// axis, which is exactly when boxGapSquared is positive, without forming the
// gap.
func boxesApart(alo, ahi, blo, bhi [3]proof.Dyadic) bool {
	for axis := range 3 {
		if proof.DyCmp(ahi[axis], blo[axis]) < 0 || proof.DyCmp(bhi[axis], alo[axis]) < 0 {
			return true
		}
	}
	return false
}

// frac is an exact nonnegative rational num/den with den > 0.
type frac struct {
	num, den proof.Dyadic
}

func fracCmp(x, y frac) int {
	return proof.DyCmp(proof.DyMul(x.num, y.den), proof.DyMul(y.num, x.den))
}

// fracSqrtReading encloses sqrt(num/den) between two floats, each proven by an
// exact comparison of its square, and publishes their midpoint with an
// outward half-width. An exactly representable root has a zero bound.
func fracSqrtReading(x frac) (ScalarReading, bool) {
	squareAtMost := func(f float64) bool {
		d, ok := proof.DyOf(f)
		return ok && proof.DyCmp(proof.DyMul(proof.DyMul(d, d), x.den), x.num) <= 0
	}
	squareAtLeast := func(f float64) bool {
		d, ok := proof.DyOf(f)
		return ok && proof.DyCmp(proof.DyMul(proof.DyMul(d, d), x.den), x.num) >= 0
	}
	seed := proof.DySqrtSeed(x.num) / proof.DySqrtSeed(x.den)
	if !finite(seed) || seed <= 0 {
		return ScalarReading{}, false
	}
	lo, hi := seed, seed
	for range proof.SqrtAdjustLimit {
		if squareAtMost(lo) {
			break
		}
		lo = math.Nextafter(lo, 0)
	}
	for range proof.SqrtAdjustLimit {
		if squareAtLeast(hi) {
			break
		}
		hi = math.Nextafter(hi, math.Inf(1))
	}
	if !squareAtMost(lo) || !squareAtLeast(hi) || lo <= 0 || !finite(hi) {
		return ScalarReading{}, false
	}
	if lo == hi {
		return ScalarReading{ValueMM: lo}, true
	}
	value := lo + (hi-lo)/2
	bound := proof.ProvenUpRound(math.Max(value-lo, hi-value))
	return ScalarReading{ValueMM: value, BoundMM: bound}, finite(value) && finite(bound)
}

// hpoint is the exact point x/w with w > 0. Contact points on two crossing
// edges or at a chord midpoint are rational, not dyadic; carrying the common
// denominator keeps every predicate over them a dyadic sign.
type hpoint struct {
	x proof.DyV3
	w proof.Dyadic
}

func dyPoint(v proof.DyV3) hpoint { return hpoint{x: v, w: proof.DyInt(1)} }

func dvScale(v proof.DyV3, s proof.Dyadic) proof.DyV3 {
	return proof.DyV3{proof.DyMul(v[0], s), proof.DyMul(v[1], s), proof.DyMul(v[2], s)}
}

// relative returns w·v − x, which has the sign pattern of v − x/w.
func (h hpoint) relative(v proof.DyV3) proof.DyV3 { return proof.DvSub(dvScale(v, h.w), h.x) }

// from returns x − w·u, which has the sign pattern of x/w − u.
func (h hpoint) from(u proof.DyV3) proof.DyV3 { return proof.DvSub(h.x, dvScale(u, h.w)) }

// contactSite is one zero-distance feature pair and the point (or open cell
// through it, along dir) where localSeparation must hold.
type contactSite struct {
	contact   PlanarContact
	at        hpoint
	dir       proof.DyV3
	cell      bool
	skipLocal bool
}

type planarKernel struct {
	a, b    *planarPrep
	poll    func() error
	best    frac
	bestUp  float64 // a float at or above best (fracAbove)
	hasBest bool
	sites   []contactSite
	seen    map[PlanarContact]struct{}
	crossed []PlanarCrossing
}

func orientSign(p *planarPrep, t int, v proof.DyV3) int {
	return proof.DvDot(p.normal[t], proof.DvSub(v, p.s.Verts[p.s.Tris[t][0]])).Sign()
}

// edgeSide is the in-plane test of x against the directed triangle edge u->w:
// positive strictly inside, zero on the edge line.
func edgeSide(normal, u, w proof.DyV3, x hpoint) int {
	return proof.DvDot(proof.DvCross(proof.DvSub(w, u), x.from(u)), normal).Sign()
}

// crossings records every certified transversal crossing in both directions
// and every matching coplanar facet overlap in k.crossed, and every coplanar
// edge-in-facet chord and opposed coplanar facet pair as a contact site. Any
// recorded crossing proves an overlap.
func (k *planarKernel) crossings() error {
	for _, side := range [][2]*planarPrep{{k.a, k.b}, {k.b, k.a}} {
		edges, tris := side[0], side[1]
		for e, edge := range edges.edges {
			p, q := edges.s.Verts[edge[0]], edges.s.Verts[edge[1]]
			for t := range tris.s.Tris {
				if err := k.poll(); err != nil {
					return err
				}
				if boxesApart(edges.edgeLo[e], edges.edgeHi[e], tris.triLo[t], tris.triHi[t]) {
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
			if boxesApart(k.a.triLo[ta], k.a.triHi[ta], k.b.triLo[tb], k.b.triHi[tb]) {
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
	}
}

// pruned reports whether a box pair is provably farther than the current
// minimum, so no candidate inside it can lower the minimum or touch. Every
// offered candidate has a nonnegative numerator, so boxes that are not apart
// (gap zero) are never pruned, and against a zero minimum over a positive
// denominator any boxes apart are. fa and fb are the boxes' outward float
// copies. A float gap already past the minimum's float upper bound prunes
// without an exact test (planar_prune.go): that bound is at least zero, so a
// float gap past it also proves the boxes apart.
func (k *planarKernel) pruned(alo, ahi, blo, bhi [3]proof.Dyadic, fa, fb floatBox) bool {
	if !k.hasBest {
		return false
	}
	if gapSquaredBelow(fa, fb) > k.bestUp {
		return true
	}
	if !boxesApart(alo, ahi, blo, bhi) {
		return false
	}
	if k.best.num.Sign() == 0 && k.best.den.Sign() > 0 {
		return true
	}
	return fracCmp(frac{num: boxGapSquared(alo, ahi, blo, bhi), den: proof.DyInt(1)}, k.best) > 0
}

// distances computes the exact minimum squared distance over vertex-facet
// and edge-edge candidates and records every zero-distance site.
func (k *planarKernel) distances() error {
	k.a.floatBoxes()
	k.b.floatBoxes()
	for _, side := range [][2]*planarPrep{{k.a, k.b}, {k.b, k.a}} {
		verts, tris := side[0], side[1]
		for v, point := range verts.s.Verts {
			for t := range tris.s.Tris {
				if err := k.poll(); err != nil {
					return err
				}
				if k.pruned(point, point, tris.triLo[t], tris.triHi[t], verts.vertBox[v], tris.triBox[t]) {
					continue
				}
				k.vertexFacet(verts, v, tris, t)
			}
		}
	}
	for ea, edgeA := range k.a.edges {
		for eb, edgeB := range k.b.edges {
			if err := k.poll(); err != nil {
				return err
			}
			if k.pruned(k.a.edgeLo[ea], k.a.edgeHi[ea], k.b.edgeLo[eb], k.b.edgeHi[eb], k.a.edgeBox[ea], k.b.edgeBox[eb]) {
				continue
			}
			k.edgeEdge(edgeA, edgeB)
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

// rayLadder is the fixed, deterministic sequence of parity-cast directions.
var rayLadder = [][3]int64{
	{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {-1, 0, 0}, {0, -1, 0}, {0, 0, -1},
	{3, 5, 7}, {-7, 3, 5}, {5, -7, 3}, {3, 5, -7}, {-5, -3, 7}, {7, -5, -3},
	{-3, 7, -5}, {11, 13, 17}, {-13, 17, 11}, {17, -11, 13},
}

// castResult is a parity cast's outcome for one point.
type castResult int

const (
	castAmbiguous castResult = iota
	castInside
	castOutside
	castOnBoundary
)

// cast classifies p against the closed surface of solid by counting the
// facets a ray crosses strictly inside. A ray that meets a facet's boundary
// ahead of p moves to the next direction; p on a facet is reported as such.
func (k *planarKernel) cast(p proof.DyV3, solid *planarPrep) (castResult, error) {
	for _, dir := range rayLadder {
		d := proof.DyV3{proof.DyInt(dir[0]), proof.DyInt(dir[1]), proof.DyInt(dir[2])}
		crossings := 0
		ambiguous := false
		for t, tri := range solid.s.Tris {
			if err := k.poll(); err != nil {
				return castAmbiguous, err
			}
			var signs [3]int
			for i := range 3 {
				u, w := solid.s.Verts[tri[i]], solid.s.Verts[tri[(i+1)%3]]
				signs[i] = proof.DvDot(d, proof.DvCross(proof.DvSub(u, p), proof.DvSub(w, p))).Sign()
			}
			if hasSign(signs, 1) && hasSign(signs, -1) {
				continue
			}
			normal := solid.normal[t]
			along := proof.DvDot(normal, d).Sign()
			ahead := proof.DvDot(normal, proof.DvSub(solid.s.Verts[tri[0]], p)).Sign()
			if along == 0 {
				if ahead != 0 {
					continue
				}
				if pointInFacet(solid, t, dyPoint(p)) {
					return castOnBoundary, nil
				}
				ambiguous = true
				break
			}
			if ahead == 0 {
				return castOnBoundary, nil
			}
			if ahead != along {
				continue
			}
			if signs[0] == 0 || signs[1] == 0 || signs[2] == 0 {
				ambiguous = true
				break
			}
			crossings++
		}
		if ambiguous {
			continue
		}
		if crossings%2 == 1 {
			return castInside, nil
		}
		return castOutside, nil
	}
	return castAmbiguous, nil
}

func hasSign(signs [3]int, sign int) bool {
	return signs[0] == sign || signs[1] == sign || signs[2] == sign
}

// inBox reports whether x/w lies in the closed box [lo, hi]: lo·w ≤ x ≤ hi·w
// on every axis for w > 0, the reverse for w < 0. A unit w compares the
// coordinates directly. A zero w makes no claim and reports true.
func (h hpoint) inBox(lo, hi [3]proof.Dyadic) bool {
	sign := h.w.Sign()
	if sign == 0 {
		return true
	}
	unit := h.w.Exp() == 0 && h.w.Mant().IsInt64() && h.w.Mant().Int64() == 1
	for axis := range 3 {
		low, high := lo[axis], hi[axis]
		if !unit {
			low, high = proof.DyMul(low, h.w), proof.DyMul(high, h.w)
			if sign < 0 {
				low, high = high, low
			}
		}
		if proof.DyCmp(h.x[axis], low) < 0 || proof.DyCmp(h.x[axis], high) > 0 {
			return false
		}
	}
	return true
}

// pointInFacet reports whether x lies in the closed triangle t. The closed
// triangle lies in its box, so a point outside the box, decided by
// comparisons, is outside the triangle. The box decides only for a triangle
// with a nonzero normal, the one the plane and edge tests below bound: on a
// zero normal every test reads zero and accepts any point.
func pointInFacet(p *planarPrep, t int, x hpoint) bool {
	if !proof.DvIsZero(p.normal[t]) && !x.inBox(p.triLo[t], p.triHi[t]) {
		return false
	}
	tri := p.s.Tris[t]
	if proof.DvDot(p.normal[t], x.from(p.s.Verts[tri[0]])).Sign() != 0 {
		return false
	}
	for i := range 3 {
		if edgeSide(p.normal[t], p.s.Verts[tri[i]], p.s.Verts[tri[(i+1)%3]], x) < 0 {
			return false
		}
	}
	return true
}

// shellsInside casts one witness vertex per shell of from against to. A shell
// whose every vertex lies on to's boundary needs no cast: it meets the
// contact set, which localSeparation covers. decided is false when every
// witness of some shell was ambiguous on every ray.
func (k *planarKernel) shellsInside(from, to *planarPrep) (bool, bool, error) {
	done := make([]bool, from.shells)
	pending := make([]bool, from.shells)
	for v, point := range from.s.Verts {
		shell := from.shellOf[v]
		if done[shell] {
			continue
		}
		result, err := k.cast(point, to)
		if err != nil {
			return false, false, err
		}
		switch result {
		case castInside:
			return true, true, nil
		case castOutside:
			done[shell], pending[shell] = true, false
		case castAmbiguous:
			pending[shell] = true
		case castOnBoundary:
		}
	}
	for _, p := range pending {
		if p {
			return false, false, nil
		}
	}
	return false, true, nil
}

func dvNeg(v proof.DyV3) proof.DyV3 {
	return proof.DyV3{proof.DyNeg(v[0]), proof.DyNeg(v[1]), proof.DyNeg(v[2])}
}
