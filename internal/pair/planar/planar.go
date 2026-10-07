package planar

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/pair"
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
	Relation  pair.Relation
	Reason    pair.Reason
	Gap       *pair.ScalarReading
	Contacts  []PlanarContact
	Crossings []PlanarCrossing
	// Nearest names the candidate pair that set the distance scan's
	// minimum, the hint for a later call (ClassifyPlanarHinted). It is the
	// zero hint when the scan did not run.
	Nearest PlanarHint

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
	return ClassifyPlanarHinted(a, b, PlanarHint{}, poll)
}

// ClassifyPlanarHinted is ClassifyPlanar with a guess at the nearest
// candidate pair, usually the Nearest of an earlier result for the same two
// solids at a nearby pose. The guess only speeds the distance scan: the result
// is the same for every hint, including one that names no candidate of these
// solids (planar_prune.go).
func ClassifyPlanarHinted(a, b *PlanarSolid, hint PlanarHint, poll func() error) (PlanarResult, error) {
	pa, pb := preparePlanar(a), preparePlanar(b)
	k := &planarKernel{a: pa, b: pb, poll: poll, hint: hint}
	result, err := k.classify()
	if err != nil {
		return PlanarResult{}, err
	}
	result.preps = [2]*planarPrep{pa, pb}
	result.Nearest = k.nearest
	return result, nil
}

func (k *planarKernel) classify() (PlanarResult, error) {
	pa, pb, poll := k.a, k.b, k.poll
	if err := k.crossings(); err != nil {
		return PlanarResult{}, err
	}
	if len(k.crossed) > 0 {
		return PlanarResult{Relation: pair.Overlapping, Crossings: k.crossed}, nil
	}
	for _, side := range [][2]*planarPrep{{pa, pb}, {pb, pa}} {
		inside, decided, err := k.shellsInside(side[0], side[1])
		if err != nil {
			return PlanarResult{}, err
		}
		if !decided {
			return PlanarResult{Reason: pair.AmbiguousFeature}, nil
		}
		if inside {
			return PlanarResult{Relation: pair.Overlapping}, nil
		}
	}
	if err := k.distances(); err != nil {
		return PlanarResult{}, err
	}
	if k.best.num.Sign() > 0 {
		gap, ok := fracSqrtReading(k.best)
		if !ok {
			return PlanarResult{Reason: pair.NoGapProof}, nil
		}
		return PlanarResult{Relation: pair.Separated, Gap: &gap}, nil
	}
	if len(k.sites) == 0 {
		// A zero minimum always comes with a recorded site; without one the
		// contact set is unknown.
		return PlanarResult{Reason: pair.AmbiguousFeature}, nil
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
			return PlanarResult{Reason: pair.AmbiguousFeature}, nil
		}
	}
	contacts := make([]PlanarContact, 0, len(k.sites))
	for _, site := range k.sites {
		contacts = append(contacts, site.contact)
	}
	return PlanarResult{Relation: pair.Touching, Gap: &pair.ScalarReading{}, Contacts: contacts}, nil
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
	triBlock, edgeBlock      []floatBox
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
func fracSqrtReading(x frac) (pair.ScalarReading, bool) {
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
		return pair.ScalarReading{}, false
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
		return pair.ScalarReading{}, false
	}
	if lo == hi {
		return pair.ScalarReading{ValueMM: lo}, true
	}
	value := lo + (hi-lo)/2
	bound := proof.ProvenUpRound(math.Max(value-lo, hi-value))
	return pair.ScalarReading{ValueMM: value, BoundMM: bound}, finite(value) && finite(bound)
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
	// cap is a squared distance at or above the exact minimum, known before
	// the scan, that prunes as best does but is never offered (hint).
	cap    frac
	capUp  float64 // a float at or above cap (fracAbove)
	hasCap bool
	// hint is the caller's guess at the nearest candidate pair, cur names
	// the candidate pair being offered, and nearest the one that set best.
	hint, cur, nearest PlanarHint
	sites              []contactSite
	seen               map[PlanarContact]struct{}
	crossed            []PlanarCrossing
}

func orientSign(p *planarPrep, t int, v proof.DyV3) int {
	return proof.DvDot(p.normal[t], proof.DvSub(v, p.s.Verts[p.s.Tris[t][0]])).Sign()
}

// edgeSide is the in-plane test of x against the directed triangle edge u->w:
// positive strictly inside, zero on the edge line.
func edgeSide(normal, u, w proof.DyV3, x hpoint) int {
	return proof.DvDot(proof.DvCross(proof.DvSub(w, u), x.from(u)), normal).Sign()
}
