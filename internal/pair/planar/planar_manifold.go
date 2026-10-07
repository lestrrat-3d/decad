package planar

import (
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/pair"
	"github.com/lestrrat-3d/decad/internal/proof"
)

// This file builds the contact manifold of two exact planar solids
// (docs/multibody-dynamics-design.md §9.3) from the zero-distance feature
// pairs ClassifyPlanar recorded. The face-pair patch is clipped in
// planar_patch.go; shallow penetration is handled in
// planar_penetration_manifold.go.
//
// A touch manifold is the union of pieces, each a set of contact points that
// shares one normal. X is a convex solid (§9.2) and Y the other one:
//
//   - face-face: a face F of X coplanar with and opposed to a face G of Y.
//     The piece is F clipped against G (§9.4); a zero-area clip publishes the
//     ends and isolated points of the degenerate intersection instead.
//   - support: a face H of one solid whose plane the other solid (the guest)
//     lies wholly in front of and touches along an edge or at a vertex. The
//     guest's part of the plane is that edge or vertex, since a convex guest
//     meets a supporting plane in one of its faces; the piece is that part
//     inside H's closed region. Every component must reach H's interior,
//     which makes H's plane the one admissible normal there (contact-geometry
//     §3's cone rule); a guest feature that only grazes H's boundary is a
//     vertex-on-edge or edge-on-edge contact and covers nothing. Y is a
//     guest only when Y is convex as well.
//   - crossing: a crease edge of X crossing a crease edge of Y at one interior
//     point; the normal is the cross product of the two edge directions,
//     oriented by both fans' material sides.
//
// Every contact feature pair ClassifyPlanar recorded must be covered by an
// accepted piece holding one of its faces (or, for a crossing, by the
// crossing itself). Those feature pairs cover the whole contact set, so the
// published union is the complete certified contact set; any uncovered pair
// withholds the manifold with AmbiguousFeature.

// PatchFeature names one side's feature of a manifold entry by the face ids
// of PlanarSolid.Faces: a facet names its one face, an edge the two faces
// that meet along it, a vertex every face that holds it. Faces ascend.
type PatchFeature struct {
	Kind  FeatureKind
	Faces []int
}

// PatchPoint is one manifold entry: exact witnesses on A and B, both
// features, the exact A-to-B normal direction (not unit length) and the
// signed B-minus-A separation along it (zero at a touch).
type PatchPoint struct {
	OnA, OnB   Point3
	A, B       PatchFeature
	Normal     proof.DyV3
	Separation pair.ScalarReading
}

// PlanarManifold is a published manifold, or a Reason when it is withheld.
// Supports names the host face plane of every accepted face-face and support
// piece, in the order the pieces were accepted, each once: the planes whose
// support set (PlanarSupportSets) a positive band adds to the manifold.
type PlanarManifold struct {
	Points   []PatchPoint
	Reason   pair.Reason
	Supports []SupportPlane
}

// SupportPlane names a face of one solid of a pair, the HOST, whose plane may
// support the other solid, the GUEST (docs/multibody-dynamics-design.md
// §10.5).
type SupportPlane struct {
	HostIsA bool
	Face    int
}

// patchSide is one solid with the adjacency the manifold reads.
type patchSide struct {
	prep     *planarPrep
	faces    []int
	faceTris map[int][]int
	vertTris [][]int
	edgeTris map[[2]int][]int
	flat     map[int]bool
	convex   bool
	isA      bool
}

func newPatchSide(s *PlanarSolid, convex, isA bool) *patchSide {
	return newPreparedPatchSide(preparePlanar(s), convex, isA)
}

// newPreparedPatchSide builds the adjacency over derived data already built.
func newPreparedPatchSide(prep *planarPrep, convex, isA bool) *patchSide {
	s := prep.s
	p := &patchSide{prep: prep, faces: s.Faces, faceTris: make(map[int][]int),
		vertTris: make([][]int, len(s.Verts)), edgeTris: make(map[[2]int][]int),
		flat: make(map[int]bool), convex: convex, isA: isA}
	for t, tri := range s.Tris {
		p.faceTris[s.Faces[t]] = append(p.faceTris[s.Faces[t]], t)
		for i := range 3 {
			u, w := tri[i], tri[(i+1)%3]
			p.vertTris[u] = append(p.vertTris[u], t)
			key := [2]int{min(u, w), max(u, w)}
			p.edgeTris[key] = append(p.edgeTris[key], t)
		}
	}
	return p
}

// faceIDs returns the ascending distinct faces of the given triangles.
func (p *patchSide) faceIDs(tris []int) []int {
	var out []int
	for _, t := range tris {
		if !slices.Contains(out, p.faces[t]) {
			out = append(out, p.faces[t])
		}
	}
	slices.Sort(out)
	return out
}

// facesOf returns the faces of every triangle holding a recorded feature.
func (p *patchSide) facesOf(f PlanarFeature) []int {
	switch f.Kind {
	case FeatureFacet:
		return p.faceIDs([]int{f.Facet})
	case FeatureEdge:
		return p.faceIDs(p.edgeTris[f.Edge])
	case FeatureVertex:
		return p.faceIDs(p.vertTris[f.Vertex])
	}
	return nil
}

func (p *patchSide) faceNormal(f int) proof.DyV3 { return p.prep.normal[p.faceTris[f][0]] }

func (p *patchSide) faceOrigin(f int) proof.DyV3 {
	return p.prep.s.Verts[p.prep.s.Tris[p.faceTris[f][0]][0]]
}

// isFlat reports whether every triangle of a face lies on one plane with one
// outward normal direction, which a face of an admitted solid always does.
func (p *patchSide) isFlat(f int) bool {
	if flat, ok := p.flat[f]; ok {
		return flat
	}
	n, o := p.faceNormal(f), p.faceOrigin(f)
	flat := true
	for _, t := range p.faceTris[f] {
		if !proof.DvIsZero(proof.DvCross(n, p.prep.normal[t])) || proof.DvDot(n, p.prep.normal[t]).Sign() <= 0 {
			flat = false
		}
		for _, v := range p.prep.s.Tris[t] {
			if proof.DvDot(n, proof.DvSub(p.prep.s.Verts[v], o)).Sign() != 0 {
				flat = false
			}
		}
	}
	p.flat[f] = flat
	return flat
}

// faceLoops chains a face's boundary edges (directed edges of its triangles
// whose reverse is not one of them) into loops, each wound counterclockwise
// around the face's outward normal for its outer boundary and clockwise for a
// hole. A vertex with two outgoing boundary edges is a pinch and refused.
func (p *patchSide) faceLoops(f int) ([][]int, bool) {
	directed := make(map[[2]int]struct{})
	for _, t := range p.faceTris[f] {
		tri := p.prep.s.Tris[t]
		for i := range 3 {
			directed[[2]int{tri[i], tri[(i+1)%3]}] = struct{}{}
		}
	}
	next := make(map[int]int)
	for edge := range directed {
		if _, ok := directed[[2]int{edge[1], edge[0]}]; ok {
			continue
		}
		if _, ok := next[edge[0]]; ok {
			return nil, false
		}
		next[edge[0]] = edge[1]
	}
	starts := make([]int, 0, len(next))
	for v := range next {
		starts = append(starts, v)
	}
	slices.Sort(starts)
	visited := make(map[int]struct{}, len(next))
	var loops [][]int
	for _, start := range starts {
		if _, ok := visited[start]; ok {
			continue
		}
		var loop []int
		for v := start; ; {
			if _, ok := visited[v]; ok {
				return nil, false
			}
			visited[v] = struct{}{}
			loop = append(loop, v)
			w, ok := next[v]
			if !ok {
				return nil, false
			}
			if v = w; v == start {
				break
			}
		}
		loops = append(loops, loop)
	}
	return loops, len(loops) > 0
}

// frameLoops projects a face's loops, each vertex moved by shift, into a
// frame and splits them into the one outer loop and its holes by the sign of
// their projected area against the face normal's sign along the dropped axis.
func (p *patchSide) frameLoops(f int, frame PlaneFrame, shift Point3) ([]Point2, [][]Point2, bool) {
	loops, ok := p.faceLoops(f)
	if !ok {
		return nil, nil, false
	}
	along := p.faceNormal(f)[frame.K].Sign()
	var outer []Point2
	var holes [][]Point2
	for _, loop := range loops {
		projected := make([]Point2, len(loop))
		for i, v := range loop {
			at := ratPoint3(p.prep.s.Verts[v])
			if shift[0] != nil {
				for axis := range 3 {
					at[axis] = new(big.Rat).Add(at[axis], shift[axis])
				}
			}
			projected[i] = frame.Project(at)
		}
		switch DoubleArea(projected).Sign() * along {
		case 1:
			if outer != nil {
				return nil, nil, false
			}
			outer = projected
		case -1:
			holes = append(holes, projected)
		default:
			return nil, nil, false
		}
	}
	return outer, holes, outer != nil
}

// piece is one computed candidate: accepted pieces add their points once.
type piece struct {
	ok       bool
	withhold bool
}

type pieceKey struct {
	kind        int
	host, guest int
	hostIsX     bool
}

const (
	pieceFaceFace = iota
	pieceSupport
	pieceCrossing
)

type patchKernel struct {
	x, y     *patchSide
	poll     func() error
	pieces   map[pieceKey]piece
	points   []PatchPoint
	supports []SupportPlane
}

// addSupport records a host plane of an accepted piece once.
func (k *patchKernel) addSupport(plane SupportPlane) {
	if !slices.Contains(k.supports, plane) {
		k.supports = append(k.supports, plane)
	}
}

// PlanarTouchManifold builds the manifold of a Touching pair from the
// contacts ClassifyPlanar recorded. Both snapshots must carry Faces, and at
// least one solid must be convex. poll is charged throughout; its error is
// returned unchanged.
func PlanarTouchManifold(a, b *PlanarSolid, contacts []PlanarContact, convexA, convexB bool,
	poll func() error) (PlanarManifold, error) {
	if len(a.Faces) != len(a.Tris) || len(b.Faces) != len(b.Tris) || (!convexA && !convexB) ||
		len(contacts) == 0 {
		return PlanarManifold{Reason: pair.AmbiguousFeature}, nil
	}
	sa, sb := newPatchSide(a, convexA, true), newPatchSide(b, convexB, false)
	x, y := sa, sb
	if !convexA {
		x, y = sb, sa
	}
	k := &patchKernel{x: x, y: y, poll: poll, pieces: make(map[pieceKey]piece)}
	for _, contact := range contacts {
		fx, fy := contact.A, contact.B
		if !x.isA {
			fx, fy = contact.B, contact.A
		}
		covered, err := k.cover(fx, fy)
		if err != nil {
			return PlanarManifold{}, err
		}
		if !covered {
			return PlanarManifold{Reason: pair.AmbiguousFeature}, nil
		}
	}
	return PlanarManifold{Points: k.points, Supports: k.supports}, nil
}

// PlanarGuestTouch builds the manifold of a Touching pair neither of whose
// solids is convex (docs/multibody-dynamics-design.md §10.5). Each solid in
// turn, B first, is the HOST and the other the GUEST. A flat host face h
// supports the guest when every guest vertex lies on or in front of its plane
// and every recorded contact's host feature lies in h (a facet of h, or an
// edge or vertex one of whose faces is h), which puts the whole contact set
// on h's closed region. Its contact set is every guest vertex at zero height
// whose point lies strictly inside h, holes excluded: an exact touching
// vertex whose one admissible normal is h's, since the guest lies in its
// vertices' hull and so wholly in front of the plane. Neither step reads
// convexity. A guest face overhanging h publishes only its vertices over h;
// the clipped crossing points §9.4 would add need a convex side and are not
// attempted. The manifold is every supporting face's contact set, each with
// its face's outward normal oriented A to B, and Supports names those faces.
// A nil Points result withholds it: no host face holds every contact, or none
// holds a guest vertex strictly inside it. poll is charged once per guest
// vertex per candidate face.
func PlanarGuestTouch(a, b *PlanarSolid, contacts []PlanarContact, poll func() error) (PlanarManifold, error) {
	if len(a.Faces) != len(a.Tris) || len(b.Faces) != len(b.Tris) || len(contacts) == 0 {
		return PlanarManifold{Reason: pair.AmbiguousFeature}, nil
	}
	sa, sb := newPatchSide(a, false, true), newPatchSide(b, false, false)
	var out PlanarManifold
	for _, sides := range [][2]*patchSide{{sb, sa}, {sa, sb}} {
		host, guest := sides[0], sides[1]
		// The candidate faces hold every contact's host feature.
		var common []int
		for i, contact := range contacts {
			feature := contact.B
			if host.isA {
				feature = contact.A
			}
			faces := host.facesOf(feature)
			if i == 0 {
				common = faces
				continue
			}
			common = slices.DeleteFunc(common, func(f int) bool { return !slices.Contains(faces, f) })
		}
		for _, h := range common {
			points, err := guestContactSet(host, h, guest, poll)
			if err != nil {
				return PlanarManifold{}, err
			}
			if len(points) == 0 {
				continue
			}
			out.Points = append(out.Points, points...)
			out.Supports = append(out.Supports, SupportPlane{HostIsA: host.isA, Face: h})
		}
	}
	if out.Points == nil {
		return PlanarManifold{Reason: pair.AmbiguousFeature}, nil
	}
	return out, nil
}

// guestContactSet returns the guest vertices on host face h's plane whose
// points lie strictly inside h, or nil when h is not flat or some guest vertex
// lies behind its plane.
func guestContactSet(host *patchSide, h int, guest *patchSide, poll func() error) ([]PatchPoint, error) {
	if !host.isFlat(h) {
		return nil, nil
	}
	n, o := host.faceNormal(h), host.faceOrigin(h)
	var on []int
	for v, at := range guest.prep.s.Verts {
		if err := poll(); err != nil {
			return nil, err
		}
		switch proof.DvDot(n, proof.DvSub(at, o)).Sign() {
		case -1:
			return nil, nil
		case 0:
			on = append(on, v)
		}
	}
	if len(on) == 0 {
		return nil, nil
	}
	frame := NewPlaneFrame(n, o)
	outer, holes, ok := host.frameLoops(h, frame, Point3{})
	if !ok {
		return nil, nil
	}
	region := append([][]Point2{outer}, holes...)
	hostFeature := PatchFeature{Kind: FeatureFacet, Faces: []int{h}}
	var points []PatchPoint
	for _, v := range on {
		at := ratPoint3(guest.prep.s.Verts[v])
		if locate(frame.Project(at), region) <= 0 {
			continue
		}
		feature := PatchFeature{Kind: FeatureVertex, Faces: guest.faceIDs(guest.vertTris[v])}
		points = append(points, entry(at, host, hostFeature, feature, n))
	}
	return points, nil
}

// cover finds an accepted piece holding the contact, computing candidates in
// a fixed order: a coplanar face pair, a face of Y supporting X, a face of X
// supporting a convex Y, a crease crossing.
func (k *patchKernel) cover(fx, fy PlanarFeature) (bool, error) {
	facesX, facesY := k.x.facesOf(fx), k.y.facesOf(fy)
	try := func(key pieceKey) (bool, bool, error) {
		got, ok := k.pieces[key]
		if !ok {
			var err error
			got, err = k.compute(key, fx, fy)
			if err != nil {
				return false, false, err
			}
			k.pieces[key] = got
		}
		return got.ok, got.withhold, nil
	}
	var keys []pieceKey
	for _, f := range facesX {
		for _, g := range facesY {
			keys = append(keys, pieceKey{kind: pieceFaceFace, host: g, guest: f})
		}
	}
	for _, g := range facesY {
		keys = append(keys, pieceKey{kind: pieceSupport, host: g})
	}
	if k.y.convex {
		for _, f := range facesX {
			keys = append(keys, pieceKey{kind: pieceSupport, host: f, hostIsX: true})
		}
	}
	if fx.Kind == FeatureEdge && fy.Kind == FeatureEdge {
		keys = append(keys, pieceKey{kind: pieceCrossing,
			host: k.x.edgeIndex(fx.Edge), guest: k.y.edgeIndex(fy.Edge)})
	}
	for _, key := range keys {
		ok, withhold, err := try(key)
		if err != nil || withhold {
			return false, err
		}
		if ok {
			return true, nil
		}
	}
	return false, nil
}

func (p *patchSide) edgeIndex(edge [2]int) int {
	return slices.Index(p.prep.edges, edge)
}

func (k *patchKernel) compute(key pieceKey, fx, fy PlanarFeature) (piece, error) {
	var points []PatchPoint
	var result piece
	var err error
	switch key.kind {
	case pieceFaceFace:
		points, result, err = k.faceFace(key.guest, key.host)
	case pieceSupport:
		host, guest := k.y, k.x
		if key.hostIsX {
			host, guest = k.x, k.y
		}
		points, result, err = k.support(host, key.host, guest)
	default:
		points, result = k.crossing(fx.Edge, fy.Edge)
	}
	if err != nil {
		return piece{}, err
	}
	if result.ok {
		k.points = append(k.points, points...)
		switch key.kind {
		case pieceFaceFace:
			// Both coplanar faces are planes the other solid may rest on.
			k.addSupport(SupportPlane{HostIsA: k.y.isA, Face: key.host})
			k.addSupport(SupportPlane{HostIsA: k.x.isA, Face: key.guest})
		case pieceSupport:
			host := k.y
			if key.hostIsX {
				host = k.x
			}
			k.addSupport(SupportPlane{HostIsA: host.isA, Face: key.host})
		}
	}
	return result, nil
}

// entry orients one point's features and normal from host/guest to A/B.
// hostN is the host face's outward normal, which points from the host toward
// the guest.
func entry(at Point3, host *patchSide, hostFeature, guestFeature PatchFeature, hostN proof.DyV3) PatchPoint {
	point := PatchPoint{OnA: at, OnB: at, A: hostFeature, B: guestFeature, Normal: hostN}
	if !host.isA {
		point.A, point.B, point.Normal = guestFeature, hostFeature, dvNeg(hostN)
	}
	return point
}

// faceFace clips face f of X against face g of Y when they are coplanar and
// opposed. A hole of g whose clip has positive area means material is missing
// inside the patch, which withholds the whole manifold.
func (k *patchKernel) faceFace(f, g int) ([]PatchPoint, piece, error) {
	if !k.x.isFlat(f) || !k.y.isFlat(g) {
		return nil, piece{}, nil
	}
	nF, nG := k.x.faceNormal(f), k.y.faceNormal(g)
	if !proof.DvIsZero(proof.DvCross(nF, nG)) || proof.DvDot(nF, nG).Sign() >= 0 ||
		proof.DvDot(nG, proof.DvSub(k.x.faceOrigin(f), k.y.faceOrigin(g))).Sign() != 0 {
		return nil, piece{}, nil
	}
	frame := NewPlaneFrame(nG, k.y.faceOrigin(g))
	clipOuter, clipHoles, okF := k.x.frameLoops(f, frame, Point3{})
	outer, holes, okG := k.y.frameLoops(g, frame, Point3{})
	if !okF || !okG || len(clipHoles) > 0 {
		return nil, piece{withhold: true}, nil
	}
	clip := clipOuter
	if !IsConvex(clip) {
		return nil, piece{withhold: true}, nil
	}
	polygon, err := ClipConvex(outer, clip, k.poll)
	if err != nil {
		return nil, piece{}, err
	}
	var corners []Point2
	if len(polygon) >= 3 && DoubleArea(polygon).Sign() != 0 {
		for _, hole := range holes {
			missing, err := ClipConvex(hole, clip, k.poll)
			if err != nil {
				return nil, piece{}, err
			}
			if len(missing) >= 3 && DoubleArea(missing).Sign() != 0 {
				return nil, piece{withhold: true}, nil
			}
		}
		corners = extremalVertices(polygon)
	} else {
		corners, err = degenerateContact(clip, append([][]Point2{outer}, holes...), k.poll)
		if err != nil {
			return nil, piece{}, err
		}
	}
	if len(corners) == 0 {
		return nil, piece{}, nil
	}
	hostFeature := PatchFeature{Kind: FeatureFacet, Faces: []int{g}}
	guestFeature := PatchFeature{Kind: FeatureFacet, Faces: []int{f}}
	points := make([]PatchPoint, 0, len(corners))
	for _, c := range corners {
		points = append(points, entry(frame.Lift(c), k.y, hostFeature, guestFeature, nG))
	}
	return points, piece{ok: true}, nil
}

// support publishes the guest's edge or vertex on the plane of host face h
// when the guest lies wholly in front of that plane and touches it there.
func (k *patchKernel) support(host *patchSide, h int, guest *patchSide) ([]PatchPoint, piece, error) {
	if !host.isFlat(h) {
		return nil, piece{}, nil
	}
	n, o := host.faceNormal(h), host.faceOrigin(h)
	verts := guest.prep.s.Verts
	on := make(map[int]struct{})
	var onList []int
	for v, at := range verts {
		if err := k.poll(); err != nil {
			return nil, piece{}, err
		}
		switch proof.DvDot(n, proof.DvSub(at, o)).Sign() {
		case -1:
			return nil, piece{}, nil
		case 0:
			on[v] = struct{}{}
			onList = append(onList, v)
		}
	}
	if len(onList) == 0 {
		return nil, piece{}, nil
	}
	for _, tri := range guest.prep.s.Tris {
		_, a := on[tri[0]]
		_, b := on[tri[1]]
		_, c := on[tri[2]]
		if a && b && c {
			return nil, piece{}, nil
		}
	}
	frame := NewPlaneFrame(n, o)
	outer, holes, ok := host.frameLoops(h, frame, Point3{})
	if !ok {
		return nil, piece{}, nil
	}
	region := append([][]Point2{outer}, holes...)
	hostFeature := PatchFeature{Kind: FeatureFacet, Faces: []int{h}}
	if len(onList) == 1 {
		at := ratPoint3(verts[onList[0]])
		if locate(frame.Project(at), region) <= 0 {
			return nil, piece{}, nil
		}
		feature := PatchFeature{Kind: FeatureVertex, Faces: guest.faceIDs(guest.vertTris[onList[0]])}
		return []PatchPoint{entry(at, host, hostFeature, feature, n)}, piece{ok: true}, nil
	}
	// A convex guest meets a supporting plane in a face; with no triangle on
	// the plane that face is one edge, so the on-plane vertices are collinear.
	p := verts[onList[0]]
	d := proof.DvSub(verts[onList[1]], p)
	lo, hi := onList[0], onList[0]
	for _, v := range onList {
		rel := proof.DvSub(verts[v], p)
		if !proof.DvIsZero(proof.DvCross(rel, d)) {
			return nil, piece{}, nil
		}
		if proof.DyCmp(proof.DvDot(rel, d), proof.DvDot(proof.DvSub(verts[lo], p), d)) < 0 {
			lo = v
		}
		if proof.DyCmp(proof.DvDot(rel, d), proof.DvDot(proof.DvSub(verts[hi], p), d)) > 0 {
			hi = v
		}
	}
	start, end := ratPoint3(verts[lo]), ratPoint3(verts[hi])
	spans, err := segmentSpans(frame.Project(start), frame.Project(end), region, k.poll)
	if err != nil || len(spans) == 0 {
		return nil, piece{}, err
	}
	mid := hpoint{x: proof.DvAdd(verts[lo], verts[hi]), w: proof.DyInt(2)}
	var holding []int
	for t := range guest.prep.s.Tris {
		if pointInFacet(guest.prep, t, mid) {
			holding = append(holding, t)
		}
	}
	feature := PatchFeature{Kind: FeatureEdge, Faces: guest.faceIDs(holding)}
	if len(feature.Faces) != 2 {
		return nil, piece{}, nil
	}
	var points []PatchPoint
	for _, s := range spans {
		if !s.interior {
			return nil, piece{}, nil
		}
		for _, t := range []*big.Rat{s.lo, s.hi} {
			points = append(points, entry(lerp3(start, end, t), host, hostFeature, feature, n))
		}
	}
	return points, piece{ok: true}, nil
}

func lerp3(p, q Point3, t *big.Rat) Point3 {
	var out Point3
	for axis := range 3 {
		out[axis] = new(big.Rat).Add(p[axis], new(big.Rat).Mul(t, new(big.Rat).Sub(q[axis], p[axis])))
	}
	return out
}

// crossing publishes the single point where a crease edge of X crosses a
// crease edge of Y, with the normal of their common plane oriented so X's fan
// lies on or behind it and Y's on or in front.
func (k *patchKernel) crossing(ex, ey [2]int) ([]PatchPoint, piece) {
	crease := func(side *patchSide, edge [2]int) bool {
		tris := side.edgeTris[edge]
		return len(tris) == 2 && !proof.DvIsZero(proof.DvCross(side.prep.normal[tris[0]], side.prep.normal[tris[1]])) &&
			len(side.faceIDs(tris)) == 2
	}
	if !crease(k.x, ex) || !crease(k.y, ey) {
		return nil, piece{}
	}
	xv, yv := k.x.prep.s.Verts, k.y.prep.s.Verts
	p0, q0 := xv[ex[0]], yv[ey[0]]
	d1, d2 := proof.DvSub(xv[ex[1]], p0), proof.DvSub(yv[ey[1]], q0)
	n := proof.DvCross(d1, d2)
	r := proof.DvSub(p0, q0)
	if proof.DvIsZero(n) || proof.DvDot(r, n).Sign() != 0 {
		return nil, piece{}
	}
	b, c := proof.DvDot(d1, d2), proof.DvDot(d2, d2)
	den := proof.DvDot(n, n)
	s := proof.DySubScalar(proof.DyMul(b, proof.DvDot(d2, r)), proof.DyMul(c, proof.DvDot(d1, r)))
	at := hpoint{x: proof.DvAdd(dvScale(p0, den), dvScale(d1, s)), w: den}
	fanSign := func(side *patchSide, edge [2]int, normal proof.DyV3, want int) bool {
		for _, t := range side.edgeTris[edge] {
			for _, v := range side.prep.s.Tris[t] {
				if proof.DvDot(normal, at.relative(side.prep.s.Verts[v])).Sign()*want < 0 {
					return false
				}
			}
		}
		return true
	}
	switch {
	case fanSign(k.x, ex, n, -1) && fanSign(k.y, ey, n, 1):
	case fanSign(k.x, ex, dvNeg(n), -1) && fanSign(k.y, ey, dvNeg(n), 1):
		n = dvNeg(n)
	default:
		return nil, piece{}
	}
	var point Point3
	for axis := range 3 {
		point[axis] = new(big.Rat).Quo(at.x[axis].Rat(), den.Rat())
	}
	featureX := PatchFeature{Kind: FeatureEdge, Faces: k.x.faceIDs(k.x.edgeTris[ex])}
	featureY := PatchFeature{Kind: FeatureEdge, Faces: k.y.faceIDs(k.y.edgeTris[ey])}
	// n points from X toward Y: X plays the host whose normal leaves it.
	return []PatchPoint{entry(point, k.x, featureX, featureY, n)}, piece{ok: true}
}
