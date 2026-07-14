package decad

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
)

// This file is the mesh-boolean pipeline of docs/evaluator-design.md §9: the
// two operands' tessellations meet in exact triangle/triangle intersection
// segments, each cut facet is subdivided along them (boolean_cut.go), every
// resulting piece is classified against the other solid by an exact local
// side-of-contact test (or exact ray parity for the uncut regions), the kept
// pieces are stitched by shared exact vertices, and a conforming pass plus a
// closure audit prove the result watertight by construction — an audit
// failure is an error, never a wrong mesh.

// boolMesh is one operand's tessellation prepared for exact work: float
// vertices lifted to rationals, exact facet normals, facet boxes for pair
// pruning, and the global source-face id each facet remembers.
type boolMesh struct {
	verts  []r3.Vec
	xverts []xpt
	tris   [][3]int
	norms  []xpt
	boxes  [][2]r3.Vec
	src    []int
	// degen marks facets with an exactly zero normal — the float rounding
	// of a previous boolean can collapse a sliver. A degenerate facet has
	// no plane and no interior: it takes part in no contact and rides with
	// its component's classification.
	degen []bool
}

// prepBoolMesh lifts a tessellation into the exact domain. A non-finite
// vertex has no exact form, so it is rejected outright.
func prepBoolMesh(m *Mesh, src []int) (*boolMesh, error) {
	bm := &boolMesh{verts: m.vertices, tris: m.triangles, src: src}
	bm.xverts = make([]xpt, len(m.vertices))
	for i, v := range m.vertices {
		if isNonFinite(v.X) || isNonFinite(v.Y) || isNonFinite(v.Z) {
			return nil, fmt.Errorf(`%w: a mesh vertex is not finite`, ErrBooleanFailed)
		}
		bm.xverts[i] = xptOf(v)
	}
	bm.norms = make([]xpt, len(m.triangles))
	bm.boxes = make([][2]r3.Vec, len(m.triangles))
	bm.degen = make([]bool, len(m.triangles))
	for i, tri := range m.triangles {
		a, b, c := bm.xverts[tri[0]], bm.xverts[tri[1]], bm.xverts[tri[2]]
		n := xcross(xsub(b, a), xsub(c, a))
		bm.degen[i] = n.x.Sign() == 0 && n.y.Sign() == 0 && n.z.Sign() == 0
		bm.norms[i] = n
		bm.boxes[i] = triBox(bm.verts, tri)
	}
	return bm, nil
}

func isNonFinite(f float64) bool { return math.IsNaN(f) || math.IsInf(f, 0) }

// triBox is the facet's float bounding box — float min/max are exact, so the
// box is a true bound.
func triBox(verts []r3.Vec, tri [3]int) [2]r3.Vec {
	lo, hi := verts[tri[0]], verts[tri[0]]
	for _, vi := range tri[1:] {
		v := verts[vi]
		lo = r3.Vec{X: math.Min(lo.X, v.X), Y: math.Min(lo.Y, v.Y), Z: math.Min(lo.Z, v.Z)}
		hi = r3.Vec{X: math.Max(hi.X, v.X), Y: math.Max(hi.Y, v.Y), Z: math.Max(hi.Z, v.Z)}
	}
	return [2]r3.Vec{lo, hi}
}

func boxesOverlap(a, b [2]r3.Vec) bool {
	return a[0].X <= b[1].X && b[0].X <= a[1].X &&
		a[0].Y <= b[1].Y && b[0].Y <= a[1].Y &&
		a[0].Z <= b[1].Z && b[0].Z <= a[1].Z
}

// keptFacet is one facet of the boolean result, still exact: its corners,
// the global source-face id it approximates, and which operand it came from.
type keptFacet struct {
	v   [3]xpt
	src int
}

// errDegenerateContact wraps ErrDegenerate for the contacts the exact
// predicates refuse to classify: coplanar face-on-face overlap, an edge
// grazing along the other operand's facet plane, and intersection curves
// branching at a point. A tangent contact admits no side, so no facet
// classification is proven — the operation is rejected, never a wrong mesh.
func errDegenerateContact(what string) error {
	return fmt.Errorf(`%w: %s — the exact predicates cannot classify a tangent contact`, ErrDegenerate, what)
}

// triContact is one exact triangle/triangle intersection: a segment with,
// per endpoint, whether it lies on each triangle's own boundary.
type triContact struct {
	p0, p1                     xpt
	p0OnA, p1OnA, p0OnB, p1OnB bool
	// pointOnly marks a contact that is exactly one point: carried to the
	// mesh pass, which rejects it unless it is the endpoint of some
	// segment contact (a crossing chain passing through a vertex).
	pointOnly bool
}

// planeCrossings collects the exact points where triangle t crosses the
// other triangle's plane, given the per-vertex signs and the exact
// plane-side values (computed lazily, only for crossing edges).
func planeCrossings(xt [3]xpt, xo [3]xpt, signs [3]int) []xpt {
	var out []xpt
	vals := [3]*big.Rat{}
	val := func(i int) *big.Rat {
		if vals[i] == nil {
			vals[i] = orientVal(xo[0], xo[1], xo[2], xt[i])
		}
		return vals[i]
	}
	for i := range 3 {
		if signs[i] == 0 {
			out = append(out, xt[i])
		}
		j := (i + 1) % 3
		if signs[i]*signs[j] < 0 {
			// t = vi/(vi − vj) is in (0, 1): the edge crosses the plane.
			t := new(big.Rat).Quo(val(i), new(big.Rat).Sub(val(i), val(j)))
			out = append(out, xlerp(xt[i], xt[j], t))
		}
	}
	return out
}

// triTriContact computes the exact intersection of two non-coplanar-safe
// triangles: a proper crossing segment, nothing (disjoint or a point touch),
// or a degenerate-contact rejection. ta/tb are float corners, xta/xtb their
// exact lifts, na/nb the exact normals.
func triTriContact(ta, tb [3]r3.Vec, xta, xtb [3]xpt, na, nb xpt) (triContact, bool, error) {
	var sb, sa [3]int
	for i := range 3 {
		sb[i] = orientSign(ta[0], ta[1], ta[2], tb[i])
	}
	if sb[0] > 0 && sb[1] > 0 && sb[2] > 0 || sb[0] < 0 && sb[1] < 0 && sb[2] < 0 {
		return triContact{}, false, nil
	}
	for i := range 3 {
		sa[i] = orientSign(tb[0], tb[1], tb[2], ta[i])
	}
	if sa[0] > 0 && sa[1] > 0 && sa[2] > 0 || sa[0] < 0 && sa[1] < 0 && sa[2] < 0 {
		return triContact{}, false, nil
	}

	za := countZero(sa)
	zb := countZero(sb)
	// pointTouch reports the first vertex of xt (whose plane-side sign is
	// zero at index i) that lies on the closed other triangle — a carried
	// point contact candidate.
	pointTouch := func(signs [3]int, xt, xo [3]xpt, no xpt) (xpt, bool) {
		for i := range 3 {
			if signs[i] == 0 && pointOnTri(xt[i], xo, no) {
				return xt[i], true
			}
		}
		return xpt{}, false
	}
	if za == 3 || zb == 3 {
		// Coplanar pair: an overlap of positive measure is a face-on-face
		// tangency; a vertex touching the other closed triangle is a point
		// contact, carried for the isolation check.
		if coplanarOverlap(xta, xtb, na) {
			return triContact{}, false, errDegenerateContact(`two operand facets overlap in one plane`)
		}
		if p, ok := pointTouch([3]int{0, 0, 0}, xta, xtb, nb); ok {
			return triContact{p0: p, p1: p, pointOnly: true}, true, nil
		}
		if p, ok := pointTouch([3]int{0, 0, 0}, xtb, xta, na); ok {
			return triContact{p0: p, p1: p, pointOnly: true}, true, nil
		}
		return triContact{}, false, nil
	}
	if za == 2 {
		if err := rejectEdgeGraze(sa, xta, xtb, nb); err != nil {
			return triContact{}, false, err
		}
		if p, ok := pointTouch(sa, xta, xtb, nb); ok {
			return triContact{p0: p, p1: p, pointOnly: true}, true, nil
		}
		return triContact{}, false, nil
	}
	if zb == 2 {
		if err := rejectEdgeGraze(sb, xtb, xta, na); err != nil {
			return triContact{}, false, err
		}
		if p, ok := pointTouch(sb, xtb, xta, na); ok {
			return triContact{p0: p, p1: p, pointOnly: true}, true, nil
		}
		return triContact{}, false, nil
	}
	if za == 1 && (sa[0]+sa[1]+sa[2] != 0) && zb == 1 && (sb[0]+sb[1]+sb[2] != 0) {
		// Both triangles only touch the other's plane at one vertex from
		// one side: the contact is at most a point — carried when the
		// vertex really lies on the other closed triangle.
		if p, ok := pointTouch(sa, xta, xtb, nb); ok {
			return triContact{p0: p, p1: p, pointOnly: true}, true, nil
		}
		if p, ok := pointTouch(sb, xtb, xta, na); ok {
			return triContact{p0: p, p1: p, pointOnly: true}, true, nil
		}
		return triContact{}, false, nil
	}

	ptsA := planeCrossings(xta, xtb, sa)
	ptsB := planeCrossings(xtb, xta, sb)
	ptsA = dedupePoints(ptsA)
	ptsB = dedupePoints(ptsB)
	if len(ptsA) < 2 || len(ptsB) < 2 {
		// A single crossing point on a side is at most a point contact —
		// carried when it really lies on the other closed triangle.
		if len(ptsA) == 1 && pointOnTri(ptsA[0], xtb, nb) {
			return triContact{p0: ptsA[0], p1: ptsA[0], pointOnly: true}, true, nil
		}
		if len(ptsB) == 1 && pointOnTri(ptsB[0], xta, na) {
			return triContact{p0: ptsB[0], p1: ptsB[0], pointOnly: true}, true, nil
		}
		return triContact{}, false, nil
	}
	if len(ptsA) > 2 || len(ptsB) > 2 {
		return triContact{}, false, fmt.Errorf(`%w: a facet crosses a plane more than twice`, ErrBooleanFailed)
	}

	// Both crossings lie on the planes' common line; order them along it.
	dir := xcross(na, nb)
	sOf := func(p xpt) *big.Rat { return xdot(p, dir) }
	a0, a1 := ptsA[0], ptsA[1]
	sa0, sa1 := sOf(a0), sOf(a1)
	if sa0.Cmp(sa1) > 0 {
		a0, a1, sa0, sa1 = a1, a0, sa1, sa0
	}
	b0, b1 := ptsB[0], ptsB[1]
	sb0, sb1 := sOf(b0), sOf(b1)
	if sb0.Cmp(sb1) > 0 {
		b0, b1, sb0, sb1 = b1, b0, sb1, sb0
	}
	// The contact is the overlap of the two crossing intervals. An endpoint
	// taken from ptsA lies on ta's own boundary (a vertex or an edge
	// crossing), and symmetrically for ptsB; a tie is a point on both.
	lo, loS, loA, loB := a0, sa0, true, false
	switch sb0.Cmp(sa0) {
	case 1:
		lo, loS, loA, loB = b0, sb0, false, true
	case 0:
		loB = true
	}
	hi, hiS, hiA, hiB := a1, sa1, true, false
	switch sb1.Cmp(sa1) {
	case -1:
		hi, hiS, hiA, hiB = b1, sb1, false, true
	case 0:
		hiB = true
	}
	if c := loS.Cmp(hiS); c > 0 {
		return triContact{}, false, nil // empty overlap
	} else if c == 0 {
		// A zero-length overlap is an edge-edge point touch: carried, so
		// the isolated-point rejection decides it like any point contact.
		return triContact{p0: lo, p1: lo, pointOnly: true}, true, nil
	}
	return triContact{
		p0: lo, p1: hi,
		p0OnA: loA, p1OnA: hiA,
		p0OnB: loB, p1OnB: hiB,
	}, true, nil
}

func countZero(s [3]int) int {
	n := 0
	for _, v := range s {
		if v == 0 {
			n++
		}
	}
	return n
}

// dedupePoints removes exactly-coincident points.
func dedupePoints(pts []xpt) []xpt {
	var out []xpt
	seen := map[string]struct{}{}
	for _, p := range pts {
		k := p.key()
		if _, ok := seen[k]; ok {
			continue
		}
		seen[k] = struct{}{}
		out = append(out, p)
	}
	return out
}

// rejectEdgeGraze handles the two-zero case: an edge of triangle t lies in
// the other triangle's plane. A positive-length overlap with the closed other
// triangle is a grazing tangent contact — unclassifiable — while a miss or a
// point touch is no contact.
func rejectEdgeGraze(signs [3]int, xt, xo [3]xpt, no xpt) error {
	var onPlane []xpt
	for i := range 3 {
		if signs[i] == 0 {
			onPlane = append(onPlane, xt[i])
		}
	}
	if len(onPlane) != 2 {
		return nil
	}
	u, v := projAxes(no)
	a := xp2{ratCoordOf(onPlane[0], u), ratCoordOf(onPlane[0], v)}
	b := xp2{ratCoordOf(onPlane[1], u), ratCoordOf(onPlane[1], v)}
	ta := xp2{ratCoordOf(xo[0], u), ratCoordOf(xo[0], v)}
	tb := xp2{ratCoordOf(xo[1], u), ratCoordOf(xo[1], v)}
	tc := xp2{ratCoordOf(xo[2], u), ratCoordOf(xo[2], v)}
	if segTriOverlap2(a, b, ta, tb, tc) {
		return errDegenerateContact(`an operand edge grazes along the other operand's facet`)
	}
	return nil
}

// projAxes picks the two projection coordinates for a plane with exact
// normal n: the dominant-magnitude axis is dropped, so the projection is
// invertible on the plane.
func projAxes(n xpt) (int, int) {
	ax := new(big.Rat).Abs(n.x)
	ay := new(big.Rat).Abs(n.y)
	az := new(big.Rat).Abs(n.z)
	if az.Cmp(ax) >= 0 && az.Cmp(ay) >= 0 {
		return 0, 1
	}
	if ay.Cmp(ax) >= 0 {
		return 2, 0
	}
	return 1, 2
}

// xcoordOf returns p's exact coordinate along axis i (0=x, 1=y, 2=z).
func xcoordOf(p xpt, i int) *big.Rat {
	switch i {
	case 0:
		return p.x
	case 1:
		return p.y
	default:
		return p.z
	}
}

// pointOnTri reports whether the exact point p — already on the triangle's
// plane — lies inside or on the closed triangle, via the exact projection.
func pointOnTri(p xpt, xt [3]xpt, n xpt) bool {
	u, v := projAxes(n)
	pp := xp2{xcoordOf(p, u), xcoordOf(p, v)}
	a := xp2{xcoordOf(xt[0], u), xcoordOf(xt[0], v)}
	b := xp2{xcoordOf(xt[1], u), xcoordOf(xt[1], v)}
	c := xp2{xcoordOf(xt[2], u), xcoordOf(xt[2], v)}
	return pointInTriX(pp, a, b, c)
}

// segTriOverlap2 reports whether segment (a, b) meets the closed triangle
// with positive length — exactly.
func segTriOverlap2(a, b, ta, tb, tc xp2) bool {
	// Clip the segment parameter interval [0, 1] against each closed edge
	// half-plane of the triangle, in exact arithmetic; a positive-length
	// remainder is an overlap.
	ccw := cross2x(ta, tb, tc).Sign()
	if ccw == 0 {
		return false
	}
	edges := [3][2]xp2{{ta, tb}, {tb, tc}, {tc, ta}}
	lo, hi := new(big.Rat), new(big.Rat).SetInt64(1)
	for _, e := range edges {
		fa := cross2x(e[0], e[1], a)
		fb := cross2x(e[0], e[1], b)
		if ccw < 0 {
			fa.Neg(fa)
			fb.Neg(fb)
		}
		// f(t) = fa + t·(fb − fa) must stay ≥ 0.
		diff := new(big.Rat).Sub(fb, fa)
		switch {
		case fa.Sign() >= 0 && fb.Sign() >= 0:
			continue
		case fa.Sign() < 0 && fb.Sign() < 0:
			return false
		default:
			t := new(big.Rat).Quo(new(big.Rat).Neg(fa), diff)
			if fa.Sign() < 0 {
				if t.Cmp(lo) > 0 {
					lo = t
				}
			} else {
				if t.Cmp(hi) < 0 {
					hi = t
				}
			}
		}
	}
	return lo.Cmp(hi) < 0
}

// coplanarOverlap reports whether two coplanar triangles share positive
// area or a positive-length boundary segment — exactly.
func coplanarOverlap(xta, xtb [3]xpt, n xpt) bool {
	u, v := projAxes(n)
	var a2, b2 [3]xp2
	for i := range 3 {
		a2[i] = xp2{ratCoordOf(xta[i], u), ratCoordOf(xta[i], v)}
		b2[i] = xp2{ratCoordOf(xtb[i], u), ratCoordOf(xtb[i], v)}
	}
	// Any edge of one meeting the closed other with positive length is an
	// overlap; this covers containment (all edges inside), proper crossings,
	// and collinear boundary contact alike.
	for i := range 3 {
		if segTriOverlap2(a2[i], a2[(i+1)%3], b2[0], b2[1], b2[2]) {
			return true
		}
		if segTriOverlap2(b2[i], b2[(i+1)%3], a2[0], a2[1], a2[2]) {
			return true
		}
	}
	return false
}

// booleanKeep is the classification table of §9: which side of the other
// solid each operand's boundary keeps, and whether the kept B side flips
// orientation (a cut turns the tool's skin inside out).
func booleanKeep(op OpKind) (bool, bool, bool, error) {
	switch op {
	case OpUnion:
		return false, false, false, nil
	case OpIntersect:
		return true, true, false, nil
	case OpCut:
		return false, true, true, nil
	default:
		return false, false, false, fmt.Errorf(`%w: %q is not a boolean op`, ErrBooleanFailed, op)
	}
}

// meshBoolean runs the exact-predicate boolean over two prepared operand
// tessellations and returns the kept, still-exact facets.
func meshBoolean(op OpKind, ma, mb *boolMesh) ([]keptFacet, error) {
	wantA, wantB, flipB, err := booleanKeep(op)
	if err != nil {
		return nil, err
	}

	// Exact contacts, per facet of each operand. Facet boxes prune the pairs.
	cutsA := map[int][]xseg{}
	cutsB := map[int][]xseg{}
	var pointTouches []xpt
	for i := range ma.tris {
		if ma.degen[i] {
			continue
		}
		for j := range mb.tris {
			if mb.degen[j] || !boxesOverlap(ma.boxes[i], mb.boxes[j]) {
				continue
			}
			ta := triCorners(ma, i)
			tb := triCorners(mb, j)
			contact, ok, err := triTriContact(ta, tb, xtriCorners(ma, i), xtriCorners(mb, j), ma.norms[i], mb.norms[j])
			if err != nil {
				return nil, err
			}
			if !ok {
				continue
			}
			if contact.pointOnly {
				pointTouches = append(pointTouches, contact.p0)
				continue
			}
			cutsA[i] = append(cutsA[i], xseg{
				a: contact.p0, b: contact.p1,
				aOnEdge: contact.p0OnA, bOnEdge: contact.p1OnA,
				partner: tb,
			})
			cutsB[j] = append(cutsB[j], xseg{
				a: contact.p0, b: contact.p1,
				aOnEdge: contact.p0OnB, bOnEdge: contact.p1OnB,
				partner: ta,
			})
		}
	}

	// A point-only contact is legitimate only as the endpoint of some
	// crossing segment (a chain passing exactly through a vertex). An
	// isolated one is a tangency the boolean cannot classify: the operands
	// pinch at a point, and stitching it would emit a non-manifold result —
	// refuse, never a wrong mesh.
	if len(pointTouches) > 0 {
		ends := map[string]struct{}{}
		for _, segs := range [2]map[int][]xseg{cutsA, cutsB} {
			for _, ss := range segs {
				for _, s := range ss {
					ends[s.a.key()] = struct{}{}
					ends[s.b.key()] = struct{}{}
				}
			}
		}
		for _, p := range pointTouches {
			if _, ok := ends[p.key()]; !ok {
				return nil, errDegenerateContact(`the operand boundaries touch at an isolated point`)
			}
		}
	}

	var kept []keptFacet
	keep, err := keepSide(ma, mb, cutsA, wantA, false)
	if err != nil {
		return nil, err
	}
	kept = append(kept, keep...)
	keep, err = keepSide(mb, ma, cutsB, wantB, flipB)
	if err != nil {
		return nil, err
	}
	return append(kept, keep...), nil
}

func triCorners(m *boolMesh, i int) [3]r3.Vec {
	t := m.tris[i]
	return [3]r3.Vec{m.verts[t[0]], m.verts[t[1]], m.verts[t[2]]}
}

func xtriCorners(m *boolMesh, i int) [3]xpt {
	t := m.tris[i]
	return [3]xpt{m.xverts[t[0]], m.xverts[t[1]], m.xverts[t[2]]}
}

// keepSide classifies one operand's boundary against the other solid and
// returns the kept facets: subdivided pieces for the cut facets, whole
// facets for the uncut regions (classified per connected component by exact
// ray parity — the classification is constant on a component that crosses
// nothing).
func keepSide(m, other *boolMesh, cuts map[int][]xseg, wantInside, flip bool) ([]keptFacet, error) {
	var kept []keptFacet
	emit := func(tri [3]xpt, src int) {
		if flip {
			tri[1], tri[2] = tri[2], tri[1]
		}
		kept = append(kept, keptFacet{v: tri, src: src})
	}

	// The cut facets: exact subdivision along the contact chains, then the
	// exact local side-of-contact classification per region.
	for i := range m.tris {
		segs, ok := cuts[i]
		if !ok {
			continue
		}
		regions, err := cutTriangle(xtriCorners(m, i), m.norms[i], segs)
		if err != nil {
			return nil, err
		}
		for _, reg := range regions {
			inside, err := classifyRegion(reg, other)
			if err != nil {
				return nil, err
			}
			if inside != wantInside {
				continue
			}
			for _, tri := range reg.tris {
				emit(tri, m.src[i])
			}
		}
	}

	// The uncut facets: constant classification per connected uncut
	// component, one exact parity seed each.
	comp := make([]int, len(m.tris))
	for i := range comp {
		comp[i] = -1
	}
	adj := facetAdjacency(m.tris)
	next := 0
	all := allFacets(other)
	for i := range m.tris {
		if _, cut := cuts[i]; cut || comp[i] != -1 {
			continue
		}
		id := next
		next++
		queue := []int{i}
		comp[i] = id
		var members []int
		for len(queue) > 0 {
			f := queue[0]
			queue = queue[1:]
			members = append(members, f)
			for _, nb := range adj[f] {
				if _, cut := cuts[nb]; cut || comp[nb] != -1 {
					continue
				}
				comp[nb] = id
				queue = append(queue, nb)
			}
		}
		// Seed the parity probe on a non-degenerate member: a collapsed
		// sliver has no interior to probe.
		seedIdx := -1
		for _, f := range members {
			if !m.degen[f] {
				seedIdx = f
				break
			}
		}
		if seedIdx == -1 {
			return nil, fmt.Errorf(`%w: an uncut component holds only collapsed facets`, ErrBooleanFailed)
		}
		seed := xtriCorners(m, seedIdx)
		probe := xCentroid(seed[0], seed[1], seed[2])
		inside, onBoundary, err := meshParity(probe, other.verts, other.tris, all)
		if err != nil {
			return nil, err
		}
		if onBoundary {
			return nil, errDegenerateContact(`an operand facet touches the other operand's boundary`)
		}
		if inside != wantInside {
			continue
		}
		for _, f := range members {
			emit(xtriCorners(m, f), m.src[f])
		}
	}
	return kept, nil
}

// classifyRegion decides whether a subdivision region lies inside the other
// solid. A region anchored to a contact chain is decided by the exact side
// of the partner facet's plane — the other solid's boundary IS that facet
// along the shared chain edge, so the side is the answer; an unanchored
// region (an artifact of the loop-opening split lines) falls back to exact
// parity.
func classifyRegion(reg cutRegion, other *boolMesh) (bool, error) {
	if reg.hasAnchor {
		switch s := orientSignMixed(reg.partner[0], reg.partner[1], reg.partner[2], reg.probe); {
		case s < 0:
			return true, nil
		case s > 0:
			return false, nil
		default:
			return false, fmt.Errorf(`%w: a region probe landed on its contact plane`, ErrBooleanFailed)
		}
	}
	inside, onBoundary, err := meshParity(reg.probe, other.verts, other.tris, allFacets(other))
	if err != nil {
		return false, err
	}
	if onBoundary {
		return false, errDegenerateContact(`a subdivision region touches the other operand's boundary`)
	}
	return inside, nil
}

func allFacets(m *boolMesh) []int {
	out := make([]int, len(m.tris))
	for i := range out {
		out[i] = i
	}
	return out
}

// facetAdjacency maps each facet to its edge neighbors, through the
// watertight mesh's paired directed edges.
func facetAdjacency(tris [][3]int) [][]int {
	owner := map[[2]int]int{}
	for i, tri := range tris {
		for k := range 3 {
			owner[[2]int{tri[k], tri[(k+1)%3]}] = i
		}
	}
	adj := make([][]int, len(tris))
	for i, tri := range tris {
		for k := range 3 {
			if twin, ok := owner[[2]int{tri[(k+1)%3], tri[k]}]; ok && twin != i {
				adj[i] = append(adj[i], twin)
			}
		}
	}
	return adj
}

func xCentroid(a, b, c xpt) xpt {
	third := big.NewRat(1, 3)
	s := xpt{
		new(big.Rat).Add(new(big.Rat).Add(a.x, b.x), c.x),
		new(big.Rat).Add(new(big.Rat).Add(a.y, b.y), c.y),
		new(big.Rat).Add(new(big.Rat).Add(a.z, b.z), c.z),
	}
	return xpt{
		new(big.Rat).Mul(s.x, third),
		new(big.Rat).Mul(s.y, third),
		new(big.Rat).Mul(s.z, third),
	}
}

// stitchedMesh is the boolean output after welding, conforming and rounding:
// the held float mesh, its per-facet source-face ids, and the exact bound on
// the final rounding.
type stitchedMesh struct {
	verts []r3.Vec
	tris  [][3]int
	src   []int
	round float64
}

// stitchFacets welds the kept facets by shared exact vertices, makes the
// subdivision conforming (a vertex lying exactly on another facet's edge
// splits that facet — exact incidence, so nothing moves), audits closure,
// and rounds to float64. The audit is the §9 guarantee: every directed edge
// pairs with its reverse, or the boolean fails — never a cracked mesh.
func stitchFacets(kept []keptFacet) (*stitchedMesh, error) {
	if len(kept) == 0 {
		return nil, fmt.Errorf(`%w: the operation leaves no boundary at all`, ErrBooleanFailed)
	}
	var xverts []xpt
	index := map[string]int{}
	addVert := func(p xpt) int {
		k := p.key()
		if i, ok := index[k]; ok {
			return i
		}
		index[k] = len(xverts)
		xverts = append(xverts, p)
		return len(xverts) - 1
	}
	var tris [][3]int
	var src []int
	for _, f := range kept {
		a, b, c := addVert(f.v[0]), addVert(f.v[1]), addVert(f.v[2])
		if a == b || b == c || c == a {
			return nil, fmt.Errorf(`%w: a kept facet collapsed`, ErrBooleanFailed)
		}
		tris = append(tris, [3]int{a, b, c})
		src = append(src, f.src)
	}

	for round := 0; ; round++ {
		if round > 6 {
			return nil, fmt.Errorf(`%w: the conforming pass did not converge`, ErrBooleanFailed)
		}
		split, err := conformOnce(&xverts, &tris, &src)
		if err != nil {
			return nil, err
		}
		if !split {
			break
		}
	}

	// Closure audit: each directed edge exactly once, each with its reverse.
	directed := map[[2]int]int{}
	for _, tri := range tris {
		for k := range 3 {
			directed[[2]int{tri[k], tri[(k+1)%3]}]++
		}
	}
	for e, n := range directed {
		if n != 1 || directed[[2]int{e[1], e[0]}] != 1 {
			return nil, fmt.Errorf(`%w: the stitched boundary does not close`, ErrBooleanFailed)
		}
	}

	// Round to float64, welding vertices whose roundings coincide — two
	// exact points closer than an ulp become one held vertex — and drop the
	// facets that collapse under the weld: a collapsed facet's two real
	// directed edges cancel each other, so closure survives, and the final
	// audit re-proves it.
	out := &stitchedMesh{}
	floatIdx := map[r3.Vec]int{}
	remap := make([]int, len(xverts))
	worst := new(big.Rat)
	for i, p := range xverts {
		v := p.vec()
		fi, ok := floatIdx[v]
		if !ok {
			fi = len(out.verts)
			floatIdx[v] = fi
			out.verts = append(out.verts, v)
		}
		remap[i] = fi
		d := new(big.Rat).Sub(p.x, ratOf(v.X))
		d.Abs(d)
		for _, pair := range [][2]*big.Rat{{p.y, ratOf(v.Y)}, {p.z, ratOf(v.Z)}} {
			dd := new(big.Rat).Sub(pair[0], pair[1])
			dd.Abs(dd)
			if dd.Cmp(d) > 0 {
				d = dd
			}
		}
		if d.Cmp(worst) > 0 {
			worst = d
		}
	}
	for ti, tri := range tris {
		a, b, c := remap[tri[0]], remap[tri[1]], remap[tri[2]]
		if a == b || b == c || c == a {
			continue
		}
		out.tris = append(out.tris, [3]int{a, b, c})
		out.src = append(out.src, src[ti])
	}
	if len(out.tris) == 0 {
		return nil, fmt.Errorf(`%w: the whole result collapsed under rounding`, ErrBooleanFailed)
	}
	directed = map[[2]int]int{}
	for _, tri := range out.tris {
		for k := range 3 {
			directed[[2]int{tri[k], tri[(k+1)%3]}]++
		}
	}
	for e, n := range directed {
		if n != 1 || directed[[2]int{e[1], e[0]}] != 1 {
			return nil, fmt.Errorf(`%w: the rounded boundary does not close`, ErrBooleanFailed)
		}
	}
	w, _ := worst.Float64()
	if w > 0 {
		// worst is the max PER-COORDINATE rounding; the consumers read a 3D
		// distance bound, and all three coordinates can round at once, so
		// scale by √3 (rounded up) before the ulp nudge.
		w *= 1.7320508075688774
		w = math.Nextafter(w, math.Inf(1))
	}
	out.round = w
	return out, nil
}

// conformOnce inserts, into every facet edge, the mesh vertices that lie
// exactly in that edge's interior, re-triangulating the facet so the
// subdivision conforms. Returns whether anything split.
func conformOnce(xverts *[]xpt, tris *[][3]int, src *[]int) (bool, error) {
	verts := *xverts
	// A coarse uniform grid over float approximations prunes the exact
	// on-edge tests; the slack absorbs the approximation, so no incidence is
	// missed.
	lo, hi := r3.Vec{}, r3.Vec{}
	for i, p := range verts {
		v := p.vec()
		if i == 0 {
			lo, hi = v, v
			continue
		}
		lo = r3.Vec{X: math.Min(lo.X, v.X), Y: math.Min(lo.Y, v.Y), Z: math.Min(lo.Z, v.Z)}
		hi = r3.Vec{X: math.Max(hi.X, v.X), Y: math.Max(hi.Y, v.Y), Z: math.Max(hi.Z, v.Z)}
	}
	diag := hi.Sub(lo).Len()
	if diag == 0 {
		return false, fmt.Errorf(`%w: the stitched boundary has no extent`, ErrBooleanFailed)
	}
	cell := diag / 64
	slack := diag*1e-9 + cell*1e-9
	cellOf := func(x, y, z float64) [3]int {
		return [3]int{int(math.Floor((x - lo.X) / cell)), int(math.Floor((y - lo.Y) / cell)), int(math.Floor((z - lo.Z) / cell))}
	}
	grid := map[[3]int][]int{}
	approx := make([]r3.Vec, len(verts))
	for i, p := range verts {
		approx[i] = p.vec()
		c := cellOf(approx[i].X, approx[i].Y, approx[i].Z)
		grid[c] = append(grid[c], i)
	}

	var outTris [][3]int
	var outSrc []int
	splitAny := false
	for ti, tri := range *tris {
		inserted := [3][]int{}
		for k := range 3 {
			a, b := tri[k], tri[(k+1)%3]
			pa, pb := approx[a], approx[b]
			cLo := cellOf(math.Min(pa.X, pb.X)-slack, math.Min(pa.Y, pb.Y)-slack, math.Min(pa.Z, pb.Z)-slack)
			cHi := cellOf(math.Max(pa.X, pb.X)+slack, math.Max(pa.Y, pb.Y)+slack, math.Max(pa.Z, pb.Z)+slack)
			var hits []int
			for cx := cLo[0]; cx <= cHi[0]; cx++ {
				for cy := cLo[1]; cy <= cHi[1]; cy++ {
					for cz := cLo[2]; cz <= cHi[2]; cz++ {
						for _, vi := range grid[[3]int{cx, cy, cz}] {
							if vi == tri[0] || vi == tri[1] || vi == tri[2] {
								continue
							}
							if onSegmentInterior3(verts[a], verts[b], verts[vi]) {
								hits = append(hits, vi)
							}
						}
					}
				}
			}
			if len(hits) > 0 {
				sortAlongEdge(verts, a, b, hits)
				inserted[k] = hits
			}
		}
		if inserted[0] == nil && inserted[1] == nil && inserted[2] == nil {
			outTris = append(outTris, tri)
			outSrc = append(outSrc, (*src)[ti])
			continue
		}
		splitAny = true
		poly := []int{tri[0]}
		poly = append(poly, inserted[0]...)
		poly = append(poly, tri[1])
		poly = append(poly, inserted[1]...)
		poly = append(poly, tri[2])
		poly = append(poly, inserted[2]...)
		newTris, err := triangulatePlanarPolygon(verts, poly)
		if err != nil {
			return false, err
		}
		for _, nt := range newTris {
			outTris = append(outTris, nt)
			outSrc = append(outSrc, (*src)[ti])
		}
	}
	*tris = outTris
	*src = outSrc
	return splitAny, nil
}

// onSegmentInterior3 reports, exactly, whether p lies strictly inside the 3D
// segment (a, b).
func onSegmentInterior3(a, b, p xpt) bool {
	d := xsub(b, a)
	ap := xsub(p, a)
	cr := xcross(d, ap)
	if cr.x.Sign() != 0 || cr.y.Sign() != 0 || cr.z.Sign() != 0 {
		return false
	}
	axis := dominantAxis(d)
	da := ratCoordOf(d, axis)
	if da.Sign() == 0 {
		return false
	}
	t := new(big.Rat).Quo(ratCoordOf(ap, axis), da)
	return t.Sign() > 0 && t.Cmp(big.NewRat(1, 1)) < 0
}

func dominantAxis(d xpt) int {
	ax := new(big.Rat).Abs(d.x)
	ay := new(big.Rat).Abs(d.y)
	az := new(big.Rat).Abs(d.z)
	if ax.Cmp(ay) >= 0 && ax.Cmp(az) >= 0 {
		return 0
	}
	if ay.Cmp(az) >= 0 {
		return 1
	}
	return 2
}

// sortAlongEdge orders the inserted vertices by their exact parameter along
// (a, b).
func sortAlongEdge(verts []xpt, a, b int, hits []int) {
	d := xsub(verts[b], verts[a])
	axis := dominantAxis(d)
	da := ratCoordOf(d, axis)
	param := func(vi int) *big.Rat {
		return new(big.Rat).Quo(ratCoordOf(xsub(verts[vi], verts[a]), axis), da)
	}
	for i := 1; i < len(hits); i++ {
		for j := i; j > 0 && param(hits[j]).Cmp(param(hits[j-1])) < 0; j-- {
			hits[j], hits[j-1] = hits[j-1], hits[j]
		}
	}
}

// triangulatePlanarPolygon triangulates a planar polygon of mesh vertices
// (a facet boundary with collinear insertions) by exact ear clipping in the
// polygon's own plane, preserving orientation and using every vertex.
func triangulatePlanarPolygon(verts []xpt, poly []int) ([][3]int, error) {
	if len(poly) < 3 {
		return nil, fmt.Errorf(`%w: a conforming polygon lost its corners`, ErrBooleanFailed)
	}
	// The polygon is a triangle with edge insertions: its normal is the
	// original facet's, recoverable from any strict corner.
	n := xpt{new(big.Rat), new(big.Rat), new(big.Rat)}
	found := false
	for i := 1; i+1 < len(poly); i++ {
		cand := xcross(xsub(verts[poly[i]], verts[poly[0]]), xsub(verts[poly[i+1]], verts[poly[0]]))
		if cand.x.Sign() != 0 || cand.y.Sign() != 0 || cand.z.Sign() != 0 {
			n = cand
			found = true
			break
		}
	}
	if !found {
		return nil, fmt.Errorf(`%w: a conforming polygon is degenerate`, ErrBooleanFailed)
	}
	u, v := projAxes(n)
	pts := make([]xp2, len(poly))
	idx := make([]int, len(poly))
	for i, vi := range poly {
		pts[i] = xp2{ratCoordOf(verts[vi], u), ratCoordOf(verts[vi], v)}
		idx[i] = i
	}
	// Keep the projected orientation counter-clockwise so ear clipping and
	// the emitted winding agree with the facet's own.
	flip := polyArea2(pts).Sign() < 0
	if flip {
		for i, j := 0, len(idx)-1; i < j; i, j = i+1, j-1 {
			idx[i], idx[j] = idx[j], idx[i]
		}
	}
	tris2, err := earClipX(pts, idx)
	if err != nil {
		return nil, err
	}
	out := make([][3]int, 0, len(tris2))
	for _, t := range tris2 {
		a, b, c := poly[t[0]], poly[t[1]], poly[t[2]]
		if flip {
			b, c = c, b
		}
		out = append(out, [3]int{a, b, c})
	}
	return out, nil
}
