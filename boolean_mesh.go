package decad

import (
	"context"
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
	// owner maps each directed mesh edge to the facet that walks it, so the
	// twin across a facet edge is one lookup. The graze-or-crossing call
	// needs it: whether an in-plane edge grazes the other operand or crosses
	// it is a property of the edge's TWO adjacent facets, which no facet pair
	// can see (docs/evaluator-design.md §9).
	owner map[[2]int]int
}

// twinFacet returns the facet across edge k of facet f — the neighbour a
// watertight mesh's reversed directed edge names.
func (bm *boolMesh) twinFacet(f, k int) (int, bool) {
	tri := bm.tris[f]
	t, ok := bm.owner[[2]int{tri[(k+1)%3], tri[k]}]
	return t, ok && t != f
}

// prepBoolMesh lifts a tessellation into the exact domain. A non-finite
// vertex has no exact form, so it is rejected outright.
//
// A COLLAPSED operand facet — one whose exact normal is zero, which a rigid
// placement's own rounding can produce on an already-faceted body — is refused
// (ErrUnsupported). A collapsed facet has no plane and no interior, so every
// contact predicate in this file is blind to it: a point or tangent contact the
// other operand makes THERE would be classified by nothing at all, and the
// facet would ride silently through the boolean with its component's verdict.
// The contact is exactly what this pipeline must not miss, so the operand is
// refused rather than partly examined. Loud beats silently wrong.
func prepBoolMeshContext(ctx context.Context, m *Mesh, src []int) (*boolMesh, error) {
	bm := &boolMesh{verts: m.vertices, tris: m.triangles, src: src}
	bm.xverts = make([]xpt, len(m.vertices))
	for i, v := range m.vertices {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		if isNonFinite(v.X) || isNonFinite(v.Y) || isNonFinite(v.Z) {
			return nil, fmt.Errorf(`%w: a mesh vertex is not finite`, ErrBooleanFailed)
		}
		bm.xverts[i] = xptOf(v)
	}
	bm.norms = make([]xpt, len(m.triangles))
	bm.boxes = make([][2]r3.Vec, len(m.triangles))
	bm.owner = make(map[[2]int]int, 3*len(m.triangles))
	for i, tri := range m.triangles {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		a, b, c := bm.xverts[tri[0]], bm.xverts[tri[1]], bm.xverts[tri[2]]
		n := xcross(xsub(b, a), xsub(c, a))
		if n.x.Sign() == 0 && n.y.Sign() == 0 && n.z.Sign() == 0 {
			return nil, fmt.Errorf(`%w: an operand holds a collapsed facet, which carries no plane and no interior — a contact made on it could not be classified at all, so this evaluator refuses the operand rather than examine it in part`, ErrUnsupported)
		}
		bm.norms[i] = n
		bm.boxes[i] = triBox(bm.verts, tri)
		for k := range 3 {
			bm.owner[[2]int{tri[k], tri[(k+1)%3]}] = i
		}
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

// errUnclassifiableContact signals a VALID-but-unclassifiable contact the exact
// predicates refuse to classify: coplanar face-on-face overlap, an edge grazing
// along the other operand's facet plane, and intersection curves branching at a
// point. A tangent contact admits no side, so no facet classification is proven
// — the operation is rejected, never a wrong mesh. The input names a real solid
// and the refusal is the evaluator's reach, so it carries the booleanExpectedContact
// signal and wraps ErrUnsupported (never ErrDegenerate); the public boundary remaps
// it to BooleanUnsupportedContact (boolean.go, asBooleanError; docs/api-design.md
// §8 / H2, docs/evaluator-design.md §9).
func errUnclassifiableContact(what string) error {
	return expectedBoolean(booleanExpectedContact,
		fmt.Errorf(`%w: %s — the exact predicates cannot classify a tangent contact`, ErrUnsupported, what))
}

// contactKind is the DIMENSION of the exact intersection of two closed
// triangles. Two closed triangles are convex sets, so their intersection is
// convex and can only be one of these four — and naming it is what makes the
// classification direction-free: nothing in it depends on whose geometry the
// answer was looked for on.
type contactKind int

const (
	// contactNone: the closed triangles are disjoint.
	contactNone contactKind = iota
	// contactPoint: they meet at exactly one point.
	contactPoint
	// contactSegment: they meet along a positive-length segment.
	contactSegment
	// contactRegion: they meet in a 2-D region — only possible for a coplanar
	// pair, and a face-on-face tangency the predicates refuse to classify.
	contactRegion
)

// triContact is one exact triangle/triangle intersection, as computed by
// triTriClassify.
type triContact struct {
	kind   contactKind
	p0, p1 xpt
	// p0OnA … p1OnB record whether each endpoint lies on each triangle's own
	// closed BOUNDARY — decided by asking what the point is (onTriBoundary),
	// never by which triangle's crossing list it happened to be drawn from.
	p0OnA, p1OnA, p0OnB, p1OnB bool
	// edgeA and edgeB name the facet edge a SEGMENT contact runs ALONG (−1
	// when it runs along none): an in-plane edge. Whether such an edge grazes
	// the other operand or genuinely crosses it is NOT a property of this pair
	// — it is a property of the edge's two adjacent facets — so the pair only
	// reports the fact and the mesh pass decides (docs/evaluator-design.md §9).
	edgeA, edgeB int
	// sin2 is the exact squared sine of the two facet planes' crossing angle,
	// nil for a coplanar or empty pair. The boolean's own rim vertices are
	// displaced by (δA + δB)/sin θ, so the smallest one over the pair's
	// contacts is what bounds the result (bounds.go, rimDelta).
	sin2 *big.Rat
}

// contactMemo caches triTriClassify's answer per facet-index pair for ONE
// operand pair, so the second ask a call reaches — the tangency gate
// (facesNearMiss) and the mesh pass (meshBoolean) walk the same two prepared
// tessellations — is served from the store instead of recomputed.
// triTriClassify is a pure function of the two facets' float corners, exact
// lifts and exact normals, all read-only fields of ma/mb, so a stored answer
// for (i, j) stays correct for as long as ma and mb do — which is exactly one
// evaluateBoolean call, the memo's whole lifetime. Binding ma/mb into the
// memo itself, rather than taking them per call, means no ask names an
// operand, so a caller cannot key an answer for one operand pair and read it
// back for another.
//
// A returned triContact is shared: its p0/p1 endpoints and its sin2 are
// *big.Rat values a second caller reads, never a fresh copy. Every consumer
// today only reads them, so this is safe, but no consumer may mutate one in
// place. contactMemo is not safe for concurrent use; nothing today shares one
// across goroutines.
type contactMemo struct {
	ma, mb *boolMesh
	m      map[[2]int32]triContact
}

// newContactMemo builds a memo scoped to one evaluateBoolean call over ma/mb.
func newContactMemo(ma, mb *boolMesh) *contactMemo {
	return &contactMemo{ma: ma, mb: mb, m: map[[2]int32]triContact{}}
}

// classify returns the exact classification of facet i of ma against facet j
// of mb, computing it on the first ask and replaying the stored answer on
// every later one. An error is never stored, so a later ask still retries.
func (c *contactMemo) classify(i, j int) (triContact, error) {
	key := [2]int32{int32(i), int32(j)}
	if v, ok := c.m[key]; ok {
		return v, nil
	}
	v, err := triTriClassify(triCorners(c.ma, i), triCorners(c.mb, j), xtriCorners(c.ma, i), xtriCorners(c.mb, j), c.ma.norms[i], c.mb.norms[j])
	if err != nil {
		return triContact{}, err
	}
	c.m[key] = v
	return v, nil
}

// triTriClassify computes the exact intersection of two CLOSED triangles: the
// single symmetric entry point every contact question goes through. ta/tb are
// float corners (the adaptive orient filter reads them), xta/xtb their exact
// lifts, na/nb the exact normals.
//
// The pair's intersection is the overlap of two convex sets, so it is empty, a
// point, a segment or a 2-D region — and the routine decides WHICH without
// ever branching on "how many of A's vertices sit on B's plane, and whose
// geometry do I look on". For a non-coplanar pair the two planes meet in one
// line; each triangle's intersection with the OTHER's plane is an interval on
// that line, and the answer is the intervals' overlap. That is the whole rule.
func triTriClassify(ta, tb [3]r3.Vec, xta, xtb [3]xpt, na, nb xpt) (triContact, error) {
	out := triContact{edgeA: -1, edgeB: -1}
	var sb, sa [3]int
	for i := range 3 {
		sb[i] = orientSign(ta[0], ta[1], ta[2], tb[i])
	}
	if allOneSide(sb) {
		return out, nil
	}
	for i := range 3 {
		sa[i] = orientSign(tb[0], tb[1], tb[2], ta[i])
	}
	if allOneSide(sa) {
		return out, nil
	}

	if countZero(sa) == 3 || countZero(sb) == 3 {
		// Coplanar. The intersection of two coplanar closed triangles is a
		// convex polygon: positive area — or a positive-length shared boundary
		// — is a face-on-face tangency (§9 refuses it); anything else is at
		// most a point, carried for the isolated-point rule.
		if coplanarOverlap(xta, xtb, na) {
			out.kind = contactRegion
			return out, nil
		}
		if p, ok := coplanarTouch(xta, xtb, na, nb); ok {
			out.kind, out.p0, out.p1 = contactPoint, p, p
		}
		return out, nil
	}

	// Non-coplanar: the planes are distinct and non-parallel (a parallel pair
	// would leave every vertex strictly on one side, already returned), so they
	// meet in exactly one line, and every point of the intersection lies on it.
	ptsA := dedupePoints(planeCrossings(xta, xtb, sa))
	ptsB := dedupePoints(planeCrossings(xtb, xta, sb))
	if len(ptsA) == 0 || len(ptsB) == 0 {
		return out, nil
	}
	if len(ptsA) > 2 || len(ptsB) > 2 {
		return out, fmt.Errorf(`%w: a facet crosses a plane more than twice`, ErrBooleanFailed)
	}
	dir := xcross(na, nb)
	loA, hiA := orderOnLine(ptsA, dir)
	loB, hiB := orderOnLine(ptsB, dir)
	lo, hi := loA, hiA
	if cmpOnLine(loB, lo, dir) > 0 {
		lo = loB
	}
	if cmpOnLine(hiB, hi, dir) < 0 {
		hi = hiB
	}
	switch cmpOnLine(lo, hi, dir) {
	case 1:
		return out, nil // the intervals miss: no contact at all
	case 0:
		out.kind, out.p0, out.p1 = contactPoint, lo, lo
	default:
		out.kind, out.p0, out.p1 = contactSegment, lo, hi
	}
	out.sin2 = sinSquared(na, nb)
	out.p0OnA = onTriBoundary(out.p0, xta, na)
	out.p0OnB = onTriBoundary(out.p0, xtb, nb)
	if out.kind == contactSegment {
		out.p1OnA = onTriBoundary(out.p1, xta, na)
		out.p1OnB = onTriBoundary(out.p1, xtb, nb)
		out.edgeA = segAlongEdge(out.p0, out.p1, xta, na)
		out.edgeB = segAlongEdge(out.p0, out.p1, xtb, nb)
	} else {
		out.p1OnA, out.p1OnB = out.p0OnA, out.p0OnB
	}
	return out, nil
}

// allOneSide reports whether all three plane-side signs are strictly the same:
// the triangle misses the plane entirely.
func allOneSide(s [3]int) bool {
	return s[0] > 0 && s[1] > 0 && s[2] > 0 || s[0] < 0 && s[1] < 0 && s[2] < 0
}

// coplanarTouch returns the single point two coplanar, non-overlapping closed
// triangles share, looking BOTH ways — the symmetric shape the whole classifier
// is built on.
func coplanarTouch(xta, xtb [3]xpt, na, nb xpt) (xpt, bool) {
	for i := range 3 {
		if pointOnTri(xta[i], xtb, nb) {
			return xta[i], true
		}
	}
	for i := range 3 {
		if pointOnTri(xtb[i], xta, na) {
			return xtb[i], true
		}
	}
	return xpt{}, false
}

// orderOnLine sorts a triangle's one or two plane crossings along the planes'
// common line, giving the interval the triangle occupies on it.
func orderOnLine(pts []xpt, dir xpt) (xpt, xpt) {
	if len(pts) == 1 {
		return pts[0], pts[0]
	}
	if cmpOnLine(pts[0], pts[1], dir) > 0 {
		return pts[1], pts[0]
	}
	return pts[0], pts[1]
}

// cmpOnLine orders two points of the planes' common line along it.
func cmpOnLine(a, b, dir xpt) int { return xdot(a, dir).Cmp(xdot(b, dir)) }

// onTriBoundary reports whether the exact point p — already on the triangle's
// plane — lies on one of its three CLOSED edges. The projection is invertible
// on the plane, so the 2-D answer IS the 3-D one.
func onTriBoundary(p xpt, xt [3]xpt, n xpt) bool {
	u, v := projAxes(n)
	pp := xp2{ratCoordOf(p, u), ratCoordOf(p, v)}
	for i := range 3 {
		a := xp2{ratCoordOf(xt[i], u), ratCoordOf(xt[i], v)}
		b := xp2{ratCoordOf(xt[(i+1)%3], u), ratCoordOf(xt[(i+1)%3], v)}
		if on, _ := onSegment2(a, b, pp); on {
			return true
		}
	}
	return false
}

// segAlongEdge names the triangle edge a positive-length contact segment runs
// along, or −1. Such a segment lies in the other facet's plane, so the edge it
// runs along has both its endpoints on that plane — the in-plane edge whose
// graze-or-crossing verdict only the edge's two adjacent facets can give.
func segAlongEdge(p0, p1 xpt, xt [3]xpt, n xpt) int {
	u, v := projAxes(n)
	q0 := xp2{ratCoordOf(p0, u), ratCoordOf(p0, v)}
	q1 := xp2{ratCoordOf(p1, u), ratCoordOf(p1, v)}
	for i := range 3 {
		a := xp2{ratCoordOf(xt[i], u), ratCoordOf(xt[i], v)}
		b := xp2{ratCoordOf(xt[(i+1)%3], u), ratCoordOf(xt[(i+1)%3], v)}
		on0, _ := onSegment2(a, b, q0)
		if !on0 {
			continue
		}
		if on1, _ := onSegment2(a, b, q1); on1 {
			return i
		}
	}
	return -1
}

// sinSquared is the exact sin²θ of the angle between two facet planes:
// |na × nb|² / (|na|²·|nb|²).
func sinSquared(na, nb xpt) *big.Rat {
	c := xcross(na, nb)
	num := xdot(c, c)
	den := new(big.Rat).Mul(xdot(na, na), xdot(nb, nb))
	if den.Sign() == 0 {
		return new(big.Rat)
	}
	return num.Quo(num, den)
}

// sinLowerBound is a PROVEN lower bound on sin θ from its exact square: the
// float square root, nudged down twice so neither the rational's rounding nor
// the root's can push it above the truth. A lower bound on the sine is what
// makes (δA + δB)/sin θ an UPPER bound on the rim displacement.
func sinLowerBound(sin2 *big.Rat) float64 {
	f, _ := sin2.Float64()
	if f <= 0 {
		return 0
	}
	s := math.Sqrt(f)
	s = math.Nextafter(s, 0)
	return math.Nextafter(s, 0)
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
	ccw := cross2xSign(ta, tb, tc)
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

// pairContact is one classified facet pair, kept whole so the decisions the
// PAIR cannot make are made later with the mesh in hand.
type pairContact struct {
	i, j   int // the facet in ma, the facet in mb
	ta, tb [3]r3.Vec
	c      triContact
}

// meshBoolean runs the exact-predicate boolean over two prepared operand
// tessellations. It returns the kept, still-exact facets and a proven LOWER
// bound on the sine of the crossing angle of every contact it used — the
// number the rim's displacement bound divides by (bounds.go, rimDelta).
func meshBoolean(ctx context.Context, op OpKind, ma, mb *boolMesh, memo *contactMemo) ([]keptFacet, float64, error) {
	wantA, wantB, flipB, err := booleanKeep(op)
	if err != nil {
		return nil, 0, err
	}

	// Exact contacts, per facet of each operand. Facet boxes prune the pairs.
	cutsA := map[int][]xseg{}
	cutsB := map[int][]xseg{}
	var pointTouches []xpt
	var segEnds []xpt
	var inPlane []pairContact
	var minSin2 *big.Rat
	work := 0
	for i := range ma.tris {
		for j := range mb.tris {
			work++
			if work%256 == 0 {
				if err := ctx.Err(); err != nil {
					return nil, 0, err
				}
			}
			if !boxesOverlap(ma.boxes[i], mb.boxes[j]) {
				continue
			}
			ta := triCorners(ma, i)
			tb := triCorners(mb, j)
			c, err := memo.classify(i, j)
			if err != nil {
				return nil, 0, err
			}
			if c.kind == contactRegion {
				return nil, 0, errUnclassifiableContact(`two operand facets overlap in one plane`)
			}
			if c.kind == contactNone {
				continue
			}
			if c.sin2 != nil && (minSin2 == nil || c.sin2.Cmp(minSin2) < 0) {
				minSin2 = c.sin2
			}
			if c.kind == contactPoint {
				pointTouches = append(pointTouches, c.p0)
				continue
			}
			segEnds = append(segEnds, c.p0, c.p1)
			if c.edgeA >= 0 || c.edgeB >= 0 {
				// The segment runs ALONG a facet edge. Graze or crossing is not
				// decidable here — hold it for the mesh-level call below.
				inPlane = append(inPlane, pairContact{i: i, j: j, ta: ta, tb: tb, c: c})
				continue
			}
			cutsA[i] = append(cutsA[i], xseg{
				a: c.p0, b: c.p1,
				aOnEdge: c.p0OnA, bOnEdge: c.p1OnA,
				partner: tb,
			})
			cutsB[j] = append(cutsB[j], xseg{
				a: c.p0, b: c.p1,
				aOnEdge: c.p0OnB, bOnEdge: c.p1OnB,
				partner: ta,
			})
		}
	}

	// The graze-or-crossing call, made once, OUTSIDE the pair loop: an in-plane
	// facet edge grazes the other operand exactly when the edge's two adjacent
	// facets lie strictly on ONE side of the other facet's plane — the operand's
	// boundary touches the plane and comes back, a tangency no side
	// classification can be proven for. Apexes that STRADDLE the plane are an
	// ordinary crossing: the boundary genuinely passes through, and the segment
	// is a real rim of the result.
	blockedA := map[[2]int]struct{}{}
	blockedB := map[[2]int]struct{}{}
	for i, p := range inPlane {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, 0, err
			}
		}
		if p.c.edgeA >= 0 {
			key, crossing, err := edgeCrosses(ma, p.i, p.c.edgeA, p.tb)
			if err != nil {
				return nil, 0, err
			}
			if !crossing {
				return nil, 0, errUnclassifiableContact(`an operand edge grazes along the other operand's facet`)
			}
			blockedA[key] = struct{}{}
		}
		if p.c.edgeB >= 0 {
			key, crossing, err := edgeCrosses(mb, p.j, p.c.edgeB, p.ta)
			if err != nil {
				return nil, 0, err
			}
			if !crossing {
				return nil, 0, errUnclassifiableContact(`an operand edge grazes along the other operand's facet`)
			}
			blockedB[key] = struct{}{}
		}
		// A crossing segment subdivides whichever facet does NOT already own it
		// as an edge — a segment lying along a facet's own boundary cuts nothing
		// off it. Its regions classify by exact parity, never by the partner
		// facet's plane: the other operand's boundary there is the DIHEDRAL
		// between two facets, and one plane of it decides nothing.
		if p.c.edgeA < 0 {
			cutsA[p.i] = append(cutsA[p.i], xseg{
				a: p.c.p0, b: p.c.p1,
				aOnEdge: p.c.p0OnA, bOnEdge: p.c.p1OnA,
				partner: p.tb, viaParity: true,
			})
		}
		if p.c.edgeB < 0 {
			cutsB[p.j] = append(cutsB[p.j], xseg{
				a: p.c.p0, b: p.c.p1,
				aOnEdge: p.c.p0OnB, bOnEdge: p.c.p1OnB,
				partner: p.ta, viaParity: true,
			})
		}
	}

	// A point contact is legitimate only as the endpoint of some crossing
	// segment (a chain passing exactly through a vertex). An isolated one is a
	// tangency the boolean cannot classify: the operands pinch at a point, and
	// stitching it would emit a non-manifold result — refuse, never a wrong mesh.
	if len(pointTouches) > 0 {
		ends := map[string]struct{}{}
		for _, p := range segEnds {
			ends[p.key()] = struct{}{}
		}
		for _, p := range pointTouches {
			if _, ok := ends[p.key()]; !ok {
				return nil, 0, errUnclassifiableContact(`the operand boundaries touch at an isolated point`)
			}
		}
	}

	var kept []keptFacet
	if err := ctx.Err(); err != nil {
		return nil, 0, err
	}
	keep, err := keepSide(ctx, ma, mb, cutsA, blockedA, wantA, false)
	if err != nil {
		return nil, 0, err
	}
	kept = append(kept, keep...)
	keep, err = keepSide(ctx, mb, ma, cutsB, blockedB, wantB, flipB)
	if err != nil {
		return nil, 0, err
	}
	sinMin := 1.0
	if minSin2 != nil {
		sinMin = sinLowerBound(minSin2)
	}
	return append(kept, keep...), sinMin, nil
}

// edgeCrosses decides whether edge k of facet f — which lies exactly in the
// partner facet's plane — GRAZES that plane or genuinely CROSSES it, and
// returns the undirected mesh-edge key either way. The verdict is read off the
// edge's two adjacent facets: their apex vertices strictly on one side is a
// graze; straddling is a crossing. An apex exactly ON the plane proves nothing
// (the facet lies in it), so it reads as a graze — reject-only.
func edgeCrosses(m *boolMesh, f, k int, partner [3]r3.Vec) ([2]int, bool, error) {
	tri := m.tris[f]
	u, v := tri[k], tri[(k+1)%3]
	key := [2]int{min(u, v), max(u, v)}
	twin, ok := m.twinFacet(f, k)
	if !ok {
		return key, false, fmt.Errorf(`%w: an in-plane facet edge has no twin facet`, ErrBooleanFailed)
	}
	tt := m.tris[twin]
	apex := m.verts[tri[0]+tri[1]+tri[2]-u-v]
	apexTwin := m.verts[tt[0]+tt[1]+tt[2]-u-v]
	s0 := orientSign(partner[0], partner[1], partner[2], apex)
	s1 := orientSign(partner[0], partner[1], partner[2], apexTwin)
	return key, s0*s1 < 0, nil
}

// blockedBetween reports whether the mesh edge two adjacent facets share
// carries an in-plane CROSSING contact. The other operand's boundary passes
// exactly along that edge, so the two facets sit on opposite sides of it: they
// must not flood-fill into one classification, or one of them inherits the
// other's answer.
func blockedBetween(m *boolMesh, a, b int, blocked map[[2]int]struct{}) bool {
	if len(blocked) == 0 {
		return false
	}
	ta, tb := m.tris[a], m.tris[b]
	for k := range 3 {
		u, v := ta[k], ta[(k+1)%3]
		if _, ok := blocked[[2]int{min(u, v), max(u, v)}]; !ok {
			continue
		}
		if sharesVertices(tb, u, v) {
			return true
		}
	}
	return false
}

func sharesVertices(t [3]int, u, v int) bool {
	hasU, hasV := false, false
	for _, x := range t {
		switch x {
		case u:
			hasU = true
		case v:
			hasV = true
		}
	}
	return hasU && hasV
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
func keepSide(ctx context.Context, m, other *boolMesh, cuts map[int][]xseg, blocked map[[2]int]struct{}, wantInside, flip bool) ([]keptFacet, error) {
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
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		segs, ok := cuts[i]
		if !ok {
			continue
		}
		regions, err := cutTriangle(ctx, xtriCorners(m, i), m.norms[i], segs)
		if err != nil {
			return nil, err
		}
		for _, reg := range regions {
			inside, err := classifyRegion(ctx, reg, other)
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
	adj, err := facetAdjacencyContext(ctx, m.tris)
	if err != nil {
		return nil, err
	}
	next := 0
	componentWork := 0
	all := allFacets(other)
	for i := range m.tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		if _, cut := cuts[i]; cut || comp[i] != -1 {
			continue
		}
		id := next
		next++
		queue := []int{i}
		comp[i] = id
		var members []int
		for len(queue) > 0 {
			componentWork++
			if componentWork%256 == 0 {
				if err := ctx.Err(); err != nil {
					return nil, err
				}
			}
			f := queue[0]
			queue = queue[1:]
			members = append(members, f)
			for _, nb := range adj[f] {
				if _, cut := cuts[nb]; cut || comp[nb] != -1 {
					continue
				}
				if blockedBetween(m, f, nb, blocked) {
					continue
				}
				comp[nb] = id
				queue = append(queue, nb)
			}
		}
		// Every facet has a real interior to probe: prepBoolMesh refused the
		// operand outright if any facet collapsed.
		seed := xtriCorners(m, members[0])
		probe := xCentroid(seed[0], seed[1], seed[2])
		inside, onBoundary, err := meshParityContext(ctx, probe, other.verts, other.tris, all)
		if err != nil {
			return nil, err
		}
		if onBoundary {
			return nil, errUnclassifiableContact(`an operand facet touches the other operand's boundary`)
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
func classifyRegion(ctx context.Context, reg cutRegion, other *boolMesh) (bool, error) {
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
	inside, onBoundary, err := meshParityContext(ctx, reg.probe, other.verts, other.tris, allFacets(other))
	if err != nil {
		return false, err
	}
	if onBoundary {
		return false, errUnclassifiableContact(`a subdivision region touches the other operand's boundary`)
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

// facetAdjacencyContext maps each facet to its edge neighbors through the
// watertight mesh's paired directed edges, with bounded cancellation checks.
func facetAdjacencyContext(ctx context.Context, tris [][3]int) ([][]int, error) {
	owner := map[[2]int]int{}
	for i, tri := range tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		for k := range 3 {
			owner[[2]int{tri[k], tri[(k+1)%3]}] = i
		}
	}
	adj := make([][]int, len(tris))
	for i, tri := range tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		for k := range 3 {
			if twin, ok := owner[[2]int{tri[(k+1)%3], tri[k]}]; ok && twin != i {
				adj[i] = append(adj[i], twin)
			}
		}
	}
	return adj, nil
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
// the held float mesh, its per-facet source-face ids, and the proven terms the
// rounding's own error composes from.
type stitchedMesh struct {
	verts []r3.Vec
	tris  [][3]int
	src   []int
	// round is the proven displacement (mm) of one vertex under the final
	// float rounding: no held vertex is farther than this from the exact
	// point it stands for.
	round float64
	// preArea upper-bounds the area of the surface the rounding acted ON —
	// the stitched surface BEFORE any facet was dropped, and along the whole
	// motion from it to the held mesh. It is what the volume error is charged
	// against, NOT the held mesh's area: a facet the weld collapses is gone
	// from the held mesh, and charging the rounding against what survived
	// would leave that facet's own swept volume out of the bound.
	preArea float64
	// dropArea upper-bounds the area of the facets the weld collapsed, which
	// the held mesh no longer carries. The reported surface area is short by
	// exactly this much, so the area bound must cover it.
	dropArea float64
}

// stitchFacets welds the kept facets by shared exact vertices, makes the
// subdivision conforming (a vertex lying exactly on another facet's edge
// splits that facet — exact incidence, so nothing moves), audits closure,
// and rounds to float64. The audit is the §9 guarantee: every directed edge
// pairs with its reverse, or the boolean fails — never a cracked mesh.
func stitchFacetsContext(ctx context.Context, kept []keptFacet) (*stitchedMesh, error) {
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
	for i, f := range kept {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
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
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		split, err := conformOnce(ctx, &xverts, &tris, &src)
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

	// Round to float64, welding vertices whose roundings coincide — two exact
	// points closer than an ulp become one held vertex — and drop the facets
	// that collapse under the weld: a collapsed facet's two real directed
	// edges cancel each other, so closure survives, and the final audit
	// re-proves it. A collapsed facet is a zero-area triangle, so it moves
	// neither the volume integral nor the area sum of the HELD mesh — but the
	// facet it stands for was not zero-area before the weld, and what it did
	// carry has to be answered for. Two answers, below: the swept volume and
	// the missing area are charged against the PRE-ROUND surface (preArea,
	// dropArea), and a whole component welded out of existence is REFUSED —
	// there no bound would help, because a lump would be gone from the body
	// (its volume, its place in the lump count, its share of the bounds) while
	// the closure audit, which the surviving components still pass, reports
	// nothing wrong.
	out := &stitchedMesh{}
	floatIdx := map[r3.Vec]int{}
	remap := make([]int, len(xverts))
	worst := new(big.Rat)
	for i, p := range xverts {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		v := p.vec()
		fi, ok := floatIdx[v]
		if !ok {
			fi = len(out.verts)
			floatIdx[v] = fi
			out.verts = append(out.verts, v)
		}
		remap[i] = fi
		d := new(big.Rat).Sub(p.x, mustRatOf(v.X))
		d.Abs(d)
		for _, pair := range [][2]*big.Rat{{p.y, mustRatOf(v.Y)}, {p.z, mustRatOf(v.Z)}} {
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
	w, _ := worst.Float64()
	// worst is the max PER-COORDINATE rounding; the consumers read a 3D
	// distance bound, and all three coordinates can round at once (bounds.go,
	// radius3D).
	out.round = radius3D(upRound(w))

	dropped := make([]bool, len(tris))
	welded := make([][3]int, len(tris))
	for ti, tri := range tris {
		a, b, c := remap[tri[0]], remap[tri[1]], remap[tri[2]]
		welded[ti] = [3]int{a, b, c}
		if a == b || b == c || c == a {
			dropped[ti] = true
			continue
		}
		out.tris = append(out.tris, [3]int{a, b, c})
		out.src = append(out.src, src[ti])
	}
	if len(out.tris) == 0 {
		return nil, fmt.Errorf(`%w: the whole result collapsed under rounding`, ErrBooleanFailed)
	}
	if err := refuseWeldedAwayComponent(ctx, tris, dropped); err != nil {
		return nil, err
	}
	// The pre-round surface: every facet the exact stitch produced, dropped
	// ones included, measured on the held vertices and inflated by the
	// rounding they may each have travelled (bounds.go, perturbedAreaUpper).
	out.preArea = perturbedAreaUpper(out.verts, welded, out.round)
	var droppedTris [][3]int
	for ti := range tris {
		if dropped[ti] {
			droppedTris = append(droppedTris, welded[ti])
		}
	}
	out.dropArea = perturbedAreaUpper(out.verts, droppedTris, out.round)

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
	return out, nil
}

// refuseWeldedAwayComponent refuses the one class of weld collapse no bound can
// answer for: a connected component of the stitched surface EVERY facet of
// which the weld collapses. That component is a shell — a lump of the result,
// or a cavity inside one — and dropping all of it removes the lump from the
// body entirely: its volume, its place in Lumps(), its reach in the bounds box.
// The closure audit does not catch it, because the components that remain still
// close; nothing else in the pipeline would report it either. Every other
// collapse is an edge contraction WITHIN a component that survives, and the
// rounding bounds account for it: the swept volume is charged against the
// pre-round surface (preArea) and the missing facet area against dropArea, both
// of which count the dropped facets.
func refuseWeldedAwayComponent(ctx context.Context, tris [][3]int, dropped []bool) error {
	comp := make([]int, len(tris))
	for i := range comp {
		comp[i] = -1
	}
	adj, err := facetAdjacencyContext(ctx, tris)
	if err != nil {
		return err
	}
	work := 0
	for i := range tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return err
			}
		}
		if comp[i] != -1 {
			continue
		}
		queue := []int{i}
		comp[i] = i
		alive := false
		for len(queue) > 0 {
			work++
			if work%256 == 0 {
				if err := ctx.Err(); err != nil {
					return err
				}
			}
			f := queue[0]
			queue = queue[1:]
			alive = alive || !dropped[f]
			for _, nb := range adj[f] {
				if comp[nb] != -1 {
					continue
				}
				comp[nb] = i
				queue = append(queue, nb)
			}
		}
		if !alive {
			return fmt.Errorf(`%w: a whole component of the result welds away under float rounding — the lump would vanish from the body, and no volume, lump count or bound could then be trusted; this evaluator refuses rather than drop it`, ErrUnsupported)
		}
	}
	return nil
}

// conformOnce inserts, into every facet edge, the mesh vertices that lie
// exactly in that edge's interior, re-triangulating the facet so the
// subdivision conforms. Returns whether anything split.
func conformOnce(ctx context.Context, xverts *[]xpt, tris *[][3]int, src *[]int) (bool, error) {
	verts := *xverts
	// One counter spans the facet walk, the grid-cell candidate scan nested
	// under it, and the along-edge ordering: the candidate vertices a single
	// facet edge sweeps are the candidate operations §7.2 counts, not the
	// facets.
	budget := newWorkBudget(ctx)
	if err := budget.err(); err != nil {
		return false, err
	}
	scan, err := newConformScan(budget, verts)
	if err != nil {
		return false, err
	}

	var outTris [][3]int
	var outSrc []int
	splitAny := false
	for ti, tri := range *tris {
		if err := budget.step(); err != nil {
			return false, err
		}
		inserted := [3][]int{}
		for k := range 3 {
			a, b := tri[k], tri[(k+1)%3]
			hits, err := scan.edgeInteriorHits(budget, a, b, tri)
			if err != nil {
				return false, err
			}
			if len(hits) > 0 {
				if err := sortAlongEdge(budget, verts, a, b, hits); err != nil {
					return false, err
				}
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
		newTris, err := triangulatePlanarPolygon(ctx, verts, poly)
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
//
// The parameter t = ap[axis]/d[axis] is compared only against 0 and 1, which
// is decided without ever forming the quotient: t > 0 and t < 1 become two
// comparisons of ap[axis] against 0 and against d[axis], with the inequality
// direction flipped when d[axis] is negative — multiplying an inequality by a
// negative divisor reverses it.
func onSegmentInterior3(a, b, p xpt) bool {
	d := xsub(b, a)
	ap := xsub(p, a)
	cr := xcross(d, ap)
	if cr.x.Sign() != 0 || cr.y.Sign() != 0 || cr.z.Sign() != 0 {
		return false
	}
	axis := dominantAxis(d)
	da := ratCoordOf(d, axis)
	apAxis := ratCoordOf(ap, axis)
	switch da.Sign() {
	case 0:
		return false
	case 1:
		return apAxis.Sign() > 0 && apAxis.Cmp(da) < 0
	default:
		return apAxis.Sign() < 0 && apAxis.Cmp(da) > 0
	}
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

// conformScan is the searchable form of the stitched vertex set each facet edge
// is swept through: the exact vertices, their float approximations, a coarse
// uniform grid over those approximations, and the reject-only filter threshold
// every candidate is measured against. The grid prunes the exact on-edge tests;
// the slack absorbs the approximation, so no incidence is missed.
type conformScan struct {
	verts  []xpt
	approx []r3.Vec
	grid   map[[3]int][]int
	lo     r3.Vec
	cell   float64
	slack  float64
	tau2   float64
}

func newConformScan(budget *workBudget, verts []xpt) (*conformScan, error) {
	lo, hi := r3.Vec{}, r3.Vec{}
	approx := make([]r3.Vec, len(verts))
	for i, p := range verts {
		if err := budget.step(); err != nil {
			return nil, err
		}
		v := p.vec()
		approx[i] = v
		if i == 0 {
			lo, hi = v, v
			continue
		}
		lo = r3.Vec{X: math.Min(lo.X, v.X), Y: math.Min(lo.Y, v.Y), Z: math.Min(lo.Z, v.Z)}
		hi = r3.Vec{X: math.Max(hi.X, v.X), Y: math.Max(hi.Y, v.Y), Z: math.Max(hi.Z, v.Z)}
	}
	diag := hi.Sub(lo).Len()
	if diag == 0 {
		return nil, fmt.Errorf(`%w: the stitched boundary has no extent`, ErrBooleanFailed)
	}
	cell := diag / 64
	// maxAbs bounds every coordinate of every vertex, so the one threshold it
	// yields serves every segment and every candidate (segAdmissionRadius2).
	maxAbs := math.Max(
		math.Max(math.Abs(lo.X), math.Abs(hi.X)),
		math.Max(math.Max(math.Abs(lo.Y), math.Abs(hi.Y)), math.Max(math.Abs(lo.Z), math.Abs(hi.Z))),
	)
	s := &conformScan{
		verts:  verts,
		approx: approx,
		grid:   map[[3]int][]int{},
		lo:     lo,
		cell:   cell,
		slack:  diag*1e-9 + cell*1e-9,
	}
	s.tau2 = segAdmissionRadius2(s.slack, maxAbs)
	for i := range verts {
		if err := budget.step(); err != nil {
			return nil, err
		}
		c := s.cellOf(approx[i].X, approx[i].Y, approx[i].Z)
		s.grid[c] = append(s.grid[c], i)
	}
	return s, nil
}

func (s *conformScan) cellOf(x, y, z float64) [3]int {
	return [3]int{
		int(math.Floor((x - s.lo.X) / s.cell)),
		int(math.Floor((y - s.lo.Y) / s.cell)),
		int(math.Floor((z - s.lo.Z) / s.cell)),
	}
}

// edgeInteriorHits returns the mesh vertices lying exactly in the interior of
// facet edge (a, b), searched through the grid cells the edge's slack box
// covers. An edge spanning the mesh sweeps the whole grid, so the cells and the
// candidate vertices in them — not the facets — are the candidate operations
// §7.2 counts, and both step the budget.
//
// Every candidate meets the reject-only segFilter before the exact predicate,
// and the exact predicate still decides every candidate the filter does not
// reject. The traversal itself is untouched by that filter: it still visits the
// whole slack box, which is a SUPERSET of the cells the edge can touch. Walking
// only the cells the segment really passes through would cut the candidate set
// at its source, but it would also owe a completeness proof a superset does not
// — and it measured no faster here, because the filter has already reduced what
// a visited cell costs to a handful of float operations.
func (s *conformScan) edgeInteriorHits(budget *workBudget, a, b int, tri [3]int) ([]int, error) {
	pa, pb := s.approx[a], s.approx[b]
	filter := newSegFilter(pa, pb, s.tau2)
	var hits []int
	cLo := s.cellOf(math.Min(pa.X, pb.X)-s.slack, math.Min(pa.Y, pb.Y)-s.slack, math.Min(pa.Z, pb.Z)-s.slack)
	cHi := s.cellOf(math.Max(pa.X, pb.X)+s.slack, math.Max(pa.Y, pb.Y)+s.slack, math.Max(pa.Z, pb.Z)+s.slack)
	for cx := cLo[0]; cx <= cHi[0]; cx++ {
		for cy := cLo[1]; cy <= cHi[1]; cy++ {
			for cz := cLo[2]; cz <= cHi[2]; cz++ {
				if err := budget.step(); err != nil {
					return nil, err
				}
				for _, vi := range s.grid[[3]int{cx, cy, cz}] {
					if err := budget.step(); err != nil {
						return nil, err
					}
					if vi == tri[0] || vi == tri[1] || vi == tri[2] {
						continue
					}
					if filter.tooFar(s.approx[vi]) {
						continue
					}
					if onSegmentInterior3(s.verts[a], s.verts[b], s.verts[vi]) {
						hits = append(hits, vi)
					}
				}
			}
		}
	}
	return hits, nil
}

// sortAlongEdge orders the inserted vertices by their exact parameter along
// (a, b). Each parameter is computed once and carried through the sort: the
// comparison is a leaf exact predicate, so recomputing an exact division inside
// it would make the pass quadratic in rational arithmetic rather than in
// comparisons, with the same ordering.
func sortAlongEdge(budget *workBudget, verts []xpt, a, b int, hits []int) error {
	d := xsub(verts[b], verts[a])
	axis := dominantAxis(d)
	da := ratCoordOf(d, axis)
	params := make([]*big.Rat, len(hits))
	for i, vi := range hits {
		if err := budget.step(); err != nil {
			return err
		}
		params[i] = new(big.Rat).Quo(ratCoordOf(xsub(verts[vi], verts[a]), axis), da)
	}
	for i := 1; i < len(hits); i++ {
		for j := i; j > 0 && params[j].Cmp(params[j-1]) < 0; j-- {
			if err := budget.step(); err != nil {
				return err
			}
			hits[j], hits[j-1] = hits[j-1], hits[j]
			params[j], params[j-1] = params[j-1], params[j]
		}
	}
	return nil
}

// triangulatePlanarPolygon triangulates a planar polygon of mesh vertices
// (a facet boundary with collinear insertions) by exact ear clipping in the
// polygon's own plane, preserving orientation and using every vertex.
func triangulatePlanarPolygon(ctx context.Context, verts []xpt, poly []int) ([][3]int, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	budget := newWorkBudget(ctx)
	if len(poly) < 3 {
		return nil, fmt.Errorf(`%w: a conforming polygon lost its corners`, ErrBooleanFailed)
	}
	// The polygon is a triangle with edge insertions: its normal is the
	// original facet's, recoverable from any strict corner.
	n := xpt{new(big.Rat), new(big.Rat), new(big.Rat)}
	found := false
	for i := 1; i+1 < len(poly); i++ {
		if err := budget.step(); err != nil {
			return nil, err
		}
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
		if err := budget.step(); err != nil {
			return nil, err
		}
		pts[i] = xp2{ratCoordOf(verts[vi], u), ratCoordOf(verts[vi], v)}
		idx[i] = i
	}
	// Keep the projected orientation counter-clockwise so ear clipping and
	// the emitted winding agree with the facet's own.
	area, err := polyArea2(budget, pts)
	if err != nil {
		return nil, err
	}
	flip := area.Sign() < 0
	if flip {
		for i, j := 0, len(idx)-1; i < j; i, j = i+1, j-1 {
			if err := budget.step(); err != nil {
				return nil, err
			}
			idx[i], idx[j] = idx[j], idx[i]
		}
	}
	tris2, err := earClipX(budget, pts, idx)
	if err != nil {
		return nil, err
	}
	out := make([][3]int, 0, len(tris2))
	for _, t := range tris2 {
		if err := budget.step(); err != nil {
			return nil, err
		}
		a, b, c := poly[t[0]], poly[t[1]], poly[t[2]]
		if flip {
			b, c = c, b
		}
		out = append(out, [3]int{a, b, c})
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return out, nil
}
