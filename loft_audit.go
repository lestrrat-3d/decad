package decad

import (
	"fmt"

	"github.com/lestrrat-3d/r3"
)

// This file is the build-time crossing audit of docs/loft-design.md §6: the
// gate that proves a loft's assembled wall-and-cap triangle set is manifold
// and watertight by construction rather than merely by convention. It reuses
// boolean_exact.go's adaptive exact predicates and boolean_mesh.go's
// triTriClassify unchanged — the same machinery the mesh boolean already uses
// to decide whether two triangles are disjoint, share a point, share a
// segment, or overlap in a 2-D region — and adds no bracket engine of its
// own. triTriCoplanarSharedEdge is the one new predicate §6 names, and it is
// audit-only: it never changes triTriClassify or mesh-boolean contact
// classification (proven by the required test that the same coplanar pair
// still reports contactRegion there).
//
// The audit takes a shared vertex table and triangles as indices into it, so
// two triangles sharing a vertex share the same INDEX — an exact, free fact
// read off recorded structure, never a distance test. Two shared indices
// expect the recorded common edge; one expects a point contact at that
// vertex; zero expects no contact at all. Any other classification is a
// proven self-contact or self-intersection (S7, ErrDegenerate); exhausting
// the fixed facet-pair ceiling before every pair is decided is S8
// (ErrUnsupported), refused before a single pair test runs.

// triangleCollapsed reports whether a triangle's three recorded vertices are
// exactly collinear (or coincident) — an exact zero cross product over the
// rational lift, never a tolerance. This is S6 (docs/loft-design.md §4): the
// triangle has no interior, so the shell it would contribute to does not
// exist.
func triangleCollapsed(verts []r3.Vec, tri [3]int) bool {
	a, b, c := xptOf(verts[tri[0]]), xptOf(verts[tri[1]]), xptOf(verts[tri[2]])
	n := xcross(xsub(b, a), xsub(c, a))
	return n.x.Sign() == 0 && n.y.Sign() == 0 && n.z.Sign() == 0
}

// loftTriCorners reads one triangle's three float corners off the shared
// vertex table.
func loftTriCorners(verts []r3.Vec, tri [3]int) [3]r3.Vec {
	return [3]r3.Vec{verts[tri[0]], verts[tri[1]], verts[tri[2]]}
}

// loftXTriCorners is loftTriCorners' exact lift.
func loftXTriCorners(verts []r3.Vec, tri [3]int) [3]xpt {
	c := loftTriCorners(verts, tri)
	return [3]xpt{xptOf(c[0]), xptOf(c[1]), xptOf(c[2])}
}

// sharedVertexIndices returns the vertex indices two triangles hold in
// common — free and exact, since a shared vertex is a shared table index.
// The loft build never emits a triangle with a repeated index (S6 refuses
// any that would collapse first), so the result holds at most 3 entries and
// no duplicates.
func sharedVertexIndices(a, b [3]int) []int {
	var shared []int
	for _, va := range a {
		for _, vb := range b {
			if va == vb {
				shared = append(shared, va)
				break
			}
		}
	}
	return shared
}

// errLoftContact is S7 (docs/loft-design.md §4/§6): the pair's exact contact
// is not the one its recorded adjacency expects, so the assembled shell
// self-touches or self-crosses away from — or instead of — its recorded
// vertex or edge. No such solid exists.
func errLoftContact(i, j int, reason string) error {
	return fmt.Errorf(`%w: loft triangles %d and %d %s`, ErrDegenerate, i, j, reason)
}

// triTriCoplanarSharedEdge is docs/loft-design.md §6's audit-only helper for
// an edge-adjacent coplanar pair that triTriClassify reports as
// contactRegion. It projects both exact triangles onto their shared plane
// and admits the pair only when the recorded edge (edgeA, edgeB) is an edge
// of BOTH triangles and their two opposite (apex) vertices lie strictly on
// opposite sides of that edge's supporting line — the exact condition under
// which the two triangles' closed intersection is precisely the recorded
// segment, never a shared area. It reads only boolean_mesh.go's projAxes/
// xcoordOf/cross2xSign and never writes to triTriClassify or any shared
// classification state, so it cannot change mesh-boolean contact
// classification (docs/loft-design.md §6, required test).
func triTriCoplanarSharedEdge(xta, xtb [3]xpt, n xpt, edgeA, edgeB xpt) bool {
	u, v := projAxes(n)
	project := func(p xpt) xp2 { return xp2{xcoordOf(p, u), xcoordOf(p, v)} }
	a2 := [3]xp2{project(xta[0]), project(xta[1]), project(xta[2])}
	b2 := [3]xp2{project(xtb[0]), project(xtb[1]), project(xtb[2])}
	e0, e1 := project(edgeA), project(edgeB)

	apexA, ok := loftEdgeApex(a2, e0, e1)
	if !ok {
		return false
	}
	apexB, ok := loftEdgeApex(b2, e0, e1)
	if !ok {
		return false
	}
	sA := cross2xSign(e0, e1, apexA)
	sB := cross2xSign(e0, e1, apexB)
	return sA != 0 && sB != 0 && sA != sB
}

// loftEdgeApex finds the triangle edge that exactly matches the unordered
// pair (e0, e1) and returns the triangle's third (opposite) vertex.
func loftEdgeApex(tri [3]xp2, e0, e1 xp2) (xp2, bool) {
	for i := range 3 {
		a, b := tri[i], tri[(i+1)%3]
		ak, bk := a.key2(), b.key2()
		if (ak == e0.key2() && bk == e1.key2()) || (ak == e1.key2() && bk == e0.key2()) {
			return tri[(i+2)%3], true
		}
	}
	return xp2{}, false
}

// segMatchesRecordedEdge reports whether a non-coplanar segment contact is
// exactly the recorded shared edge (either endpoint order).
func segMatchesRecordedEdge(c triContact, edgeA, edgeB xpt) bool {
	if c.kind != contactSegment {
		return false
	}
	k0, k1 := c.p0.key(), c.p1.key()
	ka, kb := edgeA.key(), edgeB.key()
	return (k0 == ka && k1 == kb) || (k0 == kb && k1 == ka)
}

// loftBroadPhaseWork records what ONE loftCrossingAudit call's S7 pair loop
// did: skips counts the zero-shared-vertex pairs a broad-phase tier proved
// apart, and classifications counts the pairs that reached auditLoftPair's
// exact classification. The two always sum to the pair count S8 admitted.
//
// It is per-call state, held in loftCrossingAuditWork's own frame and
// returned by value — never a package-level counter. The audit runs on the
// production path of every exported entry point that builds or re-lifts a
// loft (Document.Loft/LoftContext, and Body.Placed/PlacedContext/Duplicate/
// PlacedCopy through loftPayload.placed), and two goroutines holding two
// independent Documents may each be inside it at once, so a counter shared
// across calls would both race and mis-count. Nothing outside the call frame
// is written anywhere on this path.
type loftBroadPhaseWork struct {
	skips           int
	classifications int
}

// loftPlaneSeparated is the S7 broad-phase's second, still-float-only tier
// for a zero-shared-vertex pair whose bounding boxes DO overlap: it
// reproduces triTriClassify's OWN opening move (boolean_mesh.go) —
// allOneSide(orientSign(...)) against each triangle's plane — over nothing
// but the two triangles' float corners, so it can prove "one triangle sits
// strictly on one side of the other's plane, and so the pair cannot touch at
// all" without ever building the exact-rational lift auditLoftPair pays for
// on every call. It calls the IDENTICAL orientSign and allOneSide functions
// triTriClassify itself calls first, on the IDENTICAL float corners, so it
// cannot disagree with triTriClassify's own verdict for the cases it
// decides — a proof here is the same proof there, just reached before the
// exact lift is built. orientSign is itself adaptive (its own doc comment:
// a float evaluation whose forward error provably cannot cross zero decides
// the generic case, and only a genuinely ambiguous determinant pays the
// exact fallback), so the common case — the two triangles' planes are not
// near-tangent to one another — never touches big.Rat at all.
//
// This is deliberately NOT triTriMissesFilter (boolean_exact.go): that
// filter's own doc comment requires na/nb to be xpt.vec() — the
// correctly-rounded float64 conversion of the pair's EXACT rational
// normal — with fivRounded's extra ulp of margin calibrated for exactly
// that rounding step, so using it here would still force building the exact
// cross product it is supposed to help avoid. loftPlaneSeparated needs no
// normal at all, exact or float, so it pays nothing this pair does not
// already have in hand.
func loftPlaneSeparated(ta, tb [3]r3.Vec) bool {
	var sb [3]int
	for i := range 3 {
		sb[i] = orientSign(ta[0], ta[1], ta[2], tb[i])
	}
	if allOneSide(sb) {
		return true
	}
	var sa [3]int
	for i := range 3 {
		sa[i] = orientSign(tb[0], tb[1], tb[2], ta[i])
	}
	return allOneSide(sa)
}

// auditLoftPair classifies one triangle pair against its recorded adjacency
// (docs/loft-design.md §6) and returns S7 (ErrDegenerate) when the exact
// contact is not the one that adjacency expects.
func auditLoftPair(verts []r3.Vec, tris [][3]int, i, j int) error {
	ta := loftTriCorners(verts, tris[i])
	tb := loftTriCorners(verts, tris[j])
	xta := loftXTriCorners(verts, tris[i])
	xtb := loftXTriCorners(verts, tris[j])
	na := xcross(xsub(xta[1], xta[0]), xsub(xta[2], xta[0]))
	nb := xcross(xsub(xtb[1], xtb[0]), xsub(xtb[2], xtb[0]))

	contact, err := triTriClassify(ta, tb, xta, xtb, na, nb)
	if err != nil {
		return err
	}

	shared := sharedVertexIndices(tris[i], tris[j])
	switch len(shared) {
	case 0:
		if contact.kind == contactNone {
			return nil
		}
		return errLoftContact(i, j, "share no recorded vertex, but make contact")
	case 1:
		v := xptOf(verts[shared[0]])
		if contact.kind == contactPoint && contact.p0.key() == v.key() {
			return nil
		}
		return errLoftContact(i, j, "do not meet exactly at their recorded shared vertex")
	case 2:
		edgeA, edgeB := xptOf(verts[shared[0]]), xptOf(verts[shared[1]])
		if segMatchesRecordedEdge(contact, edgeA, edgeB) {
			return nil
		}
		if contact.kind == contactRegion && triTriCoplanarSharedEdge(xta, xtb, na, edgeA, edgeB) {
			return nil
		}
		return errLoftContact(i, j, "do not meet exactly along their recorded shared edge")
	default:
		return errLoftContact(i, j, "share an unexpected vertex count")
	}
}

// loftCrossingAudit is docs/loft-design.md §6's whole build-time audit over
// the assembled wall-and-cap triangle set: S6 (per-triangle existence) first,
// then S8 (the fixed facet-pair ceiling, checked before any pair test or
// allocation), then S7 (the pair-by-pair contact audit). budget is shared
// with the rest of the pre-commit cancellation path exactly as
// docs/modify-design.md §5's audits already share one (fillet_audit.go);
// step is called once per candidate (each triangle in S6, each pair in S7)
// and err at every phase boundary, the end of S7 among them.
//
// This is the production entry point: the broad-phase always runs. Tests that
// need the same audit with the short-circuit switched off, or need the pair
// loop's own work counts, call loftCrossingAuditWork below directly.
func loftCrossingAudit(budget *workBudget, verts []r3.Vec, tris [][3]int) error {
	_, err := loftCrossingAuditWork(budget, verts, tris, true)
	return err
}

// loftCrossingAuditWork is loftCrossingAudit's body, with the S7 broad-phase
// short-circuit under an explicit per-call switch and the pair loop's own
// work counts returned to the caller. broadPhase false runs every admitted
// pair through auditLoftPair's exact classification, which is the verdict the
// broad-phase may only ever reach sooner, never change.
//
// The returned counts are meaningful only when the audit completes: an early
// return carries whatever the loop had reached when it stopped.
func loftCrossingAuditWork(budget *workBudget, verts []r3.Vec, tris [][3]int, broadPhase bool) (loftBroadPhaseWork, error) {
	var work loftBroadPhaseWork
	if err := budget.err(); err != nil {
		return work, err
	}

	// S6: per-triangle existence, before the pair audit runs at all.
	for i, tri := range tris {
		if err := budget.step(); err != nil {
			return work, err
		}
		if triangleCollapsed(verts, tri) {
			return work, fmt.Errorf(`%w: loft triangle %d has collapsed to zero area`, ErrDegenerate, i)
		}
	}
	if err := budget.err(); err != nil {
		return work, err
	}

	// S8: the facet-pair ceiling, computed under checked arithmetic and
	// refused before a single pair test — or any pair buffer — is built.
	f := len(tris)
	pairs, ok := wallChoose2(uint64(f))
	if !ok || pairs > maxFacetPairTestsPerCall {
		return work, fmt.Errorf(`%w: the loft crossing audit's facet-pair count exceeds the fixed work ceiling`, ErrUnsupported)
	}

	// A broad-phase per-triangle bounding box (boolean_mesh.go's triBox, the
	// same helper prepBoolMesh builds for the mesh boolean's own facet-pair
	// pruning) lets S7 below skip the expensive exact classification for a
	// pair PROVEN apart. triBox's own doc comment already establishes the
	// box is exact — "float min/max are exact, so the box is a true
	// bound" — built from float64 vertex coordinates that are themselves
	// exact inputs to xptOf (no rounding occurs converting a float64 to its
	// rational value), so no epsilon widening is needed or added: every
	// point of the closed triangle, in exact arithmetic, is a convex
	// combination of its three vertices, and a convex combination of values
	// bounded by [lo, hi] on one axis is itself bounded by [lo, hi] on that
	// axis. boxesOverlap's own comparisons (<=, not <) are exact float64
	// comparisons, so two boxes it reports as NOT overlapping are proven, in
	// exact real arithmetic, to share no point on some axis — the two closed
	// triangles cannot touch at all. Building f boxes is O(f) trivial float
	// comparisons, not O(f^2) exact-rational work, so it needs no budget
	// step of its own; the S8 gate above already bounds f to a few thousand.
	boxes := make([][2]r3.Vec, f)
	for i, tri := range tris {
		boxes[i] = triBox(verts, tri)
	}

	// S7: every pair, classified against its recorded adjacency. The
	// broad-phase short-circuit runs ONLY for a pair sharing no recorded
	// vertex (len(shared) == 0): that is the one case (auditLoftPair's
	// switch) where the audit's own passing verdict is "no contact at all",
	// so a proof of no contact reaches the identical verdict auditLoftPair
	// would reach. A pair sharing one or two vertices is REQUIRED to touch —
	// at that vertex, or along that edge — so the guard below never lets the
	// short-circuit run for it; auditLoftPair always decides those pairs.
	//
	// Two tiers run, cheapest first, either one sufficient to skip: the
	// bounding-box test above, then loftPlaneSeparated for a pair whose boxes
	// do overlap but whose planes still prove separation. Both are float-only
	// and reject-only; neither ever answers "touching", so a pair either tier
	// cannot decide always falls through to the full auditLoftPair.
	for i := range f {
		for j := i + 1; j < f; j++ {
			if err := budget.step(); err != nil {
				return work, err
			}
			shared := sharedVertexIndices(tris[i], tris[j])
			if broadPhase && len(shared) == 0 {
				if !boxesOverlap(boxes[i], boxes[j]) {
					work.skips++
					continue
				}
				if loftPlaneSeparated(loftTriCorners(verts, tris[i]), loftTriCorners(verts, tris[j])) {
					work.skips++
					continue
				}
			}
			work.classifications++
			if err := auditLoftPair(verts, tris, i, j); err != nil {
				return work, err
			}
		}
	}

	// The trailing poll is what closes the gap between S7's last step and this
	// return: step observes the context only on the polling interval, so a
	// small triangle set finishes every pair without one landing, while err
	// polls unconditionally. It cannot fail an audit that legitimately
	// completed — errFn is ctx.Err, and a live context yields nil.
	//
	// Cancellation is answered in two places, and this is only one of them.
	// The audit kernel polls its own loops, which is what bounds the time
	// spent inside a single expensive phase; the caller-facing contract — a
	// cancelled operation leaves the receiver live and the recipe and document
	// unchanged — is discharged at the commit edge by the entry point, the way
	// fillet.go, chamfer.go and shell.go each check ctx.Err() immediately
	// before Document.commit. Loft's own commit-edge check belongs to
	// LoftContext.
	return work, budget.err()
}
