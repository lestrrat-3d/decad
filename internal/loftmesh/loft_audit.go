package loftmesh

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/tessellation"

	"github.com/lestrrat-3d/decad/internal/meshbool"

	"github.com/lestrrat-3d/r3"
)

// This file is the build-time crossing audit of docs/loft-design.md §6: the
// gate that proves a loft's assembled wall-and-cap triangle set is manifold
// and watertight by construction rather than merely by convention. It reuses
// internal/proof/orientation.go's adaptive exact predicates and boolean_mesh.go's
// meshbool.TriTriClassify unchanged — the same machinery the mesh boolean already uses
// to decide whether two triangles are disjoint, share a point, share a
// segment, or overlap in a 2-D region — and adds no bracket engine of its
// own. TriTriCoplanarSharedEdge is the one new predicate §6 names, and it is
// audit-only: it never changes meshbool.TriTriClassify or mesh-boolean contact
// classification (proven by the required test that the same coplanar pair
// still reports meshbool.ContactRegion there).
//
// The audit takes a shared vertex table and triangles as indices into it, so
// two triangles sharing a vertex share the same INDEX — an exact, free fact
// read off recorded structure, never a distance test. Two shared indices
// expect the recorded common edge; one expects a point contact at that
// vertex; zero expects no contact at all. Any other classification is a
// proven self-contact or self-intersection (S7, ErrDegenerate); exhausting
// the fixed facet-pair ceiling before every pair is decided is S8
// (ErrUnsupported), refused before a single pair test runs.
//
// Two kinds of shortcut let a pair reach that verdict without the exact
// classification, and LoftAuditShortcuts switches each one separately so the
// audit keeps a reference path the tests compare every verdict against. The
// broad-phase in LoftCrossingAuditWork is reject-only and runs for a pair
// expected NOT to touch; AuditLoftPairData's certificates are accept-only and
// run for a pair expected to touch at a known entity. Each carries its proof
// on its own doc comment, and each proof rests on exact rational signs alone.

// TriangleCollapsed reports whether a triangle's three recorded vertices are
// exactly collinear (or coincident) — an exact zero cross product over the
// rational lift, never a tolerance. This is S6 (docs/loft-design.md §4): the
// triangle has no interior, so the shell it would contribute to does not
// exist.
func TriangleCollapsed(verts []r3.Vec, tri [3]int) bool {
	a, b, c := proof.XptOf(verts[tri[0]]), proof.XptOf(verts[tri[1]]), proof.XptOf(verts[tri[2]])
	n := proof.Xcross(proof.Xsub(b, a), proof.Xsub(c, a))
	return n.X.Sign() == 0 && n.Y.Sign() == 0 && n.Z.Sign() == 0
}

// LoftTriCorners reads one triangle's three float corners off the shared
// vertex table.
func LoftTriCorners(verts []r3.Vec, tri [3]int) [3]r3.Vec {
	return [3]r3.Vec{verts[tri[0]], verts[tri[1]], verts[tri[2]]}
}

// LoftXTriCorners is LoftTriCorners' exact lift.
func LoftXTriCorners(verts []r3.Vec, tri [3]int) [3]proof.Xpt {
	c := LoftTriCorners(verts, tri)
	return [3]proof.Xpt{proof.XptOf(c[0]), proof.XptOf(c[1]), proof.XptOf(c[2])}
}

// LoftAuditData holds the immutable per-vertex and per-triangle values shared
// by every pair in one crossing audit. The old pair path rebuilt the same
// exact vertex lifts and triangle normals for every pair, even though both are
// determined solely by the audit's input tables. Keeping them for the audit's
// lifetime does not change a predicate or its inputs; it only avoids repeating
// the same exact arithmetic.
type LoftAuditData struct {
	Corners     [][3]r3.Vec
	Xverts      []proof.Xpt
	Xtris       [][3]proof.Xpt
	Norms       []proof.Xpt
	Planes      []LoftExactPlane
	Projections [][3]meshbool.Xp2
}

// LoftExactPlane is an owned integer numerator for the oriented plane through
// anchor with normal n. For a point p, its sign is the sign of n dot
// (p-anchor), since all three homogeneous weights are positive.
type LoftExactPlane struct{ A, B, C, D *big.Int }

func NewLoftExactPlane(anchor, n proof.Xpt) LoftExactPlane {
	d := proof.XdotNum(n, anchor)
	return LoftExactPlane{
		A: new(big.Int).Mul(n.X, anchor.W),
		B: new(big.Int).Mul(n.Y, anchor.W),
		C: new(big.Int).Mul(n.Z, anchor.W),
		D: d.Neg(d),
	}
}

func (plane LoftExactPlane) Sign(p proof.Xpt, sum, term *big.Int) int {
	sum.Mul(plane.A, p.X)
	sum.Add(sum, term.Mul(plane.B, p.Y))
	sum.Add(sum, term.Mul(plane.C, p.Z))
	sum.Add(sum, term.Mul(plane.D, p.W))
	return sum.Sign()
}

func NewLoftAuditData(verts []r3.Vec, tris [][3]int) *LoftAuditData {
	d := &LoftAuditData{
		Corners:     make([][3]r3.Vec, len(tris)),
		Xverts:      make([]proof.Xpt, len(verts)),
		Xtris:       make([][3]proof.Xpt, len(tris)),
		Norms:       make([]proof.Xpt, len(tris)),
		Planes:      make([]LoftExactPlane, len(tris)),
		Projections: make([][3]meshbool.Xp2, len(tris)),
	}
	for i, v := range verts {
		d.Xverts[i] = proof.XptOf(v)
	}
	for i, tri := range tris {
		d.Corners[i] = LoftTriCorners(verts, tri)
		d.Xtris[i] = [3]proof.Xpt{d.Xverts[tri[0]], d.Xverts[tri[1]], d.Xverts[tri[2]]}
		d.Norms[i] = proof.Xcross(proof.Xsub(d.Xtris[i][1], d.Xtris[i][0]), proof.Xsub(d.Xtris[i][2], d.Xtris[i][0]))
		d.Planes[i] = NewLoftExactPlane(d.Xtris[i][0], d.Norms[i])
		projection := ProjectionPairIndex(meshbool.ProjAxes(d.Norms[i]))
		for j, p := range d.Xtris[i] {
			switch projection {
			case 0:
				d.Projections[i][j] = meshbool.NewXP2FromXpt(p, 0, 1)
			case 1:
				d.Projections[i][j] = meshbool.NewXP2FromXpt(p, 2, 0)
			default:
				d.Projections[i][j] = meshbool.NewXP2FromXpt(p, 1, 2)
			}
		}
	}
	return d
}

// ErrLoftContact is S7 (docs/loft-design.md §4/§6): the pair's exact contact
// is not the one its recorded adjacency expects, so the assembled shell
// self-touches or self-crosses away from — or instead of — its recorded
// vertex or edge. No such solid exists.
//
// The error is a *LoftContactError so a builder that numbers its triangles by
// face (sweep_mitre_build.go) can name the two faces instead of the indices.
func ErrLoftContact(i, j int, reason string) error {
	return &LoftContactError{I: i, J: j, Reason: reason}
}

// LoftContactError is ErrLoftContact's ErrDegenerate, carrying the two
// triangle indices the audit classified.
type LoftContactError struct {
	I, J   int
	Reason string
}

func (e *LoftContactError) Error() string {
	return fmt.Sprintf(`%s: loft triangles %d and %d %s`, decaderr.ErrDegenerate, e.I, e.J, e.Reason)
}

func (e *LoftContactError) Unwrap() error { return decaderr.ErrDegenerate }

// TriTriCoplanarSharedEdge is docs/loft-design.md §6's audit-only helper for
// an edge-adjacent coplanar pair that meshbool.TriTriClassify reports as
// meshbool.ContactRegion. It reads the two exact triangles' opposite vertices and
// admits the pair only when the recorded edge (edgeA, edgeB) is an edge of
// BOTH triangles and their two opposite (apex) vertices lie strictly on
// opposite sides of that edge's supporting line — the exact condition under
// which the two triangles' closed intersection is precisely the recorded
// segment, never a shared area. It never writes to meshbool.TriTriClassify or any
// shared classification state, so it cannot change mesh-boolean contact
// classification (docs/loft-design.md §6, required test).
func TriTriCoplanarSharedEdge(xta, xtb [3]proof.Xpt, n proof.Xpt, edgeA, edgeB proof.Xpt) bool {
	edgeAKey, edgeBKey := edgeA.Key(), edgeB.Key()
	var apexA, apexB proof.Xpt
	for _, p := range xta {
		key := p.Key()
		if key != edgeAKey && key != edgeBKey {
			apexA = p
			break
		}
	}
	for _, p := range xtb {
		key := p.Key()
		if key != edgeAKey && key != edgeBKey {
			apexB = p
			break
		}
	}
	sA := meshbool.PlaneSide(edgeA, edgeB, apexA, n)
	sB := meshbool.PlaneSide(edgeA, edgeB, apexB, n)
	return sA != 0 && sB != 0 && sA != sB
}

// SegMatchesRecordedEdge reports whether a non-coplanar segment contact is
// exactly the recorded shared edge (either endpoint order).
func SegMatchesRecordedEdge(c meshbool.TriContact, edgeA, edgeB proof.Xpt) bool {
	if c.Kind != meshbool.ContactSegment {
		return false
	}
	k0, k1 := c.P0.Key(), c.P1.Key()
	ka, kb := edgeA.Key(), edgeB.Key()
	return (k0 == ka && k1 == kb) || (k0 == kb && k1 == ka)
}

// LoftAuditShortcuts selects which of the S7 pair loop's shortcuts one audit
// call may use. Its ZERO VALUE is the audit's independent reference path:
// every pair of every triangle reaches AuditLoftPairData, and inside it every
// pair reaches meshbool.TriTriClassify's exact contact classification.
// LoftCrossingAudit runs BroadPhase, Certificates and Sweep;
// LoftCrossingAuditStructured runs all four. A shortcut may only ever change
// how SOON a pair's verdict is reached, never which verdict it is — which is
// what the reference path exists to test against.
//
// The fields switch structurally different things, and each is switched
// separately so a test can isolate any one:
//
//   - BroadPhase gates the two reject-only float tiers ahead of the exact
//     classification (LoftPlaneSeparated and the bounding-box test), which
//     only ever prove a zero-shared-vertex pair APART.
//   - Certificates gates AuditLoftPairData's own accept-only proofs, which
//     only ever prove a shared-entity pair's contact IS the expected entity.
//   - Sweep replaces the all-pairs enumeration with sweepCandidates' list of
//     box-overlapping pairs (loft_audit_sweep.go). A pair it leaves out has
//     disjoint boxes, so it shares no point and no vertex index, and the
//     reference path admits it.
//   - CapProof decides every pair that has a triangle in a cap whose
//     CapFamilyProof holds (loft_cap_proof.go). It needs the structure only
//     LoftCrossingAuditStructured is handed; the generic entry ignores it.
type LoftAuditShortcuts struct {
	BroadPhase   bool
	Certificates bool
	Sweep        bool
	CapProof     bool
}

// LoftAuditWork records what ONE crossing-audit call's S7 pair loop did,
// one field per way a pair can be decided:
//
//   - skips: a zero-shared-vertex pair a broad-phase tier proved apart.
//   - edgeCerts: a pair the noncoplanar shared-edge certificate admitted
//     (AuditLoftPairData's own doc comment carries its proof).
//   - vertexCerts: a pair the isolated shared-vertex certificate admitted.
//   - classifications: every other pair, which reached AuditLoftPairData
//     without a certificate deciding it.
//
// Candidates is the number of pairs the pair loop was handed: every pair of
// the triangles no cap proof decided, or only the box-overlapping ones under
// Sweep. It is the count S8 compares against the ceiling. CapProofs is how
// many cap families (0, 1 or 2) CapFamilyProof decided.
//
// The four pair outcomes always sum to Candidates on a call that ran to
// completion. classifications counts the pairs the certificates are there to
// remove, so it is the quantity a shortcut is measured by; it also carries the
// handful of coplanar shared-edge pairs TriTriCoplanarSharedEdge decides
// without reaching meshbool.TriTriClassify, since that branch predates the certificates
// and no shortcut switch changes it.
//
// It is per-call state, held in LoftCrossingAuditWork's own frame and
// returned by value — never a package-level counter. The audit runs on the
// production path of every exported entry point that builds or re-lifts a
// loft (Document.Loft, and Body.Placed/Duplicate/
// PlacedCopy through loftPayload.placed), and two goroutines holding two
// independent Documents may each be inside it at once, so a counter shared
// across calls would both race and mis-count. Nothing outside the call frame
// is written anywhere on this path.
type LoftAuditWork struct {
	Skips           int
	EdgeCerts       int
	VertexCerts     int
	Classifications int
	Candidates      int
	CapProofs       int
}

// LoftPairOutcome names which of AuditLoftPairData's own decision paths
// settled one pair, so LoftCrossingAuditWork can count it without repeating
// the classification. It says nothing about the verdict: a pair that returns
// LoftPairClassified may have been admitted or refused.
type LoftPairOutcome int

const (
	// LoftPairClassified: the pair reached the exact contact classification
	// (or the pre-existing coplanar shared-edge branch).
	LoftPairClassified LoftPairOutcome = iota
	// LoftPairEdgeCertificate: the noncoplanar shared-edge certificate
	// admitted the pair.
	LoftPairEdgeCertificate
	// LoftPairVertexCertificate: an isolated shared-vertex certificate
	// admitted the pair.
	LoftPairVertexCertificate
)

// LoftPlaneSeparated is the S7 broad-phase's second, still-float-only tier
// for a zero-shared-vertex pair whose bounding boxes DO overlap: it
// reproduces meshbool.TriTriClassify's OWN opening move (boolean_mesh.go) —
// meshbool.AllOneSide(proof.OrientSign(...)) against each triangle's plane — over nothing
// but the two triangles' float corners, so it can prove "one triangle sits
// strictly on one side of the other's plane, and so the pair cannot touch at
// all" without ever building the exact-rational lift AuditLoftPair pays for
// on every call. It calls the IDENTICAL proof.OrientSign and meshbool.AllOneSide functions
// meshbool.TriTriClassify itself calls first, on the IDENTICAL float corners, so it
// cannot disagree with meshbool.TriTriClassify's own verdict for the cases it
// decides — a proof here is the same proof there, just reached before the
// exact lift is built. proof.OrientSign is itself adaptive (its own doc comment:
// a float evaluation whose forward error provably cannot cross zero decides
// the generic case, and only a genuinely ambiguous determinant pays the
// exact fallback), so the common case — the two triangles' planes are not
// near-tangent to one another — never touches big.Rat at all.
//
// This is deliberately NOT meshbool.TriTriMissesFilter (internal/meshbool/triangle_filter.go): that
// filter's own doc comment requires na/nb to be proof.Xpt.vec() — the
// correctly-rounded float64 conversion of the pair's EXACT rational
// normal — with meshbool.FivRounded's extra ulp of margin calibrated for exactly
// that rounding step, so using it here would still force building the exact
// cross product it is supposed to help avoid. LoftPlaneSeparated needs no
// normal at all, exact or float, so it pays nothing this pair does not
// already have in hand.
func LoftPlaneSeparated(ta, tb [3]r3.Vec) bool {
	var sb [3]int
	for i := range 3 {
		sb[i] = proof.OrientSign(ta[0], ta[1], ta[2], tb[i])
	}
	if meshbool.AllOneSide(sb) {
		return true
	}
	var sa [3]int
	for i := range 3 {
		sa[i] = proof.OrientSign(tb[0], tb[1], tb[2], ta[i])
	}
	return meshbool.AllOneSide(sa)
}

// AuditLoftPair classifies one triangle pair against its recorded adjacency
// (docs/loft-design.md §6) and returns S7 (ErrDegenerate) when the exact
// contact is not the one that adjacency expects. It is the tests' reference
// entry point: it runs AuditLoftPairData's zero-shortcut path, so every pair
// it decides reaches the exact classification.
func AuditLoftPair(verts []r3.Vec, tris [][3]int, i, j int) error {
	_, err := AuditLoftPairData(NewLoftAuditData(verts, tris), tris, i, j, LoftAuditShortcuts{})
	return err
}

// AuditLoftPairData classifies pair (i, j) and reports which decision path
// settled it. PRECONDITION: both triangles have three DISTINCT vertex indices
// and are noncollapsed — S6 (TriangleCollapsed, run over every triangle before
// the S7 pair loop starts) is what establishes that for the production caller,
// and the test-only AuditLoftPair above is called with noncollapsed fixtures.
//
// # Certificate A — the noncoplanar shared edge
//
// A pair sharing exactly two vertex indices is admitted, without any contact
// reconstruction at all, once one exact sign proves the two triangles are NOT
// coplanar. The sign is the one this function already computed to choose
// between the coplanar and the general branch, so the certificate costs
// nothing beyond a branch.
//
// Write E for the shared pair's segment and L for the line through it. E's two
// endpoints are distinct (the precondition), so L exists and is unique.
//
//  1. E is an edge of BOTH triangles: two shared INDICES into the audit's own
//     vertex table are two distinct corners of each triangle.
//  2. Both triangles' planes contain L, since each contains E's two distinct
//     endpoints.
//  3. The two planes are DISTINCT: the certificate fires only when triangle
//     j's remaining corner has a nonzero exact signed distance from triangle
//     i's plane, so that corner lies in plane(j) and not in plane(i).
//  4. Two distinct planes sharing the line L meet in exactly L, so the pair's
//     whole intersection lies on L.
//  5. A closed triangle meets the line supporting one of its own edges in
//     exactly that edge: within the triangle's plane the triangle lies in one
//     closed half-plane of L, and its intersection with L's boundary line is
//     the edge itself. So triangle i ∩ L = E and triangle j ∩ L = E.
//  6. Therefore triangle i ∩ triangle j = E — precisely the contact this
//     pair's recorded adjacency expects — and §6's shared-edge rule admits it.
//
// The sign is exact rational arithmetic (proof.XdotSign), never a float tolerance,
// so step 3 is a proof and not an estimate. A ZERO sign proves nothing about
// step 3 and takes no shortcut: the coplanar branch below keeps deciding that
// case through TriTriCoplanarSharedEdge, which is what still refuses two
// coplanar triangles that share an edge and overlap in area on one side of it.
//
// # Certificate B — the isolated shared vertex
//
// A pair sharing exactly one vertex index is admitted once one triangle's two
// remaining vertices carry the SAME strict exact sign against the other
// triangle's plane. IsolatedSharedVertex owns that reading and its proof; both
// orientations are tried, because either triangle may be the one whose plane
// isolates the vertex. The signs are the very arrays this function hands
// meshbool.TriTriClassifyWithProjections, so the certificate reads work the pair was
// going to pay for anyway and adds none of its own.
//
// A zero sign, or two differing signs, proves nothing and takes no shortcut:
// that pair falls through to the exact classification, which is what still
// refuses a vertex-sharing pair whose triangles cross away from their shared
// vertex. Sharing a vertex is not itself evidence — EVERY such pair has its
// shared vertex on both planes, which is why the certificate reads the two
// OTHER vertices and never that one.
//
// # Certificate C — the coplanar isolated shared vertex
//
// A line through the shared vertex and another corner of one triangle may
// separate its third corner strictly from both remaining corners of the
// second triangle. The second triangle meets the line only at the shared
// vertex; the first stays entirely on its own side. Convexity confines their
// intersection to that vertex. A zero sign falls through to classification.
func AuditLoftPairData(data *LoftAuditData, tris [][3]int, i, j int, shortcuts LoftAuditShortcuts) (LoftPairOutcome, error) {
	ta := data.Corners[i]
	tb := data.Corners[j]
	xta := data.Xtris[i]
	xtb := data.Xtris[j]
	na := data.Norms[i]
	nb := data.Norms[j]
	shared, sharedCount := tessellation.SharedVertexIndices(tris[i], tris[j])
	if sharedCount == 2 {
		edgeA, edgeB := data.Xverts[shared[0]], data.Xverts[shared[1]]
		apex := data.Xverts[tessellation.TriangleApexIndex(tris[j], shared[0], shared[1])]
		if proof.XdotSign(na, proof.Xsub(apex, edgeA)) == 0 {
			if TriTriCoplanarSharedEdge(xta, xtb, na, edgeA, edgeB) {
				return LoftPairClassified, nil
			}
			return LoftPairClassified, ErrLoftContact(i, j, "do not meet exactly along their recorded shared edge")
		}
		if shortcuts.Certificates {
			return LoftPairEdgeCertificate, nil
		}
	}

	signsB := TrianglePlaneSigns(data.Planes[i], xtb)
	if shortcuts.Certificates && sharedCount == 1 && meshbool.CountZero(signsB) == 3 &&
		CoplanarIsolatedSharedVertex(data, tris[i], tris[j], i, j, shared[0]) {
		return LoftPairVertexCertificate, nil
	}
	if shortcuts.Certificates && sharedCount == 1 && IsolatedSharedVertex(tris[j], shared[0], signsB) {
		return LoftPairVertexCertificate, nil
	}
	signsA := TrianglePlaneSigns(data.Planes[j], xta)
	if shortcuts.Certificates && sharedCount == 1 && IsolatedSharedVertex(tris[i], shared[0], signsA) {
		return LoftPairVertexCertificate, nil
	}
	contact, err := meshbool.TriTriClassifyWithProjections(ta, tb, xta, xtb, na, nb,
		&data.Projections[i], &data.Projections[j], &signsA, &signsB, true)
	if err != nil {
		return LoftPairClassified, err
	}

	switch sharedCount {
	case 0:
		if contact.Kind == meshbool.ContactNone {
			return LoftPairClassified, nil
		}
		return LoftPairClassified, ErrLoftContact(i, j, "share no recorded vertex, but make contact")
	case 1:
		v := data.Xverts[shared[0]]
		if contact.Kind == meshbool.ContactPoint && contact.P0.Key() == v.Key() {
			return LoftPairClassified, nil
		}
		return LoftPairClassified, ErrLoftContact(i, j, "do not meet exactly at their recorded shared vertex")
	case 2:
		edgeA, edgeB := data.Xverts[shared[0]], data.Xverts[shared[1]]
		if SegMatchesRecordedEdge(contact, edgeA, edgeB) {
			return LoftPairClassified, nil
		}
		if contact.Kind == meshbool.ContactRegion && TriTriCoplanarSharedEdge(xta, xtb, na, edgeA, edgeB) {
			return LoftPairClassified, nil
		}
		return LoftPairClassified, ErrLoftContact(i, j, "do not meet exactly along their recorded shared edge")
	default:
		return LoftPairClassified, ErrLoftContact(i, j, "share an unexpected vertex count")
	}
}

// CoplanarIsolatedSharedVertex proves that two coplanar triangles meet only
// at their one recorded common vertex. An edge line through that vertex
// separates both remaining corners of the second triangle strictly from the
// opposite corner of the first. The second triangle meets the line only at
// the shared vertex, while the first lies entirely on its own side. The
// triangles' exact normals are proportional, so meshbool.ProjAxes drops the same
// dominant coordinate for both; that projection is invertible on their plane.
// Its meshbool.Cross2xSign preserves the line-side signs and uses an exact fallback.
func CoplanarIsolatedSharedVertex(data *LoftAuditData, a, b [3]int, i, j, shared int) bool {
	var aOther, bOther [2]meshbool.Xp2
	countA, countB := 0, 0
	var v meshbool.Xp2
	for corner, vertex := range a {
		if vertex == shared {
			v = data.Projections[i][corner]
			continue
		}
		aOther[countA] = data.Projections[i][corner]
		countA++
	}
	for corner, vertex := range b {
		if vertex != shared {
			bOther[countB] = data.Projections[j][corner]
			countB++
		}
	}
	for k := range 2 {
		side := meshbool.Cross2xSign(v, aOther[k], aOther[1-k])
		if side == 0 {
			continue
		}
		if meshbool.Cross2xSign(v, aOther[k], bOther[0]) == -side &&
			meshbool.Cross2xSign(v, aOther[k], bOther[1]) == -side {
			return true
		}
	}
	return false
}

// IsolatedSharedVertex is certificate B's own reading (docs/loft-design.md
// §6): it reports whether the triangle tri meets a plane at sharedIndex's
// vertex ALONE, given signs — the exact signs of tri's three vertices against
// that plane, in tri's own index order, as TrianglePlaneSigns returns them.
//
// Write f for the plane's affine signed-distance function, v for the shared
// vertex, and q1, q2 for tri's two other vertices. The certificate fires only
// when f(q1) and f(q2) are both strictly positive or both strictly negative.
//
//  1. f(v) = 0. v is a corner of the triangle whose plane this is, so it lies
//     on that plane. The caller guarantees this by passing the index the two
//     triangles SHARE; the reading never needs to test it.
//  2. Every point of tri is p = αv + βq1 + γq2 with α + β + γ = 1 and all
//     three coefficients at least zero, since a closed triangle is the convex
//     hull of its corners.
//  3. f is affine, so f(p) = β·f(q1) + γ·f(q2). With f(q1) and f(q2) sharing
//     one strict sign, that sum is zero exactly when β = γ = 0, which is
//     exactly p = v.
//  4. So tri meets the plane only at v. The other triangle lies IN that plane,
//     so the pair's whole intersection is contained in {v} — and v belongs to
//     both triangles, so it IS {v}, the contact the pair's shared vertex
//     expects.
//
// A zero among the two tested signs breaks step 3 and is never accepted, so
// the reading is a proof and not an estimate. A collapsed triangle cannot
// smuggle a verdict through either: its exact normal is the zero vector, every
// sign against it is zero, and the certificate cannot fire.
func IsolatedSharedVertex(tri [3]int, sharedIndex int, signs [3]int) bool {
	positive, negative := 0, 0
	for k, vertex := range tri {
		if vertex == sharedIndex {
			continue
		}
		switch {
		case signs[k] > 0:
			positive++
		case signs[k] < 0:
			negative++
		}
	}
	return positive == 2 || negative == 2
}

// TrianglePlaneSigns returns the exact signs of other against the cached
// oriented plane. Its affine numerator avoids rebuilding a difference vector
// for each point of each facet pair.
func TrianglePlaneSigns(plane LoftExactPlane, other [3]proof.Xpt) [3]int {
	var signs [3]int
	var sum, term big.Int
	for i, p := range other {
		signs[i] = plane.Sign(p, &sum, &term)
	}
	return signs
}

func ProjectionPairIndex(u, v int) int {
	switch {
	case u == 0 && v == 1:
		return 0
	case u == 2 && v == 0:
		return 1
	default:
		return 2
	}
}
