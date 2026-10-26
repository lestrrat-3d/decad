package loftmesh

import (
	"errors"
	"fmt"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"

	"github.com/lestrrat-3d/decad/internal/meshbool"

	"github.com/lestrrat-3d/r3"
)

// This file is the build-time crossing audit of agent-docs/loft-design.md §6: the
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
// rational lift, never a tolerance. This is S6 (agent-docs/loft-design.md §4): the
// triangle has no interior, so the shell it would contribute to does not
// exist.
func TriangleCollapsed(verts []r3.Vec, tri [3]int) bool {
	if triangleOpenFloat(verts[tri[0]], verts[tri[1]], verts[tri[2]]) {
		return false
	}
	a, b, c := proof.XptOf(verts[tri[0]]), proof.XptOf(verts[tri[1]]), proof.XptOf(verts[tri[2]])
	n := proof.Xcross(proof.Xsub(b, a), proof.Xsub(c, a))
	return n.X.Sign() == 0 && n.Y.Sign() == 0 && n.Z.Sign() == 0
}

// LoftTriCorners reads one triangle's three float corners off the shared
// vertex table.
func LoftTriCorners(verts []r3.Vec, tri [3]int) [3]r3.Vec {
	return [3]r3.Vec{verts[tri[0]], verts[tri[1]], verts[tri[2]]}
}

// LoftAuditData holds the immutable per-vertex and per-triangle values shared
// by every pair in one crossing audit. The old pair path rebuilt the same
// exact vertex lifts and triangle normals for every pair, even though both are
// determined solely by the audit's input tables. The streamed path retains
// shared vertex projections and reconstructs triangle triples from them.
// Both layouts feed the same values to the exact pair predicates.
type LoftAuditData struct {
	Corners     [][3]r3.Vec
	Xverts      []proof.Xpt
	Xtris       [][3]proof.Xpt
	Norms       []proof.Xpt
	floatNorms  []r3.Vec
	Planes      []LoftExactPlane
	floatPlanes []proof.FloatOrientationPlane
	Projections [][3]meshbool.Xp2
	compact     bool
	verts       []r3.Vec
	tris        [][3]int
	projAxes    []uint8
	projected   [3][]meshbool.Xp2
	// certAxes, projUV and projFinite are the dense float copies of the
	// projections vertexCertificateFloat reads: per triangle the axis pair,
	// per (axis pair, vertex) the Fu, Fv of its Xp2 (two floats) and its
	// FloatFinite. They are built in the constructor and never written after.
	certAxes []uint8
	// exactLifts marks data built by NewLoftAuditDataWithExact, whose exact
	// lifts may differ from the float corners; vertexCertificateFloat is off
	// for it.
	exactLifts bool
	projUV     [3][]float64
	projFinite [3][]bool
	// fans is the exact angle table of the large flat fans, or nil. It is
	// built only by NewLoftAuditData and newCompactLoftAuditData, whose lifts
	// are the float vertices' own, and never written after.
	fans *loftAuditFans
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
	workers := loftDataWorkers(verts, len(tris))
	d := newLoftAuditDataSelected(verts, liftLoftVertices(verts, nil, workers), tris, nil, false, workers)
	d.fans = newLoftAuditFans(d, d.tris)
	return d
}

// NewLoftAuditDataWithExact audits a rational construction whose float
// corners are correctly rounded images of exact. The caller must widen any
// broad-phase boxes by its certified exact-to-float error before omitting a
// pair and use LoftAuditShortcuts.ExactInput to disable float-only filters.
func NewLoftAuditDataWithExact(verts []r3.Vec, exact []proof.Xpt, tris [][3]int) *LoftAuditData {
	d := newLoftAuditDataSelected(verts, cloneXpts(exact), tris, nil, false, loftDataWorkers(verts, len(tris)))
	d.exactLifts = true
	return d
}

// cloneXpts returns a copy of exact whose points own fresh big integers, so a
// caller that later writes to its points or to their integers cannot reach any
// table built from the copy.
func cloneXpts(exact []proof.Xpt) []proof.Xpt {
	out := make([]proof.Xpt, len(exact))
	for i, p := range exact {
		out[i] = proof.Xpt{X: cloneBigInt(p.X), Y: cloneBigInt(p.Y), Z: cloneBigInt(p.Z), W: cloneBigInt(p.W)}
	}
	return out
}

func cloneBigInt(x *big.Int) *big.Int {
	if x == nil {
		return nil
	}
	return new(big.Int).Set(x)
}

// newCompactLoftAuditData retains one projection per vertex and axis. The
// streamed S7 path reconstructs triangle triples from these immutable tables.
func newCompactLoftAuditData(verts []r3.Vec, tris [][3]int) *LoftAuditData {
	workers := loftDataWorkers(verts, len(tris))
	d := newLoftAuditDataSelected(verts, liftLoftVertices(verts, nil, workers), tris, nil, true, workers)
	d.fans = newLoftAuditFans(d, d.tris)
	return d
}

// newLoftAuditDataForCandidates keeps full-length index tables while lifting
// only triangles that S8 retained for pairwise classification. S6 and the
// projected patch proof have already checked every triangle before this call.
func newLoftAuditDataForCandidates(verts []r3.Vec, tris [][3]int, cands loftCandidates) *LoftAuditData {
	return newLoftAuditDataForCandidatesWorkers(verts, tris, cands, loftDataWorkers(verts, len(tris)))
}

func newLoftAuditDataForCandidatesWorkers(verts []r3.Vec, tris [][3]int, cands loftCandidates,
	workers int) *LoftAuditData {
	selected := make([]bool, len(tris))
	for i := range len(cands.starts) - 1 {
		if cands.starts[i] != cands.starts[i+1] {
			selected[i] = true
		}
	}
	for _, j := range cands.js {
		selected[j] = true
	}

	need := make([]bool, len(verts))
	for i, tri := range tris {
		if selected[i] {
			need[tri[0]], need[tri[1]], need[tri[2]] = true, true, true
		}
	}
	return newLoftAuditDataSelected(verts, liftLoftVertices(verts, need, workers), tris, selected, false, workers)
}

// loftDataParallelMin is the triangle count from which the audit tables are
// built on several goroutines.
const loftDataParallelMin = 2048

// loftDataWorkers is the worker count for the tables of a mesh with the given
// triangle count: one when any vertex is non-finite, so a lift that panics
// does so where the serial loop would.
func loftDataWorkers(verts []r3.Vec, triangles int) int {
	workers := orderedwork.EachWorkers(triangles, loftDataParallelMin)
	if workers >= 2 && !proofbound.AllFinite(verts) {
		return 1
	}
	return workers
}

// liftLoftVertices returns proof.XptOf of every vertex need marks (every
// vertex when need is nil); the others stay zero. Each worker writes only the
// slot of the vertex it owns.
func liftLoftVertices(verts []r3.Vec, need []bool, workers int) []proof.Xpt {
	exact := make([]proof.Xpt, len(verts))
	_, _ = orderedwork.Each(len(verts), workers, 512, nil, func(i int) error {
		if need == nil || need[i] {
			exact[i] = proof.XptOf(verts[i])
		}
		return nil
	})
	return exact
}

// newLoftAuditDataSelected builds the tables of the selected triangles (all
// when selected is nil). With workers of two or more, the per-triangle and
// per-vertex passes run on several goroutines; each pass writes only the slot
// of the item it owns, and the tables equal the serial ones.
func newLoftAuditDataSelected(verts []r3.Vec, exact []proof.Xpt, tris [][3]int,
	selected []bool, compact bool, workers int) *LoftAuditData {
	d := &LoftAuditData{
		Xverts:      exact,
		Norms:       make([]proof.Xpt, len(tris)),
		floatNorms:  make([]r3.Vec, len(tris)),
		Planes:      make([]LoftExactPlane, len(tris)),
		floatPlanes: make([]proof.FloatOrientationPlane, len(tris)),
		compact:     compact,
	}
	// The table owns a copy: a caller that reorders its own list afterwards
	// cannot reach the corners every certificate and the entry check read.
	d.tris = slices.Clone(tris)
	if compact {
		// The compact table reads float corners from verts for its whole life.
		d.verts = slices.Clone(verts)
	} else {
		d.Corners = make([][3]r3.Vec, len(tris))
		d.Xtris = make([][3]proof.Xpt, len(tris))
		d.Projections = make([][3]meshbool.Xp2, len(tris))
	}
	// Each triangle's tables depend on that triangle alone.
	projAxes := make([]uint8, len(tris))
	_, _ = orderedwork.Each(len(tris), workers, 256, nil, func(i int) error {
		if selected != nil && !selected[i] {
			return nil
		}
		tri := tris[i]
		xt := [3]proof.Xpt{d.Xverts[tri[0]], d.Xverts[tri[1]], d.Xverts[tri[2]]}
		if !compact {
			d.Corners[i] = LoftTriCorners(verts, tri)
			d.Xtris[i] = xt
		}
		d.Norms[i] = proof.XtriangleNormal(xt[0], xt[1], xt[2])
		d.floatNorms[i] = d.Norms[i].Vec()
		d.Planes[i] = NewLoftExactPlane(xt[0], d.Norms[i])
		corners := LoftTriCorners(verts, tri)
		d.floatPlanes[i] = proof.NewFloatOrientationPlane(corners[0], corners[1], corners[2])
		projAxes[i] = uint8(ProjectionPairIndex(meshbool.ProjAxes(d.Norms[i])))
		return nil
	})
	if compact {
		d.projAxes = projAxes
	}
	// Neighboring triangles share a vertex's exact projection on the same
	// plane: one value per (plane, vertex) that some selected triangle uses.
	var need [3][]bool
	for i, tri := range tris {
		if selected != nil && !selected[i] {
			continue
		}
		p := projAxes[i]
		if need[p] == nil {
			need[p] = make([]bool, len(exact))
		}
		need[p][tri[0]], need[p][tri[1]], need[p][tri[2]] = true, true, true
	}
	var projected [3][]meshbool.Xp2
	for p := range need {
		if need[p] != nil {
			projected[p] = make([]meshbool.Xp2, len(exact))
			d.projUV[p] = make([]float64, 2*len(exact))
			d.projFinite[p] = make([]bool, len(exact))
		}
	}
	d.certAxes = projAxes
	axes := [3][2]int{{0, 1}, {2, 0}, {1, 2}}
	_, _ = orderedwork.Each(len(exact), workers, 512, nil, func(vi int) error {
		for p := range need {
			if need[p] != nil && need[p][vi] {
				x := meshbool.NewXP2FromXpt(exact[vi], axes[p][0], axes[p][1])
				projected[p][vi] = x
				d.projUV[p][2*vi], d.projUV[p][2*vi+1] = x.Fu, x.Fv
				d.projFinite[p][vi] = x.FloatFinite
			}
		}
		return nil
	})
	if compact {
		d.projected = projected
		return d
	}
	for i, tri := range tris {
		if selected != nil && !selected[i] {
			continue
		}
		p := projAxes[i]
		for j, vi := range tri {
			d.Projections[i][j] = projected[p][vi]
		}
	}
	return d
}

func (d *LoftAuditData) corners(i int) [3]r3.Vec {
	if d.compact {
		return LoftTriCorners(d.verts, d.tris[i])
	}
	return d.Corners[i]
}

func (d *LoftAuditData) xtri(i int) [3]proof.Xpt {
	if d.compact {
		tri := d.tris[i]
		return [3]proof.Xpt{d.Xverts[tri[0]], d.Xverts[tri[1]], d.Xverts[tri[2]]}
	}
	return d.Xtris[i]
}

func (d *LoftAuditData) projection(i, corner int) meshbool.Xp2 {
	if d.compact {
		return d.projected[d.projAxes[i]][d.tris[i][corner]]
	}
	return d.Projections[i][corner]
}

func (d *LoftAuditData) projectionTriple(i int) [3]meshbool.Xp2 {
	return [3]meshbool.Xp2{d.projection(i, 0), d.projection(i, 1), d.projection(i, 2)}
}

// ErrLoftContact is S7 (agent-docs/loft-design.md §4/§6): the pair's exact contact
// is not the one its recorded adjacency expects, so the assembled shell
// self-touches or self-crosses away from — or instead of — its recorded
// vertex or edge. No such solid exists.
//
// The error is a *LoftContactError so a builder that numbers its triangles by
// face (sweep_mitre_build.go) can name the two faces instead of the indices.
func ErrLoftContact(i, j int, reason string) error {
	return &LoftContactError{I: i, J: j, Reason: reason}
}

// ErrLoftAuditOrder is the error AuditLoftPairData returns when the caller's
// corner list of triangle i or j is not equal, corner for corner and in order,
// to the list the table was built from. The table owns the corner order: every
// certificate pairs a corner position with the table's signs and corners.
var ErrLoftAuditOrder = errors.New("loft audit: triangle corner list differs from the audit table's")

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

// TriTriCoplanarSharedEdge is agent-docs/loft-design.md §6's audit-only helper for
// an edge-adjacent coplanar pair that meshbool.TriTriClassify reports as
// meshbool.ContactRegion. It reads the two exact triangles' opposite vertices and
// admits the pair only when the recorded edge (edgeA, edgeB) is an edge of
// BOTH triangles and their two opposite (apex) vertices lie strictly on
// opposite sides of that edge's supporting line — the exact condition under
// which the two triangles' closed intersection is precisely the recorded
// segment, never a shared area. It never writes to meshbool.TriTriClassify or any
// shared classification state, so it cannot change mesh-boolean contact
// classification (agent-docs/loft-design.md §6, required test).
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
	ExactInput   bool
}

// LoftAuditWork records what ONE crossing-audit call's S7 pair loop did,
// one field per way a pair can be decided:
//
//   - skips: a zero-shared-vertex pair a broad-phase tier proved apart.
//   - edgeCerts: a pair the noncoplanar shared-edge certificate admitted
//     (AuditLoftPairData's own doc comment carries its proof).
//   - vertexCerts: a pair a shared-vertex certificate (B, C or D) admitted.
//   - classifications: every other pair, which reached AuditLoftPairData
//     without a certificate deciding it.
//
// Candidates is the number of pairs the pair loop was handed: every pair of
// the triangles no cap proof decided, or only the box-overlapping ones under
// Sweep. It is the count S8 compares against the ceiling. Scanned is the
// counting pass's own work — sweep-axis comparisons, grid registrations and
// cell comparisons, or tree node visits and leaf comparisons (newPairScan), at least
// Candidates; S8 refuses as soon as it passes the ceiling, and it stops at
// ceiling + 1 on that refusal. CapProofs is how many cap families (0, 1 or 2)
// CapFamilyProof decided.
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
// loft (Document.Loft, Transform, and Copy through loftPayload.Placed),
// and two goroutines holding two
// independent Documents may each be inside it at once, so a counter shared
// across calls would both race and mis-count. Nothing outside the call frame
// is written anywhere on this path.
type LoftAuditWork struct {
	Skips           int
	EdgeCerts       int
	VertexCerts     int
	Classifications int
	Candidates      int
	Scanned         int
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

// loftPlaneSeparatedPrepared reads the audit's exact lifts and normals, using
// the adaptive float sign only when the lifts are the float corners' values.
func loftPlaneSeparatedPrepared(data *LoftAuditData, i, j int) bool {
	separated, _, _ := loftPlaneSeparatedSigns(data, i, j)
	return separated
}

// loftPlaneSeparatedSigns is loftPlaneSeparatedPrepared with the exact signs
// it decided handed back: sa is triangle i's corners against plane j and sb
// triangle j's against plane i, the same values TrianglePlaneSigns forms.
// Both triples are complete only when the pair is not separated.
func loftPlaneSeparatedSigns(data *LoftAuditData, i, j int) (bool, [3]int, [3]int) {
	ta, tb := data.corners(i), data.corners(j)
	xta, xtb := data.xtri(i), data.xtri(j)
	var sa, sb [3]int
	sb = data.planeSigns(i, xtb, tb)
	if meshbool.AllOneSide(sb) {
		return true, sa, sb
	}
	sa = data.planeSigns(j, xta, ta)
	return meshbool.AllOneSide(sa), sa, sb
}

// AuditLoftPairData classifies pair (i, j) and reports which decision path
// settled it. tris[i] and tris[j] must equal the triangles the table was built
// from, corner for corner and in order: the table owns the order, and any other
// list returns ErrLoftAuditOrder before a path runs. PRECONDITION: both
// triangles have three DISTINCT vertex indices
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
// meshbool.TriTriContactSpanCore, so the certificate reads work the pair was
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
//
// # Certificate D — a separating plane through the shared vertex
//
// A pair sharing exactly one vertex index is admitted once a plane through that
// vertex has one triangle's two remaining corners strictly on one side and the
// other triangle's two remaining corners strictly on the other. Convexity
// confines the intersection to the vertex. separatingPlaneCertificate owns the
// proof; it reads four float orientation signs, each certified by
// proof.OrientSignFloat, and any undecided or zero sign falls through to the
// path below.
func AuditLoftPairData(data *LoftAuditData, tris [][3]int, i, j int, shortcuts LoftAuditShortcuts) (LoftPairOutcome, error) {
	return auditLoftPairDataSigned(data, tris, i, j, shortcuts, nil, nil)
}

// auditLoftPairDataSigned is AuditLoftPairData for a pair whose exact plane
// signs the broad phase already decided (loftPlaneSeparatedSigns). The
// signs are read only for a pair sharing no vertex; nil means compute them.
func auditLoftPairDataSigned(data *LoftAuditData, tris [][3]int, i, j int, shortcuts LoftAuditShortcuts,
	knownA, knownB *[3]int) (LoftPairOutcome, error) {
	if tris[i] != data.tris[i] || tris[j] != data.tris[j] {
		return LoftPairClassified, fmt.Errorf("%w: pair %d,%d is %v,%v but the table holds %v,%v",
			ErrLoftAuditOrder, i, j, tris[i], tris[j], data.tris[i], data.tris[j])
	}
	shared, sharedCount := tessellation.SharedVertexIndices(tris[i], tris[j])
	if shortcuts.Certificates && sharedCount == 1 && data.fans != nil {
		// Both triangles are flat at the common vertex's z, so they are exactly
		// coplanar and the table's value is CoplanarIsolatedSharedVertex's.
		// A false value falls through to the unchanged path.
		if v, ok := data.fans.vertexCertificate(i, j, shared[0]); ok && v {
			return LoftPairVertexCertificate, nil
		}
	}
	if shortcuts.Certificates && sharedCount == 1 && data.vertexCertificateFloat(tris, i, j, shared[0]) {
		return LoftPairVertexCertificate, nil
	}
	if shortcuts.Certificates && sharedCount == 1 && data.separatingPlaneCertificate(tris[i], tris[j], i, j, shared[0]) {
		return LoftPairVertexCertificate, nil
	}
	ta := data.corners(i)
	tb := data.corners(j)
	xta := data.xtri(i)
	xtb := data.xtri(j)
	na := data.Norms[i]
	nb := data.Norms[j]
	if sharedCount == 2 {
		edgeA, edgeB := data.Xverts[shared[0]], data.Xverts[shared[1]]
		apex := data.Xverts[tessellation.TriangleApexIndex(tris[j], shared[0], shared[1])]
		if data.apexPlaneSign(ta, tb, xta, xtb, na, apex, edgeA) == 0 {
			if TriTriCoplanarSharedEdge(xta, xtb, na, edgeA, edgeB) {
				return LoftPairClassified, nil
			}
			return LoftPairClassified, ErrLoftContact(i, j, "do not meet exactly along their recorded shared edge")
		}
		if shortcuts.Certificates {
			return LoftPairEdgeCertificate, nil
		}
	}

	var signsB [3]int
	if knownB != nil && sharedCount == 0 {
		signsB = *knownB
	} else {
		signsB = data.planeSigns(i, xtb, tb)
	}
	if shortcuts.Certificates && sharedCount == 1 && meshbool.CountZero(signsB) == 3 &&
		CoplanarIsolatedSharedVertex(data, tris[i], tris[j], i, j, shared[0]) {
		return LoftPairVertexCertificate, nil
	}
	if shortcuts.Certificates && sharedCount == 1 && IsolatedSharedVertex(tris[j], shared[0], signsB) {
		return LoftPairVertexCertificate, nil
	}
	var signsA [3]int
	if knownA != nil && sharedCount == 0 {
		signsA = *knownA
	} else {
		signsA = data.planeSigns(j, xta, ta)
	}
	if shortcuts.Certificates && sharedCount == 1 && IsolatedSharedVertex(tris[i], shared[0], signsA) {
		return LoftPairVertexCertificate, nil
	}
	var projA, projB *[3]meshbool.Xp2
	var compactA, compactB [3]meshbool.Xp2
	if data.compact {
		compactA, compactB = data.projectionTriple(i), data.projectionTriple(j)
		projA, projB = &compactA, &compactB
	} else {
		projA, projB = &data.Projections[i], &data.Projections[j]
	}
	// The audit reads only the contact's kind and end points, so it asks for
	// the span alone: the same classification without the boundary, edge and
	// angle records of a full TriContact, which nothing below reads.
	span, err := meshbool.TriTriContactSpanCore(ta, tb, xta, xtb, na, nb,
		projA, projB, &signsA, &signsB, !data.exactLifts && !shortcuts.ExactInput,
		data.floatNorms[i], data.floatNorms[j], true)
	if err != nil {
		return LoftPairClassified, err
	}
	contact := meshbool.TriContact{Kind: span.Kind, P0: span.P0, P1: span.P1, EdgeA: -1, EdgeB: -1}

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

// vertexCertificateFloat replays the one-shared-vertex certificates
// (coplanar, then j isolated by plane i, then i isolated by plane j) on
// float-certain signs only. It returns true when one fires; false sends
// the pair down the unchanged exact path.
//
// The fast path is OFF (it returns false) for data built by
// NewLoftAuditDataWithExact: there the exact lifts may differ from the float
// corners, so a certain float sign need not agree with the exact geometry.
// For every other table the lifts are proof.XptOf of the float vertices, so
// the float corners and the exact points are the same values.
//
// Soundness on those tables: every sign it uses is one the exact path also
// uses, obtained the same way. A plane side is proof.OrientSignFloat's sign,
// which the exact path's planeSigns returns whenever that filter is certain
// (its error bound is stated on OrientSignFloat). The shared vertex's own
// corner is a corner of the plane's triangle, so its exact sign against that
// plane is identically 0 (n . (v - a) with v one of the triangle's exact
// corners); it is set to 0 without consulting its float sign. A line side is
// meshbool.Cross2xSign's float filter, run on the same Fu, Fv and FloatFinite
// values (copied from the same Xp2) with the same det, scale, 1e-12 and range
// tests, which Cross2xSign returns whenever it decides. Only a decided sign is
// consumed, and the certificate is the same boolean function of those signs as
// the exact path's, evaluated in the same order. An undecided sign returns
// false, so the pair takes the exact path.
func (d *LoftAuditData) vertexCertificateFloat(tris [][3]int, i, j, shared int) bool {
	if d.exactLifts {
		return false
	}
	ti, tj := d.corners(i), d.corners(j)
	signsB, ok := floatPlaneSigns(ti, tj, tris[j], shared)
	if !ok {
		return false
	}
	if meshbool.CountZero(signsB) == 3 {
		res, ok := d.coplanarVertexCertificateFloat(tris[i], tris[j], i, j, shared)
		if !ok {
			return false
		}
		if res {
			return true
		}
	}
	if IsolatedSharedVertex(tris[j], shared, signsB) {
		return true
	}
	signsA, ok := floatPlaneSigns(tj, ti, tris[i], shared)
	if !ok {
		return false
	}
	return IsolatedSharedVertex(tris[i], shared, signsA)
}

// floatPlaneSigns is the float-certain sign of each corner of other (whose
// vertex indices are otherTri) against the plane of plane's triangle, which
// has the corner with index shared in common. The shared corner reads 0
// unconditionally, as vertexCertificateFloat documents.
func floatPlaneSigns(plane, other [3]r3.Vec, otherTri [3]int, shared int) ([3]int, bool) {
	var signs [3]int
	for k := range signs {
		if otherTri[k] == shared {
			continue
		}
		s, ok := proof.OrientSignFloat(plane[0], plane[1], plane[2], other[k])
		if !ok {
			return signs, false
		}
		signs[k] = s
	}
	return signs, true
}

// coplanarVertexCertificateFloat is CoplanarIsolatedSharedVertex over the
// dense float projections. ok is false when a sign it needed was undecided.
func (d *LoftAuditData) coplanarVertexCertificateFloat(a, b [3]int, i, j, shared int) (bool, bool) {
	if len(d.certAxes) == 0 {
		return false, false
	}
	pa, pb := d.certAxes[i], d.certAxes[j]
	uvA, uvB := d.projUV[pa], d.projUV[pb]
	finA, finB := d.projFinite[pa], d.projFinite[pb]
	if uvA == nil || uvB == nil {
		return false, false
	}
	var aOther, bOther [2]int
	countA, countB := 0, 0
	for _, vertex := range a {
		if vertex != shared {
			aOther[countA] = vertex
			countA++
		}
	}
	for _, vertex := range b {
		if vertex != shared {
			bOther[countB] = vertex
			countB++
		}
	}
	for k := range 2 {
		side, ok := crossSignFloat(uvA, finA, shared, aOther[k], aOther[1-k], uvA, finA)
		if !ok {
			return false, false
		}
		if side == 0 {
			continue
		}
		s0, ok := crossSignFloat(uvA, finA, shared, aOther[k], bOther[0], uvB, finB)
		if !ok {
			return false, false
		}
		if s0 != -side {
			continue
		}
		s1, ok := crossSignFloat(uvA, finA, shared, aOther[k], bOther[1], uvB, finB)
		if !ok {
			return false, false
		}
		if s1 == -side {
			return true, true
		}
	}
	return false, true
}

// crossSignFloat is meshbool.Cross2xSign's float filter on dense vertex
// projections: points v and p come from the table (uv, fin), the third point c
// from (uvC, finC). The second result is false when the filter does not decide.
func crossSignFloat(uv []float64, fin []bool, va, vb, vc int, uvC []float64, finC []bool) (int, bool) {
	if !fin[va] || !fin[vb] || !finC[vc] {
		return 0, false
	}
	au, av := uv[2*va], uv[2*va+1]
	bu, bv := uv[2*vb], uv[2*vb+1]
	cu, cv := uvC[2*vc], uvC[2*vc+1]
	det := (bu-au)*(cv-av) - (bv-av)*(cu-au)
	scale := (proof.Abs(bu)+proof.Abs(au))*(proof.Abs(cv)+proof.Abs(av)) +
		(proof.Abs(bv)+proof.Abs(av))*(proof.Abs(cu)+proof.Abs(au))
	err := 1e-12 * scale
	if det != det || det > math.MaxFloat64 || det < -math.MaxFloat64 || err > math.MaxFloat64 || err <= 0 {
		return 0, false
	}
	if det > err {
		return 1, true
	}
	if det < -err {
		return -1, true
	}
	return 0, false
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
			v = data.projection(i, corner)
			continue
		}
		aOther[countA] = data.projection(i, corner)
		countA++
	}
	for corner, vertex := range b {
		if vertex != shared {
			bOther[countB] = data.projection(j, corner)
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

// IsolatedSharedVertex is certificate B's own reading (agent-docs/loft-design.md
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

// planeSigns evaluates the cached exact plane. The adaptive float filter may
// decide a sign only when the exact points are lifts of the float corners.
// An undecided sign uses that plane's affine numerator, avoiding a fresh
// homogeneous difference vector for every point of every pair.
// An identical table-owned point is one of the plane's corners and has sign
// zero without arithmetic. Equal coordinates with distinct lifts still use
// the adaptive predicate.
func (d *LoftAuditData) planeSigns(i int, xother [3]proof.Xpt, other [3]r3.Vec) [3]int {
	if d.exactLifts {
		return TrianglePlaneSigns(d.Planes[i], xother)
	}
	// The small private audit builds its own budgeted table without the
	// retained float cache. Prepare its plane on the stack for this call.
	var local proof.FloatOrientationPlane
	plane := &local
	if d.floatPlanes != nil {
		plane = &d.floatPlanes[i]
	} else {
		t := d.corners(i)
		local = proof.NewFloatOrientationPlane(t[0], t[1], t[2])
	}
	xt := d.xtri(i)
	var signs [3]int
	var sum, term big.Int
	for k := range signs {
		p := xother[k]
		if p == xt[0] || p == xt[1] || p == xt[2] {
			continue
		}
		if s, certain := plane.Sign(other[k]); certain {
			signs[k] = s
			continue
		}
		signs[k] = d.Planes[i].Sign(p, &sum, &term)
	}
	return signs
}

// apexPlaneSign is the sign of na . (apex - edgeA): which side of triangle i's
// plane the apex of triangle j lies on, for a pair sharing the edge that ends at
// edgeA (a corner of triangle i). Only whether it is zero is read.
//
// ti, tj and xti, xtj are the TABLE's corners of the two triangles, in the
// table's order (the order na was built in). The caller's vertex triples may
// list the same vertices in another order, so no position of the caller's triple
// is used: the apex and edgeA are bound to table corners by the identity of
// their exact points, which the table copies from Xverts (same big.Int
// pointers), and so by vertex index.
//
// na is (t1 - t0) x (t2 - t0) over the exact lifts of triangle i's corners in
// table order and edgeA is one of those corners, so na . (edgeA - t0) is exactly
// 0 and the value is na . (apex - t0) = det[t1 - t0, t2 - t0, apex - t0], the
// value proof.OrientSignFloat signs for (t0, t1, t2, apex). A sign that filter
// certifies is therefore the exact sign, and a certified zero is its
// exact-zero-column case. An undecided sign, an apex that is not a corner of the
// table's triangle j, or an edgeA that is not a corner of the table's triangle i
// falls through to the exact dot product. The float answer is off for data built
// by NewLoftAuditDataWithExact, whose exact lifts may differ from the float
// corners. For every other table the lifts are proof.XptOf of the float
// vertices, so a table corner and its exact point are the same value.
func (d *LoftAuditData) apexPlaneSign(ti, tj [3]r3.Vec, xti, xtj [3]proof.Xpt, na, apex, edgeA proof.Xpt) int {
	if !d.exactLifts && apex.X != nil && edgeA.X != nil &&
		(xti[0].X == edgeA.X || xti[1].X == edgeA.X || xti[2].X == edgeA.X) {
		for k := range xtj {
			if xtj[k].X != apex.X {
				continue
			}
			if s, ok := proof.OrientSignFloat(ti[0], ti[1], ti[2], tj[k]); ok {
				return s
			}
			break
		}
	}
	return proof.XdotSign(na, proof.Xsub(apex, edgeA))
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

// triangleOpenFloat proves (b-a)x(c-a) non-zero from float arithmetic alone,
// or answers false (undecided). Every coordinate must be zero or of magnitude
// in [2^-300, 2^300], so no difference or product below underflows or
// overflows; each normal component is then a 2D orientation determinant whose
// float evaluation errs by at most (3+16e)e of its float permanent (e = 2^-53,
// Shewchuk's ccwerrboundA). 2^-50 covers that, and a component whose float
// magnitude exceeds it is non-zero exactly. NaN and infinite inputs fail the
// range test and are left to the exact path.
func triangleOpenFloat(a, b, c r3.Vec) bool {
	for _, x := range [...]float64{a.X, a.Y, a.Z, b.X, b.Y, b.Z, c.X, c.Y, c.Z} {
		m := proof.Abs(x)
		if !(m == 0 || m >= 0x1p-300 && m <= 0x1p300) {
			return false
		}
	}
	ux, uy, uz := b.X-a.X, b.Y-a.Y, b.Z-a.Z
	vx, vy, vz := c.X-a.X, c.Y-a.Y, c.Z-a.Z
	return orientFloatProven(uy, uz, vy, vz) || orientFloatProven(uz, ux, vz, vx) ||
		orientFloatProven(ux, uy, vx, vy)
}

// orientFloatProven reports whether p*s - q*r is proven non-zero.
func orientFloatProven(p, q, r, s float64) bool {
	left, right := float64(p*s), float64(q*r)
	det := left - right
	bound := (proof.Abs(left) + proof.Abs(right)) * 0x1p-50
	return proof.Abs(det) > bound
}
