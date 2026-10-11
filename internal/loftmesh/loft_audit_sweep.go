package loftmesh

import (
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
)

// This file is the crossing audit's pair loop (docs/loft-design.md §6): S6,
// the S8 ceiling over the candidates the loop will test, and S7 over them.
// sweepCandidates enumerates those candidates by sweep-and-prune, and
// loft_cap_proof.go's CapFamilyProof removes each cap's pairs from them.

// LoftAuditStructure is the loft's own split of its triangle set:
// tris[:Walls] are the wall triangles, the next CapStartCount the first cap's
// triangles, and the rest the second cap's. Loops0 and Loops1 are each cap's
// polygon loops as vertex-index cycles (loftAssembly's vIdx and wIdx).
type LoftAuditStructure struct {
	Walls          int
	CapStartCount  int
	Loops0, Loops1 [][]int
}

// loftGenericShortcuts is what LoftCrossingAudit runs, and
// loftStructuredShortcuts what LoftCrossingAuditStructured runs.
var (
	loftGenericShortcuts    = LoftAuditShortcuts{BroadPhase: true, Certificates: true, Sweep: true}
	loftStructuredShortcuts = LoftAuditShortcuts{BroadPhase: true, Certificates: true, Sweep: true, CapProof: true}
)

// LoftCrossingAudit is docs/loft-design.md §6's whole build-time audit over
// an assembled triangle set with no structure of its own: S8's triangle
// ceiling (MaxLoftAuditTriangles) first, then S6 (per-triangle existence),
// then S8's pair ceiling (compared against the enumeration's work and the
// number of box-overlapping candidate pairs, before any pair test or
// pair-sized allocation), then S7 (the pair-by-pair contact audit over those
// candidates). budget is shared with the rest of the pre-commit cancellation
// path exactly as docs/modify-design.md §5's audits already share one
// (fillet_audit.go); Step is called once per triangle in S6, once per scanned
// pair in each of the two sweep passes, and once per tested pair, and Err at
// every phase boundary, the end of S7 among them.
//
// sweep_mitre_build.go, stitch.go, mass_properties_mesh.go and LoftChain call
// it. A loft calls LoftCrossingAuditStructured instead. Tests that need a
// shortcut switched off, or the pair loop's own work counts, call
// LoftCrossingAuditWork.
func LoftCrossingAudit(budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int) error {
	_, err := loftCrossingAudit(budget, verts, tris, nil, loftGenericShortcuts, proofbound.MaxFacetPairTestsPerCall)
	return err
}

// LoftCrossingAuditBevelJoin audits the dense root-ring and tooth-cap mesh.
// Its larger, fixed scan limit admits the circumferential root rings while
// retaining the generic audit's triangle limit and exact pair decisions.
func LoftCrossingAuditBevelJoin(budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int) error {
	const maxBevelJoinScans = 64_000_000
	_, err := loftCrossingAudit(budget, verts, tris, nil, loftGenericShortcuts, maxBevelJoinScans)
	return err
}

// LoftCrossingAuditBevelTwoJoin uses four times the one-tooth scan ceiling:
// a second cap set can double the triangle count, and pair scans grow with
// its square. It keeps the same triangle ceiling and exact contact verdicts.
func LoftCrossingAuditBevelTwoJoin(budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int) error {
	const maxBevelTwoJoinScans = 256_000_000
	_, err := loftCrossingAudit(budget, verts, tris, nil, loftGenericShortcuts, maxBevelTwoJoinScans)
	return err
}

// LoftCrossingAuditWork is LoftCrossingAudit's body, with each S7 shortcut
// under its own explicit per-call switch (LoftAuditShortcuts) and the pair
// loop's own work counts returned to the caller. CapProof has no effect here:
// no structure is supplied. The zero shortcuts value runs every pair through
// meshbool.TriTriClassify's exact classification, which is the verdict a
// shortcut may only ever reach sooner, never change.
//
// The returned counts are meaningful only when the audit completes: an early
// return carries whatever the loop had reached when it stopped.
func LoftCrossingAuditWork(budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int, shortcuts LoftAuditShortcuts) (LoftAuditWork, error) {
	return loftCrossingAudit(budget, verts, tris, nil, shortcuts, proofbound.MaxFacetPairTestsPerCall)
}

// LoftCrossingAuditStructured is the loft's crossing audit
// (docs/loft-design.md §6): LoftCrossingAudit's three phases, with each cap
// whose CapFamilyProof holds decided as a whole. Every pair with a triangle
// in a proven cap leaves the pair loop and the S8 count, so S8 and S7 run
// over the wall-wall candidates plus the pairs of any cap whose proof failed.
// walls, capStartCount, loops0 and loops1 are LoftAuditStructure's fields.
func LoftCrossingAuditStructured(budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int, walls, capStartCount int, loops0, loops1 [][]int) error {
	structure := LoftAuditStructure{Walls: walls, CapStartCount: capStartCount, Loops0: loops0, Loops1: loops1}
	_, err := loftCrossingAudit(budget, verts, tris, &structure, loftStructuredShortcuts, proofbound.MaxFacetPairTestsPerCall)
	return err
}

// LoftCrossingAuditStructuredWork is LoftCrossingAuditStructured's body with
// explicit shortcuts and an explicit S8 ceiling, for tests: production always
// passes proofbound.MaxFacetPairTestsPerCall.
func LoftCrossingAuditStructuredWork(budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int, structure LoftAuditStructure, shortcuts LoftAuditShortcuts, ceiling uint64) (LoftAuditWork, error) {
	return loftCrossingAudit(budget, verts, tris, &structure, shortcuts, ceiling)
}

// LoftSweepCandidates returns the generic audit's candidate list over every
// triangle, in the order its pair loop tests them. It is the tests' view of
// sweepCandidates; no production path calls it.
func LoftSweepCandidates(budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int) ([][2]int, error) {
	boxes := make([][2]r3.Vec, len(tris))
	members := make([]int, len(tris))
	for i, tri := range tris {
		boxes[i] = meshbool.TriBox(verts, tri)
		members[i] = i
	}
	order := newPairScan(boxes, members, math.MaxUint64)
	sc, err := sweepCandidateCounts(budget, order, len(tris), keepEveryPair, math.MaxUint64)
	if err != nil {
		return nil, err
	}
	starts, count := sc.starts, sc.count
	cands, err := sweepCandidates(budget, order, starts, count, keepEveryPair)
	if err != nil {
		return nil, err
	}
	out := make([][2]int, 0, count)
	_ = cands.each(func(i, j int) error {
		out = append(out, [2]int{i, j})
		return nil
	})
	return out, nil
}

// MaxLoftAuditTriangles is S8's ceiling on the triangle count one crossing
// audit accepts, checked before S6 lifts a single triangle: 8·8192 − 8, the
// most triangles a loft at docs/loft-gear-bounds-design.md §7's hard station
// ceiling of 8192 assembles (F = 4·Σstations + 4H − 4 with H ≤ Σstations − 1).
// It bounds the exact lifts NewLoftAuditData holds for the whole audit, which
// no candidate count bounds: far-apart triangles give no candidates at all.
const MaxLoftAuditTriangles = 8*8192 - 8

func errLoftAuditCeiling() error {
	return fmt.Errorf(`%w: the loft crossing audit's candidate pair count exceeds the fixed work ceiling`, decaderr.ErrUnsupported)
}

func errLoftAuditTriangles(f int) error {
	return fmt.Errorf(`%w: the loft crossing audit's %d triangles exceed the fixed ceiling of %d`, decaderr.ErrUnsupported, f, MaxLoftAuditTriangles)
}

// loftCrossingAudit is every entry point's body. structure is nil for the
// generic entry.
//
// # Which pairs the loop tests
//
// A triangle is DECIDED when it belongs to a cap whose CapFamilyProof holds,
// and a pair is left to that proof when either of its triangles is decided:
// the proof covers the cap's pairs with the walls, with its own triangles and
// with the other cap (its doc comment). Every other pair is a member pair.
// Without Sweep the loop tests every member pair; with it, only the member
// pairs whose boxes overlap (sweepCandidates). Either way the pairs are
// tested in lexicographic (i, j) order, the reference path's own order.
//
// # The refused pair
//
// The reference path reports the lexicographically first failing pair. With
// no cap decided the loop tests a subset of the reference's pairs in the same
// order, and every pair it leaves out the reference admits, so its first
// failure is the reference's. A cap proof leans on the wall-wall audit for
// its loops' simplicity, so once a member pair fails nothing is known about
// the decided pairs. The loop then tests every decided pair (box-overlapping
// under Sweep) that precedes the failing pair, in order, and returns the
// first failure among them, or the original one. That lookback runs only when
// its own candidate count fits under the ceiling beside the loop's; past it
// the audit returns the original failure, which is still S7.
func loftCrossingAudit(budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int, structure *LoftAuditStructure,
	shortcuts LoftAuditShortcuts, ceiling uint64) (LoftAuditWork, error) {
	var work LoftAuditWork
	if err := budget.Err(); err != nil {
		return work, err
	}
	f := len(tris)
	if f > MaxLoftAuditTriangles {
		return work, errLoftAuditTriangles(f)
	}

	// S6: per-triangle existence, before the pair audit runs at all.
	for i, tri := range tris {
		if err := budget.Step(); err != nil {
			return work, err
		}
		if TriangleCollapsed(verts, tri) {
			return work, fmt.Errorf(`%w: loft triangle %d has collapsed to zero area`, decaderr.ErrDegenerate, i)
		}
	}
	if err := budget.Err(); err != nil {
		return work, err
	}

	// The cap proofs read the exact lifts, so a structured audit builds them
	// before S8's candidate count. The triangle ceiling checked above is what
	// bounds them there: the loft's own station cap (S15) does not, since
	// StationCapGate never consults it for a build with no chorded pair.
	// The generic entry builds them only after S8 admits the call.
	var data *LoftAuditData
	var proven [2]bool
	if shortcuts.CapProof && structure != nil {
		data = NewLoftAuditData(verts, tris)
		for c := range proven {
			proven[c] = CapFamilyProof(data, tris, *structure, c) == LoftCapProofHolds
			if proven[c] {
				work.CapProofs++
			}
		}
	}
	decided := func(i int) bool {
		if work.CapProofs == 0 || i < structure.Walls {
			return false
		}
		if i < structure.Walls+structure.CapStartCount {
			return proven[0]
		}
		return proven[1]
	}
	members := make([]int, 0, f)
	for i := range f {
		if !decided(i) {
			members = append(members, i)
		}
	}

	// The broad-phase per-triangle bounding box (boolean_mesh.go's
	// meshbool.TriBox, the same helper prepBoolMesh builds for the mesh
	// boolean's own facet-pair pruning) feeds both the sweep and the
	// broad-phase tier. meshbool.TriBox's own doc comment establishes the box
	// is exact — "float min/max are exact, so the box is a true bound" —
	// built from float64 vertex coordinates that are themselves exact inputs
	// to proof.XptOf, so no epsilon widening is needed or added: every point
	// of the closed triangle is a convex combination of its three vertices,
	// and a convex combination of values bounded by [lo, hi] on one axis is
	// itself bounded by [lo, hi] on that axis. meshbool.BoxesOverlap's own
	// comparisons (<=, not <) are exact float64 comparisons, so two boxes it
	// reports as NOT overlapping are proven to share no point on some axis —
	// the two closed triangles cannot touch at all.
	boxes := make([][2]r3.Vec, f)
	for i, tri := range tris {
		boxes[i] = meshbool.TriBox(verts, tri)
	}

	// S8: the ceiling, over the candidates the pair loop will test, refused
	// before a single pair test runs and before any candidate list is built.
	// The enumeration's counting pass is itself held to the ceiling: it counts
	// its own work — the sweep's comparisons of two boxes that overlap on the
	// sweep axis, or the grid's registrations and in-cell comparisons
	// (newPairScan picks whichever is less) — a number at least the candidate
	// count, and refuses the moment that passes the ceiling, so the work
	// before an S8 refusal is O(F log F + ceiling).
	var order pairScan
	var count uint64
	var starts []int
	if shortcuts.Sweep {
		order = newPairScan(boxes, members, ceiling)
		sc, err := sweepCandidateCounts(budget, order, f, keepEveryPair, ceiling)
		work.Scanned = int(sc.scanned)
		if err != nil {
			return work, err
		}
		if sc.exceeded {
			return work, errLoftAuditCeiling()
		}
		starts, count = sc.starts, sc.count
	} else {
		var ok bool
		count, ok = proofbound.WallChoose2(uint64(len(members)))
		if !ok {
			return work, errLoftAuditCeiling()
		}
	}
	if count > ceiling {
		return work, errLoftAuditCeiling()
	}
	work.Candidates = int(count)
	if data == nil {
		data = NewLoftAuditData(verts, tris)
	}

	// S7: every candidate, classified against its recorded adjacency. The
	// broad-phase short-circuit runs ONLY for a pair sharing no recorded
	// vertex (sharedCount == 0): that is the one case (AuditLoftPairData's
	// switch) where the audit's own passing verdict is "no contact at all",
	// so a proof of no contact reaches the identical verdict the exact
	// classification would reach. A pair sharing one or two vertices is
	// REQUIRED to touch — at that vertex, or along that edge — so the guard
	// below never lets the short-circuit run for it.
	//
	// Two tiers run, cheapest first, either one sufficient to skip: the
	// bounding-box test, then LoftPlaneSeparated for a pair whose boxes do
	// overlap but whose planes still prove separation. Both are float-only
	// and reject-only; neither ever answers "touching", so a pair either tier
	// cannot decide always falls through to AuditLoftPairData, which owns the
	// complementary accept-only certificates and reports which path decided
	// the pair.
	failI, failJ := -1, -1
	testPair := func(i, j int) error {
		if err := budget.Step(); err != nil {
			return err
		}
		_, sharedCount := tessellation.SharedVertexIndices(tris[i], tris[j])
		if shortcuts.BroadPhase && sharedCount == 0 {
			if !meshbool.BoxesOverlap(boxes[i], boxes[j]) {
				work.Skips++
				return nil
			}
			if LoftPlaneSeparated(data.Corners[i], data.Corners[j]) {
				work.Skips++
				return nil
			}
		}
		outcome, err := AuditLoftPairData(data, tris, i, j, shortcuts)
		switch outcome {
		case LoftPairEdgeCertificate:
			work.EdgeCerts++
		case LoftPairVertexCertificate:
			work.VertexCerts++
		default:
			work.Classifications++
		}
		if err != nil {
			failI, failJ = i, j
		}
		return err
	}

	var err error
	if shortcuts.Sweep {
		var cands loftCandidates
		cands, err = sweepCandidates(budget, order, starts, count, keepEveryPair)
		if err == nil {
			err = cands.each(testPair)
		}
	} else {
		err = eachMemberPair(members, testPair)
	}
	if err != nil {
		if failI < 0 || work.CapProofs == 0 {
			return work, err
		}
		return work, loftAuditLookback(budget, boxes, f, shortcuts.Sweep, decided, failI, failJ, ceiling, &work, testPair, err)
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
	// cancelled operation leaves the receiver live and the document
	// unchanged — is discharged at the commit edge by the entry point, the way
	// fillet.go, chamfer.go and shell.go each check ctx.Err() immediately
	// before Document.commit. Loft's own commit-edge check belongs to
	// Loft.
	return work, budget.Err()
}

// loftAuditLookback tests every decided pair that precedes the failing pair
// (failI, failJ), in lexicographic order, and returns the first failure among
// them, or failed when none fails (loftCrossingAudit's "The refused pair").
func loftAuditLookback(budget *proofbound.WorkBudget, boxes [][2]r3.Vec, f int, sweep bool, decided func(int) bool,
	failI, failJ int, ceiling uint64, work *LoftAuditWork, testPair func(i, j int) error, failed error) error {
	keep := func(i, j int) bool {
		if !decided(i) && !decided(j) {
			return false
		}
		return i < failI || (i == failI && j < failJ)
	}
	all := make([]int, f)
	for i := range all {
		all[i] = i
	}
	if !sweep {
		if pairs, ok := proofbound.WallChoose2(uint64(f)); !ok || uint64(work.Candidates)+pairs > ceiling {
			return failed
		}
		count := uint64(0)
		if err := eachMemberPair(all, func(i, j int) error {
			if keep(i, j) {
				count++
			}
			return nil
		}); err != nil {
			return err
		}
		if uint64(work.Candidates)+count > ceiling {
			return failed
		}
		work.Candidates += int(count)
		err := eachMemberPair(all, func(i, j int) error {
			if !keep(i, j) {
				return nil
			}
			return testPair(i, j)
		})
		if err != nil {
			return err
		}
		return failed
	}
	order := newPairScan(boxes, all, ceiling-uint64(work.Candidates))
	sc, err := sweepCandidateCounts(budget, order, f, keep, ceiling-uint64(work.Candidates))
	if err != nil {
		return err
	}
	starts, count := sc.starts, sc.count
	if sc.exceeded || uint64(work.Candidates)+count > ceiling {
		return failed
	}
	work.Candidates += int(count)
	cands, err := sweepCandidates(budget, order, starts, count, keep)
	if err != nil {
		return err
	}
	if err := cands.each(testPair); err != nil {
		return err
	}
	return failed
}

// eachMemberPair calls fn for every pair of members (ascending), in
// lexicographic order, and stops at fn's first error.
func eachMemberPair(members []int, fn func(i, j int) error) error {
	for a, i := range members {
		for _, j := range members[a+1:] {
			if err := fn(i, j); err != nil {
				return err
			}
		}
	}
	return nil
}

func keepEveryPair(int, int) bool { return true }

// sweepOrder is one sweep-and-prune pass's input: the member triangles'
// indices sorted by their box's lower bound on the axis of largest total
// extent, ties by index.
type sweepOrder struct {
	boxes [][2]r3.Vec
	order []int32
	axis  int
}

func boxAxis(b [2]r3.Vec, axis int) (float64, float64) {
	switch axis {
	case 0:
		return b[0].X, b[1].X
	case 1:
		return b[0].Y, b[1].Y
	default:
		return b[0].Z, b[1].Z
	}
}

func newSweepOrder(boxes [][2]r3.Vec, members []int) sweepOrder {
	s := sweepOrder{boxes: boxes, order: make([]int32, len(members))}
	if len(members) == 0 {
		return s
	}
	lo, hi := boxes[members[0]][0], boxes[members[0]][1]
	for k, i := range members {
		s.order[k] = int32(i)
		b := boxes[i]
		lo = r3.Vec{X: min(lo.X, b[0].X), Y: min(lo.Y, b[0].Y), Z: min(lo.Z, b[0].Z)}
		hi = r3.Vec{X: max(hi.X, b[1].X), Y: max(hi.Y, b[1].Y), Z: max(hi.Z, b[1].Z)}
	}
	extent := [3]float64{hi.X - lo.X, hi.Y - lo.Y, hi.Z - lo.Z}
	for k := 1; k < 3; k++ {
		if extent[k] > extent[s.axis] {
			s.axis = k
		}
	}
	slices.SortFunc(s.order, func(a, b int32) int {
		la, _ := boxAxis(boxes[a], s.axis)
		lb, _ := boxAxis(boxes[b], s.axis)
		switch {
		case la < lb:
			return -1
		case la > lb:
			return 1
		default:
			return int(a) - int(b)
		}
	})
	return s
}

// visit calls fn once for every pair of members whose boxes overlap on all
// three axes, as (i, j) with i < j, in no particular order. Walking the
// sorted order, the inner scan for member a stops at the first member whose
// lower bound on the sweep axis lies past a's upper bound: every later member
// starts at least as far along that axis, so none of them overlaps a there,
// and a pair whose boxes are disjoint on one axis is disjoint. Every
// overlapping pair is therefore reached from whichever of its two triangles
// comes first in the order. Both passes are float comparisons only.
//
// Every scanned pair — one whose boxes overlap on the sweep axis — steps the
// budget once and counts toward limit; visit stops and reports true as soon
// as the count passes limit. That count is at least the number of pairs fn
// receives, and it is what bounds the pass's work: a tall shape whose boxes
// all overlap on the sweep axis scans F·(F−1)/2 pairs however few overlap on
// all three.
func (s sweepOrder) visit(budget *proofbound.WorkBudget, limit uint64, fn func(i, j int)) (uint64, bool, error) {
	scanned := uint64(0)
	for a, ia := range s.order {
		boxA := s.boxes[ia]
		_, hiA := boxAxis(boxA, s.axis)
		for _, ib := range s.order[a+1:] {
			boxB := s.boxes[ib]
			if loB, _ := boxAxis(boxB, s.axis); loB > hiA {
				break
			}
			if err := budget.Step(); err != nil {
				return scanned, false, err
			}
			scanned++
			if scanned > limit {
				return scanned, true, nil
			}
			if !meshbool.BoxesOverlap(boxA, boxB) {
				continue
			}
			i, j := int(ia), int(ib)
			if i > j {
				i, j = j, i
			}
			fn(i, j)
		}
	}
	return scanned, false, nil
}

// sweepCandidateCounts is the first sweep pass: it counts the overlapping
// pairs keep accepts, per lower index, and returns the offsets the second
// pass fills (starts[i] is where triangle i's partners begin; len f+1) and
// the total, which is the count S8 compares, beside the number of pairs the
// pass scanned. It reports true instead when the pass scans more than limit
// pairs (visit).
func sweepCandidateCounts(budget *proofbound.WorkBudget, s pairScan, f int, keep func(i, j int) bool, limit uint64) (sweepCount, error) {
	starts := make([]int, f+1)
	scanned, exceeded, err := s.visit(budget, limit, func(i, j int) {
		if keep(i, j) {
			starts[i+1]++
		}
	})
	if err != nil || exceeded {
		return sweepCount{scanned: scanned, exceeded: exceeded}, err
	}
	for i := 1; i <= f; i++ {
		starts[i] += starts[i-1]
	}
	return sweepCount{starts: starts, count: uint64(starts[f]), scanned: scanned}, nil
}

// sweepCount is sweepCandidateCounts' result.
type sweepCount struct {
	starts   []int
	count    uint64
	scanned  uint64
	exceeded bool
}

// loftCandidates is a candidate list in lexicographic order: triangle i's
// partners are js[starts[i]:starts[i+1]], ascending.
type loftCandidates struct {
	starts []int
	js     []int32
}

// sweepCandidates is the second sweep pass (docs/loft-design.md §6): it
// fills the list sweepCandidateCounts sized, then sorts each triangle's
// partners, so the list runs in the same lexicographic order the all-pairs
// loop tests in and the first refused pair is the one that loop reports. It
// scans exactly the pairs the first pass scanned, which that pass already
// held to its limit.
func sweepCandidates(budget *proofbound.WorkBudget, s pairScan, starts []int, count uint64, keep func(i, j int) bool) (loftCandidates, error) {
	c := loftCandidates{starts: starts, js: make([]int32, count)}
	next := slices.Clone(starts[:len(starts)-1])
	_, _, err := s.visit(budget, math.MaxUint64, func(i, j int) {
		if !keep(i, j) {
			return
		}
		c.js[next[i]] = int32(j)
		next[i]++
	})
	if err != nil {
		return loftCandidates{}, err
	}
	for i := range len(starts) - 1 {
		slices.Sort(c.js[starts[i]:starts[i+1]])
	}
	return c, nil
}

// each calls fn for every candidate in order and stops at fn's first error.
func (c loftCandidates) each(fn func(i, j int) error) error {
	for i := range len(c.starts) - 1 {
		for _, j := range c.js[c.starts[i]:c.starts[i+1]] {
			if err := fn(i, int(j)); err != nil {
				return err
			}
		}
	}
	return nil
}
