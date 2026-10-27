package loftmesh

import (
	"errors"
	"fmt"
	"math"
	"runtime"
	"slices"
	"sort"
	"sync"
	"sync/atomic"
	"time"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
)

// This file is the crossing audit's pair loop (agent-docs/loft-design.md §6): S6,
// the S8 ceiling over the candidates the loop will test, and S7 over them.
// sweepCandidates enumerates those candidates from the selected box scan, and
// loft_cap_proof.go's CapFamilyProof removes each cap's pairs from them.

// LoftAuditStructure is the loft's own split of its triangle set:
// tris[:Walls] are the wall triangles, the next CapStartCount the first cap's
// triangles, and the rest the second cap's. Loops0 and Loops1 are each cap's
// polygon loops as vertex-index cycles (Assembly's vIdx and wIdx). Interior
// lists vertices that belong to neither cap's loops but lie between the caps:
// the rings of a banded loft's interior sections. CapFamilyProof treats them
// as the opposite side of each cap, like the other cap's loop vertices. Nil
// for a two-ring loft.
type LoftAuditStructure struct {
	Walls          int
	CapStartCount  int
	Loops0, Loops1 [][]int
	Interior       []int
}

// loftGenericShortcuts is what LoftCrossingAudit runs, and
// loftStructuredShortcuts what LoftCrossingAuditStructured runs.
var (
	loftGenericShortcuts    = LoftAuditShortcuts{BroadPhase: true, Certificates: true, Sweep: true}
	loftStructuredShortcuts = LoftAuditShortcuts{BroadPhase: true, Certificates: true, Sweep: true, CapProof: true}
)

// LoftCrossingAudit is agent-docs/loft-design.md §6's whole build-time audit over
// an assembled triangle set with no structure of its own: S8's triangle
// ceiling (MaxLoftAuditTriangles) first, then S6 (per-triangle existence),
// then S8's pair ceiling (compared against the enumeration's work and the
// number of box-overlapping candidate pairs, before any pair test or
// pair-sized allocation), then S7 (the pair-by-pair contact audit over those
// candidates). budget is shared with the rest of the pre-commit cancellation
// path exactly as agent-docs/modify-design.md §5's audits already share one
// (fillet_audit.go); Step is called once per triangle in S6, once per scanned
// pair in each of the two sweep passes, and once per tested pair, and Err at
// every phase boundary, the end of S7 among them.
//
// sweep_mitre_build.go, stitch.go, mass_properties.go and LoftChain call
// it. A loft calls LoftCrossingAuditStructured instead. Tests that need a
// shortcut switched off, or the pair loop's own work counts, call
// LoftCrossingAuditWork. Large candidate lists classify concurrently and
// replay verdicts in pair order, preserving the first refusal and work counts.
func LoftCrossingAudit(budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int) error {
	_, err := loftCrossingAuditLimited(budget, verts, tris, nil, loftGenericShortcuts,
		proofbound.MaxFacetPairTestsPerCall, MaxLoftAuditTriangles, true)
	return err
}

// LoftCrossingAuditBounded runs the same exact, unstructured crossing audit
// as LoftCrossingAudit with caller-supplied resource ceilings. maxTriangles
// bounds the exact lifts held for the call; maxScans bounds both pair
// enumeration work and the number of box-overlapping candidates. A caller
// may choose larger ceilings for a constructed mesh whose size it separately
// bounds. The contact verdict and candidate selection do not depend on those
// ceilings. Large candidate sets use the ordered parallel classifier.
func LoftCrossingAuditBounded(budget *proofbound.WorkBudget, verts []r3.Vec,
	tris [][3]int, maxTriangles int, maxScans uint64) error {
	_, err := loftCrossingAuditLimited(budget, verts, tris, nil, loftGenericShortcuts,
		maxScans, maxTriangles, true)
	return err
}

// LoftCrossingAuditBoundedProjectedPatches first proves each supplied patch as
// a projected graph. It omits only within-patch pairs for successful proofs;
// cross-patch and patch-to-wall pairs retain the exact contact audit. A failed
// patch proof leaves that family's pairs in the audit. The caller's triangle
// and scan ceilings are unchanged. Boundary simplicity has its own fixed
// 100-million-check ceiling, shared by the supplied patches.
func LoftCrossingAuditBoundedProjectedPatches(budget *proofbound.WorkBudget,
	verts []r3.Vec, tris [][3]int, patches []ProjectedPatch,
	maxTriangles int, maxScans uint64) error {
	_, err := loftCrossingAuditLimited(budget, verts, tris, nil, loftGenericShortcuts,
		maxScans, maxTriangles, true, patches...)
	return err
}

// LoftCrossingAuditBoundedProjectedPatchesStreaming uses bounded row batches
// for a large candidate set. Callers use it only when retaining the complete
// projected-patch candidate list would exceed their memory allowance; its
// row preflight must independently fit within maxScans.
func LoftCrossingAuditBoundedProjectedPatchesStreaming(budget *proofbound.WorkBudget,
	verts []r3.Vec, tris [][3]int, patches []ProjectedPatch,
	maxTriangles int, maxScans uint64) error {
	_, err := loftCrossingAuditLimitedWithStreaming(budget, verts, tris, nil,
		loftGenericShortcuts, maxScans, maxTriangles, true, true, patches...)
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
// (agent-docs/loft-design.md §6): LoftCrossingAudit's three phases, with each cap
// whose CapFamilyProof holds decided as a whole. Every pair with a triangle
// in a proven cap leaves the pair loop and the S8 count, so S8 and S7 run
// over the wall-wall candidates plus the pairs of any cap whose proof failed.
// walls, capStartCount, loops0 and loops1 are LoftAuditStructure's fields.
func LoftCrossingAuditStructured(budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int, walls, capStartCount int, loops0, loops1 [][]int) error {
	structure := LoftAuditStructure{Walls: walls, CapStartCount: capStartCount, Loops0: loops0, Loops1: loops1}
	_, err := loftCrossingAudit(budget, verts, tris, &structure, loftStructuredShortcuts, proofbound.MaxFacetPairTestsPerCall)
	return err
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
// most triangles a loft at agent-docs/loft-gear-bounds-design.md §7's hard station
// ceiling of 8192 assembles (F = 4·Σstations + 4H − 4 with H ≤ Σstations − 1).
// It bounds the exact lifts NewLoftAuditData holds for the whole audit, which
// no candidate count bounds: far-apart triangles give no candidates at all.
const MaxLoftAuditTriangles = 8*8192 - 8

// ErrLoftAuditCeiling is wrapped by every refusal that the audit's work
// ceiling causes, so a caller that can shrink its mesh may retry with less.
var ErrLoftAuditCeiling = errors.New("loft crossing audit work ceiling")

func errLoftAuditCeiling() error {
	return fmt.Errorf(`%w: the loft crossing audit's candidate pair count exceeds the fixed work ceiling: %w`,
		decaderr.ErrUnsupported, ErrLoftAuditCeiling)
}

func errLoftAuditScanLimit(scanned, next, ceiling uint64) error {
	err := errLoftAuditCeiling()
	observed, ok := proofbound.WallCheckedAdd(scanned, next)
	if !ok {
		return err
	}
	return proofbound.WithWorkLimit(err, "pair_enumeration_steps", observed, ceiling)
}

func errLoftAuditTriangles(f int) error {
	return proofbound.WithWorkLimit(fmt.Errorf(`%w: the loft crossing audit's %d triangles exceed the fixed ceiling of %d`, decaderr.ErrUnsupported, f, MaxLoftAuditTriangles),
		"mesh_facets", uint64(f), MaxLoftAuditTriangles)
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
	return loftCrossingAuditLimited(budget, verts, tris, structure, shortcuts, ceiling, MaxLoftAuditTriangles, false)
}

func loftCrossingAuditLimited(budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int,
	structure *LoftAuditStructure, shortcuts LoftAuditShortcuts, ceiling uint64,
	triangleLimit int, parallelPairs bool, patches ...ProjectedPatch) (LoftAuditWork, error) {
	return loftCrossingAuditLimitedWithStreaming(budget, verts, tris, structure,
		shortcuts, ceiling, triangleLimit, parallelPairs, false, patches...)
}

func loftCrossingAuditLimitedWithStreaming(budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int,
	structure *LoftAuditStructure, shortcuts LoftAuditShortcuts, ceiling uint64,
	triangleLimit int, parallelPairs, streamProjected bool, patches ...ProjectedPatch) (LoftAuditWork, error) {
	var work LoftAuditWork
	if err := budget.Err(); err != nil {
		return work, err
	}
	f := len(tris)
	if f > triangleLimit {
		if triangleLimit == MaxLoftAuditTriangles {
			return work, errLoftAuditTriangles(f)
		}
		err := fmt.Errorf(`%w: the loft crossing audit's %d triangles exceed the fixed ceiling of %d`,
			decaderr.ErrUnsupported, f, triangleLimit)
		if triangleLimit >= 0 {
			return work, proofbound.WithWorkLimit(err, "mesh_facets", uint64(f), uint64(triangleLimit))
		}
		return work, err
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
	var patchFamily []int
	if len(patches) != 0 {
		patchFamily = make([]int, f)
		for i := range patchFamily {
			patchFamily[i] = -1
		}
	}
	provenPatches := 0
	remainingPatchChecks := uint64(100_000_000)
	for family, patch := range patches {
		if projectedPatchProofLimited(budget, verts, tris, patch,
			&remainingPatchChecks) != ProjectedPatchHolds {
			continue
		}
		if err := budget.Err(); err != nil {
			return work, err
		}
		unique := true
		for _, index := range patch.Triangles {
			if patchFamily[index] >= 0 {
				unique = false
				break
			}
		}
		if !unique {
			continue
		}
		for _, index := range patch.Triangles {
			patchFamily[index] = family
		}
		provenPatches++
	}
	if len(patches) != 0 {
		if err := budget.Err(); err != nil {
			return work, err
		}
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
	// sweep axis, the grid's registrations and in-cell comparisons, or the
	// tree's node visits and leaf comparisons
	// (newPairScan picks whichever is less) — a number at least the candidate
	// count, and refuses the moment that passes the ceiling, so the work
	// before an S8 refusal is O(F log² F + ceiling).
	var order pairScan
	var count, scanned uint64
	var starts []int
	if shortcuts.Sweep {
		if provenPatches != 0 {
			order = newGroupedTreeScan(boxes, members, patchFamily)
		} else {
			order = newPairScan(boxes, members, ceiling)
		}
		sc, err := sweepCandidateCounts(budget, order, f, keepEveryPair, ceiling)
		work.Scanned = int(sc.scanned)
		if err != nil {
			return work, err
		}
		if sc.exceeded {
			return work, proofbound.WithWorkLimit(errLoftAuditCeiling(), "pair_enumeration_steps", sc.scanned, ceiling)
		}
		starts, count, scanned = sc.starts, sc.count, sc.scanned
	} else {
		var ok bool
		count, ok = proofbound.WallChoose2(uint64(len(members)))
		if !ok {
			return work, errLoftAuditCeiling()
		}
	}
	if count > ceiling {
		return work, proofbound.WithWorkLimit(errLoftAuditCeiling(), "facet_pairs", count, ceiling)
	}
	work.Candidates = int(count)
	streamed := data == nil && structure == nil && shortcuts.Sweep &&
		(provenPatches == 0 || streamProjected) &&
		parallelPairs && count > loftAuditCandidateBatchSize
	var rowTree loftRowScan
	if streamed {
		if provenPatches != 0 {
			grouped, ok := order.(groupedTreeScan)
			if !ok {
				return work, fmt.Errorf("%w: proven loft patches lack a grouped scan", decaderr.ErrUnsupported)
			}
			rowTree = grouped
		} else {
			var ok bool
			rowTree, ok = order.(treeScan)
			if !ok {
				rowTree = newTreeScan(boxes, members)
			}
		}
		if err := preflightLoftRows(budget, rowTree, members, starts, ceiling); err != nil {
			return work, err
		}
	}
	var cands loftCandidates
	sparse := data == nil && structure == nil && shortcuts.Sweep && provenPatches != 0 && !streamed
	if sparse {
		var err error
		cands, err = sweepCandidatesByRows(budget, order, members, starts, count, scanned)
		if err != nil {
			return work, err
		}
		data = newLoftAuditDataForCandidates(verts, tris, cands)
	}
	if data == nil {
		if streamed {
			data = newCompactLoftAuditData(verts, tris)
		} else {
			data = NewLoftAuditData(verts, tris)
		}
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
		var knownA, knownB *[3]int
		if shortcuts.BroadPhase && sharedCount == 0 {
			if !meshbool.BoxesOverlap(boxes[i], boxes[j]) {
				work.Skips++
				return nil
			}
			separated, sa, sb := loftPlaneSeparatedSigns(data, i, j)
			if separated {
				work.Skips++
				return nil
			}
			knownA, knownB = &sa, &sb
		}
		outcome, err := auditLoftPairDataSigned(data, tris, i, j, shortcuts, knownA, knownB)
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
		if streamed {
			failI, failJ, err = streamLoftPairs(budget, rowTree, members, starts, count, ceiling,
				loftAuditCandidateBatchSize, tris, boxes, data, shortcuts, &work, testPair)
		} else if !sparse {
			cands, err = sweepCandidates(budget, order, starts, count, keepEveryPair)
		}
		if err == nil && !streamed {
			if parallelPairs && len(cands.js) >= 100_000 && runtime.GOMAXPROCS(0) > 1 {
				failI, failJ, err = parallelLoftPairs(budget, tris, boxes, data, shortcuts, cands, &work)
			} else {
				err = cands.each(testPair)
			}
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

// parallelLoftPairs classifies a large immutable candidate list concurrently.
// The shared budget and work counts are replayed in the
// original pair order, so the first refusal and all completed-call counts
// agree with the serial audit. Only the caller touches the budget closures.
func parallelLoftPairs(budget *proofbound.WorkBudget, tris [][3]int, boxes [][2]r3.Vec,
	data *LoftAuditData, shortcuts LoftAuditShortcuts, cands loftCandidates,
	work *LoftAuditWork) (int, int, error) {
	classify := func(i, j int) (uint8, error) {
		_, shared := tessellation.SharedVertexIndices(tris[i], tris[j])
		var knownA, knownB *[3]int
		if shortcuts.BroadPhase && shared == 0 {
			if !meshbool.BoxesOverlap(boxes[i], boxes[j]) {
				return pairSkipped, nil
			}
			separated, sa, sb := loftPlaneSeparatedSigns(data, i, j)
			if separated {
				return pairSkipped, nil
			}
			knownA, knownB = &sa, &sb
		}
		outcome, err := auditLoftPairDataSigned(data, tris, i, j, shortcuts, knownA, knownB)
		switch outcome {
		case LoftPairEdgeCertificate:
			return pairEdgeCertified, err
		case LoftPairVertexCertificate:
			return pairVertexCertified, err
		default:
			return pairClassified, err
		}
	}
	return parallelLoftPairsWorkers(budget, min(runtime.GOMAXPROCS(0), 4), cands, classify, work)
}

// Outcomes of one candidate pair in parallelLoftPairs.
const (
	pairSkipped uint8 = iota + 1
	pairClassified
	pairEdgeCertified
	pairVertexCertified
)

// parallelLoftPairsWorkers is parallelLoftPairs with an explicit worker count
// and the per-pair classification as classify, which workers call
// concurrently and which must not touch budget.
func parallelLoftPairsWorkers(budget *proofbound.WorkBudget, workers int, cands loftCandidates,
	classify func(i, j int) (uint8, error), work *LoftAuditWork) (int, int, error) {
	workers, reserved := orderedwork.ReserveWorkers(workers)
	defer orderedwork.ReleaseWorkers(reserved)
	results := make([]uint8, len(cands.js))
	type failure struct {
		index int
		err   error
	}
	failures := make([]failure, workers)
	// tallies[w] counts worker w's outcomes up to and including its first
	// failure, so the caller need not walk results again.
	tallies := make([]loftPairTally, workers)
	var stopped atomic.Bool
	var group sync.WaitGroup
	group.Add(workers)
	for worker := range workers {
		start := len(cands.js) * worker / workers
		end := len(cands.js) * (worker + 1) / workers
		failures[worker].index = end
		go func() {
			defer group.Done()
			var tally loftPairTally
			defer func() { tallies[worker] = tally }()
			i := sort.Search(len(cands.starts)-1, func(row int) bool {
				return cands.starts[row+1] > start
			}) + cands.baseRow
			for k := start; k < end; k++ {
				if k%proofbound.WorkPollInterval == 0 && stopped.Load() {
					return
				}
				for k >= cands.starts[i-cands.baseRow+1] {
					i++
				}
				outcome, err := classify(i, int(cands.js[k]))
				results[k] = outcome
				tally.add(outcome)
				if err != nil {
					failures[worker] = failure{index: k, err: err}
					return
				}
			}
		}()
	}
	done := make(chan struct{})
	go func() {
		group.Wait()
		close(done)
	}()
	ticker := time.NewTicker(50 * time.Millisecond)
	defer ticker.Stop()
	for running := true; running; {
		select {
		case <-done:
			running = false
		case <-ticker.C:
			if err := budget.Err(); err != nil {
				stopped.Store(true)
				<-done
				return -1, -1, err
			}
		}
	}
	first := len(cands.js)
	firstWorker := workers
	var firstErr error
	for w, f := range failures {
		if f.err != nil && f.index < first {
			first, firstErr, firstWorker = f.index, f.err, w
		}
	}
	if len(cands.js) == 0 {
		return -1, -1, nil
	}
	// The serial loop steps the budget once per pair up to and including the
	// failing one, and counts a pair's outcome after its step succeeds.
	last := min(first, len(cands.js)-1)
	stepped, err := budget.StepN(uint64(last + 1))
	if err != nil {
		var tally loftPairTally
		for _, r := range results[:stepped] {
			tally.add(r)
		}
		tally.addTo(work)
		return -1, -1, err
	}
	// Workers below the failing one ran to the end of their ranges; the
	// failing worker's tally stops at its failure.
	var total loftPairTally
	for w := range min(firstWorker+1, workers) {
		total.merge(tallies[w])
	}
	total.addTo(work)
	if first == len(cands.js) {
		return -1, -1, nil
	}
	i := sort.Search(len(cands.starts)-1, func(row int) bool { return cands.starts[row+1] > first })
	return cands.baseRow + i, int(cands.js[first]), firstErr
}

// loftPairTally counts parallelLoftPairs' pair outcomes.
type loftPairTally struct {
	skips, edgeCerts, vertexCerts, classifications int
}

func (t *loftPairTally) add(outcome uint8) {
	switch outcome {
	case pairSkipped:
		t.skips++
	case pairEdgeCertified:
		t.edgeCerts++
	case pairVertexCertified:
		t.vertexCerts++
	default:
		t.classifications++
	}
}

func (t *loftPairTally) merge(o loftPairTally) {
	t.skips += o.skips
	t.edgeCerts += o.edgeCerts
	t.vertexCerts += o.vertexCerts
	t.classifications += o.classifications
}

func (t loftPairTally) addTo(work *LoftAuditWork) {
	work.Skips += t.skips
	work.EdgeCerts += t.edgeCerts
	work.VertexCerts += t.vertexCerts
	work.Classifications += t.classifications
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

// sweepKey is one member's sort key: its lower bound on the sweep axis and
// its index.
type sweepKey struct {
	lo  float64
	idx int32
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
	// Sort keys carry the lower bound so the comparator loads no box.
	keys := make([]sweepKey, len(s.order))
	for k, i := range s.order {
		lo, _ := boxAxis(boxes[i], s.axis)
		keys[k] = sweepKey{lo: lo, idx: i}
	}
	slices.SortFunc(keys, func(a, b sweepKey) int {
		switch {
		case a.lo < b.lo:
			return -1
		case a.lo > b.lo:
			return 1
		default:
			return int(a.idx) - int(b.idx)
		}
	})
	for k, key := range keys {
		s.order[k] = key.idx
	}
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
// pairs (visit). A tree scan or grouped tree scan with enough members counts
// on up to orderedwork.MaxEachWorkers goroutines.
func sweepCandidateCounts(budget *proofbound.WorkBudget, s pairScan, f int, keep func(i, j int) bool, limit uint64) (sweepCount, error) {
	workers := 1
	switch scan := s.(type) {
	case treeScan:
		workers = orderedwork.EachWorkers(len(scan.order), loftTreeParallelMin)
	case groupedTreeScan:
		workers = orderedwork.EachWorkers(scan.members(), loftTreeParallelMin)
	}
	return sweepCandidateCountsWorkers(budget, s, f, keep, limit, workers)
}

// sweepCandidateCountsWorkers is sweepCandidateCounts with an explicit worker
// count. A tree scan or grouped tree scan with workers >= 2 and at least
// loftTreeParallelMin members counts in parallel; every other case is the
// serial walk.
func sweepCandidateCountsWorkers(budget *proofbound.WorkBudget, s pairScan, f int, keep func(i, j int) bool,
	limit uint64, workers int) (sweepCount, error) {
	if tree, ok := s.(treeScan); ok && workers >= 2 && len(tree.order) >= loftTreeParallelMin {
		return tree.countParallel(budget, f, keep, limit, workers)
	}
	if g, ok := s.(groupedTreeScan); ok && workers >= 2 && g.members() >= loftTreeParallelMin {
		return g.countParallel(budget, f, keep, limit, workers)
	}
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
	baseRow int
	starts  []int
	js      []int32
}

// sweepCandidates is the second sweep pass (agent-docs/loft-design.md §6): it
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

// loftRowPairScan is a scan that walks by pairs and by rows.
type loftRowPairScan interface {
	pairScan
	loftRowScan
}

// sweepCandidatesByRows is sweepCandidates for keepEveryPair. A row scan with
// at least loftRowParallelMin members fills the list by rows on several
// goroutines (sweepCandidatesRowsWorkers); every other scan, or fewer members,
// takes sweepCandidates' serial walk. scanned is the S8 count's figure for the
// same complete walk.
func sweepCandidatesByRows(budget *proofbound.WorkBudget, s pairScan, members, starts []int,
	count, scanned uint64) (loftCandidates, error) {
	workers := orderedwork.EachWorkers(len(members), loftRowParallelMin)
	if rows, ok := s.(loftRowPairScan); ok && workers >= 2 {
		return sweepCandidatesRowsWorkers(budget, rows, members, starts, count, scanned, workers)
	}
	return sweepCandidates(budget, s, starts, count, keepEveryPair)
}

// sweepCandidatesRowsWorkers fills sweepCandidates' list for keepEveryPair by
// rows. Row i's partners come from tree.walkRow, which passes exactly the
// partners visit pairs with i as the lower index. Each worker writes only row
// i's own slots js[starts[i]:starts[i+1]] and sorts them, so the list equals
// sweepCandidates'. Workers never touch budget: after the walks the caller
// steps budget scanned times, the units visit steps over the same complete
// walk, so a budget fails at the same step with the same error. A row whose
// partner count differs from starts is refused, and no partner is written
// outside its row's slots. If a cancellation poll stops the workers, the
// serial walk runs on the same budget, because the serial walk polls only every
// WorkPollInterval steps and may finish the list, or reach a later refusal,
// before it sees the cancellation.
func sweepCandidatesRowsWorkers(budget *proofbound.WorkBudget, tree loftRowPairScan, members, starts []int,
	count, scanned uint64, workers int) (loftCandidates, error) {
	c := loftCandidates{starts: starts, js: make([]int32, count)}
	mismatch := make([]bool, len(members))
	if _, err := orderedwork.Each(len(members), workers, 64, budget.Err, func(k int) error {
		i := members[k]
		row := c.js[starts[i]:starts[i]:starts[i+1]]
		n := 0
		_, exceeded := tree.walkRow(i, math.MaxUint64, func(j int) {
			if n < cap(row) {
				row = append(row, int32(j))
			}
			n++
		})
		if exceeded || n != starts[i+1]-starts[i] {
			mismatch[k] = true
			return nil
		}
		slices.Sort(row)
		return nil
	}); err != nil {
		return sweepCandidates(budget, tree, starts, count, keepEveryPair)
	}
	if _, err := budget.StepN(scanned); err != nil {
		return loftCandidates{}, err
	}
	if slices.Contains(mismatch, true) {
		return loftCandidates{}, fmt.Errorf("%w: row-ordered loft candidates differ from S8 count",
			decaderr.ErrUnsupported)
	}
	return c, nil
}

// each calls fn for every candidate in order and stops at fn's first error.
func (c loftCandidates) each(fn func(i, j int) error) error {
	for i := range len(c.starts) - 1 {
		for _, j := range c.js[c.starts[i]:c.starts[i+1]] {
			if err := fn(c.baseRow+i, int(j)); err != nil {
				return err
			}
		}
	}
	return nil
}
