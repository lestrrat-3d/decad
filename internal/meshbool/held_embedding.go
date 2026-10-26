package meshbool

import (
	"context"
	"encoding/binary"
	"fmt"
	"math"
	"math/big"
	"runtime"
	"slices"
	"sync"
	"sync/atomic"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// This file keeps a held faceted mesh embedded through its float rounding
// (agent-docs/evaluator-design.md §9, "The final rounding to float64 keeps the mesh
// embedded"). The exact mesh a boolean stitches, or the exact affine image a
// placement moves, is embedded. Rounding each exact vertex to binary64 moves it
// by up to an ulp, and where a vertex sits closer than that to an edge or facet
// it does not touch, the rounded mesh can fold through itself. Only a facet
// with a vertex that moved can meet anything differently from the exact mesh,
// so only those facets are checked, against every held facet near them, with
// the exact predicates over the held binary64 values. A vertex on a failing
// pair may move to another float of its exact point's float box; a mesh still
// failing afterwards is refused.

// HeldRounding is one held mesh under the embedding check: the rounded float
// vertices it holds, its facets, and per held vertex the exact point it stands
// for. Moved marks a held vertex that differs from what it stands for — rounded
// off its exact point, or welded from several exact points — and Movable marks
// one the search may place at another corner of its exact point's float box.
// A Movable vertex must stand for exactly one exact point, Exact[v], and be
// Moved.
type HeldRounding struct {
	Verts   []r3.Vec
	Tris    [][3]int
	Exact   []proof.Xpt
	Moved   []bool
	Movable []bool
	// Reach is how far a Movable vertex may sit from its exact point once the
	// float-box corners have failed to clear it, in millimetres. Zero keeps
	// the search to the float box.
	Reach float64
	// ScanAxes optionally selects linear projections for the first read-only
	// box-tree scan. Outward interval dot products bound every held corner;
	// disjoint projected boxes prove separation without orthogonality.
	// Zero axes keep world boxes. Repair always uses the original world grid.
	ScanAxes [3]r3.Vec
}

// heldEmbeddingSweeps caps the passes over the offending vertices. A pass that
// moves nothing ends the search early; the cap only bounds the work.
const heldEmbeddingSweeps = 16

// heldGridCellSpan is the most grid cells one facet registers in. A facet
// spanning more (a long trunk side, a large cap) joins the list every query
// scans, so one huge facet cannot cost a cell per unit of its length.
const heldGridCellSpan = 512

// EnforceHeldEmbedding proves the held mesh embedded wherever rounding moved
// it, moving offending vertices to other float-box corners when that is what
// it takes, and refuses (ErrUnsupported) a mesh still folded after that search. It
// updates h.Verts in place and returns how many vertices it moved off their
// first rounding.
//
// The rule is deterministic. Each pass visits the movable vertices of the
// failing pairs in ascending held index, and each moves to the corner of its
// float box that leaves the fewest failing pairs among its own facets, when
// that is fewer than where it sits (improve). Passes repeat over the facets
// that failed or that a move touched until one moves nothing; the facets
// that are still failing then are refused.
func EnforceHeldEmbedding(ctx context.Context, h HeldRounding) (int, error) {
	return enforceHeldEmbedding(ctx, h, true)
}

// enforceHeldEmbedding is EnforceHeldEmbedding with the per-facet box cache
// and the pair memo switched by memo. Both only replay answers the same
// positions already gave, so the result is the same either way; the tests
// run both paths and compare them.
func enforceHeldEmbedding(ctx context.Context, h HeldRounding, memo bool) (int, error) {
	moved, _, err := enforceHeldEmbeddingWithSummary(ctx, h, memo)
	return moved, err
}

func enforceHeldEmbeddingWithSummary(ctx context.Context, h HeldRounding, memo bool) (int, *EmbeddingReach, error) {
	var summary EmbeddingReach
	moved, failing, err := repairHeldEmbeddingSummary(ctx, h, memo, &summary)
	if err != nil {
		return 0, nil, err
	}
	if len(failing) > 0 {
		return 0, nil, fmt.Errorf(`%w: the result's float rounding folds its held mesh through itself, and the bounded float-box search did not clear it — the held mesh would not be embedded, so this evaluator refuses the result rather than publish it`, decaderr.ErrUnsupported)
	}
	if summary.DisplacementBound == 0 {
		return moved, nil, nil
	}
	return moved, &summary, nil
}

// RepairHeldEmbedding runs the float-box search of EnforceHeldEmbedding and
// returns, instead of refusing, the facets still folded after it, in ascending
// order, with the number of vertices it moved. A caller that can still change
// the mesh uses them to find what to change.
func RepairHeldEmbedding(ctx context.Context, h HeldRounding) (int, []int, error) {
	return repairHeldEmbedding(ctx, h, true)
}

func repairHeldEmbedding(ctx context.Context, h HeldRounding, memo bool) (int, []int, error) {
	return repairHeldEmbeddingSummary(ctx, h, memo, nil)
}

func repairHeldEmbeddingSummary(ctx context.Context, h HeldRounding, memo bool, summary *EmbeddingReach) (int, []int, error) {
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return 0, nil, err
	}
	var query []int
	for i, t := range h.Tris {
		if err := budget.Step(); err != nil {
			return 0, nil, err
		}
		if h.Moved[t[0]] || h.Moved[t[1]] || h.Moved[t[2]] {
			query = append(query, i)
		}
	}
	if len(query) == 0 {
		return 0, nil, nil
	}
	e := newHeldEmbeddingScan(h, budget, memo)
	// The tree reads fixed facet boxes. Only a failed first scan needs the
	// padded grid that keeps covering facets while repair moves vertices.
	parallelFirst := memo && len(query) >= 4096 && runtime.GOMAXPROCS(0) > 1
	if parallelFirst {
		clean, err := e.parallelClean(ctx, query)
		if err != nil {
			return 0, nil, err
		}
		if clean {
			return 0, nil, nil
		}
	}
	if err := e.buildGrid(); err != nil {
		return 0, nil, err
	}
	// Every facet outside check is known embedded against all its
	// neighbours; a move puts the facets around the moved vertex back in.
	check := query
	moved := map[int]struct{}{}
	// Stage 0 searches the float box of each exact point. A mesh it leaves
	// folded gets stage 1 when Reach allows, which also tries positions up to
	// Reach away (improveWithinReach) over a grid padded for them.
	for stage := range 2 {
		if stage == 1 {
			e.reachOn = true
			if err := e.buildGrid(); err != nil {
				return 0, nil, err
			}
		}
		for sweep := range heldEmbeddingSweeps {
			if stage == 0 && sweep == 0 && memo && !parallelFirst {
				queryFacet := make([]bool, len(e.h.Tris))
				for _, i := range check {
					queryFacet[i] = true
				}
				// The scan counts its own steps: a discarded scan must not move
				// the repair path's step count or polling points.
				repairBudget := e.budget
				e.budget = proofbound.NewWorkBudget(ctx)
				clean, err := e.serialClean(check, queryFacet)
				e.budget = repairBudget
				if err != nil {
					return 0, nil, err
				}
				if clean {
					return 0, nil, nil
				}
			}
			e.prepareRepair()
			if memo && e.pairs == nil {
				// Repair needs replayable pair answers; a clean first scan does not.
				e.pairs = make(map[uint64]heldPair, 32*len(query))
			}
			offenders, failing, err := e.offenders(check)
			if err != nil {
				return 0, nil, err
			}
			if len(failing) == 0 {
				if err := e.summarizeReach(ctx, summary); err != nil {
					return 0, nil, err
				}
				return len(moved), nil, nil
			}
			next := map[int]struct{}{}
			for _, f := range failing {
				next[f] = struct{}{}
			}
			changed := false
			for _, v := range offenders {
				placed, err := e.improve(v)
				if err != nil {
					return 0, nil, err
				}
				if !placed {
					continue
				}
				moved[v] = struct{}{}
				changed = true
				for _, f := range e.inc[v] {
					next[f] = struct{}{}
				}
			}
			check = make([]int, 0, len(next))
			for f := range next {
				check = append(check, f)
			}
			slices.Sort(check)
			if !changed {
				break
			}
		}
		_, failing, err := e.offenders(check)
		if err != nil {
			return 0, nil, err
		}
		slices.Sort(failing)
		if len(failing) == 0 || stage == 1 || e.h.Reach <= 0 {
			if len(failing) == 0 {
				if err := e.summarizeReach(ctx, summary); err != nil {
					return 0, nil, err
				}
			}
			return len(moved), failing, nil
		}
		check = failing
	}
	if err := e.summarizeReach(ctx, summary); err != nil {
		return 0, nil, err
	}
	return len(moved), nil, nil
}

// HeldFolds returns the facets of the held mesh that meet a facet other than
// their neighbours, among those that touch a Moved vertex, each with the
// facets it meets, in ascending order of the first. It moves nothing; Movable
// is ignored. A mesh with no Moved vertex has none.
func HeldFolds(ctx context.Context, h HeldRounding) ([][2]int, error) {
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, err
	}
	var query []int
	for i, t := range h.Tris {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		if h.Moved[t[0]] || h.Moved[t[1]] || h.Moved[t[2]] {
			query = append(query, i)
		}
	}
	if len(query) == 0 {
		return nil, nil
	}
	e := newHeldEmbeddingScan(h, budget, true)
	if err := e.buildGrid(); err != nil {
		return nil, err
	}
	e.pairs = make(map[uint64]heldPair, 32*len(query))
	var out [][2]int
	for _, f := range query {
		bad, collapsed, err := e.facetFailures(f)
		if err != nil {
			return nil, err
		}
		if collapsed {
			out = append(out, [2]int{f, f})
		}
		for _, g := range bad {
			out = append(out, [2]int{f, g})
		}
	}
	return out, nil
}

// parallelClean tests each box-overlapping pair involving a query facet once. A
// failing pair sends the caller through the original ordered repair scan.
// Each worker polls after at most 256/workers operations, so their combined
// unchecked work stays below one polling interval.
func (e *heldEmbedding) parallelClean(ctx context.Context, query []int) (bool, error) {
	workers := min(runtime.GOMAXPROCS(0), 4)
	if err := e.budget.Err(); err != nil {
		return false, err
	}
	queryFacet := make([]bool, len(e.h.Tris))
	queryCount := 0
	for _, i := range query {
		if !queryFacet[i] {
			queryCount++
		}
		queryFacet[i] = true
	}
	allQuery := queryCount == len(e.h.Tris) && queryCount != 0
	tree, err := newHeldScanTree(ctx, e)
	if err != nil {
		return false, err
	}
	if !allQuery {
		if err := tree.prepareQueryCeilings(ctx, queryFacet); err != nil {
			return false, err
		}
	}
	// The first scan cannot move vertices. Build its projected certificates
	// once before workers share them; repair invalidates incident facets only
	// after every worker has joined.
	for i := range e.h.Tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return false, err
			}
		}
		e.projection(i)
		// Alternate scan axes do not populate the world boxes. Workers also
		// read those boxes, so prepare them before sharing their cache.
		e.triBox(i)
		if allQuery && !e.facetOpenFloat(i) {
			if _, _, n := e.facet(i); zeroXpt(n) {
				return false, nil
			}
		}
	}
	var tasks []heldNodePair
	if allQuery {
		tasks, err = tree.selfPairTasks(ctx, 16*workers)
		if err != nil {
			return false, err
		}
	}
	clean := make([]bool, workers)
	errors := make([]error, workers)
	locals := make([]*heldEmbedding, workers)
	e.pollErr = ctx.Err
	e.pollInterval = proofbound.WorkPollInterval / workers
	e.pollRemaining = e.pollInterval
	locals[0] = e
	for worker := 1; worker < workers; worker++ {
		locals[worker] = e.readOnlyScanCopy(ctx, workers)
	}
	var group sync.WaitGroup
	// Nearby facets can have very different candidate counts. Small shared
	// chunks keep one dense patch from occupying a worker for the whole scan.
	var next atomic.Int64
	chunkSize, count := 32, len(query)
	if allQuery {
		chunkSize, count = 1, len(tasks)
	}
	group.Add(workers)
	for worker := range workers {
		local := locals[worker]
		go func() {
			defer group.Done()
			clean[worker] = true
			for {
				start := int(next.Add(int64(chunkSize))) - chunkSize
				if start >= count {
					return
				}
				end := min(start+chunkSize, count)
				if allQuery {
					clean[worker], errors[worker] = local.cleanSelfPairs(tasks[start:end], &tree)
				} else {
					clean[worker], errors[worker] = local.cleanPairs(query[start:end], queryFacet, &tree)
				}
				if !clean[worker] || errors[worker] != nil {
					return
				}
			}
		}()
	}
	group.Wait()
	e.pollErr = nil
	if err := e.budget.Err(); err != nil {
		return false, err
	}
	for worker := range workers {
		if errors[worker] != nil {
			return false, errors[worker]
		}
		if !clean[worker] {
			return false, nil
		}
	}
	return true, nil
}

// facetOpenFloat reports whether facet i's held corners are proven not
// collinear by floats alone, so its exact normal is non-zero.
func (e *heldEmbedding) facetOpenFloat(i int) bool {
	t := e.h.Tris[i]
	return proof.TriangleOpenFloat(e.h.Verts[t[0]], e.h.Verts[t[1]], e.h.Verts[t[2]])
}

// pairApartFloat proves, from certain float orientation signs alone, the
// answers pairSeparated would give as (true, true) for one shared edge or one
// shared vertex. False decides nothing.
func (e *heldEmbedding) pairApartFloat(i, j int, sharedI, sharedJ []int) bool {
	ti, tj, vs := e.h.Tris[i], e.h.Tris[j], e.h.Verts
	switch len(sharedI) {
	case 2:
		c := ti[3-sharedI[0]-sharedI[1]]
		d := tj[3-sharedJ[0]-sharedJ[1]]
		s, ok := proof.OrientSignFloat(vs[ti[sharedI[0]]], vs[ti[sharedI[1]]], vs[c], vs[d])
		return ok && s != 0
	case 1:
		// sideI reads j's unshared corners against i's plane, sideJ the
		// reverse. A certain non-zero reading also proves the plane's own
		// facet is not collinear.
		sideI, openI := e.strictSideFloat(ti, tj, sharedJ[0])
		sideJ, openJ := e.strictSideFloat(tj, ti, sharedI[0])
		openI = openI || e.facetOpenFloat(i)
		openJ = openJ || e.facetOpenFloat(j)
		return openI && openJ && (sideI || sideJ)
	}
	return false
}

// strictSideFloat reports whether the two corners of u other than slot skip
// lie strictly on one side of t's plane by certain float signs, and whether
// any certain non-zero sign was read (so t's corners are not collinear).
func (e *heldEmbedding) strictSideFloat(t, u [3]int, skip int) (bool, bool) {
	vs := e.h.Verts
	side, open, strict := 0, false, true
	for k, q := range u {
		if k == skip {
			continue
		}
		s, ok := proof.OrientSignFloat(vs[t[0]], vs[t[1]], vs[t[2]], vs[q])
		if ok && s != 0 {
			open = true
		}
		if !ok || s == 0 || side != 0 && s != side {
			strict = false
		}
		side = s
	}
	return strict, open
}

func (e *heldEmbedding) cleanPairs(query []int, queryFacet []bool, tree *heldBoxTree) (bool, error) {
	// This scan runs only on a parallel worker. Keep its polling countdown on
	// the worker's stack rather than writing the predicate-cache object on
	// every node and candidate. Save it across chunks, including early exits.
	remaining, interval, poll := e.pollRemaining, e.pollInterval, e.pollErr
	defer func() { e.pollRemaining = remaining }()
	step := func() error {
		remaining--
		if remaining != 0 {
			return nil
		}
		remaining = interval
		return poll()
	}
	stack := make([]int, 0, 64)
	oriented := tree.oriented
	for _, i := range query {
		if !e.facetOpenFloat(i) {
			if _, _, n := e.facet(i); zeroXpt(n) {
				return false, nil
			}
		}
		box := tree.boxes[i]
		if len(tree.nodes) == 0 {
			continue
		}
		stack = append(stack[:0], 0)
		for len(stack) > 0 {
			last := len(stack) - 1
			nodeIndex := stack[last]
			node := tree.nodes[nodeIndex]
			stack = stack[:last]
			if err := step(); err != nil {
				return false, err
			}
			if len(tree.queryCeiling) > 0 && tree.queryCeiling[nodeIndex] <= i {
				continue
			}
			if !BoxesOverlap(box, node.box) {
				continue
			}
			if node.leafStart >= 0 {
				for _, j := range tree.facets[node.leafStart : node.leafStart+node.leafCount] {
					if err := step(); err != nil {
						return false, err
					}
					if j == i || j < i && queryFacet[j] || !BoxesOverlap(box, tree.boxes[j]) {
						continue
					}
					if oriented && !BoxesOverlap(e.triBox(i), e.triBox(j)) {
						continue
					}
					if !e.pairEmbeddedOverlapping(i, j) {
						return false, nil
					}
				}
				continue
			}
			stack = append(stack, node.right, node.left)
		}
	}
	return true, nil
}

// scanStep preserves the parallel scan's per-worker polling interval without
// an indirect callback for every candidate in its innermost loop.
func (e *heldEmbedding) scanStep() error {
	if e.pollErr == nil {
		return e.budget.Step()
	}
	e.pollRemaining--
	if e.pollRemaining != 0 {
		return nil
	}
	e.pollRemaining = e.pollInterval
	return e.pollErr()
}

// readOnlyScanCopy shares the immutable facet boxes, prepared projections and
// incident-facet lists. Each worker writes only its own remaining predicate caches.
func (e *heldEmbedding) readOnlyScanCopy(ctx context.Context, workers int) *heldEmbedding {
	local := *e
	local.pollErr = ctx.Err
	local.pollInterval = proofbound.WorkPollInterval / workers
	local.pollRemaining = local.pollInterval
	local.xv = make([]proof.Xpt, len(e.xv))
	local.xvOK = make([]bool, len(e.xvOK))
	local.norm = make([]proof.Xpt, len(e.norm))
	local.normOK = make([]bool, len(e.normOK))
	local.plane = make([]int, len(e.plane))
	local.planeSense = make([]int8, len(e.planeSense))
	for i := range local.plane {
		local.plane[i] = -1
	}
	local.planes = map[string]int{}
	local.fnorm = make([]r3.Vec, len(e.fnorm))
	local.fnormOK = make([]bool, len(e.fnormOK))
	local.cone = make([][3][2]r3.Vec, len(e.cone))
	local.coneOK = make([][3]bool, len(e.coneOK))
	local.sideSigns = make(map[[2]int]int)
	local.stamp = nil
	local.cands = nil
	// parallelClean filled every box before starting its workers. Repair
	// starts only after all workers have joined.
	local.box = e.box
	local.boxOK = e.boxOK
	local.pairs = nil
	return &local
}

type heldEmbedding struct {
	h      HeldRounding
	budget *proofbound.WorkBudget
	// pollErr is set only during the read-only parallel first scan. Each
	// worker has its own countdown; later repair uses budget again.
	pollErr       func() error
	pollInterval  int
	pollRemaining int
	// inc and taken are needed only when the ordered repair may move vertices.
	inc   [][]int
	taken map[r3.Vec]int

	// Exact lifts of the held float vertices and facets, invalidated when a
	// vertex moves.
	xv     []proof.Xpt
	xvOK   []bool
	norm   []proof.Xpt
	normOK []bool
	// plane is each facet's exact plane, interned so two facets are coplanar
	// exactly when their ids match; -1 is not yet computed, -2 no plane.
	plane      []int
	planeSense []int8
	planes     map[string]int
	fnorm      []r3.Vec
	// fnormOK marks fnorm[i] as the float rounding of norm[i], computed on demand.
	fnormOK []bool
	// cone caches, per facet and corner, a box around the unit directions
	// the facet leaves that corner in (coneBox).
	cone   [][3][2]r3.Vec
	coneOK [][3]bool
	// projected holds each facet's three coordinate projections and their
	// stable directed-edge signs. Worker copies own these mutable caches.
	projected   []heldProjectedFacet
	projectedOK []bool
	// sideSigns stores a vertex's sign against the canonical normal of an
	// exact plane. Read-only scan copies can share one reading across facets
	// in that plane; repair keeps this nil because vertices can move.
	sideSigns map[[2]int]int
	stamp     []int
	stampID   int
	cands     []int
	// reachOn turns on the reach moves of improveWithinReach. The search sets
	// it once the float box has done what it can.
	reachOn bool
	// reachUsed names vertices whose final accepted position came from the
	// positive-reach stage, never positions visited only by a trial.
	reachUsed map[int]struct{}

	// memoOn enables the two caches below. box is each facet's float box
	// (TriBox), invalidated when a vertex moves. pairs replays pairEmbedded
	// for an unordered pair of facets: gen stamps each facet's current
	// vertex positions, a move gives every incident facet a fresh stamp, and
	// an entry holds only while both facets carry the stamps it was stored
	// under. pairEmbedded is symmetric in its facets and depends on nothing
	// but their six vertex positions, so a replayed answer is the answer.
	memoOn  bool
	box     [][2]r3.Vec
	boxOK   []bool
	gen     []uint64
	nextGen uint64
	pairs   map[uint64]heldPair

	// The grid every query reads, built over facet boxes padded by pad so a
	// vertex's move inside its float box never leaves its facets' cells.
	pad    float64
	origin r3.Vec
	cell   float64
	dims   [3]int
	cells  [][]int32
	wide   []int32
}

// heldPair is one replayable pairEmbedded answer: the stamps of the lower and
// the higher facet when it was decided, and whether the pair was embedded.
type heldPair struct {
	genLo, genHi uint64
	ok           bool
}

func newHeldEmbedding(h HeldRounding, budget *proofbound.WorkBudget, memo bool) *heldEmbedding {
	e := newHeldEmbeddingScan(h, budget, memo)
	e.prepareRepair()
	return e
}

// newHeldEmbeddingScan allocates predicate caches without the vertex incidence
// lists and occupied-position map used only by repair.
func newHeldEmbeddingScan(h HeldRounding, budget *proofbound.WorkBudget, memo bool) *heldEmbedding {
	e := &heldEmbedding{
		h:           h,
		budget:      budget,
		memoOn:      memo,
		xv:          make([]proof.Xpt, len(h.Verts)),
		xvOK:        make([]bool, len(h.Verts)),
		norm:        make([]proof.Xpt, len(h.Tris)),
		normOK:      make([]bool, len(h.Tris)),
		plane:       make([]int, len(h.Tris)),
		planeSense:  make([]int8, len(h.Tris)),
		fnorm:       make([]r3.Vec, len(h.Tris)),
		fnormOK:     make([]bool, len(h.Tris)),
		cone:        make([][3][2]r3.Vec, len(h.Tris)),
		coneOK:      make([][3]bool, len(h.Tris)),
		projected:   make([]heldProjectedFacet, len(h.Tris)),
		projectedOK: make([]bool, len(h.Tris)),
		planes:      map[string]int{},
		stamp:       make([]int, len(h.Tris)),
	}
	for i := range e.plane {
		e.plane[i] = -1
	}
	if memo {
		e.box = make([][2]r3.Vec, len(h.Tris))
		e.boxOK = make([]bool, len(h.Tris))
		e.gen = make([]uint64, len(h.Tris))
		e.nextGen = 1
	}
	return e
}

func (e *heldEmbedding) prepareRepair() {
	if e.inc != nil {
		return
	}
	e.inc = make([][]int, len(e.h.Verts))
	e.taken = make(map[r3.Vec]int, len(e.h.Verts))
	for i, t := range e.h.Tris {
		for _, v := range t {
			e.inc[v] = append(e.inc[v], i)
		}
	}
	for v, p := range e.h.Verts {
		e.taken[p] = v
	}
}

// triBox is facet i's float box, TriBox over the held vertices.
func (e *heldEmbedding) triBox(i int) [2]r3.Vec {
	if !e.memoOn {
		return TriBox(e.h.Verts, e.h.Tris[i])
	}
	if !e.boxOK[i] {
		e.box[i] = TriBox(e.h.Verts, e.h.Tris[i])
		e.boxOK[i] = true
	}
	return e.box[i]
}

// pairOK is pairEmbedded, replayed from the pair memo when neither facet has
// changed since the pair was last decided.
func (e *heldEmbedding) pairOK(i, j int) bool {
	if !e.memoOn {
		return e.pairEmbedded(i, j)
	}
	if !BoxesOverlap(e.triBox(i), e.triBox(j)) {
		return true
	}
	lo, hi := min(i, j), max(i, j)
	key := uint64(lo)<<32 | uint64(hi)
	if p, ok := e.pairs[key]; ok && p.genLo == e.gen[lo] && p.genHi == e.gen[hi] {
		return p.ok
	}
	ok := e.pairEmbeddedOverlapping(i, j)
	e.pairs[key] = heldPair{genLo: e.gen[lo], genHi: e.gen[hi], ok: ok}
	return ok
}

// stamps copies the current stamps of v's incident facets.
func (e *heldEmbedding) stamps(v int) []uint64 {
	out := make([]uint64, len(e.inc[v]))
	for k, f := range e.inc[v] {
		out[k] = e.gen[f]
	}
	return out
}

// restamp gives v's incident facets back the stamps saved by stamps, which
// is sound only while every vertex of those facets is where it was then.
func (e *heldEmbedding) restamp(v int, saved []uint64) {
	for k, f := range e.inc[v] {
		e.gen[f] = saved[k]
	}
}

func (e *heldEmbedding) lift(v int) proof.Xpt {
	if !e.xvOK[v] {
		e.xv[v] = proof.XptOf(e.h.Verts[v])
		e.xvOK[v] = true
	}
	return e.xv[v]
}

func (e *heldEmbedding) facet(i int) ([3]r3.Vec, [3]proof.Xpt, proof.Xpt) {
	t := e.h.Tris[i]
	f := [3]r3.Vec{e.h.Verts[t[0]], e.h.Verts[t[1]], e.h.Verts[t[2]]}
	x := [3]proof.Xpt{e.lift(t[0]), e.lift(t[1]), e.lift(t[2])}
	if !e.normOK[i] {
		e.norm[i] = proof.Xcross(proof.Xsub(x[1], x[0]), proof.Xsub(x[2], x[0]))
		e.normOK[i] = true
	}
	return f, x, e.norm[i]
}

// floatNorm is facet i's exact normal rounded to float64, computed on first
// use: only the triangle-pair classifier reads it.
func (e *heldEmbedding) floatNorm(i int) r3.Vec {
	if !e.fnormOK[i] {
		_, _, n := e.facet(i)
		e.fnorm[i] = n.Vec()
		e.fnormOK[i] = true
	}
	return e.fnorm[i]
}

func (e *heldEmbedding) setVert(v int, p r3.Vec) {
	delete(e.taken, e.h.Verts[v])
	e.h.Verts[v] = p
	e.taken[p] = v
	e.xvOK[v] = false
	for _, f := range e.inc[v] {
		e.normOK[f] = false
		e.fnormOK[f] = false
		e.plane[f] = -1
		e.coneOK[f] = [3]bool{}
		e.projectedOK[f] = false
		if e.memoOn {
			e.boxOK[f] = false
			e.gen[f] = e.nextGen
			e.nextGen++
		}
	}
}

func zeroXpt(n proof.Xpt) bool {
	return n.X.Sign() == 0 && n.Y.Sign() == 0 && n.Z.Sign() == 0
}

// pairEmbedded decides, exactly, whether facets i and j meet only along what
// their indices share: nothing, the one shared vertex, or the one shared edge
// (agent-docs/tessellation-design.md §1, the Embedding row). An unclassifiable pair
// fails, the refusing direction.
func (e *heldEmbedding) pairEmbedded(i, j int) bool {
	if !BoxesOverlap(e.triBox(i), e.triBox(j)) {
		return true
	}
	return e.pairEmbeddedOverlapping(i, j)
}

// pairEmbeddedOverlapping classifies a pair whose facet boxes are already
// known to overlap. Callers must establish that precondition before entry.
func (e *heldEmbedding) pairEmbeddedOverlapping(i, j int) bool {
	ti, tj := e.h.Tris[i], e.h.Tris[j]
	var bufI, bufJ [3]int
	sharedI, sharedJ := bufI[:0], bufJ[:0]
	for a, va := range ti {
		for b, vb := range tj {
			if va == vb {
				sharedI = append(sharedI, a)
				sharedJ = append(sharedJ, b)
			}
		}
	}
	if len(sharedI) == 3 {
		return false
	}
	if len(sharedI) == 0 && e.projectionsSeparated(i, j) {
		return true
	}
	if len(sharedI) == 1 && !BoxesOverlap(e.coneBox(i, sharedI[0]), e.coneBox(j, sharedJ[0])) {
		return true
	}
	if e.pairApartFloat(i, j, sharedI, sharedJ) {
		return true
	}
	return e.pairEmbeddedExact(i, j, sharedI, sharedJ)
}

// pairEmbeddedExact is pairEmbedded's exact path, for a pair that passed the
// box, projection and cone tests and that no float certificate decided.
func (e *heldEmbedding) pairEmbeddedExact(i, j int, sharedI, sharedJ []int) bool {
	ok, decided, signs := e.pairSeparated(i, j, sharedI, sharedJ)
	if decided {
		return ok
	}
	fa, xa, na := e.facet(i)
	fb, xb, nb := e.facet(j)
	if zeroXpt(na) || zeroXpt(nb) {
		return false
	}
	c, err := TriTriClassifyCore(fa, fb, xa, xb, na, nb, nil, nil,
		&signs.a, &signs.b, true, e.floatNorm(i), e.floatNorm(j), true)
	if err != nil {
		return false
	}
	switch len(sharedI) {
	case 0:
		return c.Kind == ContactNone
	case 1:
		switch c.Kind {
		case ContactPoint:
			return true
		case ContactRegion:
			// Coplanar facets sharing one vertex: embedded exactly when
			// they share no positive-length piece of boundary or area.
			return !CoplanarOverlap(xa, xb, na)
		default:
			return false
		}
	default:
		switch c.Kind {
		case ContactSegment:
			return true
		case ContactRegion:
			// Coplanar facets sharing an edge: embedded exactly when their
			// apexes lie strictly on opposite sides of it.
			a, b := xa[sharedI[0]], xa[sharedI[1]]
			apexI := xa[3-sharedI[0]-sharedI[1]]
			apexJ := xb[3-sharedJ[0]-sharedJ[1]]
			return PlaneSide(a, b, apexI, na)*PlaneSide(a, b, apexJ, na) < 0
		default:
			return false
		}
	}
}

type heldProjectedFacet struct {
	points [3][3][2]float64
	signs  [3][3]int8
	stable [3]bool
}

// projection builds each directed edge sign once for this facet. A vertex
// move invalidates its incident facets; the first parallel scan prepares all
// projections before sharing them, so its workers only read these caches.
func (e *heldEmbedding) projection(i int) *heldProjectedFacet {
	if e.projectedOK[i] {
		return &e.projected[i]
	}
	p := &e.projected[i]
	tri := e.h.Tris[i]
	for axis, axes := range [...][2]int{{0, 1}, {2, 0}, {1, 2}} {
		p.stable[axis] = true
		for k, v := range tri {
			point := e.h.Verts[v]
			p.points[axis][k] = [2]float64{AxisCoord(point, axes[0]), AxisCoord(point, axes[1])}
			for _, coord := range p.points[axis][k] {
				magnitude := proof.Abs(coord)
				if !(magnitude == 0 || magnitude >= 1e-100 && magnitude <= 1e100) {
					p.stable[axis] = false
				}
			}
		}
		// Keep every subtraction and product in Orient2Float away from
		// overflow and underflow; other scales use the exact pair checks.
		if !p.stable[axis] {
			continue
		}
		for k := range 3 {
			p.signs[axis][k] = int8(Orient2Float(p.points[axis][k],
				p.points[axis][(k+1)%3], p.points[axis][(k+2)%3]))
		}
	}
	e.projectedOK[i] = true
	return p
}

// projectionsSeparated proves two triangles disjoint when one coordinate
// projection has a strict separating edge. A projected contact is necessary
// for a 3D contact, and Orient2Float abstains near rounding uncertainty.
func (e *heldEmbedding) projectionsSeparated(i, j int) bool {
	a, b := e.projection(i), e.projection(j)
	for axis := range 3 {
		if !a.stable[axis] || !b.stable[axis] {
			continue
		}
		for k := range 3 {
			if projectedEdgeSeparates(a, b, axis, k) || projectedEdgeSeparates(b, a, axis, k) {
				return true
			}
		}
	}
	return false
}

func projectedEdgeSeparates(a, b *heldProjectedFacet, axis, edge int) bool {
	sign := int(a.signs[axis][edge])
	if sign == 0 {
		return false
	}
	start := a.points[axis][edge]
	end := a.points[axis][(edge+1)%3]
	bx, by := end[0]-start[0], end[1]-start[1]
	mxEdge, myEdge := proof.Abs(bx), proof.Abs(by)
	// A nonzero cached sign means Orient2Float admitted both edge
	// differences. Compute them once for all three queried points and check
	// only each point's new differences, with the same error bound.
	for _, point := range b.points[axis] {
		cx, cy := point[0]-start[0], point[1]-start[1]
		mx, my := proof.Abs(cx), proof.Abs(cy)
		if !(cx == 0 || mx >= 0x1p-300 && mx <= 0x1p250) ||
			!(cy == 0 || my >= 0x1p-300 && my <= 0x1p250) {
			return false
		}
		det := bx*cy - by*cx
		perm := mxEdge*my + myEdge*mx
		err := 1e-12 * perm
		if sign > 0 && !(det < -err) || sign < 0 && !(det > err) {
			return false
		}
	}
	return true
}

// coneBox bounds the unit directions in which facet i leaves its corner k:
// the arc of the unit sphere between its two edge directions there. Two facets
// sharing only that corner meet beyond it exactly when they leave it in a
// common direction — their intersection is convex and holds the corner — so
// boxes that do not overlap prove the pair meets only at the corner.
//
// The box is conservative. Each edge direction is a float normalisation good
// to a few ulps, the arc bulges past the box of its endpoints by at most the
// factor 1/cos(θ/2) for the angle θ between them, and a margin of 1e-9 covers
// every float error in that with room to spare. An edge shorter than 2⁻²⁰ of
// the coordinates' own scale, or an angle past 170°, gets the whole cube: its
// direction is not worth trusting to floats.
func (e *heldEmbedding) coneBox(i, k int) [2]r3.Vec {
	if e.coneOK[i][k] {
		return e.cone[i][k]
	}
	whole := [2]r3.Vec{r3.NewVec(-2, -2, -2), r3.NewVec(2, 2, 2)}
	t := e.h.Tris[i]
	v, a, b := e.h.Verts[t[k]], e.h.Verts[t[(k+1)%3]], e.h.Verts[t[(k+2)%3]]
	scale := math.Max(proof.Abs(v.X), math.Max(proof.Abs(v.Y), proof.Abs(v.Z)))
	s := math.Max(scale, math.SmallestNonzeroFloat64)
	var floor float64
	if s >= 0x1p-1000 {
		// A normal power of two times a value this large is exact, so the
		// product equals Ldexp(s, -20).
		floor = s * 0x1p-20
	} else {
		floor = math.Ldexp(s, -20)
	}
	da, db := a.Sub(v), b.Sub(v)
	la, lb := da.Len(), db.Len()
	box := whole
	if la > floor && lb > floor {
		ua, ub := da.Scale(1/la), db.Scale(1/lb)
		cosHalf := math.Sqrt(math.Max(0, (1+ua.Dot(ub))/2))
		if cosHalf > math.Cos(85*math.Pi/180) {
			grow := 1 / cosHalf
			lo := r3.NewVec(math.Min(ua.X, ub.X), math.Min(ua.Y, ub.Y), math.Min(ua.Z, ub.Z))
			hi := r3.NewVec(math.Max(ua.X, ub.X), math.Max(ua.Y, ub.Y), math.Max(ua.Z, ub.Z))
			ext := func(l, h float64) (float64, float64) {
				return math.Min(l, l*grow) - 1e-9, math.Max(h, h*grow) + 1e-9
			}
			lx, hx := ext(lo.X, hi.X)
			ly, hy := ext(lo.Y, hi.Y)
			lz, hz := ext(lo.Z, hi.Z)
			box = [2]r3.Vec{r3.NewVec(lx, ly, lz), r3.NewVec(hx, hy, hz)}
		}
	}
	e.cone[i][k] = box
	e.coneOK[i][k] = true
	return box
}

// pairSeparated decides the common pairs without the full classifier, by
// exact plane membership and adaptive orientation signs over the held floats:
//
//   - a facet with no plane fails;
//   - one shared edge, two planes: the planes meet only in the edge's line,
//     which each facet meets only in that edge;
//   - no shared vertex or one, two planes, and the unshared corners of one
//     facet strictly on one side of the other's plane: that facet meets the
//     plane, and so the other facet, only at what they share;
//   - one plane, decided in it: a shared edge is embedded exactly when the
//     apexes lie strictly on opposite sides of it, and otherwise an edge line
//     of one facet, through every shared vertex, that leaves every unshared
//     corner of the other strictly outside separates them.
//
// Anything else is left undecided, for the classifier.
// pairSideSigns stores each triangle's exact side against the other plane.
// The same signs feed the full contact classifier when separation is undecided.
type pairSideSigns struct{ a, b [3]int }

func (e *heldEmbedding) pairSeparated(i, j int, sharedI, sharedJ []int) (bool, bool, pairSideSigns) {
	ti, tj := e.h.Tris[i], e.h.Tris[j]
	otherI, otherJ := unshared(ti, sharedI), unshared(tj, sharedJ)
	// A side is -1, 0 or 1. Two marks a reading not needed yet.
	signs := pairSideSigns{a: [3]int{2, 2, 2}, b: [3]int{2, 2, 2}}
	for k, a := range sharedI {
		signs.a[a], signs.b[sharedJ[k]] = 0, 0
	}
	// strictSide reports whether the unshared corners lie strictly on one
	// side of facet f's plane, retaining every sign it reads.
	strictSide := func(f int, tri [3]int, shared []int, values *[3]int) bool {
		s0 := 0
		for k, p := range tri {
			if slices.Contains(shared, k) {
				continue
			}
			if values[k] == 2 {
				values[k] = e.planeSign(f, p)
			}
			s := values[k]
			if s == 0 || (s0 != 0 && s != s0) {
				return false
			}
			s0 = s
		}
		return true
	}
	if len(sharedI) == 2 && e.sideSigns == nil {
		// A nondegenerate facet sharing this edge is coplanar exactly when
		// its other corner lies on i's plane. The repair and serial scans
		// need no canonical plane key for this decision.
		_, _, ni := e.facet(i)
		_, _, nj := e.facet(j)
		if zeroXpt(ni) || zeroXpt(nj) {
			return false, true, pairSideSigns{}
		}
		if e.planeSign(i, otherJ.verts[0]) != 0 {
			return true, true, pairSideSigns{}
		}
		a, b := ti[sharedI[0]], ti[sharedI[1]]
		u, v, _ := e.planeAxes(i)
		return e.orient2(a, b, otherI.verts[0], u, v)*e.orient2(a, b, otherJ.verts[0], u, v) < 0,
			true, pairSideSigns{}
	}
	pi, pj := e.planeOf(i), e.planeOf(j)
	if pi == -2 || pj == -2 {
		return false, true, pairSideSigns{}
	}
	coplanar := pi == pj
	if len(sharedI) == 2 {
		a, b := ti[sharedI[0]], ti[sharedI[1]]
		if !coplanar {
			return true, true, pairSideSigns{}
		}
		u, v, _ := e.planeAxes(i)
		return e.orient2(a, b, otherI.verts[0], u, v)*e.orient2(a, b, otherJ.verts[0], u, v) < 0,
			true, pairSideSigns{}
	}
	if !coplanar {
		if strictSide(i, tj, sharedJ, &signs.b) || strictSide(j, ti, sharedI, &signs.a) {
			return true, true, pairSideSigns{}
		}
		for k, p := range ti {
			if signs.a[k] == 2 {
				signs.a[k] = e.planeSign(j, p)
			}
		}
		for k, p := range tj {
			if signs.b[k] == 2 {
				signs.b[k] = e.planeSign(i, p)
			}
		}
		return false, false, signs
	}
	u, v, ok := e.planeAxes(i)
	if !ok {
		return false, true, pairSideSigns{}
	}
	var shared []int
	for _, k := range sharedI {
		shared = append(shared, ti[k])
	}
	if e.edgeSeparates(ti, otherJ.verts[:otherJ.n], shared, u, v) ||
		e.edgeSeparates(tj, otherI.verts[:otherI.n], shared, u, v) {
		return true, true, pairSideSigns{}
	}
	// Equal exact planes put every corner on both planes.
	return false, false, pairSideSigns{}
}

// unsharedCorners keeps the at most three unshared vertices in the caller's
// stack frame. Pair separation checks this for every candidate facet pair.
type unsharedCorners struct {
	verts [3]int
	n     int
}

// unshared lists the corners of t whose positions are not in shared.
func unshared(t [3]int, shared []int) unsharedCorners {
	var out unsharedCorners
	for k, x := range t {
		if !slices.Contains(shared, k) {
			out.verts[out.n] = x
			out.n++
		}
	}
	return out
}

// planeOf interns facet i's exact plane: its normal reduced to the primitive
// integer direction with a positive leading component, and that direction's
// exact offset at one corner. Two facets share an id exactly when they lie in
// one plane, whichever way each is wound. A facet with no plane answers -2.
func (e *heldEmbedding) planeOf(i int) int {
	if e.plane[i] != -1 {
		return e.plane[i]
	}
	_, x, n := e.facet(i)
	if zeroXpt(n) {
		e.plane[i] = -2
		return -2
	}
	g := new(big.Int).GCD(nil, nil, new(big.Int).Abs(n.X), new(big.Int).Abs(n.Y))
	g.GCD(nil, nil, g, new(big.Int).Abs(n.Z))
	d := [3]*big.Int{new(big.Int).Quo(n.X, g), new(big.Int).Quo(n.Y, g), new(big.Int).Quo(n.Z, g)}
	lead := d[0].Sign()
	if lead == 0 {
		lead = d[1].Sign()
	}
	if lead == 0 {
		lead = d[2].Sign()
	}
	if lead < 0 {
		e.planeSense[i] = -1
		for k := range d {
			d[k].Neg(d[k])
		}
	} else {
		e.planeSense[i] = 1
	}
	off := new(big.Int).Mul(d[0], x[0].X)
	off.Add(off, new(big.Int).Mul(d[1], x[0].Y))
	off.Add(off, new(big.Int).Mul(d[2], x[0].Z))
	key := planeKey(d, off, x[0].W)
	id, ok := e.planes[string(key)]
	if !ok {
		id = len(e.planes)
		e.planes[string(key)] = id
	}
	e.plane[i] = id
	return id
}

// planeKey encodes a plane's primitive direction d and its offset off/w in
// lowest terms as bytes. Each of the five integers is a sign byte, a uvarint
// length and the magnitude, so equal keys mean equal reduced pairs.
func planeKey(d [3]*big.Int, off, w *big.Int) []byte {
	var num, den *big.Int
	switch {
	case off.Sign() == 0:
		num, den = new(big.Int), big.NewInt(1)
	case w.Sign() > 0 && w.BitLen() == int(w.TrailingZeroBits())+1:
		tz := min(off.TrailingZeroBits(), w.TrailingZeroBits())
		num, den = new(big.Int).Rsh(off, tz), new(big.Int).Rsh(w, tz)
	default:
		r := new(big.Rat).SetFrac(off, w)
		num, den = new(big.Int).Set(r.Num()), new(big.Int).Set(r.Denom())
	}
	buf := make([]byte, 0, 96)
	for _, v := range [5]*big.Int{d[0], d[1], d[2], num, den} {
		buf = append(buf, byte(v.Sign()+1))
		n := (v.BitLen() + 7) / 8
		buf = binary.AppendUvarint(buf, uint64(n))
		buf = slices.Grow(buf, n)[:len(buf)+n]
		v.FillBytes(buf[len(buf)-n:])
	}
	return buf
}

// planeAxes is the projection that keeps facet i's plane non-degenerate, or
// false for a facet with no plane.
func (e *heldEmbedding) planeAxes(i int) (int, int, bool) {
	_, _, n := e.facet(i)
	if zeroXpt(n) {
		return 0, 0, false
	}
	u, v := ProjAxes(n)
	return u, v, true
}

// edgeSeparates reports whether some edge line of t through every shared
// vertex leaves every point of pts strictly on the side away from t's third
// corner, in the (u, v) projection. The other facet then lies, apart from the
// shared vertices on the line, strictly outside t.
func (e *heldEmbedding) edgeSeparates(t [3]int, pts, shared []int, u, v int) bool {
	for k := range 3 {
		a, b, c := t[k], t[(k+1)%3], t[(k+2)%3]
		if len(shared) == 1 && shared[0] != a && shared[0] != b {
			continue
		}
		inner := e.orient2(a, b, c, u, v)
		if inner == 0 {
			return false
		}
		out := true
		for _, p := range pts {
			if e.orient2(a, b, p, u, v) != -inner {
				out = false
				break
			}
		}
		if out {
			return true
		}
	}
	return false
}

// orient2 is the exact orientation of held vertices a, b, c projected on axes
// (u, v): a float evaluation whose error bound decides it, else integer signs.
func (e *heldEmbedding) orient2(a, b, c, u, v int) int {
	vs := e.h.Verts
	pa := [2]float64{AxisCoord(vs[a], u), AxisCoord(vs[a], v)}
	pb := [2]float64{AxisCoord(vs[b], u), AxisCoord(vs[b], v)}
	pc := [2]float64{AxisCoord(vs[c], u), AxisCoord(vs[c], v)}
	if s := Orient2Float(pa, pb, pc); s != 0 {
		return s
	}
	project := func(p proof.Xpt) Xp2 {
		return Xp2{Hu: XIntCoordOf(p, u), Hv: XIntCoordOf(p, v), Hw: p.W}
	}
	return cross2xHomNumerator(project(e.lift(a)), project(e.lift(b)), project(e.lift(c))).Sign()
}

// planeSign is the exact side of held vertex p against facet f's plane: the
// float filter first, then f's cached exact normal.
func (e *heldEmbedding) planeSign(f, p int) int {
	var key [2]int
	cache := false
	if e.sideSigns != nil {
		if plane := e.planeOf(f); plane >= 0 {
			key, cache = [2]int{plane, p}, true
			if sign, ok := e.sideSigns[key]; ok {
				return int(e.planeSense[f]) * sign
			}
		}
	}
	t, v := e.h.Tris[f], e.h.Verts
	if s, certain := proof.OrientSignFloat(v[t[0]], v[t[1]], v[t[2]], v[p]); certain {
		if cache {
			e.sideSigns[key] = int(e.planeSense[f]) * s
		}
		return s
	}
	_, x, n := e.facet(f)
	sign := proof.XdotDiffSign(n, e.lift(p), x[0])
	if cache {
		e.sideSigns[key] = int(e.planeSense[f]) * sign
	}
	return sign
}

// facetFailures returns the facets j that facet i meets improperly, and
// whether i itself has collapsed to no plane.
func (e *heldEmbedding) facetFailures(i int) ([]int, bool, error) {
	if _, _, n := e.facet(i); zeroXpt(n) {
		return nil, true, nil
	}
	cands, err := e.candidates(i)
	if err != nil {
		return nil, false, err
	}
	var bad []int
	for _, j := range cands {
		if err := e.budget.Step(); err != nil {
			return nil, false, err
		}
		if j == i || e.pairOK(i, j) {
			continue
		}
		bad = append(bad, j)
	}
	return bad, false, nil
}

// vertexFailures counts the failing pairs that involve a facet incident to v,
// each pair once, plus each incident facet that has collapsed to no plane.
func (e *heldEmbedding) vertexFailures(v int) (int, error) {
	n := 0
	for _, f := range e.inc[v] {
		bad, collapsed, err := e.facetFailures(f)
		if err != nil {
			return 0, err
		}
		if collapsed {
			n++
			continue
		}
		for _, j := range bad {
			if j < f && slices.Contains(e.inc[v], j) {
				continue
			}
			n++
		}
	}
	return n, nil
}

// offenders scans the query facets and returns, in ascending order, the
// movable vertices of every failing facet and of every facet it fails
// against, and the failing query facets themselves.
func (e *heldEmbedding) offenders(query []int) ([]int, []int, error) {
	movable := make([]bool, len(e.h.Verts))
	vertices := make([]int, 0)
	var failing []int
	for _, i := range query {
		bad, collapsed, err := e.facetFailures(i)
		if err != nil {
			return nil, nil, err
		}
		if !collapsed && len(bad) == 0 {
			continue
		}
		failing = append(failing, i)
		e.addMovable(movable, &vertices, i)
		for _, j := range bad {
			e.addMovable(movable, &vertices, j)
		}
	}
	slices.Sort(vertices)
	return vertices, failing, nil
}

func (e *heldEmbedding) addMovable(movable []bool, vertices *[]int, f int) {
	for _, v := range e.h.Tris[f] {
		if e.h.Movable[v] && !movable[v] {
			movable[v] = true
			*vertices = append(*vertices, v)
		}
	}
}

// improve moves v to the corner of its exact point's float box that leaves
// the fewest failing pairs among the facets incident to v, when that is fewer
// than where v sits now; ties go to the earlier corner. A corner another held
// vertex occupies is skipped. Each move strictly lowers the mesh's count of
// failing pairs — only pairs touching v's facets change — so the search
// cannot cycle.
func (e *heldEmbedding) improve(v int) (bool, error) {
	orig := e.h.Verts[v]
	best, err := e.vertexFailures(v)
	if err != nil || best == 0 {
		return false, err
	}
	bestAt := orig
	usesReach := false
	// Each trial moves only v, so putting v back restores every incident
	// facet's positions, and with them the stamps they carried.
	var origGen []uint64
	if e.memoOn {
		origGen = e.stamps(v)
	}
	for _, c := range FloatBoxCorners(e.h.Exact[v]) {
		if err := e.budget.Step(); err != nil {
			return false, err
		}
		if c == orig {
			continue
		}
		if _, ok := e.taken[c]; ok {
			continue
		}
		e.setVert(v, c)
		n, err := e.vertexFailures(v)
		e.setVert(v, orig)
		if e.memoOn {
			e.restamp(v, origGen)
		}
		if err != nil {
			return false, err
		}
		if n < best {
			best, bestAt = n, c
		}
	}
	if best > 0 && e.reachOn {
		reachAt, reachBest, err := e.improveWithinReach(v, orig, best, origGen)
		if err != nil {
			return false, err
		}
		if reachBest < best {
			bestAt = reachAt
			usesReach = true
		}
	}
	if bestAt == orig {
		return false, nil
	}
	e.setVert(v, bestAt)
	if usesReach {
		if e.reachUsed == nil {
			e.reachUsed = make(map[int]struct{})
		}
		e.reachUsed[v] = struct{}{}
	} else {
		delete(e.reachUsed, v)
	}
	return true, nil
}

// reachSteps are the fractions of Reach a reach move tries, nearest first.
var reachSteps = [...]float64{1.0 / 64, 1.0 / 16, 1.0 / 4, 1}

// improveWithinReach tries positions up to Reach from v's exact point, moving
// along the unit normals of v's facets and of the facets they fail against and
// along the coordinate axes, both senses, nearest step first. It returns the position leaving the fewest
// failing pairs among v's facets when that is fewer than best, and best
// itself with orig otherwise. A step that clears every pair ends the search:
// the nearest clean position is the one the caller charges least for.
func (e *heldEmbedding) improveWithinReach(v int, orig r3.Vec, best int, origGen []uint64) (r3.Vec, int, error) {
	facets := slices.Clone(e.inc[v])
	for _, f := range e.inc[v] {
		bad, _, err := e.facetFailures(f)
		if err != nil {
			return orig, best, err
		}
		facets = append(facets, bad...)
	}
	slices.Sort(facets)
	facets = slices.Compact(facets)
	var dirs []r3.Vec
	for _, f := range facets {
		n, ok := e.floatNorm(f).Normalize()
		if !ok {
			continue
		}
		dirs = append(dirs, n, n.Scale(-1))
	}
	// A collapsed facet has no normal, so the axes are always tried too.
	for _, u := range [...]r3.Vec{{X: 1}, {Y: 1}, {Z: 1}} {
		dirs = append(dirs, u, u.Scale(-1))
	}
	base := e.h.Exact[v].Vec()
	bestAt := orig
	for _, step := range reachSteps {
		for _, u := range dirs {
			if err := e.budget.Step(); err != nil {
				return orig, best, err
			}
			c := base.Add(u.Scale(step * e.h.Reach))
			if c == orig {
				continue
			}
			if _, ok := e.taken[c]; ok {
				continue
			}
			e.setVert(v, c)
			n, err := e.vertexFailures(v)
			e.setVert(v, orig)
			if e.memoOn {
				e.restamp(v, origGen)
			}
			if err != nil {
				return orig, best, err
			}
			if n < best {
				best, bestAt = n, c
			}
		}
		if best == 0 {
			break
		}
	}
	return bestAt, best, nil
}

// FloatBoxCorners lists the binary64 points within an ulp of p per coordinate
// that bracket it: per coordinate the float at or below and the float at or
// above, one value when the coordinate is itself a float. Order is fixed — x
// before y before z, the lower float before the upper — so the search that
// reads it is deterministic.
func FloatBoxCorners(p proof.Xpt) []r3.Vec {
	px, py, pz := proof.XhpRat(proof.Xhp(p))
	x, y, z := floatBracket(px), floatBracket(py), floatBracket(pz)
	out := make([]r3.Vec, 0, 8)
	for _, a := range x {
		for _, b := range y {
			for _, c := range z {
				out = append(out, r3.NewVec(a, b, c))
			}
		}
	}
	return out
}

func floatBracket(r *big.Rat) []float64 {
	f, _ := r.Float64()
	switch polynomial.MustRatOf(f).Cmp(r) {
	case 0:
		return []float64{f}
	case -1:
		return []float64{f, math.Nextafter(f, math.Inf(1))}
	default:
		return []float64{math.Nextafter(f, math.Inf(-1)), f}
	}
}

// buildGrid registers every facet's padded box in a uniform grid sized to
// about one cell per facet.
func (e *heldEmbedding) buildGrid() error {
	maxAbs, maxDev := 0.0, 0.0
	for v, p := range e.h.Verts {
		if err := e.budget.Step(); err != nil {
			return err
		}
		maxAbs = math.Max(maxAbs, math.Max(math.Abs(p.X), math.Max(math.Abs(p.Y), math.Abs(p.Z))))
		if !e.h.Movable[v] {
			continue
		}
		q := e.h.Exact[v].Vec()
		maxDev = math.Max(maxDev, math.Max(math.Abs(p.X-q.X), math.Max(math.Abs(p.Y-q.Y), math.Abs(p.Z-q.Z))))
	}
	// A corner lies within an ulp of the exact point, a reach move within
	// Reach of it, and the exact point within maxDev (plus that rounding's
	// own ulp) of the first rounding, so no move leaves this pad. The pad
	// doubles because two facets can each move. A search that has not
	// turned reach moves on pads for corners alone, so a mesh the float box
	// clears is checked exactly as it is with no Reach.
	reach := 0.0
	if e.reachOn {
		reach = e.h.Reach
	}
	e.pad = 2*(maxDev+reach) + 8*proofbound.UlpOf(maxAbs)

	lo := r3.NewVec(math.Inf(1), math.Inf(1), math.Inf(1))
	hi := r3.NewVec(math.Inf(-1), math.Inf(-1), math.Inf(-1))
	for i, t := range e.h.Tris {
		b := TriBox(e.h.Verts, t)
		if e.memoOn {
			e.box[i] = b
			e.boxOK[i] = true
		}
		lo = r3.NewVec(math.Min(lo.X, b[0].X), math.Min(lo.Y, b[0].Y), math.Min(lo.Z, b[0].Z))
		hi = r3.NewVec(math.Max(hi.X, b[1].X), math.Max(hi.Y, b[1].Y), math.Max(hi.Z, b[1].Z))
	}
	lo = lo.Sub(r3.NewVec(e.pad, e.pad, e.pad))
	hi = hi.Add(r3.NewVec(e.pad, e.pad, e.pad))
	ext := hi.Sub(lo)
	n := float64(len(e.h.Tris))
	longest := math.Max(ext.X, math.Max(ext.Y, ext.Z))
	cell := math.Cbrt(ext.X * ext.Y * ext.Z / n)
	if !(cell > 0) || math.IsInf(cell, 0) {
		cell = longest / math.Cbrt(n)
	}
	if !(cell > 0) || math.IsInf(cell, 0) {
		cell = 1
	}
	cell = math.Max(cell, longest/1024)
	dims := [3]int{}
	for k, x := range [3]float64{ext.X, ext.Y, ext.Z} {
		dims[k] = max(1, min(1024, int(math.Ceil(x/cell))))
	}
	for dims[0]*dims[1]*dims[2] > 8*len(e.h.Tris)+64 {
		cell *= 2
		for k, x := range [3]float64{ext.X, ext.Y, ext.Z} {
			dims[k] = max(1, int(math.Ceil(x/cell)))
		}
	}
	e.origin, e.cell, e.dims = lo, cell, dims
	e.wide = nil
	e.cells = make([][]int32, dims[0]*dims[1]*dims[2])
	for i := range e.h.Tris {
		if err := e.budget.Step(); err != nil {
			return err
		}
		c0, c1 := e.cellRange(e.triBox(i))
		span := (c1[0] - c0[0] + 1) * (c1[1] - c0[1] + 1) * (c1[2] - c0[2] + 1)
		if span > heldGridCellSpan {
			e.wide = append(e.wide, int32(i))
			continue
		}
		for x := c0[0]; x <= c1[0]; x++ {
			for y := c0[1]; y <= c1[1]; y++ {
				for z := c0[2]; z <= c1[2]; z++ {
					k := (x*dims[1]+y)*dims[2] + z
					e.cells[k] = append(e.cells[k], int32(i))
				}
			}
		}
	}
	return nil
}

func (e *heldEmbedding) cellOf(x float64, axis int) int {
	o := [3]float64{e.origin.X, e.origin.Y, e.origin.Z}[axis]
	c := int(math.Floor((x - o) / e.cell))
	return max(0, min(e.dims[axis]-1, c))
}

func (e *heldEmbedding) cellRange(b [2]r3.Vec) ([3]int, [3]int) {
	lo := b[0].Sub(r3.NewVec(e.pad, e.pad, e.pad))
	hi := b[1].Add(r3.NewVec(e.pad, e.pad, e.pad))
	return [3]int{e.cellOf(lo.X, 0), e.cellOf(lo.Y, 1), e.cellOf(lo.Z, 2)},
		[3]int{e.cellOf(hi.X, 0), e.cellOf(hi.Y, 1), e.cellOf(hi.Z, 2)}
}

// candidates lists, once each, the facets registered in any cell facet i's
// padded box reaches, plus every wide facet.
func (e *heldEmbedding) candidates(i int) ([]int, error) {
	e.stampID++
	id := e.stampID
	// The list is reused across calls: each caller is done with it before
	// the next query.
	out := e.cands[:0]
	add := func(j int32) {
		if e.stamp[j] == id {
			return
		}
		e.stamp[j] = id
		out = append(out, int(j))
	}
	c0, c1 := e.cellRange(e.triBox(i))
	for x := c0[0]; x <= c1[0]; x++ {
		for y := c0[1]; y <= c1[1]; y++ {
			for z := c0[2]; z <= c1[2]; z++ {
				if err := e.scanStep(); err != nil {
					return nil, err
				}
				for _, j := range e.cells[(x*e.dims[1]+y)*e.dims[2]+z] {
					add(j)
				}
			}
		}
	}
	for _, j := range e.wide {
		add(j)
	}
	e.cands = out
	return out, nil
}

// ExactRigidImage is the exact image of p under the linear part b followed by
// the translation t, every product and sum taken in rationals: the point a
// float evaluation of the motion only approximates.
func ExactRigidImage(b r3.Basis, t, p r3.Vec) proof.Xpt {
	coord := func(ex, ey, ez, tc float64) *big.Rat {
		s := proof.MulRat(new(big.Rat), polynomial.MustRatOf(ex), polynomial.MustRatOf(p.X))
		proof.AddRat(s, s, proof.MulRat(new(big.Rat), polynomial.MustRatOf(ey), polynomial.MustRatOf(p.Y)))
		proof.AddRat(s, s, proof.MulRat(new(big.Rat), polynomial.MustRatOf(ez), polynomial.MustRatOf(p.Z)))
		return proof.AddRat(s, s, polynomial.MustRatOf(tc))
	}
	x := coord(b.EX.X, b.EY.X, b.EZ.X, t.X)
	y := coord(b.EX.Y, b.EY.Y, b.EZ.Y, t.Y)
	z := coord(b.EX.Z, b.EY.Z, b.EZ.Z, t.Z)
	w := new(big.Int).Set(x.Denom())
	for _, r := range []*big.Rat{y, z} {
		g := new(big.Int).GCD(nil, nil, w, r.Denom())
		w.Mul(w, new(big.Int).Quo(r.Denom(), g))
	}
	num := func(r *big.Rat) *big.Int {
		return new(big.Int).Mul(r.Num(), new(big.Int).Quo(w, r.Denom()))
	}
	return proof.Xpt(proof.XhpStripTwosOwned(proof.Xhp{X: num(x), Y: num(y), Z: num(z), W: w}))
}

// serialClean is parallelClean's first scan on the calling goroutine over the
// grid: each box-overlapping pair involving a query facet is decided once,
// without the pair memo. A pair of two query facets is decided when the lower
// index is scanned: whenever two facets' boxes meet, the grid lists each for
// the other (both register in and query the cells their padded boxes reach,
// and a wide facet is listed for every query), a pair whose boxes do not meet
// is embedded, and pairEmbedded is symmetric (HeldPairsAsymmetric is empty),
// so skipping it from the higher index misses nothing. A failing pair or a
// collapsed facet reports false and sends the caller through the ordered
// repair scan.
func (e *heldEmbedding) serialClean(query []int, queryFacet []bool) (bool, error) {
	for _, i := range query {
		if !e.facetOpenFloat(i) {
			if _, _, n := e.facet(i); zeroXpt(n) {
				return false, nil
			}
		}
		cands, err := e.candidates(i)
		if err != nil {
			return false, err
		}
		for _, j := range cands {
			if err := e.budget.Step(); err != nil {
				return false, err
			}
			if j == i || j < i && queryFacet[j] {
				continue
			}
			if !e.pairEmbedded(i, j) {
				return false, nil
			}
		}
	}
	return true, nil
}
