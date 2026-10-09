package prismcells

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/sketchrecord"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Merge is §4.2's/§4.3's shared cell-merge machinery, common to
// Union's own select-all path (selected is every returned cell) and the
// crossing sub-case's per-op selection (prism_boolean_crossing.go): count
// every boundary edge (Outer and every Hole loop) of every selected cell by
// (Entity, TStart, TEnd) — a shared wall between two adjacent selected cells
// is walked in opposite senses by each but reports the SAME natural-direction
// TStart < TEnd range (Reversed, not the order, states the walk direction —
// sketch-seam-design's own contract), so this key alone matches it on both
// sides. Drop every edge counted exactly twice (an interior wall between two
// selected cells); keep every edge counted exactly once (a wall against the
// unbounded exterior, an unselected neighbor, or a coincident-carrier wall
// reported under only one operand's entity). A count outside {1, 2} is
// topology this increment does not cover.
//
// resolved=false (err always nil in that case) means the selected set's own
// topology is unresolved (§4.4): a disjoint footprint, an internal void, an
// edge count outside {1, 2}, or a chain that does not close into one simple
// loop. opName names the op for RB1's message alone.
func Merge(budget *proofbound.WorkBudget, selected []*sketch.Profile, opName string) (momentinput.Profile, float64, bool, error) {
	survivors, resolved, err := mergeSurvivors(budget, selected, opName)
	if err != nil || !resolved {
		return momentinput.Profile{}, 0, false, err
	}
	chain, resolved, err := ChainClosedSurvivors(budget, survivors)
	if err != nil {
		return momentinput.Profile{}, 0, false, err
	}
	if !resolved {
		return momentinput.Profile{}, 0, false, nil // §4.4: the survivors do not close into one simple loop
	}
	loop, cutDelta, err := recordChain(budget, chain)
	if err != nil {
		return momentinput.Profile{}, 0, false, err
	}
	return momentinput.Profile{Outer: loop}, cutDelta, true, nil
}

// MergeLoops is Merge for a selection whose survivors close into several
// loops (docs/general-boolean-design.md §3 A5's disjoint Union): the same
// edge counting, then ChainClosedLoops partitions the survivors into closed
// cycles, and each cycle is recorded and closure-falsified as Merge records
// its one loop. The loops are the candidate boundaries only: whether they
// bound disjoint lumps is the caller's to prove. cutDelta is the largest
// over every loop.
func MergeLoops(budget *proofbound.WorkBudget, selected []*sketch.Profile, opName string) ([]sectionrecord.LoopRecord, float64, bool, error) {
	survivors, resolved, err := mergeSurvivors(budget, selected, opName)
	if err != nil || !resolved {
		return nil, 0, false, err
	}
	chains, resolved, err := ChainClosedLoops(budget, survivors)
	if err != nil || !resolved {
		return nil, 0, false, err
	}
	loops := make([]sectionrecord.LoopRecord, 0, len(chains))
	cutDelta := 0.0
	for _, chain := range chains {
		loop, d, err := recordChain(budget, chain)
		if err != nil {
			return nil, 0, false, err
		}
		loops = append(loops, loop)
		cutDelta = math.Max(cutDelta, d)
	}
	return loops, cutDelta, true, nil
}

// mergeSurvivors is Merge's edge count: every boundary edge of every selected
// cell keyed by (Entity, TStart, TEnd), an edge counted twice dropped as an
// interior wall, an edge counted once kept, any other count unresolved. An
// invalid selected cell is RB1.
func mergeSurvivors(budget *proofbound.WorkBudget, selected []*sketch.Profile, opName string) ([]sketch.BoundaryEdge, bool, error) {
	type edgeKey struct {
		entity sketch.Entity
		t0, t1 float64
	}
	counts := map[edgeKey]int{}
	for _, p := range selected {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		if !p.Valid {
			// RB1: a candidate region the merge depends on is invalid. Every
			// cell in `selected` is part of this op's result, so any invalid
			// cell among them is a genuine refusal, not an unresolved topology.
			return nil, false, InvalidRegionError(opName)
		}
		for _, loop := range append([][]sketch.BoundaryEdge{p.Outer}, p.Holes...) {
			for _, e := range loop {
				if err := budget.Step(); err != nil {
					return nil, false, err
				}
				counts[edgeKey{entity: e.Entity, t0: e.TStart, t1: e.TEnd}]++
			}
		}
	}

	// Second pass, in the arrangement's own deterministic profile/edge order.
	var survivors []sketch.BoundaryEdge
	for _, p := range selected {
		for _, loop := range append([][]sketch.BoundaryEdge{p.Outer}, p.Holes...) {
			for _, e := range loop {
				if err := budget.Step(); err != nil {
					return nil, false, err
				}
				n := counts[edgeKey{entity: e.Entity, t0: e.TStart, t1: e.TEnd}]
				switch n {
				case 1:
					survivors = append(survivors, e)
				case 2:
					// dropped: an interior wall
				default:
					return nil, false, nil // §4.4: not a shape this increment covers
				}
			}
		}
	}
	if len(survivors) == 0 {
		return nil, false, nil
	}
	return survivors, true, nil
}

// recordChain records one chained loop. Past the chaining every problem is
// genuine: a rejected TExact fragment (§9's RB8) or a non-closing loop (RB9).
func recordChain(budget *proofbound.WorkBudget, chain []sketch.BoundaryEdge) (sectionrecord.LoopRecord, float64, error) {
	segs := make([]sectionrecord.CurveSegment, len(chain))
	joins := make([]sketchrecord.LoopJoin, len(chain))
	cutDelta := 0.0
	for i, e := range chain {
		if err := budget.Step(); err != nil {
			return sectionrecord.LoopRecord{}, 0, err
		}
		seg, err := sketchrecord.RecordEdge(e)
		if err != nil {
			return sectionrecord.LoopRecord{}, 0, err
		}
		segs[i] = seg
		join, err := sketchrecord.EdgeJoin(e, seg)
		if err != nil {
			return sectionrecord.LoopRecord{}, 0, err
		}
		joins[i] = join
		d, err := CutDelta(e, seg)
		if err != nil {
			return sectionrecord.LoopRecord{}, 0, err
		}
		cutDelta = math.Max(cutDelta, d)
	}
	// §6's closure row (RB9): the seam's own junction falsifier, run on the
	// merged chain's recorded coordinates. recordLoop is not called here — it
	// takes no budget.step() charge and would drop cutDelta's per-edge
	// pairing — so this calls edgeJoin/falsifyLoopJoins directly, unchanged.
	if err := sketchrecord.FalsifyLoopJoins("merged loop", joins); err != nil {
		return sectionrecord.LoopRecord{}, 0, err
	}
	return sectionrecord.LoopRecord{Segments: segs}, cutDelta, nil
}

// CutDelta is §7's cut charge for one surviving boundary edge: how
// far the point the recorded range names can sit from the crossing it denotes.
//
// A WHOLE edge charges nothing. Its range is the entity's own full domain, so
// its endpoints are the entity's own recorded endpoints and this union computed
// no coordinate for it at all.
//
// A Partial fragment charges proofbound.CutDisplacementAllow. Its carrier is the entity's
// unchanged defining data, but its two ends are named by TStart/TEnd — the
// parameters sketch's arrangement COMPUTED for this pair, which are freshly
// rounded whatever the two operands carried, and which the operands' own zero
// displacement therefore says nothing about. That is the whole of what §7's
// decidable zero case does not reach.
func CutDelta(e sketch.BoundaryEdge, seg sectionrecord.CurveSegment) (float64, error) {
	if !e.Partial {
		return 0, nil
	}
	speed, err := CarrierSpeedUpper(seg)
	if err != nil {
		return 0, err
	}
	return proofbound.CutDisplacementAllow(speed), nil
}

// CarrierSpeedUpper is a proven upper bound on |dP/dt| over the WHOLE of a
// recorded line, circle or arc's own parameterisation — the factor that turns a
// parameter allowance into a coordinate one. G4 admits only these three kinds,
// so no other kind can reach it.
//
// Each bound is the parameterisation walkOf reads the segment through
// (extrude.go): a line runs Start → End over [0, 1], so its speed is the chord;
// a circle runs a full turn over [0, 1], so its speed is 2πR; an arc runs its
// own sweep, which is at most a full turn, so 2πR bounds it too.
func CarrierSpeedUpper(seg sectionrecord.CurveSegment) (float64, error) {
	seg, err := sectionrecord.NormalizeSegment(seg)
	if err != nil {
		return 0, err
	}
	switch seg := seg.(type) {
	case sectionrecord.LineSeg:
		return point2SeparationUpper(seg.Start, seg.End), nil
	case sectionrecord.CircleSeg:
		r, err := seg.Radius.In(units.Millimeter)
		if err != nil {
			return 0, fmt.Errorf(`decad: a merged circle segment's radius is not a length: %w`, err)
		}
		return proofbound.ProductUpper(proofbound.TwoPiUpper(), math.Abs(r)), nil
	case sectionrecord.ArcSeg:
		return proofbound.ProductUpper(proofbound.TwoPiUpper(), point2SeparationUpper(seg.Center, seg.Start)), nil
	default:
		return 0, fmt.Errorf(`%w: a %T segment has no carrier speed this evaluator states`, decaderr.ErrUnsupported, seg)
	}
}

// point2SeparationUpper bounds |b − a| for two recorded plane points. The two
// differences are taken EXACTLY over rationals and summed, which bounds the
// Euclidean norm because the L1 norm never falls below it. A non-finite
// coordinate has no separation to state and answers +Inf.
func point2SeparationUpper(a, b sectionrecord.Point2) float64 {
	au, av, bu, bv := proofarith.FloatRat(a.U), proofarith.FloatRat(a.V), proofarith.FloatRat(b.U), proofarith.FloatRat(b.V)
	if au == nil || av == nil || bu == nil || bv == nil {
		return math.Inf(1)
	}
	return boundarywalk.RatL1Upper(new(big.Rat).Sub(bu, au), new(big.Rat).Sub(bv, av))
}

// HasSplitBoundary reports whether sketch narrowed any arranged boundary
// edge: whether the arrangement cut anything, which is where an input
// displacement can be amplified by a crossing angle (CrossingCharge).
func HasSplitBoundary(budget *proofbound.WorkBudget, profiles []*sketch.Profile) (bool, error) {
	for _, profile := range profiles {
		if err := budget.Step(); err != nil {
			return false, err
		}
		for _, loop := range append([][]sketch.BoundaryEdge{profile.Outer}, profile.Holes...) {
			for _, edge := range loop {
				if err := budget.Step(); err != nil {
					return false, err
				}
				if edge.Partial {
					return true, nil
				}
			}
		}
	}
	return false, nil
}

// ChainClosedSurvivors chains the surviving boundary edges into one
// closed directed walk (§4.2), the same directed-edge-loop-closure shape
// boolean_body.go's face-patch construction performs for the mesh boolean's
// own loops. resolved=false means the survivors do not close into exactly
// one simple loop using every one of them — "not resolved" (§4.4): a
// disjoint pair of footprints (two separate lumps) or a union enclosing an
// internal void both leave a dangling end or a smaller cycle here.
//
// Connectivity is read off sketch's OWN walked boundary sample — Polyline[0]
// is the walk's start, Polyline[len-1] its end (sketch-seam-design's
// contract) — matched by exact (u, v) equality: these are vertices the SAME
// arrangement pass already computed and shares between the edges that meet
// there, so no tolerance is introduced. This is bookkeeping on sketch's own
// answer, not a re-derived geometric fact (CLAUDE.md's carve-out for this
// design).
func ChainClosedSurvivors(budget *proofbound.WorkBudget, survivors []sketch.BoundaryEdge) (chain []sketch.BoundaryEdge, resolved bool, err error) {
	byStart := make(map[sectionrecord.Point2]int, len(survivors))
	for i, e := range survivors {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		if len(e.Polyline) < 2 {
			return nil, false, nil // defensive: no walked endpoints to key on
		}
		start := sectionrecord.Point2{U: e.Polyline[0][0], V: e.Polyline[0][1]}
		if _, dup := byStart[start]; dup {
			return nil, false, nil // ambiguous: more than one survivor leaves this vertex
		}
		byStart[start] = i
	}

	used := make([]bool, len(survivors))
	chain = make([]sketch.BoundaryEdge, 0, len(survivors))
	cur := 0
	for range survivors {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		if used[cur] {
			return nil, false, nil // a smaller cycle closed before every edge was used
		}
		used[cur] = true
		e := survivors[cur]
		chain = append(chain, e)
		end := sectionrecord.Point2{U: e.Polyline[len(e.Polyline)-1][0], V: e.Polyline[len(e.Polyline)-1][1]}
		next, ok := byStart[end]
		if !ok {
			return nil, false, nil // a dead end: no survivor continues from here
		}
		cur = next
	}
	if cur != 0 {
		return nil, false, nil // the walk did not return to its own start
	}
	return chain, true, nil
}

// ChainClosedLoops partitions the survivors into closed directed cycles,
// connectivity read off sketch's own walked Polyline endpoints by exact
// equality, as ChainClosedSurvivors reads it. resolved=false (err always nil
// in that case) means some vertex has more than one survivor leaving it, or a
// walk reaches a dead end: survivors that do not partition into simple closed
// loops are a shape this evaluator does not cover.
func ChainClosedLoops(budget *proofbound.WorkBudget, survivors []sketch.BoundaryEdge) ([][]sketch.BoundaryEdge, bool, error) {
	byStart := make(map[sectionrecord.Point2]int, len(survivors))
	ends := make([]sectionrecord.Point2, len(survivors))
	for i, e := range survivors {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		if len(e.Polyline) < 2 {
			return nil, false, nil // defensive: no walked endpoints to key on
		}
		start := sectionrecord.Point2{U: e.Polyline[0][0], V: e.Polyline[0][1]}
		if _, dup := byStart[start]; dup {
			return nil, false, nil // ambiguous: more than one survivor leaves this vertex
		}
		byStart[start] = i
		ends[i] = sectionrecord.Point2{U: e.Polyline[len(e.Polyline)-1][0], V: e.Polyline[len(e.Polyline)-1][1]}
	}
	used := make([]bool, len(survivors))
	var loops [][]sketch.BoundaryEdge
	for first := range survivors {
		if used[first] {
			continue
		}
		var chain []sketch.BoundaryEdge
		cur := first
		for {
			if err := budget.Step(); err != nil {
				return nil, false, err
			}
			if used[cur] {
				if cur != first {
					return nil, false, nil // the walk joined another loop part-way
				}
				break
			}
			used[cur] = true
			chain = append(chain, survivors[cur])
			next, ok := byStart[ends[cur]]
			if !ok {
				return nil, false, nil // a dead end: no survivor continues from here
			}
			cur = next
		}
		loops = append(loops, chain)
	}
	return loops, true, nil
}

// Select filters profiles to the cells keep admits, given each
// cell's own classification — a pure data selection, no geometry read.
func Select(budget *proofbound.WorkBudget, profiles []*sketch.Profile, matterA, matterB []bool, keep func(a, b bool) bool) ([]*sketch.Profile, error) {
	var selected []*sketch.Profile
	for i, p := range profiles {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		if keep(matterA[i], matterB[i]) {
			selected = append(selected, p)
		}
	}
	return selected, nil
}

// InvalidRegionError is §9's RB1: a candidate region this op's result
// depends on reports Profile.Valid == false. It is a genuine refusal past
// §3.4's point of no return, never a reroute to the mesh path.
func InvalidRegionError(op string) error {
	return fmt.Errorf(`%w: the %s scene's arrangement reports an invalid region`, decaderr.ErrUnsupported, op)
}
