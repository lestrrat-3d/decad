package prismcells

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
)

// SurvivingFragments reads §3.2's Trim side: for every RECEIVER boundary
// fragment, Classify's own tool-side reading for the cell that is
// the RECEIVER's own material (matterRcv true) decides its fate.
//
// A fragment's OTHER side is usually the unbounded face s.Profiles() does not
// return, in which case matterRcv is true for the one cell it bounds and
// "one cell settles it" (§3.2) needs nothing further. It is a SECOND bounded
// cell only where the tool's own section extends past the receiver's on that
// side — a tool taller or wider than the sheet it trims, which every fixture
// here that spans the sheet axially produces on its unswept sides — and that
// second cell is never the receiver's own material (matterRcv false there),
// so filtering on matterRcv alone picks the one occurrence that matters
// without a special case for which shape produced it. Two occurrences BOTH
// reporting matterRcv true is RS5's own coincident carrier: the tool's
// boundary runs exactly along the receiver's, so a receiver fragment bounds
// the receiver's material on both sides at once.
//
// keepInside selects the survivors: false keeps a fragment whose settling
// cell Classify puts outside the tool's material, true keeps one
// it puts inside. total is the count of distinct receiver fragments the
// arrangement produced, for RS6's "keeps every fragment or none" check.
func SurvivingFragments(budget *proofbound.WorkBudget, tags map[sketch.Entity]Origin, matterRcv, matterTool []bool, profiles []*sketch.Profile, keepInside bool) (survivors []sketch.BoundaryEdge, total int, err error) {
	type edgeKey struct {
		entity sketch.Entity
		t0, t1 float64
	}
	type settled struct {
		edge   sketch.BoundaryEdge
		inTool bool
	}
	byKey := map[edgeKey]settled{}
	var order []edgeKey
	for i, p := range profiles {
		if err := budget.Step(); err != nil {
			return nil, 0, err
		}
		if !p.Valid {
			return nil, 0, fmt.Errorf(`%w: a cell the trim resolution depends on is not valid`, decaderr.ErrUnsupported)
		}
		if !matterRcv[i] {
			continue // not the receiver's own material: never the settling cell
		}
		for _, loop := range append([][]sketch.BoundaryEdge{p.Outer}, p.Holes...) {
			for _, e := range loop {
				if err := budget.Step(); err != nil {
					return nil, 0, err
				}
				origin, ok := tags[e.Entity]
				if !ok {
					return nil, 0, fmt.Errorf(`decad: a trim arrangement entity traces to neither operand`)
				}
				if origin.IsB {
					continue // tool-sourced: never part of a trimmed result
				}
				key := edgeKey{entity: e.Entity, t0: e.TStart, t1: e.TEnd}
				if _, dup := byKey[key]; dup {
					return nil, 0, fmt.Errorf(`%w: a receiver boundary fragment coincides with the tool's own boundary`, decaderr.ErrUnsupported)
				}
				byKey[key] = settled{edge: e, inTool: matterTool[i]}
				order = append(order, key)
			}
		}
	}
	total = len(order)
	for _, key := range order {
		s := byKey[key]
		if s.inTool == keepInside {
			survivors = append(survivors, s.edge)
		}
	}
	return survivors, total, nil
}

// ChainSurvivorWalks is §3.3's open-walk chaining: chainPrismUnionSurvivors
// (prism_boolean.go) generalized two ways, and nothing else — the walk is not
// required to close, and a survivor set that falls into several runs yields
// several walks rather than failing. Connectivity reads sketch's own walked
// Polyline endpoints exactly as chainPrismUnionSurvivors does: bookkeeping on
// an answer sketch already computed, never a re-derived geometric fact.
// resolved=false (err always nil in that case) means the survivors do not
// partition cleanly into dangling-ended runs — a shape this evaluator does
// not cover.
func ChainSurvivorWalks(budget *proofbound.WorkBudget, survivors []sketch.BoundaryEdge) ([][]sketch.BoundaryEdge, bool, error) {
	type endpoints struct{ start, end sectionrecord.Point2 }
	pts := make([]endpoints, len(survivors))
	byStart := make(map[sectionrecord.Point2]int, len(survivors))
	for i, e := range survivors {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		if len(e.Polyline) < 2 {
			return nil, false, nil // defensive: no walked endpoints to key on
		}
		start := sectionrecord.Point2{U: e.Polyline[0][0], V: e.Polyline[0][1]}
		end := sectionrecord.Point2{U: e.Polyline[len(e.Polyline)-1][0], V: e.Polyline[len(e.Polyline)-1][1]}
		if _, dup := byStart[start]; dup {
			return nil, false, nil // ambiguous: more than one survivor leaves this vertex
		}
		byStart[start] = i
		pts[i] = endpoints{start: start, end: end}
	}
	hasIncoming := make(map[sectionrecord.Point2]bool, len(survivors))
	for _, p := range pts {
		hasIncoming[p.end] = true
	}

	used := make([]bool, len(survivors))
	var walks [][]sketch.BoundaryEdge
	for i := range survivors {
		if used[i] || hasIncoming[pts[i].start] {
			continue // not a run's own start: reached by following its predecessor
		}
		var walk []sketch.BoundaryEdge
		cur := i
		for {
			if err := budget.Step(); err != nil {
				return nil, false, err
			}
			used[cur] = true
			walk = append(walk, survivors[cur])
			next, ok := byStart[pts[cur].end]
			if !ok {
				break // a free end: this run is done
			}
			if used[next] {
				return nil, false, nil // a cycle folding back without a dangling start
			}
			cur = next
		}
		walks = append(walks, walk)
	}
	for _, u := range used {
		if !u {
			return nil, false, nil // a pure cycle among the survivors: not a shape this covers
		}
	}
	return walks, true, nil
}
