package meshbool

import (
	"github.com/lestrrat-3d/decad/internal/edgetable"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// conformEdgeMemo remembers, per undirected facet edge, what scanning that edge
// found and cost, so a second facet using the same edge (its twin, or any
// further user in a non-manifold mesh) does not walk the point tree again.
//
// Equivalence with ConformScan.EdgeInteriorHits rests on four facts, each held
// by edgeInteriorHitsMemo and pinned by the differential tests:
//
//   - Walk: the query box is built from min/max of the endpoints' coordinates,
//     so (a, b) and (b, a) give the bitwise same box. The tree walk, which
//     depends only on the box, therefore visits the same candidates in the same
//     order and steps the budget the same number of times, S. The memo stores S
//     and charges it with StepN, which is S calls of Step stopping at the first
//     error, so a budget or context refusal falls at the same step.
//   - Filter and predicate: the reject-only filter and the exact predicate are
//     evaluated for both directions of the edge in the one walk, each with that
//     direction's own arguments, so no symmetry of either is assumed.
//   - Exclusion: the walk drops only the edge's own endpoints, which every use
//     drops; each use then drops its own triangle's vertices, which keeps the
//     relative order of the rest.
//   - Order: each use sorts its own list by the same total order as before.
type conformEdgeMemo struct {
	// steps maps the undirected edge (lo, hi) to the steps its walk charged.
	steps *edgetable.Table
	// hits maps (lo, hi) to the hit lists of edges that have any.
	hits map[[2]int]conformEdgeHits
}

// conformEdgeHits holds an edge's hits in tree-walk order, without the
// endpoints: fwd for the direction lo to hi, rev for hi to lo.
type conformEdgeHits struct {
	fwd, rev []int
}

func newConformEdgeMemo(tris int) *conformEdgeMemo {
	return &conformEdgeMemo{steps: edgetable.New(tris*3/2 + 8)}
}

// edgeInteriorHitsMemo returns what EdgeInteriorHits(budget, a, b, tri)
// returns and charges the budget the same steps, walking the tree only the
// first time an undirected edge is seen.
func (s *ConformScan) edgeInteriorHitsMemo(budget *proofbound.WorkBudget, memo *conformEdgeMemo, a, b int,
	tri [3]int) ([]int, error) {
	if a == b {
		return s.EdgeInteriorHits(budget, a, b, tri)
	}
	lo, hi := min(a, b), max(a, b)
	var entry conformEdgeHits
	if steps, ok := memo.steps.Get(lo, hi); ok {
		if _, err := budget.StepN(uint64(steps)); err != nil {
			return nil, err
		}
		entry = memo.hits[[2]int{lo, hi}]
	} else {
		var err error
		entry, err = s.walkEdge(budget, memo, lo, hi)
		if err != nil {
			return nil, err
		}
	}
	list := entry.fwd
	if a != lo {
		list = entry.rev
	}
	var hits []int
	for _, i := range list {
		if i == tri[0] || i == tri[1] || i == tri[2] {
			continue
		}
		hits = append(hits, i)
	}
	if len(hits) > 1 {
		s.sortHits(hits)
	}
	return hits, nil
}

// walkEdge walks the tree once for the undirected edge (lo, hi), records its
// step count and hit lists in memo, and returns the hit lists (empty when none).
func (s *ConformScan) walkEdge(budget *proofbound.WorkBudget, memo *conformEdgeMemo, lo, hi int) (conformEdgeHits, error) {
	pl, ph := s.Approx[lo], s.Approx[hi]
	fwdFilter := NewSegFilter(pl, ph, s.Tau2)
	revFilter := NewSegFilter(ph, pl, s.Tau2)
	var entry conformEdgeHits
	steps := 0
	step := func() error {
		steps++
		return budget.Step()
	}
	if err := s.points.visitOverlaps(conformSegmentQueryBox(pl, ph, s.Tau2), step, func(i int) error {
		if err := step(); err != nil {
			return err
		}
		if i == lo || i == hi {
			return nil
		}
		p := s.Approx[i]
		if !fwdFilter.TooFar(p) && OnSegmentInterior3(s.Verts[lo], s.Verts[hi], s.Verts[i]) {
			entry.fwd = append(entry.fwd, i)
		}
		if !revFilter.TooFar(p) && OnSegmentInterior3(s.Verts[hi], s.Verts[lo], s.Verts[i]) {
			entry.rev = append(entry.rev, i)
		}
		return nil
	}); err != nil {
		return conformEdgeHits{}, err
	}
	memo.steps.Set(lo, hi, steps)
	if entry.fwd == nil && entry.rev == nil {
		return entry, nil
	}
	if memo.hits == nil {
		memo.hits = make(map[[2]int]conformEdgeHits)
	}
	memo.hits[[2]int{lo, hi}] = entry
	return entry, nil
}
