package prismcells

import (
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
)

// membership is Classify's per-cell, per-operand verdict.
// known reports whether propagation reached one at all; val, meaningful only
// when known, reports which side.
type membership struct {
	known bool
	val   bool
}

// link is one arrangement edge shared by exactly two cells: a and b
// are their indices into the profiles slice Classify received, and
// isB names which operand's entity the shared edge traces to.
type link struct {
	a, b int
	isB  bool
	// both marks an edge on a span sketch resolved between two operands'
	// coincident lines (CoincidentEdges): a boundary of both operands, so no
	// propagation crosses it.
	both bool
}

// Classify is §4.2's edge-orientation propagation: for every cell
// profiles holds, whether it sits on operand A's material side and on
// operand B's material side. resolved=false (err always nil in that case)
// means at least one cell's membership could not be reached for one of the
// two operands, or the arrangement holds a shape this classifier does not
// cover (an invalid cell, a cell carrying its own hole, an edge shared by
// more than two cells, or an entity this scene did not create) — the whole
// attempt is unresolved, as required by prism boolean §4.4.
func Classify(budget *proofbound.WorkBudget, tags map[sketch.Entity]Origin, profiles []*sketch.Profile) (matterA, matterB []bool, resolved bool, err error) {
	n := len(profiles)
	memberA := make([]membership, n)
	memberB := make([]membership, n)

	type edgeKey struct {
		entity sketch.Entity
		t0, t1 float64
	}
	type occurrence struct {
		cell int
		isB  bool
		both bool
	}
	occ := map[edgeKey][]occurrence{}

	// A span two operands' coincident lines share is emitted once, under one
	// operand's entity; it is the other operand's boundary too.
	coincident, ok, err := CoincidentEdges(budget, tags, profiles)
	if err != nil || !ok {
		return nil, nil, false, err
	}
	setMember := func(i int, isB, match bool) bool {
		member := &memberA[i]
		if isB {
			member = &memberB[i]
		}
		if member.known && member.val != match {
			return false
		}
		*member = membership{known: true, val: match}
		return true
	}

	for i, p := range profiles {
		if err := budget.Step(); err != nil {
			return nil, nil, false, err
		}
		if !p.Valid || len(p.Holes) != 0 {
			// Not a shape this classifier covers: an invalid cell, or one
			// carrying its own hole (both operands are hole-free on this
			// path, so a holed cell here is defensive, not expected).
			return nil, nil, false, nil
		}
		for _, e := range p.Outer {
			if err := budget.Step(); err != nil {
				return nil, nil, false, err
			}
			origin, ok := tags[e.Entity]
			if !ok {
				return nil, nil, false, nil // defensive: an entity this scene did not create
			}
			// The flag comparison itself (prism boolean §4.2):
			// e.Reversed is this cell's own walk direction on the entity;
			// origin.AuthoredReversed is the operand's own authored walk
			// direction on it. A match puts this cell on that operand's
			// material side, a mismatch on its void side.
			match := e.Reversed == origin.AuthoredReversed
			if !setMember(i, origin.IsB, match) {
				return nil, nil, false, nil // conflicting direct edges: not covered
			}

			k := edgeKey{entity: e.Entity, t0: e.TStart, t1: e.TEnd}
			c, shared := coincident.Edges[edgeSpan{entity: e.Entity, t0: e.TStart, t1: e.TEnd}]
			if shared {
				// The losing line walks this span too: the cell walks it
				// against the losing line's natural direction when it walks
				// it against the named line's, unless the two lines oppose.
				partner, ok := tags[c.Partner]
				if !ok || partner.IsB == origin.IsB {
					return nil, nil, false, nil
				}
				partnerMatch := (e.Reversed != c.Opposite) == partner.AuthoredReversed
				if !setMember(i, partner.IsB, partnerMatch) {
					return nil, nil, false, nil
				}
			}
			occ[k] = append(occ[k], occurrence{cell: i, isB: origin.IsB, both: shared})
		}
	}

	// Every edge shared by exactly two cells is a propagation link; more
	// than two occurrences of the same edgeKey is topology this classifier
	// does not cover. A single occurrence is a perimeter edge (against the
	// unbounded exterior) — already spent above for its direct
	// classification, nothing further to connect.
	var links []link
	for _, os := range occ {
		if err := budget.Step(); err != nil {
			return nil, nil, false, err
		}
		switch len(os) {
		case 1:
		case 2:
			links = append(links, link{a: os[0].cell, b: os[1].cell, isB: os[0].isB, both: os[0].both})
		default:
			return nil, nil, false, nil // §4.4: not a shape this increment covers
		}
	}

	if err := propagate(budget, links, true, memberA); err != nil {
		return nil, nil, false, err
	}
	if err := propagate(budget, links, false, memberB); err != nil {
		return nil, nil, false, err
	}

	matterA = make([]bool, n)
	matterB = make([]bool, n)
	for i := range profiles {
		if !memberA[i].known || !memberB[i].known {
			return nil, nil, false, nil // §4.4: isolated from an operand's own boundary
		}
		matterA[i] = memberA[i].val
		matterB[i] = memberB[i].val
	}
	return matterA, matterB, true, nil
}

// propagate floods a known cell classification across cells
// connected by an edge belonging to the OTHER operand: crossing such an edge
// cannot move across THIS operand's own boundary, so the two cells it joins
// carry the SAME membership. connectorIsB selects which links qualify — true
// when propagating operand A's own membership (over B's edges), false when
// propagating B's (over A's edges). A multi-source BFS from every
// already-known cell reaches every cell propagation can settle; what remains
// unknown afterward is genuinely unresolved (§4.4), read by the caller.
func propagate(budget *proofbound.WorkBudget, links []link, connectorIsB bool, member []membership) error {
	adj := make([][]int, len(member))
	for _, l := range links {
		if l.both || l.isB != connectorIsB {
			continue
		}
		adj[l.a] = append(adj[l.a], l.b)
		adj[l.b] = append(adj[l.b], l.a)
	}
	queue := make([]int, 0, len(member))
	visited := make([]bool, len(member))
	for i := range member {
		if member[i].known {
			queue = append(queue, i)
			visited[i] = true
		}
	}
	for len(queue) > 0 {
		cur := queue[0]
		queue = queue[1:]
		if err := budget.Step(); err != nil {
			return err
		}
		for _, nb := range adj[cur] {
			if visited[nb] {
				continue
			}
			visited[nb] = true
			member[nb] = member[cur]
			queue = append(queue, nb)
		}
	}
	return nil
}
