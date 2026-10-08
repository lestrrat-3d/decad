package prismcells

import (
	"slices"

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

// CellsHaveNoVoid reports whether every cell has at least one boundary edge
// on its operand's material side, making Union's select-all merge the union
// (docs/prism-boolean-design.md §4.2). Membership is constant over a cell,
// so one material-side edge puts the cell inside that operand. A cell whose
// every edge lies on its operand's void side is outside each operand with an
// edge on it. Since both operands are hole-free, such a cell is an enclosed
// void. The check reads sketch's Reversed flag against the authored sense,
// as Classify does. False can only send the pair to the mesh path.
func CellsHaveNoVoid(budget *proofbound.WorkBudget, tags map[sketch.Entity]Origin, profiles []*sketch.Profile) (bool, error) {
	for _, p := range profiles {
		material := false
		for _, loop := range append([][]sketch.BoundaryEdge{p.Outer}, p.Holes...) {
			for _, e := range loop {
				if err := budget.Step(); err != nil {
					return false, err
				}
				origin, ok := tags[e.Entity]
				if !ok {
					return false, nil
				}
				if e.Reversed == origin.AuthoredReversed {
					material = true
				}
			}
		}
		if !material {
			return false, nil
		}
	}
	return true, nil
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

// RegionKey names one record of a scene: an operand and one of its regions,
// as the tag map's Origin names them.
type RegionKey struct {
	IsB    bool
	Region int
}

// ClassifyRegions is Classify run per record rather than per operand
// (docs/general-boolean-design.md §3 "A1 as a brep"): for every cell and
// every record of the scene, whether the cell sits on that record's material
// side. Each record's membership propagates across the edges of every other
// record and never across its own; a span two records share
// (CoincidentEdgesRegions) is a boundary of both. resolved=false (err always
// nil then) means a cell's membership in some record could not be reached,
// or a shape Classify does not cover.
func ClassifyRegions(budget *proofbound.WorkBudget, tags map[sketch.Entity]Origin, profiles []*sketch.Profile) (map[RegionKey][]bool, bool, error) {
	n := len(profiles)
	member := map[RegionKey][]membership{}
	for _, origin := range tags {
		k := RegionKey{IsB: origin.IsB, Region: origin.Region}
		if _, ok := member[k]; !ok {
			member[k] = make([]membership, n)
		}
	}
	type edgeKey struct {
		entity sketch.Entity
		t0, t1 float64
	}
	type occurrence struct {
		cell int
		keys []RegionKey
	}
	occ := map[edgeKey][]occurrence{}
	coincident, ok, err := CoincidentEdgesRegions(budget, tags, profiles)
	if err != nil || !ok {
		return nil, false, err
	}
	setMember := func(i int, k RegionKey, match bool) bool {
		m := &member[k][i]
		if m.known && m.val != match {
			return false
		}
		*m = membership{known: true, val: match}
		return true
	}
	for i, p := range profiles {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		if !p.Valid || len(p.Holes) != 0 {
			return nil, false, nil
		}
		for _, e := range p.Outer {
			if err := budget.Step(); err != nil {
				return nil, false, err
			}
			origin, ok := tags[e.Entity]
			if !ok {
				return nil, false, nil
			}
			k := RegionKey{IsB: origin.IsB, Region: origin.Region}
			if !setMember(i, k, e.Reversed == origin.AuthoredReversed) {
				return nil, false, nil
			}
			keys := []RegionKey{k}
			if c, shared := coincident.Edges[edgeSpan{entity: e.Entity, t0: e.TStart, t1: e.TEnd}]; shared {
				partner, ok := tags[c.Partner]
				pk := RegionKey{IsB: partner.IsB, Region: partner.Region}
				if !ok || pk == k {
					return nil, false, nil
				}
				if !setMember(i, pk, (e.Reversed != c.Opposite) == partner.AuthoredReversed) {
					return nil, false, nil
				}
				keys = append(keys, pk)
			}
			occ[edgeKey{entity: e.Entity, t0: e.TStart, t1: e.TEnd}] = append(occ[edgeKey{entity: e.Entity, t0: e.TStart, t1: e.TEnd}], occurrence{cell: i, keys: keys})
		}
	}
	type regionLink struct {
		a, b int
		keys []RegionKey
	}
	var links []regionLink
	for _, os := range occ {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		switch len(os) {
		case 1:
		case 2:
			links = append(links, regionLink{a: os[0].cell, b: os[1].cell, keys: os[0].keys})
		default:
			return nil, false, nil
		}
	}
	for k, m := range member {
		adj := make([][]int, n)
		for _, l := range links {
			if slices.Contains(l.keys, k) {
				continue
			}
			adj[l.a] = append(adj[l.a], l.b)
			adj[l.b] = append(adj[l.b], l.a)
		}
		if err := flood(budget, adj, m); err != nil {
			return nil, false, err
		}
	}
	out := make(map[RegionKey][]bool, len(member))
	for k, m := range member {
		vals := make([]bool, n)
		for i := range m {
			if !m[i].known {
				return nil, false, nil
			}
			vals[i] = m[i].val
		}
		out[k] = vals
	}
	return out, true, nil
}

// flood is propagate's breadth-first walk over an adjacency the caller has
// already filtered.
func flood(budget *proofbound.WorkBudget, adj [][]int, member []membership) error {
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
