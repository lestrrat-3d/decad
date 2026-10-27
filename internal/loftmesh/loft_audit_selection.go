package loftmesh

import (
	"slices"
	"sync/atomic"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// prepareLoftStreamAuditData runs the existing bounded row preflight before
// constructing exact tables. Proof workers start only after the row workers
// have joined and the caller has replayed their original budget steps.
func prepareLoftStreamAuditData(budget *proofbound.WorkBudget, tree loftRowScan,
	members, starts []int, ceiling uint64, verts []r3.Vec, tris [][3]int,
	shortcuts LoftAuditShortcuts, selectExact bool,
) (*LoftAuditData, error) {
	var selected []bool
	var observe loftRowObserver
	var selector *loftFacetSelector
	if selectExact && shortcuts.BroadPhase && shortcuts.Certificates && !shortcuts.ExactInput &&
		loftCompleteRows(tree, members, len(tris)) && proofbound.AllFinite(verts) {
		selected = loftFlatSelection(verts, tris)
		if !slices.Contains(selected, false) {
			selected = nil
		} else {
			selector = &loftFacetSelector{verts: verts, tris: tris, flat: selected,
				uncertain: make([]atomic.Bool, len(tris)), shortcuts: shortcuts}
			observe = selector
		}
	}
	if err := preflightLoftRows(budget, tree, members, starts, ceiling, observe); err != nil {
		return nil, err
	}
	if selected == nil {
		return newCompactLoftAuditData(verts, tris), nil
	}
	for i := range selected {
		selected[i] = selected[i] || selector.uncertain[i].Load()
	}
	return newSelectedCompactLoftAuditData(verts, tris, selected), nil
}

// loftFlatSelection retains every z-flat triangle so the existing exact fan
// tables remain available. Other triangles need exact tables only when some
// candidate pair cannot be settled by loftFloatPairProof during row preflight.
func loftFlatSelection(verts []r3.Vec, tris [][3]int) []bool {
	selected := make([]bool, len(tris))
	for i, tri := range tris {
		selected[i] = verts[tri[0]].Z == verts[tri[1]].Z && verts[tri[0]].Z == verts[tri[2]].Z
	}
	return selected
}

// loftCompleteRows excludes filtered or partial candidate universes. With
// every triangle in an ordinary box tree, row preflight visits EVERY pair
// whose boxes overlap. A pair omitted by that tree has disjoint boxes and
// therefore no shared vertex or geometric contact.
func loftCompleteRows(tree loftRowScan, members []int, triangles int) bool {
	t, ok := tree.(treeScan)
	if !ok || len(members) != triangles || len(t.order) != triangles || len(t.boxes) != triangles {
		return false
	}
	for i, member := range members {
		if member != i {
			return false
		}
	}
	seen := make([]bool, triangles)
	for _, member := range t.order {
		if member < 0 || int(member) >= triangles || seen[member] {
			return false
		}
		seen[member] = true
	}
	return true
}

// newSelectedCompactLoftAuditData MUST follow successful full row preflight
// in prepareLoftStreamAuditData. Every candidate pair involving an unselected
// facet passed a certified float proof; otherwise BOTH facets were selected.
// Non-candidates have disjoint boxes (loftCompleteRows). Thus every pair
// involving an unselected facet has exactly its expected contact. Its verdict
// can be read from the shared indices WITHOUT repeating the float proof.
// This is a cache of completed proofs, not an admission from adjacency alone.
// The table owns immutable copies of the indexed geometry those proofs read.
func newSelectedCompactLoftAuditData(verts []r3.Vec, tris [][3]int, selected []bool) *LoftAuditData {
	need := make([]bool, len(verts))
	count := 0
	for i, tri := range tris {
		if selected[i] {
			count++
			need[tri[0]], need[tri[1]], need[tri[2]] = true, true, true
		}
	}
	if count == len(tris) {
		return newCompactLoftAuditData(verts, tris)
	}
	workers := loftDataWorkers(verts, len(tris))
	d := newLoftAuditDataSelected(verts, liftLoftVertices(verts, need, workers), tris, selected, true, workers)
	d.preflightProofs = true
	d.fans = newLoftAuditFans(d, d.tris)
	return d
}

func (d *LoftAuditData) preflightCertified(i, j int) bool {
	return d.preflightProofs && (d.Norms[i].X == nil || d.Norms[j].X == nil)
}
