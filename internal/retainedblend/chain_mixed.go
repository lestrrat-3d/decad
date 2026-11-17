package retainedblend

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// This file holds the selection that mixes closed tangent chains with other edges
// (agent-docs/charged-window-chamfer-design.md §3.4). The other edges take their own
// routes first, one at a time on the evolving mesh, and each chain is then found again
// on the result by its edges' end points and built.

// mixedChains holds the chains of a selection that also names other edges, as they were planned
// on the body the selection was made on, with the whole selection's edges and reference tags.
type mixedChains struct {
	edges  []*topo.Edge
	refs   []refTag
	chains []chainCycle
}

// checkRestApart refuses a selection whose unlinked edges may not be blended beside its
// chains: a fillet, or an edge that shares a vertex with a chain edge, keeps the refusal of a
// selection that names a chain and other edges. An open chain's end faces may be among the
// faces the other edges' blends rewrite; the chain is found again on their result and reads
// its end faces there, as a later call would. With no unlinked edge it accepts.
func checkRestApart(chains []chainCycle, rest []*topo.Edge, fillet bool) error {
	if len(rest) == 0 {
		return nil
	}
	refuse := fmt.Errorf("%w: a selection that holds a tangent chain names only chain edges",
		decaderr.ErrUnsupported)
	if fillet {
		return refuse
	}
	onChain := map[*topo.Vertex]struct{}{}
	for _, c := range chains {
		for _, e := range c.edges {
			onChain[e.Start], onChain[e.End] = struct{}{}, struct{}{}
		}
	}
	for _, e := range rest {
		_, a := onChain[e.Start]
		_, b := onChain[e.End]
		if a || b {
			return refuse
		}
	}
	return nil
}

// chainEdgeFinder is the predicate that finds a chain edge again on a partly blended body:
// the straight-edge or open-arc class of the per-edge routes, or the chain class
// (chainCandidate), which also reads the faces a charged chain holds.
func chainEdgeFinder(arc bool) func(*topo.Edge) bool {
	if arc {
		return func(e *topo.Edge) bool { return looksLikeArc(e) || chainCandidate(e) && isArcEdge(e) }
	}
	return func(e *topo.Edge) bool { return looksLikeLine(e) || chainCandidate(e) && !isArcEdge(e) }
}

// isArcEdge reports whether e is an Arc3 edge.
func isArcEdge(e *topo.Edge) bool {
	_, ok := e.Curve.(surfacegeom.Arc3)
	return ok
}
