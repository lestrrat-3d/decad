package retainedblend

import (
	"fmt"

	clr "github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/selectorquery"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// chainShape reports whether e has the curve of a chain edge.
func chainShape(e *topo.Edge) bool {
	if e == nil || e.Start == nil || e.End == nil || e.Start == e.End {
		return false
	}
	switch e.Curve.(type) {
	case surfacegeom.Line3, surfacegeom.Arc3:
		return true
	}
	return false
}

type chainLink struct {
	other *topo.Edge
	at    *topo.Vertex
	j     junction
}

// planChains groups the selected edges into closed tangent chains
// (agent-docs/retained-modify-reach-design.md §5). handled is false when no
// selected edge ends at a proven tangent junction, and the caller keeps its
// per-edge routes. Each chain lists its edges in cyclic order and carries the
// junction between each edge and the next. A junction at the end of an arc that
// tangent proves a C3 corner (boss-root-overflow-design §5.5.7) is no chain
// link: the arc end takes the corner's own reading.
//
// The faces are read in tagChain mode: a chain may hold walls and a base that state a
// charge, which only a closed chamfer chain builds (refuseChargedChain; fillet says
// which). A selected edge with no link is returned in rest, for the caller to blend on
// its own when it touches no chain (checkRestApart).
func planChains(b *topo.Body, edges []*topo.Edge, tangent TangentEnds, fillet bool) (chains []chainCycle,
	rest []*topo.Edge, handled bool, err error) {
	all := topobuild.Edges(b)
	selected := map[*topo.Edge]struct{}{}
	for _, e := range edges {
		selected[e] = struct{}{}
	}
	links := map[*topo.Edge][]chainLink{}
	for _, e := range edges {
		if !chainCandidate(e) {
			continue
		}
		for _, v := range []*topo.Vertex{e.Start, e.End} {
			if provenEnd(tangent, all, e, v) {
				continue
			}
			var found []chainLink
			for _, c := range all {
				if c == e || c.Start == c.End || (c.Start != v && c.End != v) || len(c.Faces) != 2 {
					continue
				}
				j, state, near := tangentStateIn(e, c, v, tagChain)
				if near {
					return nil, nil, true, fmt.Errorf("%w: the junction at %s is nearly tangent and the tags cannot decide it",
						decaderr.ErrUnsupported, selectorquery.RenderVec(v.Position))
				}
				if state == clr.DegYes {
					found = append(found, chainLink{other: c, at: v, j: j})
				}
			}
			if len(found) > 1 {
				return nil, nil, true, fmt.Errorf("%w: the tangent chain branches at %s", decaderr.ErrUnsupported,
					selectorquery.RenderVec(v.Position))
			}
			if len(found) == 1 {
				links[e] = append(links[e], found[0])
			}
		}
	}
	if len(links) == 0 {
		return nil, nil, false, nil
	}
	for _, e := range edges {
		ls := links[e]
		if len(ls) == 0 {
			rest = append(rest, e)
			continue
		}
		for _, l := range ls {
			if _, ok := selected[l.other]; !ok {
				return nil, nil, true, fmt.Errorf("%w: the selected edge ends at the tangent junction %s whose "+
					"continuation is not selected; WithTangentChain selects the whole chain",
					decaderr.ErrUnsupported, selectorquery.RenderVec(l.at.Position))
			}
		}
	}
	seen := map[*topo.Edge]bool{}
	// An open chain starts at an edge with one link and ends at the other such
	// edge.
	for _, e := range edges {
		if seen[e] || len(links[e]) != 1 {
			continue
		}
		cyc := chainCycle{open: true}
		cur, via := e, (*topo.Vertex)(nil)
		for {
			seen[cur] = true
			cyc.edges = append(cyc.edges, cur)
			var next *chainLink
			for i := range links[cur] {
				if links[cur][i].at != via {
					next = &links[cur][i]
				}
			}
			if next == nil {
				break
			}
			cyc.joins = append(cyc.joins, *next)
			cur, via = next.other, next.at
			if seen[cur] {
				return nil, nil, true, fmt.Errorf("%w: a tangent chain does not close", decaderr.ErrUnsupported)
			}
		}
		chains = append(chains, cyc)
	}
	for _, e := range edges {
		if seen[e] || len(links[e]) == 0 {
			continue
		}
		var cyc chainCycle
		cur, via := e, (*topo.Vertex)(nil)
		for {
			seen[cur] = true
			cyc.edges = append(cyc.edges, cur)
			var next *chainLink
			for i := range links[cur] {
				if links[cur][i].at != via {
					next = &links[cur][i]
				}
			}
			if next == nil {
				return nil, nil, true, fmt.Errorf("%w: a tangent chain does not close", decaderr.ErrUnsupported)
			}
			cyc.joins = append(cyc.joins, *next)
			cur, via = next.other, next.at
			if cur == e {
				break
			}
			if seen[cur] {
				return nil, nil, true, fmt.Errorf("%w: a tangent chain does not close", decaderr.ErrUnsupported)
			}
		}
		if len(cyc.edges) < 3 {
			return nil, nil, true, fmt.Errorf("%w: a closed tangent chain needs at least three edges",
				decaderr.ErrUnsupported)
		}
		chains = append(chains, cyc)
	}
	for _, cyc := range chains {
		if err := refuseChargedChain(cyc, fillet); err != nil {
			return nil, nil, true, err
		}
	}
	if err := checkRestApart(chains, rest, fillet); err != nil {
		return nil, nil, true, err
	}
	return chains, rest, true, nil
}

// chainCycle is one chain: edges[i] meets edges[i+1] at joins[i]. A closed
// chain has as many joins as edges, and joins[n-1] meets edges[0]. An open
// chain has one join fewer and two free ends.
type chainCycle struct {
	edges []*topo.Edge
	joins []chainLink
	open  bool
}

// freeEnds are the two vertices where an open chain ends: the vertex of the
// first edge that is no junction, and the same for the last edge.
func (c chainCycle) freeEnds() (first, last *topo.Vertex) {
	n := len(c.edges)
	other := func(e *topo.Edge, v *topo.Vertex) *topo.Vertex {
		if e.Start == v {
			return e.End
		}
		return e.Start
	}
	return other(c.edges[0], c.joins[0].at), other(c.edges[n-1], c.joins[n-2].at)
}
