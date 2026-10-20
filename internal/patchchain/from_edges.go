package patchchain

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/selectorquery"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// ChainsFromEdges proves the closed chains the selected free edges form: every
// edge must have exactly one adjacent face, the selection must partition into
// closed chains, and each chain must lie in one plane.
func ChainsFromEdges(edges []*topo.Edge) ([]Chain, error) {
	for _, e := range edges {
		if len(e.Faces) != 1 {
			return nil, fmt.Errorf(`%w: PatchEdges selection holds an edge that is not free (docs/surface-design.md Table R row R4)`, decaderr.ErrDegenerate)
		}
	}
	refs := make([]EdgeRef[*topo.Edge, *topo.Vertex], len(edges))
	for i, e := range edges {
		refs[i] = EdgeRef[*topo.Edge, *topo.Vertex]{Edge: e, Start: e.Start, End: e.End}
	}
	groups, err := Partition(refs, func(v *topo.Vertex) string { return renderCoord3(v.Position) })
	if err != nil {
		return nil, err
	}
	chains := make([]Chain, len(groups))
	for i, g := range groups {
		lvl, axialBound, hasAxialBound, err := ProveChainPlane(g)
		if err != nil {
			return nil, err
		}
		chains[i] = Chain{Edges: g, AxialBound: axialBound, HasAxialBound: hasAxialBound, Level: lvl}
	}
	return chains, nil
}

// renderCoord3 renders a world point the way a selector message does.
func renderCoord3(p r3.Vec) string {
	return fmt.Sprintf(`(%s, %s, %s)`, selectorquery.RenderCoord(p.X), selectorquery.RenderCoord(p.Y), selectorquery.RenderCoord(p.Z))
}
