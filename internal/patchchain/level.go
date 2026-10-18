package patchchain

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/denotation"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// ProvePlaneLevel is gate 3's second arm: it requires every chain vertex,
// and every chain edge, to carry a level token with the SAME non-zero id
// (denotation.go) — an identity comparison alone, never a coordinate, a
// residual or a bound magnitude. A shared level proves the chain's true
// vertices lie on one plane by shared construction
// (docs/surface-design.md §5.2), so nothing here reads a position or a
// bound to decide admission; the returned token and bound are read back off
// the chain's own vertices only to state what the new face's own plane and
// AxialDelta must publish, never to decide whether this arm admits. ok is
// false whenever the chain holds no vertex, any vertex or edge carries no
// level (the zero value, "no certificate"), or not every one of them shares
// the same id.
func ProvePlaneLevel(edges []*topo.Edge) (denotation.LevelToken, float64, bool) {
	verts := ChainVertices(edges)
	if len(verts) == 0 {
		return denotation.LevelToken{}, 0, false
	}
	lvl := verts[0].Level
	if lvl.ID == 0 {
		return denotation.LevelToken{}, 0, false
	}
	bound := 0.0
	for _, v := range verts {
		if !denotation.SameLevel(v.Level, lvl) {
			return denotation.LevelToken{}, 0, false
		}
		bound = math.Max(bound, v.Bound.Base())
	}
	for _, e := range edges {
		if !denotation.SameLevel(e.Level, lvl) {
			return denotation.LevelToken{}, 0, false
		}
	}
	return lvl, bound, true
}

// ChainVertices returns the chain's distinct vertices, deduplicated by
// pointer identity, in first-seen order.
func ChainVertices(edges []*topo.Edge) []*topo.Vertex {
	seen := map[*topo.Vertex]struct{}{}
	var out []*topo.Vertex
	for _, e := range edges {
		for _, v := range [2]*topo.Vertex{e.Start, e.End} {
			if _, ok := seen[v]; ok {
				continue
			}
			seen[v] = struct{}{}
			out = append(out, v)
		}
	}
	return out
}
