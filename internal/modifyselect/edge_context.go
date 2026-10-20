// Package modifyselect renders and matches the selected edges of a modify
// call: the per-edge context text a refusal quotes, the section corner a
// selected lateral edge maps to, and the matched-corner context a failed audit
// carries.
package modifyselect

import (
	"fmt"
	"strings"

	"github.com/lestrrat-3d/decad/internal/selectorquery"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// EdgeContext renders one selected edge as its result ordinal plus the
// geometry that identifies it.
//
// Closure is decided by shared vertex identity, never by coordinates: both
// builders that mint a closed edge intern one *Vertex for its two ends
// (internal/facetedrecord/topology.go's vertexOf memoizes by mesh-vertex index; extrude.go
// assigns a full circle's edge the same start and end vertex), so the test is
// the pointer comparison edge.Start == edge.End. A Circle3 additionally
// reports the centre and radius no endpoint pair can carry, and falls under
// that same closed case; every other closed curve — including a
// boolean-built body's FacetedCurve rim — renders "closed through (p)"
// instead of repeating the one point as if it were two. Two DISTINCT
// vertices that merely sit at one position are a genuinely collapsed edge,
// not a closed one, and still render "from (p) to (p)" — that is the
// zero-length edge they are. Coordinates go through selectorquery.RenderVec,
// so value-equal geometry renders identically.
func EdgeContext(ordinal int, edge *topo.Edge) string {
	if edge == nil || edge.Start == nil || edge.End == nil {
		return fmt.Sprintf(`selected edge[%d]`, ordinal)
	}
	if c, ok := edge.Curve.(surfacegeom.Circle3); ok {
		return fmt.Sprintf(`selected edge[%d] closed circle through (%s), centre (%s), radius %s`,
			ordinal, selectorquery.RenderVec(edge.Start.Position), selectorquery.RenderVec(c.Center), c.Radius)
	}
	if c, ok := edge.Curve.(surfacegeom.Ellipse3); ok && edge.Start == edge.End {
		return fmt.Sprintf(`selected edge[%d] closed ellipse through (%s), centre (%s), semi-axes %s and %s`,
			ordinal, selectorquery.RenderVec(edge.Start.Position), selectorquery.RenderVec(c.Center),
			c.SemiMajor, c.SemiMinor)
	}
	if c, ok := edge.Curve.(surfacegeom.Ellipse3); ok {
		return fmt.Sprintf(`selected edge[%d] ellipse arc from (%s) to (%s), centre (%s), semi-axes %s and %s`,
			ordinal, selectorquery.RenderVec(edge.Start.Position), selectorquery.RenderVec(edge.End.Position),
			selectorquery.RenderVec(c.Center), c.SemiMajor, c.SemiMinor)
	}
	if edge.Start == edge.End {
		return fmt.Sprintf(`selected edge[%d] closed through (%s)`, ordinal, selectorquery.RenderVec(edge.Start.Position))
	}
	return fmt.Sprintf(`selected edge[%d] from (%s) to (%s)`, ordinal,
		selectorquery.RenderVec(edge.Start.Position), selectorquery.RenderVec(edge.End.Position))
}

// EdgesContext renders every selected edge, in result order, joined by "; ".
func EdgesContext(edges []*topo.Edge) string {
	contexts := make([]string, len(edges))
	for i, edge := range edges {
		contexts[i] = EdgeContext(i, edge)
	}
	return strings.Join(contexts, `; `)
}
