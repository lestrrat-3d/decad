package modifyselect

import (
	"fmt"
	"strings"

	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/selectorquery"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// MatchedCorner retains one selector result beside the section coordinate it
// resolved to. The selected-edge ordinal follows SelectEdges' stable result
// order; the loop/corner coordinate and plane-local point identify the same
// junction in the recorded section.
type MatchedCorner struct {
	EdgeOrdinal int
	Edge        *topo.Edge
	Loop        int
	Corner      int
	Point       sectionrecord.Point2
}

// NewMatchedCorner records that the selected edge at edgeOrdinal maps to
// corner of loop, whose walk starts at the corner's section point.
func NewMatchedCorner(edgeOrdinal int, edge *topo.Edge, loop, corner int, cl survey2d.CornerLoop) MatchedCorner {
	w := cl.Walks[corner]
	return MatchedCorner{
		EdgeOrdinal: edgeOrdinal,
		Edge:        edge,
		Loop:        loop,
		Corner:      corner,
		Point:       sectionrecord.Point2{U: w.StartU, V: w.StartV},
	}
}

func (m MatchedCorner) String() string {
	return fmt.Sprintf(`%s maps to loop %d corner %d at (u, v) = (%s, %s)`,
		EdgeContext(m.EdgeOrdinal, m.Edge), m.Loop, m.Corner,
		selectorquery.RenderCoord(m.Point.U), selectorquery.RenderCoord(m.Point.V))
}

// WrapAuditError attaches the matched-entity context to an audit failure.
// The audit's own error — sentinel and reason — leads, and the selector plus the
// per-corner mappings follow it, so a refusal over many selected edges still
// reads its cause first. The wrapped sentinel is unchanged, so errors.Is keeps
// branching on it. selector is the selector's rendering.
func WrapAuditError(selector string, matched []MatchedCorner, err error) error {
	coordinates := make([]string, len(matched))
	for i, corner := range matched {
		coordinates[i] = corner.String()
	}
	err = sectionaudit.RenderCoordinates(err)
	return fmt.Errorf(`%w; selector %s matched [%s]`, err, selector, strings.Join(coordinates, `; `))
}
