package booleaneval

import (
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// retainedPlaneOf names the exact plane of each facet's source face when that
// face is a retained Plane whose tag is the plane itself: no stated normal
// departure, so no placement or similarity left the tag a float image
// (agent-docs/faceted-source-retention-design.md §11).
func retainedPlaneOf(m *topo.Mesh) func(facet int) (r3.Frame, bool) {
	return func(facet int) (r3.Frame, bool) {
		if facet >= len(m.Source) {
			return r3.Frame{}, false
		}
		f := m.Source[facet]
		if f == nil || !f.RetainedTag || f.NormalBound != 0 || f.Denoted != nil {
			return r3.Frame{}, false
		}
		pl, ok := f.Surface.(surfacegeom.Plane)
		if !ok {
			return r3.Frame{}, false
		}
		return pl.Frame, true
	}
}
