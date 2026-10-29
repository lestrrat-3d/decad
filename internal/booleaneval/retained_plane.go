package booleaneval

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// retainedPlaneOf names the exact plane of each facet's source face when that
// face is a retained Plane whose tag is the plane itself: no stated normal
// departure, so no placement or similarity left the tag a float image
// (agent-docs/faceted-source-retention-design.md §11).
//
// The second result is the plane's stated displacement (topo.Face.AxialDelta):
// the true piece lies within it of the tag along the normal, and the slab
// charges it (meshbool.BoolMesh.SetChargedSlabBounds). A negative or non-finite
// displacement names no plane.
func retainedPlaneOf(m *topo.Mesh) func(facet int) (r3.Frame, float64, bool) {
	return func(facet int) (r3.Frame, float64, bool) {
		if facet >= len(m.Source) {
			return r3.Frame{}, 0, false
		}
		f := m.Source[facet]
		if f == nil || !f.RetainedTag || f.NormalBound != 0 || f.Denoted != nil {
			return r3.Frame{}, 0, false
		}
		pl, ok := f.Surface.(surfacegeom.Plane)
		if !ok {
			return r3.Frame{}, 0, false
		}
		delta := 0.0
		if f.HasAxialDelta {
			delta = f.AxialDelta
		}
		if !(delta >= 0) || math.IsInf(delta, 0) {
			return r3.Frame{}, 0, false
		}
		return pl.Frame, delta, true
	}
}

type withoutRetainedPlaneSlabKey struct{}

// WithoutRetainedPlaneSlab returns a context under which PrepBoolMesh reads no
// retained plane, so every facet keeps its FacetBound. It is the
// full-displacement control that tests compare a real run against; production
// code never sets it.
func WithoutRetainedPlaneSlab(ctx context.Context) context.Context {
	return context.WithValue(ctx, withoutRetainedPlaneSlabKey{}, struct{}{})
}

func retainedPlaneSlabDisabled(ctx context.Context) bool {
	_, ok := ctx.Value(withoutRetainedPlaneSlabKey{}).(struct{})
	return ok
}
