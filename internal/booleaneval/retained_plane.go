package booleaneval

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/stitchflux"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// retainedPlaneOf names the exact plane of each facet's source face when that
// face is a retained Plane whose tag is the plane itself: no stated normal
// departure, so no placement or similarity left the tag a float image
// (agent-docs/faceted-source-retention-design.md §11).
//
// A face with a denoted revolve patch (topo.Face.Denoted) names its plane only
// when the tag is exactly the plane the patch denotes, decided in exact
// arithmetic (stitchflux.TagIsDenoted). The face's true piece lies on the
// denoted surface (Claim R, §2), so it then lies on the tag plane itself. A
// tag that only re-expresses the denoted surface in floats names no plane.
//
// The second result is the plane's stated displacement (topo.Face.AxialDelta):
// the true piece lies within it of the tag along the normal, and the slab
// charges it (meshbool.BoolMesh.SetChargedSlabBounds). A negative or non-finite
// displacement names no plane.
//
// The returned function is not safe for concurrent use: it memoises the
// denoted-plane decision per source face.
func retainedPlaneOf(m *topo.Mesh) func(facet int) (r3.Frame, float64, bool) {
	denoted := map[*topo.Face]bool{}
	return func(facet int) (r3.Frame, float64, bool) {
		if facet >= len(m.Source) {
			return r3.Frame{}, 0, false
		}
		f := m.Source[facet]
		if claim := f.MeshPlaneSlab(); claim != nil {
			if claim.Distance >= 0 && !math.IsInf(claim.Distance, 0) {
				return claim.Frame, claim.Distance, true
			}
		}
		if f == nil || !f.RetainedTag || f.NormalBound != 0 {
			return r3.Frame{}, 0, false
		}
		if f.Denoted != nil {
			exact, seen := denoted[f]
			if !seen {
				exact = denotedPlaneIsTag(f)
				denoted[f] = exact
			}
			if !exact {
				return r3.Frame{}, 0, false
			}
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

// denotedPlaneIsTag reports whether f's Plane tag is exactly the plane its
// denoted revolve patch sweeps: a meridian whose every recorded end lies at
// one axial level, with the tag's frame vectors exactly perpendicular to the
// recorded axis and its origin exactly at that level
// (stitchflux.TagIsDenoted). The edge curves are not passed: they bear on the
// flux arms' rims, not on where the true piece lies.
func denotedPlaneIsTag(f *topo.Face) bool {
	if _, ok := f.Surface.(surfacegeom.Plane); !ok {
		return false
	}
	return stitchflux.TagIsDenoted(stitchflux.TaggedFace{Denoted: f.Denoted, Surface: f.Surface})
}

// denotedPlaneWork is the flat work a bounded replay charges for one
// denotedPlaneIsTag decision. The decision reads a fixed number of the
// record's float leaves as exact rationals and forms a fixed number of dot
// products of them, so its cost is a constant multiple of their size.
const denotedPlaneWork = 64

// denotedPlaneFaces counts the distinct source faces of m whose slab reading
// runs denotedPlaneIsTag: retained Plane faces with no stated normal
// departure that carry a denoted patch.
func denotedPlaneFaces(m *topo.Mesh) uint64 {
	seen := map[*topo.Face]struct{}{}
	for _, f := range m.Source {
		if f == nil || !f.RetainedTag || f.NormalBound != 0 || f.Denoted == nil {
			continue
		}
		if _, ok := f.Surface.(surfacegeom.Plane); ok {
			seen[f] = struct{}{}
		}
	}
	return uint64(len(seen))
}

type withoutRetainedPlaneSlabKey struct{}

// WithoutRetainedPlaneSlab returns a context under which PrepBoolMesh reads no
// retained plane or cylinder, so every facet keeps its FacetBound. It is the
// full-displacement control that tests compare a real run against; production
// code never sets it.
func WithoutRetainedPlaneSlab(ctx context.Context) context.Context {
	return context.WithValue(ctx, withoutRetainedPlaneSlabKey{}, struct{}{})
}

func retainedPlaneSlabDisabled(ctx context.Context) bool {
	_, ok := ctx.Value(withoutRetainedPlaneSlabKey{}).(struct{})
	return ok
}
