package patchchain

import (
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
)

// CircularPrismSource admits a complete single-circle surface
// extrusion. Its one source face supplies both rim station grids, and each
// selected rim is exactly one of the source body's own full-circle edges.
func CircularPrismSource(pp BodyPayload) (prismrecord.Payload, bool) {
	if len(pp.Faces) != 1 || len(pp.Chains) == 0 || len(pp.Chains) > 2 || pp.Faces[0].Body == nil {
		return prismrecord.Payload{}, false
	}
	prism, ok := pp.Faces[0].Body.Payload.(prismrecord.Payload)
	if !ok || !prism.SurfaceResult || prism.SectionDelta != 0 || len(prism.Profile.Holes) != 0 ||
		len(prism.Profile.Outer.Segments) != 1 {
		return prismrecord.Payload{}, false
	}
	if _, ok := pp.Faces[0].Surface.(surfacegeom.Cylinder); !ok {
		return prismrecord.Payload{}, false
	}
	for _, chain := range pp.Chains {
		if len(chain.Edges) != 1 {
			return prismrecord.Payload{}, false
		}
		if _, ok := chain.Edges[0].Curve.(surfacegeom.Circle3); !ok {
			return prismrecord.Payload{}, false
		}
		if len(chain.Edges[0].Faces) != 1 || chain.Edges[0].Faces[0] != pp.Faces[0] {
			return prismrecord.Payload{}, false
		}
	}
	return prism, true
}
