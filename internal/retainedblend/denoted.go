package retainedblend

import (
	"github.com/lestrrat-3d/decad/internal/stitchflux"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// tagIsRecord reports whether f's tag is the surface its record states, so a
// route may read the tag as exact. A face with no denoted patch is its own
// record. A revolve face carries one, and its tag is a float re-expression of
// it: only a Plane or Cylinder tag is admitted, and only where every leaf of
// the patch is a point, the axis frame is exactly orthonormal, and the tag
// equals the denoted surface exactly (stitchflux.TagIsDenoted). Any other
// surface kind is refused here, whatever TagIsDenoted says of it, because
// these routes read no cone, sphere or torus. A widened patch, a partial-sweep
// cap, a rotated placement or an off-axis sketch line fails the test, so those
// faces stay refused. The test reads recorded data by exact equality and never
// admits on a residual.
func tagIsRecord(f *topo.Face) bool {
	if f.Denoted == nil {
		return true
	}
	switch f.Surface.(type) {
	case surfacegeom.Plane, surfacegeom.Cylinder:
	default:
		return false
	}
	tagged := stitchflux.TaggedFace{Denoted: f.Denoted, Surface: f.Surface}
	for _, loop := range f.Loops {
		for _, ce := range loop.CoEdges {
			tagged.Curves = append(tagged.Curves, ce.Edge.Curve)
		}
	}
	return stitchflux.TagIsDenoted(tagged)
}
