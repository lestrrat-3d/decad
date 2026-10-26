package retainedblend

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
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

// sphereIsRecord refuses a retained Sphere face whose tag is not exactly the
// sphere its revolve record denotes (agent-docs/retained-edge-blend-design.md
// §4.5). A face with no denoted patch is its own record. Otherwise every leaf of
// the circular patch must be a point and its axis frame exactly orthonormal; the
// recorded centre must lie on the axis and the tag's centre must be that axis
// point exactly; and the recorded radius must be a point equal to the tag's
// radius exactly. stitchflux.TagIsDenoted reads no radius, so the radius rule is
// this route's own. Each test compares recorded rationals by exact equality and
// never admits on a residual.
func sphereIsRecord(f *topo.Face, s surfacegeom.Sphere) error {
	if f.Denoted == nil {
		return nil
	}
	r := *f.Denoted
	ex, ok := stitchflux.ExactRevolveOf(r)
	if !r.Valid || r.Cap || !r.Circular || !ok {
		return fmt.Errorf("%w: the sphere's revolve record is not exact: its axis frame or centre is widened",
			decaderr.ErrUnsupported)
	}
	if ex.Centre[1].Sign() != 0 || !ex.AtAxis(s.Center, ex.Centre[0]) {
		return fmt.Errorf("%w: the sphere's tag centre is not exactly its recorded centre on the revolve axis",
			decaderr.ErrUnsupported)
	}
	radius, ok := stitchflux.RatMillimetres(s.Radius)
	if !ok || ex.Radius == nil || ex.Radius.Cmp(radius) != 0 {
		return fmt.Errorf("%w: the sphere's tag radius is not exactly its recorded radius", decaderr.ErrUnsupported)
	}
	return nil
}
