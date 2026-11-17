// Package facedeparture lets the STEP writer (export) read whether a face's
// surface tag departs from the surface the face carries. The root package
// installs Reader when it loads; export calls Stated. It adds no public API.
package facedeparture

import (
	"github.com/lestrrat-3d/decad/internal/stitchflux"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// Reader reports whether a *decad.Face departs from its tag (TagDeparts on
// the node it holds). Any other value, which it cannot read, reports true.
// The root package sets it once at load time.
var Reader func(face any) bool

// Stated reports whether face departs from its tag. Before the root package
// has installed Reader it reports true, so a reader that only refuses on it
// refuses every face rather than admitting one unread.
func Stated(face any) bool {
	if Reader == nil {
		return true
	}
	return Reader(face)
}

// ChargedReader reports whether a *decad.Face holds a charged wall (ChargedTag
// on the node it holds). Any other value, which it cannot read, reports true.
// The root package sets it once at load time.
var ChargedReader func(face any) bool

// Charged reports whether face is a curved face whose tag states a nonzero wall
// charge: the true wall may lie off the tag by that charge, so a reader that
// takes the tag's radius and axis as the wall's must not read it. Before the
// root package has installed ChargedReader it reports true, so a reader that
// only refuses on it refuses every face rather than admitting one unread.
func Charged(face any) bool {
	if ChargedReader == nil {
		return true
	}
	return ChargedReader(face)
}

// ChargedTag reports whether f's tag states a charge: a nonzero AxialDelta on
// a tag that is not a plane (a retained Cylinder wall whose true surface lies
// within it), or a nonzero ExtentDelta on a plane (an oblique wall whose true
// patch lies within it of the tag over the face). NaN is nonzero. A plane's
// AxialDelta is read by TagDeparts.
func ChargedTag(f *topo.Face) bool {
	if _, plane := f.Surface.(surfacegeom.Plane); plane {
		// An oblique wall's tag lies off its true plane by ExtentDelta over
		// the face's extent and not beyond it, so a reader that takes the tag
		// as the plane must not read it. NaN is nonzero.
		return f.ExtentDelta != 0
	}
	return f.HasAxialDelta && f.AxialDelta != 0
}

// TagDeparts reports whether f's tag is not proven to be the surface f
// carries: f states a nonzero NormalBound or HeldNormalBound, a nonzero
// AxialDelta (a planar face displaced along its own normal from the level its
// tag holds), or f carries a denoted surface (topo.Face.Denoted, a revolve
// face's record) that its tag is not proven exactly equal to (tagIsDenoted).
// It only refuses: a false answer means no departure is stated and none is
// left unproven.
func TagDeparts(f *topo.Face) bool {
	if f.NormalBound != 0 || f.HeldNormalBound != 0 || f.AxialDelta != 0 {
		return true
	}
	return f.Denoted != nil && !tagIsDenoted(f)
}

// tagIsDenoted reports whether a face with a denoted surface has a tag that
// is exactly that surface, by exact rational equality on the recorded data
// (stitchflux.TagIsDenoted). A Plane, Cylinder or Torus tag is decided there.
// A Sphere tag also needs its radius to equal the record's (sphereRadiusIsRecord),
// which TagIsDenoted does not read. A Cone tag is never admitted:
// TagIsDenoted checks only that the meridian lines meet at one apex and that
// the axis is the record's, never the tag's own apex or half-angle, and the
// revolve builds both from a float apex and an atan2 half-angle.
func tagIsDenoted(f *topo.Face) bool {
	tagged := stitchflux.TaggedFace{Denoted: f.Denoted, Surface: f.Surface}
	for _, loop := range f.Loops {
		for _, ce := range loop.CoEdges {
			tagged.Curves = append(tagged.Curves, ce.Edge.Curve)
		}
	}
	switch s := f.Surface.(type) {
	case surfacegeom.Plane, surfacegeom.Cylinder, surfacegeom.Torus:
		return stitchflux.TagIsDenoted(tagged)
	case surfacegeom.Sphere:
		return stitchflux.TagIsDenoted(tagged) && sphereRadiusIsRecord(f, s)
	}
	return false
}

// sphereRadiusIsRecord reports whether a sphere tag's radius equals its
// circular record's recorded radius exactly. A widened radius leaf refuses.
func sphereRadiusIsRecord(f *topo.Face, s surfacegeom.Sphere) bool {
	ex, ok := stitchflux.ExactRevolveOf(*f.Denoted)
	if !ok || ex.Radius == nil {
		return false
	}
	radius, ok := stitchflux.RatMillimetres(s.Radius)
	return ok && ex.Radius.Cmp(radius) == 0
}
