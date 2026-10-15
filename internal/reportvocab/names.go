package reportvocab

import "github.com/lestrrat-3d/decad/internal/surfacegeom"

// SurfaceTypeName names the root package's public form of s in a public
// error message, as fmt's %T prints it: each surfacegeom variant reads as
// the root Surface variant that carries it, a nil surface as "<nil>", and a
// variant without a public form as the root borrowed surface.
func SurfaceTypeName(s surfacegeom.Surface) string {
	switch s.(type) {
	case nil:
		return "<nil>"
	case surfacegeom.Plane:
		return "decad.Plane"
	case surfacegeom.Cylinder:
		return "decad.Cylinder"
	case surfacegeom.Cone:
		return "decad.Cone"
	case surfacegeom.Sphere:
		return "decad.Sphere"
	case surfacegeom.Torus:
		return "decad.Torus"
	case surfacegeom.NURBSSurface:
		return "decad.NURBSSurface"
	case surfacegeom.Faceted:
		return "decad.Faceted"
	default:
		return "decad.borrowedSurface"
	}
}

// CurveTypeName names the root package's public form of c in a public error
// message, as fmt's %T prints it: each surfacegeom variant reads as the root
// Curve variant that carries it, a nil curve as "<nil>", and a variant
// without a public form as the root borrowed curve.
func CurveTypeName(c surfacegeom.Curve) string {
	switch c.(type) {
	case nil:
		return "<nil>"
	case surfacegeom.Line3:
		return "decad.Line3"
	case surfacegeom.Circle3:
		return "decad.Circle3"
	case surfacegeom.Arc3:
		return "decad.Arc3"
	case surfacegeom.Ellipse3:
		return "decad.Ellipse3"
	case surfacegeom.FilletMiter3:
		return "decad.FilletMiter3"
	case surfacegeom.NURBSCurve:
		return "decad.NURBSCurve"
	case surfacegeom.FacetedCurve:
		return "decad.FacetedCurve"
	default:
		return "decad.borrowedCurve"
	}
}
