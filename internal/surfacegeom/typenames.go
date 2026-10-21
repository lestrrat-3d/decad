package surfacegeom

// SurfaceTypeName names the root package's public form of s in a public
// error message, as fmt's %T prints it: each surfacegeom variant reads as
// the root Surface variant that carries it, a nil surface as "<nil>", and a
// variant without a public form as the root borrowed surface.
func SurfaceTypeName(s Surface) string {
	switch s.(type) {
	case nil:
		return "<nil>"
	case Plane:
		return "decad.Plane"
	case Cylinder:
		return "decad.Cylinder"
	case Cone:
		return "decad.Cone"
	case Sphere:
		return "decad.Sphere"
	case Torus:
		return "decad.Torus"
	case NURBSSurface:
		return "decad.NURBSSurface"
	case Faceted:
		return "decad.Faceted"
	default:
		return "decad.borrowedSurface"
	}
}

// CurveTypeName names the root package's public form of c in a public error
// message, as fmt's %T prints it: each surfacegeom variant reads as the root
// Curve variant that carries it, a nil curve as "<nil>", and a variant
// without a public form as the root borrowed curve.
func CurveTypeName(c Curve) string {
	switch c.(type) {
	case nil:
		return "<nil>"
	case Line3:
		return "decad.Line3"
	case Circle3:
		return "decad.Circle3"
	case Arc3:
		return "decad.Arc3"
	case Ellipse3:
		return "decad.Ellipse3"
	case FilletMiter3:
		return "decad.FilletMiter3"
	case NURBSCurve:
		return "decad.NURBSCurve"
	case FacetedCurve:
		return "decad.FacetedCurve"
	default:
		return "decad.borrowedCurve"
	}
}
