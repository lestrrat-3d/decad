package surfacegeom

import (
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Surface is the sealed face-geometry set (core §6.1): a tagged variant, not
// everything-is-NURBS, so intent is preserved. A switch on Surface MUST
// carry a default — vN adds variants.
type Surface interface {
	// Kind reports the discriminant.
	Kind() SurfaceKind
	surface()
}

// SurfaceKind is the discriminant a Surface reports; the constants are
// Kind-prefixed because the unprefixed names are the variant types.
type SurfaceKind int

const (
	// KindPlane is a planar face.
	KindPlane SurfaceKind = iota
	// KindCylinder is a right circular cylindrical face.
	KindCylinder
	// KindCone is a right circular conical face.
	KindCone
	// KindSphere is a spherical face.
	KindSphere
	// KindTorus is a toroidal face.
	KindTorus
	// KindNURBS is a NURBS face.
	KindNURBS
	// KindFaceted is the honest v1 boolean output: the analytic identity is
	// gone, and this variant is the flag.
	KindFaceted
)

// Plane is a planar face's geometry: the face lies in the frame's UV plane.
type Plane struct {
	Frame r3.Frame
}

// Cylinder is a right circular cylinder: Origin a point on the axis (mm),
// Axis the unit axis direction, Radius the cylinder radius.
type Cylinder struct {
	Origin r3.Vec
	Axis   r3.Vec
	Radius units.Value
}

// Cone is a right circular cone: Origin a point on the axis (mm), Axis the
// unit axis direction along which the radius GROWS, Radius the cone's radius
// at Origin (zero when Origin is the apex), HalfAngle the angle between the
// axis and the wall.
type Cone struct {
	Origin    r3.Vec
	Axis      r3.Vec
	Radius    units.Value
	HalfAngle units.Value
}

// Sphere is a sphere: Center (mm) and Radius.
type Sphere struct {
	Center r3.Vec
	Radius units.Value
}

// Torus is a torus: Center (mm) on the axis in the plane of the major
// circle, Axis the unit axis direction, Major the major-circle radius
// (center to tube center), Minor the tube radius. Major < Minor is a valid
// spindle torus — a revolved arc patch can sit on one without the surface
// self-intersecting.
type Torus struct {
	Center r3.Vec
	Axis   r3.Vec
	Major  units.Value
	Minor  units.Value
}

// NURBSSurface is a free-form face's geometry — the exact extruded or
// revolved surface of a recorded free-form curve (docs/spline-design.md §7):
// extruded, the control net is the curve's control points against the two
// sweep ends, degree (p, 1), weights carried through; revolved, the standard
// rational quadratic circle representation, degree (p, 2), weights
// multiplied. A NURBSSurface built from a recorded control net IS the
// surface, not an approximation of it — core §6.1's Exact-by-construction
// promise for an analytic variant holds unstrained. Its control net stays
// private in v1: this is a tagged, opaque marker carrying no exported
// geometry. Widening it later is compatible; narrowing an exposed net would
// not be.
type NURBSSurface struct{}

// Faceted is the honest v1 boolean-output variant (core §6.1): a face a
// boolean produced, whose analytic identity is gone. A Faceted face IS
// exactly its polygons — what it approximates is which surface it stands
// for, never what it is — and this variant's presence is exactly why a
// measurement on its body reads Approximate. Bound is the proven chord bound
// its polygons carry: no point of the surface the face stands for lies
// farther from them. The polygons themselves are read through
// Body.Tessellate (core §3 invariant #1 — triangles are an output, never
// the representation).
type Faceted struct {
	Bound units.Value
}

// Kind reports KindFaceted.
func (Faceted) Kind() SurfaceKind { return KindFaceted }

func (Faceted) surface() {}

// Kind reports KindPlane.
func (Plane) Kind() SurfaceKind { return KindPlane }

// Kind reports KindCylinder.
func (Cylinder) Kind() SurfaceKind { return KindCylinder }

// Kind reports KindCone.
func (Cone) Kind() SurfaceKind { return KindCone }

// Kind reports KindSphere.
func (Sphere) Kind() SurfaceKind { return KindSphere }

// Kind reports KindTorus.
func (Torus) Kind() SurfaceKind { return KindTorus }

// Kind reports KindNURBS.
func (NURBSSurface) Kind() SurfaceKind { return KindNURBS }

func (Plane) surface()        {}
func (Cylinder) surface()     {}
func (Cone) surface()         {}
func (Sphere) surface()       {}
func (Torus) surface()        {}
func (NURBSSurface) surface() {}

// Curve is the sealed edge-geometry set, Surface's one-dimensional analog.
// A switch on Curve MUST carry a default — vN adds variants.
type Curve interface{ curve() }

// Line3 is a straight edge between its vertices.
type Line3 struct{}

// Circle3 is a circular edge: Center (mm), Axis the unit normal of the
// circle's plane, Radius the circle radius. A full circle's edge has its
// start and end vertex coincide.
type Circle3 struct {
	Center r3.Vec
	Axis   r3.Vec
	Radius units.Value
}

// Arc3 is a circular arc edge on the circle (Center, Axis, Radius), swept
// counter-clockwise about Axis from the start vertex to the end vertex.
type Arc3 struct {
	Center r3.Vec
	Axis   r3.Vec
	Radius units.Value
}

// Ellipse3 is an elliptical arc edge: Center (mm), Axis the unit normal of the
// ellipse's plane, Major the unit direction of the semi-major axis (in that
// plane), SemiMajor and SemiMinor the semi-axis lengths. The arc is swept
// counter-clockwise about Axis from the start vertex to the end vertex, as
// Arc3 is, so the point at angle φ from Major is
// Center + SemiMajor·cos φ·Major + SemiMinor·sin φ·(Axis × Major)
// (docs/loop-fillet-design.md §2).
type Ellipse3 struct {
	Center, Axis, Major  r3.Vec
	SemiMajor, SemiMinor units.Value
}

// FilletMiter3 names the exact intersection edge of two adjacent fillet
// patches at a sharp corner involving a circular wall. The edge's vertices
// and adjacent faces identify the branch; its offset-foot parameterization
// stays in the fillet band's record.
type FilletMiter3 struct{}

// NURBSCurve is a free-form edge's geometry, NURBSSurface's 1-D analog
// (docs/spline-design.md §7). It reports no Kind at all: Curve is sealed by
// its marker method alone and declares no Kind method, so this variant seals
// in with that method and exports nothing else. Its control points stay
// private in v1, the same opaque-marker treatment as NURBSSurface.
type NURBSCurve struct{}

// FacetedCurve is Faceted's one-dimensional analog: a boolean-built edge —
// a chain of straight chords along the contact of two faceted faces, whose
// analytic identity is gone. Bound is the proven chord bound the chain
// carries.
type FacetedCurve struct {
	Bound units.Value
}

func (Line3) curve()        {}
func (Circle3) curve()      {}
func (Arc3) curve()         {}
func (Ellipse3) curve()     {}
func (FilletMiter3) curve() {}
func (NURBSCurve) curve()   {}
func (FacetedCurve) curve() {}

// CircleOf reads a circular curve's held center, axis and radius.
func CircleOf(c Curve) (r3.Vec, r3.Vec, float64, bool) {
	switch v := c.(type) {
	case Circle3:
		return v.Center, v.Axis, v.Radius.Base(), true
	case Arc3:
		return v.Center, v.Axis, v.Radius.Base(), true
	}
	return r3.Vec{}, r3.Vec{}, 0, false
}
