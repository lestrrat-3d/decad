package decad

import (
	"fmt"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the topology model of docs/evaluator-design.md §3: concrete
// structs with unexported fields; the accessors of core §6/§6.1 are the whole
// public surface. Identity is the pointer, scoped to one body — never an
// index. A Body is immutable after construction, so it is safe to read from
// many goroutines (core §12).

// FeatureRef identifies the feature role that created a body, a face or an
// edge: the producing StepRef plus a stable role within that step. Roles
// derive from the recorded step, so re-evaluation reproduces them
// (docs/evaluator-design.md §3).
type FeatureRef struct {
	// Step is the recipe step that produced the entity.
	Step StepRef `json:"step"`
	// Role names the entity within the step: "side(i,j)" (loop i, segment
	// j), "capStart", "capEnd", "body".
	Role string `json:"role"`
}

// Surface is the sealed face-geometry set (core §6.1): a tagged variant, not
// everything-is-NURBS, so intent is preserved. A switch on Surface MUST
// carry a default — vN adds variants.
type Surface interface {
	// Kind reports the discriminant.
	Kind() SurfaceKind
	surface()
}

// SurfaceKind is the discriminant a Surface reports; the constants are
// Kind-prefixed because the unprefixed names are the variant types (core §6.2).
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

// Kind reports KindPlane.
func (Plane) Kind() SurfaceKind { return KindPlane }

// Kind reports KindCylinder.
func (Cylinder) Kind() SurfaceKind { return KindCylinder }

func (Plane) surface()    {}
func (Cylinder) surface() {}

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

func (Line3) curve()   {}
func (Circle3) curve() {}
func (Arc3) curve()    {}

// Vertex is a topological point.
type Vertex struct {
	position r3.Vec
	bound    units.Value
}

// Position returns the vertex position in millimetres — a computed
// coordinate, so it is bounded (core §6): feature-built vertices are Exact
// with a zero bound, boolean-built ones carry the tessellation's chord bound.
func (v *Vertex) Position() VecMeasurement {
	exactness := Exact
	if v.bound.Mag() != 0 {
		exactness = Approximate
	}
	return VecMeasurement{Value: v.position, Exactness: exactness, Bound: v.bound}
}

// coedge is one directed use of an edge by a loop.
type coedge struct {
	edge    *Edge
	forward bool
}

// Edge is a topological edge: a tagged curve between two vertices, with the
// faces adjacent to it.
type Edge struct {
	curve  Curve
	start  *Vertex
	end    *Vertex
	faces  []*Face
	convex bool
	length float64 // millimetres, exact for the v1 curve set
}

// Curve returns the edge's tagged geometry.
func (e *Edge) Curve() Curve { return e.curve }

// Start returns the edge's start vertex.
func (e *Edge) Start() *Vertex { return e.start }

// End returns the edge's end vertex.
func (e *Edge) End() *Vertex { return e.end }

// Faces returns the faces adjacent to this edge. On a closed manifold body
// every edge bounds exactly two faces; len != 2 is precisely how non-manifold
// topology is observable (core §6.1).
func (e *Edge) Faces() []*Face { return append([]*Face(nil), e.faces...) }

// IsConvex reports whether the material angle across the edge is convex — a
// decided answer, not an approximation of one (core §6).
func (e *Edge) IsConvex() bool { return e.convex }

// Length returns the edge's length. Exact (zero bound) for every curve the
// v1 features build.
func (e *Edge) Length() (Measurement, error) {
	return Measurement{Value: units.Millimeters(e.length), Exactness: Exact, Bound: units.Millimeters(0)}, nil
}

// Loop is one boundary loop of a face.
type Loop struct {
	coedges []coedge
	outer   bool
}

// IsOuter reports whether this is the face's outer boundary; false for a
// hole.
func (l *Loop) IsOuter() bool { return l.outer }

// Edges returns the loop's edges in walk order.
func (l *Loop) Edges() []*Edge {
	out := make([]*Edge, len(l.coedges))
	for i, ce := range l.coedges {
		out[i] = ce.edge
	}
	return out
}

// Face is a topological face: a tagged surface bounded by loops.
type Face struct {
	surface Surface
	loops   []*Loop
	origins []FeatureRef
	body    *Body
	area    float64 // millimetres², exact for the v1 face set
	// reversed is true when the OUTWARD (material-leaving) normal is the
	// surface's geometric normal negated — a hole's cylinder wall, whose
	// material lies outside the cylinder.
	reversed bool
}

// NormalAt returns the face's outward normal at p — a computed direction, so
// it is a measurement (core §6.1): Exact with a zero dimensionless bound for
// the analytic v1 faces. A point that gives the surface no direction — a
// cylinder's own axis — is ErrDegenerate.
func (f *Face) NormalAt(p r3.Vec) (VecMeasurement, error) {
	sign := 1.0
	if f.reversed {
		sign = -1
	}
	switch s := f.surface.(type) {
	case Plane:
		return VecMeasurement{Value: s.Frame.N().Scale(sign), Exactness: Exact, Bound: units.Scalar(0)}, nil
	case Cylinder:
		rel := p.Sub(s.Origin)
		radial := rel.Sub(s.Axis.Scale(rel.Dot(s.Axis)))
		dir, ok := radial.Normalize()
		if !ok {
			return VecMeasurement{}, fmt.Errorf(`%w: a point on the cylinder axis has no normal`, ErrDegenerate)
		}
		return VecMeasurement{Value: dir.Scale(sign), Exactness: Exact, Bound: units.Scalar(0)}, nil
	default:
		return VecMeasurement{}, fmt.Errorf(`%w: this evaluator computes normals for its own analytic faces only`, ErrUnsupported)
	}
}

// Surface returns the face's tagged geometry.
func (f *Face) Surface() Surface { return f.surface }

// Loops returns the face's boundary loops, the outer loop first.
func (f *Face) Loops() []*Loop { return append([]*Loop(nil), f.loops...) }

// Edges returns the face's edges across all loops.
func (f *Face) Edges() []*Edge {
	var out []*Edge
	for _, l := range f.loops {
		out = append(out, l.Edges()...)
	}
	return out
}

// Area returns the face's area. Exact (zero bound) for every face the v1
// features build.
func (f *Face) Area() (Measurement, error) {
	return Measurement{Value: units.SquareMillimeters(f.area), Exactness: Exact, Bound: units.SquareMillimeters(0)}, nil
}

// Origins returns every feature role that created this face —
// canonicalization may merge coplanar faces, and a merged face carries ALL
// contributing roles (core §6.1).
func (f *Face) Origins() []FeatureRef { return append([]FeatureRef(nil), f.origins...) }

// Shell is a connected set of faces.
type Shell struct {
	faces []*Face
	void  bool
}

// IsVoid reports whether the shell bounds an internal cavity.
func (s *Shell) IsVoid() bool { return s.void }

// Faces returns the shell's faces.
func (s *Shell) Faces() []*Face { return append([]*Face(nil), s.faces...) }

// Lump is a connected solid piece of a body.
type Lump struct {
	shells []*Shell
}

// Shells returns the lump's shells, the outer shell first.
func (l *Lump) Shells() []*Shell { return append([]*Shell(nil), l.shells...) }

// Body is an immutable solid: every operation returns a new one, and the
// input body is retired from its document (core §6).
type Body struct {
	doc    *Document
	origin FeatureRef
	lumps  []*Lump

	// The evaluator computes measurements at build time; a Body never
	// recomputes (immutability is what makes it goroutine-safe to read).
	volume   Measurement
	area     Measurement
	centroid VecMeasurement
	bounds   Box
	solid    bool

	// payload is the evaluator's own record of how this body was built —
	// what Placed re-evaluates under a composed motion
	// (docs/evaluator-design.md §8). Nil for a body this evaluator did not
	// build.
	payload *prismPayload
}

// bodyRef seals *Body into BodyRef: a live body is what a caller passes at a
// feature call.
func (*Body) bodyRef() {}

// Document returns the document that owns (or owned) this body.
func (b *Body) Document() *Document { return b.doc }

// Origin returns the feature that created this body.
func (b *Body) Origin() FeatureRef { return b.origin }

// Lumps returns the body's disjoint pieces; more than one means the body is
// disconnected.
func (b *Body) Lumps() []*Lump { return append([]*Lump(nil), b.lumps...) }

// Shells returns the body's shells across all lumps.
func (b *Body) Shells() []*Shell {
	var out []*Shell
	for _, l := range b.lumps {
		out = append(out, l.Shells()...)
	}
	return out
}

// Faces returns the body's faces across all shells.
func (b *Body) Faces() []*Face {
	var out []*Face
	for _, s := range b.Shells() {
		out = append(out, s.Faces()...)
	}
	return out
}

// Edges returns the body's edges, deduplicated.
func (b *Body) Edges() []*Edge {
	seen := map[*Edge]struct{}{}
	var out []*Edge
	for _, f := range b.Faces() {
		for _, e := range f.Edges() {
			if _, ok := seen[e]; ok {
				continue
			}
			seen[e] = struct{}{}
			out = append(out, e)
		}
	}
	return out
}

// Vertices returns the body's vertices, deduplicated.
func (b *Body) Vertices() []*Vertex {
	seen := map[*Vertex]struct{}{}
	var out []*Vertex
	for _, e := range b.Edges() {
		for _, v := range []*Vertex{e.start, e.end} {
			if v == nil {
				continue
			}
			if _, ok := seen[v]; ok {
				continue
			}
			seen[v] = struct{}{}
			out = append(out, v)
		}
	}
	return out
}

// IsSolid reports whether the body is a valid solid — a decided answer
// (core §6).
func (b *Body) IsSolid() bool { return b.solid }

// Bounds returns the body's axis-aligned bounding box, as tight as the
// evaluator can prove, reporting its own exactness (core §6).
func (b *Body) Bounds() (Box, error) { return b.bounds, nil }

// Volume returns the enclosed volume. A body that is not a solid encloses
// none: ErrNotSolid, never zero (core §4/§6).
func (b *Body) Volume() (Measurement, error) {
	if !b.solid {
		return Measurement{}, ErrNotSolid
	}
	return b.volume, nil
}

// Area returns the total surface area of the body's boundary.
func (b *Body) Area() (Measurement, error) { return b.area, nil }

// Centroid returns the centroid of the enclosed region. A body that is not a
// solid has none: ErrNotSolid.
func (b *Body) Centroid() (VecMeasurement, error) {
	if !b.solid {
		return VecMeasurement{}, ErrNotSolid
	}
	return b.centroid, nil
}
