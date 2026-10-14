package decad

import (
	"fmt"
	"math"
	"sync/atomic"

	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the topology model of docs/evaluator-design.md §3: concrete
// structs with unexported fields; the accessors of core §6/§6.1 are the whole
// public surface. Identity is the pointer, scoped to one body — never an
// index. A Body is immutable after construction, so it is safe to read from
// many goroutines (core §12).

// The stable role name only the root mints (evaluator §3); the roles every
// analytic evaluator mints live in internal/provenance.
const (
	// rolePatch is a Patch face's own Origin() role (docs/surface-design.md
	// §5.1). It stays internal this increment: a one-face body is already
	// selected by Faces(Planar()), and no public FeatureRef helper names it
	// yet — Stitch is what decides the right shape for one.
	rolePatch = "patch"
)

// FeatureRef identifies the stable role that created a body, face, or edge.
// The producing operation is tracked privately so callers cannot forge or
// depend on document-local operation identities.
type FeatureRef struct {
	producer producerID
	// Role names the entity within the producing feature: "side(i,j)" (loop i, segment
	// j), "capStart", "capEnd", "body".
	Role string
}

// Surface is the sealed face-geometry set. A switch on it needs a default
// because later versions may add variants.
type Surface interface {
	Kind() SurfaceKind
	surface()
}

// SurfaceKind identifies a surface variant.
type SurfaceKind int

const (
	KindPlane SurfaceKind = iota
	KindCylinder
	KindCone
	KindSphere
	KindTorus
	KindNURBS
	KindFaceted
)

// BodyKind states what a body IS: whether its boundary encloses a material
// region. It is decided by the operation that built the body, never inferred
// from the boundary afterwards (docs/surface-design.md §2.1, core §6).
type BodyKind int

const (
	// BodySolid is a body that encloses a material region. Its region
	// quantities are the question; Volume and Centroid answer once validity
	// proves the boundary closed.
	BodySolid BodyKind = iota
	// BodySheet is a body that encloses no material region. It is a boundary
	// on its own, and Volume and Centroid are ErrNotSolid for every sheet,
	// proven sound or not.
	BodySheet
)

// Plane is a planar face's geometry in the frame's UV plane.
type Plane struct{ Frame r3.Frame }

// Cylinder is a right circular cylinder. Origin lies on its axis, Axis is a
// unit direction, and Radius is its radius.
type Cylinder struct {
	Origin r3.Vec
	Axis   r3.Vec
	Radius units.Value
}

// Cone is a right circular cone. Axis points toward growing radius, and
// Radius is measured at Origin.
type Cone struct {
	Origin    r3.Vec
	Axis      r3.Vec
	Radius    units.Value
	HalfAngle units.Value
}

// Sphere is a spherical face with Center in millimetres.
type Sphere struct {
	Center r3.Vec
	Radius units.Value
}

// Torus is a toroidal face. Major measures from Center to the tube center;
// Minor is the tube radius. A spindle torus may have Major below Minor.
type Torus struct {
	Center r3.Vec
	Axis   r3.Vec
	Major  units.Value
	Minor  units.Value
}

// NURBSSurface marks a free-form face. Its control net remains private.
type NURBSSurface struct{}

// Faceted marks a polygonal face. Bound encloses the displacement from its
// held polygons to the boundary patches they represent.
type Faceted struct{ Bound units.Value }

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

// Kind reports KindFaceted.
func (Faceted) Kind() SurfaceKind { return KindFaceted }

func (Plane) surface()        {}
func (Cylinder) surface()     {}
func (Cone) surface()         {}
func (Sphere) surface()       {}
func (Torus) surface()        {}
func (NURBSSurface) surface() {}
func (Faceted) surface()      {}

func publicSurface(s surfacegeom.Surface) Surface {
	if s == nil {
		return nil
	}
	switch v := s.(type) {
	case surfacegeom.Plane:
		return Plane(v)
	case surfacegeom.Cylinder:
		return Cylinder(v)
	case surfacegeom.Cone:
		return Cone(v)
	case surfacegeom.Sphere:
		return Sphere(v)
	case surfacegeom.Torus:
		return Torus(v)
	case surfacegeom.NURBSSurface:
		return NURBSSurface(v)
	case surfacegeom.Faceted:
		return Faceted(v)
	default:
		return borrowedSurface{inner: s}
	}
}

// borrowedSurface keeps a future private variant inspectable through Kind.
type borrowedSurface struct{ inner surfacegeom.Surface }

func (s borrowedSurface) Kind() SurfaceKind { return SurfaceKind(s.inner.Kind()) }
func (borrowedSurface) surface()            {}

// Curve is the sealed edge-geometry set. A switch on it needs a default
// because later versions may add variants.
type Curve interface{ curve() }

// Line3 is a straight edge between its vertices.
type Line3 struct{}

// Circle3 is a circular edge. Center is in millimetres, Axis is a unit
// normal to its plane, and Radius is the circle radius.
type Circle3 struct {
	Center r3.Vec
	Axis   r3.Vec
	Radius units.Value
}

// Arc3 is swept counter-clockwise about Axis from the start vertex to the end.
type Arc3 struct {
	Center r3.Vec
	Axis   r3.Vec
	Radius units.Value
}

// Ellipse3 is an elliptical arc edge. Major is the unit direction of its
// semi-major axis in the plane normal to Axis.
type Ellipse3 struct {
	Center, Axis, Major  r3.Vec
	SemiMajor, SemiMinor units.Value
}

// FilletMiter3 is the intersection edge of two analytic fillet patches.
type FilletMiter3 struct{}

// NURBSCurve marks a free-form edge. Its control points remain private.
type NURBSCurve struct{}

// FacetedCurve is a polygonal edge. Bound encloses the displacement from its
// held chord chain to the curve it represents.
type FacetedCurve struct{ Bound units.Value }

func (Line3) curve()        {}
func (Circle3) curve()      {}
func (Arc3) curve()         {}
func (Ellipse3) curve()     {}
func (FilletMiter3) curve() {}
func (NURBSCurve) curve()   {}
func (FacetedCurve) curve() {}

func publicCurve(c surfacegeom.Curve) Curve {
	if c == nil {
		return nil
	}
	switch v := c.(type) {
	case surfacegeom.Line3:
		return Line3(v)
	case surfacegeom.Circle3:
		return Circle3(v)
	case surfacegeom.Arc3:
		return Arc3(v)
	case surfacegeom.Ellipse3:
		return Ellipse3(v)
	case surfacegeom.FilletMiter3:
		return FilletMiter3(v)
	case surfacegeom.NURBSCurve:
		return NURBSCurve(v)
	case surfacegeom.FacetedCurve:
		return FacetedCurve(v)
	default:
		return borrowedCurve{inner: c}
	}
}

type borrowedCurve struct{ inner surfacegeom.Curve }

func (borrowedCurve) curve() {}

// Vertex is a topological point.
type Vertex struct {
	_ noCopy
	// n is the node data internal packages read and build (internal/topo).
	n topo.Vertex
}

// Position returns the vertex position in millimetres — a computed
// coordinate, so it is bounded (core §6). A vertex read from a recorded
// coordinate is Exact with a zero bound. A vertex at a COMPUTED coordinate
// carries that computation's own proven displacement wherever the payload
// tracks one: a boolean-built vertex carries the tessellation's chord bound;
// a cap-loop chamfer's cap-level feet, which a float offset solve places
// (docs/modify-reach-design.md §8.4), carry that solve's own displacement; and
// a PLACED loft's vertex, re-lifted from the record under a rigid motion,
// carries that motion's own rounding (docs/loft-design.md §5) — a recorded
// coordinate the identity transform leaves alone is the zero-bound case of
// that same rule, not an exception to it. An analytic prism's, revolve's,
// cap-loop chamfer's, brep's or patch's own rim/junction/cap vertex carries
// the rounding its own lift through the payload's frame and accumulated
// placement committed, measured exactly for that vertex
// (proofbound.ExactFrameLiftRound, revolvemesh.RevolveLift.SweptPointGap;
// docs/evaluator-design.md §8). A rim vertex where two recorded walks meet
// also carries the bound that reaches the points BOTH walks denote there —
// each walk's own end bound plus the gap between their held ends
// (boundarywalk.JunctionVertex; docs/evaluator-design.md §3) — so a cut
// junction, a trimmed line end and a circle seam whose centre plus radius
// rounds are never Exact. A revolve's vertex is measured against its
// recorded plane point rotated about the recorded axis by the angle the
// record states, so the same bound also covers the float axis coordinates it
// was placed from, a radius snapped onto the axis, and the axis's own anchor
// and direction error (docs/evaluator-design.md §6). The frame origin is
// part of that lift: a sketch plane whose axes are the world's own still
// rounds origin.X + u whenever the sum is not representable, and a body built
// far away and placed back keeps the far lift's rounding. A lift that is exact
// for the coordinates at hand — an integer origin and integer coordinates
// under the identity placement, for one — charges nothing.
// A swept vertex is read from two independent coordinates and carries what each
// was read from: its plane-local pair from the section, and its sweep level from
// the extent. A level a ToFace or ThroughAll stop resolved in float, a magnitude
// rescaled from a non-base unit, and a chamfered end's setback are all COMPUTED
// levels, and each carries its own proven displacement into the vertex
// (extrude.go's z0Delta/z1Delta). Being feature-built is not by itself a claim
// of exactness: what the coordinate was READ FROM is.
func (v *Vertex) Position() VecMeasurement {
	exactness := Exact
	if v.n.Bound.Mag() != 0 {
		exactness = Approximate
	}
	return VecMeasurement{Value: v.n.Position, Exactness: exactness, Bound: v.n.Bound}
}

// CoEdge is one directed use of an Edge by a Loop. Its Start and End follow
// the loop walk, which may oppose the underlying Edge's global Start-to-End
// orientation.
type CoEdge struct {
	// n is the node data internal packages read and build (internal/topo).
	n topo.CoEdge
}

// Edge returns the shared topological edge this loop use traverses.
func (ce CoEdge) Edge() *Edge { return edgeHandle(ce.n.Edge) }

// Start returns the vertex where this loop use starts. It returns nil for the
// zero CoEdge.
func (ce CoEdge) Start() *Vertex { return vertexHandle(coedgeStart(ce.n)) }

// End returns the vertex where this loop use ends. It returns nil for the zero
// CoEdge.
func (ce CoEdge) End() *Vertex { return vertexHandle(coedgeEnd(ce.n)) }

// IsForward reports whether this loop use walks from Edge.Start to Edge.End.
// False means it walks from Edge.End to Edge.Start.
func (ce CoEdge) IsForward() bool { return ce.n.Forward }

// Edge is a topological edge: a tagged curve between two vertices, with the
// faces adjacent to it.
type Edge struct {
	_ noCopy
	// n is the node data internal packages read and build (internal/topo).
	n topo.Edge
}

// Curve returns the edge's tagged geometry.
func (e *Edge) Curve() Curve { return publicCurve(e.n.Curve) }

// Start returns the edge's start vertex.
func (e *Edge) Start() *Vertex { return vertexHandle(e.n.Start) }

// End returns the edge's end vertex.
func (e *Edge) End() *Vertex { return vertexHandle(e.n.End) }

// Faces returns the faces adjacent to this edge. What len(Faces()) means is
// read per body kind (docs/surface-design.md §2.2, core §6.1): on a
// BodySolid, one adjacent face is non-manifold; on a BodySheet, one adjacent
// face is a FREE edge — the boundary of the sheet, and the expected shape —
// which IsFree reports and the Free() predicate selects. Two is an interior
// edge on either kind; three or more is non-manifold on either.
func (e *Edge) Faces() []*Face { return faceHandles(e.n.Faces) }

// IsConvex reports the edge's convexity under decad's WALKED-BOUNDARY
// convention — a decided answer, not an approximation of one (core §6). It is
// the sense of the 2D profile walk that produced the edge, NOT the 3D material
// angle across it; the two agree where two walls meet and part company at a
// rim.
//
// Every profile is walked with the material on the left: the outer loop
// counter-clockwise, every hole clockwise (moments.go). Three classes of edge
// read that walk:
//
//   - A JUNCTION edge, where two walls meet, is convex when the walk turns
//     left there — the material wedge closes to less than a half turn — and
//     concave when it turns right. This one is also the material angle.
//   - A RIM edge, a wall's own copy in a cap plane, takes the sense of the
//     wall it runs along: a CIRCULAR wall by its own turn — walked
//     counter-clockwise convex, clockwise concave — and a STRAIGHT wall, which
//     turns not at all, by the role of the loop it belongs to: outer convex,
//     hole concave.
//   - The on-axis edge shared by both caps of a partial revolve is convex when
//     the sweep is less than a half turn: there the two caps ARE the adjacent
//     faces, and the sweep is the angle between them.
//
// So a HOLE's rim edges are CONCAVE, and so are the rim edges of a concave
// round bitten out of the outer boundary — even though the material across
// such a rim is a plain quarter-turn wedge a chamfer could take. Convexity
// here is the walk's answer, not the wedge's: Concave() picks a hole's rim,
// Convex() never does.
func (e *Edge) IsConvex() bool { return e.n.Convex }

// IsFree reports whether exactly one face is adjacent to this edge — a
// sheet's boundary, and the expected shape there (docs/surface-design.md
// §2.2). On a BodySolid the same count of one is non-manifold rather than
// free; IsFree only names the count Faces reports, and Faces's own doc comment
// states the per-kind reading in full.
func (e *Edge) IsFree() bool { return len(e.n.Faces) == 1 }

// Length returns the edge's length: Exact only when the analytic result is
// proved exactly representable, otherwise Approximate with its evaluation
// bound. A boolean-built straight rim (two planar sources) carries its proven
// chord bound. A boolean rim on a curved source has a true length the chord
// chain provably understates by an amount this evaluator cannot bound, so it
// is ErrUnsupported — an honest refusal, never an understated Bound. A
// cap-loop chamfer's miter ruling is the same refusal one construction over:
// where either carrier meeting at the corner is circular, the ruling's tagged
// chord understates the curved locus the exact offset family denotes
// (docs/modify-reach-design.md §8.3), and a corner whose locus enclosure this
// evaluator cannot build refuses the same way rather than publish an
// understated Bound.
func (e *Edge) Length() (Measurement, error) {
	if e.n.LengthUnbounded {
		return Measurement{}, fmt.Errorf(`%w: this edge's true length has no proven bound this evaluator can publish`, ErrUnsupported)
	}
	return Measurement{
		Value:     units.Millimeters(e.n.Length),
		Exactness: exactnessOf(e.n.LengthBound),
		Bound:     units.Millimeters(e.n.LengthBound),
	}, nil
}

// Loop is one boundary loop of a face.
type Loop struct {
	_ noCopy
	// n is the node data internal packages read and build (internal/topo).
	n topo.Loop
}

// IsOuter reports whether this is the face's outer boundary; false for a
// hole.
func (l *Loop) IsOuter() bool { return l.n.Outer }

// CoEdges returns the loop's directed edge uses in walk order. The returned
// slice is a copy; each CoEdge is an immutable view of the loop's stored use.
func (l *Loop) CoEdges() []CoEdge { return coedgeHandles(l.n.CoEdges) }

// Edges returns the loop's edges in walk order. It is the compatibility view
// of CoEdges: the same edge identities in the same order, without direction.
func (l *Loop) Edges() []*Edge { return loopEdges(&l.n) }

// Face is a topological face: a tagged surface bounded by loops.
type Face struct {
	_ noCopy
	// n is the node data internal packages read and build (internal/topo).
	n topo.Face
}

// denotedUnder carries a face's denoted surface onto a copy placed by xform:
// the exact image of the source face's own, never a re-reading of the copy's
// rounded tag.
func (f *Face) denotedUnder(xform r3.Transform) *surfacenormal.Revolved {
	if f.n.Denoted == nil || xform == r3.Identity() {
		return f.n.Denoted
	}
	out := f.n.Denoted.Transformed(xform)
	return &out
}

// NormalAt returns the face's outward normal at p. Its bound combines the
// arithmetic proof for the normal of the face's tagged surface
// (normal_bound.go) and any proven departure of the surface actually carried
// from that tag (normalBound). A revolve wall's or cap's bound is instead
// proven against the surface its record denotes (topo.Face.Denoted), which covers
// its tag's departure and the arm's arithmetic at once. It is Exact only when the
// whole bound is zero. A point that gives the surface no direction is
// ErrDegenerate. A reading whose own enclosure cannot separate the direction
// from zero is ErrUnsupported.
//
// The five analytic variants — Plane, Cylinder, Cone, Sphere and Torus — are
// the whole set this answers for. Any other tagged surface is ErrUnsupported:
// a NURBSSurface for the reason docs/spline-design.md §7 owns, and a Faceted
// face — the tag every boolean-produced face carries — because its answer
// waits on the faceted certificate stage
// (docs/payload-verification-design.md §5.4, §13).
func (f *Face) NormalAt(p r3.Vec) (VecMeasurement, error) {
	switch s := f.n.Surface.(type) {
	case surfacegeom.Plane:
		n := s.Frame.N()
		return f.normalMeasurement(p, n, func() (float64, surfacenormal.Status) {
			return surfacenormal.PlaneAllow(s.Frame, n)
		}, "the frame of this plane names no direction")
	case surfacegeom.Cylinder:
		rel := p.Sub(s.Origin)
		radial := rel.Sub(s.Axis.Scale(rel.Dot(s.Axis)))
		dir, ok := radial.Normalize()
		if !ok {
			return VecMeasurement{}, fmt.Errorf(`%w: a point on the cylinder axis has no normal`, ErrDegenerate)
		}
		return f.normalMeasurement(p, dir, func() (float64, surfacenormal.Status) {
			return surfacenormal.AxialAllow(p, s.Origin, s.Axis, dir)
		}, "a point on the cylinder axis has no normal")
	case surfacegeom.Cone:
		rel := p.Sub(s.Origin)
		radial := rel.Sub(s.Axis.Scale(rel.Dot(s.Axis)))
		dir, ok := radial.Normalize()
		if !ok {
			return VecMeasurement{}, fmt.Errorf(`%w: the cone apex has no normal`, ErrDegenerate)
		}
		half, err := s.HalfAngle.In(units.Radian)
		if err != nil {
			return VecMeasurement{}, fmt.Errorf(`decad: a cone's half angle is not an angle: %w`, err)
		}
		// The wall leans outward by the half angle along the growth axis, so
		// the geometric normal tilts against it by the same angle.
		n := dir.Scale(math.Cos(half)).Sub(s.Axis.Scale(math.Sin(half)))
		return f.normalMeasurement(p, n, func() (float64, surfacenormal.Status) {
			return surfacenormal.ConeAllow(p, s.Origin, s.Axis, half, n)
		}, "the cone apex has no normal")
	case surfacegeom.Sphere:
		dir, ok := p.Sub(s.Center).Normalize()
		if !ok {
			return VecMeasurement{}, fmt.Errorf(`%w: the sphere center has no normal`, ErrDegenerate)
		}
		return f.normalMeasurement(p, dir, func() (float64, surfacenormal.Status) {
			return surfacenormal.RadialAllow(p, s.Center, dir)
		}, "the sphere center has no normal")
	case surfacegeom.Torus:
		major, err := s.Major.In(units.Millimeter)
		if err != nil {
			return VecMeasurement{}, fmt.Errorf(`decad: a torus's major radius is not a length: %w`, err)
		}
		rel := p.Sub(s.Center)
		radial := rel.Sub(s.Axis.Scale(rel.Dot(s.Axis)))
		rdir, ok := radial.Normalize()
		if !ok {
			return VecMeasurement{}, fmt.Errorf(`%w: a point on the torus axis has no normal`, ErrDegenerate)
		}
		tube := s.Center.Add(rdir.Scale(major))
		dir, ok := p.Sub(tube).Normalize()
		if !ok {
			return VecMeasurement{}, fmt.Errorf(`%w: the tube center has no normal`, ErrDegenerate)
		}
		return f.normalMeasurement(p, dir, func() (float64, surfacenormal.Status) {
			return surfacenormal.TorusAllow(p, s.Center, s.Axis, major, dir)
		}, "the tube center has no normal")
	default:
		return VecMeasurement{}, fmt.Errorf(`%w: this evaluator computes normals for its own analytic faces only`, ErrUnsupported)
	}
}

// normalMeasurement publishes one arm's computed geometric direction under
// the face's own outward sign. tagAllow is the arm's proof against its tagged
// surface, and the face's own departure from its tag (normalBound) composes
// with it by triangle inequality. A face carrying its denoted surface is
// proven against that instead: the comparison is with the surface the face
// carries, so it covers the departure normalBound states, and neither
// tagAllow nor normalBound enters. The sign is exact, so it never changes the
// bound.
func (f *Face) normalMeasurement(p, dir r3.Vec, tagAllow func() (float64, surfacenormal.Status), degenerate string) (VecMeasurement, error) {
	if f.n.Reversed {
		dir = dir.Scale(-1)
	}
	var allow float64
	var st surfacenormal.Status
	departure := 0.0
	if f.n.Denoted != nil {
		allow, st = f.n.Denoted.Allow(p, dir, f.n.Reversed)
	} else {
		allow, st = tagAllow()
		departure = f.n.NormalBound
	}
	switch st {
	case surfacenormal.Zero:
		return VecMeasurement{}, fmt.Errorf(`%w: %s`, ErrDegenerate, degenerate)
	case surfacenormal.Unproven:
		return VecMeasurement{}, fmt.Errorf(`%w: this normal's own direction is not proven away from zero, so no bound covers it`, ErrUnsupported)
	}
	if allow != 0 || departure != 0 {
		allow = math.Nextafter(allow+departure, math.Inf(1))
	}
	return VecMeasurement{
		Value:     dir,
		Exactness: exactnessOf(allow),
		Bound:     units.Scalar(allow),
	}, nil
}

// Surface returns the face's tagged geometry.
func (f *Face) Surface() Surface { return publicSurface(f.n.Surface) }

// Loops returns the face's boundary loops, the outer loop first.
func (f *Face) Loops() []*Loop { return loopHandles(f.n.Loops) }

// Edges returns the face's edges across all loops.
func (f *Face) Edges() []*Edge {
	var out []*Edge
	for _, l := range f.n.Loops {
		out = append(out, loopEdges(l)...)
	}
	return out
}

// Area returns the face's area: Exact only when the result is proved exactly
// representable. Analytic float evaluation and boolean-built Faceted faces
// return Approximate with their proven bounds.
func (f *Face) Area() (Measurement, error) {
	return Measurement{
		Value:     units.SquareMillimeters(f.n.Area),
		Exactness: exactnessOf(f.n.AreaBound),
		Bound:     units.SquareMillimeters(f.n.AreaBound),
	}, nil
}

// Centroid returns the area centroid of a face with a proved certificate.
// Point-loft far caps and their planar Split caps provide one; other faces
// currently return ErrUnsupported.
func (f *Face) Centroid() (VecMeasurement, error) {
	if !f.n.HasCentroid {
		return VecMeasurement{}, fmt.Errorf("%w: this face has no bounded area centroid", ErrUnsupported)
	}
	return vecMeasurementFromInternal(f.n.Centroid), nil
}

// isPlanar reports whether the face this evaluator holds is FLAT: an analytic
// Plane is by construction, and a Faceted face is when every source it descends
// from was. It is not the same question as the surface tag.
func (f *Face) isPlanar() bool {
	return f.n.Surface.Kind() == surfacegeom.KindPlane || f.n.HeldPlanar
}

// Origins returns every feature role that created this face —
// canonicalization may merge coplanar faces, and a merged face carries ALL
// contributing roles (core §6.1).
func (f *Face) Origins() []FeatureRef { return featureRefsOf(f.n.Origins) }

// FaceShell is a connected set of faces.
type FaceShell struct {
	faces []*Face
	// n is the node data internal packages read and build (internal/topo).
	n topo.FaceShell
}

// IsVoid reports whether the shell bounds an internal cavity. IsVoid and
// IsOpen are independent questions and neither implies the other
// (docs/surface-design.md §2.2): a void solid shell and an open sheet shell
// are both ordinary states, and nothing rules out either combination.
func (s *FaceShell) IsVoid() bool { return s.n.Void }

// IsOpen reports whether the shell has at least one free edge
// (docs/surface-design.md §2.2). IsOpen and IsVoid are independent questions
// and neither implies the other. A solid's shell is never open; a
// surface-result feature's sheet shell reports true wherever its build
// omitted a closing face and left a rim with no second face to pair it with
// (extrude.go, revolve_build.go), and false for a closed sheet that mints no
// such rim — a full revolution (docs/surface-design.md Table W).
func (s *FaceShell) IsOpen() bool { return s.n.Open }

// Faces returns the shell's faces.
func (s *FaceShell) Faces() []*Face { return append([]*Face(nil), s.faces...) }

// Lump is a connected solid piece of a body.
type Lump struct {
	shells []*FaceShell
}

// Shells returns the lump's shells, the outer shell first.
func (l *Lump) Shells() []*FaceShell { return append([]*FaceShell(nil), l.shells...) }

// Body is an immutable solid: every operation returns a new one, and the
// input body is retired from its document (core §6).
type Body struct {
	doc   *Document
	lumps []*Lump

	// n is the node data internal packages read and build (internal/topo).
	n topo.Body

	// payload is the evaluator's own record of how this body was built —
	// what Transform re-evaluates under a composed motion
	// (docs/evaluator-design.md §8). Nil for a body this evaluator did not
	// build.
	payload featurePayload

	tessellationCache atomic.Pointer[tessellationCacheEntry]
	// pairReports keeps the recent ContactPair reports this body is the
	// first operand of (internal/reportvocab/contact_memo.go); it changes no outcome either.
	pairReports pairReportMemo
	// sweepRadii keeps the recent sweep radii read on this body
	// (contact_sweep_memo.go); it changes no outcome either.
	sweepRadii sweepRadiusMemo
}

// Document returns the document that owns (or owned) this body.
func (b *Body) Document() *Document { return b.doc }

// Origin returns the feature that created this body.
func (b *Body) Origin() FeatureRef { return featureRefOf(b.n.Origin) }

// Lumps returns the body's disjoint pieces; more than one means the body is
// disconnected.
func (b *Body) Lumps() []*Lump { return append([]*Lump(nil), b.lumps...) }

// Shells returns the body's shells across all lumps.
func (b *Body) Shells() []*FaceShell {
	var out []*FaceShell
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
	seen := map[*topo.Edge]struct{}{}
	var out []*Edge
	for _, f := range b.Faces() {
		for _, l := range f.n.Loops {
			for _, ce := range l.CoEdges {
				if _, ok := seen[ce.Edge]; ok {
					continue
				}
				seen[ce.Edge] = struct{}{}
				out = append(out, edgeHandle(ce.Edge))
			}
		}
	}
	return out
}

// Vertices returns the body's vertices, deduplicated.
func (b *Body) Vertices() []*Vertex {
	seen := map[*topo.Vertex]struct{}{}
	var out []*Vertex
	for _, e := range b.Edges() {
		for _, v := range [2]*topo.Vertex{e.n.Start, e.n.End} {
			if v == nil {
				continue
			}
			if _, ok := seen[v]; ok {
				continue
			}
			seen[v] = struct{}{}
			out = append(out, vertexHandle(v))
		}
	}
	return out
}

// Kind reports what the body IS: BodySolid or BodySheet
// (docs/surface-design.md §2.1). Kind says what the body is; IsSolid says
// whether it is sound. A BodySheet with IsSolid() == true is never produced —
// a sheet encloses no material region, so no evaluator may prove one a solid.
func (b *Body) Kind() BodyKind { return BodyKind(b.n.Kind) }

// IsSolid reports whether the body is a valid solid — a decided answer
// (core §6).
func (b *Body) IsSolid() bool { return b.n.Solid }

// Bounds returns the body's axis-aligned bounding box, as tight as the
// evaluator can prove, reporting its own exactness (core §6).
func (b *Body) Bounds() (Box, error) { return boxFromInternal(b.n.Bounds), nil }

// Volume returns the enclosed volume. A body that is not a solid encloses
// none: ErrNotSolid, never zero (core §4/§6).
func (b *Body) Volume() (Measurement, error) {
	if !b.n.Solid {
		return Measurement{}, ErrNotSolid
	}
	return measurementFromInternal(b.n.Volume), nil
}

// Area returns the total surface area of the body's boundary.
func (b *Body) Area() (Measurement, error) { return measurementFromInternal(b.n.Area), nil }

// Centroid returns the centroid of the enclosed region. A body that is not a
// solid has none: ErrNotSolid.
func (b *Body) Centroid() (VecMeasurement, error) {
	if !b.n.Solid {
		return VecMeasurement{}, ErrNotSolid
	}
	return vecMeasurementFromInternal(b.n.Centroid), nil
}

// originRefs returns a copy of the face's origin roles in their internal form,
// nil when it has none, exactly as Origins copies the public form.
func (f *Face) originRefs() []provenance.Ref { return append([]provenance.Ref(nil), f.n.Origins...) }

// featureRefOf is the public form of an internal provenance reference.
func featureRefOf(r provenance.Ref) FeatureRef { return FeatureRef{producer: r.Producer, Role: r.Role} }

// refOf is the internal form of a public FeatureRef.
func refOf(f FeatureRef) provenance.Ref { return provenance.Ref{Producer: f.producer, Role: f.Role} }

// featureRefsOf copies rs into their public form. It returns nil for an empty
// input, as append([]FeatureRef(nil), ...) does.
func featureRefsOf(rs []provenance.Ref) []FeatureRef {
	if len(rs) == 0 {
		return nil
	}
	out := make([]FeatureRef, len(rs))
	for i, r := range rs {
		out[i] = featureRefOf(r)
	}
	return out
}
