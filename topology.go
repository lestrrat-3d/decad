package decad

import (
	"fmt"
	"math"
	"sync"
	"sync/atomic"

	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the topology model of docs/evaluator-design.md §3: concrete
// structs with unexported fields; the accessors of core §6/§6.1 are the whole
// public surface. Identity is the pointer, scoped to one body — never an
// index. A Body is immutable after construction, so it is safe to read from
// many goroutines (core §12).

// The stable cap-role names the analytic evaluators mint (evaluator §3): the
// two ends of a prism sweep, and a revolve's caps on a partial turn.
const (
	roleCapStart = "capStart"
	roleCapEnd   = "capEnd"
	// rolePatch is a Patch face's own Origin() role (docs/surface-design.md
	// §5.1). It stays internal this increment: a one-face body is already
	// selected by Faces(Planar()), and no public FeatureRef helper names it
	// yet — Stitch is what decides the right shape for one.
	rolePatch = "patch"
	// roleBody is the body's own Origin() role, minted by every evaluator
	// build.
	roleBody = "body"
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

// Surface is the sealed face-geometry set.
type Surface = surfacegeom.Surface

// SurfaceKind identifies a surface variant.
type SurfaceKind = surfacegeom.SurfaceKind

const (
	KindPlane    = surfacegeom.KindPlane
	KindCylinder = surfacegeom.KindCylinder
	KindCone     = surfacegeom.KindCone
	KindSphere   = surfacegeom.KindSphere
	KindTorus    = surfacegeom.KindTorus
	KindNURBS    = surfacegeom.KindNURBS
	KindFaceted  = surfacegeom.KindFaceted
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

// Plane is a planar face's geometry.
type Plane = surfacegeom.Plane

// Cylinder is a right circular cylindrical face's geometry.
type Cylinder = surfacegeom.Cylinder

// Cone is a right circular conical face's geometry.
type Cone = surfacegeom.Cone

// Sphere is a spherical face's geometry.
type Sphere = surfacegeom.Sphere

// Torus is a toroidal face's geometry.
type Torus = surfacegeom.Torus

// NURBSSurface is a free-form face's geometry.
type NURBSSurface = surfacegeom.NURBSSurface

// Faceted is a polygonal face's geometry.
type Faceted = surfacegeom.Faceted

// Curve is the sealed edge-geometry set.
type Curve = surfacegeom.Curve

// Line3 is a straight edge between its vertices.
type Line3 = surfacegeom.Line3

// Circle3 is a circular edge's geometry.
type Circle3 = surfacegeom.Circle3

// Arc3 is a circular arc edge's geometry.
type Arc3 = surfacegeom.Arc3

// Ellipse3 is an elliptical arc edge's geometry.
type Ellipse3 = surfacegeom.Ellipse3

// NURBSCurve is a free-form edge's geometry.
type NURBSCurve = surfacegeom.NURBSCurve

// FacetedCurve is a polygonal edge's geometry.
type FacetedCurve = surfacegeom.FacetedCurve

// Vertex is a topological point.
type Vertex struct {
	position r3.Vec
	bound    units.Value
	// level is the LEVEL half of the shared-denotation certificate
	// (denotation.go): non-zero only for a vertex a builder stamped at one
	// denoted sweep level, and zero ("no certificate") for every vertex no
	// builder in this package mints one for.
	level levelToken
	// denot is the CURVE half of the shared-denotation certificate
	// (denotation.go): non-zero only for a vertex a builder minted one for —
	// today a straight prism's rim corner (prism_build.go) and a revolve's
	// own internal junction vertex (revolve_build.go; a partial revolve's
	// own SEAM — its boundary copy at phi0/phi1 — mints none) — and
	// propagated unchanged,
	// composing xform, by every copier that reproduces the same point
	// (stitch.go's rebuildStitchTopology, unstitch.go's
	// copyFaceUnderContext, patch_body.go's copyPatchFacesUnder). Zero ("no
	// certificate") for every vertex no builder or copier stamps.
	denot curveToken
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
	if v.bound.Mag() != 0 {
		exactness = Approximate
	}
	return VecMeasurement{Value: v.position, Exactness: exactness, Bound: v.bound}
}

// CoEdge is one directed use of an Edge by a Loop. Its Start and End follow
// the loop walk, which may oppose the underlying Edge's global Start-to-End
// orientation.
type CoEdge struct {
	edge    *Edge
	forward bool
}

// coedge keeps the evaluator's internal construction spelling while CoEdge is
// the public immutable view.
type coedge = CoEdge

// Edge returns the shared topological edge this loop use traverses.
func (ce CoEdge) Edge() *Edge { return ce.edge }

// Start returns the vertex where this loop use starts. It returns nil for the
// zero CoEdge.
func (ce CoEdge) Start() *Vertex {
	if ce.edge == nil {
		return nil
	}
	if ce.forward {
		return ce.edge.start
	}
	return ce.edge.end
}

// End returns the vertex where this loop use ends. It returns nil for the zero
// CoEdge.
func (ce CoEdge) End() *Vertex {
	if ce.edge == nil {
		return nil
	}
	if ce.forward {
		return ce.edge.end
	}
	return ce.edge.start
}

// IsForward reports whether this loop use walks from Edge.Start to Edge.End.
// False means it walks from Edge.End to Edge.Start.
func (ce CoEdge) IsForward() bool { return ce.forward }

// Edge is a topological edge: a tagged curve between two vertices, with the
// faces adjacent to it.
type Edge struct {
	curve  Curve
	start  *Vertex
	end    *Vertex
	faces  []*Face
	convex bool    // the walked-boundary convexity — see IsConvex
	length float64 // held millimetres for the analytic curve set
	// lengthBound is the proven error bound on length: zero only when the
	// analytic curve's held result is exact, otherwise its evaluation bound;
	// boolean-built chains carry their proven chord bound.
	lengthBound float64
	// lengthUnbounded marks an edge whose true length this evaluator cannot
	// bound, so Length refuses rather than understate: a boolean rim on a
	// curved source, whose chord chain understates the true curve's length
	// by an amount no chord bound can cover, or a cap-blend miter ruling
	// adjacent to a circular wall whose corner-foot locus this evaluator
	// cannot enclose (docs/modify-reach-design.md §8.3).
	lengthUnbounded bool
	// curveBound, valid only where curveBounded is set, bounds how far the
	// curve a Circle3 or Arc3 edge denotes lies from its held circle — the
	// circle of the held Radius about the held Center, normal to the held
	// Axis: every denoted point is within curveBound of that circle, and
	// curveBound is below half the held radius, so projecting radially about
	// the held centre carries the denoted curve continuously onto the held
	// circle. It says nothing about where along the circle the curve ends;
	// the end vertices' own bounds say that. A builder that proves it sets
	// both (prismPayload.circleCurveBound); every copier carries it through
	// its placement (surfacegeom.PlacedCurveBound). An edge no builder bounds leaves
	// curveBounded false, and a reading that needs the bound refuses.
	curveBound   float64
	curveBounded bool
	// level is the LEVEL half of the shared-denotation certificate
	// (denotation.go): non-zero only for a rim edge a builder stamped at one
	// denoted sweep level, and zero ("no certificate") for every edge no
	// builder in this package mints one for.
	level levelToken
	// denot is the CURVE half of the shared-denotation certificate
	// (denotation.go): non-zero only for an edge a builder minted one for —
	// today a straight prism's rim edge (prism_build.go) and a revolve's
	// junction edge (revolve_build.go) — and propagated unchanged, composing
	// xform, by every copier that reproduces the same curve (stitch.go's
	// rebuildStitchTopology, unstitch.go's copyFaceUnderContext,
	// patch_body.go's copyPatchFacesUnder). It is what stitch_weld.go's
	// buildStitchWeldPlan reads to lift Table J's J5 for a bounded pair. Zero
	// ("no certificate") for every edge no builder or copier stamps.
	denot curveToken
}

// Curve returns the edge's tagged geometry.
func (e *Edge) Curve() Curve { return e.curve }

// Start returns the edge's start vertex.
func (e *Edge) Start() *Vertex { return e.start }

// End returns the edge's end vertex.
func (e *Edge) End() *Vertex { return e.end }

// Faces returns the faces adjacent to this edge. What len(Faces()) means is
// read per body kind (docs/surface-design.md §2.2, core §6.1): on a
// BodySolid, one adjacent face is non-manifold; on a BodySheet, one adjacent
// face is a FREE edge — the boundary of the sheet, and the expected shape —
// which IsFree reports and the Free() predicate selects. Two is an interior
// edge on either kind; three or more is non-manifold on either.
func (e *Edge) Faces() []*Face { return append([]*Face(nil), e.faces...) }

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
func (e *Edge) IsConvex() bool { return e.convex }

// IsFree reports whether exactly one face is adjacent to this edge — a
// sheet's boundary, and the expected shape there (docs/surface-design.md
// §2.2). On a BodySolid the same count of one is non-manifold rather than
// free; IsFree only names the count Faces reports, and Faces's own doc comment
// states the per-kind reading in full.
func (e *Edge) IsFree() bool { return len(e.faces) == 1 }

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
	if e.lengthUnbounded {
		return Measurement{}, fmt.Errorf(`%w: this edge's true length has no proven bound this evaluator can publish`, ErrUnsupported)
	}
	return Measurement{
		Value:     units.Millimeters(e.length),
		Exactness: exactnessOf(e.lengthBound),
		Bound:     units.Millimeters(e.lengthBound),
	}, nil
}

// Loop is one boundary loop of a face.
type Loop struct {
	coedges []coedge
	outer   bool
}

// IsOuter reports whether this is the face's outer boundary; false for a
// hole.
func (l *Loop) IsOuter() bool { return l.outer }

// CoEdges returns the loop's directed edge uses in walk order. The returned
// slice is a copy; each CoEdge is an immutable view of the loop's stored use.
func (l *Loop) CoEdges() []CoEdge {
	return append([]CoEdge(nil), l.coedges...)
}

// Edges returns the loop's edges in walk order. It is the compatibility view
// of CoEdges: the same edge identities in the same order, without direction.
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
	area    float64 // millimetres²
	// areaBound is the proven error bound on area: analytic float evaluation
	// or the composed chord bound for a boolean-built Faceted face.
	areaBound float64
	// axialDelta is the proven displacement of this planar face along its own
	// normal. A body-relative ToFace stop carries this selected face's bound
	// into the level it computes; hasAxialDelta distinguishes an exact zero
	// from a face whose payload does not state an axial bound.
	axialDelta    float64
	hasAxialDelta bool
	// heldPlanar records that a Faceted face descends only from PLANAR source
	// faces, so the face it stands for in the true result is flat — and a rim
	// between two such faces is a straight line, whose chord length is honest.
	// The surface TAG cannot say this: a boolean's faces all read Faceted,
	// including the ones that are plainly flat.
	heldPlanar bool
	// reversed is true when the OUTWARD (material-leaving) normal is the
	// surface's geometric normal negated — a hole's cylinder wall, whose
	// material lies outside the cylinder.
	reversed bool
	// normalBound is the proven DIMENSIONLESS bound on NormalAt's own answer:
	// how far the surface this face really carries can tilt away from the
	// tagged variant the normal is computed from. It is zero for every face
	// whose own geometry IS its tag, and for a revolve wall, whose departure
	// rides in denoted instead. It is nonzero for a revolve's cap (its
	// angular displacement, which NormalAt reads only where the cap carries
	// no denoted plane) and for a cap-loop chamfer's band patch: the
	// patch is RULED between two built directrices, and the `Cone` or `Plane`
	// it publishes is that ruled surface only to within a bound measured from
	// the numbers the body publishes for it
	// (capblend_departure.go, docs/modify-reach-design.md §8.3). The bound is a
	// world-space one, so it covers both a mitered corner's own angular skew and
	// the placement's independent rounding of every coordinate the build emits —
	// which leaves the tag even on a flat patch, and on a circular one whose two
	// windows coincide exactly. A zero term there would omit a direction
	// difference the built surface has. A band patch's bound also carries the
	// turn between that ruled surface and the wall its records denote, since
	// the cap contour it rules to is a solved offset held within a proven
	// displacement (capband.DenotedNormalAllow). NormalAt separately composes
	// its arithmetic proof (normal_bound.go).
	normalBound float64
	// denoted is the surface a revolve wall or cap DENOTES — a wall's
	// recorded meridian segment swept about its recorded axis, or a cap's
	// plane through that axis at its end's recorded angle, enclosed exactly
	// (surfacenormal.Revolved) — where the tag is only a float re-expression
	// of it. NormalAt judges such a face's normal against it rather than
	// against the tag, so the bound covers the tag's whole departure; nil
	// for every face whose tag is its own denotation. It is not
	// normalBound: no dimensionless term states the departure for every p.
	// Stitch's flux arms integrate the tag, so they admit a revolve wall only
	// where its tag is exactly this surface (stitch_flux.go's
	// stitchFluxTagsDenoted).
	denoted *surfacenormal.Revolved
}

// denotedUnder carries a face's denoted surface onto a copy placed by xform:
// the exact image of the source face's own, never a re-reading of the copy's
// rounded tag.
func (f *Face) denotedUnder(xform r3.Transform) *surfacenormal.Revolved {
	if f.denoted == nil || xform == r3.Identity() {
		return f.denoted
	}
	out := f.denoted.Transformed(xform)
	return &out
}

// NormalAt returns the face's outward normal at p. Its bound combines the
// arithmetic proof for the normal of the face's tagged surface
// (normal_bound.go) and any proven departure of the surface actually carried
// from that tag (normalBound). A revolve wall's or cap's bound is instead
// proven against the surface its record denotes (Face.denoted), which covers
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
	switch s := f.surface.(type) {
	case Plane:
		n := s.Frame.N()
		return f.normalMeasurement(p, n, func() (float64, surfacenormal.Status) {
			return surfacenormal.PlaneAllow(s.Frame, n)
		}, "the frame of this plane names no direction")
	case Cylinder:
		rel := p.Sub(s.Origin)
		radial := rel.Sub(s.Axis.Scale(rel.Dot(s.Axis)))
		dir, ok := radial.Normalize()
		if !ok {
			return VecMeasurement{}, fmt.Errorf(`%w: a point on the cylinder axis has no normal`, ErrDegenerate)
		}
		return f.normalMeasurement(p, dir, func() (float64, surfacenormal.Status) {
			return surfacenormal.AxialAllow(p, s.Origin, s.Axis, dir)
		}, "a point on the cylinder axis has no normal")
	case Cone:
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
	case Sphere:
		dir, ok := p.Sub(s.Center).Normalize()
		if !ok {
			return VecMeasurement{}, fmt.Errorf(`%w: the sphere center has no normal`, ErrDegenerate)
		}
		return f.normalMeasurement(p, dir, func() (float64, surfacenormal.Status) {
			return surfacenormal.RadialAllow(p, s.Center, dir)
		}, "the sphere center has no normal")
	case Torus:
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
	if f.reversed {
		dir = dir.Scale(-1)
	}
	var allow float64
	var st surfacenormal.Status
	departure := 0.0
	if f.denoted != nil {
		allow, st = f.denoted.Allow(p, dir, f.reversed)
	} else {
		allow, st = tagAllow()
		departure = f.normalBound
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

// Area returns the face's area: Exact only when the result is proved exactly
// representable. Analytic float evaluation and boolean-built Faceted faces
// return Approximate with their proven bounds.
func (f *Face) Area() (Measurement, error) {
	return Measurement{
		Value:     units.SquareMillimeters(f.area),
		Exactness: exactnessOf(f.areaBound),
		Bound:     units.SquareMillimeters(f.areaBound),
	}, nil
}

// isPlanar reports whether the face this evaluator holds is FLAT: an analytic
// Plane is by construction, and a Faceted face is when every source it descends
// from was. It is not the same question as the surface tag.
func (f *Face) isPlanar() bool {
	return f.surface.Kind() == KindPlane || f.heldPlanar
}

// Origins returns every feature role that created this face —
// canonicalization may merge coplanar faces, and a merged face carries ALL
// contributing roles (core §6.1).
func (f *Face) Origins() []FeatureRef { return append([]FeatureRef(nil), f.origins...) }

// Shell is a connected set of faces.
type Shell struct {
	faces []*Face
	void  bool
	open  bool
}

// IsVoid reports whether the shell bounds an internal cavity. IsVoid and
// IsOpen are independent questions and neither implies the other
// (docs/surface-design.md §2.2): a void solid shell and an open sheet shell
// are both ordinary states, and nothing rules out either combination.
func (s *Shell) IsVoid() bool { return s.void }

// IsOpen reports whether the shell has at least one free edge
// (docs/surface-design.md §2.2). IsOpen and IsVoid are independent questions
// and neither implies the other. A solid's shell is never open; a
// surface-result feature's sheet shell reports true wherever its build
// omitted a closing face and left a rim with no second face to pair it with
// (extrude.go, revolve_build.go), and false for a closed sheet that mints no
// such rim — a full revolution (docs/surface-design.md Table W).
func (s *Shell) IsOpen() bool { return s.open }

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
	kind     BodyKind

	// payload is the evaluator's own record of how this body was built —
	// what Placed re-evaluates under a composed motion
	// (docs/evaluator-design.md §8). Nil for a body this evaluator did not
	// build.
	payload featurePayload

	tessellationCache atomic.Pointer[tessellationCacheEntry]
	// planarConvexity caches the exact planar convexity certificate and
	// planarSnapshot the exact held snapshot it is read off, each keyed by
	// the held chord when the snapshot depends on it
	// (contact_faceted_pair.go); like the tessellation cache neither changes
	// the body's logical geometry.
	planarConvexity atomic.Pointer[planarConvexityEntry]
	planarSnapshot  atomic.Pointer[planarSnapshotEntry]
	// pairReports keeps the recent ContactPair reports this body is the
	// first operand of (contact_pair_memo.go); it changes no outcome either.
	pairReports pairReportMemo
	// sweepRadii keeps the recent sweep radii read on this body
	// (contact_sweep_memo.go); it changes no outcome either.
	sweepRadii sweepRadiusMemo
	// planarHints maps each partner body to the nearest candidate pair the
	// last exact planar relation with this body first found
	// (contact_faceted_pair.go). It only speeds the next relation's
	// distance scan, never changes its result.
	planarHints sync.Map
}

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

// Kind reports what the body IS: BodySolid or BodySheet
// (docs/surface-design.md §2.1). Kind says what the body is; IsSolid says
// whether it is sound. A BodySheet with IsSolid() == true is never produced —
// a sheet encloses no material region, so no evaluator may prove one a solid.
func (b *Body) Kind() BodyKind { return b.kind }

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
