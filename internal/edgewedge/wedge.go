// Package edgewedge reads the material wedge of an edge: whether the solid's
// material angle across the edge is reflex (a boss root, an inside corner) or
// salient (a box's outer edge, a hole's rim). It is the reading behind the
// public Edge.Wedge and the Reflex and Salient selector predicates
// (agent-docs/api-design.md §6.1, §9).
//
// It is distinct from topo.Edge.Convex, the walked-boundary convexity the
// builders stamp (agent-docs/evaluator-design.md §3). That flag reads a boss
// root on a Union convex, because the root is a cap line that reads its
// owner's loop role. This package reads the angle between the two faces'
// outward normals instead, and answers only where those normals decide it
// beyond their own proven bounds.
package edgewedge

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/selectorquery"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Wedge is the material wedge across an edge.
type Wedge int

const (
	// Undecided is an edge this package does not read, or whose reading the
	// proven bounds do not separate from zero.
	Undecided Wedge = iota
	// Salient is an edge whose material angle is under a half turn.
	Salient
	// Reflex is an edge whose material angle is over a half turn.
	Reflex
)

// arithSlack covers the float rounding of one triple product of three
// vectors of length near 1, and of the normalization of the tangent: a few
// units in the last place of each product and sum, taken with margin.
const arithSlack = 64 * 0x1p-53

// station is one point of the edge at which the wedge is read: the held
// point, the proven bound on its displacement from the true edge (mm), the
// edge's unit tangent there in the edge's own sense (a line from Start to
// End, a circle or arc counter-clockwise about its Axis), and the proven
// bound on that tangent's departure from the true one.
type station struct {
	p       r3.Vec
	delta   float64
	tangent r3.Vec
	tanErr  float64
}

// Of reads e's material wedge. It answers Salient or Reflex only when every
// reading it takes decides the same sign beyond its bound; any other outcome
// is Undecided. e must not be nil.
//
// The reading is the sign of (n1 × n2)·t at each end of the edge (one point
// for a whole circle), where n1 is the outward normal of a planar face of the
// edge, n2 that of the other face, and t the direction the planar face walks
// the edge with its material on the left seen from outside. Positive is
// salient. The planar face's walk sense comes from the signed area of the
// loop that holds the edge about the face's outward normal, and the loop's
// outer or hole role, never from a stored loop orientation, which builders
// do not agree on. Where both faces are planar, both readings must agree.
//
// Only edges whose angle cannot change along the edge are read: a line
// between two planes, a line between a plane and a cylinder whose axis is
// parallel to the line, and a circle or arc on a plane normal to its axis
// meeting a coaxial cylinder, cone, sphere or torus. Every other edge, an
// edge without exactly two distinct faces, a face whose normal NormalAt
// refuses (a Faceted or free-form face), and a loop holding a curve this
// package cannot integrate are Undecided.
func Of(e *topo.Edge) Wedge {
	if len(e.Faces) != 2 || e.Faces[0] == e.Faces[1] || e.Start == nil || e.End == nil {
		return Undecided
	}
	stations, ok := stationsOf(e)
	if !ok {
		return Undecided
	}
	spread, ok := admitted(e)
	if !ok {
		return Undecided
	}
	got := Undecided
	read := false
	for i, f := range e.Faces {
		if _, planar := f.Surface.(surfacegeom.Plane); !planar {
			continue
		}
		other := e.Faces[1-i]
		walk, ok := materialWalk(f, e)
		if !ok {
			return Undecided
		}
		for _, st := range stations {
			w := readAt(f, other, walk, st, spread)
			if w == Undecided || (read && w != got) {
				return Undecided
			}
			got, read = w, true
		}
	}
	return got
}

// readAt reads the wedge at one station from planar face f, which walks the
// edge with its material on the left in the edge's own sense when walk is
// +1 and against it when walk is −1. spread bounds how far the reading can
// move between this station and any other point of the edge.
func readAt(f, other *topo.Face, walk float64, st station, spread float64) Wedge {
	n1, e1, ok := outwardNormal(f, st)
	if !ok {
		return Undecided
	}
	n2, e2, ok := outwardNormal(other, st)
	if !ok {
		return Undecided
	}
	turn := n1.Cross(n2).Dot(st.tangent.Scale(walk))
	// Every factor is within its bound of a unit vector, so the triple
	// product of the held vectors is within this of the true one.
	bound := (1+e1)*(1+e2)*(1+st.tanErr) - 1 + arithSlack + spread
	bound = math.Nextafter(bound*(1+0x1p-50), math.Inf(1))
	switch {
	case math.IsNaN(turn) || math.IsNaN(bound) || math.Abs(turn) <= bound:
		return Undecided
	case turn > 0:
		return Salient
	default:
		return Reflex
	}
}

// outwardNormal is f's outward normal at the station and its proven bound:
// NormalAt's own bound plus, on a curved face, how far the normal can turn
// over the station's displacement.
func outwardNormal(f *topo.Face, st station) (r3.Vec, float64, bool) {
	m, err := topobuild.NormalAt(f, st.p)
	if err != nil {
		return r3.Vec{}, 0, false
	}
	b := m.Bound.Base()
	shift, ok := normalShift(f.Surface, st.p, st.delta)
	if !ok || math.IsNaN(b) || math.IsInf(b, 0) {
		return r3.Vec{}, 0, false
	}
	return m.Value, b + shift, true
}

// normalShift bounds how far a surface's normal turns between the held point
// p and any point within delta of it. A unit vector v/|v| moves at most
// 2|Δv|/|v| when v moves by Δv, so each arm divides twice the displacement
// of the vector it normalizes by that vector's smallest length.
func normalShift(s surfacegeom.Surface, p r3.Vec, delta float64) (float64, bool) {
	if delta == 0 {
		return 0, true
	}
	shift := func(moved, length float64) (float64, bool) {
		if !(length > moved) {
			return 0, false
		}
		return 2 * moved / (length - moved), true
	}
	switch s := s.(type) {
	case surfacegeom.Plane:
		return 0, true
	case surfacegeom.Cylinder:
		return shift(delta, radialDistance(p, s.Origin, s.Axis))
	case surfacegeom.Cone:
		// The cone's normal turns only with the radial direction.
		return shift(delta, radialDistance(p, s.Origin, s.Axis))
	case surfacegeom.Sphere:
		return shift(delta, p.Sub(s.Center).Len())
	case surfacegeom.Torus:
		major, err := s.Major.In(units.Millimeter)
		if err != nil {
			return 0, false
		}
		radial, ok := shift(delta, radialDistance(p, s.Center, s.Axis))
		if !ok {
			return 0, false
		}
		rel := p.Sub(s.Center)
		rdir, ok := rel.Sub(s.Axis.Scale(rel.Dot(s.Axis))).Normalize()
		if !ok {
			return 0, false
		}
		// The tube centre moves with the radial direction.
		moved := delta + math.Abs(major)*radial
		return shift(moved, p.Sub(s.Center.Add(rdir.Scale(major))).Len())
	default:
		return 0, false
	}
}

// radialDistance is p's distance from the axis line through origin along the
// unit direction axis.
func radialDistance(p, origin, axis r3.Vec) float64 {
	rel := p.Sub(origin)
	return rel.Sub(axis.Scale(rel.Dot(axis))).Len()
}

// stationsOf returns the points the wedge is read at: both ends of a line or
// an arc, and the one vertex of a whole circle.
func stationsOf(e *topo.Edge) ([]station, bool) {
	ds, ok := vertexBound(e.Start)
	if !ok {
		return nil, false
	}
	de, ok := vertexBound(e.End)
	if !ok {
		return nil, false
	}
	switch c := e.Curve.(type) {
	case surfacegeom.Line3:
		chord := e.End.Position.Sub(e.Start.Position)
		t, ok := chord.Normalize()
		length := chord.Len()
		if !ok || !(length > ds+de) {
			return nil, false
		}
		tanErr := 2 * (ds + de) / (length - ds - de)
		return []station{
			{p: e.Start.Position, delta: ds, tangent: t, tanErr: tanErr},
			{p: e.End.Position, delta: de, tangent: t, tanErr: tanErr},
		}, true
	case surfacegeom.Circle3:
		return circleStations(e, c.Center, c.Axis, ds, de)
	case surfacegeom.Arc3:
		return circleStations(e, c.Center, c.Axis, ds, de)
	default:
		return nil, false
	}
}

// circleStations reads a circle or arc at its vertices. The tangent at p is
// the axis crossed with the radius, and it departs from the true tangent by
// at most twice the radius's displacement over its smallest length; the
// radius moves with the vertex and with the curve's own proven displacement
// from its held circle, which an edge without that proof does not have.
func circleStations(e *topo.Edge, center, axis r3.Vec, ds, de float64) ([]station, bool) {
	if !e.CurveBounded {
		return nil, false
	}
	at := func(p r3.Vec, d float64) (station, bool) {
		radius := axis.Cross(p.Sub(center))
		t, ok := radius.Normalize()
		moved := d + e.CurveBound
		length := radius.Len()
		if !ok || !(length > moved) {
			return station{}, false
		}
		return station{p: p, delta: d, tangent: t, tanErr: 2 * moved / (length - moved)}, true
	}
	first, ok := at(e.Start.Position, ds)
	if !ok {
		return nil, false
	}
	if e.Start == e.End {
		return []station{first}, true
	}
	last, ok := at(e.End.Position, de)
	if !ok {
		return nil, false
	}
	return []station{first, last}, true
}

// vertexBound is v's proven displacement in millimetres.
func vertexBound(v *topo.Vertex) (float64, bool) {
	if v.Bound.Mag() == 0 {
		return 0, true
	}
	d, err := v.Bound.In(units.Millimeter)
	if err != nil || math.IsNaN(d) || math.IsInf(d, 0) || d < 0 {
		return 0, false
	}
	return d, true
}

// admitted reports whether e is an edge whose material angle is the same all
// along it, up to the spread it returns, so its ends speak for the whole
// edge, and at least one of its faces is planar to read the material side
// from.
//
// The class gate admits on selectorquery.ParallelDirs, a relative tolerance
// of 1e-9; it only chooses which edges are read and never decides a sign.
// Its tolerance cannot flip a decided sign between the points read:
//
//   - On a line, a plane's normal and the tangent are constant, and a
//     cylinder's normal turns monotonically about the cylinder's axis as the
//     line runs, through far less than a half turn. The reading is then a
//     cosine of that turn, which cannot dip to the other sign between two
//     ends that both decide one sign. Both ends are always read.
//   - On a circle or arc, the plane's tilt from the circle's axis and the
//     round face's tilt and offset from it move both normals around the
//     circle by at most their misalignment, which the spread charges twice.
func admitted(e *topo.Edge) (float64, bool) {
	f0, f1 := e.Faces[0].Surface, e.Faces[1].Surface
	_, p0 := f0.(surfacegeom.Plane)
	_, p1 := f1.(surfacegeom.Plane)
	switch c := e.Curve.(type) {
	case surfacegeom.Line3:
		dir := e.End.Position.Sub(e.Start.Position)
		return 0, (p0 || p1) && alongLine(f0, dir) && alongLine(f1, dir)
	case surfacegeom.Circle3:
		return circleFaces(f0, f1, c.Center, c.Axis, c.Radius)
	case surfacegeom.Arc3:
		return circleFaces(f0, f1, c.Center, c.Axis, c.Radius)
	default:
		return 0, false
	}
}

// alongLine reports whether a straight edge along dir meets surface s at one
// angle all along it: s is a plane, or a cylinder whose axis is parallel to
// the edge.
func alongLine(s surfacegeom.Surface, dir r3.Vec) bool {
	switch s := s.(type) {
	case surfacegeom.Plane:
		return true
	case surfacegeom.Cylinder:
		return selectorquery.ParallelDirs(s.Axis, dir)
	default:
		return false
	}
}

// circleFaces reports whether a circle about center and axis lies on a plane
// normal to axis and on a surface of revolution coaxial with it, and returns
// the spread its misalignment charges: twice the sine of the plane normal's
// tilt from the axis, plus twice the round face's axis tilt and its centre's
// offset from the circle's axis over the radius.
func circleFaces(f0, f1 surfacegeom.Surface, center, axis r3.Vec, radius units.Value) (float64, bool) {
	r, err := radius.In(units.Millimeter)
	if err != nil || !(r > 0) {
		return 0, false
	}
	plane, round := f0, f1
	if _, ok := plane.(surfacegeom.Plane); !ok {
		plane, round = f1, f0
	}
	pl, ok := plane.(surfacegeom.Plane)
	if !ok {
		return 0, false
	}
	n, ok := pl.Frame.N().Normalize()
	if !ok || !selectorquery.ParallelDirs(n, axis) {
		return 0, false
	}
	tilt := n.Cross(axis).Len()
	var misalign float64
	switch s := round.(type) {
	case surfacegeom.Cylinder:
		misalign, ok = coaxial(center, axis, s.Origin, s.Axis, r)
	case surfacegeom.Cone:
		misalign, ok = coaxial(center, axis, s.Origin, s.Axis, r)
	case surfacegeom.Torus:
		misalign, ok = coaxial(center, axis, s.Center, s.Axis, r)
	case surfacegeom.Sphere:
		rel := s.Center.Sub(center)
		ok = rel == (r3.Vec{}) || selectorquery.ParallelDirs(rel, axis)
		misalign = rel.Cross(axis).Len() / r
	default:
		return 0, false
	}
	if !ok {
		return 0, false
	}
	spread := 2 * (tilt + misalign)
	if math.IsNaN(spread) || math.IsInf(spread, 0) {
		return 0, false
	}
	return spread, true
}

// coaxial reports whether the circle about center and axis, of radius r,
// shares the axis line through origin along the unit direction dir, and
// returns the misalignment: the sine of the tilt between the axes plus the
// centre's offset from the line over r.
func coaxial(center, axis, origin, dir r3.Vec, r float64) (float64, bool) {
	if !selectorquery.ParallelDirs(axis, dir) {
		return 0, false
	}
	rel := center.Sub(origin)
	if rel != (r3.Vec{}) && !selectorquery.ParallelDirs(rel, dir) {
		return 0, false
	}
	return axis.Cross(dir).Len() + rel.Cross(dir).Len()/r, true
}
