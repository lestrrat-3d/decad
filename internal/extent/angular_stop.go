package extent

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

const angularStopTolerance = 1e-9

// AngularStop is the axis point and the directions spanning its radial
// profile plane. W is the caller's unit axis direction and E1 is W crossed
// with R0.
type AngularStop struct {
	A3, W, R0, E1 r3.Vec
}

// AngularEdgeKind identifies the closed-form boundary curves this stop reads.
type AngularEdgeKind uint8

const (
	AngularEdgeUnknown AngularEdgeKind = iota
	AngularEdgeLine
	AngularEdgeCircle
	AngularEdgeArc
)

// AngularEdge is one source edge's held geometry without public topology.
// CurveType names an unsupported curve in the caller's refusal message.
type AngularEdge struct {
	Start, End   *r3.Vec
	Kind         AngularEdgeKind
	CurveType    string
	Center, Axis r3.Vec
	Radius       units.Value
}

// ToFaceAngularStop resolves an angular to-face stop at face into the signed
// stop angle about the axis, caller frame, radians. The resolved face must be
// usable as a revolve cap — the build table's caps are planar faces in the
// rotated profile plane, which CONTAINS the axis — so the face must be
// PLANAR, its plane must contain the revolve axis, and its material must lie
// in one half-plane of the axis; a non-planar face, or a plane that does not
// contain the axis, is a solid this evaluator cannot build
// (ErrUnsupported, staged), while a face spanning both half-planes names two
// stops at once and one in the profile's own half-plane a zero sweep — both
// ErrDegenerate.
//
// travel is 0 for a standalone ToFaceAngular — the target face supplies the
// sense, and both senses reach a radial plane, so the sweep takes the nearer
// way around (Along on the exact half-turn tie) — and ±1 for a TwoSidedAngle
// side.
func ToFaceAngularStop(face *topo.Face, stop AngularStop, travel float64) (float64, error) {
	pl, ok := face.Surface.(surfacegeom.Plane)
	if !ok {
		// The gate is an ANALYTIC planar surface (a Plane — a revolve radial
		// cap), not merely a flat face: a boolean-built face can be flat yet
		// carry a Faceted surface, and this evaluator cannot use it as a stop.
		// The message names the actual public surface type so the caller sees
		// why.
		return 0, fmt.Errorf(`%w: ToFaceAngular requires a stop face with an analytic Plane surface whose plane contains the revolve axis, not merely a flat face; this face's surface is %s, which this evaluator cannot use as a stop even when flat — choose any analytic planar face whose plane contains the axis, or use an angle extent (AngleExtent, or AngleSide inside a TwoSidedAngle extent)`, decaderr.ErrUnsupported, surfacegeom.SurfaceTypeName(face.Surface))
	}
	nf := pl.Frame.N()
	if math.Abs(nf.Dot(stop.W)) > stopTol {
		return 0, fmt.Errorf(`%w: ToFaceAngular requires the stop face's plane to contain the revolve axis; this plane is not parallel to the axis — choose a radial face or use an angle extent (AngleExtent, or AngleSide inside a TwoSidedAngle extent)`, decaderr.ErrUnsupported)
	}
	off := pl.Frame.Origin().Sub(stop.A3).Dot(nf)
	if math.Abs(off) > RelativeStopTolerance(math.Max(pl.Frame.Origin().Len(), stop.A3.Len())) {
		return 0, fmt.Errorf(`%w: ToFaceAngular requires the stop face's plane to contain the revolve axis; this plane runs parallel to the axis but offset from it — choose a face through the axis or use an angle extent (AngleExtent, or AngleSide inside a TwoSidedAngle extent)`, decaderr.ErrUnsupported)
	}
	var boundary []AngularEdge
	for _, loop := range face.Loops {
		for _, coedge := range loop.CoEdges {
			boundary = append(boundary, angularEdgeOf(coedge.Edge))
		}
	}
	phi, err := FaceHalfPlane(stop, boundary)
	if err != nil {
		return 0, err
	}
	if phi <= angularStopTolerance || phi >= 2*math.Pi-angularStopTolerance {
		return 0, fmt.Errorf(`%w: the stop face lies in the profile's own half-plane`, decaderr.ErrDegenerate)
	}
	switch {
	case travel > 0:
		return phi, nil
	case travel < 0:
		return phi - 2*math.Pi, nil
	case phi <= math.Pi:
		return phi, nil
	default:
		return phi - 2*math.Pi, nil
	}
}

// angularEdgeOf reads the held geometry the half-plane audit needs from e.
func angularEdgeOf(e *topo.Edge) AngularEdge {
	record := AngularEdge{CurveType: surfacegeom.CurveTypeName(e.Curve)}
	if e.Start != nil {
		record.Start = &e.Start.Position
	}
	if e.End != nil {
		record.End = &e.End.Position
	}
	switch c := e.Curve.(type) {
	case surfacegeom.Line3:
		record.Kind = AngularEdgeLine
	case surfacegeom.Circle3:
		record.Kind, record.Center, record.Axis, record.Radius = AngularEdgeCircle, c.Center, c.Axis, c.Radius
	case surfacegeom.Arc3:
		record.Kind, record.Center, record.Axis, record.Radius = AngularEdgeArc, c.Center, c.Axis, c.Radius
	}
	return record
}

var errStopFaceBothSides = fmt.Errorf(`%w: the stop face spans both sides of the revolve axis`, decaderr.ErrDegenerate)

// FaceHalfPlane finds the one radial half-plane occupied by a face boundary.
// A circular edge is checked between its endpoint probes before admission.
func FaceHalfPlane(stop AngularStop, edges []AngularEdge) (float64, error) {
	phi := math.NaN()
	for _, edge := range edges {
		for _, p := range boundaryProbes(edge) {
			x := p.Sub(stop.A3)
			u, w := x.Dot(stop.R0), x.Dot(stop.E1)
			if math.Hypot(u, w) <= RelativeStopTolerance(x.Len()) {
				continue
			}
			a := math.Atan2(w, u)
			if a < 0 {
				a += 2 * math.Pi
			}
			if math.IsNaN(phi) {
				phi = a
				continue
			}
			diff := math.Abs(a - phi)
			if diff > math.Pi {
				diff = 2*math.Pi - diff
			}
			if diff > angularStopTolerance {
				return 0, errStopFaceBothSides
			}
		}
	}
	if math.IsNaN(phi) {
		return 0, fmt.Errorf(`%w: the stop face has no material off the revolve axis`, decaderr.ErrDegenerate)
	}
	m := stop.R0.Scale(math.Cos(phi)).Add(stop.E1.Scale(math.Sin(phi)))
	for _, edge := range edges {
		if err := RejectAxisCrossing(stop, edge, m); err != nil {
			return 0, err
		}
	}
	return phi, nil
}

// boundaryProbes uses each edge's vertices and one point on a circular walk.
func boundaryProbes(edge AngularEdge) []r3.Vec {
	var probes []r3.Vec
	if edge.Start != nil {
		probes = append(probes, *edge.Start)
	}
	if edge.End != nil {
		probes = append(probes, *edge.End)
	}
	switch edge.Kind {
	case AngularEdgeCircle:
		if edge.Start != nil {
			probes = append(probes, edge.Center.Scale(2).Sub(*edge.Start))
		}
	case AngularEdgeArc:
		if edge.Start == nil || edge.End == nil {
			break
		}
		n, ok := edge.Axis.Normalize()
		if !ok {
			break
		}
		r, err := edge.Radius.In(units.Millimeter)
		if err != nil {
			break
		}
		u0, ok := edge.Start.Sub(edge.Center).Normalize()
		if !ok {
			break
		}
		v0 := n.Cross(u0)
		x := edge.End.Sub(edge.Center)
		sweep := math.Atan2(x.Dot(v0), x.Dot(u0))
		if sweep < 0 {
			sweep += 2 * math.Pi
		}
		sin, cos := math.Sincos(sweep / 2)
		probes = append(probes, edge.Center.Add(u0.Scale(cos*r)).Add(v0.Scale(sin*r)))
	}
	return probes
}

// RejectAxisCrossing checks whether a curve reaches the opposite radial
// half-plane. A line between agreeing vertices cannot cross. A circular walk
// has a closed-form deepest point; the audit checks whether the walk reaches
// it. An unsupported curve can bulge across the axis between agreeing
// vertices, so it refuses instead of treating the missing probe as clearance.
func RejectAxisCrossing(stop AngularStop, edge AngularEdge, m r3.Vec) error {
	if edge.Kind == AngularEdgeLine {
		return nil
	}
	if edge.Kind != AngularEdgeCircle && edge.Kind != AngularEdgeArc {
		return fmt.Errorf(`%w: the angular stop audit has no closed-form axis-crossing probe for a %s boundary edge`,
			decaderr.ErrUnsupported, edge.CurveType)
	}
	r, err := edge.Radius.In(units.Millimeter)
	if err != nil {
		return fmt.Errorf(`%w: a stop face edge's radius is not representable: %s`, decaderr.ErrNotFinite, err)
	}
	depth := edge.Center.Sub(stop.A3).Dot(m) - r
	if depth >= -RelativeStopTolerance(r) {
		return nil
	}
	if edge.Kind == AngularEdgeCircle {
		return errStopFaceBothSides
	}
	n, ok := edge.Axis.Normalize()
	if !ok {
		return fmt.Errorf(`%w: a stop face arc edge has no axis`, decaderr.ErrDegenerate)
	}
	if edge.Start == nil || edge.End == nil {
		return errStopFaceBothSides
	}
	u0, ok := edge.Start.Sub(edge.Center).Normalize()
	if !ok {
		return errStopFaceBothSides
	}
	v0 := n.Cross(u0)
	ang := func(x r3.Vec) float64 {
		a := math.Atan2(x.Dot(v0), x.Dot(u0))
		if a < 0 {
			a += 2 * math.Pi
		}
		return a
	}
	sweep := ang(edge.End.Sub(edge.Center))
	deepest := ang(m.Scale(-1))
	if deepest <= sweep+angularStopTolerance {
		return errStopFaceBothSides
	}
	return nil
}
