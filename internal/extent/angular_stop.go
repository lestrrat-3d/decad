package extent

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

const angularStopTolerance = 1e-9

// AngularStop is the axis point and the two directions spanning its radial
// profile plane. E1 is the caller's axis direction crossed with R0.
type AngularStop struct {
	A3, R0, E1 r3.Vec
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
