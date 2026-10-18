package clearance

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// NewCEdge reads a topology edge as a clearance segment, circle, or arc.
// A zero-length line contributes no edge; unsupported curves refuse.
func NewCEdge(e *topo.Edge) (*CEdge, bool) {
	switch c := e.Curve.(type) {
	case surfacegeom.Line3:
		if e.Start.Position.Sub(e.End.Position).Len() == 0 {
			return nil, true
		}
		ce := &CEdge{Line: true, A: e.Start.Position, B: e.End.Position}
		ce.Box = BoxOf(ce.A, ce.B)
		return ce, true
	case surfacegeom.Circle3:
		r, err := c.Radius.In(units.Millimeter)
		if err != nil {
			return nil, false
		}
		u := PerpTo(c.Axis)
		v := c.Axis.Cross(u)
		ce := &CEdge{Center: c.Center, Axis: c.Axis, RefU: u, RefV: v, Radius: r, Ang: AngWindow{Full: true}}
		ce.Box = CircleBox(c.Center, c.Axis, r)
		return ce, true
	case surfacegeom.Arc3:
		r, err := c.Radius.In(units.Millimeter)
		if err != nil {
			return nil, false
		}
		rel := e.Start.Position.Sub(c.Center)
		u, ok := rel.Normalize()
		if !ok {
			return nil, false
		}
		v := c.Axis.Cross(u)
		relEnd := e.End.Position.Sub(c.Center)
		sweep := math.Atan2(relEnd.Dot(v), relEnd.Dot(u))
		if sweep <= 0 {
			sweep += 2 * math.Pi
		}
		ce := &CEdge{Center: c.Center, Axis: c.Axis, RefU: u, RefV: v, Radius: r, Ang: NewAngWindow(0, sweep)}
		ce.Box = CircleBox(c.Center, c.Axis, r)
		return ce, true
	default:
		return nil, false
	}
}

// ShellWitness returns a point provenly on the shell: the first available
// coedge start, or a loop-less closed face's synthesized surface point (§2).
func ShellWitness(sh *topo.FaceShell) (r3.Vec, bool) {
	for _, f := range sh.Faces {
		for _, l := range f.Loops {
			for _, ce := range l.CoEdges {
				if ce.Edge.Start != nil {
					return ce.Edge.Start.Position, true
				}
			}
		}
	}
	for _, f := range sh.Faces {
		switch s := f.Surface.(type) {
		case surfacegeom.Sphere:
			r, err := s.Radius.In(units.Millimeter)
			if err != nil {
				return r3.Vec{}, false
			}
			return s.Center.Add(PerpTo(r3.NewVec(0, 0, 1)).Scale(r)), true
		case surfacegeom.Torus:
			major, err1 := s.Major.In(units.Millimeter)
			minor, err2 := s.Minor.In(units.Millimeter)
			if err1 != nil || err2 != nil {
				return r3.Vec{}, false
			}
			return s.Center.Add(PerpTo(s.Axis).Scale(major + minor)), true
		}
	}
	return r3.Vec{}, false
}
