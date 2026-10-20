package selectorquery

import (
	"slices"
	"strings"

	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// RoleSidePrefix opens every wall role a sweep mints, side(i, j). The Walls
// predicate and the complete-wall-set test of Draft read it.
const RoleSidePrefix = "side("

// IsWallOf reports whether f carries a side(i, j) role minted by producer.
func IsWallOf(f *topo.Face, producer provenance.Producer) bool {
	for _, o := range f.Origins {
		if o.Producer == producer && strings.HasPrefix(o.Role, RoleSidePrefix) {
			return true
		}
	}
	return false
}

// EdgeNode exposes only the analytic facts a predicate reads of an edge node.
type EdgeNode struct{ *topo.Edge }

func (e EdgeNode) SelectorConvex() bool { return e.Convex }
func (e EdgeNode) SelectorFree() bool   { return len(e.Faces) == 1 }
func (e EdgeNode) SelectorLineDirection() (r3.Vec, bool) {
	if e.SelectorDirectionProven {
		return e.SelectorDirection, true
	}
	if _, ok := e.Curve.(surfacegeom.Line3); !ok {
		return r3.Vec{}, false
	}
	return e.End.Position.Sub(e.Start.Position), true
}
func (e EdgeNode) SelectorEndpoints() (r3.Vec, r3.Vec) {
	return e.Start.Position, e.End.Position
}
func (e EdgeNode) SelectorLengthMM() float64 { return e.Length }
func (e EdgeNode) SelectorHasOrigin(ref provenance.Ref) bool {
	for _, face := range e.Faces {
		if slices.Contains(face.Origins, ref) {
			return true
		}
	}
	return false
}
func (e EdgeNode) SelectorOnOuterLoop(ref provenance.Ref) bool {
	for _, face := range e.Faces {
		if !slices.Contains(face.Origins, ref) {
			continue
		}
		for _, loop := range face.Loops {
			if !loop.Outer {
				continue
			}
			for _, ce := range loop.CoEdges {
				if ce.Edge == e.Edge {
					return true
				}
			}
		}
	}
	return false
}
func (e EdgeNode) SelectorCircular() bool {
	switch e.Curve.(type) {
	case surfacegeom.Circle3, surfacegeom.Arc3:
		return true
	default:
		return false
	}
}

// FaceNode exposes surface identity and the outward planar normal of a face
// node.
type FaceNode struct{ *topo.Face }

func (f FaceNode) SelectorPlanarNormal() (r3.Vec, bool) {
	plane, ok := f.Surface.(surfacegeom.Plane)
	if !ok {
		return r3.Vec{}, false
	}
	return plane.Frame.N(), true
}
func (f FaceNode) SelectorOutwardPlanarNormal() (r3.Vec, bool) {
	normal, ok := f.SelectorPlanarNormal()
	if ok && f.Reversed {
		normal = normal.Scale(-1)
	}
	return normal, ok
}
func (f FaceNode) SelectorCylindrical() bool {
	_, ok := f.Surface.(surfacegeom.Cylinder)
	return ok
}
func (f FaceNode) SelectorHasOrigin(ref provenance.Ref) bool {
	return slices.Contains(f.Origins, ref)
}
func (f FaceNode) SelectorIsWallOf(ref provenance.Ref) bool {
	return IsWallOf(f.Face, ref.Producer)
}
