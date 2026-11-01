package topo

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/denotation"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// These snapshots are private and are created only during native prism Build.
// The small cap keeps optional sealing from growing with arbitrary topology.
const prismFaceTopologyLimit = 64

type prismFaceTopologySeal struct{ loops []prismFaceLoopSeal }
type prismFaceLoopSeal struct {
	ptr   *Loop
	outer bool
	edges []prismFaceEdgeSeal
}
type prismFaceEdgeSeal struct {
	ptr                  *Edge
	forward              bool
	curve                surfacegeom.Curve
	start, end           *Vertex
	startValue, endValue prismFaceVertexSeal
	adjacent             [2]*Face
	adjacentCount        int
	direction            r3.Vec
	flags                [5]bool
	scalars              [4]uint64
	level                denotation.LevelToken
	denot                denotation.CurveToken
}
type prismFaceVertexSeal struct {
	position r3.Vec
	bound    units.Value
	level    denotation.LevelToken
	denot    denotation.CurveToken
}

func prismFaceVertexSnapshot(v *Vertex) prismFaceVertexSeal {
	return prismFaceVertexSeal{v.Position, v.Bound, v.Level, v.Denot}
}
func prismFaceEdgeScalars(e *Edge) [4]uint64 {
	return [4]uint64{math.Float64bits(e.Length), math.Float64bits(e.LengthBound), math.Float64bits(e.CurveBound), math.Float64bits(e.ChainBound)}
}
func prismFaceEdgeFlags(e *Edge) [5]bool {
	return [5]bool{e.SelectorDirectionProven, e.Convex, e.LengthUnbounded, e.CurveBounded, e.RetainedCurve}
}

func capturePrismFaceTopology(f *Face) (*prismFaceTopologySeal, bool) {
	if f == nil || len(f.Loops) > prismFaceTopologyLimit {
		return nil, false
	}
	count := len(f.Loops)
	for _, l := range f.Loops {
		if l == nil || len(l.CoEdges) > prismFaceTopologyLimit-count {
			return nil, false
		}
		count += len(l.CoEdges)
		for _, ce := range l.CoEdges {
			if ce.Edge == nil || ce.Edge.Start == nil || ce.Edge.End == nil || len(ce.Edge.Faces) > 2 {
				return nil, false
			}
			switch ce.Edge.Curve.(type) {
			case surfacegeom.Line3, surfacegeom.Circle3, surfacegeom.Arc3:
			default:
				return nil, false
			}
		}
	}
	out := &prismFaceTopologySeal{loops: make([]prismFaceLoopSeal, 0, len(f.Loops))}
	for _, l := range f.Loops {
		sl := prismFaceLoopSeal{ptr: l, outer: l.Outer, edges: make([]prismFaceEdgeSeal, 0, len(l.CoEdges))}
		for _, ce := range l.CoEdges {
			e := ce.Edge
			se := prismFaceEdgeSeal{
				ptr: e, forward: ce.Forward, curve: e.Curve, start: e.Start, end: e.End,
				startValue: prismFaceVertexSnapshot(e.Start), endValue: prismFaceVertexSnapshot(e.End),
				adjacentCount: len(e.Faces), direction: e.SelectorDirection, flags: prismFaceEdgeFlags(e),
				scalars: prismFaceEdgeScalars(e), level: e.Level, denot: e.Denot,
			}
			copy(se.adjacent[:], e.Faces)
			sl.edges = append(sl.edges, se)
		}
		out.loops = append(out.loops, sl)
	}
	return out, true
}
func samePrismTopologyValue(a, b units.Value) bool {
	return a == b && math.Float64bits(a.Base()) == math.Float64bits(b.Base())
}
func samePrismTopologyLevel(a, b denotation.LevelToken) bool {
	return a.ID == b.ID && samePrismVec(a.Origin, b.Origin) && samePrismVec(a.Normal, b.Normal)
}
func samePrismTopologyDenot(a, b denotation.CurveToken) bool {
	return a.ID == b.ID && samePrismTransform(a.Xform, b.Xform)
}
func samePrismTopologyVertex(v *Vertex, s prismFaceVertexSeal) bool {
	return v != nil && samePrismVec(v.Position, s.position) && samePrismTopologyValue(v.Bound, s.bound) && samePrismTopologyLevel(v.Level, s.level) && samePrismTopologyDenot(v.Denot, s.denot)
}
func samePrismTopologyCurve(a, b surfacegeom.Curve) bool {
	switch x := a.(type) {
	case surfacegeom.Line3:
		_, ok := b.(surfacegeom.Line3)
		return ok
	case surfacegeom.Circle3:
		y, ok := b.(surfacegeom.Circle3)
		return ok && samePrismVec(x.Center, y.Center) && samePrismVec(x.Axis, y.Axis) && samePrismTopologyValue(x.Radius, y.Radius)
	case surfacegeom.Arc3:
		y, ok := b.(surfacegeom.Arc3)
		return ok && samePrismVec(x.Center, y.Center) && samePrismVec(x.Axis, y.Axis) && samePrismTopologyValue(x.Radius, y.Radius)
	default:
		return false
	}
}

// ReplayMatchesTopology checks the construction-time topology under the caller's meter.
func (t PrismFaceToken) ReplayMatchesTopology(ctx context.Context, f *Face, b proof.ReplayKeyBudget) (bool, error) {
	if err := ctx.Err(); err != nil {
		return false, err
	}
	if b == nil {
		return false, nil
	}
	if err := b.SpendWork(ctx, 64); err != nil {
		return false, err
	}
	if t.face != f || f == nil || t.topology == nil || len(f.Loops) != len(t.topology.loops) {
		return false, nil
	}
	for i, l := range f.Loops {
		if err := b.SpendWork(ctx, 64); err != nil {
			return false, err
		}
		s := t.topology.loops[i]
		if l == nil || l != s.ptr || l.Outer != s.outer || len(l.CoEdges) != len(s.edges) {
			return false, nil
		}
		for j, ce := range l.CoEdges {
			if err := b.SpendWork(ctx, 512); err != nil {
				return false, err
			}
			x := s.edges[j]
			e := ce.Edge
			if e == nil || e != x.ptr || ce.Forward != x.forward ||
				e.Start != x.start || e.End != x.end || len(e.Faces) != x.adjacentCount ||
				!samePrismTopologyCurve(e.Curve, x.curve) ||
				!samePrismTopologyVertex(e.Start, x.startValue) || !samePrismTopologyVertex(e.End, x.endValue) ||
				!samePrismVec(e.SelectorDirection, x.direction) || prismFaceEdgeFlags(e) != x.flags ||
				prismFaceEdgeScalars(e) != x.scalars || !samePrismTopologyLevel(e.Level, x.level) ||
				!samePrismTopologyDenot(e.Denot, x.denot) {
				return false, nil
			}
			for k, g := range e.Faces {
				if g != x.adjacent[k] {
					return false, nil
				}
			}
		}
	}
	return true, nil
}
