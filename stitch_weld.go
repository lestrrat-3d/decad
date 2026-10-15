package decad

import (
	"github.com/lestrrat-3d/decad/internal/stitchweld"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// This file adapts docs/surface-design.md §6.2's Table J to the held
// vertices and edges of the operand faces. The class and pair decisions in
// internal/stitchweld are recorded once and replayed by every later placement.

// stitchVertexTable pairs each class with its first registered vertex's
// position, bound and curve token. The generic table compares float bits,
// so a negative zero never merges with a positive zero.
type stitchVertexTable struct {
	*stitchweld.Table[*Vertex, curveToken]
}

func newStitchVertexTable() *stitchVertexTable {
	return &stitchVertexTable{Table: stitchweld.NewTable[*Vertex, curveToken]()}
}

func (t *stitchVertexTable) classOf(v *Vertex) int {
	return t.ClassOf(v, v.n.Position, v.n.Bound.Base(), v.n.Denot, v.n.Denot.ID != 0)
}

// stitchWeldPlan records the admitted free-edge pairs over one Stitch call.
// A placement replays these groups without running Table J again.
type stitchWeldPlan struct {
	Table  *stitchVertexTable
	Group  map[*Edge]int // welded edges only; absent means the edge stays free
	Groups int
}

// buildStitchWeldPlan registers operand vertices, then groups eligible free
// edges in their original walk order. Line3's geometry is stated entirely by
// its endpoint classes. Other variants need a nonzero curve token and the
// variant check in stitchEdgesJoin. Exactly two edges may claim a weld key;
// one or three or more leave every member free.
func buildStitchWeldPlan(faces []*Face) *stitchWeldPlan {
	table := newStitchVertexTable()
	for _, f := range faces {
		for _, l := range f.n.Loops {
			for _, ce := range l.CoEdges {
				table.classOf(vertexHandle(coedgeStart(ce)))
				table.classOf(vertexHandle(coedgeEnd(ce)))
			}
		}
	}

	var candidates []stitchweld.Candidate[*Edge]
	seen := map[*topo.Edge]struct{}{}
	for _, f := range faces {
		for _, l := range f.n.Loops {
			for _, ce := range l.CoEdges {
				e := ce.Edge
				if _, dup := seen[e]; dup {
					continue
				}
				seen[e] = struct{}{}
				if len(e.Faces) != 1 { // J1
					continue
				}
				_, isLine := e.Curve.(surfacegeom.Line3)
				if !isLine && e.Denot.ID == 0 { // J2, then J5, or the certificate
					continue
				}
				candidates = append(candidates, stitchweld.Candidate[*Edge]{
					Edge: edgeHandle(e), Start: table.classOf(vertexHandle(e.Start)), End: table.classOf(vertexHandle(e.End)),
				})
			}
		}
	}
	group, groups := stitchweld.WeldPairs(candidates, stitchEdgesJoin)
	return &stitchWeldPlan{Table: table, Group: group, Groups: groups}
}

// stitchEdgesJoin decides Table J's J2/J3 for two free edges that already
// share an unordered pair of vertex classes. A Line3 needs no further data;
// another variant joins only with the same nonzero token and held geometry.
func stitchEdgesJoin(a, b *Edge) bool {
	_, aLine := a.n.Curve.(surfacegeom.Line3)
	_, bLine := b.n.Curve.(surfacegeom.Line3)
	if aLine && bLine {
		return true
	}
	return sameCurve(a.n.Denot, b.n.Denot) && sameCurveVariant(a.n.Curve, b.n.Curve)
}

// sameCurveVariant compares held Circle3 and Arc3 parameters, allowing
// opposite axis signs. Any other pairing declines.
func sameCurveVariant(a, b surfacegeom.Curve) bool {
	switch av := a.(type) {
	case surfacegeom.Circle3:
		bv, ok := b.(surfacegeom.Circle3)
		if !ok {
			return false
		}
		return av.Center == bv.Center && av.Radius.Base() == bv.Radius.Base() &&
			(av.Axis == bv.Axis || av.Axis == bv.Axis.Scale(-1))
	case surfacegeom.Arc3:
		bv, ok := b.(surfacegeom.Arc3)
		if !ok {
			return false
		}
		return av.Center == bv.Center && av.Radius.Base() == bv.Radius.Base() &&
			(av.Axis == bv.Axis || av.Axis == bv.Axis.Scale(-1))
	default:
		return false
	}
}
