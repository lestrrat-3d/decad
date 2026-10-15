package stitchrecord

import (
	"github.com/lestrrat-3d/decad/internal/denotation"
	"github.com/lestrrat-3d/decad/internal/stitchweld"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// This file adapts docs/surface-design.md §6.2's Table J to the held
// vertices and edges of the operand faces. The class and pair decisions in
// internal/stitchweld are recorded once and replayed by every later placement.

// VertexTable pairs each class with its first registered vertex's
// position, bound and curve token. The generic table compares float bits,
// so a negative zero never merges with a positive zero.
type VertexTable struct {
	*stitchweld.Table[*topo.Vertex, denotation.CurveToken]
}

func NewVertexTable() *VertexTable {
	return &VertexTable{Table: stitchweld.NewTable[*topo.Vertex, denotation.CurveToken]()}
}

func (t *VertexTable) classOf(v *topo.Vertex) int {
	return t.ClassOf(v, v.Position, v.Bound.Base(), v.Denot, v.Denot.ID != 0)
}

// Plan records the admitted free-edge pairs over one Stitch call.
// A placement replays these groups without running Table J again.
type Plan struct {
	Table  *VertexTable
	Group  map[*topo.Edge]int // welded edges only; absent means the edge stays free
	Groups int
}

// BuildPlan registers operand vertices, then groups eligible free
// edges in their original walk order. Line3's geometry is stated entirely by
// its endpoint classes. Other variants need a nonzero curve token and the
// variant check in edgesJoin. Exactly two edges may claim a weld key;
// one or three or more leave every member free.
func BuildPlan(faces []*topo.Face) *Plan {
	table := NewVertexTable()
	for _, f := range faces {
		for _, l := range f.Loops {
			for _, ce := range l.CoEdges {
				table.classOf(topobuild.CoEdgeStart(ce))
				table.classOf(topobuild.CoEdgeEnd(ce))
			}
		}
	}

	var candidates []stitchweld.Candidate[*topo.Edge]
	seen := map[*topo.Edge]struct{}{}
	for _, f := range faces {
		for _, l := range f.Loops {
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
				candidates = append(candidates, stitchweld.Candidate[*topo.Edge]{
					Edge: e, Start: table.classOf(e.Start), End: table.classOf(e.End),
				})
			}
		}
	}
	group, groups := stitchweld.WeldPairs(candidates, edgesJoin)
	return &Plan{Table: table, Group: group, Groups: groups}
}

// edgesJoin decides Table J's J2/J3 for two free edges that already
// share an unordered pair of vertex classes. A Line3 needs no further data;
// another variant joins only with the same nonzero token and held geometry.
func edgesJoin(a, b *topo.Edge) bool {
	_, aLine := a.Curve.(surfacegeom.Line3)
	_, bLine := b.Curve.(surfacegeom.Line3)
	if aLine && bLine {
		return true
	}
	return denotation.SameCurve(a.Denot, b.Denot) && sameCurveVariant(a.Curve, b.Curve)
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
