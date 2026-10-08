package decad

import "github.com/lestrrat-3d/decad/internal/stitchweld"

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
	return t.ClassOf(v, v.position, v.bound.Base(), v.denot, v.denot.ID != 0)
}

// stitchWeldPlan records the admitted free-edge pairs over one Stitch call.
// A placement replays these groups without running Table J again.
type stitchWeldPlan struct {
	table  *stitchVertexTable
	group  map[*Edge]int // welded edges only; absent means the edge stays free
	groups int
}

// buildStitchWeldPlan registers operand vertices, then groups eligible free
// edges in their original walk order. Line3's geometry is stated entirely by
// its endpoint classes. Other variants need a nonzero curve token and the
// variant check in stitchEdgesJoin. Exactly two edges may claim a weld key;
// one or three or more leave every member free.
func buildStitchWeldPlan(faces []*Face) *stitchWeldPlan {
	table := newStitchVertexTable()
	for _, f := range faces {
		for _, l := range f.loops {
			for _, ce := range l.coedges {
				table.classOf(ce.Start())
				table.classOf(ce.End())
			}
		}
	}

	var candidates []stitchweld.Candidate[*Edge]
	seen := map[*Edge]struct{}{}
	for _, f := range faces {
		for _, l := range f.loops {
			for _, ce := range l.coedges {
				e := ce.edge
				if _, dup := seen[e]; dup {
					continue
				}
				seen[e] = struct{}{}
				if !e.IsFree() { // J1
					continue
				}
				_, isLine := e.curve.(Line3)
				if !isLine && e.denot.ID == 0 { // J2, then J5, or the certificate
					continue
				}
				candidates = append(candidates, stitchweld.Candidate[*Edge]{
					Edge: e, Start: table.classOf(e.start), End: table.classOf(e.end),
				})
			}
		}
	}
	group, groups := stitchweld.WeldPairs(candidates, stitchEdgesJoin)
	return &stitchWeldPlan{table: table, group: group, groups: groups}
}

// stitchEdgesJoin decides Table J's J2/J3 for two free edges that already
// share an unordered pair of vertex classes. A Line3 needs no further data;
// another variant joins only with the same nonzero token and held geometry.
func stitchEdgesJoin(a, b *Edge) bool {
	_, aLine := a.curve.(Line3)
	_, bLine := b.curve.(Line3)
	if aLine && bLine {
		return true
	}
	return sameCurve(a.denot, b.denot) && sameCurveVariant(a.curve, b.curve)
}

// sameCurveVariant compares held Circle3 and Arc3 parameters, allowing
// opposite axis signs. Any other pairing declines.
func sameCurveVariant(a, b Curve) bool {
	switch av := a.(type) {
	case Circle3:
		bv, ok := b.(Circle3)
		if !ok {
			return false
		}
		return av.Center == bv.Center && av.Radius.Base() == bv.Radius.Base() &&
			(av.Axis == bv.Axis || av.Axis == bv.Axis.Scale(-1))
	case Arc3:
		bv, ok := b.(Arc3)
		if !ok {
			return false
		}
		return av.Center == bv.Center && av.Radius.Base() == bv.Radius.Base() &&
			(av.Axis == bv.Axis || av.Axis == bv.Axis.Scale(-1))
	default:
		return false
	}
}
