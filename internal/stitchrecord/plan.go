package stitchrecord

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/denotation"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stitchweld"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// This file adapts agent-docs/surface-design.md §6.2's Table J to the held
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
	// Merged is, per vertex class, how far the farthest vertex a gap tolerance
	// merged into the class lies from the class's held position, as a proven
	// upper bound in millimetres. A class no vertex was merged into is absent.
	// The merge is recorded here and replayed by every placement.
	Merged map[int]float64
}

// MaxMerged is the largest Merged bound, zero when the plan merged nothing.
func (p *Plan) MaxMerged() float64 {
	worst := 0.0
	for _, m := range p.Merged {
		worst = max(worst, m)
	}
	return worst
}

// BuildPlan registers operand vertices, then groups eligible free
// edges in their original walk order. Line3's geometry is stated entirely by
// its endpoint classes. Other variants need a nonzero curve token and the
// variant check in edgesJoin. Exactly two edges may claim a weld key;
// one or three or more leave every member free.
func BuildPlan(faces []*topo.Face) *Plan {
	return buildPlan(faces, nil)
}

// BuildTolerantPlan is BuildPlan with a gap tolerance: zero-bound vertices of
// planar, straight-edged faces that lie within reach of an earlier such vertex
// join its class, so free edges whose endpoints miss by no more than reach weld
// (agent-docs/gap-tolerance-design.md §5.3). A tolerance of zero, and a merge
// that would collapse an edge or repeat a vertex in a face's loop, give the
// exact plan.
func BuildTolerantPlan(faces []*topo.Face, reach float64) *Plan {
	if !(reach > 0) {
		return buildPlan(faces, nil)
	}
	exact := buildPlan(faces, nil)
	merged := mergeNear(faces, reach)
	if len(merged) == 0 {
		return exact
	}
	plan := buildPlan(faces, merged)
	if !loopsKeepTheirShape(faces, exact.Table, plan.Table) {
		return exact
	}
	return plan
}

// mergedVertex is where a tolerance merge puts a vertex, and how far that is
// from where the vertex holds.
type mergedVertex struct {
	at   r3.Vec
	dist float64
}

// mergeNear clusters the vertices a gap tolerance may merge. Only a zero-bound
// vertex every edge of which is a straight line on a planar face with no normal
// bound takes part: those are the vertices whose held position is their whole
// geometry, so a curve certificate has nothing to say about them. Vertices are taken in walk order and
// each joins the first earlier representative within reach, so a class never
// reaches farther than reach from its representative.
func mergeNear(faces []*topo.Face, reach float64) map[*topo.Vertex]mergedVertex {
	eligible := map[*topo.Vertex]bool{}
	var order []*topo.Vertex
	note := func(v *topo.Vertex, ok bool) {
		prev, seen := eligible[v]
		if !seen {
			order = append(order, v)
			prev = true
		}
		eligible[v] = prev && ok && v.Bound.Base() == 0
	}
	for _, f := range faces {
		_, plane := f.Surface.(surfacegeom.Plane)
		planar := plane && f.NormalBound == 0
		for _, l := range f.Loops {
			for _, ce := range l.CoEdges {
				_, line := ce.Edge.Curve.(surfacegeom.Line3)
				ok := planar && line
				note(topobuild.CoEdgeStart(ce), ok)
				note(topobuild.CoEdgeEnd(ce), ok)
			}
		}
	}
	type cell struct{ x, y, z int64 }
	cellOf := func(p r3.Vec) cell {
		return cell{int64(math.Floor(p.X / reach)), int64(math.Floor(p.Y / reach)), int64(math.Floor(p.Z / reach))}
	}
	rank := make(map[*topo.Vertex]int, len(order))
	for i, v := range order {
		rank[v] = i
	}
	grid := map[cell][]*topo.Vertex{}
	out := map[*topo.Vertex]mergedVertex{}
	for _, v := range order {
		if !eligible[v] {
			continue
		}
		c := cellOf(v.Position)
		var rep *topo.Vertex
		for dx := int64(-1); dx <= 1 && rep == nil; dx++ {
			for dy := int64(-1); dy <= 1 && rep == nil; dy++ {
				for dz := int64(-1); dz <= 1; dz++ {
					for _, r := range grid[cell{c.x + dx, c.y + dy, c.z + dz}] {
						if separation(v.Position, r.Position) <= reach && (rep == nil || rank[r] < rank[rep]) {
							rep = r
						}
					}
				}
			}
		}
		if rep == nil {
			grid[c] = append(grid[c], v)
			continue
		}
		if v.Position == rep.Position {
			continue
		}
		out[v] = mergedVertex{at: rep.Position, dist: separation(v.Position, rep.Position)}
	}
	return out
}

// separation is an upper bound on the distance between two held positions.
func separation(a, b r3.Vec) float64 {
	d := math.Max(math.Abs(a.X-b.X), math.Max(math.Abs(a.Y-b.Y), math.Abs(a.Z-b.Z)))
	return proofbound.Radius3D(proofbound.UpRound(d))
}

// loopsKeepTheirShape reports whether the tolerant table collapses no edge and
// repeats no vertex in a face's loop that the exact table does not already.
func loopsKeepTheirShape(faces []*topo.Face, exact, tolerant *VertexTable) bool {
	repeats := func(t *VertexTable, l *topo.Loop) int {
		seen := map[int]struct{}{}
		n := 0
		for _, ce := range l.CoEdges {
			c := t.classOf(topobuild.CoEdgeStart(ce))
			if _, dup := seen[c]; dup {
				n++
			}
			seen[c] = struct{}{}
		}
		return n
	}
	for _, f := range faces {
		for _, l := range f.Loops {
			for _, ce := range l.CoEdges {
				if tolerant.classOf(topobuild.CoEdgeStart(ce)) == tolerant.classOf(topobuild.CoEdgeEnd(ce)) &&
					exact.classOf(topobuild.CoEdgeStart(ce)) != exact.classOf(topobuild.CoEdgeEnd(ce)) {
					return false
				}
			}
			if repeats(tolerant, l) > repeats(exact, l) {
				return false
			}
		}
	}
	return true
}

// buildPlan registers the vertices, a vertex in merged at the position its
// entry names, and groups the free edges.
func buildPlan(faces []*topo.Face, merged map[*topo.Vertex]mergedVertex) *Plan {
	table := NewVertexTable()
	mergedBound := map[int]float64{}
	register := func(v *topo.Vertex) {
		m, ok := merged[v]
		if !ok {
			table.classOf(v)
			return
		}
		class := table.ClassOf(v, m.at, 0, v.Denot, false)
		mergedBound[class] = max(mergedBound[class], m.dist)
	}
	for _, f := range faces {
		for _, l := range f.Loops {
			for _, ce := range l.CoEdges {
				register(topobuild.CoEdgeStart(ce))
				register(topobuild.CoEdgeEnd(ce))
			}
		}
	}
	for class, d := range mergedBound {
		table.Bounds[class] = proofbound.AbsSumUpper(table.Bounds[class], d)
	}

	var lines, curves []stitchweld.Candidate[*topo.Edge]
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
				if !isLine && e.Denot.ID == 0 && !zeroBoundCircular(e) { // J2, then J5, or the certificate
					continue
				}
				c := stitchweld.Candidate[*topo.Edge]{
					Edge: e, Start: table.classOf(e.Start), End: table.classOf(e.End),
				}
				if isLine {
					lines = append(lines, c)
					continue
				}
				curves = append(curves, c)
			}
		}
	}
	// A straight edge and a curved edge between the same two classes are
	// different edges, so each kind claims its weld keys on its own: a curved
	// pair never makes a straight pair one claim too many, nor the reverse.
	group, groups := stitchweld.WeldPairs(lines, edgesJoin)
	curveGroup, curveGroups := stitchweld.WeldPairs(curves, edgesJoin)
	for e, id := range curveGroup {
		group[e] = groups + id
	}
	groups += curveGroups
	return &Plan{Table: table, Group: group, Groups: groups, Merged: mergedBound}
}

// edgesJoin decides Table J's J2/J3 for two free edges that already
// share an unordered pair of vertex classes. A Line3 needs no further data;
// another variant joins with the same nonzero token and held geometry, or, for
// a Circle3 or Arc3, when both edges prove a zero curve bound and hold one circle
// (agent-docs/surface-design.md §6.2, J5 for circles).
func edgesJoin(a, b *topo.Edge) bool {
	_, aLine := a.Curve.(surfacegeom.Line3)
	_, bLine := b.Curve.(surfacegeom.Line3)
	if aLine && bLine {
		return true
	}
	if denotation.SameCurve(a.Denot, b.Denot) && sameCurveVariant(a.Curve, b.Curve) {
		return true
	}
	return zeroBoundCircular(a) && zeroBoundCircular(b) && sameZeroBoundCircle(a, b)
}

// zeroBoundCircular reports whether e is a Circle3 or Arc3 edge whose builder
// proved CurveBound zero and whose end vertices all carry a zero bound, so
// that its held data is the true curve.
func zeroBoundCircular(e *topo.Edge) bool {
	switch e.Curve.(type) {
	case surfacegeom.Circle3, surfacegeom.Arc3:
	default:
		return false
	}
	return e.CurveBounded && e.CurveBound == 0 && e.Start.Bound.Base() == 0 && e.End.Bound.Base() == 0
}

// sameZeroBoundCircle compares the held circles of two zero-bound circular
// edges bit for bit. A Circle3 pairs only with a Circle3. An Arc3 pairs only
// with an Arc3 sweeping the same angular range: an arc is counter-clockwise
// about its axis from its start to its end, so edges that meet start to start
// need equal axes and edges that meet start to end need opposite axes. Any
// other meeting is the complementary arc, or an arc whose endpoints coincide.
func sameZeroBoundCircle(a, b *topo.Edge) bool {
	switch av := a.Curve.(type) {
	case surfacegeom.Circle3:
		bv, ok := b.Curve.(surfacegeom.Circle3)
		return ok && sameHeldCircle(av.Center, av.Axis, av.Radius.Base(), bv.Center, bv.Axis, bv.Radius.Base())
	case surfacegeom.Arc3:
		bv, ok := b.Curve.(surfacegeom.Arc3)
		if !ok || !sameHeldCircle(av.Center, av.Axis, av.Radius.Base(), bv.Center, bv.Axis, bv.Radius.Base()) {
			return false
		}
		if a.Start.Position == a.End.Position {
			return false
		}
		switch {
		case a.Start.Position == b.Start.Position && a.End.Position == b.End.Position:
			return av.Axis == bv.Axis
		case a.Start.Position == b.End.Position && a.End.Position == b.Start.Position:
			return av.Axis == bv.Axis.Scale(-1)
		}
	}
	return false
}

func sameHeldCircle(ac, aa r3.Vec, ar float64, bc, ba r3.Vec, br float64) bool {
	return ac == bc && ar == br && (aa == ba || aa == ba.Scale(-1))
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
