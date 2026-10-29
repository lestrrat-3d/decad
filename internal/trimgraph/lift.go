package trimgraph

import (
	"context"
	"fmt"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/anasurf"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

func notLiftable(format string, args ...any) error {
	return fmt.Errorf("%w: %s", ErrNotLiftable, fmt.Sprintf(format, args...))
}

// Lift reads b, a body over fp, into a graph. It returns ErrNotLiftable (wrapped with the reason) when a lift rule
// fails, and ctx.Err() when ctx is done; it never returns a partial graph.
func Lift(ctx context.Context, b *topo.Body, fp facetedrecord.Payload) (Graph, error) {
	// L1.
	if !b.Solid || len(b.Lumps) != 1 || len(b.Lumps[0].Shells) != 1 {
		return Graph{}, notLiftable("the body is not one solid shell (L1)")
	}
	if shell := b.Lumps[0].Shells[0]; shell.Open || shell.Void {
		return Graph{}, notLiftable("the body's shell is open or a void (L1)")
	}
	nodes := topobuild.Faces(b)
	if len(fp.FaceOf) != len(fp.Tris) || len(fp.Src) != len(fp.Tris) {
		return Graph{}, notLiftable("the payload does not map its facets to faces")
	}
	var g Graph
	faceIdx := make(map[*topo.Face]int, len(nodes))
	for i, f := range nodes {
		faceIdx[f] = i
	}
	groupOf, err := groupsOfFaces(fp, len(nodes))
	if err != nil {
		return Graph{}, err
	}
	for i, f := range nodes {
		if err := ctx.Err(); err != nil {
			return Graph{}, err
		}
		face, err := liftFace(f, groupOf[i], i)
		if err != nil {
			return Graph{}, err
		}
		g.Faces = append(g.Faces, face)
	}
	if err := g.liftTopology(ctx, faceIdx); err != nil {
		return Graph{}, err
	}
	if err := g.checkConvexity(ctx); err != nil {
		return Graph{}, err
	}
	return g, nil
}

// groupsOfFaces returns the one group id every facet of each face carries.
func groupsOfFaces(fp facetedrecord.Payload, faceCount int) ([]int, error) {
	groupOf := make([]int, faceCount)
	for i := range groupOf {
		groupOf[i] = -1
	}
	for k, fi := range fp.FaceOf {
		if fi < 0 || fi >= faceCount || fp.Src[k] < 0 || fp.Src[k] >= len(fp.Groups) {
			return nil, notLiftable("facet %d maps outside the faces or groups", k)
		}
		if groupOf[fi] >= 0 && groupOf[fi] != fp.Src[k] {
			return nil, notLiftable("face %d carries facets of two groups", fi)
		}
		groupOf[fi] = fp.Src[k]
	}
	for fi, g := range groupOf {
		if g < 0 {
			return nil, notLiftable("face %d has no facet", fi)
		}
	}
	return groupOf, nil
}

// liftFace applies L2 to one face.
func liftFace(f *topo.Face, group, index int) (Face, error) {
	switch {
	case !f.RetainedTag:
		return Face{}, notLiftable("face %d has no retained tag (L2)", index)
	case f.TagDeparture != 0 || f.NormalBound != 0 || f.HeldNormalBound != 0:
		return Face{}, notLiftable("face %d states a tag departure (L2)", index)
	case f.Denoted != nil:
		return Face{}, notLiftable("face %d is denoted (L2)", index)
	case f.HasAxialDelta && f.AxialDelta != 0:
		return Face{}, notLiftable("face %d states an axial departure (L2)", index)
	}
	if _, ok := f.Surface.(surfacegeom.Plane); !ok {
		return Face{}, notLiftable("face %d is not a plane (L2)", index)
	}
	return Face{Node: f, Group: group, Tag: f.Surface, Reversed: f.Reversed}, nil
}

// corner is one face's corner at a vertex: the edge its loop arrives by.
type corner struct{ face, in int }

// liftTopology builds edges, loops and vertices (L3, L5).
func (g *Graph) liftTopology(ctx context.Context, faceIdx map[*topo.Face]int) error {
	type use struct {
		face    int
		forward bool
	}
	edgeIdx := map[*topo.Edge]int{}
	vertexIdx := map[*topo.Vertex]int{}
	var uses [][]use
	var corners [][]corner
	vertexOf := func(v *topo.Vertex) int {
		if i, ok := vertexIdx[v]; ok {
			return i
		}
		vertexIdx[v] = len(g.Vertices)
		g.Vertices = append(g.Vertices, Vertex{Node: v, Pos: v.Position, Bound: v.Bound.Base()})
		corners = append(corners, nil)
		return vertexIdx[v]
	}
	for fi := range g.Faces {
		if err := ctx.Err(); err != nil {
			return err
		}
		node := g.Faces[fi].Node
		if len(node.Loops) == 0 || !node.Loops[0].Outer {
			return notLiftable("face %d has no outer loop first", fi)
		}
		for li, loop := range node.Loops {
			if loop.Outer != (li == 0) || len(loop.CoEdges) < 3 {
				return notLiftable("face %d has a malformed loop", fi)
			}
			var walk []Use
			prevEnd := -1
			firstStart, lastEdge := -1, -1
			for ci, ce := range loop.CoEdges {
				e := ce.Edge
				if _, ok := e.Curve.(surfacegeom.Line3); !ok {
					return notLiftable("an edge of face %d is not a line (L3)", fi)
				}
				if e.Start == nil || e.End == nil || e.Start == e.End {
					return notLiftable("an edge of face %d does not join two vertices (L3)", fi)
				}
				ei, seen := edgeIdx[e]
				if !seen {
					ei = len(g.Edges)
					edgeIdx[e] = ei
					g.Edges = append(g.Edges, Edge{Node: e, Start: vertexOf(e.Start), End: vertexOf(e.End), Curve: CurveLine})
					uses = append(uses, nil)
				}
				uses[ei] = append(uses[ei], use{face: fi, forward: ce.Forward})
				start, end := g.Edges[ei].Start, g.Edges[ei].End
				if !ce.Forward {
					start, end = end, start
				}
				if ci > 0 && start != prevEnd {
					return notLiftable("a loop of face %d is not a closed walk", fi)
				}
				if ci == 0 {
					firstStart = start
				} else {
					corners[start] = append(corners[start], corner{face: fi, in: lastEdge})
				}
				prevEnd, lastEdge = end, ei
				walk = append(walk, Use{Edge: ei, Forward: ce.Forward})
			}
			if prevEnd != firstStart {
				return notLiftable("a loop of face %d does not close", fi)
			}
			corners[firstStart] = append(corners[firstStart], corner{face: fi, in: lastEdge})
			g.Faces[fi].Loops = append(g.Faces[fi].Loops, walk)
		}
	}
	for ei := range g.Edges {
		u := uses[ei]
		e := &g.Edges[ei]
		if len(u) != 2 || u[0].face == u[1].face || u[0].forward == u[1].forward {
			return notLiftable("edge %d does not join two distinct faces with opposite walks (L3)", ei)
		}
		if u[1].forward {
			u[0], u[1] = u[1], u[0]
		}
		e.Faces = [2]int{u[0].face, u[1].face}
		if len(e.Node.Faces) != 2 {
			return notLiftable("edge %d is not between two faces (L3)", ei)
		}
		a, okA := faceIdx[e.Node.Faces[0]]
		b, okB := faceIdx[e.Node.Faces[1]]
		if !okA || !okB || (a != e.Faces[0] || b != e.Faces[1]) && (a != e.Faces[1] || b != e.Faces[0]) {
			return notLiftable("edge %d disagrees with its faces' loops (L3)", ei)
		}
		g.Vertices[e.Start].Edges = append(g.Vertices[e.Start].Edges, ei)
		g.Vertices[e.End].Edges = append(g.Vertices[e.End].Edges, ei)
	}
	for vi := range g.Vertices {
		if err := g.orderVertexFaces(vi, corners[vi]); err != nil {
			return err
		}
	}
	return nil
}

// orderVertexFaces applies L5 and orders the faces around vertex vi.
func (g *Graph) orderVertexFaces(vi int, corners []corner) error {
	v := &g.Vertices[vi]
	if len(v.Edges) != 3 || len(corners) != 3 {
		return notLiftable("the vertex at %v has valence %d, not 3 (L5)", v.Pos, len(v.Edges))
	}
	slices.SortFunc(corners, func(a, b corner) int { return a.face - b.face })
	if corners[0].face == corners[1].face || corners[1].face == corners[2].face {
		return notLiftable("a face meets the vertex at %v twice (L5)", v.Pos)
	}
	cornerOf := func(f int) (int, bool) {
		for _, c := range corners {
			if c.face == f {
				return c.in, true
			}
		}
		return 0, false
	}
	f := corners[0].face
	for range corners {
		v.Faces = append(v.Faces, f)
		in, _ := cornerOf(f)
		e := g.Edges[in]
		next := e.Faces[0]
		if next == f {
			next = e.Faces[1]
		}
		if _, ok := cornerOf(next); !ok {
			return notLiftable("the faces at the vertex at %v do not close up (L5)", v.Pos)
		}
		f = next
	}
	if f != corners[0].face || v.Faces[0] == v.Faces[1] || v.Faces[1] == v.Faces[2] || v.Faces[0] == v.Faces[2] {
		return notLiftable("the faces at the vertex at %v do not close up (L5)", v.Pos)
	}
	return nil
}

// checkConvexity applies L6: it decides each edge's convexity from the tags and requires the held flag to agree.
func (g *Graph) checkConvexity(ctx context.Context) error {
	planes := make([]anasurf.Plane, len(g.Faces))
	for i, f := range g.Faces {
		p, err := anasurf.PlaneOf(f.Tag.(surfacegeom.Plane), f.Reversed)
		if err != nil {
			return notLiftable("face %d has no exact plane (L6): %v", i, err)
		}
		planes[i] = p
	}
	for ei := range g.Edges {
		if err := ctx.Err(); err != nil {
			return err
		}
		e := &g.Edges[ei]
		p0, p1 := g.Vertices[e.Start], g.Vertices[e.End]
		d := [3]*big.Rat{}
		okAll := true
		for i, c := range [3][2]float64{{p1.Pos.X, p0.Pos.X}, {p1.Pos.Y, p0.Pos.Y}, {p1.Pos.Z, p0.Pos.Z}} {
			hi, ok1 := proofbound.RatOf(c[0])
			lo, ok2 := proofbound.RatOf(c[1])
			okAll = okAll && ok1 && ok2
			if ok1 && ok2 {
				d[i] = new(big.Rat).Sub(hi, lo)
			}
		}
		b0, ok0 := proofbound.RatOf(p0.Bound)
		b1, ok1 := proofbound.RatOf(p1.Bound)
		if !okAll || !ok0 || !ok1 || p0.Bound < 0 || p1.Bound < 0 {
			return notLiftable("edge %d has a non-finite vertex (L6)", ei)
		}
		bounds := proofbound.RatAdd(b0, b1)
		cross := ratCross(planes[e.Faces[0]].N, planes[e.Faces[1]].N)
		s := new(big.Rat)
		q := new(big.Rat)
		for i := range cross {
			s.Add(s, new(big.Rat).Mul(cross[i], d[i]))
			q.Add(q, new(big.Rat).Mul(cross[i], cross[i]))
		}
		// |s| > |cross|·(β0+β1), compared squared so no root is needed.
		margin := new(big.Rat).Mul(bounds, bounds)
		margin.Mul(margin, q)
		if new(big.Rat).Mul(s, s).Cmp(margin) <= 0 {
			return notLiftable("the tags cannot decide the convexity of edge %d (L6)", ei)
		}
		e.Convex = s.Sign() > 0
		if e.Convex != e.Node.Convex {
			return notLiftable("the tags and the held mesh disagree on the convexity of edge %d (L6)", ei)
		}
	}
	return nil
}

func ratCross(a, b [3]*big.Rat) [3]*big.Rat {
	c := func(i, j int) *big.Rat {
		return new(big.Rat).Sub(new(big.Rat).Mul(a[i], b[j]), new(big.Rat).Mul(a[j], b[i]))
	}
	return [3]*big.Rat{c(1, 2), c(2, 0), c(0, 1)}
}
