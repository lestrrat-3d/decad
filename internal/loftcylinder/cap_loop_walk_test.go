package loftcylinder_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestChamferCapWalkReadsCoEdgeDirection(t *testing.T) {
	verts := []r3.Vec{{}, {X: 1}, {Y: 1}}
	a := &topo.Vertex{Position: verts[0]}
	b := &topo.Vertex{Position: verts[1]}
	c := &topo.Vertex{Position: verts[2]}
	ab := &topo.Edge{Start: a, End: b}
	bc := &topo.Edge{Start: b, End: c}
	ca := &topo.Edge{Start: c, End: a}
	forward := []topo.CoEdge{{Edge: ab, Forward: true}, {Edge: bc, Forward: true}, {Edge: ca, Forward: true}}
	backward := []topo.CoEdge{{Edge: ca}, {Edge: bc}, {Edge: ab}}
	ring := []int{0, 1, 2}

	require.True(t, loftcylinder.ChamferCapWalk(&topo.Loop{Outer: true, CoEdges: forward}, verts, ring, false))
	require.True(t, loftcylinder.ChamferCapWalk(&topo.Loop{Outer: true, CoEdges: backward}, verts, ring, true))
	require.False(t, loftcylinder.ChamferCapWalk(&topo.Loop{Outer: true, CoEdges: backward}, verts, ring, false))

	missingEdge := append([]topo.CoEdge(nil), forward...)
	missingEdge[0] = topo.CoEdge{}
	require.False(t, loftcylinder.ChamferCapWalk(&topo.Loop{Outer: true, CoEdges: missingEdge}, verts, ring, false))
	missingStart := append([]topo.CoEdge(nil), forward...)
	missingStart[0].Edge = &topo.Edge{End: b}
	require.False(t, loftcylinder.ChamferCapWalk(&topo.Loop{Outer: true, CoEdges: missingStart}, verts, ring, false))
}

// TestChamferSelectsOuterLoops admits exactly the edges of both outer cap
// loops and refuses a split or missing cap, a repeated, missing or extra
// edge, and a loop that does not walk its recorded ring.
func TestChamferSelectsOuterLoops(t *testing.T) {
	t.Parallel()
	verts := []r3.Vec{{}, {X: 1}, {Y: 1}, {Z: 1}, {X: 1, Z: 1}, {Y: 1, Z: 1}}
	nodes := make([]*topo.Vertex, len(verts))
	for i, v := range verts {
		nodes[i] = &topo.Vertex{Position: v}
	}
	triangle := func(a, b, c int) [3]*topo.Edge {
		return [3]*topo.Edge{
			{Start: nodes[a], End: nodes[b]},
			{Start: nodes[b], End: nodes[c]},
			{Start: nodes[c], End: nodes[a]},
		}
	}
	bottom, top := triangle(0, 1, 2), triangle(3, 4, 5)
	// The bottom cap's ring is CCW from +Z, so its public loop walks it backward.
	bottomLoop := &topo.Loop{Outer: true, CoEdges: []topo.CoEdge{
		{Edge: bottom[2]}, {Edge: bottom[1]}, {Edge: bottom[0]},
	}}
	topLoop := &topo.Loop{Outer: true, CoEdges: []topo.CoEdge{
		{Edge: top[0], Forward: true}, {Edge: top[1], Forward: true}, {Edge: top[2], Forward: true},
	}}
	start := provenance.Ref{Producer: 3, Role: provenance.RoleCapStart}
	end := provenance.Ref{Producer: 3, Role: provenance.RoleCapEnd}
	outer := [2][]int{{0, 1, 2}, {3, 4, 5}}
	body := func(faces ...*topo.Face) *topo.Body {
		return &topo.Body{Lumps: []*topo.Lump{{Shells: []*topo.FaceShell{{Faces: faces}}}}}
	}
	startFace := &topo.Face{Origins: []provenance.Ref{start}, Loops: []*topo.Loop{bottomLoop}}
	endFace := &topo.Face{Origins: []provenance.Ref{end}, Loops: []*topo.Loop{topLoop}}
	side := &topo.Edge{Start: nodes[0], End: nodes[3]}
	all := []*topo.Edge{bottom[0], bottom[1], bottom[2], top[0], top[1], top[2]}

	require.NoError(t, loftcylinder.ChamferSelectsOuterLoops(body(startFace, endFace), start, end,
		all, verts, outer), "both complete outer cap loops must be admitted")

	for _, tc := range []struct {
		name  string
		body  *topo.Body
		edges []*topo.Edge
		outer [2][]int
		want  string
	}{
		{"split start cap", body(startFace, endFace, startFace), all, outer, "start cap is split"},
		{"split end cap", body(startFace, endFace, endFace), all, outer, "end cap is split"},
		{"no end cap", body(startFace), all, outer, "no current cap faces"},
		{"repeated edge", body(startFace, endFace), append(append([]*topo.Edge(nil), all...), top[0]), outer,
			"repeats an edge"},
		{"missing edge", body(startFace, endFace), all[:5], outer, "needs both complete outer cap loops"},
		{"extra edge", body(startFace, endFace), append(append([]*topo.Edge(nil), all...), side), outer,
			"includes a bore or side edge"},
		{"reversed ring", body(startFace, endFace), all, [2][]int{{0, 1, 2}, {5, 4, 3}}, "cap 1 has no outer loop"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			err := loftcylinder.ChamferSelectsOuterLoops(tc.body, start, end, tc.edges, verts, tc.outer)
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
			require.ErrorContains(t, err, tc.want)
		})
	}
}
