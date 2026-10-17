package loftcylinder_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftcylinder"
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
