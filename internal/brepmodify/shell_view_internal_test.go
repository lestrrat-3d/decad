package brepmodify

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// viewOf is a one-face-loop body whose faces a and b hold the given edges in
// one loop of a, so topobuild.Edges walks them in order.
func viewOf(a, b *topo.Face, edges ...*topo.Edge) *topo.Body {
	loop := &topo.Loop{Outer: true}
	for _, e := range edges {
		e.Faces = []*topo.Face{a, b}
		loop.CoEdges = append(loop.CoEdges, topo.CoEdge{Edge: e, Forward: true})
	}
	a.Loops = []*topo.Loop{loop}
	return &topo.Body{Lumps: []*topo.Lump{{Shells: []*topo.FaceShell{{Faces: []*topo.Face{a, b}}}}}}
}

func lineEdge(p, q r3.Vec) *topo.Edge {
	return &topo.Edge{Start: &topo.Vertex{Position: p}, End: &topo.Vertex{Position: q}}
}

// TestViewEdge maps a shell edge to the one view edge between its faces'
// images whose ends sit exactly at its own, and declines every other case.
func TestViewEdge(t *testing.T) {
	t.Parallel()
	srcA, srcB := &topo.Face{}, &topo.Face{}
	p0, p1, p2 := r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0)
	source := lineEdge(p0, p1)
	source.Faces = []*topo.Face{srcA, srcB}
	images := func(a, b *topo.Face) map[*topo.Face][]*topo.Face {
		return map[*topo.Face][]*topo.Face{srcA: {a}, srcB: {b}}
	}

	t.Run("matching ends", func(t *testing.T) {
		t.Parallel()
		a, b := &topo.Face{}, &topo.Face{}
		want := lineEdge(p1, p0)
		view := viewOf(a, b, lineEdge(p2, p1), want)
		got, ok := viewEdge(view, images(a, b), source)
		require.True(t, ok)
		require.Same(t, want, got)
	})
	t.Run("ends off the source edge", func(t *testing.T) {
		t.Parallel()
		a, b := &topo.Face{}, &topo.Face{}
		view := viewOf(a, b, lineEdge(p0, r3.NewVec(0, 0, 1+1.0/1024)))
		_, ok := viewEdge(view, images(a, b), source)
		require.False(t, ok)
	})
	t.Run("two candidates", func(t *testing.T) {
		t.Parallel()
		a, b := &topo.Face{}, &topo.Face{}
		view := viewOf(a, b, lineEdge(p0, p1), lineEdge(p0, p1))
		_, ok := viewEdge(view, images(a, b), source)
		require.False(t, ok)
	})
	t.Run("faces outside the images", func(t *testing.T) {
		t.Parallel()
		a, b := &topo.Face{}, &topo.Face{}
		view := viewOf(a, b, lineEdge(p0, p1))
		_, ok := viewEdge(view, images(a, &topo.Face{}), source)
		require.False(t, ok)
	})
}
