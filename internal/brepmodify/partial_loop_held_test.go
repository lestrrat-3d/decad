package brepmodify

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// squareFace is a 10 mm square cap at z = 0 over material below it, whose four edges each lie between the cap and a wall
// normal to it. curves gives the curve of each edge, in walk order from the origin.
func squareFace(t *testing.T, curves [4]surfacegeom.Curve) (*topo.Face, *topo.Loop, *topo.Body) {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	corners := [4]r3.Vec{{X: 0, Y: 0}, {X: 10, Y: 0}, {X: 10, Y: 10}, {X: 0, Y: 10}}
	top := &topo.Face{Surface: surfacegeom.Plane{Frame: frame}}
	loop := &topo.Loop{Outer: true}
	var walls []*topo.Face
	for i := range corners {
		from, to := corners[i], corners[(i+1)%4]
		edge := &topo.Edge{Start: &topo.Vertex{Position: from}, End: &topo.Vertex{Position: to}, Curve: curves[i], Convex: true}
		dir := to.Sub(from)
		wf, err := r3.NewFrame(from, dir, r3.Vec{Z: 1})
		require.NoError(t, err)
		wall := &topo.Face{Surface: surfacegeom.Plane{Frame: wf}}
		low := &topo.Edge{Start: &topo.Vertex{Position: r3.Vec{X: from.X, Y: from.Y, Z: -3}},
			End: &topo.Vertex{Position: r3.Vec{X: to.X, Y: to.Y, Z: -3}}, Curve: surfacegeom.Line3{}}
		wall.Loops = []*topo.Loop{{Outer: true, CoEdges: []topo.CoEdge{{Edge: edge, Forward: true}, {Edge: low, Forward: true}}}}
		edge.Faces = []*topo.Face{top, wall}
		loop.CoEdges = append(loop.CoEdges, topo.CoEdge{Edge: edge, Forward: true})
		walls = append(walls, wall)
	}
	top.Loops = []*topo.Loop{loop}
	body := &topo.Body{Lumps: []*topo.Lump{{Shells: []*topo.FaceShell{{Faces: append([]*topo.Face{top}, walls...)}}}}}
	return top, loop, body
}

// The edge beside a selected edge is read as an exact line or arc, so an edge of no exact curve there is not read and
// the loop is declined; an edge farther away is read for the polygon alone, as the held chain of the face's boundary.
func TestReadChamferLoopReadsHeldChainsOfDistantEdgesOnly(t *testing.T) {
	t.Parallel()
	line, held := surfacegeom.Line3{}, surfacegeom.FacetedCurve{}
	read := func(t *testing.T, curves [4]surfacegeom.Curve, selected int, fp *heldFace) (*chamferLoop, bool) {
		top, loop, _ := squareFace(t, curves)
		reading := &loopReading{selected: map[*topo.Edge]bool{loop.CoEdges[selected].Edge: true}, held: fp}
		return readChamferLoop(top, top.Surface.(surfacegeom.Plane), loop, true, reading)
	}
	// The top's held boundary bulges out between the third corner and the fourth, through one more vertex.
	verts := []r3.Vec{{X: 0, Y: 0}, {X: 10, Y: 0}, {X: 10, Y: 10}, {X: 5, Y: 10.5}, {X: 0, Y: 10}}
	tris := [][3]int{{0, 1, 2}, {0, 2, 3}, {0, 3, 4}}
	hf, ok := newHeldFace(facetedrecord.Payload{Verts: verts, Tris: tris, FaceOf: make([]int, len(tris))}, 0)
	require.True(t, ok)

	t.Run("exact edges all round", func(t *testing.T) {
		l, ok := read(t, [4]surfacegeom.Curve{line, line, line, line}, 0, hf)
		require.True(t, ok)
		require.Len(t, l.segs, 4)
	})
	t.Run("a held edge beside the selected one", func(t *testing.T) {
		for _, beside := range []int{1, 3} {
			curves := [4]surfacegeom.Curve{line, line, line, line}
			curves[beside] = held
			_, ok := read(t, curves, 0, hf)
			require.False(t, ok, "edge %d", beside)
		}
	})
	t.Run("a held edge away from the selected one", func(t *testing.T) {
		l, ok := read(t, [4]surfacegeom.Curve{line, line, held, line}, 0, hf)
		require.True(t, ok)
		// The held edge reads as the two chords of the face's boundary through the extra vertex.
		require.Len(t, l.segs, 5)
		require.Len(t, l.edges, 5)
		for i, held := range []bool{false, false, true, true, false} {
			require.Equal(t, held, l.segs[i].held, "segment %d", i)
		}
		require.InDelta(t, 5, l.segs[3].p0.x, 1e-12)
		require.InDelta(t, 10.5, l.segs[3].p0.y, 1e-12)
		require.Same(t, l.edges[2], l.edges[3])
	})
}
