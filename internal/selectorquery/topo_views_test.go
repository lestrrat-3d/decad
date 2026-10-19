package selectorquery_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/selectorquery"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

var (
	wallRef = provenance.Ref{Producer: 3, Role: "side(0,1)"}
	capRef  = provenance.Ref{Producer: 3, Role: provenance.RoleCapStart}
)

// borderedEdge is a straight edge from (0,0,0) to (3,0,4) with one face that
// carries wallRef and walks it on its outer loop, and a second face with no
// origin that walks it on a hole.
func borderedEdge() (*topo.Edge, *topo.Face, *topo.Face) {
	e := &topo.Edge{
		Curve: surfacegeom.Line3{}, Start: &topo.Vertex{Position: r3.Vec{}},
		End: &topo.Vertex{Position: r3.Vec{X: 3, Z: 4}}, Length: 5, Convex: true,
	}
	wall := &topo.Face{Origins: []provenance.Ref{wallRef},
		Loops: []*topo.Loop{{Outer: true, CoEdges: []topo.CoEdge{{Edge: e}}}}}
	other := &topo.Face{Loops: []*topo.Loop{{Outer: false, CoEdges: []topo.CoEdge{{Edge: e}}}}}
	e.Faces = []*topo.Face{wall, other}
	return e, wall, other
}

func TestEdgeNode(t *testing.T) {
	t.Parallel()

	t.Run("plain facts", func(t *testing.T) {
		t.Parallel()
		e, _, _ := borderedEdge()
		view := selectorquery.EdgeNode{Edge: e}
		require.True(t, view.SelectorConvex())
		require.InDelta(t, 5, view.SelectorLengthMM(), 0)
		start, end := view.SelectorEndpoints()
		require.Equal(t, r3.Vec{}, start)
		require.Equal(t, r3.Vec{X: 3, Z: 4}, end)
		require.False(t, view.SelectorCircular())
	})
	t.Run("free means exactly one face", func(t *testing.T) {
		t.Parallel()
		e, wall, _ := borderedEdge()
		view := selectorquery.EdgeNode{Edge: e}
		require.False(t, view.SelectorFree(), "two faces")
		e.Faces = []*topo.Face{wall}
		require.True(t, view.SelectorFree(), "one face")
		e.Faces = nil
		require.False(t, view.SelectorFree(), "no face")
	})
	t.Run("line direction", func(t *testing.T) {
		t.Parallel()
		e, _, _ := borderedEdge()
		dir, ok := selectorquery.EdgeNode{Edge: e}.SelectorLineDirection()
		require.True(t, ok)
		require.Equal(t, r3.Vec{X: 3, Z: 4}, dir)

		e.SelectorDirection, e.SelectorDirectionProven = r3.Vec{Y: 1}, true
		dir, ok = selectorquery.EdgeNode{Edge: e}.SelectorLineDirection()
		require.True(t, ok)
		require.Equal(t, r3.Vec{Y: 1}, dir, "a proven direction wins over the endpoints")

		e.SelectorDirectionProven = false
		e.Curve = surfacegeom.Circle3{}
		_, ok = selectorquery.EdgeNode{Edge: e}.SelectorLineDirection()
		require.False(t, ok, "a curved edge has no line direction")
	})
	t.Run("circular", func(t *testing.T) {
		t.Parallel()
		for _, curve := range []surfacegeom.Curve{surfacegeom.Circle3{}, surfacegeom.Arc3{}} {
			e, _, _ := borderedEdge()
			e.Curve = curve
			require.True(t, selectorquery.EdgeNode{Edge: e}.SelectorCircular())
		}
		e, _, _ := borderedEdge()
		e.Curve = surfacegeom.Ellipse3{}
		require.False(t, selectorquery.EdgeNode{Edge: e}.SelectorCircular())
	})
	t.Run("origin on any bordering face", func(t *testing.T) {
		t.Parallel()
		e, _, _ := borderedEdge()
		view := selectorquery.EdgeNode{Edge: e}
		require.True(t, view.SelectorHasOrigin(wallRef))
		require.False(t, view.SelectorHasOrigin(capRef))
	})
	t.Run("outer loop of the creating face", func(t *testing.T) {
		t.Parallel()
		e, wall, other := borderedEdge()
		view := selectorquery.EdgeNode{Edge: e}
		require.True(t, view.SelectorOnOuterLoop(wallRef))
		require.False(t, view.SelectorOnOuterLoop(capRef), "no face carries the origin")

		wall.Loops[0].Outer = false
		require.False(t, view.SelectorOnOuterLoop(wallRef), "the creating face walks it on a hole")

		wall.Loops[0].Outer = true
		wall.Loops[0].CoEdges[0].Edge = &topo.Edge{}
		other.Origins = nil
		require.False(t, view.SelectorOnOuterLoop(wallRef), "the creating face's outer loop walks another edge")
	})
}

func TestFaceNode(t *testing.T) {
	t.Parallel()

	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	plane := &topo.Face{Surface: surfacegeom.Plane{Frame: frame}, Origins: []provenance.Ref{capRef}}

	t.Run("planar normal", func(t *testing.T) {
		t.Parallel()
		n, ok := selectorquery.FaceNode{Face: plane}.SelectorPlanarNormal()
		require.True(t, ok)
		require.Equal(t, frame.N(), n)
		_, ok = selectorquery.FaceNode{Face: &topo.Face{Surface: surfacegeom.Cylinder{}}}.SelectorPlanarNormal()
		require.False(t, ok)
	})
	t.Run("outward normal follows the face sense", func(t *testing.T) {
		t.Parallel()
		face := &topo.Face{Surface: plane.Surface}
		n, ok := selectorquery.FaceNode{Face: face}.SelectorOutwardPlanarNormal()
		require.True(t, ok)
		require.Equal(t, frame.N(), n)
		face.Reversed = true
		n, ok = selectorquery.FaceNode{Face: face}.SelectorOutwardPlanarNormal()
		require.True(t, ok)
		require.Equal(t, frame.N().Scale(-1), n)
		_, ok = selectorquery.FaceNode{Face: &topo.Face{Surface: surfacegeom.Cylinder{}, Reversed: true}}.
			SelectorOutwardPlanarNormal()
		require.False(t, ok)
	})
	t.Run("cylindrical", func(t *testing.T) {
		t.Parallel()
		require.True(t, selectorquery.FaceNode{Face: &topo.Face{Surface: surfacegeom.Cylinder{}}}.SelectorCylindrical())
		require.False(t, selectorquery.FaceNode{Face: plane}.SelectorCylindrical())
	})
	t.Run("origin", func(t *testing.T) {
		t.Parallel()
		view := selectorquery.FaceNode{Face: plane}
		require.True(t, view.SelectorHasOrigin(capRef))
		require.False(t, view.SelectorHasOrigin(wallRef))
	})
	t.Run("wall of", func(t *testing.T) {
		t.Parallel()
		_, wall, _ := borderedEdge()
		view := selectorquery.FaceNode{Face: wall}
		require.True(t, view.SelectorIsWallOf(provenance.Ref{Producer: 3}))
		require.False(t, view.SelectorIsWallOf(provenance.Ref{Producer: 4}))
	})
}

func TestIsWallOf(t *testing.T) {
	t.Parallel()

	face := &topo.Face{Origins: []provenance.Ref{
		{Producer: 2, Role: "side(0,0)"}, {Producer: 3, Role: provenance.RoleCapStart}, {Producer: 5, Role: "sideways"}}}
	require.True(t, selectorquery.IsWallOf(face, 2), "a side role of the producer")
	require.False(t, selectorquery.IsWallOf(face, 3), "a cap role is no wall")
	require.False(t, selectorquery.IsWallOf(face, 5), "the prefix is side(")
	require.False(t, selectorquery.IsWallOf(face, 4), "no role of the producer")
	require.False(t, selectorquery.IsWallOf(&topo.Face{}, 2))
	require.Equal(t, "side(", selectorquery.RoleSidePrefix)
}
