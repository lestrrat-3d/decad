package tangentchain

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestContinuesRequiresMatchingFaceSheets(t *testing.T) {
	t.Parallel()
	vertex := &topo.Vertex{Position: r3.NewVec(10, 0, 4)}
	start := &topo.Vertex{Position: r3.NewVec(0, 0, 4)}
	far := &topo.Vertex{Position: r3.NewVec(20, 0, 4)}
	back := &topo.Vertex{Position: r3.NewVec(5, 0, 4)}
	topFrame, err := r3.NewFrame(r3.NewVec(0, 0, 4), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	frontFrame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	rightFrame, err := r3.NewFrame(r3.NewVec(10, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	top := &topo.Face{Surface: surfacegeom.Plane{Frame: topFrame}}
	front := &topo.Face{Surface: surfacegeom.Plane{Frame: frontFrame}}
	right := &topo.Face{Surface: surfacegeom.Plane{Frame: rightFrame}}
	e := &topo.Edge{Curve: surfacegeom.Line3{}, Start: start, End: vertex, Faces: []*topo.Face{front, top}}
	continues := &topo.Edge{Curve: surfacegeom.Line3{}, Start: vertex, End: far, Faces: []*topo.Face{front, top}}
	wrongSheets := &topo.Edge{Curve: surfacegeom.Line3{}, Start: vertex, End: far, Faces: []*topo.Face{right, top}}
	backtracks := &topo.Edge{Curve: surfacegeom.Line3{}, Start: vertex, End: back, Faces: []*topo.Face{front, top}}

	require.Equal(t, clearance.DegYes, Continues(e, continues, vertex))
	require.Equal(t, clearance.DegNo, Continues(e, wrongSheets, vertex))
	require.Equal(t, clearance.DegNo, Continues(e, backtracks, vertex))
}
