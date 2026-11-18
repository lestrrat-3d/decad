package tangentchain_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/tangentchain"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestApart(t *testing.T) {
	t.Parallel()
	v := &topo.Vertex{Position: r3.NewVec(10, 0, 0)}
	e := &topo.Edge{Curve: surfacegeom.Line3{}, Start: &topo.Vertex{Position: r3.Vec{}}, End: v}
	at := func(p r3.Vec) *topo.Edge {
		return &topo.Edge{Curve: surfacegeom.Line3{}, Start: v, End: &topo.Vertex{Position: p}}
	}
	require.True(t, tangentchain.Apart(e, at(r3.NewVec(10, 5, 0)), v), "a right-angled turn")
	require.True(t, tangentchain.Apart(e, at(r3.NewVec(5, 1, 0)), v), "a turn back")
	require.False(t, tangentchain.Apart(e, at(r3.NewVec(20, 0, 0)), v), "straight on")
	require.False(t, tangentchain.Apart(e, at(r3.NewVec(20, 1e-12, 0)), v), "inside the noise band")
	chord := &topo.Edge{Curve: surfacegeom.FacetedCurve{}, Start: v, End: &topo.Vertex{Position: r3.NewVec(10, 5, 0)}}
	require.False(t, tangentchain.Apart(e, chord, v), "a chord chain carries no direction")
}

func TestHeldApart(t *testing.T) {
	t.Parallel()
	// The held vertex 0 is v. Faces 0 and 1 meet along the chord from v to vertex 1; face 2 holds vertex 2 alone.
	v := &topo.Vertex{Position: r3.NewVec(10, 0, 0)}
	e := &topo.Edge{Curve: surfacegeom.Line3{}, Start: &topo.Vertex{Position: r3.Vec{}}, End: v}
	mesh := func(far r3.Vec, bound float64) tangentchain.HeldMesh {
		return tangentchain.HeldMesh{
			Verts:  []r3.Vec{v.Position, far, r3.NewVec(10, 0, 1), r3.NewVec(10, 0, -1)},
			Bounds: []float64{0, bound, 0, 0},
			Tris:   [][3]int{{0, 1, 2}, {0, 3, 1}},
			FaceOf: []int{0, 1},
		}
	}
	require.True(t, tangentchain.HeldApart(mesh(r3.NewVec(10, 5, 0), 0), 0, 1, 0.002, e, v), "a right-angled chord")
	require.False(t, tangentchain.HeldApart(mesh(r3.NewVec(20, 1e-6, 0), 0), 0, 1, 0.002, e, v),
		"a chord straight on within its uncertainty")
	require.False(t, tangentchain.HeldApart(mesh(r3.NewVec(10, 0.01, 0), 0.01), 0, 1, 0.002, e, v),
		"a chord shorter than its bounds can read")
	require.False(t, tangentchain.HeldApart(mesh(r3.NewVec(10, 5, 0), 0), 0, 2, 0.002, e, v),
		"no chord the two faces share")
}
