package explodeblock_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/explodeblock"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func moved(m explodeblock.Mesh, off r3.Vec) explodeblock.Mesh {
	out := m
	out.Vertices = make([]r3.Vec, len(m.Vertices))
	for i, v := range m.Vertices {
		out.Vertices[i] = v.Add(off)
	}
	return out
}

func TestGroupSharesTranslatedMeshes(t *testing.T) {
	base := collarShaft()
	offs := []r3.Vec{{}, r3.NewVec(10.1, -3.7, 0.3), r3.NewVec(-7, 22.25, 5)}
	meshes := []explodeblock.Mesh{base, ring(0.02), moved(base, offs[1]), moved(base, offs[2])}
	shares, classes := explodeblock.Group(meshes)
	require.Len(t, classes, 2)
	require.Equal(t, shares[0].Class, shares[2].Class)
	require.Equal(t, shares[0].Class, shares[3].Class)
	require.NotEqual(t, shares[0].Class, shares[1].Class)
	for _, k := range []int{2, 3} {
		require.InDelta(t, offs[k-1].X, shares[k].Offset.X, 1e-12)
		require.InDelta(t, offs[k-1].Y, shares[k].Offset.Y, 1e-12)
		require.InDelta(t, offs[k-1].Z, shares[k].Offset.Z, 1e-12)
	}
	require.Equal(t, r3.Vec{}, shares[0].Offset)
}

func TestGroupKeepsDifferentMeshesApart(t *testing.T) {
	a := collarShaft()
	b := moved(a, r3.NewVec(5, 0, 0))
	b.Vertices[3] = b.Vertices[3].Add(r3.NewVec(0, 0, 0.01)) // one vertex off by far more than rounding
	c := a
	c.Triangles = append([][3]int(nil), a.Triangles...)
	c.Triangles[0], c.Triangles[1] = c.Triangles[1], c.Triangles[0]
	_, classes := explodeblock.Group([]explodeblock.Mesh{a, b, c})
	require.Len(t, classes, 3)
}

func TestGroupBoundCoversMembers(t *testing.T) {
	a := collarShaft()
	b := moved(a, r3.NewVec(3, 4, 5))
	b.Bound = units.Millimeters(0.05)
	_, classes := explodeblock.Group([]explodeblock.Mesh{a, b})
	require.Len(t, classes, 1)
	require.GreaterOrEqual(t, classes[0].Bound.Base(), 0.05)
}

// A projection of the class mesh moved by the offset answers exactly as a projection of the moved mesh.
func TestGroupedInstanceBlocksLikeTheMovedMesh(t *testing.T) {
	off := r3.NewVec(0.3, -0.2, 0)
	bearing := moved(ring(0.02), off)
	shaftM := collarShaft()
	shares, classes := explodeblock.Group([]explodeblock.Mesh{ring(0.02), bearing})
	require.Len(t, classes, 1)
	pc, err := explodeblock.Project(classes[0], down)
	require.NoError(t, err)
	ps, err := explodeblock.Project(shaftM, down)
	require.NoError(t, err)
	pd, err := explodeblock.Project(bearing, down)
	require.NoError(t, err)
	shared := explodeblock.Block(explodeblock.Instance{P: ps}, explodeblock.Instance{P: pc, Offset: shares[1].Offset})
	direct := explodeblock.Block(explodeblock.Instance{P: ps}, explodeblock.Instance{P: pd})
	require.Equal(t, direct.Verdict, shared.Verdict)
	require.Equal(t, explodeblock.Blocked, shared.Verdict)
	require.InDelta(t, direct.Witness.Height.Base(), shared.Witness.Height.Base(), 1e-9)
}
