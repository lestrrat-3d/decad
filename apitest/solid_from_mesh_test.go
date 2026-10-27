package apitest_test

import (
	"bytes"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/mesh-solid-design.md's public-surface tests for SolidFromMesh.

// cubeMesh returns the vertices and outward triangles of the box from lo to hi.
func cubeMesh(lo, hi r3.Vec) ([]r3.Vec, [][3]int) {
	v := []r3.Vec{
		{X: lo.X, Y: lo.Y, Z: lo.Z}, {X: hi.X, Y: lo.Y, Z: lo.Z}, {X: hi.X, Y: hi.Y, Z: lo.Z}, {X: lo.X, Y: hi.Y, Z: lo.Z},
		{X: lo.X, Y: lo.Y, Z: hi.Z}, {X: hi.X, Y: lo.Y, Z: hi.Z}, {X: hi.X, Y: hi.Y, Z: hi.Z}, {X: lo.X, Y: hi.Y, Z: hi.Z},
	}
	quads := [][4]int{{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4}, {3, 7, 6, 2}, {0, 4, 7, 3}, {1, 2, 6, 5}}
	var t [][3]int
	for _, q := range quads {
		t = append(t, [3]int{q[0], q[1], q[2]}, [3]int{q[0], q[2], q[3]})
	}
	return v, t
}

func solidFromBlock(t *testing.T) (*decad.Document, *decad.Body) {
	t.Helper()
	var buf bytes.Buffer
	require.NoError(t, export.STL(t.Context(), &buf, importBlock(t), units.Millimeters(0.1)))
	m, err := decad.ReadSTL(t.Context(), &buf)
	require.NoError(t, err)
	doc := decad.New()
	body, err := decad.SolidFromMesh(t.Context(), doc, m)
	require.NoError(t, err)
	return doc, body
}

func TestSolidFromMeshReadsAnImportedBlockAsASolid(t *testing.T) {
	t.Parallel()
	doc, body := solidFromBlock(t)
	require.Equal(t, []*decad.Body{body}, doc.Bodies())
	require.True(t, body.IsSolid())
	operations := doc.OperationHistory().Operations()
	require.Equal(t, decad.OperationSolidFromMesh, operations[len(operations)-1].Kind())
	// Triangles on one exact plane make one face: six faces for a block of twelve triangles.
	require.Len(t, body.Faces(), 6)
	volume, err := body.Volume()
	require.NoError(t, err)
	require.InDelta(t, 24, volume.Value.Base(), 1e-12)
	require.Zero(t, volume.Bound.Base(), "the mesh is the solid, so nothing bounds the volume")
	box, err := body.Bounds()
	require.NoError(t, err)
	require.True(t, box.Min.Equal(r3.NewVec(0, 0, 0), 1e-12))
	require.True(t, box.Max.Equal(r3.NewVec(4, 3, 2), 1e-12))
	inside, err := body.LocatePoint(t.Context(), r3.NewVec(2, 1.5, 1), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointInside, inside)
	outside, err := body.LocatePoint(t.Context(), r3.NewVec(5, 1.5, 1), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointOutside, outside)
}

func TestSolidFromMeshTakesPartInBooleans(t *testing.T) {
	t.Parallel()
	doc, body := solidFromBlock(t)
	// A tool box overlapping the block's x 2..4, y 1..2, z 0..2 corner region removes 2 x 1 x 2 = 4.
	tool := boxBody(t, doc, 2, 1, 6, 2, 3)
	out, err := decad.Cut(t.Context(), body, tool)
	require.NoError(t, err)
	volume, err := out.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-20), volume.Bound.Base()+1e-9)
	// And a bar through the block's end joined to the cut result by a Union adds its volume less
	// the overlap: the bar is x 3..8, y -1..4, z 0.5..1.5 for 25, and inside the block's x 3..4 it
	// meets the cut result over the 2 mm of y the notch leaves, 1 x 2 x 1 = 2, so the union holds 20 + 25 - 2 = 43.
	bar := boxBodyAtZ(t, doc, 3, -1, 8, 4, 0.5, 1)
	joined, err := decad.Union(t.Context(), out, bar)
	require.NoError(t, err)
	jv, err := joined.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(jv.Value.Base()-43), jv.Bound.Base()+1e-9)
}

func TestSolidFromMeshReadsATetrahedron(t *testing.T) {
	t.Parallel()
	facets := importTetrahedron()
	m, err := decad.ReadSTL(t.Context(), bytes.NewReader(importBinarySTL(facets[:])))
	require.NoError(t, err)
	body, err := decad.SolidFromMesh(t.Context(), decad.New(), m)
	require.NoError(t, err)
	require.Len(t, body.Faces(), 4)
	volume, err := body.Volume()
	require.NoError(t, err)
	require.InDelta(t, 1.0/6, volume.Value.Base(), 1e-15)
}

func TestSolidFromMeshRefusals(t *testing.T) {
	t.Parallel()
	v, tris := cubeMesh(r3.NewVec(0, 0, 0), r3.NewVec(2, 2, 2))
	build := func(vertices []r3.Vec, triangles [][3]int) *decad.MeshBody {
		m, err := decad.NewMeshBody(t.Context(), vertices, triangles)
		require.NoError(t, err)
		return m
	}
	solid := func(m *decad.MeshBody) error {
		_, err := decad.SolidFromMesh(t.Context(), decad.New(), m)
		return err
	}
	require.NoError(t, solid(build(v, tris)))
	require.ErrorIs(t, solid(build(v, tris[:len(tris)-1])), decad.ErrNotSolid, "an open mesh")
	inward := make([][3]int, len(tris))
	for i, tri := range tris {
		inward[i] = [3]int{tri[0], tri[2], tri[1]}
	}
	require.ErrorIs(t, solid(build(v, inward)), decad.ErrNotSolid, "a mesh wound inward")
	// Two cubes that overlap, in one mesh: closed and wound outward, but each cuts the other's walls.
	v2, t2 := cubeMesh(r3.NewVec(1, 1, 1), r3.NewVec(3, 3, 3))
	both := append(append([]r3.Vec{}, v...), v2...)
	bothTris := append([][3]int{}, tris...)
	for _, tri := range t2 {
		bothTris = append(bothTris, [3]int{tri[0] + len(v), tri[1] + len(v), tri[2] + len(v)})
	}
	require.ErrorIs(t, solid(build(both, bothTris)), decad.ErrDegenerate, "a mesh that crosses itself")
	_, err := decad.SolidFromMesh(nil, decad.New(), build(v, tris)) //nolint:staticcheck // a nil context is the case under test
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.SolidFromMesh(t.Context(), nil, build(v, tris))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.SolidFromMesh(t.Context(), decad.New(), nil)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	doc := decad.New()
	_, err = decad.SolidFromMesh(t.Context(), doc, build(v, tris[:len(tris)-1]))
	require.Error(t, err)
	require.Empty(t, doc.Bodies(), "a refusal leaves the document unchanged")
}

func TestSolidFromMeshBodyKeepsWorkingAfterPlacementAndReadings(t *testing.T) {
	t.Parallel()
	_, body := solidFromBlock(t)
	// Mass properties read the exact mesh: density 0.5 kg/mm^3 over 24 mm^3 is 12 kg, the centre is
	// the block's middle, and a solid block's inertia about z is m·(a² + b²)/12 = 12·(16 + 9)/12 = 25.
	props, err := body.MassProperties(t.Context(), units.KilogramsPerCubicMillimeter(0.5))
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(props.Mass.Value.Base()-12), props.Mass.Bound.Base()+1e-12)
	require.True(t, props.Center.Value.Equal(r3.NewVec(2, 1.5, 1), 1e-9))
	require.LessOrEqual(t, math.Abs(props.Inertia.ZZ.Value.Base()-25), props.Inertia.ZZ.Bound.Base()+1e-9)
	// A translated copy lands where it should and keeps its faces and volume.
	move, err := r3.Translation(r3.NewVec(5, 0, 0))
	require.NoError(t, err)
	moved, err := decad.Transform(t.Context(), body, move)
	require.NoError(t, err)
	box, err := moved.Bounds()
	require.NoError(t, err)
	require.True(t, box.Min.Equal(r3.NewVec(5, 0, 0), 1e-9))
	require.True(t, box.Max.Equal(r3.NewVec(9, 3, 2), 1e-9))
	require.Len(t, moved.Faces(), 6)
	mv, err := moved.Volume()
	require.NoError(t, err)
	require.InDelta(t, 24, mv.Value.Base(), 1e-9)
	// And the result writes out as STL and reads back as the same closed mesh.
	var out bytes.Buffer
	require.NoError(t, export.STL(t.Context(), &out, moved, units.Millimeters(0.1)))
	back, err := decad.ReadSTL(t.Context(), &out)
	require.NoError(t, err)
	require.True(t, back.Report().Closed)
	require.Equal(t, 12, back.Report().Triangles)
}
