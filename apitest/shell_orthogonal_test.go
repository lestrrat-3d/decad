package apitest_test

import (
	"bytes"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/orthogonal-shell-design.md's public-surface tests for Shell on a faceted
// solid every face of which squares to a world axis.

func blockSolid(t *testing.T, lo, hi r3.Vec) (*decad.Document, *decad.Body) {
	t.Helper()
	v, tris := cubeMesh(lo, hi)
	m, err := decad.NewMeshBody(t.Context(), v, tris)
	require.NoError(t, err)
	doc := decad.New()
	body, err := decad.SolidFromMesh(t.Context(), doc, m)
	require.NoError(t, err)
	return doc, body
}

func TestShellOpensABoxMeshSolid(t *testing.T) {
	t.Parallel()
	doc, block := blockSolid(t, r3.NewVec(0, 0, 0), r3.NewVec(10, 8, 6))
	top := decad.Faces(decad.Facing(r3.NewVec(0, 0, 1)))
	out, err := decad.Shell(t.Context(), block, top, units.Millimeters(1))
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies())
	volume, err := out.Volume()
	require.NoError(t, err)
	// The box is 480 mm³ and the cavity 8 x 6 x 5, from the floor 1 up through the open top: 240.
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-(480-240)), volume.Bound.Base()+1e-9)
	inWall, err := out.LocatePoint(t.Context(), r3.NewVec(0.5, 4, 3), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointInside, inWall)
	inCavity, err := out.LocatePoint(t.Context(), r3.NewVec(5, 4, 3), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointOutside, inCavity)
	inFloor, err := out.LocatePoint(t.Context(), r3.NewVec(5, 4, 0.5), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointInside, inFloor)
	require.Len(t, out.Shells(), 1)
}

func TestShellHollowsAClosedBoxMeshSolid(t *testing.T) {
	t.Parallel()
	_, block := blockSolid(t, r3.NewVec(0, 0, 0), r3.NewVec(10, 8, 6))
	out, err := decad.Shell(t.Context(), block, nil, units.Millimeters(1), decad.WithNoOpenings())
	require.NoError(t, err)
	volume, err := out.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-(480-8*6*4)), volume.Bound.Base()+1e-9)
	inCavity, err := out.LocatePoint(t.Context(), r3.NewVec(5, 4, 3), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointOutside, inCavity, "the void is empty")
}

func TestShellOfAMeshSolidRefusals(t *testing.T) {
	t.Parallel()
	_, block := blockSolid(t, r3.NewVec(0, 0, 0), r3.NewVec(10, 8, 6))
	top := decad.Faces(decad.Facing(r3.NewVec(0, 0, 1)))
	// Walls that meet leave no cavity: the block is 6 thick in z and t = 3 takes the cavity to nothing.
	_, err := decad.Shell(t.Context(), block, nil, units.Millimeters(3), decad.WithNoOpenings())
	require.ErrorIs(t, err, decad.ErrDegenerate)
	// An outward shell of a mesh solid keeps the receiver's own refusal.
	_, err = decad.Shell(t.Context(), block, top, units.Millimeters(1), decad.WithShellSense(decad.Outward))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	_, err = decad.Shell(t.Context(), block, top, units.Millimeters(0))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	// A slanted face is not orthogonal: the tetrahedron keeps the refusal of every other receiver.
	facets := importTetrahedron()
	tet, err := decad.ReadSTL(t.Context(), bytes.NewReader(importBinarySTL(facets[:])))
	require.NoError(t, err)
	body, err := decad.SolidFromMesh(t.Context(), decad.New(), tet)
	require.NoError(t, err)
	_, err = decad.Shell(t.Context(), body, decad.Faces(decad.Facing(r3.NewVec(0, 0, -1))), units.Millimeters(0.05))
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

// lPrismSolid is an L-shaped prism 2 thick whose section is (0,0), (4,0), (4,2), (2,2), (2,4), (0,4), a
// closed manifold mesh: the bottom and top are fans from the corner at the origin.
func lPrismSolid(t *testing.T) *decad.Body {
	t.Helper()
	section := [][2]float64{{0, 0}, {4, 0}, {4, 2}, {2, 2}, {2, 4}, {0, 4}}
	var v []r3.Vec
	for _, z := range []float64{0, 2} {
		for _, p := range section {
			v = append(v, r3.NewVec(p[0], p[1], z))
		}
	}
	var tris [][3]int
	n := len(section)
	for i := 1; i+1 < n; i++ {
		tris = append(tris, [3]int{0, i + 1, i})         // bottom, wound down
		tris = append(tris, [3]int{n, n + i, n + i + 1}) // top, wound up
	}
	for i := range n {
		j := (i + 1) % n
		tris = append(tris, [3]int{i, j, n + j}, [3]int{i, n + j, n + i})
	}
	m, err := decad.NewMeshBody(t.Context(), v, tris)
	require.NoError(t, err)
	body, err := decad.SolidFromMesh(t.Context(), decad.New(), m)
	require.NoError(t, err)
	return body
}

func TestShellOpensAnLShapedMeshSolid(t *testing.T) {
	t.Parallel()
	l := lPrismSolid(t)
	out, err := decad.Shell(t.Context(), l, decad.Faces(decad.Facing(r3.NewVec(0, 0, 1))), units.Millimeters(0.5))
	require.NoError(t, err)
	volume, err := out.Volume()
	require.NoError(t, err)
	// The L is 12 x 2 = 24. Its wall is 0.5 thick, so the cavity is the L eroded by 0.5 in the plane, a
	// sharp inside corner at (1.5, 1.5) and area 3 + 2 = 5, from the floor at z = 0.5 to the open top: 1.5 high.
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-(24-5*1.5)), volume.Bound.Base()+1e-9)
	// The wall at the reflex corner is mitered: the point (1.7, 1.7) is 0.3 from each of the two faces
	// that meet at (2, 2) and is in the wall, while (1.4, 1.4) is in the cavity.
	wall, err := out.LocatePoint(t.Context(), r3.NewVec(1.7, 1.7, 1), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointInside, wall)
	cavity, err := out.LocatePoint(t.Context(), r3.NewVec(1.4, 1.4, 1), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointOutside, cavity)
}
