package apitest_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/mesh-repair-design.md's public-surface tests for
// RepairMesh, on a 2 mm cube whose volume is 8 mm³.

// repairCube returns the cube's corners and its twelve triangles, wound
// counter-clockwise seen from outside.
func repairCube() ([]r3.Vec, [][3]int) {
	return []r3.Vec{
		r3.NewVec(0, 0, 0), r3.NewVec(2, 0, 0), r3.NewVec(2, 2, 0), r3.NewVec(0, 2, 0),
		r3.NewVec(0, 0, 2), r3.NewVec(2, 0, 2), r3.NewVec(2, 2, 2), r3.NewVec(0, 2, 2),
	}, [][3]int{
		{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4},
		{2, 3, 7}, {2, 7, 6}, {1, 2, 6}, {1, 6, 5}, {3, 0, 4}, {3, 4, 7},
	}
}

func repairMesh(t *testing.T, v []r3.Vec, tris [][3]int) *decad.MeshBody {
	t.Helper()
	m, err := decad.NewMeshBody(t.Context(), v, tris)
	require.NoError(t, err)
	return m
}

func requireCubeVolume(t *testing.T, m *decad.MeshBody) {
	t.Helper()
	require.True(t, m.Report().Closed)
	volume, err := m.Volume()
	require.NoError(t, err)
	require.InDelta(t, 8, volume.Value.Base(), 1e-6)
}

func TestRepairMeshWeldsASplitCube(t *testing.T) {
	t.Parallel()
	v, tris := repairCube()
	var split []r3.Vec
	var splitTris [][3]int
	for _, tri := range tris {
		var out [3]int
		for k, i := range tri {
			out[k] = len(split)
			split = append(split, v[i].Add(r3.NewVec(1e-7*float64(k), 0, 0)))
		}
		splitTris = append(splitTris, out)
	}
	broken := repairMesh(t, split, splitTris)
	require.False(t, broken.Report().Closed)
	require.Equal(t, 36, broken.Report().BoundaryEdges, "every edge of a split cube is open")
	fixed, report, err := decad.RepairMesh(t.Context(), broken, decad.WithMeshWeld(units.Millimeters(1e-6)))
	require.NoError(t, err)
	requireCubeVolume(t, fixed)
	require.Equal(t, 8, fixed.Report().Vertices)
	require.Equal(t, 28, report.WeldedVertices)
	require.Equal(t, 36, broken.Report().Vertices, "the input mesh is not changed")
}

func TestRepairMeshOrientsAShell(t *testing.T) {
	t.Parallel()
	v, tris := repairCube()
	for _, i := range []int{1, 4, 9} {
		tris[i][1], tris[i][2] = tris[i][2], tris[i][1]
	}
	broken := repairMesh(t, v, tris)
	require.NotZero(t, broken.Report().MisorientedEdges)
	_, err := broken.Volume()
	require.ErrorIs(t, err, decad.ErrNotSolid)
	fixed, report, err := decad.RepairMesh(t.Context(), broken, decad.WithMeshOrient())
	require.NoError(t, err)
	requireCubeVolume(t, fixed)
	require.Zero(t, fixed.Report().MisorientedEdges)
	require.NotZero(t, report.FlippedTriangles)
}

func TestRepairMeshTurnsAnInsideOutCubeRightWayOut(t *testing.T) {
	t.Parallel()
	v, tris := repairCube()
	for i := range tris {
		tris[i][1], tris[i][2] = tris[i][2], tris[i][1]
	}
	broken := repairMesh(t, v, tris)
	require.True(t, broken.Report().Inverted)
	fixed, report, err := decad.RepairMesh(t.Context(), broken, decad.WithMeshOrient())
	require.NoError(t, err)
	require.False(t, fixed.Report().Inverted)
	requireCubeVolume(t, fixed)
	require.Equal(t, 12, report.FlippedTriangles)
}

func TestRepairMeshFillsAHole(t *testing.T) {
	t.Parallel()
	v, tris := repairCube()
	broken := repairMesh(t, v, tris[2:]) // the bottom face is missing
	require.Equal(t, 4, broken.Report().BoundaryEdges)
	fixed, report, err := decad.RepairMesh(t.Context(), broken, decad.WithMeshFillHoles())
	require.NoError(t, err)
	requireCubeVolume(t, fixed)
	require.Equal(t, decad.MeshRepairReport{FilledLoops: 1, AddedTriangles: 4}, report)
	require.Equal(t, 9, fixed.Report().Vertices, "the fan adds a vertex at the loop's centre")
}

func TestRepairMeshDropsTrianglesThatCarryNoSurface(t *testing.T) {
	t.Parallel()
	v, tris := repairCube()
	tris = append(tris, [3]int{0, 0, 1}, [3]int{0, 1, 5})
	fixed, report, err := decad.RepairMesh(t.Context(), repairMesh(t, v, tris))
	require.NoError(t, err)
	requireCubeVolume(t, fixed)
	require.Equal(t, 2, report.DroppedTriangles)
	require.Equal(t, 12, fixed.Report().Triangles)
}

func TestRepairMeshMendsAllThree(t *testing.T) {
	t.Parallel()
	v, tris := repairCube()
	var split []r3.Vec
	var splitTris [][3]int
	for _, tri := range tris[2:] { // no bottom face
		var out [3]int
		for k, i := range tri {
			out[k] = len(split)
			split = append(split, v[i])
		}
		splitTris = append(splitTris, out)
	}
	splitTris[3][1], splitTris[3][2] = splitTris[3][2], splitTris[3][1]
	fixed, report, err := decad.RepairMesh(t.Context(), repairMesh(t, split, splitTris),
		decad.WithMeshWeld(units.Millimeters(0)), decad.WithMeshOrient(), decad.WithMeshFillHoles())
	require.NoError(t, err)
	requireCubeVolume(t, fixed)
	require.Equal(t, 1, report.FilledLoops)
	require.NotZero(t, report.WeldedVertices)
	require.NotZero(t, report.FlippedTriangles)
}

func TestRepairMeshLeavesAGoodMeshWhole(t *testing.T) {
	t.Parallel()
	v, tris := repairCube()
	fixed, report, err := decad.RepairMesh(t.Context(), repairMesh(t, v, tris),
		decad.WithMeshWeld(units.Millimeters(1e-6)), decad.WithMeshOrient(), decad.WithMeshFillHoles())
	require.NoError(t, err)
	requireCubeVolume(t, fixed)
	require.Equal(t, decad.MeshRepairReport{}, report)
}

func TestRepairMeshRefusals(t *testing.T) {
	t.Parallel()
	v, tris := repairCube()
	m := repairMesh(t, v, tris)
	var nilContext context.Context
	_, _, err := decad.RepairMesh(nilContext, m)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, _, err = decad.RepairMesh(t.Context(), nil)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, _, err = decad.RepairMesh(t.Context(), m, nil)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, _, err = decad.RepairMesh(t.Context(), m, decad.WithMeshWeld(units.Millimeters(-1)))
	require.ErrorIs(t, err, decad.ErrNegativeMagnitude)
	_, _, err = decad.RepairMesh(t.Context(), m, decad.WithMeshWeld(units.Degrees(1)))
	require.ErrorIs(t, err, decad.ErrUnitKind)
	_, _, err = decad.RepairMesh(t.Context(), m, decad.WithMeshOrient(), decad.WithMeshOrient())
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, _, err = decad.RepairMesh(t.Context(), m, decad.WithMeshWeld(units.Millimeters(1)), decad.WithMeshWeld(units.Millimeters(2)))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, _, err = decad.RepairMesh(t.Context(), m, decad.WithMeshFillHoles(), decad.WithMeshFillHoles())
	require.ErrorIs(t, err, decad.ErrDegenerate)
	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	_, _, err = decad.RepairMesh(canceled, m, decad.WithMeshOrient())
	require.ErrorIs(t, err, context.Canceled)
}
