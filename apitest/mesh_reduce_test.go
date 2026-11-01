package apitest_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/mesh-import-design.md §9's public-surface tests for
// ReduceMesh.

// reduceGrid is an n x n grid of unit quads on z = 0, two triangles each facing +Z.
func reduceGrid(t *testing.T, n int) *decad.MeshBody {
	t.Helper()
	var v []r3.Vec
	for y := 0; y <= n; y++ {
		for x := 0; x <= n; x++ {
			v = append(v, r3.NewVec(float64(x), float64(y), 0))
		}
	}
	var tris [][3]int
	at := func(x, y int) int { return y*(n+1) + x }
	for y := range n {
		for x := range n {
			tris = append(tris, [3]int{at(x, y), at(x+1, y), at(x+1, y+1)}, [3]int{at(x, y), at(x+1, y+1), at(x, y+1)})
		}
	}
	m, err := decad.NewMeshBody(t.Context(), v, tris)
	require.NoError(t, err)
	return m
}

// reduceSphere is a UV sphere of radius r about the origin, 48 around and 24 up,
// wound counter-clockwise seen from outside.
func reduceSphere(t testing.TB, r float64) *decad.MeshBody {
	t.Helper()
	const around, up = 48, 24
	v := []r3.Vec{r3.NewVec(0, 0, r)}
	for i := 1; i < up; i++ {
		phi := math.Pi * float64(i) / up
		for j := range around {
			theta := 2 * math.Pi * float64(j) / around
			v = append(v, r3.NewVec(r*math.Sin(phi)*math.Cos(theta), r*math.Sin(phi)*math.Sin(theta), r*math.Cos(phi)))
		}
	}
	v = append(v, r3.NewVec(0, 0, -r))
	ring := func(i, j int) int { return 1 + (i-1)*around + j%around }
	var tris [][3]int
	for j := range around {
		tris = append(tris, [3]int{0, ring(1, j), ring(1, j+1)})
	}
	for i := 1; i < up-1; i++ {
		for j := range around {
			tris = append(tris, [3]int{ring(i, j), ring(i+1, j), ring(i+1, j+1)}, [3]int{ring(i, j), ring(i+1, j+1), ring(i, j+1)})
		}
	}
	bottom := len(v) - 1
	for j := range around {
		tris = append(tris, [3]int{bottom, ring(up-1, j+1), ring(up-1, j)})
	}
	m, err := decad.NewMeshBody(t.Context(), v, tris)
	require.NoError(t, err)
	require.True(t, m.Report().Closed)
	return m
}

func TestReduceMeshThinsAGrid(t *testing.T) {
	t.Parallel()
	m := reduceGrid(t, 16)
	out, report, err := decad.ReduceMesh(t.Context(), m, units.Millimeters(4))
	require.NoError(t, err)
	require.Equal(t, 25, out.Report().Vertices)
	require.Equal(t, 32, out.Report().Triangles)
	require.Equal(t, 289-25, report.MergedVertices)
	require.Equal(t, 512-32, report.DroppedTriangles)
	require.InDelta(t, math.Hypot(1.5, 1.5), report.MaxVertexMove.Base(), 1e-12)
	area, err := out.Area()
	require.NoError(t, err)
	require.InDelta(t, 14.5*14.5, area.Value.Base(), 1e-9)
	require.Equal(t, 289, m.Report().Vertices, "the input mesh is not changed")
}

func TestReduceMeshThinsASphereAndKeepsItsShape(t *testing.T) {
	t.Parallel()
	m := reduceSphere(t, 10)
	out, report, err := decad.ReduceMesh(t.Context(), m, units.Millimeters(2.5))
	require.NoError(t, err)
	require.Less(t, out.Report().Triangles, m.Report().Triangles/2, "the sphere loses over half its triangles")
	require.Positive(t, report.MergedVertices)
	// Every original vertex has a replacement within MaxVertexMove.
	move := report.MaxVertexMove.Base()
	require.LessOrEqual(t, move, 2.5*math.Sqrt(3), "no farther than a cell's diagonal")
	replacements := out.Vertices()
	for _, v := range m.Vertices() {
		nearest := math.Inf(1)
		for _, p := range replacements {
			nearest = math.Min(nearest, p.Sub(v).Len())
		}
		require.LessOrEqual(t, nearest, move+1e-9)
	}
	// The vertices stay within the cell's reach of the sphere, and the shape stays
	// roughly round: its bounds are within a cell of the sphere's.
	box := out.Bounds()
	for _, c := range []float64{box.Max.X, box.Max.Y, box.Max.Z, -box.Min.X, -box.Min.Y, -box.Min.Z} {
		require.InDelta(t, 10, c, 2.5*math.Sqrt(3)+1e-9)
	}
}

func TestReduceMeshWithAFineCellChangesNothing(t *testing.T) {
	t.Parallel()
	m := reduceGrid(t, 4)
	out, report, err := decad.ReduceMesh(t.Context(), m, units.Millimeters(0.5))
	require.NoError(t, err)
	require.Equal(t, m.Report(), out.Report())
	require.Equal(t, decad.MeshReduceReport{MaxVertexMove: units.Millimeters(0)}, report)
}

func TestReduceMeshRefusals(t *testing.T) {
	t.Parallel()
	m := reduceGrid(t, 4)
	var nilContext context.Context
	_, _, err := decad.ReduceMesh(nilContext, m, units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, _, err = decad.ReduceMesh(t.Context(), nil, units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, _, err = decad.ReduceMesh(t.Context(), m, units.Millimeters(0))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, _, err = decad.ReduceMesh(t.Context(), m, units.Millimeters(-1))
	require.ErrorIs(t, err, decad.ErrNegativeMagnitude)
	_, _, err = decad.ReduceMesh(t.Context(), m, units.Degrees(1))
	require.ErrorIs(t, err, decad.ErrUnitKind)
	_, _, err = decad.ReduceMesh(t.Context(), m, units.Millimeters(1000))
	require.ErrorIs(t, err, decad.ErrDegenerate, "a cell larger than the mesh leaves no triangle")
	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	_, _, err = decad.ReduceMesh(canceled, m, units.Millimeters(2))
	require.ErrorIs(t, err, context.Canceled)
}
