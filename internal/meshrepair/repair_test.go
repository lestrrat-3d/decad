package meshrepair_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshrepair"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// cube is a 2 mm cube at the origin, counter-clockwise seen from outside.
func cube() ([]r3.Vec, [][3]int) {
	v := []r3.Vec{
		r3.NewVec(0, 0, 0), r3.NewVec(2, 0, 0), r3.NewVec(2, 2, 0), r3.NewVec(0, 2, 0),
		r3.NewVec(0, 0, 2), r3.NewVec(2, 0, 2), r3.NewVec(2, 2, 2), r3.NewVec(0, 2, 2),
	}
	t := [][3]int{
		{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4},
		{2, 3, 7}, {2, 7, 6}, {1, 2, 6}, {1, 6, 5}, {3, 0, 4}, {3, 4, 7},
	}
	return v, t
}

// volume is the divergence-theorem volume of a mesh.
func volume(v []r3.Vec, tris [][3]int) float64 {
	total := 0.0
	for _, t := range tris {
		total += v[t[0]].Dot(v[t[1]].Cross(v[t[2]])) / 6
	}
	return total
}

// boundaryAndMisoriented counts the edges used once, and those used twice in the
// same direction.
func boundaryAndMisoriented(tris [][3]int) (boundary, misoriented int) {
	directed := map[[2]int]int{}
	for _, t := range tris {
		for k := range t {
			directed[[2]int{t[k], t[(k+1)%3]}]++
		}
	}
	for e, n := range directed {
		back := directed[[2]int{e[1], e[0]}]
		if n == 1 && back == 0 {
			boundary++
		}
		if n >= 2 {
			misoriented++
		}
	}
	return boundary, misoriented
}

func TestRepairWeldsVerticesWithinTheDistance(t *testing.T) {
	t.Parallel()
	v, tris := cube()
	// Give each triangle its own copies of its corners, nudged by 1e-7.
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
	var opts meshrepair.Options
	opts.Weld.Enabled, opts.Weld.Distance = true, 1e-6
	got, err := meshrepair.Repair(t.Context(), split, splitTris, opts)
	require.NoError(t, err)
	require.Len(t, got.Vertices, 8)
	require.Equal(t, 36-8, got.Stats.WeldedVertices)
	boundary, misoriented := boundaryAndMisoriented(got.Triangles)
	require.Zero(t, boundary)
	require.Zero(t, misoriented)
	require.InDelta(t, 8, volume(got.Vertices, got.Triangles), 1e-5)
}

func TestRepairWeldDoesNotMergeVerticesFartherApart(t *testing.T) {
	t.Parallel()
	v, tris := cube()
	var opts meshrepair.Options
	opts.Weld.Enabled, opts.Weld.Distance = true, 1.0
	got, err := meshrepair.Repair(t.Context(), v, tris, opts)
	require.NoError(t, err)
	require.Len(t, got.Vertices, 8, "the cube's corners are 2 mm apart")
	require.Zero(t, got.Stats.WeldedVertices)
}

func TestRepairDropsDegenerateAndRepeatedTriangles(t *testing.T) {
	t.Parallel()
	v, tris := cube()
	tris = append(tris, [3]int{0, 0, 1}, [3]int{0, 1, 5}, [3]int{5, 0, 1})
	// {0,1,5} repeats the cube's {0,1,5}; {5,0,1} is the same vertex set again.
	got, err := meshrepair.Repair(t.Context(), v, tris, meshrepair.Options{})
	require.NoError(t, err)
	require.Len(t, got.Triangles, 12)
	require.Equal(t, 3, got.Stats.DroppedTriangles)
	// A triangle of collinear points has zero area.
	v = append(v, r3.NewVec(1, 0, 0))
	got, err = meshrepair.Repair(t.Context(), v, append(tris[:12:12], [3]int{0, 8, 1}), meshrepair.Options{})
	require.NoError(t, err)
	require.Equal(t, 1, got.Stats.DroppedTriangles)
	require.Len(t, got.Vertices, 8, "the unused vertex is gone")
}

func TestRepairOrientsAShell(t *testing.T) {
	t.Parallel()
	v, tris := cube()
	for _, i := range []int{1, 4, 9} {
		tris[i][1], tris[i][2] = tris[i][2], tris[i][1]
	}
	_, misoriented := boundaryAndMisoriented(tris)
	require.NotZero(t, misoriented)
	got, err := meshrepair.Repair(t.Context(), v, tris, meshrepair.Options{Orient: true})
	require.NoError(t, err)
	_, misoriented = boundaryAndMisoriented(got.Triangles)
	require.Zero(t, misoriented)
	require.InDelta(t, 8, volume(got.Vertices, got.Triangles), 1e-12, "the shell comes out right way out")
	require.Contains(t, []int{3, 9}, got.Stats.FlippedTriangles, "either the three bad triangles turn or the nine good ones")
}

func TestRepairTurnsAnInsideOutShellRightWayOut(t *testing.T) {
	t.Parallel()
	v, tris := cube()
	for i := range tris {
		tris[i][1], tris[i][2] = tris[i][2], tris[i][1]
	}
	require.InDelta(t, -8, volume(v, tris), 1e-12)
	got, err := meshrepair.Repair(t.Context(), v, tris, meshrepair.Options{Orient: true})
	require.NoError(t, err)
	require.InDelta(t, 8, volume(got.Vertices, got.Triangles), 1e-12)
	require.Equal(t, 12, got.Stats.FlippedTriangles)
}

func TestRepairLeavesAnOrientedMeshAlone(t *testing.T) {
	t.Parallel()
	v, tris := cube()
	got, err := meshrepair.Repair(t.Context(), v, tris, meshrepair.Options{Orient: true})
	require.NoError(t, err)
	require.Zero(t, got.Stats.FlippedTriangles)
	require.Len(t, got.Triangles, 12)
	require.InDelta(t, 8, volume(got.Vertices, got.Triangles), 1e-12)
}

func TestRepairFillsAHole(t *testing.T) {
	t.Parallel()
	v, tris := cube()
	// Remove the two triangles of the bottom face {0,2,1} and {0,3,2}.
	tris = tris[2:]
	boundary, _ := boundaryAndMisoriented(tris)
	require.Equal(t, 4, boundary)
	got, err := meshrepair.Repair(t.Context(), v, tris, meshrepair.Options{FillHoles: true})
	require.NoError(t, err)
	require.Equal(t, 1, got.Stats.FilledLoops)
	require.Equal(t, 4, got.Stats.AddedTriangles, "a four-corner loop is a fan of four from its centre")
	require.Len(t, got.Vertices, 9)
	boundary, misoriented := boundaryAndMisoriented(got.Triangles)
	require.Zero(t, boundary)
	require.Zero(t, misoriented)
	require.InDelta(t, 8, volume(got.Vertices, got.Triangles), 1e-12, "a flat loop closes to the same solid")
}

func TestRepairFillsATriangularHoleWithOneTriangle(t *testing.T) {
	t.Parallel()
	v := []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)}
	tris := [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}} // a tetrahedron missing {1,2,3}
	got, err := meshrepair.Repair(t.Context(), v, tris, meshrepair.Options{FillHoles: true})
	require.NoError(t, err)
	require.Equal(t, 1, got.Stats.AddedTriangles)
	require.Len(t, got.Vertices, 4)
	require.InDelta(t, 1.0/6, volume(got.Vertices, got.Triangles), 1e-12)
}

func TestRepairSkipsABoundaryThatBranches(t *testing.T) {
	t.Parallel()
	// Two triangles sharing only a vertex: the boundary passes through vertex 0 twice.
	v := []r3.Vec{
		r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(-1, 0, 0), r3.NewVec(0, -1, 0),
	}
	tris := [][3]int{{0, 1, 2}, {0, 3, 4}}
	got, err := meshrepair.Repair(t.Context(), v, tris, meshrepair.Options{FillHoles: true})
	require.NoError(t, err)
	require.Equal(t, 0, got.Stats.FilledLoops)
	require.Equal(t, 1, got.Stats.SkippedLoops)
	require.Len(t, got.Triangles, 2)
}

func TestRepairReadsTheContext(t *testing.T) {
	t.Parallel()
	v, tris := cube()
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err := meshrepair.Repair(ctx, v, tris, meshrepair.Options{Orient: true})
	require.ErrorIs(t, err, context.Canceled)
}
