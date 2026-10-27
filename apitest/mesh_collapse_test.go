package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/mesh-import-design.md §13's public-surface tests for CollapseMesh, on the sphere of
// mesh_reduce_test.go (2,208 triangles).

func TestCollapseMeshThinsASphereAndKeepsItClosed(t *testing.T) {
	t.Parallel()
	m := reduceSphere(t, 10)
	full, err := m.Volume()
	require.NoError(t, err)
	for target, loss := range map[int]float64{1500: 0.02, 600: 0.06, 200: 0.15} {
		out, report, err := decad.CollapseMesh(t.Context(), m, target)
		require.NoError(t, err)
		got := len(out.Triangles())
		require.LessOrEqual(t, got, target)
		require.GreaterOrEqual(t, got, target-60, "the thinning stops near the target, not far short of it")
		require.Equal(t, 2*report.MergedVertices, report.DroppedTriangles)
		require.Equal(t, len(m.Triangles())-got, report.DroppedTriangles)
		require.True(t, out.Report().Closed)
		require.Zero(t, out.Report().NonManifoldEdges)
		require.False(t, out.Report().Inverted)
		v, err := out.Volume()
		require.NoError(t, err)
		// A sphere thinned to a coarser polyhedron loses volume near its curvature.
		require.InDelta(t, full.Value.Base(), v.Value.Base(), loss*full.Value.Base(), "target %d", target)
		require.Less(t, v.Value.Base(), full.Value.Base()+1e-9)
		// Every vertex of the result lies on or inside the sphere, as midpoints of chords do.
		for _, p := range out.Vertices() {
			require.LessOrEqual(t, p.Len(), 10+1e-9)
		}
		require.Less(t, report.MaxVertexMove.Base(), 10.0)
	}
}

func TestCollapseMeshLeavesAMeshThatIsAlreadyCoarseEnough(t *testing.T) {
	t.Parallel()
	m := reduceSphere(t, 7)
	out, report, err := decad.CollapseMesh(t.Context(), m, len(m.Triangles()))
	require.NoError(t, err)
	require.Equal(t, len(m.Triangles()), len(out.Triangles()))
	require.Zero(t, report.MergedVertices)
	require.Zero(t, report.MaxVertexMove.Base())
	for i, p := range m.Vertices() {
		require.Equal(t, p, out.Vertices()[i])
	}
}

func TestCollapseMeshStopsWhereNoEdgeCanBeMerged(t *testing.T) {
	t.Parallel()
	m := reduceSphere(t, 10)
	out, _, err := decad.CollapseMesh(t.Context(), m, 4)
	require.NoError(t, err)
	// The sphere cannot thin to a tetrahedron by midpoint merges that keep every neighbour within 78 degrees; it stops
	// at a closed mesh of at least four triangles.
	require.GreaterOrEqual(t, len(out.Triangles()), 4)
	require.True(t, out.Report().Closed)
	v, err := out.Volume()
	require.NoError(t, err)
	require.Greater(t, v.Value.Base(), 0.0)
}

func TestCollapseMeshRefusals(t *testing.T) {
	t.Parallel()
	m := reduceSphere(t, 10)
	_, _, err := decad.CollapseMesh(t.Context(), m, 3)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, _, err = decad.CollapseMesh(t.Context(), nil, 100)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	// An open sheet has no closed surface to keep.
	_, _, err = decad.CollapseMesh(t.Context(), reduceGrid(t, 4), 10)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	// Two triangles sharing an edge but wound against each other are not consistently wound.
	bad, err := decad.NewMeshBody(t.Context(),
		[]r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)},
		[][3]int{{0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}})
	require.NoError(t, err)
	_, _, err = decad.CollapseMesh(t.Context(), bad, 4)
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

func TestCollapseMeshMovesVerticesByHalfTheirEdge(t *testing.T) {
	t.Parallel()
	m := reduceSphere(t, 10)
	_, report, err := decad.CollapseMesh(t.Context(), m, len(m.Triangles())-2)
	require.NoError(t, err)
	require.Equal(t, 1, report.MergedVertices)
	// One merge puts the shortest edge's two ends at its midpoint: the sphere's shortest edges are the polar
	// triangles', 10 sin(pi/24)... either way each end moves half the edge.
	shortest := math.Inf(1)
	for _, tri := range m.Triangles() {
		for i := range tri {
			shortest = math.Min(shortest, m.Vertices()[tri[i]].Sub(m.Vertices()[tri[(i+1)%3]]).Len())
		}
	}
	require.InDelta(t, shortest/2, report.MaxVertexMove.Base(), 1e-9)
}

// bipyramid is an n-sided bipyramid of radius 5 and half height h about the origin: two apexes and a ring of n
// vertices, 2n triangles, wound outward.
func bipyramid(t *testing.T, n int, h float64) *decad.MeshBody {
	t.Helper()
	return starBipyramid(t, n, h, 5, 5)
}

// starBipyramid is bipyramid with the ring's radius alternating between outer and inner, a star that has concave
// corners between its points.
func starBipyramid(t *testing.T, n int, h, outer, inner float64) *decad.MeshBody {
	t.Helper()
	v := []r3.Vec{r3.NewVec(0, 0, h), r3.NewVec(0, 0, -h)}
	for i := range n {
		a := 2 * math.Pi * float64(i) / float64(n)
		r := outer
		if i%2 == 1 {
			r = inner
		}
		v = append(v, r3.NewVec(r*math.Cos(a), r*math.Sin(a), 0))
	}
	var tris [][3]int
	for i := range n {
		j := (i + 1) % n
		tris = append(tris, [3]int{0, 2 + i, 2 + j}, [3]int{1, 2 + j, 2 + i})
	}
	m, err := decad.NewMeshBody(t.Context(), v, tris)
	require.NoError(t, err)
	require.True(t, m.Report().Closed)
	return m
}

func TestCollapseMeshKeepsSmallSolidsClosedAndWoundOutward(t *testing.T) {
	t.Parallel()
	for name, m := range map[string]*decad.MeshBody{
		"octahedron": bipyramid(t, 4, 5), "hexagonal": bipyramid(t, 6, 3), "tall": bipyramid(t, 8, 20), "flat": bipyramid(t, 12, 0.5),
		"star": starBipyramid(t, 12, 3, 6, 1.5), "spiky": starBipyramid(t, 16, 2, 8, 0.8),
	} {
		for target := 4; target <= len(m.Triangles()); target++ {
			out, _, err := decad.CollapseMesh(t.Context(), m, target)
			require.NoError(t, err, name)
			report := out.Report()
			require.True(t, report.Closed, "%s to %d", name, target)
			require.Zero(t, report.NonManifoldEdges, "%s to %d", name, target)
			require.False(t, report.Inverted, "%s to %d", name, target)
			v, err := out.Volume()
			require.NoError(t, err)
			require.Greater(t, v.Value.Base(), 0.0, "%s to %d", name, target)
			// Every directed edge occurs once with its reverse: a consistently wound closed surface.
			directed := map[[2]int]int{}
			for _, tri := range out.Triangles() {
				for i := range tri {
					directed[[2]int{tri[i], tri[(i+1)%3]}]++
				}
			}
			for e, count := range directed {
				require.Equal(t, 1, count, "%s to %d", name, target)
				require.Equal(t, 1, directed[[2]int{e[1], e[0]}], "%s to %d", name, target)
			}
		}
	}
}
