package apitest_test

import (
	"math"
	"sort"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/mesh-import-design.md §14's public-surface tests for RemeshMesh, on the sphere of
// mesh_reduce_test.go, whose triangles are thin slivers at the poles and wide quads at the equator.

// edgeStats returns the mean and the standard deviation of the edge lengths of m, and the share of its vertices that
// have six neighbours.
func edgeStats(m *decad.MeshBody) (mean, deviation, regular float64) {
	verts, tris := m.Vertices(), m.Triangles()
	seen := map[[2]int]struct{}{}
	valence := map[int]int{}
	var lengths []float64
	for _, t := range tris {
		for i := range t {
			a, b := t[i], t[(i+1)%3]
			if a > b {
				a, b = b, a
			}
			if _, ok := seen[[2]int{a, b}]; ok {
				continue
			}
			seen[[2]int{a, b}] = struct{}{}
			valence[a]++
			valence[b]++
			lengths = append(lengths, verts[a].Sub(verts[b]).Len())
		}
	}
	for _, l := range lengths {
		mean += l
	}
	mean /= float64(len(lengths))
	for _, l := range lengths {
		deviation += (l - mean) * (l - mean)
	}
	deviation = math.Sqrt(deviation / float64(len(lengths)))
	six := 0
	for _, n := range valence {
		if n == 6 {
			six++
		}
	}
	return mean, deviation, float64(six) / float64(len(valence))
}

func TestRemeshMeshEvensTheEdgesAndTheValences(t *testing.T) {
	t.Parallel()
	m := reduceSphere(t, 10)
	meanBefore, before, _ := edgeStats(m)
	full, err := m.Volume()
	require.NoError(t, err)
	const edge = 2.0
	out, err := decad.RemeshMesh(t.Context(), m, units.Millimeters(edge))
	require.NoError(t, err)
	report := out.Report()
	require.True(t, report.Closed)
	require.Zero(t, report.NonManifoldEdges)
	require.False(t, report.Inverted)
	mean, deviation, regular := edgeStats(out)
	// The edges gather near the asked length, far more evenly than the sphere's own.
	require.InDelta(t, edge, mean, 0.25*edge)
	require.Less(t, deviation/mean, 0.5*before/meanBefore, "the spread of edge lengths falls by at least half")
	require.Less(t, deviation/mean, 0.25)
	// Most vertices have six neighbours, and the mean square distance from six stays small: edge flips bring it to about
	// 0.17 here, where the splits and merges alone leave about 0.55.
	require.Greater(t, regular, 0.6)
	require.Less(t, valenceDeviation(out), 0.3)
	v, err := out.Volume()
	require.NoError(t, err)
	require.InDelta(t, full.Value.Base(), v.Value.Base(), 0.06*full.Value.Base())
	require.Less(t, v.Value.Base(), full.Value.Base()+1e-9, "a convex surface stays inside the sphere it started on")
	// A vertex of the result stays near the sphere's surface; sliding along the tangent plane can leave it a hair outside.
	radii := make([]float64, 0, len(out.Vertices()))
	for _, p := range out.Vertices() {
		radii = append(radii, p.Len())
	}
	sort.Float64s(radii)
	require.LessOrEqual(t, radii[len(radii)-1], 10.05)
	require.Greater(t, radii[0], 9.0)
	require.Equal(t, 2208, len(m.Triangles()), "the input mesh is not changed")
}

func TestRemeshMeshCoarsensAsWellAsRefines(t *testing.T) {
	t.Parallel()
	m := reduceSphere(t, 10)
	coarse, err := decad.RemeshMesh(t.Context(), m, units.Millimeters(6))
	require.NoError(t, err)
	fine, err := decad.RemeshMesh(t.Context(), m, units.Millimeters(0.8))
	require.NoError(t, err)
	require.Less(t, len(coarse.Triangles()), len(m.Triangles()))
	require.Greater(t, len(fine.Triangles()), len(m.Triangles()))
	for _, r := range []*decad.MeshBody{coarse, fine} {
		require.True(t, r.Report().Closed)
		require.Zero(t, r.Report().NonManifoldEdges)
	}
}

func TestRemeshMeshRefusals(t *testing.T) {
	t.Parallel()
	m := reduceSphere(t, 10)
	_, err := decad.RemeshMesh(t.Context(), m, units.Millimeters(0))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.RemeshMesh(t.Context(), nil, units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.RemeshMesh(t.Context(), m, units.Millimeters(-1))
	require.ErrorIs(t, err, decad.ErrNegativeMagnitude)
	_, err = decad.RemeshMesh(t.Context(), reduceGrid(t, 4), units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	// 4 pi 10^2 mm^2 of surface in 0.01 mm edges is over a million triangles.
	_, err = decad.RemeshMesh(t.Context(), m, units.Millimeters(0.01))
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

// valenceRange returns the least and the greatest number of neighbours a vertex of m has.
func valenceRange(m *decad.MeshBody) (least, greatest int) {
	neighbours := map[int]map[int]struct{}{}
	for _, t := range m.Triangles() {
		for i := range t {
			a, b := t[i], t[(i+1)%3]
			for _, p := range [][2]int{{a, b}, {b, a}} {
				if neighbours[p[0]] == nil {
					neighbours[p[0]] = map[int]struct{}{}
				}
				neighbours[p[0]][p[1]] = struct{}{}
			}
		}
	}
	least = math.MaxInt
	for _, n := range neighbours {
		least, greatest = min(least, len(n)), max(greatest, len(n))
	}
	return least, greatest
}

func TestRemeshMeshFlipsEdgesTowardSixNeighbours(t *testing.T) {
	t.Parallel()
	// A 24-sided bipyramid has two apexes of 24 neighbours and a ring of vertices of 4, as far from six as a closed
	// mesh gets; remeshing at about the ring's own spacing must even that out.
	m := bipyramid(t, 24, 3)
	_, most := valenceRange(m)
	require.Equal(t, 24, most)
	out, err := decad.RemeshMesh(t.Context(), m, units.Millimeters(1.3))
	require.NoError(t, err)
	require.True(t, out.Report().Closed)
	require.Zero(t, out.Report().NonManifoldEdges)
	least, most := valenceRange(out)
	require.GreaterOrEqual(t, least, 3)
	require.LessOrEqual(t, most, 9, "no vertex keeps the apex's crowd of neighbours")
}

// valenceDeviation is the mean over the vertices of m of the square of the distance from six neighbours.
func valenceDeviation(m *decad.MeshBody) float64 {
	neighbours := map[int]map[int]struct{}{}
	for _, t := range m.Triangles() {
		for i := range t {
			a, b := t[i], t[(i+1)%3]
			for _, p := range [][2]int{{a, b}, {b, a}} {
				if neighbours[p[0]] == nil {
					neighbours[p[0]] = map[int]struct{}{}
				}
				neighbours[p[0]][p[1]] = struct{}{}
			}
		}
	}
	sum := 0.0
	for _, n := range neighbours {
		d := float64(len(n) - 6)
		sum += d * d
	}
	return sum / float64(len(neighbours))
}
