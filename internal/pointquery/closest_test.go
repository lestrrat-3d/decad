package pointquery_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/pointquery"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// unitSquare is the square [0,1] x [0,1] at z = 0 as two triangles.
func unitSquare() ([]r3.Vec, [][3]int) {
	return []r3.Vec{{}, {X: 1}, {X: 1, Y: 1}, {Y: 1}}, [][3]int{{0, 1, 2}, {0, 2, 3}}
}

// zero is the rational string of 0, a coordinate most expected points share.
const (
	zero = "0"
	half = "1/2"
)

func ratString(q [3]*big.Rat) [3]string {
	return [3]string{q[0].RatString(), q[1].RatString(), q[2].RatString()}
}

func TestMeshClosest(t *testing.T) {
	t.Parallel()
	verts, tris := unitSquare()
	cases := []struct {
		name  string
		p     r3.Vec
		dist2 string
		point [3]string
	}{
		{"above the inside", r3.Vec{X: 0.25, Y: 0.5, Z: 3}, "9", [3]string{"1/4", half, zero}},
		{"below the inside", r3.Vec{X: 0.75, Y: 0.25, Z: -2}, "4", [3]string{"3/4", "1/4", zero}},
		{"beside an edge", r3.Vec{X: 2, Y: 0.5, Z: 0}, "1", [3]string{"1", half, zero}},
		{"past a corner", r3.Vec{X: 2, Y: 3, Z: 2}, "9", [3]string{"1", "1", zero}},
		{"off the diagonal seam", r3.Vec{X: 0.5, Y: 0.5, Z: 1}, "1", [3]string{half, half, zero}},
		{"beside a slanted foot", r3.Vec{X: 1.5, Y: -0.5, Z: 0}, half, [3]string{"1", "0", zero}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			dist2, point, err := pointquery.MeshClosest(t.Context(), tc.p, verts, tris)
			require.NoError(t, err)
			require.Equal(t, tc.dist2, dist2.RatString())
			require.Equal(t, tc.point, ratString(point))
		})
	}
}

func TestMeshClosestFindsAFootInsideASegment(t *testing.T) {
	t.Parallel()
	// A thin triangle whose nearest point to p is the interior of its long edge.
	verts := []r3.Vec{{}, {X: 4}, {X: 2, Y: 1}}
	dist2, point, err := pointquery.MeshClosest(t.Context(), r3.Vec{X: 1, Y: -2, Z: 0}, verts, [][3]int{{0, 1, 2}})
	require.NoError(t, err)
	require.Equal(t, "4", dist2.RatString())
	require.Equal(t, [3]string{"1", "0", zero}, ratString(point))
}

func TestMeshClosestMatchesMeshDistanceSquared(t *testing.T) {
	t.Parallel()
	verts, tris := unitSquare()
	for _, p := range []r3.Vec{{X: 3, Y: -1, Z: 2}, {X: 0.1, Y: 0.9, Z: -0.3}, {X: -2, Y: -2, Z: 0}} {
		want, err := pointquery.MeshDistanceSquared(t.Context(), p, verts, tris)
		require.NoError(t, err)
		got, _, err := pointquery.MeshClosest(t.Context(), p, verts, tris)
		require.NoError(t, err)
		require.Zero(t, want.Cmp(got), "squared distance for %v", p)
	}
}

func TestMeshClosestRefusesEmptyMesh(t *testing.T) {
	t.Parallel()
	verts, _ := unitSquare()
	_, _, err := pointquery.MeshClosest(t.Context(), r3.Vec{}, verts, nil)
	require.Error(t, err)
}
