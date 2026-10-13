package pointcone_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/pointcone"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestSectorSubdivideKeepsRootLabels(t *testing.T) {
	nodes := []pointcone.SectorNode{
		{P: r3.Vec{}, GroupToNext: 2, Crossing: -1, RootBoundary: true, Angle: 1},
		{P: r3.Vec{X: 1}, GroupToNext: 3, Crossing: -1},
		{P: r3.Vec{Y: 1}, GroupToNext: -1, Crossing: -1, RootBoundary: true, Angle: 0},
	}
	out, tris, edges, round, err := pointcone.SectorSubdivide(t.Context(), nodes, [][3]int{{0, 1, 2}}, 1)
	require.NoError(t, err)
	require.Len(t, out, 6)
	require.Equal(t, [][3]int{{0, 3, 5}, {3, 1, 4}, {5, 4, 2}, {3, 4, 5}}, tris)
	require.Equal(t, []pointcone.SectorEdge{
		{U: 0, V: 3, Group: 2}, {U: 3, V: 1, Group: 2},
		{U: 1, V: 4, Group: 3}, {U: 4, V: 2, Group: 3},
		{U: 2, V: 5, Group: -1}, {U: 5, V: 0, Group: -1},
	}, edges)
	require.Equal(t, r3.Vec{X: 0.5}, out[3].P)
	require.False(t, out[3].RootBoundary)
	require.True(t, out[5].RootBoundary)
	require.Equal(t, 0.5, out[5].Angle)
	require.Greater(t, round, 0.0)

	require.True(t, pointcone.SectorRootEdgeOrdered(out, edges, [2]float64{0, 1}))
	require.False(t, pointcone.SectorRootEdgeOrdered(out, edges, [2]float64{0, 2}))

	angles, rootIndex, err := pointcone.SectorMergedAngles([]float64{0, 0.25, 1}, out)
	require.NoError(t, err)
	require.Equal(t, []float64{0, 0.25, 0.5, 1}, angles)
	require.Equal(t, []int{3, -1, 0, -1, -1, 2}, rootIndex)

	bad := append([]pointcone.SectorNode(nil), out...)
	bad[0].Angle = math.NaN()
	_, _, err = pointcone.SectorMergedAngles([]float64{0}, bad)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "root angle is non-finite")
}
