package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/pointcone"
	"github.com/lestrrat-3d/decad/internal/revolveshell"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestSectorCapRefinementKeepsSharedEdges(t *testing.T) {
	cone := &pointConeLimit{Apex: big.NewRat(20, 1), Slope: big.NewRat(1, 1)}
	record := &pointConeTrimRecord{Lower: cone, Upper: cone}
	nodes := []pointcone.SectorNode{
		{P: r3.NewVec(10, 5, 0), RootBoundary: true, Angle: 0},
		{P: r3.NewVec(10, 6, 0), RootBoundary: true, Angle: 0.1},
		{P: r3.NewVec(10, 5, 1)},
		{P: r3.NewVec(10, 10, 10)},
	}
	triangles := [][3]int{{0, 1, 2}, {1, 3, 2}}
	edges := []pointcone.SectorEdge{
		{U: 0, V: 1, Group: -1},
		{U: 1, V: 3, Group: 2},
		{U: 3, V: 2, Group: 3},
		{U: 2, V: 0, Group: 3},
	}
	before, _, err := revolveshell.SectorConeCapBounds(nodes, triangles, record)
	require.NoError(t, err)
	limit := before * 0.01
	refined, cells, perimeter, round, err := revolveshell.SectorRefineCap(t.Context(), nodes,
		triangles, edges, record, limit, 32)
	require.NoError(t, err)
	require.Greater(t, round, 0.0)
	require.Greater(t, len(cells), len(triangles))
	after, _, err := revolveshell.SectorConeCapBounds(refined, cells, record)
	require.NoError(t, err)
	require.Less(t, after, before)
	require.Equal(t, -1, perimeter[0].Group)
	require.Equal(t, -1, perimeter[1].Group)
	require.True(t, refined[perimeter[0].V].RootBoundary)
	require.Equal(t, 0.05, refined[perimeter[0].V].Angle)

	counts := map[pointcone.Edge]int{}
	for _, tri := range cells {
		for i := range 3 {
			counts[pointcone.OrderedEdge(tri[i], tri[(i+1)%3])]++
		}
	}
	for _, edge := range perimeter {
		require.Equal(t, 1, counts[pointcone.OrderedEdge(edge.U, edge.V)])
	}
	for _, count := range counts {
		require.LessOrEqual(t, count, 2)
	}
	refusedNodes, refusedCells, refusedEdges, refusedRound, err := revolveshell.SectorRefineCap(t.Context(), nodes,
		triangles, edges, record, limit, 5)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Nil(t, refusedNodes)
	require.Nil(t, refusedCells)
	require.Nil(t, refusedEdges)
	require.Zero(t, refusedRound)
}
